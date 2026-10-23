#include "WindEstimator.h"
#include "Gps.h"
#include <math.h>

// ============================================================
// WIND / AIRSPEED ESTIMATOR -- circle fit on ground velocity
// ============================================================
float estimatedWindSpeedKph = 0.0f;
float estimatedAirspeedKph = 0.0f;
float estimatedWindDirectionDeg = 0.0f;
bool windEstimateValid = false;

namespace {

uint32_t gWindPublished_ms = 0;
constexpr uint32_t kWindMaxAge_ms = 10UL * 60UL * 1000UL;
constexpr uint32_t kGpsSampleMaxAge_ms = 2000;
constexpr uint32_t kMinSampleInterval_ms = 500;     // decimates fast GPS rates
constexpr uint32_t kMaxCircleDuration_ms = 45000;   // slower than this is not a thermal turn
constexpr float kMinGroundSpeed_kph = 10.0f;        // course is unreliable below this
constexpr float kFullCircleTurn_deg = 340.0f;
constexpr float kMaxFitRmsFraction = 0.15f;         // rms residual / radius
constexpr float kMinFitDeterminant = 1.0f;          // rejects collinear points
constexpr float kDegToRad = 0.017453292f;
constexpr float kRadToDeg = 57.295780f;

// 45 s / 0.5 s = 90 samples, plus margin.
constexpr size_t kCapacity = 96;

struct Sample {
  float vEast_kph;
  float vNorth_kph;
  float turn_deg;   // signed turn from the previous sample (+ = clockwise)
  uint32_t t_ms;
};

// Fixed-capacity ring buffer. Logical index 0 is the oldest sample.
Sample gRing[kCapacity];
size_t gHead = 0;
size_t gCount = 0;
float gLastTrack_deg = 0.0f;
uint32_t gLastSample_ms = 0;

const Sample& at(size_t logical) {
  return gRing[(gHead + logical) % kCapacity];
}

void push(const Sample& s) {
  if (gCount < kCapacity) {
    gRing[(gHead + gCount) % kCapacity] = s;
    gCount++;
  } else {
    gRing[gHead] = s;
    gHead = (gHead + 1) % kCapacity;
  }
}

float wrapDelta(float delta_deg) {
  if (delta_deg > 180.0f) delta_deg -= 360.0f;
  if (delta_deg < -180.0f) delta_deg += 360.0f;
  return delta_deg;
}

// Walk back from the newest sample until the signed turn reaches a full
// circle. Signed, so wobble and left/right reversals cancel instead of
// counting as a circle. Returns false if no full circle is buffered.
// startOut is the logical index of the oldest sample of the window.
bool findCircleWindow(size_t& startOut) {
  if (gCount < 4) return false;
  const size_t newest = gCount - 1;
  float turn_deg = 0.0f;
  for (size_t k = newest; k >= 1; k--) {
    turn_deg += at(k).turn_deg;
    if (fabsf(turn_deg) >= kFullCircleTurn_deg) {
      startOut = k - 1;
      return true;
    }
  }
  return false;
}

struct CircleFit {
  float centreEast_kph;
  float centreNorth_kph;
  float radius_kph;
  float rms_kph;
};

// Kasa least-squares circle fit on samples [start .. gCount-1], done on
// mean-centred data so single-precision float stays well conditioned.
bool fitCircle(size_t start, CircleFit& out) {
  const size_t n = gCount - start;
  if (n < 4) return false;

  float meanE = 0.0f;
  float meanN = 0.0f;
  for (size_t i = start; i < gCount; i++) {
    meanE += at(i).vEast_kph;
    meanN += at(i).vNorth_kph;
  }
  meanE /= static_cast<float>(n);
  meanN /= static_cast<float>(n);

  float suu = 0.0f, svv = 0.0f, suv = 0.0f;
  float suuu = 0.0f, svvv = 0.0f, suvv = 0.0f, svuu = 0.0f;
  for (size_t i = start; i < gCount; i++) {
    const float u = at(i).vEast_kph - meanE;
    const float v = at(i).vNorth_kph - meanN;
    suu += u * u;
    svv += v * v;
    suv += u * v;
    suuu += u * u * u;
    svvv += v * v * v;
    suvv += u * v * v;
    svuu += v * u * u;
  }

  const float det = suu * svv - suv * suv;
  if (!isfinite(det) || fabsf(det) < kMinFitDeterminant) return false;

  const float b1 = 0.5f * (suuu + suvv);
  const float b2 = 0.5f * (svvv + svuu);
  const float uc = (b1 * svv - b2 * suv) / det;
  const float vc = (suu * b2 - suv * b1) / det;

  const float radiusSq = uc * uc + vc * vc + (suu + svv) / static_cast<float>(n);
  if (!isfinite(radiusSq) || radiusSq <= 0.0f) return false;
  const float radius = sqrtf(radiusSq);

  float sumSqErr = 0.0f;
  for (size_t i = start; i < gCount; i++) {
    const float du = at(i).vEast_kph - meanE - uc;
    const float dv = at(i).vNorth_kph - meanN - vc;
    const float err = sqrtf(du * du + dv * dv) - radius;
    sumSqErr += err * err;
  }

  out.centreEast_kph = meanE + uc;
  out.centreNorth_kph = meanN + vc;
  out.radius_kph = radius;
  out.rms_kph = sqrtf(sumSqErr / static_cast<float>(n));
  return isfinite(out.rms_kph);
}

// Validate the fit and, if good, publish it. Returns true if published.
bool publishIfValid(const CircleFit& fit) {
  if (fit.radius_kph < WIND_MIN_CIRCLE_SPEED_KPH ||
      fit.radius_kph > WIND_MAX_CIRCLE_SPEED_KPH) {
    return false;
  }
  if (fit.rms_kph > kMaxFitRmsFraction * fit.radius_kph) return false;

  const float windSpeed_kph = sqrtf(fit.centreEast_kph * fit.centreEast_kph +
                                    fit.centreNorth_kph * fit.centreNorth_kph);
  if (windSpeed_kph > WIND_MAX_ESTIMATE_KPH) return false;

  // The circle centre is the wind vector: where the wind blows TO.
  // Negate it to get where the wind comes FROM, clockwise from north.
  float from_deg = atan2f(-fit.centreEast_kph, -fit.centreNorth_kph) * kRadToDeg;
  if (from_deg < 0.0f) from_deg += 360.0f;
  if (from_deg >= 360.0f) from_deg -= 360.0f;

  estimatedWindSpeedKph = windSpeed_kph;
  estimatedAirspeedKph = fit.radius_kph;
  estimatedWindDirectionDeg = from_deg;
  windEstimateValid = true;
  gWindPublished_ms = millis();
  return true;
}

}  // namespace

void windEstimatorReset() {
  gHead = 0;
  gCount = 0;
}

void windEstimatorFeed(float groundSpeedKph, float trackDeg, uint32_t now_ms) {
  // GPS input is untrusted (P1-6): reject NaN/inf and out-of-range values.
  if (!isfinite(groundSpeedKph) || !isfinite(trackDeg) ||
      trackDeg < 0.0f || trackDeg > 360.0f) {
    windEstimatorReset();
    return;
  }

  // Nearly stationary: course is meaningless and the circle is broken.
  if (groundSpeedKph < kMinGroundSpeed_kph) {
    windEstimatorReset();
    return;
  }

  // Unsigned subtraction, safe across millis() wrap (P2-14).
  if (gCount > 0 && (now_ms - gLastSample_ms) < kMinSampleInterval_ms) return;

  Sample s;
  const float trackRad = trackDeg * kDegToRad;
  s.vEast_kph = groundSpeedKph * sinf(trackRad);
  s.vNorth_kph = groundSpeedKph * cosf(trackRad);
  s.turn_deg = (gCount == 0) ? 0.0f : wrapDelta(trackDeg - gLastTrack_deg);
  s.t_ms = now_ms;
  push(s);
  gLastTrack_deg = trackDeg;
  gLastSample_ms = now_ms;

  size_t start = 0;
  if (!findCircleWindow(start)) return;

  // A full turn that took too long is wind drift / a wide arc, not a circle.
  if ((at(gCount - 1).t_ms - at(start).t_ms) > kMaxCircleDuration_ms) return;

  CircleFit fit;
  if (!fitCircle(start, fit)) return;

  if (publishIfValid(fit)) {
    // Require a fresh full turn before the next estimate.
    windEstimatorReset();
  }
}

void updateWindEstimator() {
  if (windEstimateValid &&
      (uint32_t)(millis() - gWindPublished_ms) > kWindMaxAge_ms) {
    windEstimateValid = false;
  }

  // Only consume complete, fresh GPS updates. Reusing the same sample on
  // every loop pass would bias the fit.
  const bool speedFresh = gps.speed.isValid() &&
                          gps.speed.age() < kGpsSampleMaxAge_ms;
  const bool courseFresh = gps.course.isValid() &&
                           gps.course.age() < kGpsSampleMaxAge_ms;
  if (!speedFresh || !courseFresh) {
    windEstimatorReset();
    return;
  }

  if (!gps.speed.isUpdated() || !gps.course.isUpdated()) return;

  windEstimatorFeed(static_cast<float>(gps.speed.kmph()),
                    static_cast<float>(gps.course.deg()),
                    millis());
}
