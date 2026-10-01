// Vario.cpp -- BMP580 barometer sampling, climb-rate regression, QNH calibration.
//
// Written to the flightcomputer-power-of-ten standard. Public interface is
// unchanged from the previous version (everything declared in Vario.h is
// still defined here with the same name and type), so no other file needs
// to change.
//
// Ownership / threading:
//   updateVario() must be called from ONE task. That task owns every
//   variable in this file. Other tasks currently read the public globals
//   directly (legacy, see the P2-9 note in the change summary).
//   updateVario() writes to Serial, so it must not run in the audio task.
//
// Degraded mode (P2-10): when no good baro sample has arrived for
// kBaroStale_ms, windowCount is reset to 0 and currentClimbRateMS to 0.
// windowCount == 0 is the "no valid baro data" signal, matching how the
// display already decides between a value and "--". currentAltitudeM keeps
// its last value but must not be shown as live while windowCount == 0.

#include "Vario.h"
#include "Gps.h"
#include "settings.h"
#include <cmath>
#include <math.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>

// ---------------------------------------------------------------------
// Public state, declared extern in Vario.h. Names and types are dictated
// by the header (legacy). No new globals are added by this rewrite.
// ---------------------------------------------------------------------
Adafruit_BMP5xx bmp;
bool bmpOK = false;
float currentPressureHpa = NAN;  // latest validated BMP580 pressure, hPa

float altWindow[CLIMB_WINDOW_N];
unsigned long timeWindow[CLIMB_WINDOW_N];
int windowCount = 0;
int windowIndex = 0;
unsigned long lastBaroSample = 0;  // not used here; scheduling is the caller's
float currentAltitudeM = 0.0f;
float currentClimbRateMS = 0.0f;
float currentQNH = SEA_LEVEL_QNH_DEFAULT;
bool qnhCalibrated = false;
// True only while the current QNH came from the no-GPS timeout fallback
// rather than a real GPS-derived calibration, so a later good fix can
// upgrade it without reopening the otherwise one-shot calibration.
bool qnhIsFallback = false;

namespace {

// ---------------------------------------------------------------------
// Constants. Typed copies of the Vario.h macros, plus file-local tuning.
// ---------------------------------------------------------------------
constexpr uint32_t kQnhAverage_ms = QNH_GPS_AVERAGE_MS;
constexpr uint16_t kQnhMinSamples = QNH_GPS_MIN_SAMPLES;
constexpr uint32_t kQnhRecalInterval_ms = QNH_RECALIBRATION_INTERVAL_MS;
constexpr uint32_t kGpsFallback_ms = GPS_QNH_FALLBACK_MS;
constexpr float kDefaultQnh_hPa = SEA_LEVEL_QNH_DEFAULT;

constexpr int kMinRegressionPts = 3;

constexpr float kAltScale_m = 44330.0f;
constexpr float kBaroExponent = 0.1903f;
constexpr float kInvBaroExponent = 1.0f / kBaroExponent;

// A paraglider cannot change altitude faster than this between two
// consecutive samples; anything beyond it is treated as a corrupt I2C read
// (suspected RF coupling from the FANET radio during TX).
constexpr float kRejectRate_mps = 15.0f;

// No accepted baro sample for this long => data is stale (P2-10). Must be
// comfortably longer than the sample period, checked below.
constexpr uint32_t kBaroStale_ms = 1000U;

// Sanity band for a raw pressure reading (P1-6 / P2-10). Taken from my
// recollection of the BMP580 operating range; check against the datasheet.
constexpr float kPressureMin_hPa = 300.0f;
constexpr float kPressureMax_hPa = 1250.0f;

constexpr float kQnhMin_hPa = 850.0f;
constexpr float kQnhMax_hPa = 1100.0f;

// GPS quality gate for QNH calibration. Relaxed from the original
// (>= 6 sats, HDOP <= 2.5) after hillside testing never held a fix that
// strict for the whole averaging window.
constexpr uint32_t kGpsMaxAge_ms = 2000U;
constexpr uint32_t kGpsMinSats = 5U;
constexpr float kGpsMaxHdop = 3.0f;
constexpr float kGpsAltMin_m = -500.0f;   // plausibility band for one GPS altitude
constexpr float kGpsAltMax_m = 9000.0f;

constexpr uint32_t kDiagInterval_ms = 2000U;  // throttle for diagnostic prints
constexpr size_t kLogBufLen = 160U;

static_assert(CLIMB_WINDOW_N >= kMinRegressionPts, "regression needs a window of at least 3");
static_assert(QNH_GPS_MIN_SAMPLES >= 1, "QNH needs at least one GPS sample");
static_assert(QNH_GPS_AVERAGE_MS > 0, "QNH averaging window must be non-zero");
static_assert(kBaroStale_ms > 2U * BARO_SAMPLE_MS, "stale timeout must exceed a couple of sample periods");

// ---------------------------------------------------------------------
// File-local state. Previously function-local statics inside
// updateVario(); gathered here so it is visible and resettable. Owner:
// the task that calls updateVario().
// ---------------------------------------------------------------------
struct VarioState {
  bool started;
  uint32_t start_ms;            // first updateVario() call; reference for the GPS fallback
  uint32_t lastGoodSample_ms;   // last accepted baro sample
  uint32_t lastDiag_ms;         // last throttled diagnostic print

  bool avgActive;               // QNH averaging window in progress
  uint32_t avgStart_ms;
  float avgAltSum_m;
  uint16_t avgCount;
  uint32_t lastCalibration_ms;  // last real (non-fallback) calibration

  uint32_t windowResets;        // times corrupt window indices were repaired
};
VarioState sState = {};

enum class CycleResult : uint8_t { Accepted, ReadFailed, Rejected };

// ---------------------------------------------------------------------
// Logging. Every message is formatted into a fixed stack buffer and
// written with Serial.write(); Serial.printf() would malloc for any
// message over 64 bytes (P1-2). Rate policy (P2-11):
//   * diagnostics go through diagDue(), at most one per kDiagInterval_ms
//     across ALL diagnostic paths;
//   * event messages are either one-shot or bounded by the QNH averaging
//     window (at least kQnhAverage_ms between prints).
// ---------------------------------------------------------------------
__attribute__((format(printf, 1, 2)))
void logLine(const char* fmt, ...) {
  char buf[kLogBufLen];
  va_list args;
  va_start(args, fmt);
  const int n = vsnprintf(buf, sizeof buf, fmt, args);
  va_end(args);
  if (n <= 0) return;
  const size_t len = (static_cast<size_t>(n) < sizeof buf) ? static_cast<size_t>(n) : (sizeof buf - 1U);
  // Debug output: a short write is irrelevant to flight behaviour (P1-4 exception).
  //(void)Serial.write(reinterpret_cast<const uint8_t*>(buf), len);
  //maybe change the above to write to an error buffer then to a logfile inside the SD card?
}

[[nodiscard]] bool diagDue(uint32_t now_ms) {
  if (static_cast<uint32_t>(now_ms - sState.lastDiag_ms) < kDiagInterval_ms) return false;
  sState.lastDiag_ms = now_ms;
  return true;
}

// ---------------------------------------------------------------------
// Pressure / altitude maths
// ---------------------------------------------------------------------
float pressureToAltitude_m(float pressure_hPa, float qnh_hPa) {
  return kAltScale_m * (1.0f - powf(pressure_hPa / qnh_hPa, kBaroExponent));
}

float pressureToQnh_hPa(float pressure_hPa, float alt_m) {
  return pressure_hPa / powf(1.0f - (alt_m / kAltScale_m), kInvBaroExponent);
}

// ---------------------------------------------------------------------
// Window bookkeeping
// ---------------------------------------------------------------------
[[nodiscard]] bool windowStateValid() {
  return windowCount >= 0 && windowCount <= CLIMB_WINDOW_N &&
         windowIndex >= 0 && windowIndex < CLIMB_WINDOW_N;
}

void resetWindow() {
  windowCount = 0;
  windowIndex = 0;
}

// windowIndex / windowCount are public mutable globals, so check they are
// sane before they are used as array indices (P2-15: recover, keep going).
void repairWindowIfCorrupt(uint32_t now_ms) {
  if (windowStateValid()) return;
  resetWindow();
  ++sState.windowResets;
  if (diagDue(now_ms)) {
    logLine("[VARIO] window indices corrupt -- reset (#%lu)\n",
            static_cast<unsigned long>(sState.windowResets));
  }
}

// No good sample for kBaroStale_ms: stop presenting the old numbers as live.
void expireStaleData(uint32_t now_ms) {
  if (windowCount == 0) return;  // already stale, or never had data
  if (static_cast<uint32_t>(now_ms - sState.lastGoodSample_ms) < kBaroStale_ms) return;
  resetWindow();
  currentClimbRateMS = 0.0f;
  if (diagDue(now_ms)) logLine("[VARIO] baro data stale -- window cleared\n");
}

// ---------------------------------------------------------------------
// Baro read
// ---------------------------------------------------------------------
[[nodiscard]] bool readPressure(float& pressure_hPa) {
  if (!bmpOK) return false;
  // DEVIATION(P1-3): bmp.performReading() blocks inside the Adafruit driver
  // and this file cannot set its timeout. NOT VERIFIED: confirm setup()
  // calls Wire.setTimeOut() (or equivalent) so a stuck I2C bus cannot hang
  // this task beyond a few tens of ms.
  if (!bmp.performReading()) return false;
  const float p_hPa = bmp.pressure;
  if (!std::isfinite(p_hPa) || p_hPa < kPressureMin_hPa || p_hPa > kPressureMax_hPa) return false;
  pressure_hPa = p_hPa;
  return true;
}

// ---------------------------------------------------------------------
// GPS input (P1-6). TinyGPS++ has already length- and checksum-checked the
// NMEA sentence; here we gate on quality and check the value is plausible.
// ---------------------------------------------------------------------
[[nodiscard]] bool gpsAltitudeGood() {
  return gps.altitude.isValid() && gps.altitude.age() < kGpsMaxAge_ms &&
         gps.satellites.isValid() && gps.satellites.value() >= kGpsMinSats &&
         gps.hdop.isValid() && static_cast<float>(gps.hdop.hdop()) <= kGpsMaxHdop;
}

// Bank one sample per fresh GPS sentence. isUpdated() clears itself on
// read, so the same fix is never counted twice however often
// updateVario() runs.
void bankGpsSample() {
  if (!gps.altitude.isUpdated()) return;
  const float alt_m = static_cast<float>(gps.altitude.meters());
  if (!(alt_m >= kGpsAltMin_m && alt_m <= kGpsAltMax_m)) return;  // also rejects NaN
  sState.avgAltSum_m += alt_m;
  // Bounded: a window lasts kQnhAverage_ms, so the count cannot approach 65535.
  ++sState.avgCount;
}

// ---------------------------------------------------------------------
// QNH calibration
// ---------------------------------------------------------------------
[[nodiscard]] bool qnhCalibrationWanted(uint32_t now_ms) {
  if (!qnhCalibrated || qnhIsFallback) return true;
  return static_cast<uint32_t>(now_ms - sState.lastCalibration_ms) >= kQnhRecalInterval_ms;
}

// Adopt a new QNH. The stored window altitudes were computed with the old
// QNH, so shift them by the step this change causes at the current
// pressure. Without this the regression sees a fake altitude jump (a
// climb-rate spike) and the outlier filter rejects samples until enough
// time has passed to make the jump look slow enough.
void commitQnh(float qnh_hPa, float pressure_hPa, uint32_t now_ms, bool isFallback) {
  const float step_m = pressureToAltitude_m(pressure_hPa, qnh_hPa) -
                       pressureToAltitude_m(pressure_hPa, currentQNH);
  if (std::isfinite(step_m)) {
    for (int i = 0; i < CLIMB_WINDOW_N; ++i) altWindow[i] += step_m;
  } else {
    resetWindow();  // currentQNH was corrupt; start the window clean
  }
  currentQNH = qnh_hPa;
  qnhCalibrated = true;
  qnhIsFallback = isFallback;
  if (!isFallback) sState.lastCalibration_ms = now_ms;
}

void startAveraging(uint32_t now_ms) {
  sState.avgActive = true;
  sState.avgStart_ms = now_ms;
  sState.avgAltSum_m = 0.0f;
  sState.avgCount = 0U;
}

void finishAveraging(uint32_t now_ms, float pressure_hPa) {
  if (sState.avgCount < kQnhMinSamples) {
    logLine("[VARIO] QNH window ended with %u fixes (need %u) -- retrying\n",
            static_cast<unsigned>(sState.avgCount), static_cast<unsigned>(kQnhMinSamples));
    return;
  }

  const float gpsAlt_m = sState.avgAltSum_m / static_cast<float>(sState.avgCount);
  const float qnh_hPa = pressureToQnh_hPa(pressure_hPa, gpsAlt_m);

  if (!(qnh_hPa >= kQnhMin_hPa && qnh_hPa <= kQnhMax_hPa)) {  // also rejects NaN
    logLine("[VARIO] averaged GPS altitude gave implausible QNH %.1f -- discarding, retrying\n",
            static_cast<double>(qnh_hPa));
    return;
  }

  const char* kind = "calibrated";
  if (qnhIsFallback) {
    kind = "upgraded (fallback replaced)";
  } else if (qnhCalibrated) {
    kind = "re-calibrated (periodic refresh)";
  }
  commitQnh(qnh_hPa, pressure_hPa, now_ms, false);
  logLine("[VARIO] QNH %s from GPS altitude: %.2f hPa (%u fixes / %lus)\n",
          kind, static_cast<double>(qnh_hPa), static_cast<unsigned>(sState.avgCount),
          static_cast<unsigned long>(kQnhAverage_ms / 1000U));
}

void advanceAveraging(uint32_t now_ms, float pressure_hPa) {
  if (!sState.avgActive) startAveraging(now_ms);
  bankGpsSample();

  const uint32_t elapsed_ms = static_cast<uint32_t>(now_ms - sState.avgStart_ms);
  if (diagDue(now_ms)) {
    logLine("[VARIO DEBUG] QNH averaging: %u samples over %lus/%lus\n",
            static_cast<unsigned>(sState.avgCount),
            static_cast<unsigned long>(elapsed_ms / 1000U),
            static_cast<unsigned long>(kQnhAverage_ms / 1000U));
  }
  if (elapsed_ms < kQnhAverage_ms) return;

  finishAveraging(now_ms, pressure_hPa);
  // Reset either way: success waits for the next recalibration interval,
  // a failed window starts fresh on the next good fix.
  sState.avgActive = false;
}

// GPS never came good (missing module, no sky view): stop waiting and run
// the BMP580 alone on standard atmosphere. Flagged as fallback so a later
// good fix still upgrades it. Evaluated independently of GPS state so it
// cannot be starved by a window that keeps failing while GPS looks good.
void applyFallbackQnhIfNeeded(uint32_t now_ms, float pressure_hPa) {
  if (qnhCalibrated) return;
  if (static_cast<uint32_t>(now_ms - sState.start_ms) < kGpsFallback_ms) return;
  commitQnh(kDefaultQnh_hPa, pressure_hPa, now_ms, true);
  logLine("[VARIO] GPS unavailable -- QNH defaulted to %.2f, altitude/vario off BMP580 only\n",
          static_cast<double>(kDefaultQnh_hPa));
}

void updateQnh(uint32_t now_ms, float pressure_hPa) {
  if (gpsAltitudeGood() && qnhCalibrationWanted(now_ms)) {
    advanceAveraging(now_ms, pressure_hPa);
  } else {
    // GPS dropped out, or nothing to calibrate: abandon any partial window
    // so a gap never silently counts toward it.
    sState.avgActive = false;
  }
  applyFallbackQnhIfNeeded(now_ms, pressure_hPa);
}

// ---------------------------------------------------------------------
// Sample acceptance and storage
// ---------------------------------------------------------------------
[[nodiscard]] bool isOutlier(uint32_t now_ms, float alt_m) {
  if (windowCount == 0) return false;

  const int lastIdx = (windowIndex + CLIMB_WINDOW_N - 1) % CLIMB_WINDOW_N;
  const uint32_t dt_ms = static_cast<uint32_t>(now_ms - static_cast<uint32_t>(timeWindow[lastIdx]));
  const float dt_s = static_cast<float>(dt_ms) / 1000.0f;
  if (dt_s <= 0.001f) return false;

  const float rate_mps = (alt_m - altWindow[lastIdx]) / dt_s;
  if (fabsf(rate_mps) <= kRejectRate_mps) return false;

  if (diagDue(now_ms)) {
    logLine("[VARIO DEBUG] outlier rejected: alt=%.1f implied=%.1f m/s last=%.1f dt=%.3f s\n",
            static_cast<double>(alt_m), static_cast<double>(rate_mps),
            static_cast<double>(altWindow[lastIdx]), static_cast<double>(dt_s));
  }
  return true;
}

void pushSample(uint32_t now_ms, float alt_m) {
  currentAltitudeM = alt_m;
  altWindow[windowIndex] = alt_m;
  timeWindow[windowIndex] = now_ms;
  windowIndex = (windowIndex + 1) % CLIMB_WINDOW_N;
  if (windowCount < CLIMB_WINDOW_N) ++windowCount;

  if (windowCount >= kMinRegressionPts) {
    currentClimbRateMS = computeClimbRateLeastSquares();
  }
}

[[nodiscard]] CycleResult runBaroCycle() {
  float pressure_hPa = 0.0f;
  if (!readPressure(pressure_hPa)) return CycleResult::ReadFailed;

  const uint32_t now_ms = millis();  // stamped after the read completes
  currentPressureHpa = pressure_hPa;

  updateQnh(now_ms, pressure_hPa);

  const float alt_m = pressureToAltitude_m(pressure_hPa, currentQNH);
  if (!std::isfinite(alt_m)) return CycleResult::Rejected;
  if (isOutlier(now_ms, alt_m)) return CycleResult::Rejected;

  pushSample(now_ms, alt_m);
  sState.lastGoodSample_ms = now_ms;
  return CycleResult::Accepted;
}

}  // namespace

//=====================================================
// VARIO: sample baro, push into regression window, compute climb rate
//=====================================================
void updateVario() {
  const uint32_t entry_ms = millis();
  if (!sState.started) {
    sState.started = true;
    sState.start_ms = entry_ms;
  }
  repairWindowIfCorrupt(entry_ms);

  const CycleResult result = runBaroCycle();
  if (result == CycleResult::Accepted) return;

  // No usable sample this cycle. Never update the published values; age
  // them out if this has gone on too long.
  const uint32_t now_ms = millis();
  if (result == CycleResult::ReadFailed && diagDue(now_ms)) {
    logLine("[VARIO DEBUG] baro read failed or implausible -- skipping this cycle\n");
  }
  expireStaleData(now_ms);
}

// Least-squares slope of altitude against time over the window, in m/s.
float computeClimbRateLeastSquares() {
  if (!windowStateValid() || windowCount < 2) return 0.0f;

  const int oldest = (windowIndex + CLIMB_WINDOW_N - windowCount) % CLIMB_WINDOW_N;
  const uint32_t t0_ms = static_cast<uint32_t>(timeWindow[oldest]);

  float sumT = 0.0f;
  float sumA = 0.0f;
  float sumTT = 0.0f;
  float sumTA = 0.0f;

  // Bounded: windowCount <= CLIMB_WINDOW_N, checked by windowStateValid().
  for (int i = 0; i < windowCount; ++i) {
    const int idx = (oldest + i) % CLIMB_WINDOW_N;
    const uint32_t dt_ms = static_cast<uint32_t>(static_cast<uint32_t>(timeWindow[idx]) - t0_ms);
    const float t_s = static_cast<float>(dt_ms) / 1000.0f;
    const float a_m = altWindow[idx];
    sumT += t_s;
    sumA += a_m;
    sumTT += t_s * t_s;
    sumTA += t_s * a_m;
  }

  const float n = static_cast<float>(windowCount);
  const float denom = (n * sumTT) - (sumT * sumT);
  if (fabsf(denom) < 1e-6f) return 0.0f;

  return ((n * sumTA) - (sumT * sumA)) / denom;
}
