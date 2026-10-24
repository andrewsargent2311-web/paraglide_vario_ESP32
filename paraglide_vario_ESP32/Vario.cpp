#include "Vario.h"
#include "Gps.h"
#include "settings.h"
#include "Utils.h"
#include <Wire.h>
#include <math.h>

Adafruit_BMP5xx bmp;
bool bmpOK = false;
float currentPressureHpa = NAN;   // latest BMP580 pressure, hPa

float altWindow[CLIMB_WINDOW_N];
unsigned long timeWindow[CLIMB_WINDOW_N];
int windowCount = 0;
int windowIndex = 0;
unsigned long lastBaroSample = 0;
unsigned long lastSuccessfulBaroSample = 0;
float currentAltitudeM = 0.0f;
float currentPressureAltStdM = NAN;
float currentClimbRateMS = 0.0f;
float currentClimbRateAvgMS = 0.0f;
float currentQNH = SEA_LEVEL_QNH_DEFAULT;
// QNH is usable from the very first loop: boot starts on standard
// atmosphere (1013.25) so altitude/vario/AGL etc. run immediately and
// nothing waits on GPS. qnhIsFallback stays true until a real GPS-derived
// calibration replaces it, after which QNH is refreshed every
// QNH_RECALIBRATION_INTERVAL_MS.
bool qnhCalibrated = true;
// True while the current QNH is the boot default (1013.25) rather than a
// real GPS-derived value. Cleared by the first successful GPS calibration.
bool qnhIsFallback = true;

bool initBaro() {
  // Cheap presence check before touching the library's begin(): a plain
  // I2C address probe is bounded by Wire.setTimeOut(), so an absent device
  // cannot leave the initialization path waiting indefinitely.
  uint8_t bmpAddress = 0;
  if (i2cDevicePresent(BMP5XX_DEFAULT_I2C_ADDR)) {
    bmpAddress = BMP5XX_DEFAULT_I2C_ADDR;
  } else if (i2cDevicePresent(BMP5XX_ALT_I2C_ADDR)) {
    bmpAddress = BMP5XX_ALT_I2C_ADDR;
  }
  bmpOK = (bmpAddress != 0) && bmp.begin(bmpAddress, &Wire);
  if (bmpOK) {
    // begin() leaves the sensor in NORMAL mode. Per the BMP5xx datasheet,
    // OSR/ODR/press-enable config registers should only be written while in
    // STANDBY -- but setOutputDataRate()/setPressureOversampling()/
    // enablePressure() don't enforce that themselves (only the IIR filter
    // setter does), so writing them straight after begin() means they hit
    // the sensor mid-measurement with no guarantee they're actually applied.
    // Force standby first, configure everything, then switch to NORMAL last
    // so measurement only starts once the config is known-good.
    bmp.setPowerMode(BMP5XX_POWERMODE_STANDBY);
    bmp.setTemperatureOversampling(BMP5XX_OVERSAMPLING_2X);
    bmp.setPressureOversampling(BMP5XX_OVERSAMPLING_8X);
    bmp.setIIRFilterCoeff(BMP5XX_IIR_FILTER_COEFF_3);
    bmp.setOutputDataRate(BMP5XX_ODR_50_HZ);
    bmp.enablePressure(true);
    bmp.setPowerMode(BMP5XX_POWERMODE_NORMAL);
  }
  return bmpOK;
}

bool baroIsFresh() {
  return bmpOK && lastSuccessfulBaroSample != 0 &&
         (uint32_t)(millis() - (uint32_t)lastSuccessfulBaroSample) < 2000UL;
}

bool baroAltitudeTrusted() {
  return baroIsFresh() && !qnhIsFallback;
}

// =====================================================
// KALMAN CLIMB-RATE ESTIMATOR (see Vario.h for the rationale and tuning)
// =====================================================
void VarioKalman::reset() {
  initialised_ = false;
  altM_ = 0.0f;
  vsMS_ = 0.0f;
  p00_ = p01_ = p11_ = 0.0f;
  lastMs_ = 0;
  consecutiveRejects_ = 0;
  rejectedTotal_ = 0;
}

// (Re)start the altitude estimate on this sample. The vertical speed is
// deliberately left alone: the caller zeroes it when a stale value would
// be wrong (first sample, long gap). The wide vertical-speed variance makes
// the filter converge quickly on whatever the next samples say.
void VarioKalman::seed(float altM, uint32_t sampleMs) {
  initialised_ = true;
  altM_ = altM;
  p00_ = cfg_.altSigmaM * cfg_.altSigmaM;
  p01_ = 0.0f;
  p11_ = cfg_.initVsSigmaMS * cfg_.initVsSigmaMS;
  lastMs_ = sampleMs;
  consecutiveRejects_ = 0;
}

bool VarioKalman::update(float altM, uint32_t sampleMs) {
  if (!isfinite(altM)) return false;  // never let a bad read into the state

  if (!initialised_) {
    vsMS_ = 0.0f;
    seed(altM, sampleMs);
    return true;
  }

  const uint32_t dtMs = sampleMs - lastMs_;  // unsigned: safe across millis() wrap
  if (dtMs == 0) return false;
  if (dtMs > cfg_.maxGapMs) {  // also catches a clock that went backwards
    vsMS_ = 0.0f;
    seed(altM, sampleMs);
    return true;
  }

  // ---- Predict: constant vertical speed, white-noise acceleration ----
  const float dt = (float)dtMs * 0.001f;
  const float dt2 = dt * dt;
  const float q = cfg_.accelSigmaMS2 * cfg_.accelSigmaMS2;

  const float altPred = altM_ + vsMS_ * dt;
  const float p00 = p00_ + dt * (2.0f * p01_ + dt * p11_) + q * dt2 * dt2 * 0.25f;
  const float p01 = p01_ + dt * p11_ + q * dt2 * dt * 0.5f;
  const float p11 = p11_ + q * dt2;

  // ---- Gate: is this sample plausible given the prediction? ----
  const float s = p00 + cfg_.altSigmaM * cfg_.altSigmaM;  // innovation variance
  const float innovation = altM - altPred;

  const float sigmaGateM2 = cfg_.gateSigmas * cfg_.gateSigmas * s;
  const float gateM2 = (sigmaGateM2 > cfg_.minGateM * cfg_.minGateM) ? sigmaGateM2 : cfg_.minGateM * cfg_.minGateM;

  if (innovation * innovation > gateM2) {
    rejectedTotal_++;
    altM_ = altPred;  // keep coasting on the prediction
    p00_ = p00;
    p01_ = p01;
    p11_ = p11;
    lastMs_ = sampleMs;
    if (++consecutiveRejects_ > cfg_.maxConsecutiveRejects) {
      seed(altM, sampleMs);  // it kept saying the same thing: the altitude really moved
      return true;
    }
    return false;
  }
  consecutiveRejects_ = 0;

  // ---- Update ----
  const float k0 = p00 / s;
  const float k1 = p01 / s;
  altM_ = altPred + k0 * innovation;
  vsMS_ += k1 * innovation;
  p00_ = (1.0f - k0) * p00;
  p01_ = (1.0f - k0) * p01;
  p11_ = p11 - k1 * p01;
  if (p11_ < 0.0f) p11_ = 0.0f;  // float rounding guard; true value is >= 0
  lastMs_ = sampleMs;
  return true;
}

// ---------------------------------------------------------------------
// File-local state. Everything here is touched only from updateVario()
// (loop task); the rest of the firmware reads the published
// currentClimbRateMS / currentClimbRateAvgMS globals as before.
// ---------------------------------------------------------------------
static VarioKalman varioKalman;
static VarioDiagnostics varioDiag = {};
static bool haveRead = false;               // a successful baro read has happened
static uint32_t lastReadMs = 0;             // timestamp of the previous successful read
static bool haveAcceptedSample = false;     // an estimator has taken a sample
static uint32_t lastAcceptedSampleMs = 0;

const VarioDiagnostics& varioDiagnostics() { return varioDiag; }

void resetVarioDiagnostics() {
  varioDiag = VarioDiagnostics{};
  haveRead = false;
}

// Gap between consecutive successful baro reads. A late sample means
// loop() was held up (display redraw, SD flush, I2C retry...).
static void recordReadTiming(uint32_t sampleMs) {
  if (haveRead) {
    const uint32_t gapMs = sampleMs - lastReadMs;
    if (gapMs > varioDiag.maxGapMs) varioDiag.maxGapMs = gapMs;
    if (gapMs > VARIO_LATE_SAMPLE_MS) varioDiag.lateSamples++;
  }
  varioDiag.samples++;
  lastReadMs = sampleMs;
  haveRead = true;
}

static void printVarioDiagnosticsIfDue(uint32_t nowMs) {
  if (!VARIO_DIAG_PRINT) return;
  static uint32_t lastPrintMs = 0;
  if ((uint32_t)(nowMs - lastPrintMs) < VARIO_DIAG_PRINT_MS) return;
  lastPrintMs = nowMs;
  Serial.printf("[VARIO DIAG] samples=%lu late=%lu maxGap=%lums readFail=%lu gateRej=%lu kfRej=%lu\n",
                (unsigned long)varioDiag.samples, (unsigned long)varioDiag.lateSamples,
                (unsigned long)varioDiag.maxGapMs, (unsigned long)varioDiag.readFailures,
                (unsigned long)varioDiag.gateRejects, (unsigned long)varioDiag.kalmanRejects);
}

// If nothing has been accepted for VARIO_STALE_TIMEOUT_MS, stop publishing
// the last climb rate: left alone it would keep the buzzer playing the same
// beep for as long as the sensor stayed silent.
static void expireStaleClimbRate(uint32_t nowMs) {
  if (!haveAcceptedSample) return;
  if ((uint32_t)(nowMs - lastAcceptedSampleMs) < VARIO_STALE_TIMEOUT_MS) return;

  currentClimbRateMS = 0.0f;
  resetClimbRateAverage();
  varioKalman.reset();
  windowCount = 0;
  windowIndex = 0;
  haveAcceptedSample = false;  // once per stall, not on every call
}

// Push one accepted altitude sample (fixed 1013.25 hPa reference) into both
// estimators and publish the climb rate. The least-squares window is kept
// running even when the Kalman filter is selected, so the two can be
// compared by flipping VARIO_USE_KALMAN.
static void feedClimbEstimators(float fixedRefAltM, uint32_t sampleMs) {
  const bool kalmanAccepted = varioKalman.update(fixedRefAltM, sampleMs);
  if (!kalmanAccepted) {
    varioDiag.kalmanRejects++;
    expireStaleClimbRate(sampleMs);
    if (VARIO_USE_KALMAN) return;
  }

  altWindow[windowIndex] = fixedRefAltM;
  timeWindow[windowIndex] = sampleMs;
  windowIndex = (windowIndex + 1) % CLIMB_WINDOW_N;
  if (windowCount < CLIMB_WINDOW_N) windowCount++;

  float rateMS = currentClimbRateMS;  // unchanged if no estimator is ready yet
  if (VARIO_USE_KALMAN) {
    if (varioKalman.initialised()) rateMS = varioKalman.climbRateMS();
  } else if (windowCount >= 3) {
    rateMS = computeClimbRateLeastSquares();
  }

  currentClimbRateMS = rateMS;
  updateClimbRateAverage(rateMS, sampleMs);
  lastAcceptedSampleMs = sampleMs;
  haveAcceptedSample = true;
}

//=====================================================
//VARIO: sample baro, update the climb-rate estimator
//=====================================================
void updateVario() {
  // Temporary throttled diagnostics -- ALTITUDE/CLIMB RATE only need
  // bmpOK && windowCount > 0, which is completely independent of GPS,
  // so if those boxes are stuck on "--" the cause has to be here: either
  // dataReady()/performReading() failing every call, or this function
  // not running at all. Prints at most once every 2s so it won't flood
  // the serial monitor. Safe to remove once the real cause is found.
  static unsigned long lastVarioDebug = 0;
  bool debugNow = (millis() - lastVarioDebug >= 2000);


  if (!bmp.performReading()) {
    varioDiag.readFailures++;
    expireStaleClimbRate(millis());
    if (debugNow) {
      Serial.println("[VARIO DEBUG] bmp.performReading() FAILED -- skipping this cycle");
      lastVarioDebug = millis();
    }
    return;
  }

  // One timestamp for this sample, taken right after the read. Everything
  // below (outlier gate, window, Kalman filter) uses it, instead of each
  // calling millis() again after QNH work and Serial prints have run.
  lastSuccessfulBaroSample = millis();
  const uint32_t sampleMs = (uint32_t)lastSuccessfulBaroSample;
  recordReadTiming(sampleMs);
  printVarioDiagnosticsIfDue(sampleMs);
  currentPressureHpa = bmp.pressure;

  // Thresholds relaxed from the original (satellites >= 6, HDOP <= 2.5)
  // after real-world testing on a hillside site never held a fix that
  // strict for the full averaging window, even with a clear sky. >= 5
  // satellites / HDOP <= 3 is still solidly "good" GPS accuracy (HDOP
  // <5 is generally considered good, <2 excellent), just not demanding
  // near-perfect satellite geometry to calibrate.
  bool gpsAltitudeGood =
    gps.altitude.isValid() && gps.altitude.age() < 2000 && gps.satellites.isValid() && gps.satellites.value() >= 5 && gps.hdop.isValid() && gps.hdop.hdop() <= 3.0;

  // Runs the real GPS-derived calibration the first time a good fix
  // shows up, AND -- if we're currently sitting on the no-GPS fallback
  // value -- also the first time a good fix shows up *after* that, so a
  // merely-slow GPS still gets upgraded to a proper calibration instead
  // of being stuck on 1013.25 for the rest of the flight. AND, once
  // genuinely calibrated (qnhIsFallback == false), again every
  // QNH_RECALIBRATION_INTERVAL_MS from that point on, so QNH keeps
  // tracking real atmospheric pressure changes over a long flight
  // instead of staying frozen at whatever it was on takeoff.
  //
  // Averages GPS altitude over QNH_GPS_AVERAGE_MS of continuous
  // good-quality fixes rather than calibrating off a single instantaneous
  // sample -- see QNH_GPS_AVERAGE_MS's comment above for why a one-shot
  // sample proved unreliable in real-world testing. qnhAvgActive/
  // qnhAvgStartMs/qnhAvgAltSum/qnhAvgCount persist this averaging window
  // across calls; static rather than global since nothing outside this
  // function needs them. lastQnhCalibrationMs tracks when calibration
  // last actually succeeded, so the periodic-recalibration check below
  // has something to measure from.
  static bool qnhAvgActive = false;
  static unsigned long qnhAvgStartMs = 0;
  static double qnhAvgAltSum = 0.0;
  static double qnhAvgPressureSum = 0.0;
  static uint16_t qnhAvgCount = 0;
  static uint32_t lastGpsAltAge = UINT32_MAX;
  // Set when a real GPS calibration succeeds; the 15-minute refresh timer
  // is measured from that moment (not from boot).
  static unsigned long lastQnhCalibrationMs = 0;

  bool qnhRecalibrationDue =
    qnhCalibrated && !qnhIsFallback &&
    (millis() - lastQnhCalibrationMs >= QNH_RECALIBRATION_INTERVAL_MS);

  if (gpsAltitudeGood && (!qnhCalibrated || qnhIsFallback || qnhRecalibrationDue)) {

    if (!qnhAvgActive) {
      // First good fix, or the periodic recalibration interval just
      // came due -- start a fresh averaging window.
      qnhAvgActive = true;
      qnhAvgStartMs = millis();
      qnhAvgAltSum = 0.0;
      qnhAvgPressureSum = 0.0;
      qnhAvgCount = 0;
      lastGpsAltAge = gps.altitude.age();  // don't count the fix already on hand twice
    }

    // Only bank a sample once per fresh GPS sentence. isUpdated() clears
    // itself on read, so this can't double-count the same fix just
    // because updateVario() runs far more often than the GPS module
    // actually reports a new position (typically 1Hz) -- without this
    // check, a single fix held between GPS updates would get counted
    // once per updateVario() call and dominate the average.
    // NOTE: don't use gps.altitude.isUpdated() here. TinyGPSPlus clears
    // that flag whenever ANYONE reads altitude.feet()/meters(), and
    // loop() reads it every pass, so this function almost never saw it
    // set and the window never filled. Instead detect a new fix by the
    // altitude age() dropping back toward 0 (it resets on each new GGA).
    uint32_t altAge = gps.altitude.age();
    if (altAge < lastGpsAltAge) {
      qnhAvgAltSum += gps.altitude.meters();
      qnhAvgPressureSum += bmp.pressure;
      qnhAvgCount++;
    }
    lastGpsAltAge = altAge;

    if (debugNow && qnhAvgActive) {
      Serial.printf("[VARIO DEBUG] QNH averaging: %u samples over %lus/%lus\n",
                    qnhAvgCount, (millis() - qnhAvgStartMs) / 1000UL, QNH_GPS_AVERAGE_MS / 1000UL);
    }

    if (millis() - qnhAvgStartMs >= QNH_GPS_AVERAGE_MS) {

      if (qnhAvgCount >= QNH_GPS_MIN_SAMPLES) {

        float gpsAltM = (float)(qnhAvgAltSum / qnhAvgCount);

        float calculatedQNH =
          (float)(qnhAvgPressureSum / qnhAvgCount) /
          powf(1.0f - (gpsAltM / 44330.0f), 1.0f / 0.1903f);

        if (calculatedQNH >= 850.0f && calculatedQNH <= 1100.0f) {

          bool wasFallback = qnhIsFallback;
          currentQNH = calculatedQNH;
          qnhCalibrated = true;
          qnhIsFallback = false;
          lastQnhCalibrationMs = millis();

          if (wasFallback) {
            Serial.print("QNH upgraded from GPS altitude (fallback replaced): ");
          } else if (qnhRecalibrationDue) {
            Serial.print("QNH re-calibrated from GPS altitude (15-minute refresh): ");
          } else {
            Serial.print("QNH calibrated from GPS altitude: ");
          }
          Serial.printf("%.2f (averaged over %u fixes / %lus)\n",
                        currentQNH, qnhAvgCount, QNH_GPS_AVERAGE_MS / 1000UL);

        } else {
          Serial.printf("[VARIO] Averaged GPS altitude produced an implausible QNH (%.1f) -- discarding, retrying\n", calculatedQNH);
        }

      } else {
        Serial.printf("[VARIO] QNH averaging window elapsed with only %u fresh fixes (need %u) -- retrying\n",
                      qnhAvgCount, QNH_GPS_MIN_SAMPLES);
      }

      // Reset either way -- a successful non-recalibration calibration
      // means this whole block won't fire again until the next
      // QNH_RECALIBRATION_INTERVAL_MS comes due (the qnhCalibrated &&
      // !qnhIsFallback && recalibration-due check above); a failed/
      // underfilled window just starts a fresh attempt on the next good
      // fix, same as before.
      qnhAvgActive = false;
    }

  } else {

    // Either GPS quality isn't good enough right now, or we're already
    // calibrated and not yet due for a recalibration -- either way,
    // abandon any in-progress averaging window rather than let a
    // dropped-out stretch silently count toward it. A later good fix
    // starts a fresh window from scratch. (No-op most of the time once
    // already calibrated, since qnhAvgActive is already false between
    // recalibration windows.)
    qnhAvgActive = false;
  }

  // Altitude for the climb-rate window uses a FIXED reference (standard
  // 1013.25 hPa), NOT currentQNH. QNH gets replaced by a GPS-derived value
  // shortly after boot and again every QNH_RECALIBRATION_INTERVAL_MS; if the
  // window held QNH-corrected altitude, each of those updates would add a
  // tens-of-metres step to the samples and the regression would read it as
  // a huge climb/sink spike. Climb RATE doesn't depend on the reference, so
  // the window stays on the fixed one and only currentAltitudeM (display,
  // AGL, logging) gets the QNH-corrected value.
  float newAltitudeM = bmp.readAltitude(SEA_LEVEL_QNH_DEFAULT);

  // ------------------------------------------------------------------
  // Outlier rejection -- guards against a single corrupted I2C read
  // (suspected cause: RF coupling into the SDA/SCL wiring from the
  // FANET radio during a TX burst) poisoning the climb-rate window.
  // A paraglider physically can't jump more than a few m/s between two
  // consecutive ~BARO_SAMPLE_MS-apart samples, so anything wildly
  // outside that is treated as a bad sample and dropped rather than
  // fed into the regression. Tune REJECT_RATE_MS if this ever proves
  // too tight/loose in practice.
  // ------------------------------------------------------------------
  static constexpr float REJECT_RATE_MS = 15.0f;  // m/s

  if (windowCount > 0) {
    int lastIdx = (windowIndex + CLIMB_WINDOW_N - 1) % CLIMB_WINDOW_N;
    float dt = (uint32_t)(sampleMs - (uint32_t)timeWindow[lastIdx]) / 1000.0f;

    if (dt > 0.001f) {
      float impliedRateMS = (newAltitudeM - altWindow[lastIdx]) / dt;

      if (fabsf(impliedRateMS) > REJECT_RATE_MS) {
        varioDiag.gateRejects++;
        expireStaleClimbRate(sampleMs);
        if (debugNow) {
          Serial.printf(
            "[VARIO DEBUG] Rejected outlier sample: alt=%.1f m implied=%.1f m/s "
            "(last alt=%.1f m, dt=%.3f s) -- keeping previous window\n",
            newAltitudeM, impliedRateMS, altWindow[lastIdx], dt);
          lastVarioDebug = millis();
        }
        return;
      }
    }
  }

  // QNH-corrected altitude for everything that shows or logs altitude.
  currentAltitudeM = bmp.readAltitude(currentQNH);
  currentPressureAltStdM = newAltitudeM;

  // Fixed-reference altitude into the estimators, see above.
  feedClimbEstimators(newAltitudeM, sampleMs);

  // if (debugNow) {
  //   Serial.printf(
  //     "[VARIO DEBUG] OK -- pressure=%.2f hPa, QNH=%.2f (calibrated=%d, fallback=%d), "
  //     "altitude=%.1f m, windowCount=%d, climb=%.2f m/s\n",
  //     bmp.pressure, currentQNH, qnhCalibrated, qnhIsFallback,
  //     currentAltitudeM, windowCount, currentClimbRateMS);
  //   lastVarioDebug = millis();
  // }
}

// ---------------------------------------------------------------------
// Averaged climb rate for display/glide ratio -- see DISPLAY_CLIMB_AVG_MS.
// Time-stamped ring buffer so the average covers a fixed span of time even
// if a sample or two was dropped by the outlier check.
// ---------------------------------------------------------------------
static float avgRate[DISPLAY_CLIMB_BUF_N];
static unsigned long avgTimeMs[DISPLAY_CLIMB_BUF_N];
static int avgHead = 0;   // next slot to write
static int avgCount = 0;  // valid entries (<= DISPLAY_CLIMB_BUF_N)

void resetClimbRateAverage() {
  avgHead = 0;
  avgCount = 0;
  currentClimbRateAvgMS = 0.0f;
}

void updateClimbRateAverage(float rateMS, unsigned long nowMs) {
  avgRate[avgHead] = rateMS;
  avgTimeMs[avgHead] = nowMs;
  avgHead = (avgHead + 1) % DISPLAY_CLIMB_BUF_N;
  if (avgCount < DISPLAY_CLIMB_BUF_N) avgCount++;

  float sum = 0.0f;
  int used = 0;
  for (int i = 0; i < avgCount; i++) {
    int idx = (avgHead + DISPLAY_CLIMB_BUF_N - 1 - i) % DISPLAY_CLIMB_BUF_N;  // newest first
    if (nowMs - avgTimeMs[idx] > DISPLAY_CLIMB_AVG_MS) break;                 // older than the window
    sum += avgRate[idx];
    used++;
  }
  // used >= 1 always: the sample just pushed has age 0.
  currentClimbRateAvgMS = sum / used;
}

float computeClimbRateLeastSquares() {
  float sumT = 0, sumA = 0, sumTT = 0, sumTA = 0;
  unsigned long t0 = timeWindow[(windowIndex + CLIMB_WINDOW_N - windowCount) % CLIMB_WINDOW_N];

  for (int i = 0; i < windowCount; i++) {
    int idx = (windowIndex + CLIMB_WINDOW_N - windowCount + i) % CLIMB_WINDOW_N;
    float t = (timeWindow[idx] - t0) / 1000.0f;
    float a = altWindow[idx];
    sumT += t;
    sumA += a;
    sumTT += t * t;
    sumTA += t * a;
  }

  float n = windowCount;
  float denom = (n * sumTT - sumT * sumT);
  if (fabs(denom) < 1e-6f) return 0.0f;

  return (n * sumTA - sumT * sumA) / denom;
}
