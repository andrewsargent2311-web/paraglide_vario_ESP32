#include "Vario.h"
#include "Gps.h"
#include "settings.h"
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

//=====================================================
//VARIO: sample baro, push into regression window, compute climb rate
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
    if (debugNow) {
      Serial.println("[VARIO DEBUG] bmp.performReading() FAILED -- skipping this cycle");
      lastVarioDebug = millis();
    }
    return;
  }

  lastSuccessfulBaroSample = millis();
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
          bmp.pressure / powf(1.0f - (gpsAltM / 44330.0f), 1.0f / 0.1903f);

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
    float dt = (millis() - timeWindow[lastIdx]) / 1000.0f;

    if (dt > 0.001f) {
      float impliedRateMS = (newAltitudeM - altWindow[lastIdx]) / dt;

      if (fabsf(impliedRateMS) > REJECT_RATE_MS) {
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

  altWindow[windowIndex] = newAltitudeM;  // fixed-reference altitude, see above
  timeWindow[windowIndex] = millis();
  windowIndex = (windowIndex + 1) % CLIMB_WINDOW_N;
  if (windowCount < CLIMB_WINDOW_N) windowCount++;

  if (windowCount >= 3) {
    currentClimbRateMS = computeClimbRateLeastSquares();
    updateClimbRateAverage(currentClimbRateMS, millis());
  }

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
