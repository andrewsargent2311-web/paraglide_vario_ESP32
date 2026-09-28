#include "Vario.h"
#include "Gps.h"  // gps.altitude / gps.satellites / gps.hdop, used for QNH calibration

#include <math.h>

Adafruit_BMP5xx bmp;
bool bmpOK = false;

float altWindow[CLIMB_WINDOW_N];
unsigned long timeWindow[CLIMB_WINDOW_N];
int windowCount = 0;
int windowIndex = 0;
unsigned long lastBaroSample = 0;
float currentAltitudeM = 0.0f;
float currentClimbRateMS = 0.0f;
float currentQNH = SEA_LEVEL_QNH_DEFAULT;
bool qnhCalibrated = false;
bool qnhIsFallback = false;

unsigned long lastBeepToggle = 0;
unsigned long lastSinkBeep = 0;
// beepOn moved to Buzzer.h/.cpp -- see the NOTE in Vario.h.

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

  bool gpsAltitudeGood =
    gps.altitude.isValid() && gps.altitude.age() < 2000 && gps.satellites.isValid() && gps.satellites.value() >= 6 && gps.hdop.isValid() && gps.hdop.hdop() <= 2.5;

  // Runs the real GPS-derived calibration the first time a good fix
  // shows up, AND -- if we're currently sitting on the no-GPS fallback
  // value -- also the first time a good fix shows up *after* that, so a
  // merely-slow GPS still gets upgraded to a proper calibration instead
  // of being stuck on 1013.25 for the rest of the flight. Once genuinely
  // calibrated (qnhIsFallback == false), this never fires again.
  //
  // Averages GPS altitude over QNH_GPS_AVERAGE_MS (30s) of continuous
  // good-quality fixes rather than calibrating off a single instantaneous
  // sample -- see QNH_GPS_AVERAGE_MS's comment above for why a one-shot
  // sample proved unreliable in real-world testing. qnhAvgActive/
  // qnhAvgStartMs/qnhAvgAltSum/qnhAvgCount persist this averaging window
  // across calls; static rather than global since nothing outside this
  // function needs them.
  static bool qnhAvgActive = false;
  static unsigned long qnhAvgStartMs = 0;
  static double qnhAvgAltSum = 0.0;
  static uint16_t qnhAvgCount = 0;

  if (gpsAltitudeGood && (!qnhCalibrated || qnhIsFallback)) {

    if (!qnhAvgActive) {
      // First good fix -- start a fresh 30s averaging window.
      qnhAvgActive = true;
      qnhAvgStartMs = millis();
      qnhAvgAltSum = 0.0;
      qnhAvgCount = 0;
    }

    // Only bank a sample once per fresh GPS sentence. isUpdated() clears
    // itself on read, so this can't double-count the same fix just
    // because updateVario() runs far more often than the GPS module
    // actually reports a new position (typically 1Hz) -- without this
    // check, a single fix held between GPS updates would get counted
    // once per updateVario() call and dominate the average.
    if (gps.altitude.isUpdated()) {
      qnhAvgAltSum += gps.altitude.meters();
      qnhAvgCount++;
    }

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

          Serial.print(wasFallback ? "QNH upgraded from GPS altitude (fallback replaced): "
                                   : "QNH calibrated from GPS altitude: ");
          Serial.printf("%.2f (averaged over %u fixes / %lus)\n",
                        currentQNH, qnhAvgCount, QNH_GPS_AVERAGE_MS / 1000UL);

        } else {
          Serial.printf("[VARIO] Averaged GPS altitude produced an implausible QNH (%.1f) -- discarding, retrying\n", calculatedQNH);
        }

      } else {
        Serial.printf("[VARIO] QNH averaging window elapsed with only %u fresh fixes (need %u) -- retrying\n",
                      qnhAvgCount, QNH_GPS_MIN_SAMPLES);
      }

      // Reset either way -- a successful calibration means this whole
      // block won't run again (the qnhCalibrated && !qnhIsFallback check
      // above short-circuits it); a failed/underfilled window just
      // starts a fresh 30s attempt on the next good fix.
      qnhAvgActive = false;
    }

  } else {

    // Either GPS quality isn't good enough right now, or we're already
    // properly calibrated and don't need this at all -- either way,
    // abandon any in-progress averaging window rather than let a
    // dropped-out stretch silently count toward it. A later good fix
    // starts a fresh 30s window from scratch. (No-op once already
    // calibrated, since qnhAvgActive is already false by then.)
    qnhAvgActive = false;

    if (!qnhCalibrated && millis() >= GPS_QNH_FALLBACK_MS) {
      // GPS never came good (missing/unwired module, or just no fix
      // after a full minute) -- stop waiting on it. Default to standard
      // atmosphere so the BMP580 alone can drive altitude/vario for the
      // rest of the flight. Flagged as a fallback so a later good fix
      // can still upgrade it, above.
      currentQNH = SEA_LEVEL_QNH_DEFAULT;
      qnhCalibrated = true;
      qnhIsFallback = true;

      Serial.println("GPS unavailable -- defaulting QNH to 1013.25, running altitude/vario off BMP580 only");
    }
  }

  float newAltitudeM = bmp.readAltitude(currentQNH);

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

  currentAltitudeM = newAltitudeM;

  altWindow[windowIndex] = currentAltitudeM;
  timeWindow[windowIndex] = millis();
  windowIndex = (windowIndex + 1) % CLIMB_WINDOW_N;
  if (windowCount < CLIMB_WINDOW_N) windowCount++;

  if (windowCount >= 3) {
    currentClimbRateMS = computeClimbRateLeastSquares();
  }

  if (debugNow) {
    Serial.printf(
      "[VARIO DEBUG] OK -- pressure=%.2f hPa, QNH=%.2f (calibrated=%d, fallback=%d), "
      "altitude=%.1f m, windowCount=%d, climb=%.2f m/s\n",
      bmp.pressure, currentQNH, qnhCalibrated, qnhIsFallback,
      currentAltitudeM, windowCount, currentClimbRateMS);
    lastVarioDebug = millis();
  }
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
