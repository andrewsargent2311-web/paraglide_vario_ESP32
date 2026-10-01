#pragma once
// Vario -- BMP580 barometer sampling, altitude / climb-rate state, QNH
// calibration. BMP580 probing/config at boot stays inline in setup(); all
// audio lives in Buzzer.cpp.
#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP5xx.h>

extern Adafruit_BMP5xx bmp;
extern bool bmpOK;
// BMP58x breakouts use either 0x47 (default) or 0x46 (SDO/address jumper).
// Probe both: the barometer is an external flight sensor, not onboard.
constexpr uint8_t BMP5XX_DEFAULT_I2C_ADDR = 0x47;
constexpr uint8_t BMP5XX_ALT_I2C_ADDR = 0x46;

// =====================================================
// Variov- Variables
// =====================================================
// Tuning
#define SEA_LEVEL_QNH_DEFAULT 1013.25f
// QNH starts at SEA_LEVEL_QNH_DEFAULT on boot (no waiting for GPS), is
// replaced by a GPS-derived value on the first good fix, then refreshed
// every QNH_RECALIBRATION_INTERVAL_MS. See updateVario().

// How long a continuous run of good-quality GPS fixes (see
// gpsAltitudeGood in updateVario()) is averaged over before calibrating
// QNH from it. A single instantaneous GPS altitude sample is noisy
// enough (GPS vertical error is typically 2-3x worse than horizontal,
// and HDOP doesn't bound it specifically) that calibrating off just one
// fix can lock in a QNH that's meaningfully wrong for the rest of the
// flight -- averaging over a real time window smooths that out. If GPS
// quality drops mid-window (gpsAltitudeGood goes false), the window is
// abandoned and a fresh one starts from scratch on the next good fix,
// rather than silently averaging across a gap.
#define QNH_GPS_AVERAGE_MS 10000UL

// A window that reaches QNH_GPS_AVERAGE_MS with fewer fresh fixes than
// this is treated as not enough data to trust (e.g. a flaky GPS module
// only reporting a handful of updates in the window) -- the window
// resets and tries again rather than calibrating off too few samples.
// At a typical 1Hz GPS this is up to half the fixes a full
// QNH_GPS_AVERAGE_MS window could contain missed and still trusted --
// worth tightening if that proves too lenient in practice now that the
// window itself is shorter than it was.
#define QNH_GPS_MIN_SAMPLES 5

// Once calibrated, QNH is re-averaged and updated on this cadence for
// as long as the flight continues, rather than being locked in once and
// never touched again -- real atmospheric pressure drifts over a
// multi-hour flight as weather systems move through, and an unchanging
// QNH would let indicated altitude/AGL quietly drift away from reality
// over that time. Each recalibration reuses the exact same
// QNH_GPS_AVERAGE_MS averaging window and QNH_GPS_MIN_SAMPLES floor as
// the initial calibration -- see updateVario().
#define QNH_RECALIBRATION_INTERVAL_MS (15UL * 60UL * 1000UL)
#define CLIMB_WINDOW_N 8
#define BARO_SAMPLE_MS 100

extern float altWindow[CLIMB_WINDOW_N];
extern unsigned long timeWindow[CLIMB_WINDOW_N];
extern int windowCount;
extern int windowIndex;
extern unsigned long lastBaroSample;
extern float currentAltitudeM;
extern float currentClimbRateMS;
extern float currentQNH;
extern bool qnhCalibrated;
extern bool qnhIsFallback;
extern float currentPressureHpa;   // latest BMP580 pressure, hPa

void updateVario();
float computeClimbRateLeastSquares();
