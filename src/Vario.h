#pragma once

#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP5xx.h>

// =====================================================
// VARIO / BAROMETER (BMP580)
// -----------------------------------------------------
// I2C_SDA/I2C_SCL are declared under this same "BMP580 BAROMETER" banner
// in the original .ino, but they feed one shared Wire.begin() call used
// by FOUR devices (BMP580, SHTC3, RTC, ES8311) -- not just this one.
// They stay as plain #defines in the main .ino, not moved here.
//
// BMP580 probing + config-at-boot (setPowerMode/oversampling/IIR/ODR,
// the 3-iteration test-read loop) stays inline in setup(), unchanged --
// it just references the bmp/bmpOK externs below instead of file-local
// globals.
//
// SINK_ALARM_MS, SINK_TONE_MAX_HZ, SINK_TONE_MIN_HZ, SINK_TONE_MAX_MS
// and CLIMB_TONE_MAX_MS are declared under this same "Variov-
// Variables" banner in the original file, but are only ever read inside
// updateI2sAudioBuzzer() -- per the plan's shared/ambiguous ownership
// call, those move to Buzzer.h, not here.
// =====================================================

// BMP58x breakouts use either 0x47 (default) or 0x46 (SDO/address jumper).
// Probe both: the barometer is an external flight sensor, not onboard.
constexpr uint8_t BMP5XX_DEFAULT_I2C_ADDR = 0x47;
constexpr uint8_t BMP5XX_ALT_I2C_ADDR = 0x46;

extern Adafruit_BMP5xx bmp;
extern bool bmpOK;

// ---- Tuning ----
#define SEA_LEVEL_QNH_DEFAULT 1013.25f

// If GPS hasn't produced a usable altitude fix (see gpsAltitudeGood in
// updateVario()) within this long after boot -- no module wired, no sky
// view, whatever the cause -- stop waiting on it and default QNH to
// standard atmosphere so altitude/climb-rate keep running off the BMP580
// alone instead of sitting "not calibrated" for the whole flight.
#define GPS_QNH_FALLBACK_MS 60000UL

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
#define QNH_GPS_AVERAGE_MS 30000UL

// A window that reaches QNH_GPS_AVERAGE_MS with fewer fresh fixes than
// this is treated as not enough data to trust (e.g. a flaky GPS module
// only reporting a handful of updates in 30s) -- the window resets and
// tries again rather than calibrating off too few samples.
#define QNH_GPS_MIN_SAMPLES 5
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

// NOTE: lastBeepToggle / lastSinkBeep were declared alongside this
// state in the original file but don't appear to be read or written
// anywhere else in the codebase provided -- likely leftovers from an
// older buzzer implementation, superseded by climbToneOn /
// climbPulseStart / sinkAlarmStart (now declared in Buzzer.h). Carried
// forward here, unused, since this is physically where they lived in
// the source.
//
// beepOn was declared in this same block originally, but tracing its
// uses turned up a second write site inside updatePageButton()'s
// mute-toggle handler (`beepOn = false;`), alongside pageBeepUntil and
// setToneFrequency(0) -- so it isn't purely Vario-local like the two
// below. It now lives in Buzzer.h instead, so PageButton (which already
// depends on Buzzer.h for playFeedbackTone()/muteToneActive/etc.) can
// reach it without also taking on a new dependency on this file.
extern unsigned long lastBeepToggle;
extern unsigned long lastSinkBeep;

void updateVario();
float computeClimbRateLeastSquares();
