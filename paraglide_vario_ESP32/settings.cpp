#include "settings.h"

// Defaults match the values the display code used to have hard-coded:
// altitude in feet, ground speed in km/h, climb tone 500-1200Hz.
AltitudeUnit altitudeUnit = ALT_UNIT_FEET;
SpeedUnit speedUnit = SPEED_UNIT_KMH;

int climbToneMinHz = 500;
int climbToneMaxHz = 500 + VARIO_TONE_SPAN_HZ;

// Default matches the app's original behaviour: automatic NZ time with
// DST applied. utcOffsetHours only takes effect once the pilot picks a
// manual entry from the Time menu.
TimeZoneMode timeZoneMode = TZ_MODE_AUTO_NZ;
int8_t utcOffsetHours = 12;

void getPilotLocalTime(struct tm* outTm) {
  time_t now;
  time(&now);

  if (timeZoneMode == TZ_MODE_MANUAL) {
    // Shift the UTC epoch by the fixed offset and read it back out with
    // gmtime_r() (not localtime_r()) so the NZ_TIMEZONE DST rule set at
    // boot never gets applied on top of it.
    time_t shifted = now + (time_t)utcOffsetHours * 3600L;
    gmtime_r(&shifted, outTm);
  } else {
    // NZ_TIMEZONE (set once via setenv("TZ", ...) in the main .ino's
    // setup()) already encodes the DST transition rule, so localtime_r()
    // handles the adjustment automatically.
    localtime_r(&now, outTm);
  }
}

float altitudeToDisplay(float meters) {
  return (altitudeUnit == ALT_UNIT_FEET) ? (meters * 3.28084f) : meters;
}

const char* altitudeUnitLabel() {
  return (altitudeUnit == ALT_UNIT_FEET) ? "ft" : "m";
}

float speedKphToDisplay(float kph) {
  switch (speedUnit) {
    case SPEED_UNIT_MS: return kph / 3.6f;
    case SPEED_UNIT_MPH: return kph * 0.621371f;
    default: return kph;  // SPEED_UNIT_KMH
  }
}

const char* speedUnitLabel() {
  switch (speedUnit) {
    case SPEED_UNIT_MS: return "m/s";
    case SPEED_UNIT_MPH: return "mph";
    default: return "km/h";  // SPEED_UNIT_KMH
  }
}

void setClimbToneMinHz(int minHz) {
  climbToneMinHz = minHz;
  climbToneMaxHz = minHz + VARIO_TONE_SPAN_HZ;
}

// Defaults match the values that used to be hard-coded (100/500/100/400).
unsigned long climbGapMinMs = 100UL;
unsigned long climbGapMaxMs = 500UL;
unsigned long climbPulseMinMs = 100UL;
unsigned long climbPulseMaxMs = 400UL;

// Default approximates the original hard-coded ES8311 register value
// (0xBF of 0xFF, ~75%) at the nearest 20% step.
uint8_t buzzerVolumePercent = 80;

// Defaults match the values that used to be hard-coded: 5km/2000ft threat
// trigger, auto-jump on, alarm audible, weather polled every 5 minutes,
// 4 stations shown.
float adsbAlertRadiusKm = 5.0f;
float adsbAlertVerticalFt = 2000.0f;
bool adsbAutoJumpEnabled = true;
bool adsbAlarmMuted = false;

// Default matches the original hardcoded far/default rings (30km outer,
// 15km inner).
float adsbRingOuterKm = 30.0f;
float adsbRingInnerKm = 15.0f;

unsigned long weatherPollIntervalMs = 5UL * 60UL * 1000UL;
uint8_t weatherStationsShown = 4;
