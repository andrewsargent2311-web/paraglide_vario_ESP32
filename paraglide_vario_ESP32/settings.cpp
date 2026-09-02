#include "settings.h"

// Defaults match the values the display code used to have hard-coded:
// altitude in feet, ground speed in km/h, climb tone 500-1200Hz.
AltitudeUnit altitudeUnit = ALT_UNIT_FEET;
SpeedUnit speedUnit = SPEED_UNIT_KMH;

int climbToneMinHz = 500;
int climbToneMaxHz = 500 + VARIO_TONE_SPAN_HZ;

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

// Defaults match the values that used to be hard-coded: 5km/2000ft threat
// trigger, auto-jump on, alarm audible, weather polled every 5 minutes,
// 4 stations shown.
float adsbAlertRadiusKm = 5.0f;
float adsbAlertVerticalFt = 2000.0f;
bool adsbAutoJumpEnabled = true;
bool adsbAlarmMuted = false;

unsigned long weatherPollIntervalMs = 5UL * 60UL * 1000UL;
uint8_t weatherStationsShown = 4;
