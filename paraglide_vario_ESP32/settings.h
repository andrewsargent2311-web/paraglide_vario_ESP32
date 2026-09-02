#ifndef SETTINGS_H
#define SETTINGS_H

#include <Arduino.h>

// =====================================================
// UNITS
// =====================================================
enum AltitudeUnit { ALT_UNIT_METERS = 0,
                     ALT_UNIT_FEET };

enum SpeedUnit { SPEED_UNIT_KMH = 0,
                 SPEED_UNIT_MS,
                 SPEED_UNIT_MPH };

extern AltitudeUnit altitudeUnit;
extern SpeedUnit speedUnit;

// Converts a value already in metres into the pilot's chosen display unit.
float altitudeToDisplay(float meters);
// "m" or "ft" -- matches whatever altitudeToDisplay() just converted to.
const char* altitudeUnitLabel();

// Converts a value already in km/h (GPS ground speed) into the pilot's
// chosen display unit.
float speedKphToDisplay(float kph);
// "km/h", "m/s" or "mph" -- matches speedKphToDisplay().
const char* speedUnitLabel();

// =====================================================
// VARIO CLIMB TONE FREQUENCY RANGE
// climbToneMaxHz always tracks climbToneMinHz + VARIO_TONE_SPAN_HZ -- set
// both together via setClimbToneMinHz() rather than assigning directly.
// =====================================================
#define VARIO_TONE_SPAN_HZ 700

extern int climbToneMinHz;
extern int climbToneMaxHz;

void setClimbToneMinHz(int minHz);

// =====================================================
// ADS-B ALERTS
// A "new threat" is any tracked aircraft within adsbAlertRadiusKm
// horizontally AND adsbAlertVerticalFt vertically -- see the threat scan
// in performADSBUpdate() in the main .ino.
// =====================================================
extern float adsbAlertRadiusKm;    // horizontal trigger distance, km
extern float adsbAlertVerticalFt;  // vertical trigger distance, ft

// Whether a new threat auto-switches the display to the ADS-B page.
// The 5s intercept alarm tone (see adsbAlarmMuted) fires regardless of
// this setting -- it only controls what's on screen, not the audio alert.
extern bool adsbAutoJumpEnabled;

// Mutes just the ADS-B intercept alarm tone, independent of the main
// vario mute (buzzerMuted in the main .ino).
extern bool adsbAlarmMuted;

// =====================================================
// WEATHER
// =====================================================
// How often updateWeather() polls the Zephyr station network.
extern unsigned long weatherPollIntervalMs;

// How many of the tracked stations drawWeatherPage() actually draws.
// TRACKED_METERS (main .ino) is the max number of stations collected and
// sorted by distance -- this must never exceed it.
extern uint8_t weatherStationsShown;

// Wind speed, gust, and estimated airspeed readouts (Weather page and the
// Paraglider page's WIND/AIRSPEED box) are converted with the same
// speedKphToDisplay()/speedUnitLabel() used for GPS ground speed above,
// so all speed readouts in the app always agree on units.

#endif  // SETTINGS_H
