#ifndef SETTINGS_H
#define SETTINGS_H

#include <Arduino.h>
#include <time.h>

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
// CLOCK / TIMEZONE
// The system clock (see syncClockFromGPS() in the main .ino) always holds
// true UTC -- it is never adjusted for timezone or DST. All of that is
// applied only when producing a value for on-screen display, via
// getPilotLocalTime() below, so nothing about how the clock is synced or
// stored needs to change based on this setting.
//
// TZ_MODE_AUTO_NZ (the default) reproduces the app's original behaviour:
// NZ time with daylight saving applied automatically, via the NZ_TIMEZONE
// POSIX rule (set once at boot in the main .ino's setup()).
//
// TZ_MODE_MANUAL applies a fixed whole-hour offset from UTC instead
// (utcOffsetHours, -12..+14) -- intended for use outside NZ, where the
// NZ DST rule wouldn't apply anyway. A manual offset does NOT self-adjust
// for DST; if you're using it in a DST-observing region, you'll need to
// change it by hand around the local DST transition dates.
// =====================================================
enum TimeZoneMode { TZ_MODE_AUTO_NZ = 0,
                     TZ_MODE_MANUAL };

extern TimeZoneMode timeZoneMode;
extern int8_t utcOffsetHours;  // only used while timeZoneMode == TZ_MODE_MANUAL

// Fills outTm with the pilot-facing local time (right now), honouring
// timeZoneMode/utcOffsetHours above. Always reads the live system clock.
void getPilotLocalTime(struct tm* outTm);

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
// VARIO BEEP TIMING
// Controls the climb-tone pulse pattern in updateVario() (main .ino):
// as lift gets stronger, the gap between beeps shrinks from
// climbGapMaxMs down to climbGapMinMs, AND each beep's length also
// shrinks, from climbPulseMaxMs down to climbPulseMinMs -- both compress
// together toward the continuous-tone region. Adjustable in 100ms steps,
// 100-1000ms, via Config > Vario Beep in the menu. Defaults match the
// values that used to be hard-coded (100/500/100/400).
//
// NOTE: each of the 4 values below is an independent menu choice --
// nothing enforces climbGapMinMs <= climbGapMaxMs or
// climbPulseMinMs <= climbPulseMaxMs. If either pair ends up inverted,
// the interpolation in updateVario() runs backwards (gaps or pulses
// would lengthen as lift gets stronger, instead of shrinking) -- worth
// keeping min <= max for each pair.
// =====================================================
extern unsigned long climbGapMinMs;
extern unsigned long climbGapMaxMs;
extern unsigned long climbPulseMinMs;
extern unsigned long climbPulseMaxMs;

// =====================================================
// BUZZER VOLUME
// Applied to the ES8311 codec's DAC digital volume register by
// applyBuzzerVolume() (main .ino) -- called once at boot (es8311Init())
// and again immediately whenever this changes via Config > Volume in the
// menu. 20-100: 20% steps up to 80%, then 5% steps from 80-100% (see
// VOLUME_CHOICES_PERCENT in menu.cpp) -- the finer top-end steps exist
// so a speaker that clips at 100% has room to back off a little without
// dropping all the way to 80%. Default (80%) approximates the app's
// original hard-coded register value (0xBF of 0xFF, ~75%).
//
// NOTE: that register is dB-linear (0.5dB per LSB across its range), not
// linear in perceived loudness, so a straightforward "percent of the
// register's full range" mapping means the low end (20%/40%) may come
// out quieter than a literal "20%/40% as loud" would suggest -- worth a
// listen on real hardware; see applyBuzzerVolume() if that mapping needs
// adjusting.
// =====================================================
extern uint8_t buzzerVolumePercent;

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
// ADS-B RANGE RINGS
// The "far" (default) range rings drawn by drawADSBPage() -- outer and
// inner -- shown whenever no aircraft is within the near-zoom trigger
// distance. Chosen from a fixed preset list (see ADSB_RING_OUTER_CHOICES_KM
// in menu.cpp); inner is always half of outer. Default (30km/15km)
// matches the app's original hardcoded values.
//
// The near-zoom trigger and its own ring pair are a fixed 10km/5km,
// triggered whenever any aircraft is within 10km -- that's independent
// of this setting and is NOT changed by it. See drawADSBPage() in the
// main .ino for both.
//
// performADSBUpdate() also reads adsbRingOuterKm directly, as the radius
// it requests from the adsb.fi API -- see the comment there for why that
// matters.
// =====================================================
extern float adsbRingOuterKm;
extern float adsbRingInnerKm;

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
