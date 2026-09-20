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

// Shows/hides the airspace-intrusion banner (drawn whenever the pilot is
// inside tracked airspace -- see OpenAirScanner.h/.cpp). Deliberately
// NOT persisted -- no loadSettings()/saveSettings() entry below -- so it
// always starts back on at boot regardless of how it was last left, and
// can never accidentally stay silenced into a flight where it matters.
extern bool airspaceAlertBarEnabled;

// Shows/hides the "Airspace Info" bar (ADS-B page only, bottom of
// screen -- see drawAirspaceInfoBar(), DrawPages.cpp). Unlike
// airspaceAlertBarEnabled above, this one IS persisted -- it's a
// pilot preference about a purely informational readout (current
// airspace name + recommended frequency, CFZ included), not a safety
// alert that should always default back on. See settings.h's
// PERSISTENCE section below.
extern bool airspaceInfoBarEnabled;

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

// Preferred weather data source for the Weather page -- WiFi-fetched
// Zephyr network stations (localMeters[], main .ino), or locally-received
// FANET Service (type 4) weather-station beacons (fanetWeatherStations[],
// DrawPages.h/.cpp). Regardless of which is preferred, drawWeatherPage()
// falls back to FANET stations if the preferred source currently has no
// data -- FANET needs no WiFi/internet and is often available when
// Zephyr isn't (mid-flight, out of WiFi range). Applied at draw time, not
// cached -- see Config > Weather Settings > Source in menu.cpp. Persisted.
enum WeatherSource { WEATHER_SOURCE_ZEPHYR = 0,
                      WEATHER_SOURCE_FANET };

extern WeatherSource weatherSource;

// Wind speed, gust, and estimated airspeed readouts (Weather page and the
// Paraglider page's WIND/AIRSPEED box) are converted with the same
// speedKphToDisplay()/speedUnitLabel() used for GPS ground speed above,
// so all speed readouts in the app always agree on units.

// =====================================================
// VARIO MUTE
// Toggled by a long-press of the page button (see updatePageButton() in
// the main .ino). Lives here (rather than the .ino) so it persists the
// same way as every other setting on this page -- see loadSettings()/
// saveSettings() below.
// =====================================================
extern bool buzzerMuted;

// =====================================================
// WIFI ENABLE
// Turns the ESP32's WiFi radio fully on or off -- see setWifiEnabled()
// (main .ino), applied at boot and immediately on change via
// Connections > WiFi in menu.cpp. Off stops the radio itself (and with
// it, the Zephyr weather fetch and ADS-B fetch, both of which need WiFi
// -- see weatherSource above for the FANET-based fallback that keeps the
// Weather page useful without it). Persisted.
// =====================================================
extern bool wifiEnabled;

// =====================================================
// TERRAIN DEM FILE SELECTION
// The *.ADEM tile getGroundElevationM() (TerrainDem.h) reads from. Chosen
// from the Map menu screen (menu.cpp's scanMapFiles()/setSelectedDemFile())
// -- always go through setSelectedDemFile() to change it, never assign it
// directly, since a scan running on the other core could otherwise read a
// half-written filename. DEM_FILENAME_MAX_LEN also bounds mapFileNames[]
// in menu.cpp and is shared (via this header) with TerrainDem.h.
// =====================================================
#define DEM_FILENAME_MAX_LEN 32
extern char selectedDemFile[DEM_FILENAME_MAX_LEN];

// =====================================================
// FANET RADIO ENABLE
// Turns the FANET radio (SX1262/HT-RA62, see Sx126xLink.h) fully on or
// off. OFF puts the chip itself into low-power sleep over SPI (no RX, no
// TX, no beacons) via Sx126xLink::setEnabled(), rather than just muting
// the software stack -- useful for RF-quiet bench testing. Applied at
// boot (setup(), main .ino) and immediately on change via
// setFanetEnabled() (main .ino) -- see Config > FANET in menu.cpp.
// Persisted, so it survives a reboot.
// =====================================================
extern bool fanetEnabled;

// =====================================================
// SCREEN ORIENTATION
// The case can be mounted either way up; this flips the display 180
// degrees to match, via u8g2.setDisplayRotation() (see
// applyScreenOrientation(), main .ino). "GPS Bottom" is the default and
// matches how the app has always shipped; "GPS Top" is for the case
// mounted the other way up. Applied at boot and immediately on change --
// see Config > Screen in menu.cpp. Persisted.
// =====================================================
enum ScreenOrientation { SCREEN_ORIENTATION_GPS_BOTTOM = 0,
                          SCREEN_ORIENTATION_GPS_TOP };

extern ScreenOrientation screenOrientation;

// =====================================================
// MAIN PAGE SELECTION
// Mirrors activePages[0] (menu.h/.ino) as a plain value rather than the
// Page enum itself, since Page is defined in menu.h and menu.h already
// includes this header -- storing the enum here would be circular.
// 0 = PAGE_PARAGLIDER, 1 = PAGE_PARAMOTOR. setup() (main .ino) and the
// Main Page Select handler (menu.cpp) are responsible for keeping
// activePages[0]/currentPage in sync with this value.
// =====================================================
extern uint8_t mainPageSelection;

// =====================================================
// PERSISTENCE (NVS, via the Preferences library, namespace "vario")
// loadSettings() reads every value on this page from flash, falling back
// to that value's compiled-in default (above) for any key that's never
// been written -- e.g. first boot after a fresh flash. Call once, early
// in the main .ino's setup(), before anything (buzzer volume, climb
// tone, etc.) reads these values.
//
// saveSettings() writes every value on this page to flash in one go.
// Call it after any single value changes (see the menu.cpp selection
// handlers and the mute toggle in the main .ino) -- simpler than a
// bespoke save function per setting, and NVS write endurance is a
// complete non-issue at menu-interaction frequency.
// =====================================================
void loadSettings();
void saveSettings();

#endif  // SETTINGS_H
