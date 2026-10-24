#include "settings.h"
#include <Preferences.h>

static Preferences prefs;

// Defaults match the values the display code used to have hard-coded:
// altitude in feet, ground speed in km/h.
AltitudeUnit altitudeUnit = ALT_UNIT_FEET;
SpeedUnit speedUnit = SPEED_UNIT_KMH;

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
uint8_t adsbAlarmMode = ADSB_ALARM_TONE;  // preserves the old default (alarm audible, not muted)
uint8_t adsbAlertVolumePercent = 100;

// Deliberately NOT persisted -- see the comment in settings.h. Always
// starts true at boot.
bool airspaceAlertBarEnabled = true;

// Persisted (unlike airspaceAlertBarEnabled above) -- default on to
// match the app's original always-on airspace display behaviour.
bool airspaceInfoBarEnabled = true;

// Deliberately NOT persisted -- see the comment in settings.h. Always
// starts true at boot.
bool flightRecorderEnabled = true;

// Persisted (unlike flightRecorderEnabled above) -- off by default, an
// opt-in feature.
bool igcAutoStopEnabled = false;

// Default matches the original hardcoded far/default rings (30km outer,
// 15km inner).
float adsbRingOuterKm = 30.0f;
float adsbRingInnerKm = 15.0f;

unsigned long weatherPollIntervalMs = 5UL * 60UL * 1000UL;
uint8_t weatherStationsShown = 4;

// Default matches the app's original (WiFi/Zephyr-only) behaviour.
WeatherSource weatherSource = WEATHER_SOURCE_ZEPHYR;

bool buzzerMuted = false;

// Default: WiFi on, matching the app's original always-on behaviour.
bool wifiEnabled = true;

// Default matches the renamed tile now shipped on the SD card.
char selectedDemFile[DEM_FILENAME_MAX_LEN] = "/Lower_North_Island.ADEM";

char voiceProfile[VOICE_PROFILE_NAME_MAX] = "";

// Default matches the app's original behaviour: boots on the Paraglider page.
uint8_t mainPageSelection = 0;  // 0 = Paraglider, 1 = Paramotor

// Default: radio on, matching the app's original always-on behaviour.
bool fanetEnabled = true;

// Default on -- matches fanetEnabled's default.
bool fanetMessagingEnabled = true;

// Default matches the app's original (and only, until now) orientation.
ScreenOrientation screenOrientation = SCREEN_ORIENTATION_GPS_BOTTOM;

// =====================================================
// PERSISTENCE
// =====================================================
// Values read back from flash are not trusted: a corrupted or out-of-range
// entry must never reach code that indexes arrays or divides by them (for
// example drawWeatherPage() divides by weatherStationsShown, and
// PAGE_NAMES[] / activePages[] are indexed by mainPageSelection). Any value
// outside what the menus can produce is replaced by its default. Comparisons
// are written so that NaN fails them and also gets the default.
static void sanitizeLoadedSettings() {
  auto reject = [](const char* name) {
    Serial.printf("[Settings] %s out of range in NVS; using default\n", name);
  };

  if ((uint8_t)altitudeUnit > (uint8_t)ALT_UNIT_FEET) {
    altitudeUnit = ALT_UNIT_FEET;
    reject("altUnit");
  }
  if ((uint8_t)speedUnit > (uint8_t)SPEED_UNIT_MPH) {
    speedUnit = SPEED_UNIT_KMH;
    reject("spdUnit");
  }
  if ((uint8_t)timeZoneMode > (uint8_t)TZ_MODE_MANUAL) {
    timeZoneMode = TZ_MODE_AUTO_NZ;
    reject("tzMode");
  }
  if (utcOffsetHours < -12 || utcOffsetHours > 14) {
    utcOffsetHours = 12;
    reject("utcOff");
  }
  if (buzzerVolumePercent < 10 || buzzerVolumePercent > 100) {
    buzzerVolumePercent = 80;
    reject("volPct");
  }
  if (!(adsbAlertRadiusKm >= 1.0f && adsbAlertRadiusKm <= 50.0f)) {
    adsbAlertRadiusKm = 5.0f;
    reject("adsbRadius");
  }
  if (!(adsbAlertVerticalFt >= 100.0f && adsbAlertVerticalFt <= 20000.0f)) {
    adsbAlertVerticalFt = 2000.0f;
    reject("adsbVert");
  }
  if (!(adsbRingOuterKm >= 5.0f && adsbRingOuterKm <= 200.0f)) {
    adsbRingOuterKm = 30.0f;
    reject("ringOuter");
  }
  adsbRingInnerKm = adsbRingOuterKm / 2.0f;
  if (adsbAlarmMode > ADSB_ALARM_VOICE) {
    adsbAlarmMode = ADSB_ALARM_TONE;
    reject("adsbAlarmMode");
  }
  if (adsbAlertVolumePercent > 100) {
    adsbAlertVolumePercent = 100;
    reject("adsbAlertVol");
  }
  if (weatherPollIntervalMs < 60UL * 1000UL ||
      weatherPollIntervalMs > 60UL * 60UL * 1000UL) {
    weatherPollIntervalMs = 5UL * 60UL * 1000UL;
    reject("wxPoll");
  }
  if (weatherStationsShown < 1 || weatherStationsShown > 6) {
    weatherStationsShown = 4;
    reject("wxStations");
  }
  if ((uint8_t)weatherSource > (uint8_t)WEATHER_SOURCE_FANET) {
    weatherSource = WEATHER_SOURCE_ZEPHYR;
    reject("wxSource");
  }
  if (mainPageSelection > 1) {
    mainPageSelection = 0;
    reject("mainPage");
  }
  if ((uint8_t)screenOrientation > (uint8_t)SCREEN_ORIENTATION_GPS_TOP) {
    screenOrientation = SCREEN_ORIENTATION_GPS_BOTTOM;
    reject("scrOrient");
  }
}

void loadSettings() {
  prefs.begin("vario", false);

  altitudeUnit = (AltitudeUnit)prefs.getUChar("altUnit", (uint8_t)altitudeUnit);
  speedUnit = (SpeedUnit)prefs.getUChar("spdUnit", (uint8_t)speedUnit);

  timeZoneMode = (TimeZoneMode)prefs.getUChar("tzMode", (uint8_t)timeZoneMode);
  utcOffsetHours = (int8_t)prefs.getChar("utcOff", utcOffsetHours);

  climbGapMinMs = prefs.getUInt("gapMin", climbGapMinMs);
  climbGapMaxMs = prefs.getUInt("gapMax", climbGapMaxMs);
  climbPulseMinMs = prefs.getUInt("pulseMin", climbPulseMinMs);
  climbPulseMaxMs = prefs.getUInt("pulseMax", climbPulseMaxMs);

  buzzerVolumePercent = prefs.getUChar("volPct", buzzerVolumePercent);

  adsbAlertRadiusKm = prefs.getFloat("adsbRadius", adsbAlertRadiusKm);
  adsbAlertVerticalFt = prefs.getFloat("adsbVert", adsbAlertVerticalFt);
  adsbAutoJumpEnabled = prefs.getBool("adsbJump", adsbAutoJumpEnabled);
  adsbAlarmMode = prefs.getUChar("adsbAlarmMode", adsbAlarmMode);
  adsbAlertVolumePercent = prefs.getUChar("adsbAlertVol", adsbAlertVolumePercent);

  // Inner ring is always half of outer (see menu.cpp) -- only the outer
  // value is persisted, inner is re-derived here the same way.
  adsbRingOuterKm = prefs.getFloat("ringOuter", adsbRingOuterKm);
  adsbRingInnerKm = adsbRingOuterKm / 2.0f;

  weatherPollIntervalMs = prefs.getUInt("wxPoll", weatherPollIntervalMs);
  weatherStationsShown = prefs.getUChar("wxStations", weatherStationsShown);
  weatherSource = (WeatherSource)prefs.getUChar("wxSource", (uint8_t)weatherSource);

  String dem = prefs.getString("demFile", selectedDemFile);
  strncpy(selectedDemFile, dem.c_str(), DEM_FILENAME_MAX_LEN - 1);
  selectedDemFile[DEM_FILENAME_MAX_LEN - 1] = '\0';

  char storedVoiceProfile[VOICE_PROFILE_NAME_MAX] = { 0 };
  size_t storedVoiceLength = prefs.isKey("voiceProf")
                               ? prefs.getStringLength("voiceProf") : 0;
  bool validVoiceProfile = storedVoiceLength == 0 ||
                           storedVoiceLength <= sizeof(storedVoiceProfile);
  if (validVoiceProfile && storedVoiceLength > 0) {
    size_t readLength = prefs.getString(
      "voiceProf", storedVoiceProfile, sizeof(storedVoiceProfile));
    validVoiceProfile = readLength == storedVoiceLength &&
                        storedVoiceProfile[storedVoiceLength - 1] == '\0';
  }
  size_t voiceProfileLength = storedVoiceLength > 0 ? storedVoiceLength - 1 : 0;
  for (size_t i = 0; validVoiceProfile && i < voiceProfileLength; i++) {
    char c = storedVoiceProfile[i];
    validVoiceProfile = (c >= 'A' && c <= 'Z') ||
                        (c >= 'a' && c <= 'z') ||
                        (c >= '0' && c <= '9') ||
                        c == '_' || c == '-';
  }
  if (validVoiceProfile) {
    strncpy(voiceProfile, storedVoiceProfile, VOICE_PROFILE_NAME_MAX - 1);
    voiceProfile[VOICE_PROFILE_NAME_MAX - 1] = '\0';
  } else {
    Serial.println("[Settings] Invalid voice profile in NVS; using Default");
    voiceProfile[0] = '\0';
  }

  mainPageSelection = prefs.getUChar("mainPage", mainPageSelection);

  buzzerMuted = prefs.getBool("muted", buzzerMuted);
  wifiEnabled = prefs.getBool("wifiOn", wifiEnabled);

  fanetEnabled = prefs.getBool("fanetOn", fanetEnabled);
  fanetMessagingEnabled = prefs.getBool("fanetMsgOn", fanetMessagingEnabled);
  screenOrientation = (ScreenOrientation)prefs.getUChar("scrOrient", (uint8_t)screenOrientation);

  airspaceInfoBarEnabled = prefs.getBool("aspInfoBar", airspaceInfoBarEnabled);
  igcAutoStopEnabled = prefs.getBool("igcAutoStop", igcAutoStopEnabled);

  // airspaceAlertBarEnabled and flightRecorderEnabled are intentionally
  // never read here -- see their comments in settings.h. Both keep
  // their compiled-in default (true) every boot.

  sanitizeLoadedSettings();
}

void saveSettings() {
  prefs.putUChar("altUnit", (uint8_t)altitudeUnit);
  prefs.putUChar("spdUnit", (uint8_t)speedUnit);

  prefs.putUChar("tzMode", (uint8_t)timeZoneMode);
  prefs.putChar("utcOff", utcOffsetHours);

  prefs.putUInt("gapMin", climbGapMinMs);
  prefs.putUInt("gapMax", climbGapMaxMs);
  prefs.putUInt("pulseMin", climbPulseMinMs);
  prefs.putUInt("pulseMax", climbPulseMaxMs);

  prefs.putUChar("volPct", buzzerVolumePercent);

  prefs.putFloat("adsbRadius", adsbAlertRadiusKm);
  prefs.putFloat("adsbVert", adsbAlertVerticalFt);
  prefs.putBool("adsbJump", adsbAutoJumpEnabled);
  prefs.putUChar("adsbAlarmMode", adsbAlarmMode);
  prefs.putUChar("adsbAlertVol", adsbAlertVolumePercent);

  prefs.putFloat("ringOuter", adsbRingOuterKm);

  prefs.putUInt("wxPoll", weatherPollIntervalMs);
  prefs.putUChar("wxStations", weatherStationsShown);
  prefs.putUChar("wxSource", (uint8_t)weatherSource);

  prefs.putString("demFile", selectedDemFile);
  if (prefs.putString("voiceProf", voiceProfile) == 0) {
    Serial.println("[Settings] Failed to save voice profile");
  }

  prefs.putUChar("mainPage", mainPageSelection);

  prefs.putBool("muted", buzzerMuted);
  prefs.putBool("wifiOn", wifiEnabled);

  prefs.putBool("fanetOn", fanetEnabled);
  prefs.putBool("fanetMsgOn", fanetMessagingEnabled);
  prefs.putUChar("scrOrient", (uint8_t)screenOrientation);

  prefs.putBool("aspInfoBar", airspaceInfoBarEnabled);
  prefs.putBool("igcAutoStop", igcAutoStopEnabled);

  // airspaceAlertBarEnabled and flightRecorderEnabled are intentionally
  // never written here -- see their comments in settings.h.
}
