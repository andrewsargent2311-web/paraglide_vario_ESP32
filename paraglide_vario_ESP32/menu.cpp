#include "menu.h"
#include "wifi_manager.h"
#include "WifiSetup.h"
#include "FileServer.h"
#include "FanetMessaging.h"
#include "ble_manager.h"
#include "VoiceAlert.h"
#include "secrets.h"
#include <FS.h>
#include <SD_MMC.h>
#include <string.h>
#include <strings.h>  // strcasecmp
#include <math.h>     // fabsf
#include <qrcode.h>   // QR encoder built into the ESP32 framework (esp_qrcode_*)

bool menuActive = false;
uint8_t menuSelectedIndex = 0;

// =====================================================
// MAP FILE LIST
// Populated by scanMapFiles() whenever the Map screen is opened.
// =====================================================
#define MAX_MAP_FILES 8
static char mapFileNames[MAX_MAP_FILES][DEM_FILENAME_MAX_LEN];
static uint8_t mapFileCount = 0;
static bool voiceProfileRequestPending = false;

// =====================================================
// SCREEN NAVIGATION STACK
// 4 deep now that Vario Beep adds a 4th level: MAIN > CONFIG >
// VARIO_BEEP > (Climb Volume / Sink Volume). Every other
// screen in the app still only goes 3 deep, so this just adds headroom
// -- it doesn't change how any existing screen behaves.
// =====================================================
#define MENU_MAX_DEPTH 4
static MenuScreen menuStack[MENU_MAX_DEPTH] = { MENU_SCREEN_MAIN };
static uint8_t menuStackDepth = 1;

static MenuScreen currentMenuScreen() {
  return menuStack[menuStackDepth - 1];
}

static void scanMapFiles();  // forward declaration, defined below

static void pushMenuScreen(MenuScreen screen) {
  if (menuStackDepth < MENU_MAX_DEPTH) {
    menuStack[menuStackDepth++] = screen;
  }
  menuSelectedIndex = 0;
  displayDirty = true;
  if (screen == MENU_SCREEN_MAP) {
    scanMapFiles();
  }
  if (screen == MENU_SCREEN_BLUETOOTH_SCAN) {
    bleStartScan();
  }
}

void menuGoBack() {
  // Leaving the scan screen -- no point burning radio time on a scan
  // nobody's looking at anymore.
  if (currentMenuScreen() == MENU_SCREEN_BLUETOOTH_SCAN) {
    bleStopScan();
  }
  // Leaving Add Network closes the setup hotspot (no-op once it has
  // already closed itself after a save).
  if (currentMenuScreen() == MENU_SCREEN_WIFI_SETUP) {
    stopWifiSetup();
  }

  if (menuStackDepth > 1) {
    menuStackDepth--;
    menuSelectedIndex = 0;
    displayDirty = true;
    playFeedbackTone(400.0f, 60);
  } else {
    closeMenu();
  }
}

// =====================================================
// PAGE / MENU HELPERS
// =====================================================

void openMenu() {
  menuActive = true;
  menuStack[0] = MENU_SCREEN_MAIN;
  menuStackDepth = 1;
  menuSelectedIndex = 0;
  displayDirty = true;
  playFeedbackTone(700.0f, 90);
}

void closeMenu() {
  menuActive = false;
  displayDirty = true;
}

// =====================================================
// MAP FILE SCANNING
// Looks for *.ADEM tiles in the SD card root. Shares sdMutex with the
// background DEM/airspace scans (see the main .ino) so a directory listing
// here can never land mid-read/write of the same SPI bus from Core 0.
// =====================================================
static bool hasExtension(const char* name, const char* ext) {
  size_t nameLen = strlen(name);
  size_t extLen = strlen(ext);
  if (nameLen < extLen) return false;
  return strcasecmp(name + (nameLen - extLen), ext) == 0;
}

static void scanMapFiles() {
  mapFileCount = 0;
  if (!sdCardOK || sdMutex == nullptr) return;
  if (xSemaphoreTake(sdMutex, pdMS_TO_TICKS(500)) != pdTRUE) {
    Serial.println("[MENU] SD busy -- map scan skipped");
    return;
  }

  File root = SD_MMC.open("/");
  if (root && root.isDirectory()) {
    File entry = root.openNextFile();
    while (entry && mapFileCount < MAX_MAP_FILES) {
      if (!entry.isDirectory()) {
        const char* name = entry.name();
        if (hasExtension(name, ".ADEM")) {
          // Always store with a leading '/' so the name can be passed
          // straight into getGroundElevationM()/SD_MMC.open() later.
          if (name[0] == '/') {
            strncpy(mapFileNames[mapFileCount], name, DEM_FILENAME_MAX_LEN - 1);
          } else {
            snprintf(mapFileNames[mapFileCount], DEM_FILENAME_MAX_LEN, "/%s", name);
          }
          mapFileNames[mapFileCount][DEM_FILENAME_MAX_LEN - 1] = '\0';
          mapFileCount++;
        }
      }
      entry = root.openNextFile();
    }
  }
  xSemaphoreGive(sdMutex);
}

void setSelectedDemFile(const char* filename) {
  if (sdMutex != nullptr && xSemaphoreTake(sdMutex, pdMS_TO_TICKS(500)) == pdTRUE) {
    strncpy(selectedDemFile, filename, DEM_FILENAME_MAX_LEN - 1);
    selectedDemFile[DEM_FILENAME_MAX_LEN - 1] = '\0';
    xSemaphoreGive(sdMutex);
  }
  saveSettings();
}

// Strips the path and extension for a cleaner on-screen label, e.g.
// "/DEM.ADEM" -> "DEM".
static void extractMapLabel(const char* filename, char* out, size_t outLen) {
  const char* base = filename;
  const char* lastSlash = strrchr(filename, '/');
  if (lastSlash) base = lastSlash + 1;
  strncpy(out, base, outLen - 1);
  out[outLen - 1] = '\0';
  char* dot = strrchr(out, '.');
  if (dot) *dot = '\0';
}

// =====================================================
// TIME / TIMEZONE CHOICES
// Index 0 is always "NZ (Auto)" (DST-aware, TZ_MODE_AUTO_NZ). Indices
// 1..TIMEZONE_CHOICE_COUNT-1 are fixed manual whole-hour UTC offsets,
// -12 through +14 inclusive -- the full real-world range -- selecting
// TZ_MODE_MANUAL. See the TimeZoneMode comment in settings.h for why a
// manual offset doesn't self-adjust for DST the way "Auto" does.
// =====================================================
#define TIMEZONE_MANUAL_MIN_HOURS (-12)
#define TIMEZONE_MANUAL_MAX_HOURS (14)
#define TIMEZONE_MANUAL_CHOICE_COUNT (TIMEZONE_MANUAL_MAX_HOURS - TIMEZONE_MANUAL_MIN_HOURS + 1)
#define TIMEZONE_CHOICE_COUNT (TIMEZONE_MANUAL_CHOICE_COUNT + 1)

// =====================================================
// VOLUME CHOICES
// =====================================================
// 10-100% in 10% steps, dB-scaled -- see the BUZZER VOLUME comment in
// settings.h for the percent -> dB mapping (shown next to each entry).
static const uint8_t VOLUME_CHOICES_PERCENT[] = { 10, 20, 30, 40, 50, 60, 70, 80, 90, 100 };
static const uint8_t VOLUME_CHOICE_COUNT = 10;

// =====================================================
// VARIO CLIMB / SINK VOLUME CHOICES
// 0-100% in 10% steps for both climbVolumePercent and sinkVolumePercent
// (settings.h) -- choice[index] = index * 10. The percentage is converted
// to a dB-scaled amplitude in the main .ino (varioVolumeToGain()).
// =====================================================
#define VARIO_VOLUME_CHOICE_COUNT 11
#define VARIO_VOLUME_CHOICE_PERCENT(index) ((uint8_t)((index) * 10))

// Test tones played back at the chosen level when a volume is picked:
// ~1200Hz is a typical climb beep, ~250Hz is a typical sink tone.
#define VARIO_VOLUME_PREVIEW_CLIMB_HZ 1200.0f
#define VARIO_VOLUME_PREVIEW_CLIMB_MS 350UL
#define VARIO_VOLUME_PREVIEW_SINK_HZ 250.0f
#define VARIO_VOLUME_PREVIEW_SINK_MS 700UL

// =====================================================
// ADS-B ALERT CHOICES
// =====================================================
static const float ADSB_RADIUS_CHOICES_KM[] = { 3.0f, 5.0f, 8.0f, 10.0f };
static const uint8_t ADSB_RADIUS_CHOICE_COUNT = 4;

static const float ADSB_VERTICAL_CHOICES_FT[] = { 1000.0f, 1500.0f, 2000.0f, 3000.0f };
static const uint8_t ADSB_VERTICAL_CHOICE_COUNT = 4;

// Alert Volume: 0-100% in 10% steps (11 levels, same as VARIO_VOLUME_CHOICE_COUNT),
// plus a "Test" item at the end that previews the current Alarm Sound
// mode (siren or voice) at this volume.
#define ADSB_ALERT_VOLUME_CHOICE_COUNT 12
#define ADSB_ALERT_VOLUME_TEST_INDEX 11

// Index 0 ("Default") reproduces the app's original hardcoded far/default
// rings (30km outer / 15km inner). Inner is always half of outer -- see
// the adsbRingOuterKm/adsbRingInnerKm comment in settings.h for what this
// does (and doesn't) affect.
static const float ADSB_RING_OUTER_CHOICES_KM[] = { 30.0f, 20.0f, 40.0f, 60.0f };
static const uint8_t ADSB_RING_CHOICE_COUNT = 4;

// =====================================================
// WEATHER CHOICES
// =====================================================
static const unsigned long WEATHER_POLL_CHOICES_MS[] = { 2UL * 60UL * 1000UL, 5UL * 60UL * 1000UL, 10UL * 60UL * 1000UL, 15UL * 60UL * 1000UL };
static const char* WEATHER_POLL_LABELS[] = { "2 min", "5 min", "10 min", "15 min" };
static const uint8_t WEATHER_POLL_CHOICE_COUNT = 4;

// Must never exceed TRACKED_METERS (main .ino) -- that's the max number of
// stations actually collected and sorted by distance.
static const uint8_t WEATHER_STATIONS_CHOICES[] = { 2, 4, 6 };
static const uint8_t WEATHER_STATIONS_CHOICE_COUNT = 3;

// =====================================================
// SCREEN CONTENT: item counts, titles, and labels
// =====================================================
// =====================================================
// WIFI LIST HELPERS
// The WiFi screen lists only the networks that actually exist (secrets.h
// entries plus any added from the device -- see wifi_manager.h), then
// "Add Network", then "Forget Network" once there's something to forget:
//   row 0                 WiFi: On/Off
//   rows 1..N             networks (N = wifiListedCount())
//   row N+1               Add Network
//   row N+2               Forget Network   (only if any are device-saved)
// =====================================================
static uint8_t wifiListedCount() {
  uint8_t n = 0;
  for (uint8_t s = 0; s < wifiTotalSlots(); s++) {
    if (wifiSlotValid(s)) n++;
  }
  return n;
}

static int wifiListedSlot(uint8_t nth) {  // nth listed network -> slot
  for (uint8_t s = 0; s < wifiTotalSlots(); s++) {
    if (!wifiSlotValid(s)) continue;
    if (nth == 0) return s;
    nth--;
  }
  return -1;
}

static uint8_t wifiDeviceSavedCount() {
  uint8_t n = 0;
  for (uint8_t s = 0; s < wifiTotalSlots(); s++) {
    if (wifiSlotIsDeviceSaved(s) && wifiSlotValid(s)) n++;
  }
  return n;
}

static int wifiDeviceSavedSlot(uint8_t nth) {  // nth forgettable network -> slot
  for (uint8_t s = 0; s < wifiTotalSlots(); s++) {
    if (!wifiSlotIsDeviceSaved(s) || !wifiSlotValid(s)) continue;
    if (nth == 0) return s;
    nth--;
  }
  return -1;
}

static uint8_t getMenuItemCount(MenuScreen screen) {
  switch (screen) {
    case MENU_SCREEN_MAIN: return 8;
    case MENU_SCREEN_MAIN_PAGE_SELECT: return 2;
    case MENU_SCREEN_CONFIG: return 5;
    case MENU_SCREEN_CONFIG_TIME: return TIMEZONE_CHOICE_COUNT;
    case MENU_SCREEN_UNITS: return 2;
    case MENU_SCREEN_CONFIG_VOLUME: return VOLUME_CHOICE_COUNT;
    case MENU_SCREEN_VARIO_BEEP: return 2;
    case MENU_SCREEN_VARIO_BEEP_CLIMB_VOL: return VARIO_VOLUME_CHOICE_COUNT;
    case MENU_SCREEN_VARIO_BEEP_SINK_VOL: return VARIO_VOLUME_CHOICE_COUNT;
    case MENU_SCREEN_SCREEN: return 2;
    case MENU_SCREEN_CONNECTIONS: return 3;
    case MENU_SCREEN_FANET_MESSAGING: return 1 + FANET_MESSAGE_PRESET_COUNT;
    case MENU_SCREEN_WIFI_LIST:
      return 1 + wifiListedCount() + 1 + (wifiDeviceSavedCount() > 0 ? 1 : 0);
    case MENU_SCREEN_WIFI_SETUP: return 1;  // Cancel/Done; the rest is drawWifiSetupScreen()
    case MENU_SCREEN_WIFI_FORGET: return wifiDeviceSavedCount() > 0 ? wifiDeviceSavedCount() : 1;
    case MENU_SCREEN_BLUETOOTH: return 3;
    case MENU_SCREEN_BLUETOOTH_SCAN: return bleScanResultCount() > 0 ? bleScanResultCount() : 1;
    case MENU_SCREEN_MAP: return mapFileCount > 0 ? mapFileCount : 1;
    case MENU_SCREEN_ADSB_SETTINGS: return 9;
    case MENU_SCREEN_ADSB_RADIUS: return ADSB_RADIUS_CHOICE_COUNT;
    case MENU_SCREEN_ADSB_VERTICAL: return ADSB_VERTICAL_CHOICE_COUNT;
    case MENU_SCREEN_ADSB_ALERT_VOLUME: return ADSB_ALERT_VOLUME_CHOICE_COUNT;
    case MENU_SCREEN_ADSB_RANGE_RINGS: return ADSB_RING_CHOICE_COUNT;
    case MENU_SCREEN_WEATHER_SETTINGS: return 3;
    case MENU_SCREEN_WEATHER_POLL_INTERVAL: return WEATHER_POLL_CHOICE_COUNT;
    case MENU_SCREEN_WEATHER_STATIONS_SHOWN: return WEATHER_STATIONS_CHOICE_COUNT;
    case MENU_SCREEN_WEATHER_SOURCE: return 2;
    case MENU_SCREEN_FLIGHT_RECORDINGS: return 3;
    case MENU_SCREEN_EXPORT_FILES: return isFileServerRunning() ? 2 : 1;
    default: return 1;
  }
}

static const char* getMenuTitle(MenuScreen screen) {
  switch (screen) {
    case MENU_SCREEN_MAIN: return "MENU";
    case MENU_SCREEN_MAIN_PAGE_SELECT: return "MAIN PAGE";
    case MENU_SCREEN_CONFIG: return "CONFIG";
    case MENU_SCREEN_CONFIG_TIME: return "TIME";
    case MENU_SCREEN_UNITS: return "UNITS";
    case MENU_SCREEN_CONFIG_VOLUME: return "VOLUME";
    case MENU_SCREEN_VARIO_BEEP: return "VARIO BEEP";
    case MENU_SCREEN_VARIO_BEEP_CLIMB_VOL: return "CLIMB VOLUME";
    case MENU_SCREEN_VARIO_BEEP_SINK_VOL: return "SINK VOLUME";
    case MENU_SCREEN_SCREEN: return "SCREEN";
    case MENU_SCREEN_CONNECTIONS: return "CONNECTIONS";
    case MENU_SCREEN_FANET_MESSAGING: return "FANET MESSAGING";
    case MENU_SCREEN_WIFI_LIST: return "WIFI";
    case MENU_SCREEN_WIFI_SETUP: return "ADD NETWORK";
    case MENU_SCREEN_WIFI_FORGET: return "FORGET NETWORK";
    case MENU_SCREEN_BLUETOOTH: return "BLUETOOTH";
    case MENU_SCREEN_BLUETOOTH_SCAN: return "SCAN DEVICES";
    case MENU_SCREEN_MAP: return "MAP";
    case MENU_SCREEN_ADSB_SETTINGS: return "ADSB SETTINGS";
    case MENU_SCREEN_ADSB_RADIUS: return "ALERT RADIUS";
    case MENU_SCREEN_ADSB_VERTICAL: return "VERT THRESHOLD";
    case MENU_SCREEN_ADSB_ALERT_VOLUME: return "ALERT VOLUME";
    case MENU_SCREEN_ADSB_RANGE_RINGS: return "RANGE RINGS";
    case MENU_SCREEN_WEATHER_SETTINGS: return "WEATHER SETTINGS";
    case MENU_SCREEN_WEATHER_POLL_INTERVAL: return "POLL INTERVAL";
    case MENU_SCREEN_WEATHER_STATIONS_SHOWN: return "STATIONS SHOWN";
    case MENU_SCREEN_WEATHER_SOURCE: return "SOURCE";
    case MENU_SCREEN_FLIGHT_RECORDINGS: return "FLIGHT RECORDINGS";
    case MENU_SCREEN_EXPORT_FILES: return "EXPORT FILES";
    default: return "MENU";
  }
}

static void getMenuItemLabel(MenuScreen screen, uint8_t index, char* buf, size_t buflen) {
  switch (screen) {
    case MENU_SCREEN_MAIN: {
      static const char* items[] = { "Main Page", "Config", "Connections", "FANET Messaging", "Map", "ADSB Settings", "Weather Settings", "Flight Recordings" };
      snprintf(buf, buflen, "%s", items[index]);
      break;
    }
    case MENU_SCREEN_MAIN_PAGE_SELECT: {
      const char* name = (index == 0) ? "Paraglider" : "Paramotor";
      bool isActive = (index == 0 && activePages[0] == PAGE_PARAGLIDER) || (index == 1 && activePages[0] == PAGE_PARAMOTOR);
      snprintf(buf, buflen, "%s%s", name, isActive ? " *" : "");
      break;
    }
    case MENU_SCREEN_CONFIG:
      switch (index) {
        case 0: snprintf(buf, buflen, "Time"); break;
        case 1: snprintf(buf, buflen, "Units"); break;
        case 2: snprintf(buf, buflen, "Volume"); break;
        case 3: snprintf(buf, buflen, "Vario Beep"); break;
        case 4: snprintf(buf, buflen, "Screen"); break;
      }
      break;
    case MENU_SCREEN_CONFIG_TIME:
      if (index == 0) {
        bool isActive = (timeZoneMode == TZ_MODE_AUTO_NZ);
        snprintf(buf, buflen, "NZ (Auto)%s", isActive ? " *" : "");
      } else {
        int offset = TIMEZONE_MANUAL_MIN_HOURS + (index - 1);
        bool isActive = (timeZoneMode == TZ_MODE_MANUAL && utcOffsetHours == offset);
        snprintf(buf, buflen, "UTC%+d%s", offset, isActive ? " *" : "");
      }
      break;
    case MENU_SCREEN_UNITS:
      if (index == 0) {
        snprintf(buf, buflen, "Altitude: %s", altitudeUnitLabel());
      } else {
        snprintf(buf, buflen, "Speed: %s", speedUnitLabel());
      }
      break;
    case MENU_SCREEN_CONFIG_VOLUME: {
      uint8_t pct = VOLUME_CHOICES_PERCENT[index];
      bool isActive = (buzzerVolumePercent == pct);
      int db = (int)lroundf(buzzerVolumePercentToDb(pct));
      if (db == 0) {
        snprintf(buf, buflen, "%u%%  0 dB%s", pct, isActive ? " *" : "");
      } else {
        snprintf(buf, buflen, "%u%%  %+d dB%s", pct, db, isActive ? " *" : "");
      }
      break;
    }
    case MENU_SCREEN_VARIO_BEEP: {
      static const char* items[] = { "Climb Volume", "Sink Volume" };
      snprintf(buf, buflen, "%s", items[index]);
      break;
    }
    case MENU_SCREEN_VARIO_BEEP_CLIMB_VOL: {
      uint8_t pct = VARIO_VOLUME_CHOICE_PERCENT(index);
      bool isActive = (climbVolumePercent == pct);
      snprintf(buf, buflen, "%u%%%s", pct, isActive ? " *" : "");
      break;
    }
    case MENU_SCREEN_VARIO_BEEP_SINK_VOL: {
      uint8_t pct = VARIO_VOLUME_CHOICE_PERCENT(index);
      bool isActive = (sinkVolumePercent == pct);
      snprintf(buf, buflen, "%u%%%s", pct, isActive ? " *" : "");
      break;
    }
    case MENU_SCREEN_SCREEN: {
      const char* name = (index == 0) ? "GPS Top" : "GPS Bottom";
      bool isActive = (index == 0 && screenOrientation == SCREEN_ORIENTATION_GPS_TOP) ||
                       (index == 1 && screenOrientation == SCREEN_ORIENTATION_GPS_BOTTOM);
      snprintf(buf, buflen, "%s%s", name, isActive ? " *" : "");
      break;
    }
    case MENU_SCREEN_CONNECTIONS:
      switch (index) {
        case 0: snprintf(buf, buflen, "WiFi"); break;
        case 1: snprintf(buf, buflen, "Bluetooth"); break;
        case 2: snprintf(buf, buflen, "FANET: %s", fanetEnabled ? "On" : "Off"); break;
      }
      break;
    case MENU_SCREEN_FANET_MESSAGING: {
      if (index == 0) {
        snprintf(buf, buflen, "Messaging: %s", fanetMessagingEnabled ? "On" : "Off");
      } else {
        // Bare preset text, no "Send: " prefix -- the longest preset
        // ("Sink here, be careful") plus a prefix risks running past
        // the visible 300px screen width at this font size; the screen
        // title ("FANET MESSAGING") and position below the toggle
        // already make these read as sendable messages without it.
        snprintf(buf, buflen, "%s", FANET_MESSAGE_PRESETS[index - 1]);
      }
      break;
    }
    case MENU_SCREEN_WIFI_LIST: {
      if (index == 0) {
        snprintf(buf, buflen, "WiFi: %s", wifiEnabled ? "On" : "Off");
        break;
      }
      uint8_t listed = wifiListedCount();
      if (index == listed + 1) {
        snprintf(buf, buflen, "+ Add Network");
        break;
      }
      if (index == listed + 2) {
        snprintf(buf, buflen, "- Forget Network");
        break;
      }
      int slot = wifiListedSlot(index - 1);
      const char* ssid = (slot >= 0) ? wifiSlotSsid((uint8_t)slot) : "";
      if (slot >= 0 && slot == selectedWifiIndex && wifiEnabled) {
        snprintf(buf, buflen, "%s%s", ssid, wifiConnected ? " * connected" : " * connecting");
      } else {
        snprintf(buf, buflen, "%s", ssid);
      }
      break;
    }
    case MENU_SCREEN_WIFI_SETUP:
      snprintf(buf, buflen, "%s", isWifiSetupActive() ? "Cancel" : "Done");
      break;
    case MENU_SCREEN_WIFI_FORGET: {
      int slot = wifiDeviceSavedSlot(index);
      if (slot < 0) {
        snprintf(buf, buflen, "(none saved)");
      } else {
        snprintf(buf, buflen, "%s", wifiSlotSsid((uint8_t)slot));
      }
      break;
    }
    case MENU_SCREEN_BLUETOOTH:
      switch (index) {
        case 0:
          snprintf(buf, buflen, "Bluetooth: %s", bleEnabled ? "On" : "Off");
          break;
        case 1:
          snprintf(buf, buflen, "Scan for Devices");
          break;
        case 2:
          if (bleRememberedAddress[0] != '\0') {
            const char* label = (bleRememberedName[0] != '\0') ? bleRememberedName : bleRememberedAddress;
            snprintf(buf, buflen, "Forget: %s%s", label, bleConnected ? " (connected)" : "");
          } else {
            snprintf(buf, buflen, "No Device Saved");
          }
          break;
      }
      break;
    case MENU_SCREEN_BLUETOOTH_SCAN:
      if (!bleEnabled) {
        snprintf(buf, buflen, "Bluetooth is Off");
      } else if (bleScanResultCount() == 0) {
        snprintf(buf, buflen, "Scanning...");
      } else {
        BleScanResult r = bleScanResultAt(index);
        const char* label = (r.name[0] != '\0') ? r.name : r.address;
        bool isConnectedRow = bleConnected && bleRememberedAddress[0] != '\0'
                               && strcasecmp(r.address, bleRememberedAddress) == 0;
        char shortLabel[20];
        strncpy(shortLabel, label, sizeof(shortLabel) - 1);
        shortLabel[sizeof(shortLabel) - 1] = '\0';
        snprintf(buf, buflen, "%s %ddBm%s", shortLabel, r.rssi, isConnectedRow ? " *" : "");
      }
      break;
    case MENU_SCREEN_MAP:
      if (mapFileCount == 0) {
        snprintf(buf, buflen, "No Maps Found");
      } else {
        char label[24];
        extractMapLabel(mapFileNames[index], label, sizeof(label));
        bool isActive = (strcmp(mapFileNames[index], selectedDemFile) == 0);
        snprintf(buf, buflen, "%s%s", label, isActive ? " *" : "");
      }
      break;
    case MENU_SCREEN_ADSB_SETTINGS: {
      if (index == 2) {
        snprintf(buf, buflen, "Auto-Jump: %s", adsbAutoJumpEnabled ? "On" : "Off");
      } else if (index == 3) {
        const char* modeStr = (adsbAlarmMode == ADSB_ALARM_OFF) ? "Off"
                             : (adsbAlarmMode == ADSB_ALARM_TONE) ? "Alarm" : "Voice";
        snprintf(buf, buflen, "Alarm Sound: %s", modeStr);
      } else if (index == 4) {
        snprintf(buf, buflen, "Alert Volume");
      } else if (index == 5) {
        snprintf(buf, buflen, "Range Rings");
      } else if (index == 6) {
        snprintf(buf, buflen, "Airspace Alert Bar: %s", airspaceAlertBarEnabled ? "On" : "Off");
      } else if (index == 7) {
        snprintf(buf, buflen, "Airspace Info: %s", airspaceInfoBarEnabled ? "On" : "Off");
      } else if (index == 8) {
        if (voiceProfileLoading()) {
          snprintf(buf, buflen, "Voice: Loading...");
        } else {
          const char* profile = voiceClipsLoaded() ? voiceActiveProfile() : voiceProfile;
          snprintf(buf, buflen, "Voice: %s",
                   profile[0] == '\0' ? "Default" : profile);
        }
      } else {
        static const char* items[] = { "Alert Radius", "Vertical Threshold" };
        snprintf(buf, buflen, "%s", items[index]);
      }
      break;
    }
    case MENU_SCREEN_ADSB_ALERT_VOLUME: {
      if (index == ADSB_ALERT_VOLUME_TEST_INDEX) {
        snprintf(buf, buflen, "Test");
      } else {
        uint8_t pct = VARIO_VOLUME_CHOICE_PERCENT(index);
        bool isActive = (adsbAlertVolumePercent == pct);
        snprintf(buf, buflen, "%u%%%s", pct, isActive ? " *" : "");
      }
      break;
    }
    case MENU_SCREEN_ADSB_RADIUS: {
      float km = ADSB_RADIUS_CHOICES_KM[index];
      bool isActive = fabsf(adsbAlertRadiusKm - km) < 0.01f;
      snprintf(buf, buflen, "%.0f km%s", km, isActive ? " *" : "");
      break;
    }
    case MENU_SCREEN_ADSB_VERTICAL: {
      float ft = ADSB_VERTICAL_CHOICES_FT[index];
      bool isActive = fabsf(adsbAlertVerticalFt - ft) < 0.5f;
      snprintf(buf, buflen, "%.0f ft%s", ft, isActive ? " *" : "");
      break;
    }
    case MENU_SCREEN_ADSB_RANGE_RINGS: {
      float outer = ADSB_RING_OUTER_CHOICES_KM[index];
      float inner = outer / 2.0f;
      bool isActive = fabsf(adsbRingOuterKm - outer) < 0.01f;
      if (index == 0) {
        snprintf(buf, buflen, "Default (%.0f/%.0f)%s", inner, outer, isActive ? " *" : "");
      } else {
        snprintf(buf, buflen, "%.0f/%.0f km%s", inner, outer, isActive ? " *" : "");
      }
      break;
    }
    case MENU_SCREEN_WEATHER_SETTINGS: {
      static const char* items[] = { "Poll Interval", "Stations Shown", "Source" };
      snprintf(buf, buflen, "%s", items[index]);
      break;
    }
    case MENU_SCREEN_WEATHER_POLL_INTERVAL: {
      bool isActive = (weatherPollIntervalMs == WEATHER_POLL_CHOICES_MS[index]);
      snprintf(buf, buflen, "%s%s", WEATHER_POLL_LABELS[index], isActive ? " *" : "");
      break;
    }
    case MENU_SCREEN_WEATHER_STATIONS_SHOWN: {
      uint8_t n = WEATHER_STATIONS_CHOICES[index];
      bool isActive = (weatherStationsShown == n);
      snprintf(buf, buflen, "%d Stations%s", n, isActive ? " *" : "");
      break;
    }
    case MENU_SCREEN_WEATHER_SOURCE: {
      const char* name = (index == 0) ? "FANET" : "Zephyr";
      bool isActive = (index == 0 && weatherSource == WEATHER_SOURCE_FANET) ||
                       (index == 1 && weatherSource == WEATHER_SOURCE_ZEPHYR);
      snprintf(buf, buflen, "%s%s", name, isActive ? " *" : "");
      break;
    }
    case MENU_SCREEN_FLIGHT_RECORDINGS:
      switch (index) {
        case 0: snprintf(buf, buflen, "Recording: %s", flightRecorderEnabled ? "On" : "Off"); break;
        case 1: snprintf(buf, buflen, "Export Files"); break;
        case 2: snprintf(buf, buflen, "Auto-Stop: %s", igcAutoStopEnabled ? "On" : "Off"); break;
      }
      break;
    case MENU_SCREEN_EXPORT_FILES: {
      if (index == 0) {
        snprintf(buf, buflen, "%s", isFileServerRunning() ? "Stop Server" : "Start Server");
      } else {
        // index == 1 -- only present while running (see the dynamic
        // count above); informational, not actionable.
        String url = getFileServerURL();
        snprintf(buf, buflen, "%s", url.c_str());
      }
      break;
    }
    default:
      snprintf(buf, buflen, "?");
      break;
  }
}

// =====================================================
// SCREEN ACTIONS: what a hold-select does on each screen
// =====================================================
static void selectMenuItem(MenuScreen screen, uint8_t index) {
  switch (screen) {
    case MENU_SCREEN_MAIN:
      switch (index) {
        case 0: pushMenuScreen(MENU_SCREEN_MAIN_PAGE_SELECT); break;
        case 1: pushMenuScreen(MENU_SCREEN_CONFIG); break;
        case 2: pushMenuScreen(MENU_SCREEN_CONNECTIONS); break;
        case 3: pushMenuScreen(MENU_SCREEN_FANET_MESSAGING); break;
        case 4: pushMenuScreen(MENU_SCREEN_MAP); break;
        case 5: pushMenuScreen(MENU_SCREEN_ADSB_SETTINGS); break;
        case 6: pushMenuScreen(MENU_SCREEN_WEATHER_SETTINGS); break;
        case 7: pushMenuScreen(MENU_SCREEN_FLIGHT_RECORDINGS); break;
      }
      playFeedbackTone(900.0f, 80);
      return;

    case MENU_SCREEN_MAIN_PAGE_SELECT:
      activePages[0] = (index == 0) ? PAGE_PARAGLIDER : PAGE_PARAMOTOR;
      activePageIndex = 0;
      currentPage = activePages[0];
      mainPageSelection = (index == 0) ? 0 : 1;
      saveSettings();
      playFeedbackTone(1100.0f, 120);
      closeMenu();  // matches the original single-level behaviour: pick a
                    // main page and go straight back to flying
      return;

    case MENU_SCREEN_CONFIG:
      switch (index) {
        case 0: pushMenuScreen(MENU_SCREEN_CONFIG_TIME); break;
        case 1: pushMenuScreen(MENU_SCREEN_UNITS); break;
        case 2:
          pushMenuScreen(MENU_SCREEN_CONFIG_VOLUME);
          menuSelectedIndex = (buzzerVolumePercent >= 10) ? (buzzerVolumePercent / 10 - 1) : 0;  // start on the current level
          if (menuSelectedIndex >= VOLUME_CHOICE_COUNT) menuSelectedIndex = VOLUME_CHOICE_COUNT - 1;
          break;
        case 3: pushMenuScreen(MENU_SCREEN_VARIO_BEEP); break;
        case 4: pushMenuScreen(MENU_SCREEN_SCREEN); break;
      }
      playFeedbackTone(900.0f, 80);
      return;

    case MENU_SCREEN_CONFIG_TIME:
      if (index == 0) {
        timeZoneMode = TZ_MODE_AUTO_NZ;
      } else {
        timeZoneMode = TZ_MODE_MANUAL;
        utcOffsetHours = (int8_t)(TIMEZONE_MANUAL_MIN_HOURS + (index - 1));
      }
      saveSettings();
      displayDirty = true;
      playFeedbackTone(1100.0f, 120);
      menuGoBack();  // back to Config
      return;

    case MENU_SCREEN_UNITS:
      // Cycles the value in place and stays on this screen, so both
      // Altitude and Speed can be tweaked in one visit.
      if (index == 0) {
        altitudeUnit = (altitudeUnit == ALT_UNIT_METERS) ? ALT_UNIT_FEET : ALT_UNIT_METERS;
      } else {
        speedUnit = (SpeedUnit)((speedUnit + 1) % 3);
      }
      saveSettings();
      displayDirty = true;
      playFeedbackTone(600.0f, 50);
      return;

    case MENU_SCREEN_CONFIG_VOLUME:
      buzzerVolumePercent = VOLUME_CHOICES_PERCENT[index];
      applyBuzzerVolume();  // takes effect immediately, not just on next boot
      saveSettings();
      playFeedbackTone(1100.0f, 120);
      menuGoBack();  // back to Config
      return;

    case MENU_SCREEN_VARIO_BEEP:
      switch (index) {
        case 0:
          pushMenuScreen(MENU_SCREEN_VARIO_BEEP_CLIMB_VOL);
          menuSelectedIndex = climbVolumePercent / 10;  // start on the current level
          break;
        case 1:
          pushMenuScreen(MENU_SCREEN_VARIO_BEEP_SINK_VOL);
          menuSelectedIndex = sinkVolumePercent / 10;
          break;
      }
      playFeedbackTone(900.0f, 80);
      return;

    // The two volume screens apply the level immediately, play a short test
    // tone at that level, and STAY on the screen -- so several levels can be
    // tried back to back. Double-press to go back to Vario Beep.
    case MENU_SCREEN_VARIO_BEEP_CLIMB_VOL:
      climbVolumePercent = VARIO_VOLUME_CHOICE_PERCENT(index);
      saveVarioVolumes();
      displayDirty = true;
      playVolumePreviewTone(VARIO_VOLUME_PREVIEW_CLIMB_HZ, VARIO_VOLUME_PREVIEW_CLIMB_MS, climbVolumePercent);
      return;

    case MENU_SCREEN_VARIO_BEEP_SINK_VOL:
      sinkVolumePercent = VARIO_VOLUME_CHOICE_PERCENT(index);
      saveVarioVolumes();
      displayDirty = true;
      playVolumePreviewTone(VARIO_VOLUME_PREVIEW_SINK_HZ, VARIO_VOLUME_PREVIEW_SINK_MS, sinkVolumePercent);
      return;

    case MENU_SCREEN_SCREEN:
      screenOrientation = (index == 0) ? SCREEN_ORIENTATION_GPS_TOP : SCREEN_ORIENTATION_GPS_BOTTOM;
      applyScreenOrientation();  // takes effect immediately, not just on next boot
      saveSettings();
      playFeedbackTone(1100.0f, 120);
      menuGoBack();  // back to Config
      return;

    case MENU_SCREEN_CONNECTIONS:
      switch (index) {
        case 0: pushMenuScreen(MENU_SCREEN_WIFI_LIST); break;
        case 1: pushMenuScreen(MENU_SCREEN_BLUETOOTH); break;
        case 2:
          // Cycles in place, like Bluetooth's On/Off -- actually
          // sleeps/wakes the SX1262 (see setFanetEnabled()), not just
          // the software FANET stack.
          fanetEnabled = !fanetEnabled;
          setFanetEnabled(fanetEnabled);
          saveSettings();
          displayDirty = true;
          playFeedbackTone(600.0f, 50);
          return;
      }
      playFeedbackTone(900.0f, 80);
      return;

    case MENU_SCREEN_WIFI_LIST: {
      if (index == 0) {
        // Cycles in place, like Bluetooth's and FANET's On/Off --
        // actually turns the WiFi radio off/on and keeps the connect/
        // retry state machine in sync (see setWifiRadioEnabled(),
        // wifi_manager.h/.cpp, which also persists wifiEnabled itself),
        // not just a display-layer flag.
        setWifiRadioEnabled(!wifiEnabled);
        displayDirty = true;
        playFeedbackTone(600.0f, 50);
        return;
      }
      uint8_t listed = wifiListedCount();
      if (index == listed + 1) {
        // Add Network -- opens the Vario-Setup hotspot (WifiSetup.h).
        pushMenuScreen(MENU_SCREEN_WIFI_SETUP);
        startWifiSetup();
        playFeedbackTone(900.0f, 80);
        return;
      }
      if (index == listed + 2) {
        pushMenuScreen(MENU_SCREEN_WIFI_FORGET);
        playFeedbackTone(900.0f, 80);
        return;
      }
      int slot = wifiListedSlot(index - 1);
      if (slot >= 0) {
        selectWifiNetwork((uint8_t)slot);
        playFeedbackTone(1100.0f, 120);
      } else {
        playFeedbackTone(300.0f, 60);
      }
      menuGoBack();  // back to Connections
      return;
    }

    case MENU_SCREEN_WIFI_SETUP:
      menuGoBack();  // Cancel/Done -- also closes the hotspot if it's still up
      return;

    case MENU_SCREEN_WIFI_FORGET: {
      int slot = wifiDeviceSavedSlot(index);
      if (slot >= 0) {
        forgetSavedWifiNetwork((uint8_t)slot);
        playFeedbackTone(600.0f, 120);
      } else {
        playFeedbackTone(300.0f, 60);
      }
      menuGoBack();  // back to the WiFi list
      return;
    }

    case MENU_SCREEN_BLUETOOTH:
      switch (index) {
        case 0:
          // Cycles in place, like the ADS-B toggles -- stays on this
          // screen so the pilot can immediately hit "Scan for Devices"
          // right after turning it on.
          bleSetEnabled(!bleEnabled);
          displayDirty = true;
          playFeedbackTone(600.0f, 50);
          return;
        case 1:
          pushMenuScreen(MENU_SCREEN_BLUETOOTH_SCAN);
          playFeedbackTone(900.0f, 80);
          return;
        case 2:
          if (bleRememberedAddress[0] != '\0') {
            bleForgetDevice();
            playFeedbackTone(600.0f, 50);
          } else {
            playFeedbackTone(300.0f, 60);
          }
          displayDirty = true;
          return;
      }
      return;

    case MENU_SCREEN_BLUETOOTH_SCAN:
      if (bleEnabled && bleScanResultCount() > 0) {
        if (bleConnectToScanResult(index)) {
          playFeedbackTone(1100.0f, 120);  // success
        } else {
          playFeedbackTone(200.0f, 250);   // low/long tone -- connect failed, check Serial for why
        }
      } else {
        playFeedbackTone(300.0f, 60);
      }
      menuGoBack();  // back to Bluetooth (also stops the scan -- see menuGoBack())
      return;

    case MENU_SCREEN_MAP:
      if (mapFileCount > 0) {
        setSelectedDemFile(mapFileNames[index]);
        playFeedbackTone(1100.0f, 120);
      } else {
        playFeedbackTone(300.0f, 60);
      }
      menuGoBack();  // back to the main menu
      return;

    case MENU_SCREEN_ADSB_SETTINGS:
      switch (index) {
        case 0:
          pushMenuScreen(MENU_SCREEN_ADSB_RADIUS);
          playFeedbackTone(900.0f, 80);
          return;
        case 1:
          pushMenuScreen(MENU_SCREEN_ADSB_VERTICAL);
          playFeedbackTone(900.0f, 80);
          return;
        case 2:
          // Cycles in place -- stays on this screen so both toggles can
          // be flipped in one visit.
          adsbAutoJumpEnabled = !adsbAutoJumpEnabled;
          saveSettings();
          displayDirty = true;
          playFeedbackTone(600.0f, 50);
          return;
        case 3:
          // Off -> Alarm -> Voice -> Off. Cycles in place, like Auto-Jump.
          adsbAlarmMode = (adsbAlarmMode + 1) % 3;
          saveSettings();
          displayDirty = true;
          playFeedbackTone(600.0f, 50);
          return;
        case 4:
          pushMenuScreen(MENU_SCREEN_ADSB_ALERT_VOLUME);
          menuSelectedIndex = adsbAlertVolumePercent / 10;  // start on the current level
          playFeedbackTone(900.0f, 80);
          return;
        case 5:
          pushMenuScreen(MENU_SCREEN_ADSB_RANGE_RINGS);
          playFeedbackTone(900.0f, 80);
          return;
        case 6:
          // Deliberately NOT saved -- see airspaceAlertBarEnabled's
          // comment in settings.h; always back on at next boot.
          airspaceAlertBarEnabled = !airspaceAlertBarEnabled;
          displayDirty = true;
          playFeedbackTone(600.0f, 50);
          return;
        case 7:
          // Unlike airspaceAlertBarEnabled above, this one IS saved --
          // see its comment in settings.h.
          airspaceInfoBarEnabled = !airspaceInfoBarEnabled;
          saveSettings();
          displayDirty = true;
          playFeedbackTone(600.0f, 50);
          return;
        case 8: {
          if (voiceProfileLoading()) return;
          char profiles[VOICE_MAX_PROFILES][VOICE_PROFILE_NAME_MAX] = {};
          uint8_t profileCount = listVoiceProfiles(profiles, VOICE_MAX_PROFILES);
          if (profileCount == 0) return;
          if (profileCount == 1) {
            playFeedbackTone(600.0f, 50);
            return;
          }

          int activeIndex = -1;
          if (voiceClipsLoaded()) {
            const char* activeProfile = voiceActiveProfile();
            for (uint8_t i = 0; i < profileCount; i++) {
              if (strcmp(profiles[i], activeProfile) == 0) {
                activeIndex = i;
                break;
              }
            }
          }
          uint8_t nextIndex = (activeIndex < 0)
                                ? 0
                                : (uint8_t)((activeIndex + 1) % profileCount);
          if (requestVoiceProfile(profiles[nextIndex])) {
            voiceProfileRequestPending = true;
            displayDirty = true;
          } else {
            playFeedbackTone(300.0f, 60);
          }
          return;
        }
      }
      return;

    // Sets the level immediately and STAYS on the screen (like the vario
    // volume screens), but -- unlike those -- does NOT auto-play a
    // preview on every selection. Press "Test" (the last item) when you
    // actually want to hear it. Double-press to go back to ADS-B Settings.
    case MENU_SCREEN_ADSB_ALERT_VOLUME:
      if (index == ADSB_ALERT_VOLUME_TEST_INDEX) {
        if (adsbAlarmMode == ADSB_ALARM_TONE) {
          interceptAlarmActive = true;
          interceptAlarmStart = millis();
        } else if (adsbAlarmMode == ADSB_ALARM_VOICE && voiceClipsLoaded()) {
          // Fixed example values -- same sentence structure a real alert
          // uses, so this is a true preview of what you'll hear in the
          // field, just with made-up numbers (2 o'clock, flying NW, at
          // 2400ft, 1200ft above you, 800m away).
          VoiceClipId sentence[VOICE_MAX_SENTENCE_CLIPS];
          int n = buildAlertSentence(sentence, VOICE_MAX_SENTENCE_CLIPS,
                                      2, true, 315.0f,
                                      2400, 1200, true,
                                      800, true);
          voicePlaySequence(sentence, n, varioVolumeToGain(adsbAlertVolumePercent));
        }
        // ADSB_ALARM_OFF: nothing to test -- silently does nothing.
        return;
      }
      adsbAlertVolumePercent = VARIO_VOLUME_CHOICE_PERCENT(index);
      saveSettings();
      displayDirty = true;
      playFeedbackTone(600.0f, 50);
      return;

    case MENU_SCREEN_ADSB_RADIUS:
      adsbAlertRadiusKm = ADSB_RADIUS_CHOICES_KM[index];
      saveSettings();
      playFeedbackTone(1100.0f, 120);
      menuGoBack();  // back to ADS-B Settings
      return;

    case MENU_SCREEN_ADSB_VERTICAL:
      adsbAlertVerticalFt = ADSB_VERTICAL_CHOICES_FT[index];
      saveSettings();
      playFeedbackTone(1100.0f, 120);
      menuGoBack();  // back to ADS-B Settings
      return;

    case MENU_SCREEN_ADSB_RANGE_RINGS:
      adsbRingOuterKm = ADSB_RING_OUTER_CHOICES_KM[index];
      adsbRingInnerKm = adsbRingOuterKm / 2.0f;
      saveSettings();
      playFeedbackTone(1100.0f, 120);
      menuGoBack();  // back to ADS-B Settings
      return;

    case MENU_SCREEN_WEATHER_SETTINGS:
      switch (index) {
        case 0: pushMenuScreen(MENU_SCREEN_WEATHER_POLL_INTERVAL); break;
        case 1: pushMenuScreen(MENU_SCREEN_WEATHER_STATIONS_SHOWN); break;
        case 2: pushMenuScreen(MENU_SCREEN_WEATHER_SOURCE); break;
      }
      playFeedbackTone(900.0f, 80);
      return;

    case MENU_SCREEN_WEATHER_POLL_INTERVAL:
      weatherPollIntervalMs = WEATHER_POLL_CHOICES_MS[index];
      saveSettings();
      playFeedbackTone(1100.0f, 120);
      menuGoBack();  // back to Weather Settings
      return;

    case MENU_SCREEN_WEATHER_STATIONS_SHOWN:
      weatherStationsShown = WEATHER_STATIONS_CHOICES[index];
      saveSettings();
      playFeedbackTone(1100.0f, 120);
      menuGoBack();  // back to Weather Settings
      return;

    case MENU_SCREEN_WEATHER_SOURCE:
      weatherSource = (index == 0) ? WEATHER_SOURCE_FANET : WEATHER_SOURCE_ZEPHYR;
      saveSettings();
      playFeedbackTone(1100.0f, 120);
      menuGoBack();  // back to Weather Settings
      return;

    case MENU_SCREEN_FLIGHT_RECORDINGS:
      switch (index) {
        case 0:
          // Cycles in place. Deliberately NOT saved -- see
          // flightRecorderEnabled's comment in settings.h; always back
          // on at next boot. setFlightRecorderEnabled() also cleanly
          // closes an in-progress recording if this just turned it off.
          setFlightRecorderEnabled(!flightRecorderEnabled);
          displayDirty = true;
          playFeedbackTone(600.0f, 50);
          return;
        case 1:
          pushMenuScreen(MENU_SCREEN_EXPORT_FILES);
          playFeedbackTone(900.0f, 80);
          return;
        case 2:
          // Cycles in place. Persisted -- see igcAutoStopEnabled's
          // comment in settings.h.
          igcAutoStopEnabled = !igcAutoStopEnabled;
          saveSettings();
          displayDirty = true;
          playFeedbackTone(600.0f, 50);
          return;
      }
      return;

    case MENU_SCREEN_EXPORT_FILES:
      if (index == 0) {
        if (isFileServerRunning()) {
          stopFileServer();
          playFeedbackTone(600.0f, 50);
        } else {
          bool started = startFileServer();
          // startFileServer() fails cleanly (does nothing) if WiFi isn't
          // connected or the SD card isn't mounted -- the low tone here
          // is the only feedback for that, same as an empty WiFi slot
          // elsewhere in this menu; Serial has the specific reason.
          playFeedbackTone(started ? 1100.0f : 300.0f, started ? 120 : 60);
        }
        displayDirty = true;  // item count (1 <-> 2) and label both change
        return;
      }
      // index == 1 -- the URL row, informational only.
      playFeedbackTone(900.0f, 40);
      return;

    case MENU_SCREEN_FANET_MESSAGING:
      if (index == 0) {
        // Cycles in place, like FANET's own On/Off -- persisted (unlike
        // airspaceAlertBarEnabled-style safety toggles), see
        // fanetMessagingEnabled's comment in settings.h.
        fanetMessagingEnabled = !fanetMessagingEnabled;
        saveSettings();
        displayDirty = true;
        playFeedbackTone(600.0f, 50);
        return;
      }
      // index >= 1 -- broadcast the corresponding preset immediately.
      // sendFanetMessagePreset() fails cleanly (messaging/FANET off, or
      // the radio's channel-busy/TX-in-flight -- see its comment in
      // FanetMessaging.h) rather than queuing or retrying; the tone is
      // the only feedback either way, matching Export Files' pattern
      // above.
      {
        bool sent = sendFanetMessagePreset((uint8_t)(index - 1));
        playFeedbackTone(sent ? 1100.0f : 300.0f, sent ? 120 : 60);
      }
      return;
  }
}

void menuMoveDown() {
  uint8_t count = getMenuItemCount(currentMenuScreen());
  if (count == 0) count = 1;
  menuSelectedIndex = (menuSelectedIndex + 1) % count;
  displayDirty = true;
  playFeedbackTone(500.0f, 40);
}

void menuSelectCurrentItem() {
  selectMenuItem(currentMenuScreen(), menuSelectedIndex);
}

void menuService() {
  if (!voiceProfileRequestPending || voiceProfileLoading()) return;
  voiceProfileRequestPending = false;
  if (voiceLastReloadOk() && voiceClipsLoaded()) {
    const char* activeProfile = voiceActiveProfile();
    strncpy(voiceProfile, activeProfile, VOICE_PROFILE_NAME_MAX - 1);
    voiceProfile[VOICE_PROFILE_NAME_MAX - 1] = '\0';
    saveSettings();
  } else {
    playFeedbackTone(200.0f, 250);
  }
  displayDirty = true;
}

bool menuSelectionIsImmediate() {
  return currentMenuScreen() == MENU_SCREEN_ADSB_ALERT_VOLUME &&
         menuSelectedIndex == ADSB_ALERT_VOLUME_TEST_INDEX;
}

// =====================================================
// ADD NETWORK SCREEN: the two steps side by side (left, then right), each
// with its own QR code and written fallback, then a status line saying
// which step to do next, with Cancel/Done as the only selectable row. It
// has its own layout because the steps must all stay on screen at once,
// which the big list rows in drawMenu() can't fit.
//
// Both QR codes are encoded once, the first time this screen is drawn,
// then kept: their text never changes, and this runs on Core 1, which
// mustn't repeat work it doesn't need to.
// =====================================================

// Version 3 is 29x29 modules. If either string below ever outgrows it,
// encoding fails, that QR code is left off the screen and the written
// instructions still work.
//
// The two codes should look the same size, but the encoder always picks
// the smallest version that fits, and the URL is much shorter. So each
// code gets its own error correction level, chosen to land on version 3:
//   Join string, 37 bytes: ECC medium (version 3 holds 42; version 2, 26)
//   URL, 19 bytes:         ECC high   (version 3 holds 24; version 2, 14)
#define WIFI_SETUP_QR_MAX_VERSION 3
#define WIFI_SETUP_QR_MAX_MODULES 29
// 4px per module is about 0.85mm on this panel (~4.7 px/mm), which phones
// scan comfortably, and leaves each code its blank margin in a half-screen
// column: 116px wide in 150px, so 17px either side.
#define WIFI_SETUP_QR_SCALE_PX 4

struct WifiSetupQr {
  uint8_t size;  // modules per side; 0 = not available
  uint32_t rows[WIFI_SETUP_QR_MAX_MODULES];  // bit x of rows[y] set = dark
};

static WifiSetupQr wifiJoinQr;
static WifiSetupQr wifiPageQr;
static WifiSetupQr* wifiQrBeingEncoded = nullptr;

// esp_qrcode_generate() hands the finished code to a callback, with no
// way to pass it a destination -- hence wifiQrBeingEncoded above. The
// callback runs before esp_qrcode_generate() returns, on the same task.
static void captureWifiSetupQr(esp_qrcode_handle_t qrcode) {
  int size = esp_qrcode_get_size(qrcode);
  if (size > WIFI_SETUP_QR_MAX_MODULES) return;

  for (int y = 0; y < size; y++) {
    uint32_t row = 0;
    for (int x = 0; x < size; x++) {
      if (esp_qrcode_get_module(qrcode, x, y)) row |= (1UL << x);
    }
    wifiQrBeingEncoded->rows[y] = row;
  }
  wifiQrBeingEncoded->size = (uint8_t)size;
}

static void encodeWifiSetupQr(WifiSetupQr& qr, const char* text, int eccLevel) {
  qr.size = 0;
  wifiQrBeingEncoded = &qr;

  esp_qrcode_config_t config = {};
  config.display_func = captureWifiSetupQr;
  config.max_qrcode_version = WIFI_SETUP_QR_MAX_VERSION;
  config.qrcode_ecc_level = eccLevel;

  if (esp_qrcode_generate(&config, text) != ESP_OK || qr.size == 0) {
    Serial.printf("[WIFI SETUP] Could not make a QR code for '%s'\n", text);
  }
}

// Draws a QR code centred on (centreX, centreY).
static void drawWifiSetupQr(const WifiSetupQr& qr, int centreX, int centreY) {
  const int sidePx = qr.size * WIFI_SETUP_QR_SCALE_PX;
  const int left = centreX - sidePx / 2;
  const int top = centreY - sidePx / 2;

  u8g2.setDrawColor(1);
  for (int y = 0; y < qr.size; y++) {
    for (int x = 0; x < qr.size; x++) {
      if (qr.rows[y] & (1UL << x)) {
        u8g2.drawBox(left + x * WIFI_SETUP_QR_SCALE_PX, top + y * WIFI_SETUP_QR_SCALE_PX,
                     WIFI_SETUP_QR_SCALE_PX, WIFI_SETUP_QR_SCALE_PX);
      }
    }
  }
}

static void drawCentredStr(int centreX, int baselineY, const char* text) {
  u8g2.drawStr(centreX - u8g2.getStrWidth(text) / 2, baselineY, text);
}

static void drawWifiSetupScreen() {
  static bool qrEncoded = false;
  if (!qrEncoded) {
    encodeWifiSetupQr(wifiJoinQr, "WIFI:T:WPA;S:" WIFI_SETUP_AP_NAME
                                  ";P:" WIFI_SETUP_AP_PASSWORD ";;", ESP_QRCODE_ECC_MED);
    encodeWifiSetupQr(wifiPageQr, WIFI_SETUP_URL, ESP_QRCODE_ECC_HIGH);
    qrEncoded = true;
  }

  // Layout notes: a QR code needs 4 modules (16px) of blank space around
  // it to scan reliably, which is what sets the gaps above and below the
  // codes, and why the divider sits exactly between them.
  const int leftCentreX = SCREEN_W / 4;
  const int rightCentreX = SCREEN_W * 3 / 4;
  const int headingY = 62;
  const int detailLine1Y = 81;
  const int detailLine2Y = 98;
  const int qrCentreY = 176;
  const int captionY = 263;
  const int stepsBottom = 276;
  const int statusY = 306;
  const int noteY = 334;
  const int cancelTop = 356;
  const int cancelH = SCREEN_H - cancelTop;

  // The steps are always shown, whatever the status line says.
  u8g2.setDrawColor(1);
  u8g2.drawLine(SCREEN_W / 2, 40, SCREEN_W / 2, stepsBottom);
  u8g2.drawLine(0, stepsBottom, SCREEN_W, stepsBottom);

  u8g2.setFont(u8g2_font_helvB12_tf);
  drawCentredStr(leftCentreX, headingY, "1. Join WiFi");
  drawCentredStr(rightCentreX, headingY, "2. Open page");

  drawWifiSetupQr(wifiJoinQr, leftCentreX, qrCentreY);
  drawWifiSetupQr(wifiPageQr, rightCentreX, qrCentreY);

  u8g2.setFont(u8g2_font_helvR10_tf);
  drawCentredStr(leftCentreX, detailLine1Y, "Name: " WIFI_SETUP_AP_NAME);
  drawCentredStr(leftCentreX, detailLine2Y, "Password: " WIFI_SETUP_AP_PASSWORD);
  drawCentredStr(leftCentreX, captionY, "Scan to join");
  drawCentredStr(rightCentreX, detailLine1Y, "In a browser, go to:");
  drawCentredStr(rightCentreX, detailLine2Y, WIFI_SETUP_IP);
  drawCentredStr(rightCentreX, captionY, "Scan to open");

  // Status gets its own line, so it can't push a step off the screen.
  char status[48];
  const WifiSetupState state = wifiSetupState();
  switch (state) {
    case WIFI_SETUP_STARTING:     snprintf(status, sizeof(status), "Scanning for networks..."); break;
    case WIFI_SETUP_WAITING:      snprintf(status, sizeof(status), "Hotspot on: do step 1"); break;
    case WIFI_SETUP_PHONE_JOINED: snprintf(status, sizeof(status), "Phone joined: now do step 2"); break;
    case WIFI_SETUP_SAVED:        snprintf(status, sizeof(status), "Saved: %s", wifiSetupSavedSsid()); break;
    case WIFI_SETUP_FAILED:       snprintf(status, sizeof(status), "Hotspot failed to start"); break;
    default:                      snprintf(status, sizeof(status), "Hotspot closed"); break;
  }
  // A long network name is trimmed to fit rather than running off the edge.
  u8g2.setFont(u8g2_font_helvB10_tf);
  size_t statusLen = strlen(status);
  while (statusLen > 0 && u8g2.getStrWidth(status) > SCREEN_W - 20) {
    status[--statusLen] = '\0';
  }
  drawCentredStr(SCREEN_W / 2, statusY, status);

  // The hotspot has no internet, so a phone with mobile data on may leave
  // it or send the page over mobile data instead, and the page won't open.
  // This has to be here, not only on the setup page, because that page is
  // exactly what can't be reached when it happens.
  u8g2.setFont(u8g2_font_helvR10_tf);
  if (state == WIFI_SETUP_SAVED) {
    drawCentredStr(SCREEN_W / 2, noteY, "You can turn mobile data back on");
  } else if (state == WIFI_SETUP_STARTING || state == WIFI_SETUP_WAITING ||
             state == WIFI_SETUP_PHONE_JOINED) {
    drawCentredStr(SCREEN_W / 2, noteY, "Page won't open? Turn mobile data off");
  }

  // Cancel/Done is the screen's only item, so it's always highlighted.
  char label[32];
  getMenuItemLabel(MENU_SCREEN_WIFI_SETUP, 0, label, sizeof(label));
  u8g2.setFont(u8g2_font_helvB14_tf);
  u8g2.drawBox(0, cancelTop, SCREEN_W, cancelH);
  u8g2.setDrawColor(0);  // inverted text, as on any highlighted menu row
  u8g2.drawStr(14, cancelTop + cancelH / 2 + 5, label);
  u8g2.setDrawColor(1);
}

// =====================================================
// MENU RENDERING: full-screen list shown instead of the normal top bar +
// page while menuActive is true. Generic over whichever screen is on top
// of the navigation stack -- see getMenuTitle()/getMenuItemCount()/
// getMenuItemLabel() above for the per-screen content.
// =====================================================
void drawMenu() {
  MenuScreen screen = currentMenuScreen();
  uint8_t itemCount = getMenuItemCount(screen);

  u8g2.setFont(u8g2_font_helvB14_tf);
  u8g2.drawStr(10, 30, getMenuTitle(screen));
  u8g2.drawLine(0, 40, SCREEN_W, 40);

  if (screen == MENU_SCREEN_WIFI_SETUP) {
    drawWifiSetupScreen();
    return;
  }

  const int top = 40;
  const int available = SCREEN_H - top;

  // Long lists (currently just the 28-entry Time menu) use a fixed,
  // comfortably-sized row plus a small gap between rows, scrolling to
  // keep the highlighted item in view -- like a normal phone menu --
  // instead of squeezing every item onto the screen at once, which is
  // what made rows unreadably thin once there were more than about 8 of
  // them. Short lists keep the original behaviour of stretching evenly
  // to fill the screen with no gap between rows.
  const int scrollRowGapPx = 3;
  const int scrollRowHeightPx = 40;  // sized for u8g2_font_helvB14_tf below
  const int scrollRowPitchPx = scrollRowHeightPx + scrollRowGapPx;
  const int visibleRows = available / scrollRowPitchPx;

  bool scrolling = (visibleRows > 0) && (itemCount > (uint8_t)visibleRows);

  int rowH;
  int rowPitch;
  int firstVisible = 0;
  int lastVisible = itemCount - 1;

  if (scrolling) {
    rowH = scrollRowHeightPx;
    rowPitch = scrollRowPitchPx;

    // Keeps the highlighted row inside the visible window, scrolling
    // only as far as necessary -- and never past the point where the
    // last item would leave a blank gap at the bottom.
    int maxFirstVisible = itemCount - visibleRows;
    firstVisible = (int)menuSelectedIndex - (visibleRows - 1);
    if (firstVisible < 0) firstVisible = 0;
    if (firstVisible > maxFirstVisible) firstVisible = maxFirstVisible;
    lastVisible = firstVisible + visibleRows - 1;
  } else {
    rowH = available / itemCount;
    rowPitch = rowH;
  }

  u8g2.setFont(u8g2_font_helvB14_tf);
  for (int i = firstVisible; i <= lastVisible; i++) {
    int rowY = top + (i - firstVisible) * rowPitch;

    if (i == menuSelectedIndex) {
      u8g2.setDrawColor(1);
      u8g2.drawBox(0, rowY, SCREEN_W, rowH);
      u8g2.setDrawColor(0);  // inverted text on the highlighted row
    }

    char label[32];
    getMenuItemLabel(screen, i, label, sizeof(label));
    u8g2.drawStr(14, rowY + rowH / 2 + 5, label);

    u8g2.setDrawColor(1);
  }
}
