#include "menu.h"
#include "wifi_manager.h"
#include "FileServer.h"
#include "FanetMessaging.h"
#include "ble_manager.h"
#include "secrets.h"
#include <FS.h>
#include <SD_MMC.h>
#include <string.h>
#include <strings.h>  // strcasecmp
#include <math.h>     // fabsf

bool menuActive = false;
uint8_t menuSelectedIndex = 0;

// =====================================================
// MAP FILE LIST
// Populated by scanMapFiles() whenever the Map screen is opened.
// =====================================================
#define MAX_MAP_FILES 8
static char mapFileNames[MAX_MAP_FILES][DEM_FILENAME_MAX_LEN];
static uint8_t mapFileCount = 0;

// =====================================================
// SCREEN NAVIGATION STACK
// 4 deep now that Vario Beep adds a 4th level: MAIN > CONFIG >
// VARIO_BEEP > (Min Gap / Max Gap / Min Pulse / Max Pulse). Every other
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
// VARIO FREQUENCY CHOICES
// =====================================================
static const int VARIO_FREQ_CHOICES[] = { 500, 550, 600, 650, 700 };
static const uint8_t VARIO_FREQ_CHOICE_COUNT = 5;

// =====================================================
// VOLUME CHOICES
// =====================================================
static const uint8_t VOLUME_CHOICES_PERCENT[] = { 20, 40, 60, 80, 85, 90, 95, 100 };
static const uint8_t VOLUME_CHOICE_COUNT = 8;

// =====================================================
// VARIO BEEP TIMING CHOICES
// Same 100ms-1000ms/100ms-step range used for all 4 climbGapMinMs/
// climbGapMaxMs/climbPulseMinMs/climbPulseMaxMs settings (settings.h) --
// choice[index] = (index + 1) * 100.
// =====================================================
#define VARIO_BEEP_CHOICE_COUNT 10
#define VARIO_BEEP_CHOICE_MS(index) (((unsigned long)(index) + 1UL) * 100UL)

// =====================================================
// ADS-B ALERT CHOICES
// =====================================================
static const float ADSB_RADIUS_CHOICES_KM[] = { 3.0f, 5.0f, 8.0f, 10.0f };
static const uint8_t ADSB_RADIUS_CHOICE_COUNT = 4;

static const float ADSB_VERTICAL_CHOICES_FT[] = { 1000.0f, 1500.0f, 2000.0f, 3000.0f };
static const uint8_t ADSB_VERTICAL_CHOICE_COUNT = 4;

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
static uint8_t getMenuItemCount(MenuScreen screen) {
  switch (screen) {
    case MENU_SCREEN_MAIN: return 7;
    case MENU_SCREEN_MAIN_PAGE_SELECT: return 2;
    case MENU_SCREEN_CONFIG: return 6;
    case MENU_SCREEN_CONFIG_TIME: return TIMEZONE_CHOICE_COUNT;
    case MENU_SCREEN_UNITS: return 2;
    case MENU_SCREEN_VARIO_FREQ: return VARIO_FREQ_CHOICE_COUNT;
    case MENU_SCREEN_CONFIG_VOLUME: return VOLUME_CHOICE_COUNT;
    case MENU_SCREEN_VARIO_BEEP: return 4;
    case MENU_SCREEN_VARIO_BEEP_GAP_MIN: return VARIO_BEEP_CHOICE_COUNT;
    case MENU_SCREEN_VARIO_BEEP_GAP_MAX: return VARIO_BEEP_CHOICE_COUNT;
    case MENU_SCREEN_VARIO_BEEP_PULSE_MIN: return VARIO_BEEP_CHOICE_COUNT;
    case MENU_SCREEN_VARIO_BEEP_PULSE_MAX: return VARIO_BEEP_CHOICE_COUNT;
    case MENU_SCREEN_SCREEN: return 2;
    case MENU_SCREEN_CONNECTIONS: return 4;
    case MENU_SCREEN_FANET_MESSAGING: return 1 + FANET_MESSAGE_PRESET_COUNT;
    case MENU_SCREEN_WIFI_LIST: return WIFI_NETWORK_COUNT + 1;
    case MENU_SCREEN_BLUETOOTH: return 3;
    case MENU_SCREEN_BLUETOOTH_SCAN: return bleScanResultCount() > 0 ? bleScanResultCount() : 1;
    case MENU_SCREEN_MAP: return mapFileCount > 0 ? mapFileCount : 1;
    case MENU_SCREEN_ADSB_SETTINGS: return 7;
    case MENU_SCREEN_ADSB_RADIUS: return ADSB_RADIUS_CHOICE_COUNT;
    case MENU_SCREEN_ADSB_VERTICAL: return ADSB_VERTICAL_CHOICE_COUNT;
    case MENU_SCREEN_ADSB_RANGE_RINGS: return ADSB_RING_CHOICE_COUNT;
    case MENU_SCREEN_WEATHER_SETTINGS: return 3;
    case MENU_SCREEN_WEATHER_POLL_INTERVAL: return WEATHER_POLL_CHOICE_COUNT;
    case MENU_SCREEN_WEATHER_STATIONS_SHOWN: return WEATHER_STATIONS_CHOICE_COUNT;
    case MENU_SCREEN_WEATHER_SOURCE: return 2;
    case MENU_SCREEN_FLIGHT_RECORDINGS: return 2;
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
    case MENU_SCREEN_VARIO_FREQ: return "VARIO FREQ";
    case MENU_SCREEN_CONFIG_VOLUME: return "VOLUME";
    case MENU_SCREEN_VARIO_BEEP: return "VARIO BEEP";
    case MENU_SCREEN_VARIO_BEEP_GAP_MIN: return "MIN GAP";
    case MENU_SCREEN_VARIO_BEEP_GAP_MAX: return "MAX GAP";
    case MENU_SCREEN_VARIO_BEEP_PULSE_MIN: return "MIN PULSE";
    case MENU_SCREEN_VARIO_BEEP_PULSE_MAX: return "MAX PULSE";
    case MENU_SCREEN_SCREEN: return "SCREEN";
    case MENU_SCREEN_CONNECTIONS: return "CONNECTIONS";
    case MENU_SCREEN_FANET_MESSAGING: return "FANET MESSAGING";
    case MENU_SCREEN_WIFI_LIST: return "WIFI";
    case MENU_SCREEN_BLUETOOTH: return "BLUETOOTH";
    case MENU_SCREEN_BLUETOOTH_SCAN: return "SCAN DEVICES";
    case MENU_SCREEN_MAP: return "MAP";
    case MENU_SCREEN_ADSB_SETTINGS: return "ADSB SETTINGS";
    case MENU_SCREEN_ADSB_RADIUS: return "ALERT RADIUS";
    case MENU_SCREEN_ADSB_VERTICAL: return "VERT THRESHOLD";
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
      static const char* items[] = { "Main Page", "Config", "Connections", "Map", "ADSB Settings", "Weather Settings", "Flight Recordings" };
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
        case 2: snprintf(buf, buflen, "Vario Freq"); break;
        case 3: snprintf(buf, buflen, "Volume"); break;
        case 4: snprintf(buf, buflen, "Vario Beep"); break;
        case 5: snprintf(buf, buflen, "Screen"); break;
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
    case MENU_SCREEN_VARIO_FREQ: {
      int hz = VARIO_FREQ_CHOICES[index];
      bool isActive = (climbToneMinHz == hz);
      snprintf(buf, buflen, "%d Hz%s", hz, isActive ? " *" : "");
      break;
    }
    case MENU_SCREEN_CONFIG_VOLUME: {
      uint8_t pct = VOLUME_CHOICES_PERCENT[index];
      bool isActive = (buzzerVolumePercent == pct);
      snprintf(buf, buflen, "%u%%%s", pct, isActive ? " *" : "");
      break;
    }
    case MENU_SCREEN_VARIO_BEEP: {
      static const char* items[] = { "Min Gap", "Max Gap", "Min Pulse", "Max Pulse" };
      snprintf(buf, buflen, "%s", items[index]);
      break;
    }
    case MENU_SCREEN_VARIO_BEEP_GAP_MIN: {
      unsigned long ms = VARIO_BEEP_CHOICE_MS(index);
      bool isActive = (climbGapMinMs == ms);
      snprintf(buf, buflen, "%lu ms%s", ms, isActive ? " *" : "");
      break;
    }
    case MENU_SCREEN_VARIO_BEEP_GAP_MAX: {
      unsigned long ms = VARIO_BEEP_CHOICE_MS(index);
      bool isActive = (climbGapMaxMs == ms);
      snprintf(buf, buflen, "%lu ms%s", ms, isActive ? " *" : "");
      break;
    }
    case MENU_SCREEN_VARIO_BEEP_PULSE_MIN: {
      unsigned long ms = VARIO_BEEP_CHOICE_MS(index);
      bool isActive = (climbPulseMinMs == ms);
      snprintf(buf, buflen, "%lu ms%s", ms, isActive ? " *" : "");
      break;
    }
    case MENU_SCREEN_VARIO_BEEP_PULSE_MAX: {
      unsigned long ms = VARIO_BEEP_CHOICE_MS(index);
      bool isActive = (climbPulseMaxMs == ms);
      snprintf(buf, buflen, "%lu ms%s", ms, isActive ? " *" : "");
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
        case 3: snprintf(buf, buflen, "FANET Messaging"); break;
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
      const char* ssid = WIFI_NETWORKS[index - 1].ssid;
      bool isEmpty = (ssid == nullptr || ssid[0] == '\0');
      bool isSelected = (index - 1 == selectedWifiIndex);
      if (isEmpty) {
        snprintf(buf, buflen, "(empty)");
      } else if (isSelected) {
        snprintf(buf, buflen, "%s%s", ssid, wifiConnected ? " * connected" : " * connecting");
      } else {
        snprintf(buf, buflen, "%s", ssid);
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
        snprintf(buf, buflen, "Alarm Sound: %s", adsbAlarmMuted ? "Off" : "On");
      } else if (index == 4) {
        snprintf(buf, buflen, "Range Rings");
      } else if (index == 5) {
        snprintf(buf, buflen, "Airspace Alert Bar: %s", airspaceAlertBarEnabled ? "On" : "Off");
      } else if (index == 6) {
        snprintf(buf, buflen, "Airspace Info: %s", airspaceInfoBarEnabled ? "On" : "Off");
      } else {
        static const char* items[] = { "Alert Radius", "Vertical Threshold" };
        snprintf(buf, buflen, "%s", items[index]);
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
        case 3: pushMenuScreen(MENU_SCREEN_MAP); break;
        case 4: pushMenuScreen(MENU_SCREEN_ADSB_SETTINGS); break;
        case 5: pushMenuScreen(MENU_SCREEN_WEATHER_SETTINGS); break;
        case 6: pushMenuScreen(MENU_SCREEN_FLIGHT_RECORDINGS); break;
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
        case 2: pushMenuScreen(MENU_SCREEN_VARIO_FREQ); break;
        case 3: pushMenuScreen(MENU_SCREEN_CONFIG_VOLUME); break;
        case 4: pushMenuScreen(MENU_SCREEN_VARIO_BEEP); break;
        case 5: pushMenuScreen(MENU_SCREEN_SCREEN); break;
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

    case MENU_SCREEN_VARIO_FREQ:
      setClimbToneMinHz(VARIO_FREQ_CHOICES[index]);
      saveSettings();
      playFeedbackTone(1100.0f, 120);
      menuGoBack();  // back to Config
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
        case 0: pushMenuScreen(MENU_SCREEN_VARIO_BEEP_GAP_MIN); break;
        case 1: pushMenuScreen(MENU_SCREEN_VARIO_BEEP_GAP_MAX); break;
        case 2: pushMenuScreen(MENU_SCREEN_VARIO_BEEP_PULSE_MIN); break;
        case 3: pushMenuScreen(MENU_SCREEN_VARIO_BEEP_PULSE_MAX); break;
      }
      playFeedbackTone(900.0f, 80);
      return;

    case MENU_SCREEN_VARIO_BEEP_GAP_MIN:
      climbGapMinMs = VARIO_BEEP_CHOICE_MS(index);
      saveSettings();
      playFeedbackTone(1100.0f, 120);
      menuGoBack();  // back to Vario Beep
      return;

    case MENU_SCREEN_VARIO_BEEP_GAP_MAX:
      climbGapMaxMs = VARIO_BEEP_CHOICE_MS(index);
      saveSettings();
      playFeedbackTone(1100.0f, 120);
      menuGoBack();  // back to Vario Beep
      return;

    case MENU_SCREEN_VARIO_BEEP_PULSE_MIN:
      climbPulseMinMs = VARIO_BEEP_CHOICE_MS(index);
      saveSettings();
      playFeedbackTone(1100.0f, 120);
      menuGoBack();  // back to Vario Beep
      return;

    case MENU_SCREEN_VARIO_BEEP_PULSE_MAX:
      climbPulseMaxMs = VARIO_BEEP_CHOICE_MS(index);
      saveSettings();
      playFeedbackTone(1100.0f, 120);
      menuGoBack();  // back to Vario Beep
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
        case 3: pushMenuScreen(MENU_SCREEN_FANET_MESSAGING); break;
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
      const char* ssid = WIFI_NETWORKS[index - 1].ssid;
      if (ssid != nullptr && ssid[0] != '\0') {
        selectWifiNetwork(index - 1);
        playFeedbackTone(1100.0f, 120);
      } else {
        playFeedbackTone(300.0f, 60);  // empty slot -- nothing to connect to
      }
      menuGoBack();  // back to Connections
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
          adsbAlarmMuted = !adsbAlarmMuted;
          saveSettings();
          displayDirty = true;
          playFeedbackTone(600.0f, 50);
          return;
        case 4:
          pushMenuScreen(MENU_SCREEN_ADSB_RANGE_RINGS);
          playFeedbackTone(900.0f, 80);
          return;
        case 5:
          // Deliberately NOT saved -- see airspaceAlertBarEnabled's
          // comment in settings.h; always back on at next boot.
          airspaceAlertBarEnabled = !airspaceAlertBarEnabled;
          displayDirty = true;
          playFeedbackTone(600.0f, 50);
          return;
        case 6:
          // Unlike airspaceAlertBarEnabled above, this one IS saved --
          // see its comment in settings.h.
          airspaceInfoBarEnabled = !airspaceInfoBarEnabled;
          saveSettings();
          displayDirty = true;
          playFeedbackTone(600.0f, 50);
          return;
      }
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
