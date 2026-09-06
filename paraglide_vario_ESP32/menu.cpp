#include "menu.h"
#include <FS.h>
#include <SD_MMC.h>
#include <string.h>
#include <strings.h>  // strcasecmp
#include <math.h>     // fabsf

bool menuActive = false;
uint8_t menuSelectedIndex = 0;

char selectedDemFile[DEM_FILENAME_MAX_LEN] = "/DEM.ADEM";

// =====================================================
// MAP FILE LIST
// Populated by scanMapFiles() whenever the Map screen is opened.
// =====================================================
#define MAX_MAP_FILES 8
static char mapFileNames[MAX_MAP_FILES][DEM_FILENAME_MAX_LEN];
static uint8_t mapFileCount = 0;

// =====================================================
// SCREEN NAVIGATION STACK
// =====================================================
#define MENU_MAX_DEPTH 3
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
}

void menuGoBack() {
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
// ADS-B ALERT CHOICES
// =====================================================
static const float ADSB_RADIUS_CHOICES_KM[] = { 3.0f, 5.0f, 8.0f, 10.0f };
static const uint8_t ADSB_RADIUS_CHOICE_COUNT = 4;

static const float ADSB_VERTICAL_CHOICES_FT[] = { 1000.0f, 1500.0f, 2000.0f, 3000.0f };
static const uint8_t ADSB_VERTICAL_CHOICE_COUNT = 4;

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
    case MENU_SCREEN_MAIN: return 5;
    case MENU_SCREEN_MAIN_PAGE_SELECT: return 2;
    case MENU_SCREEN_CONFIG: return 3;
    case MENU_SCREEN_CONFIG_TIME: return TIMEZONE_CHOICE_COUNT;
    case MENU_SCREEN_UNITS: return 2;
    case MENU_SCREEN_VARIO_FREQ: return VARIO_FREQ_CHOICE_COUNT;
    case MENU_SCREEN_MAP: return mapFileCount > 0 ? mapFileCount : 1;
    case MENU_SCREEN_ADSB_SETTINGS: return 4;
    case MENU_SCREEN_ADSB_RADIUS: return ADSB_RADIUS_CHOICE_COUNT;
    case MENU_SCREEN_ADSB_VERTICAL: return ADSB_VERTICAL_CHOICE_COUNT;
    case MENU_SCREEN_WEATHER_SETTINGS: return 2;
    case MENU_SCREEN_WEATHER_POLL_INTERVAL: return WEATHER_POLL_CHOICE_COUNT;
    case MENU_SCREEN_WEATHER_STATIONS_SHOWN: return WEATHER_STATIONS_CHOICE_COUNT;
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
    case MENU_SCREEN_MAP: return "MAP";
    case MENU_SCREEN_ADSB_SETTINGS: return "ADSB SETTINGS";
    case MENU_SCREEN_ADSB_RADIUS: return "ALERT RADIUS";
    case MENU_SCREEN_ADSB_VERTICAL: return "VERT THRESHOLD";
    case MENU_SCREEN_WEATHER_SETTINGS: return "WEATHER SETTINGS";
    case MENU_SCREEN_WEATHER_POLL_INTERVAL: return "POLL INTERVAL";
    case MENU_SCREEN_WEATHER_STATIONS_SHOWN: return "STATIONS SHOWN";
    default: return "MENU";
  }
}

static void getMenuItemLabel(MenuScreen screen, uint8_t index, char* buf, size_t buflen) {
  switch (screen) {
    case MENU_SCREEN_MAIN: {
      static const char* items[] = { "Main Page", "Config", "Map", "ADSB Settings", "Weather Settings" };
      snprintf(buf, buflen, "%s", items[index]);
      break;
    }
    case MENU_SCREEN_MAIN_PAGE_SELECT: {
      const char* name = (index == 0) ? "Paraglider" : "Paramotor";
      bool isActive = (index == 0 && activePages[0] == PAGE_PARAGLIDER) || (index == 1 && activePages[0] == PAGE_PARAMOTOR);
      snprintf(buf, buflen, "%s%s", name, isActive ? " *" : "");
      break;
    }
    case MENU_SCREEN_CONFIG: {
      static const char* items[] = { "Time", "Units", "Vario Freq" };
      snprintf(buf, buflen, "%s", items[index]);
      break;
    }
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
    case MENU_SCREEN_WEATHER_SETTINGS: {
      static const char* items[] = { "Poll Interval", "Stations Shown" };
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
        case 2: pushMenuScreen(MENU_SCREEN_MAP); break;
        case 3: pushMenuScreen(MENU_SCREEN_ADSB_SETTINGS); break;
        case 4: pushMenuScreen(MENU_SCREEN_WEATHER_SETTINGS); break;
      }
      playFeedbackTone(900.0f, 80);
      return;

    case MENU_SCREEN_MAIN_PAGE_SELECT:
      activePages[0] = (index == 0) ? PAGE_PARAGLIDER : PAGE_PARAMOTOR;
      activePageIndex = 0;
      currentPage = activePages[0];
      playFeedbackTone(1100.0f, 120);
      closeMenu();  // matches the original single-level behaviour: pick a
                    // main page and go straight back to flying
      return;

    case MENU_SCREEN_CONFIG:
      switch (index) {
        case 0: pushMenuScreen(MENU_SCREEN_CONFIG_TIME); break;
        case 1: pushMenuScreen(MENU_SCREEN_UNITS); break;
        case 2: pushMenuScreen(MENU_SCREEN_VARIO_FREQ); break;
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
      displayDirty = true;
      playFeedbackTone(600.0f, 50);
      return;

    case MENU_SCREEN_VARIO_FREQ:
      setClimbToneMinHz(VARIO_FREQ_CHOICES[index]);
      playFeedbackTone(1100.0f, 120);
      menuGoBack();  // back to Config
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
          displayDirty = true;
          playFeedbackTone(600.0f, 50);
          return;
        case 3:
          adsbAlarmMuted = !adsbAlarmMuted;
          displayDirty = true;
          playFeedbackTone(600.0f, 50);
          return;
      }
      return;

    case MENU_SCREEN_ADSB_RADIUS:
      adsbAlertRadiusKm = ADSB_RADIUS_CHOICES_KM[index];
      playFeedbackTone(1100.0f, 120);
      menuGoBack();  // back to ADS-B Settings
      return;

    case MENU_SCREEN_ADSB_VERTICAL:
      adsbAlertVerticalFt = ADSB_VERTICAL_CHOICES_FT[index];
      playFeedbackTone(1100.0f, 120);
      menuGoBack();  // back to ADS-B Settings
      return;

    case MENU_SCREEN_WEATHER_SETTINGS:
      switch (index) {
        case 0: pushMenuScreen(MENU_SCREEN_WEATHER_POLL_INTERVAL); break;
        case 1: pushMenuScreen(MENU_SCREEN_WEATHER_STATIONS_SHOWN); break;
      }
      playFeedbackTone(900.0f, 80);
      return;

    case MENU_SCREEN_WEATHER_POLL_INTERVAL:
      weatherPollIntervalMs = WEATHER_POLL_CHOICES_MS[index];
      playFeedbackTone(1100.0f, 120);
      menuGoBack();  // back to Weather Settings
      return;

    case MENU_SCREEN_WEATHER_STATIONS_SHOWN:
      weatherStationsShown = WEATHER_STATIONS_CHOICES[index];
      playFeedbackTone(1100.0f, 120);
      menuGoBack();  // back to Weather Settings
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
  const int rowH = (SCREEN_H - top) / itemCount;

  u8g2.setFont(u8g2_font_helvB12_tf);
  for (int i = 0; i < itemCount; i++) {
    int rowY = top + i * rowH;

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
