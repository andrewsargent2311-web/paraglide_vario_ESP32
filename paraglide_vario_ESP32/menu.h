#ifndef MENU_H
#define MENU_H

#include <Arduino.h>
#include <U8g2lib.h>
#include "settings.h"

// =====================================================
// Display geometry. Duplicated (identically) from the main .ino so this
// header is self-contained -- see the derivation notes next to SCREEN_W/
// SCREEN_H in the .ino if that panel geometry ever changes.
// =====================================================
#define SCREEN_W 300
#define SCREEN_H 400

// =====================================================
// PAGES
// The actual page-drawing functions (drawParagliderPage() etc.) still live
// in the main .ino -- this type/state lives here because the menu needs to
// read and write it ("Main Page" submenu swaps the active main page).
// =====================================================
enum Page { PAGE_PARAGLIDER = 0,
            PAGE_WEATHER,
            PAGE_ADSB,
            PAGE_PARAMOTOR,
            PAGE_COUNT };

// Only 3 pages are cycled through with a short press. Slot 0 is the "main"
// page and is swappable between Paraglider and Paramotor from the menu;
// slots 1 and 2 are fixed at Weather and ADS-B.
#define ACTIVE_PAGE_COUNT 3

extern Page currentPage;
extern const char* PAGE_NAMES[PAGE_COUNT];
extern Page activePages[ACTIVE_PAGE_COUNT];
extern uint8_t activePageIndex;

// =====================================================
// MENU SCREENS
// The menu is a small tree, navigated with a stack (see menu.cpp):
//   MAIN
//   |-- MAIN_PAGE_SELECT  (Paraglider / Paramotor)
//   |-- CONFIG
//   |     |-- CONFIG_TIME     ("NZ (Auto)" DST-aware default, or a fixed
//   |     |                    manual UTC offset from -12 to +14 hours)
//   |     |-- UNITS           (Altitude / Speed, cycled in place)
//   |     `-- VARIO_FREQ      (climb-tone min Hz)
//   |-- MAP                (list of *.ADEM files found on the SD card)
//   |-- ADSB_SETTINGS
//   |     |-- ADSB_RADIUS       (horizontal alert trigger distance)
//   |     |-- ADSB_VERTICAL     (vertical alert trigger distance)
//   |     |-- (Auto-Jump toggle, cycled in place)
//   |     |-- (Alarm Sound toggle, cycled in place)
//   |     `-- ADSB_RANGE_RINGS  (far/default range ring pair -- 15/30km,
//   |                            10/20km, 20/40km, or 30/60km)
//   `-- WEATHER_SETTINGS
//         |-- WEATHER_POLL_INTERVAL   (Zephyr station poll cadence)
//         `-- WEATHER_STATIONS_SHOWN  (how many stations to display)
//
// Gestures (see updatePageButton() in the main .ino):
//   short press  -- move the highlight down (wraps within the current screen)
//   hold 2s      -- select the highlighted item (enter a submenu, apply a
//                   value, or toggle a setting)
//   double press -- go back one level, or close the menu from the top level
// =====================================================
enum MenuScreen {
  MENU_SCREEN_MAIN = 0,
  MENU_SCREEN_MAIN_PAGE_SELECT,
  MENU_SCREEN_CONFIG,
  MENU_SCREEN_CONFIG_TIME,
  MENU_SCREEN_UNITS,
  MENU_SCREEN_VARIO_FREQ,
  MENU_SCREEN_MAP,
  MENU_SCREEN_ADSB_SETTINGS,
  MENU_SCREEN_ADSB_RADIUS,
  MENU_SCREEN_ADSB_VERTICAL,
  MENU_SCREEN_ADSB_RANGE_RINGS,
  MENU_SCREEN_WEATHER_SETTINGS,
  MENU_SCREEN_WEATHER_POLL_INTERVAL,
  MENU_SCREEN_WEATHER_STATIONS_SHOWN
};

extern bool menuActive;
extern uint8_t menuSelectedIndex;

// A "double press" is two presses with less than this many ms between the
// first release and the second press-down; a 2s hold on an open menu
// selects the highlighted item. Consumed by updatePageButton() in the
// main .ino.
#define MENU_DOUBLE_PRESS_MS 800
#define MENU_SELECT_HOLD_MS 2000

// =====================================================
// MAP FILE SELECTION
// The Map screen scans the SD card root for *.ADEM tiles when opened (see
// scanMapFiles() in menu.cpp) and lets the pilot choose which one
// getGroundElevationM() should read from. selectedDemFile is read by the
// DEM lookup in the main .ino's background task, guarded by sdMutex --
// always go through setSelectedDemFile() to change it, never assign it
// directly, or a scan running on the other core could read a half-written
// filename.
// =====================================================
#define DEM_FILENAME_MAX_LEN 32
extern char selectedDemFile[DEM_FILENAME_MAX_LEN];
void setSelectedDemFile(const char* filename);

// =====================================================
// Dependencies provided by the main .ino
// =====================================================
extern U8G2_ST7305_300X400_1_4W_HW_SPI u8g2;
extern bool sdCardOK;
extern SemaphoreHandle_t sdMutex;

// Set whenever page/menu state changes; drives an immediate redraw instead
// of waiting for the next 1Hz display tick.
extern bool displayDirty;

// Short non-blocking UI feedback tone (defined in the main .ino).
void playFeedbackTone(float freq, unsigned long durationMs);

// =====================================================
// Menu functions
// =====================================================
void openMenu();
void closeMenu();
void menuMoveDown();
void menuSelectCurrentItem();
void menuGoBack();
void drawMenu();

#endif  // MENU_H
