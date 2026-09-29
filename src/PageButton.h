#pragma once

#include <Arduino.h>

#include "Buzzer.h"     // playFeedbackTone(), setToneFrequency(), muteToneActive/Start/IsMuteSequence, pageBeepUntil, beepOn, AMP_ENABLE_PIN
#include "Display.h"    // displayDirty
#include "DrawPages.h"  // Page, ACTIVE_PAGE_COUNT, PAGE_COUNT, PAGE_PARAGLIDER, PAGE_WEATHER, PAGE_ADSB
#include "menu.h"       // menuActive, openMenu(), menuGoBack(), menuMoveDown(), menuSelectCurrentItem(), MENU_DOUBLE_PRESS_MS, MENU_SELECT_HOLD_MS
#include "settings.h"   // buzzerMuted, saveSettings()

// =====================================================
// Gestures:
//   Menu closed: short press cycles the 3 active pages; double press opens
//                the menu; holding 3s toggles the vario mute.
//   Menu open:   short press moves the selection down (wraps); holding 2s
//                selects the highlighted item and closes the menu.
// -----------------------------------------------------
// pageBeepUntil and beepOn were declared under this same original
// banner but now live in Buzzer.h -- see that file's header comment for
// why (pageBeepUntil is read only by Buzzer's audio state machine;
// beepOn is written from both this file and Vario's old home, read by
// neither). PAGE_BEEP_FREQ/PAGE_BEEP_MS stay here: they're just the
// specific tone/duration this file asks Buzzer's playFeedbackTone() to
// play.
// =====================================================
#define KEY_PIN 18
#define KEY_DEBOUNCE_MS 10
#define KEY_LONG_PRESS_MS 4000
#define PAGE_BEEP_FREQ 500  // Sets page beep frequency
#define PAGE_BEEP_MS 200    // This sets how long the page beep tone goes for
// A "double press" is two presses with less than this many ms between the
// first release and the second press-down. 50ms is what was asked for, but
// note it's faster than most people can physically double-click (a typical
// double-click is more like 150-400ms) -- raise this if the menu doesn't
// open reliably for you. Every short press is held for up to this long
// before it's actioned (to see whether a second press follows), so this
// value also sets the latency added to ordinary page-cycle/menu-navigate
// presses.

extern Page currentPage;
extern const char* PAGE_NAMES[PAGE_COUNT];

// Only 3 pages are cycled through with a short press. Slot 0 is the "main"
// page and is swappable between Paraglider and Paramotor from the menu;
// slots 1 and 2 are fixed at Weather and ADS-B. (ACTIVE_PAGE_COUNT is
// defined in DrawPages.h, shared with drawTopBar()'s page-dots indicator.)
extern Page activePages[ACTIVE_PAGE_COUNT];
extern uint8_t activePageIndex;  // index into activePages[]; kept in sync with currentPage

void advanceActivePage();

// Jumps straight to a page if it's currently one of the 3 active slots
// (used by the ADS-B intercept alert to force-switch to the traffic page),
// keeping activePageIndex in sync so short-press cycling continues
// correctly afterward. Does nothing if the page isn't currently active.
void jumpToActivePage(Page page);

// =====================================================
// PAGE BUTTON: drives page cycling, the vario mute hold, and the on-screen
// menu (double press to open; short press to navigate; 2s hold to select).
// See the gesture summary above.
// =====================================================
void updatePageButton();
