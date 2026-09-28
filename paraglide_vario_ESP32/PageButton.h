#pragma once
// PageButton -- page cycling, vario-mute hold and menu gestures from the
// single KEY button; also owns the active-page table.
#include <Arduino.h>
#include "menu.h"  // Page, currentPage, activePages[], PAGE_NAMES[], ...

// =====================================================
// Gestures:
//   Menu closed: short press cycles the 3 active pages; double press opens
//                the menu; holding 3s toggles the vario mute.
//   Menu open:   short press moves the selection down (wraps); holding 2s
//                selects the highlighted item and closes the menu.
// =====================================================
#define KEY_PIN 18  
#define KEY_DEBOUNCE_MS 10
#define KEY_LONG_PRESS_MS 4000
#define PAGE_BEEP_FREQ 500    // Sets page beep frequency
#define PAGE_BEEP_MS 200    // This sets how long the page beep tone goes for
// A "double press" is two presses with less than this many ms between the
// first release and the second press-down. 50ms is what was asked for, but
// note it's faster than most people can physically double-click (a typical
// double-click is more like 150-400ms) -- raise this if the menu doesn't
// open reliably for you. Every short press is held for up to this long
// before it's actioned (to see whether a second press follows), so this
// value also sets the latency added to ordinary page-cycle/menu-navigate
// presses.

void advanceActivePage();
void jumpToActivePage(Page page);
void updatePageButton();
