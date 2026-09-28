#include "PageButton.h"

Page currentPage = PAGE_PARAGLIDER;
const char* PAGE_NAMES[PAGE_COUNT] = { "GLDR", "WIND", "ADSB", "ENG" };
// Only 3 pages are cycled through with a short press. Slot 0 is the "main"
// page and is swappable between Paraglider and Paramotor from the menu;
// slots 1 and 2 are fixed at Weather and ADS-B. (ACTIVE_PAGE_COUNT is
// defined in DrawPages.h, shared with drawTopBar()'s page-dots indicator.)
Page activePages[ACTIVE_PAGE_COUNT] = { PAGE_PARAGLIDER, PAGE_WEATHER, PAGE_ADSB };
uint8_t activePageIndex = 0;  // index into activePages[]; kept in sync with currentPage

void advanceActivePage() {
  activePageIndex = (activePageIndex + 1) % ACTIVE_PAGE_COUNT;
  currentPage = activePages[activePageIndex];
  displayDirty = true;
  playFeedbackTone(PAGE_BEEP_FREQ, PAGE_BEEP_MS);
}
// Jumps straight to a page if it's currently one of the 3 active slots
// (used by the ADS-B intercept alert to force-switch to the traffic page),
// keeping activePageIndex in sync so short-press cycling continues
// correctly afterward. Does nothing if the page isn't currently active.
void jumpToActivePage(Page page) {
  for (uint8_t i = 0; i < ACTIVE_PAGE_COUNT; i++) {
    if (activePages[i] == page) {
      activePageIndex = i;
      currentPage = page;
      displayDirty = true;
      return;
    }
  }
}
// =====================================================
// PAGE BUTTON: drives page cycling, the vario mute hold, and the on-screen
// menu (double press to open; short press to navigate; 2s hold to select).
// See the gesture summary in the KEY BUTTON section above.
// =====================================================
void updatePageButton() {
  static bool lastReading = HIGH;
  static bool stableState = HIGH;
  static unsigned long lastDebounceTime = 0;
  static unsigned long pressStartedAt = 0;
  static unsigned long lastReleaseAt = 0;
  static bool longPressHandled = false;
  static bool awaitingSecondPress = false;  // true after a short release, until the double-press window closes

  unsigned long now = millis();
  bool reading = digitalRead(KEY_PIN);
  if (reading != lastReading) {
    lastDebounceTime = now;
  }

  if (now - lastDebounceTime > KEY_DEBOUNCE_MS) {
    if (reading != stableState) {
      stableState = reading;

      if (stableState == LOW) {  // active-low press
        if (awaitingSecondPress && (now - lastReleaseAt) < MENU_DOUBLE_PRESS_MS) {
          // Second press landed inside the double-press window.
          awaitingSecondPress = false;
          longPressHandled = true;  // this press's own release does nothing
          if (!menuActive) {
            openMenu();
          } else {
            menuGoBack();  // step back one menu level, or close if already at the top
          }
        } else {
          longPressHandled = false;
        }
        pressStartedAt = now;
      } else if (!longPressHandled) {  // released after a short press
        // Could be a lone short press, or the first half of a double
        // press -- don't act yet, wait out the double-press window
        // in case another press follows.
        lastReleaseAt = now;
        awaitingSecondPress = true;
      }
    }
  }

  // Double-press window closed with no second press: resolve the
  // pending release as an ordinary short press.
  if (awaitingSecondPress && stableState == HIGH && now - lastReleaseAt >= MENU_DOUBLE_PRESS_MS) {
    awaitingSecondPress = false;
    if (menuActive) {
      menuMoveDown();
    } else {
      advanceActivePage();
    }
  }

  // Long-press handling while the button is still held down.
  if (stableState == LOW && !longPressHandled) {
    unsigned long heldFor = now - pressStartedAt;
    if (menuActive) {
      if (heldFor >= MENU_SELECT_HOLD_MS) {
        longPressHandled = true;
        awaitingSecondPress = false;
        menuSelectCurrentItem();
      }
    } else if (heldFor >= KEY_LONG_PRESS_MS) {
      longPressHandled = true;
      awaitingSecondPress = false;
      buzzerMuted = !buzzerMuted;
      saveSettings();
      pageBeepUntil = 0;
      setToneFrequency(0);
      beepOn = false;

      // Kick off the confirmation jingle -- actually sequenced
      // non-blockingly in updateI2sAudioBuzzer(). buzzerMuted already
      // holds the *new* state here, which is exactly the direction we
      // want to play.
      muteToneActive = true;
      muteToneStart = millis();
      muteToneIsMuteSequence = buzzerMuted;

      if (buzzerMuted) {
        // Muting: leave the amp powered through the jingle so it's
        // actually audible -- it gets switched off only once the jingle
        // finishes, in updateI2sAudioBuzzer().
      } else {
        // Unmuting: turn the amp on immediately so the jingle is audible
        // right away.
        digitalWrite(AMP_ENABLE_PIN, HIGH);
      }
      Serial.println(buzzerMuted ? "VARIO BUZZER MUTED" : "VARIO BUZZER UNMUTED");
    }
  }

  lastReading = reading;
}
