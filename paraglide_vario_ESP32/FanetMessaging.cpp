#include "FanetMessaging.h"
#include "settings.h"  // fanetMessagingEnabled
#include "menu.h"      // playFeedbackTone(), displayDirty
#include <string.h>

// ---------------------------------------------------------------------------
// Preset messages
// ---------------------------------------------------------------------------
// Starter set covering common paragliding radio calls. No way to edit
// this from the device itself (no keyboard) -- change this array and
// reflash for different wording. Keep each one comfortably under
// FANET_MAX_MESSAGE_LEN (40) chars; all of these are well within that.
const char* const FANET_MESSAGE_PRESETS[] = {
  "Landing now",
  "Landed safely",
  "Thermal here",
  "Good lift here",
  "Sink here, be careful",
  "Heading to LZ",
  "All OK",
  "Need assistance"
};

const uint8_t FANET_MESSAGE_PRESET_COUNT =
  sizeof(FANET_MESSAGE_PRESETS) / sizeof(FANET_MESSAGE_PRESETS[0]);

// ---------------------------------------------------------------------------
// Sending
// ---------------------------------------------------------------------------

bool sendFanetMessagePreset(uint8_t index) {
  if (index >= FANET_MESSAGE_PRESET_COUNT) return false;
  if (!fanetMessagingEnabled) return false;
  if (!fanetRadioOK) return false;

  return fanet.sendMessageNow(FANET_MESSAGE_PRESETS[index]);
}

// ---------------------------------------------------------------------------
// Receiving
// ---------------------------------------------------------------------------

FanetMessageEvent lastFanetMessage = { false, { 0, 0 }, "", 0 };

void onFanetMessageReceived(const FanetAddress& src, const FanetMessage& msg, float rssi, float snr) {
  if (!fanetMessagingEnabled) return;

  Serial.printf("[FANET MSG] from %02X:%04X  \"%s\"  rssi=%.0f snr=%.1f\n",
                src.manufacturer, src.id, msg.text, rssi, snr);

  lastFanetMessage.valid = true;
  lastFanetMessage.src = src;
  strncpy(lastFanetMessage.text, msg.text, sizeof(lastFanetMessage.text) - 1);
  lastFanetMessage.text[sizeof(lastFanetMessage.text) - 1] = '\0';
  lastFanetMessage.receivedMs = millis();

  // One-shot alert -- a different pitch/pattern from the airspace entry
  // tone (900Hz/600ms) so the two are distinguishable by ear.
  playFeedbackTone(1300.0f, 150);

  displayDirty = true;  // draw the banner immediately rather than waiting for the next 1Hz tick
}
