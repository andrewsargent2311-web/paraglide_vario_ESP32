#ifndef FANET_MESSAGING_H
#define FANET_MESSAGING_H

#include <Arduino.h>
#include "Fanet.h"  // FanetStack, FanetAddress, FanetMessage, FANET_MAX_MESSAGE_LEN

// =====================================================
// FanetMessaging
// -----------------------------------------------------
// Application layer on top of FANET Message (type 3) packets -- see
// Fanet.h/.cpp for the protocol-level encode/decode (broadcast only;
// see Fanet.h's FanetMessage comment for why). This file owns:
//
//   - The preset message list. The hardware has a single button and no
//     keyboard, so sending is always "pick one of these", never
//     freeform text -- see FANET_MESSAGE_PRESETS below. There's no way
//     to edit the list from the device itself; change the array in
//     FanetMessaging.cpp and reflash if you want different wording.
//
//   - Receiving: onFanetMessageReceived() is registered with
//     fanet.onMessage() wherever FANET gets initialised (main .ino --
//     see setup() and setFanetEnabled()). It records the latest message
//     into lastFanetMessage so drawFanetMessageBanner() (DrawPages.cpp)
//     can show it, and plays a one-shot alert tone. Only the single
//     most recent message is kept -- no history, no persistence.
//
//   - Sending: sendFanetMessagePreset(), called from the menu (FANET
//     Messaging, menu.cpp).
//
// Everything here is gated on fanetMessagingEnabled (settings.h,
// persisted like the FANET/WiFi on-off toggles) -- OFF means
// sendFanetMessagePreset() always fails and onFanetMessageReceived()
// ignores incoming messages entirely (no banner, no tone), same as
// FANET itself being off.
//
// No mutex anywhere in here: onFanetMessageReceived() runs inside
// fanet.update(), and drawFanetMessageBanner() runs inside
// drawDashboard() -- both are only ever called from loop() on Core 1,
// so they can never run concurrently. Same reasoning as FanetContact in
// DrawPages.h.
// =====================================================

// -----------------------------------------------------
// Preset messages
// -----------------------------------------------------
extern const char* const FANET_MESSAGE_PRESETS[];
extern const uint8_t FANET_MESSAGE_PRESET_COUNT;

// Broadcasts FANET_MESSAGE_PRESETS[index]. Returns false (does nothing)
// if index is out of range, fanetMessagingEnabled is false, FANET
// itself isn't up (fanetRadioOK), or the underlying radio send fails
// (channel busy, TX already in flight -- see Sx126xLink::send()). A
// single attempt -- no queue, no automatic retry; the menu gives the
// pilot immediate tone feedback either way so they know whether to
// just press the button again.
bool sendFanetMessagePreset(uint8_t index);

// -----------------------------------------------------
// Receiving
// -----------------------------------------------------

// How long a received message's banner stays on screen after arriving
// -- see drawFanetMessageBanner(), DrawPages.cpp.
#define FANET_MESSAGE_BANNER_MS (10UL * 1000UL)

struct FanetMessageEvent {
  bool valid;
  FanetAddress src;
  char text[FANET_MAX_MESSAGE_LEN + 1];
  unsigned long receivedMs;
};

// The single most recently received message (if any) -- read directly
// by drawFanetMessageBanner() (DrawPages.cpp), no snapshot/getter
// needed; see the no-mutex-needed reasoning above.
extern FanetMessageEvent lastFanetMessage;

// Registered with fanet.onMessage() -- matches FanetMessageCallback's
// signature (Fanet.h).
void onFanetMessageReceived(const FanetAddress& src, const FanetMessage& msg, float rssi, float snr);

// -----------------------------------------------------
// .ino-owned globals this file needs (defined in the main .ino).
// -----------------------------------------------------
extern FanetStack fanet;
extern bool fanetRadioOK;

#endif  // FANET_MESSAGING_H
