#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

#include <Arduino.h>

// =====================================================
// WI-FI CONNECTION MANAGER
// Owns which saved network (secrets.h's WIFI_NETWORKS) is active,
// persists that choice to flash (NVS, via the Preferences library) so it
// survives a reboot, and owns the connect/retry state machine that used
// to live directly in the main .ino's setup()/backgroundTask().
// =====================================================

// True whenever WiFi.status() == WL_CONNECTED as of the last check --
// read by backgroundTask() to gate ADS-B/weather polling, exactly like
// the old global did.
extern volatile bool wifiConnected;

// Index into secrets.h's WIFI_NETWORKS of the network currently selected
// from the menu (persisted -- see loadWifiSettings()). Only ever change
// this via selectWifiNetwork(), never assign it directly, so the flash
// write and the reconnect attempt always happen together.
extern uint8_t selectedWifiIndex;

// Call once from setup(), before startWifiConnect(). Reads
// selectedWifiIndex back from flash (defaults to 0 -- the first slot --
// if nothing has been saved yet, e.g. first boot ever).
void loadWifiSettings();

// Non-blocking. Kicks off WiFi.begin() for whichever network is
// currently selected and returns immediately -- does NOT wait to find
// out whether the connection succeeds. wifiManagerLoop() (already
// polled continuously from backgroundTask() on Core 0) picks up the
// moment WiFi.status() flips to WL_CONNECTED and takes over retry duty
// from there. Safe to call from setup() without stalling boot -- this
// replaces the old connectSavedWifi(), which blocked setup() for up to
// WIFI_CONNECT_TIMEOUT_MS (10s) waiting for the connection.
void startWifiConnect();

// Non-blocking. Call this from backgroundTask()'s existing poll loop
// (same place the old inline retry block lived) -- reconnects a dropped
// connection using whatever's currently selected, with the same
// WIFI_RETRY_MS/MAX_WIFI_RETRIES backoff the old code had.
void wifiManagerLoop();

// Call from backgroundTask() on Core 0 after each ADS-B fetch. Repeated
// failures while joined to the network restart the connection, because
// wifiManagerLoop() only notices when the network itself is lost.
void wifiReportFetchResult(bool ok);

// Called by the menu (Connections > WiFi) when the pilot picks a
// different saved network. No-ops on an empty slot. Persists the new
// index to flash, then disconnects and kicks off a fresh (non-blocking)
// connect attempt -- wifiConnected will reflect the result once it lands.
// If the radio is currently off (wifiEnabled, settings.h), this also
// turns it back on -- see setWifiRadioEnabled() below.
void selectWifiNetwork(uint8_t index);

// Turns the WiFi radio itself fully on or off, keeping wifiConnected and
// the retry backoff consistent with reality -- this is what
// Connections > WiFi's On/Off toggle calls (menu.cpp already includes
// this header directly, no indirection through the main .ino needed).
// OFF disconnects immediately and puts the radio to sleep;
// wifiManagerLoop()'s retry logic checks wifiEnabled too, so it won't
// silently undo an OFF state on the next tick. ON resumes with a fresh
// (non-blocking) connect attempt via startWifiConnect(). Also called
// automatically by selectWifiNetwork() if the radio was off when a
// specific network gets picked, since that's a strong enough signal of
// intent to turn it back on.
void setWifiRadioEnabled(bool enabled);

// =====================================================
// NETWORKS SAVED FROM THE DEVICE (Connections > WiFi > Add Network)
// On top of the fixed list compiled in from secrets.h, up to
// WIFI_SAVED_MAX extra networks can be added from a phone through the
// setup portal (WifiSetup.h) -- stored in flash (NVS), so they survive
// reboots AND firmware updates, no reflash needed to add one.
//
// One combined "slot" numbering covers both lists, so selectedWifiIndex
// and the existing NVS key keep working unchanged:
//   0 .. WIFI_NETWORK_COUNT-1                       -> secrets.h entries
//   WIFI_NETWORK_COUNT .. WIFI_TOTAL_SLOTS-1        -> device-saved entries
// =====================================================
#define WIFI_SAVED_MAX 5
#define WIFI_SSID_MAX_LEN 32      // 802.11 limit
#define WIFI_PASSWORD_MAX_LEN 64  // WPA2 passphrase limit (63) + spare

uint8_t wifiTotalSlots();
// True if the slot holds a usable network (non-empty SSID).
bool wifiSlotValid(uint8_t slot);
// SSID for a slot, or "" if the slot is empty/out of range. Never null.
const char* wifiSlotSsid(uint8_t slot);
// True for slots added from the device (and so forgettable from the device).
bool wifiSlotIsDeviceSaved(uint8_t slot);

// Stores a network added through the setup portal. If that SSID is
// already device-saved its password is updated in place; otherwise the
// first free slot is used, or -- once all WIFI_SAVED_MAX are used -- the
// oldest one is replaced. Returns the combined slot index, or -1 if the
// SSID/password is invalid. Does NOT connect -- call selectWifiNetwork()
// with the returned slot for that.
int addSavedWifiNetwork(const char* ssid, const char* password);

// Removes a device-saved network (combined slot index). If it was the
// selected one, falls back to the first remaining network (if any).
// No-op for secrets.h slots, which can only be changed by reflashing.
void forgetSavedWifiNetwork(uint8_t slot);

#endif  // WIFI_MANAGER_H
