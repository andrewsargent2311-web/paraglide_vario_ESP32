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

// Call once from setup(), before connectSavedWifi(). Reads
// selectedWifiIndex back from flash (defaults to 0 -- the first slot --
// if nothing has been saved yet, e.g. first boot ever).
void loadWifiSettings();

// Bounded blocking connect attempt using whichever network is currently
// selected -- same call site/semantics as the old connectWiFi(), for
// setup() to call once at boot. Returns false without blocking if the
// selected slot is empty.
bool connectSavedWifi(unsigned long timeoutMs);

// Non-blocking. Call this from backgroundTask()'s existing poll loop
// (same place the old inline retry block lived) -- reconnects a dropped
// connection using whatever's currently selected, with the same
// WIFI_RETRY_MS/MAX_WIFI_RETRIES backoff the old code had.
void wifiManagerLoop();

// Called by the menu (Connections > WiFi) when the pilot picks a
// different saved network. No-ops on an empty slot. Persists the new
// index to flash, then disconnects and kicks off a fresh (non-blocking)
// connect attempt -- wifiConnected will reflect the result once it lands.
void selectWifiNetwork(uint8_t index);

#endif  // WIFI_MANAGER_H
