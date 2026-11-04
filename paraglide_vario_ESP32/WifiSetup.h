#ifndef WIFI_SETUP_H
#define WIFI_SETUP_H

#include <Arduino.h>

// =====================================================
// WiFi SETUP PORTAL  (Connections > WiFi > Add Network)
// -----------------------------------------------------
// Lets the pilot add a WiFi network from a phone, with no laptop or
// reflash needed -- the same way a smart plug is set up:
//
//   1. Pick "Add Network" in the menu.
//   2. The vario scans for nearby networks, then turns itself into a
//      temporary open hotspot called WIFI_SETUP_AP_NAME.
//   3. Join that hotspot on the phone. Most phones pop up the setup page
//      by themselves (captive portal); if not, browse to
//      http://192.168.4.1/
//   4. Tap a network (or type a hidden one), enter the password, Save.
//   5. The network is stored in flash (addSavedWifiNetwork(),
//      wifi_manager.h), the hotspot closes, and the vario connects to it.
//
// Runs in its own small Core 0 task while active, so page loads stay
// quick (backgroundTask() only loops every 500ms) and Core 1 is never
// touched. Normal WiFi (ADS-B, weather, Export Files) is paused while
// the portal is up, and resumes afterwards -- on the new network if one
// was saved, otherwise on whatever was selected before.
//
// Closes itself after WIFI_SETUP_TIMEOUT_MS of no saved network, so an
// open hotspot can never be left running by accident.
// =====================================================

#define WIFI_SETUP_AP_NAME "Vario-Setup"
#define WIFI_SETUP_TIMEOUT_MS (10UL * 60UL * 1000UL)

enum WifiSetupState {
  WIFI_SETUP_IDLE = 0,     // not running
  WIFI_SETUP_STARTING,     // scanning, then bringing the hotspot up
  WIFI_SETUP_WAITING,      // hotspot up, no phone joined yet
  WIFI_SETUP_PHONE_JOINED, // a phone is connected to the hotspot
  WIFI_SETUP_SAVED,        // network saved -- closing, then connecting
  WIFI_SETUP_FAILED        // hotspot couldn't start (see Serial)
};

// Starts the portal. Safe to call while already running (no-op).
// Stops Export Files first if it's running (both use port 80).
void startWifiSetup();

// Closes the portal without saving anything and resumes normal WiFi.
// Safe to call when not running.
void stopWifiSetup();

// True from startWifiSetup() until the portal has fully shut down --
// wifiManagerLoop() stands aside while this is true.
bool isWifiSetupActive();

WifiSetupState wifiSetupState();

// SSID of the network saved in this session (for the menu's status
// line), or "" if none yet.
const char* wifiSetupSavedSsid();

#endif  // WIFI_SETUP_H
