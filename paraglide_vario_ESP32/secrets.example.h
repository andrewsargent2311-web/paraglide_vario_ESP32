// secrets.example.h
// -----------------------------------------------------------------------------
// 1. Copy this file and rename the copy to  secrets.h
//    (keep it in the same folder as paraglide_vario_ESP32.ino)
// 2. Fill in your WiFi network name(s) and password(s) below.
//
// secrets.h is listed in .gitignore, so your passwords will NOT be uploaded
// to GitHub. Never rename this example file and commit real passwords in it.
//
// Notes:
//  - 2.4 GHz WiFi only (the ESP32 cannot use 5 GHz).
//  - Names and passwords are case-sensitive.
//  - Leave unused slots as "" (empty quotes).
//  - Pick which saved network to use from the device menu:
//    Menu > Connections > WiFi.
// -----------------------------------------------------------------------------
#pragma once

struct WifiNetwork {
  const char* ssid;
  const char* password;
};

// How many network slots you list below.
#define WIFI_NETWORK_COUNT 3

static const WifiNetwork WIFI_NETWORKS[WIFI_NETWORK_COUNT] = {
  { "MyHomeWiFi",     "my-home-password" },
  { "MyPhoneHotspot", "hotspot-password" },
  { "",               "" },
};
