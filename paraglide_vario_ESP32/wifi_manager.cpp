#include "wifi_manager.h"
#include "secrets.h"
#include <WiFi.h>
#include <Preferences.h>

// How often wifiManagerLoop() retries a dropped/never-established
// connection, and how many times it'll retry before giving up until the
// next reboot -- unchanged from the values that used to live directly in
// the main .ino.
#define WIFI_RETRY_MS 50000UL
#define MAX_WIFI_RETRIES 10

volatile bool wifiConnected = false;
uint8_t selectedWifiIndex = 0;

static unsigned long lastWifiRetry = 0;
static uint8_t wifiRetryCount = 0;
static bool wifiDisabledUntilReboot = false;

// NVS namespace shared with ble_manager.cpp's remembered-device keys --
// "conn" for "connections", so both live under one small namespace
// instead of two.
static const char* NVS_NAMESPACE = "conn";
static const char* NVS_KEY_WIFI_INDEX = "wifiIdx";

static bool networkSlotValid(uint8_t index) {
  return index < WIFI_NETWORK_COUNT
         && WIFI_NETWORKS[index].ssid != nullptr
         && WIFI_NETWORKS[index].ssid[0] != '\0';
}

static void beginConnect(uint8_t index) {
  if (!networkSlotValid(index)) return;
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_NETWORKS[index].ssid, WIFI_NETWORKS[index].password);
}

void loadWifiSettings() {
  Preferences prefs;
  prefs.begin(NVS_NAMESPACE, /*readOnly=*/true);
  uint8_t saved = prefs.getUChar(NVS_KEY_WIFI_INDEX, 0);
  prefs.end();

  // Falls back to slot 0 if the saved index points at an empty slot --
  // e.g. it was cleared out of secrets.h since it was last selected.
  selectedWifiIndex = networkSlotValid(saved) ? saved : 0;
}

bool connectSavedWifi(unsigned long timeoutMs) {
  if (!networkSlotValid(selectedWifiIndex)) {
    Serial.println("[WIFI] Selected network slot is empty -- skipping connect");
    return false;
  }

  beginConnect(selectedWifiIndex);

  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < timeoutMs) {
    delay(200);  // only used here, during the bounded setup() attempt
  }

  wifiConnected = (WiFi.status() == WL_CONNECTED);
  return wifiConnected;
}

// Checks WiFi.status() every call (cheap) so a fresh connect after
// selectWifiNetwork() is picked up on the very next tick, instead of the
// old code's up-to-50-second detection lag -- only the actual
// re-associate attempt (WiFi.begin()) is throttled to WIFI_RETRY_MS.
void wifiManagerLoop() {
  if (wifiDisabledUntilReboot) return;

  bool nowConnected = (WiFi.status() == WL_CONNECTED);

  if (nowConnected && !wifiConnected) {
    wifiConnected = true;
    wifiRetryCount = 0;
    Serial.println("WIFI CONNECTED");
  } else if (!nowConnected && wifiConnected) {
    wifiConnected = false;
    Serial.println("WIFI DROPPED");
  }

  if (wifiConnected || !networkSlotValid(selectedWifiIndex)) return;

  unsigned long now = millis();
  if (wifiRetryCount >= MAX_WIFI_RETRIES || now - lastWifiRetry < WIFI_RETRY_MS) return;

  lastWifiRetry = now;
  wifiRetryCount++;

  WiFi.disconnect();
  beginConnect(selectedWifiIndex);

  Serial.printf("WIFI RETRY %u/%u\n", wifiRetryCount, MAX_WIFI_RETRIES);

  if (wifiRetryCount >= MAX_WIFI_RETRIES) {
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    wifiDisabledUntilReboot = true;
    Serial.println("WIFI DISABLED UNTIL REBOOT");
  }
}

void selectWifiNetwork(uint8_t index) {
  if (!networkSlotValid(index)) return;

  selectedWifiIndex = index;

  Preferences prefs;
  prefs.begin(NVS_NAMESPACE, /*readOnly=*/false);
  prefs.putUChar(NVS_KEY_WIFI_INDEX, index);
  prefs.end();

  wifiConnected = false;
  wifiRetryCount = 0;
  wifiDisabledUntilReboot = false;
  lastWifiRetry = millis();

  WiFi.disconnect();
  beginConnect(index);
}
