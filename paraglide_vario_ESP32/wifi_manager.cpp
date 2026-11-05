#include "wifi_manager.h"
#include "secrets.h"
#include "settings.h"  // wifiEnabled, saveSettings()
#include "WifiSetup.h"  // isWifiSetupActive()
#include <WiFi.h>
#include <Preferences.h>
#include <string.h>

// How often wifiManagerLoop() retries a dropped/never-established
// connection, and how many times it'll retry before giving up until the
// next reboot -- unchanged from the values that used to live directly in
// the main .ino.
#define WIFI_RETRY_MS 50000UL
#define MAX_WIFI_RETRIES 10
// After MAX_WIFI_RETRIES quick attempts, keep trying at this slower pace
// instead of switching WiFi off until reboot: a phone hotspot that was off or
// out of reach for a while must not cost the pilot ADS-B and weather for the
// rest of the flight.
#define WIFI_SLOW_RETRY_MS 300000UL

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

// Device-saved networks (Connections > WiFi > Add Network -- see
// WifiSetup.h). Own NVS namespace so they can't collide with the
// "conn" keys above. Keys: s0..s4 = SSID, p0..p4 = password, "next" =
// which slot gets replaced when all WIFI_SAVED_MAX are full.
static const char* NVS_SAVED_NAMESPACE = "wifisav";

#define WIFI_TOTAL_SLOTS (WIFI_NETWORK_COUNT + WIFI_SAVED_MAX)

static char savedSsid[WIFI_SAVED_MAX][WIFI_SSID_MAX_LEN + 1];
static char savedPass[WIFI_SAVED_MAX][WIFI_PASSWORD_MAX_LEN + 1];
static uint8_t savedNextReplace = 0;

uint8_t wifiTotalSlots() {
  return WIFI_TOTAL_SLOTS;
}

bool wifiSlotIsDeviceSaved(uint8_t slot) {
  return slot >= WIFI_NETWORK_COUNT && slot < WIFI_TOTAL_SLOTS;
}

const char* wifiSlotSsid(uint8_t slot) {
  if (slot < WIFI_NETWORK_COUNT) {
    const char* ssid = WIFI_NETWORKS[slot].ssid;
    return (ssid != nullptr) ? ssid : "";
  }
  if (slot < WIFI_TOTAL_SLOTS) {
    return savedSsid[slot - WIFI_NETWORK_COUNT];
  }
  return "";
}

static const char* wifiSlotPassword(uint8_t slot) {
  if (slot < WIFI_NETWORK_COUNT) {
    const char* pass = WIFI_NETWORKS[slot].password;
    return (pass != nullptr) ? pass : "";
  }
  if (slot < WIFI_TOTAL_SLOTS) {
    return savedPass[slot - WIFI_NETWORK_COUNT];
  }
  return "";
}

bool wifiSlotValid(uint8_t slot) {
  return wifiSlotSsid(slot)[0] != '\0';
}

// Kept under its old name so the rest of this file reads unchanged.
static bool networkSlotValid(uint8_t index) {
  return wifiSlotValid(index);
}

static void beginConnect(uint8_t index) {
  if (!networkSlotValid(index)) return;
  WiFi.mode(WIFI_STA);
  const char* pass = wifiSlotPassword(index);
  // Open networks: pass nullptr rather than "" so the core doesn't try WPA.
  WiFi.begin(wifiSlotSsid(index), pass[0] != '\0' ? pass : nullptr);
}

static void loadSavedNetworks() {
  Preferences prefs;
  prefs.begin(NVS_SAVED_NAMESPACE, /*readOnly=*/true);
  char key[4];
  for (uint8_t i = 0; i < WIFI_SAVED_MAX; i++) {
    snprintf(key, sizeof(key), "s%u", i);
    String ssid = prefs.getString(key, "");
    snprintf(key, sizeof(key), "p%u", i);
    String pass = prefs.getString(key, "");
    strlcpy(savedSsid[i], ssid.c_str(), sizeof(savedSsid[i]));
    strlcpy(savedPass[i], pass.c_str(), sizeof(savedPass[i]));
  }
  savedNextReplace = prefs.getUChar("next", 0) % WIFI_SAVED_MAX;
  prefs.end();
}

static void storeSavedNetwork(uint8_t i) {
  Preferences prefs;
  prefs.begin(NVS_SAVED_NAMESPACE, /*readOnly=*/false);
  char key[4];
  snprintf(key, sizeof(key), "s%u", i);
  prefs.putString(key, savedSsid[i]);
  snprintf(key, sizeof(key), "p%u", i);
  prefs.putString(key, savedPass[i]);
  prefs.putUChar("next", savedNextReplace);
  prefs.end();
}

int addSavedWifiNetwork(const char* ssid, const char* password) {
  if (ssid == nullptr || password == nullptr) return -1;
  size_t ssidLen = strlen(ssid);
  size_t passLen = strlen(password);
  if (ssidLen == 0 || ssidLen > WIFI_SSID_MAX_LEN) return -1;
  // WPA2 needs 8-63 characters; empty means an open network.
  if (passLen != 0 && (passLen < 8 || passLen > 63)) return -1;

  int target = -1;
  for (uint8_t i = 0; i < WIFI_SAVED_MAX; i++) {    // same SSID -> update it
    if (strcmp(savedSsid[i], ssid) == 0) { target = i; break; }
  }
  if (target < 0) {
    for (uint8_t i = 0; i < WIFI_SAVED_MAX; i++) {  // else first free slot
      if (savedSsid[i][0] == '\0') { target = i; break; }
    }
  }
  if (target < 0) {                                  // else replace oldest
    target = savedNextReplace;
    savedNextReplace = (savedNextReplace + 1) % WIFI_SAVED_MAX;
  }

  strlcpy(savedSsid[target], ssid, sizeof(savedSsid[target]));
  strlcpy(savedPass[target], password, sizeof(savedPass[target]));
  storeSavedNetwork((uint8_t)target);

  Serial.printf("[WIFI] Saved network '%s' in device slot %d\n", ssid, target);
  return WIFI_NETWORK_COUNT + target;
}

void forgetSavedWifiNetwork(uint8_t slot) {
  if (!wifiSlotIsDeviceSaved(slot)) return;
  uint8_t i = slot - WIFI_NETWORK_COUNT;
  Serial.printf("[WIFI] Forgetting saved network '%s'\n", savedSsid[i]);
  savedSsid[i][0] = '\0';
  savedPass[i][0] = '\0';
  storeSavedNetwork(i);

  if (slot != selectedWifiIndex) return;

  // Forgot the network in use -- move to the first one left, if any.
  for (uint8_t s = 0; s < WIFI_TOTAL_SLOTS; s++) {
    if (networkSlotValid(s)) {
      selectWifiNetwork(s);
      return;
    }
  }
  WiFi.disconnect();
  wifiConnected = false;
}

void loadWifiSettings() {
  loadSavedNetworks();

  Preferences prefs;
  prefs.begin(NVS_NAMESPACE, /*readOnly=*/true);
  uint8_t saved = prefs.getUChar(NVS_KEY_WIFI_INDEX, 0);
  prefs.end();

  // Falls back to slot 0 if the saved index points at an empty slot --
  // e.g. it was cleared out of secrets.h since it was last selected.
  selectedWifiIndex = networkSlotValid(saved) ? saved : 0;
}

void startWifiConnect() {
  if (!wifiEnabled) {
    Serial.println("[WIFI] Radio disabled (Connections > WiFi) -- skipping connect");
    return;
  }

  if (!networkSlotValid(selectedWifiIndex)) {
    Serial.println("[WIFI] Selected network slot is empty -- skipping connect");
    return;
  }

  beginConnect(selectedWifiIndex);

  // Stops wifiManagerLoop() from firing an immediate extra retry on top
  // of the attempt we just started -- give this one WIFI_RETRY_MS to
  // land before the retry backoff considers it stale.
  lastWifiRetry = millis();
}

// Checks WiFi.status() every call (cheap) so a fresh connect after
// selectWifiNetwork() is picked up on the very next tick, instead of the
// old code's up-to-50-second detection lag -- only the actual
// re-associate attempt (WiFi.begin()) is throttled to WIFI_RETRY_MS.
void wifiManagerLoop() {
  // The setup portal owns the radio (access-point mode) while it runs.
  if (isWifiSetupActive()) return;
  if (!wifiEnabled || wifiDisabledUntilReboot) return;

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
  const unsigned long retryIntervalMs =
      (wifiRetryCount >= MAX_WIFI_RETRIES) ? WIFI_SLOW_RETRY_MS : WIFI_RETRY_MS;
  if (now - lastWifiRetry < retryIntervalMs) return;

  lastWifiRetry = now;
  if (wifiRetryCount < 255) wifiRetryCount++;

  WiFi.disconnect();
  beginConnect(selectedWifiIndex);

  if (wifiRetryCount <= MAX_WIFI_RETRIES) {
    Serial.printf("WIFI RETRY %u/%u\n", wifiRetryCount, MAX_WIFI_RETRIES);
  } else {
    Serial.println("WIFI RETRY (slow)");
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

  if (!wifiEnabled) {
    // Picking a specific network is a strong enough signal of intent
    // that we turn the radio back on for them here, rather than
    // silently doing nothing -- setWifiRadioEnabled() also keeps
    // Connections > WiFi's On/Off toggle in sync, since it's the same
    // wifiEnabled flag the toggle reads/writes (settings.h).
    setWifiRadioEnabled(true);
    return;  // setWifiRadioEnabled(true) already calls startWifiConnect()
  }

  WiFi.disconnect();
  beginConnect(index);
}

void setWifiRadioEnabled(bool enabled) {
  wifiEnabled = enabled;
  saveSettings();

  if (enabled) {
    // A fresh explicit enable overrides a prior retry-exhaustion
    // lockout too -- the pilot turning it back on is a clean slate.
    wifiDisabledUntilReboot = false;
    wifiRetryCount = 0;
    startWifiConnect();
  } else {
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    wifiConnected = false;
  }
}
