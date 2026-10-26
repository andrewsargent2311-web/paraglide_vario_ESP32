#include "ble_manager.h"
#include "EngineNmea.h"
#include <NimBLEDevice.h>
#include <Preferences.h>
#include <atomic>
#include <string>
#include <strings.h>

// =====================================================
// ENGINE-METER PROTOCOL: the Nordic UART Service TX characteristic sends
// ASCII NMEA-style EMS and PEMBAT sentences. MTU-sized notifications are
// treated as arbitrary stream fragments and reassembled into lines.
// =====================================================
static const char* NUS_SERVICE_UUID = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E";
static const char* NUS_CHAR_TX_UUID = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E";  // notify: peripheral -> us

// Seconds per scan burst while the Scan menu screen is open, and while
// hunting for a remembered device -- NimBLEScan::start() below is given
// a callback so this never blocks the caller.
#define BLE_SCAN_WINDOW_S 4
// How often bleManagerLoop() starts a fresh scan burst to look for the
// remembered device when we're not currently connected to it.
#define BLE_RECONNECT_RETRY_MS 15000UL
static const char* NVS_NAMESPACE = "conn";  // shared with wifi_manager.cpp
static const char* NVS_KEY_BLE_ADDR = "bleAddr";
static const char* NVS_KEY_BLE_NAME = "bleName";
static const char* NVS_KEY_BLE_ON = "bleOn";  // pilot's Bluetooth on/off choice

std::atomic<bool> bleEnabled(false);
std::atomic<bool> bleConnected(false);
char bleRememberedName[BLE_DEVICE_NAME_MAX_LEN] = "";
char bleRememberedAddress[18] = "";

static SemaphoreHandle_t bleStateMutex = nullptr;
static SemaphoreHandle_t engineDataMutex = nullptr;
static SemaphoreHandle_t bleDeviceMutex = nullptr;

static BleScanResult scanResults[BLE_MAX_SCAN_RESULTS];
static uint8_t scanResultCount = 0;
static std::atomic<bool> scanWanted(false);  // true while the Bluetooth Scan menu screen is open

static NimBLEClient* pClient = nullptr;

// A match against the remembered device is recorded here (NOT as a
// pointer into the scan's own device cache -- that cache gets cleared by
// onScanComplete() right after the burst ends, so anything we want to
// survive past the burst has to be copied out into plain fields while
// still inside onResult()).
static std::atomic<bool> connectPending(false);
// Set by onScanComplete() once the scan that found the saved device has
// fully stopped. bleManagerLoop() then does the (blocking) connect on its
// own task -- never on the NimBLE host task, which would deadlock.
static std::atomic<bool> connectScanDone(false);
static char pendingConnectAddress[18] = "";
static uint8_t pendingConnectAddressType = 0;
static char pendingConnectName[BLE_DEVICE_NAME_MAX_LEN] = "";
static uint32_t pendingConnectGeneration = 0;
static std::atomic<uint32_t> connectGeneration(0);
static std::atomic<bool> connectInProgress(false);

static constexpr size_t kEngineFifoCapacity = 256U;
static constexpr size_t kLegacyPacketCapacity = 10U;
static constexpr TickType_t kEngineDataMutexTimeout = pdMS_TO_TICKS(20);
static uint8_t engineNmeaFifo[kEngineFifoCapacity];
static size_t engineNmeaHead = 0U;
static size_t engineNmeaTail = 0U;
static size_t engineNmeaCount = 0U;
static uint8_t legacyEnginePacket[kLegacyPacketCapacity];
static size_t legacyEnginePacketLen = 0U;
static bool legacyEnginePacketIsNew = false;
static std::atomic<uint32_t> engineNotificationDrops(0U);
static EngineNmeaLineAssembler engineLineAssembler;
static EngineNmeaCounters engineNmeaCounters = {};

static unsigned long lastReconnectAttempt = 0;

static void startScanBurst();
static void onScanComplete(const NimBLEScanResults& results, int reason);
static bool tryConnect(NimBLEAddress address, const char* name, uint32_t generation);
static bool rememberDevice(const char* address, const char* name, uint32_t generation);
static void decodeEnginePacket(const uint8_t* buf, size_t len);
static void handleEngineNmeaLine(const char* line, size_t len, void* context);
static void printEngineDebug();

struct EngineDataBatch {
  uint8_t nmea[kEngineFifoCapacity];
  size_t nmeaLen;
  uint8_t legacy[kLegacyPacketCapacity];
  size_t legacyLen;
  bool legacyIsNew;
};

[[nodiscard]] static bool takeEngineData(EngineDataBatch& batch);

// =====================================================
// DECODED ENGINE TELEMETRY -- see ble_manager.h for field docs. Written
// only from bleManagerLoop() (main/UI core), so no separate mutex is
// needed; notification bytes are passed over the mutex-protected FIFO.
// =====================================================
#define ENGINE_DATA_TIMEOUT_MS 2000UL
#define ENGINE_PKT_HEADER 0xAE
#define ENGINE_PKT_LEN_LEGACY 9
#define ENGINE_PKT_LEN_WITH_BATTERY 10

volatile bool engineDataValid = false;
volatile float engineRpm = 0.0f;
volatile float engineEgtC = 0.0f;
volatile float engineChtC = 0.0f;
volatile uint8_t engineBatteryPercent = 0;
volatile bool engineBatteryPercentValid = false;
volatile bool engineEgtFault = false;
volatile bool engineChtFault = false;

static unsigned long lastEngineDataMillis = 0;

// =====================================================
// SCAN CALLBACKS: runs on the BLE stack's own task. NimBLEScanCallbacks::
// onResult() fires once per device per scan result, AFTER scan-response
// data (if the device sends any -- which is where a lot of peripherals,
// especially small nRF52 boards, put their name to leave room for a
// service UUID in the primary advertising packet) has been merged in.
// That merge is exactly the step the classic Bluedroid-based BLE library
// has a long-standing open bug against, which is why this port is here.
// =====================================================
class VarioScanCallbacks : public NimBLEScanCallbacks {
  // NimBLE-Arduino 2.x reports "scan finished" ONLY through this callback.
  void onScanEnd(const NimBLEScanResults& results, int reason) override {
    onScanComplete(results, reason);
  }

  void onResult(const NimBLEAdvertisedDevice* advertisedDevice) override {
    std::string addr = advertisedDevice->getAddress().toString();
    std::string name = advertisedDevice->haveName()
                         ? advertisedDevice->getName() : std::string();
    int rssi = advertisedDevice->getRSSI();
    uint8_t addrType = advertisedDevice->getAddress().getType();

    if (bleStateMutex != nullptr && xSemaphoreTake(bleStateMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
      int slot = -1;
      for (uint8_t i = 0; i < scanResultCount; i++) {
        if (strcasecmp(addr.c_str(), scanResults[i].address) == 0) {
          slot = i;
          break;
        }
      }
      if (slot < 0 && scanResultCount < BLE_MAX_SCAN_RESULTS) {
        slot = scanResultCount++;
      }
      if (slot >= 0) {
        strncpy(scanResults[slot].address, addr.c_str(), sizeof(scanResults[slot].address) - 1);
        scanResults[slot].address[sizeof(scanResults[slot].address) - 1] = '\0';
        scanResults[slot].addressType = addrType;
        // Only replace the stored name if this result actually contains one --
        // kept as a defensive no-op belt-and-braces even though NimBLE's
        // onResult() should already have the merged/complete name whenever
        // the device sent one at all.
        if (!name.empty()) {
          strncpy(scanResults[slot].name, name.c_str(), BLE_DEVICE_NAME_MAX_LEN - 1);
          scanResults[slot].name[BLE_DEVICE_NAME_MAX_LEN - 1] = '\0';
        }
        scanResults[slot].rssi = rssi;
      }
      xSemaphoreGive(bleStateMutex);
    }

    // Auto-reconnect match -- only while not already connected/mid-attempt.
    // Copy everything we need out to plain fields now: the scan's own
    // device cache doesn't survive past onScanComplete()'s clearResults().
    if (!bleConnected && !connectPending.load() &&
        (bleRememberedAddress[0] != '\0' || bleRememberedName[0] != '\0')) {
      bool addressMatch = bleRememberedAddress[0] != '\0' &&
                          strcasecmp(addr.c_str(), bleRememberedAddress) == 0;
      bool nameMatch = !addressMatch && bleRememberedName[0] != '\0' &&
                       name == bleRememberedName;
      if (addressMatch || nameMatch) {
        strncpy(pendingConnectAddress, addr.c_str(), sizeof(pendingConnectAddress) - 1);
        pendingConnectAddress[sizeof(pendingConnectAddress) - 1] = '\0';
        pendingConnectAddressType = addrType;
        strncpy(pendingConnectName, name.c_str(), BLE_DEVICE_NAME_MAX_LEN - 1);
        pendingConnectName[BLE_DEVICE_NAME_MAX_LEN - 1] = '\0';
        pendingConnectGeneration = connectGeneration.load();
        connectPending.store(true);
        Serial.printf("[BLE] Found saved engine meter: %s (%s)\n",
                      name.length() ? name.c_str() : "unnamed", addr.c_str());
        NimBLEDevice::getScan()->stop();  // onScanComplete() does the actual connect once this settles
      }
    }
  }
};
static VarioScanCallbacks scanCallbacks;

// =====================================================
// NOTIFY CALLBACK: runs on the BLE stack task and only copies bytes into
// the legacy slot or fixed NMEA FIFO under a bounded mutex wait.
// =====================================================
static void engineNotifyCallback(NimBLERemoteCharacteristic* pChar, uint8_t* pData, size_t length, bool isNotify) {
  (void)pChar;
  (void)isNotify;
  if (engineDataMutex == nullptr || pData == nullptr || length == 0U) {
    engineNotificationDrops.fetch_add(1U);
    return;
  }
  if (xSemaphoreTake(engineDataMutex, kEngineDataMutexTimeout) != pdTRUE) {
    engineNotificationDrops.fetch_add(1U);
    return;
  }

  // TRANSITIONAL: remove once all engine meters run the NMEA firmware
  if ((length == ENGINE_PKT_LEN_LEGACY || length == ENGINE_PKT_LEN_WITH_BATTERY) &&
      pData[0] == ENGINE_PKT_HEADER) {
    memcpy(legacyEnginePacket, pData, length);
    legacyEnginePacketLen = length;
    legacyEnginePacketIsNew = true;
  } else if (length <= kEngineFifoCapacity - engineNmeaCount) {
    for (size_t i = 0U; i < length; ++i) {
      engineNmeaFifo[engineNmeaTail] = pData[i];
      engineNmeaTail = (engineNmeaTail + 1U) % kEngineFifoCapacity;
    }
    engineNmeaCount += length;
  } else {
    engineNotificationDrops.fetch_add(1U);
  }

  if (xSemaphoreGive(engineDataMutex) != pdTRUE) {
    engineNotificationDrops.fetch_add(1U);
  }
}

// =====================================================
// CLIENT CALLBACK: keeps bleConnected honest, so a drop gets picked up
// by bleManagerLoop()'s reconnect timer instead of the menu silently
// still claiming to be connected.
// =====================================================
class VarioClientCallbacks : public NimBLEClientCallbacks {
  void onConnect(NimBLEClient* client) override { (void)client; }
  void onDisconnect(NimBLEClient* client, int reason) override {
    Serial.printf("[BLE] Disconnected, reason=%d\n", reason);
    bleConnected = false;
  }
};
static VarioClientCallbacks clientCallbacks;

// =====================================================
// SETUP / PERSISTENCE
// =====================================================
void loadBleSettings() {
  if (bleStateMutex == nullptr) bleStateMutex = xSemaphoreCreateMutex();
  if (engineDataMutex == nullptr) engineDataMutex = xSemaphoreCreateMutex();
  if (bleDeviceMutex == nullptr) bleDeviceMutex = xSemaphoreCreateMutex();

  Preferences prefs;
  prefs.begin(NVS_NAMESPACE, /*readOnly=*/true);
  String addr = prefs.getString(NVS_KEY_BLE_ADDR, "");
  String name = prefs.getString(NVS_KEY_BLE_NAME, "");
  // Pilot's saved on/off choice. If it has never been saved (older firmware),
  // fall back to the old behaviour: on only when a device is remembered.
  const bool bleOnSaved = prefs.isKey(NVS_KEY_BLE_ON);
  const bool bleOnValue = prefs.getBool(NVS_KEY_BLE_ON, false);
  prefs.end();

  strncpy(bleRememberedAddress, addr.c_str(), sizeof(bleRememberedAddress) - 1);
  bleRememberedAddress[sizeof(bleRememberedAddress) - 1] = '\0';
  strncpy(bleRememberedName, name.c_str(), BLE_DEVICE_NAME_MAX_LEN - 1);
  bleRememberedName[BLE_DEVICE_NAME_MAX_LEN - 1] = '\0';

  NimBLEDevice::init("ParaVario");
  if (!NimBLEDevice::setMTU(247U)) {
    Serial.println("[BLE] Could not set preferred MTU to 247");
  }

  const bool haveRemembered =
    (bleRememberedAddress[0] != '\0' || bleRememberedName[0] != '\0');
  const bool startEnabled = bleOnSaved ? bleOnValue : haveRemembered;
  if (startEnabled) {
    Serial.println("[BLE] Bluetooth was left on -- starting it to look for the remembered device");
    bleSetEnabled(true, /*persist=*/false);  // already what's saved (or legacy default)
  } else if (haveRemembered) {
    Serial.println("[BLE] Bluetooth was left off -- not auto-connecting");
  }
}

// =====================================================
// SCANNING
// =====================================================
static void startScanBurst() {
  if (!bleEnabled) return;

  NimBLEScan* pScan = NimBLEDevice::getScan();
  const bool dense = scanWanted.load();
  pScan->setScanCallbacks(&scanCallbacks, dense);
  pScan->setActiveScan(true);
  pScan->setInterval(dense ? 100 : 160);
  pScan->setWindow(dense ? 99 : 40);
  pScan->start(BLE_SCAN_WINDOW_S * 1000, /*isContinue=*/false, /*restart=*/true);  // ms; completion arrives via onScanEnd()
}

static void onScanComplete(const NimBLEScanResults& results, int reason) {
  NimBLEDevice::getScan()->clearResults();  // free the stack's own cache -- we keep our own list in scanResults[]

  if (connectPending.load()) {
    connectScanDone.store(true);  // bleManagerLoop() performs the connect
    return;  // don't immediately restart scanning right after a match
  }

  if (bleEnabled && !bleConnected &&
      (bleRememberedAddress[0] != '\0' || bleRememberedName[0] != '\0') &&
      !scanWanted) {
    Serial.println("[BLE] Saved engine meter not found in this scan; will retry");
  }

  if (scanWanted) {
    startScanBurst();  // keep the Scan screen's list refreshing
  }
}

void bleStartScan() {
  if (!bleEnabled) return;

  if (bleStateMutex != nullptr && xSemaphoreTake(bleStateMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    scanResultCount = 0;  // fresh list each time the Scan screen is opened
    xSemaphoreGive(bleStateMutex);
  }

  scanWanted = true;
  startScanBurst();
}

void bleStopScan() {
  scanWanted = false;
  NimBLEDevice::getScan()->stop();
}

uint8_t bleScanResultCount() {
  uint8_t n = 0;
  if (bleStateMutex != nullptr && xSemaphoreTake(bleStateMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    n = scanResultCount;
    xSemaphoreGive(bleStateMutex);
  }
  return n;
}

BleScanResult bleScanResultAt(uint8_t index) {
  BleScanResult r = {};
  if (bleStateMutex != nullptr && xSemaphoreTake(bleStateMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    if (index < scanResultCount) r = scanResults[index];
    xSemaphoreGive(bleStateMutex);
  }
  return r;
}

// =====================================================
// CONNECTING
// =====================================================
static bool rememberDevice(const char* address, const char* name, uint32_t generation) {
  if (bleDeviceMutex == nullptr ||
      xSemaphoreTake(bleDeviceMutex, pdMS_TO_TICKS(100)) != pdTRUE) {
    Serial.println("[BLE] Could not lock saved-device settings");
    return false;
  }

  if (!bleEnabled || generation != connectGeneration.load()) {
    xSemaphoreGive(bleDeviceMutex);
    return false;
  }

  Preferences prefs;
  if (!prefs.begin(NVS_NAMESPACE, /*readOnly=*/false)) {
    xSemaphoreGive(bleDeviceMutex);
    Serial.println("[BLE] Could not open saved-device settings");
    return false;
  }
  size_t addressBytes = prefs.putString(NVS_KEY_BLE_ADDR, address);
  size_t nameBytes = 0;
  if (name[0] != '\0') {
    nameBytes = prefs.putString(NVS_KEY_BLE_NAME, name);
  }
  if (addressBytes == 0 || name[0] == '\0' || nameBytes == 0) {
    prefs.remove(NVS_KEY_BLE_NAME);
  }
  prefs.end();
  if (addressBytes == 0) {
    xSemaphoreGive(bleDeviceMutex);
    Serial.println("[BLE] Could not save engine-meter address");
    return false;
  }

  strncpy(bleRememberedAddress, address, sizeof(bleRememberedAddress) - 1);
  bleRememberedAddress[sizeof(bleRememberedAddress) - 1] = '\0';
  const char* savedName = nameBytes > 0 ? name : "";
  strncpy(bleRememberedName, savedName, BLE_DEVICE_NAME_MAX_LEN - 1);
  bleRememberedName[BLE_DEVICE_NAME_MAX_LEN - 1] = '\0';
  xSemaphoreGive(bleDeviceMutex);
  if (name[0] != '\0' && nameBytes == 0) {
    Serial.println("[BLE] Saved engine-meter address, but could not save its name");
  }
  return true;
}

// Returns true if the connection AND the NUS TX subscription both succeeded.
static bool tryConnect(NimBLEAddress address, const char* name, uint32_t generation) {
  bool expected = false;
  if (!connectInProgress.compare_exchange_strong(expected, true)) {
    Serial.println("[BLE] Connection attempt already in progress");
    return false;
  }

  struct ConnectAttemptGuard {
    ~ConnectAttemptGuard() { connectInProgress.store(false); }
  } connectAttemptGuard;

  if (!bleEnabled || generation != connectGeneration.load()) {
    Serial.println("[BLE] Connection attempt cancelled before connecting");
    return false;
  }

  if (pClient == nullptr) {
    pClient = NimBLEDevice::createClient();
    if (pClient == nullptr) {
      Serial.println("[BLE] Failed to create BLE client");
      return false;
    }
    pClient->setClientCallbacks(&clientCallbacks, /*deleteOnDisconnect=*/false);
    pClient->setConnectTimeout(5000);
  }
  if (pClient->isConnected()) {
    pClient->disconnect();
  }

  Serial.printf("[BLE] Connecting to %s (%s, type=%d)...\n", name, address.toString().c_str(), address.getType());

  // This is the fix for the "click and hold does nothing" problem: NimBLEAddress
  // carries the public/random type discovered during scanning, so this connect
  // call uses the correct type instead of silently assuming public like the
  // classic library's BLEAddress did.
  if (!pClient->connect(address)) {
    Serial.println("[BLE] Connect failed");
    return false;
  }
  if (!bleEnabled || generation != connectGeneration.load()) {
    Serial.println("[BLE] Connection cancelled while connecting");
    pClient->disconnect();
    return false;
  }

  NimBLERemoteService* pService = pClient->getService(NUS_SERVICE_UUID);
  if (pService == nullptr) {
    Serial.println("[BLE] Connected, but the NUS service wasn't found -- check "
                    "NUS_SERVICE_UUID in ble_manager.cpp against your nRF52840 firmware");
    pClient->disconnect();
    return false;
  }
  if (!bleEnabled || generation != connectGeneration.load()) {
    Serial.println("[BLE] Connection cancelled during service discovery");
    pClient->disconnect();
    return false;
  }

  NimBLERemoteCharacteristic* pChar = pService->getCharacteristic(NUS_CHAR_TX_UUID);
  if (pChar == nullptr || !pChar->canNotify()) {
    Serial.println("[BLE] NUS TX characteristic missing or doesn't support notify -- "
                    "check NUS_CHAR_TX_UUID in ble_manager.cpp");
    pClient->disconnect();
    return false;
  }
  if (!pChar->subscribe(true, engineNotifyCallback)) {
    Serial.println("[BLE] Failed to subscribe to engine data notifications");
    pClient->disconnect();
    return false;
  }
  if (!bleEnabled || generation != connectGeneration.load()) {
    Serial.println("[BLE] Connection cancelled during notification setup");
    pClient->disconnect();
    return false;
  }

  if (!rememberDevice(address.toString().c_str(), name, generation)) {
    Serial.println("[BLE] Connected, but could not save engine-meter settings");
    pClient->disconnect();
    return false;
  }
  if (!bleEnabled || generation != connectGeneration.load()) {
    Serial.println("[BLE] Connection cancelled while saving device settings");
    pClient->disconnect();
    return false;
  }
  bleConnected = true;
  Serial.printf("[BLE] Negotiated MTU: %u\n", static_cast<unsigned>(pClient->getMTU()));
  Serial.println("[BLE] Connected and subscribed to engine data");
  return true;
}

bool bleConnectToScanResult(uint8_t index) {
  BleScanResult r = bleScanResultAt(index);
  if (r.address[0] == '\0') return false;

  uint32_t generation = connectGeneration.fetch_add(1) + 1;
  connectPending.store(false);
  connectScanDone.store(false);
  bleStopScan();
  return tryConnect(NimBLEAddress(std::string(r.address), r.addressType), r.name,
                    generation);
}

void bleForgetDevice() {
  connectGeneration.fetch_add(1);
  connectPending.store(false);
  connectScanDone.store(false);
  bleStopScan();
  if (pClient != nullptr && pClient->isConnected()) {
    pClient->disconnect();
  }
  bleConnected = false;

  if (bleDeviceMutex == nullptr ||
      xSemaphoreTake(bleDeviceMutex, pdMS_TO_TICKS(100)) != pdTRUE) {
    Serial.println("[BLE] Could not lock saved-device settings to forget device");
    return;
  }
  bleRememberedAddress[0] = '\0';
  bleRememberedName[0] = '\0';
  Preferences prefs;
  if (prefs.begin(NVS_NAMESPACE, /*readOnly=*/false)) {
    prefs.remove(NVS_KEY_BLE_ADDR);
    prefs.remove(NVS_KEY_BLE_NAME);
    prefs.end();
  } else {
    Serial.println("[BLE] Could not open saved-device settings to forget device");
  }
  xSemaphoreGive(bleDeviceMutex);
}

// =====================================================
// ON / OFF + BACKGROUND RECONNECT
// =====================================================
void bleSetEnabled(bool enabled, bool persist) {
  bleEnabled = enabled;

  if (persist) {
    Preferences prefs;
    if (prefs.begin(NVS_NAMESPACE, /*readOnly=*/false)) {
      if (prefs.putBool(NVS_KEY_BLE_ON, enabled) == 0U) {
        Serial.println("[BLE] Could not save Bluetooth on/off setting");
      }
      prefs.end();
    } else {
      Serial.println("[BLE] Could not open settings to save Bluetooth on/off");
    }
  }

  if (!enabled) {
    connectGeneration.fetch_add(1);
    connectPending.store(false);
    connectScanDone.store(false);
    bleStopScan();
    if (pClient != nullptr && pClient->isConnected()) {
      pClient->disconnect();
    }
    bleConnected = false;
  } else {
    // Kick off a scan burst right away, so an auto-reconnect (or the
    // pilot opening the Scan screen right after turning this on) doesn't
    // have to wait for the next bleManagerLoop() tick.
    startScanBurst();
  }
}

void bleManagerLoop() {
  EngineDataBatch batch = {};
  if (takeEngineData(batch)) {
    if (batch.legacyIsNew) decodeEnginePacket(batch.legacy, batch.legacyLen);
    if (batch.nmeaLen > 0U) {
      engineLineAssembler.feed(batch.nmea, batch.nmeaLen, handleEngineNmeaLine, nullptr);
    }
    if (batch.legacyIsNew || batch.nmeaLen > 0U) printEngineDebug();
  }
  if (engineDataValid && millis() - lastEngineDataMillis > ENGINE_DATA_TIMEOUT_MS) {
    engineDataValid = false;  // engine meter went quiet -- out of range, off, or disconnected
    engineBatteryPercentValid = false;
  }

  if (connectPending.load() && connectScanDone.load()) {
    connectScanDone.store(false);
    connectPending.store(false);
    lastReconnectAttempt = millis();
    if (!bleEnabled || bleConnected ||
        (bleRememberedAddress[0] == '\0' && bleRememberedName[0] == '\0') ||
        pendingConnectGeneration != connectGeneration.load()) {
      Serial.println("[BLE] Pending engine-meter reconnect cancelled");
    } else {
      NimBLEAddress addr(std::string(pendingConnectAddress), pendingConnectAddressType);
      tryConnect(addr, pendingConnectName, pendingConnectGeneration);
    }
    return;
  }

  if (!bleEnabled || bleConnected || connectPending.load()) return;
  if (bleRememberedAddress[0] == '\0' && bleRememberedName[0] == '\0') return;

  unsigned long now = millis();
  if (now - lastReconnectAttempt < BLE_RECONNECT_RETRY_MS) return;

  lastReconnectAttempt = now;
  startScanBurst();  // VarioScanCallbacks::onResult() triggers the actual connect on a match
}

// =====================================================
// TRANSITIONAL LEGACY PACKET DECODE -- supports old meter firmware only.
// New meter firmware uses the NMEA parser in EngineNmea.cpp.
// =====================================================
static void decodeEnginePacket(const uint8_t* buf, size_t len) {
  if ((len != ENGINE_PKT_LEN_LEGACY && len != ENGINE_PKT_LEN_WITH_BATTERY) ||
      buf == nullptr || buf[0] != ENGINE_PKT_HEADER) {
    return;
  }

  uint8_t chk = 0;
  for (size_t i = 0; i < len - 1; i++) chk ^= buf[i];
  if (chk != buf[len - 1]) {
    return;
  }

  uint16_t rpmRaw = (uint16_t)buf[1] | ((uint16_t)buf[2] << 8);
  int16_t egtRaw = (int16_t)((uint16_t)buf[3] | ((uint16_t)buf[4] << 8));
  int16_t chtRaw = (int16_t)((uint16_t)buf[5] | ((uint16_t)buf[6] << 8));
  uint8_t flags = buf[7];

  engineRpm = (float)rpmRaw;
  engineEgtC = egtRaw / 10.0f;
  engineChtC = chtRaw / 10.0f;
  engineBatteryPercentValid =
      len == ENGINE_PKT_LEN_WITH_BATTERY && buf[8] <= 100;
  if (engineBatteryPercentValid) {
    engineBatteryPercent = buf[8];
  }
  engineEgtFault = (flags & 0x01) != 0;
  engineChtFault = (flags & 0x02) != 0;
  engineDataValid = true;
  lastEngineDataMillis = millis();
}

[[nodiscard]] static bool takeEngineData(EngineDataBatch& batch) {
  if (engineDataMutex == nullptr ||
      xSemaphoreTake(engineDataMutex, kEngineDataMutexTimeout) != pdTRUE) {
    return false;
  }

  batch.nmeaLen = engineNmeaCount;
  for (size_t i = 0U; i < batch.nmeaLen; ++i) {
    batch.nmea[i] = engineNmeaFifo[engineNmeaHead];
    engineNmeaHead = (engineNmeaHead + 1U) % kEngineFifoCapacity;
  }
  engineNmeaCount = 0U;
  engineNmeaTail = engineNmeaHead;
  batch.legacyLen = legacyEnginePacketLen;
  batch.legacyIsNew = legacyEnginePacketIsNew;
  if (legacyEnginePacketIsNew) {
    memcpy(batch.legacy, legacyEnginePacket, legacyEnginePacketLen);
  }
  legacyEnginePacketLen = 0U;
  legacyEnginePacketIsNew = false;

  if (xSemaphoreGive(engineDataMutex) != pdTRUE) {
    engineNotificationDrops.fetch_add(1U);
    return false;
  }
  return true;
}

static void handleEngineNmeaLine(const char* line, size_t len, void* context) {
  (void)context;
  EngineReading reading = {};
  if (!parseEngineSentence(line, len, reading, engineNmeaCounters)) return;

  if (reading.rpmUpdated) engineRpm = reading.rpm;
  if (reading.egtUpdated) {
    engineEgtC = reading.egtC;
    engineEgtFault = reading.egtFault;
  }
  if (reading.chtUpdated) {
    engineChtC = reading.chtC;
    engineChtFault = reading.chtFault;
  }
  if (reading.batteryUpdated) {
    engineBatteryPercent = reading.batteryPercent;
    engineBatteryPercentValid = true;
  }
  if (reading.rpmUpdated || reading.egtUpdated || reading.chtUpdated) {
    engineDataValid = true;
    lastEngineDataMillis = millis();
  }
}

static void printEngineDebug() {
  static unsigned long lastDebugMillis = 0U;
  const unsigned long now = millis();
  if (now - lastDebugMillis < 1000UL) return;
  lastDebugMillis = now;
  const uint32_t rejected = engineNmeaCounters.checksumRejected +
                            engineNmeaCounters.malformedRejected +
                            engineLineAssembler.overflowCount() +
                            engineNotificationDrops.load();
  Serial.printf("[ENG] rpm=%.1f cht=%.1f egt=%.1f faults=%d/%d battery=%u%% valid=%d reject=%lu\n",
                engineRpm, engineChtC, engineEgtC, engineChtFault ? 1 : 0,
                engineEgtFault ? 1 : 0, static_cast<unsigned>(engineBatteryPercent),
                engineDataValid ? 1 : 0,
                static_cast<unsigned long>(rejected));
}
