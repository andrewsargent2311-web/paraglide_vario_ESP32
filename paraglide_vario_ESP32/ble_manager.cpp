#include "ble_manager.h"
#include <BLEDevice.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <BLEClient.h>
#include <BLEUtils.h>
#include <Preferences.h>

// =====================================================
// ASSUMED ENGINE-METER PROTOCOL -- see the note at the top of
// ble_manager.h. This is the Nordic UART Service (NUS): a de facto
// standard "just stream bytes" pattern many nRF5 projects use, NOT
// something this vario invented. NUS_CHAR_TX_UUID is the characteristic
// the nRF52840 (as a GATT *server*) notifies on -- i.e. what we, as the
// central, subscribe to in order to receive engine data.
//
// If the real firmware differs, these two lines are the only thing that
// needs to change for the connection itself to work -- update them to
// match, then adjust how bleReadLatestEngineData()'s bytes get decoded
// wherever you display them.
// =====================================================
static const char* NUS_SERVICE_UUID = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E";
static const char* NUS_CHAR_TX_UUID = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E";  // notify: peripheral -> us

// Seconds per scan burst while the Scan menu screen is open, and while
// hunting for a remembered device -- BLEScan::start() below is given a
// callback so this never blocks the caller.
#define BLE_SCAN_WINDOW_S 4
// How often bleManagerLoop() starts a fresh scan burst to look for the
// remembered device when we're not currently connected to it.
#define BLE_RECONNECT_RETRY_MS 15000UL
// Max bytes kept from a single engine-meter notification. NUS payloads
// are capped by the BLE MTU anyway (typically <=20 bytes on a default
// MTU); 64 leaves headroom without the buffer being unreasonably large.
#define BLE_ENGINE_DATA_MAX_LEN 64

static const char* NVS_NAMESPACE = "conn";  // shared with wifi_manager.cpp
static const char* NVS_KEY_BLE_ADDR = "bleAddr";
static const char* NVS_KEY_BLE_NAME = "bleName";

bool bleEnabled = false;
volatile bool bleConnected = false;
char bleRememberedName[BLE_DEVICE_NAME_MAX_LEN] = "";
char bleRememberedAddress[18] = "";

static SemaphoreHandle_t bleStateMutex = nullptr;
static SemaphoreHandle_t engineDataMutex = nullptr;

static BleScanResult scanResults[BLE_MAX_SCAN_RESULTS];
static uint8_t scanResultCount = 0;
static bool scanWanted = false;  // true while the Bluetooth Scan menu screen is open

static BLEClient* pClient = nullptr;
static BLEAdvertisedDevice* pPendingConnectDevice = nullptr;
static volatile bool connectPending = false;

static uint8_t engineDataBuf[BLE_ENGINE_DATA_MAX_LEN];
static size_t engineDataLen = 0;
static bool engineDataIsNew = false;

static unsigned long lastReconnectAttempt = 0;

static void startScanBurst();
static void onScanComplete(BLEScanResults results);
static void tryConnect(BLEAddress address, const char* name);
static void rememberDevice(const char* address, const char* name);

// =====================================================
// ADVERTISED-DEVICE CALLBACK: runs on the BLE stack's own task, once per
// advertisement seen during a scan burst. Records/refreshes the device
// in our results list (for the Scan menu screen), and -- if it matches
// whatever device we're remembering -- flags it for auto-reconnect.
// =====================================================
class VarioAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) override {
    String addr = advertisedDevice.getAddress().toString().c_str();
    String name = advertisedDevice.haveName() ? advertisedDevice.getName().c_str() : "";
    int rssi = advertisedDevice.haveRSSI() ? advertisedDevice.getRSSI() : -127;

    if (bleStateMutex != nullptr && xSemaphoreTake(bleStateMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
      int slot = -1;
      for (uint8_t i = 0; i < scanResultCount; i++) {
        if (addr.equalsIgnoreCase(scanResults[i].address)) {
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
        strncpy(scanResults[slot].name, name.c_str(), BLE_DEVICE_NAME_MAX_LEN - 1);
        scanResults[slot].name[BLE_DEVICE_NAME_MAX_LEN - 1] = '\0';
        scanResults[slot].rssi = rssi;
      }
      xSemaphoreGive(bleStateMutex);
    }

    // Auto-reconnect match -- only while not already connected/mid-attempt.
    if (!bleConnected && !connectPending && bleRememberedAddress[0] != '\0') {
      bool addressMatch = addr.equalsIgnoreCase(bleRememberedAddress);
      bool nameMatch = !addressMatch && bleRememberedName[0] != '\0' && name.equals(bleRememberedName);
      if (addressMatch || nameMatch) {
        connectPending = true;
        if (pPendingConnectDevice != nullptr) {
          delete pPendingConnectDevice;
        }
        pPendingConnectDevice = new BLEAdvertisedDevice(advertisedDevice);
        BLEDevice::getScan()->stop();  // onScanComplete() does the actual connect once this settles
      }
    }
  }
};
static VarioAdvertisedDeviceCallbacks advertisedDeviceCallbacks;

// =====================================================
// NOTIFY CALLBACK: fires on the BLE stack's task whenever the engine
// meter's TX characteristic notifies. Just stashes the raw bytes --
// decoding them into actual RPM/EGT/etc. fields is a job for whichever
// page ends up displaying them, once the data format is settled.
// =====================================================
static void engineNotifyCallback(BLERemoteCharacteristic* pChar, uint8_t* pData, size_t length, bool isNotify) {
  if (engineDataMutex == nullptr || xSemaphoreTake(engineDataMutex, pdMS_TO_TICKS(20)) != pdTRUE) return;

  size_t n = length < sizeof(engineDataBuf) ? length : sizeof(engineDataBuf);
  memcpy(engineDataBuf, pData, n);
  engineDataLen = n;
  engineDataIsNew = true;

  xSemaphoreGive(engineDataMutex);
}

// =====================================================
// CLIENT CALLBACK: keeps bleConnected honest, so a drop gets picked up
// by bleManagerLoop()'s reconnect timer instead of the menu silently
// still claiming to be connected.
// =====================================================
class VarioClientCallbacks : public BLEClientCallbacks {
  void onConnect(BLEClient* client) override {
    bleConnected = true;
  }
  void onDisconnect(BLEClient* client) override {
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

  Preferences prefs;
  prefs.begin(NVS_NAMESPACE, /*readOnly=*/true);
  String addr = prefs.getString(NVS_KEY_BLE_ADDR, "");
  String name = prefs.getString(NVS_KEY_BLE_NAME, "");
  prefs.end();

  strncpy(bleRememberedAddress, addr.c_str(), sizeof(bleRememberedAddress) - 1);
  bleRememberedAddress[sizeof(bleRememberedAddress) - 1] = '\0';
  strncpy(bleRememberedName, name.c_str(), BLE_DEVICE_NAME_MAX_LEN - 1);
  bleRememberedName[BLE_DEVICE_NAME_MAX_LEN - 1] = '\0';

  BLEDevice::init("ParaVario");

  if (bleRememberedAddress[0] != '\0' || bleRememberedName[0] != '\0') {
    Serial.println("[BLE] Remembered device found in flash -- turning Bluetooth on to look for it");
    bleSetEnabled(true);
  }
}

// =====================================================
// SCANNING
// =====================================================
static void startScanBurst() {
  if (!bleEnabled) return;

  BLEScan* pScan = BLEDevice::getScan();
  pScan->setAdvertisedDeviceCallbacks(&advertisedDeviceCallbacks, /*wantDuplicates=*/true);
  pScan->setActiveScan(true);
  pScan->setInterval(100);
  pScan->setWindow(99);
  pScan->start(BLE_SCAN_WINDOW_S, onScanComplete, false);
}

static void onScanComplete(BLEScanResults results) {
  BLEDevice::getScan()->clearResults();  // free the stack's own cache -- we keep our own list in scanResults[]

  if (connectPending && pPendingConnectDevice != nullptr) {
    BLEAdvertisedDevice* dev = pPendingConnectDevice;
    pPendingConnectDevice = nullptr;

    tryConnect(dev->getAddress(), dev->haveName() ? dev->getName().c_str() : "");

    delete dev;
    connectPending = false;
    return;  // don't immediately restart scanning right after a connect attempt
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
  BLEDevice::getScan()->stop();
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
static void rememberDevice(const char* address, const char* name) {
  strncpy(bleRememberedAddress, address, sizeof(bleRememberedAddress) - 1);
  bleRememberedAddress[sizeof(bleRememberedAddress) - 1] = '\0';
  strncpy(bleRememberedName, name, BLE_DEVICE_NAME_MAX_LEN - 1);
  bleRememberedName[BLE_DEVICE_NAME_MAX_LEN - 1] = '\0';

  Preferences prefs;
  prefs.begin(NVS_NAMESPACE, /*readOnly=*/false);
  prefs.putString(NVS_KEY_BLE_ADDR, bleRememberedAddress);
  prefs.putString(NVS_KEY_BLE_NAME, bleRememberedName);
  prefs.end();
}

static void tryConnect(BLEAddress address, const char* name) {
  if (pClient == nullptr) {
    pClient = BLEDevice::createClient();
    pClient->setClientCallbacks(&clientCallbacks);
  }
  if (pClient->isConnected()) {
    pClient->disconnect();
  }

  Serial.printf("[BLE] Connecting to %s (%s)...\n", name, address.toString().c_str());

  if (!pClient->connect(address)) {
    Serial.println("[BLE] Connect failed");
    return;
  }

  BLERemoteService* pService = pClient->getService(NUS_SERVICE_UUID);
  if (pService == nullptr) {
    Serial.println("[BLE] Connected, but the NUS service wasn't found -- check "
                    "NUS_SERVICE_UUID in ble_manager.cpp against your nRF52840 firmware");
    return;
  }

  BLERemoteCharacteristic* pChar = pService->getCharacteristic(NUS_CHAR_TX_UUID);
  if (pChar == nullptr || !pChar->canNotify()) {
    Serial.println("[BLE] NUS TX characteristic missing or doesn't support notify -- "
                    "check NUS_CHAR_TX_UUID in ble_manager.cpp");
    return;
  }
  pChar->registerForNotify(engineNotifyCallback);

  rememberDevice(address.toString().c_str(), name);
  Serial.println("[BLE] Connected and subscribed to engine data");
}

void bleConnectToScanResult(uint8_t index) {
  BleScanResult r = bleScanResultAt(index);
  if (r.address[0] == '\0') return;

  bleStopScan();
  tryConnect(BLEAddress(String(r.address)), r.name);
}

void bleForgetDevice() {
  if (pClient != nullptr && pClient->isConnected()) {
    pClient->disconnect();
  }
  bleConnected = false;
  bleRememberedAddress[0] = '\0';
  bleRememberedName[0] = '\0';

  Preferences prefs;
  prefs.begin(NVS_NAMESPACE, /*readOnly=*/false);
  prefs.remove(NVS_KEY_BLE_ADDR);
  prefs.remove(NVS_KEY_BLE_NAME);
  prefs.end();
}

// =====================================================
// ON / OFF + BACKGROUND RECONNECT
// =====================================================
void bleSetEnabled(bool enabled) {
  bleEnabled = enabled;

  if (!enabled) {
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
  if (!bleEnabled || bleConnected || connectPending) return;
  if (bleRememberedAddress[0] == '\0') return;  // nothing to look for

  unsigned long now = millis();
  if (now - lastReconnectAttempt < BLE_RECONNECT_RETRY_MS) return;

  lastReconnectAttempt = now;
  startScanBurst();  // VarioAdvertisedDeviceCallbacks::onResult() triggers the actual connect on a match
}

// =====================================================
// ENGINE DATA READOUT
// =====================================================
size_t bleReadLatestEngineData(uint8_t* outBuf, size_t outBufLen, bool* outIsNew) {
  size_t n = 0;
  if (engineDataMutex == nullptr || xSemaphoreTake(engineDataMutex, pdMS_TO_TICKS(20)) != pdTRUE) {
    if (outIsNew) *outIsNew = false;
    return 0;
  }

  n = engineDataLen < outBufLen ? engineDataLen : outBufLen;
  memcpy(outBuf, engineDataBuf, n);
  if (outIsNew) *outIsNew = engineDataIsNew;
  engineDataIsNew = false;

  xSemaphoreGive(engineDataMutex);
  return n;
}
