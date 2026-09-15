#include "ble_manager.h"
#include <NimBLEDevice.h>
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
// hunting for a remembered device -- NimBLEScan::start() below is given
// a callback so this never blocks the caller.
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

static NimBLEClient* pClient = nullptr;

// A match against the remembered device is recorded here (NOT as a
// pointer into the scan's own device cache -- that cache gets cleared by
// onScanComplete() right after the burst ends, so anything we want to
// survive past the burst has to be copied out into plain fields while
// still inside onResult()).
static volatile bool connectPending = false;
static char pendingConnectAddress[18] = "";
static uint8_t pendingConnectAddressType = 0;
static char pendingConnectName[BLE_DEVICE_NAME_MAX_LEN] = "";

static uint8_t engineDataBuf[BLE_ENGINE_DATA_MAX_LEN];
static size_t engineDataLen = 0;
static bool engineDataIsNew = false;

static unsigned long lastReconnectAttempt = 0;

static void startScanBurst();
static void onScanComplete(const NimBLEScanResults& results, int reason);
static bool tryConnect(NimBLEAddress address, const char* name);
static void rememberDevice(const char* address, const char* name);
static void decodeEnginePacket(const uint8_t* buf, size_t len);

// =====================================================
// DECODED ENGINE TELEMETRY -- see ble_manager.h for field docs. Written
// only from decodeEnginePacket(), called from bleManagerLoop() (main/UI
// core), so no separate mutex needed the way engineDataBuf's raw bytes
// have one -- there's no BLE-stack-task writer to race against here.
// =====================================================
#define ENGINE_DATA_TIMEOUT_MS 2000UL
#define ENGINE_PKT_HEADER 0xAE
#define ENGINE_PKT_LEN 9

volatile bool engineDataValid = false;
volatile float engineRpm = 0.0f;
volatile float engineEgtC = 0.0f;
volatile float engineChtC = 0.0f;
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
  void onResult(const NimBLEAdvertisedDevice* advertisedDevice) override {
    String addr = advertisedDevice->getAddress().toString().c_str();
    String name = advertisedDevice->haveName() ? advertisedDevice->getName().c_str() : "";
    int rssi = advertisedDevice->getRSSI();
    uint8_t addrType = advertisedDevice->getAddress().getType();

    Serial.printf("[BLE SCAN] Address: %s (type=%d) | Name: %s | RSSI: %d\n",
              addr.c_str(), addrType, name.length() ? name.c_str() : "(none)", rssi);
    Serial.printf("[BLE SCAN] hasName=%d hasServiceUUID=%d hasManufacturerData=%d\n",
              advertisedDevice->haveName(),
              advertisedDevice->haveServiceUUID(),
              advertisedDevice->haveManufacturerData());

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
        scanResults[slot].addressType = addrType;
        // Only replace the stored name if this result actually contains one --
        // kept as a defensive no-op belt-and-braces even though NimBLE's
        // onResult() should already have the merged/complete name whenever
        // the device sent one at all.
        if (name.length() > 0) {
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
    if (!bleConnected && !connectPending && bleRememberedAddress[0] != '\0') {
      bool addressMatch = addr.equalsIgnoreCase(bleRememberedAddress);
      bool nameMatch = !addressMatch && bleRememberedName[0] != '\0' && name.equals(bleRememberedName);
      if (addressMatch || nameMatch) {
        strncpy(pendingConnectAddress, addr.c_str(), sizeof(pendingConnectAddress) - 1);
        pendingConnectAddress[sizeof(pendingConnectAddress) - 1] = '\0';
        pendingConnectAddressType = addrType;
        strncpy(pendingConnectName, name.c_str(), BLE_DEVICE_NAME_MAX_LEN - 1);
        pendingConnectName[BLE_DEVICE_NAME_MAX_LEN - 1] = '\0';
        connectPending = true;
        NimBLEDevice::getScan()->stop();  // onScanComplete() does the actual connect once this settles
      }
    }
  }
};
static VarioScanCallbacks scanCallbacks;

// =====================================================
// NOTIFY CALLBACK: fires on the BLE stack's task whenever the engine
// meter's TX characteristic notifies. Just stashes the raw bytes --
// decoding them into actual RPM/EGT/etc. fields is a job for whichever
// page ends up displaying them, once the data format is settled.
// =====================================================
static void engineNotifyCallback(NimBLERemoteCharacteristic* pChar, uint8_t* pData, size_t length, bool isNotify) {
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
class VarioClientCallbacks : public NimBLEClientCallbacks {
  void onConnect(NimBLEClient* client) override {
    bleConnected = true;
  }
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

  Preferences prefs;
  prefs.begin(NVS_NAMESPACE, /*readOnly=*/true);
  String addr = prefs.getString(NVS_KEY_BLE_ADDR, "");
  String name = prefs.getString(NVS_KEY_BLE_NAME, "");
  prefs.end();

  strncpy(bleRememberedAddress, addr.c_str(), sizeof(bleRememberedAddress) - 1);
  bleRememberedAddress[sizeof(bleRememberedAddress) - 1] = '\0';
  strncpy(bleRememberedName, name.c_str(), BLE_DEVICE_NAME_MAX_LEN - 1);
  bleRememberedName[BLE_DEVICE_NAME_MAX_LEN - 1] = '\0';

  NimBLEDevice::init("ParaVario");

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

  NimBLEScan* pScan = NimBLEDevice::getScan();
  pScan->setScanCallbacks(&scanCallbacks, /*wantDuplicates=*/true);
  pScan->setActiveScan(true);
  pScan->setInterval(100);
  pScan->setWindow(99);
  pScan->start(BLE_SCAN_WINDOW_S * 1000, onScanComplete, false);  // NimBLEScan::start() takes ms, not seconds
}

static void onScanComplete(const NimBLEScanResults& results, int reason) {
  NimBLEDevice::getScan()->clearResults();  // free the stack's own cache -- we keep our own list in scanResults[]

  if (connectPending) {
    connectPending = false;
    NimBLEAddress addr(std::string(pendingConnectAddress), pendingConnectAddressType);
    tryConnect(addr, pendingConnectName);
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

// Returns true if the connection AND the NUS TX subscription both succeeded.
static bool tryConnect(NimBLEAddress address, const char* name) {
  if (pClient == nullptr) {
    pClient = NimBLEDevice::createClient();
    pClient->setClientCallbacks(&clientCallbacks, /*deleteOnDisconnect=*/false);
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

  NimBLERemoteService* pService = pClient->getService(NUS_SERVICE_UUID);
  if (pService == nullptr) {
    Serial.println("[BLE] Connected, but the NUS service wasn't found -- check "
                    "NUS_SERVICE_UUID in ble_manager.cpp against your nRF52840 firmware");
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
  pChar->subscribe(true, engineNotifyCallback);

  rememberDevice(address.toString().c_str(), name);
  Serial.println("[BLE] Connected and subscribed to engine data");
  return true;
}

bool bleConnectToScanResult(uint8_t index) {
  BleScanResult r = bleScanResultAt(index);
  if (r.address[0] == '\0') return false;

  bleStopScan();
  return tryConnect(NimBLEAddress(std::string(r.address), r.addressType), r.name);
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
  // ---- Engine telemetry: decode any new notification, and time out a
  // stale reading, regardless of connect/reconnect state below. ----
  uint8_t rawBuf[BLE_ENGINE_DATA_MAX_LEN];
  bool isNew = false;
  size_t n = bleReadLatestEngineData(rawBuf, sizeof(rawBuf), &isNew);
  if (isNew) {
    decodeEnginePacket(rawBuf, n);
  }
  if (engineDataValid && millis() - lastEngineDataMillis > ENGINE_DATA_TIMEOUT_MS) {
    engineDataValid = false;  // engine meter went quiet -- out of range, off, or disconnected
  }

  if (!bleEnabled || bleConnected || connectPending) return;
  if (bleRememberedAddress[0] == '\0') return;  // nothing to look for

  unsigned long now = millis();
  if (now - lastReconnectAttempt < BLE_RECONNECT_RETRY_MS) return;

  lastReconnectAttempt = now;
  startScanBurst();  // VarioScanCallbacks::onResult() triggers the actual connect on a match
}

// =====================================================
// ENGINE PACKET DECODE -- matches the nice!nano engine meter's fixed
// 9-byte layout (see sendEnginePacket() in the engine meter's firmware):
//   byte 0    : 0xAE header
//   byte 1-2  : uint16_t RPM, little-endian
//   byte 3-4  : int16_t  EGT, little-endian, 0.1 degC units
//   byte 5-6  : int16_t  CHT, little-endian, 0.1 degC units
//   byte 7    : flags (bit0 = EGT fault, bit1 = CHT fault)
//   byte 8    : checksum = XOR of bytes 0..7
// Silently drops anything that doesn't match length/header/checksum --
// a garbled or torn notification just gets skipped rather than shown.
// =====================================================
static void decodeEnginePacket(const uint8_t* buf, size_t len) {
  if (len != ENGINE_PKT_LEN || buf[0] != ENGINE_PKT_HEADER) return;

  uint8_t chk = 0;
  for (int i = 0; i < 8; i++) chk ^= buf[i];
  if (chk != buf[8]) {
    Serial.println("[BLE] Engine packet checksum mismatch -- dropped");
    return;
  }

  uint16_t rpmRaw = (uint16_t)buf[1] | ((uint16_t)buf[2] << 8);
  int16_t egtRaw = (int16_t)((uint16_t)buf[3] | ((uint16_t)buf[4] << 8));
  int16_t chtRaw = (int16_t)((uint16_t)buf[5] | ((uint16_t)buf[6] << 8));
  uint8_t flags = buf[7];

  engineRpm = (float)rpmRaw;
  engineEgtC = egtRaw / 10.0f;
  engineChtC = chtRaw / 10.0f;
  engineEgtFault = (flags & 0x01) != 0;
  engineChtFault = (flags & 0x02) != 0;

  engineDataValid = true;
  lastEngineDataMillis = millis();
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
