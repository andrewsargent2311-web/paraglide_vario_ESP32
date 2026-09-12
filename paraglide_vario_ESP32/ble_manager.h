#ifndef BLE_MANAGER_H
#define BLE_MANAGER_H

#include <Arduino.h>

// =====================================================
// BLUETOOTH (BLE CENTRAL) MANAGER
//
// This board (ESP32-S3) only has a Bluetooth LOW ENERGY radio -- there is
// no Classic Bluetooth (BR/EDR) on the S3. That means this can scan for
// and connect to other BLE peripherals (like a BLE-based engine meter),
// but it can never see or pair with phones/headsets/etc. using Classic
// BT profiles the way a phone's Bluetooth settings screen does.
//
// PROTOCOL ASSUMPTION: built around the nRF52840 engine meter advertising
// the Nordic UART Service (NUS) and notifying raw bytes on its TX
// characteristic -- see NUS_SERVICE_UUID/NUS_CHAR_TX_UUID at the top of
// ble_manager.cpp. That's the common simple pattern for a custom nRF52
// peripheral, but if your engine meter's firmware actually uses different
// UUIDs (or a GATT layout entirely), update those two lines to match --
// everything else here (scanning, connecting, persistence, the menu)
// doesn't care what service the far end exposes. Once you've settled on
// the actual byte format the engine meter sends, decode
// bleReadLatestEngineData()'s output wherever you want to display it
// (e.g. the Paramotor page) rather than reaching into the raw bytes from
// multiple places.
// =====================================================

#define BLE_MAX_SCAN_RESULTS 12
#define BLE_DEVICE_NAME_MAX_LEN 32

struct BleScanResult {
  char name[BLE_DEVICE_NAME_MAX_LEN];  // "" if the device didn't advertise a name
  char address[18];                    // "AA:BB:CC:DD:EE:FF\0"
  int rssi;
};

// Whether the pilot has turned Bluetooth on from the menu. Starts off at
// boot unless a device was remembered from a previous session (see
// loadBleSettings()), in which case it's turned on automatically so the
// vario can go looking for it.
extern bool bleEnabled;

// True while actually connected to a peripheral right now.
extern volatile bool bleConnected;

// Name/address of whichever device is currently connected, or was last
// remembered (persisted to flash) -- "" if none. Shown on the Bluetooth
// menu screen and used to auto-reconnect after a reboot.
extern char bleRememberedName[BLE_DEVICE_NAME_MAX_LEN];
extern char bleRememberedAddress[18];

// Call once from setup(), after Serial is up. Reads
// bleRememberedName/Address back from flash and initializes the BLE
// stack; if a device was remembered, turns Bluetooth on and starts
// looking for it.
void loadBleSettings();

// Call every backgroundTask() tick (cheap no-op when Bluetooth is off) --
// drives the auto-reconnect-to-remembered-device retry timer.
void bleManagerLoop();

// Turns the BLE radio on/off. Called from Connections > Bluetooth's
// on/off toggle. Turning off disconnects and stops any scan in progress.
void bleSetEnabled(bool enabled);

// Starts (or keeps alive) a continuous scan, refreshing
// bleScanResultCount()/bleScanResultAt() live as advertisements come in
// -- call when the Bluetooth Scan menu screen opens. No-op if Bluetooth
// is off.
void bleStartScan();

// Stops scanning -- call when leaving the Bluetooth Scan menu screen (no
// point burning radio time once the pilot isn't looking at the list).
void bleStopScan();

// Read-only view of the current scan results for the menu to render.
// Mutex-protected internally against the scan callback's own task, so
// it's safe to call from the menu's draw code.
uint8_t bleScanResultCount();
BleScanResult bleScanResultAt(uint8_t index);

// Connects to whichever device is at `index` in the current scan
// results, and remembers it (persisted to flash) so future boots
// auto-reconnect to it. Called when the pilot selects a device from the
// Bluetooth Scan menu screen.
void bleConnectToScanResult(uint8_t index);

// Disconnects (if connected) and forgets the remembered device -- future
// boots won't try to auto-reconnect until a new one is picked.
void bleForgetDevice();

// Copies out the raw bytes most recently notified by the engine meter's
// NUS TX characteristic (up to outBufLen), and reports via *outIsNew
// whether they're new since the last call. Format is whatever the
// nRF52840 firmware actually sends -- see the note at the top of this
// file. Mutex-protected internally; safe to call from anywhere. Returns
// the number of bytes copied.
size_t bleReadLatestEngineData(uint8_t* outBuf, size_t outBufLen, bool* outIsNew);

#endif  // BLE_MANAGER_H
