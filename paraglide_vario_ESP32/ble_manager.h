#ifndef BLE_MANAGER_H
#define BLE_MANAGER_H

#include <Arduino.h>
#include <atomic>

// =====================================================
// BLUETOOTH (BLE CENTRAL) MANAGER -- NimBLE-Arduino port
//
// This board (ESP32-S3) only has a Bluetooth LOW ENERGY radio -- there is
// no Classic Bluetooth (BR/EDR) on the S3. That means this can scan for
// and connect to other BLE peripherals (like a BLE-based engine meter),
// but it can never see or pair with phones/headsets/etc. using Classic
// BT profiles the way a phone's Bluetooth settings screen does.
//
// PROTOCOL: the nRF52840 engine meter advertises the Nordic UART Service
// (NUS) and notifies ASCII NMEA-style EMS and PEMBAT sentences on its TX
// characteristic. Notifications are reassembled into CRLF/LF-delimited
// lines before validation and decoded updates are exposed below.
//
// This file requires the "NimBLE-Arduino" library (h2zero) instead of the
// arduino-esp32 core's bundled classic BLE library -- install it via
// Library Manager. NimBLE fixes two problems the classic library has:
// it reliably merges scan-response data (device names) into the scan
// result, and its NimBLEAddress type carries the public/random address
// type discovered during scanning through to connect(), instead of
// silently assuming "public" the way the classic BLEAddress does.
// =====================================================

#define BLE_MAX_SCAN_RESULTS 12
#define BLE_DEVICE_NAME_MAX_LEN 32

struct BleScanResult {
  char name[BLE_DEVICE_NAME_MAX_LEN];  // "" if the device didn't advertise a name
  char address[18];                    // "AA:BB:CC:DD:EE:FF\0"
  uint8_t addressType;                 // 0 = public, 1 = random (NimBLEAddress::getType())
  int rssi;
};

// Whether the pilot has turned Bluetooth on from the menu. Starts off at
// boot unless a device was remembered from a previous session (see
// loadBleSettings()), in which case it's turned on automatically so the
// vario can go looking for it.
extern std::atomic<bool> bleEnabled;

// True while actually connected to a peripheral right now.
extern std::atomic<bool> bleConnected;

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
// Bluetooth Scan menu screen. Returns true if the connection (and NUS
// characteristic subscription) actually succeeded.
bool bleConnectToScanResult(uint8_t index);

// Disconnects (if connected) and forgets the remembered device -- future
// boots won't try to auto-reconnect until a new one is picked.
void bleForgetDevice();

// =====================================================
// DECODED ENGINE TELEMETRY
//
// bleManagerLoop() validates NMEA EMS and PEMBAT sentences and updates
// these values. Pages/menus should read these fields rather than consume
// the NUS notification stream directly.
//
// engineDataValid is automatically cleared ~2s after the last valid EMS
// sentence (engine meter out of range, powered off, or disconnected) so a
// stale reading doesn't sit on screen looking current -- always check it
// before displaying engineRpm/engineEgtC/engineChtC.
// =====================================================
extern volatile bool engineDataValid;
extern volatile float engineRpm;     // RPM
extern volatile float engineEgtC;    // Exhaust gas temp, deg C
extern volatile float engineChtC;    // Cylinder head temp, deg C
extern volatile uint8_t engineBatteryPercent;  // 0-100 from PEMBAT sentences
extern volatile bool engineBatteryPercentValid;
extern volatile bool engineEgtFault; // true if the EGT thermocouple amp reported a fault (open/short circuit)
extern volatile bool engineChtFault; // same, for the CHT channel

#endif  // BLE_MANAGER_H
