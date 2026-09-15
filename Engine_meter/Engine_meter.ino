// =====================================================================
// ENGINE METER firmware -- nice!nano (nRF52840)
// Reads RPM (inductive pickup -> LMV331 comparator) + dual MAX31856
// thermocouple channels (EGT + CHT), and streams the data over BLE
// to the paraglide_vario_ESP32 flight computer using the Nordic UART
// Service (NUS) -- the exact same service/characteristic the ESP32's
// ble_manager.cpp already scans for and subscribes to.
//
// Board: nice!nano (nRF52840), Adafruit nRF52 Arduino core
// Libraries needed (Library Manager):
//   - "Adafruit MAX31856 library" (by Adafruit)
//   - Bluefruit (bluefruit.h) ships with the Adafruit nRF52 BSP already
//
// Pin mapping traced from Engine_monitor_V1.kicad_sch. Adafruit's nRF52
// core numbers digital pins as: P0.xx -> pin xx, P1.xx -> pin (32+xx).
//
// *** CONTAINS TEMPORARY TEST CODE ***
// Search this file for "TEST CODE" -- there's a simulation mode that
// kicks in if no real RPM/CHT signal shows up within 45s of boot, so you
// can bench-test the BLE link to the vario without an engine attached.
// Delete it once real hardware is verified working.
// =====================================================================

#include <bluefruit.h>
#include <SPI.h>
#include <Adafruit_MAX31856.h>

// ---------------------------------------------------------------------
// PINS (from schematic global labels)
// ---------------------------------------------------------------------
#define PIN_SDO     32   // P1.00 -- MAX31856 SDO  -> MCU MISO (shared bus)
#define PIN_CS_CHT  11   // P0.11 -- MAX31856 #2 (CHT) chip select
#define PIN_CS_EGT  36   // P1.04 -- MAX31856 #1 (EGT) chip select
#define PIN_SCK     47   // P1.15 -- shared SPI clock
#define PIN_SDI     45   // P1.13 -- MCU MOSI -> MAX31856 SDI (shared bus)
#define PIN_RPM     43   // P1.11 -- comparator output (LMV331), pulled up by R8

// ---------------------------------------------------------------------
// RPM CONFIG
// ---------------------------------------------------------------------
// How many spark pulses per crank revolution the pickup sees.
// Most single-cylinder 2-stroke paramotor engines: 1 pulse/rev.
// Change to 0.5 for a twin sharing one kill lead, 2 for a wasted-spark
// 4-stroke twin that fires every pulse but only every 2nd is a power
// stroke, etc. -- tune this to what you actually observe.
#define PULSES_PER_REV        1.0f

// If no pulse has arrived in this long, report RPM = 0 (engine off/stalled)
// rather than showing a stale reading.
#define RPM_TIMEOUT_MS         1500

// ---------------------------------------------------------------------
// BLE -- Nordic UART Service, matching ble_manager.cpp on the ESP32 side
// ---------------------------------------------------------------------
BLEUart bleuart;  // Bluefruit's BLEUart *is* the Nordic UART Service --
                   // same service UUID (6E400001-...) and TX characteristic
                   // (6E400003-...) the ESP32 vario already looks for.

// ---------------------------------------------------------------------
// Thermocouple amps -- software SPI (bit-banged) so we can use the
// arbitrary GPIOs the board routes them on, sharing one SCK/SDI/SDO bus
// with separate chip-selects.
// ---------------------------------------------------------------------
Adafruit_MAX31856 maxEGT(PIN_CS_EGT, PIN_SDI, PIN_SDO, PIN_SCK);
Adafruit_MAX31856 maxCHT(PIN_CS_CHT, PIN_SDI, PIN_SDO, PIN_SCK);

// ---------------------------------------------------------------------
// RPM: measured from the period between pulses (captured in an ISR),
// not from a counted-pulses-per-window average -- this responds
// immediately to RPM changes instead of lagging behind a sample window.
// ---------------------------------------------------------------------
volatile uint32_t lastPulseUs = 0;
volatile uint32_t lastPeriodUs = 0;
volatile uint32_t lastPulseMillis = 0;

void rpmISR() {
  uint32_t now = micros();
  uint32_t period = now - lastPulseUs;
  // Basic debounce: ignore pulses closer together than 1ms (would imply
  // >30,000 RPM at 2 pulses/rev, well above anything realistic here).
  if (period > 1000) {
    lastPeriodUs = period;
    lastPulseUs = now;
    lastPulseMillis = millis();
  }
}

float readRpm() {
  noInterrupts();
  uint32_t period = lastPeriodUs;
  uint32_t sinceLastPulse = millis() - lastPulseMillis;
  interrupts();

  if (lastPulseMillis == 0 || sinceLastPulse > RPM_TIMEOUT_MS || period == 0) {
    return 0.0f;
  }
  float revsPerSec = (1000000.0f / period) / PULSES_PER_REV;
  return revsPerSec * 60.0f;
}

// ---------------------------------------------------------------------
// OUTGOING PACKET FORMAT
// Fixed 9-byte layout, well under the default ~20 byte NUS notify MTU.
// This is a starting-point protocol -- change it freely, just keep the
// ESP32-side decoder (which you'll add in bleReadLatestEngineData()'s
// caller / drawParamotorPage()) in sync with whatever you settle on.
//
//   byte 0    : 0xAE  sync/header byte
//   byte 1-2  : uint16_t RPM, little-endian
//   byte 3-4  : int16_t  EGT, little-endian, in 0.1 degC units
//   byte 5-6  : int16_t  CHT, little-endian, in 0.1 degC units
//   byte 7    : status flags (bit0 = EGT fault, bit1 = CHT fault)
//   byte 8    : checksum = XOR of bytes 0..7
// ---------------------------------------------------------------------
#define PKT_HEADER 0xAE

void sendEnginePacket(float rpm, float egtC, bool egtFault,
                       float chtC, bool chtFault) {
  uint8_t pkt[9];
  uint16_t rpmVal = (uint16_t)constrain(rpm, 0, 65535);
  int16_t egtVal = (int16_t)constrain(egtC * 10.0f, -3000, 30000);
  int16_t chtVal = (int16_t)constrain(chtC * 10.0f, -3000, 30000);

  pkt[0] = PKT_HEADER;
  pkt[1] = (uint8_t)(rpmVal & 0xFF);
  pkt[2] = (uint8_t)(rpmVal >> 8);
  pkt[3] = (uint8_t)(egtVal & 0xFF);
  pkt[4] = (uint8_t)(egtVal >> 8);
  pkt[5] = (uint8_t)(chtVal & 0xFF);
  pkt[6] = (uint8_t)(chtVal >> 8);
  pkt[7] = (egtFault ? 0x01 : 0x00) | (chtFault ? 0x02 : 0x00);

  uint8_t chk = 0;
  for (int i = 0; i < 8; i++) chk ^= pkt[i];
  pkt[8] = chk;

  if (Bluefruit.connected()) {
    bleuart.write(pkt, sizeof(pkt));
  }
}

// ---------------------------------------------------------------------
// BLE connect/disconnect callbacks -- just for a status LED / Serial log
// ---------------------------------------------------------------------
void connectCallback(uint16_t conn_handle) {
  BLEConnection* conn = Bluefruit.Connection(conn_handle);
  char name[32] = {0};
  conn->getPeerName(name, sizeof(name));
  Serial.print("[BLE] Connected to ");
  Serial.println(name);
}

void disconnectCallback(uint16_t conn_handle, uint8_t reason) {
  (void)conn_handle;
  (void)reason;
  Serial.println("[BLE] Disconnected -- advertising again");
}

void startAdv() {
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addService(bleuart);
  Bluefruit.Advertising.addName();

  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);  // in units of 0.625ms
  Bluefruit.Advertising.setFastTimeout(30);
  Bluefruit.Advertising.start(0);              // 0 = never stop advertising
}

// =====================================================================
// ############################  TEST CODE  ############################
// SIMULATION MODE -- FOR BENCH TESTING ONLY, DELETE THIS WHOLE BLOCK
// (and the two call-outs to it in loop(), each marked the same way)
// BEFORE FLYING.
//
// If 45 seconds pass after boot with no real RPM pulses AND the CHT
// thermocouple channel is faulted (i.e. nothing's actually wired up --
// the normal state while this board is sitting on a bench away from an
// engine), this switches to feeding the ESP32 a fake but plausible-
// looking data stream instead of real sensor readings. The only point
// is to prove the BLE link and packet decode work end-to-end on the
// vario's Paramotor page without needing an engine running. It does NOT
// touch real sensor logic -- once you plug in a real pickup/thermocouple
// this condition never triggers and the board behaves exactly as before.
// =====================================================================
#define SIM_TRIGGER_DELAY_MS   45000UL
bool simulationMode = false;

void maybeEnterSimulationMode(float realRpm, bool realChtFault) {
  if (simulationMode) return;
  if (millis() < SIM_TRIGGER_DELAY_MS) return;
  if (realRpm > 0.0f || !realChtFault) return;  // real hardware looks present -- leave it alone

  simulationMode = true;
  Serial.println("[TEST] No RPM pulses / no CHT thermocouple detected after 45s -- "
                  "entering SIMULATION MODE (fake test data). "
                  "REMOVE maybeEnterSimulationMode()/generateSimulatedData() before flying!");
}

// Triangle-wave RPM (idle <-> redline over ~20s) and sine-wave CHT/EGT
// (over ~60s) -- just needs to move around convincingly on the display,
// doesn't need to be physically realistic.
void generateSimulatedData(float& rpm, float& egtC, bool& egtFault,
                            float& chtC, bool& chtFault) {
  const float RPM_MIN = 1200.0f, RPM_MAX = 6500.0f, RPM_PERIOD_MS = 20000.0f;
  float rpmPhase = fmodf(millis(), RPM_PERIOD_MS) / RPM_PERIOD_MS;  // 0..1
  float rpmTri = (rpmPhase < 0.5f) ? (rpmPhase * 2.0f) : (2.0f - rpmPhase * 2.0f);  // triangle 0..1..0
  rpm = RPM_MIN + rpmTri * (RPM_MAX - RPM_MIN);

  const float CHT_MIN = 70.0f, CHT_MAX = 150.0f, CHT_PERIOD_MS = 60000.0f;
  float chtSine = (sinf(2.0f * PI * millis() / CHT_PERIOD_MS) + 1.0f) / 2.0f;  // 0..1
  chtC = CHT_MIN + chtSine * (CHT_MAX - CHT_MIN);
  chtFault = false;

  const float EGT_MIN = 300.0f, EGT_MAX = 650.0f;
  egtC = EGT_MIN + chtSine * (EGT_MAX - EGT_MIN);  // rides the same phase as CHT, close enough
  egtFault = false;
}
// ##########################  END TEST CODE  ##########################

void setup() {
  Serial.begin(115200);
  // Uncomment while debugging on USB power if you want to see setup
  // logs before the vario is nearby to connect to:
  // while (!Serial) delay(10);

  // ---- RPM input ----
  pinMode(PIN_RPM, INPUT);  // external 10k pull-up (R8) already on board
  attachInterrupt(digitalPinToInterrupt(PIN_RPM), rpmISR, FALLING);

  // ---- Thermocouples ----
  maxEGT.begin();
  maxEGT.setThermocoupleType(MAX31856_TCTYPE_K);
  maxCHT.begin();
  maxCHT.setThermocoupleType(MAX31856_TCTYPE_K);

  // ---- BLE ----
  Bluefruit.begin();
  Bluefruit.setTxPower(4);
  Bluefruit.setName("ParaEngineMeter");  // shows up in the vario's BT scan list
  Bluefruit.Periph.setConnectCallback(connectCallback);
  Bluefruit.Periph.setDisconnectCallback(disconnectCallback);

  bleuart.begin();

  startAdv();
  Serial.println("[BOOT] Engine meter ready, advertising as ParaEngineMeter");
}

void loop() {
  static uint32_t lastSend = 0;
  const uint32_t SEND_INTERVAL_MS = 200;  // 5 Hz telemetry

  if (millis() - lastSend >= SEND_INTERVAL_MS) {
    lastSend = millis();

    float rpm = readRpm();

    float egtC = maxEGT.readThermocoupleTemperature();
    uint8_t egtFaultBits = maxEGT.readFault();
    bool egtFault = (egtFaultBits != 0);

    float chtC = maxCHT.readThermocoupleTemperature();
    uint8_t chtFaultBits = maxCHT.readFault();
    bool chtFault = (chtFaultBits != 0);

    // ---- TEST CODE: delete this call along with the block it calls
    // (maybeEnterSimulationMode/generateSimulatedData above) once you're
    // done bench-testing the BLE link. See the block comment above for
    // what this does and why. ----
    maybeEnterSimulationMode(rpm, chtFault);
    if (simulationMode) {
      generateSimulatedData(rpm, egtC, egtFault, chtC, chtFault);
    }
    // ---- END TEST CODE ----

    sendEnginePacket(rpm, egtC, egtFault, chtC, chtFault);

    Serial.printf("RPM=%.0f  EGT=%.1fC%s  CHT=%.1fC%s%s\n",
                  rpm, egtC, egtFault ? " [FAULT]" : "",
                  chtC, chtFault ? " [FAULT]" : "",
                  simulationMode ? "  [SIMULATED -- TEST DATA]" : "");
  }
}
