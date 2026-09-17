// Sx126xLink.cpp

#include "Sx126xLink.h"
static void probeSX1262Raw();
// ============================================================================
// Constructor
// ============================================================================

Sx126xLink::Sx126xLink()

    : _spi(HSPI)

    , _module(nullptr)
    , _radio(nullptr)

    , _txInFlight(false)

    , _rxLen(0)

    , _lastRssi(0.0f)
    , _lastSnr(0.0f)

    , _lastStatus(RADIOLIB_ERR_NONE)
{
}
// ============================================================================
// Raw SPI SX1262 probe — bypasses RadioLib entirely.
//
// Call this BEFORE Sx126xLink::begin() / _radio->begin().
//
// Sends GetStatus (0xC0) and prints:
//   - BUSY pin behavior around the transaction
//   - the raw status byte returned
//
// A responsive chip returns a status byte where bits 1-3 encode chip mode
// and bits 4-6 encode command status. A value of 0x00 or 0xFF (stuck high/low
// on MISO, i.e. no real reply) means the chip isn't answering over SPI at all.
// ============================================================================

void probeSX1262Raw() {
    delay(200);
    pinMode(PIN_LORA_BUSY, INPUT);
    pinMode(PIN_LORA_CS, OUTPUT);
    digitalWrite(PIN_LORA_CS, HIGH);

    SPIClass probeSpi(HSPI);
    probeSpi.begin(PIN_LORA_SCK, PIN_LORA_MISO, PIN_LORA_MOSI, PIN_LORA_CS);

    Serial.println("[PROBE] --- Raw SX1262 SPI probe ---");
    Serial.printf("[PROBE] BUSY before probe: %d\n", digitalRead(PIN_LORA_BUSY));

    // Wait for BUSY to go low (chip ready to accept a command).
    // Bounded so we never hang forever if BUSY is stuck.
    uint32_t waitStart = millis();
    while (digitalRead(PIN_LORA_BUSY) == HIGH) {
        if (millis() - waitStart > 100) {
            Serial.println("[PROBE] BUSY never went LOW before command -- chip may be unreset/stuck");
            break;
        }
    }
    Serial.printf("[PROBE] BUSY immediately before CS low: %d (waited %lums)\n",
                  digitalRead(PIN_LORA_BUSY), millis() - waitStart);

    // --- Send GetStatus (0xC0) ---
    probeSpi.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE0));
    digitalWrite(PIN_LORA_CS, LOW);

    uint8_t rx0 = probeSpi.transfer(0xC0);  // opcode
    uint8_t rx1 = probeSpi.transfer(0x00);  // NOP, status returned here

    digitalWrite(PIN_LORA_CS, HIGH);
    probeSpi.endTransaction();

    Serial.printf("[PROBE] byte0 (during opcode): 0x%02X\n", rx0);
    Serial.printf("[PROBE] byte1 (status):        0x%02X\n", rx1);

    // Watch BUSY after the transaction too -- should pulse then settle low.
    Serial.printf("[PROBE] BUSY right after CS high: %d\n", digitalRead(PIN_LORA_BUSY));
    delay(1);
    Serial.printf("[PROBE] BUSY +1ms after CS high:  %d\n", digitalRead(PIN_LORA_BUSY));

    if (rx1 == 0x00 || rx1 == 0xFF) {
        Serial.println("[PROBE] Status byte looks invalid (0x00/0xFF) -- MISO likely not toggling, chip not responding");
    } else {
        uint8_t chipMode = (rx1 >> 4) & 0x07;
        uint8_t cmdStatus = (rx1 >> 1) & 0x07;
        Serial.printf("[PROBE] Chip appears responsive. chipMode=%d cmdStatus=%d\n", chipMode, cmdStatus);
    }

    Serial.println("[PROBE] --- End probe ---");
}
// ============================================================================
// Begin radio
// ============================================================================

bool Sx126xLink::begin(
    float freqMHz,
    float bwKHz,
    uint8_t sf,
    uint8_t cr,
    uint8_t syncWord,
    int8_t powerDbm,
    uint16_t preambleLen
) {
    // >>> INSERT probeSX1262Raw() CALL HERE <
    probeSX1262Raw();

    // ------------------------------------------------------------------------
    // Start the dedicated FANET SPI bus.
    // ------------------------------------------------------------------------

    _spi.begin(
        PIN_LORA_SCK,
        PIN_LORA_MISO,
        PIN_LORA_MOSI,
        PIN_LORA_CS
    );

    // ------------------------------------------------------------------------
    // Construct RadioLib module.
    //
    // DIO1 = RADIOLIB_NC
    // RST  = RADIOLIB_NC
    //
    // BUSY remains connected because the SX1262 requires it for SPI
    // transaction timing.
    // ------------------------------------------------------------------------

    _module =
        new Module(
            PIN_LORA_CS,
            RADIOLIB_NC,       // DIO1 intentionally not connected
            RADIOLIB_NC,       // RESET intentionally not connected
            PIN_LORA_BUSY,
            _spi
        );

    if (_module == nullptr) {

        _lastStatus =
            RADIOLIB_ERR_UNKNOWN;

        return false;
    }

    _radio =
        new SX1262(_module);

    if (_radio == nullptr) {

        _lastStatus =
            RADIOLIB_ERR_UNKNOWN;

        return false;
    }

    // ------------------------------------------------------------------------
    // Initialise LoRa modem.
    //
    // For New Zealand FANET:
    //
    //   868.2 MHz
    //   250 kHz
    //   SF7
    //   CR 4/5
    //   Sync 0xF1
    //
    // The calling sketch supplies these values.
    // ------------------------------------------------------------------------
    Serial.println("[FANET] Starting SX1262 initialization...");
    Serial.printf("[FANET] SPI pins: SCK=%d MISO=%d MOSI=%d CS=%d BUSY=%d\n",
              PIN_LORA_SCK,
              PIN_LORA_MISO,
              PIN_LORA_MOSI,
              PIN_LORA_CS,
              PIN_LORA_BUSY);

    Serial.printf("[FANET] BUSY pin state before begin: %d\n",
              digitalRead(PIN_LORA_BUSY));
    _lastStatus =
        _radio->begin(
            freqMHz,
            bwKHz,
            sf,
            cr,
            syncWord,
            powerDbm,
            preambleLen
        );
    Serial.printf("[FANET] SX1262 begin() returned: %d\n",
              _lastStatus);
    if (_lastStatus != RADIOLIB_ERR_NONE) {

        return false;
    }

    // ------------------------------------------------------------------------
    // HT-RA62 uses DIO2 for the RF switch.
    // ------------------------------------------------------------------------

    _lastStatus =
        _radio->setDio2AsRfSwitch(true);

    if (_lastStatus != RADIOLIB_ERR_NONE) {

        return false;
    }

    // ------------------------------------------------------------------------
    // FANET uses normal explicit LoRa packets with CRC.
    //
    // RadioLib's begin() already configures the normal LoRa packet mode,
    // but explicitly requesting CRC here makes the intended configuration
    // clear.
    // ------------------------------------------------------------------------

    _lastStatus =
        _radio->setCRC(
            2
        );

    if (_lastStatus != RADIOLIB_ERR_NONE) {

        return false;
    }

    // ------------------------------------------------------------------------
    // Start listening.
    // ------------------------------------------------------------------------

    restartReceive();

    return (
        _lastStatus ==
        RADIOLIB_ERR_NONE
    );
}

// ============================================================================
// Restart RX
// ============================================================================

void Sx126xLink::restartReceive() {

    if (_radio == nullptr) {
        return;
    }

    _lastStatus =
        _radio->startReceive();

    _txInFlight = false;
}

// ============================================================================
// Poll radio events
// ============================================================================
//
// This is the DIO1 replacement.
//
// Instead of:
//
//   DIO1 -> ESP32 interrupt
//
// we do:
//
//   ESP32 -> SPI -> SX1262 IRQ_STATUS
//
// RadioLib exposes getIrqFlags()/clearIrqFlags() specifically for this.
// ============================================================================

RadioEvent Sx126xLink::poll() {

    if (_radio == nullptr) {
        return RadioEvent::NONE;
    }

    uint32_t irq =
        _radio->getIrqFlags();

    if (irq == 0) {
        return RadioEvent::NONE;
    }

    // ------------------------------------------------------------------------
    // TX complete
    // ------------------------------------------------------------------------

    if (irq &
        RADIOLIB_SX126X_IRQ_TX_DONE) {

        _radio->clearIrqFlags(
            RADIOLIB_SX126X_IRQ_TX_DONE
        );

        _txInFlight = false;

        // Return immediately to RX.
        restartReceive();

        return RadioEvent::TX_DONE;
    }

    // ------------------------------------------------------------------------
    // CRC error
    //
    // RX_DONE and CRC_ERR may be reported together. We deliberately discard
    // CRC-invalid packets rather than passing them to FANET decoding.
    // ------------------------------------------------------------------------

    if (irq &
        RADIOLIB_SX126X_IRQ_CRC_ERR) {

        _radio->clearIrqFlags(
            RADIOLIB_SX126X_IRQ_CRC_ERR
        );

        // Also clear RX_DONE if it happened alongside CRC_ERR.
        if (irq &
            RADIOLIB_SX126X_IRQ_RX_DONE) {

            _radio->clearIrqFlags(
                RADIOLIB_SX126X_IRQ_RX_DONE
            );
        }

        restartReceive();

        return RadioEvent::CRC_ERROR;
    }

    // ------------------------------------------------------------------------
    // RX timeout
    // ------------------------------------------------------------------------

    if (irq &
        RADIOLIB_SX126X_IRQ_TIMEOUT) {

        _radio->clearIrqFlags(
            RADIOLIB_SX126X_IRQ_TIMEOUT
        );

        restartReceive();

        return RadioEvent::TIMEOUT;
    }

    // ------------------------------------------------------------------------
    // Packet received
    // ------------------------------------------------------------------------

    if (irq &
        RADIOLIB_SX126X_IRQ_RX_DONE) {

        size_t len =
            _radio->getPacketLength();

        if (len > RX_BUF_SIZE) {
            len = RX_BUF_SIZE;
        }

        _lastStatus =
            _radio->readData(
                _rxBuf,
                len
            );

        if (_lastStatus != RADIOLIB_ERR_NONE) {

            _rxLen = 0;

            restartReceive();

            if (_lastStatus ==
                RADIOLIB_ERR_CRC_MISMATCH) {

                return RadioEvent::CRC_ERROR;
            }

            return RadioEvent::NONE;
        }

        _rxLen = len;

        _lastRssi =
            _radio->getRSSI();

        _lastSnr =
            _radio->getSNR();

        // readData() clears the packet IRQ state itself.
        restartReceive();

        return RadioEvent::RX_DONE;
    }

    // ------------------------------------------------------------------------
    // Anything else.
    // ------------------------------------------------------------------------

    // Clear anything unexpected so it doesn't remain latched forever.
    _radio->clearIrqFlags(
        irq
    );

    return RadioEvent::NONE;
}

// ============================================================================
// DIO1-independent channel activity detection
// ============================================================================
//
// RadioLib's scanChannel() does:
//
//     startChannelScan()
//     while(!digitalRead(DIO1)) yield()
//     getChannelScanResult()
//
// That is unsuitable for this hardware because DIO1 is physically NC.
//
// Instead:
//
//     startChannelScan()
//     repeatedly read IRQ_STATUS over SPI
//     wait for CAD_DONE
//     inspect CAD_DETECTED
//
// The SX1262 itself performs the CAD operation; we're only replacing the
// physical interrupt with SPI polling.
// ============================================================================

bool Sx126xLink::channelClear() {

    if (_radio == nullptr) {
        return false;
    }

    // Make sure the radio isn't currently transmitting.
    if (_txInFlight) {
        return false;
    }

    // ------------------------------------------------------------------------
    // Start RadioLib's CAD state machine.
    //
    // IMPORTANT:
    // We intentionally do NOT call _radio->scanChannel().
    // scanChannel() waits for DIO1.
    // ------------------------------------------------------------------------

    _lastStatus =
        _radio->startChannelScan();

    if (_lastStatus != RADIOLIB_ERR_NONE) {

        // Recover to RX mode.
        restartReceive();

        return false;
    }

    const uint32_t start =
        millis();

    // ------------------------------------------------------------------------
    // Poll IRQ_STATUS until CAD_DONE.
    // ------------------------------------------------------------------------

    while (
        (uint32_t)(millis() - start)
        < CAD_TIMEOUT_MS
    ) {

        uint32_t irq =
            _radio->getIrqFlags();

        // CAD detected LoRa activity.
        if (irq &
            RADIOLIB_SX126X_IRQ_CAD_DETECTED) {

            _radio->clearIrqFlags(
                RADIOLIB_SX126X_IRQ_CAD_DONE |
                RADIOLIB_SX126X_IRQ_CAD_DETECTED
            );

            // Return to normal receive mode.
            restartReceive();

            return false;
        }

        // CAD has finished and no activity was detected.
        if (irq &
            RADIOLIB_SX126X_IRQ_CAD_DONE) {

            _radio->clearIrqFlags(
                RADIOLIB_SX126X_IRQ_CAD_DONE
            );

            // Return to RX before attempting TX.
            restartReceive();

            return true;
        }

        // Give the ESP32 scheduler a chance to run.
        delay(0);
    }

    // ------------------------------------------------------------------------
    // CAD took too long.
    //
    // Never leave the flight computer stuck waiting for the radio.
    // ------------------------------------------------------------------------

    _radio->clearIrqFlags(
        RADIOLIB_SX126X_IRQ_CAD_DONE |
        RADIOLIB_SX126X_IRQ_CAD_DETECTED |
        RADIOLIB_SX126X_IRQ_TIMEOUT
    );

    restartReceive();

    return false;
}

// ============================================================================
// Send
// ============================================================================

bool Sx126xLink::send(
    const uint8_t* data,
    size_t len
) {

    if (_radio == nullptr) {
        return false;
    }

    if (data == nullptr ||
        len == 0) {

        return false;
    }

    if (len > RADIOLIB_SX126X_MAX_PACKET_LENGTH) {
        return false;
    }

    // Do not start another TX while one is already running.
    if (_txInFlight) {
        return false;
    }

    // ------------------------------------------------------------------------
    // Channel activity detection.
    // ------------------------------------------------------------------------

    if (!channelClear()) {

        return false;
    }

    // ------------------------------------------------------------------------
    // Start non-blocking transmission.
    //
    // We do NOT call transmit(), because that would wait for TX completion.
    //
    // poll() will detect TX_DONE from the IRQ register.
    // ------------------------------------------------------------------------

    _lastStatus =
        _radio->startTransmit(
            data,
            len
        );

    if (_lastStatus != RADIOLIB_ERR_NONE) {

        restartReceive();

        return false;
    }

    _txInFlight = true;

    return true;
}
