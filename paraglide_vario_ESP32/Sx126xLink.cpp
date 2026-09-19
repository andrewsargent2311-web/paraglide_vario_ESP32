// Sx126xLink.cpp
#define RADIOLIB_DEBUG
#include "Sx126xLink.h"




//static void probeSX1262Raw();
// ============================================================================
// Constructor
// ============================================================================

Sx126xLink::Sx126xLink()

    : _spi(HSPI)

    , _module(nullptr)
    , _radio(nullptr)

    , _txInFlight(false)
    , _enabled(true)

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

// void probeSX1262Raw() {

//     Serial.println();
//     Serial.println("[PROBE] ========================================");
//     Serial.println("[PROBE] MANUAL SPI PIN TEST");
//     Serial.println("[PROBE] ========================================");

//     // ------------------------------------------------------------
//     // Configure pins manually
//     // ------------------------------------------------------------

//     pinMode(PIN_LORA_CS, OUTPUT);
//     pinMode(PIN_LORA_SCK, OUTPUT);
//     pinMode(PIN_LORA_MOSI, OUTPUT);
//     pinMode(PIN_LORA_MISO, INPUT);
//     pinMode(PIN_LORA_BUSY, INPUT);

//     digitalWrite(PIN_LORA_CS, HIGH);
//     digitalWrite(PIN_LORA_SCK, LOW);
//     digitalWrite(PIN_LORA_MOSI, LOW);

//     delay(100);

//     Serial.printf(
//         "[PROBE] CS   = GPIO%d = %d\n",
//         PIN_LORA_CS,
//         digitalRead(PIN_LORA_CS)
//     );

//     Serial.printf(
//         "[PROBE] SCK  = GPIO%d = %d\n",
//         PIN_LORA_SCK,
//         digitalRead(PIN_LORA_SCK)
//     );

//     Serial.printf(
//         "[PROBE] MOSI = GPIO%d = %d\n",
//         PIN_LORA_MOSI,
//         digitalRead(PIN_LORA_MOSI)
//     );

//     Serial.printf(
//         "[PROBE] MISO = GPIO%d = %d\n",
//         PIN_LORA_MISO,
//         digitalRead(PIN_LORA_MISO)
//     );

//     Serial.printf(
//         "[PROBE] BUSY = GPIO%d = %d\n",
//         PIN_LORA_BUSY,
//         digitalRead(PIN_LORA_BUSY)
//     );

//     // ------------------------------------------------------------
//     // Manually reset the SX1262
//     // ------------------------------------------------------------

 

//     // ------------------------------------------------------------
//     // Helper to send one byte manually.
//     //
//     // SPI mode 0:
//     //   clock idle LOW
//     //   sample MISO on rising edge
//     // ------------------------------------------------------------

//     auto transferByte = [] (uint8_t tx) -> uint8_t {

//         uint8_t rx = 0;

//         for (int8_t bit = 7; bit >= 0; bit--) {

//             // Set MOSI before rising edge
//             digitalWrite(
//                 PIN_LORA_MOSI,
//                 (tx >> bit) & 0x01
//             );

//             // Rising edge
//             digitalWrite(PIN_LORA_SCK, HIGH);

//             delayMicroseconds(2);

//             // Sample MISO
//             rx <<= 1;

//             if (digitalRead(PIN_LORA_MISO)) {
//                 rx |= 1;
//             }

//             // Falling edge
//             digitalWrite(PIN_LORA_SCK, LOW);

//             delayMicroseconds(2);
//         }

//         return rx;
//     };

//     // ------------------------------------------------------------
//     // GET_STATUS = 0xC0
//     //
//     // Byte 0:
//     //     send 0xC0
//     //
//     // Byte 1:
//     //     send dummy 0x00
//     //     receive status
//     // ------------------------------------------------------------

//     Serial.println();
//     Serial.println("[PROBE] Sending manual GET_STATUS 0xC0...");

//     digitalWrite(PIN_LORA_CS, LOW);

//     delayMicroseconds(10);

//     uint8_t rx0 = transferByte(0xC0);
//     uint8_t rx1 = transferByte(0x00);

//     delayMicroseconds(10);

//     digitalWrite(PIN_LORA_CS, HIGH);

//     digitalWrite(PIN_LORA_MOSI, LOW);

//     Serial.printf(
//         "[PROBE] Manual SPI response0 = 0x%02X\n",
//         rx0
//     );

//     Serial.printf(
//         "[PROBE] Manual SPI status    = 0x%02X\n",
//         rx1
//     );

//     Serial.printf(
//         "[PROBE] BUSY after command  = %d\n",
//         digitalRead(PIN_LORA_BUSY)
//     );

//     // ------------------------------------------------------------
//     // Interpret result
//     // ------------------------------------------------------------

//     if (rx1 != 0x00 && rx1 != 0xFF) {

//         uint8_t chipMode =
//             (rx1 >> 4) & 0x07;

//         uint8_t cmdStatus =
//             (rx1 >> 1) & 0x07;

//         Serial.println();
//         Serial.println(
//             "[PROBE] *** SX1262 RESPONDED ***"
//         );

//         Serial.printf(
//             "[PROBE] chipMode  = %d\n",
//             chipMode
//         );

//         Serial.printf(
//             "[PROBE] cmdStatus = %d\n",
//             cmdStatus
//         );

//     } else {

//         Serial.println();
//         Serial.println(
//             "[PROBE] *** NO VALID SX1262 RESPONSE ***"
//         );

//         Serial.println(
//             "[PROBE] Manual GPIO SPI also returned 0x00/0xFF."
//         );
//     }

//     Serial.println();
//     Serial.println("[PROBE] ========================================");
//     Serial.println("[PROBE] End manual SPI test");
//     Serial.println("[PROBE] ========================================");
// }
// void probeSX1262HardwareSPI(SPIClass& spi) {

//     Serial.println();
//     Serial.println("[PROBE] ========================================");
//     Serial.println("[PROBE] SPIClass HARDWARE SPI TEST");
//     Serial.println("[PROBE] ========================================");

//     spi.begin(
//         PIN_LORA_SCK,
//         PIN_LORA_MISO,
//         PIN_LORA_MOSI,
//         PIN_LORA_CS
//     );

//     pinMode(PIN_LORA_CS, OUTPUT);
//     digitalWrite(PIN_LORA_CS, HIGH);

//     delay(100);

//     Serial.printf(
//         "[PROBE] BUSY before command = %d\n",
//         digitalRead(PIN_LORA_BUSY)
//     );

//     spi.beginTransaction(
//         SPISettings(
//             500000,
//             MSBFIRST,
//             SPI_MODE0
//         )
//     );

//     digitalWrite(PIN_LORA_CS, LOW);

//     uint8_t rx0 = spi.transfer(0xC0);
//     uint8_t rx1 = spi.transfer(0x00);

//     digitalWrite(PIN_LORA_CS, HIGH);

//     spi.endTransaction();

//     Serial.printf(
//         "[PROBE] SPIClass response0 = 0x%02X\n",
//         rx0
//     );

//     Serial.printf(
//         "[PROBE] SPIClass status    = 0x%02X\n",
//         rx1
//     );

//     Serial.printf(
//         "[PROBE] BUSY after command = %d\n",
//         digitalRead(PIN_LORA_BUSY)
//     );

//     if (rx1 != 0x00 && rx1 != 0xFF) {

//         Serial.println(
//             "[PROBE] *** SPIClass CAN COMMUNICATE WITH SX1262 ***"
//         );

//     } else {

//         Serial.println(
//             "[PROBE] *** SPIClass CANNOT COMMUNICATE WITH SX1262 ***"
//         );
//     }

//     Serial.println(
//         "[PROBE] ========================================"
//     );
// }

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

    // ------------------------------------------------------------------------
    // Bring up the SPI bus on the HT-RA62's actual pins.
    //
    // ROOT CAUSE (found via probeSX1262Raw()):
    //
    //   A manual bit-banged GET_STATUS (0xC0) on these exact pins returned a
    //   valid SX1262 status byte (0x22 -> STBY_RC / OK), proving the chip and
    //   wiring are both fine.
    //
    //   RadioLib's Module/SPIClass path, however, never had these pins told
    //   to it. Without an explicit spi.begin(sck, miso, mosi, cs), ESP32's
    //   SPIClass silently falls back to its default FSPI/HSPI pin mapping,
    //   which is NOT GPIO0/1/2/3/17. RadioLib's SPI transactions were
    //   therefore going out on the wrong pins entirely, which is why
    //   SX1262::begin() reported RADIOLIB_ERR_CHIP_NOT_FOUND (-2) even though
    //   the chip itself was answering correctly.
    //
    // This must happen before Module/SX1262 issue any SPI transactions.
    // ------------------------------------------------------------------------

    _spi.begin(
        PIN_LORA_SCK,
        PIN_LORA_MISO,
        PIN_LORA_MOSI,
        PIN_LORA_CS
    );

    SPISettings radioSpiSettings(
        100000,
        MSBFIRST,
        SPI_MODE0
    );

    Serial.println("[FANET] About to construct Module...");

    _module =
        new Module(
            PIN_LORA_CS,
            RADIOLIB_NC,
            RADIOLIB_NC,
            PIN_LORA_BUSY,
            _spi,
            radioSpiSettings
        );

    Serial.println("[FANET] Module constructed.");

    if (_module == nullptr) {
        Serial.println("[FANET] Module allocation FAILED");
        _lastStatus = RADIOLIB_ERR_UNKNOWN;
        return false;
    }

    Serial.println("[FANET] About to construct SX1262...");

    _radio = new SX1262(_module);

    Serial.println("[FANET] SX1262 constructed.");

    if (_radio == nullptr) {
        Serial.println("[FANET] SX1262 allocation FAILED");
        _lastStatus = RADIOLIB_ERR_UNKNOWN;
        return false;
    }

    Serial.println("[FANET] About to call RadioLib begin()...");

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

    Serial.printf("[FANET] RadioLib begin status = %d\n", _lastStatus);

    if (_lastStatus != RADIOLIB_ERR_NONE) {
        return false;
    }

    _lastStatus =
        _radio->setDio2AsRfSwitch(true);

    if (_lastStatus != RADIOLIB_ERR_NONE) {
        return false;
    }

    _lastStatus =
        _radio->setCRC(2);

    if (_lastStatus != RADIOLIB_ERR_NONE) {
        return false;
    }

    restartReceive();

    return (_lastStatus == RADIOLIB_ERR_NONE);
}

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

    // Deliberately no SPI traffic at all while disabled -- any
    // transaction (including a register read) wakes the chip back up
    // per the datasheet, which would defeat setEnabled(false).
    if (!_enabled) {
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

    if (!_enabled) {
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

    // ------------------------------------------------------------------------
    // TX-start timestamp log.
    //
    // Diagnostic only: lets a bad vario/altitude sample be correlated
    // against real TX activity (suspected RF coupling into nearby I2C
    // wiring). Compare this timestamp against [VARIO DEBUG] output in
    // the .ino. Safe to remove once the correlation is confirmed or
    // ruled out.
    // ------------------------------------------------------------------------

    Serial.printf(
        "[FANET TX_DEBUG] TX started at t=%lu ms, len=%u\n",
        millis(),
        (unsigned)len
    );

    return true;
}

// ============================================================================
// Enable / disable
// ============================================================================

bool Sx126xLink::setEnabled(bool enabled) {

    if (_radio == nullptr) {
        return false;
    }

    if (enabled == _enabled) {
        return true;
    }

    if (!enabled) {

        // Drop anything in flight rather than leaving it stranded --
        // there is no DIO1 to tell us it finished after this point.
        _txInFlight = false;

        _lastStatus = _radio->sleep();

        _enabled = false;

        return (_lastStatus == RADIOLIB_ERR_NONE);
    }

    // Waking is just resuming normal operation -- the chip already came
    // back to STDBY_RC on its own the moment we next talk to it over SPI.
    restartReceive();

    _enabled = true;

    return true;
}
