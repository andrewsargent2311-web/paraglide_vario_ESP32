// Sx126xLink.cpp

#include "Sx126xLink.h"

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
