// Sx126xLink.cpp
#include "Sx126xLink.h"

Sx126xLink::Sx126xLink()
    : _spi(HSPI)
    , _module(nullptr)
    , _radio(nullptr)
    , _txInFlight(false)
    , _rxLen(0)
    , _lastRssi(0)
    , _lastSnr(0)
    , _lastStatus(RADIOLIB_ERR_NONE)
{}

bool Sx126xLink::begin(float freqMHz, float bwKHz, uint8_t sf, uint8_t cr,
                        uint8_t syncWord, int8_t powerDbm, uint16_t preambleLen) {

    _spi.begin(PIN_LORA_SCK, PIN_LORA_MISO, PIN_LORA_MOSI, PIN_LORA_CS);

    // NC for both IRQ (DIO1) and RESET -- see header comment. RadioLib
    // treats RADIOLIB_NC pins as "not wired" and skips toggling them.
    _module = new Module(PIN_LORA_CS, RADIOLIB_NC, RADIOLIB_NC, PIN_LORA_BUSY, _spi);
    _radio  = new SX1262(_module);

    _lastStatus = _radio->begin(freqMHz, bwKHz, sf, cr, syncWord,
                                 powerDbm, preambleLen);
    if (_lastStatus != RADIOLIB_ERR_NONE) {
        return false;
    }

    // DIO2 as RF switch control is the common wiring for these cheap
    // SX1262 modules (HT-RA62 included) -- confirm against your module's
    // datasheet if range/output power looks wrong.
    _radio->setDio2AsRfSwitch(true);

    restartReceive();
    return true;
}

void Sx126xLink::restartReceive() {
    _lastStatus = _radio->startReceive();
    _txInFlight = false;
}

RadioEvent Sx126xLink::poll() {
    // Ask the chip directly what happened -- this is the substitute for
    // a DIO1 interrupt. Cheap over SPI, safe to call often.
    // getIrqFlags()/clearIrqFlags() are RadioLib's current cross-chip
    // public API for this (older RadioLib versions used differently-named
    // methods here, so if you're on an older library version and this
    // doesn't compile, check what your installed version calls these).
    uint32_t irq = _radio->getIrqFlags();

    if (irq == 0) {
        return RadioEvent::NONE;
    }

    RadioEvent result = RadioEvent::NONE;

    if (irq & RADIOLIB_SX126X_IRQ_TX_DONE) {
        _radio->clearIrqFlags(RADIOLIB_SX126X_IRQ_TX_DONE);
        _txInFlight = false;
        result = RadioEvent::TX_DONE;
        // Go straight back to listening -- FANET devices spend almost all
        // their time receiving.
        restartReceive();
        return result;
    }

    if (irq & RADIOLIB_SX126X_IRQ_CRC_ERR) {
        _radio->clearIrqFlags(RADIOLIB_SX126X_IRQ_CRC_ERR);
        restartReceive();
        return RadioEvent::CRC_ERROR;
    }

    if (irq & RADIOLIB_SX126X_IRQ_TIMEOUT) {
        _radio->clearIrqFlags(RADIOLIB_SX126X_IRQ_TIMEOUT);
        restartReceive();
        return RadioEvent::TIMEOUT;
    }

    if (irq & RADIOLIB_SX126X_IRQ_RX_DONE) {
        _radio->clearIrqFlags(RADIOLIB_SX126X_IRQ_RX_DONE);

        size_t len = _radio->getPacketLength();
        if (len > RX_BUF_SIZE) {
            len = RX_BUF_SIZE;
        }
        _lastStatus = _radio->readData(_rxBuf, len);
        _rxLen  = len;
        _lastRssi = _radio->getRSSI();
        _lastSnr  = _radio->getSNR();

        restartReceive();
        return RadioEvent::RX_DONE;
    }

    return RadioEvent::NONE;
}

bool Sx126xLink::send(const uint8_t* data, size_t len) {
    if (_txInFlight) {
        return false; // previous send hasn't completed yet
    }

    // Best-effort CSMA: FANET expects senders to check the channel is
    // clear before transmitting. scanChannel() blocks briefly (CAD is
    // fast, on the order of a couple of symbol periods) -- acceptable
    // since it's not interrupt-driven anyway on this wiring.
    int16_t cadResult = _radio->scanChannel();
    if (cadResult == RADIOLIB_LORA_DETECTED) {
        // Channel busy -- caller should retry shortly (e.g. after a
        // short random backoff, per FANET's channel access rules).
        return false;
    }

    _lastStatus = _radio->startTransmit(const_cast<uint8_t*>(data), len);
    if (_lastStatus != RADIOLIB_ERR_NONE) {
        return false;
    }
    _txInFlight = true;
    return true;
}
