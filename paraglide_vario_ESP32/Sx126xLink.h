// Sx126xLink.h
//
// Thin wrapper around RadioLib's SX1262 driver, built specifically for a
// wiring where DIO1 is NOT connected (no free GPIO) and RST is tied
// externally (to 3.3V or to the host MCU's EN line) rather than to a pin
// this firmware drives.
//
// Consequences of that wiring, baked into this class:
//   - No hardware interrupt on packet events. We use RadioLib's
//     non-blocking startTransmit()/startReceive() calls and poll
//     getIrqFlags() ourselves from the main loop (call poll() often).
//   - No software-triggered radio reset. begin() assumes the chip is
//     already out of reset (power-on state) when it runs.
//
// Pin mapping (per your wiring):
//   MISO  -> GPIO0
//   SCK   -> GPIO1
//   BUSY  -> GPIO2
//   MOSI  -> GPIO3
//   CS/NSS-> GPIO17
//   DIO1  -> not connected (RADIOLIB_NC)
//   RST   -> not connected (RADIOLIB_NC) -- tied externally

#pragma once

#include <Arduino.h>
#include <SPI.h>
#include <RadioLib.h>

// ---- Pin definitions -------------------------------------------------
static const int PIN_LORA_MISO = 0;
static const int PIN_LORA_SCK  = 1;
static const int PIN_LORA_BUSY = 2;
static const int PIN_LORA_MOSI = 3;
static const int PIN_LORA_CS   = 17;

// Radio link states, surfaced to the protocol layer so it knows what
// happened without needing to know about IRQ bits itself.
enum class RadioEvent {
    NONE,
    TX_DONE,
    RX_DONE,
    TIMEOUT,
    CRC_ERROR
};

class Sx126xLink {
public:
    Sx126xLink();

    // Bring up SPI + the radio. Call once from setup().
    // Returns true on success. On failure, check lastStatus() for the
    // RadioLib status code (see RadioLib's RADIOLIB_ERR_* constants).
    bool begin(float freqMHz, float bwKHz, uint8_t sf, uint8_t cr,
               uint8_t syncWord, int8_t powerDbm, uint16_t preambleLen);

    // Must be called frequently from loop() (every few ms is plenty for
    // FANET's timing). Polls IRQ status over SPI, handles the
    // TX-done -> re-arm-RX housekeeping, and copies out received packets.
    RadioEvent poll();

    // Queue a packet for transmission. Internally does a channel-activity
    // check first (best-effort CSMA) before calling startTransmit().
    // Returns true if the packet was handed to the radio.
    bool send(const uint8_t* data, size_t len);

    // After poll() returns RX_DONE, call these to retrieve the packet.
    size_t receivedLength() const { return _rxLen; }
    const uint8_t* receivedData() const { return _rxBuf; }
    float lastRssi() const { return _lastRssi; }
    float lastSnr() const { return _lastSnr; }

    int16_t lastStatus() const { return _lastStatus; }

    // True while a transmit is in flight (poll() hasn't seen TX_DONE yet).
    bool isTransmitting() const { return _txInFlight; }

private:
    static const size_t RX_BUF_SIZE = 256;

    SPIClass    _spi;
    Module*     _module;
    SX1262*     _radio;

    bool        _txInFlight;
    uint8_t     _rxBuf[RX_BUF_SIZE];
    size_t      _rxLen;
    float       _lastRssi;
    float       _lastSnr;
    int16_t     _lastStatus;

    void restartReceive();
};
