// Sx126xLink.h
//
// Thin SX1262 / HT-RA62 radio wrapper.
//
// IMPORTANT HARDWARE CONSTRAINT:
//
// DIO1 is NOT connected to the ESP32-S3.
// RST is also not controlled by an ESP32 GPIO.
//
// Therefore this class NEVER uses RadioLib's blocking scanChannel(),
// because RadioLib's scanChannel() waits for the DIO1 interrupt.
//
// Instead:
//
//   - TX completion is detected by polling the SX1262 IRQ register.
//   - RX completion is detected by polling the SX1262 IRQ register.
//   - CAD/channel activity is started normally but its IRQ register is
//     polled directly over SPI.
//
// This allows the HT-RA62 to operate without consuming another ESP32 GPIO.
//
// Hardware:
//
//   HT-RA62 / SX1262
//     MISO -> GPIO0
//     SCK  -> GPIO1
//     BUSY -> GPIO2
//     MOSI -> GPIO3
//     NSS  -> GPIO17
//     DIO1 -> NC
//     RST  -> NC / externally handled
//
// DIO2 is used by the SX1262 as the RF switch control.
//

#pragma once

#include <Arduino.h>
#include <SPI.h>
#include <RadioLib.h>

// ============================================================================
// HT-RA62 pin mapping
// ============================================================================

static constexpr int PIN_LORA_MISO = 0;
static constexpr int PIN_LORA_SCK  = 1;
static constexpr int PIN_LORA_BUSY = 2;
static constexpr int PIN_LORA_MOSI = 3;
static constexpr int PIN_LORA_CS   = 17;

// ============================================================================
// Radio events
// ============================================================================

enum class RadioEvent {

    NONE,

    TX_DONE,

    RX_DONE,

    TIMEOUT,

    CRC_ERROR
};

// ============================================================================
// SX1262 link
// ============================================================================

class Sx126xLink {

public:

    Sx126xLink();

    // Initialise the HT-RA62.
    //
    // freqMHz:
    //   FANET NZ = 868.2 MHz
    //
    // bwKHz:
    //   FANET NZ = 250 kHz
    //
    // sf:
    //   FANET = SF7
    //
    // cr:
    //   RadioLib coding-rate denominator.
    //   5 = 4/5.
    //
    // syncWord:
    //   FANET = 0xF1
    //
    // powerDbm:
    //   HT-RA62 output power.
    //
    // preambleLen:
    //   FANET = 8 symbols in this implementation.
    bool begin(
        float freqMHz,
        float bwKHz,
        uint8_t sf,
        uint8_t cr,
        uint8_t syncWord,
        int8_t powerDbm,
        uint16_t preambleLen
    );

    // Poll the SX1262 IRQ register.
    //
    // Call this frequently from loop().
    //
    // No DIO1 is required.
    RadioEvent poll();

    // Check the channel using SX1262 CAD and transmit if clear.
    //
    // This does NOT call RadioLib::scanChannel(), because that function
    // waits for DIO1.
    //
    // Returns true only when startTransmit() successfully starts the TX.
    bool send(
        const uint8_t* data,
        size_t len
    );

    size_t receivedLength() const {
        return _rxLen;
    }

    const uint8_t* receivedData() const {
        return _rxBuf;
    }

    float lastRssi() const {
        return _lastRssi;
    }

    float lastSnr() const {
        return _lastSnr;
    }

    int16_t lastStatus() const {
        return _lastStatus;
    }

    bool isTransmitting() const {
        return _txInFlight;
    }

private:

    static constexpr size_t RX_BUF_SIZE = 256;

    // CAD should finish in only a few milliseconds at SF7 / 250 kHz.
    // This is deliberately bounded so a radio fault can never stall the
    // flight-computer loop indefinitely.
    static constexpr uint32_t CAD_TIMEOUT_MS = 25;

    SPIClass _spi;

    Module* _module;
    SX1262* _radio;

    bool _txInFlight;

    uint8_t _rxBuf[RX_BUF_SIZE];

    size_t _rxLen;

    float _lastRssi;
    float _lastSnr;

    int16_t _lastStatus;

    // Re-arm the radio in continuous receive mode.
    void restartReceive();

    // Perform DIO1-independent CAD.
    //
    // Returns:
    //   true  = channel clear
    //   false = channel busy or CAD failure
    bool channelClear();
};
