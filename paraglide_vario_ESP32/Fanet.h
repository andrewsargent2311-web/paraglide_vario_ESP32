// Fanet.h
//
// FANET protocol layer for the ESP32-S3 flight computer.
//
// This implementation follows the documented FANET tracking packet format:
//
//   MAC header:
//     Byte 0: Extended Header / Forward / Type
//     Byte 1: Manufacturer
//     Byte 2: Unique ID low byte
//     Byte 3: Unique ID high byte
//
//   Tracking payload (type 1):
//     Bytes 0-2 : Latitude
//     Bytes 3-5 : Longitude
//     Bytes 6-7 : Online / aircraft type / altitude scaling / altitude
//     Byte  8   : Speed
//     Byte  9   : Climb
//     Byte 10   : Heading
//
// The radio layer deliberately does NOT require DIO1.
//
// Hardware wiring:
//
//   HT-RA62 / SX1262
//     MISO -> GPIO0
//     SCK  -> GPIO1
//     BUSY -> GPIO2
//     MOSI -> GPIO3
//     CS   -> GPIO17
//     DIO1 -> NC
//     RST  -> NC / externally held out of reset
//
// IMPORTANT:
//   FANET manufacturer/device ID is supplied by the main sketch.
//   For an unregistered device use manufacturer 0xFC or 0xFD and a unique
//   ID from 0x0001 to 0xFFFE.
//
// Protocol reference:
//   https://github.com/3s1d/fanet-stm32/blob/master/Src/fanet/radio/protocol.txt
//

#pragma once

#include <Arduino.h>
#include "Sx126xLink.h"

// -----------------------------------------------------------------------------
// FANET message types
// -----------------------------------------------------------------------------

enum FanetType : uint8_t {
    FANET_TYPE_ACK             = 0x00,
    FANET_TYPE_TRACKING        = 0x01,
    FANET_TYPE_NAME            = 0x02,
    FANET_TYPE_MESSAGE         = 0x03,
    FANET_TYPE_SERVICE         = 0x04,
    FANET_TYPE_LANDMARK        = 0x05,
    FANET_TYPE_REMOTE_CONFIG   = 0x06,
    FANET_TYPE_GROUND_TRACKING = 0x07,
    FANET_TYPE_HW_INFO         = 0x08,
    FANET_TYPE_THERMAL        = 0x09
};

// -----------------------------------------------------------------------------
// FANET address
// -----------------------------------------------------------------------------

struct FanetAddress {
    uint8_t  manufacturer;
    uint16_t id;
};

// -----------------------------------------------------------------------------
// Decoded FANET tracking packet
// -----------------------------------------------------------------------------

struct FanetTracking {

    double  latitude;
    double  longitude;

    int32_t altitudeM;

    float speedKmh;
    float climbMs;
    float headingDeg;

    uint8_t aircraftType;

    bool onlineTracking;

    // Optional turn-rate field.
    // The current transmitter does not send it.
    bool  hasTurnRate;
    float turnRateDegS;
};

// -----------------------------------------------------------------------------
// Callback types
// -----------------------------------------------------------------------------

typedef void (*FanetTrackingCallback)(
    const FanetAddress& src,
    const FanetTracking& pkt,
    float rssi,
    float snr
);

typedef void (*FanetRawCallback)(
    uint8_t type,
    const FanetAddress& src,
    const uint8_t* payload,
    size_t len,
    float rssi,
    float snr
);

// -----------------------------------------------------------------------------
// FANET stack
// -----------------------------------------------------------------------------

class FanetStack {

public:

    FanetStack(
        Sx126xLink& link,
        const FanetAddress& myAddress
    );

    // Call once after Sx126xLink::begin().
    void begin();

    // Call frequently from loop().
    // Handles RX processing and periodic tracking transmission.
    void update();

    // Update the local aircraft state used for tracking packets.
    void setPosition(
        double lat,
        double lon,
        int32_t altM,
        float speedKmh,
        float climbMs,
        float headingDeg
    );

    // FANET aircraft type:
    //
    // 0 = Other
    // 1 = Paraglider
    // 2 = Hangglider
    // 3 = Balloon
    // 4 = Glider
    // 5 = Powered Aircraft
    // 6 = Helicopter
    // 7 = UAV
    void setAircraftType(uint8_t type) {
        _aircraftType = type & 0x07;
    }

    void setBeaconIntervalMs(uint32_t ms) {
        _beaconIntervalMs = ms;
    }

    void setOnlineTracking(bool enabled) {
        _onlineTracking = enabled;
    }

    void onTracking(FanetTrackingCallback cb) {
        _trackingCb = cb;
    }

    void onRawPacket(FanetRawCallback cb) {
        _rawCb = cb;
    }

    // Immediately transmit the current tracking packet.
    // Returns true if the packet was handed to the radio.
    bool sendTrackingNow();

private:

    Sx126xLink& _link;

    FanetAddress _myAddress;

    // Current outgoing aircraft state.
    double _lat;
    double _lon;

    int32_t _altM;

    float _speedKmh;
    float _climbMs;
    float _headingDeg;

    uint8_t _aircraftType;
    bool _onlineTracking;

    // Tracking beacon scheduler.
    uint32_t _beaconIntervalMs;
    uint32_t _lastBeaconMs;

    // Retry backoff when the channel is busy.
    uint32_t _nextSendAttemptMs;

    FanetTrackingCallback _trackingCb;
    FanetRawCallback _rawCb;

    void handleReceived();

    void decodeAndDispatch(
        const uint8_t* buf,
        size_t len,
        float rssi,
        float snr
    );

    size_t encodeHeader(
        uint8_t* out,
        uint8_t type,
        bool forward
    ) const;

    size_t encodeTracking(
        uint8_t* out,
        size_t maxLen
    ) const;
};
