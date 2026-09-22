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
// Decoded FANET Service (type 4) weather-station payload.
//
// Only the fields this flight computer currently has a use for are
// decoded: wind (heading/speed/gust), position, and temperature -- that's
// what drives the Weather page. Humidity, barometric pressure,
// state-of-charge, and the internet-gateway/remote-config-advertisement
// flags are all part of the Service packet format but are NOT decoded
// into this struct (see FanetStack::decodeServiceAndDispatch() if a
// future feature needs them -- their bytes are still correctly skipped
// so later fields decode right, just not surfaced here). onRawPacket()
// still sees every Service packet regardless, decoded or not.
//
// A Service packet without the wind bit set (e.g. a temperature-only or
// gateway-only station) is NOT dispatched to onWeather() at all -- see
// decodeServiceAndDispatch().
// -----------------------------------------------------------------------------

struct FanetWeather {

    double latitude;
    double longitude;

    float windHeadingDeg;
    float windSpeedKmh;
    float windGustKmh;

    bool  hasTemperature;
    float temperatureC;
};

// -----------------------------------------------------------------------------
// Decoded FANET Message (type 3) payload.
//
// Broadcast only -- this implementation's encodeHeader() only ever
// writes the plain 4-byte broadcast header (see Fanet.cpp), so there is
// no addressed/unicast messaging to a specific pilot. Only subtype 0
// ("Normal Message") is defined by the protocol; any other subtype is
// still decoded (text may or may not be meaningful) rather than
// dropped, so the application layer can decide what to do with it.
//
// text is NOT necessarily null-terminated by the sender per the FANET
// spec ("8bit String, of arbitrary length") -- decodeAndDispatch()
// always null-terminates it here regardless, truncating to
// FANET_MAX_MESSAGE_LEN if the received text is longer.
// -----------------------------------------------------------------------------

#define FANET_MAX_MESSAGE_LEN 40

struct FanetMessage {

    uint8_t subtype;

    char text[FANET_MAX_MESSAGE_LEN + 1];
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

typedef void (*FanetWeatherCallback)(
    const FanetAddress& src,
    const FanetWeather& pkt,
    float rssi,
    float snr
);

typedef void (*FanetMessageCallback)(
    const FanetAddress& src,
    const FanetMessage& pkt,
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

    void onWeather(FanetWeatherCallback cb) {
        _weatherCb = cb;
    }

    void onMessage(FanetMessageCallback cb) {
        _messageCb = cb;
    }

    void onRawPacket(FanetRawCallback cb) {
        _rawCb = cb;
    }

    // Immediately transmit the current tracking packet.
    // Returns true if the packet was handed to the radio.
    bool sendTrackingNow();

    // Immediately broadcasts a Type-3 Message packet (subtype 0, Normal
    // Message) containing text. Truncated to FANET_MAX_MESSAGE_LEN if
    // longer. Returns true if the packet was handed to the radio --
    // like sendTrackingNow(), a single attempt (respects the radio's
    // own CAD/channel-busy check, but does not queue or retry on
    // failure; the caller decides whether to try again).
    bool sendMessageNow(const char* text);

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
    FanetWeatherCallback _weatherCb;
    FanetMessageCallback _messageCb;
    FanetRawCallback _rawCb;

    void handleReceived();

    void decodeAndDispatch(
        const uint8_t* buf,
        size_t len,
        float rssi,
        float snr
    );

    // Service (type 4) payload decode -- called from decodeAndDispatch()
    // once the MAC header/source address have already been stripped off.
    void decodeServiceAndDispatch(
        const uint8_t* payload,
        size_t payloadLen,
        const FanetAddress& src,
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
