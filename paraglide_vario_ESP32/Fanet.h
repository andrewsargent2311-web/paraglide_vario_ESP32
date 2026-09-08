// Fanet.h
//
// Minimal FANET protocol implementation sitting on top of Sx126xLink.
//
// IMPORTANT: the byte-level field layout and scaling factors below follow
// the publicly documented FANET protocol (as implemented in the reference
// fanet-stm32 firmware / protocol spec). Before relying on this for real
// interop with other FANET devices, cross-check every field offset and
// scaling constant against the current spec/reference source -- protocol
// details are exactly the kind of thing that's easy to get subtly wrong
// from memory, and a subtle mismatch here won't throw an error, it'll
// just silently produce garbage positions on someone else's screen.
// Treat FANET_TYPE_* and the Tracking payload layout below as a starting
// skeleton, not a verified-correct implementation.

#pragma once

#include <Arduino.h>
#include "Sx126xLink.h"

// ---- FANET message types ----------------------------------------------
enum FanetType : uint8_t {
    FANET_TYPE_TRACKING        = 0x01,
    FANET_TYPE_NAME            = 0x02,
    FANET_TYPE_MESSAGE         = 0x03,
    FANET_TYPE_SERVICE         = 0x04,
    FANET_TYPE_LANDMARK        = 0x05,
    FANET_TYPE_REMOTE_CONFIG   = 0x06,
    FANET_TYPE_GROUND_TRACKING = 0x07,
    FANET_TYPE_HW_INFO         = 0x08,
    FANET_TYPE_THERMAL         = 0x09,
};

// 3-byte FANET address: {manufacturer, id_hi, id_lo} per spec.
struct FanetAddress {
    uint8_t manufacturer;
    uint16_t id;
};

// Decoded tracking (type 1) payload -- the "where is this aircraft" beacon.
struct FanetTracking {
    double  latitude;      // degrees
    double  longitude;     // degrees
    int32_t altitudeM;     // metres
    float   speedKmh;
    float   climbMs;
    float   headingDeg;
    uint8_t aircraftType;  // per spec: paraglider, hangglider, balloon, etc.
    bool    onlineTracking;
    bool    hasTurnRate;
    float   turnRateDegS;  // valid only if hasTurnRate
};

// Callback signature for received-and-decoded tracking packets.
typedef void (*FanetTrackingCallback)(const FanetAddress& src,
                                       const FanetTracking& pkt,
                                       float rssi, float snr);

// Callback for any other packet type your app wants to see raw
// (name, message, service, etc.) -- decode as needed for your use case.
typedef void (*FanetRawCallback)(uint8_t type, const FanetAddress& src,
                                  const uint8_t* payload, size_t len,
                                  float rssi, float snr);

class FanetStack {
public:
    FanetStack(Sx126xLink& link, const FanetAddress& myAddress);

    // Call once, after link.begin() has already succeeded.
    void begin();

    // Call frequently from loop(). Drives both RX processing and the
    // periodic tracking beacon schedule.
    void update();

    // Configure this device's own state used to build outgoing tracking
    // beacons. Call whenever fresh GPS/baro data is available.
    void setPosition(double lat, double lon, int32_t altM,
                      float speedKmh, float climbMs, float headingDeg);
    void setAircraftType(uint8_t type) { _aircraftType = type; }
    void setBeaconIntervalMs(uint32_t ms) { _beaconIntervalMs = ms; }
    void setOnlineTracking(bool enabled) { _onlineTracking = enabled; }

    void onTracking(FanetTrackingCallback cb) { _trackingCb = cb; }
    void onRawPacket(FanetRawCallback cb) { _rawCb = cb; }

    // Build and (CSMA-permitting) send this device's current tracking
    // state immediately, bypassing the periodic schedule.
    bool sendTrackingNow();

private:
    Sx126xLink&   _link;
    FanetAddress  _myAddress;

    // Current outgoing state
    double  _lat, _lon;
    int32_t _altM;
    float   _speedKmh, _climbMs, _headingDeg;
    uint8_t _aircraftType;
    bool    _onlineTracking;

    uint32_t _beaconIntervalMs;
    uint32_t _lastBeaconMs;

    FanetTrackingCallback _trackingCb;
    FanetRawCallback      _rawCb;

    void handleReceived();
    void decodeAndDispatch(const uint8_t* buf, size_t len, float rssi, float snr);

    size_t encodeHeader(uint8_t* out, uint8_t type, bool forward) const;
    size_t encodeTracking(uint8_t* out, size_t maxLen) const;
};
