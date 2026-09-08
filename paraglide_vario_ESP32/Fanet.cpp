// Fanet.cpp
#include "Fanet.h"
#include <string.h>
#include <math.h>

// ---- Scaling constants --------------------------------------------------
// TODO: VERIFY these against the current FANET protocol spec before
// trusting decoded output. These are the commonly-cited values from the
// reference implementation but I'm flagging them explicitly rather than
// asserting them as fact -- a wrong scale factor here fails silently
// (you'll get a plausible-looking but wrong position), which is worse
// than a crash.
static const double  FANET_LATLON_SCALE   = 1.0 / 93206.755;  // deg per LSB, 3-byte signed field -- VERIFY
static const float   FANET_SPEED_SCALE    = 0.5f;             // km/h per LSB -- VERIFY
static const float   FANET_CLIMB_SCALE    = 0.1f;             // m/s per LSB -- VERIFY
static const float   FANET_HEADING_SCALE  = 360.0f / 256.0f;  // deg per LSB -- VERIFY

FanetStack::FanetStack(Sx126xLink& link, const FanetAddress& myAddress)
    : _link(link)
    , _myAddress(myAddress)
    , _lat(0), _lon(0), _altM(0)
    , _speedKmh(0), _climbMs(0), _headingDeg(0)
    , _aircraftType(1) // 1 = paraglider, per spec table -- VERIFY value for your craft
    , _onlineTracking(true)
    , _beaconIntervalMs(5000)
    , _lastBeaconMs(0)
    , _trackingCb(nullptr)
    , _rawCb(nullptr)
{}

void FanetStack::begin() {
    _lastBeaconMs = millis();
}

void FanetStack::setPosition(double lat, double lon, int32_t altM,
                              float speedKmh, float climbMs, float headingDeg) {
    _lat = lat;
    _lon = lon;
    _altM = altM;
    _speedKmh = speedKmh;
    _climbMs = climbMs;
    _headingDeg = headingDeg;
}

void FanetStack::update() {
    handleReceived();

    uint32_t now = millis();
    if (now - _lastBeaconMs >= _beaconIntervalMs) {
        if (sendTrackingNow()) {
            _lastBeaconMs = now;
        }
        // if send() fails (channel busy / tx already in flight), we just
        // retry on the next update() call rather than resetting the
        // schedule -- keeps beacon timing close to nominal under
        // moderate channel load.
    }
}

void FanetStack::handleReceived() {
    RadioEvent ev = _link.poll();
    if (ev == RadioEvent::RX_DONE) {
        decodeAndDispatch(_link.receivedData(), _link.receivedLength(),
                           _link.lastRssi(), _link.lastSnr());
    }
    // TX_DONE / TIMEOUT / CRC_ERROR / NONE: nothing else to do here,
    // Sx126xLink already re-arms receive internally.
}

// ---- Header ---------------------------------------------------------
// FANET header (1-byte form, no extended/signature header):
//   bit 7    : forward flag
//   bit 6    : extended header present (always 0 here -- not implemented)
//   bits 5-0 : message type
//   followed by 3-byte source address (manufacturer, id_lo, id_hi)
size_t FanetStack::encodeHeader(uint8_t* out, uint8_t type, bool forward) const {
    out[0] = (type & 0x3F) | (forward ? 0x80 : 0x00);
    out[1] = _myAddress.manufacturer;
    out[2] = (uint8_t)(_myAddress.id & 0xFF);
    out[3] = (uint8_t)((_myAddress.id >> 8) & 0xFF);
    return 4;
}

// ---- Tracking payload (type 1) --------------------------------------
// Layout below follows the general shape of the spec's tracking packet:
// 3-byte lat, 3-byte lon, 2-byte alt+aircraft-type/online-tracking flags,
// 1-byte speed, 1-byte climb, 1-byte heading, optional turn rate byte.
// VERIFY bit-for-bit against the spec -- flagged in header comment above.
size_t FanetStack::encodeTracking(uint8_t* out, size_t maxLen) const {
    if (maxLen < 11) return 0;

    size_t i = 0;

    int32_t latRaw = (int32_t)lround(_lat / FANET_LATLON_SCALE);
    out[i++] = (uint8_t)(latRaw & 0xFF);
    out[i++] = (uint8_t)((latRaw >> 8) & 0xFF);
    out[i++] = (uint8_t)((latRaw >> 16) & 0xFF);

    int32_t lonRaw = (int32_t)lround(_lon / FANET_LATLON_SCALE);
    out[i++] = (uint8_t)(lonRaw & 0xFF);
    out[i++] = (uint8_t)((lonRaw >> 8) & 0xFF);
    out[i++] = (uint8_t)((lonRaw >> 16) & 0xFF);

    // altitude: 12 bits + 4 bits packed with aircraft type -- simplified
    // here to a plain 12-bit altitude clamp + separate type byte for
    // clarity. Replace with the exact bit-packed layout once verified.
    uint16_t altClamped = (uint16_t)constrain(_altM, 0, 4095);
    out[i++] = (uint8_t)(altClamped & 0xFF);
    out[i++] = (uint8_t)(((altClamped >> 8) & 0x0F) |
                          ((_aircraftType & 0x07) << 4) |
                          (_onlineTracking ? 0x80 : 0x00));

    out[i++] = (uint8_t)constrain((int)lround(_speedKmh / FANET_SPEED_SCALE), 0, 255);

    int32_t climbRaw = (int32_t)lround(_climbMs / FANET_CLIMB_SCALE);
    out[i++] = (uint8_t)constrain(climbRaw, -128, 127);

    out[i++] = (uint8_t)constrain((int)lround(_headingDeg / FANET_HEADING_SCALE), 0, 255);

    return i;
}

bool FanetStack::sendTrackingNow() {
    uint8_t buf[32];
    size_t hlen = encodeHeader(buf, FANET_TYPE_TRACKING, false);
    size_t plen = encodeTracking(buf + hlen, sizeof(buf) - hlen);
    if (plen == 0) return false;
    return _link.send(buf, hlen + plen);
}

void FanetStack::decodeAndDispatch(const uint8_t* buf, size_t len,
                                    float rssi, float snr) {
    if (len < 4) return; // shorter than a header, drop

    uint8_t typeByte = buf[0];
    uint8_t type = typeByte & 0x3F;
    bool extended = typeByte & 0x40;
    if (extended) {
        // Extended header (destination address / signature) not handled
        // by this skeleton -- drop rather than misparse.
        return;
    }

    FanetAddress src;
    src.manufacturer = buf[1];
    src.id = (uint16_t)buf[2] | ((uint16_t)buf[3] << 8);

    const uint8_t* payload = buf + 4;
    size_t payloadLen = len - 4;

    if (_rawCb) {
        _rawCb(type, src, payload, payloadLen, rssi, snr);
    }

    if (type == FANET_TYPE_TRACKING && _trackingCb && payloadLen >= 11) {
        FanetTracking pkt;

        int32_t latRaw = payload[0] | (payload[1] << 8) | (payload[2] << 16);
        if (latRaw & 0x800000) latRaw |= 0xFF000000; // sign-extend 24-bit
        pkt.latitude = latRaw * FANET_LATLON_SCALE;

        int32_t lonRaw = payload[3] | (payload[4] << 8) | (payload[5] << 16);
        if (lonRaw & 0x800000) lonRaw |= 0xFF000000;
        pkt.longitude = lonRaw * FANET_LATLON_SCALE;

        uint16_t altBits = payload[6] | ((payload[7] & 0x0F) << 8);
        pkt.altitudeM = altBits;
        pkt.aircraftType = (payload[7] >> 4) & 0x07;
        pkt.onlineTracking = payload[7] & 0x80;

        pkt.speedKmh = payload[8] * FANET_SPEED_SCALE;

        int8_t climbRaw = (int8_t)payload[9];
        pkt.climbMs = climbRaw * FANET_CLIMB_SCALE;

        pkt.headingDeg = payload[10] * FANET_HEADING_SCALE;

        pkt.hasTurnRate = false;
        pkt.turnRateDegS = 0;

        _trackingCb(src, pkt, rssi, snr);
    }
}
