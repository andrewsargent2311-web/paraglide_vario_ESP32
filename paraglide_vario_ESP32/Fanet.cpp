// Fanet.cpp

#include "Fanet.h"

#include <string.h>
#include <math.h>

// ============================================================================
// FANET TRACKING SCALING
// ============================================================================
//
// Absolute coordinates:
//
//   Latitude  = signed24 / 93206
//   Longitude = signed24 / 46603
//
// Tracking:
//
//   Speed:
//     bit 7    = scaling
//     bits 0-6 = value
//     value = 0.5 km/h
//     scaling = x5
//
//   Climb:
//     bit 7    = scaling
//     bits 0-6 = signed value
//     value = 0.1 m/s
//     scaling = x5
//
//   Altitude:
//     bit 15     = online tracking
//     bits 12-14 = aircraft type
//     bit 11     = altitude scaling
//     bits 0-10  = altitude
//     scaling = x4
//
//   Heading:
//     360 / 256 degrees per LSB
//
// ============================================================================

static constexpr double FANET_LAT_SCALE = 93206.0;
static constexpr double FANET_LON_SCALE = 46603.0;

static constexpr float FANET_SPEED_STEP_KMH = 0.5f;
static constexpr float FANET_CLIMB_STEP_MS = 0.1f;
static constexpr float FANET_HEADING_STEP_DEG = 360.0f / 256.0f;

// Maximum values represented by the FANET compact fields.
static constexpr float FANET_MAX_SPEED_KMH = 317.5f;
static constexpr float FANET_MAX_CLIMB_MS = 31.5f;
static constexpr int32_t FANET_MAX_ALT_M = 8188;

// ============================================================================
// Helpers
// ============================================================================

// Encode a signed 24-bit integer little-endian.
static void writeS24LE(
    uint8_t* dst,
    int32_t value
) {
    dst[0] = (uint8_t)(value & 0xFF);
    dst[1] = (uint8_t)((value >> 8) & 0xFF);
    dst[2] = (uint8_t)((value >> 16) & 0xFF);
}

// Decode a signed 24-bit little-endian integer.
static int32_t readS24LE(
    const uint8_t* src
) {
    int32_t value =
        ((int32_t)src[0]) |
        ((int32_t)src[1] << 8) |
        ((int32_t)src[2] << 16);

    // Sign extend 24-bit two's-complement.
    if (value & 0x00800000) {
        value |= 0xFF000000;
    }

    return value;
}

// Decode signed 7-bit two's complement.
static int8_t decodeSigned7(
    uint8_t value
) {
    value &= 0x7F;

    if (value & 0x40) {
        value |= 0x80;
    }

    return (int8_t)value;
}

// ============================================================================
// Constructor
// ============================================================================

FanetStack::FanetStack(
    Sx126xLink& link,
    const FanetAddress& myAddress
)
    : _link(link)
    , _myAddress(myAddress)

    , _lat(0.0)
    , _lon(0.0)
    , _altM(0)

    , _speedKmh(0.0f)
    , _climbMs(0.0f)
    , _headingDeg(0.0f)

    , _aircraftType(1)       // Paraglider
    , _onlineTracking(true)

    , _beaconIntervalMs(5000UL)
    , _lastBeaconMs(0)
    , _nextSendAttemptMs(0)

    , _trackingCb(nullptr)
    , _rawCb(nullptr)
{
}

// ============================================================================
// Begin
// ============================================================================

void FanetStack::begin() {

    uint32_t now = millis();

    _lastBeaconMs = now;
    _nextSendAttemptMs = now;

    Serial.printf(
        "[FANET] Address %02X:%04X\n",
        _myAddress.manufacturer,
        _myAddress.id
    );
}

// ============================================================================
// Position update
// ============================================================================

void FanetStack::setPosition(
    double lat,
    double lon,
    int32_t altM,
    float speedKmh,
    float climbMs,
    float headingDeg
) {

    _lat = lat;
    _lon = lon;
    _altM = altM;

    _speedKmh = speedKmh;
    _climbMs = climbMs;
    _headingDeg = headingDeg;
}

// ============================================================================
// Main update
// ============================================================================

void FanetStack::update() {

    // Always service the receiver first.
    handleReceived();

    const uint32_t now = millis();

    // Wait until the next normal beacon interval.
    if ((uint32_t)(now - _lastBeaconMs) < _beaconIntervalMs) {
        return;
    }

    // If a previous attempt found the channel busy, wait for the
    // small randomized retry delay before performing CAD again.
    if ((int32_t)(now - _nextSendAttemptMs) < 0) {
        return;
    }

    if (sendTrackingNow()) {

        // Successful packet transmission.
        _lastBeaconMs = now;

        _nextSendAttemptMs =
            now + _beaconIntervalMs;

    } else {

        // Channel busy or radio unavailable.
        //
        // Do not perform CAD continuously on every loop pass.
        // A short random delay prevents multiple FANET devices from
        // repeatedly colliding after finding the channel busy.
        uint32_t backoffMs =
            (uint32_t)random(50L, 251L);

        _nextSendAttemptMs = now + backoffMs;
    }
}

// ============================================================================
// RX processing
// ============================================================================

void FanetStack::handleReceived() {

    RadioEvent ev = _link.poll();

    if (ev == RadioEvent::RX_DONE) {

        decodeAndDispatch(
            _link.receivedData(),
            _link.receivedLength(),
            _link.lastRssi(),
            _link.lastSnr()
        );
    }
}

// ============================================================================
// FANET MAC HEADER
// ============================================================================
//
// Byte 0:
//
//   bit 7     Extended Header
//   bit 6     Forward
//   bits 5-0  Type
//
// Byte 1:
//
//   Manufacturer
//
// Bytes 2-3:
//
//   Unique ID, little-endian
//
// ============================================================================

size_t FanetStack::encodeHeader(
    uint8_t* out,
    uint8_t type,
    bool forward
) const {

    // Broadcast/non-extended FANET header.
    //
    // IMPORTANT:
    // bit 7 = Extended Header
    // bit 6 = Forward
    //
    // The previous implementation had these two bits reversed.

    out[0] =
        (type & 0x3F) |
        (forward ? 0x40 : 0x00);

    out[1] = _myAddress.manufacturer;

    out[2] =
        (uint8_t)(_myAddress.id & 0xFF);

    out[3] =
        (uint8_t)((_myAddress.id >> 8) & 0xFF);

    return 4;
}

// ============================================================================
// Encode speed
// ============================================================================
//
// Unscaled:
//   0.5 km/h per LSB
//   maximum 63.5 km/h
//
// Scaled:
//   0.5 km/h * 5 per LSB
//   maximum 317.5 km/h
//
// bit 7 selects scaling.
// bits 0-6 contain the value.
// ============================================================================

static uint8_t encodeSpeed(
    float speedKmh
) {

    if (!isfinite(speedKmh) || speedKmh < 0.0f) {
        speedKmh = 0.0f;
    }

    speedKmh =
        constrain(
            speedKmh,
            0.0f,
            FANET_MAX_SPEED_KMH
        );

    // Use normal resolution whenever possible.
    if (speedKmh <= 63.5f) {

        int value =
            (int)lroundf(
                speedKmh / FANET_SPEED_STEP_KMH
            );

        value = constrain(value, 0, 127);

        return (uint8_t)value;
    }

    // Scaled mode.
    int value =
        (int)lroundf(
            speedKmh /
            (FANET_SPEED_STEP_KMH * 5.0f)
        );

    value = constrain(value, 0, 127);

    return (uint8_t)(0x80 | value);
}

// ============================================================================
// Decode speed
// ============================================================================

static float decodeSpeed(
    uint8_t value
) {

    bool scaled =
        (value & 0x80) != 0;

    uint8_t raw =
        value & 0x7F;

    float speed =
        raw * FANET_SPEED_STEP_KMH;

    if (scaled) {
        speed *= 5.0f;
    }

    return speed;
}

// ============================================================================
// Encode climb
// ============================================================================
//
// Normal:
//   +/- 6.3 m/s
//
// Scaled:
//   +/- 31.5 m/s
//
// bit 7 = scaling
// bits 0-6 = signed 7-bit two's complement
// ============================================================================

static uint8_t encodeClimb(
    float climbMs
) {

    if (!isfinite(climbMs)) {
        climbMs = 0.0f;
    }

    climbMs =
        constrain(
            climbMs,
            -FANET_MAX_CLIMB_MS,
            FANET_MAX_CLIMB_MS
        );

    // Normal resolution.
    if (fabsf(climbMs) <= 6.3f) {

        int value =
            (int)lroundf(
                climbMs / FANET_CLIMB_STEP_MS
            );

        value = constrain(value, -63, 63);

        return (uint8_t)(value & 0x7F);
    }

    // Scaled resolution.
    int value =
        (int)lroundf(
            climbMs /
            (FANET_CLIMB_STEP_MS * 5.0f)
        );

    value = constrain(value, -63, 63);

    return (uint8_t)(
        0x80 |
        (value & 0x7F)
    );
}

// ============================================================================
// Decode climb
// ============================================================================

static float decodeClimb(
    uint8_t value
) {

    bool scaled =
        (value & 0x80) != 0;

    int8_t raw =
        decodeSigned7(value);

    float climb =
        raw * FANET_CLIMB_STEP_MS;

    if (scaled) {
        climb *= 5.0f;
    }

    return climb;
}

// ============================================================================
// Encode tracking payload
// ============================================================================

size_t FanetStack::encodeTracking(
    uint8_t* out,
    size_t maxLen
) const {

    // Tracking payload without optional turn-rate/QNE fields.
    //
    // 3 lat
    // 3 lon
    // 2 altitude/type
    // 1 speed
    // 1 climb
    // 1 heading
    //
    // = 11 bytes

    if (out == nullptr || maxLen < 11) {
        return 0;
    }

    size_t i = 0;

    // ------------------------------------------------------------------------
    // Latitude
    // ------------------------------------------------------------------------

    double lat =
        constrain(
            _lat,
            -90.0,
            90.0
        );

    int32_t latRaw =
        (int32_t)llround(
            lat * FANET_LAT_SCALE
        );

    // 24-bit signed range.
    latRaw =
        constrain(
            latRaw,
            -8388608L,
            8388607L
        );

    writeS24LE(
        &out[i],
        latRaw
    );

    i += 3;

    // ------------------------------------------------------------------------
    // Longitude
    // ------------------------------------------------------------------------

    double lon =
        constrain(
            _lon,
            -180.0,
            180.0
        );

    int32_t lonRaw =
        (int32_t)llround(
            lon * FANET_LON_SCALE
        );

    lonRaw =
        constrain(
            lonRaw,
            -8388608L,
            8388607L
        );

    writeS24LE(
        &out[i],
        lonRaw
    );

    i += 3;

    // ------------------------------------------------------------------------
    // Altitude / aircraft type / flags
    // ------------------------------------------------------------------------

    int32_t altitude =
        constrain(
            _altM,
            0,
            FANET_MAX_ALT_M
        );

    bool altitudeScaled =
        altitude > 2047;

    uint16_t altitudeRaw;

    if (altitudeScaled) {

        altitudeRaw =
            (uint16_t)lroundf(
                altitude / 4.0f
            );

        altitudeRaw =
            constrain(
                altitudeRaw,
                0,
                2047
            );

    } else {

        altitudeRaw =
            (uint16_t)altitude;
    }

    uint16_t altitudeType =
        altitudeRaw & 0x07FF;

    // Bit 11 = altitude scaling.
    if (altitudeScaled) {
        altitudeType |= 0x0800;
    }

    // Bits 12-14 = aircraft type.
    altitudeType |=
        (uint16_t)(
            (_aircraftType & 0x07)
            << 12
        );

    // Bit 15 = online tracking.
    if (_onlineTracking) {
        altitudeType |= 0x8000;
    }

    // Little-endian.
    out[i++] =
        (uint8_t)(
            altitudeType & 0xFF
        );

    out[i++] =
        (uint8_t)(
            altitudeType >> 8
        );

    // ------------------------------------------------------------------------
    // Speed
    // ------------------------------------------------------------------------

    out[i++] =
        encodeSpeed(_speedKmh);

    // ------------------------------------------------------------------------
    // Climb
    // ------------------------------------------------------------------------

    out[i++] =
        encodeClimb(_climbMs);

    // ------------------------------------------------------------------------
    // Heading
    // ------------------------------------------------------------------------

    float heading =
        _headingDeg;

    if (!isfinite(heading)) {
        heading = 0.0f;
    }

    while (heading < 0.0f) {
        heading += 360.0f;
    }

    while (heading >= 360.0f) {
        heading -= 360.0f;
    }

    int headingRaw =
        (int)lroundf(
            heading / FANET_HEADING_STEP_DEG
        );

    headingRaw &= 0xFF;

    out[i++] =
        (uint8_t)headingRaw;

    return i;
}

// ============================================================================
// Send tracking packet
// ============================================================================

bool FanetStack::sendTrackingNow() {

    uint8_t packet[32];

    size_t headerLen =
        encodeHeader(
            packet,
            FANET_TYPE_TRACKING,
            false
        );

    size_t payloadLen =
        encodeTracking(
            packet + headerLen,
            sizeof(packet) - headerLen
        );

    if (payloadLen == 0) {
        return false;
    }

    size_t packetLen =
        headerLen + payloadLen;

    return _link.send(
        packet,
        packetLen
    );
}

// ============================================================================
// Decode received FANET packet
// ============================================================================

void FanetStack::decodeAndDispatch(
    const uint8_t* buf,
    size_t len,
    float rssi,
    float snr
) {

    if (buf == nullptr || len < 4) {
        return;
    }

    // ------------------------------------------------------------------------
    // MAC header
    // ------------------------------------------------------------------------

    uint8_t header =
        buf[0];

    // Correct FANET bit allocation:
    //
    // bit 7 = Extended Header
    // bit 6 = Forward
    // bits 5-0 = Type

    bool extended =
        (header & 0x80) != 0;

    bool forward =
        (header & 0x40) != 0;

    uint8_t type =
        header & 0x3F;

    (void)forward;

    // The current implementation handles broadcast/non-extended packets.
    //
    // Extended headers can contain ACK, destination, signature and
    // forwarding information. Do not attempt to interpret those packets
    // as normal four-byte headers.

    if (extended) {

        if (_rawCb) {

            FanetAddress src;

            src.manufacturer =
                buf[1];

            src.id =
                (uint16_t)buf[2] |
                ((uint16_t)buf[3] << 8);

            _rawCb(
                type,
                src,
                nullptr,
                0,
                rssi,
                snr
            );
        }

        return;
    }

    // ------------------------------------------------------------------------
    // Source address
    // ------------------------------------------------------------------------

    FanetAddress src;

    src.manufacturer =
        buf[1];

    src.id =
        (uint16_t)buf[2] |
        ((uint16_t)buf[3] << 8);

    // ------------------------------------------------------------------------
    // Payload
    // ------------------------------------------------------------------------

    const uint8_t* payload =
        buf + 4;

    size_t payloadLen =
        len - 4;

    // Raw callback gets every non-extended packet.
    if (_rawCb) {

        _rawCb(
            type,
            src,
            payload,
            payloadLen,
            rssi,
            snr
        );
    }

    // ------------------------------------------------------------------------
    // Tracking packet
    // ------------------------------------------------------------------------

    if (type != FANET_TYPE_TRACKING) {
        return;
    }

    // Minimum tracking payload:
    //
    // lat 3
    // lon 3
    // type/alt 2
    // speed 1
    // climb 1
    // heading 1
    //
    // = 11 bytes

    if (payloadLen < 11) {
        return;
    }

    FanetTracking pkt{};

    // ------------------------------------------------------------------------
    // Latitude
    // ------------------------------------------------------------------------

    int32_t latRaw =
        readS24LE(
            &payload[0]
        );

    pkt.latitude =
        ((double)latRaw) /
        FANET_LAT_SCALE;

    // ------------------------------------------------------------------------
    // Longitude
    // ------------------------------------------------------------------------

    int32_t lonRaw =
        readS24LE(
            &payload[3]
        );

    pkt.longitude =
        ((double)lonRaw) /
        FANET_LON_SCALE;

    // ------------------------------------------------------------------------
    // Altitude / type / flags
    // ------------------------------------------------------------------------

    uint16_t altitudeType =
        (uint16_t)payload[6] |
        ((uint16_t)payload[7] << 8);

    pkt.onlineTracking =
        (altitudeType & 0x8000) != 0;

    pkt.aircraftType =
        (uint8_t)(
            (altitudeType >> 12) &
            0x07
        );

    bool altitudeScaled =
        (altitudeType & 0x0800) != 0;

    uint16_t altitudeRaw =
        altitudeType & 0x07FF;

    if (altitudeScaled) {

        pkt.altitudeM =
            (int32_t)altitudeRaw * 4;

    } else {

        pkt.altitudeM =
            (int32_t)altitudeRaw;
    }

    // ------------------------------------------------------------------------
    // Speed
    // ------------------------------------------------------------------------

    pkt.speedKmh =
        decodeSpeed(
            payload[8]
        );

    // ------------------------------------------------------------------------
    // Climb
    // ------------------------------------------------------------------------

    pkt.climbMs =
        decodeClimb(
            payload[9]
        );

    // ------------------------------------------------------------------------
    // Heading
    // ------------------------------------------------------------------------

    pkt.headingDeg =
        ((float)payload[10]) *
        FANET_HEADING_STEP_DEG;

    // ------------------------------------------------------------------------
    // Optional turn rate
    // ------------------------------------------------------------------------

    if (payloadLen >= 12) {

        pkt.hasTurnRate = true;

        uint8_t value =
            payload[11];

        bool scaled =
            (value & 0x80) != 0;

        int8_t raw =
            decodeSigned7(value);

        pkt.turnRateDegS =
            raw * 0.25f;

        if (scaled) {
            pkt.turnRateDegS *= 4.0f;
        }

    } else {

        pkt.hasTurnRate = false;
        pkt.turnRateDegS = 0.0f;
    }

    // ------------------------------------------------------------------------
    // Deliver decoded tracking packet.
    // ------------------------------------------------------------------------

    if (_trackingCb) {

        _trackingCb(
            src,
            pkt,
            rssi,
            snr
        );
    }
}
