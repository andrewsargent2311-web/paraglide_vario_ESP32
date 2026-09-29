// Flarm.h
//
// RECEIVE-ONLY FLARM decoder skeleton.
//
// FLARM's actual air protocol is proprietary and unpublished by FLARM
// Technology GmbH. Everything the open-source world can do with it comes
// from the OGN (Open Glider Network) project's reverse-engineering work,
// used for passive reception/display only -- not for transmitting
// FLARM-compatible frames. This file follows that same scope: it only
// ever configures the radio to listen and hands you decoded (or raw,
// pre-decode) frames. There is no send() here, intentionally.
//
// *** WHAT'S NOT FILLED IN, AND WHY ***
// The bit-level demodulation (GFSK at FLARM's specific bitrate/deviation),
// the data-whitening LFSR (polynomial + seed), and the CRC used to
// validate a decoded frame are NOT included below with concrete values.
// I don't have verified-correct constants for these to hand you -- and
// getting them subtly wrong wouldn't throw an error, it would just make
// this silently decode garbage and *look* like it's working. Pull the
// exact radio config (bitrate/deviation/bandwidth), whitening
// polynomial/seed, and CRC parameters from OGN's public decoder source
// (the ogn-rf / rtlsdr-ogn / glidernet projects) and drop them into the
// TODOs marked below. Treat everything in this file as plumbing, not a
// verified protocol implementation.

#pragma once

#include <Arduino.h>
#include "Sx126xLink.h"

struct FlarmFrame {
    uint32_t id;         // aircraft ID/address, once decoded -- format TODO
    double   latitude;   // TODO: decode from raw frame
    double   longitude;  // TODO: decode from raw frame
    int32_t  altitudeM;  // TODO: decode from raw frame
    float    rssi;
    float    snr;
    bool     crcValid;   // set true only once the real CRC check (TODO) passes
};

// Raw callback: fires for every received-and-descrambled frame, before
// any attempt at field decoding. Useful for bring-up/debugging against
// known reference captures while you fill in the TODOs.
typedef void (*FlarmRawCallback)(const uint8_t* rawBytes, size_t len,
                                  float rssi, float snr);

// Decoded callback: fires once you've filled in the field-decode TODOs
// in Flarm.cpp and are confident the CRC check is correct.
typedef void (*FlarmFrameCallback)(const FlarmFrame& frame);

class FlarmReceiver {
public:
    explicit FlarmReceiver(Sx126xLink& link);

    // Switches the SX1262 into FSK/GFSK mode at FLARM's frequency.
    // bitrateKbps/deviationKHz/rxBwKHz are left as parameters rather than
    // hardcoded -- fill in the correct values (from OGN's decoder source)
    // when you call this, rather than trusting a default here.
    bool begin(float freqMHz, float bitrateKbps, float deviationKHz, float rxBwKHz);

    // Call frequently from loop(), same as FanetStack::update().
    void update();

    void onRawFrame(FlarmRawCallback cb) { _rawCb = cb; }
    void onDecodedFrame(FlarmFrameCallback cb) { _decodedCb = cb; }

private:
    Sx126xLink& _link;
    FlarmRawCallback    _rawCb;
    FlarmFrameCallback  _decodedCb;

    // TODO: implement using the verified whitening polynomial/seed.
    void dewhiten(uint8_t* buf, size_t len);

    // TODO: implement using the verified CRC parameters. Return true
    // only if the frame's checksum actually validates.
    bool checkCrc(const uint8_t* buf, size_t len);

    // TODO: implement field extraction once whitening/CRC are solid.
    bool decodeFrame(const uint8_t* buf, size_t len, FlarmFrame& out);
};
