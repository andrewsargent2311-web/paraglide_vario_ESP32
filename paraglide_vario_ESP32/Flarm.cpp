// Flarm.cpp
#include "Flarm.h"

FlarmReceiver::FlarmReceiver(Sx126xLink& link)
    : _link(link)
    , _rawCb(nullptr)
    , _decodedCb(nullptr)
{}

bool FlarmReceiver::begin(float freqMHz, float bitrateKbps,
                           float deviationKHz, float rxBwKHz) {
    // NOTE: this reconfigures the SAME radio used for FANET into FSK
    // mode. If you need to receive both simultaneously you need a second
    // radio (a second HT-RA62) since one SX1262 can only run one modem
    // config at a time -- worth deciding that up front rather than
    // discovering it after wiring everything to one module.
    //
    // Sx126xLink currently only exposes the LoRa begin() path from our
    // earlier code -- you'll want to extend it (or add a sibling method)
    // to call RadioLib's SX1262::beginFSK(...) with these parameters,
    // plus setDataShaping()/setRxBandwidth() etc. as needed to match
    // FLARM's actual modulation. Left unimplemented here deliberately --
    // fill in once you've sourced correct values from OGN's decoder.
    (void)freqMHz; (void)bitrateKbps; (void)deviationKHz; (void)rxBwKHz;
    return false; // placeholder until the FSK path above is wired up
}

void FlarmReceiver::update() {
    RadioEvent ev = _link.poll();
    if (ev != RadioEvent::RX_DONE) {
        return;
    }

    // Work on a local copy since dewhiten() mutates in place.
    uint8_t buf[256];
    size_t len = _link.receivedLength();
    if (len > sizeof(buf)) len = sizeof(buf);
    memcpy(buf, _link.receivedData(), len);

    dewhiten(buf, len);

    if (_rawCb) {
        _rawCb(buf, len, _link.lastRssi(), _link.lastSnr());
    }

    if (!checkCrc(buf, len)) {
        return; // drop -- either a real corrupt frame, or (more likely
                // right now) checkCrc() isn't implemented yet
    }

    FlarmFrame frame;
    frame.rssi = _link.lastRssi();
    frame.snr  = _link.lastSnr();
    frame.crcValid = true;

    if (decodeFrame(buf, len, frame) && _decodedCb) {
        _decodedCb(frame);
    }
}

void FlarmReceiver::dewhiten(uint8_t* buf, size_t len) {
    // TODO: XOR buf against the correct LFSR-generated whitening sequence.
    // Left as a no-op so this is obviously incomplete rather than
    // silently wrong.
    (void)buf; (void)len;
}

bool FlarmReceiver::checkCrc(const uint8_t* buf, size_t len) {
    // TODO: implement the real check. Returning false unconditionally
    // until then, so nothing downstream mistakes unverified data for a
    // valid frame.
    (void)buf; (void)len;
    return false;
}

bool FlarmReceiver::decodeFrame(const uint8_t* buf, size_t len, FlarmFrame& out) {
    (void)buf; (void)len; (void)out;
    return false;
}
