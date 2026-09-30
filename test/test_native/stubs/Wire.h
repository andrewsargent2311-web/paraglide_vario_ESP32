#pragma once
// Link-only stand-in for the Arduino Wire library (native tests only).
// Utils.cpp's i2cDevicePresent() references Wire, but no native test calls it,
// so this does no real I2C. Do not write tests that assert against it: they
// would be testing this stub rather than production code.
#include <stdint.h>

class TwoWire
{
public:
    void    beginTransmission(uint8_t) {}
    uint8_t endTransmission() { return 4; }  // "other error": nothing is attached
};

extern TwoWire Wire;
