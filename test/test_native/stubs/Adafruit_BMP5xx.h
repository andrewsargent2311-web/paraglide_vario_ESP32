#pragma once
// Link-only stand-in. The native tests never call updateVario(), so these
// members exist only so Vario.cpp compiles.
class Adafruit_BMP5xx
{
public:
    float pressure = 0.0f;
    bool  performReading() { return false; }
    float readAltitude(float) { return 0.0f; }
};
