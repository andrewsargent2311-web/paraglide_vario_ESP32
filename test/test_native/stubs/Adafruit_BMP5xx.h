#pragma once
// Link-only stand-in. The native tests never call updateVario(), so these
// members exist only so Vario.cpp compiles.
#include <stdint.h>
class TwoWire;

#define BMP5XX_POWERMODE_STANDBY 0
#define BMP5XX_POWERMODE_NORMAL 1
#define BMP5XX_OVERSAMPLING_2X 2
#define BMP5XX_OVERSAMPLING_8X 8
#define BMP5XX_IIR_FILTER_COEFF_3 3
#define BMP5XX_ODR_50_HZ 50

class Adafruit_BMP5xx
{
public:
    float pressure = 0.0f;
    bool begin(uint8_t, TwoWire*) { return false; }
    void setPowerMode(int) {}
    void setTemperatureOversampling(int) {}
    void setPressureOversampling(int) {}
    void setIIRFilterCoeff(int) {}
    void setOutputDataRate(int) {}
    void enablePressure(bool) {}
    bool  performReading() { return false; }
    float readAltitude(float) { return 0.0f; }
};
