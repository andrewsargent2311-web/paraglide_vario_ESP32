#pragma once
// Utils -- stateless helpers (geo math, compass text, I2C probe). No state.
#include <Arduino.h>

float deg2rad(float deg);
float rad2deg(float rad);
float getBearing(float lat1, float lon1, float lat2, float lon2);
float getDistanceKM(float lat1, float lon1, float lat2, float lon2);
const char* getCompassDirection(float heading);
bool i2cDevicePresent(uint8_t addr);
