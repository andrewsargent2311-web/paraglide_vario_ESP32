#pragma once

#include <Arduino.h>
#include <math.h>
#include <Wire.h>

// =====================================================
// Utils: generic geo-math and a bare I2C presence probe.
// -----------------------------------------------------
// getBearing()/getDistanceKM() are shared by the ADS-B and Weather
// clients. getCompassDirection() is not called anywhere in the main
// .ino -- it is almost certainly used by DrawPages.cpp; confirm that
// and make sure DrawPages.cpp picks up this header once it exists.
// i2cDevicePresent() is called from two inline setup() probes (BMP580,
// RTC) and once from inside Buzzer's es8311Init() -- all three call
// sites just need this header included.
// =====================================================

float deg2rad(float deg);
float rad2deg(float rad);
float getBearing(float lat1, float lon1, float lat2, float lon2);
float getDistanceKM(float lat1, float lon1, float lat2, float lon2);
const char* getCompassDirection(float heading);

// Bare I2C address probe -- bounded by Wire.setTimeOut() (set once in
// setup()), so it can never hang even if nothing responds. Used to skip
// calling into a sensor library's begin()/readTime() at all when the
// device isn't physically present, rather than trusting every
// third-party library to handle "device absent" gracefully internally.
bool i2cDevicePresent(uint8_t addr);
