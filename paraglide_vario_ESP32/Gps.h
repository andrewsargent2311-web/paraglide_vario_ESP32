#pragma once

#include <TinyGPS++.h>

// =====================================================
// GPS LC76G
// -----------------------------------------------------
// GPS_RX_PIN/TX_PIN/BAUD and the gps object itself. Serial1.begin() in
// setup() and the while (Serial1.available()) decode loop in loop()
// both stay inline in the main .ino, unchanged -- they just reference
// the extern declared here instead of a file-local global.
//
// gps is read by nearly every other subsystem (Vario, WindEstimator,
// ADS-B, Weather, IGC, AuxSensors' Clock section) -- this header exists
// purely so all of them have one place to extern it from.
//
// NOTE: in the original .ino, WIND_MIN_CIRCLE_SPEED_KPH /
// WIND_MAX_CIRCLE_SPEED_KPH / WIND_MAX_ESTIMATE_KPH and the wind
// estimator's own globals sit under this same "GPS LC76G" banner,
// between these #defines and the gps object. Those belong to
// WindEstimator, not here -- they're left in the main .ino for now and
// move in the WindEstimator extraction step, not this one.
// =====================================================

#define GPS_RX_PIN 44
#define GPS_TX_PIN 43
#define GPS_BAUD 115200

extern TinyGPSPlus gps;
