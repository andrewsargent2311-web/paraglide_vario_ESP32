#pragma once
// Gps -- the shared TinyGPSPlus object. UART init and the decode loop stay
// inline in setup()/loop().
#include <Arduino.h>
#include <TinyGPS++.h>

// =====================================================
// GPS LC76G
// =====================================================
#define GPS_RX_PIN 44
#define GPS_TX_PIN 43
#define GPS_BAUD 115200

extern TinyGPSPlus gps;
