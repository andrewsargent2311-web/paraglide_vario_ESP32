#pragma once
// Minimal host-side stand-in for Arduino.h (native tests only).
#include <stdint.h>
#include <stdio.h>
#include <stdarg.h>
#include <math.h>
#include <chrono>

typedef uint8_t byte;

#ifndef PI
#define PI 3.1415926535897932384626433832795
#endif
#ifndef TWO_PI
#define TWO_PI 6.283185307179586476925286766559
#endif
#ifndef radians
#define radians(deg) ((deg) * PI / 180.0)
#endif
#ifndef degrees
#define degrees(rad) ((rad) * 180.0 / PI)
#endif
#ifndef sq
#define sq(x) ((x) * (x))
#endif

// Defined by TinyGPS++.cpp when ARDUINO is not defined.
unsigned long millis();

// Vario.cpp only prints diagnostics; the tests don't check them.
struct HostSerial
{
    void print(const char*) {}
    void println(const char*) {}
    void println() {}
    void printf(const char*, ...) {}
 
    // Numeric overloads, e.g. Serial.print(uint16_t) / Serial.print(x, 2).
    template <typename T> void print(T) {}
    template <typename T, typename U> void print(T, U) {}
    template <typename T> void println(T) {}
    template <typename T, typename U> void println(T, U) {}
};
extern HostSerial Serial;
