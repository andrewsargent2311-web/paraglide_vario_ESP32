#pragma once
// Link-only stand-in for the ESP32 Preferences (NVS) library (native tests
// only). settings.cpp's loadSettings()/saveSettings() use it, but no native
// test calls them. Every getter just returns the default it is given.
#include <Arduino.h>
#include <SD_MMC.h>   // String

class Preferences
{
public:
    bool begin(const char*, bool = false) { return true; }

    uint8_t  getUChar(const char*, uint8_t d)  { return d; }
    int8_t   getChar (const char*, int8_t d)   { return d; }
    int      getInt  (const char*, int d)      { return d; }
    uint32_t getUInt (const char*, uint32_t d) { return d; }
    float    getFloat(const char*, float d)    { return d; }
    bool     getBool (const char*, bool d)     { return d; }
    String   getString(const char*, const char*) { return String(); }

    void putUChar (const char*, uint8_t)     {}
    void putChar  (const char*, int8_t)      {}
    void putInt   (const char*, int)         {}
    void putUInt  (const char*, uint32_t)    {}
    void putFloat (const char*, float)       {}
    void putBool  (const char*, bool)        {}
    void putString(const char*, const char*) {}
};
