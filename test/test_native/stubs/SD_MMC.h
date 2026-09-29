#pragma once
// Link-only stand-ins for the ESP32 SD_MMC library, Arduino String and File
// (native tests only).
//
// OpenAirScanner.cpp's loadAirspaceDatabase() reads its OpenAir file through
// these. Nothing here can supply file contents: SD_MMC.open() always returns
// a closed File. So the native tests can only cover the parts of
// OpenAirScanner that never reach the SD card. Do not write tests that assert
// against this stub: they would be testing the stub, not production code.
#include <Arduino.h>
#include <string>
#include <stdexcept>

#define FILE_READ "r"
#define FILE_WRITE "w"

// Just the String members OpenAirScanner.cpp uses.
class String
{
public:
    String() {}
    void        trim()           {}
    unsigned    length() const   { return (unsigned)s_.size(); }
    char        charAt(unsigned i) const { return i < s_.size() ? s_[i] : 0; }
    const char* c_str() const    { return s_.c_str(); }
private:
    std::string s_;
};

class File
{
public:
    explicit operator bool() const { return false; }  // never opens
    int    available()                   { return 0; }
    String readStringUntil(char)         { return String(); }
    void   close()                       {}

    // Write side, used by IgcRecorder.cpp. Nothing is stored.
    void println(const char*)            {}
    void flush()                         {}
    template <typename... A> void printf(const char*, A...) {}
};

class SDMMCFS
{
public:
    File open(const char* path, const char* = FILE_READ)
    {
        // Real SD_MMC does not accept a null path. Fail loudly here so a
        // production null-check that goes missing is caught by a test.
        if (!path) throw std::invalid_argument("SD_MMC.open(nullptr)");
        return File();
    }
};

extern SDMMCFS SD_MMC;