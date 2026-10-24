#pragma once
// Link-only stand-ins for the ESP32 SD_MMC library, Arduino String and File
// (native tests only).
//
// OpenAirScanner.cpp's loadAirspaceDatabase() reads its OpenAir file through
// these. Native tests can supply in-memory file contents with
// SD_MMC_SetFileContents(); paths without supplied contents remain closed.
#include <Arduino.h>
#include <map>
#include <string>
#include <stdexcept>
#include <utility>
#include <string.h>

#define FILE_READ "r"
#define FILE_WRITE "w"
#define FILE_APPEND "a"

// Just the String members OpenAirScanner.cpp uses.
class String
{
public:
    String() {}
    explicit String(std::string value) : s_(std::move(value)) {}
    void trim()
    {
        const auto first = s_.find_first_not_of(" \t\r\n");
        if (first == std::string::npos)
        {
            s_.clear();
            return;
        }

        const auto last = s_.find_last_not_of(" \t\r\n");
        s_ = s_.substr(first, last - first + 1);
    }
    unsigned    length() const   { return (unsigned)s_.size(); }
    char        charAt(unsigned i) const { return i < s_.size() ? s_[i] : 0; }
    const char* c_str() const    { return s_.c_str(); }
private:
    std::string s_;
};

class File
{
public:
    File() : open_(false), cursor_(0) {}
    explicit File(const std::string& contents) : contents_(contents), open_(true), cursor_(0) {}
    explicit operator bool() const { return open_; }
    int available() { return open_ && cursor_ < contents_.size(); }
    String readStringUntil(char delimiter)
    {
        const size_t start = cursor_;
        while (cursor_ < contents_.size() && contents_[cursor_] != delimiter)
        {
            ++cursor_;
        }
        const std::string line = contents_.substr(start, cursor_ - start);
        if (cursor_ < contents_.size())
        {
            ++cursor_;
        }
        return String(line);
    }
    void close() { open_ = false; }

    // Write side, used by IgcRecorder.cpp. Nothing is stored.
    size_t println(const char* text)     { return text ? strlen(text) + 2 : 0; }
    void flush()                         {}
    int getWriteError() const            { return 0; }
    void clearWriteError()               {}
    template <typename... A> void printf(const char*, A...) {}
private:
    std::string contents_;
    bool open_;
    size_t cursor_;
};

inline std::map<std::string, std::string>& SD_MMC_TestFiles()
{
    static std::map<std::string, std::string> files;
    return files;
}

inline void SD_MMC_SetFileContents(const char* path, const char* text)
{
    if (!path) throw std::invalid_argument("SD_MMC_SetFileContents(nullptr)");
    if (text)
    {
        SD_MMC_TestFiles()[path] = text;
    }
    else
    {
        SD_MMC_TestFiles().erase(path);
    }
}

inline void SD_MMC_ClearFileContents()
{
    SD_MMC_TestFiles().clear();
}

class SDMMCFS
{
public:
    File open(const char* path, const char* = FILE_READ)
    {
        // Real SD_MMC does not accept a null path. Fail loudly here so a
        // production null-check that goes missing is caught by a test.
        if (!path) throw std::invalid_argument("SD_MMC.open(nullptr)");
        const auto found = SD_MMC_TestFiles().find(path);
        return found == SD_MMC_TestFiles().end() ? File() : File(found->second);
    }
    bool exists(const char* path) const
    {
        return path && SD_MMC_TestFiles().find(path) != SD_MMC_TestFiles().end();
    }
};

extern SDMMCFS SD_MMC;