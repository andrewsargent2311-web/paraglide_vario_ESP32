#include <gtest/gtest.h>

#include <cmath>
#include <cstdlib>
#include <cstring>
#include <string>

// Declares formatIgcLatLon(). Also pulls in SdCard.h and settings.h.
#include "IgcRecorder.h"

// SCOPE: formatIgcLatLon() only. writeIgcBRecord(), start/stopIgcRecording(),
// updateIgcRecorder() and setFlightRecorderEnabled() depend on SD_MMC, a
// FreeRTOS semaphore, the system clock and millis(), and have no seam.
//
// IGC position fields (B-record columns):
//     latitude   DDMMmmmN|S     7 digits + hemisphere      (8 chars)
//     longitude  DDDMMmmmE|W    8 digits + hemisphere      (9 chars)
// The function writes both back to back: always exactly 17 characters.
//
// Exact expected strings were computed independently in exact decimal
// arithmetic, not with the production algorithm.

namespace
{

std::string format(double lat, double lon)
{
    char buf[24];
    formatIgcLatLon(lat, lon, buf, sizeof(buf));
    return std::string(buf);
}

// 17 characters: 7 digits, N|S, 8 digits, E|W.
bool isWellFormed(const std::string& s)
{
    if (s.size() != 17) return false;

    for (size_t i = 0; i < 17; ++i)
    {
        if (i == 7)       { if (s[i] != 'N' && s[i] != 'S') return false; }
        else if (i == 16) { if (s[i] != 'E' && s[i] != 'W') return false; }
        else if (s[i] < '0' || s[i] > '9')                  return false;
    }
    return true;
}

// Reads one field back into signed degrees. Only valid on a well-formed
// string. This is the inverse of the format, not the production algorithm.
double decodeField(const std::string& s, size_t start, size_t degDigits)
{
    const int deg = std::atoi(s.substr(start, degDigits).c_str());
    const int min = std::atoi(s.substr(start + degDigits, 2).c_str());
    const int mmm = std::atoi(s.substr(start + degDigits + 2, 3).c_str());
    const char hemi = s[start + degDigits + 5];

    const double value = deg + (min + mmm / 1000.0) / 60.0;
    return (hemi == 'S' || hemi == 'W') ? -value : value;
}

double decodeLat(const std::string& s) { return decodeField(s, 0, 2); }
double decodeLon(const std::string& s) { return decodeField(s, 8, 3); }

// One thousandth of a minute, in degrees. That is the format's resolution.
constexpr double kOneThousandthMinuteDeg = 0.001 / 60.0;

} // namespace


// ---------------------------------------------------------
// Format
// ---------------------------------------------------------

TEST(IgcLatLonFormat, KnownPositionsMatchIgcFormat)
{
    EXPECT_EQ(format(-41.2866, 174.7756), "4117196S17446536E");   // Wellington
    EXPECT_EQ(format(-45.0312, 168.6626), "4501872S16839756E");   // Queenstown
    EXPECT_EQ(format(46.6863, 7.8632),    "4641178N00751792E");   // Interlaken
}

TEST(IgcLatLonFormat, ZeroZeroIsNorthEast)
{
    EXPECT_EQ(format(0.0, 0.0), "0000000N00000000E");
}

TEST(IgcLatLonFormat, AllFourHemispheresUseTheirLetters)
{
    EXPECT_EQ(format( 33.5,  70.25), "3330000N07015000E");
    EXPECT_EQ(format(-33.5,  70.25), "3330000S07015000E");
    EXPECT_EQ(format( 33.5, -70.25), "3330000N07015000W");
    EXPECT_EQ(format(-33.5, -70.25), "3330000S07015000W");
}

TEST(IgcLatLonFormat, TinyOffsetsKeepTheirHemisphere)
{
    // 0.0001 degrees = 0.006 minutes.
    EXPECT_EQ(format(-0.0001, 0.0001), "0000006S00000006E");
    EXPECT_EQ(format(0.0001, -0.0001), "0000006N00000006W");
}

TEST(IgcLatLonFormat, LeadingZerosPadDegreesAndMinutes)
{
    EXPECT_EQ(format(5.5, 8.25), "0530000N00815000E");
}

TEST(IgcLatLonFormat, LongitudeOverOneHundredUsesThreeDegreeDigits)
{
    EXPECT_EQ(format(45.0, 174.5), "4500000N17430000E");
}

TEST(IgcLatLonFormat, PolesAndAntimeridian)
{
    EXPECT_EQ(format( 90.0,  180.0), "9000000N18000000E");
    EXPECT_EQ(format(-90.0, -180.0), "9000000S18000000W");
}

TEST(IgcLatLonFormat, MinutesFractionRoundsToNearestThousandth)
{
    // Minutes 0.1234 round down to 123; minutes 0.1236 round up to 124.
    // Checked in both fields, so a fault in either one shows up.
    EXPECT_EQ(format(10.0 + 0.1234 / 60.0, 10.0 + 0.1236 / 60.0), "1000123N01000124E");
    EXPECT_EQ(format(10.0 + 0.1236 / 60.0, 10.0 + 0.1234 / 60.0), "1000124N01000123E");
}

TEST(IgcLatLonFormat, RealisticSitesAreWellFormedAndAccurate)
{
    struct Site { const char* name; double lat; double lon; };
    const Site sites[] = {
        {"Wellington",   -41.2866,  174.7756},
        {"Queenstown",   -45.0312,  168.6626},
        {"Interlaken",    46.6863,    7.8632},
        {"Annecy",        45.8992,    6.1294},
        {"Owens Valley",  37.3600, -118.3900},
        {"Rio",          -22.9068,  -43.1729},
        {"London",        51.5074,   -0.1278},
        {"Tromso",        69.6492,   18.9553},
        {"Kathmandu",     27.7172,   85.3240},
    };

    for (const Site& s : sites)
    {
        const std::string out = format(s.lat, s.lon);

        ASSERT_TRUE(isWellFormed(out)) << s.name << ": \"" << out << "\"";
        EXPECT_NEAR(decodeLat(out), s.lat, kOneThousandthMinuteDeg) << s.name;
        EXPECT_NEAR(decodeLon(out), s.lon, kOneThousandthMinuteDeg) << s.name;
    }
}


// ---------------------------------------------------------
// Rounding at a minute boundary
// ---------------------------------------------------------

TEST(IgcLatLonFormat, RoundingUpAtAMinuteBoundaryStaysWellFormed)
{
    // When the minutes fraction is within half a thousandth of the next
    // whole minute (e.g. 59.9996'), rounding to thousandths gives 1.000 of a
    // minute. That must carry into the minutes/degrees (or otherwise stay in
    // range); it must never overflow the 3-digit fraction field and shift the
    // rest of the B-record. Either rounding up with a carry ("4800000N") or
    // truncating ("4759999N") is a valid answer; a wrong-width string is not.
    struct Case { double lat; double lon; };
    const Case cases[] = {
        { 47.0 + 59.9996 / 60.0,   8.5 },                       // latitude, minutes
        { 46.5,                    8.0 + 59.9996 / 60.0 },      // longitude, minutes
        { 46.99999999,             8.5 },                       // latitude, degrees
        { 46.5,                    8.99999999 },                // longitude, degrees
        {-(47.0 + 59.9996 / 60.0), -(8.0 + 59.9996 / 60.0) },   // southern / western
        { 1.0 / 60.0 - 0.0001 / 60.0, 0.0 },                    // 0d 00.9999' near zero
    };

    for (const Case& c : cases)
    {
        const std::string out = format(c.lat, c.lon);

        SCOPED_TRACE("lat=" + std::to_string(c.lat) + " lon=" + std::to_string(c.lon)
                     + " -> \"" + out + "\"");

        const bool wellFormed = isWellFormed(out);
        EXPECT_TRUE(wellFormed);
        if (!wellFormed) continue;   // keep going so every bad case is reported

        EXPECT_NEAR(decodeLat(out), c.lat, kOneThousandthMinuteDeg);
        EXPECT_NEAR(decodeLon(out), c.lon, kOneThousandthMinuteDeg);
    }
}


// ---------------------------------------------------------
// Buffer handling
// ---------------------------------------------------------

TEST(IgcLatLonFormat, NeverWritesPastTheGivenBufferSize)
{
    char buf[12];
    std::memset(buf, 'X', sizeof(buf));

    formatIgcLatLon(-41.2866, 174.7756, buf, 8);

    EXPECT_EQ(buf[7], '\0');                       // terminated inside the 8 bytes
    EXPECT_STREQ(buf, "4117196");                  // truncated, not overflowed
    for (int i = 8; i < 12; ++i)
    {
        EXPECT_EQ(buf[i], 'X') << "byte " << i << " was overwritten";
    }
}
