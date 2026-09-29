#include <gtest/gtest.h>

#include <cmath>
#include <cstring>

// Declares haversine_km, resolveAltitudeFt, loadAirspaceDatabase,
// findNearestControlledAirspace and the status getters.
#include "OpenAirScanner.h"

// SCOPE: only the parts of OpenAirScanner that never touch the SD card.
// The OpenAir parser, the cached-airspace search (polygon / circle geometry,
// alertOnly filtering, vertKnown) and CFZ detection are NOT covered here:
// the cache is file-static and can only be filled through
// loadAirspaceDatabase(), which reads via SD_MMC. See the notes that came
// with this file.
//
// haversine_km() uses R = 6371.0088 km. Reference distances come from
// geometry or from an independent n-vector calculation with the same R.
// It works in double, so tolerances are tight.

static constexpr double kEarthRadiusKm = 6371.0088;
static constexpr double kPi            = 3.14159265358979323846;


// ---------------------------------------------------------
// haversine_km
// ---------------------------------------------------------

TEST(OpenAirHaversine, IdenticalPointsAreZeroApart)
{
    EXPECT_NEAR(haversine_km(-41.2866, 174.7756, -41.2866, 174.7756), 0.0, 1e-9);
}

TEST(OpenAirHaversine, OneDegreeOfLatitude)
{
    // Meridian arc: R * pi / 180.
    EXPECT_NEAR(haversine_km(0.0, 0.0, 1.0, 0.0), 111.19508, 1e-4);
}

TEST(OpenAirHaversine, QuarterOfEquator)
{
    EXPECT_NEAR(haversine_km(0.0, 0.0, 0.0, 90.0), 10007.5572, 1e-3);
}

TEST(OpenAirHaversine, LongitudeDegreeShrinksWithLatitude)
{
    EXPECT_NEAR(haversine_km(60.0, 0.0, 60.0, 1.0), 55.59701, 1e-4);
}

TEST(OpenAirHaversine, KnownCityPairLondonToParis)
{
    EXPECT_NEAR(haversine_km(51.5074, -0.1278, 48.8566, 2.3522), 343.5565, 1e-3);
}

TEST(OpenAirHaversine, LongHaulAcrossSouthernAndWesternHemispheres)
{
    // Sydney to Santiago.
    EXPECT_NEAR(haversine_km(-33.8688, 151.2093, -33.4489, -70.6693), 11346.7308, 1e-2);
}

TEST(OpenAirHaversine, WellingtonToAuckland)
{
    // Regional flying scale, southern hemisphere.
    EXPECT_NEAR(haversine_km(-41.2866, 174.7756, -36.8485, 174.7633), 493.4960, 1e-3);
}

TEST(OpenAirHaversine, ShortDistanceAtFlyingScale)
{
    // 100 m due north at 46 N: 0.0008993204 degrees of latitude.
    EXPECT_NEAR(haversine_km(46.0, 8.0, 46.0008993204, 8.0), 0.1, 1e-6);
}

TEST(OpenAirHaversine, TakesShortWayAcrossTheAntimeridian)
{
    // 179.5 E to 179.5 W is one degree, not 359.
    EXPECT_NEAR(haversine_km(0.0, 179.5, 0.0, -179.5), 111.19508, 1e-4);
}

TEST(OpenAirHaversine, IsSymmetric)
{
    const double ab = haversine_km(51.5074, -0.1278, 48.8566, 2.3522);
    const double ba = haversine_km(48.8566, 2.3522, 51.5074, -0.1278);

    EXPECT_NEAR(ab, ba, 1e-9);
}

TEST(OpenAirHaversine, AntipodalPointsGiveHalfTheCircumferenceNotNaN)
{
    // For some antipodal pairs, floating-point rounding pushes the
    // intermediate haversine value just past 1 (by about 2e-16). Unclamped,
    // that makes sqrt(1 - a) NaN. The (-12, -170) -> (12, 10) pair does this
    // on common libm implementations (found by search; other libms may round
    // differently, which would only make that case less strict).
    const double half = kPi * kEarthRadiusKm;   // 20015.1144 km

    const double onEquator  = haversine_km(0.0, 0.0, 0.0, 180.0);
    const double offEquator = haversine_km(10.0, 20.0, -10.0, -160.0);
    const double overshoots = haversine_km(-12.0, -170.0, 12.0, 10.0);

    EXPECT_TRUE(std::isfinite(onEquator));
    EXPECT_TRUE(std::isfinite(offEquator));
    EXPECT_TRUE(std::isfinite(overshoots));
    EXPECT_NEAR(onEquator,  half, 1e-3);
    EXPECT_NEAR(offEquator, half, 1e-3);
    EXPECT_NEAR(overshoots, half, 1e-3);
}


// ---------------------------------------------------------
// resolveAltitudeFt
// ---------------------------------------------------------

TEST(OpenAirResolveAltitude, MslIsReturnedUnchangedAndIgnoresGround)
{
    const Altitude alt = {4500.0f, ALTREF_MSL};

    EXPECT_FLOAT_EQ(resolveAltitudeFt(alt, 0.0f),    4500.0f);
    EXPECT_FLOAT_EQ(resolveAltitudeFt(alt, 2000.0f), 4500.0f);
}

TEST(OpenAirResolveAltitude, FlightLevelIsReturnedUnchangedAndIgnoresGround)
{
    // value_ft already holds the FL converted to feet (FL65 = 6500 ft).
    const Altitude alt = {6500.0f, ALTREF_FL};

    EXPECT_FLOAT_EQ(resolveAltitudeFt(alt, 0.0f),    6500.0f);
    EXPECT_FLOAT_EQ(resolveAltitudeFt(alt, 3000.0f), 6500.0f);
}

TEST(OpenAirResolveAltitude, AglIsAddedToGroundElevation)
{
    const Altitude alt = {1500.0f, ALTREF_AGL};

    EXPECT_FLOAT_EQ(resolveAltitudeFt(alt, 0.0f),    1500.0f);
    EXPECT_FLOAT_EQ(resolveAltitudeFt(alt, 2000.0f), 3500.0f);
}

TEST(OpenAirResolveAltitude, AglOverGroundBelowSeaLevelSubtracts)
{
    const Altitude alt = {1000.0f, ALTREF_AGL};

    EXPECT_FLOAT_EQ(resolveAltitudeFt(alt, -400.0f), 600.0f);
}

TEST(OpenAirResolveAltitude, SurfaceIsTheGroundElevationRegardlessOfStoredValue)
{
    const Altitude alt = {123.0f, ALTREF_SFC};

    EXPECT_FLOAT_EQ(resolveAltitudeFt(alt, 850.0f), 850.0f);
    EXPECT_FLOAT_EQ(resolveAltitudeFt(alt, 0.0f),   0.0f);
}

TEST(OpenAirResolveAltitude, UnlimitedIsFarAboveAnyRealAltitude)
{
    const Altitude alt = {0.0f, ALTREF_UNL};

    EXPECT_FLOAT_EQ(resolveAltitudeFt(alt, 0.0f),    999999.0f);
    EXPECT_FLOAT_EQ(resolveAltitudeFt(alt, 5000.0f), 999999.0f);
}

TEST(OpenAirResolveAltitude, UnknownReferenceIsTreatedAsMsl)
{
    // The source groups "default" with MSL, so a corrupt reference value
    // falls back to the stored altitude.
    const Altitude alt = {3000.0f, static_cast<AltRef>(99)};

    EXPECT_FLOAT_EQ(resolveAltitudeFt(alt, 1000.0f), 3000.0f);
}


// ---------------------------------------------------------
// Load failure and empty database
// (all of these return before any SD access)
// ---------------------------------------------------------

namespace
{
const char* kControlled[] = {"A", "B", "C", "D", "CTR"};
}

TEST(OpenAirLoad, NullFilenameFailsAndLeavesDatabaseEmpty)
{
    EXPECT_FALSE(loadAirspaceDatabase(nullptr, kControlled, 5));

    EXPECT_FALSE(isAirspaceDatabaseLoaded());
    EXPECT_EQ(getAirspaceCount(), 0);
    EXPECT_EQ(getAirspacePointPoolUsed(), 0);
}

TEST(OpenAirLoad, NullClassListFailsAndLeavesDatabaseEmpty)
{
    EXPECT_FALSE(loadAirspaceDatabase("/AIRSPACE.TXT", nullptr, 5));

    EXPECT_FALSE(isAirspaceDatabaseLoaded());
    EXPECT_EQ(getAirspaceCount(), 0);
    EXPECT_EQ(getAirspacePointPoolUsed(), 0);
}

TEST(OpenAirLoad, ZeroClassesFailsAndLeavesDatabaseEmpty)
{
    EXPECT_FALSE(loadAirspaceDatabase("/AIRSPACE.TXT", kControlled, 0));

    EXPECT_FALSE(isAirspaceDatabaseLoaded());
    EXPECT_EQ(getAirspaceCount(), 0);
    EXPECT_EQ(getAirspacePointPoolUsed(), 0);
}

TEST(OpenAirSearch, EmptyDatabaseFindsNothingAndLeavesResultUntouched)
{
    // A failed load resets the database, so this doesn't depend on test order.
    ASSERT_FALSE(loadAirspaceDatabase(nullptr, kControlled, 5));

    for (bool alertOnly : {true, false})
    {
        AirspaceResult out;
        std::memset(&out, 0xAB, sizeof(out));
        AirspaceResult before = out;

        EXPECT_FALSE(findNearestControlledAirspace(
            -41.2866, 174.7756, 3000.0f, 100.0f, true, alertOnly, out))
            << "alertOnly = " << alertOnly;

        EXPECT_EQ(0, std::memcmp(&out, &before, sizeof(out)))
            << "alertOnly = " << alertOnly;
    }
}
