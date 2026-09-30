#include <gtest/gtest.h>

#include <cmath>

// Declares deg2rad, rad2deg, getBearing, getDistanceKM, getCompassDirection.
#include "Utils.h"

// Expected values for distance and bearing come from geometry (meridian
// arcs, equator arcs) or from an independent double-precision calculation
// (n-vector method), never from the production formula itself.
// getDistanceKM() uses R = 6371 km, so reference distances use the same
// mean Earth radius. Utils.h works in float, so tolerances allow for
// single-precision rounding.

static constexpr float kPi = 3.14159265358979f;


// ---------------------------------------------------------
// deg2rad / rad2deg
// ---------------------------------------------------------

TEST(UtilsAngleConversion, KnownDegreesConvertToRadians)
{
    EXPECT_NEAR(deg2rad(0.0f),    0.0f,        1e-6f);
    EXPECT_NEAR(deg2rad(90.0f),   kPi / 2.0f,  1e-6f);
    EXPECT_NEAR(deg2rad(180.0f),  kPi,         1e-6f);
    EXPECT_NEAR(deg2rad(-90.0f), -kPi / 2.0f,  1e-6f);
    EXPECT_NEAR(deg2rad(360.0f),  2.0f * kPi,  1e-5f);
}

TEST(UtilsAngleConversion, KnownRadiansConvertToDegrees)
{
    EXPECT_NEAR(rad2deg(0.0f),           0.0f,   1e-4f);
    EXPECT_NEAR(rad2deg(kPi / 2.0f),     90.0f,  1e-4f);
    EXPECT_NEAR(rad2deg(kPi),            180.0f, 1e-4f);
    EXPECT_NEAR(rad2deg(-kPi / 2.0f),   -90.0f,  1e-4f);
}

TEST(UtilsAngleConversion, RoundTripReturnsOriginalValue)
{
    for (float deg : {-270.0f, -45.5f, 0.0f, 12.345f, 179.99f, 720.0f})
    {
        EXPECT_NEAR(rad2deg(deg2rad(deg)), deg, 1e-3f) << "deg = " << deg;
    }
}


// ---------------------------------------------------------
// getDistanceKM
// ---------------------------------------------------------

TEST(UtilsDistance, IdenticalPointsAreZeroApart)
{
    EXPECT_NEAR(getDistanceKM(46.5f, 8.2f, 46.5f, 8.2f), 0.0f, 0.001f);
}

TEST(UtilsDistance, OneDegreeOfLatitudeIsAboutOneHundredElevenKm)
{
    // Meridian arc: R * (pi / 180) = 111.195 km.
    EXPECT_NEAR(getDistanceKM(0.0f, 0.0f, 1.0f, 0.0f), 111.195f, 0.05f);
}

TEST(UtilsDistance, QuarterOfEquatorIsQuarterOfCircumference)
{
    // R * pi / 2 = 10007.543 km.
    EXPECT_NEAR(getDistanceKM(0.0f, 0.0f, 0.0f, 90.0f), 10007.54f, 0.5f);
}

TEST(UtilsDistance, LongitudeDegreeShrinksWithLatitude)
{
    // One degree of longitude at 60 N is about half its equatorial length.
    EXPECT_NEAR(getDistanceKM(60.0f, 0.0f, 60.0f, 1.0f), 55.597f, 0.05f);
}

TEST(UtilsDistance, KnownCityPairLondonToParis)
{
    // Reference great-circle distance: 343.56 km.
    EXPECT_NEAR(getDistanceKM(51.5074f, -0.1278f, 48.8566f, 2.3522f), 343.56f, 0.5f);
}

TEST(UtilsDistance, LongHaulAcrossSouthernAndWesternHemispheres)
{
    // Sydney to Santiago. Reference great-circle distance: 11346.7 km.
    EXPECT_NEAR(getDistanceKM(-33.8688f, 151.2093f, -33.4489f, -70.6693f), 11346.7f, 5.0f);
}

TEST(UtilsDistance, IsSymmetric)
{
    const float ab = getDistanceKM(51.5074f, -0.1278f, 48.8566f, 2.3522f);
    const float ba = getDistanceKM(48.8566f,  2.3522f, 51.5074f, -0.1278f);

    EXPECT_NEAR(ab, ba, 0.001f);
}

TEST(UtilsDistance, OneKilometreNorthAtAlpineLatitude)
{
    // Flying-scale distance at typical alpine latitude and coordinate
    // magnitude. 1 km north = 0.0089932 degrees of latitude.
    // Guards against float precision loss on small differences.
    EXPECT_NEAR(getDistanceKM(46.0f, 8.0f, 46.0089932f, 8.0f), 1.0f, 0.01f);
}


// ---------------------------------------------------------
// getBearing
// ---------------------------------------------------------

TEST(UtilsBearing, CardinalDirectionsFromOrigin)
{
    EXPECT_NEAR(getBearing(0.0f, 0.0f,  1.0f,  0.0f),   0.0f, 0.01f);  // north
    EXPECT_NEAR(getBearing(0.0f, 0.0f,  0.0f,  1.0f),  90.0f, 0.01f);  // east
    EXPECT_NEAR(getBearing(1.0f, 0.0f,  0.0f,  0.0f), 180.0f, 0.01f);  // south
    EXPECT_NEAR(getBearing(0.0f, 0.0f,  0.0f, -1.0f), 270.0f, 0.01f);  // west
}

TEST(UtilsBearing, NorthEastDiagonalAtEquator)
{
    // atan(cos 1 deg) = 44.9956 degrees (slightly under 45 because the
    // meridians converge).
    EXPECT_NEAR(getBearing(0.0f, 0.0f, 1.0f, 1.0f), 44.9956f, 0.02f);
}

TEST(UtilsBearing, KnownCityPairLondonToParis)
{
    // Reference initial bearing: 148.116 degrees.
    EXPECT_NEAR(getBearing(51.5074f, -0.1278f, 48.8566f, 2.3522f), 148.116f, 0.1f);
}

TEST(UtilsBearing, ReverseDirectionOfCityPairIsNotSimplyOppositeOnASphere)
{
    // Paris to London reference initial bearing: 330.021 degrees
    // (not 148.116 + 180 = 328.116, because great circles curve).
    EXPECT_NEAR(getBearing(48.8566f, 2.3522f, 51.5074f, -0.1278f), 330.021f, 0.1f);
}

TEST(UtilsBearing, SouthernAndWesternHemispheres)
{
    // Sydney to Santiago. Reference initial bearing: 145.283 degrees.
    EXPECT_NEAR(getBearing(-33.8688f, 151.2093f, -33.4489f, -70.6693f), 145.283f, 0.1f);
}

TEST(UtilsBearing, ResultIsAlwaysInZeroToThreeSixtyRange)
{
    // Includes targets to the west, where the raw atan2 result is negative
    // and must be wrapped.
    const float lats[] = {-60.0f, -1.0f, 0.0f, 1.0f, 46.0f};
    const float lons[] = {-170.0f, -10.0f, 0.0f, 10.0f, 170.0f};

    for (float lat : lats)
    {
        for (float lon : lons)
        {
            const float b = getBearing(10.0f, 5.0f, lat, lon);
            if (lat == 10.0f && lon == 5.0f) continue;

            EXPECT_GE(b, 0.0f)   << "to (" << lat << ", " << lon << ")";
            EXPECT_LT(b, 360.0f) << "to (" << lat << ", " << lon << ")";
        }
    }
}

TEST(UtilsBearing, IdenticalPointsGiveFiniteValueInRange)
{
    // Direction to yourself is undefined; the contract worth protecting is
    // that the result is still a usable number, not NaN.
    const float b = getBearing(46.5f, 8.2f, 46.5f, 8.2f);

    EXPECT_TRUE(std::isfinite(b));
    EXPECT_GE(b, 0.0f);
    EXPECT_LT(b, 360.0f);
}


// ---------------------------------------------------------
// getCompassDirection
// ---------------------------------------------------------

TEST(UtilsCompassDirection, SectorCentresMapToTheirNames)
{
    struct Case { float heading; const char* expected; };
    const Case cases[] = {
        {0.0f,   "N"},  {45.0f,  "NE"}, {90.0f,  "E"},  {135.0f, "SE"},
        {180.0f, "S"},  {225.0f, "SW"}, {270.0f, "W"},  {315.0f, "NW"},
    };

    for (const Case& c : cases)
    {
        EXPECT_STREQ(getCompassDirection(c.heading), c.expected) << "heading = " << c.heading;
    }
}

TEST(UtilsCompassDirection, SectorBoundariesBelongToTheClockwiseSector)
{
    // Each boundary is the first value of the next sector; the value just
    // below it is still in the previous one. All boundary values are exactly
    // representable in float.
    struct Case { float heading; const char* expected; };
    const Case cases[] = {
        {22.4f,  "N"},  {22.5f,  "NE"},
        {67.4f,  "NE"}, {67.5f,  "E"},
        {112.4f, "E"},  {112.5f, "SE"},
        {157.4f, "SE"}, {157.5f, "S"},
        {202.4f, "S"},  {202.5f, "SW"},
        {247.4f, "SW"}, {247.5f, "W"},
        {292.4f, "W"},  {292.5f, "NW"},
        {337.4f, "NW"}, {337.5f, "N"},
    };

    for (const Case& c : cases)
    {
        EXPECT_STREQ(getCompassDirection(c.heading), c.expected) << "heading = " << c.heading;
    }
}

TEST(UtilsCompassDirection, NorthWrapsAtThreeSixty)
{
    EXPECT_STREQ(getCompassDirection(359.9f), "N");
    EXPECT_STREQ(getCompassDirection(360.0f), "N");
}

TEST(UtilsCompassDirection, SingleNegativeTurnIsNormalised)
{
    EXPECT_STREQ(getCompassDirection(-90.0f),  "W");
    EXPECT_STREQ(getCompassDirection(-180.0f), "S");
    EXPECT_STREQ(getCompassDirection(-10.0f),  "N");
    EXPECT_STREQ(getCompassDirection(-45.0f),  "NW");
}

TEST(UtilsCompassDirection, NotANumberReturnsPlaceholder)
{
    EXPECT_STREQ(getCompassDirection(std::nanf("")), "??");
}
