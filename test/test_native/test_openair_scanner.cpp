#include <gtest/gtest.h>

#include <cmath>
#include <cstring>
#include <string>

#include "SD_MMC.h"

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

namespace
{
const char* kTestAirspaceFile = "/TEST_AIRSPACE.TXT";
const char* kCircleWithFiveNmRadius =
    "AC CTR\n"
    "AN Test circle\n"
    "AL SFC\n"
    "AH 3500ft\n"
    "V X=41:00:00S 175:12:00E\n"
    "DC 5\n";
const char* kCircleThreeKmFromPilot =
    "AC CTR\n"
    "AN Nearby circle\n"
    "AL SFC\n"
    "AH 3500ft\n"
    "V X=41:00:00S 175:14:06E\n"
    "DC 0.1\n";
const char* kCShapedPolygon =
    "AC CTR\n"
    "AN C shaped polygon\n"
    "AL 1500ft\n"
    "AH 9500ft\n"
    "DP 41:30:00S 174:30:00E\n"
    "DP 41:30:00S 175:30:00E\n"
    "DP 41:18:00S 175:30:00E\n"
    "DP 41:18:00S 174:42:00E\n"
    "DP 40:42:00S 174:42:00E\n"
    "DP 40:42:00S 175:30:00E\n"
    "DP 40:30:00S 175:30:00E\n"
    "DP 40:30:00S 174:30:00E\n";

bool loadTestAirspace(const char* contents)
{
    SD_MMC_SetFileContents(kTestAirspaceFile, contents);
    return loadAirspaceDatabase(kTestAirspaceFile, kControlled, 5);
}
}

TEST(OpenAirSearch, CircleReportsEntryAndExitWithTrueHorizontalDistance)
{
    ASSERT_TRUE(loadTestAirspace(kCircleWithFiveNmRadius));

    const double outsideLat = -40.7;
    const double longitude = 175.2;
    AirspaceResult result;

    ASSERT_TRUE(findNearestControlledAirspace(
        outsideLat, longitude, 2500.0f, 100.0f, true, true, result));
    const double centerDistance = haversine_km(outsideLat, longitude, -41.0, longitude);
    const double expectedOutsideDistance = centerDistance - 5.0 * 1.852;
    EXPECT_FALSE(result.insideHoriz);
    EXPECT_NEAR(result.horizDistance_km, expectedOutsideDistance, 0.05);

    ASSERT_TRUE(findNearestControlledAirspace(
        -41.0, longitude, 2500.0f, 100.0f, true, true, result));
    EXPECT_TRUE(result.insideHoriz);
    EXPECT_TRUE(result.insideVert);
    EXPECT_FLOAT_EQ(result.horizDistance_km, 0.0f);

    ASSERT_TRUE(findNearestControlledAirspace(
        outsideLat, longitude, 2500.0f, 100.0f, true, true, result));
    EXPECT_FALSE(result.insideHoriz);
    EXPECT_NEAR(result.horizDistance_km, expectedOutsideDistance, 0.05);
}

TEST(OpenAirSearch, FartherPolygonDoesNotReplaceNearestCircle)
{
    const std::string contents = std::string(kCircleThreeKmFromPilot) + kCShapedPolygon;
    ASSERT_TRUE(loadTestAirspace(contents.c_str()));

    AirspaceResult result;
    ASSERT_TRUE(findNearestControlledAirspace(
        -41.0, 175.2, 5000.0f, 100.0f, true, true, result));

    const double centerDistance = haversine_km(-41.0, 175.2, -41.0, 175.235);
    const double expectedDistance = centerDistance - 0.1 * 1.852;
    EXPECT_STREQ(result.name, "Nearby circle");
    EXPECT_NEAR(result.horizDistance_km, expectedDistance, 0.05);
    EXPECT_FALSE(result.insideHoriz);
    EXPECT_FALSE(result.insideVert);
}

TEST(OpenAirSearch, LaterallyInsideCircleAboveCeilingRemainsNearest)
{
    const char* circle =
        "AC CTR\n"
        "AN Inside circle\n"
        "AL SFC\n"
        "AH 3500ft\n"
        "V X=41:00:00S 175:12:00E\n"
        "DC 5\n";
    const std::string contents = std::string(circle) + kCShapedPolygon;
    ASSERT_TRUE(loadTestAirspace(contents.c_str()));

    AirspaceResult result;
    ASSERT_TRUE(findNearestControlledAirspace(
        -41.0, 175.2, 5000.0f, 100.0f, true, true, result));

    EXPECT_STREQ(result.name, "Inside circle");
    EXPECT_FLOAT_EQ(result.horizDistance_km, 0.0f);
    EXPECT_TRUE(result.insideHoriz);
    EXPECT_FALSE(result.insideVert);
}

TEST(OpenAirSearch, CShapedPolygonReportsTrueEdgeDistance)
{
    ASSERT_TRUE(loadTestAirspace(kCShapedPolygon));

    AirspaceResult result;
    ASSERT_TRUE(findNearestControlledAirspace(
        -41.0, 175.2, 5000.0f, 100.0f, true, true, result));

    EXPECT_STREQ(result.name, "C shaped polygon");
    EXPECT_FALSE(result.insideHoriz);
    EXPECT_NEAR(result.horizDistance_km, 33.0, 1.0);
}

TEST(OpenAirSearch, SurfaceFloorReportsWhetherGroundElevationIsKnown)
{
    ASSERT_TRUE(loadTestAirspace(kCircleWithFiveNmRadius));

    AirspaceResult result;
    ASSERT_TRUE(findNearestControlledAirspace(
        -41.0, 175.2, 2500.0f, 100.0f, false, true, result));
    EXPECT_FALSE(result.vertKnown);

    ASSERT_TRUE(findNearestControlledAirspace(
        -41.0, 175.2, 2500.0f, 100.0f, true, true, result));
    EXPECT_TRUE(result.vertKnown);
}

// ---------------------------------------------------------------------------
// Arcs (DA / DB), altitude units and parse failures
// ---------------------------------------------------------------------------
//
// Geometry used below: centre 41:00:00S 175:00:00E. At that latitude one
// degree of longitude is about 84.0 km, so 3 NM east/west of the centre is
// about 0.0661 degrees of longitude and 5 NM is about 0.1102 degrees.

namespace
{
// East half-disc: the centre plus a clockwise 5 NM arc from north (0) to
// south (180).
const char* kEastHalfDiscByDa =
    "AC CTR\n"
    "AN East half disc\n"
    "AL SFC\n"
    "AH 3500ft\n"
    "V X=41:00:00S 175:00:00E\n"
    "V D=+\n"
    "DP 41:00:00S 175:00:00E\n"
    "DA 5,0,180\n";

// Same shape with the default direction omitted: must also be clockwise.
const char* kEastHalfDiscDefaultDirection =
    "AC CTR\n"
    "AN East half disc default\n"
    "AL SFC\n"
    "AH 3500ft\n"
    "V X=41:00:00S 175:00:00E\n"
    "DP 41:00:00S 175:00:00E\n"
    "DA 5,0,180\n";

// Anti-clockwise from north (0) to south (180) goes through west.
const char* kWestHalfDiscAntiClockwise =
    "AC CTR\n"
    "AN West half disc\n"
    "AL SFC\n"
    "AH 3500ft\n"
    "V X=41:00:00S 175:00:00E\n"
    "V D=-\n"
    "DP 41:00:00S 175:00:00E\n"
    "DA 5,0,180\n";

// North half-disc: DB from the east point to the west point, anti-clockwise
// (through north); the polygon closes along the east-west diameter.
const char* kNorthHalfDiscByDb =
    "AC CTR\n"
    "AN North half disc\n"
    "AL SFC\n"
    "AH 3500ft\n"
    "V X=41:00:00S 175:00:00E\n"
    "V D=-\n"
    "DB 41:00:00S 175:06:37E, 41:00:00S 174:53:23E\n";

bool insideHorizontally(double lat, double lon)
{
    AirspaceResult result;
    if (!findNearestControlledAirspace(lat, lon, 2000.0f, 100.0f, true, true, result))
        return false;
    return result.insideHoriz;
}
}

TEST(OpenAirArcs, DaArcFormsAnEastHalfDisc)
{
    ASSERT_TRUE(loadTestAirspace(kEastHalfDiscByDa));
    EXPECT_TRUE(insideHorizontally(-41.0, 175.0661));   // 3 NM east
    EXPECT_FALSE(insideHorizontally(-41.0, 174.9339));  // 3 NM west
    EXPECT_FALSE(insideHorizontally(-41.0, 175.1286));  // 7 NM east, beyond the arc
    EXPECT_TRUE(airspaceDatabaseComplete());
}

TEST(OpenAirArcs, DirectionDefaultsToClockwise)
{
    ASSERT_TRUE(loadTestAirspace(kEastHalfDiscDefaultDirection));
    EXPECT_TRUE(insideHorizontally(-41.0, 175.0661));
    EXPECT_FALSE(insideHorizontally(-41.0, 174.9339));
}

TEST(OpenAirArcs, AntiClockwiseDirectionSweepsTheOtherWay)
{
    ASSERT_TRUE(loadTestAirspace(kWestHalfDiscAntiClockwise));
    EXPECT_TRUE(insideHorizontally(-41.0, 174.9339));   // 3 NM west
    EXPECT_FALSE(insideHorizontally(-41.0, 175.0661));  // 3 NM east
}

TEST(OpenAirArcs, DbArcFormsANorthHalfDisc)
{
    ASSERT_TRUE(loadTestAirspace(kNorthHalfDiscByDb));
    EXPECT_TRUE(insideHorizontally(-40.9500, 175.0));   // about 3 NM north
    EXPECT_FALSE(insideHorizontally(-41.0500, 175.0));  // about 3 NM south
    EXPECT_FALSE(insideHorizontally(-40.9000, 175.0));  // about 6 NM north, beyond the arc
}

TEST(OpenAirArcs, ArcWithoutCentreMarksTheDatabaseIncomplete)
{
    const char* noCentre =
        "AC CTR\n"
        "AN Arc without centre\n"
        "AL SFC\n"
        "AH 3500ft\n"
        "DP 41:00:00S 175:00:00E\n"
        "DP 41:00:00S 175:06:00E\n"
        "DP 40:54:00S 175:06:00E\n"
        "DA 5,0,180\n";
    ASSERT_TRUE(loadTestAirspace(noCentre));
    EXPECT_FALSE(airspaceDatabaseComplete());
}

TEST(OpenAirAltitudes, MetresAreConvertedAndMslIsNotMistakenForMetres)
{
    const char* contents =
        "AC CTR\n"
        "AN Metres\n"
        "AL 1000M AMSL\n"
        "AH 5000 MSL\n"
        "V X=41:00:00S 175:00:00E\n"
        "DC 5\n";
    ASSERT_TRUE(loadTestAirspace(contents));

    AirspaceResult result;
    ASSERT_TRUE(findNearestControlledAirspace(
        -41.0, 175.0, 4000.0f, 100.0f, true, true, result));
    EXPECT_NEAR(result.floor_ft_msl, 3280.84f, 0.5f);
    EXPECT_NEAR(result.ceiling_ft_msl, 5000.0f, 0.5f);
    EXPECT_TRUE(airspaceDatabaseComplete());
}

TEST(OpenAirAltitudes, UnparseableCeilingFailsSafeToUnlimitedAndIsReported)
{
    const char* contents =
        "AC CTR\n"
        "AN Bad ceiling\n"
        "AL ???\n"
        "AH ABOVE\n"
        "V X=41:00:00S 175:00:00E\n"
        "DC 5\n";
    ASSERT_TRUE(loadTestAirspace(contents));

    AirspaceResult result;
    ASSERT_TRUE(findNearestControlledAirspace(
        -41.0, 175.0, 20000.0f, 100.0f, true, true, result));
    EXPECT_TRUE(result.insideHoriz);
    EXPECT_TRUE(result.insideVert);
    EXPECT_FALSE(airspaceDatabaseComplete());
}

TEST(OpenAirAltitudes, CircleWithoutAPositiveRadiusIsSkippedAndReported)
{
    const char* contents =
        "AC CTR\n"
        "AN Zero radius\n"
        "AL SFC\n"
        "AH 3500ft\n"
        "V X=41:00:00S 175:00:00E\n"
        "DC 0\n"
        "AC CTR\n"
        "AN Good circle\n"
        "AL SFC\n"
        "AH 3500ft\n"
        "V X=41:30:00S 175:00:00E\n"
        "DC 5\n";
    ASSERT_TRUE(loadTestAirspace(contents));
    EXPECT_FALSE(airspaceDatabaseComplete());

    AirspaceResult result;
    ASSERT_TRUE(findNearestControlledAirspace(
        -41.5, 175.0, 2000.0f, 100.0f, true, true, result));
    EXPECT_STREQ("Good circle", result.name);
}

// ---------------------------------------------------------------------------
// Regressions found by loading the real NZ AIRSPACE.txt
// ---------------------------------------------------------------------------

TEST(OpenAirRealFileFormats, CentreWithSpacesAroundEqualsSignIsAccepted)
{
    // The real NZ file writes "V X = lat lon" and "V D=-"; with the old
    // parser a spaced centre was silently ignored, so circles were never
    // created and arcs had no centre.
    const char* contents =
        "AC CTR\n"
        "AN Spaced centre circle\n"
        "AL SFC\n"
        "AH 3500 FT\n"
        "V X = 41:00:00 S 175:00:00 E\n"
        "DC 5\n";
    ASSERT_TRUE(loadTestAirspace(contents));
    EXPECT_TRUE(insideHorizontally(-41.0, 175.0661));  // 3 NM east
    EXPECT_FALSE(insideHorizontally(-41.0, 175.1286)); // 7 NM east
    EXPECT_TRUE(airspaceDatabaseComplete());
}

TEST(OpenAirRealFileFormats, SpacedCentreAndDirectionWorkForArcs)
{
    const char* contents =
        "AC CTR\n"
        "AN Spaced arc\n"
        "AL SFC\n"
        "AH 3500 FT\n"
        "V X = 41:00:00 S 175:00:00 E\n"
        "V D=-\n"
        "DB 41:00:00 S 175:06:37 E, 41:00:00 S 174:53:23 E\n";
    ASSERT_TRUE(loadTestAirspace(contents));
    EXPECT_TRUE(insideHorizontally(-40.9500, 175.0));   // about 3 NM north
    EXPECT_FALSE(insideHorizontally(-41.0500, 175.0));  // about 3 NM south
    EXPECT_TRUE(airspaceDatabaseComplete());
}

TEST(OpenAirRealFileFormats, TextCeilingIsUnlimitedAndNotAFailure)
{
    // NZ files give some ceilings as text, e.g. for MBZ and CFZ areas.
    const char* contents =
        "AC A\n"
        "AN NZB680, PARAPARAUMU MBZ, 118.3 MHz\n"
        "AL SFC\n"
        "AH Lower limit of controlled airspace within NZB680 lateral limits\n"
        "V X = 41:00:00 S 175:00:00 E\n"
        "DC 5\n";
    ASSERT_TRUE(loadTestAirspace(contents));

    AirspaceResult result;
    ASSERT_TRUE(findNearestControlledAirspace(
        -41.0, 175.0, 1000.0f, 100.0f, false, true, result));
    EXPECT_TRUE(result.insideHoriz);
    EXPECT_TRUE(result.insideVert);
    EXPECT_TRUE(airspaceDatabaseComplete());
}
