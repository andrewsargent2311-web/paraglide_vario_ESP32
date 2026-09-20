#include "OpenAirScanner.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

static constexpr double EARTH_RADIUS_KM = 6371.0088;
static constexpr double NM_TO_KM        = 1.852;
static constexpr double KM_PER_DEG_LAT  = 111.32;
static constexpr double OAS_DEG_TO_RAD = 0.017453292519943295;

// ---------------------------------------------------------------------------
// Cached database
// ---------------------------------------------------------------------------

static CachedAirspace airspaces[OAS_MAX_AIRSPACES];

static uint16_t airspaceCount = 0;

// Shared polygon point storage -- every CachedAirspace's points live here,
// indexed by pointStart/numPoints, instead of each airspace reserving its
// own OAS_MAX_POLY_POINTS-sized array whether it needs it or not.
static float pointPoolLat[OAS_POINT_POOL_SIZE];
static float pointPoolLon[OAS_POINT_POOL_SIZE];
static uint16_t pointPoolUsed = 0;

// Points lost because the pool itself ran out (distinct from
// OAS_MAX_POLY_POINTS, which caps a single polygon's own point count).
static uint16_t pointsDroppedForPoolFull = 0;

// Airspaces skipped entirely because fewer than 3 points fit in the
// remaining pool space.
static uint16_t airspacesRejectedForPool = 0;

static bool databaseLoaded = false;

// ---------------------------------------------------------------------------
// Geometry helpers
// ---------------------------------------------------------------------------

static inline double degToRad(double degrees) {
    return degrees * OAS_DEG_TO_RAD;
}

// ---------------------------------------------------------------------------
// Haversine distance
// ---------------------------------------------------------------------------

double haversine_km(
    double lat1,
    double lon1,
    double lat2,
    double lon2) {

    const double dLat =
        degToRad(lat2 - lat1);

    const double dLon =
        degToRad(lon2 - lon1);

    const double lat1Rad =
        degToRad(lat1);

    const double lat2Rad =
        degToRad(lat2);

    const double sLat =
        sin(dLat * 0.5);

    const double sLon =
        sin(dLon * 0.5);

    const double a =
        sLat * sLat +
        cos(lat1Rad) *
        cos(lat2Rad) *
        sLon * sLon;

    // Protect against tiny floating point overshoot.
    const double safeA =
        (a < 0.0)
            ? 0.0
            : (a > 1.0)
                ? 1.0
                : a;

    return
        EARTH_RADIUS_KM *
        2.0 *
        atan2(
            sqrt(safeA),
            sqrt(1.0 - safeA));
}

// ---------------------------------------------------------------------------
// Point in polygon
// ---------------------------------------------------------------------------

static bool pointInPolygon(
    double lat,
    double lon,
    const float* plat,
    const float* plon,
    uint8_t n) {

    if (n < 3) {
        return false;
    }

    bool inside = false;

    for (uint8_t i = 0, j = n - 1;
         i < n;
         j = i++) {

        const double yi = plat[i];
        const double yj = plat[j];

        if ((yi > lat) != (yj > lat)) {

            const double xIntersection =
                (plon[j] - plon[i]) *
                (lat - yi) /
                (yj - yi) +
                plon[i];

            if (lon < xIntersection) {
                inside = !inside;
            }
        }
    }

    return inside;
}

// ---------------------------------------------------------------------------
// Point-to-segment squared distance
//
// Coordinates are already in a local km plane.
//
// Returning squared distance avoids sqrt() for every polygon edge.
// ---------------------------------------------------------------------------

static inline double distPointToSegmentSq(
    double px,
    double py,
    double ax,
    double ay,
    double bx,
    double by) {

    const double abx = bx - ax;
    const double aby = by - ay;

    const double apx = px - ax;
    const double apy = py - ay;

    const double lenSq =
        abx * abx +
        aby * aby;

    double t = 0.0;

    if (lenSq > 1e-12) {

        t =
            (apx * abx +
             apy * aby) /
            lenSq;

        if (t < 0.0) {
            t = 0.0;
        }
        else if (t > 1.0) {
            t = 1.0;
        }
    }

    const double cx =
        ax + t * abx;

    const double cy =
        ay + t * aby;

    const double dx =
        px - cx;

    const double dy =
        py - cy;

    return
        dx * dx +
        dy * dy;
}

// ---------------------------------------------------------------------------
// Polygon distance
// ---------------------------------------------------------------------------
//
// Returns distance in kilometres.
//
// Uses squared distance internally and only performs one sqrt() at the end.
//
// bestKnownKm can be supplied to allow early termination.
// ---------------------------------------------------------------------------

static double polygonDistanceKm(
    double lat,
    double lon,
    const CachedAirspace& a,
    double bestKnownKm) {

    if (a.numPoints < 2) {
        return 1e18;
    }

    const double kmPerDegLon =
        KM_PER_DEG_LAT *
        cos(degToRad(lat));

    double bestSq = 1e18;

    if (bestKnownKm < 1e17) {
        bestSq =
            bestKnownKm *
            bestKnownKm;
    }

    for (uint8_t i = 0, j = a.numPoints - 1;
         i < a.numPoints;
         j = i++) {

        const double ax =
            (pointPoolLon[a.pointStart + j] - lon) *
            kmPerDegLon;

        const double ay =
            (pointPoolLat[a.pointStart + j] - lat) *
            KM_PER_DEG_LAT;

        const double bx =
            (pointPoolLon[a.pointStart + i] - lon) *
            kmPerDegLon;

        const double by =
            (pointPoolLat[a.pointStart + i] - lat) *
            KM_PER_DEG_LAT;

        const double distSq =
            distPointToSegmentSq(
                0.0,
                0.0,
                ax,
                ay,
                bx,
                by);

        if (distSq < bestSq) {

            bestSq = distSq;

            // We cannot beat zero.
            if (bestSq <= 0.0) {
                return 0.0;
            }
        }
    }

    return sqrt(bestSq);
}

// ---------------------------------------------------------------------------
// Bounding-box minimum distance
// ---------------------------------------------------------------------------
//
// Gives a cheap lower-bound distance from the aircraft to the polygon's
// bounding box.
//
// This is NOT the actual polygon distance.
//
// It is only used to reject polygons which cannot possibly beat the current
// nearest result.
//
// ---------------------------------------------------------------------------

static double boundingBoxDistanceKm(
    double lat,
    double lon,
    const CachedAirspace& a) {

    double dLat = 0.0;
    double dLon = 0.0;

    if (lat < a.minLat) {
        dLat = a.minLat - lat;
    }
    else if (lat > a.maxLat) {
        dLat = lat - a.maxLat;
    }

    if (lon < a.minLon) {
        dLon = a.minLon - lon;
    }
    else if (lon > a.maxLon) {
        dLon = lon - a.maxLon;
    }

    if (dLat == 0.0 &&
        dLon == 0.0) {

        return 0.0;
    }

    const double kmLat =
        dLat * KM_PER_DEG_LAT;

    const double kmLon =
        dLon *
        KM_PER_DEG_LAT *
        cos(degToRad(lat));

    return sqrt(
        kmLat * kmLat +
        kmLon * kmLon);
}

// ---------------------------------------------------------------------------
// Altitude resolver
// ---------------------------------------------------------------------------

float resolveAltitudeFt(
    const Altitude& alt,
    float groundElev_ft) {

    switch (alt.ref) {

        case ALTREF_SFC:
            return groundElev_ft;

        case ALTREF_UNL:
            return 999999.0f;

        case ALTREF_FL:
            return alt.value_ft;

        case ALTREF_AGL:
            return
                alt.value_ft +
                groundElev_ft;

        case ALTREF_MSL:
        default:
            return alt.value_ft;
    }
}

// ---------------------------------------------------------------------------
// Controlled class test
// ---------------------------------------------------------------------------

static bool isControlledClass(
    const char* classId,
    const char** controlledClasses,
    uint8_t numClasses) {

    for (uint8_t i = 0;
         i < numClasses;
         i++) {

        if (strcasecmp(
                classId,
                controlledClasses[i]) == 0) {

            return true;
        }
    }

    return false;
}

// ---------------------------------------------------------------------------
// Trim whitespace from C string
// ---------------------------------------------------------------------------

static void trimCString(char* s) {

    if (!s || !*s) {
        return;
    }

    // Remove leading whitespace by moving the string.
    char* start = s;

    while (*start &&
           (*start == ' ' ||
            *start == '\t' ||
            *start == '\r' ||
            *start == '\n')) {

        start++;
    }

    if (start != s) {
        memmove(
            s,
            start,
            strlen(start) + 1);
    }

    // Remove trailing whitespace.
    size_t len = strlen(s);

    while (len > 0 &&
           (s[len - 1] == ' ' ||
            s[len - 1] == '\t' ||
            s[len - 1] == '\r' ||
            s[len - 1] == '\n')) {

        s[--len] = '\0';
    }
}

// ---------------------------------------------------------------------------
// Upper-case C string
// ---------------------------------------------------------------------------

static void upperCString(char* s) {

    while (*s) {

        *s =
            (char)toupper(
                (unsigned char)*s);

        s++;
    }
}

// ---------------------------------------------------------------------------
// OpenAir coordinate parser
//
// Supports:
//
//   41:17:00S
//   S41:17:00
//   41:17:00.5 S
//   174:46:00E
//
// ---------------------------------------------------------------------------

static bool parseOneCoord(
    const char*& p,
    double& outValue,
    bool& isLat) {

    while (*p == ' ' ||
           *p == '\t') {

        p++;
    }

    if (!*p) {
        return false;
    }

    char hemi = 0;

    // Hemisphere before coordinate.
    if (*p == 'N' ||
        *p == 'S' ||
        *p == 'E' ||
        *p == 'W') {

        hemi = *p;
        p++;
    }

    char* end = nullptr;

    const long deg =
        strtol(
            p,
            &end,
            10);

    if (end == p) {
        return false;
    }

    p = end;

    int minutes = 0;
    double seconds = 0.0;

    // Minutes.
    if (*p == ':') {

        p++;

        const long m =
            strtol(
                p,
                &end,
                10);

        if (end == p) {
            return false;
        }

        minutes = (int)m;
        p = end;
    }

    // Seconds.
    if (*p == ':') {

        p++;

        seconds =
            strtod(
                p,
                &end);

        if (end == p) {
            return false;
        }

        p = end;
    }

    // Optional whitespace before hemisphere.
    while (*p == ' ' ||
           *p == '\t') {

        p++;
    }

    // Hemisphere after coordinate.
    if (!hemi &&
        (*p == 'N' ||
         *p == 'S' ||
         *p == 'E' ||
         *p == 'W')) {

        hemi = *p;
        p++;
    }

    double value =
        (double)deg +
        ((double)minutes / 60.0) +
        (seconds / 3600.0);

    if (hemi == 'S' ||
        hemi == 'W') {

        value = -value;
    }

    isLat =
        (hemi == 'N' ||
         hemi == 'S');

    outValue = value;

    return true;
}

// ---------------------------------------------------------------------------
// Coordinate pair
// ---------------------------------------------------------------------------

static bool parseCoordPair(
    const char* s,
    double& lat,
    double& lon) {

    const char* p = s;

    double v1;
    double v2;

    bool isLat1;
    bool isLat2;

    if (!parseOneCoord(
            p,
            v1,
            isLat1)) {

        return false;
    }

    if (!parseOneCoord(
            p,
            v2,
            isLat2)) {

        return false;
    }

    // Normally the latitude is identifiable by N/S.
    if (isLat1) {

        lat = v1;
        lon = v2;

    }
    else {

        lat = v2;
        lon = v1;
    }

    return true;
}

// ---------------------------------------------------------------------------
// Find case-insensitive substring
//
// Avoids relying on non-standard strcasestr() availability on all Arduino
// toolchains.
// ---------------------------------------------------------------------------

static bool containsIgnoreCase(
    const char* text,
    const char* needle) {

    if (!text || !needle || !*needle) {
        return false;
    }

    const size_t needleLen =
        strlen(needle);

    while (*text) {

        size_t i = 0;

        while (i < needleLen &&
               text[i] &&
               tolower(
                   (unsigned char)text[i]) ==
               tolower(
                   (unsigned char)needle[i])) {

            i++;
        }

        if (i == needleLen) {
            return true;
        }

        text++;
    }

    return false;
}

// ---------------------------------------------------------------------------
// Altitude parser
// ---------------------------------------------------------------------------

static Altitude parseAltitude(
    const char* input) {

    Altitude a;

    a.value_ft = 0.0f;
    a.ref = ALTREF_MSL;

    if (!input) {
        return a;
    }

    // Work with a small local copy.
    char s[40];

    strncpy(
        s,
        input,
        sizeof(s) - 1);

    s[sizeof(s) - 1] = '\0';

    trimCString(s);
    upperCString(s);

    // SFC / GND.
    if (strncmp(s, "SFC", 3) == 0 ||
        strncmp(s, "GND", 3) == 0) {

        a.ref = ALTREF_SFC;
        return a;
    }

    // Unlimited.
    if (strncmp(s, "UNL", 3) == 0) {

        a.ref = ALTREF_UNL;
        return a;
    }

    // Flight level.
    if (s[0] == 'F' &&
        s[1] == 'L') {

        a.ref = ALTREF_FL;

        a.value_ft =
            strtof(
                s + 2,
                nullptr) *
            100.0f;

        return a;
    }

    // Numeric altitude.
    a.value_ft =
        strtof(
            s,
            nullptr);

    // AGL.
    if (containsIgnoreCase(
            s,
            "AGL")) {

        a.ref = ALTREF_AGL;

    }
    else {

        // MSL / AMSL / bare numeric.
        a.ref = ALTREF_MSL;
    }

    return a;
}

// ---------------------------------------------------------------------------
// Temporary block while loading
// ---------------------------------------------------------------------------

struct BlockState {

    bool active = false;

    char name[OAS_MAX_NAME_LEN] = "";
    char classId[OAS_MAX_CLASS_LEN] = "";

    Altitude floor{
        0.0f,
        ALTREF_SFC
    };

    Altitude ceiling{
        0.0f,
        ALTREF_UNL
    };

    bool haveCenter = false;

    double centerLat = 0.0;
    double centerLon = 0.0;

    bool isCircle = false;

    double radius_nm = 0.0;

    double plat[OAS_MAX_POLY_POINTS];
    double plon[OAS_MAX_POLY_POINTS];

    uint8_t numPoints = 0;

    void reset() {

        active = false;

        name[0] = '\0';
        classId[0] = '\0';

        floor = {
            0.0f,
            ALTREF_SFC
        };

        ceiling = {
            0.0f,
            ALTREF_UNL
        };

        haveCenter = false;

        centerLat = 0.0;
        centerLon = 0.0;

        isCircle = false;

        radius_nm = 0.0;

        numPoints = 0;
    }

    void addPoint(
        double lat,
        double lon) {

        if (numPoints >=
            OAS_MAX_POLY_POINTS) {

            return;
        }

        plat[numPoints] = lat;
        plon[numPoints] = lon;

        numPoints++;
    }
};

// ---------------------------------------------------------------------------
// Store completed block
// ---------------------------------------------------------------------------

static bool storeBlock(
    const BlockState& b,
    const char** controlledClasses,
    uint8_t numClasses,
    uint16_t& totalBlocks,
    uint16_t& controlledBlocks,
    uint16_t& circleBlocks,
    uint16_t& polygonBlocks,
    uint32_t& polygonPoints) {

    if (!b.active) {
        return false;
    }

    totalBlocks++;

    if (b.classId[0] == '\0') {
        return false;
    }

    // Filter here so uncontrolled airspace never consumes cache RAM.
    if (!isControlledClass(
            b.classId,
            controlledClasses,
            numClasses)) {

        return false;
    }

    controlledBlocks++;

    // Common Frequency Zones are filed under Class B in this file purely
    // as a labelling convention -- they are NOT controlled airspace (no
    // ATC clearance needed) and must never trigger the proximity/entry
    // alert (top banner + tone). They're still kept in the cache and
    // still searchable, though: the "Airspace info" bar deliberately
    // wants to surface them, since a CFZ's name carries the recommended
    // reporting frequency, which is exactly what that bar is for. MBZs
    // are filed under Class A here and are genuinely meant to alert
    // (mandatory broadcast, not just advisory), so they're deliberately
    // NOT excluded the same way -- alertEligible stays true for them.
    bool isCfz =
        containsIgnoreCase(
            b.name,
            "CFZ");

    if (!b.isCircle &&
        b.numPoints < 3) {

        return false;
    }

    if (airspaceCount >=
        OAS_MAX_AIRSPACES) {

        Serial.println(
            "OpenAirScanner: CACHE FULL");

        return false;
    }

    // Circles don't use the point pool at all, so only polygons need to
    // check remaining pool space up front.
    uint8_t pointsToStore = b.numPoints;

    if (!b.isCircle) {

        const uint16_t poolRemaining =
            OAS_POINT_POOL_SIZE - pointPoolUsed;

        if (pointsToStore > poolRemaining) {
            pointsDroppedForPoolFull +=
                (pointsToStore - poolRemaining);
            pointsToStore = (uint8_t)poolRemaining;
        }

        if (pointsToStore < 3) {

            Serial.println(
                "OpenAirScanner: POINT POOL FULL -- airspace skipped");

            airspacesRejectedForPool++;

            return false;
        }
    }

    CachedAirspace& dst =
        airspaces[airspaceCount];

    // -----------------------------------------------------------------------
    // Basic information
    // -----------------------------------------------------------------------

    strncpy(
        dst.name,
        b.name,
        OAS_MAX_NAME_LEN - 1);

    dst.name[
        OAS_MAX_NAME_LEN - 1] =
        '\0';

    strncpy(
        dst.classId,
        b.classId,
        OAS_MAX_CLASS_LEN - 1);

    dst.classId[
        OAS_MAX_CLASS_LEN - 1] =
        '\0';

    dst.floor = b.floor;
    dst.ceiling = b.ceiling;

    dst.isCircle = b.isCircle;

    dst.alertEligible = !isCfz;

    // -----------------------------------------------------------------------
    // Circle
    // -----------------------------------------------------------------------

    dst.centerLat =
        (float)b.centerLat;

    dst.centerLon =
        (float)b.centerLon;

    dst.radius_nm =
        (float)b.radius_nm;

    // -----------------------------------------------------------------------
    // Polygon -- points are appended to the shared pool, not stored inline.
    // -----------------------------------------------------------------------

    dst.pointStart =
        pointPoolUsed;

    dst.numPoints =
        pointsToStore;

    dst.minLat = 90.0f;
    dst.maxLat = -90.0f;
    dst.minLon = 180.0f;
    dst.maxLon = -180.0f;

    for (uint8_t i = 0;
         i < pointsToStore;
         i++) {

        const float plat =
            (float)b.plat[i];

        const float plon =
            (float)b.plon[i];

        pointPoolLat[pointPoolUsed] = plat;
        pointPoolLon[pointPoolUsed] = plon;
        pointPoolUsed++;

        if (plat < dst.minLat)
            dst.minLat = plat;

        if (plat > dst.maxLat)
            dst.maxLat = plat;

        if (plon < dst.minLon)
            dst.minLon = plon;

        if (plon > dst.maxLon)
            dst.maxLon = plon;
    }

    // -----------------------------------------------------------------------
    // Circle bounding box
    //
    // This lets the runtime search reject distant circles cheaply too.
    // -----------------------------------------------------------------------

    if (dst.isCircle) {

        const double radiusKm =
            dst.radius_nm *
            NM_TO_KM;

        const double latDegrees =
            radiusKm /
            KM_PER_DEG_LAT;

        const double cosLat =
            cos(
                degToRad(
                    dst.centerLat));

        const double kmPerDegLon =
            KM_PER_DEG_LAT *
            ((fabs(cosLat) > 0.01)
                ? fabs(cosLat)
                : 0.01);

        const double lonDegrees =
            radiusKm /
            kmPerDegLon;

        dst.minLat =
            (float)(
                dst.centerLat -
                latDegrees);

        dst.maxLat =
            (float)(
                dst.centerLat +
                latDegrees);

        dst.minLon =
            (float)(
                dst.centerLon -
                lonDegrees);

        dst.maxLon =
            (float)(
                dst.centerLon +
                lonDegrees);

        circleBlocks++;
    }
    else {

        polygonBlocks++;
        polygonPoints +=
            pointsToStore;
    }

    airspaceCount++;

    return true;
}

// ---------------------------------------------------------------------------
// Public: load database
// ---------------------------------------------------------------------------

bool loadAirspaceDatabase(
    const char* filename,
    const char** controlledClasses,
    uint8_t numClasses) {

    databaseLoaded = false;
    airspaceCount = 0;
    pointPoolUsed = 0;
    pointsDroppedForPoolFull = 0;
    airspacesRejectedForPool = 0;

    if (!filename ||
        !controlledClasses ||
        numClasses == 0) {

        Serial.println(
            "OpenAirScanner: invalid load parameters");

        return false;
    }

    Serial.print(
        "OpenAirScanner: loading ");

    Serial.println(filename);

    File f =
        SD_MMC.open(
            filename,
            FILE_READ);

    if (!f) {

        Serial.print(
            "OpenAirScanner: could not open ");

        Serial.println(filename);

        return false;
    }

    BlockState block;

    block.reset();

    uint16_t totalBlocks = 0;
    uint16_t controlledBlocks = 0;
    uint16_t circleBlocks = 0;
    uint16_t polygonBlocks = 0;

    uint32_t polygonPoints = 0;

    uint16_t truncatedPolygons = 0;

    // -----------------------------------------------------------------------
    // Read file one line at a time.
    //
    // String is only used during the ONE-TIME loading process.
    // It is not used during GPS searches.
    // -----------------------------------------------------------------------

    while (f.available()) {

        String line =
            f.readStringUntil('\n');

        line.trim();

        if (line.length() == 0) {
            continue;
        }

        if (line.charAt(0) == '*') {
            continue;
        }

        const char* linePtr =
            line.c_str();

        // Find first whitespace.
        const char* p =
            linePtr;

        while (*p &&
               *p != ' ' &&
               *p != '\t') {

            p++;
        }

        if (p == linePtr) {
            continue;
        }

        // Command length.
        const size_t commandLen =
            (size_t)(p - linePtr);

        // We only support normal two-character OpenAir commands.
        if (commandLen == 0 ||
            commandLen >= 8) {

            continue;
        }

        char cmd[8];

        memcpy(
            cmd,
            linePtr,
            commandLen);

        cmd[commandLen] = '\0';

        upperCString(cmd);

        // Skip whitespace.
        while (*p == ' ' ||
               *p == '\t') {

            p++;
        }

        const char* rest = p;

        // -------------------------------------------------------------------
        // AC - new airspace block
        // -------------------------------------------------------------------

        if (strcmp(cmd, "AC") == 0) {

            if (block.active) {

                const uint8_t before =
                    block.numPoints;

                storeBlock(
                    block,
                    controlledClasses,
                    numClasses,
                    totalBlocks,
                    controlledBlocks,
                    circleBlocks,
                    polygonBlocks,
                    polygonPoints);

                if (before >=
                    OAS_MAX_POLY_POINTS) {

                    truncatedPolygons++;
                }
            }

            block.reset();
            block.active = true;

            strncpy(
                block.classId,
                rest,
                OAS_MAX_CLASS_LEN - 1);

            block.classId[
                OAS_MAX_CLASS_LEN - 1] =
                '\0';

            trimCString(
                block.classId);

            upperCString(
                block.classId);
        }

        // -------------------------------------------------------------------
        // AN - airspace name
        // -------------------------------------------------------------------

        else if (strcmp(cmd, "AN") == 0) {

            if (!block.active) {
                continue;
            }

            strncpy(
                block.name,
                rest,
                OAS_MAX_NAME_LEN - 1);

            block.name[
                OAS_MAX_NAME_LEN - 1] =
                '\0';

            trimCString(
                block.name);
        }

        // -------------------------------------------------------------------
        // AL - lower altitude
        // -------------------------------------------------------------------

        else if (strcmp(cmd, "AL") == 0) {

            if (!block.active) {
                continue;
            }

            block.floor =
                parseAltitude(rest);
        }

        // -------------------------------------------------------------------
        // AH - upper altitude
        // -------------------------------------------------------------------

        else if (strcmp(cmd, "AH") == 0) {

            if (!block.active) {
                continue;
            }

            block.ceiling =
                parseAltitude(rest);
        }

        // -------------------------------------------------------------------
        // V - variable
        //
        // Circle centre:
        //
        //   V X=41:17:00S 174:46:00E
        //
        // -------------------------------------------------------------------

        else if (strcmp(cmd, "V") == 0) {

            if (!block.active) {
                continue;
            }

            // Find X=
            if ((rest[0] == 'X' ||
                 rest[0] == 'x') &&
                rest[1] == '=') {

                double lat;
                double lon;

                if (parseCoordPair(
                        rest + 2,
                        lat,
                        lon)) {

                    block.centerLat =
                        lat;

                    block.centerLon =
                        lon;

                    block.haveCenter =
                        true;
                }
            }
        }

        // -------------------------------------------------------------------
        // DC - circle
        // -------------------------------------------------------------------

        else if (strcmp(cmd, "DC") == 0) {

            if (!block.active) {
                continue;
            }

            block.radius_nm =
                strtod(
                    rest,
                    nullptr);

            block.isCircle =
                block.haveCenter;
        }

        // -------------------------------------------------------------------
        // DP - polygon point
        // -------------------------------------------------------------------

        else if (strcmp(cmd, "DP") == 0) {

            if (!block.active) {
                continue;
            }

            double lat;
            double lon;

            if (parseCoordPair(
                    rest,
                    lat,
                    lon)) {

                if (block.numPoints <
                    OAS_MAX_POLY_POINTS) {

                    block.addPoint(
                        lat,
                        lon);
                }
                else {
                    // We continue reading the block but ignore excess points.
                }
            }
        }

        // -------------------------------------------------------------------
        // Other OpenAir commands intentionally ignored.
        //
        // DA/DB arcs are not reconstructed here.
        // -------------------------------------------------------------------
    }

    // -----------------------------------------------------------------------
    // Store final block.
    // -----------------------------------------------------------------------

    if (block.active) {

        const uint8_t before =
            block.numPoints;

        storeBlock(
            block,
            controlledClasses,
            numClasses,
            totalBlocks,
            controlledBlocks,
            circleBlocks,
            polygonBlocks,
            polygonPoints);

        if (before >=
            OAS_MAX_POLY_POINTS) {

            truncatedPolygons++;
        }
    }

    f.close();

    // -----------------------------------------------------------------------
    // Diagnostics
    // -----------------------------------------------------------------------

    Serial.println();
    Serial.println(
        "OpenAirScanner: database load complete");

    Serial.print(
        "  Total AC blocks:       ");

    Serial.println(totalBlocks);

    Serial.print(
        "  Controlled blocks:     ");

    Serial.println(controlledBlocks);

    Serial.print(
        "  Cached airspaces:      ");

    Serial.println(airspaceCount);

    Serial.print(
        "  Circles:               ");

    Serial.println(circleBlocks);

    Serial.print(
        "  Polygons:              ");

    Serial.println(polygonBlocks);

    Serial.print(
        "  Polygon points:        ");

    Serial.println(polygonPoints);

    Serial.print(
        "  Point pool used:       ");

    Serial.print(pointPoolUsed);

    Serial.print(
        " / ");

    Serial.println(OAS_POINT_POOL_SIZE);

    if (pointsDroppedForPoolFull > 0) {

        Serial.print(
            "  WARNING: points dropped (point pool full): ");

        Serial.println(
            pointsDroppedForPoolFull);
    }

    if (airspacesRejectedForPool > 0) {

        Serial.print(
            "  WARNING: airspaces skipped (point pool full): ");

        Serial.println(
            airspacesRejectedForPool);
    }

    if (truncatedPolygons > 0) {

        Serial.print(
            "  WARNING: polygons >= ");

        Serial.print(
            OAS_MAX_POLY_POINTS);

        Serial.print(
            " points: ");

        Serial.println(
            truncatedPolygons);
    }

    Serial.print(
        "  Cache limit:           ");

    Serial.println(
        OAS_MAX_AIRSPACES);

    if (airspaceCount >=
        OAS_MAX_AIRSPACES) {

        Serial.println(
            "  WARNING: airspace cache is FULL");
    }

    Serial.println();

    databaseLoaded =
        (airspaceCount > 0);

    return databaseLoaded;
}

// ---------------------------------------------------------------------------
// Public: number of cached airspaces
// ---------------------------------------------------------------------------

uint16_t getAirspaceCount() {
    return airspaceCount;
}

uint16_t getAirspacePointPoolUsed() {
    return pointPoolUsed;
}

uint16_t getAirspacePointPoolCapacity() {
    return OAS_POINT_POOL_SIZE;
}

// ---------------------------------------------------------------------------
// Public: database status
// ---------------------------------------------------------------------------

bool isAirspaceDatabaseLoaded() {
    return databaseLoaded;
}

// ---------------------------------------------------------------------------
// Runtime: nearest controlled airspace
// ---------------------------------------------------------------------------
//
// IMPORTANT:
//
// There is NO SD access here.
//
// There is NO OpenAir parsing here.
//
// This function only searches the cached RAM database.
//
// ---------------------------------------------------------------------------

bool findNearestControlledAirspace(
    double curLat,
    double curLon,
    float curAlt_ft_msl,
    float groundElev_ft,
    bool groundElevValid,
    bool alertOnly,
    AirspaceResult& out) {

    if (!databaseLoaded ||
        airspaceCount == 0) {

        return false;
    }

    bool haveBest = false;

    float bestHorizKm = 0.0f;
    float bestVertFt = 0.0f;

    // -----------------------------------------------------------------------
    // Search cached airspaces
    // -----------------------------------------------------------------------

    for (uint16_t i = 0;
         i < airspaceCount;
         i++) {

        const CachedAirspace& a =
            airspaces[i];

        // alertOnly == true is what the proximity/entry ALERT (top
        // banner + tone) searches with -- it should only ever trigger
        // for genuinely controlled airspace or an MBZ, never a CFZ
        // (see storeBlock()'s CFZ detection). alertOnly == false is what
        // the always-on "Airspace info" bar searches with instead, and
        // deliberately considers every cached entry -- CFZ included --
        // since that's the whole point of it existing.
        if (alertOnly && !a.alertEligible) {
            continue;
        }

        // -------------------------------------------------------------------
        // Resolve altitude.
        //
        // We deliberately do NOT discard vertically unrelated airspace here.
        //
        // Your original behaviour finds the nearest airspace horizontally
        // and then reports the vertical distance. This preserves that
        // behaviour.
        // -------------------------------------------------------------------

        const float floorFt =
            resolveAltitudeFt(
                a.floor,
                groundElev_ft);

        const float ceilingFt =
            resolveAltitudeFt(
                a.ceiling,
                groundElev_ft);

        // The floor/ceiling above are only trustworthy if they didn't need
        // groundElev_ft (i.e. they're MSL or FL referenced), or if they did
        // need it and it's currently valid.
        const bool floorNeedsGround =
            (a.floor.ref == ALTREF_AGL) ||
            (a.floor.ref == ALTREF_SFC);

        const bool ceilingNeedsGround =
            (a.ceiling.ref == ALTREF_AGL) ||
            (a.ceiling.ref == ALTREF_SFC);

        const bool vertKnown =
            groundElevValid ||
            !(floorNeedsGround ||
              ceilingNeedsGround);

        const bool insideVert =
            (curAlt_ft_msl >= floorFt) &&
            (curAlt_ft_msl <= ceilingFt);

        const float vertFt =
            insideVert
                ? 0.0f
                : (curAlt_ft_msl < floorFt
                    ? floorFt - curAlt_ft_msl
                    : curAlt_ft_msl - ceilingFt);

        // -------------------------------------------------------------------
        // Horizontal geometry
        // -------------------------------------------------------------------

        double horizKm = 0.0;
        bool insideHoriz = false;

        // -------------------------------------------------------------------
        // Circle
        // -------------------------------------------------------------------

        if (a.isCircle) {

            // Cheap bounding-box rejection first.
            //
            // The bounding box is slightly conservative, so an airspace
            // close to the edge is still passed through to the accurate
            // haversine test.

            if (curLat < a.minLat ||
                curLat > a.maxLat ||
                curLon < a.minLon ||
                curLon > a.maxLon) {

                // The aircraft is outside the circle's bounding box.
                //
                // Calculate the circle distance because this airspace could
                // still become the nearest result.

                const double centerDistance =
                    haversine_km(
                        curLat,
                        curLon,
                        a.centerLat,
                        a.centerLon);

                const double radiusKm =
                    a.radius_nm *
                    NM_TO_KM;

                if (centerDistance <= radiusKm) {

                    insideHoriz = true;
                    horizKm = 0.0;

                }
                else {

                    horizKm =
                        centerDistance -
                        radiusKm;
                }
            }
            else {

                const double centerDistance =
                    haversine_km(
                        curLat,
                        curLon,
                        a.centerLat,
                        a.centerLon);

                const double radiusKm =
                    a.radius_nm *
                    NM_TO_KM;

                insideHoriz =
                    centerDistance <= radiusKm;

                horizKm =
                    insideHoriz
                        ? 0.0
                        : centerDistance - radiusKm;
            }
        }

        // -------------------------------------------------------------------
        // Polygon
        // -------------------------------------------------------------------

        else {

            if (a.numPoints < 3) {
                continue;
            }

            // First check whether the aircraft is inside the polygon's
            // bounding box.
            const bool insideBoundingBox =
                !(curLat < a.minLat ||
                  curLat > a.maxLat ||
                  curLon < a.minLon ||
                  curLon > a.maxLon);

            if (insideBoundingBox) {

                insideHoriz =
                    pointInPolygon(
                        curLat,
                        curLon,
                        &pointPoolLat[a.pointStart],
                        &pointPoolLon[a.pointStart],
                        a.numPoints);

                if (insideHoriz) {

                    horizKm = 0.0;
                }
                else {

                    horizKm =
                        polygonDistanceKm(
                            curLat,
                            curLon,
                            a,
                            haveBest
                                ? bestHorizKm
                                : 1e18);
                }
            }
            else {

                // Aircraft is outside the bounding box.
                //
                // Get a cheap lower-bound distance.
                const double boxDistance =
                    boundingBoxDistanceKm(
                        curLat,
                        curLon,
                        a);

                // If the box itself is already farther away than the best
                // airspace, the polygon cannot possibly be closer.
                if (haveBest &&
                    boxDistance >
                    bestHorizKm) {

                    continue;
                }

                horizKm =
                    polygonDistanceKm(
                        curLat,
                        curLon,
                        a,
                        haveBest
                            ? bestHorizKm
                            : 1e18);

                insideHoriz = false;
            }
        }

        // -------------------------------------------------------------------
        // Compare with current best.
        //
        // Primary criterion remains horizontal distance.
        // Vertical distance breaks ties.
        // -------------------------------------------------------------------

        const bool better =
            !haveBest ||
            horizKm < bestHorizKm ||
            (horizKm == bestHorizKm &&
             vertFt < bestVertFt);

        if (!better) {
            continue;
        }

        // -------------------------------------------------------------------
        // Save result
        // -------------------------------------------------------------------

        strncpy(
            out.name,
            a.name,
            OAS_MAX_NAME_LEN - 1);

        out.name[
            OAS_MAX_NAME_LEN - 1] =
            '\0';

        strncpy(
            out.classId,
            a.classId,
            OAS_MAX_CLASS_LEN - 1);

        out.classId[
            OAS_MAX_CLASS_LEN - 1] =
            '\0';

        out.horizDistance_km =
            (float)horizKm;

        out.vertDistance_ft =
            vertFt;

        out.insideHoriz =
            insideHoriz;

        out.insideVert =
            insideVert;

        out.floor_ft_msl =
            floorFt;

        out.ceiling_ft_msl =
            ceilingFt;

        out.vertKnown =
            vertKnown;

        out.alertEligible =
            a.alertEligible;

        bestHorizKm =
            (float)horizKm;

        bestVertFt =
            vertFt;

        haveBest = true;

        // We can't possibly find anything horizontally closer than zero.
        // However, continue if zero because another zero-distance airspace
        // could have a smaller vertical distance.
    }

    return haveBest;
}