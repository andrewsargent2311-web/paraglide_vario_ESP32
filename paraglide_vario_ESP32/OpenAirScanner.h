#pragma once

#include <Arduino.h>
#include <SD_MMC.h>

// ---------------------------------------------------------------------------
// OpenAirScanner
//
// Two-stage airspace scanner:
//
//   BOOT:
//       loadAirspaceDatabase()
//           |
//           +-- Read OpenAir file from SD
//           +-- Parse airspace blocks
//           +-- Keep controlled airspaces only
//           +-- Store compact database in RAM
//
//   RUNTIME:
//       findNearestControlledAirspace()
//           |
//           +-- Searches RAM only
//           +-- NO SD access
//           +-- NO OpenAir parsing
//
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------
//
// Increase OAS_MAX_AIRSPACES if the loader reports the airspace cache is
// full ("CACHE FULL"); increase OAS_POINT_POOL_SIZE if it reports the
// point pool is full ("POINT POOL FULL"). These are the two independent
// limits now -- polygon points are NOT stored per-airspace any more (see
// CachedAirspace below), so OAS_MAX_POLY_POINTS only bounds how many
// points a single polygon may contribute before it gets truncated; it no
// longer drives total RAM use.
//
// RAM usage is dominated by:
//
//   OAS_MAX_AIRSPACES * sizeof(CachedAirspace)      (~108 bytes/airspace
//                                                     once points move out
//                                                     of the struct)
//   + OAS_POINT_POOL_SIZE * 2 * sizeof(float)        (shared pool, 8
//                                                      bytes/point)
//
// Example, sized for a real OpenAir file with 223 controlled polygons
// averaging ~11 points each (1,699 points across the first 150 loaded):
//
//   250 * 108           =  27,000 bytes  (airspace slots, generous headroom)
//   4000 * 2 * 4         =  32,000 bytes  (point pool, ~2.4x the ~1,700 actually seen)
//                         = 59,000 bytes total
//
// ...versus the old fixed-per-airspace layout, which for the same 250
// airspaces at 128 points each would have needed 250 * 1,132 = 283,000
// bytes -- the pool approach uses roughly a fifth of the RAM for the same
// (or bigger) capacity, because it only pays for points actually present
// in the file instead of reserving the worst case for every airspace.
//
// The actual used RAM is allocated statically according to these limits.
//
// ---------------------------------------------------------------------------

#define OAS_MAX_AIRSPACES   250
#define OAS_MAX_POLY_POINTS 128
#define OAS_POINT_POOL_SIZE 4000
#define OAS_MAX_NAME_LEN    48
#define OAS_MAX_CLASS_LEN   8

// ---------------------------------------------------------------------------
// Altitude references
// ---------------------------------------------------------------------------

enum AltRef : uint8_t {

    ALTREF_MSL,     // Explicit ft MSL / AMSL
    ALTREF_AGL,     // Explicit ft AGL
    ALTREF_FL,      // Flight level, converted to feet
    ALTREF_SFC,     // Surface / ground
    ALTREF_UNL      // Unlimited
};

struct Altitude {

    float value_ft;
    AltRef ref;
};

// ---------------------------------------------------------------------------
// Search result
// ---------------------------------------------------------------------------

struct AirspaceResult {

    char name[OAS_MAX_NAME_LEN];
    char classId[OAS_MAX_CLASS_LEN];

    float horizDistance_km;
    float vertDistance_ft;

    bool insideHoriz;
    bool insideVert;

    float floor_ft_msl;
    float ceiling_ft_msl;

    // False if this airspace's floor or ceiling is AGL- or SFC-referenced
    // and groundElevValid was false when this result was produced (see
    // findNearestControlledAirspace() below). When false, floor_ft_msl,
    // ceiling_ft_msl, vertDistance_ft and insideVert were computed from a
    // ground elevation that is not currently known to be correct (e.g.
    // aircraft has flown outside the loaded DEM tile) and should be shown
    // as unknown ("--") rather than trusted. insideHoriz/horizDistance_km
    // are unaffected either way.
    bool vertKnown;

    // True for genuinely controlled airspace and MBZs; false for a CFZ
    // (see storeBlock()'s CFZ detection in OpenAirScanner.cpp). Only
    // meaningful when this result came from a call with alertOnly =
    // false -- a call with alertOnly = true never returns a CFZ result
    // in the first place, so this is always true in that case.
    bool alertEligible;
};

// ---------------------------------------------------------------------------
// Cached airspace
// ---------------------------------------------------------------------------
//
// Only controlled airspaces are stored.
//
// Coordinates are stored as float rather than double to reduce RAM use.
// OpenAir parsing itself still uses double precision.
//
// Polygon points are NOT stored inline here -- pointStart/numPoints index
// into the shared pointPoolLat/pointPoolLon arrays (OpenAirScanner.cpp),
// so each airspace only costs RAM for the points it actually has, instead
// of always reserving OAS_MAX_POLY_POINTS worth of space whether it needs
// it or not.
//
// ---------------------------------------------------------------------------

struct CachedAirspace {

    char name[OAS_MAX_NAME_LEN];
    char classId[OAS_MAX_CLASS_LEN];

    Altitude floor;
    Altitude ceiling;

    bool isCircle;

    // True for genuinely controlled airspace and MBZs, false for a CFZ
    // (Common Frequency Zone -- filed under Class B in NZ's OpenAir file
    // as a labelling convention, but not actually controlled airspace;
    // see storeBlock()'s CFZ detection in OpenAirScanner.cpp). Governs
    // whether this entry can ever be returned by an alertOnly = true
    // call to findNearestControlledAirspace() -- i.e. whether it can
    // trigger the proximity/entry alert. It's still kept in the cache
    // and still returned by an alertOnly = false call either way, since
    // the "Airspace info" bar wants to surface CFZs too.
    bool alertEligible;

    // Circle geometry
    float centerLat;
    float centerLon;
    float radius_nm;

    // Polygon geometry -- indexes into the shared point pool, not an
    // inline array. See OAS_POINT_POOL_SIZE.
    uint16_t pointStart;
    uint8_t numPoints;

    // Polygon bounding box
    float minLat;
    float maxLat;
    float minLon;
    float maxLon;
};

// ---------------------------------------------------------------------------
// Load database
// ---------------------------------------------------------------------------
//
// Call ONCE after SD_MMC has been initialised.
//
// Example:
//
//   const char* controlled[] = {
//       "A", "B", "C", "D", "CTR"
//   };
//
//   loadAirspaceDatabase(
//       "/airspace_nz.txt",
//       controlled,
//       5);
//
// ---------------------------------------------------------------------------

bool loadAirspaceDatabase(
    const char* filename,
    const char** controlledClasses,
    uint8_t numClasses);

// ---------------------------------------------------------------------------
// Database status
// ---------------------------------------------------------------------------

uint16_t getAirspaceCount();

bool isAirspaceDatabaseLoaded();

// How much of the shared polygon point pool (OAS_POINT_POOL_SIZE) is
// currently used, for boot-time RAM tuning diagnostics.
uint16_t getAirspacePointPoolUsed();
uint16_t getAirspacePointPoolCapacity();

// ---------------------------------------------------------------------------
// Runtime search
// ---------------------------------------------------------------------------
//
// Searches the cached database.
//
// IMPORTANT:
// This function does NOT access the SD card.
//
// groundElev_ft is only meaningful when groundElevValid is true. Pass
// groundElevValid = false whenever the caller's ground elevation source
// (e.g. a DEM lookup) is currently unresolved -- the search still runs and
// still returns the nearest airspace by horizontal distance, but any
// AGL/SFC-referenced floor or ceiling in the result is flagged via
// AirspaceResult::vertKnown = false rather than silently computed from a
// stale groundElev_ft.
//
// alertOnly = true restricts the search to alertEligible entries only
// (genuinely controlled airspace + MBZs) -- this is what the proximity/
// entry ALERT (top banner + tone) should always call with, since it must
// never trigger for a CFZ. alertOnly = false considers every cached
// entry, CFZ included -- this is what the always-on "Airspace info" bar
// should call with instead, since surfacing a CFZ's name/frequency is
// the whole point of it existing. The two calls are independent; call
// both once per update if you need both the alert and the info bar fed.
//
// ---------------------------------------------------------------------------

bool findNearestControlledAirspace(
    double curLat,
    double curLon,
    float curAlt_ft_msl,
    float groundElev_ft,
    bool groundElevValid,
    bool alertOnly,
    AirspaceResult& out);

// ---------------------------------------------------------------------------
// Clearances: "what is in front of me" and "what is above/below me"
// ---------------------------------------------------------------------------
//
// findNearestControlledAirspace() answers "which airspace is nearest
// horizontally" and reports the vertical gap to that SAME airspace, which
// is misleading when an airspace nearby is far above/below you while a
// different one is at your height. findAirspaceClearances() answers the
// two questions a pilot actually asks, independently:
//
//   HORIZONTAL -- how far until I fly into controlled airspace at MY
//                 CURRENT ALTITUDE? Only airspace whose floor/ceiling band
//                 contains your altitude is considered (an airspace sitting
//                 entirely above or below you can't stop you flying on
//                 level). 0 = you are inside one right now.
//   VERTICAL   -- how far above/below me is the nearest controlled airspace
//                 I am horizontally INSIDE (i.e. directly over/under me)?
//                 0 = you are inside one right now.
//
// alertEligible entries only (no CFZ), same as the alert/banner search.
// AGL/SFC-referenced floors or ceilings need a valid ground elevation: when
// groundElevValid is false, such an airspace is treated as *possibly* at
// your altitude for the horizontal answer (the safe side, same policy as
// the banner), and makes the vertical answer "unknown" rather than wrong.
//
// Like findNearestControlledAirspace(), searches the RAM cache only -- no
// SD access.

struct AirspaceClearance {

    // Horizontal distance to the nearest controlled airspace at your altitude.
    bool  horizValid;          // false = none at your altitude in the database
    float horizDistance_km;    // 0 = inside it (horizontally, at your height)
    char  horizName[OAS_MAX_NAME_LEN];
    char  horizClassId[OAS_MAX_CLASS_LEN];

    // Vertical clearance to the nearest controlled airspace directly over/under you.
    bool  vertValid;           // false = not horizontally inside any (or unknown, see vertUnknown)
    bool  vertUnknown;         // horizontally inside one but its floor/ceiling can't be resolved (no ground elevation)
    float vertDistance_ft;     // 0 = inside it (vertically, too)
    bool  vertAbove;           // true = airspace is above you (its floor), false = below you (its ceiling)
    char  vertName[OAS_MAX_NAME_LEN];
    char  vertClassId[OAS_MAX_CLASS_LEN];
};

// Returns false only if the database isn't loaded (out untouched); otherwise
// true, with each half valid or not per the flags above.
bool findAirspaceClearances(
    double curLat,
    double curLon,
    float curAlt_ft_msl,
    float groundElev_ft,
    bool groundElevValid,
    AirspaceClearance& out);

// ---------------------------------------------------------------------------
// Geometry helper
// ---------------------------------------------------------------------------

double haversine_km(
    double lat1,
    double lon1,
    double lat2,
    double lon2);

// ---------------------------------------------------------------------------
// Altitude helper
// ---------------------------------------------------------------------------

float resolveAltitudeFt(
    const Altitude& alt,
    float groundElev_ft);