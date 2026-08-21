#pragma once
#include <Arduino.h>
#include <SD.h>

// ---------------------------------------------------------------------------
// OpenAirScanner
//
// Streams an OpenAir-format airspace file (e.g. the NZ file from
// soaringweb.org/Airspace/AU.html#NZ) off an SD card and finds the nearest
// "controlled" airspace to a given lat/lon/altitude, WITHOUT loading the
// whole file into RAM. Each airspace block is parsed, measured, and
// discarded before the next one is read.
//
// Usage:
//   AirspaceResult nearest;
//   const char* controlled[] = {"A", "B", "C", "D", "CTR"};
//   bool found = findNearestControlledAirspace(
//       "/airspace_nz.txt", currentLat, currentLon, currentAltFtMSL,
//       0.0f /* groundElevFt, see caveat below */, nearest,
//       controlled, 5);
//
// IMPORTANT CAVEAT (altitude):
// OpenAir floors/ceilings are given as SFC/GND, a flight level (FLxxx), or a
// number in ft AGL or ft MSL. AGL values are converted to MSL using
// groundElevFt, which this code cannot determine on its own (GPS altitude is
// NOT terrain elevation). Pass in a real ground elevation for your area
// (e.g. a fixed value for your home field, or a value you look up) or the
// AGL-based floors will be wrong. FLxxx is treated as approximately equal to
// its MSL altitude in feet (FL x 100), which is only exact at standard
// pressure (1013.25 hPa) -- fine for a proximity warning, not for legal
// separation.
// ---------------------------------------------------------------------------

#define OAS_MAX_POLY_POINTS 128
#define OAS_MAX_NAME_LEN 48
#define OAS_MAX_CLASS_LEN 8

enum AltRef : uint8_t {
  ALTREF_MSL,   // explicit ft MSL/AMSL
  ALTREF_AGL,   // explicit ft AGL, needs groundElevFt to resolve
  ALTREF_FL,    // flight level (value already converted to ft, see .cpp)
  ALTREF_SFC,   // surface / ground
  ALTREF_UNL    // unlimited
};

struct Altitude {
  float value_ft;   // raw value as parsed (0 for SFC, ignored for UNL)
  AltRef ref;
};

struct AirspaceResult {
  char name[OAS_MAX_NAME_LEN];
  char classId[OAS_MAX_CLASS_LEN];

  float horizDistance_km;   // 0 if current position is laterally inside
  float vertDistance_ft;    // 0 if current altitude is inside floor/ceiling
  bool insideHoriz;
  bool insideVert;

  float floor_ft_msl;       // resolved floor, for display/debugging
  float ceiling_ft_msl;     // resolved ceiling, for display/debugging
};

// Scans the whole file and returns the nearest controlled airspace by
// horizontal distance (ties broken by vertical distance). Returns false if
// no controlled airspace block could be parsed from the file at all.
//
// controlledClasses / numClasses: airspace AC-class strings (e.g. "C",
// "CTR") to be considered "controlled". Any block whose class is not in
// this list is measured but never returned as the result.
bool findNearestControlledAirspace(
    const char* filename,
    double curLat, double curLon, float curAlt_ft_msl,
    float groundElev_ft,
    AirspaceResult& out,
    const char** controlledClasses, uint8_t numClasses);

// Lower-level helpers, exposed in case you want to build your own scan
// (e.g. to log every controlled airspace within N km, not just the nearest).

double haversine_km(double lat1, double lon1, double lat2, double lon2);

// Resolves an OpenAir altitude to feet MSL using groundElev_ft for AGL refs.
float resolveAltitudeFt(const Altitude& alt, float groundElev_ft);
