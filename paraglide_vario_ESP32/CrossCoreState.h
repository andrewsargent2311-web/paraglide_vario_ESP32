#pragma once
// CrossCoreState -- Core 1 -> Core 0 handoff (position, nearest-threat
// snapshot, the mutex that guards them and other background data).
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

// Cross-core position snapshot for backgroundTask() to read. Grouped into
// one struct (rather than individual volatiles like sharedGpsAltitudeFeet)
// because lat+lon+alt need to be read together as one consistent fix --
// TinyGPS++'s own fields aren't safe to read piecemeal from another core
// while gps.encode() is actively updating them in loop().
struct PositionSnapshot {
  double lat = 0, lon = 0;
  float altFt = 0;
  bool valid = false;     // fresh fix right now (age < 2000ms) -- used by DEM/airspace scan, unchanged
  bool everValid = false; // true forever once the GPS has produced at least one real fix --
                           // lat/lon above still hold whatever the last fix was, stale or not.
                           // Used by ADS-B/weather so they run off the last known position
                           // instead of skipping a poll just because the fix is momentarily stale.
};

// Published by performADSBUpdate() (Core 0) each poll, alongside
// activeThreatHexes above -- same mutex, same publish point. Holds full
// detail on whichever currently-alerting aircraft is nearest, so the
// main loop can speak/sound an alert about it without re-deriving
// anything from the raw ADS-B JSON (which isn't safely readable outside
// performADSBUpdate() anyway). valid=false means no aircraft is
// currently inside the alert radius/vertical band at all.
struct NearestThreatSnapshot {
  bool valid = false;
  char hex[9] = "";
  float distanceKm = 0;
  float verticalDeltaFt = 0;   // always >= 0 -- see aircraftAbove for direction
  bool aircraftAbove = false;
  float altitudeFt = 0;        // aircraft's absolute altitude (alt_baro)
  float bearingFromMeDeg = 0;  // compass bearing from the pilot to the aircraft
  bool headingKnown = false;
  float headingDeg = 0;        // aircraft's own track/heading, if the feed has it
};

// How the last ADS-B or weather fetch went, for the "No internet" message on
// those pages. Each is written on Core 0 and read on Core 1: a single aligned
// value, so no mutex is needed.
enum FetchStatus {
  FETCH_NOT_TRIED = 0,
  FETCH_OK,
  FETCH_FAILED
};

// Reports FETCH_FAILED when it goes out of scope, unless ok was set to true
// first -- so every early return in a long fetch function counts as a failure
// without each one having to say so.
struct FetchStatusScope {
  volatile FetchStatus& status;
  bool ok = false;
  explicit FetchStatusScope(volatile FetchStatus& s) : status(s) {}
  ~FetchStatusScope() { status = ok ? FETCH_OK : FETCH_FAILED; }
};

extern volatile FetchStatus adsbFetchStatus;
extern volatile FetchStatus weatherFetchStatus;

// Goes up by one each time performADSBUpdate() publishes a new adsbDoc, with
// the time it did so. Both are written and read under backgroundDataMutex,
// which is how the traffic page knows there's something new to merge.
extern uint32_t adsbFetchGeneration;
extern unsigned long adsbDocFetchMs;

extern PositionSnapshot sharedPosition;
extern NearestThreatSnapshot sharedNearestThreat;
extern SemaphoreHandle_t backgroundDataMutex;
extern volatile float sharedGpsAltitudeFeet;
extern volatile float sharedOwnPressureAltFeet;
