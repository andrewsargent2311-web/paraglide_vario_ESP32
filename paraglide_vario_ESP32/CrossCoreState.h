#pragma once

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

// =====================================================
// CrossCoreState: position/altitude handoff between Core 1 (loop()) and
// Core 0 (backgroundTask() and everything it calls -- ADS-B, Weather,
// Airspace, DEM).
// -----------------------------------------------------
// Owned by none of its readers -- loop() is the sole writer, everything
// on Core 0 only reads. backgroundDataMutex guards PositionSnapshot AND
// the ADS-B/Weather in-RAM data sets (adsbDoc, localMeters[],
// hasWeatherData) -- see the comment at its original declaration site.
//
// The original .ino never explicitly includes the FreeRTOS headers
// below -- SemaphoreHandle_t etc. arrive transitively via Arduino.h on
// ESP32. They're included explicitly here since this header is now the
// one place that declares the mutex; this doesn't change behaviour, it
// just makes this header self-sufficient rather than relying on
// whichever .cpp happens to include it first.
// =====================================================

// Cross-core position snapshot for backgroundTask() to read. Grouped into
// one struct (rather than individual volatiles like sharedGpsAltitudeFeet)
// because lat+lon+alt need to be read together as one consistent fix --
// TinyGPS++'s own fields aren't safe to read piecemeal from another core
// while gps.encode() is actively updating them in loop().
struct PositionSnapshot {
  double lat = 0, lon = 0;
  float altFt = 0;
  bool valid = false;
};

extern PositionSnapshot sharedPosition;

// Guards adsbDoc (ADS-B) and localMeters[]/hasWeatherData (weather) --
// both are written by backgroundTask() on Core 0 and read by the display
// draw functions on Core 1. Also guards sharedPosition above and the
// airspace/DEM result globals declared in AirspaceProximity.h.
extern SemaphoreHandle_t backgroundDataMutex;

// gps.altitude.feet() is written by loop() (Core 1) via gps.encode() and
// would otherwise be read directly by performADSBUpdate() running on the
// background task (Core 0) -- an unsynchronized cross-core read/write on
// the same TinyGPSPlus object. loop() refreshes this each pass instead,
// and the background task reads only this cached copy. A plain aligned
// float read/write is atomic on ESP32, so no mutex is needed for this
// single scalar handoff.
extern volatile float sharedGpsAltitudeFeet;
