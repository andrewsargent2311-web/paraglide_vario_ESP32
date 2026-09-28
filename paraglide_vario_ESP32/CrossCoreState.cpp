#include "CrossCoreState.h"

// Guards adsbDoc (ADS-B) and localMeters[]/hasWeatherData (weather) --
// both are written by backgroundTask() on Core 0 and read by the display
// draw functions on Core 1. Renamed from the earlier ADS-B-only name since
// it now protects both background data sets.
SemaphoreHandle_t backgroundDataMutex = nullptr;

PositionSnapshot sharedPosition;

NearestThreatSnapshot sharedNearestThreat;

// gps.altitude.feet() is written by loop() (Core 1) via gps.encode() and
// would otherwise be read directly by performADSBUpdate() running on the
// background task (Core 0) -- an unsynchronized cross-core read/write on the
// same TinyGPSPlus object. loop() refreshes this each pass instead, and
// the background task reads only this cached copy. A plain aligned float
// read/write is atomic on ESP32, so no mutex is needed for this single
// scalar handoff.
volatile float sharedGpsAltitudeFeet = 0.0f;
