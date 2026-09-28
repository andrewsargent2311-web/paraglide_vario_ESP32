#pragma once

#include <Arduino.h>

#include "OpenAirScanner.h"  // AirspaceResult

// =====================================================
// AIRSPACE PROXIMITY (OpenAir file on SD card) + TERRAIN (DEM) LOOKUP
// -----------------------------------------------------
// Nearest controlled airspace is scanned periodically on the Core 0
// background task (see BackgroundTask.cpp) so a multi-hundred-KB SD read
// can never stall GPS/vario/button handling on Core 1. Ground elevation
// (for AGL-referenced airspace floors and the paraglider page's ALTITUDE
// AGL box) is looked up from a DEM tile on the same schedule, just
// before the airspace scan each cycle so a fresh value is available to
// it. Both scans, and the DEM lookup call itself, stay inline inside
// BackgroundTask.cpp's main loop -- this file only declares the shared
// result state they read/write.
//
// AIRSPACE_FILE, AIRSPACE_CONTROLLED_CLASSES[] and
// AIRSPACE_NUM_CONTROLLED_CLASSES are declared in the .cpp, not here --
// see the note there. They look like unused duplicates of a differently
// named, locally-scoped array that setup() actually passes to
// loadAirspaceDatabase(); nothing else in the codebase provided
// references these three. Carried forward, unused, per the no-cleanup
// rule.
// =====================================================

// Ground elevation (ft MSL). GPS/baro altitude is height above sea
// level, not height above terrain -- groundElevationValid is false
// until the first successful DEM lookup, and goes false again if the
// aircraft flies outside the downloaded tile. Both the airspace scanner
// and the AGL box (drawParagliderPage()) must check it before trusting
// groundElevationFt.
extern float groundElevationFt;
extern volatile bool groundElevationValid;

// The active DEM filename is chosen from the menu's Map screen
// (selectedDemFile / setSelectedDemFile() in menu.h/.cpp), not fixed
// here -- it defaults to "/DEM.ADEM" until changed.
#define DEM_SCAN_INTERVAL_MS 5000UL
extern unsigned long demScanAnchor;

#define AIRSPACE_SCAN_INTERVAL_MS 10000UL
extern unsigned long airspaceScanAnchor;

extern AirspaceResult nearestAirspace;
extern volatile bool airspaceResultValid;

// alertOnly = false counterpart of the above -- includes CFZ entries,
// feeds only the ADS-B page's "Airspace Info" bar. See the comment at
// the findNearestControlledAirspace() call site in BackgroundTask.cpp.
extern AirspaceResult nearestAirspaceInfo;
extern volatile bool airspaceInfoResultValid;
