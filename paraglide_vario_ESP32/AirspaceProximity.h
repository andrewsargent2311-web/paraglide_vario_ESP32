#pragma once
// AirspaceProximity -- shared RESULT state of the DEM (ground elevation)
// and airspace scans. The scans themselves run in BackgroundTask.cpp.
#include <Arduino.h>
#include "OpenAirScanner.h"  // AirspaceResult

extern float groundElevationFt;
extern volatile bool groundElevationValid;

// Produced offline from a LINZ DEM GeoTIFF by dem_to_agldem.py (downscaled
// + reprojected to WGS84 lat/lon) -- see that script for how to (re)build
// this for a different flying site.
// The active filename is now chosen from the menu's Map screen (see
// selectedDemFile / setSelectedDemFile() in menu.h/.cpp) rather than fixed
// here -- it defaults to "/DEM.ADEM" until changed.
#define DEM_SCAN_INTERVAL_MS 5000UL

#define AIRSPACE_SCAN_INTERVAL_MS 10000UL

extern unsigned long demScanAnchor;
extern unsigned long airspaceScanAnchor;

extern AirspaceResult nearestAirspace;
extern volatile bool airspaceResultValid;
extern AirspaceResult nearestAirspaceInfo;
extern volatile bool airspaceInfoResultValid;

// Independent horizontal-at-my-altitude / vertical-above-or-below-me
// answers (findAirspaceClearances()). Feeds only the VERT / HORI rows of
// the AIR SPACE box on the Paraglider and Paramotor pages.
extern AirspaceClearance airspaceClearance;
extern volatile bool airspaceClearanceValid;
