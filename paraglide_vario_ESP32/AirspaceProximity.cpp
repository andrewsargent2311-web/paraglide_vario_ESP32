#include "AirspaceProximity.h"

// =====================================================
// AIRSPACE PROXIMITY (OpenAir file on SD card)
// =====================================================
// Nearest controlled airspace is scanned periodically on the existing
// Core 0 background task (see backgroundTask()) so a multi-hundred-KB SD
// read can never stall GPS/vario/button handling on Core 1. Result is
// written back under backgroundDataMutex, same pattern as the ADS-B
// aircraft list, and copied out under that same lock by the draw code.
static const char* AIRSPACE_FILE = "/AIRSPACE.TXT";
static const char* AIRSPACE_CONTROLLED_CLASSES[] = { "A", "B", "C", "D", "CTR" };
static const uint8_t AIRSPACE_NUM_CONTROLLED_CLASSES = 5;

// Ground elevation (ft MSL), used both to resolve AGL-referenced airspace
// floors and to compute the "ALTITUDE AGL" box on the paraglider page.
// GPS/baro altitude is height above sea level, not height above terrain,
// so this is now a live lookup into a pre-processed DEM tile on the SD
// card (see TerrainDem.h / selectedDemFile below) rather than a fixed site value.
// groundElevationValid is false until the first successful lookup, and
// goes false again if the aircraft flies outside the downloaded tile --
// both the airspace scanner and the AGL box must check it before trusting
// groundElevationFt.
float groundElevationFt = 0.0f;
volatile bool groundElevationValid = false;

unsigned long demScanAnchor = 0;
unsigned long airspaceScanAnchor = 0;


AirspaceResult nearestAirspace;
volatile bool airspaceResultValid = false;

// alertOnly = false counterpart of the above -- includes CFZ entries,
// feeds only the ADS-B page's "Airspace Info" bar. See the comment at
// the findNearestControlledAirspace() call site in backgroundTask().
AirspaceResult nearestAirspaceInfo;
volatile bool airspaceInfoResultValid = false;

// Feeds the VERT / HORI rows of the AIR SPACE box -- see
// findAirspaceClearances() in OpenAirScanner.h for exactly what each means.
AirspaceClearance airspaceClearance;
volatile bool airspaceClearanceValid = false;
