#include "AirspaceProximity.h"

float groundElevationFt = 0.0f;
volatile bool groundElevationValid = false;

unsigned long demScanAnchor = 0;
unsigned long airspaceScanAnchor = 0;

AirspaceResult nearestAirspace;
volatile bool airspaceResultValid = false;

AirspaceResult nearestAirspaceInfo;
volatile bool airspaceInfoResultValid = false;

// NOTE: these three were declared at file scope in the original .ino,
// static (internal linkage), right next to the block above -- but
// nothing else in the codebase provided appears to read them. setup()'s
// call to loadAirspaceDatabase() uses its own, differently-named,
// locally-scoped CONTROLLED_CLASSES[] array instead of these. Carried
// forward here, unused, per the no-cleanup rule -- worth confirming
// they're truly dead across the whole project before removing them.
static const char* AIRSPACE_FILE = "/AIRSPACE.TXT";
static const char* AIRSPACE_CONTROLLED_CLASSES[] = { "A", "B", "C", "D", "CTR" };
static const uint8_t AIRSPACE_NUM_CONTROLLED_CLASSES = 5;
