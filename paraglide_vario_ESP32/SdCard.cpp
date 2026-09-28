#include "SdCard.h"

bool sdCardOK = false;

// Guards ALL SD card access. The card is shared between two cores now:
// the IGC logger (writeIgcBRecord() etc, called from loop() on Core 1) and
// the airspace scanner (called from backgroundTask() on Core 0). Without
// this, a scan and a log write could hit the SPI bus at the same moment
// from two different tasks -- worst case, a corrupted IGC file. Both sides
// must take this before touching SD and give it back immediately after.
SemaphoreHandle_t sdMutex = nullptr;
