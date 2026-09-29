#include "CrossCoreState.h"

PositionSnapshot sharedPosition;

// Actually created with xSemaphoreCreateMutex() inline in setup(), same
// as in the original file -- only this declaration moved.
SemaphoreHandle_t backgroundDataMutex = nullptr;

volatile float sharedGpsAltitudeFeet = 0.0f;
