#include "SdCard.h"

bool sdCardOK = false;

// Actually created with xSemaphoreCreateMutex() inline in setup(), same
// as in the original file -- only this declaration moved.
SemaphoreHandle_t sdMutex = nullptr;
