#pragma once
#include "FreeRTOS.h"

inline BaseType_t xSemaphoreTake(SemaphoreHandle_t, TickType_t) { return pdFALSE; }
inline BaseType_t xSemaphoreGive(SemaphoreHandle_t) { return pdTRUE; }
