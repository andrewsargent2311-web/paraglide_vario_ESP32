#pragma once
// Link-only stand-in for FreeRTOS (native tests only).
// The native tests never take or give a semaphore: xSemaphoreTake() always
// reports "not obtained", so a test that reaches the SD-writing paths would
// see a busy card. Do not assert against this behaviour.
typedef void*        SemaphoreHandle_t;
typedef int          BaseType_t;
typedef unsigned int TickType_t;

#define pdTRUE  1
#define pdFALSE 0
#define pdMS_TO_TICKS(ms) ((TickType_t)(ms))
