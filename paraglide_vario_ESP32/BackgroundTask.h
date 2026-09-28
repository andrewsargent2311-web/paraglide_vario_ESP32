#pragma once
// BackgroundTask -- the Core 0 loop: Wi-Fi/BLE/file-server loops, ADS-B and
// weather poll gating, DEM lookup, airspace scan. Task creation stays in
// setup().
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

extern TaskHandle_t backgroundTaskHandle;
void backgroundTask(void* parameter);
