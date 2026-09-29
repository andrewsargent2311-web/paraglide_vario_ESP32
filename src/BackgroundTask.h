#pragma once

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// =====================================================
// BACKGROUND TASK (Core 0): everything that isn't needed for the
// paraglider page runs here, deliberately kept off Core 1 so it can
// never delay GPS/vario/audio/display/buttons.
// -----------------------------------------------------
// The last file in the extraction order, since it's the one place that
// ties together CrossCoreState, SdCard, AirspaceProximity, AdsbClient
// and WeatherClient, plus several existing external modules
// (wifi_manager, ble_manager, FileServer, TerrainDem, OpenAirScanner,
// menu). The task itself moves as one intact function -- its internal
// DEM-scan and airspace-scan sub-blocks travel with it unchanged, since
// they were never separate functions in the original .ino.
//
// xTaskCreatePinnedToCore(backgroundTask, ...) stays inline in setup(),
// unchanged -- only this declaration and the handle it's given moved
// here.
// =====================================================

extern TaskHandle_t backgroundTaskHandle;

void backgroundTask(void* parameter);
