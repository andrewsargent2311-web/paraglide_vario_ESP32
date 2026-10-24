#include "BackgroundTask.h"
#include "CrossCoreState.h"
#include "SdCard.h"
#include "AirspaceProximity.h"
#include "AdsbClient.h"
#include "WeatherClient.h"
#include "OpenAirScanner.h"
#include "TerrainDem.h"  // header-only -- include from THIS .cpp only
#include "wifi_manager.h"
#include "ble_manager.h"
#include "FileServer.h"
#include "menu.h"
#include "settings.h"
#include "VoiceAlert.h"
#include "Vario.h"

TaskHandle_t backgroundTaskHandle = nullptr;

// =====================================================
// BACKGROUND TASK (Core 0): everything that isn't needed for the
// paraglider page runs here, deliberately kept off Core 1 so it can never
// delay GPS/vario/audio/display/buttons. Checks every 500ms whether it's
// time to do any of its interval-gated work, then sleeps -- so it costs
// essentially nothing between polls.
// =====================================================
void backgroundTask(void* parameter) {

  for (;;) {

    uint32_t now = millis();

    if (sdCardOK) {
      voiceProfileService();
    }

    if (groundElevationValid &&
        (uint32_t)(now - groundElevationMs) > 30000UL) {
      groundElevationValid = false;
    }

    // ---------------------------------------------------------
    // Ground elevation (AGL): local SD file only, no WiFi needed. Runs
    // before the airspace scan below so a fresh groundElevationFt is
    // available for this same cycle's AGL-referenced airspace floors.
    // ---------------------------------------------------------
    if (sdCardOK && now - demScanAnchor >= DEM_SCAN_INTERVAL_MS) {
      demScanAnchor = now;

      PositionSnapshot demPos;
      bool demPosRead = false;
      if (backgroundDataMutex != nullptr && xSemaphoreTake(backgroundDataMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
        demPos = sharedPosition;
        demPosRead = true;
        xSemaphoreGive(backgroundDataMutex);
      }
      if (demPosRead && !demPos.valid) groundElevationValid = false;

      if (demPos.valid) {
        if (sdMutex != nullptr && xSemaphoreTake(sdMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
          float elevM;
          // getGroundElevationM() (TerrainDem.h) yields periodically via
          // vTaskDelay() while seeking into the DEM file now, so a deep
          // first-time seek into a large tile can't starve IDLE0 on core 0
          // long enough to trip the task watchdog. See seekWithYield() in
          // that file for why esp_task_wdt_reset() couldn't fix this
          // (BackgroundTask was never watchdog-registered in the first
          // place -- the trip was always about IDLE0, not this task).
          bool found = getGroundElevationM(selectedDemFile, demPos.lat, demPos.lon, elevM);
          xSemaphoreGive(sdMutex);

          if (found) {
            groundElevationFt = elevM * 3.28084f;
            groundElevationMs = millis();
            groundElevationValid = true;
          } else {
            groundElevationValid = false;  // outside tile / no DEM loaded
          }
        } else {
          Serial.println("[DEM] SD busy -- lookup skipped this cycle");
        }
      }
    }

    const uint32_t airspaceStartMs = millis();
    bool airspaceScanAttempted = false;
    // ---------------------------------------------------------
    // Airspace proximity: findNearestControlledAirspace() searches the
    // in-RAM cache only (see OpenAirScanner.h -- no SD access, no
    // parsing), so unlike the DEM lookup above this does NOT need
    // sdMutex. It previously took sdMutex here anyway, which meant it
    // competed with the IGC logger for the same lock every 10s for no
    // reason -- that contention (not actual SD I/O) was why scans were
    // being skipped.
    // ---------------------------------------------------------
    if (sdCardOK && now - airspaceScanAnchor >= AIRSPACE_SCAN_INTERVAL_MS) {
      airspaceScanAttempted = true;
      airspaceScanAnchor = now;

      PositionSnapshot pos;
      bool posRead = false;
      if (backgroundDataMutex != nullptr && xSemaphoreTake(backgroundDataMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
        pos = sharedPosition;
        posRead = true;
        xSemaphoreGive(backgroundDataMutex);
      }

      if (posRead && pos.valid) {
        AirspaceResult nearest;
        bool found = findNearestControlledAirspace(
          pos.lat,
          pos.lon,
          pos.altFt,
          groundElevationFt,
          groundElevationValid,
          /*alertOnly=*/true,
          nearest
        );

        AirspaceResult nearestInfo;
        bool foundInfo = findNearestControlledAirspace(
          pos.lat,
          pos.lon,
          pos.altFt,
          groundElevationFt,
          groundElevationValid,
          /*alertOnly=*/false,
          nearestInfo
        );

        AirspaceClearance clearance;
        bool foundClearance = findAirspaceClearances(
          pos.lat,
          pos.lon,
          pos.altFt,
          groundElevationFt,
          groundElevationValid,
          clearance
        );

        if (backgroundDataMutex != nullptr && xSemaphoreTake(backgroundDataMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
          if (foundClearance) {
            airspaceClearance = clearance;
            airspaceClearanceValid = true;
          } else {
            airspaceClearanceValid = false;
          }

          if (found) {
            nearestAirspace = nearest;
            airspaceResultValid = true;
          } else {
            airspaceResultValid = false;
          }

          if (foundInfo) {
            nearestAirspaceInfo = nearestInfo;
            airspaceInfoResultValid = true;
          } else {
            airspaceInfoResultValid = false;
          }
          airspaceScanDoneMs = millis();
          xSemaphoreGive(backgroundDataMutex);
        }
      }
      else if (posRead) {
        if (backgroundDataMutex != nullptr && xSemaphoreTake(backgroundDataMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
          airspaceResultValid = false;
          airspaceInfoResultValid = false;
          airspaceClearanceValid = false;
          airspaceScanDoneMs = millis();
          xSemaphoreGive(backgroundDataMutex);
        }
      }
    }
    if (VARIO_DIAG_PRINT && airspaceScanAttempted) {
      Serial.printf("[ASP] scan took %lums\n",
                    (unsigned long)(millis() - airspaceStartMs));
    }

    // ---------------------------------------------------------
    // Wi-Fi + Bluetooth connection management -- the connect/retry state
    // machines themselves now live in wifi_manager.cpp/ble_manager.cpp
    // (this used to all be inline here).
    // ---------------------------------------------------------
    wifiManagerLoop();
    bleManagerLoop();

    // Export Files (Flight Recordings > Export Files) -- serves IGC
    // files over WiFi when running. Deliberately on Core 0, not in the
    // main loop() on Core 1: handleClient() can block for a noticeable
    // stretch while streaming a file to a slow/distant client, and Core
    // 1 must never be delayed (see this function's header comment).
    // No-ops immediately if the server isn't currently started.
    fileServerLoop();

    // ---------------------------------------------------------
    // Network state machines
    // ---------------------------------------------------------
    if (wifiConnected) {

      // ADS-B: only poll every ADSB_INTERVAL_MS
      if (now - lastAdsbCheckTime >= ADSB_INTERVAL_MS) {
        lastAdsbCheckTime = now;
        adsbTaskRunning = true;
        performADSBUpdate();
        adsbTaskRunning = false;
      }
      // Weather: first poll fires WEATHER_FIRST_POLL_DELAY_MS after boot;
      // every poll after that reverts to the menu-adjustable weatherPollIntervalMs cadence.
      unsigned long weatherDueInterval = weatherFirstPollDone ? weatherPollIntervalMs : WEATHER_FIRST_POLL_DELAY_MS;

      if (now - weatherTimerAnchor >= weatherDueInterval) {
        weatherTimerAnchor = now;
        weatherFirstPollDone = true;
        Serial.println("[MAIN] Calling weather update...");
        updateWeather();
      }
    }

    // All real work above is interval-gated (5s / 10s or longer), so this task
    // spends nearly all its time asleep here rather than busy-polling.
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}
