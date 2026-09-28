#pragma once

#include <Arduino.h>
#include <FS.h>
#include <SD_MMC.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

// =====================================================
// SD CARD (shared infrastructure)
// -----------------------------------------------------
// The microSD slot on the Waveshare ESP32-S3-RLCD-4.2 is wired to the
// ESP32-S3's native SDMMC peripheral in 1-bit mode, NOT to SPI -- there
// is no CS line run to the card at all. CLK/CMD/D0 below match
// Waveshare's own SD card example for this exact board (02_Example/
// Arduino/06_SD_Card). Unlike classic ESP32, the S3's SDMMC pins are
// routed through the GPIO matrix, so they must be assigned with
// SD_MMC.setPins() before SD_MMC.begin() -- that call, and the
// SD_MMC.begin() call itself, both stay inline in setup(); only the
// declarations below moved here.
//
// sdMutex guards ALL SD card access -- it's shared between the IGC
// logger (writeIgcBRecord() etc., called from loop() on Core 1) and the
// airspace/DEM scanner (called from backgroundTask() on Core 0). Both
// sides must take it before touching the card and give it back
// immediately after. Deliberately kept in its own header rather than
// folded into the eventual IgcRecorder file, since IgcRecorder is only
// one of its two consumers.
//
// igcFile / igcRecording / igcFilename are NOT declared here even
// though they're SD-related -- they're IGC-specific state and move with
// IgcRecorder in a later extraction step, not this one.
// =====================================================

#define SD_MMC_CLK_PIN 38
#define SD_MMC_CMD_PIN 21
#define SD_MMC_D0_PIN 39

extern bool sdCardOK;
extern SemaphoreHandle_t sdMutex;
