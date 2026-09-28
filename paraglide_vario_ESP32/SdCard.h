#pragma once
// SdCard -- shared SD_MMC state. SD mounting stays inline in setup().
#include <Arduino.h>
#include <FS.h>
#include <SD_MMC.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

// =====================================================
// SD CARD / IGC FLIGHT LOG
// =====================================================

// The microSD slot on the Waveshare ESP32-S3-RLCD-4.2 is wired to the
// ESP32-S3's native SDMMC peripheral in 1-bit mode, NOT to SPI -- there is
// no CS line run to the card at all, which is why treating pin 1 as a CS
// pin never worked. CLK/CMD/D0 below match Waveshare's own SD card example
// for this exact board (02_Example/Arduino/06_SD_Card). Unlike classic
// ESP32, the S3's SDMMC pins are routed through the GPIO matrix, so they
// must be assigned with SD_MMC.setPins() before SD_MMC.begin().
#define SD_MMC_CLK_PIN 38
#define SD_MMC_CMD_PIN 21
#define SD_MMC_D0_PIN 39

extern bool sdCardOK;
extern SemaphoreHandle_t sdMutex;
