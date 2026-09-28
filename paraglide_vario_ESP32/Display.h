#pragma once
// Display -- the u8g2 panel object, redraw flags, splash constants and
// screen orientation. Page drawing lives in DrawPages.cpp.
#include <Arduino.h>
#include <U8g2lib.h>

// =====================================================
// ESP32-S3-RLCD-4.2 DISPLAY
// =====================================================
// R0 is 90 degrees anticlockwise from the current R1 landscape layout,
// giving a 300px wide x 400px tall portrait canvas.
#define RLCD_SCK 11
#define RLCD_MOSI 12
#define RLCD_DC 5
#define RLCD_CS 40
#define RLCD_RST 41

extern U8G2_ST7305_300X400_1_4W_HW_SPI u8g2;

// =====================================================
// BOOT SPLASH IMAGE (Roy)
// =====================================================
// Loaded from the SD card as a raw 1bpp packed bitmap (XBM byte layout:
// rows padded to a whole byte, bits LSB-first, 1 = black/ink) rather than
// baked into the firmware, so it's easy to swap the picture just by
// replacing the file on the card. Generated from Roy.png at 280x210 --
// see the accompanying ROY.BIN this was built from.
#define SPLASH_IMG_FILE "/ROY.BIN"
#define SPLASH_IMG_W 280
#define SPLASH_IMG_H 210
#define SPLASH_IMG_ROW_BYTES ((SPLASH_IMG_W + 7) / 8)
#define SPLASH_IMG_BYTES (SPLASH_IMG_ROW_BYTES * SPLASH_IMG_H)
#define SPLASH_DISPLAY_MS 3000UL

extern bool displayDirty;
extern unsigned long lastDisplayUpdate;
void applyScreenOrientation();
