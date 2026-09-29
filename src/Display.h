#pragma once

#include <Arduino.h>
#include <U8g2lib.h>

#include "settings.h"  // screenOrientation, SCREEN_ORIENTATION_GPS_TOP

// =====================================================
// DISPLAY (ESP32-S3-RLCD-4.2, 300x400 portrait, R0 orientation)
// -----------------------------------------------------
// The boot splash draw itself (malloc'ing a buffer, reading /ROY.BIN off
// SD, the u8g2.firstPage()/nextPage() loop, and the "hold the splash on
// screen for at least SPLASH_DISPLAY_MS" timing) has no persistent
// globals of its own -- every variable involved is local to setup() --
// so it stays inline there entirely, unchanged. Only the u8g2 object and
// the SPLASH_* #defines it references move here.
//
// displayDirty and lastDisplayUpdate were declared under two different
// banners in the original .ino (displayDirty under "Gestures"/page
// button, lastDisplayUpdate under "TIME & SCHEDULING") but both exist
// purely to drive loop()'s display redraw gate -- per the plan's
// ownership call, they belong here, not with PageButton or the Clock
// section of AuxSensors.
// =====================================================

#define RLCD_SCK 11
#define RLCD_MOSI 12
#define RLCD_DC 5
#define RLCD_CS 40
#define RLCD_RST 41

extern U8G2_ST7305_300X400_1_4W_HW_SPI u8g2;

// =====================================================
// BOOT SPLASH IMAGE (Roy)
// -----------------------------------------------------
// Loaded from the SD card as a raw 1bpp packed bitmap (XBM byte layout:
// rows padded to a whole byte, bits LSB-first, 1 = black/ink) rather than
// baked into the firmware, so it's easy to swap the picture just by
// replacing the file on the card. Generated from Roy.png at 280x210 --
// see the accompanying ROY.BIN this was built from.
// =====================================================
#define SPLASH_IMG_FILE "/ROY.BIN"
#define SPLASH_IMG_W 280
#define SPLASH_IMG_H 210
#define SPLASH_IMG_ROW_BYTES ((SPLASH_IMG_W + 7) / 8)
#define SPLASH_IMG_BYTES (SPLASH_IMG_ROW_BYTES * SPLASH_IMG_H)
#define SPLASH_DISPLAY_MS 3000UL

// Set whenever page/menu state changes; drives an immediate redraw
// instead of waiting for the next 1Hz display tick, so menu navigation
// feels responsive rather than laggy.
extern bool displayDirty;
extern unsigned long lastDisplayUpdate;

// =====================================================
// SCREEN ORIENTATION (Config > Screen)
// Called once from setup() right after u8g2.begin(), and again
// immediately from menu.cpp any time the pilot changes it. U8G2_R0 is
// the app's original (GPS Bottom) orientation; U8G2_R2 is the same
// panel rotated 180 degrees (GPS Top).
// =====================================================
void applyScreenOrientation();
