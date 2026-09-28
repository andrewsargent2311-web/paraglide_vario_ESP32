#pragma once

// =====================================================
// AuxSensors: Battery, Ambient (SHTC3), and Clock/RTC
// -----------------------------------------------------
// Three small, independently-loaded subsystems, kept in one file pair
// per the component-split plan -- each is low-coupling enough that a
// dedicated file per subsystem wasn't worth the extra pair. Each section
// below keeps its original comment banner from the monolithic .ino so
// it's still obvious which lines came from which subsystem.
//
// setup()-time bring-up (ADC pin mode/attenuation, shtc3.begin(), the
// RTC presence probe + initial time seed) stays inline in the main
// .ino, unchanged -- only the declarations those blocks touch live here.
// Same for the periodic SHTC3 read in loop(): the original code has no
// dedicated update function for it, so none is invented here.
// =====================================================

#include <Arduino.h>
#include <time.h>
#include <Adafruit_SHTC3.h>
#include <Adafruit_Sensor.h>
#include "PCF85063A.h"

// =====================================================
// BATTERY MONITOR- Variables
// =====================================================
#define BATT_ADC_PIN 4
#define BATT_DIVIDER_RATIO 3.0f
#define BATT_EMPTY_V 2.5f
#define BATT_FULL_V 4.2f
#define BATT_SAMPLE_MS 2000
#define BATT_EMA_ALPHA 0.2f

// NOTE: in the original .ino these four variables were defined further
// down, interleaved with the vario's globals under the "Variov-
// Variables" banner, rather than next to the #defines above. Grouped
// with their #defines here -- same variables, same values, nothing
// about their behaviour changes.
extern float batteryVoltage;
extern uint8_t batteryPercent;
extern unsigned long lastBattSample;
extern bool battInitialized;

void updateBattery();

// =====================================================
// SHTC3 SENSOR (ambient temp/humidity -- also feeds top bar temp)
// =====================================================
extern Adafruit_SHTC3 shtc3;
extern bool shtc3OK;
extern float currentTempC;
#define SHT_SAMPLE_MS 10000
extern unsigned long lastShtSample;

// =====================================================
// PCF85063 HARDWARE RTC
// =====================================================
extern PCF85063A rtc;
extern unsigned long lastRtcPush;
#define PCF85063_I2C_ADDR 0x51
extern bool rtcOK;

// =====================================================
// TIME & SCHEDULING (clock-sync portion only)
// -----------------------------------------------------
// lastDisplayUpdate was declared under this same banner in the original
// file but belongs to the display redraw gate, not the clock -- it is
// NOT declared here. It lives in Display.h per the migration plan.
// =====================================================
// New Zealand timezone, including daylight saving.
#define NZ_TIMEZONE "NZST-12NZDT,M9.5.0,M4.1.0/3"
extern bool clockSynced;

// GPS corrects the clock once after each boot; the system clock and the
// hardware RTC then continue running without repeated GPS writes.
extern bool gpsClockSyncedThisBoot;

// Arduino-ESP32's supplied C library does not expose timegm(). Convert a
// validated UTC calendar time to Unix time without consulting the local TZ.
bool utcTmToEpoch(const struct tm& utc, time_t& epoch);

bool syncClockFromGPS();
