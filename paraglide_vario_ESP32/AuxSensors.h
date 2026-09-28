#pragma once
// AuxSensors -- three small, low-coupling subsystems in one file pair:
//   1. Battery monitor
//   2. SHTC3 ambient sensor (+ cloud base estimate)
//   3. Clock / hardware RTC
// Sensor/RTC init and the SHTC3 read stay inline in setup()/loop().
#include <Arduino.h>
#include <time.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_SHTC3.h>
#include "PCF85063A.h"

// =====================================================
// BATTERY
// =====================================================
#define BATT_ADC_PIN 4
#define BATT_DIVIDER_RATIO 3.0f
#define BATT_EMPTY_V 2.5f
#define BATT_FULL_V 4.2f
#define BATT_SAMPLE_MS 2000
#define BATT_EMA_ALPHA 0.2f
extern float batteryVoltage;
extern uint8_t batteryPercent;
extern unsigned long lastBattSample;
extern bool battInitialized;
void updateBattery();

// =====================================================
// SHTC3 AMBIENT SENSOR (+ cloud base estimate)
// =====================================================
// The SHTC3 sits on the Waveshare board next to the ESP32 and battery, so it reads
// warmer than the outside air. Dew point is NOT affected by that (warming air doesn't
// change its moisture), but the air temperature is, and that would make the cloud base
// read too high (about +125 m per degree C of error). Set this to how many degrees C
// your unit reads above a trusted thermometer after it has been on for ~10 minutes.
#define SHT_SELF_HEAT_OFFSET_C 0.0f
#define CLOUD_BASE_SMOOTH_ALPHA 0.3f  // 0..1, lower = smoother (samples every SHT_SAMPLE_MS)
#define SHT_SAMPLE_MS 10000
extern Adafruit_SHTC3 shtc3;
extern bool shtc3OK;
extern float currentTempC;
extern float currentHumidityPct;   // raw SHTC3 relative humidity, %
extern float cloudBaseAboveM;      // smoothed estimate: metres from you up to cloud base (NAN = no estimate)
extern unsigned long lastShtSample;
void updateCloudBaseEstimate();

// =====================================================
// CLOCK / PCF85063 HARDWARE RTC
// =====================================================
// New Zealand timezone, including daylight saving.
#define NZ_TIMEZONE "NZST-12NZDT,M9.5.0,M4.1.0/3"
#define PCF85063_I2C_ADDR 0x51
extern PCF85063A rtc;
extern unsigned long lastRtcPush;
extern bool rtcOK;
extern bool clockSynced;
extern bool gpsClockSyncedThisBoot;
bool syncClockFromGPS();
bool utcTmToEpoch(const struct tm& utc, time_t& epoch);
