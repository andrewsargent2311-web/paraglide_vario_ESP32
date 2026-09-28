#include "AuxSensors.h"
#include "CloudBase.h"
#include "Gps.h"
#include "Vario.h"  // currentPressureHpa (produced by the BMP580 code)
#include <Wire.h>
#include <math.h>
#include <sys/time.h>

// =====================================================
// BATTERY
// =====================================================
float batteryVoltage = 0.0f;
uint8_t batteryPercent = 0;
unsigned long lastBattSample = 0;
bool battInitialized = false;

void updateBattery() {
  uint32_t pinMv = analogReadMilliVolts(BATT_ADC_PIN);
  float rawVoltage = (pinMv / 1000.0f) * BATT_DIVIDER_RATIO;

  if (!battInitialized) {
    batteryVoltage = rawVoltage;
    battInitialized = true;
  } else {
    batteryVoltage = BATT_EMA_ALPHA * rawVoltage + (1.0f - BATT_EMA_ALPHA) * batteryVoltage;
  }

  float pct = (batteryVoltage - BATT_EMPTY_V) / (BATT_FULL_V - BATT_EMPTY_V) * 100.0f;
  batteryPercent = (uint8_t)constrain(pct, 0.0f, 100.0f);
}

// =====================================================
// SHTC3 AMBIENT SENSOR (+ cloud base estimate)
// =====================================================
Adafruit_SHTC3 shtc3;
bool shtc3OK = false;
float currentTempC = NAN;
float currentHumidityPct = NAN;   // raw SHTC3 relative humidity, %
float cloudBaseAboveM = NAN;      // smoothed estimate: metres from you up to cloud base (NAN = no estimate)
unsigned long lastShtSample = 0;

// =====================================================
// CLOUD BASE ESTIMATE
// Dew point comes from the SHTC3's own temp + humidity (they must come from
// the same sensor). The air temp is corrected for board self-heating, then
// combined with the BMP580 pressure to find the lifting condensation level.
// Result is metres from where you are up to cloud base, low-pass filtered.
// =====================================================
void updateCloudBaseEstimate() {
  if (isnan(currentTempC) || isnan(currentHumidityPct) || isnan(currentPressureHpa)) return;

  float tdC = cloudDewPointC(currentTempC, currentHumidityPct);
  float tAirC = currentTempC - SHT_SELF_HEAT_OFFSET_C;
  float rawM = cloudBaseHeightM(tAirC, tdC, currentPressureHpa);
  if (isnan(rawM)) return;

  if (isnan(cloudBaseAboveM)) {
    cloudBaseAboveM = rawM;
  } else {
    cloudBaseAboveM += CLOUD_BASE_SMOOTH_ALPHA * (rawM - cloudBaseAboveM);
  }
}

// =====================================================
// CLOCK / PCF85063 HARDWARE RTC
// =====================================================
PCF85063A rtc(&Wire);
unsigned long lastRtcPush = 0;
bool rtcOK = false;
bool clockSynced = false;

// GPS corrects the clock once after each boot; the system clock and the
// hardware RTC then continue running without repeated GPS writes.
bool gpsClockSyncedThisBoot = false;

// Arduino-ESP32's supplied C library does not expose timegm(). Convert a
// validated UTC calendar time to Unix time without consulting the local TZ.
bool utcTmToEpoch(const struct tm& utc, time_t& epoch) {
  const int year = utc.tm_year + 1900;
  const int month = utc.tm_mon;
  static const uint8_t daysInMonth[] = {
    31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31
  };

  if (year < 1970 || month < 0 || month > 11 || utc.tm_hour < 0 || utc.tm_hour > 23 || utc.tm_min < 0 || utc.tm_min > 59 || utc.tm_sec < 0 || utc.tm_sec > 59) {
    return false;
  }

  const bool leapYear = (year % 4 == 0 && (year % 100 != 0 || year % 400 == 0));
  int maxDay = daysInMonth[month] + (month == 1 && leapYear ? 1 : 0);
  if (utc.tm_mday < 1 || utc.tm_mday > maxDay) {
    return false;
  }

  int64_t days = 0;
  for (int y = 1970; y < year; ++y) {
    days += (y % 4 == 0 && (y % 100 != 0 || y % 400 == 0)) ? 366 : 365;
  }
  for (int m = 0; m < month; ++m) {
    days += daysInMonth[m] + (m == 1 && leapYear ? 1 : 0);
  }
  days += utc.tm_mday - 1;

  const int64_t seconds = days * 86400LL + utc.tm_hour * 3600L + utc.tm_min * 60L + utc.tm_sec;
  const time_t converted = (time_t)seconds;
  if ((int64_t)converted != seconds) {
    return false;  // Timestamp does not fit this core's time_t.
  }

  epoch = converted;
  return true;
}

bool syncClockFromGPS() {
  if (!gps.date.isValid() || !gps.time.isValid()) {
    return false;
  }

  struct tm t = {};

  t.tm_year = gps.date.year() - 1900;
  t.tm_mon = gps.date.month() - 1;
  t.tm_mday = gps.date.day();
  t.tm_hour = gps.time.hour();
  t.tm_min = gps.time.minute();
  t.tm_sec = gps.time.second();
  t.tm_isdst = 0;

  // GPS supplies UTC. Convert it without applying the NZ local timezone.
  time_t utcEpoch;
  if (!utcTmToEpoch(t, utcEpoch)) {
    return false;
  }

  struct timeval tv;
  tv.tv_sec = utcEpoch;
  tv.tv_usec = 0;

  settimeofday(&tv, nullptr);
  clockSynced = true;

  // Calculate weekday from the actual UTC timestamp.
  struct tm utcTm;
  gmtime_r(&utcEpoch, &utcTm);

  if (rtcOK && (lastRtcPush == 0 || millis() - lastRtcPush >= 5UL * 60UL * 1000UL)) {

    // The installed PCF85063A library takes the two-digit RTC year.
    uint8_t rtcYear = (uint8_t)((utcTm.tm_year + 1900) % 100);

    rtc.setTime(
      utcTm.tm_hour,
      utcTm.tm_min,
      utcTm.tm_sec);

    rtc.setDate(
      utcTm.tm_wday,
      utcTm.tm_mday,
      utcTm.tm_mon + 1,
      rtcYear);

    lastRtcPush = millis();
  }

  return true;
}
