#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <ArduinoJson.h>

#include "DrawPages.h"  // WindMeter, TRACKED_METERS

// =====================================================
// WEATHER CLIENT (Zephyr network station polling)
// -----------------------------------------------------
// Runs on the Core 0 background task. The actual "is it time to poll
// yet" decision -- which reads weatherPollIntervalMs from settings.h --
// stays inline in backgroundTask() (not yet extracted), not here; this
// file just declares the timer state that decision reads/writes, and
// does the poll itself once called.
// =====================================================

// Allocated with `new` in setup(), after Serial is confirmed alive --
// that allocation stays inline in setup(); only this pointer declaration
// moved here.
extern WiFiClientSecure* globalSecureWeatherClient;
extern bool secureWeatherClientInitialized;

// WindMeter, WEATHER_STATION_NAME_MAX, and TRACKED_METERS live in
// DrawPages.h (shared with drawWeatherPage(), which reads this array).
extern WindMeter localMeters[TRACKED_METERS];
extern volatile bool hasWeatherData;

// Steady-state poll cadence is weatherPollIntervalMs (settings.h),
// editable from Weather Settings > Poll Interval -- read only by
// backgroundTask(), not by this file.
extern const unsigned long WEATHER_FIRST_POLL_DELAY_MS;  // First poll fires 15s after boot
extern unsigned long weatherTimerAnchor;                 // Fresh, clean background timer
extern bool weatherFirstPollDone;                        // True once the initial poll has fired

// ---------------------------------------------------------
// Find the end of one JSON object in a JSON array.
//
// Starts at '{' and returns the number of characters
// occupied by the complete object, including the braces.
//
// Handles nested objects/arrays and braces inside strings.
// ---------------------------------------------------------
size_t findJsonObjectLength(const char* start, size_t remaining);

void updateWeather();
