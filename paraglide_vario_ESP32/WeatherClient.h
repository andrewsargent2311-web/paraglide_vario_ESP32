#pragma once
// WeatherClient -- Zephyr weather-network poll/parse (updateWeather) and its
// shared state. The "when to poll" decision is in BackgroundTask.cpp.
#include <Arduino.h>
#include <WiFiClientSecure.h>
#include "DrawPages.h"  // WindMeter, TRACKED_METERS

// Steady-state poll cadence is now weatherPollIntervalMs (settings.h),
// editable from Weather Settings > Poll Interval.
constexpr unsigned long WEATHER_FIRST_POLL_DELAY_MS = 15UL * 1000UL;  // First poll fires 15s after boot

extern WiFiClientSecure* globalSecureWeatherClient;
extern bool secureWeatherClientInitialized;
extern WindMeter localMeters[TRACKED_METERS];
extern volatile bool hasWeatherData;
extern unsigned long weatherTimerAnchor;
extern bool weatherFirstPollDone;

size_t findJsonObjectLength(const char* start, size_t remaining);
void updateWeather();
