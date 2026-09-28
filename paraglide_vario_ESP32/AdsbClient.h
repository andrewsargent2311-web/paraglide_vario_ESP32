#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <ArduinoJson.h>

#include "CrossCoreState.h"  // PositionSnapshot, sharedPosition, backgroundDataMutex, sharedGpsAltitudeFeet
#include "settings.h"        // adsbRingOuterKm, adsbAlertRadiusKm, adsbAlertVerticalFt

// =====================================================
// ADS-B CLIENT (adsb.fi polling + threat detection)
// -----------------------------------------------------
// Runs on the Core 0 background task, gated on WiFi + a valid GPS fix.
// BackgroundTask (not yet extracted) is what actually calls
// performADSBUpdate() on its own poll timer -- that call site isn't
// part of this file.
//
// Note what this file does NOT own: the alarm SOUND is Buzzer's
// (loop()'s inline threat-check block reads adsbNewThreat and sets
// Buzzer's interceptAlarmActive directly -- that block isn't called
// from here), and hasAdsbData's only reader, drawADSBPage(), lives in
// DrawPages.cpp. This file just polls, detects threats, and flips flags
// for those to read.
// =====================================================

extern DynamicJsonDocument adsbDoc;  // Single shared ADS-B document; avoids a second JSON copy.
extern volatile bool hasAdsbData;
extern volatile bool adsbTaskRunning;
extern volatile bool adsbNewThreat;

extern unsigned long lastAdsbCheckTime;       // Stores the last time we requested data
extern const unsigned long ADSB_INTERVAL_MS;  // Poll the server every 15 seconds

#define MAX_TRACKED_THREATS 10
extern char activeThreatHexes[MAX_TRACKED_THREATS][9];  // Fixed-size ADS-B hex IDs
extern int activeThreatCount;

// Declared in the original file but never read or written anywhere else
// found in this codebase -- looks vestigial. Carried forward as-is.
extern bool conflictDetectedThisFrame;

void performADSBUpdate();
