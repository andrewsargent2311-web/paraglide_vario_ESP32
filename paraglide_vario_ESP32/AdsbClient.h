#pragma once
// AdsbClient -- ADS-B poll/parse (performADSBUpdate) and its shared state.
// The "when to poll" decision is in BackgroundTask.cpp; the alarm sound is
// in Buzzer.cpp.
#include <Arduino.h>
#include <ArduinoJson.h>

const unsigned long ADSB_INTERVAL_MS = 15000;  // Poll the server every 15 seconds

// Re-alerts (siren or voice, whichever is selected) every this many ms
// while any aircraft remains inside the alert radius/vertical band --
// reset by every alert (new or repeat), not on a fixed clock, so a
// repeat alert doesn't fire right on the heels of an immediate one.
// See the "4.5 ADS-B new intruder" block in loop().
#define ADSB_ALERT_REPEAT_MS 60000UL

#define MAX_TRACKED_THREATS 10

extern volatile bool adsbTaskRunning;
extern volatile bool adsbNewThreat;
extern JsonDocument adsbDoc;
extern bool conflictDetectedThisFrame;
extern volatile bool hasAdsbData;
extern unsigned long lastAdsbCheckTime;
extern unsigned long lastAdsbAlertAnnounceMs;
extern char activeThreatHexes[MAX_TRACKED_THREATS][9];
extern int activeThreatCount;

void performADSBUpdate();
