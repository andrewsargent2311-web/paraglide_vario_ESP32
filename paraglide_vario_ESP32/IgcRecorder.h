#pragma once

#include <Arduino.h>
#include <FS.h>
#include <time.h>

#include "SdCard.h"    // sdCardOK, sdMutex
#include "Gps.h"       // gps
#include "Vario.h"     // bmpOK, currentAltitudeM
#include "settings.h"  // flightRecorderEnabled

// =====================================================
// IGC FLIGHT RECORDER
// -----------------------------------------------------
// Auto-starts recording once groundspeed has stayed above
// IGC_START_SPEED_KPH for IGC_START_SUSTAIN_MS, auto-stops once it's
// stayed below IGC_STOP_SPEED_KPH for IGC_STOP_SUSTAIN_MS, and writes
// one IGC B-record every IGC_FIX_INTERVAL_MS while recording.
// igcFile/sdMutex access is shared with the Core 0 airspace/DEM scanner
// -- see the sdMutex comment in SdCard.h.
// =====================================================

extern File igcFile;
extern bool igcRecording;
extern char igcFilename[32];

#define IGC_START_SPEED_KPH 10.0f
#define IGC_STOP_SPEED_KPH 0.0f
#define IGC_START_SUSTAIN_MS 10000UL
#define IGC_STOP_SUSTAIN_MS 10000UL
#define IGC_FIX_INTERVAL_MS 4000UL

extern unsigned long igcAboveThresholdSince;
extern unsigned long igcBelowThresholdSince;
extern unsigned long lastIgcFixWrite;

void formatIgcLatLon(double lat, double lon, char* out, size_t outSize);
void writeIgcBRecord();
void startIgcRecording();
void stopIgcRecording();
void updateIgcRecorder();

// =====================================================
// FLIGHT RECORDER ENABLE/DISABLE (Flight Recordings > Recording)
// Called from menu.cpp any time the pilot flips the toggle. No boot-time
// call needed -- flightRecorderEnabled (settings.h) is never persisted,
// so it's already true (its compiled-in default) the moment setup()
// runs; updateIgcRecorder() reads it directly every call regardless.
// =====================================================
void setFlightRecorderEnabled(bool enabled);
