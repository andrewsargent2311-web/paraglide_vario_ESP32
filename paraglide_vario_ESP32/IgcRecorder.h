#pragma once
// IgcRecorder -- IGC flight logging (B-records, auto start/stop). SD mutex
// and mount state live in SdCard.
#include <Arduino.h>
#include <FS.h>

#define IGC_START_SPEED_KPH 10.0f
#define IGC_START_SUSTAIN_MS 10000UL

// Auto-stop (Flight Recordings > Auto-Stop, igcAutoStopEnabled in
// settings.h -- off by default). Previously this existed as
// IGC_STOP_SPEED_KPH/IGC_STOP_SUSTAIN_MS with a threshold of 0.0f kph,
// which GPS speed can never go below -- so recording never actually
// auto-stopped via this path; it only ever stopped by the pilot turning
// flightRecorderEnabled off (see setFlightRecorderEnabled()) or power
// off. Renamed now that it's a real, user-facing feature.
#define IGC_AUTOSTOP_SPEED_KPH 5.0f
#define IGC_AUTOSTOP_SUSTAIN_MS 20000UL

#define IGC_FIX_INTERVAL_MS 4000UL

extern File igcFile;
extern bool igcRecording;
extern char igcFilename[32];
extern unsigned long igcAboveThresholdSince;
extern unsigned long igcBelowThresholdSince;
extern unsigned long lastIgcFixWrite;

void formatIgcLatLon(double lat, double lon, char* out, size_t outSize);
void writeIgcBRecord();
void startIgcRecording();
void stopIgcRecording();
void updateIgcRecorder();
void setFlightRecorderEnabled(bool enabled);
