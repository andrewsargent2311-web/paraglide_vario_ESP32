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

// Recorder-side GPS sanity limits. A bad fix is logged as invalid using the
// last accepted position instead of adding a jump to the flight track.
// Position-check intervals are capped so a long GPS gap cannot legitimize a
// large jump.
#define IGC_MIN_SATELLITES 4
#define IGC_MAX_HDOP 5.0f
#define IGC_MAX_GROUND_SPEED_KPH 150.0f
#define IGC_POSITION_REANCHOR_FIXES 3
#define IGC_MAX_POSITION_INTERVAL_MS 10000UL
#define IGC_MAX_VERTICAL_SPEED_MPS 30.0f
#define IGC_ALTITUDE_JUMP_ALLOWANCE_M 30.0f

class IgcPositionFilter {
public:
  IgcPositionFilter();
  bool update(double lat, double lon, unsigned long now);
  void resetCandidate();
  bool hasAcceptedPosition() const;
  double acceptedLatitude() const;
  double acceptedLongitude() const;

private:
  bool addCandidate(double lat, double lon, unsigned long now);
  void accept(double lat, double lon, unsigned long now);

  double acceptedLat_;
  double acceptedLon_;
  bool haveAcceptedPosition_;
  unsigned long acceptedPositionAt_;
  double candidateLat_;
  double candidateLon_;
  unsigned long candidatePositionAt_;
  uint8_t candidateCount_;
};

extern File igcFile;
extern bool igcRecording;
extern char igcFilename[32];
extern unsigned long igcAboveThresholdSince;
extern unsigned long igcBelowThresholdSince;
extern unsigned long lastIgcFixWrite;

bool igcSpeedIsPlausible(float speedKph);
bool igcGpsAgeIndicatesNewData(uint32_t currentAge, uint32_t& previousAge);
bool igcPositionJumpIsPlausible(double fromLat, double fromLon,
                                double toLat, double toLon,
                                unsigned long elapsedMs);
bool igcAltitudeJumpIsPlausible(float fromAltitudeM, float toAltitudeM,
                                unsigned long elapsedMs);
void formatIgcLatLon(double lat, double lon, char* out, size_t outSize);
void writeIgcBRecord();
void startIgcRecording();
void stopIgcRecording();
void updateIgcRecorder();
void setFlightRecorderEnabled(bool enabled);
