#include "IgcRecorder.h"
#include "SdCard.h"
#include "Gps.h"
#include "Vario.h"
#include "settings.h"
#include <SD_MMC.h>
#include <time.h>
#include <math.h>

File igcFile;
bool igcRecording = false;
char igcFilename[32] = "";

unsigned long igcAboveThresholdSince = 0;
unsigned long igcBelowThresholdSince = 0;
unsigned long lastIgcFixWrite = 0;

namespace {
IgcPositionFilter positionFilter;
uint32_t lastLocationAge = UINT32_MAX;
bool latestPositionAccepted = false;

float acceptedGpsAltitudeM = 0.0f;
bool haveAcceptedGpsAltitude = false;
unsigned long acceptedGpsAltitudeAt = 0;
uint32_t lastAltitudeAge = UINT32_MAX;

double distanceMeters(double lat1, double lon1, double lat2, double lon2) {
  const double lat1Rad = radians(lat1);
  const double lat2Rad = radians(lat2);
  const double deltaLat = radians(lat2 - lat1);
  const double deltaLon = radians(lon2 - lon1);
  const double a = sin(deltaLat / 2.0) * sin(deltaLat / 2.0) +
                   cos(lat1Rad) * cos(lat2Rad) *
                   sin(deltaLon / 2.0) * sin(deltaLon / 2.0);
  return 6371000.0 * 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
}

unsigned long cappedPositionInterval(unsigned long elapsedMs) {
  return elapsedMs > IGC_MAX_POSITION_INTERVAL_MS
           ? IGC_MAX_POSITION_INTERVAL_MS
           : elapsedMs;
}

bool gpsQualityIsGood() {
  return gps.location.isValid() && gps.location.age() < 2000 &&
         gps.satellites.isValid() && gps.satellites.age() < 2000 &&
         gps.satellites.value() >= IGC_MIN_SATELLITES &&
         gps.hdop.isValid() && gps.hdop.age() < 2000 &&
         gps.hdop.hdop() <= IGC_MAX_HDOP;
}

void updateAcceptedGpsFix(unsigned long now) {
  const uint32_t locationAge = gps.location.age();
  if (igcGpsAgeIndicatesNewData(locationAge, lastLocationAge)) {
    latestPositionAccepted = false;

    if (gpsQualityIsGood()) {
      const double lat = gps.location.lat();
      const double lon = gps.location.lng();
      const bool coordinatesInRange =
        isfinite(lat) && isfinite(lon) && lat >= -90.0 && lat <= 90.0 &&
        lon >= -180.0 && lon <= 180.0;
      if (coordinatesInRange) {
        latestPositionAccepted = positionFilter.update(lat, lon, now);
        if (!latestPositionAccepted) {
          Serial.println(positionFilter.hasAcceptedPosition()
                           ? "[IGC] GPS position jump rejected"
                           : "[IGC] GPS position awaiting consistent fixes");
        }
      } else {
        positionFilter.resetCandidate();
        latestPositionAccepted = false;
        Serial.println("[IGC] GPS coordinates out of range");
      }
    } else {
      positionFilter.resetCandidate();
      latestPositionAccepted = false;
      Serial.println("[IGC] GPS fix rejected: stale or poor quality");
    }
  } else if (!gps.location.isValid() || locationAge >= 2000) {
    latestPositionAccepted = false;
  }

  const uint32_t altitudeAge = gps.altitude.age();
  if (igcGpsAgeIndicatesNewData(altitudeAge, lastAltitudeAge)) {
    if (gps.altitude.isValid() && altitudeAge < 2000) {
      const float altitudeM = gps.altitude.meters();
      const unsigned long elapsed = now - acceptedGpsAltitudeAt;
      if (isfinite(altitudeM) &&
          (!haveAcceptedGpsAltitude ||
           igcAltitudeJumpIsPlausible(acceptedGpsAltitudeM, altitudeM, elapsed))) {
        acceptedGpsAltitudeM = altitudeM;
        acceptedGpsAltitudeAt = now;
        haveAcceptedGpsAltitude = true;
      } else if (isfinite(altitudeM)) {
        Serial.println("[IGC] GPS altitude jump rejected");
      }
    }
  }
}
}  // namespace

IgcPositionFilter::IgcPositionFilter()
  : acceptedLat_(0.0),
    acceptedLon_(0.0),
    haveAcceptedPosition_(false),
    acceptedPositionAt_(0),
    candidateLat_(0.0),
    candidateLon_(0.0),
    candidatePositionAt_(0),
    candidateCount_(0) {}

bool IgcPositionFilter::update(double lat, double lon, unsigned long now) {
  if (!isfinite(lat) || !isfinite(lon) ||
      lat < -90.0 || lat > 90.0 || lon < -180.0 || lon > 180.0) {
    resetCandidate();
    return false;
  }

  if (!haveAcceptedPosition_) {
    if (addCandidate(lat, lon, now)) {
      accept(lat, lon, now);
      return true;
    }
    return false;
  }

  const unsigned long elapsed = cappedPositionInterval(now - acceptedPositionAt_);
  if (igcPositionJumpIsPlausible(acceptedLat_, acceptedLon_, lat, lon, elapsed)) {
    accept(lat, lon, now);
    return true;
  }

  if (addCandidate(lat, lon, now)) {
    accept(lat, lon, now);
    return true;
  }
  return false;
}

bool IgcPositionFilter::addCandidate(double lat, double lon,
                                     unsigned long now) {
  const unsigned long elapsed = cappedPositionInterval(now - candidatePositionAt_);
  const bool consistent =
    candidateCount_ > 0 &&
    igcPositionJumpIsPlausible(candidateLat_, candidateLon_, lat, lon, elapsed);
  candidateCount_ = consistent ? candidateCount_ + 1 : 1;
  candidateLat_ = lat;
  candidateLon_ = lon;
  candidatePositionAt_ = now;
  return candidateCount_ >= IGC_POSITION_REANCHOR_FIXES;
}

void IgcPositionFilter::accept(double lat, double lon, unsigned long now) {
  acceptedLat_ = lat;
  acceptedLon_ = lon;
  acceptedPositionAt_ = now;
  haveAcceptedPosition_ = true;
  resetCandidate();
}

void IgcPositionFilter::resetCandidate() {
  candidateCount_ = 0;
}

bool IgcPositionFilter::hasAcceptedPosition() const {
  return haveAcceptedPosition_;
}

double IgcPositionFilter::acceptedLatitude() const {
  return acceptedLat_;
}

double IgcPositionFilter::acceptedLongitude() const {
  return acceptedLon_;
}

// =====================================================
// IGC FLIGHT RECORDER
// =====================================================
bool igcSpeedIsPlausible(float speedKph) {
  return isfinite(speedKph) && speedKph >= 0.0f &&
         speedKph <= IGC_MAX_GROUND_SPEED_KPH;
}

bool igcGpsAgeIndicatesNewData(uint32_t currentAge, uint32_t& previousAge) {
  const bool isNewData = currentAge < previousAge;
  previousAge = currentAge;
  return isNewData;
}

bool igcPositionJumpIsPlausible(double fromLat, double fromLon,
                                double toLat, double toLon,
                                unsigned long elapsedMs) {
  if (!isfinite(fromLat) || !isfinite(fromLon) ||
      !isfinite(toLat) || !isfinite(toLon) ||
      fromLat < -90.0 || fromLat > 90.0 || toLat < -90.0 || toLat > 90.0 ||
      fromLon < -180.0 || fromLon > 180.0 || toLon < -180.0 || toLon > 180.0) {
    return false;
  }
  if (elapsedMs == 0) return fromLat == toLat && fromLon == toLon;
  const double speedKph =
    distanceMeters(fromLat, fromLon, toLat, toLon) * 3600.0 / elapsedMs;
  return isfinite(speedKph) && speedKph <= IGC_MAX_GROUND_SPEED_KPH;
}

bool igcAltitudeJumpIsPlausible(float fromAltitudeM, float toAltitudeM,
                                unsigned long elapsedMs) {
  if (!isfinite(fromAltitudeM) || !isfinite(toAltitudeM)) return false;
  const float allowedChangeM =
    IGC_ALTITUDE_JUMP_ALLOWANCE_M +
    IGC_MAX_VERTICAL_SPEED_MPS * (elapsedMs / 1000.0f);
  return fabsf(toAltitudeM - fromAltitudeM) <= allowedChangeM;
}

void formatIgcLatLon(double lat, double lon, char* out, size_t outSize) {
  char latHemi = (lat >= 0) ? 'N' : 'S';
  char lonHemi = (lon >= 0) ? 'E' : 'W';

  double absLat = fabs(lat);
  int latDeg = (int)absLat;
  double latMinFull = (absLat - latDeg) * 60.0;
  int latMinInt = (int)latMinFull;
  int latMinFrac = (int)roundf((latMinFull - latMinInt) * 1000.0f);

  double absLon = fabs(lon);
  int lonDeg = (int)absLon;
  double lonMinFull = (absLon - lonDeg) * 60.0;
  int lonMinInt = (int)lonMinFull;
  int lonMinFrac = (int)roundf((lonMinFull - lonMinInt) * 1000.0f);

  snprintf(out, outSize, "%02d%02d%03d%c%03d%02d%03d%c",
           latDeg, latMinInt, latMinFrac, latHemi,
           lonDeg, lonMinInt, lonMinFrac, lonHemi);
}
void writeIgcBRecord() {
  if (!igcFile) return;

  time_t nowEpoch;
  time(&nowEpoch);
  struct tm utcTm;
  gmtime_r(&nowEpoch, &utcTm);

  char latLonBuf[24];
  formatIgcLatLon(positionFilter.hasAcceptedPosition()
                    ? positionFilter.acceptedLatitude() : 0.0,
                  positionFilter.hasAcceptedPosition()
                    ? positionFilter.acceptedLongitude() : 0.0,
                  latLonBuf, sizeof(latLonBuf));

  bool fixValid = latestPositionAccepted && gpsQualityIsGood();

  int pressureAltM = bmpOK ? (int)roundf(currentAltitudeM) : 0;
  int gpsAltM = haveAcceptedGpsAltitude ? (int)roundf(acceptedGpsAltitudeM) : 0;

  char bRecord[64];
  snprintf(bRecord, sizeof(bRecord), "B%02d%02d%02d%s%c%05d%05d",
           utcTm.tm_hour, utcTm.tm_min, utcTm.tm_sec,
           latLonBuf, fixValid ? 'A' : 'V',
           pressureAltM, gpsAltM);

  // Shared with the Core 0 airspace scan -- see sdMutex declaration. A
  // fix is only 4s apart (IGC_FIX_INTERVAL_MS) so a short wait here is
  // fine; if the airspace scan is genuinely stuck this skips one fix
  // rather than blocking flight logging indefinitely.
  if (sdMutex != nullptr && xSemaphoreTake(sdMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
    igcFile.println(bRecord);
    igcFile.flush();  // flush every fix -- a lost flight log is worse than the SD write cost
    xSemaphoreGive(sdMutex);
  } else {
    Serial.println("[IGC] SD busy -- fix skipped this cycle");
  }
}
void startIgcRecording() {
  if (igcRecording || !sdCardOK) return;

  time_t nowEpoch;
  time(&nowEpoch);
  struct tm utcTm;
  gmtime_r(&nowEpoch, &utcTm);

  snprintf(igcFilename, sizeof(igcFilename), "/%02d_%02d_%04d_%02d%02d%02d.IGC",
           utcTm.tm_mday, utcTm.tm_mon + 1, utcTm.tm_year + 1900,
           utcTm.tm_hour, utcTm.tm_min, utcTm.tm_sec);

  if (sdMutex == nullptr || xSemaphoreTake(sdMutex, pdMS_TO_TICKS(200)) != pdTRUE) {
    Serial.println("[IGC] SD busy -- could not start recording this cycle");
    return;
  }

  igcFile = SD_MMC.open(igcFilename, FILE_WRITE);
  if (!igcFile) {
    Serial.printf("[IGC] Failed to open %s\n", igcFilename);
    xSemaphoreGive(sdMutex);
    return;
  }

  igcFile.println("AXXXFC1 Paraglide Flight Computer");
  igcFile.printf("HFDTE%02d%02d%02d\n", utcTm.tm_mday, utcTm.tm_mon + 1, (utcTm.tm_year + 1900) % 100);
  igcFile.println("HFFTYFRTYPE:DIY ESP32-S3 Flight Computer");
  igcFile.println("HFGPS:Quectel LC76G");
  igcFile.println("HFPRSPRESSALTSENSOR:Bosch BMP580");
  igcFile.println("HFDTM100GPSDATUM:WGS-1984");
  igcFile.flush();
  xSemaphoreGive(sdMutex);

  igcRecording = true;
  lastIgcFixWrite = 0;
  Serial.printf("[IGC] Recording started: %s\n", igcFilename);
}
void stopIgcRecording() {
  if (!igcRecording) return;

  if (sdMutex != nullptr && xSemaphoreTake(sdMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
    igcFile.close();
    xSemaphoreGive(sdMutex);
  } else {
    Serial.println("[IGC] SD busy -- closing file without the lock (best effort)");
    igcFile.close();
  }

  igcRecording = false;
  Serial.printf("[IGC] Recording stopped: %s\n", igcFilename);
}
void updateIgcRecorder() {
  if (!flightRecorderEnabled) return;

  unsigned long now = millis();
  updateAcceptedGpsFix(now);
  bool speedValid = latestPositionAccepted && gpsQualityIsGood() &&
                    gps.speed.isValid() && gps.speed.age() < 2000 &&
                    igcSpeedIsPlausible(gps.speed.kmph());
  if (!speedValid) {
    igcAboveThresholdSince = 0;
    igcBelowThresholdSince = 0;
    if (!igcRecording) return;
  }
  float speedKph = speedValid ? gps.speed.kmph() : 0.0f;

  if (!igcRecording) {
    if (speedKph >= IGC_START_SPEED_KPH) {
      if (igcAboveThresholdSince == 0) {
        igcAboveThresholdSince = now;
      } else if (now - igcAboveThresholdSince >= IGC_START_SUSTAIN_MS) {
        startIgcRecording();
        igcAboveThresholdSince = 0;
      }
    } else {
      igcAboveThresholdSince = 0;
    }
    return;
  }

  // Already recording. Auto-stop is opt-in (igcAutoStopEnabled, off by
  // default, settings.h) -- when off, recording only ever stops via
  // flightRecorderEnabled being switched off or power loss, same as
  // before this feature existed.
  if (igcAutoStopEnabled && speedValid) {
    if (speedKph < IGC_AUTOSTOP_SPEED_KPH) {
      if (igcBelowThresholdSince == 0) {
        igcBelowThresholdSince = now;
      } else if (now - igcBelowThresholdSince >= IGC_AUTOSTOP_SUSTAIN_MS) {
        stopIgcRecording();
        igcBelowThresholdSince = 0;
        return;
      }
    } else {
      igcBelowThresholdSince = 0;
    }
  } else {
    // Keep the timer clean in case the setting gets re-enabled later in
    // the same flight -- otherwise a stale timestamp from before it was
    // turned off could make the very next low-speed moment look like
    // it's already been sustained for a while.
    igcBelowThresholdSince = 0;
  }

  if (now - lastIgcFixWrite >= IGC_FIX_INTERVAL_MS) {
    lastIgcFixWrite = now;
    writeIgcBRecord();
  }
}

// =====================================================
// FLIGHT RECORDER ENABLE/DISABLE (Flight Recordings > Recording)
// Called from menu.cpp any time the pilot flips the toggle. No boot-time
// call needed -- flightRecorderEnabled (settings.h) is never persisted,
// so it's already true (its compiled-in default) the moment setup()
// runs; updateIgcRecorder() reads it directly every call regardless.
// =====================================================
void setFlightRecorderEnabled(bool enabled) {
  flightRecorderEnabled = enabled;

  if (!enabled && igcRecording) {
    // Close cleanly rather than leave the file open-but-dangling --
    // updateIgcRecorder() is gated on flightRecorderEnabled and would
    // otherwise just stop being called at all from this point on,
    // without ever reaching its normal landing-detected stopIgcRecording()
    // path.
    stopIgcRecording();
  }

  Serial.printf("[IGC] Flight recorder %s\n", enabled ? "enabled" : "disabled");
}
