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

// =====================================================
// IGC FLIGHT RECORDER
// =====================================================
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
  formatIgcLatLon(gps.location.lat(), gps.location.lng(), latLonBuf, sizeof(latLonBuf));

  bool fixValid = gps.location.isValid() && gps.location.age() < 2000 && gps.satellites.isValid() && gps.satellites.value() >= 4;

  int pressureAltM = bmpOK ? (int)roundf(currentAltitudeM) : 0;
  int gpsAltM = gps.altitude.isValid() ? (int)roundf(gps.altitude.meters()) : 0;

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
  if (!gps.speed.isValid()) return;

  float speedKph = gps.speed.kmph();
  unsigned long now = millis();

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
  if (igcAutoStopEnabled) {
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
