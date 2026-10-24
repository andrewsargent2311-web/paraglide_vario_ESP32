#include "AdsbClient.h"
#include "CrossCoreState.h"
#include "Utils.h"
#include "settings.h"
#include "Gps.h"
#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <string.h>
#include <math.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

volatile bool adsbTaskRunning = false;
volatile bool adsbNewThreat = false;

JsonDocument adsbDoc;  // Single shared ADS-B document; avoids a second JSON copy.
bool conflictDetectedThisFrame = false;
volatile bool hasAdsbData = false;

unsigned long lastAdsbCheckTime = 0;           // Stores the last time we requested data
unsigned long lastAdsbAlertAnnounceMs = 0;

char activeThreatHexes[MAX_TRACKED_THREATS][9] = {};  // Fixed-size ADS-B hex IDs
int activeThreatCount = 0;
static char* adsbBody = nullptr;
static size_t adsbBodyCap = 0;

// =====================================================
// ADSB data handling
// =====================================================
void performADSBUpdate() {

  Serial.printf("[ADS-B] performADSBUpdate() starting on core %d, free heap: %u bytes\n",
                xPortGetCoreID(), ESP.getFreeHeap());
  if (WiFi.status() != WL_CONNECTED) {
    return;
  }

  // Snapshot the live GPS fix -- same mutex pattern used for the DEM/
  // airspace scans further down loop(). Uses everValid (the GPS has
  // produced at least one real fix, ever) rather than valid (fresh right
  // now): myPos.lat/lon hold the last known position regardless of how
  // stale the fix currently is, so a momentary dropout doesn't skip this
  // poll -- at typical paraglider ground speeds the position from a few
  // seconds ago is still close enough to be useful.
  PositionSnapshot myPos;
  if (backgroundDataMutex != nullptr && xSemaphoreTake(backgroundDataMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
    myPos = sharedPosition;
    xSemaphoreGive(backgroundDataMutex);
  }
  if (!myPos.everValid) {
    Serial.println("[ADS-B] No GPS fix yet -- skipping this poll");
    return;
  }

  HTTPClient http;
  WiFiClientSecure client;

  client.setInsecure();

  String domain = "https://opendata.adsb.fi";
  String apiPath = "/api/v3/lat/";

  // Requests exactly the current far/default range ring's outer radius
  // (see the Range Rings menu, ADSB_SETTINGS > Range Rings) -- adsb.fi
  // filters server-side by this "dist" value, so it must be at least as
  // large as whatever drawADSBPage() might draw out to, or the outer part
  // of a wider ring would always appear empty regardless of real traffic.
  // A menu change here takes effect on the next scheduled poll, not
  // instantly.
  int queryRadiusKm = (int)adsbRingOuterKm;

  String url =
    domain + apiPath + String(myPos.lat, 4) + "/lon/" + String(myPos.lon, 4) + "/dist/" + String(queryRadiusKm);

  Serial.print("[ADS-B] Connecting to: ");
  Serial.println(url);

  // adsb.fi's response is dynamically generated JSON with no known length
  // up front, so it comes back as Transfer-Encoding: chunked. Reading
  // chunked data straight off http.getStreamPtr() below skips HTTPClient's
  // own chunk-decoding (that only runs inside getString()/writeToStream()),
  // so ArduinoJson would see raw chunk-size lines (e.g. "1a3\r\n") instead
  // of the opening '{' -- which is exactly the "InvalidInput" seen in the
  // logs, on every request, regardless of network conditions. Forcing
  // HTTP/1.0 makes the server send Content-Length instead of chunking, so
  // the stream is plain JSON again. Must be set before http.begin().
  http.useHTTP10(true);

  if (!http.begin(client, url)) {
    Serial.println("[ADS-B] http.begin() FAILED");
    hasAdsbData = false;
    return;
  }

  http.setTimeout(2500);
  http.setUserAgent("ESP32-S3-Flight-Computer/1.0");

  int httpCode = http.GET();

  Serial.printf(
    "[ADS-B] HTTP Response Code: %d\n",
    httpCode);

  if (httpCode != HTTP_CODE_OK) {

    hasAdsbData = false;

    Serial.printf(
      "[ADS-B] Network Error: %s (%d)\n",
      http.errorToString(httpCode).c_str(),
      httpCode);

    http.end();
    return;
  }

  static bool allocationAttempted = false;
  static bool allocationFailureLogged = false;
  static bool overflowLogged = false;
  if (!allocationAttempted) {
    allocationAttempted = true;
    // DEVIATION(P1-2): one-time allocation on first poll, never freed.
    adsbBody = (char*)ps_malloc(49152);
    if (adsbBody != nullptr) {
      adsbBodyCap = 49152;
    } else {
      adsbBody = (char*)malloc(24576);
      if (adsbBody != nullptr) adsbBodyCap = 24576;
    }
  }
  if (adsbBody == nullptr) {
    if (!allocationFailureLogged) {
      Serial.println("[ADS-B] Unable to allocate response buffer");
      allocationFailureLogged = true;
    }
    hasAdsbData = false;
    http.end();
    return;
  }

  WiFiClient* adsbStream = http.getStreamPtr();
  if (adsbStream == nullptr) {
    Serial.println("[ADS-B] HTTP response stream unavailable");
    hasAdsbData = false;
    http.end();
    return;
  }
  const int responseSize = http.getSize();
  size_t bodyLength = 0;
  if (responseSize > 0) {
    if ((size_t)responseSize >= adsbBodyCap) {
      if (!overflowLogged) {
        Serial.println("[ADS-B] Response body exceeds buffer capacity");
        overflowLogged = true;
      }
      hasAdsbData = false;
      http.end();
      return;
    }
    bodyLength = adsbStream->readBytes(adsbBody, (size_t)responseSize);
    if (bodyLength != (size_t)responseSize) {
      Serial.printf("[ADS-B] Incomplete response body: %u/%d bytes\n",
                    (unsigned)bodyLength, responseSize);
      hasAdsbData = false;
      http.end();
      return;
    }
  } else {
    bodyLength = adsbStream->readBytes(adsbBody, adsbBodyCap - 1);
    if (bodyLength == adsbBodyCap - 1 && adsbStream->available() > 0) {
      if (!overflowLogged) {
        Serial.println("[ADS-B] Response body exceeds buffer capacity");
        overflowLogged = true;
      }
      hasAdsbData = false;
      http.end();
      return;
    }
  }
  adsbBody[bodyLength] = '\0';
  http.end();
  Serial.printf("[ADS-B] Response body bytes read: %u\n", (unsigned)bodyLength);

  // ---------------------------------------------------------
  // Parse into the shared document
  // ---------------------------------------------------------
  if (backgroundDataMutex == nullptr ||
      xSemaphoreTake(backgroundDataMutex, pdMS_TO_TICKS(500)) != pdTRUE) {
    hasAdsbData = false;
    Serial.println("[ADS-B] Timed out waiting for background data mutex");
    return;
  }

  adsbDoc.clear();

  DeserializationError error =
    deserializeJson(adsbDoc, adsbBody, bodyLength);

  if (error) {

    hasAdsbData = false;

    if (backgroundDataMutex != nullptr) {
      xSemaphoreGive(backgroundDataMutex);
    }

    Serial.print("[ADS-B] JSON Data Error: ");
    Serial.println(error.c_str());

    return;
  }

  JsonArray aircraftList =
    adsbDoc["ac"].as<JsonArray>();

  Serial.printf(
    "[ADS-B] Total fetched aircraft: %d\n",
    aircraftList.size());

  // ---------------------------------------------------------
  // Remove slow ground traffic
  // ---------------------------------------------------------

  for (int i = aircraftList.size() - 1;
       i >= 0;
       i--) {

    JsonObject ac = aircraftList[i];

    if (!ac["gs"].is<float>() || (float)ac["gs"] < 10.0f) {

      aircraftList.remove(i);
    }
  }

  Serial.printf(
    "[ADS-B] Mapped %d aircraft after 10kt speed filter.\n",
    aircraftList.size());

  // ---------------------------------------------------------
  // Threat detection
  // ---------------------------------------------------------

  bool brandNewThreatDetected = false;

  float myAltitudeFeet = sharedOwnPressureAltFeet;

  char currentFrameThreatHexes[MAX_TRACKED_THREATS][9] = {};
  int currentFrameThreatCount = 0;

  // Built up alongside currentFrameThreatHexes above -- whichever
  // qualifying aircraft ends up closest this poll becomes the published
  // sharedNearestThreat (see below the loop).
  NearestThreatSnapshot nearestThisFrame;
  float nearestDistanceSoFar = 1.0e9f;

  for (JsonObject ac : aircraftList) {

    if (!ac["lat"].is<float>() || !ac["lon"].is<float>() ||
        !ac["hex"].is<const char*>()) {

      continue;
    }
    if (!ac["alt_baro"].is<float>()) continue;

    float acLat = ac["lat"];
    float acLon = ac["lon"];

    const char* acHex = ac["hex"];
    if (acHex == nullptr) {
      continue;  // "hex" key present but not a string -- can't track this one safely
    }

    if (acLat == 0.0f || acLon == 0.0f) {
      continue;
    }

    float distanceKM =
      getDistanceKM(
        myPos.lat,
        myPos.lon,
        acLat,
        acLon);

    float acAltitudeFt = ac["alt_baro"];
    float verticalDeltaFeet =
      fabsf(
        acAltitudeFt - myAltitudeFeet);

    if (distanceKM <= adsbAlertRadiusKm && verticalDeltaFeet <= adsbAlertVerticalFt) {

      if (currentFrameThreatCount < MAX_TRACKED_THREATS) {

        strncpy(currentFrameThreatHexes[currentFrameThreatCount],
                acHex,
                sizeof(currentFrameThreatHexes[0]) - 1);
        currentFrameThreatHexes[currentFrameThreatCount]
                               [sizeof(currentFrameThreatHexes[0]) - 1] = '\0';

        currentFrameThreatCount++;
      }

      if (distanceKM < nearestDistanceSoFar) {
        nearestDistanceSoFar = distanceKM;
        nearestThisFrame.valid = true;
        strncpy(nearestThisFrame.hex, acHex, sizeof(nearestThisFrame.hex) - 1);
        nearestThisFrame.hex[sizeof(nearestThisFrame.hex) - 1] = '\0';
        nearestThisFrame.distanceKm = distanceKM;
        nearestThisFrame.verticalDeltaFt = verticalDeltaFeet;
        nearestThisFrame.aircraftAbove = (acAltitudeFt >= myAltitudeFeet);
        nearestThisFrame.altitudeFt = acAltitudeFt;
        nearestThisFrame.bearingFromMeDeg = getBearing(myPos.lat, myPos.lon, acLat, acLon);
        // "track" (heading) isn't sent by every aircraft/feed state --
        // ADSB Settings > the voice alert says "heading unknown" rather
        // than guessing when it's missing.
        nearestThisFrame.headingKnown = ac["track"].is<float>();
        nearestThisFrame.headingDeg = nearestThisFrame.headingKnown ? (float)ac["track"] : 0.0f;
      }

      bool isExistingThreat = false;

      for (int i = 0;
           i < activeThreatCount;
           i++) {

        if (strcmp(activeThreatHexes[i], acHex) == 0) {

          isExistingThreat = true;
          break;
        }
      }

      if (!isExistingThreat) {
        brandNewThreatDetected = true;
      }
    }
  }

  // ---------------------------------------------------------
  // Update threat history
  // ---------------------------------------------------------

  activeThreatCount =
    currentFrameThreatCount;

  for (int i = 0;
       i < activeThreatCount;
       i++) {

    strncpy(activeThreatHexes[i],
            currentFrameThreatHexes[i],
            sizeof(activeThreatHexes[0]) - 1);
    activeThreatHexes[i][sizeof(activeThreatHexes[0]) - 1] = '\0';
  }

  // nearestThisFrame.valid stays false (its default) if no aircraft
  // qualified this poll, which correctly clears any previous alert
  // target once nothing's in range any more.
  sharedNearestThreat = nearestThisFrame;

  // ---------------------------------------------------------
  // Publish completed ADS-B data
  // ---------------------------------------------------------
  // adsbDoc is already the published document; no deep copy is needed.
  hasAdsbData = (aircraftList.size() > 0);

  if (backgroundDataMutex != nullptr) {
    xSemaphoreGive(backgroundDataMutex);
  }

  // ---------------------------------------------------------
  // Notify main loop of new threat
  // ---------------------------------------------------------

  if (brandNewThreatDetected) {
    adsbNewThreat = true;

    Serial.println(
      "[RADAR INTERCEPT] New aircraft detected!");
  }
}
