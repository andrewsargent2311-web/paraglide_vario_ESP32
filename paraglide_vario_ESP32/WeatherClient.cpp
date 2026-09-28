#include "WeatherClient.h"
#include "CrossCoreState.h"
#include "Utils.h"
#include "Gps.h"
#include "settings.h"
#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <ArduinoJson.h>
#include <string.h>
#include <math.h>

// Add this right next to your weatherTimerAnchor global variable
WiFiClientSecure* globalSecureWeatherClient = nullptr;  // allocated in setup(), after Serial is confirmed alive
bool secureWeatherClientInitialized = false;

// WindMeter, WEATHER_STATION_NAME_MAX, and TRACKED_METERS now live in
// DrawPages.h (shared with drawWeatherPage(), which reads this array).
WindMeter localMeters[TRACKED_METERS];
volatile bool hasWeatherData = false;

unsigned long weatherTimerAnchor = 0;                                 // Fresh, clean background timer
bool weatherFirstPollDone = false;                                    // True once the initial 10s poll has fired

//=====================================================
// Weather data handling
//=====================================================
// ---------------------------------------------------------
// Find the end of one JSON object in a JSON array.
//
// Starts at '{' and returns the number of characters
// occupied by the complete object, including the braces.
//
// Handles nested objects/arrays and braces inside strings.
// ---------------------------------------------------------
size_t findJsonObjectLength(const char* start, size_t remaining) {

  if (start == nullptr || remaining == 0 || *start != '{') {
    return 0;
  }

  int depth = 0;
  bool inString = false;
  bool escaped = false;

  for (size_t i = 0; i < remaining; i++) {

    char c = start[i];

    // -----------------------------------------------------
    // Handle JSON strings
    // -----------------------------------------------------
    if (inString) {

      if (escaped) {
        escaped = false;
        continue;
      }

      if (c == '\\') {
        escaped = true;
        continue;
      }

      if (c == '"') {
        inString = false;
      }

      continue;
    }

    // -----------------------------------------------------
    // Outside a string
    // -----------------------------------------------------
    if (c == '"') {
      inString = true;
      continue;
    }

    if (c == '{') {
      depth++;
    }
    else if (c == '}') {

      depth--;

      if (depth == 0) {
        return i + 1;
      }
    }
  }

  // Incomplete/malformed object
  return 0;
}

void updateWeather() {
  Serial.println("[Zephyr] ENTERED weather function");

  // ---------------------------------------------------------
  // Wi-Fi check
  // ---------------------------------------------------------
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[Zephyr] WiFi not connected");
    hasWeatherData = false;
    return;
  }

  // ---------------------------------------------------------
  // GPS check -- station distance/bearing below need a real position to
  // measure from. Same sharedPosition snapshot pattern used by the
  // DEM/airspace scans and performADSBUpdate(). Uses everValid, not
  // valid -- see performADSBUpdate()'s comment on the same pattern for
  // why a stale-but-known position is fine here rather than skipping.
  // ---------------------------------------------------------
  PositionSnapshot myPos;
  if (backgroundDataMutex != nullptr && xSemaphoreTake(backgroundDataMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
    myPos = sharedPosition;
    xSemaphoreGive(backgroundDataMutex);
  }
  if (!myPos.everValid) {
    Serial.println("[Zephyr] No GPS fix yet -- skipping this poll");
    hasWeatherData = false;
    return;
  }

  // ---------------------------------------------------------
  // Initialise TLS client once
  // ---------------------------------------------------------
  if (!secureWeatherClientInitialized) {
    globalSecureWeatherClient->setInsecure();
    secureWeatherClientInitialized = true;
  }

  HTTPClient http;

  const char* url = "https://api.zephyrapp.nz/stations";

  Serial.println("[Zephyr] Starting station update...");

  // ---------------------------------------------------------
  // Start HTTP connection
  // ---------------------------------------------------------
  if (!http.begin(*globalSecureWeatherClient, url)) {
    Serial.println("[Zephyr] http.begin() FAILED");
    hasWeatherData = false;
    return;
  }

  http.setConnectTimeout(4500);
  http.setTimeout(4500);
  http.setUserAgent("ESP32-S3-Flight-Computer/1.0");
  http.addHeader("Accept-Encoding", "identity");

  // ---------------------------------------------------------
  // HTTP GET
  // ---------------------------------------------------------
  Serial.printf(
    "[Zephyr] Heap before HTTP GET: %u | Min heap: %u\n",
    ESP.getFreeHeap(),
    ESP.getMinFreeHeap());

  int httpCode = http.GET();

  Serial.printf(
    "[Zephyr] HTTP Response Code: %d\n",
    httpCode);

  // ---------------------------------------------------------
  // Check HTTP response
  // ---------------------------------------------------------
  if (httpCode != HTTP_CODE_OK) {

    Serial.printf(
      "[Zephyr] HTTP failure: %d\n",
      httpCode);

    Serial.printf(
      "[Zephyr] Error: %s\n",
      http.errorToString(httpCode).c_str());

    hasWeatherData = false;

    http.end();

    Serial.printf(
      "[Zephyr] HTTP connection closed after failure. "
      "Free heap: %u | Min heap: %u\n",
      ESP.getFreeHeap(),
      ESP.getMinFreeHeap());

    return;
  }

  // ---------------------------------------------------------
  // Report response size
  // ---------------------------------------------------------
  int responseSize = http.getSize();

  Serial.printf(
    "[Zephyr] HTTP Content-Length / response size: %d bytes\n",
    responseSize);

  // ---------------------------------------------------------
  // Download the COMPLETE response
  //
  // This is deliberately using getString() rather than
  // feeding the WiFiClient directly into ArduinoJson.
  //
  // The working implementation proves that HTTPClient can
  // successfully retrieve the complete 251 KB response.
  // ---------------------------------------------------------
  Serial.printf(
    "[Zephyr] Heap before getString: %u | Min heap: %u\n",
    ESP.getFreeHeap(),
    ESP.getMinFreeHeap());

  String payload = http.getString();

  Serial.printf(
    "[Zephyr] Payload size: %u bytes\n",
    payload.length());

  Serial.printf(
    "[Zephyr] Heap after getString: %u | Min heap: %u\n",
    ESP.getFreeHeap(),
    ESP.getMinFreeHeap());

  // ---------------------------------------------------------
  // Validate response
  // ---------------------------------------------------------
  if (payload.length() == 0) {

    Serial.println("[Zephyr] ERROR: Empty response");

    hasWeatherData = false;

    http.end();

    Serial.printf(
      "[Zephyr] HTTP connection closed after empty response. "
      "Free heap: %u | Min heap: %u\n",
      ESP.getFreeHeap(),
      ESP.getMinFreeHeap());

    return;
  }

  // ---------------------------------------------------------
  // Debug response contents
  //
  // Keep these while diagnosing the API response.
  // They can be removed later.
  // ---------------------------------------------------------
  Serial.println("[Zephyr] First 300 bytes:");

  Serial.println(
    payload.substring(
      0,
      min((size_t)300, payload.length())));

  Serial.println("[Zephyr] Last 100 bytes:");

  if (payload.length() > 100) {
    Serial.println(
      payload.substring(
        payload.length() - 100));
  } else {
    Serial.println(payload);
  }

  // ---------------------------------------------------------
  // ArduinoJson filter
    // ---------------------------------------------------------
  // We deliberately DO NOT deserialize the entire 550-station
  // array into one ArduinoJson document.
  //
  // Instead:
  //
  //   downloaded payload
  //          |
  //          v
  //   find one station object
  //          |
  //          v
  //   parse that station only
  //          |
  //          v
  //   keep/discard it
  //          |
  //          v
  //   clear small JSON document
  //          |
  //          v
  //   next station
  //
  // This keeps RAM usage essentially independent of the number
  // of stations in the Zephyr response.
  // ---------------------------------------------------------

  // ---------------------------------------------------------
  // Filter for ONE station object.
  //
  // Note that there is NO [0] here because we are parsing an
  // individual station object rather than the whole array.
  // ---------------------------------------------------------
  JsonDocument stationFilter;

  stationFilter["name"] = true;
  stationFilter["isOffline"] = true;
  stationFilter["currentAverage"] = true;
  stationFilter["currentBearing"] = true;
  stationFilter["currentGust"] = true;
  stationFilter["location"]["coordinates"][0] = true;
  stationFilter["location"]["coordinates"][1] = true;

  // ---------------------------------------------------------
  // Small JSON document for ONE station only.
  //
  // 2048 bytes is deliberately much smaller than the previous
  // 50000-byte document.
  // ---------------------------------------------------------
  DynamicJsonDocument stationDoc(2048);

  Serial.printf(
    "[Zephyr] Heap before station processing: %u | Min heap: %u\n",
    ESP.getFreeHeap(),
    ESP.getMinFreeHeap());

  // ---------------------------------------------------------
  // Pointer into our already-downloaded mutable String.
  //
  // payload.begin() gives us access to the downloaded buffer
  // without making another 277 KB copy.
  // ---------------------------------------------------------
  char* json = payload.begin();

  size_t jsonLength = payload.length();

  // ---------------------------------------------------------
  // Find the beginning of the top-level JSON array.
  // ---------------------------------------------------------
  char* cursor = json;
  size_t remaining = jsonLength;

  while (remaining > 0 && *cursor != '[') {
    cursor++;
    remaining--;
  }

  if (remaining == 0) {

    Serial.println(
      "[Zephyr] ERROR: Could not find JSON array");

    hasWeatherData = false;

    http.end();

    return;
  }

  // Move past '['
  cursor++;
  remaining--;

  // ---------------------------------------------------------
  // Lock shared weather data while we populate localMeters.
  // ---------------------------------------------------------
  if (backgroundDataMutex != nullptr) {
    xSemaphoreTake(backgroundDataMutex, portMAX_DELAY);
  }

  // ---------------------------------------------------------
  // Clear old station validity flags
  // ---------------------------------------------------------
  for (int i = 0; i < TRACKED_METERS; i++) {
    localMeters[i].valid = false;
  }

  // ---------------------------------------------------------
  // Station counter
  // ---------------------------------------------------------
  int processedStations = 0;
  int validStations = 0;

  Serial.println(
    "[Zephyr] Processing stations one at a time...");

  // ---------------------------------------------------------
  // Scan the top-level array.
  // ---------------------------------------------------------
  while (remaining > 0) {

    // -------------------------------------------------------
    // Skip whitespace and commas between objects.
    // -------------------------------------------------------
    while (
      remaining > 0 &&
      (*cursor == ' ' ||
       *cursor == '\r' ||
       *cursor == '\n' ||
       *cursor == '\t' ||
       *cursor == ',')
    ) {
      cursor++;
      remaining--;
    }

    // -------------------------------------------------------
    // End of array
    // -------------------------------------------------------
    if (remaining == 0 || *cursor == ']') {
      break;
    }

    // -------------------------------------------------------
    // We expect a station object.
    // -------------------------------------------------------
    if (*cursor != '{') {

      Serial.printf(
        "[Zephyr] Unexpected JSON character '%c' "
        "after %d stations\n",
        *cursor,
        processedStations);

      break;
    }

    // -------------------------------------------------------
    // Find complete object length.
    // -------------------------------------------------------
    size_t objectLength =
      findJsonObjectLength(cursor, remaining);

    if (objectLength == 0) {

      Serial.printf(
        "[Zephyr] ERROR: Could not find end of station "
        "object after %d stations\n",
        processedStations);

      break;
    }

    processedStations++;

    // -------------------------------------------------------
    // Clear previous station.
    //
    // This means stationDoc never contains more than ONE
    // station at a time.
    // -------------------------------------------------------
    stationDoc.clear();

    // -------------------------------------------------------
    // Parse this single station.
    //
    // cursor points directly into payload's mutable buffer,
    // so ArduinoJson can use zero-copy parsing.
    // -------------------------------------------------------
    DeserializationError stationError =
      deserializeJson(
        stationDoc,
        cursor,
        objectLength,
        DeserializationOption::Filter(stationFilter));

    if (!stationError) {

      JsonObject st =
        stationDoc.as<JsonObject>();

      // -----------------------------------------------------
      // Ignore offline stations
      // -----------------------------------------------------
      if (st["isOffline"] == true) {
        cursor += objectLength;
        remaining -= objectLength;
        continue;
      }

      // -----------------------------------------------------
      // Get GeoJSON coordinates
      //
      // GeoJSON = [longitude, latitude]
      // -----------------------------------------------------
      JsonArray coords =
        st["location"]["coordinates"].as<JsonArray>();

      if (coords.size() < 2) {
        cursor += objectLength;
        remaining -= objectLength;
        continue;
      }

      float stLon =
        coords[0].as<float>();

      float stLat =
        coords[1].as<float>();

      // -----------------------------------------------------
      // Validate coordinates
      // -----------------------------------------------------
      if (!isfinite(stLat) ||
          !isfinite(stLon) ||
          stLat == 0.0f ||
          stLon == 0.0f) {

        cursor += objectLength;
        remaining -= objectLength;
        continue;
      }

      validStations++;

      // -----------------------------------------------------
      // Calculate distance from aircraft/user location
      // -----------------------------------------------------
      float distanceKm =
        getDistanceKM(
          myPos.lat,
          myPos.lon,
          stLat,
          stLon);

      // -----------------------------------------------------
      // Compass bearing from glider to station.
      // -----------------------------------------------------
      float geoBearing =
        getBearing(
          myPos.lat,
          myPos.lon,
          stLat,
          stLon);

      // -----------------------------------------------------
      // Find insertion position among closest stations.
      // -----------------------------------------------------
      int insertAt = -1;

      for (int j = 0; j < TRACKED_METERS; j++) {

        if (!localMeters[j].valid ||
            distanceKm < localMeters[j].distanceKm) {

          insertAt = j;
          break;
        }
      }

      // -----------------------------------------------------
      // Station isn't close enough to enter our list.
      // -----------------------------------------------------
      if (insertAt >= 0) {

        // ---------------------------------------------------
        // Shift existing stations down.
        // ---------------------------------------------------
        for (
          int j = TRACKED_METERS - 1;
          j > insertAt;
          j--
        ) {
          localMeters[j] =
            localMeters[j - 1];
        }

        // ---------------------------------------------------
        // Extract station data.
        // ---------------------------------------------------
        const char* name =
          st["name"].as<const char*>();

        float averageKph =
          st["currentAverage"].as<float>();

        float gustKph =
          st["currentGust"].as<float>();

        float bearing =
          st["currentBearing"].as<float>();

        // ---------------------------------------------------
        // Copy station name safely.
        // ---------------------------------------------------
        strncpy(
          localMeters[insertAt].name,
          name ? name : "ANON",
          sizeof(localMeters[insertAt].name) - 1);

        localMeters[insertAt]
          .name[
            sizeof(localMeters[insertAt].name) - 1
          ] = '\0';

        // ---------------------------------------------------
        // Store station data.
        // ---------------------------------------------------
        localMeters[insertAt].distanceKm =
          distanceKm;

        localMeters[insertAt].speedKph =
          averageKph;

        localMeters[insertAt].gustKph =
          gustKph;

        localMeters[insertAt].bearingDeg =
          bearing;

        localMeters[insertAt].geoBearingDeg =
          geoBearing;

        localMeters[insertAt].valid = true;
      }
    }
    else {

      // -----------------------------------------------------
      // Don't abort the entire weather update because one
      // station is malformed.
      // -----------------------------------------------------
      Serial.printf(
        "[Zephyr] Station %d parse error: %s\n",
        processedStations,
        stationError.c_str());
    }

    // -------------------------------------------------------
    // Advance to next station.
    // -------------------------------------------------------
    cursor += objectLength;
    remaining -= objectLength;
  }

  // ---------------------------------------------------------
  // Determine whether we have usable weather data.
  // ---------------------------------------------------------
  hasWeatherData = false;

  for (int i = 0; i < TRACKED_METERS; i++) {

    if (localMeters[i].valid) {
      hasWeatherData = true;
      break;
    }
  }

  // ---------------------------------------------------------
  // Release shared weather data.
  // ---------------------------------------------------------
  if (backgroundDataMutex != nullptr) {
    xSemaphoreGive(backgroundDataMutex);
  }

  Serial.printf(
    "[Zephyr] Station processing complete: "
    "%d stations scanned, %d valid\n",
    processedStations,
    validStations);

  Serial.printf(
    "[Zephyr] Heap after station processing: "
    "%u | Min heap: %u\n",
    ESP.getFreeHeap(),
    ESP.getMinFreeHeap());

  // ---------------------------------------------------------
  // Print closest stations
  // ---------------------------------------------------------
  if (hasWeatherData) {

    Serial.println("[Zephyr] Closest stations:");

    for (int i = 0; i < TRACKED_METERS; i++) {

      if (!localMeters[i].valid) {
        continue;
      }

      Serial.printf(
        "  %d: %s | %.1f km | %.1f kt | "
        "gust %.1f kt | %.0f deg\n",
        i + 1,
        localMeters[i].name,
        localMeters[i].distanceKm,
        localMeters[i].speedKph,
        localMeters[i].gustKph,
        localMeters[i].bearingDeg);
    }

  } else {

    Serial.println(
      "[Zephyr] No valid weather stations found.");
  }

  // ---------------------------------------------------------
  // Print closest stations
  // ---------------------------------------------------------
  if (hasWeatherData) {

    Serial.println("[Zephyr] Closest stations:");

    for (int i = 0; i < TRACKED_METERS; i++) {

      if (!localMeters[i].valid) {
        continue;
      }

      Serial.printf(
        "  %d: %s | %.1f km | %.1f kt | "
        "gust %.1f kt | %.0f deg\n",
        i + 1,
        localMeters[i].name,
        localMeters[i].distanceKm,
        localMeters[i].speedKph,
        localMeters[i].gustKph,
        localMeters[i].bearingDeg);
    }

  } else {

    Serial.println(
      "[Zephyr] No valid weather stations found.");
  }

  // ---------------------------------------------------------
  // Close HTTP connection
  // ---------------------------------------------------------
  http.end();

  // ---------------------------------------------------------
  // Final memory diagnostics
  // ---------------------------------------------------------
  Serial.printf(
    "[Zephyr] Weather update finished. "
    "Free heap: %u | Min heap: %u\n",
    ESP.getFreeHeap(),
    ESP.getMinFreeHeap());
}
