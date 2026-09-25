// =====================================================
// DrawPages.cpp
// -----------------------------------------------------
// Implementation of the top bar, the four page screens, and their shared
// drawing helpers. Moved out of the main .ino (where this used to be the
// last ~1700 lines of a 5000-line file) so it can be edited on its own.
// See DrawPages.h for the declarations these functions rely on.
// =====================================================
#include "DrawPages.h"

// Used only by drawADSBPage() below to snapshot the shared adsbDoc JSON
// into a plain array before drawing -- not needed anywhere else, so it
// stays private to this file rather than living in DrawPages.h.
#define MAX_DISPLAYED_AIRCRAFT 24
struct AircraftSnapshot {
  float lat, lon, altFeet, speedKt, headingDeg;

  // FANET-only fields, ignored for ADS-B entries (isFanet == false).
  bool isFanet;
  float climbMs;
  char label[12];  // e.g. "FC:0001" -- FanetAddress manufacturer:id
};

// =====================================================
// TOP BAR: 38px tall (8mm), inverted (black background, white text),
// spans the full 300px width, present on every page. Left-to-right it reads:
// temperature, QNH, local time, then battery percentage inside its icon.
// =====================================================
void drawTopBar() {
  const int barH = TOP_BAR_HEIGHT_PX;
  const int textY = 26;

  u8g2.setDrawColor(1);
  u8g2.drawBox(0, 0, SCREEN_W, barH);
  u8g2.setDrawColor(0);

  // ---------------------------------------------------------
  // Top bar font
  // ---------------------------------------------------------
  u8g2.setFont(u8g2_font_helvB14_tf);

  // ---------------------------------------------------------
  // Page name
  // ---------------------------------------------------------
  int x = 4;

  const char* pageName = PAGE_NAMES[currentPage];

  u8g2.drawStr(x, textY, pageName);

  x += u8g2.getStrWidth(pageName) + 8;

  // ---------------------------------------------------------
  // Page dots
  // ---------------------------------------------------------
  const int dotRadius = 3;
  const int dotSpacing = 12;
  const int dotY = barH / 2;

  for (int i = 0; i < ACTIVE_PAGE_COUNT; i++) {
    int cx = x + i * dotSpacing;

    if (i == activePageIndex) {
      u8g2.drawDisc(cx, dotY, dotRadius);
    } else {
      u8g2.drawCircle(cx, dotY, dotRadius);
    }
  }

  // Leave some room after the page dots.
  x += (ACTIVE_PAGE_COUNT - 1) * dotSpacing + dotRadius + 10;


  // =========================================================
  // FIXED BATTERY BOX
  // =========================================================

  const int batteryW = 43;
  const int batteryH = 24;
  const int batteryY = 7;

  // Exactly 4 pixels from the right edge.
  const int batteryX = SCREEN_W - batteryW - 4;


  // =========================================================
  // TIME
  //
  // Right edge is 15 px left of the battery.
  // =========================================================

  char timeBuf[8];

  if (clockSynced) {
    struct tm localTime;
    getPilotLocalTime(&localTime);

    snprintf(
      timeBuf,
      sizeof(timeBuf),
      "%02d:%02d",
      localTime.tm_hour,
      localTime.tm_min);
  } else {
    snprintf(timeBuf, sizeof(timeBuf), "--:--");
  }

  int timeW = u8g2.getStrWidth(timeBuf);

  const int fieldGap = 7;  // this is the gap between time, QNH etc

  int timeX = batteryX - fieldGap - timeW;

  u8g2.drawStr(timeX, textY, timeBuf);


  // =========================================================
  // QNH
  //
  // Right edge is 15 px left of the time.
  // =========================================================

  char qnhBuf[10];

  if (qnhCalibrated) {
    snprintf(
      qnhBuf,
      sizeof(qnhBuf),
      "Q%.0f",
      currentQNH);
  } else {
    snprintf(
      qnhBuf,
      sizeof(qnhBuf),
      "Q----");
  }

  int qnhW = u8g2.getStrWidth(qnhBuf);

  int qnhX = timeX - fieldGap - qnhW;

  u8g2.drawStr(qnhX, textY, qnhBuf);


  // =========================================================
  // TEMPERATURE
  //
  // Right edge is 15 px left of the QNH.
  // =========================================================

  char tempBuf[8];

  if (!isnan(currentTempC)) {
    snprintf(
      tempBuf,
      sizeof(tempBuf),
      "%.0f\xb0",
      currentTempC);
  } else {
    snprintf(
      tempBuf,
      sizeof(tempBuf),
      "--\xb0");
  }

  int tempW = u8g2.getStrWidth(tempBuf);

  int tempX = qnhX - fieldGap - tempW;

  u8g2.drawStr(tempX, textY, tempBuf);


  // =========================================================
  // BATTERY FRAME
  // =========================================================

  u8g2.drawFrame(
    batteryX,
    batteryY,
    batteryW,
    batteryH);


  // =========================================================
  // BATTERY PERCENTAGE
  // =========================================================

  char battBuf[8];

  snprintf(
    battBuf,
    sizeof(battBuf),
    "%d%%",
    battInitialized ? batteryPercent : 0);

  int battTextW = u8g2.getStrWidth(battBuf);

  u8g2.drawStr(
    batteryX + (batteryW - battTextW) / 2,
    textY,
    battBuf);


  // ---------------------------------------------------------
  // Restore normal drawing colour
  // ---------------------------------------------------------

  u8g2.setDrawColor(1);
}
// =====================================================
// Draw a value centered horizontally. Select the largest bold font that
// fits inside the given width, keeping long flight values legible.
void drawLargestBoldCentered(int centerX, int baselineY, int maxWidth, const char* text) {
  const uint8_t* fonts[] = {
    u8g2_font_fub20_tf,
    u8g2_font_helvB18_tf,
    u8g2_font_helvB14_tf,
    u8g2_font_helvB12_tf
  };

  for (const uint8_t* font : fonts) {
    u8g2.setFont(font);
    if (u8g2.getStrWidth(text) <= maxWidth) {
      break;
    }
  }
  u8g2.drawStr(centerX - u8g2.getStrWidth(text) / 2, baselineY, text);
}
// =====================================================
// Same font-fit logic as drawLargestBoldCentered() above, but (a) also
// requires the chosen font's total height to fit within maxHeight, and
// (b) draws left-aligned at x instead of centering. Used where several
// stacked lines need to shrink *together* to fit a shorter row -- e.g.
// drawWeatherPage() with more stations packed into the same screen --
// rather than just avoiding horizontal clipping on a single value.
void drawLargestBold(int x, int baselineY, int maxWidth, int maxHeight, const char* text) {
  const uint8_t* fonts[] = {
    u8g2_font_fub20_tf,
    u8g2_font_helvB18_tf,
    u8g2_font_helvB14_tf,
    u8g2_font_helvB12_tf
  };

  for (const uint8_t* font : fonts) {
    u8g2.setFont(font);
    int fontHeight = u8g2.getFontAscent() - u8g2.getFontDescent();
    if (u8g2.getStrWidth(text) <= maxWidth && fontHeight <= maxHeight) {
      break;
    }
  }
  u8g2.drawStr(x, baselineY, text);
}
// Determines (without drawing) the largest of the same candidate fonts
// whose total height fits within maxHeight. Used to size a whole block of
// stacked lines as one unit -- e.g. to work out the line spacing for a
// weather-station row -- before any of its individual strings are
// measured or drawn. drawLargestBold() above independently re-checks both
// width and height per string when it actually draws, so it can only ever
// pick this font or something smaller -- never something taller than the
// spacing this was used to plan for.
const uint8_t* pickBoldFontForHeight(int maxHeight) {
  const uint8_t* fonts[] = {
    u8g2_font_fub20_tf,
    u8g2_font_helvB18_tf,
    u8g2_font_helvB14_tf,
    u8g2_font_helvB12_tf
  };

  const uint8_t* chosen = fonts[3];  // smallest -- fallback if nothing fits
  for (const uint8_t* font : fonts) {
    u8g2.setFont(font);
    int fontHeight = u8g2.getFontAscent() - u8g2.getFontDescent();
    if (fontHeight <= maxHeight) {
      chosen = font;
      break;
    }
  }
  return chosen;
}
void drawLargeValueWithSmallUnit(
  int centerX,
  int baselineY,
  int maxWidth,
  const char* value,
  const char* unit) {
  // ---------------------------------------------------------
  // Large numeric value
  // ---------------------------------------------------------
  const uint8_t* valueFonts[] = {
    u8g2_font_fub35_tn,
    u8g2_font_fub30_tn,  // 💡 Added: 30-pixel tall font
    u8g2_font_fub25_tn,  // 💡 Added: 25-pixel tall font
    u8g2_font_fub20_tn,
    u8g2_font_helvB18_tf,
    u8g2_font_helvB14_tf
  };

  // Select largest value font that fits
  const uint8_t* selectedFont = u8g2_font_helvB14_tf;

  for (const uint8_t* font : valueFonts) {
    u8g2.setFont(font);

    if (u8g2.getStrWidth(value) <= maxWidth) {
      selectedFont = font;
      break;
    }
  }

  u8g2.setFont(selectedFont);

  int valueWidth = u8g2.getStrWidth(value);

  // ---------------------------------------------------------
  // Small unit
  // ---------------------------------------------------------
  u8g2.setFont(u8g2_font_helvB10_tf);

  int unitWidth = u8g2.getStrWidth(unit);

  // Gap between value and unit
  const int gap = 4;

  // Total combined width
  int totalWidth = valueWidth + gap + unitWidth;

  // If combined width is too large, centre the whole thing
  int startX = centerX - totalWidth / 2;

  // ---------------------------------------------------------
  // Draw large value
  // ---------------------------------------------------------
  u8g2.setFont(selectedFont);

  u8g2.drawStr(
    startX,
    baselineY,
    value);

  // ---------------------------------------------------------
  // Draw small unit
  //
  // The unit uses the same baseline. This gives a clean
  // instrument-style readout.
  // ---------------------------------------------------------
  u8g2.setFont(u8g2_font_helvB10_tf);

  u8g2.drawStr(
    startX + valueWidth + gap,
    baselineY,
    unit);
}
// =====================================================
// PARAGLIDER PAGE: 6-box grid (2 cols x 3 rows) below the top bar.
// =====================================================

// AIR SPACE box: each row is now drawn as a fixed-width label ("VERT"/"HORI"/
// "BASE") plus a value starting at this x offset from the row's left edge, so
// the three numbers land in one aligned column instead of trailing off after
// labels of different lengths ("NR VERT: " vs "NR HORI: " vs "CLD BASE: ").
#define AIRSPACE_VALUE_X_OFFSET 46

// Fills buf with just the vertical-distance value, e.g. "3ft" or "--ft".
static void formatVertValue(char* buf, size_t len, bool valid, float ft) {
  if (valid) snprintf(buf, len, "%dft", (int)roundf(ft));
  else snprintf(buf, len, "--ft");
}

// Fills buf with just the horizontal-distance value. Under 1km this switches
// to whole metres ("450m") instead of "0.4km" -- one less decimal point to
// read at the ranges where this box matters most.
static void formatHoriValue(char* buf, size_t len, bool valid, float km) {
  if (!valid) { snprintf(buf, len, "--km"); return; }
  if (km < 1.0f) snprintf(buf, len, "%dm", (int)roundf(km * 1000.0f));
  else snprintf(buf, len, "%.1fkm", km);
}

// Fills buf with just the cloud-base value, e.g. "4200ft" or "--ft". Shown as
// altitude above sea level (same reference as the ALTITUDE box), so it needs
// a calibrated QNH; otherwise falls back to "--".
static void formatCloudBaseValue(char* buf, size_t len) {
  if (bmpOK && windowCount > 0 && qnhCalibrated && !isnan(cloudBaseAboveM)) {
    float baseM = currentAltitudeM + cloudBaseAboveM;
    snprintf(buf, len, "%d%s", (int)roundf(altitudeToDisplay(baseM)), altitudeUnitLabel());
  } else {
    snprintf(buf, len, "--%s", altitudeUnitLabel());
  }
}

void drawParagliderPage() {
  const int top = TOP_BAR_HEIGHT_PX;
  const int colW = SCREEN_W / 2;
  const int rowH = (SCREEN_H - top) / 3;

  char buffer[32];

  // =========================================================
  // BOX OUTLINES
  // =========================================================

  for (int r = 0; r < 3; r++) {
    for (int c = 0; c < 2; c++) {
      u8g2.drawFrame(
        c * colW,
        top + r * rowH,
        colW,
        rowH);
    }
  }


  // =========================================================
  // BOX (0,0): ALTITUDE
  // =========================================================

  u8g2.setFont(u8g2_font_helvB10_tf);

  u8g2.drawStr(
    5,
    top + 14,
    "ALTITUDE");

  if (bmpOK && windowCount > 0) {

    snprintf(
      buffer,
      sizeof(buffer),
      "%d",
      (int)roundf(altitudeToDisplay(currentAltitudeM)));

    drawLargeValueWithSmallUnit(
      colW / 2,
      top + rowH / 2 + +20,  // 💡 Changed from +10 to +20 to shift down 10px
      colW - 10,
      buffer,
      altitudeUnitLabel());

  } else {

    drawLargeValueWithSmallUnit(
      colW / 2,
      top + rowH / 2 + 10,
      colW - 10,
      "--",
      altitudeUnitLabel());
  }

  // ALT AGL -- moved here from the AGL/AIR SPC box, bottom-centre of
  // this box (6px up from the box's bottom edge, same margin used
  // elsewhere on this page for a bottom-anchored line).
  char aglLineBuf[24];
  if (bmpOK && windowCount > 0 && qnhCalibrated && groundElevationValid) {
    float aglM = currentAltitudeM - (groundElevationFt / 3.28084f);
    snprintf(aglLineBuf, sizeof(aglLineBuf), "ALT AGL: %d%s", (int)roundf(altitudeToDisplay(aglM)), altitudeUnitLabel());
  } else {
    snprintf(aglLineBuf, sizeof(aglLineBuf), "ALT AGL: --%s", altitudeUnitLabel());
  }
  u8g2.setFont(u8g2_font_helvB10_tf);
  u8g2.drawStr((colW - u8g2.getStrWidth(aglLineBuf)) / 2, top + rowH - 6, aglLineBuf);


  // =========================================================
  // BOX (0,1): GROUND SPEED + HEADING
  // =========================================================

  u8g2.setFont(u8g2_font_helvB10_tf);

  u8g2.drawStr(
    colW + 5,
    top + 14,
    "V GROUND");

  if (gps.speed.isValid()) {

    snprintf(
      buffer,
      sizeof(buffer),
      "%d",
      (int)roundf(speedKphToDisplay(gps.speed.kmph())));

    drawLargeValueWithSmallUnit(
      colW + colW / 2,
      top + rowH / 2,
      colW - 10,
      buffer,
      speedUnitLabel());

  } else {

    drawLargeValueWithSmallUnit(
      colW + colW / 2,
      top + rowH / 2,
      colW - 10,
      "--",
      speedUnitLabel());
  }


  // ---------------------------------------------------------
  // Heading
  // ---------------------------------------------------------

  u8g2.setFont(u8g2_font_helvB10_tf);

  if (gps.course.isValid()) {

    snprintf(
      buffer,
      sizeof(buffer),
      "HDG %s",
      getCompassDirection(gps.course.deg()));

  } else {

    snprintf(
      buffer,
      sizeof(buffer),
      "HDG ---");
  }

  u8g2.drawStr(
    colW + (colW - u8g2.getStrWidth(buffer)) / 2 - 20,  // 💡 Subtracted 20 to shift left
    top + rowH - 16,
    buffer);


  // =========================================================
  // BOX (1,0): CLIMB RATE
  // =========================================================

  u8g2.setFont(u8g2_font_helvB10_tf);

  u8g2.drawStr(
    5,
    top + rowH + 14,
    "CLIMB RATE");

  if (bmpOK && windowCount >= 3) {

    snprintf(
      buffer,
      sizeof(buffer),
      "%+.1f",
      currentClimbRateMS);

    drawLargeValueWithSmallUnit(
      colW / 2 - 3,
      top + rowH + rowH / 2 + 20,  // Changed from 10 to 20 to shift it down
      colW - 10,
      buffer,
      "m/s");

  } else {

    drawLargeValueWithSmallUnit(
      colW / 2,
      top + rowH + rowH / 2 + 10,
      colW - 10,
      "--",
      "m/s");
  }


  // =========================================================
  // BOX (1,1): AIR SPACE -- ALT AGL moved out to the ALTITUDE box above,
  // so this now just shows nearest-airspace vertical/horizontal
  // distance (see getAirspaceSnapshot(), same background-task scan used
  // elsewhere), recentred as a 2-line block around the box's vertical
  // middle instead of the old 3-line block that started at the middle
  // and ran downward. Each line falls back to "--" independently if its
  // data isn't available yet.
  // =========================================================

  u8g2.setFont(u8g2_font_helvB10_tf);

  u8g2.drawStr(
    colW + 5,
    top + rowH + 14,
    "AIR SPACE");

  AirspaceResult boxAirspace;
  bool boxAirspaceValid = getAirspaceSnapshot(boxAirspace);

  char vertBuf[16];
  char horiBuf[16];
  char cloudBuf[16];
  formatVertValue(vertBuf, sizeof(vertBuf), boxAirspaceValid && boxAirspace.vertKnown, boxAirspace.vertDistance_ft);
  formatHoriValue(horiBuf, sizeof(horiBuf), boxAirspaceValid, boxAirspace.horizDistance_km);
  formatCloudBaseValue(cloudBuf, sizeof(cloudBuf));

  int rowY1 = top + rowH + rowH / 2 - 10;
  int rowY2 = top + rowH + rowH / 2 + 10;
  int rowY3 = top + rowH + rowH / 2 + 30;
  int valueX = colW + 5 + AIRSPACE_VALUE_X_OFFSET;

  u8g2.drawStr(colW + 5, rowY1, "VERT");
  u8g2.drawStr(valueX, rowY1, vertBuf);
  u8g2.drawStr(colW + 5, rowY2, "HORI");
  u8g2.drawStr(valueX, rowY2, horiBuf);
  u8g2.drawStr(colW + 5, rowY3, "BASE");
  u8g2.drawStr(valueX, rowY3, cloudBuf);


  // =========================================================
  // BOX (2,0): GLIDE RATIO
  // =========================================================

  u8g2.setFont(u8g2_font_helvB10_tf);

  u8g2.drawStr(
    5,
    top + 2 * rowH + 14,
    "GLIDE RATIO");

  bool glideValid =
    bmpOK && gps.speed.isValid() && currentClimbRateMS < -CLIMB_DEADBAND_MS && currentClimbRateMS > -20.0f;

  if (glideValid) {

    float glideRatio =
      gps.speed.mps() / (-currentClimbRateMS);

    snprintf(
      buffer,
      sizeof(buffer),
      "%.1f",
      glideRatio);

    drawLargeValueWithSmallUnit(
      colW / 2,
      top + 2 * rowH + rowH / 2 + 10,
      colW - 10,
      buffer,
      ":1");

  } else {

    drawLargeValueWithSmallUnit(
      colW / 2,
      top + 2 * rowH + rowH / 2 + 10,
      colW - 10,
      "--",
      ":1");
  }


  // =========================================================
  // BOX (2,1): WIND / AIRSPEED
  // =========================================================

  u8g2.setFont(u8g2_font_helvB10_tf);

  u8g2.drawStr(
    colW + 5,
    top + 2 * rowH + 14,
    "WIND / AIRSPEED");


  char windBuf[20];
  char airBuf[20];
  char windDirBuf[20];
  if (windEstimateValid) {

    snprintf(
      windBuf,
      sizeof(windBuf),
      "WIND %.0f %s",
      speedKphToDisplay(estimatedWindSpeedKph),
      speedUnitLabel());

    snprintf(
      windDirBuf,
      sizeof(airBuf),
      "FROM %s",
      getCompassDirection(estimatedWindDirectionDeg));

    snprintf(
      airBuf,
      sizeof(airBuf),
      "AIR %.0f %s",
      speedKphToDisplay(estimatedAirspeedKph),
      speedUnitLabel());

  } else {

    snprintf(
      windBuf,
      sizeof(windBuf),
      "WIND -- %s",
      speedUnitLabel());

    snprintf(
      windDirBuf,
      sizeof(windDirBuf),
      "FROM --");

    snprintf(
      airBuf,
      sizeof(airBuf),
      "AIR -- %s",
      speedUnitLabel());
  }


  u8g2.setFont(u8g2_font_helvB14_tf);

  u8g2.drawStr(
    colW + (colW - u8g2.getStrWidth(windBuf)) / 2,
    top + 2 * rowH + 40,
    windBuf);

  u8g2.drawStr(
    colW + (colW - u8g2.getStrWidth(windBuf)) / 2,
    top + 2 * rowH + 60,
    windDirBuf);
  u8g2.drawStr(
    colW + (colW - u8g2.getStrWidth(airBuf)) / 2,
    top + 2 * rowH + 100,
    airBuf);
}

// =====================================================
// WEATHER PAGE -- stub. WiFi init + phone-tether weather data is
// future work (item 6) -- this is just the page shell for now.
// =====================================================
void drawWeatherPage() {
  // ---------------------------------------------------------
  // 0. Pick a data source and build localMetersSnapshot[] from it.
  //
  // weatherSource (Config > Weather Settings > Source) picks the
  // preferred source, but if that source currently has nothing to show,
  // this falls back to whichever source DOES -- most usefully Zephyr
  // (needs WiFi/internet) falling back to FANET (needs neither) when
  // out of WiFi range mid-flight, which is the direction explicitly
  // asked for; the reverse (FANET selected but quiet -> Zephyr) is
  // included too since it's a natural, harmless extension of the same
  // idea -- flag it if you'd rather Weather stay strictly FANET-only
  // when that's the selected source.
  // ---------------------------------------------------------
  WindMeter localMetersSnapshot[TRACKED_METERS];
  bool snapshotHasWeatherData;
  bool usingFanetFallback = false;

  bool zephyrAvailable = hasWeatherData;

  unsigned long wxNowMs = millis();
  int fanetStationCount = 0;
  for (int i = 0; i < MAX_FANET_WEATHER_STATIONS; i++) {
    if (fanetWeatherStations[i].valid &&
        (wxNowMs - fanetWeatherStations[i].lastSeenMs) <= FANET_WEATHER_TIMEOUT_MS) {
      fanetStationCount++;
    }
  }
  bool fanetAvailable = (fanetStationCount > 0);

  bool useZephyr;
  bool useFanet;

  if (weatherSource == WEATHER_SOURCE_ZEPHYR) {
    useZephyr = zephyrAvailable;
    useFanet = !zephyrAvailable && fanetAvailable;
  } else {
    useFanet = fanetAvailable;
    useZephyr = !fanetAvailable && zephyrAvailable;
  }

  if (useZephyr) {

    snapshotHasWeatherData = true;

    if (backgroundDataMutex != nullptr &&
        xSemaphoreTake(
          backgroundDataMutex,
          pdMS_TO_TICKS(20)) == pdTRUE) {

      memcpy(
        localMetersSnapshot,
        localMeters,
        sizeof(localMeters));

      xSemaphoreGive(backgroundDataMutex);

    } else {

      for (int i = 0; i < TRACKED_METERS; i++) {
        localMetersSnapshot[i].valid = false;
      }
    }

  } else if (useFanet) {

    snapshotHasWeatherData = true;
    usingFanetFallback = true;

    // No mutex needed for fanetWeatherStations[] here -- see the
    // FanetWeatherStation comment in DrawPages.h.
    bool haveFix = gps.location.isValid();
    float myLat = haveFix ? (float)gps.location.lat() : 0.0f;
    float myLon = haveFix ? (float)gps.location.lng() : 0.0f;

    int slot = 0;

    for (int i = 0;
         i < MAX_FANET_WEATHER_STATIONS && slot < TRACKED_METERS;
         i++) {

      if (!fanetWeatherStations[i].valid) continue;
      if (wxNowMs - fanetWeatherStations[i].lastSeenMs > FANET_WEATHER_TIMEOUT_MS) continue;

      WindMeter& m = localMetersSnapshot[slot];

      snprintf(m.name, sizeof(m.name), "FANET %02X:%04X",
               fanetWeatherStations[i].addr.manufacturer,
               fanetWeatherStations[i].addr.id);

      m.speedKph = fanetWeatherStations[i].windSpeedKmh;
      m.gustKph = fanetWeatherStations[i].windGustKmh;
      m.bearingDeg = fanetWeatherStations[i].windHeadingDeg;

      if (haveFix) {
        m.distanceKm = getDistanceKM(myLat, myLon,
                                      fanetWeatherStations[i].lat,
                                      fanetWeatherStations[i].lon);
        m.geoBearingDeg = getBearing(myLat, myLon,
                                      fanetWeatherStations[i].lat,
                                      fanetWeatherStations[i].lon);
      } else {
        // No GPS fix yet -- can't compute distance/bearing to the
        // station, so show it but with those two fields zeroed rather
        // than a misleading number.
        m.distanceKm = 0.0f;
        m.geoBearingDeg = 0.0f;
      }

      m.valid = true;
      slot++;
    }

    for (int i = slot; i < TRACKED_METERS; i++) {
      localMetersSnapshot[i].valid = false;
    }

    // Sort by distance, closest first -- matches how Zephyr's
    // localMeters[] already arrives (see updateWeather(), main .ino).
    if (haveFix) {
      for (int a = 0; a < slot - 1; a++) {
        for (int b = a + 1; b < slot; b++) {
          if (localMetersSnapshot[b].distanceKm < localMetersSnapshot[a].distanceKm) {
            WindMeter tmp = localMetersSnapshot[a];
            localMetersSnapshot[a] = localMetersSnapshot[b];
            localMetersSnapshot[b] = tmp;
          }
        }
      }
    }

  } else {

    snapshotHasWeatherData = false;

    for (int i = 0; i < TRACKED_METERS; i++) {
      localMetersSnapshot[i].valid = false;
    }
  }

  (void)usingFanetFallback;  // not currently drawn -- see reply for why

  // ---------------------------------------------------------
  // 1. Row layout
  //
  // topPaddingPx/bottomPaddingPx are independent, so line 1 (station
  // name) can sit close to the box top while line 3 (GUST) stays
  // anchored to the box bottom -- line 2 is split evenly between them
  // below (see step 7), rather than every line being a fixed offset
  // stacked from the top the way it used to be.
  // ---------------------------------------------------------
  const int startY = TOP_BAR_HEIGHT_PX;
  const int availableHeight = SCREEN_H - startY;

  const int rowHeight =
    availableHeight / weatherStationsShown;

  const int topPaddingPx = 3;
  const int bottomPaddingPx = 6;
  const int lineGapPx = 4;

  int perLineBudget =
    (rowHeight - topPaddingPx - bottomPaddingPx - 2 * lineGapPx) / 3;

  if (perLineBudget < 8) {
    perLineBudget = 8;
  }

  // ---------------------------------------------------------
  // 2. No weather data
  // ---------------------------------------------------------
  if (!snapshotHasWeatherData) {

    u8g2.setFont(u8g2_font_helvB24_tf);

    const char* msg = "No Weather Data";

    int msgWidth =
      u8g2.getStrWidth(msg);

    u8g2.drawStr(
      (SCREEN_W - msgWidth) / 2,
      startY + (availableHeight / 2),
      msg);

    return;
  }

  // ---------------------------------------------------------
  // 3. Determine vertical font sizing.
  //
  // ONE bold font is picked here and used for BOTH the AVE/GUST values
  // AND the line spacing (lineHeight), so the spacing always matches
  // what's actually drawn -- previously these were sized independently
  // (numbers were hardcoded to helvB14 regardless of this), which is
  // part of why the station name ended up sitting well below the box
  // top no matter how much room a row actually had.
  // ---------------------------------------------------------
  const uint8_t* boldNumberFont =
    pickBoldFontForHeight(perLineBudget);

  u8g2.setFont(boldNumberFont);

  const int lineHeight =
    u8g2.getFontAscent() -
    u8g2.getFontDescent();

  // Small labels/units/distance/compass -- one step larger than before
  // (10px -> 12px), but only if that still fits under whatever
  // lineHeight this row ended up with, so it can never overflow a line.
  const uint8_t* regularFont = u8g2_font_helvR12_tf;
  u8g2.setFont(regularFont);
  if (u8g2.getFontAscent() - u8g2.getFontDescent() > lineHeight) {
    regularFont = u8g2_font_helvR10_tf;
  }

  // Bold counterpart of regularFont, same size -- used for the wind
  // direction (N/S/NE/etc) so it stands out from the plain labels/units.
  const uint8_t* regularBoldFont =
    (regularFont == u8g2_font_helvR12_tf) ? u8g2_font_helvB12_tf : u8g2_font_helvB10_tf;

  // ---------------------------------------------------------
  // 4. Determine the REAL horizontal space available for
  //    station names.
  //
  // Distance and compass are calculated using the actual
  // contents of each row rather than reserving an arbitrary
  // fixed number of pixels.
  //
  // We use the smallest available width from all displayed
  // rows so every station name can use the same font.
  // ---------------------------------------------------------
  const int stationNameX = 6;
  const int stationNameRightGap = 8;

  int minimumNameWidth =
    SCREEN_W - stationNameX;

  u8g2.setFont(regularFont);

  for (int i = 0;
       i < weatherStationsShown &&
       i < TRACKED_METERS;
       i++) {

    if (!localMetersSnapshot[i].valid) {
      continue;
    }

    // -------------------------------------------------------
    // Compass to station
    // -------------------------------------------------------
    const char* geoCompass =
      getCompassDirection(
        localMetersSnapshot[i].geoBearingDeg);

    int geoCompassWidth =
      u8g2.getStrWidth(geoCompass);

    const int geoCompassX =
      SCREEN_W -
      geoCompassWidth -
      6;

    // -------------------------------------------------------
    // Distance
    // -------------------------------------------------------
    char distBuf[16];

    snprintf(
      distBuf,
      sizeof(distBuf),
      "%.1f km",
      localMetersSnapshot[i].distanceKm);

    int distanceWidth =
      u8g2.getStrWidth(distBuf);

    const int distGap = 8;

    const int distanceX =
      geoCompassX -
      distGap -
      distanceWidth;

    // -------------------------------------------------------
    // Name must stop before the distance.
    // -------------------------------------------------------
    const int availableNameWidth =
      distanceX -
      stationNameRightGap -
      stationNameX;

    if (availableNameWidth < minimumNameWidth) {
      minimumNameWidth = availableNameWidth;
    }
  }

  // Safety floor.
  if (minimumNameWidth < 20) {
    minimumNameWidth = 20;
  }

  // ---------------------------------------------------------
  // 5. Find the LONGEST displayed station name.
  //
  // WEATHER_STATION_NAME_MAX (20) characters are allowed -- matches
  // WindMeter::name's own storage, so nothing gets truncated here that
  // wasn't already cut off further upstream.
  // ---------------------------------------------------------
  char longestStationName[WEATHER_STATION_NAME_MAX + 1] = "";
  int longestNameLength = 0;

  for (int i = 0;
       i < weatherStationsShown &&
       i < TRACKED_METERS;
       i++) {

    if (!localMetersSnapshot[i].valid) {
      continue;
    }

    char nameBuf[WEATHER_STATION_NAME_MAX + 1];

    snprintf(
      nameBuf,
      sizeof(nameBuf),
      "%.20s",  // keep in sync with WEATHER_STATION_NAME_MAX
      localMetersSnapshot[i].name);

    int nameLength =
      strlen(nameBuf);

    if (nameLength > longestNameLength) {

      longestNameLength = nameLength;

      strncpy(
        longestStationName,
        nameBuf,
        sizeof(longestStationName) - 1);

      longestStationName[
        sizeof(longestStationName) - 1
      ] = '\0';
    }
  }

  // ---------------------------------------------------------
  // 6. Select ONE font for ALL station names -- one step larger than
  // before (14/12/10 -> 18/14/12/10 ladder), but every candidate is
  // checked against lineHeight as well as width, so a bigger station
  // name font can never vertically overflow into the next line even
  // when there's plenty of horizontal room for it.
  // ---------------------------------------------------------
  const uint8_t* nameFontLadder[] = {
    u8g2_font_helvR18_tf,
    u8g2_font_helvR14_tf,
    u8g2_font_helvR12_tf,
    u8g2_font_helvR10_tf
  };

  const uint8_t* stationNameFont = u8g2_font_helvR10_tf;  // safe fallback

  for (const uint8_t* candidate : nameFontLadder) {
    u8g2.setFont(candidate);

    int candidateHeight =
      u8g2.getFontAscent() -
      u8g2.getFontDescent();

    if (candidateHeight > lineHeight) {
      continue;  // would overflow this row vertically -- try smaller
    }

    if (longestNameLength == 0 ||
        u8g2.getStrWidth(longestStationName) <= minimumNameWidth) {
      stationNameFont = candidate;
      break;
    }
  }

  // ---------------------------------------------------------
  // 7. Draw each station row
  // ---------------------------------------------------------
  for (int i = 0;
       i < weatherStationsShown &&
       i < TRACKED_METERS;
       i++) {

    if (!localMetersSnapshot[i].valid) {
      continue;
    }

    const int currentBoxY =
      startY + (i * rowHeight);

    // -------------------------------------------------------
    // Row separator
    // -------------------------------------------------------
    if (i > 0) {

      u8g2.drawLine(
        0,
        currentBoxY,
        SCREEN_W,
        currentBoxY);
    }

    // =======================================================
    // Line baselines -- line 1 sits close to the top (small
    // topPaddingPx), line 3 is anchored to the bottom (bottomPaddingPx
    // up from the box's bottom edge, same margin GUST always used to
    // land on by accident before), and line 2 is split evenly between
    // them so it never bunches up against either neighbour.
    // =======================================================
    const int line1Y =
      currentBoxY +
      topPaddingPx +
      lineHeight;

    const int line3Y =
      currentBoxY +
      rowHeight -
      bottomPaddingPx;

    const int line2Y =
      (line1Y + line3Y) / 2;

    // =======================================================
    // LINE 1
    // Station name + distance + compass
    // =======================================================

    // -------------------------------------------------------
    // Station name
    // -------------------------------------------------------
    char stationNameBuf[WEATHER_STATION_NAME_MAX + 1];

    snprintf(
      stationNameBuf,
      sizeof(stationNameBuf),
      "%.20s",  // keep in sync with WEATHER_STATION_NAME_MAX
      localMetersSnapshot[i].name);

    // -------------------------------------------------------
    // Calculate this row's actual right-hand information
    // position before drawing the station name.
    // -------------------------------------------------------
    u8g2.setFont(regularFont);

    const char* geoCompass =
      getCompassDirection(
        localMetersSnapshot[i].geoBearingDeg);

    int geoCompassWidth =
      u8g2.getStrWidth(geoCompass);

    const int geoCompassX =
      SCREEN_W -
      geoCompassWidth -
      6;

    char distBuf[16];

    snprintf(
      distBuf,
      sizeof(distBuf),
      "%.1f km",
      localMetersSnapshot[i].distanceKm);

    int distanceWidth =
      u8g2.getStrWidth(distBuf);

    const int distGap = 8;

    const int distanceX =
      geoCompassX -
      distGap -
      distanceWidth;

    const int maxNameWidth =
      distanceX -
      stationNameRightGap -
      stationNameX;

    // -------------------------------------------------------
    // Final safety clipping.
    //
    // Normally this isn't needed because stationNameFont was
    // selected using the longest name. It protects us if a
    // particular row has less room than expected.
    // -------------------------------------------------------
    char clippedStationName[WEATHER_STATION_NAME_MAX + 1];

    strncpy(
      clippedStationName,
      stationNameBuf,
      sizeof(clippedStationName) - 1);

    clippedStationName[
      sizeof(clippedStationName) - 1
    ] = '\0';

    u8g2.setFont(stationNameFont);

    while (
      strlen(clippedStationName) > 0 &&
      u8g2.getStrWidth(clippedStationName) >
        maxNameWidth) {

      size_t len =
        strlen(clippedStationName);

      clippedStationName[len - 1] =
        '\0';
    }

    u8g2.drawStr(
      stationNameX,
      line1Y,
      clippedStationName);

    // -------------------------------------------------------
    // Draw distance
    // -------------------------------------------------------
    u8g2.setFont(regularFont);

    u8g2.drawStr(
      distanceX,
      line1Y,
      distBuf);

    // -------------------------------------------------------
    // Draw compass to station
    // -------------------------------------------------------
    u8g2.drawStr(
      geoCompassX,
      line1Y,
      geoCompass);

    // =======================================================
    // LINE 2
    // AVE: [BOLD NUMBER] [unit] [wind direction]
    // =======================================================

    char aveNumberBuf[12];

    snprintf(
      aveNumberBuf,
      sizeof(aveNumberBuf),
      "%.0f",
      speedKphToDisplay(
        localMetersSnapshot[i].speedKph));

    // -------------------------------------------------------
    // AVE label - small normal font
    // -------------------------------------------------------
    u8g2.setFont(regularFont);

    const char* aveLabel =
      "AVE:";

    const int aveLabelWidth =
      u8g2.getStrWidth(aveLabel);

    u8g2.drawStr(
      6,
      line2Y,
      aveLabel);

    // -------------------------------------------------------
    // Average speed NUMBER - large bold font
    // -------------------------------------------------------
    u8g2.setFont(boldNumberFont);

    const int aveNumberX =
      6 +
      aveLabelWidth +
      5;

    u8g2.drawStr(
      aveNumberX,
      line2Y,
      aveNumberBuf);

    // Measured HERE, while boldNumberFont is still the active font --
    // measuring after switching to regularFont (as this used to do)
    // reports the number's width as if it were drawn in the small font,
    // which undershoots the real (wider, bold) width and pushes the
    // unit left into the last digit. The gap grows with font size, which
    // is why this only became visible at fewer stations (bigger row =
    // bigger boldNumberFont = bigger error).
    const int aveNumberWidth =
      u8g2.getStrWidth(aveNumberBuf);

    // -------------------------------------------------------
    // Unit - small normal font
    // -------------------------------------------------------
    u8g2.setFont(regularFont);

    const char* unit =
      speedUnitLabel();

    const int unitX =
      aveNumberX +
      aveNumberWidth +
      4;

    u8g2.drawStr(
      unitX,
      line2Y,
      unit);

    // -------------------------------------------------------
    // Wind direction reported by station -- bold, to stand out from the
    // plain unit/label text around it.
    // -------------------------------------------------------
    const char* windCompass =
      getCompassDirection(
        localMetersSnapshot[i].bearingDeg);

    const int unitWidth =
      u8g2.getStrWidth(unit);

    const int windCompassGap = 8;

    u8g2.setFont(regularBoldFont);

    u8g2.drawStr(
      unitX +
      unitWidth +
      windCompassGap,
      line2Y,
      windCompass);

    // =======================================================
    // LINE 3
    // GUST: [BOLD NUMBER] [unit]
    // =======================================================

    char gustNumberBuf[12];

    snprintf(
      gustNumberBuf,
      sizeof(gustNumberBuf),
      "%.0f",
      speedKphToDisplay(
        localMetersSnapshot[i].gustKph));

    // -------------------------------------------------------
    // GUST label - small normal font
    // -------------------------------------------------------
    u8g2.setFont(regularFont);

    const char* gustLabel =
      "GUST:";

    const int gustLabelWidth =
      u8g2.getStrWidth(gustLabel);

    u8g2.drawStr(
      6,
      line3Y,
      gustLabel);

    // -------------------------------------------------------
    // Gust speed NUMBER - large bold font
    // -------------------------------------------------------
    u8g2.setFont(boldNumberFont);

    const int gustNumberX =
      6 +
      gustLabelWidth +
      5;

    u8g2.drawStr(
      gustNumberX,
      line3Y,
      gustNumberBuf);

    // Measured HERE (still boldNumberFont) -- see the matching comment
    // in the AVE block above for why this has to happen before the font
    // switches to regularFont.
    const int gustNumberWidth =
      u8g2.getStrWidth(gustNumberBuf);

    // -------------------------------------------------------
    // Unit - small normal font
    // -------------------------------------------------------
    u8g2.setFont(regularFont);

    const int gustUnitX =
      gustNumberX +
      gustNumberWidth +
      4;

    u8g2.drawStr(
      gustUnitX,
      line3Y,
      unit);
  }
}
// =====================================================
// ADSB TRAFFIC PAGE -- stub. Real traffic data (Gaggle-style feed)
// is future work (item 7). For now: a 25km reference circle and a
// placeholder message.
// =====================================================
// Helper function to draw a rotated triangle pointing in your flight direction
void drawGliderHeadingArrow(int cx, int cy, int arrowRadius, float headingDeg) {
  // If the GPS has no valid heading data yet (e.g. standing still), draw a clean circle instead
  if (headingDeg < 0.0f || headingDeg > 360.0f) {
    u8g2.drawCircle(cx, cy, arrowRadius);
    return;
  }

  // Convert compass navigation heading to math radians (North = Top)
  float angleRad = deg2rad(headingDeg) - (PI / 2.0f);

  // Tip tip coordinates (pointing forward)
  int tipX = cx + (int)(arrowRadius * cos(angleRad));
  int tipY = cy + (int)(arrowRadius * sin(angleRad));

  // Left and Right wing trailing coordinates (offset by 140 degrees from the front nose tip)
  float leftWingRad = angleRad + deg2rad(140.0f);
  float rightWingRad = angleRad - deg2rad(140.0f);

  int leftX = cx + (int)((arrowRadius * 0.8f) * cos(leftWingRad));
  int leftY = cy + (int)((arrowRadius * 0.8f) * sin(leftWingRad));

  int rightX = cx + (int)((arrowRadius * 0.8f) * cos(rightWingRad));
  int rightY = cy + (int)((arrowRadius * 0.8f) * sin(rightWingRad));

  // Draw the structural vectors for the custom arrow shape
  u8g2.drawTriangle(tipX, tipY, leftX, leftY, rightX, rightY);
}
void drawADSBPage() {

  const int cx = SCREEN_W / 2;
  const int cy = TOP_BAR_HEIGHT_PX + (SCREEN_H - TOP_BAR_HEIGHT_PX) / 2;
  const int r = 130;

  // Reset conflict evaluation flag for this pass
  conflictDetectedThisFrame = false;

  // 1. Gather your heading and altitude telemetry from the TinyGPS++ stream
  float gliderHeading = 0.0f;
  bool isMoving = false;
  float MY_LAT = 0.0f;
  float MY_LON = 0.0f;

  if (gps.location.isValid()) {
    MY_LAT = gps.location.lat();
    MY_LON = gps.location.lng();
}

  if (gps.course.isValid() && gps.course.age() < 4000 && gps.speed.knots() > 2.0f) {
    gliderHeading = (float)gps.course.deg();
    isMoving = true;
  }

  // Pull current altitude (defaults to 0 if GPS is not connected/locked yet)
  float myAltitudeFeet = gps.altitude.feet();

  // 2. Snapshot the aircraft list under the mutex, then release it
  // immediately -- moved up from later in this function so we know, before
  // drawing the range rings below, whether any traffic is close enough to
  // justify auto-zooming in. All the trig/SPI rendering below runs without
  // holding the lock, so a slow redraw can never make the Core 0 background
  // task wait, and vice versa.
  AircraftSnapshot snapshotAircraft[MAX_DISPLAYED_AIRCRAFT];
  int snapshotCount = 0;
  bool snapshotHasData = hasAdsbData;

  if (snapshotHasData && backgroundDataMutex != nullptr && xSemaphoreTake(backgroundDataMutex, pdMS_TO_TICKS(20)) == pdTRUE) {

    JsonArray snapshotSource = adsbDoc["ac"].as<JsonArray>();
    for (JsonObject ac : snapshotSource) {
      if (snapshotCount >= MAX_DISPLAYED_AIRCRAFT) break;

      float acLat = ac["lat"];
      float acLon = ac["lon"];
      if (acLat == 0.0f || acLon == 0.0f) continue;

      snapshotAircraft[snapshotCount].lat = acLat;
      snapshotAircraft[snapshotCount].lon = acLon;
      snapshotAircraft[snapshotCount].altFeet = ac["alt_baro"];
      snapshotAircraft[snapshotCount].speedKt = ac["gs"];
      snapshotAircraft[snapshotCount].headingDeg = ac["track"];
      snapshotAircraft[snapshotCount].isFanet = false;
      snapshotAircraft[snapshotCount].climbMs = 0.0f;
      snapshotCount++;
    }

    xSemaphoreGive(backgroundDataMutex);
  }

  // ---------------------------------------------------------
  // Merge in live FANET contacts (paragliders/paramotors broadcasting
  // Type-1 tracking beacons, decoded by FanetStack -- see
  // onFanetTracking() in the main .ino). No mutex needed here -- see the
  // FanetContact comment in DrawPages.h for why.
  // ---------------------------------------------------------
  unsigned long fanetNowMs = millis();
  for (int i = 0; i < MAX_FANET_CONTACTS && snapshotCount < MAX_DISPLAYED_AIRCRAFT; i++) {
    if (!fanetContacts[i].valid) continue;

    if (fanetNowMs - fanetContacts[i].lastSeenMs > FANET_CONTACT_TIMEOUT_MS) {
      // Gone quiet long enough to treat as stale -- clearing valid here
      // (rather than only skipping it) also frees the slot for reuse
      // instead of it only ever getting reclaimed via eviction.
      fanetContacts[i].valid = false;
      continue;
    }

    snapshotAircraft[snapshotCount].lat = fanetContacts[i].lat;
    snapshotAircraft[snapshotCount].lon = fanetContacts[i].lon;
    snapshotAircraft[snapshotCount].altFeet = fanetContacts[i].altitudeM * 3.28084f;
    snapshotAircraft[snapshotCount].speedKt = fanetContacts[i].speedKmh * 0.539957f;  // km/h -> kt
    snapshotAircraft[snapshotCount].headingDeg = fanetContacts[i].headingDeg;
    snapshotAircraft[snapshotCount].isFanet = true;
    snapshotAircraft[snapshotCount].climbMs = fanetContacts[i].climbMs;
    snprintf(snapshotAircraft[snapshotCount].label, sizeof(snapshotAircraft[snapshotCount].label),
             "%02X:%04X", fanetContacts[i].addr.manufacturer, fanetContacts[i].addr.id);
    snapshotCount++;
  }

  // 3. Auto-zoom: switch the rings (and the aircraft plot scale) to
  // 10km/5km if any currently-known aircraft is within 10km, otherwise
  // fall back to the pilot's chosen far/default rings (adsbRingOuterKm/
  // adsbRingInnerKm -- ADSB_SETTINGS > Range Rings menu, 30km/15km by
  // default). Re-evaluated every redraw from the full snapshot above, so
  // it snaps back out automatically once nothing is close -- no state is
  // carried between frames.
  bool anyPlaneWithin10km = false;
  for (int i = 0; i < snapshotCount; i++) {
    if (getDistanceKM(MY_LAT, MY_LON, snapshotAircraft[i].lat, snapshotAircraft[i].lon) <= 10.0f) {
      anyPlaneWithin10km = true;
      break;
    }
  }
  const float MAX_RADIUS_KM = anyPlaneWithin10km ? 10.0f : adsbRingOuterKm;

  // Labels need to be built into buffers now that the far/default ring
  // (adsbRingOuterKm/adsbRingInnerKm, see the Range Rings menu) is
  // configurable rather than a fixed "30km"/"15km" pair. The near-zoom
  // state (anyPlaneWithin10km) is untouched by that setting and still
  // always shows 10km/5km.
  char outerRingLabelBuf[8];
  char innerRingLabelBuf[8];
  if (anyPlaneWithin10km) {
    snprintf(outerRingLabelBuf, sizeof(outerRingLabelBuf), "10km");
    snprintf(innerRingLabelBuf, sizeof(innerRingLabelBuf), "5km");
  } else {
    snprintf(outerRingLabelBuf, sizeof(outerRingLabelBuf), "%.0fkm", adsbRingOuterKm);
    snprintf(innerRingLabelBuf, sizeof(innerRingLabelBuf), "%.0fkm", adsbRingInnerKm);
  }
  const char* outerRingLabel = outerRingLabelBuf;
  const char* innerRingLabel = innerRingLabelBuf;

  // =========================================================
  // ALTITUDE OVERLAY BOX -- top-left
  // =========================================================

  u8g2.setFont(u8g2_font_helvB18_tf);

  char altitudeText[24];

  if (bmpOK && windowCount > 0) {
    snprintf(altitudeText, sizeof(altitudeText), "ALT: %d%s", (int)roundf(altitudeToDisplay(currentAltitudeM)), altitudeUnitLabel());
  } else {
    snprintf(altitudeText, sizeof(altitudeText), "ALT: --%s", altitudeUnitLabel());
  }

  int altitudeW = u8g2.getStrWidth(altitudeText) + 10;
  int altitudeH = u8g2.getFontAscent() - u8g2.getFontDescent() + 6;

  int altitudeX = 5;                           // left side, 5px in from the left edge
  int altitudeY = TOP_BAR_HEIGHT_PX + 10 - 4;  // 4px higher than the old top-right box

  u8g2.drawFrame(
    altitudeX,
    altitudeY,
    altitudeW,
    altitudeH);

  u8g2.drawStr(
    altitudeX + 5,
    altitudeY + 23,
    altitudeText);

  // =========================================================
  // GROUND SPEED OVERLAY BOX -- top-right (where Altitude used to sit)
  // =========================================================

  char gsText[16];
  if (gps.speed.isValid()) {
    snprintf(gsText, sizeof(gsText), "%d %s", (int)roundf(speedKphToDisplay(gps.speed.kmph())), speedUnitLabel());
  } else {
    snprintf(gsText, sizeof(gsText), "-- %s", speedUnitLabel());
  }

  int gsW = u8g2.getStrWidth(gsText) + 10;
  int gsH = u8g2.getFontAscent() - u8g2.getFontDescent() + 6;

  int gsX = SCREEN_W - gsW - 20;
  int gsY = TOP_BAR_HEIGHT_PX + 10 - 4;  // top-right corner, just below the top bar

  u8g2.drawFrame(
    gsX,
    gsY,
    gsW,
    gsH);

  u8g2.drawStr(
    gsX + 5,
    gsY + 23,
    gsText);

  // 4. Draw Rotating Crosshair Grid Lines (Compass Rose Matrix)
  u8g2.drawCircle(cx, cy, r);
  u8g2.drawCircle(cx, cy, r / 2);  // Inner reference ring

  float crosshairAngleRad = isMoving ? -deg2rad(gliderHeading) : 0.0f;

  // N-S Crosshair
  int nX = cx + (int)(r * sin(crosshairAngleRad));
  int nY = cy - (int)(r * cos(crosshairAngleRad));
  int sX = cx - (int)(r * sin(crosshairAngleRad));
  int sY = cy + (int)(r * cos(crosshairAngleRad));
  u8g2.drawLine(nX, nY, sX, sY);

  // E-W Crosshair
  int eX = cx + (int)(r * cos(crosshairAngleRad));
  int eY = cy + (int)(r * sin(crosshairAngleRad));
  int wX = cx - (int)(r * cos(crosshairAngleRad));
  int wY = cy - (int)(r * sin(crosshairAngleRad));
  u8g2.drawLine(eX, eY, wX, wY);

  // Compass Typography Markers -- bumped up one size from 7x14_tf
  u8g2.setFont(u8g2_font_8x13_tf);
  u8g2.drawStr(nX - 2, nY - 2, "N");
  u8g2.drawStr(sX - 2, sY + 8, "S");
  u8g2.drawStr(eX + 4, eY + 3, "E");
  u8g2.drawStr(wX - 8, wY + 3, "W");

  // Range ring labels -- placed along the same N radial as the "N" marker
  // itself, so they rotate together with it in track-up mode rather than
  // staying fixed to the top of the screen. Text now reflects whichever
  // scale is active this frame (the pilot's chosen far/default rings, or
  // 10km/5km when zoomed in).
  int innerNX = cx + (int)((r / 2) * sin(crosshairAngleRad));
  int innerNY = cy - (int)((r / 2) * cos(crosshairAngleRad));

  u8g2.setFont(u8g2_font_7x13_tf);  // bumped up one size from 6x10_tf
  u8g2.drawStr(nX + 10, nY - 2, outerRingLabel);
  u8g2.drawStr(innerNX + 10, innerNY - 2, innerRingLabel);

  // 5. Draw Center Navigation Reference Symbol
  drawGliderHeadingArrow(cx, cy, 6, isMoving ? 0.0f : -1.0f);

  // 6. DRAW VARIO OVERLAY BOX (Upgraded to Helvetica Bold)
  u8g2.setFont(u8g2_font_helvB18_tf);  // True Helvetica Bold 14px — matches flight tags!

  char varioText[12];  // Buffer to store formatted layout text
  if (currentClimbRateMS >= 0.0f) {
    snprintf(varioText, sizeof(varioText), "+%.1f m/s", currentClimbRateMS);
  } else {
    snprintf(varioText, sizeof(varioText), "%.1f m/s", currentClimbRateMS);
  }

  // A. CALCULATE POSITION LOGIC
  int varioX = 4 + 20;                // Moved 20 pixels inwards
  int varioY = (SCREEN_H - 26) - 13;  // Shifted 10 pixels up

  // B. COMPUTE AUTO-SCALING BOUNDARIES FOR BOLD TEXT
  // Measures exact bold text string pixel width and adds 10 pixels padding
  int varioW = u8g2.getStrWidth(varioText) + 10;
  int varioH = 25;  // Increased to 23 to match the height of your aircraft data boxes

  // C. DRAW THE SCALED LAYOUT STRUCTURE
  u8g2.drawFrame(varioX, varioY, varioW, varioH);

  // Shifted text baseline down to safely center the bold letters vertically inside the box
  u8g2.drawStr(varioX + 5, varioY + 23, varioText);

  // Drawn here (before the no-traffic-data early return below) so it
  // still renders on a "No data fetched"/"No local traffic" frame --
  // airspace info is entirely independent of ADS-B/FANET traffic. If
  // that return does NOT fire, this same call happens again at the very
  // end of the function instead, on top of the aircraft markers, so the
  // bar keeps display priority either way; the two calls are mutually
  // exclusive per frame. See drawAirspaceInfoBar()'s header comment for
  // why this isn't in drawDashboard() alongside drawAirspaceWarning().
  drawAirspaceInfoBar();

  // Combined check -- snapshotCount now reflects ADS-B AND FANET
  // contacts together, so this only reports "nothing at all" once both
  // sources have had their say. snapshotHasData still distinguishes the
  // two empty cases: the ADS-B feed itself never having been fetched
  // (independent of whether any FANET contact is around) vs. a feed that
  // was fetched but is genuinely empty right now.
  if (snapshotCount == 0) {
    u8g2.setFont(u8g2_font_6x10_tf);
    if (!snapshotHasData) {
      u8g2.drawStr(cx - 50, cy + 30, "No data fetched");
    } else {
      u8g2.drawStr(cx - 55, cy + 30, "No local traffic");
    }
    return;
  }

  // 7. Set Typography: Switch to a Heavy, High-Contrast True Bold Font
  u8g2.setFont(u8g2_font_helvB14_tf);  // True Helvetica Bold 14px — razor-sharp in direct sunlight!

  for (int i = 0; i < snapshotCount; i++) {
    float acLat = snapshotAircraft[i].lat;
    float acLon = snapshotAircraft[i].lon;

    float distanceKM = getDistanceKM(MY_LAT, MY_LON, acLat, acLon);
    if (distanceKM > MAX_RADIUS_KM) continue;

    float absoluteBearing = getBearing(MY_LAT, MY_LON, acLat, acLon);

    float relativeBearing = absoluteBearing;
    if (isMoving) {
      relativeBearing = absoluteBearing - gliderHeading;
    }

    float angleRad = deg2rad(relativeBearing) - (PI / 2.0f);
    float pixelDistance = (distanceKM / MAX_RADIUS_KM) * (float)r;

    int acX = cx + (int)(pixelDistance * cos(angleRad));
    int acY = cy + (int)(pixelDistance * sin(angleRad));

    // --- TELEMETRY EXTRACTION (pulled up so heading is available before
    // we draw the aircraft symbol below) ---
    float altitudeFeet = snapshotAircraft[i].altFeet;
    float speedKnots = snapshotAircraft[i].speedKt;
    float headingDeg = snapshotAircraft[i].headingDeg;

    // Draw the target as a small heading arrow instead of a plain dot,
    // rotated the same way the own-ship marker is -- relative to your
    // heading in track-up mode, or absolute compass heading in north-up
    // mode -- so it stays visually consistent with the rest of the display.
    float acDisplayHeading = fmodf(headingDeg - (isMoving ? gliderHeading : 0.0f) + 360.0f, 360.0f);
    drawGliderHeadingArrow(acX, acY, 5, acDisplayHeading);

    // Same 5km / 2000ft bubble used for the one-time "new intruder" chirp
    // in performADSBUpdate(), re-evaluated every redraw so the alarm keeps
    // re-triggering every ALARM_SILENCE_MS for as long as a conflicting
    // aircraft remains on screen. Deliberately excludes FANET contacts --
    // this alarm is tuned for powered ADS-B traffic, and a paraglider/
    // paramotor pilot thermalling alongside other FANET-equipped pilots
    // would very often be within 2000ft/5km of one, which would make this
    // fire constantly and turn it into noise rather than a real alert.
    if (!snapshotAircraft[i].isFanet &&
        distanceKM <= 5.0f && fabsf(altitudeFeet - myAltitudeFeet) <= 2000.0f) {
      conflictDetectedThisFrame = true;
    }

    char dataTag[24];

    if (snapshotAircraft[i].isFanet) {
      // Climb rate is far more relevant than flight level for a nearby
      // paraglider/paramotor -- and the FANET address doubles as a
      // visual "this is FANET, not ADS-B" cue next to the marker.
      snprintf(dataTag, sizeof(dataTag), "PG %s %+.1fm/s",
               snapshotAircraft[i].label, snapshotAircraft[i].climbMs);
    } else {
      float flightLevelFloat = altitudeFeet / 1000.0f;
      if (flightLevelFloat < 0.0f) flightLevelFloat = 0.0f;

      const char* compassHdg = getCompassDirection(headingDeg);

      snprintf(dataTag, sizeof(dataTag), "FL%.1f %s %.0fkt", flightLevelFloat, compassHdg, speedKnots);
    }

    // 8. Render Anti-Clipping Text Box Frame Safely Beside Target Node
    int textX = acX + 8;  // Offset further out to avoid crowding the icon

    // FIX: Subtracted 4 from textY to lift the top line of the box up 4 pixels
    int textY = acY - 10 - 4;  // Moves the top boundary higher up

    // Added 10 pixels of horizontal padding to prevent side wall clipping
    int textW = u8g2.getStrWidth(dataTag) + 10;

    // FIX: Added 4 to textH (from 19 to 23) so the bottom line stays in the same place
    int textH = 19 + 4;

    // Screen collision safety limits engine calculations
    if (textX + textW > SCREEN_W) textX = acX - textW - 4;
    if (textY < TOP_BAR_HEIGHT_PX) textY = acY + 4;
    if (textY + textH > SCREEN_H) textY = SCREEN_H - textH - 2;

    // Draw the auto-scaled frame shield
    u8g2.drawFrame(textX, textY, textW, textH);

    // FIX: Shifted the text baseline down by 4 to compensate for lifting textY
    // This keeps the letters sitting exactly where they were before the change
    u8g2.drawStr(textX + 5, textY + 15 + 4, dataTag);
  }

  // Drawn again here, on top of the aircraft markers just plotted above
  // -- the earlier call (before the no-traffic early return) only
  // covers the case where that return fires; this call and that one are
  // mutually exclusive per frame (snapshotCount == 0 takes the early
  // return and never reaches here), so this never double-draws.
  drawAirspaceInfoBar();
}

// =====================================================
// PARAMOTOR PAGE: ALTITUDE / V GROUND / AGL/AIR SPC boxes are copied
// straight from drawParagliderPage() (same code, same positions) so the
// two pages stay visually consistent and any future tweak to those boxes
// only has to be made in one place conceptually (this duplication is a
// known tradeoff -- see the note below if that becomes annoying).
// WIND/AIRSPEED moves into the CLIMB RATE box's old slot (row 1, col 0)
// since climb rate isn't meaningful under power. Bottom row is engine
// data: RPM (col 0), CHT (col 1). EGT is intentionally left out here --
// a menu option to show CHT-only vs CHT+EGT is coming (not every motor
// has an EGT bung), so this page doesn't touch engineEgtC yet.
// =====================================================
void drawParamotorPage() {
  const int top = TOP_BAR_HEIGHT_PX;
  const int colW = SCREEN_W / 2;
  const int rowH = (SCREEN_H - top) / 3;
  char buffer[32];

  for (int r = 0; r < 3; r++) {
    for (int c = 0; c < 2; c++) {
      u8g2.drawFrame(c * colW, top + r * rowH, colW, rowH);
    }
  }

  // =========================================================
  // BOX (0,0): ALTITUDE -- identical to drawParagliderPage()
  // =========================================================
  u8g2.setFont(u8g2_font_helvB10_tf);
  u8g2.drawStr(5, top + 14, "ALTITUDE");

  if (bmpOK && windowCount > 0) {
    snprintf(buffer, sizeof(buffer), "%d", (int)roundf(altitudeToDisplay(currentAltitudeM)));
    drawLargeValueWithSmallUnit(colW / 2, top + rowH / 2 + 20, colW - 10, buffer, altitudeUnitLabel());
  } else {
    drawLargeValueWithSmallUnit(colW / 2, top + rowH / 2 + 10, colW - 10, "--", altitudeUnitLabel());
  }

  // ALT AGL -- moved here from the AGL/AIR SPC box, bottom-centre of
  // this box (6px up from the box's bottom edge, same margin used
  // elsewhere on this page for a bottom-anchored line).
  char aglLineBuf[24];
  if (bmpOK && windowCount > 0 && qnhCalibrated && groundElevationValid) {
    float aglM = currentAltitudeM - (groundElevationFt / 3.28084f);
    snprintf(aglLineBuf, sizeof(aglLineBuf), "ALT AGL: %d%s", (int)roundf(altitudeToDisplay(aglM)), altitudeUnitLabel());
  } else {
    snprintf(aglLineBuf, sizeof(aglLineBuf), "ALT AGL: --%s", altitudeUnitLabel());
  }
  u8g2.setFont(u8g2_font_helvB10_tf);
  u8g2.drawStr((colW - u8g2.getStrWidth(aglLineBuf)) / 2, top + rowH - 6, aglLineBuf);

  // =========================================================
  // BOX (0,1): V GROUND + HDG -- identical to drawParagliderPage()
  // =========================================================
  u8g2.setFont(u8g2_font_helvB10_tf);
  u8g2.drawStr(colW + 5, top + 14, "V GROUND");

  if (gps.speed.isValid()) {
    snprintf(buffer, sizeof(buffer), "%d", (int)roundf(speedKphToDisplay(gps.speed.kmph())));
    drawLargeValueWithSmallUnit(colW + colW / 2, top + rowH / 2, colW - 10, buffer, speedUnitLabel());
  } else {
    drawLargeValueWithSmallUnit(colW + colW / 2, top + rowH / 2, colW - 10, "--", speedUnitLabel());
  }

  u8g2.setFont(u8g2_font_helvB10_tf);
  if (gps.course.isValid()) {
    snprintf(buffer, sizeof(buffer), "HDG %s", getCompassDirection(gps.course.deg()));
  } else {
    snprintf(buffer, sizeof(buffer), "HDG ---");
  }
  u8g2.drawStr(colW + (colW - u8g2.getStrWidth(buffer)) / 2 - 20, top + rowH - 16, buffer);

  // =========================================================
  // BOX (1,0): WIND / AIRSPEED -- same code as drawParagliderPage()'s
  // box (2,1), just re-anchored to row 1 / col 0 (no colW offset, one
  // rowH instead of two).
  // =========================================================
  u8g2.setFont(u8g2_font_helvB10_tf);
  u8g2.drawStr(5, top + rowH + 14, "WIND / AIRSPEED");

  char windBuf[20];
  char airBuf[20];
  char windDirBuf[20];
  if (windEstimateValid) {
    snprintf(windBuf, sizeof(windBuf), "WIND %.0f %s", speedKphToDisplay(estimatedWindSpeedKph), speedUnitLabel());
    snprintf(windDirBuf, sizeof(airBuf), "FROM %s", getCompassDirection(estimatedWindDirectionDeg));
    snprintf(airBuf, sizeof(airBuf), "AIR %.0f %s", speedKphToDisplay(estimatedAirspeedKph), speedUnitLabel());
  } else {
    snprintf(windBuf, sizeof(windBuf), "WIND -- %s", speedUnitLabel());
    snprintf(windDirBuf, sizeof(windDirBuf), "FROM --");
    snprintf(airBuf, sizeof(airBuf), "AIR -- %s", speedUnitLabel());
  }

  u8g2.setFont(u8g2_font_helvB14_tf);
  u8g2.drawStr((colW - u8g2.getStrWidth(windBuf)) / 2, top + rowH + 40, windBuf);
  u8g2.drawStr((colW - u8g2.getStrWidth(windBuf)) / 2, top + rowH + 60, windDirBuf);
  u8g2.drawStr((colW - u8g2.getStrWidth(windBuf)) / 2, top + rowH + 100, airBuf);

  // =========================================================
  // BOX (1,1): AIR SPACE -- ALT AGL moved out to the ALTITUDE box above,
  // so this now just shows nearest-airspace vertical/horizontal
  // distance, recentred as a 2-line block around the box's vertical
  // middle instead of the old 3-line block that started at the middle
  // and ran downward.
  // =========================================================
  u8g2.setFont(u8g2_font_helvB10_tf);
  u8g2.drawStr(colW + 5, top + rowH + 14, "AIR SPACE");

  AirspaceResult boxAirspace;
  bool boxAirspaceValid = getAirspaceSnapshot(boxAirspace);

  char vertBuf[16];
  char horiBuf[16];
  char cloudBuf[16];
  formatVertValue(vertBuf, sizeof(vertBuf), boxAirspaceValid && boxAirspace.vertKnown, boxAirspace.vertDistance_ft);
  formatHoriValue(horiBuf, sizeof(horiBuf), boxAirspaceValid, boxAirspace.horizDistance_km);
  formatCloudBaseValue(cloudBuf, sizeof(cloudBuf));

  int rowY1 = top + rowH + rowH / 2 - 10;
  int rowY2 = top + rowH + rowH / 2 + 10;
  int rowY3 = top + rowH + rowH / 2 + 30;
  int valueX = colW + 5 + AIRSPACE_VALUE_X_OFFSET;

  u8g2.drawStr(colW + 5, rowY1, "VERT");
  u8g2.drawStr(valueX, rowY1, vertBuf);
  u8g2.drawStr(colW + 5, rowY2, "HORI");
  u8g2.drawStr(valueX, rowY2, horiBuf);
  u8g2.drawStr(colW + 5, rowY3, "BASE");
  u8g2.drawStr(valueX, rowY3, cloudBuf);

  // =========================================================
  // BOX (2,0): RPM -- from the nRF52840 engine meter over BLE, see
  // engineDataValid/engineRpm in ble_manager.h.
  // =========================================================
  u8g2.setFont(u8g2_font_helvB10_tf);
  u8g2.drawStr(5, top + 2 * rowH + 14, "RPM");
  if (engineDataValid) {
    snprintf(buffer, sizeof(buffer), "%.0f", engineRpm);
  } else {
    snprintf(buffer, sizeof(buffer), "--");
  }
  drawLargeValueWithSmallUnit(colW / 2, top + 2 * rowH + rowH / 2 + 10, colW - 10, buffer, "");

  // =========================================================
  // BOX (2,1): CHT -- EGT intentionally omitted for now (see function
  // header comment); a per-channel thermocouple fault shows as "--"
  // rather than a wrong-looking number.
  // =========================================================
  u8g2.setFont(u8g2_font_helvB10_tf);
  u8g2.drawStr(colW + 5, top + 2 * rowH + 14, "CHT");
  if (engineDataValid && !engineChtFault) {
    snprintf(buffer, sizeof(buffer), "%.0f", engineChtC);
  } else {
    snprintf(buffer, sizeof(buffer), "--");
  }
  drawLargeValueWithSmallUnit(colW + colW / 2, top + 2 * rowH + rowH / 2 + 10, colW - 10, buffer, "\xb0" "C");
}

void drawDashboard() {
  u8g2.firstPage();
  do {
    if (menuActive) {
      drawMenu();
    } else {
      drawTopBar();
      switch (currentPage) {
        case PAGE_PARAGLIDER: drawParagliderPage(); break;
        case PAGE_WEATHER: drawWeatherPage(); break;
        case PAGE_ADSB: drawADSBPage(); break;
        case PAGE_PARAMOTOR: drawParamotorPage(); break;
        default: break;
      }
      // Drawn last, on top of whatever page is active. Airspace warning
      // takes priority for this slot (safety-critical); a received
      // FANET message only gets it when there's no airspace warning to
      // show -- see drawFanetMessageBanner()'s header comment.
      bool airspaceBannerShown = drawAirspaceWarning();
      if (!airspaceBannerShown) {
        drawFanetMessageBanner();
      }
    }
  } while (u8g2.nextPage());
}

// =====================================================
// AIRSPACE WARNING OVERLAY
//
// Silent and invisible when there's nothing to report. Shows a banner
// when the nearest controlled airspace is within AIRSPACE_WARN_HORIZ_KM /
// AIRSPACE_WARN_VERT_FT, and a more prominent one if you're actually
// inside its lateral+vertical bounds. A short alert tone fires once on
// the transition into "inside" (edge-triggered, not every redraw).
// =====================================================
// Copies out the latest airspace scan result under backgroundDataMutex.
// Quick by design -- a plain struct copy, no SD/SPI work while the lock is
// held -- so callers on Core 1 (drawing) never make backgroundTask() (Core
// 0) wait for anything slower than a memcpy.
bool getAirspaceSnapshot(AirspaceResult& out) {
  bool valid = false;
  if (backgroundDataMutex != nullptr && xSemaphoreTake(backgroundDataMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
    valid = airspaceResultValid;
    if (valid) out = nearestAirspace;
    xSemaphoreGive(backgroundDataMutex);
  }
  return valid;
}

// alertOnly = false counterpart of the above -- can return a CFZ. See
// the findNearestControlledAirspace() call site in backgroundTask()
// (main .ino) for why this is a second, independent result rather than
// just relaxing getAirspaceSnapshot() itself.
bool getAirspaceInfoSnapshot(AirspaceResult& out) {
  bool valid = false;
  if (backgroundDataMutex != nullptr && xSemaphoreTake(backgroundDataMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
    valid = airspaceInfoResultValid;
    if (valid) out = nearestAirspaceInfo;
    xSemaphoreGive(backgroundDataMutex);
  }
  return valid;
}

// Returns true if a banner was actually drawn (occupying the top
// banner slot), false if there was nothing to show. Used by
// drawDashboard() to decide whether drawFanetMessageBanner() gets to
// use that same slot instead -- see the comment there for why airspace
// warnings take priority over a received message.
bool drawAirspaceWarning() {
  AirspaceResult result;
  bool valid = getAirspaceSnapshot(result);

  if (!valid) return false;

  // insideVert / vertDistance_ft are meaningless when result.vertKnown is
  // false (floor or ceiling is AGL/SFC-referenced and there's currently no
  // valid ground elevation to resolve it against -- see
  // AirspaceResult::vertKnown). Don't claim a confident "inside" in that
  // case -- and don't use an untrustworthy vertDistance_ft to decide
  // "nearby" either; treat vertical range as unknown-but-possible instead,
  // so a real infringement can't go silently unwarned just because the
  // ground elevation lookup is temporarily unresolved.
  bool insideNow = result.vertKnown && result.insideHoriz && result.insideVert;
  bool nearby = !insideNow &&
                result.horizDistance_km <= AIRSPACE_WARN_HORIZ_KM &&
                (!result.vertKnown || result.vertDistance_ft <= AIRSPACE_WARN_VERT_FT);

  // One-shot alert on entering controlled airspace, not on every redraw.
  // Deliberately NOT gated on airspaceAlertBarEnabled -- that setting
  // only hides the visual banner below, same as adsbAlarmMuted is kept
  // independent of ADS-B's Auto-Jump (settings.h). A pilot who's hidden
  // the banner still gets the one-time tone telling them they've
  // actually entered controlled airspace.
  static bool wasInside = false;
  if (insideNow && !wasInside) {
    playFeedbackTone(900.0f, 600);
  }
  wasInside = insideNow;

  // Visual banner only -- see the comment above for why the tone above
  // this point is unaffected by the setting.
  if (!airspaceAlertBarEnabled) return false;

  if (!insideNow && !nearby) return false;

  char line1[40];
  char line2[40];

  if (insideNow) {
    snprintf(line1, sizeof(line1), "INSIDE %s", result.name);
    snprintf(line2, sizeof(line2), "Class %s", result.classId);
  } else {
    snprintf(line1, sizeof(line1), "%s", result.name);
    if (result.vertKnown) {
      snprintf(line2, sizeof(line2), "%.1fkm  %.0fft", result.horizDistance_km, result.vertDistance_ft);
    } else {
      snprintf(line2, sizeof(line2), "%.1fkm  VERT: --", result.horizDistance_km);
    }
  }

  const int bannerY = TOP_BAR_HEIGHT_PX;
  const int bannerH = 30;

  u8g2.setDrawColor(1);
  u8g2.drawBox(0, bannerY, SCREEN_W, bannerH);
  u8g2.setDrawColor(0);

  u8g2.setFont(u8g2_font_helvB10_tf);
  u8g2.drawStr(6, bannerY + 13, line1);
  u8g2.drawStr(6, bannerY + 27, line2);

  u8g2.setDrawColor(1);  // restore default before returning to normal page drawing

  return true;
}

// =====================================================
// FANET MESSAGE BANNER
//
// Shares the top-of-screen slot with drawAirspaceWarning() above --
// drawDashboard() only calls this when that one didn't draw anything,
// so an airspace warning (safety-critical) is never obscured by a
// received message (not safety-critical, even for a "Need assistance"
// preset -- see FanetMessaging.h). The tone already fired back in
// onFanetMessageReceived() when the message arrived, not here -- this
// function only draws; it runs on every redraw while the message is
// still within its display window, which would replay the tone
// constantly if it lived here instead.
// =====================================================
void drawFanetMessageBanner() {
  if (!lastFanetMessage.valid) return;

  if (millis() - lastFanetMessage.receivedMs >= FANET_MESSAGE_BANNER_MS) return;

  char line1[40];
  char line2[40];

  snprintf(line1, sizeof(line1), "MSG from %02X:%04X",
           lastFanetMessage.src.manufacturer, lastFanetMessage.src.id);
  snprintf(line2, sizeof(line2), "%s", lastFanetMessage.text);

  const int bannerY = TOP_BAR_HEIGHT_PX;
  const int bannerH = 30;

  u8g2.setDrawColor(1);
  u8g2.drawBox(0, bannerY, SCREEN_W, bannerH);
  u8g2.setDrawColor(0);

  u8g2.setFont(u8g2_font_helvB10_tf);
  u8g2.drawStr(6, bannerY + 13, line1);
  u8g2.drawStr(6, bannerY + 27, line2);

  u8g2.setDrawColor(1);  // restore default before returning to normal page drawing
}

// =====================================================
// AIRSPACE INFO BAR (ADS-B page only)
//
// Distinct from drawAirspaceWarning() above in every way that matters:
//   - Reads getAirspaceInfoSnapshot() (alertOnly = false), so it CAN
//     show a CFZ -- that's the point of it existing (CFZ names carry
//     the recommended reporting frequency).
//   - Never plays a tone -- purely informational, not a safety alert.
//   - Always shows something when enabled -- whatever charted airspace
//     the pilot is currently inside, or "Class G" if they're not inside
//     anything charted right now -- rather than only near/inside
//     alert-eligible airspace.
//   - Drawn at the bottom of the screen, and only from drawADSBPage()
//     (not drawDashboard(), so it never appears on the other three
//     pages). Pilot has explicitly said it's fine for this to cover the
//     ADS-B page's own VARIO overlay box.
// =====================================================
void drawAirspaceInfoBar() {
  if (!airspaceInfoBarEnabled) return;

  AirspaceResult result;
  bool valid = getAirspaceInfoSnapshot(result);

  // Same vertKnown caution as drawAirspaceWarning() -- but this bar is
  // informational rather than a safety alert, so where that function
  // treats an unresolved ground elevation as "can't confirm inside,
  // stay quiet", this one treats horizontal containment alone as enough
  // to show the name/frequency rather than hide useful info just
  // because a DEM lookup is temporarily unresolved.
  bool insideNow = valid && result.insideHoriz && (!result.vertKnown || result.insideVert);

  char line1[40];
  char line2[40];

  if (insideNow) {
    snprintf(line1, sizeof(line1), "%s", result.name);
    snprintf(line2, sizeof(line2), "Class %s", result.classId);
  } else {
    // Nothing charted currently contains the pilot's position -- by
    // elimination, that's Class G (uncontrolled) airspace.
    snprintf(line1, sizeof(line1), "Class G");
    line2[0] = '\0';
  }

  const int bannerH = 30;
  const int bannerY = SCREEN_H - bannerH;

  u8g2.setDrawColor(1);
  u8g2.drawBox(0, bannerY, SCREEN_W, bannerH);
  u8g2.setDrawColor(0);

  u8g2.setFont(u8g2_font_helvB10_tf);
  u8g2.drawStr(6, bannerY + 13, line1);
  if (line2[0] != '\0') {
    u8g2.drawStr(6, bannerY + 27, line2);
  }

  u8g2.setDrawColor(1);  // restore default before returning to normal page drawing
}
