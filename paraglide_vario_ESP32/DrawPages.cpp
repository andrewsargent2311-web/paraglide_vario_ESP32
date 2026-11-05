// =====================================================
// DrawPages.cpp
// -----------------------------------------------------
// Implementation of the top bar, the four page screens, and their shared
// drawing helpers. Moved out of the main .ino (where this used to be the
// last ~1700 lines of a 5000-line file) so it can be edited on its own.
// See DrawPages.h for the declarations these functions rely on.
// =====================================================
#include "DrawPages.h"
#include "CrossCoreState.h"
#include "AirspaceProximity.h" 
#include "AglDisplay.h"
#include "AdsbClient.h"
#include "TrafficIcons.h"
#include "wifi_manager.h"  // wifiConnected

static AglDisplayBuffer aglDisplayBuffer;

static bool airspaceScanIsFresh() {
  const uint32_t scanDoneMs = airspaceScanDoneMs;
  return scanDoneMs != 0 &&
         (uint32_t)(millis() - scanDoneMs) <= AIRSPACE_MAX_AGE_MS;
}

static bool getDisplayAglM(float& aglM) {
  const bool baroFresh = millis() - lastSuccessfulBaroSample < 1000UL;
  const bool gpsFresh = gps.location.isValid() && gps.location.age() < 2000;
  const bool sourcesValid = bmpOK && windowCount > 0 && !qnhIsFallback &&
                            baroFresh && gpsFresh && groundElevationValid;
  const float currentAglM = currentAltitudeM - (groundElevationFt / 3.28084f);
  return aglDisplayBuffer.update(sourcesValid, currentAglM, millis(), aglM);
}

// Used only by drawADSBPage() below: one aircraft as it should appear this
// frame, wherever it came from -- not needed anywhere else, so it stays
// private to this file rather than living in DrawPages.h. Sized for every
// ADS-B track (MAX_ADSB_TRACKS) plus every FANET contact.
#define MAX_DISPLAYED_AIRCRAFT 40
struct AircraftSnapshot {
  float lat, lon, altFeet, speedKt, headingDeg;
  float distanceKm;
  bool altKnown;   // false if ADS-B reports it as on the ground, or not at all
  bool estimated;  // position older than 30 s: drawn as an outline
  bool isClose;    // gets the flashing rings: see TRAFFIC_CLOSE_KM/_FT
  TrafficIconType icon;

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
      "Q%.0f%s",
      currentQNH,
      qnhIsFallback ? "*" : "");
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

  // Airspace data stale: the background scan has not completed for
  // AIRSPACE_MAX_AGE_MS while GPS is valid, so "no banner" does not mean
  // "no airspace". Only drawn if it fits in the free space after the page
  // dots; nothing else is moved.
  if (sdCardOK && millis() > 60000UL && gps.location.isValid() &&
      gps.location.age() < 5000 && !airspaceScanIsFresh()) {
    const char* aspStale = "ASP?";
    const int aspW = u8g2.getStrWidth(aspStale);
    if (x + aspW + fieldGap <= tempX) {
      u8g2.drawStr(x, textY, aspStale);
    }
  }


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

// Works out the 3 row y-positions for the AIR SPACE / WIND-AIRSPEED boxes:
// row1 sits one line-height below the title (a blank line's worth of gap),
// row3 sits 6px above the box's bottom edge (same margin AGL/HDG use in the
// boxes above), and row2 is placed exactly halfway between them -- so the
// 3 rows are evenly spaced rather than packed at a fixed gap. Must be
// called right after the title is drawn, while its font (helvB10) is still
// the active font, since that's what sets the line-height measurement.
static void computeBoxRows(int titleY, int boxBottom, int& row1, int& row2, int& row3) {
  int lineHeight = u8g2.getFontAscent() - u8g2.getFontDescent();
  row1 = titleY + 2 * lineHeight;  // a bit more breathing room than a single line-height
  row3 = boxBottom - 6;
  row2 = (row1 + row3) / 2;
}

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

// Draws a left-aligned "label" (small, regular) at xLabel, and its "value"
// (bold, one size up) at a fixed offset to its right -- the AIR SPACE /
// WIND-AIRSPEED box row style, so a box's values form one aligned column.
static void drawBoxRow(int xLabel, int y, const char* label, const char* value) {
  u8g2.setFont(u8g2_font_helvR10_tf);
  u8g2.drawStr(xLabel, y, label);
  u8g2.setFont(u8g2_font_helvB18_tf);
  u8g2.drawStr(xLabel + AIRSPACE_VALUE_X_OFFSET, y, value);
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
  char aglLineBuf[16];
  float aglM;
  if (getDisplayAglM(aglM)) {
    snprintf(aglLineBuf, sizeof(aglLineBuf), "%d%s", (int)roundf(altitudeToDisplay(aglM)), altitudeUnitLabel());
  } else {
    snprintf(aglLineBuf, sizeof(aglLineBuf), "--%s", altitudeUnitLabel());
  }
  drawBoxRow(5, top + rowH - 6, "AGL", aglLineBuf);  // left-aligned under the ALTITUDE header (x=5)


  // =========================================================
  // BOX (0,1): GROUND SPEED + HEADING
  // =========================================================

  u8g2.setFont(u8g2_font_helvB10_tf);

  u8g2.drawStr(
    colW + 5,
    top + 14,
    "V GROUND");

  if (gps.speed.isValid() && gps.speed.age() < 2000) {

    snprintf(
      buffer,
      sizeof(buffer),
      "%d",
      (int)roundf(speedKphToDisplay(gps.speed.kmph())));

    drawLargeValueWithSmallUnit(
      colW + colW / 2,
      top + rowH / 2 + 20,  // matches ALTITUDE's +20 so the two big numbers sit inline
      colW - 10,
      buffer,
      speedUnitLabel());

  } else {

    drawLargeValueWithSmallUnit(
      colW + colW / 2,
      top + rowH / 2 + 20,
      colW - 10,
      "--",
      speedUnitLabel());
  }


  // ---------------------------------------------------------
  // Heading
  // ---------------------------------------------------------

  char hdgValueBuf[8];
  if (gps.course.isValid()) {
    snprintf(hdgValueBuf, sizeof(hdgValueBuf), "%s", getCompassDirection(gps.course.deg()));
  } else {
    snprintf(hdgValueBuf, sizeof(hdgValueBuf), "---");
  }

  drawBoxRow(colW + 5, top + rowH - 6, "HDG", hdgValueBuf);  // left-aligned under the V GROUND header (x=colW+5)


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
      currentClimbRateAvgMS);

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
  // distance (see getAirspaceClearanceSnapshot() -- VERT is the airspace
  // directly above/below you, HORI the nearest airspace at your altitude),
  // recentred as a 2-line block around the box's vertical
  // middle instead of the old 3-line block that started at the middle
  // and ran downward. Each line falls back to "--" independently if its
  // data isn't available yet.
  // =========================================================

  u8g2.setFont(u8g2_font_helvB10_tf);

  u8g2.drawStr(
    colW + 5,
    top + rowH + 14,
    "AIR SPACE");

  // VERT = nearest controlled airspace directly above/below you (you're
  // horizontally inside it); HORI = distance to the nearest controlled
  // airspace at YOUR altitude. Independent searches -- see
  // findAirspaceClearances(). "--" when there's nothing to report.
  AirspaceClearance boxClearance;
  bool boxClearanceValid = getAirspaceClearanceSnapshot(boxClearance);

  char vertBuf[16];
  char horiBuf[16];
  char cloudBuf[16];
  formatVertValue(vertBuf, sizeof(vertBuf), boxClearanceValid && boxClearance.vertValid, boxClearance.vertDistance_ft);
  formatHoriValue(horiBuf, sizeof(horiBuf), boxClearanceValid && boxClearance.horizValid, boxClearance.horizDistance_km);
  formatCloudBaseValue(cloudBuf, sizeof(cloudBuf));

  int rowY1, rowY2, rowY3;
  computeBoxRows(top + rowH + 14, top + 2 * rowH, rowY1, rowY2, rowY3);

  drawBoxRow(colW + 5, rowY1, "VERT", vertBuf);
  drawBoxRow(colW + 5, rowY2, "HORI", horiBuf);
  drawBoxRow(colW + 5, rowY3, "BASE", cloudBuf);


  // =========================================================
  // BOX (2,0): GLIDE RATIO
  // =========================================================

  u8g2.setFont(u8g2_font_helvB10_tf);

  u8g2.drawStr(
    5,
    top + 2 * rowH + 14,
    "GLIDE RATIO");

  bool glideValid =
    bmpOK && gps.speed.isValid() && gps.speed.age() < 2000 &&
    currentClimbRateAvgMS < -CLIMB_DEADBAND_MS && currentClimbRateAvgMS > -20.0f;

  if (glideValid) {

    float glideRatio =
      gps.speed.mps() / (-currentClimbRateAvgMS);

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
  // BOX (2,1): WIND / AIRSPEED -- left-aligned to match the AIR SPACE
  // box directly above (same column), so all six values in this column
  // line up in one place.
  // =========================================================

  u8g2.setFont(u8g2_font_helvB10_tf);

  u8g2.drawStr(
    colW + 5,
    top + 2 * rowH + 14,
    "WIND / AIRSPEED");

  char windBuf[12];
  char airBuf[12];
  char windDirBuf[8];
  if (windEstimateValid) {
    snprintf(windBuf, sizeof(windBuf), "%.0f %s", speedKphToDisplay(estimatedWindSpeedKph), speedUnitLabel());
    snprintf(windDirBuf, sizeof(windDirBuf), "%s", getCompassDirection(estimatedWindDirectionDeg));
    snprintf(airBuf, sizeof(airBuf), "%.0f %s", speedKphToDisplay(estimatedAirspeedKph), speedUnitLabel());
  } else {
    snprintf(windBuf, sizeof(windBuf), "-- %s", speedUnitLabel());
    snprintf(windDirBuf, sizeof(windDirBuf), "--");
    snprintf(airBuf, sizeof(airBuf), "-- %s", speedUnitLabel());
  }

  int windRowY1, windRowY2, windRowY3;
  computeBoxRows(top + 2 * rowH + 14, top + 3 * rowH, windRowY1, windRowY2, windRowY3);

  drawBoxRow(colW + 5, windRowY1, "WIND", windBuf);
  drawBoxRow(colW + 5, windRowY2, "FROM", windDirBuf);
  drawBoxRow(colW + 5, windRowY3, "AIR", airBuf);
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

    // Smaller than helvB24, so the longest message fits across the screen.
    u8g2.setFont(u8g2_font_helvB18_tf);

    const char* msg = "No Weather Data";
    if (!sharedPosition.everValid) {
      msg = "Waiting for GPS fix";
    } else if (!wifiConnected) {
      msg = "No WiFi";
    } else if (weatherFetchStatus == FETCH_FAILED) {
      msg = "No internet";
    }

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
// =====================================================
// TRAFFIC TRACKS
// Each ADS-B poll replaces adsbDoc, which only says where aircraft are right
// now. The page keeps its own short memory of them here, so an aircraft can
// keep moving between polls (15 s apart) and survive a missed poll. Only
// drawADSBPage() touches this, on Core 1, so it needs no mutex of its own.
// =====================================================
#define MAX_ADSB_TRACKS 24
#define TRAFFIC_ESTIMATED_AFTER_MS 30000UL  // older than this: drawn as an outline
// Close traffic: 4,000 ft allows for others climbing well above or below you.
#define TRAFFIC_CLOSE_KM 5.0f
#define TRAFFIC_CLOSE_FT 4000.0f
// Ground Traffic: Hide also removes anything below both of these.
#define TRAFFIC_GROUND_BELOW_FT 200.0f
#define TRAFFIC_GROUND_BELOW_KT 40.0f
// Labels: Relevant labels aircraft within this of own altitude (the zoom rule
// is separate: TRAFFIC_ZOOM_RELEVANT_FT). Every mode stops at the label cap.
#define TRAFFIC_LABEL_RELEVANT_FT 7000.0f
#define TRAFFIC_MAX_LABELS 8
// Removal after 60 s is TRAFFIC_REMOVE_AFTER_MS, in DrawPages.h.
#define TRAFFIC_REMOVE_OFFLINE_MS 120000UL  // removal while the connection is down

struct AdsbTrack {
  bool used;
  char hex[9];
  float lat, lon, altFeet, speedKt, headingDeg;
  bool altKnown;
  bool onGround;
  bool headingKnown;
  TrafficIconType icon;
  unsigned long lastPositionMs;
  float lastDistanceKm;  // from the last draw, for choosing which track to replace
};

static AdsbTrack adsbTracks[MAX_ADSB_TRACKS];
static uint32_t adsbTracksGeneration = 0;

// Set each time the page draws. Asks for faster redraws so the rings around
// close traffic can flash: see displayRefreshIntervalMs().
static bool trafficHasCloseAircraft = false;

// Static rather than on the stack: Core 1's stack is small and these are big.
static AircraftSnapshot trafficSnapshot[MAX_DISPLAYED_AIRCRAFT];

// The Ground Traffic setting: see adsbHideGroundTraffic in settings.h. A
// helicopter (only ever ADS-B category A7 here) can hover low and slow, so
// it's always shown. Airborne traffic otherwise shows at any altitude.
static bool adsbTrackHidden(bool onGround, bool altKnown, float altFeet, float speedKt,
                            TrafficIconType icon) {
  if (!adsbHideGroundTraffic || icon == TRAFFIC_ICON_HELICOPTER) return false;
  if (onGround) return true;
  return altKnown && altFeet < TRAFFIC_GROUND_BELOW_FT && speedKt < TRAFFIC_GROUND_BELOW_KT;
}

// Brings adsbTracks[] up to date if a new poll has been published. If the
// mutex is busy, the next redraw simply tries again.
static void mergeAdsbTracks(float myLat, float myLon) {
  if (adsbFetchGeneration == adsbTracksGeneration) return;
  if (backgroundDataMutex == nullptr ||
      xSemaphoreTake(backgroundDataMutex, pdMS_TO_TICKS(20)) != pdTRUE) {
    return;
  }

  const unsigned long fetchMs = adsbDocFetchMs;
  JsonArray aircraft = adsbDoc["ac"].as<JsonArray>();

  for (JsonObject ac : aircraft) {
    const char* hex = ac["hex"];
    float acLat = ac["lat"];
    float acLon = ac["lon"];
    if (hex == nullptr || acLat == 0.0f || acLon == 0.0f) continue;

    // seen_pos = how many seconds old this position already was at the
    // server, so a position that's gone stale there isn't treated as fresh.
    float seenPosSeconds = ac["seen_pos"].is<float>() ? (float)ac["seen_pos"] : 0.0f;
    if (seenPosSeconds < 0.0f) seenPosSeconds = 0.0f;
    if (seenPosSeconds * 1000.0f >= (float)TRAFFIC_REMOVE_AFTER_MS) continue;
    const unsigned long positionMs = fetchMs - (unsigned long)(seenPosSeconds * 1000.0f);

    // alt_baro is the word "ground" for an aircraft on the ground.
    const char* altText = ac["alt_baro"].as<const char*>();
    const bool onGround = altText != nullptr && strcmp(altText, "ground") == 0;
    const bool altKnown = ac["alt_baro"].is<float>();
    const float altFeet = ac["alt_baro"];
    const float speedKt = ac["gs"];
    const TrafficIconType icon = trafficIconFromAdsbCategory(ac["category"].as<const char*>());
    const bool hidden = adsbTrackHidden(onGround, altKnown, altFeet, speedKt, icon);

    int slot = -1;
    int freeSlot = -1;
    int farthestSlot = 0;
    for (int i = 0; i < MAX_ADSB_TRACKS; i++) {
      if (!adsbTracks[i].used) {
        if (freeSlot < 0) freeSlot = i;
        continue;
      }
      if (strcmp(adsbTracks[i].hex, hex) == 0) {
        slot = i;
        break;
      }
      if (adsbTracks[i].lastDistanceKm > adsbTracks[farthestSlot].lastDistanceKm) farthestSlot = i;
    }

    float distanceKm = -1.0f;
    if (slot < 0) {
      // Don't let parked aircraft at a nearby airport use up the table.
      if (hidden) continue;
      distanceKm = getDistanceKM(myLat, myLon, acLat, acLon);
      if (freeSlot >= 0) {
        slot = freeSlot;
      } else if (distanceKm < adsbTracks[farthestSlot].lastDistanceKm) {
        slot = farthestSlot;  // table full: closer aircraft matter more
      } else {
        continue;
      }
      strlcpy(adsbTracks[slot].hex, hex, sizeof(adsbTracks[slot].hex));
      adsbTracks[slot].used = true;
      adsbTracks[slot].lastDistanceKm = distanceKm;
    }

    AdsbTrack& track = adsbTracks[slot];
    track.lat = acLat;
    track.lon = acLon;
    track.onGround = onGround;
    track.altKnown = altKnown;
    track.altFeet = altFeet;
    track.speedKt = speedKt;
    track.headingKnown = ac["track"].is<float>();
    track.headingDeg = ac["track"];
    track.icon = icon;
    track.lastPositionMs = positionMs;
  }

  adsbTracksGeneration = adsbFetchGeneration;
  xSemaphoreGive(backgroundDataMutex);
}

// Moves a position forward along its track for the time since it was reported.
static void estimateTrafficPosition(float lat, float lon, float speedKt, float headingDeg,
                                    unsigned long ageMs, float& outLat, float& outLon) {
  const float distanceKm = speedKt * 1.852f * ((float)ageMs / 3600000.0f);
  const float headingRad = deg2rad(headingDeg);
  outLat = lat + (distanceKm * cosf(headingRad)) / 111.32f;
  const float cosLat = cosf(deg2rad(lat));
  outLon = (cosLat > 0.01f) ? lon + (distanceKm * sinf(headingRad)) / (111.32f * cosLat) : lon;
}

// True while the ADS-B feed can't be reached, so a silent aircraft may well
// still be flying rather than gone.
static bool trafficConnectionLost() {
  return !wifiConnected || adsbFetchStatus == FETCH_FAILED;
}

// Fills trafficSnapshot[] with every aircraft to consider this frame, at its
// estimated current position, and returns how many. Removes any that have
// gone too long with no update: 60 s, or 2 minutes for ADS-B while the
// connection is down. FANET comes by radio, so a lost connection changes
// nothing for it.
static int buildTrafficSnapshot(float myLat, float myLon, unsigned long nowMs) {
  int count = 0;
  const unsigned long adsbRemoveAfterMs =
      trafficConnectionLost() ? TRAFFIC_REMOVE_OFFLINE_MS : TRAFFIC_REMOVE_AFTER_MS;

  // FANET first: these are the nearby pilots, and they should never lose
  // their place to distant ADS-B traffic. No mutex needed: see the
  // FanetContact comment in DrawPages.h.
  for (int i = 0; i < MAX_FANET_CONTACTS && count < MAX_DISPLAYED_AIRCRAFT; i++) {
    FanetContact& contact = fanetContacts[i];
    if (!contact.valid) continue;

    const unsigned long ageMs = nowMs - contact.lastSeenMs;
    if (ageMs > TRAFFIC_REMOVE_AFTER_MS) {
      contact.valid = false;  // also frees the slot for reuse
      continue;
    }

    AircraftSnapshot& s = trafficSnapshot[count++];
    s.speedKt = contact.speedKmh * 0.539957f;  // km/h -> kt
    s.headingDeg = contact.headingDeg;
    estimateTrafficPosition(contact.lat, contact.lon, s.speedKt, s.headingDeg, ageMs, s.lat, s.lon);
    s.altFeet = contact.altitudeM * 3.28084f;
    s.altKnown = true;
    s.isFanet = true;
    s.climbMs = contact.climbMs;
    s.icon = trafficIconFromFanetType(contact.aircraftType);
    s.estimated = ageMs > TRAFFIC_ESTIMATED_AFTER_MS;
    s.distanceKm = getDistanceKM(myLat, myLon, s.lat, s.lon);
    snprintf(s.label, sizeof(s.label), "%02X:%04X", contact.addr.manufacturer, contact.addr.id);
  }

  mergeAdsbTracks(myLat, myLon);

  for (int i = 0; i < MAX_ADSB_TRACKS; i++) {
    AdsbTrack& track = adsbTracks[i];
    if (!track.used) continue;

    const unsigned long ageMs = nowMs - track.lastPositionMs;
    if (ageMs > adsbRemoveAfterMs) {
      track.used = false;
      continue;
    }
    if (adsbTrackHidden(track.onGround, track.altKnown, track.altFeet, track.speedKt, track.icon)) continue;
    if (count >= MAX_DISPLAYED_AIRCRAFT) continue;

    AircraftSnapshot& s = trafficSnapshot[count++];
    s.speedKt = track.speedKt;
    s.headingDeg = track.headingDeg;
    if (track.headingKnown) {
      estimateTrafficPosition(track.lat, track.lon, track.speedKt, track.headingDeg, ageMs, s.lat, s.lon);
    } else {
      s.lat = track.lat;  // no track reported, so nothing to move it along
      s.lon = track.lon;
    }
    s.altFeet = track.altFeet;
    s.altKnown = track.altKnown;
    s.isFanet = false;
    s.climbMs = 0.0f;
    s.icon = track.icon;
    s.estimated = ageMs > TRAFFIC_ESTIMATED_AFTER_MS;
    s.distanceKm = getDistanceKM(myLat, myLon, s.lat, s.lon);
    track.lastDistanceKm = s.distanceKm;
  }

  return count;
}

// =====================================================
// TRAFFIC AUTO-ZOOM
// Picks the smallest range step that comfortably contains the nearest
// relevant aircraft. Steps never exceed the Range Rings setting, which is
// also the view when nothing relevant is around. Zooms in at once, but only
// zooms out once the wider view has been called for throughout
// TRAFFIC_ZOOM_OUT_HOLD_MS, so the scale doesn't flicker.
// =====================================================
#define TRAFFIC_ZOOM_RELEVANT_FT 3000.0f    // relevant = within this of own altitude
#define TRAFFIC_ZOOM_COMFORT 0.8f           // nearest aircraft sits within 80% of the range
#define TRAFFIC_ZOOM_OUT_HOLD_MS 10000UL

// nearestRelevantKm is negative if there's no relevant aircraft at all.
static float updateTrafficZoom(float nearestRelevantKm, unsigned long nowMs) {
  static const float ZOOM_STEPS_KM[] = { 5.0f, 10.0f, 30.0f, 60.0f };
  static float currentRangeKm = 0.0f;
  static bool zoomOutPending = false;
  static unsigned long zoomOutPendingSinceMs = 0;

  float wantedRangeKm = adsbRingOuterKm;
  if (nearestRelevantKm >= 0.0f) {
    for (float stepKm : ZOOM_STEPS_KM) {
      if (stepKm >= adsbRingOuterKm) break;
      if (nearestRelevantKm <= stepKm * TRAFFIC_ZOOM_COMFORT) {
        wantedRangeKm = stepKm;
        break;
      }
    }
  }

  // First draw, or the Range Rings setting was just reduced below this view.
  if (currentRangeKm <= 0.0f || currentRangeKm > adsbRingOuterKm) {
    currentRangeKm = wantedRangeKm;
    zoomOutPending = false;
  } else if (wantedRangeKm <= currentRangeKm) {
    currentRangeKm = wantedRangeKm;
    zoomOutPending = false;
  } else if (!zoomOutPending) {
    zoomOutPending = true;
    zoomOutPendingSinceMs = nowMs;
  } else if (nowMs - zoomOutPendingSinceMs >= TRAFFIC_ZOOM_OUT_HOLD_MS) {
    currentRangeKm = wantedRangeKm;
    zoomOutPending = false;
  }

  return currentRangeKm;
}

// "10km", or "2.5km" for half of the 5 km view.
static void formatRingLabel(char* buf, size_t buflen, float km) {
  if (fabsf(km - roundf(km)) < 0.01f) {
    snprintf(buf, buflen, "%.0fkm", km);
  } else {
    snprintf(buf, buflen, "%.1fkm", km);
  }
}

// =====================================================
// TRAFFIC LABELS
// Every aircraft keeps its label. Each tries the sides of its icon in turn
// and takes the first clear one. In a tight cluster no side may be clear, so
// it then tries further out, in rows above and below, and failing all that
// takes whichever spot overlaps least.
// =====================================================
#define TRAFFIC_LABEL_MAX_ROWS_OUT 4

struct ScreenRect {
  int16_t x, y, w, h;
};

static int rectOverlapArea(const ScreenRect& a, const ScreenRect& b) {
  const int w = min(a.x + a.w, b.x + b.w) - max(a.x, b.x);
  const int h = min(a.y + a.h, b.y + b.h) - max(a.y, b.y);
  return (w > 0 && h > 0) ? w * h : 0;
}

static ScreenRect placeTrafficLabel(int iconX, int iconY, int clearPx, int w, int h,
                                    const ScreenRect* obstacles, int obstacleCount) {
  const int rightX = iconX + clearPx;
  const int leftX = iconX - clearPx - w;
  const int centreX = iconX - w / 2;
  const int middleY = iconY - h / 2;
  const int aboveY = iconY - clearPx - h;
  const int belowY = iconY + clearPx;
  const int rowPitch = h + 2;

  ScreenRect best = { 0, 0, (int16_t)w, (int16_t)h };
  int bestOverlap = -1;

  for (int rowsOut = 0; rowsOut <= TRAFFIC_LABEL_MAX_ROWS_OUT; rowsOut++) {
    const int up = rowsOut * rowPitch;
    const int candidates[][2] = {
      // The four sides first, then the same spots moved further out.
      { rightX, middleY - up }, { leftX, middleY - up }, { centreX, aboveY - up }, { centreX, belowY + up },
      { rightX, middleY + up }, { leftX, middleY + up }
    };

    for (const auto& candidate : candidates) {
      // Keep it on the screen, below the top bar.
      const ScreenRect rect = {
        (int16_t)constrain(candidate[0], 0, max(0, SCREEN_W - w)),
        (int16_t)constrain(candidate[1], TOP_BAR_HEIGHT_PX, SCREEN_H - h),
        (int16_t)w, (int16_t)h
      };

      int overlap = 0;
      for (int i = 0; i < obstacleCount; i++) overlap += rectOverlapArea(rect, obstacles[i]);

      if (bestOverlap < 0 || overlap < bestOverlap) {
        best = rect;
        bestOverlap = overlap;
      }
      if (overlap == 0) return best;
    }
  }
  return best;
}

// Compass letters and ring distances: text on a small cleared box, drawn over
// everything so labels and icons can never cover it.
static void drawTrafficMapText(int x, int baselineY, const char* text) {
  const int ascent = u8g2.getAscent();
  u8g2.setDrawColor(0);
  u8g2.drawBox(x - 1, baselineY - ascent - 1, u8g2.getStrWidth(text) + 2, ascent + 2);
  u8g2.setDrawColor(1);
  u8g2.drawStr(x, baselineY, text);
}

// A message on a cleared background, so the rings don't run through it.
static void drawTrafficStatus(int cx, int y, const char* text) {
  u8g2.setFont(u8g2_font_helvB14_tf);
  const int w = u8g2.getStrWidth(text);
  u8g2.setDrawColor(0);
  u8g2.drawBox(cx - w / 2 - 4, y - 18, w + 8, 24);
  u8g2.setDrawColor(1);
  u8g2.drawStr(cx - w / 2, y, text);
}

unsigned long displayRefreshIntervalMs() {
  // Flashing at 2 Hz takes 4 redraws a second: rings on, rings off, twice.
  return (!menuActive && currentPage == PAGE_ADSB && trafficHasCloseAircraft) ? 250UL : 1000UL;
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
  float myAltitudeFeet = sharedOwnPressureAltFeet;

  // 2. Gather every known aircraft at its estimated current position. All
  // the trig/SPI rendering below runs without holding the mutex, so a slow
  // redraw can never make the Core 0 background task wait, and vice versa.
  // Without a GPS fix there's no centre to plot anything around.
  const unsigned long trafficNowMs = millis();
  const bool haveGpsFix = sharedPosition.everValid;
  const int snapshotCount = haveGpsFix ? buildTrafficSnapshot(MY_LAT, MY_LON, trafficNowMs) : 0;

  // 3. Work out what those aircraft mean for this frame: the nearest one that
  // drives the zoom, close traffic, and whether anything's within range.
  // Close traffic gets the flashing rings. Airborne ADS-B only: an aircraft on
  // the ground is still shown, but there's no need to flash it. FANET is left
  // out too, because pilots thermalling together are very often this close,
  // and the rings would turn into noise. This only affects the display: voice
  // and siren alerts use the ADSB Settings values, in performADSBUpdate().
  float nearestRelevantKm = -1.0f;
  int withinRangeRingsCount = 0;
  trafficHasCloseAircraft = false;

  for (int i = 0; i < snapshotCount; i++) {
    AircraftSnapshot& s = trafficSnapshot[i];

    if (s.distanceKm <= adsbRingOuterKm) withinRangeRingsCount++;

    if (s.altKnown && fabsf(s.altFeet - myAltitudeFeet) <= TRAFFIC_ZOOM_RELEVANT_FT &&
        (nearestRelevantKm < 0.0f || s.distanceKm < nearestRelevantKm)) {
      nearestRelevantKm = s.distanceKm;
    }

    s.isClose = !s.isFanet && s.altKnown && s.distanceKm <= TRAFFIC_CLOSE_KM &&
                fabsf(s.altFeet - myAltitudeFeet) <= TRAFFIC_CLOSE_FT;
    if (s.isClose) {
      conflictDetectedThisFrame = true;
      trafficHasCloseAircraft = true;
    }
  }

  const float MAX_RADIUS_KM = updateTrafficZoom(nearestRelevantKm, trafficNowMs);

  // The ring labels always show the current range.
  char outerRingLabelBuf[8];
  char innerRingLabelBuf[8];
  formatRingLabel(outerRingLabelBuf, sizeof(outerRingLabelBuf), MAX_RADIUS_KM);
  formatRingLabel(innerRingLabelBuf, sizeof(innerRingLabelBuf), MAX_RADIUS_KM / 2.0f);
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
  if (gps.speed.isValid() && gps.speed.age() < 2000) {
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

  // Compass letters and range ring labels, drawn at the end of this function
  // so nothing can cover them. The ring labels sit along the same N radial as
  // the "N" marker, so they rotate together with it in track-up mode rather
  // than staying fixed to the top of the screen.
  const int innerNX = cx + (int)((r / 2) * sin(crosshairAngleRad));
  const int innerNY = cy - (int)((r / 2) * cos(crosshairAngleRad));

  auto drawCompassAndRingLabels = [&]() {
    u8g2.setFont(u8g2_font_8x13_tf);  // bumped up one size from 7x14_tf
    drawTrafficMapText(nX - 2, nY - 2, "N");
    drawTrafficMapText(sX - 2, sY + 8, "S");
    drawTrafficMapText(eX + 4, eY + 3, "E");
    drawTrafficMapText(wX - 8, wY + 3, "W");

    u8g2.setFont(u8g2_font_7x13_tf);  // bumped up one size from 6x10_tf
    drawTrafficMapText(nX + 10, nY - 2, outerRingLabel);
    drawTrafficMapText(innerNX + 10, innerNY - 2, innerRingLabel);
  };

  // 5. Draw Center Navigation Reference Symbol
  drawGliderHeadingArrow(cx, cy, 6, isMoving ? 0.0f : -1.0f);

  // 6. DRAW VARIO OVERLAY BOX (Upgraded to Helvetica Bold)
  u8g2.setFont(u8g2_font_helvB18_tf);  // True Helvetica Bold 14px — matches flight tags!

  char varioText[12];  // Buffer to store formatted layout text
  if (currentClimbRateAvgMS >= 0.0f) {
    snprintf(varioText, sizeof(varioText), "+%.1f m/s", currentClimbRateAvgMS);
  } else {
    snprintf(varioText, sizeof(varioText), "%.1f m/s", currentClimbRateAvgMS);
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

  // While the connection is down, say so and say it's being worked on, even
  // with aircraft still on the page: their positions are only estimates now.
  // Sits right of the vario box, above where the airspace bar starts.
  ScreenRect connectionRect = { 0, 0, 0, 0 };
  if (haveGpsFix && trafficConnectionLost()) {
    const char* line1 = !wifiEnabled ? "WiFi off" : (!wifiConnected ? "No WiFi" : "No internet");
    const char* line2 = wifiEnabled ? "Reconnecting..." : "";

    u8g2.setFont(u8g2_font_helvB10_tf);
    const int textW = max(u8g2.getStrWidth(line1), u8g2.getStrWidth(line2));
    connectionRect = { (int16_t)(SCREEN_W - textW - 8), 345, (int16_t)(textW + 8), 25 };

    u8g2.setDrawColor(0);
    u8g2.drawBox(connectionRect.x, connectionRect.y, connectionRect.w, connectionRect.h);
    u8g2.setDrawColor(1);
    u8g2.drawStr(connectionRect.x + 4, 356, line1);
    u8g2.drawStr(connectionRect.x + 4, 368, line2);
  }

  // Say why the page is empty, rather than leaving it blank. "No traffic" is
  // only claimed after a good fetch: with no WiFi or no internet nobody knows
  // what's out there, and saying "no traffic" would be falsely reassuring.
  if (withinRangeRingsCount == 0) {
    char statusBuf[32];
    if (!haveGpsFix) {
      snprintf(statusBuf, sizeof(statusBuf), "Waiting for GPS fix");
    } else if (!wifiConnected) {
      snprintf(statusBuf, sizeof(statusBuf), "No WiFi");
    } else if (adsbFetchStatus == FETCH_FAILED) {
      snprintf(statusBuf, sizeof(statusBuf), "No internet");
    } else if (adsbFetchStatus == FETCH_NOT_TRIED) {
      snprintf(statusBuf, sizeof(statusBuf), "Checking for traffic");
    } else {
      snprintf(statusBuf, sizeof(statusBuf), "No traffic within %.0f km", adsbRingOuterKm);
    }
    drawTrafficStatus(cx, cy + 34, statusBuf);
    drawCompassAndRingLabels();
    return;
  }

  // 7. Work out where the aircraft within the current range go. All their
  // positions are needed first, so the labels can be placed clear of all of
  // them. The icons themselves are drawn last, so a label can never hide one.
  struct PlottedAircraft {
    int16_t x, y;
    float displayHeading;
    int8_t snapshotIndex;
  };
  static PlottedAircraft plotted[MAX_DISPLAYED_AIRCRAFT];
  // Room for a label and an icon per aircraft, plus the own-ship arrow, the
  // three fixed boxes and the connection message.
  static ScreenRect obstacles[2 * MAX_DISPLAYED_AIRCRAFT + 5];
  int plottedCount = 0;
  int obstacleCount = 0;

  obstacles[obstacleCount++] = { (int16_t)(cx - 8), (int16_t)(cy - 8), 16, 16 };
  obstacles[obstacleCount++] = { (int16_t)altitudeX, (int16_t)altitudeY, (int16_t)altitudeW, (int16_t)altitudeH };
  obstacles[obstacleCount++] = { (int16_t)gsX, (int16_t)gsY, (int16_t)gsW, (int16_t)gsH };
  obstacles[obstacleCount++] = { (int16_t)varioX, (int16_t)varioY, (int16_t)varioW, (int16_t)varioH };
  obstacles[obstacleCount++] = connectionRect;  // zero-sized if there's no message

  for (int i = 0; i < snapshotCount; i++) {
    const AircraftSnapshot& s = trafficSnapshot[i];
    if (s.distanceKm > MAX_RADIUS_KM) continue;

    float relativeBearing = getBearing(MY_LAT, MY_LON, s.lat, s.lon);
    if (isMoving) {
      relativeBearing -= gliderHeading;
    }

    const float angleRad = deg2rad(relativeBearing) - (PI / 2.0f);
    const float pixelDistance = (s.distanceKm / MAX_RADIUS_KM) * (float)r;

    const int acX = cx + (int)(pixelDistance * cos(angleRad));
    const int acY = cy + (int)(pixelDistance * sin(angleRad));

    // Rotated the same way the crosshair is -- relative to your heading in
    // track-up mode, or absolute compass heading in north-up mode -- so it
    // stays visually consistent with the rest of the display.
    const float acDisplayHeading = fmodf(s.headingDeg - (isMoving ? gliderHeading : 0.0f) + 360.0f, 360.0f);

    plotted[plottedCount++] = { (int16_t)acX, (int16_t)acY, acDisplayHeading, (int8_t)i };
    obstacles[obstacleCount++] = { (int16_t)(acX - 12), (int16_t)(acY - 12), 24, 24 };
  }

  // Close traffic comes first, then nearest to furthest. That's the order
  // labels are placed in, and so who gets one if there are more than the cap.
  for (int a = 0; a < plottedCount - 1; a++) {
    for (int b = a + 1; b < plottedCount; b++) {
      const AircraftSnapshot& sa = trafficSnapshot[plotted[a].snapshotIndex];
      const AircraftSnapshot& sb = trafficSnapshot[plotted[b].snapshotIndex];
      if ((sb.isClose && !sa.isClose) ||
          (sb.isClose == sa.isClose && sb.distanceKm < sa.distanceKm)) {
        const PlottedAircraft tmp = plotted[a];
        plotted[a] = plotted[b];
        plotted[b] = tmp;
      }
    }
  }

  // 8. Bold labels, for sunlight, in the smaller font so they crowd less.
  u8g2.setFont(u8g2_font_helvB10_tf);
  int labelCount = 0;

  for (int i = 0; i < plottedCount && labelCount < TRAFFIC_MAX_LABELS; i++) {
    const AircraftSnapshot& s = trafficSnapshot[plotted[i].snapshotIndex];

    if (adsbLabelMode == ADSB_LABELS_OFF) break;
    // Anything else in this mode shows as an icon only. An aircraft on the
    // ground counts as altitude 0 here.
    if (adsbLabelMode == ADSB_LABELS_RELEVANT &&
        fabsf(s.altFeet - myAltitudeFeet) > TRAFFIC_LABEL_RELEVANT_FT) {
      continue;
    }
    labelCount++;

    char dataTag[24];

    if (s.isFanet) {
      // Climb rate is far more relevant than flight level for a nearby
      // paraglider/paramotor -- and the FANET address doubles as a
      // visual "this is FANET, not ADS-B" cue next to the marker.
      snprintf(dataTag, sizeof(dataTag), "PG %s %+.1fm/s", s.label, s.climbMs);
    } else {
      float flightLevelFloat = s.altFeet / 1000.0f;
      if (flightLevelFloat < 0.0f) flightLevelFloat = 0.0f;

      snprintf(dataTag, sizeof(dataTag), "FL%.1f %s %.0fkt", flightLevelFloat, getCompassDirection(s.headingDeg), s.speedKt);
    }

    const int textW = u8g2.getStrWidth(dataTag) + 8;
    const int textH = 17;
    const int clearPx = s.isClose ? 21 : 14;  // outside the rings, or the icon

    const ScreenRect labelRect = placeTrafficLabel(plotted[i].x, plotted[i].y, clearPx, textW, textH,
                                                   obstacles, obstacleCount);
    obstacles[obstacleCount++] = labelRect;

    // A label that had to move away from its icon gets a line back to it.
    const int nearestX = constrain((int)plotted[i].x, (int)labelRect.x, labelRect.x + labelRect.w - 1);
    const int nearestY = constrain((int)plotted[i].y, (int)labelRect.y, labelRect.y + labelRect.h - 1);
    if (abs(nearestX - plotted[i].x) > clearPx + 2 || abs(nearestY - plotted[i].y) > clearPx + 2) {
      u8g2.drawLine(plotted[i].x, plotted[i].y, nearestX, nearestY);
    }

    // Steady, whether or not the rings are currently on. Filled either way,
    // so the rings and crosshair don't run through the text.
    u8g2.setDrawColor(s.isClose ? 1 : 0);
    u8g2.drawBox(labelRect.x, labelRect.y, labelRect.w, labelRect.h);
    u8g2.setDrawColor(1);
    u8g2.drawFrame(labelRect.x, labelRect.y, labelRect.w, labelRect.h);

    u8g2.setDrawColor(s.isClose ? 0 : 1);
    u8g2.drawStr(labelRect.x + 4, labelRect.y + 13, dataTag);
    u8g2.setDrawColor(1);
  }

  // 9. The icons go on top. Rings on for 250 ms, off for 250 ms = 2 Hz.
  const bool closeRingsOn = ((trafficNowMs / 250UL) % 2UL) == 0;

  for (int i = 0; i < plottedCount; i++) {
    const AircraftSnapshot& s = trafficSnapshot[plotted[i].snapshotIndex];

    drawTrafficIcon(plotted[i].x, plotted[i].y, s.icon, plotted[i].displayHeading, s.estimated);

    if (s.isClose && closeRingsOn) {
      u8g2.drawCircle(plotted[i].x, plotted[i].y, 15);
      u8g2.drawCircle(plotted[i].x, plotted[i].y, 18);
    }
  }

  drawCompassAndRingLabels();

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
  char aglLineBuf[16];
  float aglM;
  if (getDisplayAglM(aglM)) {
    snprintf(aglLineBuf, sizeof(aglLineBuf), "%d%s", (int)roundf(altitudeToDisplay(aglM)), altitudeUnitLabel());
  } else {
    snprintf(aglLineBuf, sizeof(aglLineBuf), "--%s", altitudeUnitLabel());
  }
  drawBoxRow(5, top + rowH - 6, "AGL", aglLineBuf);  // left-aligned under the ALTITUDE header (x=5)

  // =========================================================
  // BOX (0,1): V GROUND + HDG -- identical to drawParagliderPage()
  // =========================================================
  u8g2.setFont(u8g2_font_helvB10_tf);
  u8g2.drawStr(colW + 5, top + 14, "V GROUND");

  if (gps.speed.isValid() && gps.speed.age() < 2000) {
    snprintf(buffer, sizeof(buffer), "%d", (int)roundf(speedKphToDisplay(gps.speed.kmph())));
    drawLargeValueWithSmallUnit(colW + colW / 2, top + rowH / 2 + 20, colW - 10, buffer, speedUnitLabel());
  } else {
    drawLargeValueWithSmallUnit(colW + colW / 2, top + rowH / 2 + 20, colW - 10, "--", speedUnitLabel());
  }

  char hdgValueBuf[8];
  if (gps.course.isValid()) {
    snprintf(hdgValueBuf, sizeof(hdgValueBuf), "%s", getCompassDirection(gps.course.deg()));
  } else {
    snprintf(hdgValueBuf, sizeof(hdgValueBuf), "---");
  }
  drawBoxRow(colW + 5, top + rowH - 6, "HDG", hdgValueBuf);  // left-aligned under the V GROUND header (x=colW+5)

  // =========================================================
  // BOX (1,0): WIND / AIRSPEED -- same left-aligned row style as
  // drawParagliderPage()'s box (2,1), re-anchored to row 1 / col 0. Note
  // this box is beside AIR SPACE on this page (not stacked under it like
  // on the glider page), so its column lines up with ALTITUDE/CHT above
  // and below it, not with AIR SPACE's column.
  // =========================================================
  u8g2.setFont(u8g2_font_helvB10_tf);
  u8g2.drawStr(5, top + rowH + 14, "WIND / AIRSPEED");

  char windBuf[12];
  char airBuf[12];
  char windDirBuf[8];
  if (windEstimateValid) {
    snprintf(windBuf, sizeof(windBuf), "%.0f %s", speedKphToDisplay(estimatedWindSpeedKph), speedUnitLabel());
    snprintf(windDirBuf, sizeof(windDirBuf), "%s", getCompassDirection(estimatedWindDirectionDeg));
    snprintf(airBuf, sizeof(airBuf), "%.0f %s", speedKphToDisplay(estimatedAirspeedKph), speedUnitLabel());
  } else {
    snprintf(windBuf, sizeof(windBuf), "-- %s", speedUnitLabel());
    snprintf(windDirBuf, sizeof(windDirBuf), "--");
    snprintf(airBuf, sizeof(airBuf), "-- %s", speedUnitLabel());
  }

  int windRowY1, windRowY2, windRowY3;
  computeBoxRows(top + rowH + 14, top + 2 * rowH, windRowY1, windRowY2, windRowY3);

  drawBoxRow(5, windRowY1, "WIND", windBuf);
  drawBoxRow(5, windRowY2, "FROM", windDirBuf);
  drawBoxRow(5, windRowY3, "AIR", airBuf);

  // =========================================================
  // BOX (1,1): AIR SPACE -- ALT AGL moved out to the ALTITUDE box above,
  // so this now just shows nearest-airspace vertical/horizontal
  // distance, recentred as a 2-line block around the box's vertical
  // middle instead of the old 3-line block that started at the middle
  // and ran downward.
  // =========================================================
  u8g2.setFont(u8g2_font_helvB10_tf);
  u8g2.drawStr(colW + 5, top + rowH + 14, "AIR SPACE");

  // VERT = nearest controlled airspace directly above/below you (you're
  // horizontally inside it); HORI = distance to the nearest controlled
  // airspace at YOUR altitude. Independent searches -- see
  // findAirspaceClearances(). "--" when there's nothing to report.
  AirspaceClearance boxClearance;
  bool boxClearanceValid = getAirspaceClearanceSnapshot(boxClearance);

  char vertBuf[16];
  char horiBuf[16];
  char cloudBuf[16];
  formatVertValue(vertBuf, sizeof(vertBuf), boxClearanceValid && boxClearance.vertValid, boxClearance.vertDistance_ft);
  formatHoriValue(horiBuf, sizeof(horiBuf), boxClearanceValid && boxClearance.horizValid, boxClearance.horizDistance_km);
  formatCloudBaseValue(cloudBuf, sizeof(cloudBuf));

  int rowY1, rowY2, rowY3;
  computeBoxRows(top + rowH + 14, top + 2 * rowH, rowY1, rowY2, rowY3);

  drawBoxRow(colW + 5, rowY1, "VERT", vertBuf);
  drawBoxRow(colW + 5, rowY2, "HORI", horiBuf);
  drawBoxRow(colW + 5, rowY3, "BASE", cloudBuf);

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
  if (engineDataValid && engineBatteryPercentValid) {
    char batteryBuf[12];
    snprintf(batteryBuf, sizeof(batteryBuf), "BAT %u%%", engineBatteryPercent);
    u8g2.drawStr(
      SCREEN_W - 10 - u8g2.getStrWidth(batteryBuf),
      top + 2 * rowH + 14,
      batteryBuf);
  }
  if (engineDataValid && !engineChtFault) {
    snprintf(buffer, sizeof(buffer), "%.0f", engineChtC);
  } else {
    snprintf(buffer, sizeof(buffer), "--");
  }
  drawLargeValueWithSmallUnit(colW + colW / 2, top + 2 * rowH + rowH / 2 + 10, colW - 10, buffer, "\xb0" "C");
}

void drawDashboard() {
  u8g2.clearBuffer();
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
  u8g2.sendBuffer();
}

// =====================================================
// AIRSPACE WARNING OVERLAY
//
// Silent and invisible when there's nothing to report. Shows a banner
// only when the pilot is laterally inside and vertically inside or the
// vertical band is unknown. A short alert tone fires once on the
// transition into "inside" (edge-triggered, not every redraw).
// =====================================================
// Copies out the latest airspace scan result under backgroundDataMutex.
// Quick by design -- a plain struct copy, no SD/SPI work while the lock is
// held -- so callers on Core 1 (drawing) never make backgroundTask() (Core
// 0) wait for anything slower than a memcpy.
bool getAirspaceSnapshot(AirspaceResult& out) {
  bool valid = false;
  if (backgroundDataMutex != nullptr && xSemaphoreTake(backgroundDataMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
    valid = airspaceResultValid && airspaceScanIsFresh();
    if (valid) out = nearestAirspace;
    xSemaphoreGive(backgroundDataMutex);
  }
  return valid;
}

// Clearance counterpart: the AIR SPACE box's VERT / HORI rows. Same copy-out-
// under-the-lock pattern as the two accessors around it.
bool getAirspaceClearanceSnapshot(AirspaceClearance& out) {
  bool valid = false;
  if (backgroundDataMutex != nullptr && xSemaphoreTake(backgroundDataMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
    valid = airspaceClearanceValid && airspaceScanIsFresh();
    if (valid) out = airspaceClearance;
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
    valid = airspaceInfoResultValid && airspaceScanIsFresh();
    if (valid) out = nearestAirspaceInfo;
    xSemaphoreGive(backgroundDataMutex);
  }
  return valid;
}

void updateAirspaceAlert() {
  static bool wasInside = false;

  AirspaceResult result;
  if (!getAirspaceSnapshot(result)) return;

  const bool insideNow = result.insideHoriz && (result.vertKnown ? result.insideVert : true);
  if (insideNow && !wasInside) {
    playFeedbackTone(900.0f, 600);
  }
  wasInside = insideNow;
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

  // Inside laterally, and either inside vertically or the vertical band can't be resolved
  // (AGL/SFC floor/ceiling with no valid ground elevation). Unknown is treated as inside
  // (safe side) and flagged on the banner.
  const bool insideNow = result.insideHoriz && (result.vertKnown ? result.insideVert : true);
  const bool altUnknown = !result.vertKnown;

  if (!airspaceAlertBarEnabled) return false;

  if (!insideNow) return false;

  char line1[40];
  char line2[40];

  snprintf(line1, sizeof(line1), "INSIDE %s", result.name);
  snprintf(line2, sizeof(line2), "Class %s", result.classId);
  if (altUnknown) {
    snprintf(line2, sizeof(line2), "Class %s  ALT UNKNOWN", result.classId);
    if (u8g2.getStrWidth(line2) > SCREEN_W - 12) {
      snprintf(line2, sizeof(line2), "%s  ALT UNKNOWN", result.classId);
    }
    if (u8g2.getStrWidth(line2) > SCREEN_W - 12) {
      snprintf(line2, sizeof(line2), "ALT UNKNOWN");
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
