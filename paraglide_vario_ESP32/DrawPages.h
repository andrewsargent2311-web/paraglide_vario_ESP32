#ifndef DRAWPAGES_H
#define DRAWPAGES_H

// =====================================================
// DrawPages.h
// -----------------------------------------------------
// Interface for everything that draws to the display (top bar, the four
// page screens, and their shared drawing helpers). Implementation lives in
// DrawPages.cpp -- this header exists so both DrawPages.cpp and the main
// .ino can see the same globals, types, and constants.
//
// NOTE ON PLACEMENT: this header must be #included from the .ino BEFORE
// anything below is first used there (WindMeter / localMeters, in
// particular) -- it's included right after the other project headers,
// near the top of the .ino, for that reason.
// The actual *definitions* of the extern globals below still live in the
// .ino; this header only declares them so DrawPages.cpp can see them too.
// =====================================================

#include <Arduino.h>
#include <math.h>
#include <U8g2lib.h>
#include <TinyGPS++.h>
#include <ArduinoJson.h>
#include "OpenAirScanner.h"  // AirspaceResult
#include "menu.h"            // Page, PAGE_PARAGLIDER / PAGE_WEATHER / PAGE_ADSB / PAGE_PARAMOTOR, PAGE_COUNT
#include "settings.h"        // altitudeUnitLabel(), altitudeToDisplay(), speedUnitLabel(), speedKphToDisplay(), etc.

// =====================================================
// Display-related constants (moved here from the .ino so both the .ino
// and DrawPages.cpp share one definition). Note: SCREEN_W, SCREEN_H, and
// ACTIVE_PAGE_COUNT are NOT redeclared here -- menu.h (included above)
// already defines all three, and this header would just be duplicating it.
// =====================================================
#define TOP_BAR_HEIGHT_PX 32
#define AIRSPACE_WARN_HORIZ_KM 5.0f
#define AIRSPACE_WARN_VERT_FT 2000.0f
#define WEATHER_STATION_NAME_MAX 20
#define TRACKED_METERS 6
#define CLIMB_DEADBAND_MS 0.15f

// One collected/sorted weather station reading -- shared between the
// weather-fetch code in the .ino (which fills localMeters[]) and
// drawWeatherPage() (which reads it), so the type lives here.
struct WindMeter {
  char name[20];
  float distanceKm;
  float speedKph;
  float gustKph;
  float bearingDeg;     // wind direction reported BY the station (e.g. "N" = wind from the north)
  float geoBearingDeg;  // compass bearing FROM the glider TO the station (e.g. "NE")
  bool valid;
};

// =====================================================
// Globals defined in the .ino, used by the draw functions below.
// (u8g2, currentPage, PAGE_NAMES, and activePageIndex are NOT redeclared
// here -- menu.h, included above, already declares all four.)
// =====================================================
extern float groundElevationFt;
extern volatile bool groundElevationValid;
extern AirspaceResult nearestAirspace;
extern volatile bool airspaceResultValid;

extern bool bmpOK;
extern float currentTempC;

extern float estimatedWindSpeedKph;
extern float estimatedAirspeedKph;
extern float estimatedWindDirectionDeg;
extern bool windEstimateValid;

extern TinyGPSPlus gps;

extern SemaphoreHandle_t backgroundDataMutex;
extern DynamicJsonDocument adsbDoc;
extern bool conflictDetectedThisFrame;
extern volatile bool hasAdsbData;

extern float MY_LAT;
extern float MY_LON;

extern WindMeter localMeters[TRACKED_METERS];
extern volatile bool hasWeatherData;

extern int windowCount;
extern float currentAltitudeM;
extern float currentClimbRateMS;
extern float currentQNH;
extern bool qnhCalibrated;

extern uint8_t batteryPercent;
extern bool battInitialized;
extern bool clockSynced;

// =====================================================
// Helper functions defined elsewhere in the .ino that the draw code calls
// but does not own.
// =====================================================
float getDistanceKM(float lat1, float lon1, float lat2, float lon2);
float getBearing(float lat1, float lon1, float lat2, float lon2);
float deg2rad(float deg);
const char* getCompassDirection(float heading);

// =====================================================
// Draw functions (implemented in DrawPages.cpp).
// =====================================================
void drawTopBar();
void drawParagliderPage();
void drawWeatherPage();
void drawADSBPage();
void drawParamotorPage();
void drawDashboard();
void drawAirspaceWarning();

// Shared drawing helpers.
void drawLargestBoldCentered(int centerX, int baselineY, int maxWidth, const char* text);
void drawLargestBold(int x, int baselineY, int maxWidth, int maxHeight, const char* text);
void drawLargeValueWithSmallUnit(int centerX, int baselineY, int maxWidth, const char* value, const char* unit);
void drawGliderHeadingArrow(int cx, int cy, int arrowRadius, float headingDeg);

// Airspace proximity lookup -- lives alongside drawAirspaceWarning() since
// it's the box that consumes it, but is also called from elsewhere (see
// the original forward-declaration list in the .ino).
bool getAirspaceSnapshot(AirspaceResult& out);

#endif  // DRAWPAGES_H
