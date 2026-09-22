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
#include "ble_manager.h"     // engineDataValid/engineRpm/engineEgtC/engineChtC for drawParamotorPage()
#include "Fanet.h"           // FanetAddress -- FanetContact below
#include "FanetMessaging.h"  // lastFanetMessage, FANET_MESSAGE_BANNER_MS -- drawFanetMessageBanner()

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

// Live FANET traffic table -- see FanetContact below. Sized well above
// what a single-radio FANET receiver realistically hears at once (FANET
// is a short-range, low-power protocol); if this ever proves tight in
// practice, bump it up rather than treating the number as load-bearing.
#define MAX_FANET_CONTACTS 16

// A contact this quiet is treated as stale and drops off the radar page
// -- roughly 6 missed beacons at the default 5s interval
// (FANET_BEACON_INTERVAL_MS, main .ino), enough margin that one or two
// dropped packets over a marginal RF link don't make a still-present
// aircraft flicker on and off.
#define FANET_CONTACT_TIMEOUT_MS 30000UL

// Live FANET weather-station table -- see FanetWeatherStation below.
// Ground stations are far less numerous than aircraft in range at once,
// so this is sized smaller than MAX_FANET_CONTACTS.
#define MAX_FANET_WEATHER_STATIONS 8

// FANET Service (weather) packets recommend a 40s broadcast interval
// (see the protocol reference in Fanet.h) -- this allows for several
// missed beacons before a station is treated as stale and dropped from
// the Weather page.
#define FANET_WEATHER_TIMEOUT_MS 180000UL

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

// One tracked FANET aircraft, updated in place by onFanetTracking() (main
// .ino) each time a Type-1 tracking beacon arrives from that address, and
// read by drawADSBPage() (DrawPages.cpp) to plot it alongside ADS-B
// traffic. No mutex needed around fanetContacts[] below -- unlike
// adsbDoc/localMeters[] (written by backgroundTask() on Core 0, read from
// Core 1), onFanetTracking() runs inside fanet.update() and drawADSBPage()
// runs inside drawDashboard(), and both are only ever called from loop()
// on Core 1 -- so the two can never actually run concurrently.
struct FanetContact {
  bool valid;
  FanetAddress addr;
  float lat;
  float lon;
  int32_t altitudeM;
  float speedKmh;
  float climbMs;
  float headingDeg;
  uint8_t aircraftType;
  float rssi;
  float snr;
  unsigned long lastSeenMs;
};

// One tracked FANET weather station, updated in place by onFanetWeather()
// (main .ino) each time a Service (type 4) beacon with wind data arrives
// from that address. Same no-mutex reasoning as FanetContact above --
// onFanetWeather() and drawWeatherPage() are both only ever called from
// loop() on Core 1.
struct FanetWeatherStation {
  bool valid;
  FanetAddress addr;
  float lat;
  float lon;
  float windHeadingDeg;
  float windSpeedKmh;
  float windGustKmh;
  bool hasTemperature;
  float temperatureC;
  unsigned long lastSeenMs;
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

extern AirspaceResult nearestAirspaceInfo;
extern volatile bool airspaceInfoResultValid;

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

extern FanetContact fanetContacts[MAX_FANET_CONTACTS];
extern FanetWeatherStation fanetWeatherStations[MAX_FANET_WEATHER_STATIONS];

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
// Returns true if it actually drew a banner (see its header comment in
// DrawPages.cpp for why drawDashboard() needs to know that).
bool drawAirspaceWarning();

// FANET message banner -- shares the same top-of-screen slot as
// drawAirspaceWarning() (drawDashboard() only calls this when that one
// didn't draw anything). Shows the most recently received FANET Message
// packet (see FanetMessaging.h) for FANET_MESSAGE_BANNER_MS, then goes
// quiet on its own. All pages, unlike the ADS-B-only airspace info bar.
void drawFanetMessageBanner();

// Shared drawing helpers.
void drawLargestBoldCentered(int centerX, int baselineY, int maxWidth, const char* text);
void drawLargestBold(int x, int baselineY, int maxWidth, int maxHeight, const char* text);
void drawLargeValueWithSmallUnit(int centerX, int baselineY, int maxWidth, const char* value, const char* unit);
void drawGliderHeadingArrow(int cx, int cy, int arrowRadius, float headingDeg);

// Airspace proximity lookup -- lives alongside drawAirspaceWarning() since
// it's the box that consumes it, but is also called from elsewhere (see
// the original forward-declaration list in the .ino).
bool getAirspaceSnapshot(AirspaceResult& out);

// alertOnly = false counterpart of the above -- can return a CFZ. Feeds
// only drawAirspaceInfoBar() below.
bool getAirspaceInfoSnapshot(AirspaceResult& out);

// "Airspace Info" bar -- ADS-B page only (Config > ADS-B Settings >
// Airspace Info, settings.h's airspaceInfoBarEnabled). Shows whichever
// charted airspace (controlled, MBZ, or CFZ) the pilot is currently
// inside, name and recommended frequency included, or "Class G" if
// they're not inside anything charted right now. Deliberately separate
// from drawAirspaceWarning() -- it must never trigger the alert tone,
// and it's meant to always show something rather than only near/inside
// alert-eligible airspace. Drawn at the bottom of the screen, called
// only from drawADSBPage().
void drawAirspaceInfoBar();

#endif  // DRAWPAGES_H
