// v5 STABLE OPTIMISED: based on known-good v4; ADS-B remains loop-driven; no RTOS ADS-B task/mutex.
#include <Arduino.h>
#include <time.h>
#include <U8g2lib.h>
#include <TinyGPS++.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP5xx.h>
#include <Adafruit_SHTC3.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClient.h>
#include <WiFiClientSecure.h>
#include <ArduinoJson.h>
#include <driver/i2s.h>
#include <math.h>
#include <string.h>
#include <esp_task_wdt.h>
#include <esp_timer.h>
#include <SD.h>  
#include "secrets.h"
#include "OpenAirScanner.h"
// =====================================================
// WIFI (feeds Weather + ADS-B pages)
// =====================================================
//#define WIFI_SSID "Your SSID goes here" uncomment to set SSID
//#define WIFI_PASSWORD "Your wifi password goes here" uncomment to set password
#define WIFI_CONNECT_TIMEOUT_MS 10000  // give up after this long in setup()
#define WIFI_RETRY_MS 50000            // how often loop() retries a dropped connection
unsigned long lastWifiRetry = 0;
uint8_t wifiRetryCount = 0;
constexpr uint8_t MAX_WIFI_RETRIES = 10;
bool wifiDisabledUntilReboot = false;
// =====================================================
// ESP32-S3-RLCD-4.2 DISPLAY
// =====================================================
// R0 is 90 degrees anticlockwise from the current R1 landscape layout,
// giving a 300px wide x 400px tall portrait canvas.
#define RLCD_SCK 11
#define RLCD_MOSI 12
#define RLCD_DC 5
#define RLCD_CS 40
#define RLCD_RST 41
#define SCREEN_W 300
#define SCREEN_H 400
// 8mm converted to pixels: this panel doesn't have a published active-area
// mm figure, so this is derived from the stated 4.2" diagonal at 300x400
// (diag = sqrt(300^2+400^2) = 500px = 106.68mm -> ~4.687 px/mm -> 8mm ~= 37.5px).
// Close enough for layout purposes, but if the exact 8mm matters, verify
// against the physical panel with calipers rather than trust this alone.
#define TOP_BAR_HEIGHT_PX 38
U8G2_ST7305_300X400_1_4W_HW_SPI u8g2(U8G2_R0, /*cs=*/RLCD_CS, /*dc=*/RLCD_DC, /*reset=*/RLCD_RST);


// =====================================================
// SD CARD / IGC FLIGHT LOG
// =====================================================

#define SD_CS_PIN 1    
#define SD_SCK_PIN  38
#define SD_MISO_PIN 39
#define SD_MOSI_PIN 21
// Your display already owns the default SPI bus (SCK 11 / MOSI 12 -- see
// SPI.begin() in setup()). These SD pins are completely different, so the
// SD card needs its own SPI peripheral instance rather than sharing that bus.
SPIClass sdSPI(HSPI);

bool sdCardOK = false;
File igcFile;
bool igcRecording = false;
char igcFilename[32] = "";

// Guards ALL SD card access. The card is shared between two cores now:
// the IGC logger (writeIgcBRecord() etc, called from loop() on Core 1) and
// the airspace scanner (called from backgroundTask() on Core 0). Without
// this, a scan and a log write could hit the SPI bus at the same moment
// from two different tasks -- worst case, a corrupted IGC file. Both sides
// must take this before touching SD and give it back immediately after.
SemaphoreHandle_t sdMutex = nullptr;

// =====================================================
// AIRSPACE PROXIMITY (OpenAir file on SD card)
// =====================================================
// Nearest controlled airspace is scanned periodically on the existing
// Core 0 background task (see backgroundTask()) so a multi-hundred-KB SD
// read can never stall GPS/vario/button handling on Core 1. Result is
// written back under backgroundDataMutex, same pattern as the ADS-B
// aircraft list, and copied out under that same lock by the draw code.
static const char* AIRSPACE_FILE = "/AIRSPACE.TXT";
static const char* AIRSPACE_CONTROLLED_CLASSES[] = { "A", "B", "C", "D", "CTR" };
static const uint8_t AIRSPACE_NUM_CONTROLLED_CLASSES = 5;

// Ground elevation (ft MSL), used both to resolve AGL-referenced airspace
// floors and to compute the "ALTITUDE AGL" box on the paraglider page.
// GPS/baro altitude is height above sea level, not height above terrain --
// this can't be derived automatically. Set it for wherever you're flying;
// this is a single fixed site value, not a live terrain lookup.
float groundElevationFt = 0.0f;

#define AIRSPACE_SCAN_INTERVAL_MS 10000UL
unsigned long airspaceScanAnchor = 0;

AirspaceResult nearestAirspace;
volatile bool airspaceResultValid = false;

// Distance thresholds for the on-screen warning overlay (see
// drawAirspaceWarning()) -- tune to taste.
#define AIRSPACE_WARN_HORIZ_KM 5.0f
#define AIRSPACE_WARN_VERT_FT 2000.0f

// Cross-core position snapshot for backgroundTask() to read. Grouped into
// one struct (rather than individual volatiles like sharedGpsAltitudeFeet)
// because lat+lon+alt need to be read together as one consistent fix --
// TinyGPS++'s own fields aren't safe to read piecemeal from another core
// while gps.encode() is actively updating them in loop().
struct PositionSnapshot {
  double lat = 0, lon = 0;
  float altFt = 0;
  bool valid = false;
};
PositionSnapshot sharedPosition;

#define IGC_START_SPEED_KPH   10.0f
#define IGC_STOP_SPEED_KPH   0.0f
#define IGC_START_SUSTAIN_MS  10000UL
#define IGC_STOP_SUSTAIN_MS  10000UL
#define IGC_FIX_INTERVAL_MS   4000UL

unsigned long igcAboveThresholdSince = 0;
unsigned long igcBelowThresholdSince = 0;
unsigned long lastIgcFixWrite = 0;

// =====================================================
// BMP580 BAROMETER (the actual vario sensor)
// =====================================================
#define I2C_SDA 13
#define I2C_SCL 14
Adafruit_BMP5xx bmp;
bool bmpOK = false;
// BMP58x breakouts use either 0x47 (default) or 0x46 (SDO/address jumper).
// Probe both: the barometer is an external flight sensor, not onboard.
constexpr uint8_t BMP5XX_DEFAULT_I2C_ADDR = 0x47;
constexpr uint8_t BMP5XX_ALT_I2C_ADDR = 0x46;
// =====================================================
// SHTC3 SENSOR (ambient temp/humidity -- also feeds top bar temp)
// =====================================================
Adafruit_SHTC3 shtc3;
bool shtc3OK = false;
float currentTempC = NAN;
#define SHT_SAMPLE_MS 10000
unsigned long lastShtSample = 0;
// =====================================================
// PCF85063 HARDWARE RTC
// =====================================================
#include "PCF85063A.h"
PCF85063A rtc(&Wire);
unsigned long lastRtcPush = 0;
#define PCF85063_I2C_ADDR 0x51
bool rtcOK = false;
// =====================================================
// GPS LC76G
// =====================================================
#define GPS_RX_PIN 44
#define GPS_TX_PIN 43
#define GPS_BAUD 115200
#define WIND_MIN_CIRCLE_SPEED_KPH  15.0f
#define WIND_MAX_CIRCLE_SPEED_KPH  80.0f
#define WIND_MAX_ESTIMATE_KPH      50.0f
// ============================================================
// 360° WIND / AIRSPEED ESTIMATOR
// ============================================================
float estimatedWindSpeedKph = 0.0f;
    float estimatedAirspeedKph = 0.0f;
    float estimatedWindDirectionDeg = 0.0f;
    bool windEstimateValid = false;

    // Circle detection state
    bool windCircleActive = false;
    float windCircleStartTrack = 0.0f;
    float windCircleAccumulatedDeg = 0.0f;
    float windCircleLastTrack = 0.0f;

    // Speed extrema during the circle
    float windCircleMaxSpeedKph = 0.0f;
    float windCircleMinSpeedKph = 999.0f;

    // Track at minimum groundspeed
    float windCircleMinSpeedTrack = 0.0f;

    // GPS validity
    bool windEstimatorInitialized = false;
    TinyGPSPlus gps;
// =====================================================
// ADSB GPS linking
// =====================================================
// Guards adsbDoc (ADS-B) and localMeters[]/hasWeatherData (weather) --
// both are written by backgroundTask() on Core 0 and read by the display
// draw functions on Core 1. Renamed from the earlier ADS-B-only name since
// it now protects both background data sets.
SemaphoreHandle_t backgroundDataMutex = nullptr;
volatile bool adsbTaskRunning = false;
volatile bool adsbNewThreat = false;
TaskHandle_t backgroundTaskHandle = nullptr;
DynamicJsonDocument adsbDoc(10000);  // Single shared ADS-B document; avoids a second JSON copy.
// gps.altitude.feet() is written by loop() (Core 1) via gps.encode() and
// would otherwise be read directly by performADSBUpdate() running on the
// background task (Core 0) -- an unsynchronized cross-core read/write on the
// same TinyGPSPlus object. loop() refreshes this each pass instead, and
// the background task reads only this cached copy. A plain aligned float
// read/write is atomic on ESP32, so no mutex is needed for this single
// scalar handoff.
volatile float sharedGpsAltitudeFeet = 0.0f;
bool conflictDetectedThisFrame = false;
// ---- Intercept alarm: fires once per new intruder, alternates tone ----
#define INTERCEPT_ALARM_DURATION_MS 5000UL
#define INTERCEPT_TONE_HIGH_HZ 600
#define INTERCEPT_TONE_LOW_HZ  400
#define INTERCEPT_TONE_TOGGLE_MS 250UL   // time on each tone before switching
bool interceptAlarmActive = false;
unsigned long interceptAlarmStart = 0;
volatile bool hasAdsbData = false;
float MY_LAT = -41.3268;   // Replace with your target latitude
float MY_LON = 174.8069;   // Replace with your target longitude
const int RADIUS_KM = 30;  // Strictly filtered 30km radius on server side

// A fixed-size, plain-data snapshot of the fields drawADSBPage() actually
// needs. The page copies out of adsbDoc under backgroundDataMutex very
// briefly, then releases the lock before doing any trig/SPI work -- so a
// slow display redraw can never make the Core 0 background task wait on
// the display, and vice versa.
#define MAX_DISPLAYED_AIRCRAFT 24
struct AircraftSnapshot {
  float lat, lon, altFeet, speedKt, headingDeg;
};
float deg2rad(float deg) {
  return deg * PI / 180.0f;
}  // Convert degrees to radians
float rad2deg(float rad) {
  return rad * 180.0f / PI;
}  // Convert radians to degrees
// Calculates the compass bearing from you to the aircraft (0 = North, 90 = East, etc.)
float getBearing(float lat1, float lon1, float lat2, float lon2) {
  float dLon = deg2rad(lon2 - lon1);
  float lat1Rad = deg2rad(lat1);
  float lat2Rad = deg2rad(lat2);

  float y = sin(dLon) * cos(lat2Rad);
  float x = cos(lat1Rad) * sin(lat2Rad) - sin(lat1Rad) * cos(lat2Rad) * cos(dLon);

  float bearing = rad2deg(atan2(y, x));
  if (bearing < 0) bearing += 360.0f;
  return bearing;
}

// Calculates distance in kilometers between two GPS points
float getDistanceKM(float lat1, float lon1, float lat2, float lon2) {
  const float R = 6371.0f;  // Earth's radius in km
  float dLat = deg2rad(lat2 - lat1);
  float dLon = deg2rad(lon2 - lon1);

  float a = sin(dLat / 2) * sin(dLat / 2) + cos(deg2rad(lat1)) * cos(deg2rad(lat2)) * sin(dLon / 2) * sin(dLon / 2);
  float c = 2 * atan2(sqrt(a), sqrt(1 - a));
  return R * c;
}
// Converts a 0-360 degree heading into a text string
const char* getCompassDirection(float heading) {
  if (heading < 0) heading += 360.0f;
  if (heading >= 337.5 || heading < 22.5) return "N";
  if (heading >= 22.5 && heading < 67.5) return "NE";
  if (heading >= 67.5 && heading < 112.5) return "E";
  if (heading >= 112.5 && heading < 157.5) return "SE";
  if (heading >= 157.5 && heading < 202.5) return "S";
  if (heading >= 202.5 && heading < 247.5) return "SW";
  if (heading >= 247.5 && heading < 292.5) return "W";
  if (heading >= 292.5 && heading < 337.5) return "NW";
  return "??";
}
// Add these to your global variables section
unsigned long lastAdsbCheckTime = 0;           // Stores the last time we requested data
const unsigned long ADSB_INTERVAL_MS = 15000;  // Poll the server every 15 seconds

// Add these to your global variables list
#define MAX_TRACKED_THREATS 10
char activeThreatHexes[MAX_TRACKED_THREATS][9] = {};  // Fixed-size ADS-B hex IDs
int activeThreatCount = 0;

// =====================================================
// =====================================================
// Weather variables
// =====================================================
// Add this right next to your weatherTimerAnchor global variable
WiFiClientSecure* globalSecureWeatherClient = nullptr;  // allocated in setup(), after Serial is confirmed alive
bool secureWeatherClientInitialized = false;

struct WindMeter {
  char name[20];
  float distanceKm;
  float speedKph;
  float gustKph;
  float bearingDeg;
  bool valid;
};

// Global array tracking the top 4 closest stations in New Zealand
#define TRACKED_METERS 4
WindMeter localMeters[TRACKED_METERS];
volatile bool hasWeatherData = false;

constexpr unsigned long WEATHER_INTERVAL_MS = 5UL * 60UL * 1000UL;  // Poll weather every 5mins
constexpr unsigned long WEATHER_FIRST_POLL_DELAY_MS = 15UL * 1000UL;    // First poll fires 15s after boot
unsigned long weatherTimerAnchor = 0;  // Fresh, clean background timer
bool weatherFirstPollDone = false;      // True once the initial 10s poll has fired

// Proximity alarm timing parameters
// KEY BUTTON (page cycling) -- GPIO18 on this board's onboard KEY
// button, active low. Confirmed against Waveshare's own docs.
// =====================================================
// Gestures:
//   Menu closed: short press cycles the 3 active pages; double press opens
//                the menu; holding 3s toggles the vario mute.
//   Menu open:   short press moves the selection down (wraps); holding 2s
//                selects the highlighted item and closes the menu.
// =====================================================
#define KEY_PIN 18
#define KEY_DEBOUNCE_MS 10
#define KEY_LONG_PRESS_MS 4000
#define PAGE_BEEP_FREQ 400
#define PAGE_BEEP_MS 700
// A "double press" is two presses with less than this many ms between the
// first release and the second press-down. 50ms is what was asked for, but
// note it's faster than most people can physically double-click (a typical
// double-click is more like 150-400ms) -- raise this if the menu doesn't
// open reliably for you. Every short press is held for up to this long
// before it's actioned (to see whether a second press follows), so this
// value also sets the latency added to ordinary page-cycle/menu-navigate
// presses.
#define MENU_DOUBLE_PRESS_MS 800
#define MENU_SELECT_HOLD_MS 2000
unsigned long pageBeepUntil = 0;  // while set, updateBuzzer() yields the pin to the page-change beep

enum Page { PAGE_PARAGLIDER = 0,
            PAGE_WEATHER,
            PAGE_ADSB,
            PAGE_PARAMOTOR,
            PAGE_COUNT };
Page currentPage = PAGE_PARAGLIDER;
const char* PAGE_NAMES[PAGE_COUNT] = { "GLDR", "WIND", "ADSB", "ENG" };
// Only 3 pages are cycled through with a short press. Slot 0 is the "main"
// page and is swappable between Paraglider and Paramotor from the menu;
// slots 1 and 2 are fixed at Weather and ADS-B.
#define ACTIVE_PAGE_COUNT 3
Page activePages[ACTIVE_PAGE_COUNT] = { PAGE_PARAGLIDER, PAGE_WEATHER, PAGE_ADSB };
uint8_t activePageIndex = 0;  // index into activePages[]; kept in sync with currentPage

// ---- Menu ----
enum MenuItemId {
  MENU_SELECT_PARAGLIDER = 0,
  MENU_SELECT_PARAMOTOR,
  MENU_PLACEHOLDER_1,
  MENU_PLACEHOLDER_2,
  MENU_PLACEHOLDER_3,
  MENU_ITEM_COUNT
};
// Placeholders are stubbed out (no-op) in menuSelectCurrentItem() -- give
// them a real name here and a real action there as you build them out.
const char* MENU_ITEM_NAMES[MENU_ITEM_COUNT] = {
  "Paraglider Page",
  "Paramotor Page",
  "Display Settings",
  "Alarm Settings",
  "Units"
};
bool menuActive = false;
uint8_t menuSelectedIndex = 0;

// Set whenever page/menu state changes; drives an immediate redraw instead
// of waiting for the next 1Hz display tick, so menu navigation feels
// responsive rather than laggy.
bool displayDirty = true;
// =====================================================
// VARIO TONE (onboard I2S speaker via ES8311 codec)
// =====================================================
// This board's audio is an I2S speaker driven through an ES8311 codec
// chip on the I2C bus (address 0x18), not a GPIO piezo buzzer -- tone()/
// noTone() do not apply here at all. Two separate things have to work
// for sound to come out: the I2S peripheral carries the audio *data*,
// and the ES8311 chip (controlled over I2C) must be explicitly woken
// and unmuted or it stays silent by design. See es8311Init() below.
//
// Confirm these four I2S pins against your board's actual schematic
// before flashing -- a wrong pin here fails silently, same as a wrong
// ES8311 register value would.
#define I2S_MCLK 16
#define I2S_BCLK 9
#define I2S_LRCK 45
#define I2S_DOUT 8
#define I2S_PORT I2S_NUM_0
#define AMP_ENABLE_PIN 46

#define I2S_SAMPLE_RATE 16000
// Deeper than a minimal setup: buffers this size (2048 samples total =
// ~128ms at 16kHz) give i2sToneService() room to tolerate loop() jitter
// (e.g. a slow SPI display redraw) without the tone audibly glitching.
// Smaller buffers would need loop() called more often than it safely can.
#define I2S_DMA_BUF_COUNT 8
#define I2S_DMA_BUF_LEN 256
#define I2S_TONE_CHUNK 64  // samples generated per i2sToneService() call
// i2sToneService() now runs from its own esp_timer callback instead of
// being polled from loop(), so it can't be starved by a slow display
// redraw or (formerly) a blocking network call. Period matches exactly
// one chunk's playback time (64 samples / 16000Hz = 4ms) so the DMA
// buffer stays topped up with minimal added latency.
#define AUDIO_SERVICE_INTERVAL_US 4000
esp_timer_handle_t audioServiceTimer = nullptr;

#define ES8311_I2C_ADDR 0x18

bool codecOK = false;   // I2S peripheral configured
bool es8311OK = false;  // ES8311 chip found and initialized over I2C

volatile float toneFrequency = 0.0f;  // 0 = silent
float tonePhase = 0.0f;

bool buzzerMuted = false;

// =====================================================
// BATTERY MONITOR- Variables
// =====================================================
#define BATT_ADC_PIN 4
#define BATT_DIVIDER_RATIO 3.0f
#define BATT_EMPTY_V 2.5f
#define BATT_FULL_V 4.2f
#define BATT_SAMPLE_MS 2000
#define BATT_EMA_ALPHA 0.2f
// =====================================================
// Variov- Variables
// =====================================================
// Tuning
#define SEA_LEVEL_QNH_DEFAULT 1013.25f
#define CLIMB_WINDOW_N 8
#define BARO_SAMPLE_MS 100
#define CLIMB_DEADBAND_MS 0.15f
#define SINK_ALARM_MS -1.0f
#define CLIMB_TONE_MAX_MS 5.0f
#define SINK_RELEASE_MS -1.7f
// Sink alarm
#define SINK_BEEP_INTERVAL_MS 500UL  // gap between sink-alarm tone bursts
#define SINK_BEEP_ON_MS 180UL
#define SINK_TONE_FREQ_HZ 220
// Climb tone frequency range
#define CLIMB_TONE_MIN_HZ 400
#define CLIMB_TONE_MAX_HZ 1100
// Climb pulse timing
#define CLIMB_MIN_GAP_MS 55UL
#define CLIMB_MAX_GAP_MS 500UL
#define CLIMB_MIN_PULSE_MS 45UL
#define CLIMB_MAX_PULSE_MS 190UL
// ============================================================
// VARIO AUDIO STATE
// ============================================================

bool sinkAlarmActive = false;
bool climbAudioActive = false;
unsigned long sinkAlarmStart = 0;
unsigned long climbPulseStart = 0;
bool climbToneOn = false;

float altWindow[CLIMB_WINDOW_N];
unsigned long timeWindow[CLIMB_WINDOW_N];
int windowCount = 0;
int windowIndex = 0;
unsigned long lastBaroSample = 0;
float currentAltitudeM = 0.0f;
float currentClimbRateMS = 0.0f;
float currentQNH = SEA_LEVEL_QNH_DEFAULT;
bool qnhCalibrated = false;
unsigned long lastBeepToggle = 0;
bool beepOn = false;
unsigned long lastSinkBeep = 0;
float batteryVoltage = 0.0f;
uint8_t batteryPercent = 0;
unsigned long lastBattSample = 0;
bool battInitialized = false;

// Placeholder connectivity state -- not yet wired to real WiFi/BT.
// WiFi gets initialized for real when the Weather page is built out
// (item 6); this just gives the top bar something honest to show
// ("--") until then instead of a fabricated status.
volatile bool wifiConnected = false;
bool btConnected = false;

// =====================================================
// TIME & SCHEDULING
// =====================================================
// New Zealand timezone, including daylight saving.
#define NZ_TIMEZONE "NZST-12NZDT,M9.5.0,M4.1.0/3"
unsigned long lastDisplayUpdate = 0;
bool clockSynced = false;

// GPS corrects the clock once after each boot; the system clock and the
// hardware RTC then continue running without repeated GPS writes.
bool gpsClockSyncedThisBoot = false;
// All the Functions are stored below
void drawDashboard();
void drawAirspaceWarning();
bool getAirspaceSnapshot(AirspaceResult& out);
void setupI2sCodec();
void i2sToneService();
void drawTopBar();
void drawMenu();
void drawParagliderPage();
void drawWeatherPage();
void drawADSBPage();
void updateIgcRecorder();
void startIgcRecording();
void stopIgcRecording();
void writeIgcBRecord();
void formatIgcLatLon();
void drawParamotorPage();
void updateVario();
void updateWindEstimator();
void updateI2sAudioBuzzer();
void updateBattery();
bool syncClockFromGPS();
void updatePageButton();
float computeClimbRateLeastSquares();
void es8311WriteReg(uint8_t reg, uint8_t value);
void es8311Init();
void setToneFrequency(float freq);
bool connectWiFi(unsigned long timeoutMs);
void performADSBUpdate();
float getDistanceKM(float lat1, float lon1, float lat2, float lon2);
float getBearing(float lat1, float lon1, float lat2, float lon2);
// ---- Page rotation / menu ----
void advanceActivePage();
void jumpToActivePage(Page page);
void openMenu();
void closeMenu();
void menuMoveDown();
void menuSelectCurrentItem();
void playFeedbackTone(float freq, unsigned long durationMs);
void updateWeather();
// ---- Core 0 background task: Wi-Fi reconnect, ADS-B poll, weather poll ----
void backgroundTask(void* parameter);

// Bare I2C address probe -- bounded by Wire.setTimeOut(), so it can never
// hang even if nothing responds. Used to skip calling into a sensor
// library's begin()/readTime() at all when the device isn't physically
// present, rather than trusting every third-party library to handle
// "device absent" gracefully internally.
bool i2cDevicePresent(uint8_t addr) {
  Wire.beginTransmission(addr);
  return (Wire.endTransmission() == 0);
}
// Arduino-ESP32's supplied C library does not expose timegm(). Convert a
// validated UTC calendar time to Unix time without consulting the local TZ.
bool utcTmToEpoch(const struct tm& utc, time_t& epoch) {
  const int year = utc.tm_year + 1900;
  const int month = utc.tm_mon;
  static const uint8_t daysInMonth[] = {
    31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31
  };

  if (year < 1970 || month < 0 || month > 11 || utc.tm_hour < 0 || utc.tm_hour > 23 || utc.tm_min < 0 || utc.tm_min > 59 || utc.tm_sec < 0 || utc.tm_sec > 59) {
    return false;
  }

  const bool leapYear = (year % 4 == 0 && (year % 100 != 0 || year % 400 == 0));
  int maxDay = daysInMonth[month] + (month == 1 && leapYear ? 1 : 0);
  if (utc.tm_mday < 1 || utc.tm_mday > maxDay) {
    return false;
  }

  int64_t days = 0;
  for (int y = 1970; y < year; ++y) {
    days += (y % 4 == 0 && (y % 100 != 0 || y % 400 == 0)) ? 366 : 365;
  }
  for (int m = 0; m < month; ++m) {
    days += daysInMonth[m] + (m == 1 && leapYear ? 1 : 0);
  }
  days += utc.tm_mday - 1;

  const int64_t seconds = days * 86400LL + utc.tm_hour * 3600L + utc.tm_min * 60L + utc.tm_sec;
  const time_t converted = (time_t)seconds;
  if ((int64_t)converted != seconds) {
    return false;  // Timestamp does not fit this core's time_t.
  }

  epoch = converted;
  return true;
}
void setup() {
  Serial.begin(115200);
  delay(2000);  // give the USB CDC host a moment to attach before the first print, or it's often lost
  Serial.println("BOOTING FLIGHT COMPUTER...");
  Serial.printf("[BOOT] Free heap: %u | Min heap: %u\n", ESP.getFreeHeap(), ESP.getMinFreeHeap());

  // Heavy objects allocated here instead of as globals, so their
  // construction happens after the boot prints above are already
  // guaranteed to have gone out over serial -- if something about
  // allocating either of these ever goes wrong, you'll see exactly
  // where, instead of silence before setup() even starts.
  globalSecureWeatherClient = new WiFiClientSecure();
  Serial.printf("[BOOT] After adsbDoc/TLS client alloc - Free heap: %u | Min heap: %u\n",
                ESP.getFreeHeap(), ESP.getMinFreeHeap());

  // Attenuation must be set BEFORE the pin is ever read: analogRead()
  // attaches and configures the ADC1 channel on first use, and
  // reconfiguring attenuation on an already-attached channel leaves the
  // oneshot driver's channel handle in a bad state on this core, causing
  // every later analogReadMilliVolts() call in updateBattery() to fail
  // with "invalid channel". So: pinMode, then attenuation, then read.
  pinMode(BATT_ADC_PIN, INPUT);
  // Fire up the speaker power amplifier stage immediately at boot
  pinMode(AMP_ENABLE_PIN, OUTPUT);
  digitalWrite(AMP_ENABLE_PIN, LOW);  // Keep amplifier off during startup.
  analogSetPinAttenuation(BATT_ADC_PIN, ADC_11db);

  int raw = analogRead(BATT_ADC_PIN);
  Serial.print("Raw ADC test read: ");
  Serial.println(raw);

  setenv("TZ", NZ_TIMEZONE, 1);
  tzset();

  Serial1.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  Serial.println("GPS UART INITIALIZED");

  SPI.begin(RLCD_SCK, -1 /*MISO unused*/, RLCD_MOSI, RLCD_CS);
  u8g2.begin();
  Serial.println("DISPLAY INITIALIZED");
  Serial.printf("[BOOT] After display - Free heap: %u | Min heap: %u\n", ESP.getFreeHeap(), ESP.getMinFreeHeap());

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setTimeOut(50);

  // Cheap presence check before touching any library's begin()/readTime():
  // a plain I2C address probe is bounded by Wire.setTimeOut() above, so
  // it can't hang even if a device is fully absent. This protects against
  // library-internal init loops that might not have their own timeout --
  // we simply never call into them for a device that isn't there.
  uint8_t bmpAddress = 0;
  if (i2cDevicePresent(BMP5XX_DEFAULT_I2C_ADDR)) {
    bmpAddress = BMP5XX_DEFAULT_I2C_ADDR;
  } else if (i2cDevicePresent(BMP5XX_ALT_I2C_ADDR)) {
    bmpAddress = BMP5XX_ALT_I2C_ADDR;
  }
  bmpOK = (bmpAddress != 0) && bmp.begin(bmpAddress, &Wire);
  if (bmpOK) {
    bmp.setTemperatureOversampling(BMP5XX_OVERSAMPLING_2X);
    bmp.setPressureOversampling(BMP5XX_OVERSAMPLING_8X);
    bmp.setIIRFilterCoeff(BMP5XX_IIR_FILTER_COEFF_3);
    bmp.setOutputDataRate(BMP5XX_ODR_50_HZ);
    bmp.setPowerMode(BMP5XX_POWERMODE_NORMAL);
    bmp.enablePressure(true);
    Serial.println("BMP580 FOUND -- VARIO ACTIVE");
  } else {
    Serial.println("BMP580 NOT FOUND -- check wiring/address, vario disabled");
  }

  shtc3OK = shtc3.begin();
  Serial.println(shtc3OK ? "SHTC3 TEMPERATURE SENSOR FOUND" : "SHTC3 NOT FOUND");

  // RTC was previously unguarded -- readTime() ran unconditionally with
  // no check the chip was even present. Same presence-check pattern here.
  rtcOK = i2cDevicePresent(PCF85063_I2C_ADDR);
  if (rtcOK) {
    rtc.readTime();
    struct tm rtcTm = {};
    rtcTm.tm_hour = rtc.getHour();
    rtcTm.tm_min = rtc.getMinute();
    rtcTm.tm_sec = rtc.getSecond();
    rtcTm.tm_mday = rtc.getDay();
    rtcTm.tm_mon = rtc.getMonth() - 1;
    rtcTm.tm_year = rtc.getYear() - 1900;

    // The RTC is stored as UTC (see syncClockFromGPS()).
    time_t rtcEpoch;
    if (utcTmToEpoch(rtcTm, rtcEpoch)) {
      struct timeval rtcTv = { .tv_sec = rtcEpoch, .tv_usec = 0 };
      settimeofday(&rtcTv, nullptr);
      clockSynced = true;
      Serial.println("Clock seeded from PCF85063 hardware RTC");
    } else {
      Serial.println("PCF85063 RTC has no valid date -- waiting for GPS");
    }
  } else {
    Serial.println("PCF85063 RTC NOT FOUND -- clock will sync from GPS once it has a fix");
  }
  // ESP32-S3 has no fixed default SDMMC pin set (unlike classic ESP32) --
  // pins must be assigned explicitly before begin().
  sdSPI.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);
  sdCardOK = SD.begin(SD_CS_PIN, sdSPI);
  Serial.println(sdCardOK ? "SD CARD MOUNTED" : "SD CARD NOT FOUND -- IGC recording disabled");

  wifiConnected = connectWiFi(WIFI_CONNECT_TIMEOUT_MS);
  if (wifiConnected) {
    Serial.print("WIFI CONNECTED, IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("WIFI NOT CONNECTED -- will retry in background");
  }
  lastWifiRetry = millis();

  // I2S data path first (no I2C dependency), then the ES8311 chip
  // itself over I2C -- Wire.begin() already ran above, so this is
  // safe here. Order matters: es8311Init() before the chip exists
  // would just fail its presence check.
  Serial.println("[BOOT] Calling setupI2sCodec()...");
  setupI2sCodec();
  Serial.println("[BOOT] setupI2sCodec() returned OK");

  Serial.println("[BOOT] Calling es8311Init()...");
  es8311Init();
  Serial.println("[BOOT] es8311Init() returned OK");

  pinMode(KEY_PIN, INPUT_PULLUP);

  Serial.println("[BOOT] Drawing splash screen...");
  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_fub20_tf);
    u8g2.drawStr(20, 200, "FLIGHT COMPUTER");
  } while (u8g2.nextPage());
  Serial.println("[BOOT] Splash screen drawn, entering 1.5s delay...");
  delay(1500);
  //esp_task_wdt_reset(); // feed the watchdog after the splash delay, before any blocking HTTP work
  Serial.println("[BOOT] Splash delay complete");

  // ---------------------------------------------------------
  // Cross-core plumbing for the background task (Wi-Fi reconnect,
  // ADS-B, weather -- everything not needed for the paraglider page).
  // ---------------------------------------------------------
  backgroundDataMutex = xSemaphoreCreateMutex();
  if (backgroundDataMutex == nullptr) {
    Serial.println("[BOOT] Failed to create backgroundDataMutex -- ADS-B/weather disabled");
  }

  sdMutex = xSemaphoreCreateMutex();
  if (sdMutex == nullptr) {
    Serial.println("[BOOT] Failed to create sdMutex -- IGC logging and airspace scan disabled");
    sdCardOK = false;
  }

  // ---------------------------------------------------------
  // Independent audio-servicing timer. i2sToneService() no longer runs
  // from loop() -- it's called on a fixed 4ms cadence regardless of what
  // either core is doing, so a slow display redraw or (now relocated)
  // network call can never starve the DMA buffer and cause the tone to
  // glitch/cut out.
  // ---------------------------------------------------------
  const esp_timer_create_args_t audioTimerConfig = {
    .callback = [](void*) { i2sToneService(); },
    .arg = nullptr,
    .dispatch_method = ESP_TIMER_TASK,
    .name = "audio_svc"
  };
  esp_err_t timerErr = esp_timer_create(&audioTimerConfig, &audioServiceTimer);
  if (timerErr == ESP_OK) {
    esp_timer_start_periodic(audioServiceTimer, AUDIO_SERVICE_INTERVAL_US);
    Serial.println("[BOOT] Audio service timer started");
  } else {
    Serial.printf("[BOOT] Failed to create audio service timer, err=%d\n", timerErr);
  }

  // ---------------------------------------------------------
  // Background task: Wi-Fi reconnect, ADS-B polling, weather polling.
  // Pinned to Core 0, away from loop() on Core 1, so none of this can
  // delay GPS/vario/audio/display/buttons. weatherTimerAnchor is left at
  // its default (0), so the task's first pass fetches weather almost
  // immediately rather than setup() blocking on it before loop() starts.
  // ---------------------------------------------------------
  BaseType_t taskCreated = xTaskCreatePinnedToCore(
    backgroundTask,
    "BackgroundTask",
    12288,  // stack (bytes) -- TLS handshake + JSON parsing need real headroom
    nullptr,
    1,  // low priority -- this only needs to run every few seconds
    &backgroundTaskHandle,
    0  // pin to Core 0
  );
  if (taskCreated != pdPASS) {
    Serial.println("[BOOT] Failed to create background task -- ADS-B/weather disabled");
    backgroundTaskHandle = nullptr;
  } else {
    Serial.println("[BOOT] Background task created and pinned to Core 0");
  }

  Serial.println("[BOOT] setup() COMPLETE -- entering loop()");
}
void loop() {
  uint32_t now = millis();
  // ---------------------------------------------------------
  // 1. GPS - drain serial continuously
  // ---------------------------------------------------------
  while (Serial1.available() > 0) {
    gps.encode(Serial1.read());
  }
  updateWindEstimator();
  // ---------------------------------------------------------
  // 2. GPS clock synchronisation - once per boot
  // ---------------------------------------------------------
  sharedGpsAltitudeFeet = gps.altitude.feet();  // cache for backgroundTask() (Core 0) to read safely
  if (!gpsClockSyncedThisBoot && gps.date.isValid() && gps.time.isValid() && gps.date.age() < 2000 && gps.time.age() < 2000 && syncClockFromGPS()) {
    gpsClockSyncedThisBoot = true;
  }
  // Publish lat/lon/altitude together as one snapshot for the airspace
  // scan on Core 0 -- see PositionSnapshot declaration. Short timeout so a
  // missed update just waits for next pass (<50ms away) rather than
  // stalling loop(); the scanner only reads this every AIRSPACE_SCAN_INTERVAL_MS.
  if (backgroundDataMutex != nullptr && xSemaphoreTake(backgroundDataMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
    sharedPosition.lat = gps.location.lat();
    sharedPosition.lon = gps.location.lng();
    // Prefer the QNH-calibrated baro altitude (more precise once
    // calibrated); fall back to raw GPS altitude before calibration.
    sharedPosition.altFt = qnhCalibrated ? (currentAltitudeM * 3.28084f) : gps.altitude.feet();
    sharedPosition.valid = gps.location.isValid() && gps.location.age() < 2000;
    xSemaphoreGive(backgroundDataMutex);
  }
  // ---------------------------------------------------------
  // 3. Flight instrumentation / high priority
  // ---------------------------------------------------------
  if (bmpOK && now - lastBaroSample >= BARO_SAMPLE_MS) {
    lastBaroSample = now;
    updateVario();
  }
  // ---------------------------------------------------------
  // 4. User input
  // ---------------------------------------------------------
  updatePageButton();
  // ---------------------------------------------------------
  // 4.5 ADS-B new intruder: jump to the traffic page and start
  //     the 5s intercept alarm. adsbNewThreat is set on Core 0
  //     by performADSBUpdate(); a plain bool is atomic on ESP32,
  //     so no mutex is needed to read/clear it here.
  // ---------------------------------------------------------
  if (adsbNewThreat) {
    adsbNewThreat = false;
    jumpToActivePage(PAGE_ADSB);
    interceptAlarmActive = true;
    interceptAlarmStart = millis();
  }
  // ---------------------------------------------------------
  // 5. Audio state machine -- decides frequency/pulse pattern only.
  //    Actual sample generation (i2sToneService) runs on its own
  //    independent timer now, not here -- see setup().
  // ---------------------------------------------------------
  updateI2sAudioBuzzer();
  // ---------------------------------------------------------
  // 6. Battery
  // ---------------------------------------------------------
  if (now - lastBattSample >= BATT_SAMPLE_MS) {
    lastBattSample = now;
    updateBattery();
  }

  // ---------------------------------------------------------
  // 7. SHTC3
  // ---------------------------------------------------------
  if (shtc3OK && now - lastShtSample >= SHT_SAMPLE_MS) {
    lastShtSample = now;

    sensors_event_t humidity, temperature;

    if (shtc3.getEvent(&humidity, &temperature)) {
      currentTempC = temperature.temperature;
    }
  }
  // ---------------------------------------------------------
  // 7.5 IGC FLIGHT RECORDER
  // ---------------------------------------------------------
  updateIgcRecorder();
  // ---------------------------------------------------------
  // 8. Display - 1 Hz, or immediately when something changed (page
  //     cycle, menu open/navigate/select) so the menu feels responsive.
  // ---------------------------------------------------------
  if (displayDirty || now - lastDisplayUpdate >= 1000) {
    lastDisplayUpdate = now;
    displayDirty = false;
    drawDashboard();
  }
}
// =====================================================
// PAGE / MENU HELPERS
// =====================================================

// Sets up a short non-blocking tone burst for UI feedback (page change,
// menu open/navigate/select). Actual sample generation happens on the
// independent audio-servicing timer (see setup()), which starts within
// AUDIO_SERVICE_INTERVAL_US regardless -- no need to pump it manually here,
// and doing so would race with the timer callback over tonePhase. Respects
// the mute setting.
void playFeedbackTone(float freq, unsigned long durationMs) {
  if (buzzerMuted) return;
  setToneFrequency(freq);
  digitalWrite(AMP_ENABLE_PIN, HIGH);
  pageBeepUntil = millis() + durationMs;
}
void advanceActivePage() {
  activePageIndex = (activePageIndex + 1) % ACTIVE_PAGE_COUNT;
  currentPage = activePages[activePageIndex];
  displayDirty = true;
  playFeedbackTone(PAGE_BEEP_FREQ, PAGE_BEEP_MS);
}
// Jumps straight to a page if it's currently one of the 3 active slots
// (used by the ADS-B intercept alert to force-switch to the traffic page),
// keeping activePageIndex in sync so short-press cycling continues
// correctly afterward. Does nothing if the page isn't currently active.
void jumpToActivePage(Page page) {
  for (uint8_t i = 0; i < ACTIVE_PAGE_COUNT; i++) {
    if (activePages[i] == page) {
      activePageIndex = i;
      currentPage = page;
      displayDirty = true;
      return;
    }
  }
}
void openMenu() {
  menuActive = true;
  menuSelectedIndex = 0;
  displayDirty = true;
  playFeedbackTone(700.0f, 90);
}
void closeMenu() {
  menuActive = false;
  displayDirty = true;
}
void menuMoveDown() {
  menuSelectedIndex = (menuSelectedIndex + 1) % MENU_ITEM_COUNT;
  displayDirty = true;
  playFeedbackTone(500.0f, 40);
}
void menuSelectCurrentItem() {
  switch (menuSelectedIndex) {
    case MENU_SELECT_PARAGLIDER:
      activePages[0] = PAGE_PARAGLIDER;
      break;
    case MENU_SELECT_PARAMOTOR:
      activePages[0] = PAGE_PARAMOTOR;
      break;
    default:
      // Placeholder items -- wire up real behaviour here as you add it.
      break;
  }
  activePageIndex = 0;
  currentPage = activePages[0];
  playFeedbackTone(1100.0f, 120);
  closeMenu();  // also marks the display dirty
}
// =====================================================
// PAGE BUTTON: drives page cycling, the vario mute hold, and the on-screen
// menu (double press to open; short press to navigate; 2s hold to select).
// See the gesture summary in the KEY BUTTON section above.
// =====================================================
void updatePageButton() {
  static bool lastReading = HIGH;
  static bool stableState = HIGH;
  static unsigned long lastDebounceTime = 0;
  static unsigned long pressStartedAt = 0;
  static unsigned long lastReleaseAt = 0;
  static bool longPressHandled = false;
  static bool awaitingSecondPress = false;  // true after a short release, until the double-press window closes

  unsigned long now = millis();
  bool reading = digitalRead(KEY_PIN);
  if (reading != lastReading) {
    lastDebounceTime = now;
  }

  if (now - lastDebounceTime > KEY_DEBOUNCE_MS) {
    if (reading != stableState) {
      stableState = reading;

      if (stableState == LOW) {  // active-low press
        if (awaitingSecondPress && (now - lastReleaseAt) < MENU_DOUBLE_PRESS_MS) {
          // Second press landed inside the double-press window.
          awaitingSecondPress = false;
          longPressHandled = true;  // this press's own release does nothing
          if (!menuActive) {
            openMenu();
          }
        } else {
          longPressHandled = false;
        }
        pressStartedAt = now;
      } else if (!longPressHandled) {  // released after a short press
        // Could be a lone short press, or the first half of a double
        // press -- don't act yet, wait out the double-press window
        // in case another press follows.
        lastReleaseAt = now;
        awaitingSecondPress = true;
      }
    }
  }

  // Double-press window closed with no second press: resolve the
  // pending release as an ordinary short press.
  if (awaitingSecondPress && stableState == HIGH && now - lastReleaseAt >= MENU_DOUBLE_PRESS_MS) {
    awaitingSecondPress = false;
    if (menuActive) {
      menuMoveDown();
    } else {
      advanceActivePage();
    }
  }

  // Long-press handling while the button is still held down.
  if (stableState == LOW && !longPressHandled) {
    unsigned long heldFor = now - pressStartedAt;
    if (menuActive) {
      if (heldFor >= MENU_SELECT_HOLD_MS) {
        longPressHandled = true;
        awaitingSecondPress = false;
        menuSelectCurrentItem();
      }
    } else if (heldFor >= KEY_LONG_PRESS_MS) {
      longPressHandled = true;
      awaitingSecondPress = false;
      buzzerMuted = !buzzerMuted;
      pageBeepUntil = 0;
      setToneFrequency(0);
      beepOn = false;
      Serial.println(buzzerMuted ? "VARIO BUZZER MUTED" : "VARIO BUZZER UNMUTED");
    }
  }

  lastReading = reading;
}
// =====================================================
// ADSB data handling
// =====================================================
void performADSBUpdate() {

  Serial.printf("[ADS-B] performADSBUpdate() starting on core %d, free heap: %u bytes\n",
                xPortGetCoreID(), ESP.getFreeHeap());
  if (WiFi.status() != WL_CONNECTED) {
    return;
  }

  HTTPClient http;
  WiFiClientSecure client;

  client.setInsecure();

  String domain = "https://opendata.adsb.fi";
  String apiPath = "/api/v3/lat/";

  String url =
    domain + apiPath + String(MY_LAT, 4) + "/lon/" + String(MY_LON, 4) + "/dist/" + String(RADIUS_KM);

  Serial.print("[ADS-B] Connecting to: ");
  Serial.println(url);

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

  // Parse directly from the HTTP stream to avoid allocating a second
  // String containing the complete JSON response.
  WiFiClient* adsbStream = http.getStreamPtr();

  // ---------------------------------------------------------
  // Parse into the shared document
  // ---------------------------------------------------------
  // This now runs on the Core 0 background task while drawADSBPage() reads
  // adsbDoc from Core 1 -- lock around the write, released again before we
  // return. Network I/O above already completed, so the lock is only held
  // for parsing + in-memory processing, never for anything that blocks on
  // the network.
  if (backgroundDataMutex != nullptr) {
    xSemaphoreTake(backgroundDataMutex, portMAX_DELAY);
  }

  adsbDoc.clear();

  DeserializationError error =
    deserializeJson(adsbDoc, *adsbStream);

  http.end();

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

    if (!ac.containsKey("gs") || (float)ac["gs"] < 10.0f) {

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

  float myAltitudeFeet = sharedGpsAltitudeFeet;

  char currentFrameThreatHexes[MAX_TRACKED_THREATS][9] = {};
  int currentFrameThreatCount = 0;

  for (JsonObject ac : aircraftList) {

    if (!ac.containsKey("lat") || !ac.containsKey("lon") || !ac.containsKey("hex")) {

      continue;
    }

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
        MY_LAT,
        MY_LON,
        acLat,
        acLon);

    float verticalDeltaFeet =
      fabsf(
        (float)ac["alt_baro"] - myAltitudeFeet);

    if (distanceKM <= 5.0f && verticalDeltaFeet <= 2000.0f) {

      if (currentFrameThreatCount < MAX_TRACKED_THREATS) {

        strncpy(currentFrameThreatHexes[currentFrameThreatCount],
                acHex,
                sizeof(currentFrameThreatHexes[0]) - 1);
        currentFrameThreatHexes[currentFrameThreatCount]
                               [sizeof(currentFrameThreatHexes[0]) - 1] = '\\0';

        currentFrameThreatCount++;
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
    activeThreatHexes[i][sizeof(activeThreatHexes[0]) - 1] = '\\0';
  }

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
// =====================================================
// BACKGROUND TASK (Core 0): everything that isn't needed for the
// paraglider page runs here, deliberately kept off Core 1 so it can never
// delay GPS/vario/audio/display/buttons. Checks every 500ms whether it's
// time to do any of its interval-gated work, then sleeps -- so it costs
// essentially nothing between polls.
// =====================================================
void backgroundTask(void* parameter) {

  for (;;) {

    uint32_t now = millis();

    // ---------------------------------------------------------
    // Wi-Fi monitoring / reconnect
    // ---------------------------------------------------------
    if (!wifiConnected && wifiRetryCount < MAX_WIFI_RETRIES && now - lastWifiRetry >= WIFI_RETRY_MS) {

      lastWifiRetry = now;

      if (WiFi.status() == WL_CONNECTED) {

        wifiConnected = true;
        wifiRetryCount = 0;

        Serial.println("WIFI RECONNECTED");

      } else {

        wifiRetryCount++;

        WiFi.disconnect();
        WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

        Serial.print("WIFI RETRY ");
        Serial.print(wifiRetryCount);
        Serial.print("/");
        Serial.println(MAX_WIFI_RETRIES);

        if (wifiRetryCount >= MAX_WIFI_RETRIES) {
          WiFi.disconnect(true);
          WiFi.mode(WIFI_OFF);

          Serial.println("WIFI DISABLED UNTIL REBOOT");
        }
      }
    }

    // Detect Wi-Fi drop
    if (wifiConnected && WiFi.status() != WL_CONNECTED) {

      wifiConnected = false;

      Serial.println("WIFI DROPPED");

      wifiRetryCount = 0;
    }

    // ---------------------------------------------------------
    // Network state machines
    // ---------------------------------------------------------
    if (wifiConnected) {

      // ADS-B: only poll every ADSB_INTERVAL_MS
      if (now - lastAdsbCheckTime >= ADSB_INTERVAL_MS) {
        lastAdsbCheckTime = now;
        adsbTaskRunning = true;
        performADSBUpdate();
        adsbTaskRunning = false;
      }
          // Weather: first poll fires WEATHER_FIRST_POLL_DELAY_MS after boot;
          // every poll after that reverts to the normal WEATHER_INTERVAL_MS cadence.
          unsigned long weatherDueInterval = weatherFirstPollDone ? WEATHER_INTERVAL_MS : WEATHER_FIRST_POLL_DELAY_MS;

          if (now - weatherTimerAnchor >= weatherDueInterval) {
          weatherTimerAnchor = now;
          weatherFirstPollDone = true;
          Serial.println("[MAIN] Calling weather update...");
          updateWeather();
          }
    }

    // ---------------------------------------------------------
    // Airspace proximity: local SD file only, no WiFi needed, so this
    // runs regardless of wifiConnected state.
    // ---------------------------------------------------------
    if (sdCardOK && now - airspaceScanAnchor >= AIRSPACE_SCAN_INTERVAL_MS) {
      airspaceScanAnchor = now;

      PositionSnapshot pos;
      if (backgroundDataMutex != nullptr && xSemaphoreTake(backgroundDataMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
        pos = sharedPosition;
        xSemaphoreGive(backgroundDataMutex);
      }

      if (pos.valid) {
        if (sdMutex != nullptr && xSemaphoreTake(sdMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
          AirspaceResult scanResult;
          bool found = findNearestControlledAirspace(
              AIRSPACE_FILE, pos.lat, pos.lon, pos.altFt, groundElevationFt,
              scanResult, AIRSPACE_CONTROLLED_CLASSES, AIRSPACE_NUM_CONTROLLED_CLASSES);
          xSemaphoreGive(sdMutex);

          if (backgroundDataMutex != nullptr && xSemaphoreTake(backgroundDataMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
            if (found) {
              nearestAirspace = scanResult;
              airspaceResultValid = true;
            } else {
              airspaceResultValid = false;
            }
            xSemaphoreGive(backgroundDataMutex);
          }
        } else {
          Serial.println("[Airspace] SD busy -- scan skipped this cycle");
        }
      }
    }

    // All real work above is interval-gated (15s / 5min), so this task
    // spends nearly all its time asleep here rather than busy-polling.
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}
//=====================================================
// Weather data handling
//=====================================================
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
  //
  // API returns an ARRAY of station objects.
  // Only retain the fields we actually need.
  // ---------------------------------------------------------
  JsonDocument filter;

  filter[0]["name"] = true;
  filter[0]["isOffline"] = true;
  filter[0]["currentAverage"] = true;
  filter[0]["currentBearing"] = true;
  filter[0]["currentGust"] = true;
  filter[0]["location"]["coordinates"][0] = true;
  filter[0]["location"]["coordinates"][1] = true;

  // ---------------------------------------------------------
  // JSON document
  // ---------------------------------------------------------
  Serial.printf(
    "[Zephyr] Heap before weatherDoc: %u | Min heap: %u\n",
    ESP.getFreeHeap(),
    ESP.getMinFreeHeap());

  DynamicJsonDocument weatherDoc(50000);

  Serial.printf(
    "[Zephyr] Heap after weatherDoc: %u | Min heap: %u\n",
    ESP.getFreeHeap(),
    ESP.getMinFreeHeap());

  // ---------------------------------------------------------
  // Parse complete downloaded response
  // ---------------------------------------------------------
  Serial.println("[Zephyr] Starting JSON deserialize...");

  DeserializationError error = deserializeJson(
    weatherDoc,
    payload,
    DeserializationOption::Filter(filter));

  Serial.println("[Zephyr] JSON deserialize returned");

  // ---------------------------------------------------------
  // JSON error handling
  // ---------------------------------------------------------
  if (error) {

    Serial.printf(
      "[Zephyr] JSON parsing failed: %s\n",
      error.c_str());

    Serial.printf(
      "[Zephyr] JSON document capacity: %u bytes\n",
      weatherDoc.capacity());

    Serial.printf(
      "[Zephyr] JSON document memory usage: %u bytes\n",
      weatherDoc.memoryUsage());

    Serial.printf(
      "[Zephyr] Heap after JSON parse failure: "
      "%u | Min heap: %u\n",
      ESP.getFreeHeap(),
      ESP.getMinFreeHeap());

    hasWeatherData = false;

    // Release HTTP resources before returning.
    http.end();

    Serial.printf(
      "[Zephyr] Weather HTTP connection closed after "
      "parse failure. Free heap: %u | Min heap: %u\n",
      ESP.getFreeHeap(),
      ESP.getMinFreeHeap());

    return;
  }

  // ---------------------------------------------------------
  // Successful JSON parse
  // ---------------------------------------------------------
  Serial.println("[Zephyr] JSON parsed successfully!");

  Serial.printf(
    "[Zephyr] Heap after JSON parse: %u | Min heap: %u\n",
    ESP.getFreeHeap(),
    ESP.getMinFreeHeap());

  // ---------------------------------------------------------
  // Get station array
  // ---------------------------------------------------------
  JsonArray stations = weatherDoc.as<JsonArray>();

  // This now runs on the Core 0 background task while drawWeatherPage()
  // reads localMeters[]/hasWeatherData from Core 1 -- lock around the
  // whole population pass. It's pure in-memory JSON iteration (the slow
  // network fetch and parse already finished above), so holding the lock
  // for its duration is fast and bounded.
  if (backgroundDataMutex != nullptr) {
    xSemaphoreTake(backgroundDataMutex, portMAX_DELAY);
  }

  // ---------------------------------------------------------
  // Clear old station validity flags
  // ---------------------------------------------------------
  for (int i = 0; i < TRACKED_METERS; i++) {
    localMeters[i].valid = false;
  }

  const int totalStations = stations.size();

  Serial.printf(
    "[Zephyr] Processing %d stations...\n",
    totalStations);

  // ---------------------------------------------------------
  // Keep only the closest TRACKED_METERS stations.
  //
  // This avoids allocating a large sorting array.
  // ---------------------------------------------------------
  for (int i = 0; i < totalStations; i++) {

    JsonObject st = stations[i];

    // -----------------------------------------------------
    // Ignore offline stations
    // -----------------------------------------------------
    if (st["isOffline"] == true) {
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
      continue;
    }

    float stLon = coords[0].as<float>();
    float stLat = coords[1].as<float>();

    // -----------------------------------------------------
    // Validate coordinates
    // -----------------------------------------------------
    if (!isfinite(stLat) || !isfinite(stLon)) {
      continue;
    }

    if (stLat == 0.0f || stLon == 0.0f) {
      continue;
    }

    // -----------------------------------------------------
    // Calculate distance from aircraft/user location
    // -----------------------------------------------------
    float distanceKm =
      getDistanceKM(
        MY_LAT,
        MY_LON,
        stLat,
        stLon);

    // -----------------------------------------------------
    // Find insertion position among closest stations
    // -----------------------------------------------------
    int insertAt = -1;

    for (int j = 0; j < TRACKED_METERS; j++) {

      if (!localMeters[j].valid || distanceKm < localMeters[j].distanceKm) {

        insertAt = j;
        break;
      }
    }

    // Station isn't close enough to enter our list.
    if (insertAt < 0) {
      continue;
    }

    // -----------------------------------------------------
    // Shift existing stations down
    // -----------------------------------------------------
    for (
      int j = TRACKED_METERS - 1;
      j > insertAt;
      j--) {
      localMeters[j] = localMeters[j - 1];
    }

    // -----------------------------------------------------
    // Extract station data
    // -----------------------------------------------------
    const char* name =
      st["name"].as<const char*>();

    float averageKph =
      st["currentAverage"].as<float>();

    float gustKph =
      st["currentGust"].as<float>();

    float bearing =
      st["currentBearing"].as<float>();

    // -----------------------------------------------------
    // Copy station name safely
    // -----------------------------------------------------
    strncpy(
      localMeters[insertAt].name,
      name ? name : "ANON",
      sizeof(localMeters[insertAt].name) - 1);

    localMeters[insertAt]
      .name[sizeof(localMeters[insertAt].name) - 1] = '\0';

    // -----------------------------------------------------
    // Store station data
    // -----------------------------------------------------
    localMeters[insertAt].distanceKm =
      distanceKm;

    // Zephyr wind values are km/h.
    localMeters[insertAt].speedKph =
      averageKph;

    localMeters[insertAt].gustKph =
      gustKph;

    localMeters[insertAt].bearingDeg =
      bearing;

    localMeters[insertAt].valid = true;
  }

  // ---------------------------------------------------------
  // Determine whether we have usable weather data
  // ---------------------------------------------------------
  hasWeatherData = false;

  for (int i = 0; i < TRACKED_METERS; i++) {

    if (localMeters[i].valid) {
      hasWeatherData = true;
      break;
    }
  }

  if (backgroundDataMutex != nullptr) {
    xSemaphoreGive(backgroundDataMutex);
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

  bool fixValid = gps.location.isValid() && gps.location.age() < 2000 &&
                  gps.satellites.isValid() && gps.satellites.value() >= 4;

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

  snprintf(igcFilename, sizeof(igcFilename), "/%04d%02d%02d_%02d%02d%02d.IGC",
           utcTm.tm_year + 1900, utcTm.tm_mon + 1, utcTm.tm_mday,
           utcTm.tm_hour, utcTm.tm_min, utcTm.tm_sec);

  if (sdMutex == nullptr || xSemaphoreTake(sdMutex, pdMS_TO_TICKS(200)) != pdTRUE) {
    Serial.println("[IGC] SD busy -- could not start recording this cycle");
    return;
  }

  igcFile = SD.open(igcFilename, FILE_WRITE);
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

  // Already recording.
  if (speedKph < IGC_STOP_SPEED_KPH) {
    if (igcBelowThresholdSince == 0) {
      igcBelowThresholdSince = now;
    } else if (now - igcBelowThresholdSince >= IGC_STOP_SUSTAIN_MS) {
      stopIgcRecording();
      igcBelowThresholdSince = 0;
      return;
    }
  } else {
    igcBelowThresholdSince = 0;
  }

  if (now - lastIgcFixWrite >= IGC_FIX_INTERVAL_MS) {
    lastIgcFixWrite = now;
    writeIgcBRecord();
  }
}
//=====================================================
//VARIO: sample baro, push into regression window, compute climb rate
//=====================================================
void updateVario() {
    if (!bmp.dataReady()) return;
    if (!bmp.performReading()) return;

    bool gpsAltitudeGood =
        gps.altitude.isValid() && gps.altitude.age() < 2000 && gps.satellites.isValid() && gps.satellites.value() >= 6 && gps.hdop.isValid() && gps.hdop.hdop() <= 2.5;

    if (!qnhCalibrated && gpsAltitudeGood) {
        float gpsAltM = gps.altitude.meters();

        float calculatedQNH =
        bmp.pressure / powf(1.0f - (gpsAltM / 44330.0f), 1.0f / 0.1903f);

        if (calculatedQNH >= 850.0f && calculatedQNH <= 1100.0f) {

        currentQNH = calculatedQNH;
        qnhCalibrated = true;

        Serial.print("QNH calibrated from GPS altitude: ");
        Serial.println(currentQNH);
        }
    }

    currentAltitudeM = bmp.readAltitude(currentQNH);

    altWindow[windowIndex] = currentAltitudeM;
    timeWindow[windowIndex] = millis();
    windowIndex = (windowIndex + 1) % CLIMB_WINDOW_N;
    if (windowCount < CLIMB_WINDOW_N) windowCount++;

    if (windowCount >= 3) {
        currentClimbRateMS = computeClimbRateLeastSquares();
    }
}

float computeClimbRateLeastSquares() {
  float sumT = 0, sumA = 0, sumTT = 0, sumTA = 0;
  unsigned long t0 = timeWindow[(windowIndex + CLIMB_WINDOW_N - windowCount) % CLIMB_WINDOW_N];

  for (int i = 0; i < windowCount; i++) {
    int idx = (windowIndex + CLIMB_WINDOW_N - windowCount + i) % CLIMB_WINDOW_N;
    float t = (timeWindow[idx] - t0) / 1000.0f;
    float a = altWindow[idx];
    sumT += t;
    sumA += a;
    sumTT += t * t;
    sumTA += t * a;
  }

  float n = windowCount;
  float denom = (n * sumTT - sumT * sumT);
  if (fabs(denom) < 1e-6f) return 0.0f;

  return (n * sumTA - sumT * sumA) / denom;
}

void updateWindEstimator() {

    // ---------------------------------------------------------
    // Need valid GPS speed and course
    // ---------------------------------------------------------
    if (!gps.speed.isValid() || !gps.course.isValid()) {
        return;
    }

    float groundSpeedKph = gps.speed.kmph();
    float trackDeg = gps.course.deg();

    // Ignore extremely low GPS speeds.
    // Course becomes unreliable when nearly stationary.
    if (groundSpeedKph < 10.0f) {
        return;
    }

    // ---------------------------------------------------------
    // First valid sample
    // ---------------------------------------------------------
    if (!windEstimatorInitialized) {

        windEstimatorInitialized = true;
        windCircleLastTrack = trackDeg;

        return;
    }

    // ---------------------------------------------------------
    // Calculate change in track since previous GPS sample.
    //
    // Handles the 359° -> 0° transition correctly.
    // ---------------------------------------------------------
    float deltaTrack = trackDeg - windCircleLastTrack;

    if (deltaTrack > 180.0f) {
        deltaTrack -= 360.0f;
    }

    if (deltaTrack < -180.0f) {
        deltaTrack += 360.0f;
    }

    // ---------------------------------------------------------
    // Detect beginning of a circle.
    //
    // We start accumulating when the aircraft has moved
    // through a meaningful amount of heading.
    // ---------------------------------------------------------
    if (!windCircleActive) {

        windCircleActive = true;

        windCircleStartTrack = trackDeg;
        windCircleAccumulatedDeg = 0.0f;

        windCircleMaxSpeedKph = groundSpeedKph;
        windCircleMinSpeedKph = groundSpeedKph;

        windCircleMinSpeedTrack = trackDeg;

        windCircleLastTrack = trackDeg;

        return;
    }

    // ---------------------------------------------------------
    // Accumulate absolute turn angle.
    //
    // We don't care whether the pilot turns left or right.
    // ---------------------------------------------------------
    windCircleAccumulatedDeg += fabsf(deltaTrack);

    // ---------------------------------------------------------
    // Record maximum and minimum groundspeed
    // ---------------------------------------------------------
    if (groundSpeedKph > windCircleMaxSpeedKph) {
        windCircleMaxSpeedKph = groundSpeedKph;
    }

    if (groundSpeedKph < windCircleMinSpeedKph) {
        windCircleMinSpeedKph = groundSpeedKph;
        windCircleMinSpeedTrack = trackDeg;
    }

    windCircleLastTrack = trackDeg;

    // ---------------------------------------------------------
    // Have we completed approximately one full circle?
    //
    // Allow 20° tolerance because GPS course samples are not
    // perfectly continuous.
    // ---------------------------------------------------------
    if (windCircleAccumulatedDeg >= 340.0f) {

        // -----------------------------------------------------
        // Calculate wind and airspeed
        // -----------------------------------------------------

        float speedRange =
            windCircleMaxSpeedKph - windCircleMinSpeedKph;

        float windSpeed =
            speedRange / 2.0f;

        float airspeed =
            (windCircleMaxSpeedKph + windCircleMinSpeedKph) / 2.0f;

        // -----------------------------------------------------
        // Basic sanity checks
        // -----------------------------------------------------

        bool valid = true;

        if (windCircleMinSpeedKph < 10.0f) {
            valid = false;
        }

        if (windCircleMaxSpeedKph > 150.0f) {
            valid = false;
        }

        if (airspeed < 15.0f || airspeed > 150.0f) {
            valid = false;
        }

        if (windSpeed < 0.0f || windSpeed > 75.0f) {
            valid = false;
        }

        // -----------------------------------------------------
        // Accept result
        // -----------------------------------------------------
        if (valid) {

            estimatedWindSpeedKph = windSpeed;
            estimatedAirspeedKph = airspeed;

            // Minimum groundspeed occurs when flying most
            // directly INTO the wind.
            //
            // Therefore the wind direction is approximately
            // opposite the aircraft track at minimum GS.
            float windDir =
                windCircleMinSpeedTrack + 180.0f;

            if (windDir >= 360.0f) {
                windDir -= 360.0f;
            }

            estimatedWindDirectionDeg = windDir;

            windEstimateValid = true;
        }

        // -----------------------------------------------------
        // Reset and wait for another circle
        // -----------------------------------------------------

        windCircleActive = false;
        windCircleAccumulatedDeg = 0.0f;
        windCircleMaxSpeedKph = 0.0f;
        windCircleMinSpeedKph = 999.0f;
    }
}
// =====================================================
// BATTERY
// =====================================================
void updateBattery() {
  uint32_t pinMv = analogReadMilliVolts(BATT_ADC_PIN);
  float rawVoltage = (pinMv / 1000.0f) * BATT_DIVIDER_RATIO;

  if (!battInitialized) {
    batteryVoltage = rawVoltage;
    battInitialized = true;
  } else {
    batteryVoltage = BATT_EMA_ALPHA * rawVoltage + (1.0f - BATT_EMA_ALPHA) * batteryVoltage;
  }

  float pct = (batteryVoltage - BATT_EMPTY_V) / (BATT_FULL_V - BATT_EMPTY_V) * 100.0f;
  batteryPercent = (uint8_t)constrain(pct, 0.0f, 100.0f);
}
// =====================================================
// WIFI: non-blocking connect attempt. Called once from setup() with a
// bounded timeout, and periodically from loop() to retry a dropped
// connection -- never blocks loop() the way WiFi.begin()+delay() would.
// =====================================================
bool connectWiFi(unsigned long timeoutMs) {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < timeoutMs) {
    delay(200);  // only used here, during the bounded setup() attempt
  }

  return WiFi.status() == WL_CONNECTED;
}
// =====================================================
// CLOCK
// =====================================================
bool syncClockFromGPS() {
  if (!gps.date.isValid() || !gps.time.isValid()) {
    return false;
  }

  struct tm t = {};

  t.tm_year = gps.date.year() - 1900;
  t.tm_mon = gps.date.month() - 1;
  t.tm_mday = gps.date.day();
  t.tm_hour = gps.time.hour();
  t.tm_min = gps.time.minute();
  t.tm_sec = gps.time.second();
  t.tm_isdst = 0;

  // GPS supplies UTC. Convert it without applying the NZ local timezone.
  time_t utcEpoch;
  if (!utcTmToEpoch(t, utcEpoch)) {
    return false;
  }

  struct timeval tv;
  tv.tv_sec = utcEpoch;
  tv.tv_usec = 0;

  settimeofday(&tv, nullptr);
  clockSynced = true;

  // Calculate weekday from the actual UTC timestamp.
  struct tm utcTm;
  gmtime_r(&utcEpoch, &utcTm);

  if (rtcOK && (lastRtcPush == 0 || millis() - lastRtcPush >= 5UL * 60UL * 1000UL)) {

    // The installed PCF85063A library takes the two-digit RTC year.
    uint8_t rtcYear = (uint8_t)((utcTm.tm_year + 1900) % 100);

    rtc.setTime(
      utcTm.tm_hour,
      utcTm.tm_min,
      utcTm.tm_sec);

    rtc.setDate(
      utcTm.tm_wday,
      utcTm.tm_mday,
      utcTm.tm_mon + 1,
      rtcYear);

    lastRtcPush = millis();
  }

  return true;
}
// =====================================================
// BUZZER: non-blocking climb/sink tone over the I2S speaker.
// Yields to the page-change beep for its short duration rather than
// talking over it -- both share the same physical speaker.
// =====================================================
void updateI2sAudioBuzzer() {

    const unsigned long now = millis();

    // ============================================================
    // PAGE-CHANGE BEEP HAS PRIORITY
    // ============================================================

    if ((int32_t)(pageBeepUntil - now) > 0) {
      return;
    }

    // Page beep has just finished
    if (pageBeepUntil != 0) {
      pageBeepUntil = 0;

      digitalWrite(AMP_ENABLE_PIN, LOW);
      setToneFrequency(0);

      sinkAlarmActive = false;
      climbAudioActive = false;
      climbToneOn = false;
    }
    // ============================================================
    // ADS-B INTERCEPT ALARM: 5s of alternating tone, takes priority
    // over vario climb/sink audio (but not the page-change beep).
    // ============================================================
    if (interceptAlarmActive) {

      unsigned long elapsed = now - interceptAlarmStart;

      if (elapsed >= INTERCEPT_ALARM_DURATION_MS) {
        interceptAlarmActive = false;

        digitalWrite(AMP_ENABLE_PIN, LOW);
        setToneFrequency(0);

        // Force vario audio to re-evaluate cleanly next pass.
        sinkAlarmActive = false;
        climbAudioActive = false;
        climbToneOn = false;

      } else {
        unsigned long phase = elapsed % (INTERCEPT_TONE_TOGGLE_MS * 2);
        float freq = (phase < INTERCEPT_TONE_TOGGLE_MS) ? INTERCEPT_TONE_HIGH_HZ : INTERCEPT_TONE_LOW_HZ;

        digitalWrite(AMP_ENABLE_PIN, HIGH);
        setToneFrequency(freq);

        return;  // Skip vario tone logic entirely while the alarm sounds
      }
    }

    // ============================================================
    // MUTED
    // ============================================================

    if (buzzerMuted) {

      digitalWrite(AMP_ENABLE_PIN, LOW);
      setToneFrequency(0);

      sinkAlarmActive = false;
      climbAudioActive = false;
      climbToneOn = false;

      return;
    }


    // ============================================================
    // SINK ALARM WITH HYSTERESIS
    //
    // Enter sink alarm at <= -2.0 m/s
  // Remain in alarm until climb rate rises above -1.7 m/s
    //
    // This prevents rapid ON/OFF switching when the measured
    // sink rate is hovering around -2.0 m/s.
    // ============================================================

    if (!sinkAlarmActive) {

      if (currentClimbRateMS <= SINK_ALARM_MS) {

        sinkAlarmActive = true;
        sinkAlarmStart = now;

        // Make sure climb audio is cancelled
        climbAudioActive = false;
        climbToneOn = false;
      }

    } else {

      // Hysteresis release
      if (currentClimbRateMS >= SINK_RELEASE_MS) {

        sinkAlarmActive = false;

        digitalWrite(AMP_ENABLE_PIN, LOW);
  setToneFrequency(0);
      }
    }


    // ============================================================
    // SINK ALARM OUTPUT
    // ============================================================

    if (sinkAlarmActive) {

      unsigned long sinkPhase =
        (now - sinkAlarmStart) % SINK_BEEP_INTERVAL_MS;

      if (sinkPhase < SINK_BEEP_ON_MS) {

        digitalWrite(AMP_ENABLE_PIN, HIGH);
        setToneFrequency(SINK_TONE_FREQ_HZ);

      } else {

        digitalWrite(AMP_ENABLE_PIN, LOW);
        setToneFrequency(0);
      }

      return;
    }


    // ============================================================
    // CLIMB DEAD BAND
    //
    // Below +0.15 m/s there is no climb tone.
    // ============================================================

    if (currentClimbRateMS <= CLIMB_DEADBAND_MS) {

      climbAudioActive = false;
      climbToneOn = false;

      digitalWrite(AMP_ENABLE_PIN, LOW);
      setToneFrequency(0);

      return;
    }


    // ============================================================
    // ENTERING CLIMB AUDIO
    // ============================================================

    if (!climbAudioActive) {

      climbAudioActive = true;
      climbToneOn = false;

      // Start the first pulse after a short delay rather than
      // immediately producing a tone.
      climbPulseStart = now;
    }


    // ============================================================
    // NORMALISE CLIMB RATE
    //
    // 0.15 m/s -> 0.0
    // 5.0  m/s -> 1.0
    //
    // Anything above 5 m/s is capped at 1.0.
    // ============================================================

    float factor =
      (currentClimbRateMS - CLIMB_DEADBAND_MS) / (CLIMB_TONE_MAX_MS - CLIMB_DEADBAND_MS);

    factor = constrain(factor, 0.0f, 1.0f);


    // ============================================================
    // NONLINEAR RESPONSE
    //
    // sqrt() gives more audio resolution in weak lift.
    //
    // This is important for a paraglider because the difference
    // between 0.2 and 0.5 m/s is much more useful to the pilot
    // than making 4 and 5 m/s dramatically different.
    // ============================================================

    float response = sqrtf(factor);


    // ============================================================
    // TONE FREQUENCY
    //
    // Approximately:
    //
    // 0.15 m/s -> 400 Hz
    // 0.5  m/s -> ~530 Hz
    // 1.0  m/s -> ~650 Hz
    // 2.0  m/s -> ~790 Hz
    // 3.0  m/s -> ~890 Hz
    // 5.0  m/s -> 1100 Hz
    // ============================================================

    int toneFreq =
      CLIMB_TONE_MIN_HZ + (int)(response * (CLIMB_TONE_MAX_HZ - CLIMB_TONE_MIN_HZ));


    // ============================================================
    // NONLINEAR PULSE TIMING
    //
    // Weak lift:
    //     long gaps
    //
    // Strong lift:
    //     short gaps
    //
    // Using sqrt() here gives a more progressive response.
    // ============================================================

    float pulseResponse = sqrtf(factor);

    unsigned long gapMs =
      CLIMB_MAX_GAP_MS - (unsigned long)(pulseResponse * (CLIMB_MAX_GAP_MS - CLIMB_MIN_GAP_MS));


    // ============================================================
    // PULSE LENGTH
    //
    // Beeps become progressively longer with increasing lift.
    // ============================================================

    unsigned long pulseMs =
      CLIMB_MIN_PULSE_MS + (unsigned long)(response * (CLIMB_MAX_PULSE_MS - CLIMB_MIN_PULSE_MS));


    // ============================================================
    // STRONG-LIFT CONTINUOUS-TONE REGION
    //
    // Above roughly 80% of the configured climb range, the
    // individual pulses become close enough together that a
    // continuous tone is more useful.
    // ============================================================

    if (factor >= 0.80f) {

      climbToneOn = true;

      digitalWrite(AMP_ENABLE_PIN, HIGH);
      setToneFrequency(toneFreq);

      return;
    }


    // ============================================================
    // CLIMB PULSE GENERATOR
    // ============================================================

    if (climbToneOn) {

      // Currently sounding
      if ((now - climbPulseStart) >= pulseMs) {

        climbToneOn = false;
        climbPulseStart = now;

        digitalWrite(AMP_ENABLE_PIN, LOW);
        setToneFrequency(0);
      }

    } else {

      // Currently silent
      if ((now - climbPulseStart) >= gapMs) {

        climbToneOn = true;
        climbPulseStart = now;

        digitalWrite(AMP_ENABLE_PIN, HIGH);
        setToneFrequency(toneFreq);
      }
    }
}
// =====================================================
// I2S CODEC SETUP: configures the ESP32-S3's I2S peripheral using the
// plain Arduino driver/i2s.h API 
// =====================================================
void setupI2sCodec() {
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate = I2S_SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = I2S_DMA_BUF_COUNT,
    .dma_buf_len = I2S_DMA_BUF_LEN,
    .use_apll = false,
    .tx_desc_auto_clear = true  // auto-fills silence on underrun instead of repeating stale samples
  };

  i2s_pin_config_t pin_config = {
    .mck_io_num = I2S_MCLK,  // required on this board -- not BCLK-derived
    .bck_io_num = I2S_BCLK,
    .ws_io_num = I2S_LRCK,
    .data_out_num = I2S_DOUT,
    .data_in_num = I2S_PIN_NO_CHANGE
  };

  esp_err_t err = i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
  if (err != ESP_OK) {
    Serial.print("I2S DRIVER INSTALL FAILED, err=");
    Serial.println(err);
    codecOK = false;
    return;
  }
  i2s_set_pin(I2S_PORT, &pin_config);
  i2s_set_clk(I2S_PORT, I2S_SAMPLE_RATE, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_MONO);
  codecOK = true;
  Serial.println("I2S PERIPHERAL INITIALIZED");
}
// =====================================================
// ES8311 CODEC CONTROL (I2C): wakes and unmutes the codec chip so the
// I2S data stream above actually reaches the speaker. 
// =====================================================
void es8311WriteReg(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(ES8311_I2C_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

void es8311Init() {
  if (!i2cDevicePresent(ES8311_I2C_ADDR)) {
    Serial.println("ES8311 NOT FOUND on I2C bus -- speaker will stay silent");
    es8311OK = false;
    return;
  }

  es8311WriteReg(0x01, 0x30);  // clock manager: power up analog, select clock source
  es8311WriteReg(0x02, 0x00);  // clock manager: clock divider defaults
  es8311WriteReg(0x03, 0x10);  // clock manager: ADC clock divider (unused, DAC-only)
  es8311WriteReg(0x16, 0x24);  // clock manager: DAC clock divider
  es8311WriteReg(0x04, 0x10);  // clock manager: DAC oversampling ratio
  es8311WriteReg(0x05, 0x00);  // clock manager: ADC oversampling ratio (unused)
  es8311WriteReg(0x0B, 0x00);  // system: power management
  es8311WriteReg(0x0C, 0x00);  // system: power management
  es8311WriteReg(0x10, 0x03);  // system: bias/power
  es8311WriteReg(0x11, 0x7F);  // system: bias/power
  es8311WriteReg(0x00, 0x80);  // reset: release reset, normal operation
  es8311WriteReg(0x0D, 0x01);  // system: power up analog
  es8311WriteReg(0x01, 0x3F);  // clock manager: enable all internal clocks
  es8311WriteReg(0x14, 0x1A);  // system: mic/line-in bias (unused, DAC-only)
  es8311WriteReg(0x12, 0x00);  // system: power management
  es8311WriteReg(0x13, 0x10);  // system: power management
  es8311WriteReg(0x0E, 0x02);  // system: power management
  es8311WriteReg(0x0F, 0x44);  // system: power management
  es8311WriteReg(0x15, 0x00);  // ADC: not used in DAC-only mode
  es8311WriteReg(0x37, 0x08);  // ADC: not used in DAC-only mode
  es8311WriteReg(0x09, 0x00);  // SDP: I2S format, 16-bit
  es8311WriteReg(0x18, 0x00);  // DAC: volume-related default

  // Clock dividers for 16kHz sample rate at a 256x (4.096MHz) MCLK ratio,
  // taken directly from Espressif's reference ES8311 driver's coefficient
  // table. These were previously missing/wrong -- 0x08 was mislabeled as
  // a GPIO register in an earlier version of this code; it's actually the
  // LRCK divider's low byte, and 0x06/0x07 weren't being written at all,
  // left at power-on-reset defaults that didn't match this sample rate.
  es8311WriteReg(0x06, 0x03);  // clock manager: BCLK divider
  es8311WriteReg(0x07, 0x00);  // clock manager: LRCK divider (high byte)
  es8311WriteReg(0x08, 0xFF);  // clock manager: LRCK divider (low byte)

  es8311WriteReg(0x32, 0xBF);  // DAC volume: near-max (0xBF of 0xFF range)
  es8311WriteReg(0x31, 0x00);  // DAC: unmute

  Serial.println("ES8311 CODEC INITIALIZED");
  es8311OK = true;
}
// =====================================================
// TONE GENERATION: non-blocking. Unlike a single long i2s_write() call
// (which blocks for the tone's whole duration and would stall GPS/baro/
// display handling), this generates and pushes only a small chunk of
// samples per call, using a zero-timeout write so it only writes as much
// as the DMA buffer currently has room for and never blocks. Called on a
// fixed cadence by the independent audio-servicing timer set up in
// setup() -- see i2sToneService() -- so it can't be starved by loop()
// or the Core 0 background task doing something slow.
// =====================================================
void setToneFrequency(float freq) {
  toneFrequency = freq;
  if (freq <= 0.0f) {
    // Flush any already-queued samples immediately so silence is
    // heard right away, rather than after the buffered tail plays out.
    if (codecOK) {
      i2s_zero_dma_buffer(I2S_PORT);
    }
  }
}
void i2sToneService() {
  if (!codecOK || !es8311OK) return;
  if (toneFrequency <= 0.0f) return;  // tx_desc_auto_clear fills silence on its own

  int16_t chunk[I2S_TONE_CHUNK];
  for (int i = 0; i < I2S_TONE_CHUNK; i++) {
    chunk[i] = (int16_t)(8000.0f * sinf(2.0f * PI * tonePhase));
    tonePhase += toneFrequency / (float)I2S_SAMPLE_RATE;
    if (tonePhase >= 1.0f) tonePhase -= 1.0f;
  }

  size_t bytesWritten = 0;
  // 0 ticks = non-blocking: writes only what currently fits, drops the
  // rest rather than waiting. A dropped chunk here just means the next
  // call tops the buffer back up -- harmless for a beep tone.
  i2s_write(I2S_PORT, chunk, sizeof(chunk), &bytesWritten, 0);
}
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
    time_t now;
    time(&now);

    struct tm localTime;
    localtime_r(&now, &localTime);

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
void drawLargeValueWithSmallUnit(
    int centerX,
    int baselineY,
    int maxWidth,
    const char* value,
    const char* unit){
    // ---------------------------------------------------------
    // Large numeric value
    // ---------------------------------------------------------
    const uint8_t* valueFonts[] = {
        u8g2_font_fub20_tf,
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
        value
    );

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
        unit
    );
}
// =====================================================
// MENU: full-screen list shown instead of the normal top bar + page while
// menuActive is true. The highlighted row is menuSelectedIndex; a short
// press moves it down (wraps), a 2s hold selects it. A trailing "*" marks
// whichever of Paraglider/Paramotor is currently the active main page.
// =====================================================
void drawMenu() {
  u8g2.setFont(u8g2_font_helvB14_tf);
  u8g2.drawStr(10, 30, "MENU");
  u8g2.drawLine(0, 40, SCREEN_W, 40);

  const int top = 40;
  const int rowH = (SCREEN_H - top) / MENU_ITEM_COUNT;

  u8g2.setFont(u8g2_font_helvB12_tf);
  for (int i = 0; i < MENU_ITEM_COUNT; i++) {
    int rowY = top + i * rowH;

    if (i == menuSelectedIndex) {
      u8g2.setDrawColor(1);
      u8g2.drawBox(0, rowY, SCREEN_W, rowH);
      u8g2.setDrawColor(0);  // inverted text on the highlighted row
    }

    bool isActiveMainPage =
      (i == MENU_SELECT_PARAGLIDER && activePages[0] == PAGE_PARAGLIDER) || (i == MENU_SELECT_PARAMOTOR && activePages[0] == PAGE_PARAMOTOR);

    char label[32];
    snprintf(label, sizeof(label), "%s%s", MENU_ITEM_NAMES[i], isActiveMainPage ? " *" : "");
    u8g2.drawStr(14, rowY + rowH / 2 + 5, label);

    u8g2.setDrawColor(1);
  }
}

// =====================================================
// PARAGLIDER PAGE: 6-box grid (2 cols x 3 rows) below the top bar.
// =====================================================

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
        rowH
      );
    }
  }


  // =========================================================
  // BOX (0,0): ALTITUDE
  // =========================================================

  u8g2.setFont(u8g2_font_helvB10_tf);

  u8g2.drawStr(
    5,
    top + 14,
    "ALTITUDE"
  );

  if (bmpOK && windowCount > 0) {

    snprintf(
      buffer,
      sizeof(buffer),
      "%d",
      (int)roundf(currentAltitudeM)
    );

    drawLargeValueWithSmallUnit(
      colW / 2,
      top + rowH / 2 + 10,
      colW - 10,
      buffer,
      "m"
    );

  } else {

    drawLargeValueWithSmallUnit(
      colW / 2,
      top + rowH / 2 + 10,
      colW - 10,
      "--",
      "m"
    );
  }


  // =========================================================
  // BOX (0,1): GROUND SPEED + HEADING
  // =========================================================

  u8g2.setFont(u8g2_font_helvB10_tf);

  u8g2.drawStr(
    colW + 5,
    top + 14,
    "V GROUND"
  );

  if (gps.speed.isValid()) {

    snprintf(
      buffer,
      sizeof(buffer),
      "%d",
      (int)gps.speed.kmph()
    );

    drawLargeValueWithSmallUnit(
      colW + colW / 2,
      top + rowH / 2,
      colW - 10,
      buffer,
      "km/h"
    );

  } else {

    drawLargeValueWithSmallUnit(
      colW + colW / 2,
      top + rowH / 2,
      colW - 10,
      "--",
      "km/h"
    );
  }


  // ---------------------------------------------------------
  // Heading
  // ---------------------------------------------------------

  u8g2.setFont(u8g2_font_helvB10_tf);

  if (gps.course.isValid()) {

    snprintf(
      buffer,
      sizeof(buffer),
      "HDG %d deg",
      (int)gps.course.deg()
    );

  } else {

    snprintf(
      buffer,
      sizeof(buffer),
      "HDG ---"
    );
  }

  u8g2.drawStr(
    colW + (colW - u8g2.getStrWidth(buffer)) / 2,
    top + rowH - 16,
    buffer
  );


  // =========================================================
  // BOX (1,0): CLIMB RATE
  // =========================================================

  u8g2.setFont(u8g2_font_helvB10_tf);

  u8g2.drawStr(
    5,
    top + rowH + 14,
    "CLIMB RATE"
  );

  if (bmpOK && windowCount >= 3) {

    snprintf(
      buffer,
      sizeof(buffer),
      "%+.1f",
      currentClimbRateMS
    );

    drawLargeValueWithSmallUnit(
      colW / 2,
      top + rowH + rowH / 2 + 10,
      colW - 10,
      buffer,
      "m/s"
    );

  } else {

    drawLargeValueWithSmallUnit(
      colW / 2,
      top + rowH + rowH / 2 + 10,
      colW - 10,
      "--",
      "m/s"
    );
  }


  // =========================================================
  // BOX (1,1): NEAREST AIRSPACE (temporary stand-in for ALTITUDE AGL
  // until a DEM is wired in -- see groundElevationFt's declaration,
  // which is still used by the airspace scanner itself either way)
  // =========================================================

  u8g2.setFont(u8g2_font_helvB10_tf);

  u8g2.drawStr(
    colW + 5,
    top + rowH + 14,
    "AIRSPACE"
  );

  AirspaceResult boxAirspace;
  bool boxAirspaceValid = getAirspaceSnapshot(boxAirspace);

  char vertBuf[24];
  char horiBuf[24];

  if (boxAirspaceValid) {
    snprintf(vertBuf, sizeof(vertBuf), "AIRSP VERT: %.0fft", boxAirspace.vertDistance_ft);
    snprintf(horiBuf, sizeof(horiBuf), "AIRSP HORI: %.1fkm", boxAirspace.horizDistance_km);
  } else {
    snprintf(vertBuf, sizeof(vertBuf), "AIRSP VERT: --ft");
    snprintf(horiBuf, sizeof(horiBuf), "AIRSP HORI: --km");
  }

  u8g2.drawStr(
    colW + (colW - u8g2.getStrWidth(vertBuf)) / 2,
    top + rowH + rowH / 2,
    vertBuf
  );

  u8g2.drawStr(
    colW + (colW - u8g2.getStrWidth(horiBuf)) / 2,
    top + rowH + rowH / 2 + 20,
    horiBuf
  );

  // ---- Restore this block (and delete the one above) once a DEM gives
  // ---- a real per-position ground elevation instead of the fixed
  // ---- groundElevationFt site value:
  //
  // if (bmpOK && windowCount > 0 && qnhCalibrated) {
  //   float aglM = currentAltitudeM - (groundElevationFt / 3.28084f);
  //   snprintf(buffer, sizeof(buffer), "%d", (int)roundf(aglM));
  //   drawLargeValueWithSmallUnit(colW + colW / 2, top + rowH + rowH / 2 + 10, colW - 10, buffer, "m");
  // } else {
  //   drawLargeValueWithSmallUnit(colW + colW / 2, top + rowH + rowH / 2 + 10, colW - 10, "--", "m");
  // }


  // =========================================================
  // BOX (2,0): GLIDE RATIO
  // =========================================================

  u8g2.setFont(u8g2_font_helvB10_tf);

  u8g2.drawStr(
    5,
    top + 2 * rowH + 14,
    "GLIDE RATIO"
  );

  bool glideValid =
    bmpOK &&
    gps.speed.isValid() &&
    currentClimbRateMS < -CLIMB_DEADBAND_MS &&
    currentClimbRateMS > -20.0f;

  if (glideValid) {

    float glideRatio =
      gps.speed.mps() / (-currentClimbRateMS);

    snprintf(
      buffer,
      sizeof(buffer),
      "%.1f",
      glideRatio
    );

    drawLargeValueWithSmallUnit(
      colW / 2,
      top + 2 * rowH + rowH / 2 + 10,
      colW - 10,
      buffer,
      ":1"
    );

  } else {

    drawLargeValueWithSmallUnit(
      colW / 2,
      top + 2 * rowH + rowH / 2 + 10,
      colW - 10,
      "--",
      ":1"
    );
  }


  // =========================================================
  // BOX (2,1): WIND / AIRSPEED
  // =========================================================

  u8g2.setFont(u8g2_font_helvB10_tf);

  u8g2.drawStr(
    colW + 5,
    top + 2 * rowH + 14,
    "WIND / AIRSPEED"
  );


  char windBuf[20];
  char airBuf[20];
  char windDirBuf[20];
  if (windEstimateValid) {

    snprintf(
      windBuf,
      sizeof(windBuf),
      "WIND %.0f km/h",
      estimatedWindSpeedKph
    );

      snprintf(
      windDirBuf,
      sizeof(airBuf),
      "FROM %s",
     getCompassDirection(estimatedWindDirectionDeg)
    );

    snprintf(
      airBuf,
      sizeof(airBuf),
      "AIR %.0f km/h",
      estimatedAirspeedKph
    );

  } else {

    snprintf(
      windBuf,
      sizeof(windBuf),
      "WIND -- km/h"
    );

    snprintf(
      windDirBuf,
      sizeof(windDirBuf),
      "FROM --"
    );

    snprintf(
      airBuf,
      sizeof(airBuf),
      "AIR -- km/h"
    );
  }


  u8g2.setFont(u8g2_font_helvB14_tf);

  u8g2.drawStr(
    colW + (colW - u8g2.getStrWidth(windBuf)) / 2,
    top + 2 * rowH + 40,
    windBuf
  );

   u8g2.drawStr(
    colW + (colW - u8g2.getStrWidth(windBuf)) / 2,
    top + 2 * rowH + 60,
    windDirBuf
  );
  u8g2.drawStr(
    colW + (colW - u8g2.getStrWidth(airBuf)) / 2,
    top + 2 * rowH + 100,
    airBuf
  );
}

// =====================================================
// WEATHER PAGE -- stub. WiFi init + phone-tether weather data is
// future work (item 6) -- this is just the page shell for now.
// =====================================================
void drawWeatherPage() {
  // ---------------------------------------------------------
  // 0. Snapshot localMeters under the mutex, then release it immediately
  // -- rendering below runs lock-free, matching the same pattern used in
  // drawADSBPage(). WindMeter is small plain data (TRACKED_METERS entries),
  // so a full-array copy is cheap.
  // ---------------------------------------------------------
  WindMeter localMetersSnapshot[TRACKED_METERS];
  bool snapshotHasWeatherData = hasWeatherData;

  if (snapshotHasWeatherData && backgroundDataMutex != nullptr &&
      xSemaphoreTake(backgroundDataMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
    memcpy(localMetersSnapshot, localMeters, sizeof(localMeters));
    xSemaphoreGive(backgroundDataMutex);
  } else {
    for (int i = 0; i < TRACKED_METERS; i++) {
      localMetersSnapshot[i].valid = false;
    }
  }

  // ---------------------------------------------------------
  // 1. Fixed four-row layout
  // ---------------------------------------------------------
  const int startY = TOP_BAR_HEIGHT_PX;
  const int availableHeight = SCREEN_H - startY;
  const int rowHeight = availableHeight / 4;

  // ---------------------------------------------------------
  // 2. No weather data
  // ---------------------------------------------------------
  if (!snapshotHasWeatherData) {
    u8g2.setFont(u8g2_font_helvB24_tf);

    const char* msg = "No Weather Data";
    int msgWidth = u8g2.getStrWidth(msg);

    u8g2.drawStr(
      (SCREEN_W - msgWidth) / 2,
      startY + (availableHeight / 2),
      msg);

    return;
  }

  // ---------------------------------------------------------
  // 3. Draw each fixed row
  // ---------------------------------------------------------
  for (int i = 0; i < TRACKED_METERS; i++) {

    if (!localMetersSnapshot[i].valid) {
      continue;
    }

    const int currentBoxY =
      startY + (i * rowHeight);

    // -----------------------------------------------------
    // Row separator
    // -----------------------------------------------------
    if (i > 0) {
      u8g2.drawLine(
        0,
        currentBoxY,
        SCREEN_W,
        currentBoxY);
    }

    // =====================================================
    // LINE 1
    // Station name + distance
    // =====================================================

    u8g2.setFont(u8g2_font_helvB18_tf);

    const int line1Y =
      currentBoxY + 25;

    // Station name - maximum 15 characters
    char stationNameBuf[16];
    snprintf(
      stationNameBuf,
      sizeof(stationNameBuf),
      "%.15s",
      localMetersSnapshot[i].name);

    u8g2.drawStr(
      6,
      line1Y,
      stationNameBuf);

    // Distance
    char distBuf[16];

    snprintf(
      distBuf,
      sizeof(distBuf),
      "%.1f km",
      localMetersSnapshot[i].distanceKm);

    int distanceWidth =
      u8g2.getStrWidth(distBuf);

    u8g2.drawStr(
      SCREEN_W - distanceWidth - 6,
      line1Y,
      distBuf);

    // =====================================================
    // LINE 2
    // AVE / GUST / DIRECTION
    // =====================================================

    const char* compassHdg =
      getCompassDirection(
        localMetersSnapshot[i].bearingDeg);

    char windBuf[64];

    snprintf(
      windBuf,
      sizeof(windBuf),
      "AVE: %.0f km/h   GUST: %.0f km/h   %s",
      localMetersSnapshot[i].speedKph,
      localMetersSnapshot[i].gustKph,
      compassHdg);

    // -----------------------------------------------------
    // Choose largest font that fits
    // -----------------------------------------------------

    const int maxWidth =
      SCREEN_W - 12;

    const uint8_t* selectedFont =
      u8g2_font_helvB18_tf;

    u8g2.setFont(u8g2_font_helvB18_tf);

    if (u8g2.getStrWidth(windBuf) > maxWidth) {

      u8g2.setFont(u8g2_font_helvB18_tf);

      if (u8g2.getStrWidth(windBuf) <= maxWidth) {
        selectedFont = u8g2_font_helvB18_tf;
      } else {
        selectedFont = u8g2_font_helvB14_tf;
      }
    }

    u8g2.setFont(selectedFont);

    int windWidth =
      u8g2.getStrWidth(windBuf);

    int windX =
      (SCREEN_W - windWidth) / 2;

    const int line2Y =
      currentBoxY + rowHeight - 15;

    u8g2.drawStr(
      windX,
      line2Y,
      windBuf);
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
   
  // =========================================================
  // ALTITUDE OVERLAY BOX
  // =========================================================

  u8g2.setFont(u8g2_font_helvB18_tf);

  char altitudeText[16];

  if (qnhCalibrated) {
    snprintf(
      altitudeText,
      sizeof(altitudeText),
      "%.0f ft",
      currentAltitudeM * 3.28084f
    );
  } else {
    snprintf(
      altitudeText,
      sizeof(altitudeText),
      "0 ft"
    );
  }

  int altitudeW = u8g2.getStrWidth(altitudeText) + 10;
  int altitudeH = u8g2.getFontAscent() - u8g2.getFontDescent() + 6;

  int altitudeX = SCREEN_W - altitudeW - 20;
  int altitudeY = (SCREEN_H - 26) - 15;  // add to the last number to shift the box up

  u8g2.drawFrame(
    altitudeX,
    altitudeY,
    altitudeW,
    altitudeH
  );

  u8g2.drawStr(
    altitudeX + 5,
    altitudeY + 23,
    altitudeText
  );

  const int cx = SCREEN_W / 2;
  const int cy = TOP_BAR_HEIGHT_PX + (SCREEN_H - TOP_BAR_HEIGHT_PX) / 2;
  const int r = 130;
  const float MAX_RADIUS_KM = 30.0f;

  // Reset conflict evaluation flag for this pass
  conflictDetectedThisFrame = false;

  // 1. Gather your heading and altitude telemetry from the TinyGPS++ stream
  float gliderHeading = 0.0f;
  bool isMoving = false;


  if (gps.course.isValid() && gps.course.age() < 4000 && gps.speed.knots() > 2.0f) {
    gliderHeading = (float)gps.course.deg();
    isMoving = true;
  }

  // Pull current altitude (defaults to 0 if GPS is not connected/locked yet)
  float myAltitudeFeet = gps.altitude.feet();

  // 2. Draw Rotating Crosshair Grid Lines (Compass Rose Matrix)
  u8g2.drawCircle(cx, cy, r);
  u8g2.drawCircle(cx, cy, r / 2);  // 15km Inner reference ring

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

  // Compass Typography Markers
  u8g2.setFont(u8g2_font_7x14_tf);
  u8g2.drawStr(nX - 2, nY - 2, "N");
  u8g2.drawStr(sX - 2, sY + 8, "S");
  u8g2.drawStr(eX + 4, eY + 3, "E");
  u8g2.drawStr(wX - 8, wY + 3, "W");

  // 3. Draw Center Navigation Reference Symbol
  drawGliderHeadingArrow(cx, cy, 6, isMoving ? 0.0f : -1.0f);

  // 4. DRAW VARIO OVERLAY BOX (Upgraded to Helvetica Bold)
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
 

  // 5. Snapshot the aircraft list under the mutex, then release it
  // immediately -- all the trig/SPI rendering below runs without holding
  // the lock, so a slow redraw can never make the Core 0 background task
  // wait, and vice versa.
  AircraftSnapshot snapshotAircraft[MAX_DISPLAYED_AIRCRAFT];
  int snapshotCount = 0;
  bool snapshotHasData = hasAdsbData;

  if (snapshotHasData && backgroundDataMutex != nullptr &&
      xSemaphoreTake(backgroundDataMutex, pdMS_TO_TICKS(20)) == pdTRUE) {

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
      snapshotCount++;
    }

    xSemaphoreGive(backgroundDataMutex);
  }

  if (!snapshotHasData) {
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(cx - 50, cy + 30, "No data fetched");
    return;
  }

  if (snapshotCount == 0) {
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(cx - 55, cy + 30, "No local traffic");
    return;
  }

  // 6. Set Typography: Switch to a Heavy, High-Contrast True Bold Font
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

    // Draw individual target node disc
    u8g2.drawDisc(acX, acY, 2);

    // --- TELEMETRY STRING GENERATION ---
    float altitudeFeet = snapshotAircraft[i].altFeet;
    float speedKnots = snapshotAircraft[i].speedKt;
    float headingDeg = snapshotAircraft[i].headingDeg;

    // Same 5km / 2000ft bubble used for the one-time "new intruder" chirp
    // in performADSBUpdate(), re-evaluated every redraw so the alarm keeps
    // re-triggering every ALARM_SILENCE_MS for as long as a conflicting
    // aircraft remains on screen.
    if (distanceKM <= 5.0f && fabsf(altitudeFeet - myAltitudeFeet) <= 2000.0f) {
      conflictDetectedThisFrame = true;
    }

    float flightLevelFloat = altitudeFeet / 1000.0f;
    if (flightLevelFloat < 0.0f) flightLevelFloat = 0.0f;

    const char* compassHdg = getCompassDirection(headingDeg);

    char dataTag[24];
    snprintf(dataTag, sizeof(dataTag), "FL%.1f %s %.0fkt", flightLevelFloat, compassHdg, speedKnots);

    // 7. Render Anti-Clipping Text Box Frame Safely Beside Target Node
    int textX = acX + 8;  // Offset further out to avoid crowding the dot

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
}

// =====================================================
// PARAMOTOR PAGE: same flight data layout as the paraglider page, with
// engine temperature and RPM reserved in the bottom row.
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

  u8g2.setFont(u8g2_font_helvB10_tf);
  u8g2.drawStr(5, top + 14, "ALTITUDE M");
  if (bmpOK && windowCount > 0) {
    snprintf(buffer, sizeof(buffer), "%d", (int)roundf(currentAltitudeM));
  } else {
    snprintf(buffer, sizeof(buffer), "--");
  }
  drawLargestBoldCentered(colW / 2, top + rowH / 2 + 10, colW - 10, buffer);

  u8g2.setFont(u8g2_font_helvB10_tf);
  u8g2.drawStr(colW + 5, top + 14, "V GROUND");
  if (gps.speed.isValid()) {
    snprintf(buffer, sizeof(buffer), "%d km/h", (int)gps.speed.kmph());
  } else {
    snprintf(buffer, sizeof(buffer), "NO FIX");
  }
  drawLargestBoldCentered(colW + colW / 2, top + rowH / 2, colW - 10, buffer);
  if (gps.course.isValid()) {
    snprintf(buffer, sizeof(buffer), "HDG %d deg", (int)gps.course.deg());
  } else {
    snprintf(buffer, sizeof(buffer), "HDG ---");
  }
  u8g2.setFont(u8g2_font_helvB10_tf);
  u8g2.drawStr(colW + (colW - u8g2.getStrWidth(buffer)) / 2, top + rowH - 16, buffer);

  u8g2.setFont(u8g2_font_helvB10_tf);
  u8g2.drawStr(5, top + rowH + 14, "CLIMB RATE");
  if (bmpOK && windowCount >= 3) {
    snprintf(buffer, sizeof(buffer), "%+.1f m/s", currentClimbRateMS);
  } else {
    snprintf(buffer, sizeof(buffer), "-- m/s");
  }
  drawLargestBoldCentered(colW / 2, top + rowH + rowH / 2 + 10, colW - 10, buffer);

  u8g2.setFont(u8g2_font_helvB10_tf);
  u8g2.drawStr(colW + 5, top + rowH + 14, "ALTITUDE AGL");
  drawLargestBoldCentered(colW + colW / 2, top + rowH + rowH / 2 + 10, colW - 10, "-- m");

  u8g2.setFont(u8g2_font_helvB10_tf);
  u8g2.drawStr(5, top + 2 * rowH + 14, "ENG TEMP");
  drawLargestBoldCentered(colW / 2, top + 2 * rowH + rowH / 2 + 10, colW - 10, "-- C");

  u8g2.setFont(u8g2_font_helvB10_tf);
  u8g2.drawStr(colW + 5, top + 2 * rowH + 14, "RPM");
  drawLargestBoldCentered(colW + colW / 2, top + 2 * rowH + rowH / 2 + 10, colW - 10, "----");
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
      // Drawn last, on top of whatever page is active, so a nearby/entered
      // controlled airspace is never hidden behind a page cycle.
      drawAirspaceWarning();
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

void drawAirspaceWarning() {
  AirspaceResult result;
  bool valid = getAirspaceSnapshot(result);

  if (!valid) return;

  bool insideNow = result.insideHoriz && result.insideVert;
  bool nearby = !insideNow &&
      result.horizDistance_km <= AIRSPACE_WARN_HORIZ_KM &&
      result.vertDistance_ft <= AIRSPACE_WARN_VERT_FT;

  // One-shot alert on entering controlled airspace, not on every redraw.
  static bool wasInside = false;
  if (insideNow && !wasInside) {
    playFeedbackTone(900.0f, 600);
  }
  wasInside = insideNow;

  if (!insideNow && !nearby) return;

  char line1[40];
  char line2[40];

  if (insideNow) {
    snprintf(line1, sizeof(line1), "INSIDE %s", result.name);
    snprintf(line2, sizeof(line2), "Class %s", result.classId);
  } else {
    snprintf(line1, sizeof(line1), "%s", result.name);
    snprintf(line2, sizeof(line2), "%.1fkm  %.0fft", result.horizDistance_km, result.vertDistance_ft);
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
}

