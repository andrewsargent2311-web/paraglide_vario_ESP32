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
#include <Preferences.h>  // climb/sink volume persistence (see loadVarioVolumes())
#include <FS.h>
#include <SD_MMC.h>
#include "secrets.h"
#include "OpenAirScanner.h"
#include "menu.h"
#include "settings.h"
#include "Sx126xLink.h"
#include "Fanet.h"
#include "FanetMessaging.h"
#include "wifi_manager.h"
#include "FileServer.h"
#include "DrawPages.h"
#include "CloudBase.h"
// =====================================================
// WIFI (feeds Weather + ADS-B pages) + BLUETOOTH (engine meter)
// Connection details (saved networks, remembered BLE device) live in
// secrets.h / wifi_manager.cpp / ble_manager.cpp, and are picked/managed
// from the Connections menu -- see menu.cpp. wifiConnected/bleConnected
// are declared in wifi_manager.h/ble_manager.h respectively.
// =====================================================
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
// 8mm converted to pixels: this panel doesn't have a published active-area
// mm figure, so this is derived from the stated 4.2" diagonal at 300x400
// (diag = sqrt(300^2+400^2) = 500px = 106.68mm -> ~4.687 px/mm -> 8mm ~= 37.5px).
// Close enough for layout purposes, but if the exact 8mm matters, verify
// against the physical panel with calipers rather than trust this alone.
U8G2_ST7305_300X400_1_4W_HW_SPI u8g2(U8G2_R0, /*cs=*/RLCD_CS, /*dc=*/RLCD_DC, /*reset=*/RLCD_RST);

// =====================================================
// BOOT SPLASH IMAGE (Roy)
// =====================================================
// Loaded from the SD card as a raw 1bpp packed bitmap (XBM byte layout:
// rows padded to a whole byte, bits LSB-first, 1 = black/ink) rather than
// baked into the firmware, so it's easy to swap the picture just by
// replacing the file on the card. Generated from Roy.png at 280x210 --
// see the accompanying ROY.BIN this was built from.
#define SPLASH_IMG_FILE "/ROY.BIN"
#define SPLASH_IMG_W 280
#define SPLASH_IMG_H 210
#define SPLASH_IMG_ROW_BYTES ((SPLASH_IMG_W + 7) / 8)
#define SPLASH_IMG_BYTES (SPLASH_IMG_ROW_BYTES * SPLASH_IMG_H)
#define SPLASH_DISPLAY_MS 3000UL


// =====================================================
// SD CARD / IGC FLIGHT LOG
// =====================================================

// The microSD slot on the Waveshare ESP32-S3-RLCD-4.2 is wired to the
// ESP32-S3's native SDMMC peripheral in 1-bit mode, NOT to SPI -- there is
// no CS line run to the card at all, which is why treating pin 1 as a CS
// pin never worked. CLK/CMD/D0 below match Waveshare's own SD card example
// for this exact board (02_Example/Arduino/06_SD_Card). Unlike classic
// ESP32, the S3's SDMMC pins are routed through the GPIO matrix, so they
// must be assigned with SD_MMC.setPins() before SD_MMC.begin().
#define SD_MMC_CLK_PIN 38
#define SD_MMC_CMD_PIN 21
#define SD_MMC_D0_PIN 39

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
// GPS/baro altitude is height above sea level, not height above terrain,
// so this is now a live lookup into a pre-processed DEM tile on the SD
// card (see TerrainDem.h / selectedDemFile below) rather than a fixed site value.
// groundElevationValid is false until the first successful lookup, and
// goes false again if the aircraft flies outside the downloaded tile --
// both the airspace scanner and the AGL box must check it before trusting
// groundElevationFt.
float groundElevationFt = 0.0f;
volatile bool groundElevationValid = false;

#include "TerrainDem.h"
// Produced offline from a LINZ DEM GeoTIFF by dem_to_agldem.py (downscaled
// + reprojected to WGS84 lat/lon) -- see that script for how to (re)build
// this for a different flying site.
// The active filename is now chosen from the menu's Map screen (see
// selectedDemFile / setSelectedDemFile() in menu.h/.cpp) rather than fixed
// here -- it defaults to "/DEM.ADEM" until changed.
#define DEM_SCAN_INTERVAL_MS 5000UL
unsigned long demScanAnchor = 0;

#define AIRSPACE_SCAN_INTERVAL_MS 10000UL
unsigned long airspaceScanAnchor = 0;

AirspaceResult nearestAirspace;
volatile bool airspaceResultValid = false;

// alertOnly = false counterpart of the above -- includes CFZ entries,
// feeds only the ADS-B page's "Airspace Info" bar. See the comment at
// the findNearestControlledAirspace() call site in backgroundTask().
AirspaceResult nearestAirspaceInfo;
volatile bool airspaceInfoResultValid = false;

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
float currentHumidityPct = NAN;   // raw SHTC3 relative humidity, %
float currentPressureHpa = NAN;   // latest BMP580 pressure, hPa
float cloudBaseAboveM = NAN;      // smoothed estimate: metres from you up to cloud base (NAN = no estimate)
// The SHTC3 sits on the Waveshare board next to the ESP32 and battery, so it reads
// warmer than the outside air. Dew point is NOT affected by that (warming air doesn't
// change its moisture), but the air temperature is, and that would make the cloud base
// read too high (about +125 m per degree C of error). Set this to how many degrees C
// your unit reads above a trusted thermometer after it has been on for ~10 minutes.
#define SHT_SELF_HEAT_OFFSET_C 0.0f
#define CLOUD_BASE_SMOOTH_ALPHA 0.3f  // 0..1, lower = smoother (samples every SHT_SAMPLE_MS)
#define SHT_SAMPLE_MS 10000
void updateCloudBaseEstimate();  // defined below, in the CLOUD BASE ESTIMATE section
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
#define WIND_MIN_CIRCLE_SPEED_KPH 15.0f
#define WIND_MAX_CIRCLE_SPEED_KPH 80.0f
#define WIND_MAX_ESTIMATE_KPH 50.0f
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
// FANET (HT-RA62 / SX1262) -- wired MISO=0 SCK=1 BUSY=2 MOSI=3 CS=17,
// DIO1 and RST both NC (see Sx126xLink.h for why). Own SPI bus, separate
// from the display's -- no pin overlap with anything else on this board.
// =====================================================
Sx126xLink fanetRadio;
bool fanetRadioOK = false;

// TODO: pick a real manufacturer ID from the FANET spec's registered
// list (or use a private/testing value while bench-testing) rather than
// this placeholder, and give this device a unique 16-bit ID.
FanetAddress myFanetAddress = { 0xFC, 0x0001 };
FanetStack fanet(fanetRadio, myFanetAddress);

// Live FANET traffic table (type/constants in DrawPages.h, alongside
// WindMeter -- same "shared between the .ino writer and the DrawPages.cpp
// reader" pattern). Definition lives here; DrawPages.h only declares it
// extern.
FanetContact fanetContacts[MAX_FANET_CONTACTS];
FanetWeatherStation fanetWeatherStations[MAX_FANET_WEATHER_STATIONS];

#define FANET_BEACON_INTERVAL_MS 5000UL

// Fires whenever a FANET tracking beacon is received from another
// aircraft. Updates fanetContacts[] so drawADSBPage() (DrawPages.cpp) can
// plot it alongside ADS-B traffic -- see the FanetContact comment in
// DrawPages.h for why no mutex is needed here despite drawADSBPage()
// reading the same array.
void onFanetTracking(const FanetAddress& src, const FanetTracking& pkt,
                      float rssi, float snr) {
  Serial.printf("[FANET] from %02X:%04X  lat=%.5f lon=%.5f alt=%ldm  spd=%.0fkm/h  rssi=%.0f snr=%.1f\n",
                src.manufacturer, src.id,
                pkt.latitude, pkt.longitude, (long)pkt.altitudeM,
                pkt.speedKmh, rssi, snr);

  // Find the existing slot for this address, or the first free slot if
  // it's new, or -- if the table is completely full of other live
  // contacts -- evict whichever one has gone quietest. A beacon actually
  // arriving right now is always more useful to show than a contact we
  // haven't heard from in a while.
  int slot = -1;
  int quietestSlot = 0;
  unsigned long quietestSeenMs = (unsigned long)-1;  // wraps to ULONG_MAX

  for (int i = 0; i < MAX_FANET_CONTACTS; i++) {
    if (fanetContacts[i].valid &&
        fanetContacts[i].addr.manufacturer == src.manufacturer &&
        fanetContacts[i].addr.id == src.id) {
      slot = i;
      break;
    }
    if (!fanetContacts[i].valid && slot == -1) {
      slot = i;  // remember the first free slot, but keep scanning for an exact match
    }
    if (fanetContacts[i].lastSeenMs < quietestSeenMs) {
      quietestSeenMs = fanetContacts[i].lastSeenMs;
      quietestSlot = i;
    }
  }

  if (slot == -1) {
    slot = quietestSlot;  // table full of other live contacts -- evict the quietest
  }

  FanetContact& c = fanetContacts[slot];
  c.valid = true;
  c.addr = src;
  c.lat = (float)pkt.latitude;
  c.lon = (float)pkt.longitude;
  c.altitudeM = pkt.altitudeM;
  c.speedKmh = pkt.speedKmh;
  c.climbMs = pkt.climbMs;
  c.headingDeg = pkt.headingDeg;
  c.aircraftType = pkt.aircraftType;
  c.rssi = rssi;
  c.snr = snr;
  c.lastSeenMs = millis();
}

// Fires whenever a FANET Service (type 4) packet with wind data is
// received from a ground weather station. Updates fanetWeatherStations[]
// so drawWeatherPage() (DrawPages.cpp) can show it -- as the preferred
// source (Config > Weather Settings > Source = FANET) or as the fallback
// when the Zephyr network has no data. See the FanetWeatherStation
// comment in DrawPages.h for why no mutex is needed here.
void onFanetWeather(const FanetAddress& src, const FanetWeather& pkt,
                     float rssi, float snr) {
  Serial.printf("[FANET WX] from %02X:%04X  lat=%.5f lon=%.5f  wind=%.0f@%.0fkm/h gust=%.0fkm/h  rssi=%.0f snr=%.1f\n",
                src.manufacturer, src.id,
                pkt.latitude, pkt.longitude,
                pkt.windHeadingDeg, pkt.windSpeedKmh, pkt.windGustKmh,
                rssi, snr);

  // Same find-slot-or-evict-quietest approach as onFanetTracking() above.
  int slot = -1;
  int quietestSlot = 0;
  unsigned long quietestSeenMs = (unsigned long)-1;

  for (int i = 0; i < MAX_FANET_WEATHER_STATIONS; i++) {
    if (fanetWeatherStations[i].valid &&
        fanetWeatherStations[i].addr.manufacturer == src.manufacturer &&
        fanetWeatherStations[i].addr.id == src.id) {
      slot = i;
      break;
    }
    if (!fanetWeatherStations[i].valid && slot == -1) {
      slot = i;
    }
    if (fanetWeatherStations[i].lastSeenMs < quietestSeenMs) {
      quietestSeenMs = fanetWeatherStations[i].lastSeenMs;
      quietestSlot = i;
    }
  }

  if (slot == -1) {
    slot = quietestSlot;
  }

  FanetWeatherStation& w = fanetWeatherStations[slot];
  w.valid = true;
  w.addr = src;
  w.lat = (float)pkt.latitude;
  w.lon = (float)pkt.longitude;
  w.windHeadingDeg = pkt.windHeadingDeg;
  w.windSpeedKmh = pkt.windSpeedKmh;
  w.windGustKmh = pkt.windGustKmh;
  w.hasTemperature = pkt.hasTemperature;
  w.temperatureC = pkt.temperatureC;
  w.lastSeenMs = millis();
}
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
#define INTERCEPT_TONE_LOW_HZ 400
#define INTERCEPT_TONE_TOGGLE_MS 400UL  // time on each tone before switching
bool interceptAlarmActive = false;
unsigned long interceptAlarmStart = 0;
volatile bool hasAdsbData = false;

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

// WindMeter, WEATHER_STATION_NAME_MAX, and TRACKED_METERS now live in
// DrawPages.h (shared with drawWeatherPage(), which reads this array).
WindMeter localMeters[TRACKED_METERS];
volatile bool hasWeatherData = false;

// Steady-state poll cadence is now weatherPollIntervalMs (settings.h),
// editable from Weather Settings > Poll Interval.
constexpr unsigned long WEATHER_FIRST_POLL_DELAY_MS = 15UL * 1000UL;  // First poll fires 15s after boot
unsigned long weatherTimerAnchor = 0;                                 // Fresh, clean background timer
bool weatherFirstPollDone = false;                                    // True once the initial 10s poll has fired

// Proximity alarm timing parameters

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
#define PAGE_BEEP_FREQ 500    // Sets page beep frequency
#define PAGE_BEEP_MS 200    // This sets how long the page beep tone goes for
// A "double press" is two presses with less than this many ms between the
// first release and the second press-down. 50ms is what was asked for, but
// note it's faster than most people can physically double-click (a typical
// double-click is more like 150-400ms) -- raise this if the menu doesn't
// open reliably for you. Every short press is held for up to this long
// before it's actioned (to see whether a second press follows), so this
// value also sets the latency added to ordinary page-cycle/menu-navigate
// presses.
unsigned long pageBeepUntil = 0;  // while set, updateBuzzer() yields the pin to the page-change beep

Page currentPage = PAGE_PARAGLIDER;
const char* PAGE_NAMES[PAGE_COUNT] = { "GLDR", "WIND", "ADSB", "ENG" };
// Only 3 pages are cycled through with a short press. Slot 0 is the "main"
// page and is swappable between Paraglider and Paramotor from the menu;
// slots 1 and 2 are fixed at Weather and ADS-B. (ACTIVE_PAGE_COUNT is
// defined in DrawPages.h, shared with drawTopBar()'s page-dots indicator.)
Page activePages[ACTIVE_PAGE_COUNT] = { PAGE_PARAGLIDER, PAGE_WEATHER, PAGE_ADSB };
uint8_t activePageIndex = 0;  // index into activePages[]; kept in sync with currentPage

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
#define I2S_TONE_CHUNK 64  // samples generated per i2sToneService() call

// ---------------------------------------------------------
// HOW THE TONE SAMPLES REACH THE I2S HARDWARE
//
// AUDIO_USE_DEDICATED_TASK = 1 (default): a dedicated high-priority task
// (audioServiceTask(), Core 1) generates a chunk and does a BLOCKING
// i2s_write(). Because the write blocks whenever the DMA ring is full, the
// I2S hardware's own sample clock paces the task: the ring is always kept
// topped up (~32ms of audio buffered) no matter what any other task or the
// esp_timer task is doing, and there is nothing to drift.
//
// AUDIO_USE_DEDICATED_TASK = 0: the original design -- an esp_timer
// callback every 4ms doing a zero-timeout i2s_write(). That has no way of
// knowing how full the DMA ring is, so how much audio is buffered depends
// on the (arbitrary) moment audio started and on timer jitter; when the
// cushion is small, any delay to the timer task drops or repeats 4ms
// chunks, which sounds like a scrambled/glitching tone. Kept only so you
// can switch back to compare.
// ---------------------------------------------------------
#define AUDIO_USE_DEDICATED_TASK 1

#if AUDIO_USE_DEDICATED_TASK
  // 4 x 128 samples = 512 samples = ~32ms of buffered audio (tone changes
  // are heard within ~35ms).
  #define I2S_DMA_BUF_COUNT 4
  #define I2S_DMA_BUF_LEN 128
#else
  // 8 x 256 samples = 2048 samples = ~128ms.
  #define I2S_DMA_BUF_COUNT 8
  #define I2S_DMA_BUF_LEN 256
#endif

// Smooths pitch changes while a tone is sounding. The vario rate (and so
// the sink/climb pitch) only updates every 100ms, so without this the pitch
// moves in audible little steps -- a rough, warbling texture on a sustained
// low sink tone. This is a ~30ms glide (one-pole filter); big jumps (e.g.
// the alarm's two-tone switch) and the start of every beep snap instantly.
// Set to 0 to disable.
#define TONE_PITCH_SMOOTH_MS 30.0f
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
// Loudness multiplier applied on top of the normal tone amplitude (1.0 =
// normal), read asynchronously by the audio side. Whoever starts a tone sets
// this to that tone's gain immediately BEFORE calling setToneFrequency();
// it is never written to a temporary value (see updateI2sAudioBuzzer()).
volatile float toneGain = 1.0f;
float tonePhase = 0.0f;

// =====================================================
// MUTE / UNMUTE CONFIRMATION TONE
// Played once whenever buzzerMuted is toggled by the long-press gesture
// (see updatePageButton()), so the pilot gets audible confirmation of
// which state they just landed in. Sequenced non-blockingly inside
// updateI2sAudioBuzzer().
//
// These reproduce the BlueFly's own mute/unmute sounds, measured from a
// recording of the real device:
//   MUTE   : 200ms @ ~3320Hz, then three 100ms beeps @ ~3420Hz with
//            growing gaps (100 / 200 / 300 ms).  Total 1.1s.
//   UNMUTE : 1.0s @ ~4020Hz, 100ms gap, 100ms @ ~4020Hz.  Total 1.2s.
//            On the real BlueFly the first ~0.42s of the long beep is
//            ~14dB louder than the rest, so it is split into two steps.
//
// Each step is {frequency Hz (0 = silence), duration ms, gain}. Gain is a
// loudness multiplier: 1.0 = the normal beep level, 2.6 = ~+8dB, 0.55 =
// ~-5dB. Edit the tables to retune -- nothing else needs to change.
// =====================================================
struct ToneStep {
  float freq;
  uint16_t ms;
  float gain;
};

static const ToneStep MUTE_SEQUENCE[] = {
  { 3320.0f, 200, 1.0f },
  {    0.0f, 100, 1.0f },
  { 3420.0f, 100, 1.0f },
  {    0.0f, 200, 1.0f },
  { 3420.0f, 100, 1.0f },
  {    0.0f, 300, 1.0f },
  { 3420.0f, 100, 1.0f },
};

static const ToneStep UNMUTE_SEQUENCE[] = {
  { 4020.0f, 420, 2.6f  },  // loud first part of the long beep
  { 4020.0f, 580, 0.55f },  // quieter remainder (1.0s total)
  {    0.0f, 100, 1.0f  },
  { 4020.0f, 100, 0.55f },
};

#define TONE_SEQ_LEN(arr) (sizeof(arr) / sizeof((arr)[0]))

bool muteToneActive = false;
unsigned long muteToneStart = 0;
bool muteToneIsMuteSequence = false;  // true = play MUTE_SEQUENCE; false = play UNMUTE_SEQUENCE

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
// If GPS hasn't produced a usable altitude fix (see gpsAltitudeGood in
// updateVario()) within this long after boot -- no module wired, no sky
// view, whatever the cause -- stop waiting on it and default QNH to
// standard atmosphere so altitude/climb-rate keep running off the BMP580
// alone instead of sitting "not calibrated" for the whole flight.
#define GPS_QNH_FALLBACK_MS 60000UL

// How long a continuous run of good-quality GPS fixes (see
// gpsAltitudeGood in updateVario()) is averaged over before calibrating
// QNH from it. A single instantaneous GPS altitude sample is noisy
// enough (GPS vertical error is typically 2-3x worse than horizontal,
// and HDOP doesn't bound it specifically) that calibrating off just one
// fix can lock in a QNH that's meaningfully wrong for the rest of the
// flight -- averaging over a real time window smooths that out. If GPS
// quality drops mid-window (gpsAltitudeGood goes false), the window is
// abandoned and a fresh one starts from scratch on the next good fix,
// rather than silently averaging across a gap.
#define QNH_GPS_AVERAGE_MS 10000UL

// A window that reaches QNH_GPS_AVERAGE_MS with fewer fresh fixes than
// this is treated as not enough data to trust (e.g. a flaky GPS module
// only reporting a handful of updates in the window) -- the window
// resets and tries again rather than calibrating off too few samples.
// At a typical 1Hz GPS this is up to half the fixes a full
// QNH_GPS_AVERAGE_MS window could contain missed and still trusted --
// worth tightening if that proves too lenient in practice now that the
// window itself is shorter than it was.
#define QNH_GPS_MIN_SAMPLES 5

// Once calibrated, QNH is re-averaged and updated on this cadence for
// as long as the flight continues, rather than being locked in once and
// never touched again -- real atmospheric pressure drifts over a
// multi-hour flight as weather systems move through, and an unchanging
// QNH would let indicated altitude/AGL quietly drift away from reality
// over that time. Each recalibration reuses the exact same
// QNH_GPS_AVERAGE_MS averaging window and QNH_GPS_MIN_SAMPLES floor as
// the initial calibration -- see updateVario().
#define QNH_RECALIBRATION_INTERVAL_MS (15UL * 60UL * 1000UL)
#define CLIMB_WINDOW_N 8
#define BARO_SAMPLE_MS 100
// CLIMB_DEADBAND_MS is defined in DrawPages.h (shared with drawParagliderPage()'s sink indicator, and with updateI2sAudioBuzzer() below).
#define SINK_ALARM_MS -0.5f  // Sink alarm set to start at -0.5m/s can alter this later to suit
#define CLIMB_TONE_MAX_MS 5.0f
//#define SINK_RELEASE_MS -5.0f  // Set to -5m/s as not uncommon to hit 4 m/s sink alarm switches off above 5 m/s to avoid distraction
//commented out max sink threshold for debugging as its causing clipping
// Sink alarm -- constant (non-pulsed) tone, pitch dropping as sink
// strengthens. Matches the BlueFly hardware settings (manual v1.8):
//   sinkFreq = sinkFreqBase - sinkFreqIncrement * |sink|
// measured from 0 m/s (NOT from the sink threshold), clamped to the
// BlueFly's 130Hz minimum. Defaults: 400Hz base, 100Hz per m/s.
// If your BlueFly's sink settings have been changed in XCSoar, change
// these to match. See the SINK ALARM OUTPUT block in updateI2sAudioBuzzer().
#define SINK_FREQ_BASE_HZ 400.0f
#define SINK_FREQ_INCREMENT_HZ 100.0f  // Hz of pitch drop per 1 m/s of sink
#define SINK_FREQ_MIN_HZ 130.0f
// Climb tone frequency range: climbToneMinHz/climbToneMaxHz (settings.h),
// editable from Config > Vario Freq in the menu.
// Climb beep cadence follows the BlueFly curve in blueflyBeepDurationMs().
// (The old climbGapMinMs/climbGapMaxMs/climbPulseMinMs/climbPulseMaxMs
// settings are no longer used by the audio code -- Config > Vario Beep now
// sets the climb and sink volumes instead.)

// ============================================================
// CLIMB / SINK VOLUME (Config > Vario Beep)
// Independent loudness for the climb beeps and the sink tone, 0-100% in
// 10% steps. 100% = the full level the tone generator produces (the same
// level as before these settings existed); lower values are scaled in dB,
// not linearly, because loudness is perceived roughly logarithmically:
// each 1% below 100 is VARIO_VOLUME_DB_PER_PERCENT dB quieter, so with
// 0.3 each 10% menu step is -3 dB (50% = -15 dB, 10% = -27 dB). 0% is
// silent. This scales the tone generator's amplitude only -- the overall
// speaker volume (Config > Volume, codec register) still applies on top.
// Defaults are VARIO_VOLUME_DEFAULT_*_PERCENT in settings.h.
// ============================================================
#define VARIO_VOLUME_DB_PER_PERCENT 0.3f

uint8_t climbVolumePercent = VARIO_VOLUME_DEFAULT_CLIMB_PERCENT;
uint8_t sinkVolumePercent = VARIO_VOLUME_DEFAULT_SINK_PERCENT;

// Gain used while a page/feedback beep (or a volume-preview tone) is
// sounding -- 1.0 for normal UI beeps.
float pageBeepGain = 1.0f;

// Converts a 0-100 volume percentage to a linear amplitude multiplier.
static float varioVolumeToGain(uint8_t percent) {
  if (percent == 0) return 0.0f;
  if (percent >= 100) return 1.0f;
  return powf(10.0f, -((float)(100 - percent) * VARIO_VOLUME_DB_PER_PERCENT) / 20.0f);
}

// ============================================================
// VARIO AUDIO STATE
// ============================================================
uint32_t climbToneStartMs = 0;
uint32_t climbToneDurationMs = 0;
uint32_t nextClimbBeepMs = 0;
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
// True only while the current QNH came from the no-GPS timeout fallback
// rather than a real GPS-derived calibration. Lets updateVario() upgrade
// to a proper calibration the moment a good fix turns up later, without
// reopening the (deliberately) one-shot calibration once it's genuine.
bool qnhIsFallback = false;
unsigned long lastBeepToggle = 0;
bool beepOn = false;
unsigned long lastSinkBeep = 0;
float batteryVoltage = 0.0f;
uint8_t batteryPercent = 0;
unsigned long lastBattSample = 0;
bool battInitialized = false;

// wifiConnected (wifi_manager.h) and bleConnected/bleEnabled
// (ble_manager.h) are the real connectivity state now -- see the
// Connections menu (menu.cpp) for where they're set.

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
// (drawDashboard, drawAirspaceWarning, getAirspaceSnapshot, drawTopBar,
// drawParagliderPage, drawWeatherPage, drawADSBPage, and drawParamotorPage
// are now declared in DrawPages.h, alongside their implementations in
// DrawPages.cpp.)
void setupI2sCodec();
void i2sToneService();
void updateIgcRecorder();
void startIgcRecording();
void stopIgcRecording();
void writeIgcBRecord();
void formatIgcLatLon();
void updateVario();
void updateWindEstimator();
void updateI2sAudioBuzzer();
void updateBattery();
bool syncClockFromGPS();
void updatePageButton();
float computeClimbRateLeastSquares();
void es8311WriteReg(uint8_t reg, uint8_t value);
void es8311Init();
void audioServiceTask(void* arg);
void applyBuzzerVolume();
void setToneFrequency(float freq);
void performADSBUpdate();
float getDistanceKM(float lat1, float lon1, float lat2, float lon2);
float getBearing(float lat1, float lon1, float lat2, float lon2);
// ---- Page rotation / menu ----
void advanceActivePage();
void jumpToActivePage(Page page);
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
  delay(1000);  // give the USB CDC host a moment to attach before the first print, or it's often lost
  Serial.println("BOOTING FLIGHT COMPUTER...");
  Serial.printf("[BOOT] Free heap: %u | Min heap: %u\n", ESP.getFreeHeap(), ESP.getMinFreeHeap());

  // Pull every persisted setting out of NVS before anything below reads
  // one of them (buzzer volume, climb tone, selected DEM file, etc.).
  loadSettings();
  loadVarioVolumes();  // climb/sink volume (Config > Vario Beep)
  activePages[0] = (mainPageSelection == 1) ? PAGE_PARAMOTOR : PAGE_PARAGLIDER;
  currentPage = activePages[0];
  Serial.println("[BOOT] Settings loaded from flash");

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

  // ---------------------------------------------------------
  // Display + SD card brought up first, ahead of everything else that
  // used to sit in front of them, so the splash screen can be drawn as
  // early as possible. Previously the splash didn't appear until AFTER
  // the BMP test loop, RTC, FANET radio, and a blocking up-to-10s WiFi
  // connect attempt had all already run -- that's what made the screen
  // look "dead" for ~14s after power-on even though everything was
  // working fine the whole time (visible on Serial immediately).
  // ---------------------------------------------------------
  SPI.begin(RLCD_SCK, -1 /*MISO unused*/, RLCD_MOSI, RLCD_CS);
  u8g2.begin();
  applyScreenOrientation();  // screenOrientation was already loaded above by loadSettings()
  Serial.println("DISPLAY INITIALIZED");
  Serial.printf("[BOOT] After display - Free heap: %u | Min heap: %u\n", ESP.getFreeHeap(), ESP.getMinFreeHeap());

  // ESP32-S3 has no fixed default SDMMC pin set (unlike classic ESP32) --
  // pins must be assigned explicitly before begin(). Done here, ahead of
  // its old spot further down, purely so the splash image can be loaded
  // from SD before everything else runs.
  if (!SD_MMC.setPins(SD_MMC_CLK_PIN, SD_MMC_CMD_PIN, SD_MMC_D0_PIN)) {
    Serial.println("SD_MMC.setPins() failed");
  }
  sdCardOK = SD_MMC.begin("/sdcard", true);  // true = 1-bit mode (only D0 is wired)
  Serial.println(sdCardOK ? "SD CARD MOUNTED" : "SD CARD NOT FOUND -- IGC recording disabled");

  Serial.println("[BOOT] Drawing splash screen...");

  // Try to load the boot image from the SD card. Kept as a plain malloc'd
  // buffer scoped to setup() -- it's only needed for this one draw, so
  // there's no reason to keep ~7KB of RAM reserved for it afterwards.
  bool splashImageLoaded = false;
  uint8_t* splashBuf = nullptr;

  if (sdCardOK) {
    File splashFile = SD_MMC.open(SPLASH_IMG_FILE, FILE_READ);
    if (splashFile && splashFile.size() == SPLASH_IMG_BYTES) {
      splashBuf = (uint8_t*)malloc(SPLASH_IMG_BYTES);
      if (splashBuf != nullptr) {
        size_t readBytes = splashFile.read(splashBuf, SPLASH_IMG_BYTES);
        splashImageLoaded = (readBytes == SPLASH_IMG_BYTES);
        if (!splashImageLoaded) {
          Serial.println("[BOOT] Splash image read short -- using text splash");
        }
      } else {
        Serial.println("[BOOT] Failed to allocate splash image buffer -- using text splash");
      }
    } else if (splashFile) {
      Serial.printf("[BOOT] %s is %u bytes, expected %u -- using text splash\n",
                    SPLASH_IMG_FILE, (unsigned)splashFile.size(), (unsigned)SPLASH_IMG_BYTES);
    } else {
      Serial.printf("[BOOT] %s not found on SD card -- using text splash\n", SPLASH_IMG_FILE);
    }
    if (splashFile) splashFile.close();
  } else {
    Serial.println("[BOOT] SD card not mounted -- using text splash");
  }
  const char* CONTROLLED_CLASSES[] = {
      "A",
      "B",
      "C",
      "D",
      "CTR"
  };

  const uint8_t NUM_CONTROLLED_CLASSES =
      sizeof(CONTROLLED_CLASSES) /
      sizeof(CONTROLLED_CLASSES[0]);

  if (loadAirspaceDatabase(
          "/AIRSPACE.txt",
          CONTROLLED_CLASSES,
          NUM_CONTROLLED_CLASSES)) {

      Serial.print("Airspace database ready: ");
      Serial.print(getAirspaceCount());
      Serial.println(" airspaces");

  } else {

      Serial.println(
          "ERROR: Airspace database failed to load");
  }
  u8g2.firstPage();
  do {
    if (splashImageLoaded) {
      u8g2.drawXBMP((SCREEN_W - SPLASH_IMG_W) / 2, (SCREEN_H - SPLASH_IMG_H) / 2,
                    SPLASH_IMG_W, SPLASH_IMG_H, splashBuf);
    } else {
      u8g2.setFont(u8g2_font_fub20_tf);
      u8g2.drawStr(20, 200, "FLIGHT COMPUTER");
    }
  } while (u8g2.nextPage());

  if (splashBuf != nullptr) {
    free(splashBuf);
  }

  // Timestamp the draw rather than just sleeping SPLASH_DISPLAY_MS right
  // here -- everything below now runs WHILE the splash is already on
  // screen, and we only make up the remaining time (if any) once it's
  // all done. See the "Splash hold" block below.
  unsigned long splashDrawnAt = millis();
  Serial.println(splashImageLoaded ? "[BOOT] Splash image drawn"
                                    : "[BOOT] Splash screen drawn (text fallback)");

  // ---------------------------------------------------------
  // Everything below is init that the pilot doesn't need to see happen
  // -- it now runs after something is already showing on screen instead
  // of before it.
  // ---------------------------------------------------------

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
    // begin() leaves the sensor in NORMAL mode. Per the BMP5xx datasheet,
    // OSR/ODR/press-enable config registers should only be written while in
    // STANDBY -- but setOutputDataRate()/setPressureOversampling()/
    // enablePressure() don't enforce that themselves (only the IIR filter
    // setter does), so writing them straight after begin() means they hit
    // the sensor mid-measurement with no guarantee they're actually applied.
    // Force standby first, configure everything, then switch to NORMAL last
    // so measurement only starts once the config is known-good.
    bmp.setPowerMode(BMP5XX_POWERMODE_STANDBY);
    bmp.setTemperatureOversampling(BMP5XX_OVERSAMPLING_2X);
    bmp.setPressureOversampling(BMP5XX_OVERSAMPLING_8X);
    bmp.setIIRFilterCoeff(BMP5XX_IIR_FILTER_COEFF_3);
    bmp.setOutputDataRate(BMP5XX_ODR_50_HZ);
    bmp.enablePressure(true);
    bmp.setPowerMode(BMP5XX_POWERMODE_NORMAL);
    Serial.println("BMP580 FOUND -- VARIO ACTIVE");
    delay(100);

    Serial.println("[BMP TEST] Testing sensor...");

    // Trimmed from 10 iterations to 3 -- this is just a diagnostic
    // sanity check, and each extra iteration cost a further 100ms of
    // boot time for no functional benefit.
    for (int i = 0; i < 3; i++) {
      Serial.printf("[BMP TEST] dataReady=%d\n", bmp.dataReady());

      if (bmp.performReading()) {
        Serial.printf(
          "[BMP TEST] TEMP=%.2f C  PRESSURE=%.2f hPa\n",
          bmp.temperature,
          bmp.pressure);
      } else {
        Serial.println("[BMP TEST] performReading FAILED");
      }

      delay(100);
    }
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

  // ---------------------------------------------------------
  // FANET radio (HT-RA62 / SX1262). Own SPI bus (HSPI -- see the
  // Sx126xLink constructor) -- independent of the display's SPI.begin()
  // above, so order relative to that doesn't matter.
  //
  // Only actually brought up if fanetEnabled (settings.h, loaded above
  // via loadSettings()) is true -- Config > FANET lets the pilot turn
  // the chip off entirely for bench testing. If it's off at boot,
  // fanetRadioOK stays false and setFanetEnabled() below runs this same
  // init the first time it's switched on from the menu.
  // ---------------------------------------------------------
  if (fanetEnabled) {
    fanetRadioOK = fanetRadio.begin(/*freqMHz=*/868.2f, /*bwKHz=*/250.0f,
                                     /*sf=*/7, /*cr=*/5, /*syncWord=*/0xF1,
                                     /*powerDbm=*/14, /*preambleLen=*/8);
    if (fanetRadioOK) {
      fanet.begin();
      fanet.onTracking(onFanetTracking);
      fanet.onWeather(onFanetWeather);
      fanet.onMessage(onFanetMessageReceived);
      fanet.setBeaconIntervalMs(FANET_BEACON_INTERVAL_MS);
      Serial.println("FANET RADIO INITIALIZED");
    } else {
      Serial.printf("FANET RADIO INIT FAILED -- status=%d (FANET disabled)\n",
                    fanetRadio.lastStatus());
    }
  } else {
    Serial.println("FANET disabled (Config > FANET) -- skipping radio init");
  }

  // ---------------------------------------------------------
  // WiFi: kicked off here but no longer BLOCKS setup() waiting for it.
  // WiFi.begin() is async by nature -- wifiManagerLoop(), which already
  // runs continuously from backgroundTask() on Core 0, picks up the
  // moment WiFi.status() flips to WL_CONNECTED and takes over from there
  // (retries, drop detection, etc), exactly as it already does for a
  // connection that drops mid-flight. This used to cost up to 10 full
  // seconds of dead time here if the saved network wasn't immediately
  // reachable -- the single biggest contributor to the slow boot.
  // ---------------------------------------------------------
  loadWifiSettings();
  startWifiConnect();
  Serial.println("[BOOT] WiFi connect started in background");

  // Brings up the BLE stack and, if a device was remembered from a
  // previous session (e.g. the engine meter), turns Bluetooth on and
  // starts looking for it -- see ble_manager.cpp.
  loadBleSettings();

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

  // Enable the speaker amp ONCE here and leave it enabled for the rest of
  // the flight (see AMP_ENABLE_PIN comments in updateI2sAudioBuzzer() for
  // why -- toggling it on/off per beep was clipping/silencing every short
  // tone). "No sound" is produced by writing silence over I2S, not by
  // powering the amp down.
  if (codecOK && es8311OK) {
    digitalWrite(AMP_ENABLE_PIN, HIGH);
  }

  pinMode(KEY_PIN, INPUT_PULLUP);

  // ---------------------------------------------------------
  // Hold the splash on screen for at least SPLASH_DISPLAY_MS total,
  // measured from when it was actually drawn -- not a flat delay tacked
  // on top of everything above. All the init above has already been
  // "spent" against that time, so most boots see little or no extra
  // wait here at all.
  // ---------------------------------------------------------
  unsigned long splashElapsedMs = millis() - splashDrawnAt;
  if (splashElapsedMs < SPLASH_DISPLAY_MS) {
    delay(SPLASH_DISPLAY_MS - splashElapsedMs);
  }
  //esp_task_wdt_reset(); // feed the watchdog after the splash hold, before any blocking HTTP work
  Serial.println("[BOOT] Splash hold complete");

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
  // Independent audio servicing -- i2sToneService() does not run from
  // loop(), so a slow display redraw or a network call can never starve
  // the DMA buffer. See AUDIO_USE_DEDICATED_TASK above for the two
  // variants.
  // ---------------------------------------------------------
#if AUDIO_USE_DEDICATED_TASK
  {
    // Priority 10: well above loop() (priority 1) so loop() can never delay
    // it, but it spends nearly all of its time blocked in i2s_write().
    // Pinned to Core 1 with loop(), away from the Core 0 network/WiFi
    // activity.
    BaseType_t audioTaskCreated = xTaskCreatePinnedToCore(
      audioServiceTask,
      "audio_svc",
      4096,
      nullptr,
      10,
      nullptr,
      1
    );
    if (audioTaskCreated == pdPASS) {
      Serial.println("[BOOT] Audio service task started (Core 1)");
    } else {
      Serial.println("[BOOT] Failed to create audio service task");
    }
  }
#else
  const esp_timer_create_args_t audioTimerConfig = {
    .callback = [](void*) {
      i2sToneService();
    },
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
#endif

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
  // 3.5 FANET -- feed current position/state, then let the stack handle
  //     its own RX polling and beacon schedule. update() is cheap (an
  //     SPI status read plus a millis() check) so it's fine to call every
  //     pass rather than duty-cycling it separately.
  // ---------------------------------------------------------
  if (fanetRadioOK && fanetEnabled && gps.location.isValid() && gps.location.age() < 2000) {
    // Prefer the QNH-calibrated baro altitude once available, same
    // preference order already used for sharedPosition above.
    int32_t altM = qnhCalibrated ? (int32_t)lroundf(currentAltitudeM)
                                  : (int32_t)lroundf(gps.altitude.meters());
    float speedKmh = gps.speed.isValid() ? gps.speed.kmph() : 0.0f;
    float headingDeg = gps.course.isValid() ? gps.course.deg() : 0.0f;

    fanet.setPosition(gps.location.lat(), gps.location.lng(), altM,
                       speedKmh, currentClimbRateMS, headingDeg);
    // 1 = paraglider, 5 = powered aircraft -- closest fit in the FANET
    // spec's 3-bit type field for a paramotor (there's no dedicated
    // "powered paraglider" slot). Was previously
    // "currentPage == PAGE_PARAMOTOR ? 1 : 1" -- both branches evaluated
    // to the same value, so the page selector never actually did
    // anything. Worth checking this choice of 5 against how other FANET
    // implementations (SoftRF etc.) tag powered paragliders before
    // relying on it for real traffic.
    fanet.setAircraftType(currentPage == PAGE_PARAMOTOR ? 5 : 1);
  }
  if (fanetRadioOK && fanetEnabled) {
    fanet.update();
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
    if (adsbAutoJumpEnabled) {
      jumpToActivePage(PAGE_ADSB);
    }
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
      currentHumidityPct = humidity.relative_humidity;
      updateCloudBaseEstimate();
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
// CLOUD BASE ESTIMATE
// Dew point comes from the SHTC3's own temp + humidity (they must come from
// the same sensor). The air temp is corrected for board self-heating, then
// combined with the BMP580 pressure to find the lifting condensation level.
// Result is metres from where you are up to cloud base, low-pass filtered.
// =====================================================
void updateCloudBaseEstimate() {
  if (isnan(currentTempC) || isnan(currentHumidityPct) || isnan(currentPressureHpa)) return;

  float tdC = cloudDewPointC(currentTempC, currentHumidityPct);
  float tAirC = currentTempC - SHT_SELF_HEAT_OFFSET_C;
  float rawM = cloudBaseHeightM(tAirC, tdC, currentPressureHpa);
  if (isnan(rawM)) return;

  if (isnan(cloudBaseAboveM)) {
    cloudBaseAboveM = rawM;
  } else {
    cloudBaseAboveM += CLOUD_BASE_SMOOTH_ALPHA * (rawM - cloudBaseAboveM);
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
  pageBeepGain = 1.0f;
  toneGain = pageBeepGain;  // gain first, then frequency (see updateI2sAudioBuzzer())
  setToneFrequency(freq);
  pageBeepUntil = millis() + durationMs;
}

// Same as playFeedbackTone(), but at the loudness a given climb/sink volume
// percentage (0-100) would produce -- used by Config > Vario Beep so the
// pilot can audition a volume level as they pick it.
void playVolumePreviewTone(float freq, unsigned long durationMs, uint8_t volumePercent) {
  if (buzzerMuted) return;
  pageBeepGain = varioVolumeToGain(volumePercent);
  toneGain = pageBeepGain;  // gain first, then frequency (see updateI2sAudioBuzzer())
  setToneFrequency(freq);
  pageBeepUntil = millis() + durationMs;
}

// Climb/sink volume persistence. Stored in the same NVS namespace
// ("vario") as the rest of the settings, under their own keys, so this is
// independent of loadSettings()/saveSettings(). Call loadVarioVolumes()
// once at boot, after loadSettings(); call saveVarioVolumes() after either
// value changes.
//
// This also does the one-time conversion of the master volume
// (buzzerVolumePercent, normally loaded by loadSettings()) from the old
// linear-register scale to the current dB scale -- see the BUZZER VOLUME
// comment in settings.h. A "volScheme" key marks it as done.
static uint8_t migrateOldBuzzerVolumePercent(uint8_t oldPercent) {
  if (oldPercent > 100) oldPercent = 100;
  // What register value (and so what dB) the old scheme produced.
  const int oldReg = (int)((oldPercent / 100.0f) * 255.0f + 0.5f);
  const float oldDb = (oldReg - 0xBF) * 0.5f;
  // Nearest 10% step on the new scale.
  const float stepsBelowMax = (BUZZER_VOLUME_MAX_DB - oldDb) / BUZZER_VOLUME_DB_PER_STEP;
  int pct = 100 - (int)lroundf(stepsBelowMax) * 10;
  if (pct < 10) pct = 10;
  if (pct > 100) pct = 100;
  return (uint8_t)pct;
}

void loadVarioVolumes() {
  uint8_t volScheme = 0;
  Preferences prefs;
  if (prefs.begin("vario", false)) {
    climbVolumePercent = prefs.getUChar("climbVol", VARIO_VOLUME_DEFAULT_CLIMB_PERCENT);
    sinkVolumePercent = prefs.getUChar("sinkVol", VARIO_VOLUME_DEFAULT_SINK_PERCENT);
    volScheme = prefs.getUChar("volScheme", 0);
    prefs.end();
  }
  if (climbVolumePercent > 100) climbVolumePercent = 100;
  if (sinkVolumePercent > 100) sinkVolumePercent = 100;

  if (volScheme < 1) {
    // First boot on the dB-scaled master volume: convert the old value.
    buzzerVolumePercent = migrateOldBuzzerVolumePercent(buzzerVolumePercent);
    saveSettings();
    if (prefs.begin("vario", false)) {
      prefs.putUChar("volScheme", 1);
      prefs.end();
    }
  } else {
    // Keep it on a valid menu step (10-100%, multiples of 10).
    int pct = ((int)buzzerVolumePercent + 5) / 10 * 10;
    if (pct < 10) pct = 10;
    if (pct > 100) pct = 100;
    buzzerVolumePercent = (uint8_t)pct;
  }
}

void saveVarioVolumes() {
  Preferences prefs;
  if (prefs.begin("vario", false)) {
    prefs.putUChar("climbVol", climbVolumePercent);
    prefs.putUChar("sinkVol", sinkVolumePercent);
    prefs.end();
  }
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
          } else {
            menuGoBack();  // step back one menu level, or close if already at the top
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
      saveSettings();
      pageBeepUntil = 0;
      setToneFrequency(0);
      beepOn = false;

      // Kick off the confirmation jingle -- actually sequenced
      // non-blockingly in updateI2sAudioBuzzer(). buzzerMuted already
      // holds the *new* state here, which is exactly the direction we
      // want to play.
      muteToneActive = true;
      muteToneStart = millis();
      muteToneIsMuteSequence = buzzerMuted;

      if (buzzerMuted) {
        // Muting: leave the amp powered through the jingle so it's
        // actually audible -- it gets switched off only once the jingle
        // finishes, in updateI2sAudioBuzzer().
      } else {
        // Unmuting: turn the amp on immediately so the jingle is audible
        // right away.
        digitalWrite(AMP_ENABLE_PIN, HIGH);
      }
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

  // Snapshot the live GPS fix -- same mutex pattern used for the DEM/
  // airspace scans further down loop(). Without a valid fix there's no
  // sensible position to query adsb.fi around, so skip this poll cycle
  // rather than falling back to a stale or default position.
  PositionSnapshot myPos;
  if (backgroundDataMutex != nullptr && xSemaphoreTake(backgroundDataMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
    myPos = sharedPosition;
    xSemaphoreGive(backgroundDataMutex);
  }
  if (!myPos.valid) {
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
        myPos.lat,
        myPos.lon,
        acLat,
        acLon);

    float verticalDeltaFeet =
      fabsf(
        (float)ac["alt_baro"] - myAltitudeFeet);

    if (distanceKM <= adsbAlertRadiusKm && verticalDeltaFeet <= adsbAlertVerticalFt) {

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
    // Wi-Fi + Bluetooth connection management -- the connect/retry state
    // machines themselves now live in wifi_manager.cpp/ble_manager.cpp
    // (this used to all be inline here).
    // ---------------------------------------------------------
    wifiManagerLoop();
    bleManagerLoop();

    // Export Files (Flight Recordings > Export Files) -- serves IGC
    // files over WiFi when running. Deliberately on Core 0, not in the
    // main loop() on Core 1: handleClient() can block for a noticeable
    // stretch while streaming a file to a slow/distant client, and Core
    // 1 must never be delayed (see this function's header comment).
    // No-ops immediately if the server isn't currently started.
    fileServerLoop();

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
      // every poll after that reverts to the menu-adjustable weatherPollIntervalMs cadence.
      unsigned long weatherDueInterval = weatherFirstPollDone ? weatherPollIntervalMs : WEATHER_FIRST_POLL_DELAY_MS;

      if (now - weatherTimerAnchor >= weatherDueInterval) {
        weatherTimerAnchor = now;
        weatherFirstPollDone = true;
        Serial.println("[MAIN] Calling weather update...");
        updateWeather();
      }
    }

    // ---------------------------------------------------------
    // Ground elevation (AGL): local SD file only, no WiFi needed. Runs
    // before the airspace scan below so a fresh groundElevationFt is
    // available for this same cycle's AGL-referenced airspace floors.
    // ---------------------------------------------------------
    if (sdCardOK && now - demScanAnchor >= DEM_SCAN_INTERVAL_MS) {
      demScanAnchor = now;

      PositionSnapshot demPos;
      if (backgroundDataMutex != nullptr && xSemaphoreTake(backgroundDataMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
        demPos = sharedPosition;
        xSemaphoreGive(backgroundDataMutex);
      }

      if (demPos.valid) {
        if (sdMutex != nullptr && xSemaphoreTake(sdMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
          float elevM;
          // getGroundElevationM() (TerrainDem.h) yields periodically via
          // vTaskDelay() while seeking into the DEM file now, so a deep
          // first-time seek into a large tile can't starve IDLE0 on core 0
          // long enough to trip the task watchdog. See seekWithYield() in
          // that file for why esp_task_wdt_reset() couldn't fix this
          // (BackgroundTask was never watchdog-registered in the first
          // place -- the trip was always about IDLE0, not this task).
          bool found = getGroundElevationM(selectedDemFile, demPos.lat, demPos.lon, elevM);
          xSemaphoreGive(sdMutex);

          if (found) {
            groundElevationFt = elevM * 3.28084f;
            groundElevationValid = true;
          } else {
            groundElevationValid = false;  // outside tile / no DEM loaded
          }
        } else {
          Serial.println("[DEM] SD busy -- lookup skipped this cycle");
        }
      }
    }

    // ---------------------------------------------------------
    // Airspace proximity: findNearestControlledAirspace() searches the
    // in-RAM cache only (see OpenAirScanner.h -- no SD access, no
    // parsing), so unlike the DEM lookup above this does NOT need
    // sdMutex. It previously took sdMutex here anyway, which meant it
    // competed with the IGC logger for the same lock every 10s for no
    // reason -- that contention (not actual SD I/O) was why scans were
    // being skipped.
    // ---------------------------------------------------------
    if (sdCardOK && now - airspaceScanAnchor >= AIRSPACE_SCAN_INTERVAL_MS) {
      airspaceScanAnchor = now;

      PositionSnapshot pos;
      if (backgroundDataMutex != nullptr && xSemaphoreTake(backgroundDataMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
        pos = sharedPosition;
        xSemaphoreGive(backgroundDataMutex);
      }

      if (pos.valid) {
        AirspaceResult nearest;

        // groundElevationValid is read here, not just groundElevationFt --
        // this is what stops an AGL/SFC-referenced floor being resolved
        // against a stale ground elevation once the aircraft has flown
        // outside the loaded DEM tile. See AirspaceResult::vertKnown.
        //
        // alertOnly = true -- this feeds the proximity/entry ALERT (top
        // banner + tone) and the Paraglider/Paramotor pages' AIR SPACE
        // box, neither of which should ever trigger for a CFZ.
        bool found = findNearestControlledAirspace(
          pos.lat,
          pos.lon,
          pos.altFt,
          groundElevationFt,
          groundElevationValid,
          /*alertOnly=*/true,
          nearest
        );

        // A second, independent search -- alertOnly = false, so a CFZ
        // (or anything else in the cache) can be returned. Feeds ONLY
        // the ADS-B page's "Airspace Info" bar (Config > ADS-B Settings
        // > Airspace Info), which exists specifically to surface a
        // CFZ's name/frequency even though it must never trigger the
        // alert above.
        AirspaceResult nearestInfo;

        bool foundInfo = findNearestControlledAirspace(
          pos.lat,
          pos.lon,
          pos.altFt,
          groundElevationFt,
          groundElevationValid,
          /*alertOnly=*/false,
          nearestInfo
        );

        if (backgroundDataMutex != nullptr && xSemaphoreTake(backgroundDataMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
          if (found) {
            nearestAirspace = nearest;
            airspaceResultValid = true;
          } else {
            // No controlled airspace nearby right now -- clear any stale
            // result so the UI doesn't keep showing the last hit after
            // the pilot has flown clear of it.
            airspaceResultValid = false;
          }

          if (foundInfo) {
            nearestAirspaceInfo = nearestInfo;
            airspaceInfoResultValid = true;
          } else {
            airspaceInfoResultValid = false;
          }

          xSemaphoreGive(backgroundDataMutex);
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
  // DEM/airspace scans and performADSBUpdate().
  // ---------------------------------------------------------
  PositionSnapshot myPos;
  if (backgroundDataMutex != nullptr && xSemaphoreTake(backgroundDataMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
    myPos = sharedPosition;
    xSemaphoreGive(backgroundDataMutex);
  }
  if (!myPos.valid) {
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
//=====================================================
//VARIO: sample baro, push into regression window, compute climb rate
//=====================================================
void updateVario() {
  // Temporary throttled diagnostics -- ALTITUDE/CLIMB RATE only need
  // bmpOK && windowCount > 0, which is completely independent of GPS,
  // so if those boxes are stuck on "--" the cause has to be here: either
  // dataReady()/performReading() failing every call, or this function
  // not running at all. Prints at most once every 2s so it won't flood
  // the serial monitor. Safe to remove once the real cause is found.
  static unsigned long lastVarioDebug = 0;
  bool debugNow = (millis() - lastVarioDebug >= 2000);


  if (!bmp.performReading()) {
    if (debugNow) {
      Serial.println("[VARIO DEBUG] bmp.performReading() FAILED -- skipping this cycle");
      lastVarioDebug = millis();
    }
    return;
  }

  currentPressureHpa = bmp.pressure;

  bool gpsAltitudeGood =
    gps.altitude.isValid() && gps.altitude.age() < 2000 && gps.satellites.isValid() && gps.satellites.value() >= 6 && gps.hdop.isValid() && gps.hdop.hdop() <= 2.5;

  // Runs the real GPS-derived calibration the first time a good fix
  // shows up, AND -- if we're currently sitting on the no-GPS fallback
  // value -- also the first time a good fix shows up *after* that, so a
  // merely-slow GPS still gets upgraded to a proper calibration instead
  // of being stuck on 1013.25 for the rest of the flight. AND, once
  // genuinely calibrated (qnhIsFallback == false), again every
  // QNH_RECALIBRATION_INTERVAL_MS from that point on, so QNH keeps
  // tracking real atmospheric pressure changes over a long flight
  // instead of staying frozen at whatever it was on takeoff.
  //
  // Averages GPS altitude over QNH_GPS_AVERAGE_MS of continuous
  // good-quality fixes rather than calibrating off a single instantaneous
  // sample -- see QNH_GPS_AVERAGE_MS's comment above for why a one-shot
  // sample proved unreliable in real-world testing. qnhAvgActive/
  // qnhAvgStartMs/qnhAvgAltSum/qnhAvgCount persist this averaging window
  // across calls; static rather than global since nothing outside this
  // function needs them. lastQnhCalibrationMs tracks when calibration
  // last actually succeeded, so the periodic-recalibration check below
  // has something to measure from.
  static bool qnhAvgActive = false;
  static unsigned long qnhAvgStartMs = 0;
  static double qnhAvgAltSum = 0.0;
  static uint16_t qnhAvgCount = 0;
  static unsigned long lastQnhCalibrationMs = 0;

  bool qnhRecalibrationDue =
    qnhCalibrated && !qnhIsFallback &&
    (millis() - lastQnhCalibrationMs >= QNH_RECALIBRATION_INTERVAL_MS);

  if (gpsAltitudeGood && (!qnhCalibrated || qnhIsFallback || qnhRecalibrationDue)) {

    if (!qnhAvgActive) {
      // First good fix, or the periodic recalibration interval just
      // came due -- start a fresh averaging window.
      qnhAvgActive = true;
      qnhAvgStartMs = millis();
      qnhAvgAltSum = 0.0;
      qnhAvgCount = 0;
    }

    // Only bank a sample once per fresh GPS sentence. isUpdated() clears
    // itself on read, so this can't double-count the same fix just
    // because updateVario() runs far more often than the GPS module
    // actually reports a new position (typically 1Hz) -- without this
    // check, a single fix held between GPS updates would get counted
    // once per updateVario() call and dominate the average.
    if (gps.altitude.isUpdated()) {
      qnhAvgAltSum += gps.altitude.meters();
      qnhAvgCount++;
    }

    if (debugNow && qnhAvgActive) {
      Serial.printf("[VARIO DEBUG] QNH averaging: %u samples over %lus/%lus\n",
                    qnhAvgCount, (millis() - qnhAvgStartMs) / 1000UL, QNH_GPS_AVERAGE_MS / 1000UL);
    }

    if (millis() - qnhAvgStartMs >= QNH_GPS_AVERAGE_MS) {

      if (qnhAvgCount >= QNH_GPS_MIN_SAMPLES) {

        float gpsAltM = (float)(qnhAvgAltSum / qnhAvgCount);

        float calculatedQNH =
          bmp.pressure / powf(1.0f - (gpsAltM / 44330.0f), 1.0f / 0.1903f);

        if (calculatedQNH >= 850.0f && calculatedQNH <= 1100.0f) {

          bool wasFallback = qnhIsFallback;
          currentQNH = calculatedQNH;
          qnhCalibrated = true;
          qnhIsFallback = false;
          lastQnhCalibrationMs = millis();

          if (wasFallback) {
            Serial.print("QNH upgraded from GPS altitude (fallback replaced): ");
          } else if (qnhRecalibrationDue) {
            Serial.print("QNH re-calibrated from GPS altitude (15-minute refresh): ");
          } else {
            Serial.print("QNH calibrated from GPS altitude: ");
          }
          Serial.printf("%.2f (averaged over %u fixes / %lus)\n",
                        currentQNH, qnhAvgCount, QNH_GPS_AVERAGE_MS / 1000UL);

        } else {
          Serial.printf("[VARIO] Averaged GPS altitude produced an implausible QNH (%.1f) -- discarding, retrying\n", calculatedQNH);
        }

      } else {
        Serial.printf("[VARIO] QNH averaging window elapsed with only %u fresh fixes (need %u) -- retrying\n",
                      qnhAvgCount, QNH_GPS_MIN_SAMPLES);
      }

      // Reset either way -- a successful non-recalibration calibration
      // means this whole block won't fire again until the next
      // QNH_RECALIBRATION_INTERVAL_MS comes due (the qnhCalibrated &&
      // !qnhIsFallback && recalibration-due check above); a failed/
      // underfilled window just starts a fresh attempt on the next good
      // fix, same as before.
      qnhAvgActive = false;
    }

  } else {

    // Either GPS quality isn't good enough right now, or we're already
    // calibrated and not yet due for a recalibration -- either way,
    // abandon any in-progress averaging window rather than let a
    // dropped-out stretch silently count toward it. A later good fix
    // starts a fresh window from scratch. (No-op most of the time once
    // already calibrated, since qnhAvgActive is already false between
    // recalibration windows.)
    qnhAvgActive = false;

    if (!qnhCalibrated && millis() >= GPS_QNH_FALLBACK_MS) {
      // GPS never came good (missing/unwired module, or just no fix
      // after a full minute) -- stop waiting on it. Default to standard
      // atmosphere so the BMP580 alone can drive altitude/vario for the
      // rest of the flight. Flagged as a fallback so a later good fix
      // can still upgrade it, above.
      currentQNH = SEA_LEVEL_QNH_DEFAULT;
      qnhCalibrated = true;
      qnhIsFallback = true;

      Serial.println("GPS unavailable -- defaulting QNH to 1013.25, running altitude/vario off BMP580 only");
    }
  }

  float newAltitudeM = bmp.readAltitude(currentQNH);

  // ------------------------------------------------------------------
  // Outlier rejection -- guards against a single corrupted I2C read
  // (suspected cause: RF coupling into the SDA/SCL wiring from the
  // FANET radio during a TX burst) poisoning the climb-rate window.
  // A paraglider physically can't jump more than a few m/s between two
  // consecutive ~BARO_SAMPLE_MS-apart samples, so anything wildly
  // outside that is treated as a bad sample and dropped rather than
  // fed into the regression. Tune REJECT_RATE_MS if this ever proves
  // too tight/loose in practice.
  // ------------------------------------------------------------------
  static constexpr float REJECT_RATE_MS = 15.0f;  // m/s

  if (windowCount > 0) {
    int lastIdx = (windowIndex + CLIMB_WINDOW_N - 1) % CLIMB_WINDOW_N;
    float dt = (millis() - timeWindow[lastIdx]) / 1000.0f;

    if (dt > 0.001f) {
      float impliedRateMS = (newAltitudeM - altWindow[lastIdx]) / dt;

      if (fabsf(impliedRateMS) > REJECT_RATE_MS) {
        if (debugNow) {
          Serial.printf(
            "[VARIO DEBUG] Rejected outlier sample: alt=%.1f m implied=%.1f m/s "
            "(last alt=%.1f m, dt=%.3f s) -- keeping previous window\n",
            newAltitudeM, impliedRateMS, altWindow[lastIdx], dt);
          lastVarioDebug = millis();
        }
        return;
      }
    }
  }

  currentAltitudeM = newAltitudeM;

  altWindow[windowIndex] = currentAltitudeM;
  timeWindow[windowIndex] = millis();
  windowIndex = (windowIndex + 1) % CLIMB_WINDOW_N;
  if (windowCount < CLIMB_WINDOW_N) windowCount++;

  if (windowCount >= 3) {
    currentClimbRateMS = computeClimbRateLeastSquares();
  }

  if (debugNow) {
    Serial.printf(
      "[VARIO DEBUG] OK -- pressure=%.2f hPa, QNH=%.2f (calibrated=%d, fallback=%d), "
      "altitude=%.1f m, windowCount=%d, climb=%.2f m/s\n",
      bmp.pressure, currentQNH, qnhCalibrated, qnhIsFallback,
      currentAltitudeM, windowCount, currentClimbRateMS);
    lastVarioDebug = millis();
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
// ============================================================================
// BlueFly-style vario beep duration
// ============================================================================
//
// Approximates the cadence curve of the Kobo BlueFly sample/defaults.
// As lift increases, the beep becomes shorter and the cadence increases.
//
// Input:  current filtered climb rate in m/s
// Output: beep duration in milliseconds
//
static unsigned long blueflyBeepDurationMs(float climbMs){
  climbMs = max(0.0f, climbMs);

  if (climbMs <= 0.20f) {
    return 380UL;
  }

  if (climbMs <= 0.50f) {
    return (unsigned long)(
      380.0f +
      (climbMs - 0.20f) *
      (300.0f - 380.0f) / 0.30f
    );
  }

  if (climbMs <= 1.00f) {
    return (unsigned long)(
      300.0f +
      (climbMs - 0.50f) *
      (200.0f - 300.0f) / 0.50f
    );
  }

  if (climbMs <= 1.50f) {
    return (unsigned long)(
      200.0f +
      (climbMs - 1.00f) *
      (140.0f - 200.0f) / 0.50f
    );
  }

  if (climbMs <= 2.00f) {
    return (unsigned long)(
      140.0f +
      (climbMs - 1.50f) *
      (100.0f - 140.0f) / 0.50f
    );
  }

  if (climbMs <= 3.00f) {
    return (unsigned long)(
      100.0f +
      (climbMs - 2.00f) *
      (65.0f - 100.0f) / 1.00f
    );
  }

  return 55UL;
}
// =====================================================
// BUZZER: non-blocking climb/sink tone over the I2S speaker.
// Yields to the page-change beep for its short duration rather than
// talking over it -- both share the same physical speaker.
// =====================================================
//===================================================================
// =====================================================
// BLUEFLY-STYLE VARIO AUDIO
// =====================================================
//
// The actual waveform is still generated by i2sToneService().
// This function only controls:
//
//   - when the vario beeps
//   - when it is silent
//   - the pitch of the beep
//
// Existing page beep, ADS-B alarm, mute confirmation and sink alarm
// behaviour are preserved.
// =====================================================

void updateI2sAudioBuzzer(){
  const unsigned long now = millis();

  // NOTE ON toneGain: this function runs in loop() while the audio side
  // reads toneGain/toneFrequency asynchronously from another task. So it
  // must NEVER be written to a temporary value (e.g. "reset to 1.0 at the
  // top, then set the real value later") -- the audio side could catch the
  // temporary value for a chunk and play a burst at the wrong loudness.
  // Instead, every branch below that sounds a tone sets its own final gain
  // immediately BEFORE setting the frequency, and branches that are silent
  // leave it alone.


  // ============================================================
  // PAGE-CHANGE BEEP HAS PRIORITY
  // ============================================================

  if ((int32_t)(pageBeepUntil - now) > 0) {
    toneGain = pageBeepGain;
    return;
  }

  // Page beep has just finished
  if (pageBeepUntil != 0) {

    pageBeepUntil = 0;

    setToneFrequency(0);

    sinkAlarmActive = false;
    climbAudioActive = false;
    climbToneOn = false;
  }


  // ============================================================
  // ADS-B INTERCEPT ALARM
  // ============================================================

  if (interceptAlarmActive) {

    unsigned long elapsed =
      now - interceptAlarmStart;

    if (elapsed >= INTERCEPT_ALARM_DURATION_MS) {

      interceptAlarmActive = false;

      setToneFrequency(0);

      sinkAlarmActive = false;
      climbAudioActive = false;
      climbToneOn = false;

    }
    else if (!adsbAlarmMuted) {

      unsigned long phase =
        elapsed % (INTERCEPT_TONE_TOGGLE_MS * 2);

      float freq =
        (phase < INTERCEPT_TONE_TOGGLE_MS)
        ? INTERCEPT_TONE_HIGH_HZ
        : INTERCEPT_TONE_LOW_HZ;

      toneGain = 1.0f;  // the alarm is always full level
      setToneFrequency(freq);

      return;
    }
  }


  // ============================================================
  // MUTE / UNMUTE CONFIRMATION TONE
  // ============================================================

  if (muteToneActive) {

    const unsigned long elapsed = now - muteToneStart;

    const ToneStep* seq =
      muteToneIsMuteSequence ? MUTE_SEQUENCE : UNMUTE_SEQUENCE;

    const size_t seqLen =
      muteToneIsMuteSequence
      ? TONE_SEQ_LEN(MUTE_SEQUENCE)
      : TONE_SEQ_LEN(UNMUTE_SEQUENCE);

    // Walk the table, accumulating durations, until we find the step
    // that covers the current elapsed time.
    unsigned long stepEnd = 0;
    bool stepFound = false;

    for (size_t i = 0; i < seqLen; i++) {

      stepEnd += seq[i].ms;

      if (elapsed < stepEnd) {

        toneGain = seq[i].gain;
        setToneFrequency(seq[i].freq);
        stepFound = true;
        break;
      }
    }

    if (stepFound) {
      return;
    }

    // Sequence finished.
    muteToneActive = false;

    setToneFrequency(0);

    if (buzzerMuted) {
      digitalWrite(AMP_ENABLE_PIN, LOW);
    }

    // Fall through to normal vario logic.
  }


  // ============================================================
  // MUTED
  // ============================================================

  if (buzzerMuted) {

    setToneFrequency(0);

    sinkAlarmActive = false;
    climbAudioActive = false;
    climbToneOn = false;

    return;
  }


  // ============================================================
  // SINK ALARM WITH HYSTERESIS
  // ============================================================

  const float SINK_ALARM_RELEASE_MS = -0.2f;

  if (!sinkAlarmActive) {

    if (currentClimbRateMS <= SINK_ALARM_MS) {

      sinkAlarmActive = true;
      sinkAlarmStart = now;

      climbAudioActive = false;
      climbToneOn = false;
    }

  }
  else {

    if (currentClimbRateMS > SINK_ALARM_RELEASE_MS) {

      sinkAlarmActive = false;

      setToneFrequency(0);
    }
  }


  // ============================================================
  // SINK ALARM OUTPUT
  // ============================================================

  if (sinkAlarmActive) {

    // BlueFly sink pitch: base minus increment per m/s of sink.
    // currentClimbRateMS is negative here, so this drops as sink grows.
    float sinkToneFreq =
      SINK_FREQ_BASE_HZ +
      (SINK_FREQ_INCREMENT_HZ * currentClimbRateMS);

    sinkToneFreq = max(SINK_FREQ_MIN_HZ, sinkToneFreq);

    toneGain = varioVolumeToGain(sinkVolumePercent);
    setToneFrequency(sinkToneFreq);

    return;
  }


  // ============================================================
  // CLIMB DEAD BAND
  // ============================================================

  if (currentClimbRateMS <= CLIMB_DEADBAND_MS) {

    climbAudioActive = false;
    climbToneOn = false;

    setToneFrequency(0);

    return;
  }


  // ============================================================
  // ENTERING CLIMB AUDIO
  // ============================================================

  if (!climbAudioActive) {

    climbAudioActive = true;
    climbToneOn = false;

    // Start the first beep immediately.
    climbPulseStart = now;

    setToneFrequency(0);
  }


  // ============================================================
  // BLUEFLY PITCH
  // ============================================================
  //
  // Approximately:
  //
  //   0.2 m/s = 1020 Hz
  //   0.5 m/s = 1050 Hz
  //   1.0 m/s = 1100 Hz
  //   2.0 m/s = 1200 Hz
  //   3.0 m/s = 1300 Hz
  //   4.0 m/s = 1400 Hz
  //   5.0 m/s = 1500 Hz
  //
  // The ES8311/speaker produces the harmonics that give the audible
  // BlueFly character.
  // ============================================================

  float blueflyFreq =
    1000.0f +
    (currentClimbRateMS * 100.0f);

  blueflyFreq =
    constrain(
      blueflyFreq,
      1000.0f,
      1800.0f
    );

  // Climb volume (Config > Vario Beep) -- applies to every beep below.
  toneGain = varioVolumeToGain(climbVolumePercent);


  // ============================================================
  // CURRENT BEEP DURATION
  // ============================================================

  unsigned long beepMs =
    blueflyBeepDurationMs(currentClimbRateMS);


  // ============================================================
  // BEEP CURRENTLY PLAYING
  // ============================================================

  if (climbToneOn) {

    // ----------------------------------------------------------
    // IMPORTANT:
    //
    // Update pitch continuously while the beep is playing.
    // ----------------------------------------------------------

    setToneFrequency(blueflyFreq);


    // ----------------------------------------------------------
    // Has this beep finished?
    // ----------------------------------------------------------

    if ((unsigned long)(now - climbPulseStart) >= beepMs) {

      climbToneOn = false;

      setToneFrequency(0);

      // --------------------------------------------------------
      // BlueFly-style silence period.
      //
      // Using the same basic duration as the current beep gives
      // the characteristic approximately 50/50 cadence in the
      // moderate-lift region of the sample.
      // --------------------------------------------------------

      climbPulseStart = now;
    }

    return;
  }


  // ============================================================
  // CURRENTLY SILENT
  // ============================================================

  // Use the same BlueFly duration for the silence.
  unsigned long silenceMs =
    blueflyBeepDurationMs(currentClimbRateMS);


  if ((unsigned long)(now - climbPulseStart) >= silenceMs) {

    climbToneOn = true;

    climbPulseStart = now;

    setToneFrequency(blueflyFreq);
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
// Counts I2C writes to the codec that still failed after retries (see below).
static uint16_t es8311WriteFailures = 0;

// Writes one codec register, retrying if the I2C transfer isn't ACKed.
// Espressif's own ES8311 driver notes that the first I2C write to this chip
// occasionally fails, and the original version of this function ignored the
// result -- a dropped write meant a register silently kept a stale/default
// value for the whole session. Returns true if the write was ACKed.
static bool es8311WriteRegChecked(uint8_t reg, uint8_t value) {
  for (uint8_t attempt = 0; attempt < 3; attempt++) {
    Wire.beginTransmission(ES8311_I2C_ADDR);
    Wire.write(reg);
    Wire.write(value);
    if (Wire.endTransmission() == 0) return true;
    delay(2);
  }
  es8311WriteFailures++;
  return false;
}

void es8311WriteReg(uint8_t reg, uint8_t value) {
  es8311WriteRegChecked(reg, value);
}

// Returns the register's value, or -1 if the read failed.
static int es8311ReadReg(uint8_t reg) {
  Wire.beginTransmission(ES8311_I2C_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return -1;
  if (Wire.requestFrom((uint8_t)ES8311_I2C_ADDR, (uint8_t)1) != 1) return -1;
  return Wire.read();
}

// One full pass of the codec register setup. Split out from es8311Init() so
// the whole sequence can be repeated if the read-back check fails.
static void es8311ConfigureRegisters() {
  // Full reset first, as Espressif's/ESPHome's drivers do. The codec is NOT
  // reset when only the ESP32 restarts (it keeps its previous register
  // contents until its own supply is fully removed), so without this a
  // restart configured the codec on top of whatever state the last run left
  // behind -- matching "scrambled until switched off for a while".
  es8311WriteReg(0x44, 0x08);  // "I2C noise immunity" -- Espressif writes this twice because
  es8311WriteReg(0x44, 0x08);  // the first write to the chip occasionally fails
  es8311WriteReg(0x00, 0x1F);  // reset all digital blocks
  delay(20);
  es8311WriteReg(0x00, 0x00);  // release reset

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
}

// Reads back the registers that decide whether the codec is running on the
// right clocks/format. Returns true if they all hold what was written.
static bool es8311VerifyRegisters() {
  static const struct { uint8_t reg; uint8_t expected; } checks[] = {
    { 0x01, 0x3F },  // all internal clocks enabled
    { 0x06, 0x03 },  // BCLK divider
    { 0x07, 0x00 },  // LRCK divider (high)
    { 0x08, 0xFF },  // LRCK divider (low)
    { 0x09, 0x00 },  // serial port: I2S, 16-bit
  };
  bool allGood = true;
  for (const auto& c : checks) {
    const int v = es8311ReadReg(c.reg);
    if (v != c.expected) {
      Serial.printf("[ES8311] register 0x%02X reads 0x%02X, expected 0x%02X\n",
                    c.reg, (v < 0) ? 0xFF : v, c.expected);
      allGood = false;
    }
  }
  return allGood;
}

void es8311Init() {
  if (!i2cDevicePresent(ES8311_I2C_ADDR)) {
    Serial.println("ES8311 NOT FOUND on I2C bus -- speaker will stay silent");
    es8311OK = false;
    return;
  }

  // Configure, then read the key registers back; if any didn't stick (a
  // dropped or corrupted I2C write), redo the whole sequence.
  bool verified = false;
  for (uint8_t attempt = 1; attempt <= 3 && !verified; attempt++) {
    es8311ConfigureRegisters();
    verified = es8311VerifyRegisters();
    if (!verified) {
      Serial.printf("[ES8311] register check failed on attempt %u\n", attempt);
    }
  }
  if (!verified) {
    Serial.println("[ES8311] WARNING: codec registers still wrong after 3 attempts");
  }
  if (es8311WriteFailures > 0) {
    Serial.printf("[ES8311] %u I2C write(s) failed even after retries\n", es8311WriteFailures);
  }

  es8311OK = true;  // must be set before applyBuzzerVolume() below, which checks it

  applyBuzzerVolume();         // DAC volume -- pilot's chosen level (buzzerVolumePercent, Config > Volume menu)
  es8311WriteReg(0x31, 0x00);  // DAC: unmute

  Serial.println("ES8311 CODEC INITIALIZED");
}
// Applies buzzerVolumePercent (settings.h) to the ES8311's DAC digital
// volume register (0x32). That register is in 0.5dB steps with 0xBF = 0dB,
// 0xFF = +32dB and 0x00 = -95.5dB (ES8311 datasheet). The pilot's percentage
// is converted to dB with buzzerVolumePercentToDb() (settings.h): 10% steps,
// BUZZER_VOLUME_DB_PER_STEP dB each, 100% = BUZZER_VOLUME_MAX_DB. Called once
// at boot above, and again immediately whenever the pilot changes the
// Config > Volume menu setting.
void applyBuzzerVolume() {
  if (!es8311OK) return;
  const float db = buzzerVolumePercentToDb(buzzerVolumePercent);
  int reg = 0xBF + (int)lroundf(db * 2.0f);  // 0.5dB per register step
  if (reg < 0x00) reg = 0x00;
  if (reg > 0xFF) reg = 0xFF;
  es8311WriteReg(0x32, (uint8_t)reg);
}

// =====================================================
// FANET ENABLE/DISABLE (Config > FANET)
// Called once from setup() (only if fanetEnabled was already true at
// boot -- see the FANET init block above, which handles that first-time
// case directly) and again immediately from menu.cpp any time the pilot
// flips the toggle.
//
// Two distinct cases:
//   - Radio was never successfully brought up (fanetRadioOK == false,
//     e.g. it was off at boot) -- run the exact same init sequence
//     setup() would have run, so switching it on later actually starts
//     it for the first time.
//   - Radio is already up -- just sleep/wake the chip itself via
//     Sx126xLink::setEnabled(), which is far cheaper than a full begin()
//     and preserves its current config.
// =====================================================
void setFanetEnabled(bool enabled) {
  if (!fanetRadioOK) {
    if (!enabled) {
      // Nothing to turn off -- it was never brought up.
      return;
    }

    fanetRadioOK = fanetRadio.begin(/*freqMHz=*/868.2f, /*bwKHz=*/250.0f,
                                     /*sf=*/7, /*cr=*/5, /*syncWord=*/0xF1,
                                     /*powerDbm=*/14, /*preambleLen=*/8);
    if (fanetRadioOK) {
      fanet.begin();
      fanet.onTracking(onFanetTracking);
      fanet.onWeather(onFanetWeather);
      fanet.onMessage(onFanetMessageReceived);
      fanet.setBeaconIntervalMs(FANET_BEACON_INTERVAL_MS);
      Serial.println("FANET RADIO INITIALIZED (enabled from menu)");
    } else {
      Serial.printf("FANET RADIO INIT FAILED -- status=%d\n", fanetRadio.lastStatus());
    }
    return;
  }

  if (!fanetRadio.setEnabled(enabled)) {
    Serial.printf("[FANET] setEnabled(%d) FAILED -- status=%d\n",
                  enabled, fanetRadio.lastStatus());
    return;
  }

  Serial.printf("[FANET] Radio %s\n", enabled ? "enabled" : "disabled (sleep)");
}

// =====================================================
// SCREEN ORIENTATION (Config > Screen)
// Called once from setup() right after u8g2.begin(), and again
// immediately from menu.cpp any time the pilot changes it. U8G2_R0 is
// the app's original (GPS Bottom) orientation; U8G2_R2 is the same
// panel rotated 180 degrees (GPS Top).
// =====================================================
void applyScreenOrientation() {
  u8g2.setDisplayRotation(screenOrientation == SCREEN_ORIENTATION_GPS_TOP
                             ? U8G2_R2
                             : U8G2_R0);
  displayDirty = true;
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
}
void i2sToneService() {

    if (!codecOK || !es8311OK) return;

    int16_t chunk[I2S_TONE_CHUNK];

    // Take a local copy so the requested frequency remains consistent
    // throughout this audio chunk.
    float freq = toneFrequency;

    // ---------------------------------------------------------
    // ANTI-CLICK ENVELOPE
    // Every on/off transition (climb pulse, sink alarm entry/exit, page
    // beep, mute jingle...) used to snap chunk[i] straight between 0 and a
    // mid-cycle sine value -- an instantaneous amplitude jump, which is
    // audible as a click/pop. At the vario's pulse rate (every 100-500ms
    // all flight) that's a constant background tick.
    //
    // Fix: toneAmplitude chases a target (TONE_PEAK_AMPLITUDE when sounding,
    // 0 when silent) by TONE_RAMP_STEP per sample instead of jumping. At
    // 16kHz / 50.0f per sample that's a ~6.25ms fade -- short enough not to
    // blur beep timing, long enough that the ear hears a fade, not a click.
    //
    // lastAudibleFreq keeps the waveform actually oscillating during a
    // fade-out (rather than freezing on one held sample) so the tail of
    // each beep decays like a real tone, not a ramped DC offset.
    // ---------------------------------------------------------
    const float TONE_PEAK_AMPLITUDE = 5000.0f;  // matches the previous fixed amplitude
    const float TONE_RAMP_STEP = 50.0f;         // ~6.25ms fade to/from full amplitude at 16kHz (at gain 1.0)

    // ---------------------------------------------------------
    // WAVEFORM: BAND-LIMITED SQUARE WAVE
    // The BlueFly drives an electromagnetic transducer with a square
    // wave; its buzzy character is the odd harmonics (3f, 5f, 7f...).
    // A pure sine has none, so on this small speaker a 130-350Hz sink
    // tone was thin and quiet. We sum the odd harmonics of a square
    // wave (1/k weighting) but stop below Nyquist so nothing aliases.
    // Harmonics are built with the recurrence
    //     sin((k+2)x) = 2cos(2x)*sin(kx) - sin((k-2)x)
    // so it costs only two trig calls per sample regardless of how
    // many harmonics are summed.
    //
    // SQUARE_LEVEL: a square wave has ~3dB more RMS than a sine of the
    // same peak; 0.72 keeps the climb beeps about as loud as they
    // were with the sine, so only the sink loudness changes.
    // ---------------------------------------------------------
    const float SQUARE_LEVEL = 0.72f;
    const float HARMONIC_LIMIT_HZ = 7500.0f;  // stay below Nyquist (8000Hz at 16kHz)

    // ---------------------------------------------------------
    // LOW-FREQUENCY GAIN COMPENSATION
    // The speaker rolls off hard below ~500Hz, so the sink tone (130-400Hz)
    // came out 6-8dB quieter than the climb beeps. Above LF_GAIN_KNEE_HZ
    // gain is 1.0; below it gain rises as sqrt(knee/freq), capped at
    // LF_GAIN_MAX. At 130Hz that is roughly +6dB. Tune these to taste:
    // raise LF_GAIN_MAX or LF_GAIN_KNEE_HZ for a louder sink tone.
    // ---------------------------------------------------------
    const float LF_GAIN_KNEE_HZ = 500.0f;
    const float LF_GAIN_MAX = 2.2f;

    static float toneAmplitude = 0.0f;
    static float lastAudibleFreq = 440.0f;
    static float smoothedFreq = 0.0f;  // 0 = no tone currently sounding

    // Pitch smoothing (see TONE_PITCH_SMOOTH_MS): a tone that is already
    // sounding glides toward its new pitch; a fresh tone (or a big jump)
    // starts at its target immediately.
    if (freq > 0.0f) {
        if (TONE_PITCH_SMOOTH_MS > 0.0f &&
            smoothedFreq > 0.0f &&
            fabsf(freq - smoothedFreq) <= 0.25f * smoothedFreq) {
            const float chunkMs = 1000.0f * (float)I2S_TONE_CHUNK / (float)I2S_SAMPLE_RATE;
            const float alpha = 1.0f - expf(-chunkMs / TONE_PITCH_SMOOTH_MS);
            smoothedFreq += (freq - smoothedFreq) * alpha;
        } else {
            smoothedFreq = freq;
        }
        lastAudibleFreq = smoothedFreq;
    } else {
        smoothedFreq = 0.0f;
    }

    const float phaseIncFreq = (freq > 0.0f) ? smoothedFreq : lastAudibleFreq;

    float lfGain = 1.0f;
    if (phaseIncFreq < LF_GAIN_KNEE_HZ) {
        lfGain = sqrtf(LF_GAIN_KNEE_HZ / phaseIncFreq);
        if (lfGain > LF_GAIN_MAX) lfGain = LF_GAIN_MAX;
    }

    // Per-tone loudness multiplier (climb/sink volume settings, page beeps
    // and the mute/unmute jingle all set toneGain). 0 = silent.
    const float stepGain = (toneGain > 0.0f) ? toneGain : 0.0f;

    const float totalGain = lfGain * stepGain;

    // Remember the gain of the tone that was last actually sounding, so the
    // fade-out at the end of a beep (when freq is 0) keeps the same ~6ms
    // fade length instead of being far too abrupt for a quiet tone.
    static float lastSoundingGain = 1.0f;
    if (freq > 0.0f && totalGain > 0.0f) {
        lastSoundingGain = totalGain;
    }

    const float targetAmplitude = (freq > 0.0f) ? (TONE_PEAK_AMPLITUDE * totalGain) : 0.0f;
    // Scale the fade step with the gain so the anti-click fade stays ~6ms.
    const float rampStep = TONE_RAMP_STEP * lastSoundingGain;

    // Number of odd harmonics that fit below HARMONIC_LIMIT_HZ.
    int maxHarmonic = (int)(HARMONIC_LIMIT_HZ / phaseIncFreq);
    if (maxHarmonic < 1) maxHarmonic = 1;
    if ((maxHarmonic & 1) == 0) maxHarmonic--;  // largest odd k <= limit

    for (int i = 0; i < I2S_TONE_CHUNK; i++) {

        if (toneAmplitude < targetAmplitude) {
            toneAmplitude += rampStep;
            if (toneAmplitude > targetAmplitude) toneAmplitude = targetAmplitude;
        } else if (toneAmplitude > targetAmplitude) {
            toneAmplitude -= rampStep;
            if (toneAmplitude < targetAmplitude) toneAmplitude = targetAmplitude;
        }

        const float x = 2.0f * PI * tonePhase;
        const float twoCos2x = 2.0f * cosf(2.0f * x);

        float sPrev = -sinf(x);  // sin(-1 * x)
        float sCur = sinf(x);    // sin(1 * x)
        float sum = sCur;        // k = 1 term (weight 1/1)

        for (int k = 3; k <= maxHarmonic; k += 2) {
            const float sNext = twoCos2x * sCur - sPrev;  // sin(k * x)
            sum += sNext / (float)k;
            sPrev = sCur;
            sCur = sNext;
        }

        // (4/pi) scales the harmonic sum to a unit-amplitude square wave.
        chunk[i] = (int16_t)(
            toneAmplitude * SQUARE_LEVEL * (4.0f / PI) * sum
        );

        tonePhase += phaseIncFreq / (float)I2S_SAMPLE_RATE;

        if (tonePhase >= 1.0f) {
            tonePhase -= 1.0f;
        }
    }

    size_t bytesWritten = 0;

#if AUDIO_USE_DEDICATED_TASK
    // Blocking write: waits for room in the DMA ring, which is what paces
    // audioServiceTask() to the I2S hardware clock. (Bounded, so a stalled
    // I2S peripheral can't hang the task forever.)
    i2s_write(
        I2S_PORT,
        chunk,
        sizeof(chunk),
        &bytesWritten,
        pdMS_TO_TICKS(100)
    );
#else
    // Zero-timeout: writes only what fits and never blocks (timer callback).
    i2s_write(
        I2S_PORT,
        chunk,
        sizeof(chunk),
        &bytesWritten,
        0
    );
#endif
}

#if AUDIO_USE_DEDICATED_TASK
// Runs forever: generate one chunk, hand it to the I2S DMA ring, repeat.
// i2s_write() blocks when the ring is full, so this loop runs exactly as
// fast as the hardware plays samples -- see AUDIO_USE_DEDICATED_TASK.
void audioServiceTask(void* arg) {
  (void)arg;
  for (;;) {
    if (!codecOK || !es8311OK) {
      // I2S/codec not up (yet, or failed) -- don't spin.
      vTaskDelay(pdMS_TO_TICKS(20));
      continue;
    }
    i2sToneService();
  }
}
#endif
