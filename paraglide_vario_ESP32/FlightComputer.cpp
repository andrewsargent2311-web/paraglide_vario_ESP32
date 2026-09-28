
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
#include <FS.h>
#include <SD_MMC.h>

// ---- Existing project modules (unchanged) ----

#include "secrets.h"
#include "OpenAirScanner.h"
#include "menu.h"
#include "settings.h"
#include "Sx126xLink.h"
#include "Fanet.h"
#include "wifi_manager.h"
#include "FileServer.h"
#include "DrawPages.h"

// ---- Component headers extracted from the old monolithic sketch ----
#include "Utils.h"
#include "CrossCoreState.h"
#include "Gps.h"
#include "SdCard.h"
#include "Display.h"
#include "AirspaceProximity.h"
#include "FanetHandlers.h"
#include "AuxSensors.h"
#include "WindEstimator.h"
#include "Vario.h"
#include "Buzzer.h"
#include "AdsbClient.h"
#include "WeatherClient.h"
#include "PageButton.h"
#include "IgcRecorder.h"
#include "BackgroundTask.h"

// =====================================================
// I2C BUS PINS
// Shared by FOUR devices (BMP580, SHTC3, PCF85063 RTC, ES8311 codec),
// all brought up off the single Wire.begin() in setup(). Originally
// declared under the "BMP580 BAROMETER" banner, but they aren't
// BMP-specific, so they stay here with the one call site that uses them.
// =====================================================
#define I2C_SDA 13
#define I2C_SCL 14

void setup() {
  Serial.begin(115200);
  delay(1000);  // give the USB CDC host a moment to attach before the first print, or it's often lost
  Serial.println("BOOTING FLIGHT COMPUTER...");
  Serial.printf("[BOOT] Free heap: %u | Min heap: %u\n", ESP.getFreeHeap(), ESP.getMinFreeHeap());

  // Pull every persisted setting out of NVS before anything below reads
  // one of them (buzzer volume, climb tone, selected DEM file, etc.).
  loadSettings();
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
  // Independent audio-servicing timer. i2sToneService() no longer runs
  // from loop() -- it's called on a fixed 4ms cadence regardless of what
  // either core is doing, so a slow display redraw or (now relocated)
  // network call can never starve the DMA buffer and cause the tone to
  // glitch/cut out.
  // ---------------------------------------------------------
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
