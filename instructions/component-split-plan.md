# ESP32 Flight Computer — Component Split: Analysis & Migration Plan

Reference document for splitting the monolithic `.ino` into per-subsystem
`.h`/`.cpp` files. This is a **code-organisation exercise only** — no
behaviour, algorithms, function names, FreeRTOS architecture, or globals
strategy changes as part of this work.

## Ground rules (carried through every step below)

- Preserve existing behaviour exactly.
- Don't rename functions unless a name collision forces it.
- Don't change algorithms or "clean up" unrelated code.
- Don't introduce classes where none exist today.
- Don't redesign the FreeRTOS task/mutex architecture.
- Don't replace globals with a new state-management system — globals just
  move file and gain `extern` declarations.
- `setup()` and `loop()` stay in the `.ino`, including every inline block
  that isn't already its own function (I2C bus init, SD_MMC bring-up,
  sensor probing at boot, the splash-screen draw, the FANET position
  update, the ADS-B new-threat check, the display redraw gate). Only the
  declarations those blocks reference move out, referenced via `extern`.
  Wrapping any of these inline blocks into a new function would itself be
  a small refactor — treat that as future work, not part of this pass.

**Caveat:** this analysis is based on the one file provided. Several
functions here are called from other project files not reviewed
(`menu.cpp`, `DrawPages.cpp`, etc.) — every such case found is flagged
below, but grep the whole project for each symbol before moving it.

---

## 1. Functional areas

### 1.1 Utility / math helpers
- **Functions:** `deg2rad()`, `rad2deg()`, `getBearing()`, `getDistanceKM()`, `getCompassDirection()`, `i2cDevicePresent()`
- **Globals:** none owned
- **Includes needed:** `<math.h>`, `<Wire.h>` (for `i2cDevicePresent`)
- **Depends on:** nothing internal
- **Notes:** `getBearing`/`getDistanceKM` are used by both ADS-B and
  Weather. `getCompassDirection()` is never called *in this file* —
  almost certainly used by `DrawPages.cpp`; confirm and update that
  file's includes. `i2cDevicePresent()` is called twice inline in
  `setup()` (BMP580, RTC probes — those call sites stay put) and once
  from inside `es8311Init()` (moves with Buzzer).

### 1.2 Cross-core shared state
- **Types:** `struct PositionSnapshot`
- **Globals:** `sharedPosition`, `backgroundDataMutex`, `sharedGpsAltitudeFeet`
- **Depends on:** nothing
- **Notes:** written by `loop()` on Core 1, read by ADS-B/Weather/Airspace/DEM
  on Core 0. Owned by none of its readers — gets its own header.

### 1.3 GPS
- **Functions:** none dedicated — `Serial1.begin()` (setup) and the
  `while(Serial1.available())` decode loop (loop) stay inline
- **Globals:** `TinyGPSPlus gps`, `GPS_RX_PIN`, `GPS_TX_PIN`, `GPS_BAUD`
- **Includes needed:** `<TinyGPS++.h>`
- **Depends on:** nothing
- **Notes:** `gps` is read by nearly everything else (Vario, WindEstimator,
  ADS-B, Weather, IGC, FANET, Clock). Give it a thin header purely so
  everyone else has one place to `extern` it from.

### 1.4 SD card (shared infrastructure)
- **Functions:** none — `SD_MMC.setPins()`/`begin()` stay inline in `setup()`
- **Globals:** `sdCardOK`, `sdMutex`, `SD_MMC_CLK_PIN`, `SD_MMC_CMD_PIN`, `SD_MMC_D0_PIN`
- **Includes needed:** `<FS.h>`, `<SD_MMC.h>`
- **Depends on:** nothing
- **Notes:** `sdMutex` guards *all* SD access per the code's own comment
  (IGC writer + airspace/DEM scanner) — not solely an IGC concern, so it
  gets its own small header rather than living inside IgcRecorder.

### 1.5 Auxiliary sensors — Battery, Ambient (SHTC3), Clock/RTC (merged into one file pair)
Three small, low-coupling subsystems, co-located in `AuxSensors.h`/`.cpp`
as three clearly separated sections (keep the original comment banners
for each — don't interleave them).

- **Battery**
  - Functions: `updateBattery()`
  - Globals: `BATT_ADC_PIN`, `BATT_DIVIDER_RATIO`, `BATT_EMPTY_V`, `BATT_FULL_V`, `BATT_SAMPLE_MS`, `BATT_EMA_ALPHA`, `batteryVoltage`, `batteryPercent`, `lastBattSample`, `battInitialized`
  - Depends on: nothing (ADC pin mode/attenuation setup stays inline in `setup()`)
- **Ambient sensor (SHTC3)**
  - Functions: none — `shtc3.begin()` (setup) and the periodic read (loop) stay inline
  - Globals: `shtc3`, `shtc3OK`, `currentTempC`, `SHT_SAMPLE_MS`, `lastShtSample`
  - Includes needed: `<Adafruit_SHTC3.h>`, `<Adafruit_Sensor.h>`
  - Depends on: nothing
- **Clock / RTC**
  - Functions: `syncClockFromGPS()`, `utcTmToEpoch()`
  - Globals: `rtc`, `rtcOK`, `lastRtcPush`, `PCF85063_I2C_ADDR`, `NZ_TIMEZONE`, `clockSynced`, `gpsClockSyncedThisBoot`
  - Includes needed: `"PCF85063A.h"`, `<time.h>`
  - **Depends on: GPS** — `syncClockFromGPS()` reads `gps.date`/`gps.time`
    directly. This is the one thing that pulls the whole merged file out
    of "zero-dependency" territory — see the corrected dependency map
    below. RTC probe + boot-time clock seed stays inline in `setup()`.

`utcTmToEpoch()` reads as a generic time utility but its only two call
sites (`syncClockFromGPS()` and the inline RTC-read block in `setup()`)
are both Clock-domain, so it travels with Clock rather than Utils.

### 1.6 Display
- **Functions:** `applyScreenOrientation()`
- **Globals:** `u8g2`, `RLCD_*` pins, `SPLASH_IMG_FILE`/`_W`/`_H`/`_ROW_BYTES`/`_BYTES`/`SPLASH_DISPLAY_MS`, `displayDirty`, `lastDisplayUpdate`
- **Includes needed:** `<U8g2lib.h>`
- **Depends on:** settings.h (`screenOrientation`, `SCREEN_ORIENTATION_GPS_TOP`)
- **Notes:** the splash-screen draw block in `setup()` has no persistent
  globals of its own (everything in it is a local variable), so it stays
  inline unchanged — only the `#define`s and the `u8g2` object move out.
  `applyScreenOrientation()` is called from `menu.cpp` per the code
  comment — needs a real prototype in the new header.
- `lastDisplayUpdate` is declared under the file's "TIME & SCHEDULING"
  banner but is only ever used for the display redraw gate — it belongs
  here, not in Clock. Flagging so it isn't moved by textual association.

### 1.7 Airspace & terrain proximity
- **Functions:** none of its own — scan-if-due blocks live inside `backgroundTask()`
- **Globals:** `groundElevationFt`, `groundElevationValid`, `demScanAnchor`, `DEM_SCAN_INTERVAL_MS`, `airspaceScanAnchor`, `AIRSPACE_SCAN_INTERVAL_MS`, `nearestAirspace`, `airspaceResultValid`, `nearestAirspaceInfo`, `airspaceInfoResultValid`, `AIRSPACE_FILE`, `AIRSPACE_CONTROLLED_CLASSES[]`, `AIRSPACE_NUM_CONTROLLED_CLASSES`
- **Types:** `AirspaceResult` (from OpenAirScanner.h)
- **Depends on:** OpenAirScanner.h, TerrainDem.h (both already external)
- **Notes:** `AIRSPACE_CONTROLLED_CLASSES[]`/`AIRSPACE_NUM_CONTROLLED_CLASSES`
  (file-scope `static`) look like unused duplicates of the differently-named
  `CONTROLLED_CLASSES[]` declared locally inside `setup()` — the one
  actually passed to `loadAirspaceDatabase()`. Carried forward as-is.

### 1.8 FANET handlers
- **Functions:** `onFanetTracking()`, `onFanetWeather()`, `setFanetEnabled()`
- **Globals:** `fanetRadio`, `fanetRadioOK`, `myFanetAddress`, `fanet`, `fanetContacts[]`, `fanetWeatherStations[]`, `FANET_BEACON_INTERVAL_MS`
- **Includes needed:** `"Sx126xLink.h"`, `"Fanet.h"` (both already exist —
  **name this new file `FanetHandlers.h`**, not `Fanet.h`, to avoid a collision)
- **Depends on:** DrawPages.h (`FanetContact`, `FanetWeatherStation`, `MAX_FANET_CONTACTS`, `MAX_FANET_WEATHER_STATIONS`), settings.h (`fanetEnabled`)
- **Notes:** radio bring-up in `setup()` and the position-update block in
  `loop()` stay inline. `setFanetEnabled()` is called from `menu.cpp`.
  **Ordering matters:** `fanet` is constructed from `fanetRadio` and
  `myFanetAddress` — C++ only guarantees static-init order *within* one
  translation unit, so all three must stay together, same relative order,
  in `FanetHandlers.cpp`.

### 1.9 Wind estimator
- **Functions:** `updateWindEstimator()`
- **Globals:** `estimatedWindSpeedKph`, `estimatedAirspeedKph`, `estimatedWindDirectionDeg`, `windEstimateValid`, `windCircleActive`, `windCircleStartTrack`, `windCircleAccumulatedDeg`, `windCircleLastTrack`, `windCircleMaxSpeedKph`, `windCircleMinSpeedKph`, `windCircleMinSpeedTrack`, `windEstimatorInitialized`, plus `WIND_MIN_CIRCLE_SPEED_KPH`/`WIND_MAX_CIRCLE_SPEED_KPH`/`WIND_MAX_ESTIMATE_KPH`
- **Depends on:** GPS (`gps.speed`, `gps.course`)
- **Notes:** cleanest, lowest-risk extraction in the file. Those three
  `WIND_*` defines appear unused (the function uses hard-coded literals
  instead) — carried forward as-is.

### 1.10 Vario / barometer (BMP580)
- **Functions:** `updateVario()`, `computeClimbRateLeastSquares()`
- **Globals:** `bmp`, `bmpOK`, `BMP5XX_DEFAULT_I2C_ADDR`, `BMP5XX_ALT_I2C_ADDR`, `altWindow[]`, `timeWindow[]`, `windowCount`, `windowIndex`, `lastBaroSample`, `currentAltitudeM`, `currentClimbRateMS`, `currentQNH`, `qnhCalibrated`, `qnhIsFallback`, `CLIMB_WINDOW_N`, `BARO_SAMPLE_MS`, `SEA_LEVEL_QNH_DEFAULT`, `GPS_QNH_FALLBACK_MS`, `QNH_GPS_AVERAGE_MS`, `QNH_GPS_MIN_SAMPLES`
- **Includes needed:** `<Adafruit_Sensor.h>`, `<Adafruit_BMP5xx.h>`
- **Depends on:** GPS (QNH calibration reads `gps.altitude`/`gps.satellites`/`gps.hdop`)
- **Notes:** BMP580 probing + config-at-boot stays inline in `setup()`,
  just needs `extern Adafruit_BMP5xx bmp; extern bool bmpOK;`.
  `SINK_ALARM_MS`/`SINK_TONE_MAX_HZ`/`SINK_TONE_MIN_HZ`/`SINK_TONE_MAX_MS`/`CLIMB_TONE_MAX_MS`
  are named like Vario constants but are only ever read inside the buzzer
  function — see §2, Shared/ambiguous ownership.

### 1.11 Buzzer / audio (I2S + ES8311)
- **Functions:** `setupI2sCodec()`, `es8311WriteReg()`, `es8311Init()`, `applyBuzzerVolume()`, `setToneFrequency()`, `i2sToneService()`, `updateI2sAudioBuzzer()`, `playFeedbackTone()`
- **Globals:** all `I2S_*`/`AMP_ENABLE_PIN`/`ES8311_I2C_ADDR` pins and constants, `audioServiceTimer`, `codecOK`, `es8311OK`, `toneFrequency`, `tonePhase`, `MUTE_TONE_*` constants + `muteToneActive`/`muteToneStart`/`muteToneIsMuteSequence`, `sinkAlarmActive`, `climbAudioActive`, `sinkAlarmStart`, `climbPulseStart`, `climbToneOn`, `INTERCEPT_*` constants + `interceptAlarmActive`/`interceptAlarmStart`
- **Includes needed:** `<driver/i2s.h>`, `<esp_timer.h>`, `<Wire.h>` (via Utils, for `i2cDevicePresent`), `<math.h>`
- **Depends on:** Vario (`currentClimbRateMS`), settings.h (`buzzerMuted`, `buzzerVolumePercent`, `climbToneMinHz/MaxHz`, `climbGapMinMs/MaxMs`, `climbPulseMinMs/MaxMs`, `adsbAlarmMuted`), DrawPages.h (`CLIMB_DEADBAND_MS`), Utils (`i2cDevicePresent`)
- **Notes:** the most cross-cutting subsystem in the file. The
  `esp_timer_create(...)` call and its lambda stay inline in `setup()`;
  the lambda calls `i2sToneService()`, so `Buzzer.h` must be included in
  the `.ino` before that timer is created.

### 1.12 ADS-B client
- **Functions:** `performADSBUpdate()`
- **Globals:** `adsbDoc`, `hasAdsbData`, `adsbTaskRunning`, `adsbNewThreat`, `lastAdsbCheckTime`, `ADSB_INTERVAL_MS`, `MAX_TRACKED_THREATS`, `activeThreatHexes[]`, `activeThreatCount`, `conflictDetectedThisFrame`
- **Includes needed:** `<WiFi.h>`, `<HTTPClient.h>`, `<WiFiClientSecure.h>`, `<ArduinoJson.h>`, `<string.h>`
- **Depends on:** CrossCoreState, Utils (`getDistanceKM`), settings.h (`adsbRingOuterKm`, `adsbAlertRadiusKm`, `adsbAlertVerticalFt`)
- **Notes:** `conflictDetectedThisFrame` is declared but never read/written
  elsewhere — looks vestigial, carried forward as-is. The actual alarm
  trigger (`interceptAlarmActive = true`) lives in `loop()`'s inline
  threat-check block, **not** inside `performADSBUpdate()` — so this file
  doesn't need to know about Buzzer at all.

### 1.13 Weather client
- **Functions:** `updateWeather()`, `findJsonObjectLength()`
- **Globals:** `globalSecureWeatherClient`, `secureWeatherClientInitialized`, `localMeters[]`, `hasWeatherData`, `WEATHER_FIRST_POLL_DELAY_MS`, `weatherTimerAnchor`, `weatherFirstPollDone`
- **Types:** `WindMeter` (from DrawPages.h)
- **Depends on:** CrossCoreState, Utils (`getDistanceKM`, `getBearing`), DrawPages.h (`WindMeter`, `TRACKED_METERS`), settings.h (`weatherPollIntervalMs`)
- **Notes:** the `new WiFiClientSecure()` allocation stays inline in
  `setup()`; only the pointer declaration moves out.

### 1.14 Page / button controller
- **Functions:** `advanceActivePage()`, `jumpToActivePage()`, `updatePageButton()`
- **Globals:** `KEY_PIN`, `KEY_DEBOUNCE_MS`, `KEY_LONG_PRESS_MS`, `PAGE_BEEP_FREQ`, `PAGE_BEEP_MS`, `pageBeepUntil`, `currentPage`, `PAGE_NAMES[]`, `activePages[]`, `activePageIndex`
- **Types:** `Page` (verify origin — likely DrawPages.h, alongside `ACTIVE_PAGE_COUNT`)
- **Depends on:** Buzzer (`playFeedbackTone`, `setToneFrequency`, `muteToneActive`/`muteToneStart`/`muteToneIsMuteSequence`, `AMP_ENABLE_PIN`), Display (`displayDirty`), menu.h (`menuActive`, `openMenu()`, `menuGoBack()`, `menuMoveDown()`, `menuSelectCurrentItem()`, `MENU_DOUBLE_PRESS_MS`, `MENU_SELECT_HOLD_MS`), settings.h (`buzzerMuted`, `saveSettings()`)
- **Notes:** reaches directly into Buzzer's internal state (mute-jingle
  fields, amp pin) more than any other cross-module relationship in the file.

### 1.15 IGC flight recorder
- **Functions:** `formatIgcLatLon()`, `writeIgcBRecord()`, `startIgcRecording()`, `stopIgcRecording()`, `updateIgcRecorder()`, `setFlightRecorderEnabled()`
- **Globals:** `igcFile`, `igcRecording`, `igcFilename`, `igcAboveThresholdSince`, `igcBelowThresholdSince`, `lastIgcFixWrite`, `IGC_START_SPEED_KPH`, `IGC_STOP_SPEED_KPH`, `IGC_START_SUSTAIN_MS`, `IGC_STOP_SUSTAIN_MS`, `IGC_FIX_INTERVAL_MS`
- **Depends on:** SdCard (`sdCardOK`, `sdMutex`), GPS, Vario (`bmpOK`, `currentAltitudeM`), settings.h (`flightRecorderEnabled`)
- **Notes:** `setFlightRecorderEnabled()` is called from `menu.cpp` — needs a real prototype.

### 1.16 Background task (Core 0)
- **Functions:** `backgroundTask()`
- **Globals:** `backgroundTaskHandle`
- **Depends on:** CrossCoreState, SdCard, AirspaceProximity, AdsbClient (`performADSBUpdate`, `lastAdsbCheckTime`, `adsbTaskRunning`), WeatherClient (`updateWeather`, `weatherTimerAnchor`, `weatherFirstPollDone`), plus existing `wifi_manager.h`, `ble_manager.h`(?), `FileServer.h`, `TerrainDem.h`, `OpenAirScanner.h`, `menu.h` (`selectedDemFile`)
- **Notes:** moves as one intact function — its internal DEM-scan/
  airspace-scan sub-blocks travel with it unchanged. Extract this **last**,
  once everything it touches already has a header. Also: `loadBleSettings()`/
  `bleManagerLoop()` are attributed to `ble_manager.h` by a comment but
  that header isn't explicitly `#include`d in this file — probably arrives
  transitively; confirm before moving.

### 1.17 `setup()` / `loop()` — stays in the main `.ino`
Unchanged. Every inline fragment inside them keeps its current location;
they just gain more `#include`s and call functions declared in the new
headers instead of functions defined further up the same file. One
sequencing note: `Wire.begin(I2C_SDA, I2C_SCL)` runs once and is relied on
by four devices (BMP580, SHTC3, RTC, ES8311) even though it's textually
filed under "BMP580" today — `I2C_SDA`/`I2C_SCL` stay as plain `#define`s
in the `.ino` since their only use site isn't moving.

---

## 2. Shared / ambiguous ownership (per your direction — using these as-is)

| Item | Set/used by | Home | Why |
|---|---|---|---|
| `interceptAlarmActive`/`Start`, `INTERCEPT_TONE_*` | Set in `loop()`; state-machined in `updateI2sAudioBuzzer()` | **Buzzer** | Timing/tone logic lives there; ADS-B code just flips a bool |
| `muteToneActive`/`Start`/`IsMuteSequence`, `AMP_ENABLE_PIN` | Triggered in `updatePageButton()`; sequenced in `updateI2sAudioBuzzer()` | **Buzzer** | Same reasoning — PageButton reaches in via `extern` |
| `SINK_ALARM_MS`, `SINK_TONE_MAX/MIN_HZ`, `SINK_TONE_MAX_MS`, `CLIMB_TONE_MAX_MS` | Named like Vario constants, only read in `updateI2sAudioBuzzer()` | **Buzzer** | Sole consumer |
| `displayDirty` | Set by page-nav + `applyScreenOrientation()`; read by `loop()`'s redraw gate | **Display** | "Does the screen need a redraw" is a Display concern |
| `lastDisplayUpdate` | Filed under "TIME & SCHEDULING" but only used for the redraw gate | **Display**, not Clock | Textual grouping is misleading here |
| `I2C_SDA`/`I2C_SCL` | Feeds one shared `Wire.begin()` for 4 devices | Stays in the `.ino` | Only use site isn't moving |
| `utcTmToEpoch()` | Generic-looking, but only called from Clock-domain code | **AuxSensors (Clock section)** | Matches actual usage |
| `getCompassDirection()` | Defined here, not called in this file | **Utils** | Likely called from `DrawPages.cpp` — verify |
| `sdCardOK`, `sdMutex` | IGC recorder, Airspace/DEM scan, boot splash | **SdCard** (own file) | Genuinely multi-owner |
| `PositionSnapshot`, `sharedPosition`, `backgroundDataMutex`, `sharedGpsAltitudeFeet` | Written by `loop()`, read by ADS-B/Weather/Airspace/DEM | **CrossCoreState** (own file) | Owned by none of its readers |
| `playFeedbackTone()` | Called from `advanceActivePage()`; possibly also `menu.cpp` (not fully confirmed) | **Buzzer** | Fundamentally a `setToneFrequency()` wrapper |

### Suspected dead code (preserved, not removed, per constraints)
- `conflictDetectedThisFrame`
- `beepOn`, `lastBeepToggle`
- File-scope `AIRSPACE_CONTROLLED_CLASSES[]` / `AIRSPACE_NUM_CONTROLLED_CLASSES`
- `WIND_MIN_CIRCLE_SPEED_KPH` / `WIND_MAX_CIRCLE_SPEED_KPH` / `WIND_MAX_ESTIMATE_KPH`

---

## 3. Proposed file structure

```
FlightComputer.ino          setup(), loop(), and every inline block that isn't its own function

Utils.h / .cpp               deg2rad, rad2deg, getBearing, getDistanceKM, getCompassDirection, i2cDevicePresent
CrossCoreState.h / .cpp      PositionSnapshot, sharedPosition, backgroundDataMutex, sharedGpsAltitudeFeet
Gps.h / .cpp                 gps object, GPS_RX_PIN/TX_PIN/BAUD
SdCard.h / .cpp              sdCardOK, sdMutex, SD_MMC_* pins
AuxSensors.h / .cpp          Battery + AmbientSensor(SHTC3) + Clock/RTC, as three sections in one file
Display.h / .cpp             u8g2, RLCD_* pins, SPLASH_* defines, applyScreenOrientation(), displayDirty, lastDisplayUpdate
AirspaceProximity.h / .cpp   ground elevation + airspace-result state (scan logic stays in BackgroundTask)
FanetHandlers.h / .cpp       fanetRadio, fanet, onFanetTracking(), onFanetWeather(), setFanetEnabled()
WindEstimator.h / .cpp       updateWindEstimator() + wind-circle state
Vario.h / .cpp                bmp, updateVario(), computeClimbRateLeastSquares()
Buzzer.h / .cpp               I2S/ES8311 stack, tone gen, vario/alarm/mute audio state machine
AdsbClient.h / .cpp           performADSBUpdate() + ADS-B state
WeatherClient.h / .cpp        updateWeather(), findJsonObjectLength() + weather state
PageButton.h / .cpp           advanceActivePage(), jumpToActivePage(), updatePageButton()
IgcRecorder.h / .cpp          IGC B-record writer, start/stop/update, setFlightRecorderEnabled()
BackgroundTask.h / .cpp       backgroundTask() (moves whole, last)

--- existing, unchanged ---
secrets.h, OpenAirScanner.h/.cpp, menu.h/.cpp, settings.h/.cpp,
Sx126xLink.h/.cpp, Fanet.h/.cpp, wifi_manager.h/.cpp, ble_manager.h/.cpp(?),
FileServer.h/.cpp, DrawPages.h/.cpp, TerrainDem.h, PCF85063A.h/.cpp
```

16 new file pairs. Every new `.h` gets a `#pragma once`.

---

## 4. Dependency map (topological order)

> **Note on the merge:** because Clock's `syncClockFromGPS()` reads the
> `gps` object directly, the merged `AuxSensors.cpp` needs `Gps.h` — so
> the whole file moves to Layer 1 (after Gps), not Layer 0. Battery and
> AmbientSensor alone would have been Layer 0; Clock pulls the merged
> file forward.

- **Layer 0** (no internal deps — any order): Utils, CrossCoreState, Gps, SdCard, Display, AirspaceProximity, FanetHandlers
- **Layer 1** (→ Gps): AuxSensors, WindEstimator, Vario
- **Layer 2** (→ Layer 1 / CrossCoreState / Utils): Buzzer (→ Vario), AdsbClient, WeatherClient (→ CrossCoreState, Utils)
- **Layer 3**: PageButton (→ Buzzer, Display), IgcRecorder (→ SdCard, Gps, Vario)
- **Layer 4**: BackgroundTask (→ CrossCoreState, SdCard, AirspaceProximity, AdsbClient, WeatherClient)
- **Layer 5**: the `.ino` (→ everything)

---

## 5. Migration plan (build after every step — don't batch)

**Phase 0 — Scaffolding, no logic moved.**
Create all 16 file pairs empty (`#pragma once` + a one-line comment).
Add matching `#include`s to the `.ino`. Build. This isolates "does the
toolchain see my new files" from any real risk.

**Phase 1 — Layer 0, any order, one file at a time:**
`Utils` → `CrossCoreState` → `Gps` → `SdCard` → `Display` →
`AirspaceProximity` → `FanetHandlers`. Build after each move.
`Display` and `FanetHandlers` additionally require checking `menu.cpp`
for calls to `applyScreenOrientation()` / `setFanetEnabled()`.

**Phase 2 — Layer 1, in three sub-steps for AuxSensors:**
1. `AuxSensors.h`/`.cpp`, Battery section only. Build.
2. Add the AmbientSensor section to the same file. Build.
3. Add the Clock section (`#include "Gps.h"` in the `.cpp`). Build.
4. `WindEstimator`. Build.
5. `Vario` (needs `Gps.h`). Build.

**Phase 3 — Layer 2:**
`Buzzer` (needs `Vario.h`) → `AdsbClient` → `WeatherClient` (both need
`CrossCoreState.h`, `Utils.h`). Build after each.

**Phase 4 — Layer 3:**
`PageButton` (needs `Buzzer.h`, `Display.h`) → `IgcRecorder` (needs
`SdCard.h`, `Gps.h`, `Vario.h`). Build after each.
`PageButton` and `IgcRecorder` additionally require checking `menu.cpp`
for calls to `advanceActivePage()`-adjacent logic and
`setFlightRecorderEnabled()`.

**Phase 5 — Layer 4:**
`BackgroundTask`, once everything above already compiles cleanly.

**Phase 6 — Verification.**
Confirm the `.ino` now contains only includes, `setup()`, and `loop()`.
Do a full clean rebuild (clear the build cache) to rule out stale object
files masking a missing symbol. If you can flash real hardware: capture
the serial boot log before and after the whole migration and diff it
line-by-line — since no logic changed, it should be byte-identical. Any
difference means something was mis-moved, not an intended change.

---

## 6. Dependency changes required (checklist)

- [ ] **Function prototypes.** Arduino's automatic prototype generation
      only applies to functions defined directly in the `.ino`. Every
      moved function needs an explicit prototype in its paired `.h`,
      included wherever it's called.
- [ ] **Globals → `extern`.** Definition stays (unchanged) in the new
      `.cpp`; add an `extern` declaration to the `.h`. E.g. `Vario.h`
      needs `extern bool bmpOK; extern float currentAltitudeM; extern float currentClimbRateMS; extern bool qnhCalibrated;`
      since `Buzzer.cpp` and `IgcRecorder.cpp` both read them.
- [ ] **`static` → non-`static` where cross-file visibility turns out to
      be needed.** Watch `AIRSPACE_CONTROLLED_CLASSES[]` /
      `AIRSPACE_NUM_CONTROLLED_CLASSES` specifically — currently
      file-scope `static`; if actually used elsewhere, the `static` has
      to come off.
- [ ] **Extern-declared-elsewhere arrays.** `fanetContacts[]` /
      `fanetWeatherStations[]` and `localMeters[]` are already declared
      `extern` in `DrawPages.h`. Moving their *definitions* into
      `FanetHandlers.cpp` / `WeatherClient.cpp` is safe — just don't let
      two files both define them.
- [ ] **Static-init ordering.** `FanetStack fanet(fanetRadio, myFanetAddress);`
      — keep all three declarations together, same relative order, in
      `FanetHandlers.cpp`.
- [ ] **Confirmed external callers to re-wire:** `setFanetEnabled()`,
      `applyScreenOrientation()`, `setFlightRecorderEnabled()` (all
      called from `menu.cpp` per this file's own comments).
- [ ] **Suspected external callers — verify:** `getCompassDirection()`,
      `playFeedbackTone()`.
- [ ] **Transitive includes — verify, don't assume.** FreeRTOS types
      (`SemaphoreHandle_t`, `xSemaphoreTake`, etc.) aren't explicitly
      `#include`d in this file — confirm they resolve in each new `.cpp`.
      Same for `ble_manager.h`.
- [ ] **Include order in the `.ino`.** Every new header must be included
      before `setup()`/`loop()` reference names from it — including
      before the `esp_timer` lambda that calls `i2sToneService()`.

---

## 7. Summary

**A. Current responsibilities found:** utility/geo math, GPS, wind
estimation, barometer/vario, ambient temp/humidity, RTC/clock sync,
battery monitoring, I2S+ES8311 buzzer/audio (vario tone, sink alarm,
mute jingle, intercept alarm), display + boot splash, page/button/
menu-glue input handling, SD card access, IGC flight logging, cross-core
shared position/mutex state, airspace and terrain-elevation proximity,
ADS-B polling and threat detection, Zephyr weather-network polling,
FANET radio integration, and the Core-0 background task tying several of
those together. `setup()`/`loop()` are pure orchestration. No IMU present.

**B. Proposed components:** 16 new `.h`/`.cpp` pairs, with Battery,
AmbientSensor, and Clock consolidated into a single `AuxSensors` pair;
`setup()`/`loop()` and their inline hardware-bring-up code stay in the
main `.ino`.

**C. Dependency map:**
Layer 0 → Utils, CrossCoreState, Gps, SdCard, Display, AirspaceProximity, FanetHandlers
Layer 1 → AuxSensors, WindEstimator, Vario (all need Gps)
Layer 2 → Buzzer, AdsbClient, WeatherClient
Layer 3 → PageButton, IgcRecorder
Layer 4 → BackgroundTask
Layer 5 → the `.ino`

**D. Recommended extraction order:** Phase 0 scaffolding → Phase 1
(Layer 0, any order) → Phase 2 (AuxSensors in three sub-steps, then
WindEstimator, then Vario) → Phase 3 (Buzzer, AdsbClient, WeatherClient)
→ Phase 4 (PageButton, IgcRecorder) → Phase 5 (BackgroundTask) →
Phase 6 (full build + boot-log diff).

**E. Risks / things to watch:**
- Loss of Arduino's automatic prototyping the moment a function leaves
  the `.ino` — every call site needs an explicit header include.
- Files not reviewed here (`menu.cpp`, `DrawPages.cpp`, etc.) may call
  back into functions/globals defined in this file — confirmed:
  `setFanetEnabled`, `applyScreenOrientation`, `setFlightRecorderEnabled`;
  suspected: `getCompassDirection`, `playFeedbackTone`.
- `Wire.begin()` is shared by four devices but textually filed under
  "BMP580" — don't let it get orphaned or duplicated.
- `FanetStack fanet(fanetRadio, myFanetAddress)` — keep all three
  declarations in one file, same order, or risk a static-init-order bug.
- Transitively-included headers (FreeRTOS types, `ble_manager.h`) —
  verify explicitly rather than assuming the transitive chain holds.
- Dead-looking code is preserved as instructed, not removed — don't let
  a future pass silently drop it without checking first.
- Compile after every single file move, not after a batch.
