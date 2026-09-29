# Flight Computer — Source Layout

ESP32-S3 paraglider/paramotor flight computer. The firmware is split into one
`.h`/`.cpp` pair per subsystem. Globals keep their original names: each
`.cpp` holds the definitions, each `.h` holds the `extern` declarations,
prototypes, and `#define`s.

`FlightComputer.ino` holds only the includes, `I2C_SDA`/`I2C_SCL`,
`setup()` and `loop()` — including the hardware bring-up blocks that were
always written inline inside them.

## What each file contains

| File | Contains | Does *not* contain |
|---|---|---|
| **`Utils.cpp`** | Stateless helpers: `deg2rad`, `rad2deg`, `getBearing`, `getDistanceKM`, `getCompassDirection`, `i2cDevicePresent` | Any state |
| **`CrossCoreState.cpp`** | Core 1 → Core 0 handoff: `sharedPosition` (`PositionSnapshot`), `sharedGpsAltitudeFeet`, `backgroundDataMutex` | Readers/writers of that data |
| **`Gps.cpp`** | The shared `gps` (`TinyGPSPlus`) object. GPS pins/baud are in `Gps.h` | UART init and the decode loop (in `setup()`/`loop()`) |
| **`SdCard.cpp`** | `sdCardOK` and `sdMutex` (guards *all* SD access). SD_MMC pins are in `SdCard.h` | SD mounting (in `setup()`), IGC file state |
| **`Display.cpp`** | The `u8g2` panel object, `displayDirty`, `lastDisplayUpdate`, `applyScreenOrientation()`. Splash constants are in `Display.h` | The splash draw (in `setup()`), page drawing (`DrawPages.cpp`) |
| **`AirspaceProximity.cpp`** | Shared result state of the DEM and airspace scans: `groundElevationFt/Valid`, `nearestAirspace`, `nearestAirspaceInfo`, scan timers | The scans themselves (in `BackgroundTask.cpp`) |
| **`FanetHandlers.cpp`** | `fanetRadio`, `myFanetAddress`, `fanet` (**keep in this order**), `fanetContacts[]`, `fanetWeatherStations[]`, `onFanetTracking`, `onFanetWeather`, `setFanetEnabled` | Radio bring-up and the per-loop position feed (in `setup()`/`loop()`) |
| **`AuxSensors.cpp`** | Three small subsystems in labelled sections: **Battery** (`updateBattery`), **SHTC3** (state only), **Clock/RTC** (`syncClockFromGPS`, `utcTmToEpoch`) | Sensor/RTC init and the SHTC3 read (in `setup()`/`loop()`) |
| **`WindEstimator.cpp`** | `updateWindEstimator()` and its circle-detection state | — |
| **`Vario.cpp`** | BMP580 sampling and altitude/climb state, QNH calibration, `updateVario()`, `computeClimbRateLeastSquares()` | BMP580 probing/config (in `setup()`), all audio (`Buzzer.cpp`) |
| **`Buzzer.cpp`** | I2S + ES8311 setup, tone generator (`i2sToneService`), the audio state machine `updateI2sAudioBuzzer()` (climb, sink, ADS-B intercept alarm, mute jingle), `playFeedbackTone()`, and the state those use (incl. `pageBeepUntil`) | The audio-timer creation (in `setup()`) |
| **`AdsbClient.cpp`** | `performADSBUpdate()`, `adsbDoc`, threat tracking, `hasAdsbData`/`adsbNewThreat` flags | The alarm sound (`Buzzer.cpp`), the "when to poll" decision (`BackgroundTask.cpp`) |
| **`WeatherClient.cpp`** | `updateWeather()`, `findJsonObjectLength()`, `localMeters[]`, `hasWeatherData`, poll-timer state | The "when to poll" decision (`BackgroundTask.cpp`) |
| **`PageButton.cpp`** | `updatePageButton()` (debounce, short/double/long press), `advanceActivePage()`, `jumpToActivePage()`, `currentPage`/`activePages[]` | Page drawing (`DrawPages.cpp`), menu logic (`menu.cpp`) |
| **`IgcRecorder.cpp`** | IGC flight logging: B-record formatting/writing, auto start/stop, `setFlightRecorderEnabled()` | SD mutex/mount (`SdCard`) |
| **`BackgroundTask.cpp`** | `backgroundTask()` — the Core 0 loop: Wi-Fi/BLE/file-server loops, ADS-B and weather poll gating, DEM lookup, airspace scan; plus `backgroundTaskHandle` | Task creation (in `setup()`) |

## Existing modules (unchanged)

`menu`, `settings`, `DrawPages`, `OpenAirScanner`, `wifi_manager`,
`ble_manager`, `FileServer`, `Sx126xLink`, `Fanet`, `secrets.h`,
`TerrainDem.h`, `PCF85063A`.

## Conventions

- **Threading:** Core 1 runs `loop()`; Core 0 runs `backgroundTask()`.
  `backgroundDataMutex` guards `sharedPosition`, `adsbDoc`, `localMeters[]`
  and the airspace results; `sdMutex` guards all SD access.
- **Dependency order** (each layer only uses those above it):
  `Utils`, `CrossCoreState`, `Gps`, `SdCard`, `Display`, `AirspaceProximity`,
  `FanetHandlers` → `AuxSensors`, `WindEstimator`, `Vario` → `Buzzer`,
  `AdsbClient`, `WeatherClient` → `PageButton`, `IgcRecorder` →
  `BackgroundTask` → `FlightComputer.ino`.
- **Where new code goes:** a new sensor or subsystem gets its own pair (or a
  section in `AuxSensors` if it's tiny); a new sound goes in
  `updateI2sAudioBuzzer()`; a new periodic network/SD job goes in
  `backgroundTask()`; a new page is drawn in `DrawPages.cpp` and registered in
  `PageButton.cpp`.

## Watch-outs

- `FanetHandlers.cpp`: `fanetRadio`, `myFanetAddress`, `fanet` must stay in one
  file, in that order (static-init order).
- `TerrainDem.h` is header-only: include it from **one** `.cpp` only
  (`BackgroundTask.cpp`).
- Some dead-looking globals are kept on purpose (`conflictDetectedThisFrame`,
  `beepOn`, `lastBeepToggle`, `lastSinkBeep`, the `AIRSPACE_*` statics, the
  `WIND_*` defines). Check the whole project before deleting any of them.
- Full history, ownership decisions and verification checklist:
  `component-split-plan.md`.
