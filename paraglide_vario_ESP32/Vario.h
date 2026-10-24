#pragma once
// Vario -- BMP580 barometer sampling, altitude / climb-rate state, QNH
// calibration. All audio lives in Buzzer.cpp.
#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP5xx.h>

extern Adafruit_BMP5xx bmp;
extern bool bmpOK;
bool initBaro();
bool baroIsFresh();
bool baroAltitudeTrusted();
// BMP58x breakouts use either 0x47 (default) or 0x46 (SDO/address jumper).
// Probe both: the barometer is an external flight sensor, not onboard.
constexpr uint8_t BMP5XX_DEFAULT_I2C_ADDR = 0x47;
constexpr uint8_t BMP5XX_ALT_I2C_ADDR = 0x46;

// =====================================================
// Variov- Variables
// =====================================================
// Tuning
#define SEA_LEVEL_QNH_DEFAULT 1013.25f
// QNH starts at SEA_LEVEL_QNH_DEFAULT on boot (no waiting for GPS), is
// replaced by a GPS-derived value on the first good fix, then refreshed
// every QNH_RECALIBRATION_INTERVAL_MS. See updateVario().

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

// The fast climb rate (currentClimbRateMS, a ~0.8 s regression) drives the
// audio buzzer. For anything a human reads or that gets broadcast (the
// on-screen vario, glide ratio) it is further averaged over this long so
// the number stops flickering. 1000-2000 ms is a sensible range; longer =
// steadier but laggier. Samples arrive every BARO_SAMPLE_MS, so
// DISPLAY_CLIMB_BUF_N must be at least DISPLAY_CLIMB_AVG_MS / BARO_SAMPLE_MS
// (plus some slack for jitter).
#define DISPLAY_CLIMB_AVG_MS 1500UL
#define DISPLAY_CLIMB_BUF_N 32

// =====================================================
// KALMAN CLIMB-RATE ESTIMATOR
// =====================================================
// Two-state (altitude, vertical speed) Kalman filter fed by the barometer.
// It replaces the 8-sample least-squares slope as the source of
// currentClimbRateMS when VARIO_USE_KALMAN is true. The least-squares code
// and its window stay in the file so the two can be compared in flight by
// flipping this one flag.
//
// Why a Kalman filter rather than a longer/shorter window:
//  * Window-edge behaviour: a sliding window drops a sample every 100 ms,
//    so any single bad sample enters AND leaves as a step. The filter's
//    memory decays smoothly instead.
//  * Irregular sample spacing: loop() also draws the display and writes the
//    SD card, so baro samples are not always 100 ms apart. The filter uses
//    the real elapsed time for every sample.
//  * Outlier gating that scales with what the filter expects: a sample is
//    rejected when it is too far from the PREDICTED altitude (in standard
//    deviations), instead of the old fixed +/-15 m/s rate limit, which at
//    100 ms spacing still let a 1.4 m jump straight through.
//
// There is no accelerometer on this board, so this is barometer-only.
// The only physics it knows is "vertical speed changes slowly, at about
// VARIO_KF_ACCEL_SIGMA_MS2 per second".
//
// TUNING (these two numbers are the whole trade-off):
//  * VARIO_KF_ACCEL_SIGMA_MS2 up   -> faster response, more jitter.
//  * VARIO_KF_ALT_SIGMA_M up       -> steadier, laggier (trusts baro less).
// The defaults are tuned on synthetic data only; confirm on a real flight
// log before trusting them (see VarioDiagnostics below).
constexpr bool VARIO_USE_KALMAN = true;
constexpr float VARIO_KF_ALT_SIGMA_M = 0.10f;        // baro altitude noise, 1 sigma (m)
constexpr float VARIO_KF_ACCEL_SIGMA_MS2 = 1.5f;     // vertical acceleration noise, 1 sigma (m/s^2)
constexpr float VARIO_KF_INIT_VS_SIGMA_MS = 2.0f;    // initial vertical-speed uncertainty (m/s)
constexpr float VARIO_KF_GATE_SIGMAS = 5.0f;         // reject a sample further than this from prediction...
constexpr float VARIO_KF_MIN_GATE_M = 1.0f;          // ...but never closer than this (m). The sigma gate alone
                                                     // rejects good samples in rough air, where the real
                                                     // acceleration is larger than the model's, and then the
                                                     // filter coasts on stale data exactly when it matters.
constexpr uint8_t VARIO_KF_MAX_CONSECUTIVE_REJECTS = 2;  // then accept it: a real change persists, a glitch does not
constexpr uint32_t VARIO_KF_MAX_GAP_MS = 5000;       // a longer gap restarts the filter

// If no baro sample has been accepted for this long, the climb rate is
// forced to zero rather than left frozen at its last value (a frozen rate
// would keep beeping the same tone indefinitely).
constexpr uint32_t VARIO_STALE_TIMEOUT_MS = 1000;

class VarioKalman {
 public:
  struct Config {
    float altSigmaM;
    float accelSigmaMS2;
    float initVsSigmaMS;
    float gateSigmas;
    float minGateM;
    uint8_t maxConsecutiveRejects;
    uint32_t maxGapMs;
  };

  static constexpr Config defaultConfig() {
    return Config{VARIO_KF_ALT_SIGMA_M, VARIO_KF_ACCEL_SIGMA_MS2,
                  VARIO_KF_INIT_VS_SIGMA_MS, VARIO_KF_GATE_SIGMAS,
                  VARIO_KF_MIN_GATE_M, VARIO_KF_MAX_CONSECUTIVE_REJECTS, VARIO_KF_MAX_GAP_MS};
  }

  explicit VarioKalman(const Config& cfg = defaultConfig()) : cfg_(cfg) { reset(); }

  void reset();

  // Feed one altitude sample (metres, any fixed reference) taken at
  // sampleMs (millis()). Returns true if the sample was used, false if it
  // was gated out as an outlier or had a non-positive time step.
  [[nodiscard]] bool update(float altM, uint32_t sampleMs);

  bool initialised() const { return initialised_; }
  float altitudeM() const { return altM_; }
  float climbRateMS() const { return vsMS_; }
  uint32_t rejectedCount() const { return rejectedTotal_; }

 private:
  void seed(float altM, uint32_t sampleMs);

  Config cfg_;
  bool initialised_ = false;
  float altM_ = 0.0f;
  float vsMS_ = 0.0f;
  float p00_ = 0.0f, p01_ = 0.0f, p11_ = 0.0f;  // covariance (symmetric)
  uint32_t lastMs_ = 0;
  uint8_t consecutiveRejects_ = 0;
  uint32_t rejectedTotal_ = 0;
};

// =====================================================
// TIMING DIAGNOSTICS
// =====================================================
// The vario is sampled from loop(), which also draws the display, writes
// the SD card and talks to other I2C sensors. If any of that blocks, the
// baro sample (and the beep timing, which is also in loop()) is late.
// These counters show whether that happens, without needing a logic
// analyser. Read them after a flight / bench run, or set
// VARIO_DIAG_PRINT to print a one-line summary every 10 s over USB.
constexpr bool VARIO_DIAG_PRINT = false;
constexpr uint32_t VARIO_LATE_SAMPLE_MS = 150;  // a gap above this counts as "late"
constexpr uint32_t VARIO_DIAG_PRINT_MS = 10000;

struct VarioDiagnostics {
  uint32_t samples;           // successful baro reads, before altitude gating
  uint32_t lateSamples;       // gaps between samples above VARIO_LATE_SAMPLE_MS
  uint32_t maxGapMs;          // longest gap between accepted samples
  uint32_t readFailures;      // performReading() returned false
  uint32_t gateRejects;       // samples dropped by the +/-15 m/s sanity gate
  uint32_t kalmanRejects;     // samples dropped by the Kalman innovation gate
};
const VarioDiagnostics& varioDiagnostics();
void resetVarioDiagnostics();

extern float altWindow[CLIMB_WINDOW_N];
extern unsigned long timeWindow[CLIMB_WINDOW_N];
extern int windowCount;
extern int windowIndex;
extern unsigned long lastBaroSample;
extern unsigned long lastSuccessfulBaroSample;
extern float currentAltitudeM;
extern float currentPressureAltStdM;
extern float currentClimbRateMS;     // fast -- audio buzzer
extern float currentClimbRateAvgMS;  // DISPLAY_CLIMB_AVG_MS average -- screen, glide ratio
extern float currentQNH;
extern bool qnhCalibrated;
extern bool qnhIsFallback;
extern float currentPressureHpa;   // latest BMP580 pressure, hPa

void updateVario();
float computeClimbRateLeastSquares();

// Push one fast climb-rate sample and refresh currentClimbRateAvgMS (mean of
// every sample from the last DISPLAY_CLIMB_AVG_MS). resetClimbRateAverage()
// clears the history; used by the unit tests.
void updateClimbRateAverage(float rateMS, unsigned long nowMs);
void resetClimbRateAverage();
