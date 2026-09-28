#include "WindEstimator.h"
#include "Gps.h"
#include <math.h>

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
