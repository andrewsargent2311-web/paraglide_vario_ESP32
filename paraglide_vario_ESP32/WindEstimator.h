#pragma once
// WindEstimator -- wind / airspeed estimate from GPS ground velocity.
//
// Method: while circling, the ground-velocity vector (east, north) traces a
// circle whose RADIUS is the airspeed and whose CENTRE is the wind vector
// (the direction the wind blows TO). A least-squares circle fit over the last
// full turn gives both. estimatedWindDirectionDeg is where the wind comes
// FROM (meteorological convention), 0..360.
#include <Arduino.h>

#define WIND_MIN_CIRCLE_SPEED_KPH 15.0f
#define WIND_MAX_CIRCLE_SPEED_KPH 80.0f
#define WIND_MAX_ESTIMATE_KPH 50.0f

extern float estimatedWindSpeedKph;
extern float estimatedAirspeedKph;
extern float estimatedWindDirectionDeg;  // direction wind comes FROM
extern bool windEstimateValid;

// Call every loop pass; consumes only fresh GPS speed/course updates.
void updateWindEstimator();

// Core of the estimator, free of any GPS/clock access so it can be unit
// tested on the host. now_ms must be a monotonic millis()-style clock.
void windEstimatorFeed(float groundSpeedKph, float trackDeg, uint32_t now_ms);

// Discard the partial circle (GPS lost, stationary, etc.). Does not clear
// the last accepted estimate.
void windEstimatorReset();
