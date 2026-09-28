#pragma once

#include <Arduino.h>

// =====================================================
// 360° WIND / AIRSPEED ESTIMATOR
// -----------------------------------------------------
// Estimates wind speed/direction and airspeed from a full GPS-tracked
// circle (a paraglider/paramotor thermalling turn), by comparing the
// max/min groundspeed around the circle. Needs the gps object (Gps.h)
// for speed/course; no other dependencies.
//
// WIND_MIN_CIRCLE_SPEED_KPH / WIND_MAX_CIRCLE_SPEED_KPH /
// WIND_MAX_ESTIMATE_KPH are declared under this same banner in the
// original file but are not referenced anywhere in updateWindEstimator()
// -- the function uses hard-coded literals instead (10.0f, 150.0f, the
// 15-150 airspeed range, 75.0f). Carried forward unused, exactly as
// found in the source -- not removed, per the "no cleanup" constraint.
// =====================================================

#define WIND_MIN_CIRCLE_SPEED_KPH 15.0f
#define WIND_MAX_CIRCLE_SPEED_KPH 80.0f
#define WIND_MAX_ESTIMATE_KPH 50.0f

extern float estimatedWindSpeedKph;
extern float estimatedAirspeedKph;
extern float estimatedWindDirectionDeg;
extern bool windEstimateValid;

// Circle detection state
extern bool windCircleActive;
extern float windCircleStartTrack;
extern float windCircleAccumulatedDeg;
extern float windCircleLastTrack;

// Speed extrema during the circle
extern float windCircleMaxSpeedKph;
extern float windCircleMinSpeedKph;

// Track at minimum groundspeed
extern float windCircleMinSpeedTrack;

// GPS validity
extern bool windEstimatorInitialized;

void updateWindEstimator();
