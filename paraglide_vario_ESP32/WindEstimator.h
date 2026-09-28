#pragma once
// WindEstimator -- 360-degree wind / airspeed estimate from GPS ground track.
#include <Arduino.h>

#define WIND_MIN_CIRCLE_SPEED_KPH 15.0f
#define WIND_MAX_CIRCLE_SPEED_KPH 80.0f
#define WIND_MAX_ESTIMATE_KPH 50.0f

extern float estimatedWindSpeedKph;
extern float estimatedAirspeedKph;
extern float estimatedWindDirectionDeg;
extern bool windEstimateValid;

void updateWindEstimator();
