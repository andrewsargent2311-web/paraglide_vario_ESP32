#pragma once
// CloudBase.h -- rough cloud-base (lifting condensation level) estimate.
// Header-only, no Arduino dependencies, so it can be tested on a PC.
#include <math.h>

// Dew point (deg C) from air temp (deg C) and relative humidity (%), Magnus formula.
inline float cloudDewPointC(float tC, float rhPct) {
  if (isnan(tC) || isnan(rhPct)) return NAN;
  if (rhPct < 1.0f) rhPct = 1.0f;
  if (rhPct > 100.0f) rhPct = 100.0f;
  const float a = 17.625f, b = 243.04f;
  float g = logf(rhPct / 100.0f) + (a * tC) / (b + tC);
  return (b * g) / (a - g);
}

// Height (m) from the sensor up to cloud base.
//  tAirC : true outside air temp (deg C)
//  tdC   : dew point (deg C)
//  pHpa  : local pressure (hPa)
// Uses Bolton's LCL temperature, then converts to a height with the pressure.
inline float cloudBaseHeightM(float tAirC, float tdC, float pHpa) {
  if (isnan(tAirC) || isnan(tdC) || isnan(pHpa) || pHpa < 300.0f) return NAN;
  if (tdC >= tAirC) return 0.0f;                    // at/inside cloud
  float tK = tAirC + 273.15f, tdK = tdC + 273.15f;
  float tLclK = 1.0f / (1.0f / (tdK - 56.0f) + logf(tK / tdK) / 800.0f) + 56.0f;
  float pLcl = pHpa * powf(tLclK / tK, 3.5f);        // 1/0.2857
  float tMeanK = 0.5f * (tK + tLclK);
  return 29.271f * tMeanK * logf(pHpa / pLcl);       // hypsometric equation
}
