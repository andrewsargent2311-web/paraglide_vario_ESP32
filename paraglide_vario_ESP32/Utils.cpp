#include "Utils.h"
#include <math.h>
#include <Wire.h>

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

// Bare I2C address probe -- bounded by Wire.setTimeOut(), so it can never
// hang even if nothing responds. Used to skip calling into a sensor
// library's begin()/readTime() at all when the device isn't physically
// present, rather than trusting every third-party library to handle
// "device absent" gracefully internally.
bool i2cDevicePresent(uint8_t addr) {
  Wire.beginTransmission(addr);
  return (Wire.endTransmission() == 0);
}
