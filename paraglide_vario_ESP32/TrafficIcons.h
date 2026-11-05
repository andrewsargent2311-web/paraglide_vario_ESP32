#ifndef TRAFFIC_ICONS_H
#define TRAFFIC_ICONS_H

#include <Arduino.h>

// =====================================================
// TRAFFIC ICONS (ADS-B page)
// About 20px aircraft-type symbols, drawn as vector shapes and rotated to
// the aircraft's heading. Used by drawADSBPage() (DrawPages.cpp). The
// own-ship centre arrow is separate: see drawGliderHeadingArrow().
// =====================================================
enum TrafficIconType {
  TRAFFIC_ICON_LIGHT_AIRCRAFT = 0,  // also used whenever the type is unknown
  TRAFFIC_ICON_JET,
  TRAFFIC_ICON_HELICOPTER,
  TRAFFIC_ICON_GLIDER,
  TRAFFIC_ICON_PARAGLIDER,
  TRAFFIC_ICON_HANG_GLIDER,
  TRAFFIC_ICON_COUNT
};

// From the adsb.fi "category" field (the ADS-B emitter category, e.g. "A1").
// nullptr or anything unrecognised gives a light aircraft.
TrafficIconType trafficIconFromAdsbCategory(const char* category);

// From the FANET aircraftType (see setAircraftType() in Fanet.h).
TrafficIconType trafficIconFromFanetType(uint8_t aircraftType);

// Draws an icon centred on (x, y). headingDeg is clockwise from the top of
// the screen. outlineOnly draws just the edge, which marks a position older
// than 30 s as an estimate.
void drawTrafficIcon(int x, int y, TrafficIconType type, float headingDeg, bool outlineOnly);

#endif  // TRAFFIC_ICONS_H
