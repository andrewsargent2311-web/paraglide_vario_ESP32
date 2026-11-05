#include "TrafficIcons.h"
#include "Display.h"  // u8g2

#include <math.h>
#include <string.h>

// =====================================================
// ICON SHAPES
// Each icon is a few overlapping polygons, in pixels from the icon centre
// with the nose pointing up (-y).
// =====================================================
struct IconPoint {
  float x, y;
};

struct IconPoly {
  const IconPoint* points;
  uint8_t count;
};

struct IconShape {
  const IconPoly* polys;
  uint8_t count;
};

#define POLY(points) { points, (uint8_t)(sizeof(points) / sizeof(points[0])) }
#define SHAPE(polys) { polys, (uint8_t)(sizeof(polys) / sizeof(polys[0])) }

// Light aircraft
static const IconPoint ICON0_POLY0[] = { {-1.50f, -10.00f}, {1.50f, -10.00f}, {2.00f, -6.00f}, {2.00f, 2.00f}, {1.00f, 9.00f}, {-1.00f, 9.00f}, {-2.00f, 2.00f}, {-2.00f, -6.00f} };
static const IconPoint ICON0_POLY1[] = { {-9.50f, -6.50f}, {9.50f, -6.50f}, {9.50f, -1.50f}, {-9.50f, -1.50f} };
static const IconPoint ICON0_POLY2[] = { {-4.50f, 6.00f}, {4.50f, 6.00f}, {4.50f, 9.50f}, {-4.50f, 9.50f} };
static const IconPoly ICON0_POLYS[] = { POLY(ICON0_POLY0), POLY(ICON0_POLY1), POLY(ICON0_POLY2) };

// Jet / airliner
static const IconPoint ICON1_POLY0[] = { {0.00f, -10.50f}, {1.50f, -8.00f}, {1.80f, 8.00f}, {0.00f, 10.00f}, {-1.80f, 8.00f}, {-1.50f, -8.00f} };
static const IconPoint ICON1_POLY1[] = { {1.00f, -4.00f}, {10.00f, 3.00f}, {10.00f, 5.50f}, {1.00f, 1.50f}, {-1.00f, 1.50f}, {-10.00f, 5.50f}, {-10.00f, 3.00f}, {-1.00f, -4.00f} };
static const IconPoint ICON1_POLY2[] = { {1.00f, 6.00f}, {5.00f, 9.00f}, {5.00f, 10.50f}, {1.00f, 9.00f}, {-1.00f, 9.00f}, {-5.00f, 10.50f}, {-5.00f, 9.00f}, {-1.00f, 6.00f} };
static const IconPoly ICON1_POLYS[] = { POLY(ICON1_POLY0), POLY(ICON1_POLY1), POLY(ICON1_POLY2) };

// Helicopter
static const IconPoint ICON2_POLY0[] = { {4.20f, -2.50f}, {3.64f, -0.40f}, {2.10f, 1.14f}, {0.00f, 1.70f}, {-2.10f, 1.14f}, {-3.64f, -0.40f}, {-4.20f, -2.50f}, {-3.64f, -4.60f}, {-2.10f, -6.14f}, {-0.00f, -6.70f}, {2.10f, -6.14f}, {3.64f, -4.60f} };
static const IconPoint ICON2_POLY1[] = { {-1.00f, 0.00f}, {1.00f, 0.00f}, {1.00f, 9.50f}, {-1.00f, 9.50f} };
static const IconPoint ICON2_POLY2[] = { {-3.50f, 7.50f}, {3.50f, 7.50f}, {3.50f, 9.80f}, {-3.50f, 9.80f} };
static const IconPoint ICON2_POLY3[] = { {-8.21f, -9.29f}, {6.79f, 5.71f}, {8.21f, 4.29f}, {-6.79f, -10.71f} };
static const IconPoint ICON2_POLY4[] = { {6.79f, -10.71f}, {-8.21f, 4.29f}, {-6.79f, 5.71f}, {8.21f, -9.29f} };
static const IconPoly ICON2_POLYS[] = { POLY(ICON2_POLY0), POLY(ICON2_POLY1), POLY(ICON2_POLY2), POLY(ICON2_POLY3), POLY(ICON2_POLY4) };

// Glider
static const IconPoint ICON3_POLY0[] = { {-12.00f, -2.00f}, {-1.00f, -3.00f}, {1.00f, -3.00f}, {12.00f, -2.00f}, {12.00f, -0.60f}, {1.00f, 0.00f}, {-1.00f, 0.00f}, {-12.00f, -0.60f} };
static const IconPoint ICON3_POLY1[] = { {-0.80f, -8.00f}, {0.80f, -8.00f}, {1.30f, -4.00f}, {0.80f, 9.00f}, {-0.80f, 9.00f}, {-1.30f, -4.00f} };
static const IconPoint ICON3_POLY2[] = { {-3.00f, 7.60f}, {3.00f, 7.60f}, {3.00f, 9.40f}, {-3.00f, 9.40f} };
static const IconPoly ICON3_POLYS[] = { POLY(ICON3_POLY0), POLY(ICON3_POLY1), POLY(ICON3_POLY2) };

// Paraglider
static const IconPoint ICON4_POLY0[] = { {-10.34f, -0.10f}, {-9.36f, -2.30f}, {-8.00f, -4.23f}, {-6.31f, -5.83f}, {-4.36f, -7.02f}, {-2.22f, -7.75f}, {0.00f, -8.00f}, {2.22f, -7.75f}, {4.36f, -7.02f}, {6.31f, -5.83f}, {8.00f, -4.23f}, {9.36f, -2.30f}, {10.34f, -0.10f}, {10.34f, 1.43f}, {9.36f, 0.06f}, {8.00f, -1.15f}, {6.31f, -2.14f}, {4.36f, -2.89f}, {2.22f, -3.35f}, {0.00f, -3.50f}, {-2.22f, -3.35f}, {-4.36f, -2.89f}, {-6.31f, -2.14f}, {-8.00f, -1.15f}, {-9.36f, 0.06f}, {-10.34f, 1.43f} };
static const IconPoint ICON4_POLY1[] = { {-9.97f, -2.55f}, {-0.47f, 7.45f}, {0.47f, 6.55f}, {-9.03f, -3.45f} };
static const IconPoint ICON4_POLY2[] = { {9.03f, -3.45f}, {-0.47f, 6.55f}, {0.47f, 7.45f}, {9.97f, -2.55f} };
static const IconPoint ICON4_POLY3[] = { {2.60f, 7.00f}, {1.84f, 8.84f}, {0.00f, 9.60f}, {-1.84f, 8.84f}, {-2.60f, 7.00f}, {-1.84f, 5.16f}, {-0.00f, 4.40f}, {1.84f, 5.16f} };
static const IconPoly ICON4_POLYS[] = { POLY(ICON4_POLY0), POLY(ICON4_POLY1), POLY(ICON4_POLY2), POLY(ICON4_POLY3) };

// Hang glider
static const IconPoint ICON5_POLY0[] = { {0.00f, -9.00f}, {11.00f, 6.00f}, {11.00f, 8.00f}, {0.00f, 2.50f}, {-11.00f, 8.00f}, {-11.00f, 6.00f} };
static const IconPoint ICON5_POLY1[] = { {-1.00f, 2.00f}, {-1.00f, 8.00f}, {1.00f, 8.00f}, {1.00f, 2.00f} };
static const IconPoint ICON5_POLY2[] = { {2.00f, 8.00f}, {1.41f, 9.41f}, {0.00f, 10.00f}, {-1.41f, 9.41f}, {-2.00f, 8.00f}, {-1.41f, 6.59f}, {-0.00f, 6.00f}, {1.41f, 6.59f} };
static const IconPoly ICON5_POLYS[] = { POLY(ICON5_POLY0), POLY(ICON5_POLY1), POLY(ICON5_POLY2) };

// Same order as TrafficIconType.
static const IconShape ICON_SHAPES[TRAFFIC_ICON_COUNT] = {
  SHAPE(ICON0_POLYS), SHAPE(ICON1_POLYS), SHAPE(ICON2_POLYS),
  SHAPE(ICON3_POLYS), SHAPE(ICON4_POLYS), SHAPE(ICON5_POLYS)
};

// =====================================================
// TYPE LOOKUPS
// =====================================================
TrafficIconType trafficIconFromAdsbCategory(const char* category) {
  if (category == nullptr || strlen(category) != 2) return TRAFFIC_ICON_LIGHT_AIRCRAFT;

  // A1 = light. A2-A6 run from small jets and turboprops up to heavy
  // airliners. A7 = rotorcraft. B1 = glider. B4 = ultralight, hang glider or
  // paraglider: ADS-B can't tell those apart, so it shows as a paraglider.
  if (category[0] == 'A') {
    if (category[1] >= '2' && category[1] <= '6') return TRAFFIC_ICON_JET;
    if (category[1] == '7') return TRAFFIC_ICON_HELICOPTER;
  } else if (category[0] == 'B') {
    if (category[1] == '1') return TRAFFIC_ICON_GLIDER;
    if (category[1] == '4') return TRAFFIC_ICON_PARAGLIDER;
  }
  return TRAFFIC_ICON_LIGHT_AIRCRAFT;
}

TrafficIconType trafficIconFromFanetType(uint8_t aircraftType) {
  switch (aircraftType) {
    case 1: return TRAFFIC_ICON_PARAGLIDER;
    case 2: return TRAFFIC_ICON_HANG_GLIDER;
    case 4: return TRAFFIC_ICON_GLIDER;
    case 6: return TRAFFIC_ICON_HELICOPTER;
    default: return TRAFFIC_ICON_LIGHT_AIRCRAFT;  // including 5 = powered aircraft
  }
}

// =====================================================
// DRAWING
// The rotated polygons are filled into a small bitmap first: a pixel is set
// if its centre falls inside any polygon. Polygons overlap, so the bitmap is
// what lets the outline follow the edge of the whole aircraft, with no lines
// across its middle.
// =====================================================
#define ICON_GRID_RADIUS 15
#define ICON_GRID_SIZE (2 * ICON_GRID_RADIUS + 1)

static bool pointInPolygon(float px, float py, const IconPoint* points, uint8_t count) {
  bool inside = false;
  for (uint8_t i = 0, j = count - 1; i < count; j = i++) {
    if ((points[i].y > py) != (points[j].y > py) &&
        px < (points[j].x - points[i].x) * (py - points[i].y) / (points[j].y - points[i].y) + points[i].x) {
      inside = !inside;
    }
  }
  return inside;
}

void drawTrafficIcon(int x, int y, TrafficIconType type, float headingDeg, bool outlineOnly) {
  if (type < 0 || type >= TRAFFIC_ICON_COUNT) type = TRAFFIC_ICON_LIGHT_AIRCRAFT;

  const float angle = headingDeg * (float)M_PI / 180.0f;
  const float c = cosf(angle);
  const float s = sinf(angle);

  uint32_t rows[ICON_GRID_SIZE] = {};  // bit gx of rows[gy] set = pixel covered
  IconPoint rotated[32];  // large enough for the biggest polygon above

  const IconShape& shape = ICON_SHAPES[type];
  for (uint8_t p = 0; p < shape.count; p++) {
    const IconPoly& poly = shape.polys[p];
    if (poly.count > 32) continue;

    // Only test the pixels the polygon could reach, to keep this quick:
    // up to 40 aircraft can be drawn per frame on Core 1.
    float minX = 1000.0f, maxX = -1000.0f, minY = 1000.0f, maxY = -1000.0f;
    for (uint8_t i = 0; i < poly.count; i++) {
      rotated[i].x = poly.points[i].x * c - poly.points[i].y * s;
      rotated[i].y = poly.points[i].x * s + poly.points[i].y * c;
      if (rotated[i].x < minX) minX = rotated[i].x;
      if (rotated[i].x > maxX) maxX = rotated[i].x;
      if (rotated[i].y < minY) minY = rotated[i].y;
      if (rotated[i].y > maxY) maxY = rotated[i].y;
    }

    int firstX = max((int)floorf(minX), -ICON_GRID_RADIUS);
    int lastX = min((int)ceilf(maxX), ICON_GRID_RADIUS);
    int firstY = max((int)floorf(minY), -ICON_GRID_RADIUS);
    int lastY = min((int)ceilf(maxY), ICON_GRID_RADIUS);

    for (int py = firstY; py <= lastY; py++) {
      for (int px = firstX; px <= lastX; px++) {
        if (pointInPolygon((float)px, (float)py, rotated, poly.count)) {
          rows[py + ICON_GRID_RADIUS] |= (1UL << (px + ICON_GRID_RADIUS));
        }
      }
    }
  }

  for (int gy = 0; gy < ICON_GRID_SIZE; gy++) {
    uint32_t row = rows[gy];
    if (outlineOnly) {
      // Keep a pixel only if a neighbour on any of its 4 sides is empty.
      uint32_t above = (gy > 0) ? rows[gy - 1] : 0;
      uint32_t below = (gy < ICON_GRID_SIZE - 1) ? rows[gy + 1] : 0;
      row &= ~(above & below & (rows[gy] << 1) & (rows[gy] >> 1));
    }
    for (int gx = 0; row != 0 && gx < ICON_GRID_SIZE; gx++) {
      if (row & (1UL << gx)) {
        u8g2.drawPixel(x + gx - ICON_GRID_RADIUS, y + gy - ICON_GRID_RADIUS);
      }
    }
  }
}
