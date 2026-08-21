#include "OpenAirScanner.h"
#include <math.h>

// ---------------------------------------------------------------------------
// Geometry helpers
// ---------------------------------------------------------------------------

static const double EARTH_RADIUS_KM = 6371.0088;
static const double NM_TO_KM = 1.852;

double haversine_km(double lat1, double lon1, double lat2, double lon2) {
  double phi1 = radians(lat1), phi2 = radians(lat2);
  double dphi = radians(lat2 - lat1);
  double dlambda = radians(lon2 - lon1);
  double a = sin(dphi / 2) * sin(dphi / 2) +
             cos(phi1) * cos(phi2) * sin(dlambda / 2) * sin(dlambda / 2);
  double c = 2 * atan2(sqrt(a), sqrt(1 - a));
  return EARTH_RADIUS_KM * c;
}

// Ray-casting point-in-polygon test on raw lat/lon. Fine for this purpose --
// we only need in/out, not a precise metric, and NZ airspace polygons are
// small enough that lat/lon behaves like a local planar grid for this test.
static bool pointInPolygon(double lat, double lon, const double* plat,
                            const double* plon, uint8_t n) {
  bool inside = false;
  for (uint8_t i = 0, j = n - 1; i < n; j = i++) {
    bool intersects = ((plat[i] > lat) != (plat[j] > lat)) &&
        (lon < (plon[j] - plon[i]) * (lat - plat[i]) / (plat[j] - plat[i]) + plon[i]);
    if (intersects) inside = !inside;
  }
  return inside;
}

// Distance from point to segment in a local km-plane (equirectangular
// projection centred on refLat). Good to well under 1% error at NZ regional
// scale (tens to low hundreds of km) -- plenty for a proximity warning.
static double distPointToSegmentKm(double px, double py, double ax, double ay,
                                    double bx, double by) {
  double abx = bx - ax, aby = by - ay;
  double apx = px - ax, apy = py - ay;
  double abLenSq = abx * abx + aby * aby;
  double t = (abLenSq > 1e-12) ? (apx * abx + apy * aby) / abLenSq : 0.0;
  t = t < 0.0 ? 0.0 : (t > 1.0 ? 1.0 : t);
  double cx = ax + t * abx, cy = ay + t * aby;
  double dx = px - cx, dy = py - cy;
  return sqrt(dx * dx + dy * dy);
}

static double polygonDistanceKm(double lat, double lon, const double* plat,
                                 const double* plon, uint8_t n) {
  double kmPerDegLat = 111.32;
  double kmPerDegLon = 111.32 * cos(radians(lat));

  double px = 0, py = 0;  // point is the local origin
  double best = 1e18;
  for (uint8_t i = 0, j = n - 1; i < n; j = i++) {
    double ax = (plon[j] - lon) * kmPerDegLon, ay = (plat[j] - lat) * kmPerDegLat;
    double bx = (plon[i] - lon) * kmPerDegLon, by = (plat[i] - lat) * kmPerDegLat;
    double d = distPointToSegmentKm(px, py, ax, ay, bx, by);
    if (d < best) best = d;
  }
  return best;
}

float resolveAltitudeFt(const Altitude& alt, float groundElev_ft) {
  switch (alt.ref) {
    case ALTREF_SFC: return groundElev_ft;
    case ALTREF_UNL: return 999999.0f;
    case ALTREF_FL:  return alt.value_ft;                  // already ft, see parseAltitude
    case ALTREF_AGL: return alt.value_ft + groundElev_ft;
    case ALTREF_MSL:
    default:         return alt.value_ft;
  }
}

// ---------------------------------------------------------------------------
// OpenAir text parsing
// ---------------------------------------------------------------------------

// Parses one coordinate token like "41:17:00S", "S41:17:00", or
// "41:17:00.5 S" (leading/trailing hemisphere letter, optional decimal
// seconds). Advances *p past the token. Returns signed decimal degrees and
// whether the hemisphere letter identifies it as a latitude (N/S) or
// longitude (E/W).
static bool parseOneCoord(const char*& p, double& outValue, bool& isLat) {
  while (*p == ' ' || *p == '\t') p++;
  if (!*p) return false;

  char hemi = 0;
  if (*p == 'N' || *p == 'S' || *p == 'E' || *p == 'W') {
    hemi = *p;
    p++;
  }

  int deg = 0, min = 0;
  double sec = 0;
  char* end;

  deg = strtol(p, &end, 10);
  if (end == p) return false;
  p = end;
  if (*p == ':') {
    p++;
    min = strtol(p, &end, 10);
    p = end;
  }
  if (*p == ':') {
    p++;
    sec = strtod(p, &end);
    p = end;
  }

  if (!hemi && (*p == 'N' || *p == 'S' || *p == 'E' || *p == 'W')) {
    hemi = *p;
    p++;
  }

  double value = deg + min / 60.0 + sec / 3600.0;
  if (hemi == 'S' || hemi == 'W') value = -value;
  isLat = (hemi == 'N' || hemi == 'S');
  outValue = value;
  return true;
}

static bool parseCoordPair(const String& s, double& lat, double& lon) {
  const char* p = s.c_str();
  double v1, v2;
  bool isLat1, isLat2;
  if (!parseOneCoord(p, v1, isLat1)) return false;
  if (!parseOneCoord(p, v2, isLat2)) return false;

  if (isLat1) { lat = v1; lon = v2; }
  else        { lat = v2; lon = v1; }
  return true;
}

static Altitude parseAltitude(String s) {
  s.trim();
  s.toUpperCase();
  Altitude a{0, ALTREF_MSL};

  if (s.startsWith("SFC") || s.startsWith("GND")) {
    a.ref = ALTREF_SFC;
    return a;
  }
  if (s.startsWith("UNL")) {
    a.ref = ALTREF_UNL;
    return a;
  }
  if (s.startsWith("FL")) {
    a.ref = ALTREF_FL;
    a.value_ft = s.substring(2).toFloat() * 100.0f;
    return a;
  }

  a.value_ft = s.toFloat();  // leading numeric run, ignores trailing text
  if (s.indexOf("AGL") >= 0) a.ref = ALTREF_AGL;
  else a.ref = ALTREF_MSL;   // covers explicit MSL/AMSL and bare "3500ft"
  return a;
}

static bool isControlledClass(const char* classId, const char** controlledClasses,
                               uint8_t numClasses) {
  for (uint8_t i = 0; i < numClasses; i++) {
    if (strcasecmp(classId, controlledClasses[i]) == 0) return true;
  }
  return false;
}

// ---------------------------------------------------------------------------
// In-progress airspace block, evaluated and discarded before the next block
// is read -- this is what keeps memory flat regardless of file size.
// ---------------------------------------------------------------------------

struct BlockState {
  bool active = false;
  char name[OAS_MAX_NAME_LEN] = "";
  char classId[OAS_MAX_CLASS_LEN] = "";
  Altitude floor{0, ALTREF_SFC};
  Altitude ceiling{0, ALTREF_UNL};

  bool haveCenter = false;
  double centerLat = 0, centerLon = 0;
  bool isCircle = false;
  double radius_nm = 0;

  double plat[OAS_MAX_POLY_POINTS];
  double plon[OAS_MAX_POLY_POINTS];
  uint8_t numPoints = 0;

  void reset() {
    active = false;
    name[0] = 0;
    classId[0] = 0;
    floor = {0, ALTREF_SFC};
    ceiling = {0, ALTREF_UNL};
    haveCenter = false;
    isCircle = false;
    radius_nm = 0;
    numPoints = 0;
  }
};

static void evaluateBlock(const BlockState& b, double curLat, double curLon,
                           float curAlt_ft_msl, float groundElev_ft,
                           const char** controlledClasses, uint8_t numClasses,
                           AirspaceResult& best, bool& haveBest) {
  if (!b.active || b.classId[0] == 0) return;
  if (!isControlledClass(b.classId, controlledClasses, numClasses)) return;
  if (!b.isCircle && b.numPoints < 3) return;  // incomplete polygon, skip

  double horizKm;
  bool insideHoriz;

  if (b.isCircle) {
    double d = haversine_km(curLat, curLon, b.centerLat, b.centerLon);
    double r = b.radius_nm * NM_TO_KM;
    insideHoriz = d <= r;
    horizKm = insideHoriz ? 0.0 : (d - r);
  } else {
    insideHoriz = pointInPolygon(curLat, curLon, b.plat, b.plon, b.numPoints);
    horizKm = insideHoriz ? 0.0 : polygonDistanceKm(curLat, curLon, b.plat, b.plon, b.numPoints);
  }

  float floorFt = resolveAltitudeFt(b.floor, groundElev_ft);
  float ceilFt = resolveAltitudeFt(b.ceiling, groundElev_ft);
  bool insideVert = (curAlt_ft_msl >= floorFt) && (curAlt_ft_msl <= ceilFt);
  float vertFt = insideVert ? 0.0f
                 : (curAlt_ft_msl < floorFt ? (floorFt - curAlt_ft_msl)
                                             : (curAlt_ft_msl - ceilFt));

  bool better = !haveBest ||
      (horizKm < best.horizDistance_km) ||
      (horizKm == best.horizDistance_km && vertFt < best.vertDistance_ft);

  if (better) {
    strncpy(best.name, b.name, OAS_MAX_NAME_LEN - 1);
    best.name[OAS_MAX_NAME_LEN - 1] = 0;
    strncpy(best.classId, b.classId, OAS_MAX_CLASS_LEN - 1);
    best.classId[OAS_MAX_CLASS_LEN - 1] = 0;
    best.horizDistance_km = (float)horizKm;
    best.vertDistance_ft = vertFt;
    best.insideHoriz = insideHoriz;
    best.insideVert = insideVert;
    best.floor_ft_msl = floorFt;
    best.ceiling_ft_msl = ceilFt;
    haveBest = true;
  }
}

// ---------------------------------------------------------------------------
// Public entry point
// ---------------------------------------------------------------------------

bool findNearestControlledAirspace(
    const char* filename,
    double curLat, double curLon, float curAlt_ft_msl,
    float groundElev_ft,
    AirspaceResult& out,
    const char** controlledClasses, uint8_t numClasses) {

  File f = SD.open(filename, FILE_READ);
  if (!f) {
    Serial.print("OpenAirScanner: could not open ");
    Serial.println(filename);
    return false;
  }

  BlockState block;
  bool haveBest = false;

  while (f.available()) {
    String line = f.readStringUntil('\n');
    line.trim();
    if (line.length() == 0 || line.startsWith("*")) continue;

    String cmd, rest;
    int sp = line.indexOf(' ');
    if (sp < 0) { cmd = line; rest = ""; }
    else { cmd = line.substring(0, sp); rest = line.substring(sp + 1); }
    rest.trim();
    cmd.toUpperCase();

    if (cmd == "AC") {
      // New block starts -- evaluate and discard whatever we were building.
      if (block.active) {
        evaluateBlock(block, curLat, curLon, curAlt_ft_msl, groundElev_ft,
                      controlledClasses, numClasses, out, haveBest);
      }
      block.reset();
      block.active = true;
      rest.toUpperCase();
      strncpy(block.classId, rest.c_str(), OAS_MAX_CLASS_LEN - 1);
      block.classId[OAS_MAX_CLASS_LEN - 1] = 0;

    } else if (cmd == "AN") {
      strncpy(block.name, rest.c_str(), OAS_MAX_NAME_LEN - 1);
      block.name[OAS_MAX_NAME_LEN - 1] = 0;

    } else if (cmd == "AL") {
      block.floor = parseAltitude(rest);

    } else if (cmd == "AH") {
      block.ceiling = parseAltitude(rest);

    } else if (cmd == "V") {
      // Expect "X=<lat> <lon>" -- sets the reference centre for a
      // following DC (circle) command.
      int eq = rest.indexOf('=');
      if (rest.startsWith("X=") || (eq == 1 && rest.charAt(0) == 'X')) {
        String coords = rest.substring(eq + 1);
        double lat, lon;
        if (parseCoordPair(coords, lat, lon)) {
          block.centerLat = lat;
          block.centerLon = lon;
          block.haveCenter = true;
        }
      }
      // Other V X=... variants (radius sign etc.) are not needed for a
      // simple nearest-distance calculation and are ignored here.

    } else if (cmd == "DC") {
      block.radius_nm = rest.toFloat();
      block.isCircle = block.haveCenter;

    } else if (cmd == "DP") {
      double lat, lon;
      if (parseCoordPair(rest, lat, lon) && block.numPoints < OAS_MAX_POLY_POINTS) {
        block.plat[block.numPoints] = lat;
        block.plon[block.numPoints] = lon;
        block.numPoints++;
        block.isCircle = false;
      }

    }
    // DA/DB (arcs), SP, AY, and other cosmetic/advanced OpenAir commands are
    // intentionally not handled -- arcs are approximated by whatever DP
    // points surround them in the file, which is fine for a proximity
    // warning but not for precise boundary plotting.
  }

  // Evaluate whatever block was still open at EOF.
  if (block.active) {
    evaluateBlock(block, curLat, curLon, curAlt_ft_msl, groundElev_ft,
                  controlledClasses, numClasses, out, haveBest);
  }

  f.close();
  return haveBest;
}
