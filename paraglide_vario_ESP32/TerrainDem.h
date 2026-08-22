#pragma once
// TerrainDem.h -- ground elevation lookup from a pre-processed DEM tile
// stored on the SD card as a flat binary grid ("ADEM" format).
//
// The .ADEM file is produced offline from a LINZ DEM GeoTIFF by
// dem_to_agldem.py (downscaled + reprojected to plain WGS84 lat/lon), so
// this code never has to parse a GeoTIFF or do any map projection maths --
// it's pure arithmetic straight from (lat, lon) to a byte offset.
//
// File layout:
//   DemHeader (packed, see below)
//   rows * cols int16_t elevations in metres, row-major, row 0 = the
//   northernmost row, col 0 = the westernmost column. nodata cells are
//   INT16_MIN-style sentinel stored in header.nodata (-32768).
//
// Call pattern mirrors findNearestControlledAirspace()/OpenAirScanner.h:
// pass the SD filename and a position each call, take sdMutex around it.

#include <SD.h>
#include <string.h>

#pragma pack(push, 1)
struct DemHeader {
  char magic[4];        // "ADEM"
  uint16_t version;      // format version, currently 1
  uint16_t reserved;
  double originLat;      // latitude of row 0 (NW corner, northernmost row)
  double originLon;      // longitude of col 0 (NW corner, westernmost col)
  double cellSizeLat;    // degrees latitude per row (positive; south = +row)
  double cellSizeLon;    // degrees longitude per col (positive; east = +col)
  uint32_t rows;
  uint32_t cols;
  int16_t nodata;        // sentinel value for missing data (-32768)
};
#pragma pack(pop)

static const size_t DEM_HEADER_SIZE = sizeof(DemHeader);  // 50 bytes, packed

// Cached after the first successful open -- the tile doesn't change at
// runtime, so there's no need to re-read these 50 bytes on every lookup.
// (Mirrors sharedGpsAltitudeFeet-style module statics used elsewhere.)
static DemHeader g_demHeader;
static bool g_demHeaderAttempted = false;
static bool g_demHeaderValid = false;

// Loads and validates the header once. Safe to call every lookup --
// after the first attempt (success or failure) it's just a bool check.
static bool loadDemHeaderIfNeeded(const char* demFile) {
  if (g_demHeaderAttempted) return g_demHeaderValid;
  g_demHeaderAttempted = true;

  File f = SD.open(demFile, FILE_READ);
  if (!f) {
    Serial.println("[DEM] Could not open DEM file");
    return false;
  }

  size_t n = f.read((uint8_t*)&g_demHeader, DEM_HEADER_SIZE);
  f.close();

  if (n != DEM_HEADER_SIZE || memcmp(g_demHeader.magic, "ADEM", 4) != 0) {
    Serial.println("[DEM] Bad header -- wrong file or corrupt");
    return false;
  }

  Serial.printf("[DEM] Loaded tile: %ux%u cells, origin (%.5f, %.5f)\n",
                g_demHeader.rows, g_demHeader.cols,
                g_demHeader.originLat, g_demHeader.originLon);
  g_demHeaderValid = true;
  return true;
}

// Looks up ground elevation (metres, MSL) at (lat, lon) via bilinear
// interpolation of the four surrounding grid cells.
//
// Returns false (elevationOut untouched) if:
//   - the DEM file couldn't be opened/parsed
//   - (lat, lon) falls outside the downloaded tile
//   - any of the 4 surrounding cells is nodata (e.g. near a warped edge)
//
// Call this the same way findNearestControlledAirspace() is called: under
// sdMutex, from the background task, not from loop()/render code.
static bool getGroundElevationM(const char* demFile, double lat, double lon,
                                 float& elevationOut) {
  if (!loadDemHeaderIfNeeded(demFile)) return false;

  double rowF = (g_demHeader.originLat - lat) / g_demHeader.cellSizeLat;
  double colF = (lon - g_demHeader.originLon) / g_demHeader.cellSizeLon;

  if (rowF < 0 || colF < 0 ||
      rowF >= (double)(g_demHeader.rows - 1) ||
      colF >= (double)(g_demHeader.cols - 1)) {
    return false;  // outside the downloaded tile -- no data for this fix
  }

  uint32_t row0 = (uint32_t)rowF;
  uint32_t col0 = (uint32_t)colF;
  double fr = rowF - row0;
  double fc = colF - col0;

  File f = SD.open(demFile, FILE_READ);
  if (!f) return false;

  auto readCell = [&](uint32_t r, uint32_t c, int16_t& out) -> bool {
    uint32_t offset = DEM_HEADER_SIZE +
                       (r * g_demHeader.cols + c) * (uint32_t)sizeof(int16_t);
    if (!f.seek(offset)) return false;
    return f.read((uint8_t*)&out, sizeof(int16_t)) == sizeof(int16_t);
  };

  int16_t v00, v01, v10, v11;
  bool ok = readCell(row0, col0, v00) &&
            readCell(row0, col0 + 1, v01) &&
            readCell(row0 + 1, col0, v10) &&
            readCell(row0 + 1, col0 + 1, v11);
  f.close();

  if (!ok) return false;
  if (v00 == g_demHeader.nodata || v01 == g_demHeader.nodata ||
      v10 == g_demHeader.nodata || v11 == g_demHeader.nodata) {
    return false;
  }

  double top = v00 + (v01 - v00) * fc;
  double bottom = v10 + (v11 - v10) * fc;
  elevationOut = (float)(top + (bottom - top) * fr);
  return true;
}
