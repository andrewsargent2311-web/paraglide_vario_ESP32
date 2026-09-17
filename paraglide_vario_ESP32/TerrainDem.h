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

#include <SD_MMC.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_timer.h>

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
static const size_t DEM_FILENAME_MAX = 64;

// Cached after the first successful open -- the tile doesn't change at
// runtime FOR A GIVEN FILENAME, so there's no need to re-read these 50
// bytes on every lookup. g_demOpenFile tracks which filename this cache
// (and g_demFile below) actually belongs to, so a tile *switch* (a
// different demFile string passed in) correctly invalidates and reloads
// both, instead of silently continuing to serve the previous tile's data.
static DemHeader g_demHeader;
static bool g_demHeaderValid = false;
static char g_demOpenFile[DEM_FILENAME_MAX] = {0};

// The persistent, kept-open handle used for cell lookups. Re-opening a
// fresh File and seeking into it cold is what made the first lookup after
// a tile load slow enough to trip the task watchdog: FAT32 has to walk the
// cluster chain from the start of the file to reach an arbitrary offset,
// and a brand-new handle has no cached position to start from. Keeping one
// handle open across calls lets the filesystem layer reuse its last seek
// position, so lookups near the previous one (which is the normal case --
// the aircraft moves gradually) stay cheap instead of re-walking the chain
// from zero every single call.
static File g_demFile;
static bool g_demFileOpen = false;

static void closeDemFile() {
  if (g_demFileOpen) {
    g_demFile.close();
    g_demFileOpen = false;
  }
}

// Seeks forward to `target` in bounded chunks, yielding to the scheduler
// between each one, instead of one raw f.seek() call.
//
// Why: FAT32 has no random-access shortcut -- reaching an arbitrary offset
// deep in a large file means walking the cluster chain, and a single
// f.seek() call does that walk as one uninterruptible library call. For a
// ~99MB tile, that walk was slow enough on its own (no other work
// involved) to starve the IDLE0 task on core 0 for longer than the task
// watchdog's timeout -- esp_task_wdt_reset() can't fix that, because the
// problem was never "BackgroundTask failed to feed its own watchdog
// subscription" (it isn't even registered -- see the removed calls and
// their "task not found" errors), it's "IDLE0 never got scheduled at
// all" while the seek ran.
//
// Chunking the same seek into forward steps with vTaskDelay(1) between
// them costs no extra I/O: FatFs continues a FORWARD seek from the file's
// current position rather than re-walking from byte 0 each time, so N
// chunked seeks that add up to the same target do the same total chain
// walk as one big seek -- just with the scheduler able to run IDLE0 (and
// anything else waiting) between chunks instead of being locked out for
// the whole walk in one go.
//
// A *backward* seek (target before the current position) can't use that
// optimisation -- FatFs has to restart from the beginning of the file
// regardless of chunking. This still chunks that walk from 0, so it's at
// least yield-punctuated even though it can't be made cheaper.
static bool seekWithYield(File& f, uint32_t target) {
  static const uint32_t SEEK_CHUNK_BYTES = 512UL * 1024UL;  // 512KB/step

  uint32_t pos = f.position();

  if (target < pos) {
    if (!f.seek(0)) return false;
    pos = 0;
  }

  while (pos < target) {
    uint32_t next = pos + SEEK_CHUNK_BYTES;
    if (next > target) next = target;

    if (!f.seek(next)) return false;
    pos = next;

    if (pos < target) {
      vTaskDelay(1);
    }
  }

  return true;
}

// (Re)opens demFile and reloads its header if this is a different file
// than what's currently cached/open, or if nothing has loaded successfully
// yet. Safe to call every lookup -- once it's loaded and the filename
// hasn't changed, this is just a strcmp.
static bool loadDemHeaderIfNeeded(const char* demFile) {
  if (g_demHeaderValid &&
      g_demFileOpen &&
      strncmp(g_demOpenFile, demFile, DEM_FILENAME_MAX) == 0) {
    return true;
  }

  // Either a genuinely new tile, or a stale/failed previous attempt --
  // either way, drop everything and start clean.
  closeDemFile();
  g_demHeaderValid = false;

  const int64_t openStartUs = esp_timer_get_time();
  g_demFile = SD_MMC.open(demFile, FILE_READ);
  if (!g_demFile) {
    Serial.println("[DEM] Could not open DEM file");
    return false;
  }
  g_demFileOpen = true;

  size_t n = g_demFile.read((uint8_t*)&g_demHeader, DEM_HEADER_SIZE);
  const int64_t openUs = esp_timer_get_time() - openStartUs;

  if (n != DEM_HEADER_SIZE || memcmp(g_demHeader.magic, "ADEM", 4) != 0) {
    Serial.println("[DEM] Bad header -- wrong file or corrupt");
    closeDemFile();
    return false;
  }

  Serial.printf("[DEM] Loaded tile: %ux%u cells, origin (%.5f, %.5f) -- open+header took %lldms\n",
                g_demHeader.rows, g_demHeader.cols,
                g_demHeader.originLat, g_demHeader.originLon,
                openUs / 1000);

  strncpy(g_demOpenFile, demFile, DEM_FILENAME_MAX - 1);
  g_demOpenFile[DEM_FILENAME_MAX - 1] = '\0';
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
  const int64_t callStartUs = esp_timer_get_time();

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

  if (!g_demFileOpen) return false;  // shouldn't happen -- loadDemHeaderIfNeeded() would have returned false above

  // col0 and col0+1 are adjacent int16_t cells in the file, so each row's
  // pair can be read in one seek+read instead of two -- 4 seeks down to 2.
  auto readCellPair = [&](uint32_t r, uint32_t c, int16_t& a, int16_t& b) -> bool {
    uint32_t offset = DEM_HEADER_SIZE +
                       (r * g_demHeader.cols + c) * (uint32_t)sizeof(int16_t);
    if (!seekWithYield(g_demFile, offset)) return false;
    int16_t pair[2];
    if (g_demFile.read((uint8_t*)pair, sizeof(pair)) != sizeof(pair)) return false;
    a = pair[0];
    b = pair[1];
    return true;
  };

  const int64_t seekStartUs = esp_timer_get_time();
  int16_t v00, v01, v10, v11;
  bool ok = readCellPair(row0, col0, v00, v01);
  ok = ok && readCellPair(row0 + 1, col0, v10, v11);
  const int64_t seekUs = esp_timer_get_time() - seekStartUs;

  const int64_t totalUs = esp_timer_get_time() - callStartUs;
  // Only log when it's slow enough to matter -- this runs on every
  // background-task cycle, so a print on every fast call would just be
  // noise. 50ms is well above what a warm lookup near the previous
  // position should ever take.
  if (totalUs > 50000) {
    Serial.printf(
      "[DEM] slow lookup: %lldms total (%lldms in seeks)\n",
      totalUs / 1000, seekUs / 1000);
  }

  if (!ok) {
    // The persistent handle may have landed in a bad state (e.g. an SD
    // hiccup) -- drop it so the next call reopens clean rather than
    // repeatedly failing silently for the rest of the flight.
    closeDemFile();
    g_demHeaderValid = false;
    return false;
  }

  if (v00 == g_demHeader.nodata || v01 == g_demHeader.nodata ||
      v10 == g_demHeader.nodata || v11 == g_demHeader.nodata) {
    return false;
  }

  double top = v00 + (v01 - v00) * fc;
  double bottom = v10 + (v11 - v10) * fc;
  elevationOut = (float)(top + (bottom - top) * fr);
  return true;
}
