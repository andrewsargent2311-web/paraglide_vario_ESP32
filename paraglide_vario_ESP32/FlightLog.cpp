#include "FlightLog.h"
#include "IgcRecorder.h"  // igcRecording, igcFilename
#include "SdCard.h"
#include "Gps.h"
#include "Vario.h"
#include "Display.h"  // displayDirty

#include <SD_MMC.h>
#include <math.h>
#include <string.h>
#include <time.h>

#define FLIGHT_LOG_CSV_PATH "/FLIGHTS.CSV"
#define FLIGHT_LOG_CHECKPOINT_PATH "/FLIGHT.CUR"
#define FLIGHT_LOG_CSV_HEADER "file,start_utc_epoch,duration_s,max_alt_m,best_climb_ms"

#define FLIGHT_LOG_SAMPLE_MS 1000UL
#define FLIGHT_LOG_CHECKPOINT_MS 60000UL
// Only the end of FLIGHTS.CSV is read: at about 50 bytes a line, this is many
// more flights than the screen lists, however long the file gets.
#define FLIGHT_LOG_TAIL_BYTES 2048
#define FLIGHT_LOG_MAX_CSV_LINES 48
#define FLIGHT_LOG_MAX_DIR_ENTRIES 300
// A climb rate above this can't be real. It's the recorder's own limit.
#define FLIGHT_LOG_MAX_CLIMB_MS IGC_MAX_VERTICAL_SPEED_MPS
#define FLIGHT_LOG_MIN_VALID_EPOCH 1704067200UL  // as in startIgcRecording()

// One recording's summary, as it builds up.
struct FlightSummary {
  char file[32];
  uint32_t startEpoch;
  uint32_t durationSec;
  bool hasMaxAlt;
  float maxAltM;
  bool hasBestClimb;
  float bestClimbMs;
};

// Handed from Core 1 to Core 0, guarded by a short critical section. A second
// finish arriving before the first is written would replace it, but it takes
// over 10 s to start a recording and Core 0 checks every 500 ms. Should it ever
// happen, that flight is still listed from its IGC file.
static portMUX_TYPE handoffMux = portMUX_INITIALIZER_UNLOCKED;
static FlightSummary pendingCheckpoint;
static FlightSummary pendingFinish;
static bool checkpointPending = false;
static bool finishPending = false;

static volatile FlightLogListState listState = FLIGHT_LIST_IDLE;
static volatile bool listRequested = false;
static FlightLogEntry listEntries[FLIGHT_LOG_MAX_LISTED];
static uint8_t listCount = 0;

// =====================================================
// CORE 1: FOLLOWING THE RECORDING
// =====================================================
static void takeSample(FlightSummary& summary) {
  float altitudeM = NAN;
  if (baroIsFresh() && windowCount > 0) {
    altitudeM = currentAltitudeM;
  } else if (gps.altitude.isValid() && gps.altitude.age() < 2000) {
    altitudeM = gps.altitude.meters();
  }
  if (isfinite(altitudeM) && (!summary.hasMaxAlt || altitudeM > summary.maxAltM)) {
    summary.maxAltM = altitudeM;
    summary.hasMaxAlt = true;
  }

  // The averaged climb rate, as shown on screen, so one noisy sample can't
  // become the flight's best climb.
  if (baroIsFresh() && isfinite(currentClimbRateAvgMS) &&
      currentClimbRateAvgMS <= FLIGHT_LOG_MAX_CLIMB_MS &&
      (!summary.hasBestClimb || currentClimbRateAvgMS > summary.bestClimbMs)) {
    summary.bestClimbMs = currentClimbRateAvgMS;
    summary.hasBestClimb = true;
  }
}

void flightLogUpdate() {
  static bool wasRecording = false;
  static FlightSummary active;
  static unsigned long startMs = 0;
  static unsigned long lastSampleMs = 0;
  static unsigned long lastCheckpointMs = 0;

  const unsigned long now = millis();

  if (igcRecording && !wasRecording) {
    memset(&active, 0, sizeof(active));
    strlcpy(active.file, igcFilename, sizeof(active.file));
    const time_t epoch = time(nullptr);
    active.startEpoch = (epoch >= (time_t)FLIGHT_LOG_MIN_VALID_EPOCH) ? (uint32_t)epoch : 0;
    startMs = now;
    lastSampleMs = now;
    lastCheckpointMs = now;
    takeSample(active);
  } else if (igcRecording) {
    if (now - lastSampleMs >= FLIGHT_LOG_SAMPLE_MS) {
      lastSampleMs = now;
      takeSample(active);
    }
    if (now - lastCheckpointMs >= FLIGHT_LOG_CHECKPOINT_MS) {
      lastCheckpointMs = now;
      active.durationSec = (now - startMs) / 1000UL;
      portENTER_CRITICAL(&handoffMux);
      pendingCheckpoint = active;
      checkpointPending = true;
      portEXIT_CRITICAL(&handoffMux);
    }
  } else if (wasRecording) {
    active.durationSec = (now - startMs) / 1000UL;
    portENTER_CRITICAL(&handoffMux);
    pendingFinish = active;
    finishPending = true;
    checkpointPending = false;
    portEXIT_CRITICAL(&handoffMux);
  }

  wasRecording = igcRecording;
}

// =====================================================
// CORE 0: READING AND WRITING THE FILES
// Everything below runs in flightLogService(), holding sdMutex.
// =====================================================

// Days since 1970-01-01 for a calendar date.
static int32_t daysFromCivil(int year, int month, int day) {
  year -= (month <= 2) ? 1 : 0;
  const int era = (year >= 0 ? year : year - 399) / 400;
  const int yearOfEra = year - era * 400;
  const int dayOfYear = (153 * (month + (month > 2 ? -3 : 9)) + 2) / 5 + day - 1;
  const int dayOfEra = yearOfEra * 365 + yearOfEra / 4 - yearOfEra / 100 + dayOfYear;
  return era * 146097 + dayOfEra - 719468;
}

static void formatSummaryLine(const FlightSummary& s, char* out, size_t outSize) {
  char alt[16] = "";
  char climb[16] = "";
  if (s.hasMaxAlt) snprintf(alt, sizeof(alt), "%.0f", s.maxAltM);
  if (s.hasBestClimb) snprintf(climb, sizeof(climb), "%.1f", s.bestClimbMs);
  snprintf(out, outSize, "%s,%lu,%lu,%s,%s", s.file, (unsigned long)s.startEpoch,
           (unsigned long)s.durationSec, alt, climb);
}

static bool appendCsvLine(const char* line) {
  const bool isNew = !SD_MMC.exists(FLIGHT_LOG_CSV_PATH);
  File file = SD_MMC.open(FLIGHT_LOG_CSV_PATH, FILE_APPEND);
  if (!file) {
    Serial.println("[FLIGHTS] Could not open " FLIGHT_LOG_CSV_PATH);
    return false;
  }
  if (isNew) file.println(FLIGHT_LOG_CSV_HEADER);
  const bool ok = file.println(line) > 0;
  file.close();
  return ok;
}

// A checkpoint left from a recording that ended with the power going off.
static void recoverCheckpoint() {
  if (!SD_MMC.exists(FLIGHT_LOG_CHECKPOINT_PATH)) return;

  File file = SD_MMC.open(FLIGHT_LOG_CHECKPOINT_PATH, FILE_READ);
  if (!file) return;
  char line[96];
  const size_t got = file.readBytesUntil('\n', line, sizeof(line) - 1);
  file.close();
  line[got] = '\0';
  if (got > 0 && line[got - 1] == '\r') line[got - 1] = '\0';

  if (line[0] == '/' && appendCsvLine(line)) {
    Serial.printf("[FLIGHTS] Saved the summary of an unfinished recording: %s\n", line);
  }
  SD_MMC.remove(FLIGHT_LOG_CHECKPOINT_PATH);
}

// Splits "file,epoch,duration,alt,climb". Alt and climb may be blank.
static bool parseCsvLine(char* line, char* fileOut, size_t fileOutSize, FlightLogEntry& entry) {
  char* fields[5] = {};
  char* cursor = line;
  for (int i = 0; i < 5; i++) {
    fields[i] = cursor;
    char* comma = strchr(cursor, ',');
    if (comma == nullptr) {
      if (i != 4) return false;
      break;
    }
    *comma = '\0';
    cursor = comma + 1;
  }
  if (fields[0][0] != '/') return false;  // the header line, or damage

  strlcpy(fileOut, fields[0], fileOutSize);
  memset(&entry, 0, sizeof(entry));
  entry.startEpoch = strtoul(fields[1], nullptr, 10);
  entry.durationSec = strtoul(fields[2], nullptr, 10);
  entry.hasDuration = true;
  entry.hasSummary = true;
  entry.hasMaxAlt = fields[3][0] != '\0';
  entry.maxAltM = atof(fields[3]);
  entry.hasBestClimb = fields[4][0] != '\0';
  entry.bestClimbMs = atof(fields[4]);
  return true;
}

// Newest-first insert, keeping only the newest FLIGHT_LOG_MAX_LISTED.
template <typename T>
static void insertNewestFirst(T* items, uint8_t& count, const T& item, uint32_t (*epochOf)(const T&)) {
  uint8_t position = 0;
  while (position < count && epochOf(items[position]) >= epochOf(item)) position++;
  if (position >= FLIGHT_LOG_MAX_LISTED) return;
  const uint8_t last = (count < FLIGHT_LOG_MAX_LISTED) ? count : FLIGHT_LOG_MAX_LISTED - 1;
  for (uint8_t i = last; i > position; i--) items[i] = items[i - 1];
  items[position] = item;
  if (count < FLIGHT_LOG_MAX_LISTED) count++;
}

static uint32_t entryEpoch(const FlightLogEntry& entry) {
  return entry.startEpoch;
}

// Reads the UTC start time from a "/DD_MM_YYYY_HHMMSS[_n].IGC" filename.
// Recordings made before the clock was set are named FLT_... and have none.
static bool epochFromIgcName(const char* path, uint32_t& epochOut) {
  const char* name = strrchr(path, '/');
  name = (name != nullptr) ? name + 1 : path;

  int day, month, year, hour, minute, second;
  if (sscanf(name, "%2d_%2d_%4d_%2d%2d%2d", &day, &month, &year, &hour, &minute, &second) != 6) {
    return false;
  }
  if (month < 1 || month > 12 || day < 1 || day > 31 || year < 2000 ||
      hour > 23 || minute > 59 || second > 60) {
    return false;
  }
  epochOut = (uint32_t)daysFromCivil(year, month, day) * 86400UL +
             hour * 3600UL + minute * 60UL + second;
  return true;
}

static bool secondsFromBRecord(const char* record, int32_t& secondsOut) {
  if (record[0] != 'B') return false;
  for (int i = 1; i <= 6; i++) {
    if (record[i] < '0' || record[i] > '9') return false;
  }
  secondsOut = ((record[1] - '0') * 10 + (record[2] - '0')) * 3600L +
               ((record[3] - '0') * 10 + (record[4] - '0')) * 60L +
               (record[5] - '0') * 10 + (record[6] - '0');
  return true;
}

// Finds the last line in buf starting with a B-record, or the first if !last.
static bool findBRecord(const char* buf, size_t len, bool last, int32_t& secondsOut) {
  bool found = false;
  for (size_t i = 0; i + 7 <= len; i++) {
    if ((i == 0 || buf[i - 1] == '\n') && secondsFromBRecord(buf + i, secondsOut)) {
      found = true;
      if (!last) return true;
    }
  }
  return found;
}

// Duration from the first and last track points, without reading the whole file.
static bool igcDurationFromFile(const char* path, uint32_t& durationOut) {
  File file = SD_MMC.open(path, FILE_READ);
  if (!file) return false;

  char buf[256];
  int32_t firstSeconds = 0;
  int32_t lastSeconds = 0;

  size_t got = file.read((uint8_t*)buf, sizeof(buf));
  bool ok = findBRecord(buf, got, false, firstSeconds);

  if (ok) {
    const size_t size = file.size();
    const size_t from = (size > sizeof(buf)) ? size - sizeof(buf) : 0;
    file.seek(from);
    got = file.read((uint8_t*)buf, sizeof(buf));
    ok = findBRecord(buf, got, true, lastSeconds);
  }
  file.close();
  if (!ok) return false;

  // Track points hold only a time of day: a flight can cross UTC midnight.
  if (lastSeconds < firstSeconds) lastSeconds += 86400L;
  durationOut = (uint32_t)(lastSeconds - firstSeconds);
  return true;
}

static void loadFlightList() {
  static char tail[FLIGHT_LOG_TAIL_BYTES + 1];
  static char knownFiles[FLIGHT_LOG_MAX_CSV_LINES][32];
  uint8_t knownCount = 0;

  listCount = 0;

  // 1. The summaries, from the end of FLIGHTS.CSV.
  File csv = SD_MMC.open(FLIGHT_LOG_CSV_PATH, FILE_READ);
  if (csv) {
    const size_t size = csv.size();
    const size_t from = (size > FLIGHT_LOG_TAIL_BYTES) ? size - FLIGHT_LOG_TAIL_BYTES : 0;
    csv.seek(from);
    const size_t got = csv.read((uint8_t*)tail, FLIGHT_LOG_TAIL_BYTES);
    csv.close();
    tail[got] = '\0';

    // Starting partway through the file, the first line may be cut off.
    char* line = tail;
    if (from > 0) {
      char* newline = strchr(tail, '\n');
      line = (newline != nullptr) ? newline + 1 : tail + got;
    }

    while (*line != '\0') {
      char* newline = strchr(line, '\n');
      if (newline != nullptr) *newline = '\0';
      const size_t len = strlen(line);
      if (len > 0 && line[len - 1] == '\r') line[len - 1] = '\0';

      char file[32];
      FlightLogEntry entry;
      if (parseCsvLine(line, file, sizeof(file), entry)) {
        // The power failing between saving a line and removing the checkpoint
        // could save a flight twice. Keep just one.
        bool duplicate = false;
        for (uint8_t i = 0; i < knownCount; i++) {
          if (strcmp(knownFiles[i], file) == 0) duplicate = true;
        }
        if (!duplicate) {
          if (knownCount < FLIGHT_LOG_MAX_CSV_LINES) strlcpy(knownFiles[knownCount++], file, sizeof(knownFiles[0]));
          insertNewestFirst<FlightLogEntry>(listEntries, listCount, entry, entryEpoch);
        }
      }
      if (newline == nullptr) break;
      line = newline + 1;
    }
  }

  // 2. Recordings with no summary: date and time from the name. They only get
  // a place if they're among the newest, so durations are read just for those.
  struct OlderFlight {
    char path[40];
    uint32_t epoch;
  };
  static OlderFlight older[FLIGHT_LOG_MAX_LISTED];
  uint8_t olderCount = 0;

  File root = SD_MMC.open("/");
  if (root) {
    for (int scanned = 0; scanned < FLIGHT_LOG_MAX_DIR_ENTRIES; scanned++) {
      bool isDirectory = false;
      String path = root.getNextFileName(&isDirectory);
      if (path.length() == 0) break;
      if (isDirectory || !(path.endsWith(".IGC") || path.endsWith(".igc"))) continue;
      if (igcRecording && path == igcFilename) continue;

      bool known = false;
      for (uint8_t i = 0; i < knownCount; i++) {
        if (path == knownFiles[i]) known = true;
      }
      if (known) continue;

      OlderFlight flight;
      if (!epochFromIgcName(path.c_str(), flight.epoch)) continue;
      strlcpy(flight.path, path.c_str(), sizeof(flight.path));
      insertNewestFirst<OlderFlight>(older, olderCount, flight,
                                     [](const OlderFlight& f) { return f.epoch; });
    }
    root.close();
  }

  for (uint8_t i = 0; i < olderCount; i++) {
    FlightLogEntry entry;
    memset(&entry, 0, sizeof(entry));
    entry.startEpoch = older[i].epoch;
    entry.hasDuration = igcDurationFromFile(older[i].path, entry.durationSec);
    insertNewestFirst<FlightLogEntry>(listEntries, listCount, entry, entryEpoch);
  }
}

void flightLogService() {
  static bool recovered = false;

  if (listRequested && !sdCardOK) {
    listRequested = false;
    listCount = 0;
    listState = FLIGHT_LIST_READY;
    displayDirty = true;
    return;
  }
  if (!sdCardOK || sdMutex == nullptr) return;

  FlightSummary checkpoint;
  FlightSummary finish;
  portENTER_CRITICAL(&handoffMux);
  const bool haveCheckpoint = checkpointPending;
  const bool haveFinish = finishPending;
  if (haveCheckpoint) checkpoint = pendingCheckpoint;
  if (haveFinish) finish = pendingFinish;
  portEXIT_CRITICAL(&handoffMux);

  if (recovered && !haveCheckpoint && !haveFinish && !listRequested) return;

  // If the card is busy, everything above is still pending for the next pass.
  if (xSemaphoreTake(sdMutex, pdMS_TO_TICKS(200)) != pdTRUE) return;

  if (!recovered) {
    recoverCheckpoint();
    recovered = true;
  }

  char line[96];

  if (haveCheckpoint) {
    formatSummaryLine(checkpoint, line, sizeof(line));
    File file = SD_MMC.open(FLIGHT_LOG_CHECKPOINT_PATH, FILE_WRITE);
    if (file) {
      file.println(line);
      file.close();
    }
  }

  if (haveFinish) {
    formatSummaryLine(finish, line, sizeof(line));
    if (appendCsvLine(line)) {
      SD_MMC.remove(FLIGHT_LOG_CHECKPOINT_PATH);
      Serial.printf("[FLIGHTS] Saved: %s\n", line);
    }
  }

  // Clear what was just handled, unless Core 1 has handed over something newer.
  portENTER_CRITICAL(&handoffMux);
  if (haveCheckpoint && pendingCheckpoint.durationSec == checkpoint.durationSec) checkpointPending = false;
  if (haveFinish && pendingFinish.durationSec == finish.durationSec &&
      strcmp(pendingFinish.file, finish.file) == 0) {
    finishPending = false;
  }
  portEXIT_CRITICAL(&handoffMux);

  if (listRequested) {
    listRequested = false;
    loadFlightList();
    listState = FLIGHT_LIST_READY;
    displayDirty = true;
  }

  xSemaphoreGive(sdMutex);
}

// =====================================================
// THE LIST, FOR THE MENU
// =====================================================
void flightLogRequestList() {
  listState = FLIGHT_LIST_LOADING;
  listRequested = true;
}

FlightLogListState flightLogListState() {
  return listState;
}

uint8_t flightLogListCount() {
  return (listState == FLIGHT_LIST_READY) ? listCount : 0;
}

const FlightLogEntry& flightLogListEntry(uint8_t index) {
  return listEntries[index];
}
