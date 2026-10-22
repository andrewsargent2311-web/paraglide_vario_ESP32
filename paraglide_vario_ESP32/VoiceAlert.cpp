#include "VoiceAlert.h"
#include <SD_MMC.h>
#include <Arduino.h>
#include "SdCard.h"
#include <string.h>

#define VOICE_POOL_BYTES (2u * 1024u * 1024u)
#define VOICE_PROFILE_SCAN_MAX_ENTRIES 64
#define VOICE_FILE_RETRIES 3
#define VOICE_READ_CHUNK_BYTES 4096

static_assert(VOICE_CLIP_COUNT == 60, "Voice profile loader expects 60 clips");

// ---- Loaded clip storage (PSRAM) ----
struct LoadedClip {
  int16_t* samples = nullptr;
  uint32_t sampleCount = 0;
};
static LoadedClip g_clips[VOICE_CLIP_COUNT];
static uint8_t* g_voicePool = nullptr;
static uint8_t g_voiceReadBuffer[VOICE_READ_CHUNK_BYTES];
static SemaphoreHandle_t g_clipMutex = nullptr;
static volatile bool g_clipsLoaded = false;
static char g_activeProfile[VOICE_PROFILE_NAME_MAX] = "";

// ---- Playback state ----
static VoiceClipId g_sequence[VOICE_MAX_SENTENCE_CLIPS];
static int g_sequenceLen = 0;
static int g_clipIndex = 0;      // which clip in g_sequence is currently sounding
static uint32_t g_sampleOffset = 0;  // position within g_clips[g_sequence[g_clipIndex]]
static uint32_t g_gapSamplesRemaining = 0;  // silence between clips
static float g_gain = 1.0f;
static volatile bool g_playing = false;

// Core 1 requests switches; the background task on Core 0 owns the reload.
static portMUX_TYPE g_reloadMux = portMUX_INITIALIZER_UNLOCKED;
static bool g_reloadPending = false;
static bool g_reloadLoading = false;
static bool g_lastReloadOk = false;
static char g_requestedProfile[VOICE_PROFILE_NAME_MAX] = "";

enum VoiceFileResult {
  VOICE_FILE_OK,
  VOICE_FILE_LOCK_FAILED,
  VOICE_FILE_OPEN_FAILED,
  VOICE_FILE_BAD_SIZE,
  VOICE_FILE_CHANGED,
  VOICE_FILE_SHORT_READ,
  VOICE_FILE_AUDIO_LOCK_FAILED
};

static bool isProfileChar(char c) {
  return (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') ||
         (c >= '0' && c <= '9') || c == '_' || c == '-';
}

static bool isValidProfileName(const char* name, bool allowDefault) {
  if (name == nullptr) return false;
  size_t length = 0;
  while (length < VOICE_PROFILE_NAME_MAX && name[length] != '\0') {
    if (!isProfileChar(name[length])) return false;
    length++;
  }
  if (length == VOICE_PROFILE_NAME_MAX) return false;
  return allowDefault || length > 0;
}

static bool makeClipPath(const char* profile, int id, char* path, size_t pathSize) {
  int written;
  if (profile[0] == '\0') {
    written = snprintf(path, pathSize, "/VOICE/V%03d.PCM", id);
  } else {
    written = snprintf(path, pathSize, "/VOICE/%s/V%03d.PCM", profile, id);
  }
  return written >= 0 && (size_t)written < pathSize;
}

static bool takeSdMutex() {
  return sdMutex != nullptr &&
         xSemaphoreTake(sdMutex, pdMS_TO_TICKS(200)) == pdTRUE;
}

static void giveSdMutex() {
  if (xSemaphoreGive(sdMutex) != pdTRUE) {
    Serial.println("[Voice] Failed to release sdMutex");
  }
}

static VoiceFileResult inspectVoiceFile(const char* path, bool runtime,
                                        uint32_t& fileBytes) {
  bool lockFailed = false;
  for (uint8_t attempt = 0; attempt < VOICE_FILE_RETRIES; attempt++) {
    if (runtime && !takeSdMutex()) {
      lockFailed = true;
      continue;
    }

    File file = SD_MMC.open(path, FILE_READ);
    if (!file) {
      if (runtime) giveSdMutex();
      continue;
    }

    size_t size = file.size();
    file.close();
    if (runtime) giveSdMutex();
    if (size == 0 || (size % 2u) != 0 || size > VOICE_POOL_BYTES) {
      return VOICE_FILE_BAD_SIZE;
    }
    fileBytes = (uint32_t)size;
    return VOICE_FILE_OK;
  }
  return lockFailed ? VOICE_FILE_LOCK_FAILED : VOICE_FILE_OPEN_FAILED;
}

static VoiceFileResult readVoiceFile(const char* path, bool runtime,
                                     uint32_t expectedBytes, uint8_t* destination) {
  bool lockFailed = false;
  for (uint8_t attempt = 0; attempt < VOICE_FILE_RETRIES; attempt++) {
    if (runtime && !takeSdMutex()) {
      lockFailed = true;
      continue;
    }

    File file = SD_MMC.open(path, FILE_READ);
    if (!file) {
      if (runtime) giveSdMutex();
      continue;
    }

    if (file.size() != expectedBytes) {
      file.close();
      if (runtime) giveSdMutex();
      return VOICE_FILE_CHANGED;
    }

    uint32_t offset = 0;
    bool readFailed = false;
    bool audioLockFailed = false;
    while (offset < expectedBytes) {
      uint32_t remaining = expectedBytes - offset;
      size_t chunkBytes = (remaining < sizeof(g_voiceReadBuffer))
                            ? remaining : sizeof(g_voiceReadBuffer);
      size_t bytesRead = file.read(g_voiceReadBuffer, chunkBytes);
      if (bytesRead != chunkBytes) {
        readFailed = true;
        break;
      }

      if (runtime) {
        bool copied = false;
        for (uint8_t lockAttempt = 0; lockAttempt < VOICE_FILE_RETRIES; lockAttempt++) {
          if (xSemaphoreTake(g_clipMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            memcpy(destination + offset, g_voiceReadBuffer, chunkBytes);
            if (!giveClipMutex()) {
              audioLockFailed = true;
              readFailed = true;
              break;
            }
            copied = true;
            break;
          }
        }
        if (!copied) {
          audioLockFailed = true;
          readFailed = true;
          break;
        }
      } else {
        memcpy(destination + offset, g_voiceReadBuffer, chunkBytes);
      }
      offset += (uint32_t)chunkBytes;
    }

    file.close();
    if (runtime) giveSdMutex();
    if (audioLockFailed) return VOICE_FILE_AUDIO_LOCK_FAILED;
    if (!readFailed && offset == expectedBytes) return VOICE_FILE_OK;
  }
  return lockFailed ? VOICE_FILE_LOCK_FAILED : VOICE_FILE_SHORT_READ;
}

static const char* voiceFileError(VoiceFileResult result) {
  switch (result) {
    case VOICE_FILE_LOCK_FAILED: return "SD busy";
    case VOICE_FILE_OPEN_FAILED: return "file missing or could not be opened";
    case VOICE_FILE_BAD_SIZE: return "file size is empty, odd, or too large";
    case VOICE_FILE_CHANGED: return "file changed after validation";
    case VOICE_FILE_SHORT_READ: return "short read";
    case VOICE_FILE_AUDIO_LOCK_FAILED: return "audio buffer lock failed";
    default: return "unknown file error";
  }
}

static bool validateVoiceProfile(const char* profile, bool runtime,
                                 uint32_t clipBytes[VOICE_CLIP_COUNT],
                                 uint32_t& totalBytes) {
  totalBytes = 0;
  for (int id = 0; id < VOICE_CLIP_COUNT; id++) {
    char path[32];
    if (!makeClipPath(profile, id, path, sizeof(path))) {
      Serial.println("[Voice] Clip path exceeded its fixed buffer");
      return false;
    }

    VoiceFileResult result = inspectVoiceFile(path, runtime, clipBytes[id]);
    if (result != VOICE_FILE_OK) {
      Serial.print("[Voice] Profile ");
      Serial.print(profile[0] == '\0' ? "Default" : profile);
      Serial.print(": ");
      Serial.print(path);
      Serial.print(" -- ");
      Serial.println(voiceFileError(result));
      return false;
    }
    if (clipBytes[id] > VOICE_POOL_BYTES - totalBytes) {
      Serial.print("[Voice] Profile exceeds the ");
      Serial.print(VOICE_POOL_BYTES);
      Serial.println("-byte PSRAM pool");
      return false;
    }
    totalBytes += clipBytes[id];
  }
  return true;
}

static bool takeClipMutex() {
  for (uint8_t attempt = 0; attempt < VOICE_FILE_RETRIES; attempt++) {
    if (g_clipMutex != nullptr &&
        xSemaphoreTake(g_clipMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
      return true;
    }
  }
  Serial.println("[Voice] Audio buffer busy -- profile load skipped");
  return false;
}

static bool giveClipMutex() {
  if (xSemaphoreGive(g_clipMutex) != pdTRUE) {
    Serial.println("[Voice] Failed to release audio buffer mutex");
    return false;
  }
  return true;
}

static bool loadVoiceProfile(const char* profile, bool runtime, bool& poolTouched) {
  poolTouched = false;
  uint32_t clipBytes[VOICE_CLIP_COUNT];
  uint32_t totalBytes;
  if (!validateVoiceProfile(profile, runtime, clipBytes, totalBytes)) return false;
  (void)totalBytes;

  if (!takeClipMutex()) return false;
  g_clipsLoaded = false;
  g_playing = false;
  g_sequenceLen = 0;
  poolTouched = true;
  if (!giveClipMutex()) return false;

  uint32_t offset = 0;
  for (int id = 0; id < VOICE_CLIP_COUNT; id++) {
    char path[32];
    if (!makeClipPath(profile, id, path, sizeof(path))) {
      Serial.println("[Voice] Clip path exceeded its fixed buffer");
      return false;
    }

    VoiceFileResult result = readVoiceFile(path, runtime, clipBytes[id],
                                           g_voicePool + offset);
    if (result != VOICE_FILE_OK) {
      Serial.print("[Voice] Failed loading ");
      Serial.print(path);
      Serial.print(": ");
      Serial.println(voiceFileError(result));
      return false;
    }

    offset += clipBytes[id];
  }

  if (!takeClipMutex()) return false;
  offset = 0;
  for (int id = 0; id < VOICE_CLIP_COUNT; id++) {
    g_clips[id].samples = (int16_t*)(g_voicePool + offset);
    g_clips[id].sampleCount = clipBytes[id] / 2u;
    offset += clipBytes[id];
  }
  strncpy(g_activeProfile, profile, VOICE_PROFILE_NAME_MAX - 1);
  g_activeProfile[VOICE_PROFILE_NAME_MAX - 1] = '\0';
  if (!giveClipMutex()) return false;
  g_clipsLoaded = true;

  Serial.print("[Voice] Loaded ");
  Serial.print(VOICE_CLIP_COUNT);
  Serial.print(" voice clips into PSRAM (profile: ");
  Serial.print(profile[0] == '\0' ? "Default" : profile);
  Serial.println(")");
  return true;
}

static bool copyProfileNameFromPath(const char* path, char* name, size_t nameSize) {
  if (path == nullptr || name == nullptr || nameSize == 0) return false;
  size_t pathLength = 0;
  size_t baseOffset = 0;
  while (pathLength < 32 && path[pathLength] != '\0') {
    if (path[pathLength] == '/') baseOffset = pathLength + 1;
    pathLength++;
  }
  if (pathLength == 32) return false;
  size_t length = pathLength - baseOffset;
  if (length == 0 || length >= nameSize || length >= VOICE_PROFILE_NAME_MAX) return false;
  memcpy(name, path + baseOffset, length);
  name[length] = '\0';
  return isValidProfileName(name, false);
}

static uint8_t scanVoiceProfiles(char names[][VOICE_PROFILE_NAME_MAX],
                                 uint8_t maxNames, bool runtime) {
  if (names == nullptr || maxNames == 0) return 0;
  if (maxNames > VOICE_MAX_PROFILES) maxNames = VOICE_MAX_PROFILES;
  if (runtime && !takeSdMutex()) {
    Serial.println("[Voice] SD busy -- profile scan skipped");
    return 0;
  }

  uint8_t count = 0;
  File root = SD_MMC.open("/VOICE", FILE_READ);
  if (!root || !root.isDirectory()) {
    if (root) root.close();
    if (runtime) giveSdMutex();
    return 0;
  }

  if (count < maxNames && SD_MMC.exists("/VOICE/V000.PCM")) {
    names[count++][0] = '\0';
  }

  for (uint8_t scanned = 0;
       scanned < VOICE_PROFILE_SCAN_MAX_ENTRIES && count < maxNames;
       scanned++) {
    File entry = root.openNextFile();
    if (!entry) break;
    if (entry.isDirectory()) {
      char name[VOICE_PROFILE_NAME_MAX];
      if (copyProfileNameFromPath(entry.name(), name, sizeof(name))) {
        char firstClip[32];
        int written = snprintf(firstClip, sizeof(firstClip),
                               "/VOICE/%s/V000.PCM", name);
        if (written >= 0 && (size_t)written < sizeof(firstClip) &&
            SD_MMC.exists(firstClip)) {
          bool duplicate = false;
          for (uint8_t i = 0; i < count; i++) {
            if (strcmp(names[i], name) == 0) duplicate = true;
          }
          if (!duplicate) {
            strncpy(names[count], name, VOICE_PROFILE_NAME_MAX - 1);
            names[count][VOICE_PROFILE_NAME_MAX - 1] = '\0';
            count++;
          }
        }
      }
    }
    entry.close();
  }

  root.close();
  if (runtime) giveSdMutex();
  return count;
}

uint8_t listVoiceProfiles(char names[][VOICE_PROFILE_NAME_MAX], uint8_t maxNames) {
  if (!sdCardOK || sdMutex == nullptr) {
    Serial.println("[Voice] SD unavailable -- profile scan skipped");
    return 0;
  }
  return scanVoiceProfiles(names, maxNames, true);
}

const char* voiceActiveProfile() {
  return g_activeProfile;
}

bool voiceClipsLoaded() { return g_clipsLoaded; }
bool voiceIsPlaying() { return g_playing; }

bool requestVoiceProfile(const char* name) {
  if (!isValidProfileName(name, true)) return false;
  portENTER_CRITICAL(&g_reloadMux);
  if (g_reloadLoading) {
    portEXIT_CRITICAL(&g_reloadMux);
    return false;
  }
  strncpy(g_requestedProfile, name, VOICE_PROFILE_NAME_MAX - 1);
  g_requestedProfile[VOICE_PROFILE_NAME_MAX - 1] = '\0';
  g_reloadPending = true;
  g_reloadLoading = true;
  g_lastReloadOk = false;
  portEXIT_CRITICAL(&g_reloadMux);
  return true;
}

bool voiceProfileLoading() {
  portENTER_CRITICAL(&g_reloadMux);
  bool loading = g_reloadLoading;
  portEXIT_CRITICAL(&g_reloadMux);
  return loading;
}

bool voiceLastReloadOk() {
  portENTER_CRITICAL(&g_reloadMux);
  bool succeeded = g_lastReloadOk;
  portEXIT_CRITICAL(&g_reloadMux);
  return succeeded;
}

bool loadVoiceClips() {
  if (g_voicePool == nullptr) {
    g_voicePool = (uint8_t*)ps_malloc(VOICE_POOL_BYTES);
    if (g_voicePool == nullptr) {
      Serial.println("[Voice] PSRAM pool allocation failed -- voice alerts disabled");
      return false;
    }
  }
  if (g_clipMutex == nullptr) {
    g_clipMutex = xSemaphoreCreateMutex();
    if (g_clipMutex == nullptr) {
      Serial.println("[Voice] Audio buffer mutex creation failed -- voice alerts disabled");
      return false;
    }
  }
  if (!SD_MMC.exists("/VOICE")) {
    Serial.println("[Voice] /VOICE folder not found on SD card -- voice alerts disabled");
    return false;
  }

  char requested[VOICE_PROFILE_NAME_MAX];
  strncpy(requested, voiceProfile, VOICE_PROFILE_NAME_MAX - 1);
  requested[VOICE_PROFILE_NAME_MAX - 1] = '\0';
  if (!isValidProfileName(requested, true)) requested[0] = '\0';

  bool touched = false;
  if (loadVoiceProfile(requested, false, touched)) return true;

  if (requested[0] != '\0') {
    Serial.println("[Voice] Requested profile unavailable -- trying Default");
    if (loadVoiceProfile("", false, touched)) return true;
  }

  char profiles[VOICE_MAX_PROFILES][VOICE_PROFILE_NAME_MAX] = {};
  uint8_t profileCount = scanVoiceProfiles(profiles, VOICE_MAX_PROFILES, false);
  for (uint8_t i = 0; i < profileCount; i++) {
    if (profiles[i][0] == '\0' || strcmp(profiles[i], requested) == 0) continue;
    Serial.print("[Voice] Trying profile ");
    Serial.println(profiles[i]);
    if (loadVoiceProfile(profiles[i], false, touched)) return true;
  }

  Serial.println("[Voice] No complete voice profile found -- voice alerts disabled");
  return false;
}

void voiceStop() {
  if (g_clipMutex == nullptr ||
      xSemaphoreTake(g_clipMutex, pdMS_TO_TICKS(5)) != pdTRUE) return;
  g_playing = false;
  g_sequenceLen = 0;
  giveClipMutex();
}

void voicePlaySequence(const VoiceClipId* clips, int count, float gain) {
  if (clips == nullptr || count <= 0 || g_clipMutex == nullptr ||
      xSemaphoreTake(g_clipMutex, pdMS_TO_TICKS(5)) != pdTRUE) return;
  if (!g_clipsLoaded) {
    giveClipMutex();
    return;
  }

  if (count > VOICE_MAX_SENTENCE_CLIPS) {
    Serial.println("[Voice] Sentence truncated -- exceeds VOICE_MAX_SENTENCE_CLIPS");
    count = VOICE_MAX_SENTENCE_CLIPS;
  }
  for (int i = 0; i < count; i++) g_sequence[i] = clips[i];
  g_sequenceLen = count;
  g_clipIndex = 0;
  g_sampleOffset = 0;
  g_gapSamplesRemaining = 0;
  g_gain = (gain < 0.0f) ? 0.0f : (gain > 1.0f ? 1.0f : gain);
  g_playing = true;
  giveClipMutex();
}

void voiceServiceChunk(int16_t* chunk, int chunkLen) {
  if (chunk == nullptr || chunkLen <= 0) return;
  if (g_clipMutex == nullptr || xSemaphoreTake(g_clipMutex, 0) != pdTRUE) {
    for (int i = 0; i < chunkLen; i++) chunk[i] = 0;
    return;
  }

  if (!g_clipsLoaded || !g_playing) {
    for (int i = 0; i < chunkLen; i++) chunk[i] = 0;
    giveClipMutex();
    return;
  }

  int i = 0;
  while (i < chunkLen) {
    if (g_gapSamplesRemaining > 0) {
      chunk[i++] = 0;
      g_gapSamplesRemaining--;
      continue;
    }

    const LoadedClip& clip = g_clips[g_sequence[g_clipIndex]];
    if (g_sampleOffset >= clip.sampleCount) {
      g_clipIndex++;
      g_sampleOffset = 0;
      if (g_clipIndex >= g_sequenceLen) {
        g_playing = false;
        while (i < chunkLen) chunk[i++] = 0;
        break;
      }
      g_gapSamplesRemaining =
        (uint32_t)VOICE_CLIP_SAMPLE_RATE * VOICE_INTER_CLIP_GAP_MS / 1000u;
      continue;
    }

    float sample = (float)clip.samples[g_sampleOffset] * g_gain;
    if (sample > 32767.0f) sample = 32767.0f;
    if (sample < -32768.0f) sample = -32768.0f;
    chunk[i++] = (int16_t)sample;
    g_sampleOffset++;
  }
  giveClipMutex();
}

void voiceProfileService() {
  char requested[VOICE_PROFILE_NAME_MAX];
  portENTER_CRITICAL(&g_reloadMux);
  if (!g_reloadPending) {
    portEXIT_CRITICAL(&g_reloadMux);
    return;
  }
  strncpy(requested, g_requestedProfile, VOICE_PROFILE_NAME_MAX - 1);
  requested[VOICE_PROFILE_NAME_MAX - 1] = '\0';
  g_reloadPending = false;
  portEXIT_CRITICAL(&g_reloadMux);

  char previous[VOICE_PROFILE_NAME_MAX];
  strncpy(previous, g_activeProfile, VOICE_PROFILE_NAME_MAX - 1);
  previous[VOICE_PROFILE_NAME_MAX - 1] = '\0';
  bool hadPrevious = voiceClipsLoaded();
  bool poolTouched = false;
  bool succeeded = loadVoiceProfile(requested, true, poolTouched);

  if (!succeeded && poolTouched && hadPrevious) {
    Serial.println("[Voice] Reload failed after writing the pool -- restoring previous profile");
    bool restoreTouched = false;
    if (!loadVoiceProfile(previous, true, restoreTouched)) {
      Serial.println("[Voice] Previous profile could not be restored -- voice alerts disabled");
    }
  }

  portENTER_CRITICAL(&g_reloadMux);
  g_lastReloadOk = succeeded;
  g_reloadLoading = false;
  portEXIT_CRITICAL(&g_reloadMux);
}

// ---- Number composition ----

// Composes n (0-999) into hundred/tens/ones clips.
static int appendUnderThousand(VoiceClipId* out, int cap, int used, unsigned int n) {
  if (n >= 100) {
    unsigned int h = n / 100;  // 1-9
    if (used < cap) out[used++] = VOICE_ONES[h];
    if (used < cap) out[used++] = VOICE_NUM_HUNDRED;
    n %= 100;
  }
  if (n >= 20) {
    unsigned int t = n / 10;  // 2-9
    if (used < cap) out[used++] = VOICE_TENS[t];
    n %= 10;
    if (n > 0 && used < cap) out[used++] = VOICE_ONES[n];
  } else if (n > 0) {
    if (used < cap) out[used++] = VOICE_ONES[n];  // covers 1-19, including teens
  }
  return used;
}

int appendNumberClips(VoiceClipId* out, int outCapacity, int outUsed, unsigned int value) {
  int used = outUsed;
  if (value == 0) {
    if (used < outCapacity) out[used++] = VOICE_NUM_0;
    return used;
  }
  unsigned int thousands = value / 1000;
  unsigned int remainder = value % 1000;
  if (thousands > 0) {
    used = appendUnderThousand(out, outCapacity, used, thousands);
    if (used < outCapacity) out[used++] = VOICE_NUM_THOUSAND;
  }
  if (remainder > 0) {
    used = appendUnderThousand(out, outCapacity, used, remainder);
  }
  return used;
}

// ---- Compass ----

// Mirrors getCompassDirection()'s exact boundaries (paraglide_vario_ESP32.ino)
// so the spoken direction always matches what's shown on screen.
VoiceClipId headingToVoiceClipId(float heading) {
  if (heading < 0) heading += 360.0f;
  int idx;
  if (heading >= 337.5f || heading < 22.5f) idx = 0;   // N
  else if (heading < 67.5f) idx = 1;                    // NE
  else if (heading < 112.5f) idx = 2;                   // E
  else if (heading < 157.5f) idx = 3;                   // SE
  else if (heading < 202.5f) idx = 4;                   // S
  else if (heading < 247.5f) idx = 5;                   // SW
  else if (heading < 292.5f) idx = 6;                   // W
  else idx = 7;                                         // NW
  return VOICE_COMPASS[idx];
}

// ---- Sentence builder ----

int buildAlertSentence(VoiceClipId* outClips, int outCapacity,
                        int clockHour, bool headingKnown, float headingDeg,
                        int altitudeFt, int relativeFt, bool relativeAbove,
                        int distanceValue, bool distanceIsMeters) {
  int used = 0;

  if (clockHour < 1) clockHour = 1;
  if (clockHour > 12) clockHour = 12;

  #define APPEND1(id) do { if (used < outCapacity) outClips[used++] = (id); } while (0)

  APPEND1(VOICE_PHRASE_AIRCRAFT_ON_YOUR);
  APPEND1(VOICE_CLOCK[clockHour]);

  if (headingKnown) {
    APPEND1(VOICE_PHRASE_FLYING);
    APPEND1(headingToVoiceClipId(headingDeg));
  } else {
    APPEND1(VOICE_PHRASE_HEADING_UNKNOWN);
  }

  APPEND1(VOICE_PHRASE_AT);
  used = appendNumberClips(outClips, outCapacity, used, (unsigned int)max(altitudeFt, 0));
  APPEND1(VOICE_PHRASE_FEET);

  used = appendNumberClips(outClips, outCapacity, used, (unsigned int)max(relativeFt, 0));
  APPEND1(relativeAbove ? VOICE_PHRASE_FEET_ABOVE_YOU : VOICE_PHRASE_FEET_BELOW_YOU);

  APPEND1(VOICE_PHRASE_AND_IS);
  used = appendNumberClips(outClips, outCapacity, used, (unsigned int)max(distanceValue, 0));
  APPEND1(distanceIsMeters ? VOICE_PHRASE_METERS_AWAY : VOICE_PHRASE_KILOMETERS_AWAY);

  #undef APPEND1

  if (used >= outCapacity) {
    Serial.println("[Voice] buildAlertSentence: sentence truncated -- increase VOICE_MAX_SENTENCE_CLIPS");
  }
  return used;
}
