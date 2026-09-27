#include "VoiceAlert.h"
#include <SD_MMC.h>
#include <Arduino.h>

// ---- Loaded clip storage (PSRAM) ----
struct LoadedClip {
  int16_t* samples = nullptr;
  uint32_t sampleCount = 0;
};
static LoadedClip g_clips[VOICE_CLIP_COUNT];
static bool g_clipsLoaded = false;

// ---- Playback state ----
static VoiceClipId g_sequence[VOICE_MAX_SENTENCE_CLIPS];
static int g_sequenceLen = 0;
static int g_clipIndex = 0;      // which clip in g_sequence is currently sounding
static uint32_t g_sampleOffset = 0;  // position within g_clips[g_sequence[g_clipIndex]]
static uint32_t g_gapSamplesRemaining = 0;  // silence between clips
static float g_gain = 1.0f;
static bool g_playing = false;

static void voiceClipFilename(int id, char* buf, size_t buflen) {
  snprintf(buf, buflen, "/VOICE/V%03d.PCM", id);
}

bool voiceClipsLoaded() { return g_clipsLoaded; }
bool voiceIsPlaying() { return g_playing; }

void voiceStop() {
  g_playing = false;
  g_sequenceLen = 0;
}

static void freeAllClips() {
  for (int i = 0; i < VOICE_CLIP_COUNT; i++) {
    if (g_clips[i].samples != nullptr) {
      free(g_clips[i].samples);
      g_clips[i].samples = nullptr;
      g_clips[i].sampleCount = 0;
    }
  }
}

bool loadVoiceClips() {
  g_clipsLoaded = false;

  if (!SD_MMC.exists("/VOICE")) {
    Serial.println("[Voice] /VOICE folder not found on SD card -- voice alerts disabled");
    return false;
  }

  for (int id = 0; id < VOICE_CLIP_COUNT; id++) {
    char path[32];
    voiceClipFilename(id, path, sizeof(path));

    File f = SD_MMC.open(path, FILE_READ);
    if (!f) {
      Serial.print("[Voice] Missing clip file: ");
      Serial.println(path);
      freeAllClips();
      return false;
    }

    size_t fileBytes = f.size();
    if (fileBytes == 0 || (fileBytes % 2) != 0) {
      Serial.print("[Voice] Clip file has bad size: ");
      Serial.println(path);
      f.close();
      freeAllClips();
      return false;
    }

    // PSRAM, not regular heap -- 60 clips (~1.25MB total) would eat a
    // large chunk of the ~300KB of internal SRAM left over after WiFi/
    // BLE/display buffers. ps_malloc requires PSRAM to be enabled in the
    // board's Tools menu (see the project README) -- returns nullptr if
    // it isn't, which this treats the same as a full/failed allocation.
    int16_t* buf = (int16_t*)ps_malloc(fileBytes);
    if (buf == nullptr) {
      Serial.print("[Voice] PSRAM allocation failed for: ");
      Serial.println(path);
      f.close();
      freeAllClips();
      return false;
    }

    size_t bytesRead = f.read((uint8_t*)buf, fileBytes);
    f.close();
    if (bytesRead != fileBytes) {
      Serial.print("[Voice] Short read on: ");
      Serial.println(path);
      free(buf);
      freeAllClips();
      return false;
    }

    g_clips[id].samples = buf;
    g_clips[id].sampleCount = fileBytes / 2;
  }

  g_clipsLoaded = true;
  Serial.print("[Voice] Loaded ");
  Serial.print(VOICE_CLIP_COUNT);
  Serial.println(" voice clips into PSRAM");
  return true;
}

void voicePlaySequence(const VoiceClipId* clips, int count, float gain) {
  if (!g_clipsLoaded) return;  // silently no-op -- caller doesn't need to check every time

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
  g_playing = (count > 0);
}

void voiceServiceChunk(int16_t* chunk, int chunkLen) {
  int i = 0;

  while (i < chunkLen) {
    if (!g_playing) {
      chunk[i++] = 0;
      continue;
    }

    if (g_gapSamplesRemaining > 0) {
      chunk[i++] = 0;
      g_gapSamplesRemaining--;
      continue;
    }

    const LoadedClip& clip = g_clips[g_sequence[g_clipIndex]];
    if (g_sampleOffset >= clip.sampleCount) {
      // This clip is done -- move to the next one, or end the sequence.
      g_clipIndex++;
      g_sampleOffset = 0;
      if (g_clipIndex >= g_sequenceLen) {
        g_playing = false;
        continue;  // remainder of this chunk gets filled with silence above
      }
      g_gapSamplesRemaining = (uint32_t)VOICE_CLIP_SAMPLE_RATE * VOICE_INTER_CLIP_GAP_MS / 1000;
      continue;
    }

    float s = (float)clip.samples[g_sampleOffset] * g_gain;
    if (s > 32767.0f) s = 32767.0f;
    if (s < -32768.0f) s = -32768.0f;
    chunk[i++] = (int16_t)s;
    g_sampleOffset++;
  }
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
