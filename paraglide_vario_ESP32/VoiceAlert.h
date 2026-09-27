#pragma once
// =====================================================
// VOICE ALERT ENGINE
// Plays back short pre-recorded speech clips (see VoiceClips.h) to speak
// ADS-B traffic alerts, e.g. "Aircraft on your two o'clock, flying north
// west, at two thousand four hundred feet, twelve hundred feet above you,
// and is eight hundred meters away."
//
// The clips themselves are generated offline (Piper neural TTS, see
// /voicebuild in the repo) and stored on the SD card as raw headerless
// 16kHz mono 16-bit PCM at /VOICE/V###.PCM. There is no text-to-speech or
// audio decoding on the device -- just pre-recorded words stitched
// together in the right order.
//
// ARCHITECTURE: all 60 clips (~1.25MB total) are loaded into PSRAM once
// at boot by loadVoiceClips(), called from setup() after SD_MMC.begin().
// Real-time playback then never touches the SD card -- i2sToneService()
// (paraglide_vario_ESP32.ino) copies samples directly out of the PSRAM
// buffers each audio chunk, the same real-time-safe way it already
// synthesizes tones. This matters: doing small SD reads from inside that
// timing-critical callback would risk audible stutter/blocking, since SD
// access isn't guaranteed fast or uncontended (the same card also serves
// AIRSPACE.txt, the DEM terrain lookups, and IGC logging elsewhere).
// =====================================================
#include "VoiceClips.h"

#define VOICE_MAX_SENTENCE_CLIPS 24
#define VOICE_INTER_CLIP_GAP_MS 90  // silence between spliced clips

// Call once from setup(), after SD_MMC.begin(). Loads every clip in
// VoiceClips.h from /VOICE/V###.PCM into PSRAM. If the SD card, a clip
// file, or the PSRAM allocation fails, this logs the problem to Serial
// and leaves voice alerts unavailable (voiceClipsLoaded() false) rather
// than crashing -- the rest of the device works normally either way.
bool loadVoiceClips();
bool voiceClipsLoaded();

// True while a clip sequence is still playing.
bool voiceIsPlaying();

// Stops playback immediately (e.g. a repeat alert pre-empting one still
// in progress).
void voiceStop();

// Starts playing clips[0..count-1] back to back, with a short silence
// gap between each. Pre-empts anything already playing. gain: 0.0-1.0,
// applied uniformly to every sample -- same 0-100% scale as the Alert
// Volume setting (ADSB Settings), converted the same dB-scaled way the
// existing volume settings are (see settings.h).
void voicePlaySequence(const VoiceClipId* clips, int count, float gain);

// Called every audio chunk from i2sToneService() (paraglide_vario_ESP32.ino)
// when voiceIsPlaying() is true. Fills exactly chunkLen samples from the
// current playback position (silence-padding if the sequence finishes
// partway through this chunk). Never touches the SD card or blocks --
// pure PSRAM reads, safe to call from the same timing-critical context
// the tone synthesis already runs in.
void voiceServiceChunk(int16_t* chunk, int chunkLen);

// ---- Sentence composition ----
// Appends the spoken clip sequence for value (e.g. 2400 -> "two
// thousand", "four", "hundred") into out[outUsed...], returning the new
// used count. Works for any 0-99999 value, but the alert only ever
// speaks values already rounded to a multiple of 100 (whole hundreds),
// so no tens/ones clips below 100 are ever actually needed at call time
// -- the general algorithm handles it regardless.
int appendNumberClips(VoiceClipId* out, int outCapacity, int outUsed, unsigned int value);

// Maps a 0-360 degree heading to its VoiceClipId using the exact same
// 8-way boundaries as getCompassDirection() (paraglide_vario_ESP32.ino),
// so the spoken direction always matches what's shown on screen.
VoiceClipId headingToVoiceClipId(float headingDeg);

// Builds the full alert sentence into outClips (caller-supplied buffer,
// at least VOICE_MAX_SENTENCE_CLIPS long) and returns the clip count.
//   clockHour       1-12
//   headingKnown    false -> speaks "heading unknown" instead of a
//                   compass direction; headingDeg is ignored
//   altitudeFt      intruder's absolute altitude, already rounded to the
//                   nearest 100ft by the caller
//   relativeFt      vertical separation, already rounded to the nearest
//                   100ft (always >= 0 -- see relativeAbove for sign)
//   relativeAbove   true = "feet above you", false = "feet below you"
//   distanceValue   already rounded by the caller: whole metres (nearest
//                   100) if distanceIsMeters, else whole kilometres
//   distanceIsMeters  true below 2km, false at/above 2km
int buildAlertSentence(VoiceClipId* outClips, int outCapacity,
                        int clockHour, bool headingKnown, float headingDeg,
                        int altitudeFt, int relativeFt, bool relativeAbove,
                        int distanceValue, bool distanceIsMeters);
