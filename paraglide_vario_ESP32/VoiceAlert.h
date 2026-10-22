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
// 16kHz mono 16-bit PCM at /VOICE/V###.PCM (Default) or
// /VOICE/<PROFILE>/V###.PCM. There is no text-to-speech or audio decoding
// on the device -- just pre-recorded words stitched together in order.
//
// ARCHITECTURE: all 60 clips are loaded into one fixed PSRAM pool at boot
// and profile switches reuse that pool. Playback never reads the SD card;
// the audio task uses a zero-timeout lock and emits silence while the pool
// is being replaced.
// =====================================================
#include "VoiceClips.h"
#include "settings.h"
#include <stdint.h>

#define VOICE_MAX_PROFILES 8
#define VOICE_MAX_SENTENCE_CLIPS 24
#define VOICE_INTER_CLIP_GAP_MS 90  // silence between spliced clips

// Call once from setup(), after SD_MMC.begin(). Loads the persisted voice
// profile, falling back to Default and then the first complete profile.
[[nodiscard]] bool loadVoiceClips();
bool voiceClipsLoaded();

// Returns profile names ("Default" is represented by an empty string).
// The caller supplies room for maxNames entries, each
// VOICE_PROFILE_NAME_MAX bytes. SD contention is logged and returns zero.
[[nodiscard]] uint8_t listVoiceProfiles(char names[][VOICE_PROFILE_NAME_MAX],
                                        uint8_t maxNames);
const char* voiceActiveProfile();  // valid only while voiceClipsLoaded()
[[nodiscard]] bool requestVoiceProfile(const char* name);
bool voiceProfileLoading();
bool voiceLastReloadOk();
void voiceProfileService();  // call once per backgroundTask() pass

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
