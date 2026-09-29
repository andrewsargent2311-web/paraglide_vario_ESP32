#pragma once

#include <Arduino.h>
#include <driver/i2s.h>
#include <esp_timer.h>

#include "Utils.h"      // i2cDevicePresent(), used by es8311Init()
#include "Vario.h"      // currentClimbRateMS, read by updateI2sAudioBuzzer()
#include "settings.h"   // buzzerMuted, buzzerVolumePercent, climbTone*/climbGap*/climbPulse* limits, adsbAlarmMuted
#include "DrawPages.h"  // CLIMB_DEADBAND_MS

// =====================================================
// BUZZER / AUDIO
// -----------------------------------------------------
// The most cross-cutting file in this split: it owns the I2S/ES8311
// hardware stack, the tone generator, AND the vario/sink/climb audio
// state machine -- plus several items that landed here from shared/
// ambiguous ownership calls rather than their original textual location
// in the monolithic .ino:
//
//  - The ADS-B intercept alarm's constants and state (INTERCEPT_*,
//    interceptAlarmActive/Start) were declared under the original
//    file's "ADSB GPS linking" banner, but the only thing that ever
//    reads them is updateI2sAudioBuzzer() below -- AdsbClient never
//    touches them; loop()'s inline threat-check block sets
//    interceptAlarmActive = true directly.
//  - SINK_ALARM_MS / SINK_TONE_MAX_HZ / SINK_TONE_MIN_HZ /
//    SINK_TONE_MAX_MS / CLIMB_TONE_MAX_MS were declared under the
//    "Variov- Variables" banner, but only ever read here too.
//  - pageBeepUntil was declared under the original "Gestures" (page
//    button) banner, but updateI2sAudioBuzzer() is the only place that
//    reads it -- PageButton (not yet extracted) writes it via
//    playFeedbackTone() below, and also clears it directly during the
//    mute-toggle long-press. It lives here so PageButton can reach in
//    via extern, same pattern as muteToneActive/AMP_ENABLE_PIN below.
//  - beepOn started out declared in Vario.h, but tracing its uses found
//    it's also written directly from PageButton's mute-toggle handler,
//    not just from Vario code -- moved here instead. It has no reads
//    anywhere in either location; see the note by its declaration.
//
// setupI2sCodec() and es8311Init() are both called once from setup();
// that call sequence, and the esp_timer_create()/esp_timer_start_
// periodic() call that drives i2sToneService() on its own cadence, all
// stay inline in setup(), unchanged -- including the
// digitalWrite(AMP_ENABLE_PIN, HIGH) that runs after both init calls
// succeed.
// =====================================================

// =====================================================
// I2S / ES8311 HARDWARE
// =====================================================
#define I2S_MCLK 16
#define I2S_BCLK 9
#define I2S_LRCK 45
#define I2S_DOUT 8
#define I2S_PORT I2S_NUM_0
#define AMP_ENABLE_PIN 46

#define I2S_SAMPLE_RATE 16000
// Deeper than a minimal setup: buffers this size (2048 samples total =
// ~128ms at 16kHz) give i2sToneService() room to tolerate loop() jitter
// (e.g. a slow SPI display redraw) without the tone audibly glitching.
// Smaller buffers would need loop() called more often than it safely can.
#define I2S_DMA_BUF_COUNT 8
#define I2S_DMA_BUF_LEN 256
#define I2S_TONE_CHUNK 64  // samples generated per i2sToneService() call
// i2sToneService() now runs from its own esp_timer callback instead of
// being polled from loop(), so it can't be starved by a slow display
// redraw or (formerly) a blocking network call. Period matches exactly
// one chunk's playback time (64 samples / 16000Hz = 4ms) so the DMA
// buffer stays topped up with minimal added latency.
#define AUDIO_SERVICE_INTERVAL_US 4000
extern esp_timer_handle_t audioServiceTimer;

#define ES8311_I2C_ADDR 0x18

extern bool codecOK;   // I2S peripheral configured
extern bool es8311OK;  // ES8311 chip found and initialized over I2C

extern volatile float toneFrequency;  // 0 = silent
extern float tonePhase;

void setupI2sCodec();
void es8311WriteReg(uint8_t reg, uint8_t value);
void es8311Init();
// Applies buzzerVolumePercent (settings.h) to the ES8311's DAC digital
// volume register: 0x00 = mute, 0xFF = 0dB (loudest). Called once at
// boot (inside es8311Init()) and again immediately whenever the pilot
// changes the Config > Volume menu setting.
void applyBuzzerVolume();
void setToneFrequency(float freq);
void i2sToneService();

// =====================================================
// MUTE / UNMUTE CONFIRMATION TONE
// A short two-tone jingle played once whenever buzzerMuted is toggled by
// the long-press gesture (see PageButton's updatePageButton()), so the
// pilot gets audible confirmation of which state they just landed in.
// Sequenced non-blockingly inside updateI2sAudioBuzzer(), same as the
// rest of the buzzer state machine. Muting plays 650Hz(1s) -> 10ms gap
// -> 500Hz(0.5s); unmuting plays the same three segments in reverse.
// =====================================================
#define MUTE_TONE_FREQ_HIGH_HZ 650.0f
#define MUTE_TONE_FREQ_LOW_HZ 500.0f
#define MUTE_TONE_HIGH_MS 500UL
#define MUTE_TONE_LOW_MS 200UL
#define MUTE_TONE_GAP_MS 10UL

extern bool muteToneActive;
extern unsigned long muteToneStart;
extern bool muteToneIsMuteSequence;  // true = muting order (650->gap->500); false = unmuting order (500->gap->650)

// =====================================================
// ADS-B INTERCEPT ALARM
// -----------------------------------------------------
// Moved here from the original "ADSB GPS linking" banner -- see the
// file-level note above.
// ---- Intercept alarm: fires once per new intruder, alternates tone ----
// =====================================================
#define INTERCEPT_ALARM_DURATION_MS 5000UL
#define INTERCEPT_TONE_HIGH_HZ 600
#define INTERCEPT_TONE_LOW_HZ 400
#define INTERCEPT_TONE_TOGGLE_MS 400UL  // time on each tone before switching
extern bool interceptAlarmActive;
extern unsigned long interceptAlarmStart;

// =====================================================
// VARIO SINK/CLIMB TONE STATE
// -----------------------------------------------------
// SINK_ALARM_MS and the four constants below moved here from the
// original "Variov- Variables" banner -- see the file-level note above.
// climbToneMinHz/climbToneMaxHz/climbGapMinMs/climbGapMaxMs/
// climbPulseMinMs/climbPulseMaxMs remain in settings.h, unchanged.
// =====================================================
#define SINK_ALARM_MS -0.5f  // Sink alarm set to start at -0.5m/s can alter this later to suit
#define CLIMB_TONE_MAX_MS 5.0f
#define SINK_TONE_MAX_HZ 350
#define SINK_TONE_MIN_HZ 150
#define SINK_TONE_MAX_MS -5.0f

extern bool sinkAlarmActive;
extern bool climbAudioActive;
extern unsigned long sinkAlarmStart;
extern unsigned long climbPulseStart;
extern bool climbToneOn;

// See the file-level note on why this moved out of Vario.h. Declared
// here, unused, exactly as in the original file -- nothing anywhere
// reads it; it's only ever written to false (at init, and again by
// PageButton's mute-toggle handler).
extern bool beepOn;

// =====================================================
// PAGE-CHANGE BEEP PRIORITY WINDOW
// -----------------------------------------------------
// Moved here from the original "Gestures" (page button) banner -- see
// the file-level note above. PAGE_BEEP_FREQ/PAGE_BEEP_MS stay with
// PageButton, since they're just the specific tone/duration PageButton
// asks this file to play via playFeedbackTone().
// =====================================================
extern unsigned long pageBeepUntil;  // while set, updateI2sAudioBuzzer() yields the pin to the page-change beep

void updateI2sAudioBuzzer();

// Sets up a short non-blocking tone burst for UI feedback (page change,
// menu open/navigate/select). Actual sample generation happens on the
// independent audio-servicing timer (see setup()), which starts within
// AUDIO_SERVICE_INTERVAL_US regardless -- no need to pump it manually
// here, and doing so would race with the timer callback over tonePhase.
// Respects the mute setting.
void playFeedbackTone(float freq, unsigned long durationMs);
