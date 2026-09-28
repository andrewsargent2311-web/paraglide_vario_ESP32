#pragma once
// Buzzer -- I2S + ES8311 audio: tone generator, vario/sink/intercept/mute
// audio state machine, feedback tones, volume persistence, ADS-B alert.
// The audio-service task/timer creation stays inline in setup().
#include <Arduino.h>
#include <driver/i2s.h>
#include <esp_timer.h>
#include "settings.h"
#include "CrossCoreState.h"  // NearestThreatSnapshot

// =====================================================
// VARIO TONE (onboard I2S speaker via ES8311 codec)
// =====================================================
// This board's audio is an I2S speaker driven through an ES8311 codec
// chip on the I2C bus (address 0x18), not a GPIO piezo buzzer -- tone()/
// noTone() do not apply here at all. Two separate things have to work
// for sound to come out: the I2S peripheral carries the audio *data*,
// and the ES8311 chip (controlled over I2C) must be explicitly woken
// and unmuted or it stays silent by design. See es8311Init() below.
//
// Confirm these four I2S pins against your board's actual schematic
// before flashing -- a wrong pin here fails silently, same as a wrong
// ES8311 register value would.
#define I2S_MCLK 16
#define I2S_BCLK 9
#define I2S_LRCK 45
#define I2S_DOUT 8
#define I2S_PORT I2S_NUM_0
#define AMP_ENABLE_PIN 46

#define I2S_SAMPLE_RATE 16000
#define I2S_TONE_CHUNK 64  // samples generated per i2sToneService() call

// ---------------------------------------------------------
// HOW THE TONE SAMPLES REACH THE I2S HARDWARE
//
// AUDIO_USE_DEDICATED_TASK = 1 (default): a dedicated high-priority task
// (audioServiceTask(), Core 1) generates a chunk and does a BLOCKING
// i2s_write(). Because the write blocks whenever the DMA ring is full, the
// I2S hardware's own sample clock paces the task: the ring is always kept
// topped up (~32ms of audio buffered) no matter what any other task or the
// esp_timer task is doing, and there is nothing to drift.
//
// AUDIO_USE_DEDICATED_TASK = 0: the original design -- an esp_timer
// callback every 4ms doing a zero-timeout i2s_write(). That has no way of
// knowing how full the DMA ring is, so how much audio is buffered depends
// on the (arbitrary) moment audio started and on timer jitter; when the
// cushion is small, any delay to the timer task drops or repeats 4ms
// chunks, which sounds like a scrambled/glitching tone. Kept only so you
// can switch back to compare.
// ---------------------------------------------------------
#define AUDIO_USE_DEDICATED_TASK 1

#if AUDIO_USE_DEDICATED_TASK
  // 4 x 128 samples = 512 samples = ~32ms of buffered audio (tone changes
  // are heard within ~35ms).
  #define I2S_DMA_BUF_COUNT 4
  #define I2S_DMA_BUF_LEN 128
#else
  // 8 x 256 samples = 2048 samples = ~128ms.
  #define I2S_DMA_BUF_COUNT 8
  #define I2S_DMA_BUF_LEN 256
#endif

// Smooths pitch changes while a tone is sounding. The vario rate (and so
// the sink/climb pitch) only updates every 100ms, so without this the pitch
// moves in audible little steps -- a rough, warbling texture on a sustained
// low sink tone. This is a ~30ms glide (one-pole filter); big jumps (e.g.
// the alarm's two-tone switch) and the start of every beep snap instantly.
// Set to 0 to disable.
#define TONE_PITCH_SMOOTH_MS 30.0f
// i2sToneService() now runs from its own esp_timer callback instead of
// being polled from loop(), so it can't be starved by a slow display
// redraw or (formerly) a blocking network call. Period matches exactly
// one chunk's playback time (64 samples / 16000Hz = 4ms) so the DMA
// buffer stays topped up with minimal added latency.
#define AUDIO_SERVICE_INTERVAL_US 4000
extern esp_timer_handle_t audioServiceTimer;

#define ES8311_I2C_ADDR 0x18

// CLIMB_DEADBAND_MS is defined in DrawPages.h (shared with drawParagliderPage()'s sink indicator, and with updateI2sAudioBuzzer() below).
#define SINK_ALARM_MS -0.5f  // Sink alarm set to start at -0.5m/s can alter this later to suit
#define CLIMB_TONE_MAX_MS 5.0f
//#define SINK_RELEASE_MS -5.0f  // Set to -5m/s as not uncommon to hit 4 m/s sink alarm switches off above 5 m/s to avoid distraction
//commented out max sink threshold for debugging as its causing clipping
// Sink alarm -- constant (non-pulsed) tone, pitch dropping as sink
// strengthens. Matches the BlueFly hardware settings (manual v1.8):
//   sinkFreq = sinkFreqBase - sinkFreqIncrement * |sink|
// measured from 0 m/s (NOT from the sink threshold), clamped to the
// BlueFly's 130Hz minimum. Defaults: 400Hz base, 100Hz per m/s.
// If your BlueFly's sink settings have been changed in XCSoar, change
// these to match. See the SINK ALARM OUTPUT block in updateI2sAudioBuzzer().
#define SINK_FREQ_BASE_HZ 400.0f
#define SINK_FREQ_INCREMENT_HZ 100.0f  // Hz of pitch drop per 1 m/s of sink
#define SINK_FREQ_MIN_HZ 130.0f
// Climb tone frequency range is fixed in the BLUEFLY PITCH block below
// (not user-configurable -- the old Config > Vario Freq setting was
// removed as it never actually fed into that calculation).
// Climb beep cadence follows the BlueFly curve in blueflyBeepDurationMs().
// (The old climbGapMinMs/climbGapMaxMs/climbPulseMinMs/climbPulseMaxMs
// settings are no longer used by the audio code -- Config > Vario Beep now
// sets the climb and sink volumes instead.)

// ============================================================
// CLIMB / SINK VOLUME (Config > Vario Beep)
// Independent loudness for the climb beeps and the sink tone, 0-100% in
// 10% steps. 100% = the full level the tone generator produces (the same
// level as before these settings existed); lower values are scaled in dB,
// not linearly, because loudness is perceived roughly logarithmically:
// each 1% below 100 is VARIO_VOLUME_DB_PER_PERCENT dB quieter, so with
// 0.3 each 10% menu step is -3 dB (50% = -15 dB, 10% = -27 dB). 0% is
// silent. This scales the tone generator's amplitude only -- the overall
// speaker volume (Config > Volume, codec register) still applies on top.
// Defaults are VARIO_VOLUME_DEFAULT_*_PERCENT in settings.h.
// ============================================================
#define VARIO_VOLUME_DB_PER_PERCENT 0.3f

// ---- Intercept alarm: fires once per new intruder, alternates tone ----
#define INTERCEPT_ALARM_DURATION_MS 5000UL
#define INTERCEPT_TONE_HIGH_HZ 600
#define INTERCEPT_TONE_LOW_HZ 400
#define INTERCEPT_TONE_TOGGLE_MS 400UL  // time on each tone before switching

extern bool codecOK;
extern bool es8311OK;
extern volatile float toneFrequency;
extern volatile float toneGain;
extern float tonePhase;

extern bool muteToneActive;
extern unsigned long muteToneStart;
extern bool muteToneIsMuteSequence;

extern unsigned long pageBeepUntil;
extern float pageBeepGain;
extern bool beepOn;
extern bool interceptAlarmActive;
extern unsigned long interceptAlarmStart;
extern uint8_t climbVolumePercent;
extern uint8_t sinkVolumePercent;

void setupI2sCodec();
void es8311WriteReg(uint8_t reg, uint8_t value);
void es8311Init();
void applyBuzzerVolume();
void setToneFrequency(float freq);
void i2sToneService();
void updateI2sAudioBuzzer();
void playFeedbackTone(float freq, unsigned long durationMs);
void playVolumePreviewTone(float freq, unsigned long durationMs, uint8_t volumePercent);
void loadVarioVolumes();
void saveVarioVolumes();
float varioVolumeToGain(uint8_t percent);
void triggerAdsbAlert(const NearestThreatSnapshot& threat);
#if AUDIO_USE_DEDICATED_TASK
void audioServiceTask(void* arg);
#endif
