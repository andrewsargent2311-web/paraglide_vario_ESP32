#include "Buzzer.h"
#include "Vario.h"
#include "Utils.h"
#include "VoiceAlert.h"
#include "DrawPages.h"  // CLIMB_DEADBAND_MS
#include <Wire.h>
#include <math.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <Preferences.h>

esp_timer_handle_t audioServiceTimer = nullptr;

bool codecOK = false;   // I2S peripheral configured
bool es8311OK = false;  // ES8311 chip found and initialized over I2C

volatile float toneFrequency = 0.0f;  // 0 = silent
// Loudness multiplier applied on top of the normal tone amplitude (1.0 =
// normal), read asynchronously by the audio side. Whoever starts a tone sets
// this to that tone's gain immediately BEFORE calling setToneFrequency();
// it is never written to a temporary value (see updateI2sAudioBuzzer()).
volatile float toneGain = 1.0f;
float tonePhase = 0.0f;

// =====================================================
// MUTE / UNMUTE CONFIRMATION TONE
// Played once whenever buzzerMuted is toggled by the long-press gesture
// (see updatePageButton()), so the pilot gets audible confirmation of
// which state they just landed in. Sequenced non-blockingly inside
// updateI2sAudioBuzzer().
//
// These reproduce the BlueFly's own mute/unmute sounds, measured from a
// recording of the real device:
//   MUTE   : 200ms @ ~3320Hz, then three 100ms beeps @ ~3420Hz with
//            growing gaps (100 / 200 / 300 ms).  Total 1.1s.
//   UNMUTE : 1.0s @ ~4020Hz, 100ms gap, 100ms @ ~4020Hz.  Total 1.2s.
//            On the real BlueFly the first ~0.42s of the long beep is
//            ~14dB louder than the rest, so it is split into two steps.
//
// Each step is {frequency Hz (0 = silence), duration ms, gain}. Gain is a
// loudness multiplier: 1.0 = the normal beep level, 2.6 = ~+8dB, 0.55 =
// ~-5dB. Edit the tables to retune -- nothing else needs to change.
// =====================================================
struct ToneStep {
  float freq;
  uint16_t ms;
  float gain;
};

static const ToneStep MUTE_SEQUENCE[] = {
  { 3320.0f, 200, 1.0f },
  {    0.0f, 100, 1.0f },
  { 3420.0f, 100, 1.0f },
  {    0.0f, 200, 1.0f },
  { 3420.0f, 100, 1.0f },
  {    0.0f, 300, 1.0f },
  { 3420.0f, 100, 1.0f },
};

static const ToneStep UNMUTE_SEQUENCE[] = {
  { 4020.0f, 420, 2.6f  },  // loud first part of the long beep
  { 4020.0f, 580, 0.55f },  // quieter remainder (1.0s total)
  {    0.0f, 100, 1.0f  },
  { 4020.0f, 100, 0.55f },
};

#define TONE_SEQ_LEN(arr) (sizeof(arr) / sizeof((arr)[0]))

bool muteToneActive = false;
unsigned long muteToneStart = 0;
bool muteToneIsMuteSequence = false;  // true = play MUTE_SEQUENCE; false = play UNMUTE_SEQUENCE

bool interceptAlarmActive = false;
unsigned long interceptAlarmStart = 0;

unsigned long pageBeepUntil = 0;  // while set, updateBuzzer() yields the pin to the page-change beep

uint8_t climbVolumePercent = VARIO_VOLUME_DEFAULT_CLIMB_PERCENT;
uint8_t sinkVolumePercent = VARIO_VOLUME_DEFAULT_SINK_PERCENT;

// Gain used while a page/feedback beep (or a volume-preview tone) is
// sounding -- 1.0 for normal UI beeps.
float pageBeepGain = 1.0f;

// Converts a 0-100 volume percentage to a linear amplitude multiplier.
// Declared in settings.h so other .cpp files (VoiceAlert.cpp) can reuse
// this exact formula for the ADS-B Alert Volume setting, rather than
// duplicating it.
float varioVolumeToGain(uint8_t percent) {
  if (percent == 0) return 0.0f;
  if (percent >= 100) return 1.0f;
  return powf(10.0f, -((float)(100 - percent) * VARIO_VOLUME_DB_PER_PERCENT) / 20.0f);
}

// ============================================================
// VARIO AUDIO STATE
// ============================================================
uint32_t climbToneStartMs = 0;
uint32_t climbToneDurationMs = 0;
uint32_t nextClimbBeepMs = 0;
bool sinkAlarmActive = false;
bool climbAudioActive = false;
unsigned long sinkAlarmStart = 0;
unsigned long climbPulseStart = 0;
bool climbToneOn = false;
unsigned long lastBeepToggle = 0;
bool beepOn = false;
unsigned long lastSinkBeep = 0;

// Sets up a short non-blocking tone burst for UI feedback (page change,
// menu open/navigate/select). Actual sample generation happens on the
// independent audio-servicing timer (see setup()), which starts within
// AUDIO_SERVICE_INTERVAL_US regardless -- no need to pump it manually here,
// and doing so would race with the timer callback over tonePhase. Respects
// the mute setting.
void playFeedbackTone(float freq, unsigned long durationMs) {
  if (buzzerMuted) return;
  pageBeepGain = 1.0f;
  toneGain = pageBeepGain;  // gain first, then frequency (see updateI2sAudioBuzzer())
  setToneFrequency(freq);
  pageBeepUntil = millis() + durationMs;
}

// Same as playFeedbackTone(), but at the loudness a given climb/sink volume
// percentage (0-100) would produce -- used by Config > Vario Beep so the
// pilot can audition a volume level as they pick it.
void playVolumePreviewTone(float freq, unsigned long durationMs, uint8_t volumePercent) {
  if (buzzerMuted) return;
  pageBeepGain = varioVolumeToGain(volumePercent);
  toneGain = pageBeepGain;  // gain first, then frequency (see updateI2sAudioBuzzer())
  setToneFrequency(freq);
  pageBeepUntil = millis() + durationMs;
}

// Climb/sink volume persistence. Stored in the same NVS namespace
// ("vario") as the rest of the settings, under their own keys, so this is
// independent of loadSettings()/saveSettings(). Call loadVarioVolumes()
// once at boot, after loadSettings(); call saveVarioVolumes() after either
// value changes.
//
// This also does the one-time conversion of the master volume
// (buzzerVolumePercent, normally loaded by loadSettings()) from the old
// linear-register scale to the current dB scale -- see the BUZZER VOLUME
// comment in settings.h. A "volScheme" key marks it as done.
static uint8_t migrateOldBuzzerVolumePercent(uint8_t oldPercent) {
  if (oldPercent > 100) oldPercent = 100;
  // What register value (and so what dB) the old scheme produced.
  const int oldReg = (int)((oldPercent / 100.0f) * 255.0f + 0.5f);
  const float oldDb = (oldReg - 0xBF) * 0.5f;
  // Nearest 10% step on the new scale.
  const float stepsBelowMax = (BUZZER_VOLUME_MAX_DB - oldDb) / BUZZER_VOLUME_DB_PER_STEP;
  int pct = 100 - (int)lroundf(stepsBelowMax) * 10;
  if (pct < 10) pct = 10;
  if (pct > 100) pct = 100;
  return (uint8_t)pct;
}

void loadVarioVolumes() {
  uint8_t volScheme = 0;
  Preferences prefs;
  if (prefs.begin("vario", false)) {
    climbVolumePercent = prefs.getUChar("climbVol", VARIO_VOLUME_DEFAULT_CLIMB_PERCENT);
    sinkVolumePercent = prefs.getUChar("sinkVol", VARIO_VOLUME_DEFAULT_SINK_PERCENT);
    volScheme = prefs.getUChar("volScheme", 0);
    prefs.end();
  }
  if (climbVolumePercent > 100) climbVolumePercent = 100;
  if (sinkVolumePercent > 100) sinkVolumePercent = 100;

  if (volScheme < 1) {
    // First boot on the dB-scaled master volume: convert the old value.
    buzzerVolumePercent = migrateOldBuzzerVolumePercent(buzzerVolumePercent);
    saveSettings();
    if (prefs.begin("vario", false)) {
      prefs.putUChar("volScheme", 1);
      prefs.end();
    }
  } else {
    // Keep it on a valid menu step (10-100%, multiples of 10).
    int pct = ((int)buzzerVolumePercent + 5) / 10 * 10;
    if (pct < 10) pct = 10;
    if (pct > 100) pct = 100;
    buzzerVolumePercent = (uint8_t)pct;
  }
}

void saveVarioVolumes() {
  Preferences prefs;
  if (prefs.begin("vario", false)) {
    prefs.putUChar("climbVol", climbVolumePercent);
    prefs.putUChar("sinkVol", sinkVolumePercent);
    prefs.end();
  }
}

// ============================================================================
// BlueFly-style vario beep duration
// ============================================================================
//
// Approximates the cadence curve of the Kobo BlueFly sample/defaults.
// As lift increases, the beep becomes shorter and the cadence increases.
//
// Input:  current filtered climb rate in m/s
// Output: beep duration in milliseconds
//
static unsigned long blueflyBeepDurationMs(float climbMs){
  climbMs = max(0.0f, climbMs);

  if (climbMs <= 0.20f) {
    return 380UL;
  }

  if (climbMs <= 0.50f) {
    return (unsigned long)(
      380.0f +
      (climbMs - 0.20f) *
      (300.0f - 380.0f) / 0.30f
    );
  }

  if (climbMs <= 1.00f) {
    return (unsigned long)(
      300.0f +
      (climbMs - 0.50f) *
      (200.0f - 300.0f) / 0.50f
    );
  }

  if (climbMs <= 1.50f) {
    return (unsigned long)(
      200.0f +
      (climbMs - 1.00f) *
      (140.0f - 200.0f) / 0.50f
    );
  }

  if (climbMs <= 2.00f) {
    return (unsigned long)(
      140.0f +
      (climbMs - 1.50f) *
      (100.0f - 140.0f) / 0.50f
    );
  }

  if (climbMs <= 3.00f) {
    return (unsigned long)(
      100.0f +
      (climbMs - 2.00f) *
      (65.0f - 100.0f) / 1.00f
    );
  }

  return 55UL;
}
// =====================================================
// BUZZER: non-blocking climb/sink tone over the I2S speaker.
// Yields to the page-change beep for its short duration rather than
// talking over it -- both share the same physical speaker.
// =====================================================
//===================================================================
// =====================================================
// BLUEFLY-STYLE VARIO AUDIO
// =====================================================
//
// The actual waveform is still generated by i2sToneService().
// This function only controls:
//
//   - when the vario beeps
//   - when it is silent
//   - the pitch of the beep
//
// Existing page beep, ADS-B alarm, mute confirmation and sink alarm
// behaviour are preserved.
// =====================================================

void updateI2sAudioBuzzer(){
  const unsigned long now = millis();

  // NOTE ON toneGain: this function runs in loop() while the audio side
  // reads toneGain/toneFrequency asynchronously from another task. So it
  // must NEVER be written to a temporary value (e.g. "reset to 1.0 at the
  // top, then set the real value later") -- the audio side could catch the
  // temporary value for a chunk and play a burst at the wrong loudness.
  // Instead, every branch below that sounds a tone sets its own final gain
  // immediately BEFORE setting the frequency, and branches that are silent
  // leave it alone.


  // ============================================================
  // PAGE-CHANGE BEEP HAS PRIORITY
  // ============================================================

  if ((int32_t)(pageBeepUntil - now) > 0) {
    toneGain = pageBeepGain;
    return;
  }

  // Page beep has just finished
  if (pageBeepUntil != 0) {

    pageBeepUntil = 0;

    setToneFrequency(0);

    sinkAlarmActive = false;
    climbAudioActive = false;
    climbToneOn = false;
  }


  // ============================================================
  // ADS-B INTERCEPT ALARM
  // ============================================================

  if (interceptAlarmActive) {

    unsigned long elapsed =
      now - interceptAlarmStart;

    if (elapsed >= INTERCEPT_ALARM_DURATION_MS) {

      interceptAlarmActive = false;

      setToneFrequency(0);

      sinkAlarmActive = false;
      climbAudioActive = false;
      climbToneOn = false;

    }
    else if (adsbAlarmMode == ADSB_ALARM_TONE) {

      unsigned long phase =
        elapsed % (INTERCEPT_TONE_TOGGLE_MS * 2);

      float freq =
        (phase < INTERCEPT_TONE_TOGGLE_MS)
        ? INTERCEPT_TONE_HIGH_HZ
        : INTERCEPT_TONE_LOW_HZ;

      toneGain = varioVolumeToGain(adsbAlertVolumePercent);
      setToneFrequency(freq);

      return;
    }
  }


  // ============================================================
  // MUTE / UNMUTE CONFIRMATION TONE
  // ============================================================

  if (muteToneActive) {

    const unsigned long elapsed = now - muteToneStart;

    const ToneStep* seq =
      muteToneIsMuteSequence ? MUTE_SEQUENCE : UNMUTE_SEQUENCE;

    const size_t seqLen =
      muteToneIsMuteSequence
      ? TONE_SEQ_LEN(MUTE_SEQUENCE)
      : TONE_SEQ_LEN(UNMUTE_SEQUENCE);

    // Walk the table, accumulating durations, until we find the step
    // that covers the current elapsed time.
    unsigned long stepEnd = 0;
    bool stepFound = false;

    for (size_t i = 0; i < seqLen; i++) {

      stepEnd += seq[i].ms;

      if (elapsed < stepEnd) {

        toneGain = seq[i].gain;
        setToneFrequency(seq[i].freq);
        stepFound = true;
        break;
      }
    }

    if (stepFound) {
      return;
    }

    // Sequence finished.
    muteToneActive = false;

    setToneFrequency(0);

    if (buzzerMuted) {
      digitalWrite(AMP_ENABLE_PIN, LOW);
    }

    // Fall through to normal vario logic.
  }


  // ============================================================
  // MUTED
  // ============================================================

  if (buzzerMuted) {

    setToneFrequency(0);

    sinkAlarmActive = false;
    climbAudioActive = false;
    climbToneOn = false;

    return;
  }


  // ============================================================
  // SINK ALARM WITH HYSTERESIS
  // ============================================================

  const float SINK_ALARM_RELEASE_MS = -0.2f;

  if (!sinkAlarmActive) {

    if (currentClimbRateMS <= SINK_ALARM_MS) {

      sinkAlarmActive = true;
      sinkAlarmStart = now;

      climbAudioActive = false;
      climbToneOn = false;
    }

  }
  else {

    if (currentClimbRateMS > SINK_ALARM_RELEASE_MS) {

      sinkAlarmActive = false;

      setToneFrequency(0);
    }
  }


  // ============================================================
  // SINK ALARM OUTPUT
  // ============================================================

  if (sinkAlarmActive) {

    // BlueFly sink pitch: base minus increment per m/s of sink.
    // currentClimbRateMS is negative here, so this drops as sink grows.
    float sinkToneFreq =
      SINK_FREQ_BASE_HZ +
      (SINK_FREQ_INCREMENT_HZ * currentClimbRateMS);

    sinkToneFreq = max(SINK_FREQ_MIN_HZ, sinkToneFreq);

    toneGain = varioVolumeToGain(sinkVolumePercent);
    setToneFrequency(sinkToneFreq);

    return;
  }


  // ============================================================
  // CLIMB DEAD BAND
  // ============================================================

  if (currentClimbRateMS <= CLIMB_DEADBAND_MS) {

    climbAudioActive = false;
    climbToneOn = false;

    setToneFrequency(0);

    return;
  }


  // ============================================================
  // ENTERING CLIMB AUDIO
  // ============================================================

  if (!climbAudioActive) {

    climbAudioActive = true;
    climbToneOn = false;

    // Start the first beep immediately.
    climbPulseStart = now;

    setToneFrequency(0);
  }


  // ============================================================
  // BLUEFLY PITCH
  // ============================================================
  //
  // Approximately:
  //
  //   0.2 m/s = 1020 Hz
  //   0.5 m/s = 1050 Hz
  //   1.0 m/s = 1100 Hz
  //   2.0 m/s = 1200 Hz
  //   3.0 m/s = 1300 Hz
  //   4.0 m/s = 1400 Hz
  //   5.0 m/s = 1500 Hz
  //
  // The ES8311/speaker produces the harmonics that give the audible
  // BlueFly character.
  // ============================================================

  float blueflyFreq =
    1000.0f +
    (currentClimbRateMS * 100.0f);

  blueflyFreq =
    constrain(
      blueflyFreq,
      1000.0f,
      1800.0f
    );

  // Climb volume (Config > Vario Beep) -- applies to every beep below.
  toneGain = varioVolumeToGain(climbVolumePercent);


  // ============================================================
  // CURRENT BEEP DURATION
  // ============================================================

  unsigned long beepMs =
    blueflyBeepDurationMs(currentClimbRateMS);


  // ============================================================
  // BEEP CURRENTLY PLAYING
  // ============================================================

  if (climbToneOn) {

    // ----------------------------------------------------------
    // IMPORTANT:
    //
    // Update pitch continuously while the beep is playing.
    // ----------------------------------------------------------

    setToneFrequency(blueflyFreq);


    // ----------------------------------------------------------
    // Has this beep finished?
    // ----------------------------------------------------------

    if ((unsigned long)(now - climbPulseStart) >= beepMs) {

      climbToneOn = false;

      setToneFrequency(0);

      // --------------------------------------------------------
      // BlueFly-style silence period.
      //
      // Using the same basic duration as the current beep gives
      // the characteristic approximately 50/50 cadence in the
      // moderate-lift region of the sample.
      // --------------------------------------------------------

      climbPulseStart = now;
    }

    return;
  }


  // ============================================================
  // CURRENTLY SILENT
  // ============================================================

  // Use the same BlueFly duration for the silence.
  unsigned long silenceMs =
    blueflyBeepDurationMs(currentClimbRateMS);


  if ((unsigned long)(now - climbPulseStart) >= silenceMs) {

    climbToneOn = true;

    climbPulseStart = now;

    setToneFrequency(blueflyFreq);
  }
}

// =====================================================
// ADS-B ALERT TRIGGER
// Called from loop() (the "ADS-B new intruder" hook) whenever an alert
// is due -- either a brand new threat just appeared, or the 60s repeat
// timer elapsed while a threat is still in range. Branches on
// adsbAlarmMode to decide HOW to alert (nothing / siren / voice); the
// WHEN is entirely the caller's job, not this function's.
// =====================================================
void triggerAdsbAlert(const NearestThreatSnapshot& threat) {
  if (adsbAlarmMode == ADSB_ALARM_OFF || !threat.valid) return;

  if (adsbAlarmMode == ADSB_ALARM_TONE) {
    interceptAlarmActive = true;
    interceptAlarmStart = millis();
    return;
  }

  // ADSB_ALARM_VOICE
  if (!voiceClipsLoaded()) return;  // SD card / PSRAM load failed at boot -- nothing to play

  // Clock position needs the pilot's own heading. With no valid course
  // (stationary, no GPS fix) there's no sensible "o'clock" to give, so
  // skip the alert entirely rather than guess -- agreed behaviour.
  if (!gps.course.isValid()) return;

  float relativeBearing = threat.bearingFromMeDeg - (float)gps.course.deg();
  while (relativeBearing < 0.0f) relativeBearing += 360.0f;
  while (relativeBearing >= 360.0f) relativeBearing -= 360.0f;
  int clockHour = (int)roundf(relativeBearing / 30.0f);
  if (clockHour <= 0) clockHour = 12;
  if (clockHour > 12) clockHour = 12;  // defensive only -- shouldn't occur

  int altitudeFt = (int)(roundf(threat.altitudeFt / 100.0f) * 100.0f);
  int relativeFt = (int)(roundf(threat.verticalDeltaFt / 100.0f) * 100.0f);

  // Below 2km (checked on the un-rounded distance): whole metres, nearest
  // 100. At/above 2km: whole kilometres. A raw distance just under 2km
  // that rounds up to exactly 2000m is still spoken as "2000 meters" --
  // which side of 2km it's spoken on is decided before rounding.
  int distanceValue;
  bool distanceIsMeters;
  if (threat.distanceKm < 2.0f) {
    distanceValue = (int)(roundf(threat.distanceKm * 1000.0f / 100.0f) * 100.0f);
    distanceIsMeters = true;
  } else {
    distanceValue = (int)roundf(threat.distanceKm);
    distanceIsMeters = false;
  }

  VoiceClipId sentence[VOICE_MAX_SENTENCE_CLIPS];
  int n = buildAlertSentence(sentence, VOICE_MAX_SENTENCE_CLIPS,
                              clockHour, threat.headingKnown, threat.headingDeg,
                              altitudeFt, relativeFt, threat.aircraftAbove,
                              distanceValue, distanceIsMeters);

  voicePlaySequence(sentence, n, varioVolumeToGain(adsbAlertVolumePercent));
}

// =====================================================
// I2S CODEC SETUP: configures the ESP32-S3's I2S peripheral using the
// plain Arduino driver/i2s.h API
// =====================================================
void setupI2sCodec() {
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate = I2S_SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = I2S_DMA_BUF_COUNT,
    .dma_buf_len = I2S_DMA_BUF_LEN,
    .use_apll = false,
    .tx_desc_auto_clear = true  // auto-fills silence on underrun instead of repeating stale samples
  };

  i2s_pin_config_t pin_config = {
    .mck_io_num = I2S_MCLK,  // required on this board -- not BCLK-derived
    .bck_io_num = I2S_BCLK,
    .ws_io_num = I2S_LRCK,
    .data_out_num = I2S_DOUT,
    .data_in_num = I2S_PIN_NO_CHANGE
  };

  esp_err_t err = i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
  if (err != ESP_OK) {
    Serial.print("I2S DRIVER INSTALL FAILED, err=");
    Serial.println(err);
    codecOK = false;
    return;
  }
  i2s_set_pin(I2S_PORT, &pin_config);
  i2s_set_clk(I2S_PORT, I2S_SAMPLE_RATE, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_MONO);
  codecOK = true;
  Serial.println("I2S PERIPHERAL INITIALIZED");
}

// =====================================================
// ES8311 CODEC CONTROL (I2C): wakes and unmutes the codec chip so the
// I2S data stream above actually reaches the speaker.
// =====================================================
// Counts I2C writes to the codec that still failed after retries (see below).
static uint16_t es8311WriteFailures = 0;

// Writes one codec register, retrying if the I2C transfer isn't ACKed.
// Espressif's own ES8311 driver notes that the first I2C write to this chip
// occasionally fails, and the original version of this function ignored the
// result -- a dropped write meant a register silently kept a stale/default
// value for the whole session. Returns true if the write was ACKed.
static bool es8311WriteRegChecked(uint8_t reg, uint8_t value) {
  for (uint8_t attempt = 0; attempt < 3; attempt++) {
    Wire.beginTransmission(ES8311_I2C_ADDR);
    Wire.write(reg);
    Wire.write(value);
    if (Wire.endTransmission() == 0) return true;
    delay(2);
  }
  es8311WriteFailures++;
  return false;
}

void es8311WriteReg(uint8_t reg, uint8_t value) {
  es8311WriteRegChecked(reg, value);
}

// Returns the register's value, or -1 if the read failed.
static int es8311ReadReg(uint8_t reg) {
  Wire.beginTransmission(ES8311_I2C_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return -1;
  if (Wire.requestFrom((uint8_t)ES8311_I2C_ADDR, (uint8_t)1) != 1) return -1;
  return Wire.read();
}

// One full pass of the codec register setup. Split out from es8311Init() so
// the whole sequence can be repeated if the read-back check fails.
static void es8311ConfigureRegisters() {
  // Full reset first, as Espressif's/ESPHome's drivers do. The codec is NOT
  // reset when only the ESP32 restarts (it keeps its previous register
  // contents until its own supply is fully removed), so without this a
  // restart configured the codec on top of whatever state the last run left
  // behind -- matching "scrambled until switched off for a while".
  es8311WriteReg(0x44, 0x08);  // "I2C noise immunity" -- Espressif writes this twice because
  es8311WriteReg(0x44, 0x08);  // the first write to the chip occasionally fails
  es8311WriteReg(0x00, 0x1F);  // reset all digital blocks
  delay(20);
  es8311WriteReg(0x00, 0x00);  // release reset

  es8311WriteReg(0x01, 0x30);  // clock manager: power up analog, select clock source
  es8311WriteReg(0x02, 0x00);  // clock manager: clock divider defaults
  es8311WriteReg(0x03, 0x10);  // clock manager: ADC clock divider (unused, DAC-only)
  es8311WriteReg(0x16, 0x24);  // clock manager: DAC clock divider
  es8311WriteReg(0x04, 0x10);  // clock manager: DAC oversampling ratio
  es8311WriteReg(0x05, 0x00);  // clock manager: ADC oversampling ratio (unused)
  es8311WriteReg(0x0B, 0x00);  // system: power management
  es8311WriteReg(0x0C, 0x00);  // system: power management
  es8311WriteReg(0x10, 0x03);  // system: bias/power
  es8311WriteReg(0x11, 0x7F);  // system: bias/power
  es8311WriteReg(0x00, 0x80);  // reset: release reset, normal operation
  es8311WriteReg(0x0D, 0x01);  // system: power up analog
  es8311WriteReg(0x01, 0x3F);  // clock manager: enable all internal clocks
  es8311WriteReg(0x14, 0x1A);  // system: mic/line-in bias (unused, DAC-only)
  es8311WriteReg(0x12, 0x00);  // system: power management
  es8311WriteReg(0x13, 0x10);  // system: power management
  es8311WriteReg(0x0E, 0x02);  // system: power management
  es8311WriteReg(0x0F, 0x44);  // system: power management
  es8311WriteReg(0x15, 0x00);  // ADC: not used in DAC-only mode
  es8311WriteReg(0x37, 0x08);  // ADC: not used in DAC-only mode
  es8311WriteReg(0x09, 0x00);  // SDP: I2S format, 16-bit
  es8311WriteReg(0x18, 0x00);  // DAC: volume-related default

  // Clock dividers for 16kHz sample rate at a 256x (4.096MHz) MCLK ratio,
  // taken directly from Espressif's reference ES8311 driver's coefficient
  // table. These were previously missing/wrong -- 0x08 was mislabeled as
  // a GPIO register in an earlier version of this code; it's actually the
  // LRCK divider's low byte, and 0x06/0x07 weren't being written at all,
  // left at power-on-reset defaults that didn't match this sample rate.
  es8311WriteReg(0x06, 0x03);  // clock manager: BCLK divider
  es8311WriteReg(0x07, 0x00);  // clock manager: LRCK divider (high byte)
  es8311WriteReg(0x08, 0xFF);  // clock manager: LRCK divider (low byte)
}

// Reads back the registers that decide whether the codec is running on the
// right clocks/format. Returns true if they all hold what was written.
static bool es8311VerifyRegisters() {
  static const struct { uint8_t reg; uint8_t expected; } checks[] = {
    { 0x01, 0x3F },  // all internal clocks enabled
    { 0x06, 0x03 },  // BCLK divider
    { 0x07, 0x00 },  // LRCK divider (high)
    { 0x08, 0xFF },  // LRCK divider (low)
    { 0x09, 0x00 },  // serial port: I2S, 16-bit
  };
  bool allGood = true;
  for (const auto& c : checks) {
    const int v = es8311ReadReg(c.reg);
    if (v != c.expected) {
      Serial.printf("[ES8311] register 0x%02X reads 0x%02X, expected 0x%02X\n",
                    c.reg, (v < 0) ? 0xFF : v, c.expected);
      allGood = false;
    }
  }
  return allGood;
}

void es8311Init() {
  if (!i2cDevicePresent(ES8311_I2C_ADDR)) {
    Serial.println("ES8311 NOT FOUND on I2C bus -- speaker will stay silent");
    es8311OK = false;
    return;
  }

  // Configure, then read the key registers back; if any didn't stick (a
  // dropped or corrupted I2C write), redo the whole sequence.
  bool verified = false;
  for (uint8_t attempt = 1; attempt <= 3 && !verified; attempt++) {
    es8311ConfigureRegisters();
    verified = es8311VerifyRegisters();
    if (!verified) {
      Serial.printf("[ES8311] register check failed on attempt %u\n", attempt);
    }
  }
  if (!verified) {
    Serial.println("[ES8311] WARNING: codec registers still wrong after 3 attempts");
  }
  if (es8311WriteFailures > 0) {
    Serial.printf("[ES8311] %u I2C write(s) failed even after retries\n", es8311WriteFailures);
  }

  es8311OK = true;  // must be set before applyBuzzerVolume() below, which checks it

  applyBuzzerVolume();         // DAC volume -- pilot's chosen level (buzzerVolumePercent, Config > Volume menu)
  es8311WriteReg(0x31, 0x00);  // DAC: unmute

  Serial.println("ES8311 CODEC INITIALIZED");
}
// Applies buzzerVolumePercent (settings.h) to the ES8311's DAC digital
// volume register (0x32). That register is in 0.5dB steps with 0xBF = 0dB,
// 0xFF = +32dB and 0x00 = -95.5dB (ES8311 datasheet). The pilot's percentage
// is converted to dB with buzzerVolumePercentToDb() (settings.h): 10% steps,
// BUZZER_VOLUME_DB_PER_STEP dB each, 100% = BUZZER_VOLUME_MAX_DB. Called once
// at boot above, and again immediately whenever the pilot changes the
// Config > Volume menu setting.
void applyBuzzerVolume() {
  if (!es8311OK) return;
  const float db = buzzerVolumePercentToDb(buzzerVolumePercent);
  int reg = 0xBF + (int)lroundf(db * 2.0f);  // 0.5dB per register step
  if (reg < 0x00) reg = 0x00;
  if (reg > 0xFF) reg = 0xFF;
  es8311WriteReg(0x32, (uint8_t)reg);
}

// =====================================================
// TONE GENERATION: non-blocking. Unlike a single long i2s_write() call
// (which blocks for the tone's whole duration and would stall GPS/baro/
// display handling), this generates and pushes only a small chunk of
// samples per call, using a zero-timeout write so it only writes as much
// as the DMA buffer currently has room for and never blocks. Called on a
// fixed cadence by the independent audio-servicing timer set up in
// setup() -- see i2sToneService() -- so it can't be starved by loop()
// or the Core 0 background task doing something slow.
// =====================================================
void setToneFrequency(float freq) {
  toneFrequency = freq;
}
void i2sToneService() {

    if (!codecOK || !es8311OK) return;

    // Voice alerts take priority over the tone generator -- serve PCM
    // samples straight from PSRAM (see VoiceAlert.cpp) instead of
    // synthesizing a tone this chunk. Falls through to the normal tone
    // code below once the sentence finishes (voiceIsPlaying() becomes
    // false), so a climb/sink tone that was suppressed during playback
    // resumes on its own next call.
    if (voiceIsPlaying()) {
      int16_t chunk[I2S_TONE_CHUNK];
      voiceServiceChunk(chunk, I2S_TONE_CHUNK);
      size_t bytesWritten = 0;
#if AUDIO_USE_DEDICATED_TASK
      i2s_write(I2S_PORT, chunk, sizeof(chunk), &bytesWritten, pdMS_TO_TICKS(100));
#else
      i2s_write(I2S_PORT, chunk, sizeof(chunk), &bytesWritten, 0);
#endif
      return;
    }

    int16_t chunk[I2S_TONE_CHUNK];

    // Take a local copy so the requested frequency remains consistent
    // throughout this audio chunk.
    float freq = toneFrequency;

    // ---------------------------------------------------------
    // ANTI-CLICK ENVELOPE
    // Every on/off transition (climb pulse, sink alarm entry/exit, page
    // beep, mute jingle...) used to snap chunk[i] straight between 0 and a
    // mid-cycle sine value -- an instantaneous amplitude jump, which is
    // audible as a click/pop. At the vario's pulse rate (every 100-500ms
    // all flight) that's a constant background tick.
    //
    // Fix: toneAmplitude chases a target (TONE_PEAK_AMPLITUDE when sounding,
    // 0 when silent) by TONE_RAMP_STEP per sample instead of jumping. At
    // 16kHz / 50.0f per sample that's a ~6.25ms fade -- short enough not to
    // blur beep timing, long enough that the ear hears a fade, not a click.
    //
    // lastAudibleFreq keeps the waveform actually oscillating during a
    // fade-out (rather than freezing on one held sample) so the tail of
    // each beep decays like a real tone, not a ramped DC offset.
    // ---------------------------------------------------------
    const float TONE_PEAK_AMPLITUDE = 5000.0f;  // matches the previous fixed amplitude
    const float TONE_RAMP_STEP = 50.0f;         // ~6.25ms fade to/from full amplitude at 16kHz (at gain 1.0)

    // ---------------------------------------------------------
    // WAVEFORM: BAND-LIMITED SQUARE WAVE
    // The BlueFly drives an electromagnetic transducer with a square
    // wave; its buzzy character is the odd harmonics (3f, 5f, 7f...).
    // A pure sine has none, so on this small speaker a 130-350Hz sink
    // tone was thin and quiet. We sum the odd harmonics of a square
    // wave (1/k weighting) but stop below Nyquist so nothing aliases.
    // Harmonics are built with the recurrence
    //     sin((k+2)x) = 2cos(2x)*sin(kx) - sin((k-2)x)
    // so it costs only two trig calls per sample regardless of how
    // many harmonics are summed.
    //
    // SQUARE_LEVEL: a square wave has ~3dB more RMS than a sine of the
    // same peak; 0.72 keeps the climb beeps about as loud as they
    // were with the sine, so only the sink loudness changes.
    // ---------------------------------------------------------
    const float SQUARE_LEVEL = 0.72f;
    const float HARMONIC_LIMIT_HZ = 7500.0f;  // stay below Nyquist (8000Hz at 16kHz)

    // ---------------------------------------------------------
    // LOW-FREQUENCY GAIN COMPENSATION
    // The speaker rolls off hard below ~500Hz, so the sink tone (130-400Hz)
    // came out 6-8dB quieter than the climb beeps. Above LF_GAIN_KNEE_HZ
    // gain is 1.0; below it gain rises as sqrt(knee/freq), capped at
    // LF_GAIN_MAX. At 130Hz that is roughly +6dB. Tune these to taste:
    // raise LF_GAIN_MAX or LF_GAIN_KNEE_HZ for a louder sink tone.
    // ---------------------------------------------------------
    const float LF_GAIN_KNEE_HZ = 500.0f;
    const float LF_GAIN_MAX = 2.2f;

    static float toneAmplitude = 0.0f;
    static float lastAudibleFreq = 440.0f;
    static float smoothedFreq = 0.0f;  // 0 = no tone currently sounding

    // Pitch smoothing (see TONE_PITCH_SMOOTH_MS): a tone that is already
    // sounding glides toward its new pitch; a fresh tone (or a big jump)
    // starts at its target immediately.
    if (freq > 0.0f) {
        if (TONE_PITCH_SMOOTH_MS > 0.0f &&
            smoothedFreq > 0.0f &&
            fabsf(freq - smoothedFreq) <= 0.25f * smoothedFreq) {
            const float chunkMs = 1000.0f * (float)I2S_TONE_CHUNK / (float)I2S_SAMPLE_RATE;
            const float alpha = 1.0f - expf(-chunkMs / TONE_PITCH_SMOOTH_MS);
            smoothedFreq += (freq - smoothedFreq) * alpha;
        } else {
            smoothedFreq = freq;
        }
        lastAudibleFreq = smoothedFreq;
    } else {
        smoothedFreq = 0.0f;
    }

    const float phaseIncFreq = (freq > 0.0f) ? smoothedFreq : lastAudibleFreq;

    float lfGain = 1.0f;
    if (phaseIncFreq < LF_GAIN_KNEE_HZ) {
        lfGain = sqrtf(LF_GAIN_KNEE_HZ / phaseIncFreq);
        if (lfGain > LF_GAIN_MAX) lfGain = LF_GAIN_MAX;
    }

    // Per-tone loudness multiplier (climb/sink volume settings, page beeps
    // and the mute/unmute jingle all set toneGain). 0 = silent.
    const float stepGain = (toneGain > 0.0f) ? toneGain : 0.0f;

    const float totalGain = lfGain * stepGain;

    // Remember the gain of the tone that was last actually sounding, so the
    // fade-out at the end of a beep (when freq is 0) keeps the same ~6ms
    // fade length instead of being far too abrupt for a quiet tone.
    static float lastSoundingGain = 1.0f;
    if (freq > 0.0f && totalGain > 0.0f) {
        lastSoundingGain = totalGain;
    }

    const float targetAmplitude = (freq > 0.0f) ? (TONE_PEAK_AMPLITUDE * totalGain) : 0.0f;
    // Scale the fade step with the gain so the anti-click fade stays ~6ms.
    const float rampStep = TONE_RAMP_STEP * lastSoundingGain;

    // Number of odd harmonics that fit below HARMONIC_LIMIT_HZ.
    int maxHarmonic = (int)(HARMONIC_LIMIT_HZ / phaseIncFreq);
    if (maxHarmonic < 1) maxHarmonic = 1;
    if ((maxHarmonic & 1) == 0) maxHarmonic--;  // largest odd k <= limit

    for (int i = 0; i < I2S_TONE_CHUNK; i++) {

        if (toneAmplitude < targetAmplitude) {
            toneAmplitude += rampStep;
            if (toneAmplitude > targetAmplitude) toneAmplitude = targetAmplitude;
        } else if (toneAmplitude > targetAmplitude) {
            toneAmplitude -= rampStep;
            if (toneAmplitude < targetAmplitude) toneAmplitude = targetAmplitude;
        }

        const float x = 2.0f * PI * tonePhase;
        const float twoCos2x = 2.0f * cosf(2.0f * x);

        float sPrev = -sinf(x);  // sin(-1 * x)
        float sCur = sinf(x);    // sin(1 * x)
        float sum = sCur;        // k = 1 term (weight 1/1)

        for (int k = 3; k <= maxHarmonic; k += 2) {
            const float sNext = twoCos2x * sCur - sPrev;  // sin(k * x)
            sum += sNext / (float)k;
            sPrev = sCur;
            sCur = sNext;
        }

        // (4/pi) scales the harmonic sum to a unit-amplitude square wave.
        chunk[i] = (int16_t)(
            toneAmplitude * SQUARE_LEVEL * (4.0f / PI) * sum
        );

        tonePhase += phaseIncFreq / (float)I2S_SAMPLE_RATE;

        if (tonePhase >= 1.0f) {
            tonePhase -= 1.0f;
        }
    }

    size_t bytesWritten = 0;

#if AUDIO_USE_DEDICATED_TASK
    // Blocking write: waits for room in the DMA ring, which is what paces
    // audioServiceTask() to the I2S hardware clock. (Bounded, so a stalled
    // I2S peripheral can't hang the task forever.)
    i2s_write(
        I2S_PORT,
        chunk,
        sizeof(chunk),
        &bytesWritten,
        pdMS_TO_TICKS(100)
    );
#else
    // Zero-timeout: writes only what fits and never blocks (timer callback).
    i2s_write(
        I2S_PORT,
        chunk,
        sizeof(chunk),
        &bytesWritten,
        0
    );
#endif
}

#if AUDIO_USE_DEDICATED_TASK
// Runs forever: generate one chunk, hand it to the I2S DMA ring, repeat.
// i2s_write() blocks when the ring is full, so this loop runs exactly as
// fast as the hardware plays samples -- see AUDIO_USE_DEDICATED_TASK.
void audioServiceTask(void* arg) {
  (void)arg;
  for (;;) {
    if (!codecOK || !es8311OK) {
      // I2S/codec not up (yet, or failed) -- don't spin.
      vTaskDelay(pdMS_TO_TICKS(20));
      continue;
    }
    i2sToneService();
  }
}
#endif
