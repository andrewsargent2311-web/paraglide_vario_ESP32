#include "Buzzer.h"

#include <Wire.h>
#include <math.h>

// =====================================================
// I2S / ES8311 HARDWARE
// =====================================================
esp_timer_handle_t audioServiceTimer = nullptr;

bool codecOK = false;   // I2S peripheral configured
bool es8311OK = false;  // ES8311 chip found and initialized over I2C

volatile float toneFrequency = 0.0f;  // 0 = silent
float tonePhase = 0.0f;

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
void es8311WriteReg(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(ES8311_I2C_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

void es8311Init() {
  if (!i2cDevicePresent(ES8311_I2C_ADDR)) {
    Serial.println("ES8311 NOT FOUND on I2C bus -- speaker will stay silent");
    es8311OK = false;
    return;
  }

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

  es8311OK = true;  // must be set before applyBuzzerVolume() below, which checks it

  applyBuzzerVolume();         // DAC volume -- pilot's chosen level (buzzerVolumePercent, Config > Volume menu)
  es8311WriteReg(0x31, 0x00);  // DAC: unmute

  Serial.println("ES8311 CODEC INITIALIZED");
}
// Applies buzzerVolumePercent (settings.h) to the ES8311's DAC digital
// volume register (0x32): 0x00 = mute, 0xFF = 0dB (loudest). Called once
// at boot above, and again immediately whenever the pilot changes the
// Config > Volume menu setting -- see the buzzerVolumePercent comment in
// settings.h for the dB-linear-vs-perceived-loudness caveat.
void applyBuzzerVolume() {
  if (!es8311OK) return;
  uint8_t reg = (uint8_t)((buzzerVolumePercent / 100.0f) * 255.0f + 0.5f);
  es8311WriteReg(0x32, reg);
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
    const float TONE_RAMP_STEP = 50.0f;         // ~6.25ms fade to/from full amplitude at 16kHz

    static float toneAmplitude = 0.0f;
    static float lastAudibleFreq = 440.0f;

    if (freq > 0.0f) {
        lastAudibleFreq = freq;
    }

    const float phaseIncFreq = (freq > 0.0f) ? freq : lastAudibleFreq;
    const float targetAmplitude = (freq > 0.0f) ? TONE_PEAK_AMPLITUDE : 0.0f;

    for (int i = 0; i < I2S_TONE_CHUNK; i++) {

        if (toneAmplitude < targetAmplitude) {
            toneAmplitude += TONE_RAMP_STEP;
            if (toneAmplitude > targetAmplitude) toneAmplitude = targetAmplitude;
        } else if (toneAmplitude > targetAmplitude) {
            toneAmplitude -= TONE_RAMP_STEP;
            if (toneAmplitude < targetAmplitude) toneAmplitude = targetAmplitude;
        }

        chunk[i] = (int16_t)(
            toneAmplitude * sinf(2.0f * PI * tonePhase)
        );

        tonePhase += phaseIncFreq / (float)I2S_SAMPLE_RATE;

        if (tonePhase >= 1.0f) {
            tonePhase -= 1.0f;
        }
    }

    size_t bytesWritten = 0;

    i2s_write(
        I2S_PORT,
        chunk,
        sizeof(chunk),
        &bytesWritten,
        0
    );
}

// =====================================================
// MUTE / UNMUTE CONFIRMATION TONE
// =====================================================
bool muteToneActive = false;
unsigned long muteToneStart = 0;
bool muteToneIsMuteSequence = false;

// =====================================================
// ADS-B INTERCEPT ALARM
// =====================================================
bool interceptAlarmActive = false;
unsigned long interceptAlarmStart = 0;

// =====================================================
// VARIO SINK/CLIMB TONE STATE
// =====================================================
bool sinkAlarmActive = false;
bool climbAudioActive = false;
unsigned long sinkAlarmStart = 0;
unsigned long climbPulseStart = 0;
bool climbToneOn = false;
bool beepOn = false;

// =====================================================
// PAGE-CHANGE BEEP PRIORITY WINDOW
// =====================================================
unsigned long pageBeepUntil = 0;  // while set, updateI2sAudioBuzzer() yields the pin to the page-change beep

// =====================================================
// BUZZER: non-blocking climb/sink tone over the I2S speaker.
// Yields to the page-change beep for its short duration rather than
// talking over it -- both share the same physical speaker.
// =====================================================
void updateI2sAudioBuzzer() {

  const unsigned long now = millis();

  // ============================================================
  // PAGE-CHANGE BEEP HAS PRIORITY
  // ============================================================

  if ((int32_t)(pageBeepUntil - now) > 0) {
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
  // ADS-B INTERCEPT ALARM: 5s of alternating tone, takes priority
  // over vario climb/sink audio (but not the page-change beep).
  // ============================================================
  if (interceptAlarmActive) {

    unsigned long elapsed = now - interceptAlarmStart;

    if (elapsed >= INTERCEPT_ALARM_DURATION_MS) {
      interceptAlarmActive = false;

      setToneFrequency(0);

      // Force vario audio to re-evaluate cleanly next pass.
      sinkAlarmActive = false;
      climbAudioActive = false;
      climbToneOn = false;

    } else if (!adsbAlarmMuted) {
      unsigned long phase = elapsed % (INTERCEPT_TONE_TOGGLE_MS * 2);
      float freq = (phase < INTERCEPT_TONE_TOGGLE_MS) ? INTERCEPT_TONE_HIGH_HZ : INTERCEPT_TONE_LOW_HZ;

      setToneFrequency(freq);

      return;  // Skip vario tone logic entirely while the alarm sounds
    }
    // else: alarm is muted -- still tracked (see elapsed check above) but
    // silent, and falls through to normal vario tone logic below instead
    // of overriding it.
  }

  // ============================================================
  // MUTE / UNMUTE CONFIRMATION TONE
  // Takes priority over the plain muted/vario logic below (but not the
  // page beep or ADS-B intercept alarm above), so the pilot always hears
  // it clearly -- even though buzzerMuted has often already flipped to
  // true by the time this plays.
  // ============================================================
  if (muteToneActive) {

    unsigned long elapsed = now - muteToneStart;

    float freqA = muteToneIsMuteSequence ? MUTE_TONE_FREQ_HIGH_HZ : MUTE_TONE_FREQ_LOW_HZ;
    unsigned long durA = muteToneIsMuteSequence ? MUTE_TONE_HIGH_MS : MUTE_TONE_LOW_MS;
    float freqB = muteToneIsMuteSequence ? MUTE_TONE_FREQ_LOW_HZ : MUTE_TONE_FREQ_HIGH_HZ;
    unsigned long durB = muteToneIsMuteSequence ? MUTE_TONE_LOW_MS : MUTE_TONE_HIGH_MS;

    if (elapsed < durA) {

      setToneFrequency(freqA);
      return;

    } else if (elapsed < durA + MUTE_TONE_GAP_MS) {

      setToneFrequency(0);
      return;

    } else if (elapsed < durA + MUTE_TONE_GAP_MS + durB) {

      setToneFrequency(freqB);
      return;

    } else {

      muteToneActive = false;
      setToneFrequency(0);

      if (buzzerMuted) {
        // The mute confirmation jingle has finished -- now actually cut
        // the amp. (Unmuting already turned it on immediately, back in
        // updatePageButton().)
        digitalWrite(AMP_ENABLE_PIN, LOW);
      }
      // Fall through to the MUTED / vario logic below, which now
      // correctly reflects whichever state buzzerMuted settled on.
    }
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
  //
  // Enter sink alarm at <= -0.5 m/s (SINK_ALARM_MS).
  // Remain in alarm until climb rate recovers past
  // SINK_ALARM_RELEASE_MS (-0.2 m/s) -- a separate, less-negative
  // threshold from the entry point.
  //
  // FIX: this used to release at the same threshold it entered on
  // (SINK_ALARM_MS for both), which is not hysteresis at all -- a climb
  // rate hovering around -0.5 m/s from sensor noise would flicker the
  // alarm on/off rapidly ("motorboating"). Entry and release now use
  // different thresholds, so the climb rate has to genuinely recover
  // before the alarm clears.
  // ============================================================

  const float SINK_ALARM_RELEASE_MS = -0.2f;  // must recover past this (less negative than SINK_ALARM_MS) to release

  if (!sinkAlarmActive) {

    if (currentClimbRateMS <= SINK_ALARM_MS) {

      sinkAlarmActive = true;
      sinkAlarmStart = now;

      // Make sure climb audio is cancelled
      climbAudioActive = false;
      climbToneOn = false;
    }

  } else {

    // Hysteresis release.
    // Keep the sink alarm active while descending.
    // Release only once the climb rate has recovered past the
    // separate release threshold above.

    if (currentClimbRateMS > SINK_ALARM_RELEASE_MS) {

        sinkAlarmActive = false;

        setToneFrequency(0);
    }
}


  // ============================================================
  // SINK ALARM OUTPUT
  //
  // Constant tone (no on/off pulsing) so sink reads as one continuous,
  // unambiguous warning rather than something that could be mistaken for
  // weak-lift beeping. Pitch drops as sink strengthens: SINK_TONE_MAX_HZ
  // right at the SINK_ALARM_MS threshold, down to SINK_TONE_MIN_HZ at/
  // beyond SINK_TONE_MAX_MS.
  // ============================================================

  if (sinkAlarmActive) {

    float sinkFactor =
      (currentClimbRateMS - SINK_ALARM_MS) / (SINK_TONE_MAX_MS - SINK_ALARM_MS);

    sinkFactor = constrain(sinkFactor, 0.0f, 1.0f);

    int sinkToneFreq =
      SINK_TONE_MAX_HZ - (int)(sinkFactor * (SINK_TONE_MAX_HZ - SINK_TONE_MIN_HZ));

    setToneFrequency(sinkToneFreq);

    return;
  }


  // ============================================================
  // CLIMB DEAD BAND
  //
  // Below +0.15 m/s there is no climb tone.
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

    // Start the first pulse after a short delay rather than
    // immediately producing a tone.
    climbPulseStart = now;
  }


  // ============================================================
  // NORMALISE CLIMB RATE
  //
  // 0.15 m/s -> 0.0
  // 5.0  m/s -> 1.0
  //
  // Anything above 5 m/s is capped at 1.0.
  // ============================================================

  float factor =
    (currentClimbRateMS - CLIMB_DEADBAND_MS) / (CLIMB_TONE_MAX_MS - CLIMB_DEADBAND_MS);

  factor = constrain(factor, 0.0f, 1.0f);


  // ============================================================
  // NONLINEAR RESPONSE
  //
  // sqrt() gives more audio resolution in weak lift.
  //
  // This is important for a paraglider because the difference
  // between 0.2 and 0.5 m/s is much more useful to the pilot
  // than making 4 and 5 m/s dramatically different.
  // ============================================================

    float response = powf(factor, 0.70f); //changed from;   float response = sqrtf(factor);


  // ============================================================
  // TONE FREQUENCY
  //
  // Approximately:
  //
  // 0.15 m/s -> 400 Hz
  // 0.5  m/s -> ~530 Hz
  // 1.0  m/s -> ~650 Hz
  // 2.0  m/s -> ~790 Hz
  // 3.0  m/s -> ~890 Hz
  // 5.0  m/s -> 1100 Hz
  // ============================================================

  int toneFreq =
    climbToneMinHz + (int)(response * (climbToneMaxHz - climbToneMinHz));


  // ============================================================
  // NONLINEAR PULSE TIMING
  //
  // Weak lift:
  //     long gaps
  //
  // Strong lift:
  //     short gaps
  //
  // Using sqrt() here gives a more progressive response.
  // ============================================================

  float pulseResponse = powf(factor, 0.70f);    // Changed from - float pulseResponse = sqrtf(factor);

  unsigned long gapMs =
    climbGapMaxMs - (unsigned long)(pulseResponse * (climbGapMaxMs - climbGapMinMs));


  // ============================================================
  // PULSE LENGTH
  //
  // Beeps become progressively shorter with increasing lift, mirroring
  // the gap timing above -- both compress together as lift strengthens,
  // toward the continuous-tone region below.
  // ============================================================

  unsigned long pulseMs =
    climbPulseMaxMs - (unsigned long)(response * (climbPulseMaxMs - climbPulseMinMs));


  // ============================================================
  // STRONG-LIFT CONTINUOUS-TONE REGION
  //
  // Above roughly 80% of the configured climb range, the
  // individual pulses become close enough together that a
  // continuous tone is more useful.
  // ============================================================

  if (factor >= 0.80f) {

    climbToneOn = true;

    setToneFrequency(toneFreq);

    return;
  }


  // ============================================================
  // CLIMB PULSE GENERATOR
  // ============================================================

  if (climbToneOn) {

    // Currently sounding
    if ((now - climbPulseStart) >= pulseMs) {

      climbToneOn = false;
      climbPulseStart = now;

      setToneFrequency(0);
    }

  } else {

    // Currently silent
    if ((now - climbPulseStart) >= gapMs) {

      climbToneOn = true;
      climbPulseStart = now;

      setToneFrequency(toneFreq);
    }
  }
}

// Sets up a short non-blocking tone burst for UI feedback (page change,
// menu open/navigate/select). Actual sample generation happens on the
// independent audio-servicing timer (see setup()), which starts within
// AUDIO_SERVICE_INTERVAL_US regardless -- no need to pump it manually
// here, and doing so would race with the timer callback over tonePhase.
// Respects the mute setting.
void playFeedbackTone(float freq, unsigned long durationMs) {
  if (buzzerMuted) return;
  setToneFrequency(freq);
  pageBeepUntil = millis() + durationMs;
}
