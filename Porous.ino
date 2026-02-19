#include <Arduino.h>
#include <driver/gpio.h>
#include <driver/i2s.h>
#include <math.h>
#include <stdint.h>
#include "audio_data.h"

// XIAO ESP32S3 wiring
static const i2s_port_t I2S_PORT = I2S_NUM_0;
static const int PIN_I2S_BCLK = 7;  // D8
static const int PIN_I2S_WS = 8;    // D9
static const int PIN_I2S_DIN = 3;   // D2 (mic DOUT)
static const int PIN_I2S_DOUT = 9;  // D10 (amp DIN)
static const int PIN_AMP_SD = 4;    // D3 (amp shutdown/enable)

// Shared I2S clock domain for mic + amp
static const uint32_t SAMPLE_RATE = 48000;
static const size_t MIC_FRAME_WINDOW = 1024;   // ~21.3 ms at 48 kHz

// SPL estimation + triggering
static const float SPL_OFFSET_DB = 120.0f;
static const float SPL_CALIBRATION_DB = 0.0f;
static const float ROLLING_AVG_MS = 750.0f;
static const float THRESHOLD_DB_SPL = 60.0f;
static const float HYSTERESIS_DB = 3.0f;
static const float PLAYBACK_GAIN = 0.5f;     // 0.0 = mute, 1.0 = original level

// Audio header is 16-bit mono @ 16 kHz.
// This sketch runs I2S at 48 kHz, so we upsample by repeating each sample 3x.
static const uint32_t AUDIO_SOURCE_RATE = 16000;
static const uint32_t UPSAMPLE_FACTOR = SAMPLE_RATE / AUDIO_SOURCE_RATE;

static_assert(SAMPLE_RATE % AUDIO_SOURCE_RATE == 0, "SAMPLE_RATE must be multiple of source rate");

struct ChannelStats {
  float rms_l;
  float rms_r;
};

struct UnpackDetect {
  bool prefer_left_justified;
  size_t nonzero_slots;
  size_t low_byte_nonzero;
};

static bool smooth_initialized = false;
static float smooth_rms = 0.0f;
static bool playback_active = false;

static size_t audio_sample_index = 0;
static uint32_t upsample_phase = 0;

static inline int32_t unpackLeftJustified24(int32_t raw32) {
  return raw32 >> 8;
}

static inline int32_t unpackRightJustified24(int32_t raw32) {
  uint32_t v = static_cast<uint32_t>(raw32) & 0x00FFFFFF;
  if (v & 0x00800000) v |= 0xFF000000;
  return static_cast<int32_t>(v);
}

UnpackDetect detectUnpackMode(const int32_t* slots, size_t frame_count) {
  size_t nonzero_slots = 0;
  size_t low_byte_nonzero = 0;

  for (size_t i = 0; i < frame_count * 2; ++i) {
    int32_t s = slots[i];
    if (s == 0) continue;
    nonzero_slots++;
    if ((static_cast<uint32_t>(s) & 0xFFu) != 0u) low_byte_nonzero++;
  }

  bool prefer_lj = true;
  if (nonzero_slots > 0) {
    float frac = static_cast<float>(low_byte_nonzero) / static_cast<float>(nonzero_slots);
    prefer_lj = (frac < 0.10f);
  }

  UnpackDetect out = {};
  out.prefer_left_justified = prefer_lj;
  out.nonzero_slots = nonzero_slots;
  out.low_byte_nonzero = low_byte_nonzero;
  return out;
}

ChannelStats computeStats(const int32_t* slots, size_t frame_count, bool left_justified_unpack) {
  int64_t sum_l = 0;
  int64_t sum_r = 0;

  for (size_t i = 0; i < frame_count * 2; i += 2) {
    int32_t l = left_justified_unpack ? unpackLeftJustified24(slots[i]) : unpackRightJustified24(slots[i]);
    int32_t r = left_justified_unpack ? unpackLeftJustified24(slots[i + 1]) : unpackRightJustified24(slots[i + 1]);
    sum_l += static_cast<int64_t>(l);
    sum_r += static_cast<int64_t>(r);
  }

  int32_t mean_l = static_cast<int32_t>(sum_l / static_cast<int64_t>(frame_count));
  int32_t mean_r = static_cast<int32_t>(sum_r / static_cast<int64_t>(frame_count));

  double sum_sq_l = 0.0;
  double sum_sq_r = 0.0;

  for (size_t i = 0; i < frame_count * 2; i += 2) {
    int32_t l0 = left_justified_unpack ? unpackLeftJustified24(slots[i]) : unpackRightJustified24(slots[i]);
    int32_t r0 = left_justified_unpack ? unpackLeftJustified24(slots[i + 1]) : unpackRightJustified24(slots[i + 1]);
    int32_t l = l0 - mean_l;
    int32_t r = r0 - mean_r;

    float nl = static_cast<float>(l) / 8388607.0f;
    float nr = static_cast<float>(r) / 8388607.0f;
    sum_sq_l += static_cast<double>(nl) * static_cast<double>(nl);
    sum_sq_r += static_cast<double>(nr) * static_cast<double>(nr);
  }

  ChannelStats out = {};
  out.rms_l = sqrt(sum_sq_l / static_cast<double>(frame_count));
  out.rms_r = sqrt(sum_sq_r / static_cast<double>(frame_count));
  return out;
}

void fillPlaybackFrames(int32_t* tx_slots, size_t frame_count) {
  const int16_t* samples = reinterpret_cast<const int16_t*>(branches_loop_pcm);
  const size_t sample_count = branches_loop_pcm_len / sizeof(int16_t);

  for (size_t i = 0; i < frame_count; ++i) {
    int32_t s32 = 0;

    if (playback_active && sample_count > 0) {
      int16_t s16 = samples[audio_sample_index];
      int32_t scaled = static_cast<int32_t>(lroundf(static_cast<float>(s16) * PLAYBACK_GAIN));
      if (scaled > 32767) scaled = 32767;
      if (scaled < -32768) scaled = -32768;
      s32 = static_cast<int32_t>(static_cast<int16_t>(scaled)) << 16;  // 16-bit value into 32-bit I2S slot MSBs

      upsample_phase++;
      if (upsample_phase >= UPSAMPLE_FACTOR) {
        upsample_phase = 0;
        audio_sample_index++;
        if (audio_sample_index >= sample_count) audio_sample_index = 0;
      }
    }

    tx_slots[i * 2] = s32;      // Left slot
    tx_slots[i * 2 + 1] = s32;  // Right slot
  }
}

void setupI2S() {
  i2s_config_t cfg = {};
  cfg.mode = static_cast<i2s_mode_t>(I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_RX);
  cfg.sample_rate = SAMPLE_RATE;
  cfg.bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT;
  cfg.channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT;
  cfg.communication_format = I2S_COMM_FORMAT_STAND_I2S;
  cfg.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1;
  cfg.dma_buf_count = 8;
  cfg.dma_buf_len = 256;
  cfg.use_apll = false;
  cfg.tx_desc_auto_clear = true;
  cfg.fixed_mclk = 0;

  i2s_pin_config_t pins = {};
  pins.bck_io_num = PIN_I2S_BCLK;
  pins.ws_io_num = PIN_I2S_WS;
  pins.data_out_num = PIN_I2S_DOUT;
  pins.data_in_num = PIN_I2S_DIN;

  esp_err_t e = i2s_driver_install(I2S_PORT, &cfg, 0, nullptr);
  if (e != ESP_OK) {
    Serial.printf("i2s_driver_install failed: %d\n", static_cast<int>(e));
    while (true) delay(1000);
  }

  e = i2s_set_pin(I2S_PORT, &pins);
  if (e != ESP_OK) {
    Serial.printf("i2s_set_pin failed: %d\n", static_cast<int>(e));
    while (true) delay(1000);
  }

  e = i2s_set_clk(I2S_PORT, SAMPLE_RATE, I2S_BITS_PER_SAMPLE_32BIT, I2S_CHANNEL_STEREO);
  if (e != ESP_OK) {
    Serial.printf("i2s_set_clk failed: %d\n", static_cast<int>(e));
    while (true) delay(1000);
  }

  gpio_pulldown_en(static_cast<gpio_num_t>(PIN_I2S_DIN));
  i2s_start(I2S_PORT);
  i2s_zero_dma_buffer(I2S_PORT);
}

void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(PIN_AMP_SD, OUTPUT);
  digitalWrite(PIN_AMP_SD, LOW);

  setupI2S();

  Serial.println("Porous started");
  Serial.printf("threshold=%.1f dBSPL, hysteresis=%.1f dB, smoothing=%.0f ms, gain=%.2f\n",
                THRESHOLD_DB_SPL, HYSTERESIS_DB, ROLLING_AVG_MS, PLAYBACK_GAIN);
}

void loop() {
  static int32_t mic_slots[MIC_FRAME_WINDOW * 2];
  static int32_t tx_slots[MIC_FRAME_WINDOW * 2];
  static uint32_t last_print_ms = 0;

  size_t bytes_read = 0;
  esp_err_t err = i2s_read(I2S_PORT, mic_slots, sizeof(mic_slots), &bytes_read, portMAX_DELAY);
  if (err != ESP_OK || bytes_read == 0) {
    Serial.println("i2s_read failed");
    return;
  }

  size_t frame_count = (bytes_read / sizeof(int32_t)) / 2;
  if (frame_count == 0) return;

  UnpackDetect unpack_detect = detectUnpackMode(mic_slots, frame_count);
  bool use_lj = unpack_detect.prefer_left_justified;
  ChannelStats s = computeStats(mic_slots, frame_count, use_lj);

  // Mic is wired with SEL=GND, so left slot carries the valid channel.
  float rms = s.rms_l;
  float dbfs = (rms > 0.000001f) ? 20.0f * log10f(rms) : -120.0f;
  float spl_inst = dbfs + SPL_OFFSET_DB + SPL_CALIBRATION_DB;

  float dt_s = static_cast<float>(frame_count) / static_cast<float>(SAMPLE_RATE);
  float tau_s = ROLLING_AVG_MS / 1000.0f;
  float alpha = (tau_s > 0.0f) ? (1.0f - expf(-dt_s / tau_s)) : 1.0f;
  if (alpha < 0.0f) alpha = 0.0f;
  if (alpha > 1.0f) alpha = 1.0f;

  if (!smooth_initialized) {
    smooth_rms = rms;
    smooth_initialized = true;
  } else {
    smooth_rms += alpha * (rms - smooth_rms);
  }

  float smooth_dbfs = (smooth_rms > 0.000001f) ? 20.0f * log10f(smooth_rms) : -120.0f;
  float spl_smooth = smooth_dbfs + SPL_OFFSET_DB + SPL_CALIBRATION_DB;

  bool prev_state = playback_active;
  if (!playback_active && spl_smooth >= THRESHOLD_DB_SPL) {
    playback_active = true;
  } else if (playback_active && spl_smooth <= (THRESHOLD_DB_SPL - HYSTERESIS_DB)) {
    playback_active = false;
  }

  if (playback_active != prev_state) {
    digitalWrite(PIN_AMP_SD, playback_active ? HIGH : LOW);
  }

  fillPlaybackFrames(tx_slots, frame_count);
  size_t bytes_written = 0;
  i2s_write(I2S_PORT, tx_slots, frame_count * 2 * sizeof(int32_t), &bytes_written, portMAX_DELAY);

  uint32_t now = millis();
  if (now - last_print_ms >= 100) {
    last_print_ms = now;
    Serial.print("UNPACK=");
    Serial.print(use_lj ? "LJ" : "RJ");
    Serial.print(" lsbNZ=");
    Serial.print(unpack_detect.low_byte_nonzero);
    Serial.print("/");
    Serial.print(unpack_detect.nonzero_slots);
    Serial.print(" dBSPL(inst)=");
    Serial.print(spl_inst, 1);
    Serial.print(" dBSPL(smooth)=");
    Serial.print(spl_smooth, 1);
    Serial.print(" state=");
    Serial.println(playback_active ? "PLAYING" : "IDLE");
  }
}
