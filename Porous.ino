#include <Arduino.h>
#include <driver/gpio.h>
#include <driver/i2s.h>
#include <math.h>
#include <stdint.h>
#include <Preferences.h>
#include "audio_data.h"

// XIAO ESP32S3 wiring
static const i2s_port_t I2S_PORT = I2S_NUM_0;
static const int PIN_I2S_BCLK = 7;  // D8
static const int PIN_I2S_WS = 8;    // D9
static const int PIN_I2S_DIN = 3;   // D2 (mic DOUT)
static const int PIN_I2S_DOUT = 9;  // D10 (PCM5102A DIN)

// Shared I2S clock domain for mic + DAC
static const uint32_t SAMPLE_RATE = 48000;
static const size_t MIC_FRAME_WINDOW = 1024;   // ~21.3 ms at 48 kHz

// SPL conversion constants
static const float SPL_OFFSET_DB = 120.0f;
static const float SPL_CALIBRATION_DB = 0.0f;

// Default tunables (overridable via serial + NVS)
static const float DEFAULT_ROLLING_AVG_MS_FAST = 200.0f;
static const float DEFAULT_ROLLING_AVG_MS_SLOW = 5000.0f;
static const float DEFAULT_THRESHOLD_DB_SPL = 60.0f;
static const float DEFAULT_HYSTERESIS_DB = 3.0f;
static const float DEFAULT_TARGET_MASK_DB = -8.0f;
static const float DEFAULT_BASE_GAIN = 0.5f;
static const float DEFAULT_PLAYBACK_GAIN_MIN = 0.02f;
static const float DEFAULT_PLAYBACK_GAIN_MAX = 1.00f;
static const float DEFAULT_GAIN_STEP_DB_PER_SEC = 2.0f;
static const float DEFAULT_GAIN_TRIM_DB_LIMIT = 6.0f;
static const bool DEFAULT_AUTO_GAIN_ENABLED = true;
static const uint32_t DEFAULT_MIN_ON_MS = 3000;
static const uint32_t DEFAULT_MIN_OFF_MS = 3000;
static const bool DEFAULT_DEBUG_ENABLED = true;
static const uint32_t TELEMETRY_INTERVAL_MS = 1000;
static const float FADE_IN_MS = 220.0f;
static const float FADE_OUT_MS = 220.0f;

// Audio header is 16-bit mono @ 16 kHz.
// This sketch runs I2S at 48 kHz, so we upsample by repeating each sample 3x.
static const uint32_t AUDIO_SOURCE_RATE = 16000;
static const uint32_t UPSAMPLE_FACTOR = SAMPLE_RATE / AUDIO_SOURCE_RATE;

static_assert(SAMPLE_RATE % AUDIO_SOURCE_RATE == 0, "SAMPLE_RATE must be multiple of source rate");

static Preferences prefs;

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
static float fast_rms = 0.0f;
static float ambient_rms = 0.0f;
static bool playback_active = false;
static uint32_t last_state_change_ms = 0;
static float base_playback_gain = DEFAULT_BASE_GAIN;
static float gain_trim_db = 0.0f;
static float playback_gain_effective = DEFAULT_BASE_GAIN;
static float playback_envelope = 0.0f;
static String serial_line;

// Runtime-configurable parameters
static float rolling_avg_ms_fast = DEFAULT_ROLLING_AVG_MS_FAST;
static float rolling_avg_ms_slow = DEFAULT_ROLLING_AVG_MS_SLOW;
static float threshold_db_spl = DEFAULT_THRESHOLD_DB_SPL;
static float hysteresis_db = DEFAULT_HYSTERESIS_DB;
static float target_mask_db = DEFAULT_TARGET_MASK_DB;
static float playback_gain_min = DEFAULT_PLAYBACK_GAIN_MIN;
static float playback_gain_max = DEFAULT_PLAYBACK_GAIN_MAX;
static float gain_step_db_per_sec = DEFAULT_GAIN_STEP_DB_PER_SEC;
static float gain_trim_db_limit = DEFAULT_GAIN_TRIM_DB_LIMIT;
static bool auto_gain_enabled = DEFAULT_AUTO_GAIN_ENABLED;
static uint32_t min_on_ms = DEFAULT_MIN_ON_MS;
static uint32_t min_off_ms = DEFAULT_MIN_OFF_MS;
static bool debug_enabled = DEFAULT_DEBUG_ENABLED;

enum class CalMode {
  NONE,
  ROOM,
  ACTIVE,
};

static CalMode cal_mode = CalMode::NONE;
static uint32_t cal_end_ms = 0;
static double cal_sum_db = 0.0;
static uint32_t cal_count = 0;
static bool has_cal_room = false;
static bool has_cal_active = false;
static float cal_room_db = 0.0f;
static float cal_active_db = 0.0f;

static size_t audio_sample_index = 0;
static uint32_t upsample_phase = 0;

static inline int32_t unpackLeftJustified24(int32_t raw32) {
  return raw32 >> 8;
}

static inline float clampf(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
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

void applyConfigBounds() {
  rolling_avg_ms_fast = clampf(rolling_avg_ms_fast, 20.0f, 5000.0f);
  rolling_avg_ms_slow = clampf(rolling_avg_ms_slow, rolling_avg_ms_fast, 30000.0f);
  threshold_db_spl = clampf(threshold_db_spl, 20.0f, 120.0f);
  hysteresis_db = clampf(hysteresis_db, 0.5f, 20.0f);
  target_mask_db = clampf(target_mask_db, -30.0f, 0.0f);
  playback_gain_min = clampf(playback_gain_min, 0.0f, 1.0f);
  playback_gain_max = clampf(playback_gain_max, playback_gain_min, 1.2f);
  gain_step_db_per_sec = clampf(gain_step_db_per_sec, 0.1f, 20.0f);
  gain_trim_db_limit = clampf(gain_trim_db_limit, 0.0f, 24.0f);
  min_on_ms = static_cast<uint32_t>(clampf(static_cast<float>(min_on_ms), 0.0f, 60000.0f));
  min_off_ms = static_cast<uint32_t>(clampf(static_cast<float>(min_off_ms), 0.0f, 60000.0f));
  base_playback_gain = clampf(base_playback_gain, playback_gain_min, playback_gain_max);
  gain_trim_db = clampf(gain_trim_db, -gain_trim_db_limit, gain_trim_db_limit);
}

void printConfig() {
  Serial.println("CONFIG:");
  Serial.printf("  debug=%d\n", debug_enabled ? 1 : 0);
  Serial.printf("  target_mask_db=%.2f\n", target_mask_db);
  Serial.printf("  threshold_db_spl=%.2f\n", threshold_db_spl);
  Serial.printf("  hysteresis_db=%.2f\n", hysteresis_db);
  Serial.printf("  rolling_avg_ms_fast=%.1f\n", rolling_avg_ms_fast);
  Serial.printf("  rolling_avg_ms_slow=%.1f\n", rolling_avg_ms_slow);
  Serial.printf("  gain=%.4f\n", base_playback_gain);
  Serial.printf("  auto_gain=%d\n", auto_gain_enabled ? 1 : 0);
  Serial.printf("  gain_trim_db=%.2f\n", gain_trim_db);
  Serial.printf("  gain_effective=%.4f\n", playback_gain_effective);
  Serial.printf("  gain_min=%.4f\n", playback_gain_min);
  Serial.printf("  gain_max=%.4f\n", playback_gain_max);
  Serial.printf("  gain_step_db_per_sec=%.2f\n", gain_step_db_per_sec);
  Serial.printf("  gain_trim_db_limit=%.2f\n", gain_trim_db_limit);
  Serial.printf("  min_on_ms=%lu\n", static_cast<unsigned long>(min_on_ms));
  Serial.printf("  min_off_ms=%lu\n", static_cast<unsigned long>(min_off_ms));
  if (has_cal_room) Serial.printf("  cal_room_db=%.2f\n", cal_room_db);
  if (has_cal_active) Serial.printf("  cal_active_db=%.2f\n", cal_active_db);
  if (cal_mode != CalMode::NONE) {
    Serial.printf("  calibration=running (%s)\n", cal_mode == CalMode::ROOM ? "room" : "active");
  }
}

void loadConfigFromNVS() {
  if (!prefs.begin("porous", true)) return;
  debug_enabled = prefs.getBool("debug", DEFAULT_DEBUG_ENABLED);
  target_mask_db = prefs.getFloat("targetMask", DEFAULT_TARGET_MASK_DB);
  threshold_db_spl = prefs.getFloat("thr", DEFAULT_THRESHOLD_DB_SPL);
  hysteresis_db = prefs.getFloat("hyst", DEFAULT_HYSTERESIS_DB);
  rolling_avg_ms_fast = prefs.getFloat("fastMs", DEFAULT_ROLLING_AVG_MS_FAST);
  rolling_avg_ms_slow = prefs.getFloat("slowMs", DEFAULT_ROLLING_AVG_MS_SLOW);
  base_playback_gain = prefs.getFloat("gain", DEFAULT_BASE_GAIN);
  playback_gain_min = prefs.getFloat("gainMin", DEFAULT_PLAYBACK_GAIN_MIN);
  playback_gain_max = prefs.getFloat("gainMax", DEFAULT_PLAYBACK_GAIN_MAX);
  gain_step_db_per_sec = prefs.getFloat("gainStep", DEFAULT_GAIN_STEP_DB_PER_SEC);
  gain_trim_db_limit = prefs.getFloat("gainTrimLim", DEFAULT_GAIN_TRIM_DB_LIMIT);
  auto_gain_enabled = prefs.getBool("autoGain", DEFAULT_AUTO_GAIN_ENABLED);
  min_on_ms = prefs.getULong("minOn", DEFAULT_MIN_ON_MS);
  min_off_ms = prefs.getULong("minOff", DEFAULT_MIN_OFF_MS);
  prefs.end();
  gain_trim_db = 0.0f;
  applyConfigBounds();
}

void saveConfigToNVS() {
  if (!prefs.begin("porous", false)) return;
  prefs.putBool("debug", debug_enabled);
  prefs.putFloat("targetMask", target_mask_db);
  prefs.putFloat("thr", threshold_db_spl);
  prefs.putFloat("hyst", hysteresis_db);
  prefs.putFloat("fastMs", rolling_avg_ms_fast);
  prefs.putFloat("slowMs", rolling_avg_ms_slow);
  prefs.putFloat("gain", base_playback_gain);
  prefs.putFloat("gainMin", playback_gain_min);
  prefs.putFloat("gainMax", playback_gain_max);
  prefs.putFloat("gainStep", gain_step_db_per_sec);
  prefs.putFloat("gainTrimLim", gain_trim_db_limit);
  prefs.putBool("autoGain", auto_gain_enabled);
  prefs.putULong("minOn", min_on_ms);
  prefs.putULong("minOff", min_off_ms);
  prefs.end();
}

bool setConfigField(const String& key, const String& value) {
  float f = value.toFloat();
  if (key == "target_mask_db") target_mask_db = f;
  else if (key == "threshold_db_spl") threshold_db_spl = f;
  else if (key == "hysteresis_db") hysteresis_db = f;
  else if (key == "rolling_avg_ms_fast") rolling_avg_ms_fast = f;
  else if (key == "rolling_avg_ms_slow") rolling_avg_ms_slow = f;
  else if (key == "gain") {
    base_playback_gain = f;
    gain_trim_db = 0.0f;
  }
  else if (key == "auto_gain") {
    auto_gain_enabled = (value.toInt() != 0);
    if (!auto_gain_enabled) gain_trim_db = 0.0f;
  }
  else if (key == "gain_min") playback_gain_min = f;
  else if (key == "gain_max") playback_gain_max = f;
  else if (key == "gain_step_db_per_sec") gain_step_db_per_sec = f;
  else if (key == "gain_trim_db_limit") gain_trim_db_limit = f;
  else if (key == "min_on_ms") min_on_ms = static_cast<uint32_t>(value.toInt());
  else if (key == "min_off_ms") min_off_ms = static_cast<uint32_t>(value.toInt());
  else if (key == "debug") debug_enabled = (value.toInt() != 0);
  else return false;
  applyConfigBounds();
  return true;
}

void handleCommand(const String& cmd_raw) {
  String cmd = cmd_raw;
  cmd.trim();
  if (cmd.length() == 0) return;

  if (cmd == "help") {
    Serial.println("help");
    Serial.println("show");
    Serial.println("save");
    Serial.println("load");
    Serial.println("set gain 0.30");
    Serial.println("set auto_gain 1");
    Serial.println("set target_mask_db -8");
    Serial.println("set threshold_db_spl 58");
    Serial.println("set hysteresis_db 3");
    Serial.println("set rolling_avg_ms_fast 200");
    Serial.println("set rolling_avg_ms_slow 5000");
    Serial.println("set gain_min 0.02");
    Serial.println("set gain_max 1.00");
    Serial.println("set gain_step_db_per_sec 2.0");
    Serial.println("set gain_trim_db_limit 6.0");
    Serial.println("set min_on_ms 3000");
    Serial.println("set min_off_ms 3000");
    Serial.println("set debug 1");
    Serial.println("cal_room 20");
    Serial.println("cal_active 20");
    Serial.println("Note: gain is linear (0.30), not +dB.");
    return;
  }
  if (cmd == "show") {
    printConfig();
    return;
  }
  if (cmd == "save") {
    saveConfigToNVS();
    Serial.println("Saved config to NVS");
    return;
  }
  if (cmd == "load") {
    loadConfigFromNVS();
    Serial.println("Loaded config from NVS");
    printConfig();
    return;
  }

  if (cmd.startsWith("cal_room")) {
    int sp = cmd.indexOf(' ');
    uint32_t seconds = (sp > 0) ? static_cast<uint32_t>(cmd.substring(sp + 1).toInt()) : 20;
    if (seconds < 5) seconds = 5;
    if (seconds > 120) seconds = 120;
    cal_mode = CalMode::ROOM;
    cal_end_ms = millis() + seconds * 1000UL;
    cal_sum_db = 0.0;
    cal_count = 0;
    Serial.printf("Calibration started: ROOM for %lu s\n", static_cast<unsigned long>(seconds));
    return;
  }

  if (cmd.startsWith("cal_active")) {
    int sp = cmd.indexOf(' ');
    uint32_t seconds = (sp > 0) ? static_cast<uint32_t>(cmd.substring(sp + 1).toInt()) : 20;
    if (seconds < 5) seconds = 5;
    if (seconds > 120) seconds = 120;
    cal_mode = CalMode::ACTIVE;
    cal_end_ms = millis() + seconds * 1000UL;
    cal_sum_db = 0.0;
    cal_count = 0;
    Serial.printf("Calibration started: ACTIVE for %lu s\n", static_cast<unsigned long>(seconds));
    return;
  }

  if (cmd.startsWith("set ")) {
    int sp1 = cmd.indexOf(' ');
    int sp2 = cmd.indexOf(' ', sp1 + 1);
    if (sp2 <= sp1 + 1) {
      Serial.println("Usage: set <key> <value>");
      return;
    }
    String key = cmd.substring(sp1 + 1, sp2);
    String value = cmd.substring(sp2 + 1);
    key.trim();
    value.trim();
    if (!setConfigField(key, value)) {
      Serial.println("Unknown key");
      return;
    }
    Serial.print("Set ");
    Serial.print(key);
    Serial.print(" = ");
    Serial.println(value);
    return;
  }

  Serial.println("Unknown command; type 'help'");
}

void processSerialCommands() {
  while (Serial.available() > 0) {
    char c = static_cast<char>(Serial.read());
    if (c == '\r') continue;
    if (c == '\n') {
      handleCommand(serial_line);
      serial_line = "";
    } else {
      serial_line += c;
      if (serial_line.length() > 160) serial_line = "";
    }
  }
}

void fillPlaybackFrames(int32_t* tx_slots, size_t frame_count) {
  const int16_t* samples = reinterpret_cast<const int16_t*>(branches_loop_pcm);
  const size_t sample_count = branches_loop_pcm_len / sizeof(int16_t);
  const float fade_in_step = (FADE_IN_MS > 0.0f) ? (1000.0f / (FADE_IN_MS * static_cast<float>(SAMPLE_RATE))) : 1.0f;
  const float fade_out_step = (FADE_OUT_MS > 0.0f) ? (1000.0f / (FADE_OUT_MS * static_cast<float>(SAMPLE_RATE))) : 1.0f;

  for (size_t i = 0; i < frame_count; ++i) {
    // Per-sample envelope for smooth transition edges.
    if (playback_active) {
      playback_envelope += fade_in_step;
      if (playback_envelope > 1.0f) playback_envelope = 1.0f;
    } else {
      playback_envelope -= fade_out_step;
      if (playback_envelope < 0.0f) playback_envelope = 0.0f;
    }

    int32_t s32 = 0;
    bool should_emit = (playback_envelope > 0.0f) && (sample_count > 0);

    if (should_emit) {
      int16_t s16 = samples[audio_sample_index];
      float total_gain = playback_gain_effective * playback_envelope;
      int32_t scaled = static_cast<int32_t>(lroundf(static_cast<float>(s16) * total_gain));
      if (scaled > 32767) scaled = 32767;
      if (scaled < -32768) scaled = -32768;
      s32 = static_cast<int32_t>(static_cast<int16_t>(scaled)) << 16;  // 16-bit value into 32-bit I2S slot MSBs

      // Advance source while audible so fades are natural and avoid edge clicks.
      upsample_phase++;
      if (upsample_phase >= UPSAMPLE_FACTOR) {
        upsample_phase = 0;
        audio_sample_index++;
        if (audio_sample_index >= sample_count) audio_sample_index = 0;
      }
    }

    tx_slots[i * 2] = s32;      // Left slot
    tx_slots[i * 2 + 1] = s32;  // Right slot mirrors mono content for the stereo DAC path.
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
  loadConfigFromNVS();

  setupI2S();

  if (debug_enabled) {
    Serial.println("Porous started");
    Serial.printf(
        "threshold=%.1f dBSPL, hysteresis=%.1f dB, mask=%.1f dB, fast=%.0f ms, slow=%.0f ms, minOn=%lu ms, minOff=%lu ms, gain=%.2f\n",
        threshold_db_spl, hysteresis_db, target_mask_db, rolling_avg_ms_fast, rolling_avg_ms_slow,
        static_cast<unsigned long>(min_on_ms), static_cast<unsigned long>(min_off_ms), base_playback_gain);
    Serial.println("Type 'help' for runtime commands");
  }
  last_state_change_ms = millis() - min_off_ms;
}

void loop() {
  static int32_t mic_slots[MIC_FRAME_WINDOW * 2];
  static int32_t tx_slots[MIC_FRAME_WINDOW * 2];
  static uint32_t last_print_ms = 0;
  processSerialCommands();

  size_t bytes_read = 0;
  esp_err_t err = i2s_read(I2S_PORT, mic_slots, sizeof(mic_slots), &bytes_read, portMAX_DELAY);
  if (err != ESP_OK || bytes_read == 0) {
    if (debug_enabled) Serial.println("i2s_read failed");
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
  float tau_fast_s = rolling_avg_ms_fast / 1000.0f;
  float tau_slow_s = rolling_avg_ms_slow / 1000.0f;
  float alpha_fast = (tau_fast_s > 0.0f) ? (1.0f - expf(-dt_s / tau_fast_s)) : 1.0f;
  float alpha_slow = (tau_slow_s > 0.0f) ? (1.0f - expf(-dt_s / tau_slow_s)) : 1.0f;
  if (alpha_fast < 0.0f) alpha_fast = 0.0f;
  if (alpha_fast > 1.0f) alpha_fast = 1.0f;
  if (alpha_slow < 0.0f) alpha_slow = 0.0f;
  if (alpha_slow > 1.0f) alpha_slow = 1.0f;

  if (!smooth_initialized) {
    fast_rms = rms;
    ambient_rms = rms;
    smooth_initialized = true;
  } else {
    fast_rms += alpha_fast * (rms - fast_rms);
    if (!playback_active) {
      // Ambient baseline tracks only when playback is off.
      ambient_rms += alpha_slow * (rms - ambient_rms);
    }
  }

  float fast_dbfs = (fast_rms > 0.000001f) ? 20.0f * log10f(fast_rms) : -120.0f;
  float ambient_dbfs = (ambient_rms > 0.000001f) ? 20.0f * log10f(ambient_rms) : -120.0f;
  float spl_fast = fast_dbfs + SPL_OFFSET_DB + SPL_CALIBRATION_DB;
  float spl_ambient = ambient_dbfs + SPL_OFFSET_DB + SPL_CALIBRATION_DB;

  uint32_t now = millis();
  uint32_t state_age_ms = now - last_state_change_ms;
  bool in_calibration = (cal_mode != CalMode::NONE);
  bool prev_state = playback_active;
  if (in_calibration) {
    playback_active = false;
  } else {
    if (!playback_active && state_age_ms >= min_off_ms && spl_fast >= threshold_db_spl) {
      playback_active = true;
    } else if (playback_active && state_age_ms >= min_on_ms && spl_fast <= (threshold_db_spl - hysteresis_db)) {
      playback_active = false;
    }
  }

  if (playback_active != prev_state) {
    last_state_change_ms = now;
  }

  // Estimate playback contribution and auto-adjust gain toward target masking.
  float spl_play_est = -120.0f;
  bool has_play_est = false;
  if (playback_active) {
    float p_tot = powf(10.0f, spl_fast / 10.0f);
    float p_amb = powf(10.0f, spl_ambient / 10.0f);
    float p_play = p_tot - p_amb;
    if (p_play > 1.0e-6f) {
      spl_play_est = 10.0f * log10f(p_play);
      has_play_est = true;
    }
  }

  if (playback_active && has_play_est && auto_gain_enabled) {
    float target_play_spl = spl_ambient + target_mask_db;
    float err_db = target_play_spl - spl_play_est;
    float max_delta_db = gain_step_db_per_sec * dt_s;
    float delta_db = clampf(err_db, -max_delta_db, max_delta_db);
    gain_trim_db += delta_db;
    gain_trim_db = clampf(gain_trim_db, -gain_trim_db_limit, gain_trim_db_limit);
  }
  if (!auto_gain_enabled) gain_trim_db = 0.0f;
  playback_gain_effective = base_playback_gain * powf(10.0f, gain_trim_db / 20.0f);
  playback_gain_effective = clampf(playback_gain_effective, playback_gain_min, playback_gain_max);

  if (in_calibration) {
    cal_sum_db += static_cast<double>(spl_fast);
    cal_count++;
    if (now >= cal_end_ms) {
      float avg_db = (cal_count > 0) ? static_cast<float>(cal_sum_db / static_cast<double>(cal_count)) : spl_fast;
      if (cal_mode == CalMode::ROOM) {
        cal_room_db = avg_db;
        has_cal_room = true;
        Serial.printf("Calibration complete: ROOM avg=%.2f dBSPL\n", cal_room_db);
      } else {
        cal_active_db = avg_db;
        has_cal_active = true;
        Serial.printf("Calibration complete: ACTIVE avg=%.2f dBSPL\n", cal_active_db);
      }
      cal_mode = CalMode::NONE;
      cal_sum_db = 0.0;
      cal_count = 0;

      if (has_cal_room && has_cal_active) {
        float delta = cal_active_db - cal_room_db;
        if (delta < 0.0f) delta = 0.0f;
        threshold_db_spl = cal_room_db + (0.50f * delta);
        hysteresis_db = clampf(0.20f * delta, 2.0f, 8.0f);
        applyConfigBounds();
        Serial.printf("Applied suggestion: threshold=%.2f hysteresis=%.2f (delta=%.2f)\n",
                      threshold_db_spl, hysteresis_db, delta);
        Serial.println("Use 'save' to persist these values.");
      }
    }
  }

  fillPlaybackFrames(tx_slots, frame_count);
  size_t bytes_written = 0;
  i2s_write(I2S_PORT, tx_slots, frame_count * 2 * sizeof(int32_t), &bytes_written, portMAX_DELAY);

  if (debug_enabled && now - last_print_ms >= TELEMETRY_INTERVAL_MS) {
    last_print_ms = now;
    Serial.print("UNPACK=");
    Serial.print(use_lj ? "LJ" : "RJ");
    Serial.print(" lsbNZ=");
    Serial.print(unpack_detect.low_byte_nonzero);
    Serial.print("/");
    Serial.print(unpack_detect.nonzero_slots);
    Serial.print(" dBSPL(inst)=");
    Serial.print(spl_inst, 1);
    Serial.print(" dBSPL(fast)=");
    Serial.print(spl_fast, 1);
    Serial.print(" dBSPL(ambient)=");
    Serial.print(spl_ambient, 1);
    Serial.print(" dBSPL(playEst)=");
    if (has_play_est) {
      Serial.print(spl_play_est, 1);
    } else {
      Serial.print("N/A");
    }
    Serial.print(" gain=");
    Serial.print(base_playback_gain, 3);
    Serial.print(" trimDb=");
    Serial.print(gain_trim_db, 2);
    Serial.print(" eff=");
    Serial.print(playback_gain_effective, 3);
    Serial.print(" env=");
    Serial.print(playback_envelope, 3);
    Serial.print(" ageMs=");
    Serial.print(state_age_ms);
    if (in_calibration) {
      Serial.print(" cal=");
      Serial.print(cal_mode == CalMode::ROOM ? "ROOM" : "ACTIVE");
      Serial.print(" remMs=");
      Serial.print((cal_end_ms > now) ? (cal_end_ms - now) : 0);
    }
    Serial.print(" state=");
    Serial.println(playback_active ? "PLAYING" : "IDLE");
  }
}
