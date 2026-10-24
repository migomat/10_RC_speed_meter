#include "logger.h"
#include <math.h>

#include <Arduino.h>
#include "config.h"
#include "crc32.h"
#include "flash_store.h"
#include "hall.h"
#include "imu.h"
#include "throttle.h"
#include "timebase.h"

static Sample   s_buf[MAX_SAMPLES];
static uint32_t s_w;            // następna pozycja zapisu
static uint32_t s_filled;       // ile próbek w buforze (≤ MAX_SAMPLES)
static LogState s_state = LS_IDLE;
static bool     s_armed;        // był postój (ARM_REST_MS) – wyzwolenie dozwolone
static uint32_t s_rest_n;       // próbek z rzędu: gaz w neutrum i brak zboczy
static uint32_t s_since_rest_n; // próbek od zejścia z postoju (uzbrojony)
static uint32_t s_prev_edges;
static bool     s_force;
static bool     s_sim;

// Bieżący start
static RunHeader s_hdr;
static uint32_t s_start_pos;    // pozycja pierwszej próbki w buforze
static uint32_t s_n_samples;
static uint32_t s_post_left;
static uint32_t s_edge_start, s_n_edges;
static bool     s_edges_trunc;
static uint32_t s_run_len;
static uint32_t s_run_crc;
static bool     s_ram_run;      // start dostępny tylko w RAM

// SAVING
static int      s_slot = -1;
static uint32_t s_save_off;
static uint32_t s_save_t0_ms;
static uint32_t s_last_save_ms, s_last_crc_ms;
static uint32_t s_runs;

// Czas
static bool     s_time_set;
static uint32_t s_unix_at_ms0;  // unix_s − millis()/1000 przy TIME

void logger_begin(void) {
  s_state = LS_IDLE;
  s_armed = false;
}

LogState logger_state(void) { return s_state; }

const char* logger_state_name(void) {
  switch (s_state) {
    case LS_IDLE: return "IDLE";
    case LS_RECORDING: return "RECORDING";
    case LS_SAVING: return "SAVING";
    default: return "ERROR";
  }
}

bool logger_armed(void) { return s_armed; }
bool logger_busy(void) { return s_state == LS_RECORDING || s_state == LS_SAVING; }
void logger_force_trigger(void) { s_force = true; }
void logger_set_sim_flag(bool on) { s_sim = on; }
void logger_set_error(void) { s_state = LS_ERROR; }

void logger_arm(void) {
  if (s_state == LS_ERROR) {
    s_state = LS_IDLE;
    s_armed = false;
  }
}

void logger_set_time(uint32_t unix_s) {
  s_unix_at_ms0 = unix_s - millis() / 1000;
  s_time_set = true;
}
bool logger_time_set(void) { return s_time_set; }
uint32_t logger_unix_now(void) { return s_time_set ? s_unix_at_ms0 + millis() / 1000 : 0; }

static void start_run(uint32_t trig_pos) {
  // trig_pos – pozycja próbki wyzwolenia (już zapisanej)
  uint32_t before = s_filled - 1;
  uint32_t pre = before < PRE_SAMPLES ? before : PRE_SAMPLES;
  s_start_pos = (trig_pos + MAX_SAMPLES - pre) % MAX_SAMPLES;
  s_n_samples = pre + POST_SAMPLES;
  s_post_left = POST_SAMPLES - 1;
  s_buf[trig_pos].flags |= SF_TRIGGER;
  s_ram_run = false;

  const Sample& first = s_buf[s_start_pos];
  const Sample& trig = s_buf[trig_pos];
  s_edge_start = first.hall_edges;
  s_edges_trunc = false;
  uint32_t now_edges = hall_edge_count();
  if (LOG_RAW_EDGES && now_edges - s_edge_start > EDGE_LOG_LEN) {
    s_edge_start = now_edges - EDGE_LOG_LEN;
    s_edges_trunc = true;
  }
  hall_log_protect(true, s_edge_start);

  memset(&s_hdr, 0, sizeof(s_hdr));
  s_hdr.magic = RUN_MAGIC;
  s_hdr.format_version = RUN_FORMAT_VERSION;
  s_hdr.header_size = RUN_HEADER_SIZE;
  s_hdr.seq = flash_next_seq();
  flash_note_seq(s_hdr.seq);  // zarezerwowany od razu – wyścig nie dostanie tego samego
  s_hdr.unix_s = logger_unix_now();
  s_hdr.boot_count = flash_boot_count();
  s_hdr.uptime_ms = millis();
  s_hdr.hall_lines = HALL_LINES;
  s_hdr.motor_pole_pairs = MOTOR_POLE_PAIRS;
  s_hdr.edges_per_rev = EDGES_PER_REV;
  s_hdr.throttle_fwd_higher = THROTTLE_FORWARD_IS_HIGHER;
  s_hdr.sample_rate_hz = SAMPLE_RATE_HZ;
  s_hdr.imu_odr_hz = IMU_ODR_HZ;
  s_hdr.accel_range_g = IMU_ACCEL_RANGE_G;
  s_hdr.gyro_range_dps = IMU_GYRO_RANGE_DPS;
  s_hdr.accel_scale_g = imu_accel_scale_g();
  s_hdr.gyro_scale_dps = imu_gyro_scale_dps();
  s_hdr.throttle_neutral_us = throttle_neutral_learned() ? throttle_neutral_us() : 0;
  s_hdr.throttle_full_us = THROTTLE_FULL_SCALE_US;
  s_hdr.trigger_index = pre;
  s_hdr.t0_us = first.t_us;
  s_hdr.trigger_t_us = trig.t_us;
  s_hdr.sample_size = sizeof(Sample);
  s_hdr.edge_size = sizeof(EdgeWord);
  s_hdr.fw_version = (FW_VERSION_MAJOR << 16) | (FW_VERSION_MINOR << 8) | FW_VERSION_PATCH;
  s_hdr.hall_min_edge_us = HALL_MIN_EDGE_US;
  s_hdr.pretrigger_ms = PRETRIGGER_MS;
  s_hdr.posttrigger_ms = POSTTRIGGER_MS;
  s_hdr.trigger_pct = TRIGGER_THROTTLE_PCT;
  s_hdr.hall_pullup_v_x10 = HALL_PULLUP_V_X10;
  s_hdr.hall_timeout_us = HALL_TIMEOUT_US;

  flash_prepare_for_run();  // ew. kasowanie najstarszego – w trakcie RECORDING
  s_state = LS_RECORDING;
}

static void finish_recording(uint32_t last_pos) {
  uint32_t edge_end = s_buf[last_pos].hall_edges;
  s_n_edges = 0;
#if LOG_RAW_EDGES
  s_n_edges = edge_end - s_edge_start;
  if (s_n_edges > EDGE_LOG_LEN) s_n_edges = EDGE_LOG_LEN;
  if (hall_log_truncated()) s_edges_trunc = true;
#endif
  s_hdr.n_samples = s_n_samples;
  s_hdr.n_edges = s_n_edges;
  s_hdr.flags = (s_edges_trunc ? RF_EDGES_TRUNCATED : 0) | (s_time_set ? RF_TIME_SET : 0) |
                (throttle_neutral_learned() ? RF_NEUTRAL_LEARNED : 0) |
                (imu_ok() ? 0 : RF_IMU_ERROR) | (s_sim ? RF_SIMULATED : 0);
  s_run_len = run_total_size(s_n_samples, s_n_edges);
  s_slot = -1;
  s_save_off = 0;
  s_save_t0_ms = millis();
  s_state = LS_SAVING;
}

void logger_push(const Sample& smp, float pct, bool throttle_valid) {
  if (s_state == LS_SAVING) return;  // bufor jest właśnie zapisywany
  if (s_state == LS_ERROR && s_ram_run) return;  // chronimy start w RAM do ARM

  uint32_t pos = s_w;
  s_buf[pos] = smp;
  s_w = (s_w + 1) % MAX_SAMPLES;
  if (s_filled < MAX_SAMPLES) s_filled++;

  if (s_state == LS_RECORDING) {
    if (s_post_left == 0) {
      finish_recording(pos);
    } else {
      s_post_left--;
      if (s_post_left == 0) finish_recording(pos);
    }
    return;
  }
  if (s_state != LS_IDLE) return;

  bool learned = throttle_neutral_learned();
  // Uzbrojenie po postoju; okno na przekroczenie progu liczone od zejścia z postoju.
  const uint32_t rest_need = ARM_REST_MS * SAMPLE_RATE_HZ / 1000;
  const uint32_t window = LAUNCH_WINDOW_MS * SAMPLE_RATE_HZ / 1000;
  bool rest = throttle_valid && learned && fabsf(pct) < ARM_NEUTRAL_PCT &&
              smp.hall_edges == s_prev_edges;
  s_prev_edges = smp.hall_edges;
  if (rest) {
    if (s_rest_n < rest_need) s_rest_n++;
    if (s_rest_n >= rest_need) {
      s_armed = true;
      s_since_rest_n = 0;
    }
  } else {
    s_rest_n = 0;
    if (s_armed && ++s_since_rest_n > window) s_armed = false;
  }
  bool trig = s_force || (s_armed && throttle_valid && learned && pct > TRIGGER_THROTTLE_PCT);
  if (trig) {
    s_force = false;
    s_armed = false;
    start_run(pos);
  }
}

// --- dostęp do bajtów bieżącego startu (nagłówek + próbki + zbocza)
static void read_run_bytes(uint32_t off, uint8_t* dst, uint32_t len) {
  const uint32_t samp_end = RUN_HEADER_SIZE + s_n_samples * sizeof(Sample);
  while (len) {
    uint32_t n;
    if (off < RUN_HEADER_SIZE) {
      n = min(len, RUN_HEADER_SIZE - off);
      memcpy(dst, (const uint8_t*)&s_hdr + off, n);
    } else if (off < samp_end) {
      uint32_t rel = off - RUN_HEADER_SIZE;
      uint32_t idx = rel / sizeof(Sample), b = rel % sizeof(Sample);
      n = min(len, (uint32_t)sizeof(Sample) - b);
      memcpy(dst, (const uint8_t*)&s_buf[(s_start_pos + idx) % MAX_SAMPLES] + b, n);
    } else {
      uint32_t rel = off - samp_end;
      uint32_t idx = rel / 4, b = rel % 4;
      EdgeWord w = hall_log_word(s_edge_start + idx, s_hdr.t0_us);
      n = min(len, 4 - b);
      memcpy(dst, (const uint8_t*)&w + b, n);
    }
    off += n;
    dst += n;
    len -= n;
  }
}

uint32_t run_crc_update_chunk(uint32_t crc, uint32_t offset, const uint8_t* data, uint32_t len) {
  // Pole crc32 nagłówka (bajty 124..127) liczone jako zera
  for (uint32_t i = 0; i < len; i++) {
    uint32_t o = offset + i;
    uint8_t b = (o >= 124 && o < 128) ? 0 : data[i];
    crc = crc32_update(crc, &b, 1);
  }
  return crc;
}

static uint32_t compute_run_crc(void) {
  static uint8_t buf[512] __attribute__((aligned(4)));
  uint32_t crc = crc32_begin();
  // nagłówek z wyzerowanym polem CRC
  memcpy(buf, &s_hdr, RUN_HEADER_SIZE);
  memset(buf + 124, 0, 4);
  crc = crc32_update(crc, buf, RUN_HEADER_SIZE);
  for (uint32_t off = RUN_HEADER_SIZE; off < s_run_len; off += sizeof(buf)) {
    uint32_t n = min((uint32_t)sizeof(buf), s_run_len - off);
    read_run_bytes(off, buf, n);
    crc = crc32_update(crc, buf, n);
  }
  return crc32_end(crc);
}

static void enter_ram_only(void) {
  s_hdr.flags |= RF_RAM_ONLY;
  s_hdr.crc32 = 0;
  s_run_crc = compute_run_crc();
  s_hdr.crc32 = s_run_crc;
  s_ram_run = true;
  hall_log_protect(true, s_edge_start);  // zbocza zostają chronione
  s_state = LS_ERROR;
}

void logger_poll(void) {
  if (s_state != LS_SAVING) return;
  if (s_slot < 0) {
    uint32_t t = millis();
    s_hdr.crc32 = 0;
    s_run_crc = compute_run_crc();
    s_hdr.crc32 = s_run_crc;
    s_last_crc_ms = millis() - t;
    s_slot = flash_write_begin(s_hdr.seq);
    if (s_slot < 0) {
      enter_ram_only();
      return;
    }
    return;
  }
  // jeden kawałek na przebieg pętli – IMU i takty obsługiwane pomiędzy
  static uint8_t chunk[SAVE_CHUNK_BYTES] __attribute__((aligned(4)));
  if (s_save_off < s_run_len) {
    uint32_t n = min((uint32_t)SAVE_CHUNK_BYTES, s_run_len - s_save_off);
    read_run_bytes(s_save_off, chunk, n);
    if (!flash_write(s_slot, s_save_off, chunk, n)) {
      enter_ram_only();
      return;
    }
    s_save_off += n;
    return;
  }
  bool ok = flash_write_finish(s_slot, s_run_len, s_run_crc, s_hdr.unix_s, s_hdr.boot_count,
                               s_hdr.uptime_ms);
  if (!ok) {
    enter_ram_only();
    return;
  }
  s_last_save_ms = millis() - s_save_t0_ms;
  s_runs++;
  hall_log_protect(false, 0);
  s_filled = 0;  // pre-trigger kolejnego startu zbierany od nowa
  s_state = LS_IDLE;
}

bool logger_ram_run_available(void) { return s_ram_run; }
uint32_t logger_ram_run_len(void) { return s_run_len; }
uint32_t logger_ram_run_crc(void) { return s_run_crc; }
const RunHeader* logger_ram_run_header(void) { return &s_hdr; }

bool logger_read_ram_run(uint32_t offset, void* buf, uint32_t len) {
  if (!s_ram_run || offset + len > s_run_len) return false;
  read_run_bytes(offset, (uint8_t*)buf, len);
  return true;
}

void logger_discard_ram_run(void) {
  if (!s_ram_run) return;
  s_ram_run = false;
  hall_log_protect(false, 0);
  s_filled = 0;
}

uint32_t logger_last_save_ms(void) { return s_last_save_ms; }
uint32_t logger_last_crc_ms(void) { return s_last_crc_ms; }
uint32_t logger_runs_recorded(void) { return s_runs; }
