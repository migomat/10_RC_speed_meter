// X4F Launch Logger – faza 1 (tylko pomiar). Urządzenie wyłącznie
// podsłuchuje sygnały halla i gazu; nie ma żadnego wyjścia do ESC.
#include <Arduino.h>
#include <math.h>

#include "ble_link.h"
#include "config.h"
#include "estimator.h"
#include "flash_store.h"
#include "hall.h"
#include "imu.h"
#include "led.h"
#include "logger.h"
#include "run_format.h"
#include "sim.h"
#include "throttle.h"
#include "timebase.h"

// ------------------------------------------------------------ takty 500 Hz
// ISR taktu zapisuje stan halla i gazu; pętla główna dokłada IMU.
struct Tick {
  uint32_t t_us, edges, period;
  uint16_t throttle;
  uint8_t  flags;
};
#define TICK_RING 256   // 512 ms zapasu na zablokowaną pętlę
static Tick s_ticks[TICK_RING];
static volatile uint32_t s_tick_w, s_tick_r;
static volatile uint32_t s_tick_overflows;

static void on_tick(uint32_t t) {
  HallSnapshot h;
  hall_tick_snapshot(t, &h);
  uint16_t thr = throttle_tick_width(t);
  if (s_tick_w - s_tick_r >= TICK_RING) {
    s_tick_overflows++;
    return;
  }
  Tick& k = s_ticks[s_tick_w % TICK_RING];
  k.t_us = t;
  k.edges = h.edges;
  k.period = h.period_rev_us;
  k.throttle = thr;
  k.flags = h.flags | (thr ? SF_THROTTLE_VALID : 0);
  s_tick_w = s_tick_w + 1;
}

// ------------------------------------------------------------ estymator
static EstParams s_ep;
static EstState  s_es;
static uint32_t  s_prev_t, s_prev_edges;
static bool      s_have_prev;
static bool      s_imu_ok, s_flash_ok, s_ble_ok, s_hall_ok, s_thr_ok;

static inline float axis_val(const float v[3], int idx, int sign) { return v[idx] * sign; }

static float v_front_from_period(uint32_t period_us) {
  if (!period_us) return 0.0f;
  float motor_rps = 1.0e6f / (float)period_us;
  return motor_rps / FINAL_DRIVE_RATIO * (float)M_PI * FRONT_TIRE_DIAMETER_MM * 1e-3f;
}

static void process_sample(const Sample& s) {
  float a[3] = {s.ax * imu_accel_scale_g(), s.ay * imu_accel_scale_g(), s.az * imu_accel_scale_g()};
  float g[3] = {s.gx * imu_gyro_scale_dps(), s.gy * imu_gyro_scale_dps(), s.gz * imu_gyro_scale_dps()};
  float pct = throttle_pct(s.throttle_us);
  float vf = v_front_from_period(s.period_rev_us);
  if (s_have_prev) {
    EstInput in;
    in.dt_s = (float)(uint32_t)(s.t_us - s_prev_t) * 1e-6f;
    in.a_fwd_g = axis_val(a, IMU_AXIS_FORWARD, IMU_SIGN_FORWARD);
    in.a_left_g = axis_val(a, IMU_AXIS_LEFT, IMU_SIGN_LEFT);
    in.a_up_g = axis_val(a, IMU_AXIS_UP, IMU_SIGN_UP);
    in.g_fwd_dps = axis_val(g, IMU_AXIS_FORWARD, IMU_SIGN_FORWARD);
    in.g_left_dps = axis_val(g, IMU_AXIS_LEFT, IMU_SIGN_LEFT);
    in.g_up_dps = axis_val(g, IMU_AXIS_UP, IMU_SIGN_UP);
    in.v_front_mps = vf;
    in.hall_stopped = (s.period_rev_us == 0 && s.hall_edges == s_prev_edges);
    in.throttle_pct = pct;
    est_step(&s_es, &s_ep, &in);
  }
  s_prev_t = s.t_us;
  s_prev_edges = s.hall_edges;
  s_have_prev = true;

  g_live.v = s_es.v;
  g_live.v_max = s_es.v_max;
  g_live.v_front = vf;
  g_live.spinning = s_es.spinning;
  g_live.rpm = s.period_rev_us ? 60.0e6f / s.period_rev_us : 0.0f;
  g_live.throttle_us = s.throttle_us;
  g_live.throttle_pct = pct;
  g_live.ax_g = axis_val(a, IMU_AXIS_FORWARD, IMU_SIGN_FORWARD);
  g_live.zupt = s_es.zupt;

  logger_push(s, pct, s.throttle_us != 0);
}

static void drain_ticks(uint32_t now) {
  while (s_tick_r != s_tick_w) {
    const Tick& k = s_ticks[s_tick_r % TICK_RING];
    // Czekamy, aż IMU będzie miało próbkę z chwili taktu (FIFO odczytywane
    // z opóźnieniem do 1 okresu ODR).
    if ((int32_t)(now - k.t_us) < (int32_t)SAMPLE_PERIOD_US) break;
    Sample s;
    s.t_us = k.t_us;
    s.hall_edges = k.edges;
    s.period_rev_us = k.period;
    s.throttle_us = k.throttle;
    s.flags = k.flags;
    s.reserved = 0;
    ImuRaw r;
    bool fresh = imu_get_at(k.t_us, &r);
    if (s_imu_ok) {
      s.ax = r.ax; s.ay = r.ay; s.az = r.az;
      s.gx = r.gx; s.gy = r.gy; s.gz = r.gz;
      if (!fresh) s.flags |= SF_IMU_STALE;
      if (imu_take_overrun_flag()) s.flags |= SF_IMU_OVERRUN;
    } else {
      s.ax = s.ay = s.az = s.gx = s.gy = s.gz = 0;
      s.flags |= SF_IMU_STALE;
    }
    s_tick_r = s_tick_r + 1;
    process_sample(s);
  }
}

void setup() {
  Serial.begin(115200);
  led_begin();
  s_ble_ok = ble_begin();          // SoftDevice musi działać przed sd_ppi_*
  timebase_begin(on_tick);
  timebase_request_hfxo();
  s_hall_ok = hall_begin();
  s_thr_ok = throttle_begin();
  s_imu_ok = imu_begin();
  s_flash_ok = flash_begin();
  logger_begin();
  if (!s_flash_ok) logger_set_error();
  est_default_params(&s_ep);
  est_init(&s_es);
}

void loop() {
  uint32_t now = timebase_now();
  imu_poll(now);
  sim_poll(now);
  now = timebase_now();
  drain_ticks(now);
  throttle_poll(now);
  logger_poll();
  if (logger_state() != LS_SAVING) flash_poll();
  if (vmax_reset_requested()) s_es.v_max = 0.0f;

  uint32_t ms = millis();
  links_poll(ms);

  LedInputs li;
  li.recording = logger_state() == LS_RECORDING;
  li.saving = logger_state() == LS_SAVING;
  li.error = logger_state() == LS_ERROR || !s_imu_ok || !s_flash_ok || !s_hall_ok || !s_thr_ok;
  li.no_throttle = !throttle_signal_ok(now);
  li.learning = !throttle_neutral_learned();
  li.connected = ble_connected();
  li.unread_runs = flash_count_runs() > 0 || logger_ram_run_available();
  led_poll(li, ms);
}
