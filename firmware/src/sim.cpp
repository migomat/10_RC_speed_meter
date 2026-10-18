#include "sim.h"

#include <Arduino.h>
#include "config.h"
#include "hall.h"
#include "throttle.h"
#include "timebase.h"

#if HALL_LINES == 3
// Przejścia stanów 1→3→2→6→4→5→1: (linia, poziom po zboczu)
static const uint8_t kSeq[6][2] = {{1, 1}, {0, 0}, {2, 1}, {1, 0}, {0, 1}, {2, 0}};
#define SEQ_LEN 6
#else
static const uint8_t kSeq[2][2] = {{0, 1}, {0, 0}};
#define SEQ_LEN 2
#endif

static bool     s_active;
static bool     s_pins_pulled;
static float    s_rpm, s_rpm_from, s_rpm_to;
static uint32_t s_ramp_t0, s_ramp_us;
static bool     s_ramping;
static uint32_t s_next_edge_t, s_last_edge_t;
static uint8_t  s_seq_k;
static bool     s_edges_on;
static uint16_t s_thr_w;
static uint32_t s_next_pulse_t;

static void ensure_active(uint32_t now) {
  if (!s_pins_pulled) {
    // Na biurku piny wiszą w powietrzu – podciągamy do masy, żeby szum nie
    // dodawał zboczy. (Na aucie SIM nie jest używany.)
    pinMode(PIN_HALL_A, INPUT_PULLDOWN);
#if HALL_LINES == 3
    pinMode(PIN_HALL_B, INPUT_PULLDOWN);
    pinMode(PIN_HALL_C, INPUT_PULLDOWN);
#endif
    pinMode(PIN_THROTTLE, INPUT_PULLDOWN);
    s_pins_pulled = true;
#if HALL_LINES == 3
    hall_force_levels(0x1);  // stan 1 (A=1) – początek sekwencji kSeq
#else
    hall_force_levels(0x0);
#endif
    s_seq_k = 0;
  }
  if (!s_active) {
    s_active = true;
    s_next_pulse_t = now;
  }
}

static float rpm_at(uint32_t t) {
  if (!s_ramping) return s_rpm;
  uint32_t dt = t - s_ramp_t0;
  if (dt >= s_ramp_us) {
    s_ramping = false;
    s_rpm = s_rpm_to;
    return s_rpm;
  }
  return s_rpm_from + (s_rpm_to - s_rpm_from) * ((float)dt / (float)s_ramp_us);
}

void sim_poll(uint32_t now) {
  if (!s_active) return;
  // zbocza
  for (int guard = 0; guard < 64 && s_edges_on; guard++) {
    if ((int32_t)(now - s_next_edge_t) < 0) break;
    float r = rpm_at(s_next_edge_t);
    if (r < 1.0f) {
      if (!s_ramping) {
        s_edges_on = false;
        break;
      }
      s_next_edge_t += 1000;
      continue;
    }
    hall_inject_edge(s_next_edge_t, kSeq[s_seq_k][0], kSeq[s_seq_k][1]);
    s_last_edge_t = s_next_edge_t;
    s_seq_k = (s_seq_k + 1) % SEQ_LEN;
    s_next_edge_t += (uint32_t)(60.0e6f / (r * EDGES_PER_REV));
  }
  // gaz 50 Hz
  if (s_thr_w) {
    for (int guard = 0; guard < 8; guard++) {
      if ((int32_t)(now - (s_next_pulse_t + s_thr_w)) < 0) break;
      throttle_inject_pulse(s_next_pulse_t, s_thr_w);
      s_next_pulse_t += 20000;
    }
  }
}

static void start_edges(uint32_t now) {
  if (!s_edges_on) {
    s_edges_on = true;
    s_next_edge_t = now;
  }
}

void sim_set_rpm(float rpm) {
  uint32_t now = timebase_now();
  ensure_active(now);
  s_ramping = false;
  s_rpm = rpm;
  if (rpm >= 1.0f) start_edges(now);
}

void sim_ramp(float rpm_to, uint32_t ms) {
  uint32_t now = timebase_now();
  ensure_active(now);
  s_rpm_from = rpm_at(now);
  s_rpm_to = rpm_to;
  s_ramp_t0 = now;
  s_ramp_us = ms * 1000;
  s_ramping = true;
  start_edges(now);
}

void sim_set_throttle(uint16_t w) {
  uint32_t now = timebase_now();
  ensure_active(now);
  if (!s_thr_w) s_next_pulse_t = now;
  s_thr_w = w;
}

void sim_glitch(uint32_t width_us) {
  uint32_t now = timebase_now();
  ensure_active(now);
  sim_poll(now);
  // Szpilka w połowie między zboczami symulacji (zachowana kolejność czasów)
  uint32_t t = now;
  if (s_edges_on) t = s_last_edge_t + (s_next_edge_t - s_last_edge_t) / 2;
  uint8_t cur = hall_line_level(0);
  hall_inject_edge(t, 0, cur ^ 1);
  hall_inject_edge(t + width_us, 0, cur);
}

bool sim_active(void) { return s_active; }
float sim_rpm(void) { return s_active ? rpm_at(timebase_now()) : 0.0f; }

void sim_off(void) {
  s_edges_on = false;
  s_ramping = false;
  s_rpm = 0;
  s_thr_w = 0;
}
