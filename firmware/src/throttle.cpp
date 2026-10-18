#include "throttle.h"

#include <Arduino.h>
#include "config.h"
#include "timebase.h"

static volatile uint32_t s_rise_t;
static volatile bool     s_have_rise;
static volatile uint16_t s_width;        // ostatni poprawny impuls [us]
static volatile uint32_t s_valid_t;      // czas jego zakończenia
static volatile bool     s_any_valid;
static volatile uint32_t s_pulses, s_rejects;

static uint16_t s_neutral;
static bool     s_learned;
// okno uczenia
static uint32_t s_win_start_t, s_win_sum, s_win_n, s_win_last_pulse;
static uint16_t s_win_min, s_win_max;

static void on_level(uint32_t t, uint8_t level) {
  if (level) {
    s_rise_t = t;
    s_have_rise = true;
    return;
  }
  if (!s_have_rise) return;
  s_have_rise = false;
  uint32_t w = t - s_rise_t;
  if (w < THROTTLE_MIN_US || w > THROTTLE_MAX_US) {
    s_rejects++;
    return;
  }
  s_width = (uint16_t)w;
  s_valid_t = t;
  s_any_valid = true;
  s_pulses++;
}

static void isr_throttle(void) {
  uint32_t t = timebase_captured(TB_CC_THROTTLE);
  on_level(t, nrf_gpio_pin_read(g_ADigitalPinMap[PIN_THROTTLE]) ? 1 : 0);
}

bool throttle_begin(void) {
  pinMode(PIN_THROTTLE, INPUT);
  attachInterrupt(PIN_THROTTLE, isr_throttle, CHANGE);
  return timebase_connect_capture(PIN_THROTTLE, TB_CC_THROTTLE, 3);
}

uint16_t throttle_tick_width(uint32_t t_tick) {
  if (!s_any_valid) return 0;
  if ((int32_t)(t_tick - s_valid_t) > (int32_t)THROTTLE_TIMEOUT_MS * 1000) return 0;
  return s_width;
}

bool throttle_signal_ok(uint32_t now_us) { return throttle_tick_width(now_us) != 0; }
uint16_t throttle_last_width(void) { return s_width; }

void throttle_poll(uint32_t now_us) {
  if (s_learned) return;
  noInterrupts();
  uint32_t pulses = s_pulses;
  uint16_t w = s_width;
  interrupts();
  if (!throttle_signal_ok(now_us)) {
    s_win_n = 0;
    return;
  }
  if (pulses == s_win_last_pulse) return;
  s_win_last_pulse = pulses;
  if (s_win_n == 0) {
    s_win_start_t = now_us;
    s_win_sum = 0;
    s_win_min = 0xFFFF;
    s_win_max = 0;
  }
  s_win_sum += w;
  s_win_n++;
  if (w < s_win_min) s_win_min = w;
  if (w > s_win_max) s_win_max = w;
  if (s_win_max - s_win_min >= THROTTLE_NEUTRAL_SPREAD_US) {
    s_win_n = 0;  // ruch drążka – zaczynamy od nowa
    return;
  }
  if ((uint32_t)(now_us - s_win_start_t) >= (uint32_t)THROTTLE_NEUTRAL_LEARN_MS * 1000) {
    s_neutral = (uint16_t)((s_win_sum + s_win_n / 2) / s_win_n);
    s_learned = true;
  }
}

bool throttle_neutral_learned(void) { return s_learned; }
uint16_t throttle_neutral_us(void) { return s_neutral; }

float throttle_pct(uint16_t width_us) {
  if (!width_us || !s_learned) return 0.0f;
  float d = (float)((int32_t)width_us - (int32_t)s_neutral);
#if !THROTTLE_FORWARD_IS_HIGHER
  d = -d;
#endif
  return d * 100.0f / THROTTLE_FULL_SCALE_US;
}

void throttle_inject_pulse(uint32_t t_rise_us, uint16_t width_us) {
  noInterrupts();
  on_level(t_rise_us, 1);
  on_level(t_rise_us + width_us, 0);
  interrupts();
}

uint32_t throttle_pulse_count(void) { return s_pulses; }
uint32_t throttle_reject_count(void) { return s_rejects; }
