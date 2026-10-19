#include "hall.h"

#include <Arduino.h>
#include "config.h"
#include "timebase.h"

#define RING_LEN (EDGES_PER_REV + 1)

static volatile uint32_t s_ring_t[RING_LEN];
static volatile uint32_t s_count;        // przyjęte zbocza (narastająco)
static volatile uint32_t s_since_stop;   // zbocza od ostatniego postoju
static volatile uint32_t s_period;       // okres obrotu [us], 0 = nieznany
static volatile uint32_t s_last_edge_t;
static volatile uint32_t s_glitches;
static uint32_t s_line_last_t[3];
static uint8_t  s_line_level[3];
static uint8_t  s_dir_flags;
static uint8_t  s_hall_state;            // bity A|B<<1|C<<2 (tryb 3)

// Cofnięcie ostatniego zbocza – impuls krótszy niż HALL_MIN_EDGE_US
// to para zboczy, oba trzeba odrzucić.
static struct {
  bool     valid;
  uint8_t  line;
  uint32_t since_stop, period, last_edge_t, line_last_t;
  uint8_t  dir_flags, hall_state;
} s_undo;

#if LOG_RAW_EDGES
static EdgeWord s_log[EDGE_LOG_LEN];
#endif
static volatile bool     s_protect;
static volatile uint32_t s_protect_start;
static volatile bool     s_truncated;

static const int8_t kSeqIndex[8] = {-1, 0, 2, 1, 4, 5, 3, -1};  // 1,3,2,6,4,5

static void process_edge(uint32_t t, uint8_t line, uint8_t level) {
  if (level == s_line_level[line]) {
    // Dwa zbocza zlały się w jedno zdarzenie GPIOTE – krótka szpilka.
    s_glitches++;
    return;
  }
  if ((uint32_t)(t - s_line_last_t[line]) < HALL_MIN_EDGE_US) {
    s_glitches++;
    s_line_level[line] = level;
    if (s_undo.valid && s_undo.line == line) {
      s_count--;
      s_since_stop = s_undo.since_stop;
      s_period = s_undo.period;
      s_last_edge_t = s_undo.last_edge_t;
      s_line_last_t[line] = s_undo.line_last_t;
      s_dir_flags = s_undo.dir_flags;
      s_hall_state = s_undo.hall_state;
      s_undo.valid = false;
    }
    return;
  }

  s_undo.valid = true;
  s_undo.line = line;
  s_undo.since_stop = s_since_stop;
  s_undo.period = s_period;
  s_undo.last_edge_t = s_last_edge_t;
  s_undo.line_last_t = s_line_last_t[line];
  s_undo.dir_flags = s_dir_flags;
  s_undo.hall_state = s_hall_state;

  s_line_level[line] = level;
  s_line_last_t[line] = t;
  s_last_edge_t = t;

  uint32_t n = s_count;
  s_ring_t[n % RING_LEN] = t;
#if LOG_RAW_EDGES
  if (s_protect && (uint32_t)(n - s_protect_start) >= EDGE_LOG_LEN) {
    s_truncated = true;
  } else {
    s_log[n % EDGE_LOG_LEN] = (t & EDGE_T_MASK) | ((uint32_t)level << EDGE_LEVEL_BIT) |
                              ((uint32_t)line << EDGE_LINE_SHIFT);
  }
#endif
  s_count = n + 1;
  uint32_t since = s_since_stop + 1;
  s_since_stop = since;
  if (since > EDGES_PER_REV) {
    s_period = t - s_ring_t[(n - EDGES_PER_REV) % RING_LEN];
  } else {
    s_period = 0;
  }

#if HALL_LINES == 3
  uint8_t st = s_line_level[0] | (s_line_level[1] << 1) | (s_line_level[2] << 2);
  int8_t a = kSeqIndex[s_hall_state], b = kSeqIndex[st];
  s_dir_flags = 0;
  if (a >= 0 && b >= 0) {
    int8_t d = (b - a + 6) % 6;
    if (d == 1) s_dir_flags = SF_DIR_VALID | SF_DIR_FORWARD;
    else if (d == 5) s_dir_flags = SF_DIR_VALID;
  }
  s_hall_state = st;
#endif
}

static inline void edge_isr(uint8_t line, uint8_t cc, uint32_t arduino_pin) {
  uint32_t t = timebase_captured(cc);
  uint8_t level = nrf_gpio_pin_read(g_ADigitalPinMap[arduino_pin]) ? 1 : 0;
  process_edge(t, line, level);
}

static void isr_a(void) { edge_isr(0, TB_CC_HALL_A, PIN_HALL_A); }
#if HALL_LINES == 3
static void isr_b(void) { edge_isr(1, TB_CC_HALL_B, PIN_HALL_B); }
static void isr_c(void) { edge_isr(2, TB_CC_HALL_C, PIN_HALL_C); }
#endif

bool hall_begin(void) {
  uint32_t now = timebase_now();
  const uint32_t pins[3] = {PIN_HALL_A, PIN_HALL_B, PIN_HALL_C};
  for (int i = 0; i < HALL_LINES; i++) {
    pinMode(pins[i], INPUT);  // podciąganie robi ESC (otwarty kolektor)
    s_line_level[i] = digitalRead(pins[i]) ? 1 : 0;
    s_line_last_t[i] = now - 1000000;
  }
  s_hall_state = s_line_level[0] | (s_line_level[1] << 1) | (s_line_level[2] << 2);
  bool ok = true;
  attachInterrupt(PIN_HALL_A, isr_a, CHANGE);
  ok &= timebase_connect_capture(PIN_HALL_A, TB_CC_HALL_A, 0);
#if HALL_LINES == 3
  attachInterrupt(PIN_HALL_B, isr_b, CHANGE);
  ok &= timebase_connect_capture(PIN_HALL_B, TB_CC_HALL_B, 1);
  attachInterrupt(PIN_HALL_C, isr_c, CHANGE);
  ok &= timebase_connect_capture(PIN_HALL_C, TB_CC_HALL_C, 2);
#endif
  return ok;
}

void hall_tick_snapshot(uint32_t t_tick, HallSnapshot* out) {
  if (s_since_stop && (int32_t)(t_tick - s_last_edge_t) > HALL_TIMEOUT_US) {
    s_since_stop = 0;
    s_period = 0;
    s_dir_flags = 0;
    s_undo.valid = false;
  }
  out->edges = s_count;
  out->period_rev_us = s_period;
  out->flags = (s_period ? SF_HALL_VALID : 0) | s_dir_flags;
}

void hall_inject_edge(uint32_t t_us, uint8_t line, uint8_t level) {
  if (line >= HALL_LINES) return;
  noInterrupts();
  process_edge(t_us, line, level);
  interrupts();
}

uint8_t hall_line_level(uint8_t line) { return line < 3 ? s_line_level[line] : 0; }

uint8_t hall_pin_levels(void) {
  uint8_t m = digitalRead(PIN_HALL_A) ? 1 : 0;
#if HALL_LINES == 3
  if (digitalRead(PIN_HALL_B)) m |= 2;
  if (digitalRead(PIN_HALL_C)) m |= 4;
#endif
  return m;
}

void hall_force_levels(uint8_t mask) {
  noInterrupts();
  for (int i = 0; i < 3; i++) s_line_level[i] = (mask >> i) & 1;
  s_hall_state = mask & 7;
  interrupts();
}

uint32_t hall_edge_count(void) { return s_count; }
uint32_t hall_glitch_count(void) { return s_glitches; }

void hall_log_protect(bool on, uint32_t start_count) {
  noInterrupts();
  s_protect_start = start_count;
  s_protect = on;
  if (on) s_truncated = false;
  interrupts();
}

bool hall_log_truncated(void) { return s_truncated; }

EdgeWord hall_log_word(uint32_t idx, uint32_t t0_us) {
#if LOG_RAW_EDGES
  EdgeWord w = s_log[idx % EDGE_LOG_LEN];
  return ((w - t0_us) & EDGE_T_MASK) | (w & ~EDGE_T_MASK);
#else
  (void)idx; (void)t0_us;
  return 0;
#endif
}
