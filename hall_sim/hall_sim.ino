// Symulator sygnałów halla i gazu dla X4F-LOG (ESP32, Arduino core 3.x).
// Wyjścia 3,3 V – podłączać do Xiao BEZ dzielnika 47k/68k.
//
//   GPIO25 → D1 (hall A)   GPIO26 → D2 (hall B)   GPIO27 → D3 (hall C)
//   GPIO32 → D0 (gaz)      GND → GND
//
// Komendy (Serial 115200, zakończone \n):
//   LINES 1|3            tryb halla (1 linia albo 3 linie przesunięte o 120° el.)
//   RPM <rpm>            stałe obroty (0 = stop)
//   RAMP <rpm> <ms>      liniowa rampa od bieżących obrotów
//   THR <us> [hz]        impuls gazu (np. THR 1500 50), THR 0 = brak sygnału
//   SPIKE <us>           pojedyncza szpilka na linii A o szerokości <us>
//   LAUNCH               neutrum 3 s, gaz 1900 us, po 25 ms rampa 0→30000 rpm w 1,5 s,
//                        po 3,5 s hamowanie do 0 i gaz 1500 us
//   STATUS
//
// Zbocza: jedno przerwanie timera na zbocze, w przerwaniu tylko liczby całkowite
// i zapis rejestrów GPIO. Okres (µs) liczony w pętli co 1 ms. Rozdzielczość 1 µs:
// błąd okresu ≤ 0,15 % przy 30 000 rpm w trybie 3 (333,3 µs → 333 µs).
#include <Arduino.h>
#include "soc/gpio_struct.h"

#define PIN_A 25
#define PIN_B 26
#define PIN_C 27
#define PIN_THR 32

static volatile int lines = 1;
static volatile uint8_t step_k = 0;
static hw_timer_t* tim = nullptr;
static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

// stany A|B<<1|C<<2: 1,3,2,6,4,5 (jak firmware/src/sim.cpp); tryb 1: A = 1,0
static const uint8_t SEQ3[6] = {1, 3, 2, 6, 4, 5};
static const uint8_t SEQ1[2] = {1, 0};

static double rpm = 0, rpm_from = 0, rpm_to = 0;
static uint32_t ramp_t0 = 0, ramp_ms = 0;
static bool ramping = false;
static uint32_t cur_interval = 0;   // µs, 0 = stop
static bool tim_running = false;
static uint32_t thr_hz = 50, thr_us = 1500;

static inline void IRAM_ATTR write_state(uint8_t st) {
  uint32_t set = 0, clr = 0;
  ((st & 1) ? set : clr) |= 1u << PIN_A;
  if (lines == 3) {
    ((st & 2) ? set : clr) |= 1u << PIN_B;
    ((st & 4) ? set : clr) |= 1u << PIN_C;
  }
  GPIO.out_w1ts = set;
  GPIO.out_w1tc = clr;
}

static void IRAM_ATTR on_timer() {
  portENTER_CRITICAL_ISR(&mux);
  uint8_t n = lines == 3 ? 6 : 2;
  uint8_t k = step_k + 1;
  if (k >= n) k = 0;
  step_k = k;
  write_state(lines == 3 ? SEQ3[k] : SEQ1[k]);
  portEXIT_CRITICAL_ISR(&mux);
}

static void set_thr(uint32_t us, uint32_t hz) {
  if (us == 0) {
    ledcWrite(PIN_THR, 0);
    return;
  }
  ledcChangeFrequency(PIN_THR, hz, 16);
  uint32_t period = 1000000UL / hz;
  ledcWrite(PIN_THR, (uint32_t)((uint64_t)us * 65535 / period));
}

static double current_rpm() {
  if (!ramping) return rpm;
  uint32_t dt = millis() - ramp_t0;
  if (dt >= ramp_ms) {
    ramping = false;
    rpm = rpm_to;
    return rpm;
  }
  return rpm_from + (rpm_to - rpm_from) * (double)dt / ramp_ms;
}

static void apply_rpm(double r) {
  uint32_t iv = 0;
  if (r >= 1.0) {
    double v = 60.0e6 / (r * 2 * lines) + 0.5;
    iv = v > 2e9 ? 2000000000u : (uint32_t)v;
    if (iv < 20) iv = 20;
  }
  if (iv == cur_interval) return;
  if (iv == 0) {
    if (tim_running) timerStop(tim);
    tim_running = false;
  } else {
    timerAlarm(tim, iv, true, 0);
    if (!tim_running) {
      timerWrite(tim, 0);
      timerStart(tim);
      tim_running = true;
    }
  }
  cur_interval = iv;
}

static void start_ramp(double to, uint32_t ms) {
  rpm_from = current_rpm();
  rpm = rpm_from;
  rpm_to = to;
  ramp_t0 = millis();
  ramp_ms = ms ? ms : 1;
  ramping = true;
}

// LAUNCH – nieblokująca sekwencja
static int launch_phase = 0;
static uint32_t launch_t = 0;

static void launch_poll() {
  if (!launch_phase) return;
  uint32_t dt = millis() - launch_t;
  switch (launch_phase) {
    case 1: if (dt >= 3000) { thr_us = 1900; set_thr(thr_us, thr_hz); launch_phase = 2; launch_t = millis(); } break;
    case 2: if (dt >= 25) { start_ramp(30000, 1500); launch_phase = 3; launch_t = millis(); } break;
    case 3: if (dt >= 3500) { start_ramp(0, 500); thr_us = 1500; set_thr(thr_us, thr_hz); launch_phase = 0;
                              Serial.println("LAUNCH done"); } break;
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_A, OUTPUT); pinMode(PIN_B, OUTPUT); pinMode(PIN_C, OUTPUT);
  write_state(1);
  ledcAttach(PIN_THR, thr_hz, 16);
  set_thr(thr_us, thr_hz);
  tim = timerBegin(1000000);          // 1 MHz
  timerAttachInterrupt(tim, &on_timer);
  timerStop(tim);
  Serial.println("hall_sim gotowy: LINES/RPM/RAMP/THR/SPIKE/LAUNCH/STATUS");
}

static String line;

static void handle(const String& l) {
  char cmd[16] = {0};
  double a = 0, b = 0;
  sscanf(l.c_str(), "%15s %lf %lf", cmd, &a, &b);
  String c(cmd);
  if (c == "LINES") {
    lines = (a == 3) ? 3 : 1;
    step_k = 0;
    write_state(1);
    cur_interval = 0xFFFFFFFF;   // wymuś przeliczenie okresu
  } else if (c == "RPM") { ramping = false; rpm = a; }
  else if (c == "RAMP") start_ramp(a, (uint32_t)b);
  else if (c == "THR") { thr_us = (uint32_t)a; if (b > 0) thr_hz = (uint32_t)b; set_thr(thr_us, thr_hz); }
  else if (c == "SPIKE") {
    portENTER_CRITICAL(&mux);
    uint32_t m = 1u << PIN_A;
    bool hi = GPIO.out & m;
    if (hi) GPIO.out_w1tc = m; else GPIO.out_w1ts = m;
    delayMicroseconds((uint32_t)a);
    if (hi) GPIO.out_w1ts = m; else GPIO.out_w1tc = m;
    portEXIT_CRITICAL(&mux);
  } else if (c == "LAUNCH") {
    ramping = false; rpm = 0; thr_us = 1500; set_thr(thr_us, thr_hz);
    launch_phase = 1; launch_t = millis();
  } else if (c == "STATUS") {
    Serial.printf("lines=%d rpm=%.0f interval=%u us thr=%u us @ %u Hz\n", lines, current_rpm(),
                  cur_interval, thr_us, thr_hz);
    return;
  } else {
    Serial.println("ERR");
    return;
  }
  Serial.println("OK");
}

void loop() {
  static uint32_t last_ms = 0;
  uint32_t ms = millis();
  if (ms != last_ms) {
    last_ms = ms;
    launch_poll();
    apply_rpm(current_rpm());
  }
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n') { line.trim(); if (line.length()) handle(line); line = ""; }
    else if (ch != '\r') line += ch;
  }
}
