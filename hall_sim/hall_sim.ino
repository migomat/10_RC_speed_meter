// Symulator sygnałów halla i gazu dla X4F-LOG (ESP32, Arduino core 3.x).
// Wyjścia 3,3 V – podłączać do Xiao BEZ dzielnika 47k/68k.
//
//   GPIO25 → D1 (hall A)   GPIO26 → D2 (hall B)   GPIO27 → D3 (hall C)
//   GPIO32 → D0 (gaz)      GND → GND
//
// Komendy (Serial 115200, zakończone \n):
//   LINES 1|3            tryb halla (1 linia albo 3 linie przesunięte o 120° el.)
//   RPM <rpm>            stałe obroty (0 = stop)
//   RAMP <rpm> <ms>      rampa od bieżących obrotów
//   THR <us> [hz]        impuls gazu (np. THR 1500 50), THR 0 = brak sygnału
//   STEP <us>            skok gazu do <us> (wyzwolenie startu)
//   SPIKE <us>           pojedyncza szpilka na linii A o szerokości <us>
//   LAUNCH               scenariusz: neutrum 3 s, skok gazu, rampa 0→30000 rpm w 1,5 s
//   STATUS
#include <Arduino.h>

#define PIN_A 25
#define PIN_B 26
#define PIN_C 27
#define PIN_THR 32

static volatile int lines = 1;
static volatile double rpm = 0, rpm_from = 0, rpm_to = 0;
static volatile uint32_t ramp_t0 = 0, ramp_us = 0;
static volatile bool ramping = false;
static volatile double phase = 0;   // ułamek do następnego zbocza
static volatile int step_k = 0;
static hw_timer_t* tim = nullptr;
static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

// sekwencja stanów A|B<<1|C<<2: 1,3,2,6,4,5 (jak firmware/src/sim.cpp)
static const uint8_t SEQ3[6] = {1, 3, 2, 6, 4, 5};
static const uint8_t SEQ1[2] = {1, 0};

static void write_state(uint8_t st) {
  digitalWrite(PIN_A, st & 1);
  if (lines == 3) {
    digitalWrite(PIN_B, (st >> 1) & 1);
    digitalWrite(PIN_C, (st >> 2) & 1);
  }
}

// Przerwanie co 5 us: całkowanie fazy obrotu
static void IRAM_ATTR on_timer() {
  portENTER_CRITICAL_ISR(&mux);
  double r = rpm;
  if (ramping) {
    uint32_t dt = micros() - ramp_t0;
    if (dt >= ramp_us) { ramping = false; rpm = rpm_to; r = rpm_to; }
    else r = rpm_from + (rpm_to - rpm_from) * (double)dt / ramp_us;
  }
  int epr = 2 * lines;
  phase += r * epr / 60.0e6 * 5.0;
  if (phase >= 1.0) {
    phase -= 1.0;
    int n = lines == 3 ? 6 : 2;
    step_k = (step_k + 1) % n;
    write_state(lines == 3 ? SEQ3[step_k] : SEQ1[step_k]);
  }
  portEXIT_CRITICAL_ISR(&mux);
}

static void set_thr(uint32_t us, uint32_t hz) {
  if (us == 0) {
    ledcWrite(PIN_THR, 0);
    return;
  }
  // 16-bit rozdzielczość: wypełnienie = us / okres
  ledcChangeFrequency(PIN_THR, hz, 16);
  uint32_t period = 1000000UL / hz;
  ledcWrite(PIN_THR, (uint32_t)((uint64_t)us * 65535 / period));
}

static uint32_t thr_hz = 50, thr_us = 1500;

void setup() {
  Serial.begin(115200);
  pinMode(PIN_A, OUTPUT); pinMode(PIN_B, OUTPUT); pinMode(PIN_C, OUTPUT);
  write_state(1);
  ledcAttach(PIN_THR, thr_hz, 16);
  set_thr(thr_us, thr_hz);
  tim = timerBegin(1000000);
  timerAttachInterrupt(tim, &on_timer);
  timerAlarm(tim, 5, true, 0);
  Serial.println("hall_sim gotowy: LINES/RPM/RAMP/THR/STEP/SPIKE/LAUNCH/STATUS");
}

static void ramp(double to, uint32_t ms) {
  portENTER_CRITICAL(&mux);
  rpm_from = rpm; rpm_to = to; ramp_t0 = micros(); ramp_us = ms * 1000; ramping = true;
  portEXIT_CRITICAL(&mux);
}

void loop() {
  if (!Serial.available()) return;
  String line = Serial.readStringUntil('\n');
  line.trim();
  char cmd[16] = {0};
  double a = 0, b = 0;
  sscanf(line.c_str(), "%15s %lf %lf", cmd, &a, &b);
  String c(cmd);
  if (c == "LINES") { lines = (a == 3) ? 3 : 1; step_k = 0; write_state(1); }
  else if (c == "RPM") { portENTER_CRITICAL(&mux); ramping = false; rpm = a; portEXIT_CRITICAL(&mux); }
  else if (c == "RAMP") ramp(a, (uint32_t)b);
  else if (c == "THR") { thr_us = a; if (b > 0) thr_hz = b; set_thr(thr_us, thr_hz); }
  else if (c == "STEP") { thr_us = a; set_thr(thr_us, thr_hz); }
  else if (c == "SPIKE") {
    portENTER_CRITICAL(&mux);
    int lvl = digitalRead(PIN_A);
    digitalWrite(PIN_A, !lvl);
    delayMicroseconds((uint32_t)a);
    digitalWrite(PIN_A, lvl);
    portEXIT_CRITICAL(&mux);
  } else if (c == "LAUNCH") {
    rpm = 0; thr_us = 1500; set_thr(thr_us, thr_hz);
    delay(3000);
    thr_us = 1900; set_thr(thr_us, thr_hz);
    delay(25);
    ramp(30000, 1500);
    delay(3500);
    ramp(0, 500);
    thr_us = 1500; set_thr(thr_us, thr_hz);
  } else if (c == "STATUS") {
    Serial.printf("lines=%d rpm=%.0f thr=%u us @ %u Hz\n", lines, rpm, thr_us, thr_hz);
    return;
  } else {
    Serial.println("ERR");
    return;
  }
  Serial.println("OK");
}
