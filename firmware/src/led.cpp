#include "led.h"

#include <Arduino.h>

// Diody na XIAO są aktywne niskim (initVariant ustawia HIGH = zgaszone),
// mimo że variant.h deklaruje LED_STATE_ON = 1.
static void set_rgb(bool r, bool g, bool b) {
  digitalWrite(LED_RED, r ? LOW : HIGH);
  digitalWrite(LED_GREEN, g ? LOW : HIGH);
  digitalWrite(LED_BLUE, b ? LOW : HIGH);
}

void led_begin(void) {
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
  set_rgb(false, false, false);
}

void led_poll(const LedInputs& in, uint32_t now_ms) {
  bool blink4 = (now_ms / 125) & 1;   // 4 Hz
  bool blink2 = (now_ms / 250) & 1;   // 2 Hz
  if (in.recording) {
    set_rgb(true, false, false);                    // czerwony
  } else if (in.saving) {
    set_rgb(true, false, true);                     // fioletowy
  } else if (in.error || in.no_throttle) {
    set_rgb(blink4, false, false);                  // czerwony miga
  } else if (in.learning) {
    set_rgb(blink2, blink2, false);                 // żółty miga
  } else if (in.connected) {
    set_rgb(false, false, true);                    // niebieski
  } else if (in.unread_runs && (now_ms % 2000) < 100) {
    set_rgb(false, false, true);                    // krótki niebieski błysk co 2 s
  } else {
    set_rgb(false, true, false);                    // zielony – uzbrojony
  }
}
