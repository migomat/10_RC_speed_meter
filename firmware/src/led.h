// Dioda RGB (aktywna niskim). Stan wyliczany co przebieg pętli z priorytetami.
#pragma once

#include <stdint.h>

struct LedInputs {
  bool recording, saving, error, no_throttle, learning, connected, unread_runs;
};

void led_begin(void);
void led_poll(const LedInputs& in, uint32_t now_ms);
