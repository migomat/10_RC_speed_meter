// Pomiar szerokości impulsu gazu (PPI capture na obu zboczach) i uczenie neutrum.
#pragma once

#include <stdint.h>

bool     throttle_begin(void);        // po włączeniu SoftDevice
// ISR taktu: szerokość ostatniego poprawnego impulsu albo 0 (brak sygnału).
uint16_t throttle_tick_width(uint32_t t_tick);
// Pętla główna: uczenie neutrum (THROTTLE_NEUTRAL_LEARN_MS stabilnego gazu).
void     throttle_poll(uint32_t now_us);
bool     throttle_neutral_learned(void);
uint16_t throttle_neutral_us(void);
bool     throttle_signal_ok(uint32_t now_us);
uint16_t throttle_last_width(void);
// % zakresu „do przodu” (ujemne = hamowanie/wstecz). Brak sygnału/neutrum → 0.
float    throttle_pct(uint16_t width_us);
void     throttle_inject_pulse(uint32_t t_rise_us, uint16_t width_us);  // SIM
uint32_t throttle_pulse_count(void);
uint32_t throttle_reject_count(void);
