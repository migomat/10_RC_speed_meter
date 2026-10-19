// Czujnik halla silnika: znaczniki czasu zboczy (PPI capture), filtr
// zakłóceń, prędkość z okresu pełnego obrotu, opcjonalny log surowych zboczy.
#pragma once

#include <stdint.h>
#include "run_format.h"

struct HallSnapshot {
  uint32_t edges;          // licznik zboczy
  uint32_t period_rev_us;  // 0 = nieznany / postój
  uint8_t  flags;          // SF_HALL_VALID | SF_DIR_*
};

bool hall_begin(void);             // po włączeniu SoftDevice (PPI przez sd_ppi_*)
// Wołane z ISR taktu próbkowania (priorytet 3).
void hall_tick_snapshot(uint32_t t_tick, HallSnapshot* out);
// Wstrzyknięcie zbocza (komendy SIM_*, testy) – kontekst wątku.
void hall_inject_edge(uint32_t t_us, uint8_t line, uint8_t level);

uint8_t  hall_line_level(uint8_t line);
// Bieżący stan pinów (bity A|B<<1|C<<2) – diagnostyka okablowania (STATUS)
uint8_t  hall_pin_levels(void);
// SIM: ustawienie stanu linii (bity A|B<<1|C<<2) bez generowania zboczy
void     hall_force_levels(uint8_t mask);
uint32_t hall_edge_count(void);
uint32_t hall_glitch_count(void);

// --- log surowych zboczy (indeks w logu = licznik zboczy % EDGE_LOG_LEN)
// Ochrona zakresu [start, ...) przed nadpisaniem w czasie nagrywania/zapisu.
void     hall_log_protect(bool on, uint32_t start_count);
bool     hall_log_truncated(void);
// Słowo zbocza o indeksie (licznik) idx, przeliczone na czas względem t0_us.
EdgeWord hall_log_word(uint32_t idx, uint32_t t0_us);
