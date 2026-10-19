// Komendy tekstowe (USB Serial i BLE NUS) – lista, pobieranie, kasowanie,
// podgląd na żywo. Opis protokołu: docs/PROTOCOL.md.
#pragma once

#include <stdint.h>

// Dane do STATUS / LIVE, aktualizowane przez pętlę główną
struct LiveData {
  float    v, v_max, v_front;   // m/s
  bool     spinning;
  float    rpm;                 // obroty silnika
  uint16_t throttle_us;
  float    throttle_pct;
  float    ax_g;                // przyspieszenie wzdłużne (surowe, oś „do przodu”)
  bool     zupt;
};
extern LiveData g_live;
extern uint32_t g_stack_free_min;   // [B] najmniejszy zapas stosu zadania loop
extern volatile uint32_t g_tick_overflows;
extern uint32_t g_boot_gpregret, g_boot_hf;   // diagnostyka resetów (main.cpp)

bool ble_begin(void);           // konfiguracja SoftDevice – przed timebase/hall
void links_poll(uint32_t now_ms);
bool ble_connected(void);
bool vmax_reset_requested(void);
// Wołane w trakcie czekania na bufor BLE (odczyt IMU, takty) – pomiar nie staje
void ble_set_wait_hook(void (*fn)(void));
