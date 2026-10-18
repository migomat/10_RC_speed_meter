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

bool ble_begin(void);           // konfiguracja SoftDevice – przed timebase/hall
void links_poll(uint32_t now_ms);
bool ble_connected(void);
bool vmax_reset_requested(void);
