// Syntetyczne zbocza halla i impulsy gazu (komendy SIM_* przez USB) –
// testy loggera, flash i protokołu na biurku, bez auta i symulatora ESP32.
#pragma once

#include <stdint.h>

void sim_poll(uint32_t now_us);
void sim_set_rpm(float rpm);                    // stała prędkość (0 = stop)
void sim_ramp(float rpm_to, uint32_t ms);       // liniowa rampa od bieżącej
void sim_set_throttle(uint16_t width_us);       // 0 = brak impulsów
void sim_glitch(uint32_t width_us);             // szpilka na linii A
bool sim_active(void);
float sim_rpm(void);
void sim_off(void);
