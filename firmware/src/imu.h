// LSM6DS3TR-C na Wire1 (adres 0x6A), FIFO w trybie ciągłym, 833 Hz.
#pragma once

#include <stdint.h>

struct ImuRaw {
  uint32_t t_us;           // szacowany czas pomiaru
  int16_t  ax, ay, az;     // surowe LSB
  int16_t  gx, gy, gz;
};

bool     imu_begin(void);
bool     imu_ok(void);
// Pętla główna: opróżnia FIFO do historii. Zwraca liczbę nowych próbek.
int      imu_poll(uint32_t now_us);
// Najnowsza próbka z czasem <= t_us (+pół okresu). false = brak świeżej.
bool     imu_get_at(uint32_t t_us, ImuRaw* out);
uint32_t imu_overrun_count(void);
uint32_t imu_misaligned_count(void);  // odrzucone serie FIFO (utrata wyrównania)
bool     imu_take_overrun_flag(void);   // czy było przepełnienie od ostatniego wywołania
uint32_t imu_sample_count(void);
float    imu_accel_scale_g(void);
float    imu_gyro_scale_dps(void);
