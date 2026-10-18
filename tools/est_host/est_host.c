/* Estymator z firmware (firmware/src/estimator.c) uruchamiany na PC na pliku
 * startu – wejścia liczone tak jak w firmware/src/main.cpp (float32).
 * Wypisuje v_disp dla każdej próbki. Używane przez tools/tests/test_estimator_c.py.
 *   gcc -O2 -I../../firmware/src est_host.c ../../firmware/src/estimator.c -lm */
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "estimator.h"

#pragma pack(push, 1)
typedef struct {
  uint32_t t_us, hall_edges, period_rev_us;
  uint16_t throttle_us;
  int16_t ax, ay, az, gx, gy, gz;
  uint8_t flags, reserved;
} Sample;
#pragma pack(pop)

int main(int argc, char** argv) {
  if (argc < 2) return 2;
  FILE* f = fopen(argv[1], "rb");
  if (!f) return 1;
  static uint8_t buf[1 << 20];
  size_t n = fread(buf, 1, sizeof(buf), f);
  fclose(f);
  if (n < 128) return 1;
  uint32_t ns, neutral16, full16;
  float ka, kg;
  memcpy(&ka, buf + 36, 4);
  memcpy(&kg, buf + 40, 4);
  uint16_t neutral, full;
  memcpy(&neutral, buf + 44, 2);
  memcpy(&full, buf + 46, 2);
  uint8_t fwd = buf[27];
  memcpy(&ns, buf + 52, 4);
  (void)neutral16; (void)full16;
  const float fdr = argc > 2 ? atof(argv[2]) : 5.0f;
  const float dia_mm = argc > 3 ? atof(argv[3]) : 64.0f;
  EstParams p;
  est_default_params(&p);
  EstState s;
  est_init(&s);
  const Sample* smp = (const Sample*)(buf + 128);
  for (uint32_t i = 0; i < ns; i++) {
    const Sample* x = &smp[i];
    if (i > 0) {
      const Sample* pr = &smp[i - 1];
      EstInput in;
      in.dt_s = (float)(uint32_t)(x->t_us - pr->t_us) * 1e-6f;
      in.a_fwd_g = x->ax * ka;
      in.a_left_g = x->ay * ka;
      in.a_up_g = x->az * ka;
      in.g_fwd_dps = x->gx * kg;
      in.g_left_dps = x->gy * kg;
      in.g_up_dps = x->gz * kg;
      in.v_front_mps = x->period_rev_us
          ? (1.0e6f / (float)x->period_rev_us) / fdr * 3.14159265f * dia_mm * 1e-3f : 0.0f;
      in.hall_stopped = x->period_rev_us == 0 && x->hall_edges == pr->hall_edges;
      float pct = 0.0f;
      if (x->throttle_us && neutral) {
        pct = ((int)x->throttle_us - (int)neutral) * 100.0f / (full ? full : 500);
        if (!fwd) pct = -pct;
      }
      in.throttle_pct = pct;
      est_step(&s, &p, &in);
    }
    printf("%.5f\n", s.v);
  }
  return 0;
}
