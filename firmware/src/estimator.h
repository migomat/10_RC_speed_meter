/* Estymator prędkości do wyświetlania (prędkościomierz, v_disp).
 *
 * Czysty C, bez zależności od Arduino – ten sam kod kompiluje się na PC
 * (tools/est_host) i jest porównywany z tools/rc_launch/estimator.py
 * oraz app/js/estimator.js na wektorach testowych.
 *
 * Specyfikacja kroku (wejście = jedna próbka 500 Hz w osiach auta):
 *   1. obrót poziomy o yaw_offset: af = a_fwd·cosψ + a_left·sinψ
 *   2. bezruch: hall_stopped ∧ |ω − bias| < zupt_gyro_dps ∧ ||a| − 1| < zupt_acc_g
 *      still_t += dt (inaczej still_t = 0 i zerowanie sum)
 *   3. ZUPT (still_t ≥ zupt_ms): a_ref = średnia af, bias ω = średnia ω
 *      z okresu bezruchu, pitch = 0, v = 0
 *   4. inaczej: pitch += −(ω_left − bias_left)·dt (nos w górę = +)
 *      a_long = G·(af − a_ref − sin(pitch))   [m/s²]
 *      v += a_long·dt
 *   5. a_front = d(v_front)/dt, a_front_f i a_long_f – filtr IIR 1. rzędu
 *      α = dt/(tau + dt)
 *   6. acc_ok = |a_front_f − a_long_f| < slip_acc_thr; grip_t liczy czas acc_ok
 *      spinning = (a_front_f − a_long_f ≥ slip_acc_thr) ∨
 *                 (v_front > v + slip_v_thr ∧ grip_t < grip_hold)
 *      locked   = throttle_pct < brake_pct ∧ v_front < v − slip_v_thr
 *      korekta, gdy ¬ZUPT ∧ ¬spinning ∧ ¬locked ∧ acc_ok ∧
 *                   (|v_front − v| < slip_v_thr ∨ grip_t ≥ grip_hold):
 *        v += (v_front − v)·min(1, dt/tau_corr)
 *   7. v = max(v, 0); v_max = max(v_max, v)
 */
#ifndef ESTIMATOR_H
#define ESTIMATOR_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  float yaw_offset_deg;
  float zupt_ms;
  float zupt_gyro_dps;
  float zupt_acc_g;
  float slip_acc_thr;     /* m/s² */
  float slip_v_thr;       /* m/s */
  float grip_hold_ms;
  float wheel_corr_tau_s;
  float brake_pct;
  float afront_tau_s;
} EstParams;

typedef struct {
  float dt_s;
  float a_fwd_g, a_left_g, a_up_g;       /* osie auta, g */
  float g_fwd_dps, g_left_dps, g_up_dps; /* osie auta, dps */
  float v_front_mps;
  int   hall_stopped;                    /* brak zboczy i period = 0 */
  float throttle_pct;                    /* brak sygnału → 0 */
} EstInput;

typedef struct {
  float v, v_max;
  float pitch_rad;
  float a_ref_g;
  float gbias_dps[3];
  float still_t;
  double still_sum_a, still_sum_g[3];
  uint32_t still_n;
  float vf_prev;
  float a_front_f, a_long_f;
  float a_long;
  float grip_t;
  int   spinning;
  int   zupt;
  int   first;
} EstState;

void est_default_params(EstParams* p);
void est_init(EstState* s);
void est_step(EstState* s, const EstParams* p, const EstInput* in);

#ifdef __cplusplus
}
#endif

#endif
