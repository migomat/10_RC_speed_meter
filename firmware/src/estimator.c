#include "estimator.h"

#include <math.h>
#include <string.h>

#ifdef ARDUINO
#include "config.h"
#endif

#define EST_G     9.80665f
#define EST_PI    3.14159265358979f

void est_default_params(EstParams* p) {
#ifdef ARDUINO
  p->yaw_offset_deg = IMU_YAW_OFFSET_DEG;
  p->zupt_ms = ZUPT_MS;
  p->zupt_gyro_dps = ZUPT_GYRO_DPS;
  p->zupt_acc_g = ZUPT_ACC_G;
  p->slip_acc_thr = SLIP_ACC_THR;
  p->slip_v_thr = SLIP_V_THR;
  p->grip_hold_ms = GRIP_HOLD_MS;
  p->wheel_corr_tau_s = WHEEL_CORR_TAU_S;
  p->brake_pct = BRAKE_THROTTLE_PCT;
  p->afront_tau_s = EST_AFRONT_TAU_S;
#else
  p->yaw_offset_deg = 0.0f;
  p->zupt_ms = 200.0f;
  p->zupt_gyro_dps = 4.0f;
  p->zupt_acc_g = 0.04f;
  p->slip_acc_thr = 3.0f;
  p->slip_v_thr = 0.5f;
  p->grip_hold_ms = 100.0f;
  p->wheel_corr_tau_s = 0.5f;
  p->brake_pct = -10.0f;
  p->afront_tau_s = 0.03f;
#endif
}

void est_init(EstState* s) {
  memset(s, 0, sizeof(*s));
  s->first = 1;
}

void est_step(EstState* s, const EstParams* p, const EstInput* in) {
  const float dt = in->dt_s;
  if (dt <= 0.0f) return;
  const float yaw = p->yaw_offset_deg * (EST_PI / 180.0f);
  const float c = cosf(yaw), sn = sinf(yaw);
  const float af = in->a_fwd_g * c + in->a_left_g * sn;
  const float al = -in->a_fwd_g * sn + in->a_left_g * c;
  const float au = in->a_up_g;

  /* 2. bezruch */
  const float gx = in->g_fwd_dps - s->gbias_dps[0];
  const float gy = in->g_left_dps - s->gbias_dps[1];
  const float gz = in->g_up_dps - s->gbias_dps[2];
  const float gmag = sqrtf(gx * gx + gy * gy + gz * gz);
  const float amag = sqrtf(af * af + al * al + au * au);
  const int still = in->hall_stopped && gmag < p->zupt_gyro_dps &&
                    fabsf(amag - 1.0f) < p->zupt_acc_g;
  if (still) {
    s->still_t += dt;
    s->still_n++;
    s->still_sum_a += af;
    s->still_sum_g[0] += in->g_fwd_dps;
    s->still_sum_g[1] += in->g_left_dps;
    s->still_sum_g[2] += in->g_up_dps;
  } else {
    s->still_t = 0.0f;
    s->still_n = 0;
    s->still_sum_a = 0.0;
    s->still_sum_g[0] = s->still_sum_g[1] = s->still_sum_g[2] = 0.0;
  }
  s->zupt = s->still_t * 1000.0f >= p->zupt_ms;

  /* 3./4. ZUPT albo całkowanie */
  float a_long;
  if (s->zupt) {
    const double n = (double)s->still_n;
    s->a_ref_g = (float)(s->still_sum_a / n);
    s->gbias_dps[0] = (float)(s->still_sum_g[0] / n);
    s->gbias_dps[1] = (float)(s->still_sum_g[1] / n);
    s->gbias_dps[2] = (float)(s->still_sum_g[2] / n);
    s->pitch_rad = 0.0f;
    s->v = 0.0f;
    a_long = 0.0f;
  } else {
    s->pitch_rad += -(in->g_left_dps - s->gbias_dps[1]) * dt * (EST_PI / 180.0f);
    a_long = EST_G * (af - s->a_ref_g - sinf(s->pitch_rad));
    s->v += a_long * dt;
  }
  s->a_long = a_long;

  /* 5. przyspieszenia do detekcji buksowania */
  const float vf = in->v_front_mps;
  if (s->first) {
    s->vf_prev = vf;
    s->first = 0;
  }
  const float a_front = (vf - s->vf_prev) / dt;
  s->vf_prev = vf;
  const float alpha = dt / (p->afront_tau_s + dt);
  s->a_front_f += alpha * (a_front - s->a_front_f);
  s->a_long_f += alpha * (a_long - s->a_long_f);

  /* 6. korekta z kół */
  const float da = s->a_front_f - s->a_long_f;
  const int acc_ok = fabsf(da) < p->slip_acc_thr;
  s->grip_t = acc_ok ? s->grip_t + dt : 0.0f;
  const int grip_long = s->grip_t * 1000.0f >= p->grip_hold_ms;
  const int vel_ok = fabsf(vf - s->v) < p->slip_v_thr;
  const int spinning = (da >= p->slip_acc_thr) || (vf > s->v + p->slip_v_thr && !grip_long);
  const int locked = (in->throttle_pct < p->brake_pct) && (vf < s->v - p->slip_v_thr);
  if (!s->zupt && !spinning && !locked && acc_ok && (vel_ok || grip_long)) {
    float k = dt / p->wheel_corr_tau_s;
    if (k > 1.0f) k = 1.0f;
    s->v += (vf - s->v) * k;
  }
  s->spinning = spinning;

  /* 7. */
  if (s->v < 0.0f) s->v = 0.0f;
  if (s->v > s->v_max) s->v_max = s->v;
}
