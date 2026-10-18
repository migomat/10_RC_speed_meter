// Estymator prędkości wyświetlanej – port 1:1 firmware/src/estimator.c
// i tools/rc_launch/estimator.py. Specyfikacja kroku: estimator.h.
// Prędkość z estymatora NIE wchodzi do obliczeń poślizgu.

const G = 9.80665;

export const DEFAULT_EST_PARAMS = {
  yaw_offset_deg: 0.0,
  zupt_ms: 200,
  zupt_gyro_dps: 4.0,
  zupt_acc_g: 0.04,
  slip_acc_thr: 3.0,
  slip_v_thr: 0.5,
  grip_hold_ms: 100,
  wheel_corr_tau_s: 0.5,
  brake_pct: -10,
  afront_tau_s: 0.03,
};

export function estInit() {
  return {
    v: 0, v_max: 0, pitch_rad: 0, a_ref_g: 0, gbias: [0, 0, 0],
    still_t: 0, still_sum_a: 0, still_sum_g: [0, 0, 0], still_n: 0,
    vf_prev: 0, a_front_f: 0, a_long_f: 0, a_long: 0, grip_t: 0,
    spinning: false, zupt: false, first: true,
  };
}

export function estStep(s, p, dt, aFwd, aLeft, aUp, gFwd, gLeft, gUp, vFront, hallStopped, thrPct) {
  if (dt <= 0) return;
  const yaw = p.yaw_offset_deg * Math.PI / 180;
  const c = Math.cos(yaw), sn = Math.sin(yaw);
  const af = aFwd * c + aLeft * sn;
  const al = -aFwd * sn + aLeft * c;
  const au = aUp;

  const gx = gFwd - s.gbias[0], gy = gLeft - s.gbias[1], gz = gUp - s.gbias[2];
  const gmag = Math.sqrt(gx * gx + gy * gy + gz * gz);
  const amag = Math.sqrt(af * af + al * al + au * au);
  const still = !!hallStopped && gmag < p.zupt_gyro_dps && Math.abs(amag - 1.0) < p.zupt_acc_g;
  if (still) {
    s.still_t += dt;
    s.still_n += 1;
    s.still_sum_a += af;
    s.still_sum_g[0] += gFwd; s.still_sum_g[1] += gLeft; s.still_sum_g[2] += gUp;
  } else {
    s.still_t = 0; s.still_n = 0; s.still_sum_a = 0; s.still_sum_g = [0, 0, 0];
  }
  s.zupt = s.still_t * 1000 >= p.zupt_ms;

  let aLong;
  if (s.zupt) {
    const n = s.still_n;
    s.a_ref_g = s.still_sum_a / n;
    s.gbias = [s.still_sum_g[0] / n, s.still_sum_g[1] / n, s.still_sum_g[2] / n];
    s.pitch_rad = 0;
    s.v = 0;
    aLong = 0;
  } else {
    s.pitch_rad += -(gLeft - s.gbias[1]) * dt * (Math.PI / 180);
    aLong = G * (af - s.a_ref_g - Math.sin(s.pitch_rad));
    s.v += aLong * dt;
  }
  s.a_long = aLong;

  const vf = vFront;
  if (s.first) { s.vf_prev = vf; s.first = false; }
  const aFront = (vf - s.vf_prev) / dt;
  s.vf_prev = vf;
  const alpha = dt / (p.afront_tau_s + dt);
  s.a_front_f += alpha * (aFront - s.a_front_f);
  s.a_long_f += alpha * (aLong - s.a_long_f);

  const da = s.a_front_f - s.a_long_f;
  const accOk = Math.abs(da) < p.slip_acc_thr;
  s.grip_t = accOk ? s.grip_t + dt : 0;
  const gripLong = s.grip_t * 1000 >= p.grip_hold_ms;
  const velOk = Math.abs(vf - s.v) < p.slip_v_thr;
  const spinning = (da >= p.slip_acc_thr) || (vf > s.v + p.slip_v_thr && !gripLong);
  const locked = (thrPct < p.brake_pct) && (vf < s.v - p.slip_v_thr);
  if (!s.zupt && !spinning && !locked && accOk && (velOk || gripLong)) {
    const k = Math.min(1, dt / p.wheel_corr_tau_s);
    s.v += (vf - s.v) * k;
  }
  s.spinning = spinning;

  if (s.v < 0) s.v = 0;
  if (s.v > s.v_max) s.v_max = s.v;
}
