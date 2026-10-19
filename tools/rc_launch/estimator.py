"""Estymator prędkości wyświetlanej – port 1:1 firmware/src/estimator.c.

Specyfikacja kroku w nagłówku estimator.h. Ten sam kod (w JS) jest
w app/js/estimator.js. Prędkość z estymatora NIE wchodzi do poślizgu.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

G = 9.80665

DEFAULT_PARAMS = dict(
    yaw_offset_deg=0.0,
    zupt_ms=200.0,
    zupt_gyro_dps=4.0,
    zupt_acc_g=0.06,
    slip_acc_thr=3.0,
    slip_v_thr=0.5,
    grip_hold_ms=100.0,
    wheel_corr_tau_s=0.5,
    brake_pct=-10.0,
    afront_tau_s=0.03,
    zupt_lpf_tau_s=0.1,
)


@dataclass
class EstState:
    v: float = 0.0
    v_max: float = 0.0
    pitch_rad: float = 0.0
    a_ref_g: float = 0.0
    gbias_dps: list = field(default_factory=lambda: [0.0, 0.0, 0.0])
    still_t: float = 0.0
    still_sum_a: float = 0.0
    still_sum_g: list = field(default_factory=lambda: [0.0, 0.0, 0.0])
    still_n: int = 0
    vf_prev: float = 0.0
    a_front_f: float = 0.0
    a_long_f: float = 0.0
    a_long: float = 0.0
    amag_f: float = 0.0
    gmag_f: float = 0.0
    grip_t: float = 0.0
    spinning: bool = False
    zupt: bool = False
    first: bool = True


def est_step(s: EstState, p: dict, dt, a_fwd, a_left, a_up, g_fwd, g_left, g_up,
             v_front, hall_stopped, throttle_pct):
    if dt <= 0:
        return
    yaw = math.radians(p["yaw_offset_deg"])
    c, sn = math.cos(yaw), math.sin(yaw)
    af = a_fwd * c + a_left * sn
    al = -a_fwd * sn + a_left * c
    au = a_up

    gx = g_fwd - s.gbias_dps[0]
    gy = g_left - s.gbias_dps[1]
    gz = g_up - s.gbias_dps[2]
    gmag = math.sqrt(gx * gx + gy * gy + gz * gz)
    amag = math.sqrt(af * af + al * al + au * au)
    if s.first:
        s.amag_f = amag
        s.gmag_f = gmag
    else:
        k = dt / (p["zupt_lpf_tau_s"] + dt)
        s.amag_f += k * (amag - s.amag_f)
        s.gmag_f += k * (gmag - s.gmag_f)
    still = bool(hall_stopped) and s.gmag_f < p["zupt_gyro_dps"] and abs(s.amag_f - 1.0) < p["zupt_acc_g"]
    if still:
        s.still_t += dt
        s.still_n += 1
        s.still_sum_a += af
        s.still_sum_g[0] += g_fwd
        s.still_sum_g[1] += g_left
        s.still_sum_g[2] += g_up
    else:
        s.still_t = 0.0
        s.still_n = 0
        s.still_sum_a = 0.0
        s.still_sum_g = [0.0, 0.0, 0.0]
    s.zupt = s.still_t * 1000.0 >= p["zupt_ms"]

    if s.zupt:
        n = s.still_n
        s.a_ref_g = s.still_sum_a / n
        s.gbias_dps = [x / n for x in s.still_sum_g]
        s.pitch_rad = 0.0
        s.v = 0.0
        a_long = 0.0
    else:
        s.pitch_rad += -(g_left - s.gbias_dps[1]) * dt * (math.pi / 180.0)
        a_long = G * (af - s.a_ref_g - math.sin(s.pitch_rad))
        s.v += a_long * dt
    s.a_long = a_long

    vf = v_front
    if s.first:
        s.vf_prev = vf
        s.first = False
    a_front = (vf - s.vf_prev) / dt
    s.vf_prev = vf
    alpha = dt / (p["afront_tau_s"] + dt)
    s.a_front_f += alpha * (a_front - s.a_front_f)
    s.a_long_f += alpha * (a_long - s.a_long_f)

    da = s.a_front_f - s.a_long_f
    acc_ok = abs(da) < p["slip_acc_thr"]
    s.grip_t = s.grip_t + dt if acc_ok else 0.0
    grip_long = s.grip_t * 1000.0 >= p["grip_hold_ms"]
    vel_ok = abs(vf - s.v) < p["slip_v_thr"]
    spinning = (da >= p["slip_acc_thr"]) or (vf > s.v + p["slip_v_thr"] and not grip_long)
    locked = (throttle_pct < p["brake_pct"]) and (vf < s.v - p["slip_v_thr"])
    if not s.zupt and not spinning and not locked and acc_ok and (vel_ok or grip_long):
        k = min(1.0, dt / p["wheel_corr_tau_s"])
        s.v += (vf - s.v) * k
    s.spinning = spinning

    if s.v < 0.0:
        s.v = 0.0
    if s.v > s.v_max:
        s.v_max = s.v
