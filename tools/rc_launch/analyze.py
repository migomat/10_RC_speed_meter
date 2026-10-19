"""Analiza startu – IMPLEMENTACJA REFERENCYJNA (spec wspólna z app/js/analysis.js).

Każda zmiana tutaj = ta sama zmiana w analysis.js i przejście wektorów testowych.
Algorytmy celowo napisane prostymi pętlami (łatwy, dokładny port do JS).

Kroki (numeracja jak w CLAUDE.md):
 1. surowe → jednostki (współczynniki z nagłówka)
 2. osie płytki → osie auta (imu_axes)
 3. bias: średnie z okna przed wyzwoleniem, jeśli auto stało
 4. pochylenie: całka ω_pitch = −(ω_left − bias); a_long = G·(a_fwd − a_ref − sin(pitch))
 5. Butterworth 2. rzędu, filtfilt z odbiciem nieparzystym (filtfilt_butter2)
 6. v_imu = Σ a_long·dt od startu i0 = min(wyzwolenie gazem, pierwsze zbocze halla)
 7. v_front z surowych zboczy (albo period_rev_us), ograniczenie między zboczami
 8. a_front = pochodna wygładzonego v_front
 9. poślizg s = (v_front − v_imu) / max(v_front, v_min)
10. wskaźnik buksowania a_front − a_long
11. droga z licznika zboczy i z całki v_imu
12. v_disp – estymator jak na Xiao (estimator.py)
"""
from __future__ import annotations

import math
from pathlib import Path

import numpy as np

from .estimator import DEFAULT_PARAMS, EstState, est_step
from .parse import Run, parse_run

G = 9.80665

DEFAULT_CONFIG = dict(
    final_drive_ratio=5.0,
    front_tire_diameter_mm=64.0,
    imu_axes=dict(forward="+x", left="+y", up="+z"),
    lowpass_hz=25.0,
    v_min_mps=0.3,
    rest_gyro_std_dps=1.5,
    rest_acc_std_g=0.06,
    estimator=dict(DEFAULT_PARAMS),
)


def load_config(path: str | Path | None = None) -> dict:
    cfg = {k: (dict(v) if isinstance(v, dict) else v) for k, v in DEFAULT_CONFIG.items()}
    if path is None:
        p = Path(__file__).resolve().parent.parent / "config.yaml"
        path = p if p.exists() else None
    if path:
        import yaml

        user = yaml.safe_load(open(path)) or {}
        for k, v in user.items():
            if isinstance(v, dict) and isinstance(cfg.get(k), dict):
                cfg[k].update(v)
            else:
                cfg[k] = v
    return cfg


# ------------------------------------------------------------------ filtr
def butter2_coeffs(fc: float, fs: float):
    """Butterworth 2. rzędu, transformacja biliniowa z prewarpingiem (= scipy.signal.butter(2))."""
    k = math.tan(math.pi * fc / fs)
    norm = 1.0 / (1.0 + math.sqrt(2.0) * k + k * k)
    b0 = k * k * norm
    return (b0, 2.0 * b0, b0), (2.0 * (k * k - 1.0) * norm, (1.0 - math.sqrt(2.0) * k + k * k) * norm)


def _biquad_pass(x, b, a):
    """DF-II transponowana, stan początkowy = stan ustalony dla x[0]."""
    b0, b1, b2 = b
    a1, a2 = a
    n = len(x)
    y = [0.0] * n
    if n == 0:
        return y
    u = x[0]
    z1 = (b1 - a1 + b2 - a2) * u
    z2 = (b2 - a2) * u
    for i in range(n):
        xi = x[i]
        yi = b0 * xi + z1
        z1 = b1 * xi - a1 * yi + z2
        z2 = b2 * xi - a2 * yi
        y[i] = yi
    return y


def filtfilt_butter2(x, fc: float, fs: float) -> np.ndarray:
    """Zero-fazowy LP: odbicie nieparzyste na brzegach (padlen = min(n−1, 3·ceil(fs/fc))),
    przebieg w przód i w tył ze stanem ustalonym na początku każdego przebiegu."""
    x = [float(v) for v in x]
    n = len(x)
    if n < 2 or fc <= 0 or fc >= fs / 2:
        return np.asarray(x, dtype=np.float64)
    b, a = butter2_coeffs(fc, fs)
    pad = min(n - 1, 3 * int(math.ceil(fs / fc)))
    left = [2.0 * x[0] - x[i] for i in range(pad, 0, -1)]
    right = [2.0 * x[n - 1] - x[n - 2 - i] for i in range(pad)]
    ext = left + x + right
    y = _biquad_pass(ext, b, a)
    y.reverse()
    y = _biquad_pass(y, b, a)
    y.reverse()
    return np.asarray(y[pad : pad + n], dtype=np.float64)


def gradient(v, t) -> np.ndarray:
    """Pochodna: różnice centralne wewnątrz, jednostronne na brzegach."""
    n = len(v)
    out = np.zeros(n)
    if n < 2:
        return out
    out[0] = (v[1] - v[0]) / (t[1] - t[0])
    out[n - 1] = (v[n - 1] - v[n - 2]) / (t[n - 1] - t[n - 2])
    for i in range(1, n - 1):
        out[i] = (v[i + 1] - v[i - 1]) / (t[i + 1] - t[i - 1])
    return out


def interp_at(t, y, tq: float) -> float:
    """Interpolacja liniowa y(tq); NaN poza zakresem."""
    n = len(t)
    if n == 0 or tq < t[0] or tq > t[n - 1]:
        return float("nan")
    for i in range(1, n):
        if t[i] >= tq:
            if t[i] == t[i - 1]:
                return float(y[i])
            f = (tq - t[i - 1]) / (t[i] - t[i - 1])
            return float(y[i - 1] + f * (y[i] - y[i - 1]))
    return float(y[n - 1])


def first_crossing(t, y, level: float, i_from: int = 0) -> float:
    """Pierwszy czas, gdy y ≥ level (interpolacja liniowa); NaN gdy brak."""
    for i in range(max(i_from, 1), len(y)):
        if y[i] >= level and y[i - 1] < level:
            f = (level - y[i - 1]) / (y[i] - y[i - 1])
            return float(t[i - 1] + f * (t[i] - t[i - 1]))
        if i == i_from and y[i] >= level:
            return float(t[i])
    return float("nan")


# ------------------------------------------------------------------ osie
def parse_axis(spec: str):
    spec = spec.strip().lower()
    sign = -1.0 if spec.startswith("-") else 1.0
    return "xyz".index(spec[-1]), sign


def map_axes(vx, vy, vz, axes: dict):
    board = (vx, vy, vz)
    out = []
    for name in ("forward", "left", "up"):
        idx, sign = parse_axis(axes[name])
        out.append(board[idx] * sign)
    return out


# ------------------------------------------------------------------ v_front
def v_front_from_periods(period_us, m_per_rev):
    return np.array([m_per_rev * 1e6 / p if p > 0 else 0.0 for p in period_us], dtype=np.float64)


def v_front_from_edges(t_s, e_t, epr: int, timeout_s: float, m_per_rev: float) -> np.ndarray:
    """Prędkość z surowych zboczy na siatce próbek.

    Dla zbocza k (k_run = numer zbocza od początku ruchu, przerwa > timeout = postój):
      k_run ≥ epr: v_k = m_per_rev / (t[k] − t[k−epr])      (zbocze vs zbocze obrót wcześniej)
      1 ≤ k_run < epr: v_k = m_per_rev·(k_run/epr) / (t[k] − t[k−k_run])
      k_run = 0: v_k = 0
    Na siatce: v(t) = min(v_k, m_per_rev / (t − t[k+1−epr])) dla ostatniego zbocza k ≤ t
    (następne zbocze nie może dać krótszego okresu); po przerwie > timeout → 0.
    """
    n = len(t_s)
    ne = len(e_t)
    vk = [0.0] * ne
    run_start = 0
    for k in range(ne):
        if k == 0 or e_t[k] - e_t[k - 1] > timeout_s:
            run_start = k
        kr = k - run_start
        if kr >= epr:
            vk[k] = m_per_rev / (e_t[k] - e_t[k - epr])
        elif kr >= 1:
            vk[k] = m_per_rev * (kr / epr) / (e_t[k] - e_t[k - kr])
        else:
            vk[k] = 0.0
    out = np.zeros(n)
    k = -1
    run_start = 0
    for i in range(n):
        t = t_s[i]
        while k + 1 < ne and e_t[k + 1] <= t:
            k += 1
            if k == 0 or e_t[k] - e_t[k - 1] > timeout_s:
                run_start = k
        if k < 0 or t - e_t[k] > timeout_s:
            out[i] = 0.0
            continue
        v = vk[k]
        j = k + 1 - epr
        ref = e_t[j] if j >= run_start else e_t[run_start]
        if t > ref:
            bound = m_per_rev / (t - ref)
            if bound < v:
                v = bound
        out[i] = v
    return out


# ------------------------------------------------------------------ analiza
def analyze(run: Run, cfg: dict | None = None) -> dict:
    cfg = cfg or load_config()
    h = run.header
    s = run.samples
    n = len(s)
    fs = float(h["sample_rate_hz"])
    t = run.t_s                                  # względem wyzwolenia
    ti = int(h["trigger_index"])
    warnings = []

    # 1.–2.
    ka, kg = float(h["accel_scale_g"]), float(h["gyro_scale_dps"])
    a_fwd, a_left, a_up = map_axes(s["ax"] * ka, s["ay"] * ka, s["az"] * ka, cfg["imu_axes"])
    g_fwd, g_left, g_up = map_axes(s["gx"] * kg, s["gy"] * kg, s["gz"] * kg, cfg["imu_axes"])
    a_fwd, a_left, a_up = (np.asarray(x, dtype=np.float64) for x in (a_fwd, a_left, a_up))
    g_fwd, g_left, g_up = (np.asarray(x, dtype=np.float64) for x in (g_fwd, g_left, g_up))

    # Start: wcześniejsze z (wyzwolenie gazem, pierwsze zbocze halla w nagraniu).
    # ESC zwykle rusza silnik poniżej progu wyzwolenia, więc zbocze bywa pierwsze.
    edges = s["hall_edges"]
    i_edge0 = n
    for i in range(1, n):
        if edges[i] != edges[0]:
            i_edge0 = i
            break
    i0 = min(ti, i_edge0)

    # 3. bias z okna spoczynku przed startem (min. 50 próbek)
    rest_end = i0 if i0 >= 50 else min(50, n)
    if i0 < 50:
        warnings.append("krótkie okno spoczynku przed startem")
    def mean(x):
        return float(sum(float(v) for v in x[:rest_end]) / rest_end)
    def std(x):
        m = mean(x)
        return math.sqrt(sum((float(v) - m) ** 2 for v in x[:rest_end]) / rest_end)
    amag = np.sqrt(a_fwd ** 2 + a_left ** 2 + a_up ** 2)
    # rozrzut po filtrze lowpass_hz (tylko okno spoczynku) – drgania wentylatora
    # ESC (~180 Hz) nie mogą udawać ruchu auta
    def lp(x):
        return filtfilt_butter2(x[:rest_end], float(cfg["lowpass_hz"]), fs)
    at_rest = (max(std(lp(g_fwd)), std(lp(g_left)), std(lp(g_up))) < cfg["rest_gyro_std_dps"]
               and std(lp(amag)) < cfg["rest_acc_std_g"]
               and int(edges[rest_end - 1]) == int(edges[0]))
    if not at_rest:
        warnings.append("auto nie stało przed startem – bias niepewny")
    a_ref = mean(a_fwd)
    gb_left = mean(g_left)

    # 4. pochylenie i a_long (surowe)
    dt = np.zeros(n)
    for i in range(1, n):
        dt[i] = t[i] - t[i - 1]
    pitch = np.zeros(n)
    acc = 0.0
    for i in range(n):
        acc += -(g_left[i] - gb_left) * dt[i] * (math.pi / 180.0)
        pitch[i] = acc
    a_long_raw = np.array([G * (a_fwd[i] - a_ref - math.sin(pitch[i])) for i in range(n)])

    # 5. filtr
    fc = float(cfg["lowpass_hz"])
    a_long = filtfilt_butter2(a_long_raw, fc, fs)

    # 6. v_imu od startu
    v_imu = np.zeros(n)
    for i in range(i0 + 1, n):
        v_imu[i] = v_imu[i - 1] + a_long[i] * dt[i]

    # 7. v_front
    epr = int(h["edges_per_rev"])
    m_per_rev = math.pi * cfg["front_tire_diameter_mm"] * 1e-3 / cfg["final_drive_ratio"]
    timeout_s = h["hall_timeout_us"] * 1e-6
    use_edges = h["n_edges"] > 0 and not (h["flags"] & 0x0001)
    if use_edges:
        v_front = v_front_from_edges(t, run.edges_t_s, epr, timeout_s, m_per_rev)
    else:
        v_front = v_front_from_periods(s["period_rev_us"], m_per_rev)
    v_front_sample = v_front_from_periods(s["period_rev_us"], m_per_rev)

    # 8. a_front
    v_front_f = filtfilt_butter2(v_front, fc, fs)
    a_front = gradient(v_front_f, t)

    # 9.–10.
    vmin = float(cfg["v_min_mps"])
    slip = np.array([(v_front[i] - v_imu[i]) / max(v_front[i], vmin) if i >= i0 else 0.0
                     for i in range(n)])
    spin = a_front - a_long

    # 11. droga
    d_front = np.array([max(0, int(edges[i]) - int(edges[i0])) / epr * m_per_rev if i >= i0 else 0.0
                        for i in range(n)])
    d_imu = np.zeros(n)
    for i in range(i0 + 1, n):
        d_imu[i] = d_imu[i - 1] + v_imu[i] * dt[i]

    # 12. v_disp – estymator (wejście jak na Xiao: v_front z period_rev_us)
    neutral = h["throttle_neutral_us"]
    full = h["throttle_full_us"] or 500
    sign = 1.0 if h["throttle_fwd_higher"] else -1.0
    thr_pct = np.array([((int(w) - neutral) * 100.0 / full * sign) if (w and neutral) else 0.0
                        for w in s["throttle_us"]])
    p = dict(DEFAULT_PARAMS)
    p.update(cfg.get("estimator", {}))
    st = EstState()
    v_disp = np.zeros(n)
    for i in range(n):
        if i > 0:
            stopped = s["period_rev_us"][i] == 0 and edges[i] == edges[i - 1]
            est_step(st, p, dt[i], a_fwd[i], a_left[i], a_up[i], g_fwd[i], g_left[i], g_up[i],
                     v_front_sample[i], stopped, thr_pct[i])
        v_disp[i] = st.v

    # czas pierwszego zbocza (względem wyzwolenia; ujemny = silnik ruszył przed progiem)
    t_first_edge = float("nan")
    if h["n_edges"] > 0:
        t_first_edge = float(run.edges_t_s[0])
    elif i_edge0 < n:
        t_first_edge = float(t[i_edge0])

    series = dict(t=t, throttle_pct=thr_pct, v_front=v_front, v_imu=v_imu, v_disp=v_disp,
                  a_front=a_front, a_long=a_long, slip=slip, spin=spin, pitch_deg=np.degrees(pitch),
                  d_front=d_front, d_imu=d_imu)
    metrics = compute_metrics(series, i0, vmin, t_first_edge)
    return dict(series=series, metrics=metrics, warnings=warnings, at_rest=at_rest,
                header=h, used_raw_edges=use_edges, i0=i0)


def _window_mean(t, y, t0, t1):
    vals = [float(y[i]) for i in range(len(t)) if t0 <= t[i] < t1]
    return sum(vals) / len(vals) if vals else float("nan")


def compute_metrics(sr: dict, i0: int, vmin: float, t_first_edge: float) -> dict:
    """Metryki względem chwili startu t0 = t[i0]. esc_delay_s – od progu gazu (t = 0)."""
    t = sr["t"]
    ti = i0
    t0 = float(t[i0])
    n = len(t)
    peak_slip, peak_t = float("nan"), float("nan")
    for i in range(ti, n):
        if sr["v_front"][i] >= vmin and (math.isnan(peak_slip) or sr["slip"][i] > peak_slip):
            peak_slip, peak_t = float(sr["slip"][i]), float(t[i] - t0)
    spin_int = 0.0
    for i in range(ti + 1, n):
        x = sr["spin"][i]
        if x > 0:
            spin_int += x * (t[i] - t[i - 1])
    t1m = first_crossing(t, sr["d_imu"], 1.0, ti)
    t2m = first_crossing(t, sr["d_imu"], 2.0, ti)
    return dict(
        a_long_0_300ms=_window_mean(t, sr["a_long"], t0, t0 + 0.3),
        a_long_0_1000ms=_window_mean(t, sr["a_long"], t0, t0 + 1.0),
        v_imu_0_5s=interp_at(t, sr["v_imu"], t0 + 0.5),
        v_imu_1_0s=interp_at(t, sr["v_imu"], t0 + 1.0),
        v_max=float(max(sr["v_disp"])),
        t_1m=t1m - t0,
        t_2m=t2m - t0,
        peak_slip=peak_slip,
        peak_slip_t=peak_t,
        spin_integral=spin_int,
        esc_delay_s=t_first_edge,
        t_launch_s=t0,
    )


def analyze_file(path, cfg=None):
    return analyze(parse_run(open(path, "rb").read()), cfg)


def main(argv=None):
    import argparse
    import json

    ap = argparse.ArgumentParser(description="Metryki startu")
    ap.add_argument("files", nargs="+")
    ap.add_argument("--config")
    a = ap.parse_args(argv)
    cfg = load_config(a.config)
    for f in a.files:
        r = analyze_file(f, cfg)
        print(f, json.dumps({k: round(v, 4) for k, v in r["metrics"].items()}, ensure_ascii=False))
        for w in r["warnings"]:
            print("  !", w)


if __name__ == "__main__":
    main()
