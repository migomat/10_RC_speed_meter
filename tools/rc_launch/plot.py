"""Wykresy pojedynczego startu (odpowiednik widoku „Start” w aplikacji) → PNG.

  python -m rc_launch.plot runs/xxx.bin [--range 1.0] [--out wykres.png]
"""
from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

from .analyze import analyze_file, load_config  # noqa: E402


def plot_run(path, cfg=None, t_range=None, out=None):
    res = analyze_file(path, cfg)
    s, m = res["series"], res["metrics"]
    t = s["t"]
    fig, ax = plt.subplots(5, 1, figsize=(10, 12), sharex=True,
                           gridspec_kw=dict(height_ratios=[1, 2, 2, 1.4, 1]))
    ax[0].plot(t, s["throttle_pct"], color="#f59f00"); ax[0].set_ylabel("gaz [%]")
    ax[1].plot(t, s["v_front"], label="v_front", color="#e8590c")
    ax[1].plot(t, s["v_imu"], label="v_imu", color="#2a7de1", lw=2)
    ax[1].plot(t, s["v_disp"], label="v_disp", color="#2f9e44", ls="--")
    ax[1].set_ylabel("v [m/s]"); ax[1].legend(loc="upper left")
    ax[2].plot(t, s["a_front"], label="a_front", color="#e8590c")
    ax[2].plot(t, s["a_long"], label="a_long", color="#2a7de1", lw=2)
    ax[2].set_ylabel("a [m/s²]"); ax[2].legend(loc="upper right")
    ax[3].plot(t, s["slip"] * 100, color="#d6336c"); ax[3].set_ylabel("poślizg [%]")
    ax[4].plot(t, s["pitch_deg"], color="#ae3ec9"); ax[4].set_ylabel("pitch [°]"); ax[4].set_xlabel("t [s] (0 = gaz > próg)")
    for a in ax:
        a.grid(alpha=0.3)
    t0 = m["t_launch_s"]
    if t_range:
        ax[0].set_xlim(t0 - 0.05, t0 + t_range)
    h = res["header"]
    fig.suptitle(f"{Path(path).name}  seq {h['seq']}  |  a0–0,3s {m['a_long_0_300ms']:.2f} m/s²  "
                 f"v1s {m['v_imu_1_0s']:.2f} m/s  poślizg {m['peak_slip'] * 100:.0f}%  "
                 f"ESC {m['esc_delay_s'] * 1000:.0f} ms", fontsize=10)
    fig.tight_layout()
    out = out or str(Path(path).with_suffix(".png"))
    fig.savefig(out, dpi=110)
    plt.close(fig)
    return out


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("files", nargs="+")
    ap.add_argument("--range", type=float, help="zakres [s] od startu")
    ap.add_argument("--out")
    ap.add_argument("--config")
    a = ap.parse_args(argv)
    cfg = load_config(a.config)
    for f in a.files:
        print(plot_run(f, cfg, a.range, a.out if len(a.files) == 1 else None))


if __name__ == "__main__":
    main()
