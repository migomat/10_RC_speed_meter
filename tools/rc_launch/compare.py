"""Porównanie startów / ustawień (odpowiednik widoku „Porównanie”) → PNG + tabela.

  python -m rc_launch.compare runs/*.bin [--out porownanie.png]
Grupowanie po tagu z pliku .json obok .bin (z download.py); bez tagu – nazwa pliku.
"""
from __future__ import annotations

import argparse
import json
import math
from collections import defaultdict
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from .analyze import analyze_file, load_config  # noqa: E402

KEYS = ["a_long_0_300ms", "a_long_0_1000ms", "v_imu_1_0s", "t_2m", "peak_slip", "spin_integral", "v_max"]


def tag_of(path: Path) -> str:
    j = path.with_suffix(".json")
    if j.exists():
        try:
            return json.loads(j.read_text()).get("tag") or "(bez tagu)"
        except Exception:
            pass
    return "(bez tagu)"


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("files", nargs="+")
    ap.add_argument("--out", default="porownanie.png")
    ap.add_argument("--config")
    a = ap.parse_args(argv)
    cfg = load_config(a.config)
    groups = defaultdict(list)
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))
    colors = plt.rcParams["axes.prop_cycle"].by_key()["color"]
    tags = []
    for f in map(Path, a.files):
        res = analyze_file(f, cfg)
        tag = tag_of(f)
        if tag not in tags:
            tags.append(tag)
        c = colors[tags.index(tag) % len(colors)]
        s, m = res["series"], res["metrics"]
        t0 = m["t_launch_s"]
        ax1.plot(s["t"] - t0, s["v_imu"], color=c, alpha=0.8, label=tag if len(groups[tag]) == 0 else None)
        ax2.scatter(m["peak_slip"] * 100, m["a_long_0_300ms"], color=c)
        groups[tag].append(m)
    ax1.set_xlim(-0.1, 3); ax1.set_xlabel("t od startu [s]"); ax1.set_ylabel("v_imu [m/s]"); ax1.legend(); ax1.grid(alpha=.3)
    ax2.set_xlabel("szczyt poślizgu [%]"); ax2.set_ylabel("a 0–0,3 s [m/s²]"); ax2.grid(alpha=.3)
    fig.tight_layout()
    fig.savefig(a.out, dpi=110)
    print(f"Wykres: {a.out}\n")
    print(f"{'tag':16s} {'n':>3s} " + " ".join(f"{k:>18s}" for k in KEYS))
    for tag, ms in groups.items():
        cells = []
        for k in KEYS:
            v = np.array([x[k] for x in ms if not math.isnan(x[k])])
            cells.append(f"{v.mean():8.3f} ± {v.std():6.3f}" if len(v) else f"{'–':>18s}")
        print(f"{tag:16s} {len(ms):3d} " + " ".join(cells))


if __name__ == "__main__":
    main()
