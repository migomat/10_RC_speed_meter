"""Pobieranie startów z Xiao (BLE albo USB) i import eksportu z aplikacji.

  python -m rc_launch.download [--usb [PORT]] [--keep] [--tag TAG]   # pobierz wszystko
  python -m rc_launch.download --import x4f_baza_2026-10-18.json     # eksport z telefonu
  python -m rc_launch.download --live [--usb]                        # podgląd LIVE

Start jest kasowany z Xiao (DEL) dopiero po weryfikacji CRC i zapisie pliku
na dysk; --keep wyłącza kasowanie.
"""
from __future__ import annotations

import argparse
import base64
import json
import os
import re
import sys
import time
from datetime import datetime
from pathlib import Path

from .parse import parse_run
from .protocol import BleLink, Device, SerialLink

RUNS_DIR = Path(__file__).resolve().parents[1] / "runs"


def run_filename(h: dict, tag: str = "", approx_unix: int = 0) -> str:
    u = h["unix_s"] or approx_unix
    stamp = datetime.fromtimestamp(u).strftime("%Y%m%d_%H%M%S") if u else f"boot{h['boot_count']}_{h['uptime_ms'] // 1000:06d}s"
    safe = re.sub(r"[^\w-]+", "_", tag).strip("_")
    return f"{stamp}_seq{h['seq']}" + (f"_{safe}" if safe else "")


def save_run(data: bytes, meta: dict, out: Path, tag: str = "") -> Path:
    run = parse_run(data)                     # weryfikacja CRC i formatu
    base = out / run_filename(run.header, tag, meta.get("approx_unix", 0))
    tmp = base.with_suffix(".bin.tmp")
    tmp.write_bytes(data)
    with open(tmp, "rb+") as f:
        os.fsync(f.fileno())
    tmp.rename(base.with_suffix(".bin"))
    info = dict(meta, tag=tag, header={k: v for k, v in run.header.items()})
    base.with_suffix(".json").write_text(json.dumps(info, indent=1, ensure_ascii=False))
    return base.with_suffix(".bin")


def download_all(dev: Device, out: Path, keep: bool, tag: str):
    out.mkdir(parents=True, exist_ok=True)
    dev.set_time()
    st = dev.status()
    if st.get("overwritten_unread"):
        print(f"UWAGA: {st['overwritten_unread']} startów nadpisanych przed pobraniem")
    runs = dev.list_runs()
    print(f"Na urządzeniu: {len(runs)} startów")
    for r in runs:
        t0 = time.time()
        data = dev.download(r, progress=lambda g, n: print(f"\r  seq {r['seq']}: {g}/{n} B", end=""))
        dt = time.time() - t0
        approx = 0
        if not r["unix"] and r["boot"] == st["boot"]:
            approx = int(time.time() - (st["uptime_ms"] - r["uptime_ms"]) / 1000)
        path = save_run(data, dict(r, approx_unix=approx, downloaded_at=time.time()), out, tag)
        print(f"\r  seq {r['seq']}: {len(data)} B, {len(data) / dt / 1024:.1f} kB/s → {path.name}")
        if not keep:
            dev.delete(r["seq"])


def import_export(path: Path, out: Path):
    obj = json.loads(path.read_text())
    out.mkdir(parents=True, exist_ok=True)
    n = 0
    for r in obj.get("runs", []):
        data = base64.b64decode(r["raw_b64"])
        meta = {k: v for k, v in r.items() if k not in ("raw_b64", "raw")}
        p = save_run(data, meta, out, r.get("tag", ""))
        if r.get("note"):
            print(f"  {p.name}: {r['note']}")
        n += 1
    print(f"Zaimportowano {n} startów do {out}")


def live(dev: Device):
    dev.command("LIVE 1")
    try:
        while True:
            m = dev.link.demux.live.get()
            print(f"\rv={m['v'] * 3.6:5.1f} km/h  vmax={m['v_max'] * 3.6:5.1f}  koła={m['v_front'] * 3.6:5.1f}  "
                  f"rpm={m['rpm']:6.0f}  gaz={m['thr']:4d}  {'BUKSUJE' if m['slip'] else '       '}  {m['state']:9s}", end="")
    except KeyboardInterrupt:
        pass
    finally:
        dev.command("LIVE 0")
        print()


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--usb", nargs="?", const="auto", help="USB zamiast BLE (opcjonalnie port)")
    ap.add_argument("--keep", action="store_true", help="nie kasuj startów z Xiao")
    ap.add_argument("--tag", default="")
    ap.add_argument("--out", default=str(RUNS_DIR))
    ap.add_argument("--import", dest="imp", help="plik eksportu z aplikacji (JSON)")
    ap.add_argument("--live", action="store_true")
    a = ap.parse_args(argv)
    out = Path(a.out)
    if a.imp:
        import_export(Path(a.imp), out)
        return 0
    link = SerialLink(None if a.usb == "auto" else a.usb) if a.usb else BleLink("X4F")
    dev = Device(link)
    try:
        if a.live:
            live(dev)
        else:
            download_all(dev, out, a.keep, a.tag)
    finally:
        link.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
