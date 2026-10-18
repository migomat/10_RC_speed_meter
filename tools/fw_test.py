#!/usr/bin/env python3
"""Testy firmware na biurku (Xiao przez USB, bez auta) – komendy SIM_*.

Użycie:  python tools/fw_test.py [--port /dev/ttyACM0] [testy...]
Testy: basic imu throttle hall logger flash20 powerloss del get
Uwaga: SIM generuje zbocza/impulsy programowo (wstrzykiwane do tych samych
funkcji co ISR) – sprawdza algorytmy, logger, flash i protokół, ale nie tor
sprzętowy PPI/GPIOTE (do tego potrzebny symulator ESP32 albo auto).
"""
from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from rc_launch.parse import parse_run, SF_TRIGGER  # noqa: E402
from rc_launch.protocol import Device, SerialLink, run_crc  # noqa: E402

RESULTS = []


def check(name, ok, detail=""):
    RESULTS.append((name, ok, detail))
    print(f"[{'OK' if ok else 'FAIL'}] {name} {detail}")
    return ok


def connect(port, wait=8.0):
    t_end = time.time() + wait
    while True:
        try:
            link = SerialLink(port)
            dev = Device(link)
            dev.info()
            return link, dev
        except Exception:
            if time.time() > t_end:
                raise
            time.sleep(0.5)


def wait_state(dev, state, timeout):
    t_end = time.time() + timeout
    while time.time() < t_end:
        st = dev.status()
        if st["state"] == state:
            return st
        time.sleep(0.1)
    return dev.status()


def learn_neutral(dev):
    dev.command("SIM THR 1500")
    t_end = time.time() + 6
    while time.time() < t_end:
        st = dev.status()
        if st["neutral_learned"]:
            return st
        time.sleep(0.2)
    return dev.status()


def do_launch(dev, rpm=20000, ramp_ms=1500):
    """Gaz 2000 us + rampa obrotów; czeka na koniec zapisu."""
    dev.command("SIM THR 1500")
    time.sleep(0.7)            # pre-trigger + uzbrojenie (gaz poniżej progu)
    dev.command(f"SIM RAMP {rpm} {ramp_ms}")
    dev.command("SIM THR 2000")
    st = wait_state(dev, "RECORDING", 2)
    rec = st["state"] == "RECORDING"
    st = wait_state(dev, "IDLE", 8)
    dev.command("SIM RPM 0")
    dev.command("SIM THR 1500")
    return rec, st


def t_basic(dev, a):
    info = dev.info()
    check("INFO", info["fw"] and info["jedec"] == "856015", str(info))
    st = dev.status()
    check("STATUS imu/flash", st["imu_ok"] and st["flash_ok"], f"state={st['state']}")


def t_imu(dev, a):
    st0 = dev.status()
    time.sleep(a.imu_seconds)
    st1 = dev.status()
    n = st1["imu_samples"] - st0["imu_samples"]
    rate = n / ((st1["uptime_ms"] - st0["uptime_ms"]) / 1000)
    check("IMU rate ~833 Hz", abs(rate - 833) < 15, f"{rate:.1f} Hz")
    check("IMU overrun = 0", st1["imu_overruns"] == st0["imu_overruns"],
          f"{st1['imu_overruns'] - st0['imu_overruns']} w {a.imu_seconds} s")
    d = dev.command("DBG")
    g = math.sqrt(d["ax"] ** 2 + d["ay"] ** 2 + d["az"] ** 2) * 0.122e-3
    check("|a| ≈ 1 g", abs(g - 1) < 0.05, f"{g:.3f} g")
    gyro = max(abs(d["gx"]), abs(d["gy"]), abs(d["gz"])) * 17.5e-3
    print(f"   gyro bias max {gyro:.2f} dps")


def t_throttle(dev, a):
    st = learn_neutral(dev)
    check("neutrum nauczone", st["neutral_learned"] and abs(st["neutral_us"] - 1500) <= 2,
          f"{st['neutral_us']} us")
    for w in (1100, 1720, 1950):
        dev.command(f"SIM THR {w}")
        time.sleep(0.15)
        st = dev.status()
        check(f"gaz {w} us", abs(st["throttle_us"] - w) <= 2, f"{st['throttle_us']}")
    dev.command("SIM THR 0")
    time.sleep(0.25)
    st = dev.status()
    check("brak sygnału po 100 ms", not st["throttle_ok"])
    dev.command("SIM THR 1500")


def t_hall(dev, a):
    for rpm in (500, 3000, 12000, 30000):
        dev.command(f"SIM RPM {rpm}")
        time.sleep(0.4)
        vals = [dev.command("DBG")["rpm"] for _ in range(5)]
        m = sum(vals) / len(vals)
        check(f"rpm {rpm}", abs(m - rpm) / rpm < 0.005, f"{m:.1f}")
    st0 = dev.status()
    dev.command("SIM RPM 3000")
    time.sleep(0.2)
    e0 = dev.status()["hall_edges"]
    dev.command("SIM GLITCH 10")
    time.sleep(0.2)
    st1 = dev.status()
    rpm_after = dev.command("DBG")["rpm"]
    check("szpilka 10 us odrzucona", st1["hall_glitches"] > st0["hall_glitches"],
          f"glitches +{st1['hall_glitches'] - st0['hall_glitches']}, rpm={rpm_after:.0f}")
    dev.command("SIM RPM 0")
    time.sleep(0.2)
    check("rpm → 0 po zatrzymaniu", dev.command("DBG")["rpm"] == 0)


def latest_run(dev):
    runs = dev.list_runs()
    return runs[-1] if runs else None


def t_logger(dev, a):
    learn_neutral(dev)
    before = {r["seq"] for r in dev.list_runs()}
    rec, st = do_launch(dev)
    check("wyzwolenie → RECORDING", rec)
    check("zapis → IDLE", st["state"] == "IDLE", f"save {st['last_save_ms']} ms, crc {st['last_crc_ms']} ms")
    check("SAVING < 1 s", 0 < st["last_save_ms"] < 1000, f"{st['last_save_ms']} ms")
    run = latest_run(dev)
    check("nowy start na liście", run and run["seq"] not in before, str(run))
    t0 = time.time()
    data = dev.download(run)
    dt = time.time() - t0
    r = parse_run(data)
    h = r.header
    check("liczba próbek", h["n_samples"] == 1750 and h["trigger_index"] == 250,
          f"n={h['n_samples']} trig={h['trigger_index']}")
    check("flaga wyzwolenia", r.samples["flags"][250] & SF_TRIGGER != 0)
    dt_us = (r.samples["t_us"][1:] - r.samples["t_us"][:-1]).astype("int64")
    check("odstęp próbek 2000 us", (dt_us == 2000).all(), f"min {dt_us.min()} max {dt_us.max()}")
    check("zbocza zapisane", h["n_edges"] > 100, f"{h['n_edges']} zboczy")
    print(f"   pobieranie USB: {len(data)} B w {dt:.2f} s = {len(data) / dt / 1024:.1f} kB/s")
    Path(a.out).mkdir(exist_ok=True)
    (Path(a.out) / f"fwtest_seq{h['seq']}.bin").write_bytes(data)


def t_flash20(dev, a):
    learn_neutral(dev)
    for r in dev.list_runs():
        dev.delete(r["seq"])
    time.sleep(0.5)
    ow0 = dev.status()["overwritten_unread"]
    seqs = []
    for i in range(20):
        rec, st = do_launch(dev, rpm=8000, ramp_ms=800)
        seqs.append(latest_run(dev)["seq"])
        print(f"   start {i + 1}/20 seq={seqs[-1]} save={st['last_save_ms']} ms")
    runs = dev.list_runs()
    st = dev.status()
    check("16 startów w pamięci", len(runs) == 16, f"{len(runs)}")
    check("16 najnowszych", [r["seq"] for r in runs] == seqs[-16:])
    check("overwritten_unread = 4", st["overwritten_unread"] - ow0 == 4, f"{st['overwritten_unread'] - ow0}")


def t_powerloss(dev, a, port):
    learn_neutral(dev)
    runs_before = dev.list_runs()
    dev.command("SIM RESETAT 30000")
    try:
        do_launch(dev)
    except Exception:
        pass
    dev.link.close()
    time.sleep(3)
    link, dev2 = connect(port, 15)
    runs_after = dev2.list_runs()
    st = dev2.status()
    before = {(r["seq"], r["crc"]) for r in runs_before}
    after = {(r["seq"], r["crc"]) for r in runs_after}
    check("reset w trakcie SAVING", st["uptime_ms"] < 15000, f"uptime {st['uptime_ms']} ms")
    check("brak uszkodzonego startu", after <= before, f"{len(before)} → {len(after)}")
    lost = before - after
    check("pozostałe nietknięte", len(lost) <= 1, f"utracone (nadpisany najstarszy): {lost}")
    ok = all(run_crc(dev2.download(r)) == r["crc"] for r in runs_after[-2:])
    check("pozostałe czytelne (CRC)", ok)
    return link, dev2


def t_del(dev, a, port):
    runs = dev.list_runs()
    if not runs:
        learn_neutral(dev)
        do_launch(dev)
        runs = dev.list_runs()
    seq = runs[-1]["seq"]
    free0 = dev.status()["free_slots"]
    dev.delete(seq)
    time.sleep(0.3)
    st = dev.status()
    check("DEL → slot zwolniony", st["free_slots"] == free0 + 1, f"{free0} → {st['free_slots']}")
    dev.command("REBOOT")
    dev.link.close()
    time.sleep(3)
    link, dev2 = connect(port, 15)
    check("po restarcie start nie wraca", seq not in [r["seq"] for r in dev2.list_runs()])
    return link, dev2


def t_get(dev, a):
    runs = dev.list_runs()
    if not runs:
        learn_neutral(dev)
        do_launch(dev)
        runs = dev.list_runs()
    run = runs[-1]
    # wznawianie: pobranie części, „rozłączenie”, kontynuacja od offsetu
    first = b"".join(dev.get_block(run["seq"], o, 4096) for o in (0, 4096))
    full = dev.download(run, resume=first)
    check("wznowienie od offsetu", run_crc(full) == run["crc"])
    try:
        dev.command("GET 99999 0 100")
        check("GET nieistniejącego → błąd", False)
    except Exception as e:
        check("GET nieistniejącego → błąd", "NOT_FOUND" in str(e))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--imu-seconds", type=float, default=10)
    ap.add_argument("--out", default="tools/runs")
    ap.add_argument("tests", nargs="*", default=["basic", "imu", "throttle", "hall", "logger", "get"])
    a = ap.parse_args()
    link, dev = connect(a.port)
    for t in a.tests:
        print(f"== {t}")
        if t == "powerloss":
            link, dev = t_powerloss(dev, a, a.port)
        elif t == "del":
            link, dev = t_del(dev, a, a.port)
        else:
            globals()[f"t_{t}"](dev, a)
    dev.command("SIM OFF")
    link.close()
    bad = [r for r in RESULTS if not r[1]]
    print(f"\n{len(RESULTS) - len(bad)}/{len(RESULTS)} OK")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
