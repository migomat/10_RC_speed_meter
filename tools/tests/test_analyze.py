"""Testy implementacji referencyjnej na wspólnych wektorach testowych."""
import json
import math
import struct
from pathlib import Path

import numpy as np
import pytest

from rc_launch.analyze import DEFAULT_CONFIG, analyze, butter2_coeffs, filtfilt_butter2
from rc_launch.parse import FormatError, parse_run
from rc_launch.protocol import run_crc

VEC = Path(__file__).resolve().parents[2] / "test_vectors"
NAMES = json.loads((VEC / "index.json").read_text())


def load(name):
    data = (VEC / f"{name}.bin").read_bytes()
    exp = json.loads((VEC / f"{name}.expected.json").read_text())
    return data, exp


def close(a, b, rel=0.01, abs_=1e-3):
    if a is None or b is None or (isinstance(a, float) and math.isnan(a)):
        return (a is None or math.isnan(a)) and (b is None or (isinstance(b, float) and math.isnan(b)))
    return abs(a - b) <= max(abs_, rel * abs(b))


@pytest.mark.parametrize("name", NAMES)
def test_metrics_match_expected(name):
    data, exp = load(name)
    res = analyze(parse_run(data), exp["config"])
    for k, v in exp["metrics"].items():
        got = res["metrics"][k]
        got = None if isinstance(got, float) and math.isnan(got) else got
        assert close(got, v), f"{k}: {got} != {v}"
    for k, vals in exp["series"].items():
        for i, v in zip(exp["check_idx"], vals):
            g = float(res["series"][k][i])
            assert close(None if math.isnan(g) else g, v, abs_=2e-3), f"{k}[{i}]"


@pytest.mark.parametrize("name", NAMES)
def test_physics_sanity(name):
    data, exp = load(name)
    m, truth = exp["metrics"], exp["truth"]
    assert abs(m["v_imu_1_0s"] - truth["v_car_1_0s"]) <= 0.03 * max(1.0, truth["v_car_1_0s"])
    assert exp["at_rest"]
    if "lifted" in name:
        assert m["peak_slip"] > 0.95                   # koła w powietrzu → poślizg ≈ 100 %
        assert abs(m["v_imu_1_0s"]) < 0.1
    elif "slip" in name:
        assert m["peak_slip"] > 0.5
        assert m["spin_integral"] > 0.3
    else:
        assert m["peak_slip"] < 0.15
    assert m["esc_delay_s"] >= truth["motion_start_s"]


def test_crc_and_corruption():
    data, _ = load(NAMES[0])
    run = parse_run(data)
    assert run_crc(data) == run.header["crc32"]
    bad = bytearray(data)
    bad[500] ^= 0xFF
    with pytest.raises(FormatError):
        parse_run(bytes(bad))


def test_timestamp_wraparound():
    data, _ = load(NAMES[0])
    run = parse_run(data)
    t_us = run.samples["t_us"].astype(np.int64)
    assert (np.diff(t_us) < 0).any(), "wektor powinien zawierać przepełnienie licznika"
    assert np.allclose(np.diff(run.t_s), 0.002)


def test_filter_matches_scipy():
    ss = pytest.importorskip("scipy.signal")
    b, a = ss.butter(2, 25, fs=500)
    bb, aa = butter2_coeffs(25, 500)
    assert np.allclose(b, bb) and np.allclose(a[1:], aa)
    x = np.random.default_rng(1).normal(size=1000).cumsum()
    y = filtfilt_butter2(x, 25, 500)
    yr = ss.filtfilt(b, a, x)
    assert np.abs(y - yr)[100:-100].max() < 1e-9


def test_edges_vs_periods_consistent():
    """v_front z surowych zboczy ≈ v_front z period_rev_us (ten sam start)."""
    a = analyze(parse_run((VEC / "mode1_slip.bin").read_bytes()))
    b = analyze(parse_run((VEC / "mode1_slip_noedges.bin").read_bytes()))
    t = a["series"]["t"]
    sel = (t > 0.3) & (t < 2.5)
    va, vb = a["series"]["v_front"][sel], b["series"]["v_front"][sel]
    assert np.median(np.abs(va - vb) / np.maximum(vb, 0.5)) < 0.02


def test_estimator_does_not_follow_spinning_wheels():
    data, exp = load("mode1_slip")
    res = analyze(parse_run(data), exp["config"])
    s = res["series"]
    i = int(np.argmax(s["slip"]))
    assert s["v_front"][i] - s["v_disp"][i] > 1.0
    assert abs(s["v_disp"][i] - s["v_imu"][i]) < 0.3


def test_estimator_zero_at_rest():
    data, exp = load("mode3_grip")
    res = analyze(parse_run(data), exp["config"])
    s = res["series"]
    assert np.all(np.abs(s["v_disp"][(s["t"] > -0.25) & (s["t"] < 0)]) < 1e-9)


def test_build_roundtrip():
    from rc_launch.parse import build_run

    data, _ = load("mode3_slip")
    r = parse_run(data)
    again = build_run(r.header, r.samples, r.edges_t_us, r.edges_line, r.edges_level)
    assert again == data
