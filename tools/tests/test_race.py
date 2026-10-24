"""Wyścigi: format pliku (parse.py) i analiza (analyze_race) na wektorach testowych."""
import json
import math
import struct
from pathlib import Path

import numpy as np
import pytest

from rc_launch.analyze import analyze_race
from rc_launch.parse import (RACE_SAMPLE_DTYPE, FormatError, build_race, file_kind, parse_any,
                             parse_race)

VEC = Path(__file__).resolve().parents[2] / "test_vectors"
RACES = json.loads((VEC / "races.json").read_text())


def load(name):
    return (VEC / f"{name}.bin").read_bytes(), json.loads((VEC / f"{name}.expected.json").read_text())


def close(a, b, rel=0.01, abs_=1e-3):
    if a is None or b is None:
        return a is None and b is None
    return abs(a - b) <= max(abs_, rel * abs(b))


@pytest.mark.parametrize("name", RACES)
def test_race_matches_expected(name):
    data, exp = load(name)
    res = analyze_race(parse_race(data), exp["config"])
    for k, v in exp["metrics"].items():
        assert close(float(res["metrics"][k]), v), f"{k}: {res['metrics'][k]} != {v}"
    for k, vals in exp["series"].items():
        for i, v in zip(exp["check_idx"], vals):
            assert close(float(res["series"][k][i]), v), f"{k}[{i}]"
    assert res["end_reason"] == exp["end_reason"]


@pytest.mark.parametrize("name", RACES)
def test_race_physics_sanity(name):
    data, exp = load(name)
    m = analyze_race(parse_race(data), exp["config"])["metrics"]
    tr = exp["truth"]
    # droga z licznika zboczy (po przekroczeniu mety jeszcze hamowanie)
    assert tr["distance_m"] <= m["distance_m"] < tr["distance_m"] + 3.0
    # kurs z żyroskopu po korekcie biasu z postoju: N × 360°
    assert abs(m["heading_total_deg"] - tr["heading_total_deg"]) < 10.0


def test_race_kind_crc_and_padding():
    data, _ = load("race_recovered")
    assert file_kind(data) == "race"
    r = parse_any(data)
    assert r.end_reason == 2 and r.n_samples > 0
    # wypełnienie między próbkami a trailerem (domknięcie po zaniku zasilania)
    assert len(data) > 128 + r.n_samples * 20 + 16
    bad = bytearray(data)
    bad[200] ^= 1
    with pytest.raises(FormatError):
        parse_race(bytes(bad))


def test_race_time_from_ticks_and_gap():
    data, _ = load("race_recovered")
    r = parse_race(data)
    t = r.t_s
    dt = np.diff(t)
    gap = np.nonzero(r.samples["flags"] & 0x40)[0]
    assert len(gap) == 1
    assert dt[gap[0] - 1] == pytest.approx(30 / 500)     # n_ticks = 30 przy przerwie
    assert t[r.header["trigger_index"]] == 0.0


def test_race_build_roundtrip():
    s = np.zeros(3, RACE_SAMPLE_DTYPE)
    s["n_ticks"] = 10
    s["edges"] = [0, 5, 7]
    d = build_race(dict(seq=9, tick_rate_hz=500, sample_rate_hz=50, period_unit_us=4), s, end_reason=1)
    r = parse_race(d)
    assert r.header["seq"] == 9 and r.end_reason == 1
    assert list(r.samples["edges"]) == [0, 5, 7]
    assert struct.unpack_from("<I", d, len(d) - 4)[0] == r.crc32
