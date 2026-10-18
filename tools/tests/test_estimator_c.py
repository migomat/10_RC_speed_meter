"""Estymator z firmware (C, float32) vs implementacja referencyjna (Python)."""
import json
import shutil
import subprocess
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[2]
VEC = ROOT / "test_vectors"
NAMES = json.loads((VEC / "index.json").read_text())


@pytest.fixture(scope="module")
def est_bin(tmp_path_factory):
    cc = shutil.which("gcc") or shutil.which("cc")
    if not cc:
        pytest.skip("brak kompilatora C")
    out = tmp_path_factory.mktemp("est") / "est_host"
    subprocess.check_call([cc, "-O2", "-I", str(ROOT / "firmware/src"),
                           str(ROOT / "tools/est_host/est_host.c"),
                           str(ROOT / "firmware/src/estimator.c"), "-lm", "-o", str(out)])
    return out


@pytest.mark.parametrize("name", NAMES)
def test_c_estimator_matches_python(est_bin, name):
    exp = json.loads((VEC / f"{name}.expected.json").read_text())
    out = subprocess.check_output([str(est_bin), str(VEC / f"{name}.bin")], text=True)
    v_c = np.array([float(x) for x in out.split()])
    v_py = np.array(exp["v_disp_full"])
    assert len(v_c) == len(v_py)
    # float32 vs float64: różnice rzędu mm/s, tolerancja 1 % (min. 0,02 m/s)
    tol = np.maximum(0.02, 0.01 * np.abs(v_py))
    assert np.all(np.abs(v_c - v_py) <= tol), float(np.max(np.abs(v_c - v_py)))
