"""Dekodowanie formatu binarnego startu (docs/FORMAT.md, firmware/src/run_format.h).

Konfiguracja (tryb halla, zakresy IMU, neutrum...) czytana wyłącznie z nagłówka.
"""
from __future__ import annotations

import struct
import sys
from dataclasses import dataclass, field

import numpy as np

from .protocol import run_crc

RUN_MAGIC = 0x52463458
SUPPORTED_FORMAT = 1
HEADER_SIZE = 128

# Flagi próbki
SF_HALL_VALID = 0x01
SF_THROTTLE_VALID = 0x02
SF_DIR_FORWARD = 0x04
SF_DIR_VALID = 0x08
SF_IMU_OVERRUN = 0x10
SF_IMU_STALE = 0x20
SF_TRIGGER = 0x40

# Flagi startu
RF_EDGES_TRUNCATED = 0x0001
RF_TIME_SET = 0x0002
RF_NEUTRAL_LEARNED = 0x0004
RF_IMU_ERROR = 0x0008
RF_RAM_ONLY = 0x0010
RF_SIMULATED = 0x0020

EDGE_T_MASK = 0x1FFFFFFF

_HDR = struct.Struct("<IHHIIIIBBBBHHHHffHHIIIIIIHHIHHHBBI32sI")
assert _HDR.size == HEADER_SIZE

_HDR_FIELDS = (
    "magic format_version header_size seq unix_s boot_count uptime_ms hall_lines "
    "motor_pole_pairs edges_per_rev throttle_fwd_higher sample_rate_hz imu_odr_hz "
    "accel_range_g gyro_range_dps accel_scale_g gyro_scale_dps throttle_neutral_us "
    "throttle_full_us trigger_index n_samples n_edges flags t0_us trigger_t_us sample_size "
    "edge_size fw_version hall_min_edge_us pretrigger_ms posttrigger_ms trigger_pct "
    "hall_pullup_v_x10 hall_timeout_us reserved crc32"
).split()

SAMPLE_DTYPE = np.dtype(
    [
        ("t_us", "<u4"),
        ("hall_edges", "<u4"),
        ("period_rev_us", "<u4"),
        ("throttle_us", "<u2"),
        ("ax", "<i2"),
        ("ay", "<i2"),
        ("az", "<i2"),
        ("gx", "<i2"),
        ("gy", "<i2"),
        ("gz", "<i2"),
        ("flags", "u1"),
        ("reserved", "u1"),
    ]
)
assert SAMPLE_DTYPE.itemsize == 28


class FormatError(Exception):
    pass


@dataclass
class Run:
    header: dict
    samples: np.ndarray            # SAMPLE_DTYPE
    edges_t_us: np.ndarray         # czas zbocza względem t0_us [us], uint32
    edges_line: np.ndarray         # 0=A, 1=B, 2=C
    edges_level: np.ndarray        # poziom po zboczu
    raw: bytes = field(repr=False, default=b"")

    @property
    def t_s(self) -> np.ndarray:
        """Czas próbek względem próbki wyzwolenia [s]."""
        t = (self.samples["t_us"] - np.uint32(self.header["trigger_t_us"])).astype(np.int32)
        return t.astype(np.float64) * 1e-6

    @property
    def edges_t_s(self) -> np.ndarray:
        """Czas zboczy względem próbki wyzwolenia [s]."""
        trig_rel = (self.header["trigger_t_us"] - self.header["t0_us"]) & 0xFFFFFFFF
        return (self.edges_t_us.astype(np.float64) - trig_rel) * 1e-6


def parse_header(data: bytes) -> dict:
    if len(data) < HEADER_SIZE:
        raise FormatError("Za krótki nagłówek")
    h = dict(zip(_HDR_FIELDS, _HDR.unpack_from(data, 0)))
    del h["reserved"]
    if h["magic"] != RUN_MAGIC:
        raise FormatError(f"Zły magic 0x{h['magic']:08X}")
    if h["format_version"] != SUPPORTED_FORMAT:
        raise FormatError(f"Nieobsługiwana wersja formatu {h['format_version']}")
    if h["header_size"] != HEADER_SIZE or h["sample_size"] != 28 or h["edge_size"] != 4:
        raise FormatError("Niezgodne rozmiary struktur")
    return h


def parse_run(data: bytes, verify_crc: bool = True) -> Run:
    h = parse_header(data)
    ns, ne = h["n_samples"], h["n_edges"]
    need = HEADER_SIZE + ns * 28 + ne * 4
    if len(data) < need:
        raise FormatError(f"Za krótkie dane: {len(data)} < {need}")
    if verify_crc and run_crc(data[:need]) != h["crc32"]:
        raise FormatError("CRC startu niezgodne")
    samples = np.frombuffer(data, SAMPLE_DTYPE, ns, HEADER_SIZE).copy()
    words = np.frombuffer(data, "<u4", ne, HEADER_SIZE + ns * 28).copy()
    return Run(
        header=h,
        samples=samples,
        edges_t_us=(words & EDGE_T_MASK).astype(np.uint32),
        edges_line=(words >> 30).astype(np.uint8),
        edges_level=((words >> 29) & 1).astype(np.uint8),
        raw=bytes(data[:need]),
    )


def build_run(header: dict, samples: np.ndarray, edges_t_us=None, edges_line=None,
              edges_level=None) -> bytes:
    """Składa plik startu (testy, wektory testowe, DemoTransport). Liczy CRC."""
    h = dict(header)
    edges_t_us = np.asarray(edges_t_us if edges_t_us is not None else [], dtype=np.uint32)
    ne = len(edges_t_us)
    edges_line = np.asarray(edges_line if edges_line is not None else np.zeros(ne), dtype=np.uint32)
    edges_level = np.asarray(edges_level if edges_level is not None else np.zeros(ne), dtype=np.uint32)
    h.update(
        magic=RUN_MAGIC, format_version=SUPPORTED_FORMAT, header_size=HEADER_SIZE,
        n_samples=len(samples), n_edges=ne, sample_size=28, edge_size=4, crc32=0,
    )
    vals = [h.get(k, 0) if k != "reserved" else b"\0" * 32 for k in _HDR_FIELDS]
    hdr = bytearray(_HDR.pack(*vals))
    words = (edges_t_us & EDGE_T_MASK) | (edges_level << 29) | (edges_line << 30)
    body = np.asarray(samples, dtype=SAMPLE_DTYPE).tobytes() + words.astype("<u4").tobytes()
    data = bytes(hdr) + body
    crc = run_crc(data)
    struct.pack_into("<I", hdr, 124, crc)
    return bytes(hdr) + body


def main(argv=None):
    import argparse

    ap = argparse.ArgumentParser(description="Podgląd pliku startu .bin")
    ap.add_argument("files", nargs="+")
    a = ap.parse_args(argv)
    for f in a.files:
        r = parse_run(open(f, "rb").read())
        h = r.header
        print(f"{f}: seq={h['seq']} boot={h['boot_count']} unix={h['unix_s']} "
              f"hall_lines={h['hall_lines']} samples={h['n_samples']} edges={h['n_edges']} "
              f"trigger_index={h['trigger_index']} flags=0x{h['flags']:04X}")


if __name__ == "__main__":
    sys.exit(main())
