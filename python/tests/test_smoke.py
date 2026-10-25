import pathlib
import threading
import time

import numpy as np
import pytest

import simpel as sp

DESIGNS = pathlib.Path(__file__).resolve().parents[2] / "tests" / "golden" / "designs"
BUCK = DESIGNS / "Buck_converter.json"


def test_versions_agree():
    assert sp.core_version() == sp.__version__


def test_run_design_file_buck():
    r = sp.run_design_file(BUCK)
    t = r["time"]
    assert isinstance(t, np.ndarray) and t.dtype == np.float64
    assert len(t) == 1000  # 10 ms / 10 us, from the design's own settings
    assert r["t_stop"] == pytest.approx(0.01)
    v = r["signals"]["V_C1"]
    assert v.shape == t.shape
    assert 40 < v[-1] < 70


def _fnv1a(b: bytes) -> str:
    h = 1469598103934665603
    for x in b:
        h ^= x
        h = (h * 1099511628211) & 0xFFFFFFFFFFFFFFFF
    return f"{h:016x}"


def test_bitwise_equal_to_golden():
    # The Python path must produce exactly the waveforms recorded by the C++ golden
    # harness for the same design (FNV-1a over the raw float64 bytes).
    golden = DESIGNS.parent / "expected" / "Buck_converter.golden"
    expect = {}
    for line in golden.read_text().splitlines():
        parts = line.split()
        if parts[0] == "sig":
            expect[parts[1]] = parts[3]
        elif parts[0] == "time":
            expect["<time>"] = parts[2]
    r = sp.run_design_file(BUCK)
    got = {k.replace(" ", "~"): _fnv1a(v.tobytes()) for k, v in r["signals"].items()}
    got["<time>"] = _fnv1a(r["time"].tobytes())
    assert got == expect


def test_overrides():
    base = sp.run_design_file(BUCK, t_stop=1e-3)
    assert len(base["time"]) == 100
    alt = sp.run_design_file(BUCK, t_stop=1e-3, params={"L1.L": "500u"})
    assert not np.array_equal(base["signals"]["I_L1"], alt["signals"]["I_L1"])
    ad = sp.run_design_file(BUCK, t_stop=1e-3, step="adaptive")
    assert ad["step"] == "variable"


def test_errors():
    with pytest.raises(FileNotFoundError):
        sp.run_design_file("does/not/exist.json")
    with pytest.raises(ValueError):
        sp.run_design_file(BUCK, params={"NOPE.R": 1})
    with pytest.raises(ValueError):
        sp.run_design_file(BUCK, dt=-1)
    with pytest.raises(ValueError):
        sp.run_design_file(BUCK, step="weird")


def test_gil_released_during_run():
    # A Python thread must keep running while the simulation executes.
    ticks = []
    stop = threading.Event()

    def ticker():
        while not stop.is_set():
            ticks.append(time.perf_counter())
            time.sleep(0.001)

    th = threading.Thread(target=ticker)
    th.start()
    try:
        t0 = time.perf_counter()
        sp.run_design_file(DESIGNS / "boost closed loop.json", t_stop=5e-3)
        t1 = time.perf_counter()
    finally:
        stop.set()
        th.join()
    during = [x for x in ticks if t0 < x < t1]
    if t1 - t0 > 0.05:
        assert len(during) >= 5
