import os
import time

import numpy as np
import pytest

import simpel as sp

from test_circuit import build_buck


def test_grid_sweep_matches_individual_runs():
    c = build_buck()
    params = {"L1.L": ["50u", "100u", "200u"], "R1.value": [2.5, 5, 10]}
    sw = sp.sweep(c, params, t_stop=2e-3, dt=1e-6, workers=4)
    assert len(sw) == 9 and not sw.failed
    assert sw[0].params == {"L1.L": "50u", "R1.value": 2.5}
    assert sw[8].params == {"L1.L": "200u", "R1.value": 10}
    for p in (sw[0], sw[4], sw[8]):
        cc = build_buck()
        for k, v in p.params.items():
            comp, _, name = k.partition(".")
            cc[comp][name] = v
        ref = sp.Simulation(cc, t_stop=2e-3, dt=1e-6).run()
        assert np.array_equal(ref["V_C1"], p.results["V_C1"])
        assert np.array_equal(ref.time, p.results.time)
    # The circuit itself is unchanged by the sweep.
    assert c["L1"]["L"] == "100u"


def test_zip_and_list_modes_and_table():
    c = build_buck()
    sw = sp.sweep(c, {"K1.value": [0.25, 0.5, 0.75], "L1.L": ["100u"] * 3}, mode="zip", t_stop=10e-3, dt=1e-6,
                  signals=["V_C1"])
    assert len(sw) == 3
    assert sw[0].results.names == ["V_C1"]  # only the kept signal
    df = sw.table(vout=lambda r: r.stats("V_C1", t1=8e-3).mean)
    assert list(df.columns) == ["K1.value", "L1.L", "vout"]
    v = df["vout"].to_numpy()
    # The comparator turns Q1 on while the carrier is above K1, so duty = 1 - K1:
    # the output falls as K1 rises (about 35 V, 23.6 V, 11.5 V from 48 V).
    assert v[0] > v[1] > v[2]
    sw2 = sp.sweep(c, [{"K1.value": 0.3}, {"K1.value": 0.6, "R1.value": 8}], t_stop=1e-3)
    assert len(sw2) == 2 and sw2.param_names == ["K1.value", "R1.value"]
    with pytest.raises(ValueError):
        sp.sweep(c, {"K1.value": [1, 2], "L1.L": ["1u"]}, mode="zip")


def test_sweep_validation():
    c = build_buck()
    with pytest.raises(sp.CircuitError, match="no component"):
        sp.sweep(c, {"NOPE.R": [1]})
    with pytest.raises(sp.CircuitError, match="no effect"):
        sp.sweep(c, {"Q1.Rdson": [1e-3, 2e-3]})  # misspelt: would give identical points
    with pytest.raises(ValueError, match="Component.param"):
        sp.sweep(c, {"Q1": [1]})


def test_sweep_cancel_via_progress():
    c = build_buck()
    seen = []

    def progress(done, total):
        seen.append(done)
        return done < 2

    sw = sp.sweep(c, {"L1.L": [f"{i}u" for i in range(50, 450, 10)]}, t_stop=20e-3, dt=1e-7, workers=2,
                  progress=progress)
    assert len(sw) == 40
    cancelled = [p for p in sw if p.error == "cancelled"]
    assert len(cancelled) >= 30 and seen[0] == 1


def test_sweep_is_faster_in_parallel():
    if (os.cpu_count() or 1) < 4:
        pytest.skip("needs 4+ CPU threads")
    c = build_buck()
    params = {"L1.L": [f"{i}u" for i in range(60, 300, 30)]}  # 8 points
    t0 = time.perf_counter()
    sp.sweep(c, params, t_stop=20e-3, dt=1e-7, workers=1, signals=["V_C1"])
    serial = time.perf_counter() - t0
    t0 = time.perf_counter()
    sp.sweep(c, params, t_stop=20e-3, dt=1e-7, workers=8, signals=["V_C1"])
    parallel = time.perf_counter() - t0
    print(f"serial {serial:.2f} s, parallel {parallel:.2f} s, speed-up {serial / parallel:.2f}x")
    assert parallel < serial / 1.8
