import pathlib
import threading
import time

import numpy as np
import pytest

import simpel as sp

from test_circuit import build_buck

DATA = pathlib.Path(__file__).resolve().parents[2] / "data" / "thermal_models"


def test_progress_and_cancel_from_callback():
    seen = []

    def progress(t, t_stop):
        seen.append(t)
        return t < 2e-3  # False once past 2 ms: stop

    res = sp.Simulation(build_buck(), t_stop=1.0, dt=1e-7).run(progress)
    assert res.cancelled
    assert 2e-3 <= res.time[-1] < 0.5
    assert len(seen) >= 2 and seen == sorted(seen)
    assert all(len(res[n]) == len(res.time) for n in res.names)


def test_cancel_from_another_thread():
    sim = sp.Simulation(build_buck(), t_stop=1.0, dt=1e-7)
    threading.Timer(0.3, sim.cancel).start()
    t0 = time.perf_counter()
    res = sim.run()
    assert res.cancelled and time.perf_counter() - t0 < 5
    sim.settings["t_stop"] = 1e-3  # the token is reset for the next run
    res2 = sim.run(check=False)
    assert not res2.cancelled and res2.time[-1] == pytest.approx(1e-3, rel=1e-3)


def test_progress_exception_propagates():
    def boom(t, t_stop):
        raise RuntimeError("stop here")

    with pytest.raises(RuntimeError, match="stop here"):
        sp.Simulation(build_buck(), t_stop=0.2, dt=1e-7).run(boom)


def test_to_pandas():
    res = sp.Simulation(build_buck(), t_stop=1e-3, dt=1e-6).run()
    df = res.to_pandas(["V_C1", "I_L1"])
    assert list(df.columns) == ["V_C1", "I_L1"] and df.index.name == "time"
    assert len(df) == len(res)
    assert np.array_equal(df["V_C1"].to_numpy(), res["V_C1"])
    assert set(res.to_dict()) == {"time", *res.names}


def test_stats_matches_numpy_on_uniform_grid():
    res = sp.Simulation(build_buck(), t_stop=5e-3, dt=1e-6).run()
    s = res.stats("V_C1", t1=4e-3)
    y = res["V_C1"][res.time >= 4e-3]
    assert s.min == pytest.approx(y.min()) and s.max == pytest.approx(y.max())
    assert s.mean == pytest.approx(y.mean(), rel=1e-3)
    assert s.rms == pytest.approx(np.sqrt(np.mean(y ** 2)), rel=1e-3)
    with pytest.raises(ValueError):
        res.stats("V_C1", t1=4e-3, t2=3e-3)


def test_fourier_of_known_waveform():
    # A sine source drives a resistor: V_R1 is a clean 50 Hz sine of amplitude 10.
    c = sp.Circuit("ac")
    v = c.add("AC_V", "V1", amplitude=10, frequency=50)
    r = c.add("R", "R1", value=10)
    c.connect(v.A, r.A)
    c.ground(v.B, r.B)
    res = sp.Simulation(c, t_stop=0.1, dt=1e-5).run()
    f = res.fourier("V_R1", f0=50)
    assert f.f0 == pytest.approx(50) and f.fundamental == pytest.approx(10, rel=1e-3)
    assert f.thd < 1e-3 and abs(f.dc) < 1e-3
    f3 = res.fourier("V_R1", f0=50, cycles=3, max_harmonic=10)
    assert f3.fundamental == pytest.approx(10, rel=1e-3) and len(f3.orders) == 10
    assert f3.harmonic(1) == pytest.approx(10, rel=1e-3)


@pytest.mark.skipif(not DATA.is_dir(), reason="thermal models not present")
def test_losses():
    n = sp.set_loss_model_dir(DATA)
    assert n > 100
    models = sp.loss_models()
    mosfet = next(m for m in models if "mosfet" in m.lower() or m[0].isalpha())
    res = sp.Simulation(build_buck(), t_stop=5e-3, dt=1e-7).run()
    rep = res.losses(t_start=4e-3, duration=1e-3, models={"Q1": mosfet})
    q = rep["Q1"]
    assert q.total > 0 and q.total == pytest.approx(q.conduction + q.turn_on + q.turn_off, rel=1e-9)
    assert rep.total == pytest.approx(sum(d.total for d in rep.devices))
    assert rep.to_pandas().loc["Q1", "total"] == pytest.approx(q.total)
    with pytest.raises(KeyError):
        res.losses(models={"Q1": "no-such-part"})
