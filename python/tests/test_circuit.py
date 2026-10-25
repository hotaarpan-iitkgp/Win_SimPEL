import pathlib
import warnings

import numpy as np
import pytest

import simpel as sp

DESIGNS = pathlib.Path(__file__).resolve().parents[2] / "tests" / "golden" / "designs"


def build_buck(vin=48.0, duty=0.5):
    c = sp.Circuit("buck")
    v1 = c.add("V", "V1", value=vin)
    q1 = c.add("MOSFET", "Q1", Ron=10e-3)
    d1 = c.add("D", "D1")
    l1 = c.add("L", "L1", L="100u")
    c1 = c.add("C", "C1", C=100e-6)
    r1 = c.add("R", "R1", value=5)
    tri = c.add("TRI_GEN", "TRI1", frequency="20k")
    k = c.add("CONST", "K1", value=duty)
    cmp_ = c.add("COMP", "CMP1")
    c.connect(v1.A, q1.D)
    c.connect(q1.S, d1.B, l1.A)
    c.connect(l1.B, c1.A, r1.a)  # case-insensitive pins
    c.ground(v1.B, d1.A, c1.B, r1.B)
    c.connect(tri.Out, cmp_.In1)
    c.connect(cmp_.In2, k.Out)
    c.connect(cmp_.Out, "Q1.G")  # string pin form
    return c


def test_build_validate():
    c = build_buck()
    assert len(c) == 10 and "GND" in c
    assert c.validate() == []
    q1 = c["Q1"]
    assert q1.type == "MOSFET" and q1["Ron"] == "0.01"
    assert {p.name for p in q1.pins} == {"D", "S", "G"}
    assert q1.G.kind == "control" and str(q1.d) == "Q1.D"


def test_errors():
    c = sp.Circuit()
    with pytest.raises(sp.CircuitError, match="did you mean"):
        c.add("MOSFT")
    r = c.add("R", "R1")
    with pytest.raises(sp.CircuitError, match="already used"):
        c.add("R", "R1")
    with pytest.raises(AttributeError, match="pins: A, B"):
        r.Z
    with pytest.raises(KeyError):
        c["NOPE"]
    with pytest.raises(TypeError):
        c.connect(r.A, 42)
    with pytest.raises(sp.CircuitError):
        c.connect(r.A)


def test_ineffective_parameter_warns():
    c = sp.Circuit()
    with pytest.warns(sp.ParameterWarning, match="did you mean 'Ron'"):
        c.add("MOSFET", "Q1", Rn=5e-3)
    with warnings.catch_warnings():
        warnings.simplefilter("error")
        c.add("MOSFET", "Q2", Ron=5e-3)  # known parameter: no warning
    q = c["Q2"]
    with pytest.warns(sp.ParameterWarning):
        q["Rdson"] = 1


def test_validation_findings():
    c = sp.Circuit()
    c.add("CONST", "K1")
    c.add("CONST", "K2")
    g = c.add("GAIN", "G1")
    c.connect("K1.Out", "K2.Out", g.In)
    c.add("ZOH", "Z1")
    codes = {i.code for i in c.validate()}
    assert {"multiple_drivers", "unsupported_block"} <= codes
    with pytest.raises(sp.CircuitError, match="connected together"):
        c.validate(raise_on_error=True)
    with pytest.raises(sp.CircuitError):
        sp.Simulation(c, t_stop=1e-3).run()


def test_simulate_buck():
    sim = sp.Simulation(build_buck(), t_stop=20e-3, dt=1e-6)
    res = sim.run()
    assert not res.cancelled and len(res) == 20000
    vout = res.stats("V_C1", t1=18e-3)
    assert 20 < vout.mean < 26
    assert vout.pk2pk < 2
    assert res.step == "fixed" and res.t_stop == pytest.approx(20e-3)
    assert isinstance(res["V_C1"], np.ndarray) and res["V_C1"].base is not None  # shared, not copied
    with pytest.raises(KeyError, match="did you mean"):
        res["V_C"]


def test_adaptive_buck_close_to_fixed():
    c = build_buck()
    fixed = sp.Simulation(c, t_stop=10e-3, dt=1e-7).run().stats("V_C1", t1=9e-3).mean
    adaptive = sp.Simulation(c, t_stop=10e-3, dt=1e-7, step="adaptive").run()
    assert adaptive.step == "adaptive"
    assert len(adaptive) < 100000
    assert adaptive.stats("V_C1", t1=9e-3).mean == pytest.approx(fixed, rel=0.02)


def test_load_matches_design_file():
    path = DESIGNS / "Buck_converter.json"
    a = sp.Simulation(path).run(check=False)
    b = sp.run_design_file(path)
    assert a.netlist == b["netlist"]
    assert np.array_equal(a.time, b["time"])
    for k in b["signals"]:
        assert np.array_equal(a[k], b["signals"][k])
    c = sp.Circuit.load(path)
    assert c.settings["t_stop"] == pytest.approx(0.01)


def test_save_and_reload(tmp_path):
    c = build_buck()
    p = tmp_path / "buck.json"
    c.save(p)
    again = sp.Circuit.load(p)
    assert len(again) == len(c)
    r1 = sp.Simulation(c, t_stop=1e-3, dt=1e-6).run()
    r2 = sp.Simulation(again, t_stop=1e-3, dt=1e-6).run()
    assert r1.netlist == r2.netlist
    assert np.array_equal(r1["V_C1"], r2["V_C1"])
