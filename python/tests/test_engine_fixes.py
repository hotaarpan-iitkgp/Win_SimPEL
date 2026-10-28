"""Regression tests for engine bugs found while building the converter models.

Each test fails on the engine before the corresponding fix."""

import json
import math

import numpy as np
import pytest

import simpel as sp


def _run(c, **kw):
    return sp.Simulation(c, **kw).run()


# ── MOSFET body diode ────────────────────────────────────────────────────────
def test_mosfet_body_diode_conducts():
    # 10 V through 1 ohm into the source of an OFF MOSFET whose drain is grounded:
    # the body diode must carry (10 - Vd)/(1 + Ron) from S to D.
    c = sp.Circuit("bd")
    v = c.add("V", "V1", value=10)
    r = c.add("R", "R1", value=1)
    q = c.add("MOSFET", "Q1", Ron=10e-3)
    k = c.add("CONST", "K1", value=0)
    c.connect(v.A, r.A)
    c.connect(r.B, q.S)
    c.ground(v.B, q.D)
    c.connect(k.Out, q.G)
    res = _run(c, t_stop=1e-4, dt=1e-6)
    i_expected = (10 - 0.8) / (1 + 10e-3)
    assert res.stats("I_Q1", 5e-5).mean == pytest.approx(-i_expected, rel=1e-3)
    assert res.stats("V_Q1", 5e-5).mean == pytest.approx(-(0.8 + 10e-3 * i_expected), rel=1e-3)


def test_mosfet_body_diode_blocks_forward():
    # Gate off, drain positive: the body diode is reverse biased and blocks.
    c = sp.Circuit("bd2")
    v = c.add("V", "V1", value=10)
    r = c.add("R", "R1", value=1)
    q = c.add("MOSFET", "Q1")
    k = c.add("CONST", "K1", value=0)
    c.connect(v.A, r.A)
    c.connect(r.B, q.D)
    c.ground(v.B, q.S)
    c.connect(k.Out, q.G)
    res = _run(c, t_stop=1e-4, dt=1e-6)
    assert abs(res.stats("I_Q1", 5e-5).mean) < 1e-4


# ── Transformer magnetising inductance ───────────────────────────────────────
@pytest.mark.parametrize("kind", ["XFMR"])
def test_xfmr_magnetising_inductance(kind):
    lm, f, vpk = 2e-3, 1e3, 100.0
    c = sp.Circuit("lm")
    v = c.add("AC_V", "V1", amplitude=vpk, frequency=f)
    x = c.add(kind, "T1", primary_turns="[10]", secondary_turns="[20]", Lm=lm, polarity="non_inverted")
    r = c.add("R", "R1", value=1e6)
    c.connect(v.A, x.P1A)
    c.connect(x.S1A, r.A)
    c.ground(v.B, x.P1B, x.S1B, r.B)
    res = _run(c, t_stop=6e-3, dt=1e-6)
    im = res.fourier("I_T1_Lm", f0=f).fundamental
    assert im == pytest.approx(vpk / (2 * math.pi * f * lm), rel=0.01)
    # Unloaded secondary: the source supplies the magnetising current only.
    assert res.fourier("I_V1", f0=f).fundamental == pytest.approx(im, rel=0.01)
    assert res.fourier("V_R1", f0=f).fundamental == pytest.approx(2 * vpk, rel=1e-3)


def test_xfmr_without_lm_is_ideal():
    c = sp.Circuit("ideal")
    v = c.add("AC_V", "V1", amplitude=100, frequency=1e3)
    x = c.add("XFMR", "T1", primary_turns="[10]", secondary_turns="[10]", Lm=0, polarity="non_inverted")
    r = c.add("R", "R1", value=1e6)
    c.connect(v.A, x.P1A)
    c.connect(x.S1A, r.A)
    c.ground(v.B, x.P1B, x.S1B, r.B)
    res = _run(c, t_stop=3e-3, dt=1e-6)
    assert "I_T1_Lm" not in res
    assert res.stats("I_V1", 1e-3).abs_max < 1e-3


# ── CONT_PID limits, anti-windup and gain precision ──────────────────────────
def _pid_circuit(**pid):
    c = sp.Circuit("pid")
    s = c.add("STEP", "E", step_time=0.1, initial_value=1, final_value=-1)
    p = c.add("CONT_PID", "PID", Kp=0, Kd=0, **pid)
    c.connect(s.Out, p.In)
    return c


def test_cont_pid_output_limit():
    res = _run(_pid_circuit(Ki=100, limit_output="true", upper_limit=0.5, lower_limit=-0.5), t_stop=0.1, dt=1e-4)
    out = res["PID.Out"]
    assert out.max() == pytest.approx(0.5) and out.min() >= -0.5 - 1e-12


def test_cont_pid_anti_windup():
    common = dict(Ki=100, limit_output="true", upper_limit=0.5, lower_limit=-0.5)
    wound = _run(_pid_circuit(**common), t_stop=0.15, dt=1e-4)
    held = _run(_pid_circuit(anti_windup="true", **common), t_stop=0.15, dt=1e-4)
    t = wound.time
    after = (t > 0.11) & (t < 0.12)
    # Without anti-windup the integrator reached ~10 and the output stays clamped.
    assert np.all(wound["PID.Out"][after] == pytest.approx(0.5))
    # With clamping it leaves the limit as soon as the error changes sign.
    assert np.all(held["PID.Out"][after] < 0.5)
    assert held.stats("PID.Out", 0.149).mean == pytest.approx(-0.5)


def test_cont_pid_small_gain_kept():
    # Ki = 5e-7 used to be truncated to six decimals (0) on its way to the solver.
    c = sp.Circuit("ki")
    k = c.add("CONST", "K", value=1)
    p = c.add("CONT_PID", "PID", Kp=0, Ki=5e-7, Kd=0)
    c.connect(k.Out, p.In)
    res = _run(c, t_stop=0.1, dt=1e-4)
    assert res["PID.Out"][-1] == pytest.approx(5e-8, rel=1e-2)


# ── Control-input resolution ─────────────────────────────────────────────────
def test_signal_switch_reads_its_own_ctrl_pin():
    # The In1 wire is made first. Ctrl used to resolve to it (alias "Ctrl" ~ "In1").
    c = sp.Circuit("ss")
    a = c.add("CONST", "A", value=3)
    k = c.add("CONST", "K", value=0)
    b = c.add("CONST", "B", value=7)
    s = c.add("SIGNAL_SWITCH", "SW", threshold=0.5, criteria="u2 >= threshold")
    c.connect(a.Out, s.In1)
    c.connect(k.Out, s.Ctrl)
    c.connect(b.Out, s.In2)
    res = _run(c, t_stop=1e-4, dt=1e-5)
    assert res["SW.Out"][-1] == pytest.approx(7)


def _design(wires, comps):
    return {"components": comps, "wires": wires,
            "simulationSettings": {"stopTime": "1e-4", "stepSize": "1e-6", "solver": "euler", "stepType": "fixed"}}


def _comp(id_, type_, **params):
    return {"id": id_, "type": type_, "x": 0, "y": 0, "rotation": 0, "parameters": {k: str(v) for k, v in params.items()}}


def _w(i, a, b):
    ca, ta = a.split(".")
    cb, tb = b.split(".")
    return {"id": f"w{i}", "from": {"type": "pin", "compId": ca, "terminal": ta},
            "to": {"type": "pin", "compId": cb, "terminal": tb}}


def test_input_reads_driver_through_input_chain():
    # K -> G1.In, then G1.In -> G2.In (input to input). G2 must read K.Out, not "G1.In".
    comps = [_comp("K", "CONST", value=2), _comp("G1", "GAIN", K=1), _comp("G2", "GAIN", K=3)]
    d = _design([_w(1, "K.Out", "G1.In"), _w(2, "G1.In", "G2.In")], comps)
    res = sp.Simulation(_load(d)).run(check=False)
    assert res["G2.Out"][-1] == pytest.approx(6)


# ── Ground detection ─────────────────────────────────────────────────────────
def _load(design, tmp=None):
    import os
    import tempfile
    fd, p = tempfile.mkstemp(suffix=".json")
    os.close(fd)
    with open(p, "w", encoding="utf-8") as f:
        json.dump(design, f)
    try:
        return sp.Circuit.load(p)
    finally:
        os.remove(p)


def _divider(gnd_wire, gnd_term="Gnd"):
    # V1 (10 V) across R1 + R2; ground at the R1/R2 midpoint, not at V1.B.
    comps = [_comp("V1", "V", value=10), _comp("R1", "R", value=1), _comp("R2", "R", value=1), _comp("GND1", "GND")]
    wires = [_w(1, "V1.A", "R1.A"), _w(2, "R1.B", "R2.A"), _w(3, "R2.B", "V1.B")]
    a, b = ("R1.B", f"GND1.{gnd_term}") if gnd_wire == "to_gnd" else (f"GND1.{gnd_term}", "R1.B")
    wires.append(_w(4, a, b))
    return _design(wires, comps)


@pytest.mark.parametrize("direction", ["to_gnd", "from_gnd"])
@pytest.mark.parametrize("term", ["Gnd", "A"])
def test_ground_is_the_gnd_net_whatever_the_wire_direction(direction, term):
    res = sp.Simulation(_load(_divider(direction, term))).run(check=False)
    nl = json.loads(res.netlist)
    v1 = next(s for s in nl["physical_stage"]["voltage_sources"] if s["id"] == "V1")
    pos, neg = v1["nodes"]
    assert pos != "node_0" and neg != "node_0"
    assert res[pos][-1] == pytest.approx(5) and res[neg][-1] == pytest.approx(-5)


def test_every_gnd_block_is_ground():
    # Two separate loops, each with its own GND symbol on the source's negative pin.
    comps = [_comp("V1", "V", value=10), _comp("R1", "R", value=1), _comp("GND1", "GND"),
             _comp("V2", "V", value=4), _comp("R2", "R", value=1), _comp("GND2", "GND")]
    wires = [_w(1, "V1.A", "R1.A"), _w(2, "R1.B", "GND1.Gnd"), _w(3, "V1.B", "GND1.Gnd"),
             _w(4, "V2.A", "R2.A"), _w(5, "R2.B", "GND2.Gnd"), _w(6, "V2.B", "GND2.Gnd")]
    res = sp.Simulation(_load(_design(wires, comps))).run(check=False)
    nl = json.loads(res.netlist)
    nodes = {r["id"]: r["nodes"] for r in nl["physical_stage"]["resistors"]}
    assert nodes["R1"][1] == "node_0" and nodes["R2"][1] == "node_0"
    assert res[nodes["R2"][0]][-1] == pytest.approx(4)


# ── Trapezoidal integration of capacitors ────────────────────────────────────
def _rc_error(solver, dt):
    r, cap = 1.0, 100e-6
    c = sp.Circuit("rc")
    v = c.add("V", "V1", value=10)
    rr = c.add("R", "R1", value=r)
    cc = c.add("C", "C1", C=cap)
    c.connect(v.A, rr.A)
    c.connect(rr.B, cc.A)
    c.ground(v.B, cc.B)
    res = _run(c, t_stop=500e-6, dt=dt, solver=solver)
    t = res.time
    m = t >= 100e-6
    exact = 10 * (1 - np.exp(-t[m] / (r * cap)))
    return float(np.max(np.abs(res["V_C1"][m] - exact)))


def test_trapezoidal_capacitor_is_second_order():
    be1, be2 = _rc_error("euler", 2e-6), _rc_error("euler", 1e-6)
    tr1, tr2 = _rc_error("trapezoidal", 2e-6), _rc_error("trapezoidal", 1e-6)
    assert be1 / be2 == pytest.approx(2, rel=0.15)  # first order
    assert tr1 / tr2 == pytest.approx(4, rel=0.15)  # second order
    assert tr2 < be2 / 20


def test_trapezoidal_lc_keeps_amplitude():
    # An undamped LC tank: trapezoidal preserves the oscillation, BE damps it.
    def amp(solver):
        c = sp.Circuit("lc")
        cc = c.add("C", "C1", C=1e-6, vC0=10)
        ll = c.add("L", "L1", L=1e-3)
        c.connect(cc.A, ll.A)
        c.ground(cc.B, ll.B)
        res = _run(c, t_stop=20e-3, dt=1e-6, solver=solver)
        return res.stats("V_C1", 18e-3).abs_max

    assert amp("trapezoidal") == pytest.approx(10, rel=0.01)
    assert amp("euler") < 5
