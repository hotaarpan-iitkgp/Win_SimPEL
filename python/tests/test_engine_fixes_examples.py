"""Regression tests for engine defects found while building examples/advanced.

Each test is a small circuit with an analytic answer. The defect each guards
against is named in the docstring.
"""

from __future__ import annotations

import math
import warnings

import numpy as np
import pytest

import simpel as sp


def _run(c, **kw):
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        return sp.Simulation(c, **kw).run(check=False)


def _cscript(code: str, x: float = 0.0, y: float = 0.0, *, wire_in1=True, wire_in2=None):
    c = sp.Circuit("cs")
    cs = c.add("CSCRIPT", "CS", code=code)
    if wire_in1:
        c.connect(c.add("CONST", "X", value=x).Out, cs.In1)
    if wire_in2 if wire_in2 is not None else "inputs[1]" in code:
        c.connect(c.add("CONST", "Y", value=y).Out, cs.In2)
    return float(_run(c, t_stop=3e-6, dt=1e-6)["CS.Out1"][-1])


# ── controlled sources ──────────────────────────────────────────────────────
def test_ctrl_v_follows_control_signal():
    """CTRL_V was typed as a plain DC source and output its 'value' (1 V)."""
    c = sp.Circuit("cv")
    s = c.add("SINE_WAVE", "S", amplitude=100, frequency=50)
    e = c.add("CTRL_V", "E")
    r = c.add("R", "R1", value=10)
    c.connect(s.Out, e.Ctrl); c.connect(e.A, r.A); c.ground(e.B, r.B)
    res = _run(c, t_stop=0.04, dt=1e-5)
    assert res.stats("V_R1", 0.02).abs_max == pytest.approx(100, rel=1e-3)


def test_ctrl_i_matches_dc_current_source():
    """CTRL_I ignored Ctrl (1 A always); it must behave like I(value=Ctrl)."""
    def build(kind):
        c = sp.Circuit(kind)
        r = c.add("R", "R1", value=10)
        if kind == "ctrl":
            j = c.add("CTRL_I", "J")
            c.connect(c.add("CONST", "K", value=2).Out, j.Ctrl)
        else:
            j = c.add("I", "J", value=2)
        c.connect(j.A, r.A); c.ground(j.B, r.B)
        return _run(c, t_stop=1e-4, dt=1e-6)
    a, b = build("ctrl"), build("dc")
    assert a["V_R1"][-1] == pytest.approx(b["V_R1"][-1], rel=1e-9)
    assert abs(a["V_R1"][-1]) == pytest.approx(20, rel=1e-6)
    assert a["I_J"][-1] == pytest.approx(2, rel=1e-9)


# ── CSCRIPT language ────────────────────────────────────────────────────────
def test_cscript_equality_operator_does_not_hang():
    """`a == b` sent the tokenizer into an infinite loop (hung the process)."""
    assert _cscript("outputs[0] = inputs[0] == 3;", 3) == 1.0
    assert _cscript("outputs[0] = (inputs[0] > 0 && inputs[0] < 5) || inputs[0] == -1;", 3) == 1.0


@pytest.mark.parametrize("expr,x,y,expected", [
    ("fabs(inputs[0])", -2.5, 0, 2.5),
    ("atan2(inputs[0], inputs[1])", 1, -1, math.atan2(1, -1)),
    ("atan(inputs[0])", 1, 0, math.atan(1)),
    ("asin(inputs[0])", 0.5, 0, math.asin(0.5)),
    ("acos(inputs[0])", 0.5, 0, math.acos(0.5)),
    ("floor(inputs[0])", 2.7, 0, 2.0),
    ("ceil(inputs[0])", 2.2, 0, 3.0),
    ("round(inputs[0])", 2.5, 0, 3.0),
    ("fmod(inputs[0], 2.0)", 7.5, 0, 1.5),
    ("sign(inputs[0])", -4, 0, -1.0),
    ("tanh(inputs[0])", 0.3, 0, math.tanh(0.3)),
    ("hypot(inputs[0], inputs[1])", 3, 4, 5.0),
    ("1.0 + inputs[0] % 2.0", 7.5, 0, 2.5),          # '%' had precedence 0
    ("+inputs[0] * 2.0", 3, 0, 6.0),                 # unary plus pushed a stray 0
    ("!(inputs[0] > 1)", 3, 0, 0.0),
])
def test_cscript_functions(expr, x, y, expected):
    """These compiled to an unknown variable (always 0) before."""
    assert _cscript(f"outputs[0] = {expr};", x, y) == pytest.approx(expected, rel=1e-12, abs=1e-12)


def test_cscript_ports_ordered_by_index():
    """Ports were bound in order of first appearance: reading inputs[1] before
    inputs[0] swapped the inputs, and writing outputs[1] first swapped outputs."""
    code = "outputs[1] = inputs[1] * 10.0;\noutputs[0] = inputs[0];"
    c = sp.Circuit("ord")
    cs = c.add("CSCRIPT", "CS", code=code)
    c.connect(c.add("CONST", "A", value=1).Out, cs.In1)
    c.connect(c.add("CONST", "B", value=2).Out, cs.In2)
    g1 = c.add("GAIN", "G1", K=1); g2 = c.add("GAIN", "G2", K=1)
    c.connect(cs.Out1, g1.In); c.connect(cs.Out2, g2.In)
    res = _run(c, t_stop=3e-6, dt=1e-6)
    assert res["G1.Out"][-1] == 1.0 and res["G2.Out"][-1] == 20.0


def test_cscript_unconnected_first_input_keeps_positions():
    """With In1 open, inputs[0] used to read the signal wired to In2."""
    v = _cscript("outputs[0] = inputs[0] + 10.0 * inputs[1];", wire_in1=False, y=2.0, wire_in2=True)
    assert v == 20.0


def test_cscript_single_output_in_results():
    """A one-output script only exposed '<id>.Out' (always 0) in the results."""
    c = sp.Circuit("one")
    cs = c.add("CSCRIPT", "CS", code="outputs[0] = inputs[0] * 3.0;")
    c.connect(c.add("CONST", "K", value=2).Out, cs.In1)
    res = _run(c, t_stop=3e-6, dt=1e-6)
    assert res["CS.Out1"][-1] == 6.0
    assert res["CS.Out"][-1] == 6.0 and len(res["CS.Out"]) == len(res.time)


def test_cscript_state_and_sample_time():
    c = sp.Circuit("st")
    cs = c.add("CSCRIPT", "CS", code="double n = 0.0;\nvoid step() { n += 1.0; outputs[0] = n; }", timestep=1e-4)
    c.connect(c.add("CONST", "K", value=1).Out, cs.In1)
    res = _run(c, t_stop=1e-3, dt=1e-6)
    assert res["CS.Out1"][-1] == pytest.approx(10, abs=1)


# ── PLLs ────────────────────────────────────────────────────────────────────
def _three_phase(c, f, amp=325.0):
    return [c.add("SINE_WAVE", f"S{k}", amplitude=amp, frequency=f, phase=a).Out
            for k, a in enumerate((0, -120, 120))]


def test_pll_3ph_locks_and_reports_frequency():
    """PLL_3PH was a free-running stub at fn; Theta/Freq were never written."""
    c = sp.Circuit("p3")
    p = c.add("PLL_3PH", "P", fn=50, Kp=2, Ki=200)
    a, b, cc = _three_phase(c, 60.0)
    c.connect(a, p.A); c.connect(b, p.B); c.connect(cc, p.C)
    res = _run(c, t_stop=0.3, dt=1e-5)
    m = res.time > 0.25
    assert np.mean(res["P.Freq"][m]) == pytest.approx(60, abs=1e-3)
    err = np.angle(np.exp(1j * (res["P.Theta"][m] - 2 * math.pi * 60 * res.time[m])))
    assert np.degrees(np.abs(err)).max() < 1.0      # va = Vm sin(Theta)


def test_pll_freq_is_the_rate_of_theta():
    """Freq must equal d(Theta)/dt step by step (the committed omega)."""
    c = sp.Circuit("p3r")
    p = c.add("PLL_3PH", "P", fn=50, Kp=2, Ki=200)
    a, b, cc = _three_phase(c, 53.0)
    c.connect(a, p.A); c.connect(b, p.B); c.connect(cc, p.C)
    dt = 1e-6
    res = _run(c, t_stop=0.01, dt=dt)
    d = np.diff(np.unwrap(res["P.Theta"])) / (2 * math.pi * dt)
    assert np.max(np.abs(d[5:] - res["P.Freq"][6:])) < 1e-6


def test_pll_1ph_tracks_frequency():
    """PLL_1PH was a free-running stub too; its Out pin carries Theta.

    The multiplier phase detector has a 2f ripple (amplitude Vm/2), which leaves a
    few degrees of phase wobble, so only lock and frequency are asserted here."""
    c = sp.Circuit("p1")
    s = c.add("SINE_WAVE", "S", amplitude=1, frequency=55, phase=0)
    p = c.add("PLL_1PH", "P", fn=50, Kp=200, Ki=20000)
    g = c.add("GAIN", "G", K=1)
    c.connect(s.Out, p.In); c.connect(p.Out, g.In)
    res = _run(c, t_stop=0.4, dt=1e-5)
    m = res.time > 0.3
    th = np.unwrap(res["G.Out"][m])
    assert np.polyfit(res.time[m], th, 1)[0] / (2 * math.pi) == pytest.approx(55, abs=0.01)
    err = np.angle(np.exp(1j * (res["G.Out"][m] - 2 * math.pi * 55 * res.time[m])))
    assert np.degrees(np.abs(err)).max() < 15.0


def test_pll_loop_uses_fn_kp_ki():
    """PLL_LOOP ignored fn/Kp/Ki (started at 10 kHz) and advanced twice per step."""
    c = sp.Circuit("pl")
    s = c.add("SINE_WAVE", "S", amplitude=1, frequency=55, phase=0)
    p = c.add("PLL_LOOP", "P", fn=50, Kp=200, Ki=20000)
    c.connect(s.Out, p.In)
    res = _run(c, t_stop=0.4, dt=1e-5)
    m = res.time > 0.3
    th = np.unwrap(res["P.Theta"][m])
    assert np.polyfit(res.time[m], th, 1)[0] / (2 * math.pi) == pytest.approx(55, abs=0.05)


# ── three-phase modulators ─────────────────────────────────────────────────
@pytest.mark.parametrize("kind", ["SVPWM", "PWM_3PH"])
def test_three_phase_modulators(kind):
    """fc was ignored (always 10 kHz); SVPWM re-mixed a,b,c as alpha/beta;
    PWM_3PH never drove OutA/OutB/OutC."""
    c = sp.Circuit(kind)
    blk = c.add(kind, "M", fc=5000)
    a, b, cc = (c.add("SINE_WAVE", f"R{k}", amplitude=0.8, frequency=50, phase=ph).Out
                for k, ph in enumerate((0, -120, 120)))
    c.connect(a, blk.A); c.connect(b, blk.B); c.connect(cc, blk.C)
    ga, gb = c.add("GAIN", "Ga", K=1), c.add("GAIN", "Gb", K=1)
    c.connect(blk.OutA, ga.In); c.connect(blk.OutB, gb.In)
    res = _run(c, t_stop=0.04, dt=1e-6)
    m = res.time >= 0.02
    x, y, t = res["Ga.Out"][m], res["Gb.Out"][m], res.time[m]
    assert set(np.unique(x)) <= {0.0, 1.0}
    assert int(np.sum(np.diff(x) > 0.5)) == pytest.approx(100, abs=2)      # 5 kHz over 20 ms
    d = x - y
    ll = 2 * abs(np.mean(d * np.exp(-1j * 2 * math.pi * 50 * t)))
    assert ll == pytest.approx(0.4 * math.sqrt(3), rel=0.02)              # line-line duty fundamental
