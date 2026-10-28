"""Regression tests for the block defects fixed in block-verification round 1-3.

Each test is a small circuit with an analytic answer, and each one fails on the engine
before the corresponding fix. Run with the repo venv from python/tests/blocks:

    ..\\..\\..\\..\\.venv\\Scripts\\python.exe -m pytest -q test_block_regressions.py
"""
from __future__ import annotations

import math
import time
import warnings

import numpy as np
import pytest

import simpel as sp

warnings.simplefilter("ignore")


def _run(c, t_stop, dt):
    return sp.Simulation(c, t_stop=t_stop, dt=dt).run(check=False)


# ── MOV_AVG: window parameter ignored (always 4 samples) ───────────────────────────
def _mov_avg(window: int) -> float:
    c = sp.Circuit("ma")
    r = c.add("RAMP", "R", slope=1.0, start_time=0, initial_output=0)
    m = c.add("MOV_AVG", "M", window=window)
    c.connect(r.Out, m.In)
    res = _run(c, 0.05, 1e-3)
    return float(res["M.Out"][-1])


def test_mov_avg_window_sets_lag():
    """Average of a ramp over W samples lags by (W-1)/2 * dt, on top of the block's
    one-sample output delay. The window effect is the difference from window 1."""
    dt = 1e-3
    lag5 = 0.05 - _mov_avg(5)
    lag1 = 0.05 - _mov_avg(1)
    assert lag5 - lag1 == pytest.approx(2 * dt, abs=0.25 * dt)


# ── EDGE_DETECT: edge_type not forwarded (always rising) ────────────────────────────
def _first_pulse_start(edge: str) -> float:
    c = sp.Circuit("ed")
    s = c.add("SINE_WAVE", "S", amplitude=1.0, frequency=5.0, phase=0)
    e = c.add("EDGE_DETECT", "E", edge_type=edge)
    c.connect(s.Out, e.In)
    res = _run(c, 0.2, 1e-5)
    t = res.time
    out = np.asarray(res["E.Out"])
    idx = int(np.argmax(out > 0.5))
    return float(t[idx])


def test_edge_detect_rising_and_falling_differ():
    """Rising edge of a sine crosses 0.5 at t = 1/60 s; falling at t = 5/60 s."""
    assert _first_pulse_start("rising") == pytest.approx(1 / 60, abs=2e-4)
    assert _first_pulse_start("falling") == pytest.approx(5 / 60, abs=2e-4)


# ── MONOSTABLE: pulse_width ignored, default was 0.1 s ─────────────────────────────
def _monostable_high_time(width: float) -> float:
    c = sp.Circuit("mo")
    s = c.add("SINE_WAVE", "S", amplitude=1.0, frequency=5.0, phase=0)
    m = c.add("MONOSTABLE", "M", pulse_width=width)
    c.connect(s.Out, m.In)
    res = _run(c, 0.2, 1e-5)
    out = np.asarray(res["M.Out"])
    return float(np.sum(out > 0.5) * 1e-5)


@pytest.mark.parametrize("width", [0.002, 0.005])
def test_monostable_pulse_width(width):
    assert _monostable_high_time(width) == pytest.approx(width, abs=3e-5)


# ── PWM: min / max not forwarded ───────────────────────────────────────────────────
def _pwm_duty(vmax: float) -> float:
    c = sp.Circuit("pwm")
    k = c.add("CONST", "K", value=0.5)
    p = c.add("PWM", "P", frequency=10e3, min=0, max=vmax)
    c.connect(k.Out, p.In)
    res = _run(c, 0.01, 1e-7)
    t = res.time
    m = t > 0.002
    return float(np.mean(np.asarray(res["P.Out"])[m]))


def test_pwm_min_max_change_duty():
    """Carrier 0..1 with reference 0.5 gives duty 0.5; carrier 0..2 gives 0.25."""
    assert _pwm_duty(1.0) == pytest.approx(0.5, abs=0.01)
    assert _pwm_duty(2.0) == pytest.approx(0.25, abs=0.01)


# ── PWM_3PH: dead time ignored; complementary gates overlapped ─────────────────────
def test_pwm3ph_dead_time_prevents_leg_overlap():
    c = sp.Circuit("p3")
    blk = c.add("PWM_3PH", "P", fc=5e3, dead_time=1e-5)
    for ph, ang in (("A", 0), ("B", -120), ("C", 120)):
        s = c.add("SINE_WAVE", f"S{ph}", amplitude=0.8, frequency=50, phase=ang)
        c.connect(s.Out, getattr(blk, ph))
    res = _run(c, 0.04, 1e-7)
    up = np.asarray(res["P.G1"]) > 0.5
    lo = np.asarray(res["P.G2"]) > 0.5
    assert not np.any(up & lo), "upper and lower switch of leg A conduct together"


def test_pwm3ph_dead_time_reduces_on_time():
    def duty(dt_s):
        c = sp.Circuit("p3d")
        blk = c.add("PWM_3PH", "P", fc=5e3, dead_time=dt_s)
        for ph, ang in (("A", 0), ("B", -120), ("C", 120)):
            s = c.add("SINE_WAVE", f"S{ph}", amplitude=0.8, frequency=50, phase=ang)
            c.connect(s.Out, getattr(blk, ph))
        res = _run(c, 0.04, 1e-7)
        return float(np.mean(np.asarray(res["P.G1"])[res.time > 0.02]))
    assert duty(1e-4) < duty(0.0) - 0.02


# ── D_FLIP_FLOP: clock never bound, Q stuck at 0 ───────────────────────────────────
def test_dff_samples_d_on_rising_clock():
    c = sp.Circuit("dff")
    d = c.add("CONST", "D", value=1.0)
    clk = c.add("PULSE_GEN", "CLK", amplitude=1.0, period=0.02, width=0.5, delay=0.0)
    ff = c.add("D_FLIP_FLOP", "FF")
    c.connect(d.Out, ff.D)
    c.connect(clk.Out, ff.Ctrl)
    res = _run(c, 0.05, 1e-5)
    assert res["FF.Q"][-1] == 1.0


# ── MULTIPORT_SWITCH: data inputs were never bound (output always 0) ───────────────
@pytest.mark.parametrize("sel,expected", [(1, 1.0), (2, 2.0), (3, 3.0)])
def test_multiport_selects_input(sel, expected):
    c = sp.Circuit("mp")
    m = c.add("MULTIPORT_SWITCH", "M", inputs=3)
    for i, v in enumerate((1.0, 2.0, 3.0), start=1):
        k = c.add("CONST", f"I{i}", value=v)
        c.connect(k.Out, getattr(m, f"In{i}"))
    k = c.add("CONST", "SEL", value=float(sel))
    c.connect(k.Out, m.Ctrl)
    res = _run(c, 0.001, 1e-5)
    assert res["M.Out"][-1] == pytest.approx(expected)


# ── FOURIER: f / harmonic / ts not read ────────────────────────────────────────────
def _fourier_mag(f: float) -> float:
    c = sp.Circuit("fa")
    s = c.add("SINE_WAVE", "S", amplitude=1.0, frequency=50.0, phase=0)
    fa = c.add("FOURIER_ANALYSIS", "F", f=f)
    c.connect(s.Out, fa.In)
    res = _run(c, 0.2, 1e-5)
    return float(res["F.Mag"][-1])


def test_fourier_analysis_uses_frequency_parameter():
    """At f = 50 Hz the block reads the unit 50 Hz sine at amplitude 1. At f = 100 Hz
    the one-cycle window holds half a 50 Hz period, so the reading is much smaller."""
    mag50 = _fourier_mag(50.0)
    assert mag50 == pytest.approx(1.0, abs=0.05)
    assert _fourier_mag(100.0) < 0.6 * mag50


# ── PWM_MASTER: P1..P3 outputs were registered but never written ───────────────────
def test_pwm_master_outputs_are_populated():
    """Was an xfail: the netlist never sent In1..InN, so P1 stayed 0."""
    c = sp.Circuit("pm")
    for ch, ang in (("1", 0), ("2", -120), ("3", 120)):
        s = c.add("SINE_WAVE", f"S{ch}", amplitude=0.8, frequency=50, phase=ang)
        c.connect(s.Out, getattr(c.add("PWM_MASTER", "PM") if ch == "1" else c["PM"], f"In{ch}"))
    res = _run(c, 0.02, 1e-6)
    p1 = np.asarray(res["PM.P1"])
    assert p1.size == res.time.size
    assert set(np.unique(p1)) <= {0.0, 1.0} and p1.max() == 1.0


# ── PERIODIC_IMP_AVG: trigger never bound, output stuck at its default ─────────────
def test_periodic_imp_avg_responds_to_trigger():
    c = sp.Circuit("pia")
    s = c.add("SINE_WAVE", "S", amplitude=1.0, frequency=5.0, phase=0)
    t = c.add("SINE_WAVE", "T", amplitude=1.0, frequency=5.0, phase=0)
    p = c.add("PERIODIC_IMP_AVG", "P")
    c.connect(s.Out, p.In)
    c.connect(t.Out, p.Trig)
    res = _run(c, 0.4, 1e-5)
    out = np.asarray(res["P.Out"])
    assert np.ptp(out) > 1e-3


# ── THD_VAL: pure tone gives 0; 30 % third harmonic gives 0.3 ──────────────────────
def _thd(third: float) -> float:
    c = sp.Circuit("thd")
    h1 = c.add("SINE_WAVE", "H1", amplitude=1.0, frequency=50.0, phase=0)
    h3 = c.add("SINE_WAVE", "H3", amplitude=third, frequency=150.0, phase=0)
    add = c.add("SUM", "ADD", signs="++")
    thd = c.add("THD_VAL", "T", frequency=50.0)
    c.connect(h1.Out, add.In1)
    c.connect(h3.Out, add.In2)
    c.connect(add.Out, thd.In)
    res = _run(c, 0.1, 2e-6)
    return float(res["T.Out"][-1])


def test_thd_pure_sine_is_zero():
    assert _thd(0.0) == pytest.approx(0.0, abs=0.01)


def test_thd_third_harmonic_amplitude():
    assert _thd(0.3) == pytest.approx(0.3, abs=0.02)


@pytest.mark.parametrize("fn,x,expected", [
    ("sin", 0.5, math.sin(0.5)),
    ("cos", 0.5, math.cos(0.5)),
    ("atan", 1.0, math.atan(1.0)),
    ("tanh", 0.3, math.tanh(0.3)),
    ("exp", 0.5, math.exp(0.5)),
])
def test_math_function_selects_the_named_function(fn, x, expected):
    """sin/cos/atan/tanh are listed choices of MATH_FCN but were evaluated as exp."""
    c = sp.Circuit("mf")
    k = c.add("CONST", "K", value=x)
    m = c.add("MATH_FCN", "M", function=fn)
    c.connect(k.Out, m.In)
    res = _run(c, 0.001, 1e-5)
    assert res["M.Out"][-1] == pytest.approx(expected, rel=1e-9)


def test_thd_runs_in_reasonable_time():
    """The sliding-DFT fix took 210 s down to about 1 s for this case."""
    t0 = time.perf_counter()
    _thd(0.3)
    assert time.perf_counter() - t0 < 30.0


# ── LUT_2D: nested table default was flattened into one row (output always 0) ──────
def test_lut_2d_parses_nested_table_default():
    """With the default z = [[0, 1], [1, 2]] and In2 unconnected (reads 0.0), the output is
    the first-axis interpolation between z00 = 0 and z10 = 1, so In = 0.5 gives 0.5."""
    c = sp.Circuit("lut")
    k = c.add("CONST", "K", value=0.5)
    lut = c.add("LUT_2D", "L")
    c.connect(k.Out, lut.In)
    res = _run(c, 0.001, 1e-5)
    assert res["L.Out"][-1] == pytest.approx(0.5, abs=1e-9)


def test_lut_2d_second_input_is_used():
    """Both axes at 0.5 give bilinear z = 0.25*(0 + 1 + 1 + 2) = 1.0. The second input must
    be wired to the table, not the constant 0.0 it used to read."""
    c = sp.Circuit("lut2")
    a = c.add("CONST", "A", value=0.5)
    b = c.add("CONST", "B", value=0.5)
    lut = c.add("LUT_2D", "L")
    c.connect(a.Out, lut.In)
    c.connect(b.Out, lut.In2)
    res = _run(c, 0.001, 1e-5)
    assert res["L.Out"][-1] == pytest.approx(1.0, abs=1e-9)


def test_lut_3d_trilinear_matches_linear_table():
    """The default cube is table[i][j][k] = i + j + k on axes [0, 1]. Trilinear
    interpolation of a linear function is exact, so (0.5, 0.5, 0.5) gives 1.5."""
    c = sp.Circuit("lut3")
    vals = (0.5, 0.5, 0.5)
    for name, v in zip(("A", "B", "C"), vals):
        c.add("CONST", name, value=v)
    lut = c.add("LUT_3D", "L")
    c.connect("A.Out", lut.In)
    c.connect("B.Out", lut.In2)
    c.connect("C.Out", lut.In3)
    res = _run(c, 0.001, 1e-5)
    assert res["L.Out"][-1] == pytest.approx(1.5, abs=1e-9)


# ── FOURIER_SERIES: truncated Fourier series of the last cycle ─────────────────────
def test_fourier_series_passes_fundamental_and_drops_third_harmonic():
    """Input = sin(2 pi 50 t) + 0.3 sin(2 pi 150 t). With one harmonic kept, the
    reconstruction is the fundamental only; the third harmonic is removed."""
    f = 50.0
    c = sp.Circuit("fs")
    h1 = c.add("SINE_WAVE", "H1", amplitude=1.0, frequency=f, phase=0)
    h3 = c.add("SINE_WAVE", "H3", amplitude=0.3, frequency=3 * f, phase=0)
    add = c.add("SUM", "ADD", signs="++")
    fs = c.add("FOURIER_SERIES", "F", f=f, harmonics=1, ts=1e-4)
    c.connect(h1.Out, add.In1)
    c.connect(h3.Out, add.In2)
    c.connect(add.Out, fs.In)
    res = _run(c, 0.1, 1e-5)
    t = res.time
    out = np.asarray(res["F.Out"])
    m = t > 0.04                      # after one full cycle of warm-up
    expected = np.sin(2 * math.pi * f * t[m])
    assert np.max(np.abs(out[m] - expected)) < 0.03


# ── Operator defaults: an unset operator must use the declared JSON default ─────────
def test_bitwise_default_operator_is_xor():
    """BITWISE_OP declares XOR. XOR of 1 and 0 is 1; AND (the old fallback) gives 0."""
    c = sp.Circuit("bw")
    a = c.add("CONST", "A", value=1.0)
    b = c.add("CONST", "B", value=0.0)
    op = c.add("BITWISE_OP", "O")
    c.connect(a.Out, op.In1)
    c.connect(b.Out, op.In2)
    res = _run(c, 0.001, 1e-5)
    assert res["O.Out"][-1] == pytest.approx(1.0)


def test_relational_default_operator_is_less_than():
    """RELATIONAL_OPERATOR declares "<". 0.2 < 0.5 is true; "==" (the old fallback) gives 0."""
    c = sp.Circuit("rel")
    a = c.add("CONST", "A", value=0.2)
    b = c.add("CONST", "B", value=0.5)
    op = c.add("RELATIONAL_OPERATOR", "R")
    c.connect(a.Out, op.In1)
    c.connect(b.Out, op.In2)
    res = _run(c, 0.001, 1e-5)
    assert res["R.Out"][-1] == pytest.approx(1.0)


def test_compare_to_constant_default_is_greater_equal():
    """COMPARE_TO_CONSTANT declares ">=" against constant 1.0. An input of 1.5 gives 1.0,
    and the old "==" fallback would give 0."""
    c = sp.Circuit("cmp")
    k = c.add("CONST", "K", value=1.5)
    cmp_ = c.add("COMPARE_TO_CONSTANT", "C")
    c.connect(k.Out, cmp_.In)
    res = _run(c, 0.001, 1e-5)
    assert res["C.Out"][-1] == pytest.approx(1.0)


# ── TRI: amplitude sets a symmetric +/-A triangle (it used to be ignored) ───────────
def test_tri_amplitude_sets_symmetric_range():
    c = sp.Circuit("tri")
    c.add("TRI", "T", amplitude=2.0, frequency=50.0)
    res = _run(c, 0.1, 1e-5)
    out = np.asarray(res["T.Out"])
    assert out.max() == pytest.approx(2.0, abs=0.01)
    assert out.min() == pytest.approx(-2.0, abs=0.01)


# ── DISCRETE_PID: with ts set, the output is held between samples ──────────────────
def test_discrete_pid_holds_output_between_samples():
    """With ts = 0.02 the controller updates once per 20 ms. The output must be constant
    inside each sample interval, which a continuous controller is not."""
    c = sp.Circuit("dpid")
    s = c.add("SINE_WAVE", "S", amplitude=1.0, frequency=5.0, phase=0)
    p = c.add("DISCRETE_PID", "P", ts=0.02, Kp=1.0, Ki=1.0)
    c.connect(s.Out, p.In)
    res = _run(c, 0.2, 1e-5)
    t = res.time
    out = np.asarray(res["P.Out"])
    # Samples land on multiples of 0.02 s. Take two points inside one interval.
    a = out[np.argmin(np.abs(t - 0.0201))]
    b = out[np.argmin(np.abs(t - 0.0299))]
    assert a == pytest.approx(b, abs=1e-12)


# ── PWM_MASTER: carrier range and frequency must reach the engine ───────────────────
def _pwm_master_run(mod: float, **params):
    c = sp.Circuit("pmx")
    k = c.add("CONST", "K", value=mod)
    pm = c.add("PWM_MASTER", "PM", **params)
    c.connect(k.Out, pm.In1)
    return _run(c, 0.01, 1e-7)


def test_pwm_master_max_changes_duty():
    """Modulation 0.5 against a 0..1 carrier gives duty 0.5. With max = 2 it gives 0.25.
    Before the fix, min and max were never read."""
    r1 = _pwm_master_run(0.5)
    d1 = np.mean(np.asarray(r1["PM.P1"])[r1.time > 0.002])
    r2 = _pwm_master_run(0.5, max=2.0)
    d2 = np.mean(np.asarray(r2["PM.P1"])[r2.time > 0.002])
    assert d1 == pytest.approx(0.5, abs=0.02)
    assert d2 == pytest.approx(0.25, abs=0.02)


def test_pwm_master_frequency_sets_carrier():
    """One rising edge per carrier period. Over 10 ms that is about 100 edges at 10 kHz and
    about 200 at 20 kHz. Before the fix, "frequency" was ignored and the carrier stayed 10 kHz."""
    def rising(freq):
        r = _pwm_master_run(0.5, frequency=freq)
        p = np.asarray(r["PM.P1"]) > 0.5
        return int(np.sum(p[1:] & ~p[:-1]))
    n10 = rising(10e3)
    n20 = rising(20e3)
    assert n10 == pytest.approx(100, abs=3)
    assert n20 == pytest.approx(200, abs=3)


# ── POLYNOMIAL: coefficients are read (highest power first) ─────────────────────────
def test_polynomial_uses_coefficients():
    """coefficients [2, 1] is 2u + 1. With u = 3 the output is 7. Before, the coefficients
    were never parsed and the block passed its input through unchanged."""
    c = sp.Circuit("poly")
    k = c.add("CONST", "K", value=3.0)
    p = c.add("POLYNOMIAL", "P", coefficients="[2, 1]")
    c.connect(k.Out, p.In)
    res = _run(c, 0.001, 1e-5)
    assert res["P.Out"][-1] == pytest.approx(7.0, abs=1e-9)


# ── ALGEBRAIC_CONSTRAINT: initial_guess seeds the unknown ───────────────────────────
def test_algebraic_constraint_starts_at_initial_guess():
    """With a zero residual the unknown stays at its initial guess. Before, the guess was
    never applied and the output started at 0."""
    c = sp.Circuit("alg")
    k = c.add("CONST", "K", value=0.0)
    a = c.add("ALGEBRAIC_CONSTRAINT", "A", initial_guess=0.5)
    c.connect(k.Out, a.In)
    res = _run(c, 0.001, 1e-5)
    assert res["A.Out"][-1] == pytest.approx(0.5, abs=1e-9)


# ── BITWISE_OP: bit_mask limits the bits used ───────────────────────────────────────
def _bitwise(in1, in2, **params):
    c = sp.Circuit("bw")
    a = c.add("CONST", "A", value=in1)
    b = c.add("CONST", "B", value=in2)
    op = c.add("BITWISE_OP", "O", operator="AND", **params)
    c.connect(a.Out, op.In1)
    c.connect(b.Out, op.In2)
    return _run(c, 0.001, 1e-5)["O.Out"][-1]


def test_bitwise_default_keeps_all_bits():
    """3 AND 2 = 2 with every bit kept. This is the old behaviour."""
    assert _bitwise(3.0, 2.0) == pytest.approx(2.0)


def test_bitwise_mask_limits_bits():
    """With bit_mask = 1 only the lowest bit is used: (3 & 1) AND (2 & 1) = 0."""
    assert _bitwise(3.0, 2.0, bit_mask=1) == pytest.approx(0.0)


# ── MAN_SWITCH / DBL_SWITCH: single-pole double-throw, state picks the throw ────────
def _spdt(state):
    c = sp.Circuit("spdt")
    c.add("V", "VS", value=10.0)
    c.ground("VS.B")
    sw = c.add("DBL_SWITCH", "SW", state=state)
    c.connect("VS.A", "SW.Common")
    c.add("R", "RA", value=1e3)
    c.add("R", "RB", value=1e3)
    c.connect("SW.A", "RA.A")
    c.connect("SW.B", "RB.A")
    c.ground("RA.B")
    c.ground("RB.B")
    return _run(c, 0.001, 1e-6)


def test_spdt_state_a_connects_common_to_a_only():
    r = _spdt("A")
    assert r["V_RA"][-1] == pytest.approx(10.0, abs=0.01)
    assert abs(r["V_RB"][-1]) < 0.05


def test_spdt_state_b_connects_common_to_b_only():
    """The B throw was never connected before this fix."""
    r = _spdt("B")
    assert r["V_RB"][-1] == pytest.approx(10.0, abs=0.01)
    assert abs(r["V_RA"][-1]) < 0.05


# ── MAN_DBL_SWITCH: two ganged poles, closed by default ─────────────────────────────
def _man_dbl(state=None):
    c = sp.Circuit("mdbl")
    c.add("V", "VS", value=10.0)
    c.ground("VS.B")
    params = {} if state is None else {"state": state}
    c.add("MAN_DBL_SWITCH", "M", **params)
    c.add("R", "RL", value=1e3)
    c.connect("VS.A", "M.A1")
    c.connect("M.B1", "RL.A")
    c.ground("RL.B")
    return _run(c, 0.001, 1e-6)


def test_man_dbl_switch_closed_by_default():
    assert _man_dbl()["V_RL"][-1] == pytest.approx(10.0, abs=0.01)


def test_man_dbl_switch_open_blocks_the_load():
    assert abs(_man_dbl("open")["V_RL"][-1]) < 0.05


# ── OPAMP: finite open-loop gain with rails (was a fixed 12 V source) ───────────────
def _opamp_load(vplus, gain=1e5, rail=None):
    c = sp.Circuit("oa")
    params = {"gain": gain}
    if rail is not None:
        params["rail"] = rail
    c.add("OPAMP", "U", **params)
    c.add("V", "VP", value=vplus)
    c.ground("VP.B")
    c.connect("VP.A", "U.Plus")
    c.ground("U.Minus")
    c.add("R", "RL", value=1e4)
    c.connect("U.Out", "RL.A")
    c.ground("RL.B")
    return _run(c, 0.001, 1e-6)["V_RL"][-1]


def test_opamp_open_loop_gain_is_linear():
    """1 uV in, gain 1e5 gives 0.1 V out. Gain 2e5 doubles it."""
    assert _opamp_load(1e-6, gain=1e5) == pytest.approx(0.1, rel=1e-3)
    assert _opamp_load(1e-6, gain=2e5) == pytest.approx(0.2, rel=1e-3)


def test_opamp_clamps_at_rail():
    """1 mV in would be 100 V; the output clamps at the 12 V rail."""
    assert _opamp_load(1e-3, gain=1e5) == pytest.approx(12.0, abs=0.01)
    assert _opamp_load(1e-3, gain=1e5, rail=5.0) == pytest.approx(5.0, abs=0.01)


# ── UNIT_DELAY and ZOH: samples every ts, held in between ───────────────────────────
def _ramp_sampled(btype, ts):
    c = sp.Circuit("sd")
    r = c.add("RAMP", "R", slope=1.0, start_time=0.0, initial_output=0.0)
    d = c.add(btype, "D", ts=ts)
    c.connect(r.Out, d.In)
    return _run(c, 0.1, 1e-5)


def test_zoh_holds_the_sample_taken_at_each_period():
    """Ramp of slope 1 with ts = 10 ms. At t = 50.5 ms the last sample was taken at 50 ms,
    when the ramp was 0.05."""
    r = _ramp_sampled("ZOH", 0.01)
    i = int(np.argmin(np.abs(r.time - 0.0505)))
    assert r["D.Out"][i] == pytest.approx(0.05, abs=1e-4)


def test_unit_delay_outputs_the_previous_sample():
    """Same ramp. The output at 50.5 ms is the sample from 40 ms, which is 0.04."""
    r = _ramp_sampled("UNIT_DELAY", 0.01)
    i = int(np.argmin(np.abs(r.time - 0.0505)))
    assert r["D.Out"][i] == pytest.approx(0.04, abs=1e-4)


# ── DISCRETE_INT and DISCRETE_MEAN: sampled every ts ────────────────────────────────
def test_discrete_int_forward_euler_sums_samples():
    """Input 1, ts = 10 ms, initial value 0. y[k] = k * ts, so at 50.5 ms the sample is 0.05."""
    c = sp.Circuit("di")
    k = c.add("CONST", "K", value=1.0)
    d = c.add("DISCRETE_INT", "D", ts=0.01, initial_value=0.0)
    c.connect(k.Out, d.In)
    r = _run(c, 0.1, 1e-5)
    i = int(np.argmin(np.abs(r.time - 0.0505)))
    assert r["D.Out"][i] == pytest.approx(0.05, abs=1e-6)


def test_discrete_mean_averages_the_last_period():
    """Ramp of slope 1, ts = 10 ms, period = 30 ms (three samples). At 50.5 ms the window
    holds the samples at 30, 40 and 50 ms, so the mean is 0.04."""
    c = sp.Circuit("dm")
    r_ = c.add("RAMP", "R", slope=1.0, start_time=0.0, initial_output=0.0)
    d = c.add("DISCRETE_MEAN", "D", ts=0.01, period=0.03, initial_value=0.0)
    c.connect(r_.Out, d.In)
    r = _run(c, 0.1, 1e-5)
    i = int(np.argmin(np.abs(r.time - 0.0505)))
    assert r["D.Out"][i] == pytest.approx(0.04, abs=1e-4)


# ── DISCRETE_TF and DISCRETE_SS: sampled linear systems ─────────────────────────────
def test_discrete_tf_difference_equation():
    """num = [1], den = [1, -0.5], ts = 10 ms, input 1. The recurrence is
    y[k] = 1 + 0.5 y[k-1], so y[k] = 2 (1 - 0.5^(k+1)). At 50.5 ms (sample 5): 1.96875."""
    c = sp.Circuit("dtf")
    k = c.add("CONST", "K", value=1.0)
    t = c.add("DISCRETE_TF", "T", num="[1]", den="[1, -0.5]", ts=0.01)
    c.connect(k.Out, t.In)
    r = _run(c, 0.1, 1e-5)
    i = int(np.argmin(np.abs(r.time - 0.0505)))
    assert r["T.Out"][i] == pytest.approx(1.96875, abs=1e-4)


def test_discrete_ss_state_recursion():
    """A = 0.5, B = 1, C = 1, D = 0, x0 = 0, input 1. x[k] = 2 (1 - 0.5^k), and y = x.
    At 50.5 ms (sample 5): 1.9375."""
    c = sp.Circuit("dss")
    k = c.add("CONST", "K", value=1.0)
    s = c.add("DISCRETE_SS", "S", A="[0.5]", B="[1]", C="[1]", D="[0]", x0="[0]", ts=0.01)
    c.connect(k.Out, s.In)
    r = _run(c, 0.1, 1e-5)
    i = int(np.argmin(np.abs(r.time - 0.0505)))
    assert r["S.Out"][i] == pytest.approx(1.9375, abs=1e-4)
