"""Phase 3, electrical: resistive, transient and device answers from circuit theory.

Element voltages and currents are read from the V_<id> and I_<id> signals.
"""
from __future__ import annotations

import math
import warnings

import numpy as np
import pytest

import simpel as sp

warnings.simplefilter("ignore")


def _sim(c: sp.Circuit, t_stop: float, dt: float):
    return sp.Simulation(c, t_stop=t_stop, dt=dt).run(check=False)


def _at(r, name: str, t: float) -> float:
    return float(np.asarray(r[name])[int(np.argmin(np.abs(r.time - t)))])


def test_resistive_divider_splits_voltage():
    c = sp.Circuit("div")
    c.add("V", "VS", value=10.0)
    c.ground("VS.B")
    c.add("R", "R1", value=1e3)
    c.add("R", "R2", value=1e3)
    c.connect("VS.A", "R1.A")
    c.connect("R1.B", "R2.A")
    c.ground("R2.B")
    r = _sim(c, 1e-4, 1e-6)
    assert float(np.asarray(r["V_R2"])[-1]) == pytest.approx(5.0, abs=1e-6)
    assert float(np.asarray(r["I_R2"])[-1]) == pytest.approx(5e-3, abs=1e-8)


def test_rc_step_response():
    """R = 1 kOhm, C = 1 uF, tau = 1 ms. From a 1 V step, v_C(t) = 1 - exp(-t/tau)."""
    c = sp.Circuit("rc")
    c.add("V", "VS", value=1.0)
    c.ground("VS.B")
    c.add("R", "R1", value=1e3)
    c.add("C", "C1", C=1e-6)
    c.connect("VS.A", "R1.A")
    c.connect("R1.B", "C1.A")
    c.ground("C1.B")
    r = _sim(c, 3e-3, 1e-6)
    assert _at(r, "V_C1", 1e-3) == pytest.approx(1 - math.exp(-1), abs=0.01)
    assert _at(r, "V_C1", 2e-3) == pytest.approx(1 - math.exp(-2), abs=0.01)


def test_rl_step_response():
    """R = 10 Ohm, L = 10 mH, tau = 1 ms. From 10 V, i(t) = (V/R)(1 - exp(-t/tau)).
    At tau the current is 0.632 * 1 A."""
    c = sp.Circuit("rl")
    c.add("V", "VS", value=10.0)
    c.ground("VS.B")
    c.add("R", "R1", value=10.0)
    c.add("L", "L1", L=10e-3)
    c.connect("VS.A", "R1.A")
    c.connect("R1.B", "L1.A")
    c.ground("L1.B")
    r = _sim(c, 3e-3, 1e-6)
    assert _at(r, "I_L1", 1e-3) == pytest.approx(1 - math.exp(-1), abs=0.01)


def test_forward_diode_drop():
    """10 V through a diode (Vd = 0.7 V) into 1 kOhm: the load sees about 9.3 V."""
    c = sp.Circuit("dio")
    c.add("V", "VS", value=10.0)
    c.ground("VS.B")
    c.add("D", "U")
    c.add("R", "RL", value=1e3)
    c.connect("VS.A", "U.A")
    c.connect("U.B", "RL.A")
    c.ground("RL.B")
    r = _sim(c, 2e-3, 1e-6)
    assert float(np.asarray(r["V_RL"])[-1]) == pytest.approx(9.3, abs=0.05)


def test_reverse_diode_blocks_current():
    c = sp.Circuit("dior")
    c.add("V", "VS", value=-10.0)
    c.ground("VS.B")
    c.add("D", "U")
    c.add("R", "RL", value=1e3)
    c.connect("VS.A", "U.A")
    c.connect("U.B", "RL.A")
    c.ground("RL.B")
    r = _sim(c, 2e-3, 1e-6)
    assert abs(float(np.asarray(r["V_RL"])[-1])) < 0.05


def test_closed_switch_passes_supply():
    """Switch closed (Ctrl = 1): 10 V over 1 kOhm through Ron = 10 mOhm gives 9.9999 V."""
    c = sp.Circuit("sw")
    c.add("V", "VS", value=10.0)
    c.ground("VS.B")
    c.add("S", "U")
    c.add("R", "RL", value=1e3)
    c.add("CONST", "K", value=1.0)
    c.connect("VS.A", "U.A")
    c.connect("U.B", "RL.A")
    c.connect("K.Out", "U.Ctrl")
    c.ground("RL.B")
    r = _sim(c, 1e-3, 1e-6)
    expected = 10.0 * 1e3 / (1e3 + 0.01)
    assert float(np.asarray(r["V_RL"])[-1]) == pytest.approx(expected, abs=1e-3)
