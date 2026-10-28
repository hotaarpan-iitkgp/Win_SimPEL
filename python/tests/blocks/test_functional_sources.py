"""Phase 3, sources and measurement: waveform parameters and measured values against analytic answers."""
from __future__ import annotations

import math
import warnings

import numpy as np
import pytest

import simpel as sp

warnings.simplefilter("ignore")


def _sim(c: sp.Circuit, t_stop: float, dt: float = 1e-5):
    return sp.Simulation(c, t_stop=t_stop, dt=dt).run(check=False)


def test_sine_wave_peak_matches_amplitude():
    c = sp.Circuit("s")
    c.add("SINE_WAVE", "S", amplitude=2.0, frequency=10.0, phase=0)
    v = np.asarray(_sim(c, 0.2)["S.Out"])
    assert v.max() == pytest.approx(2.0, abs=1e-3)
    assert v.min() == pytest.approx(-2.0, abs=1e-3)


def test_ramp_value_at_time():
    c = sp.Circuit("r")
    c.add("RAMP", "R", slope=3.0, start_time=0.0, initial_output=0.0)
    r = _sim(c, 0.5)
    i = int(np.argmin(np.abs(r.time - 0.5)))
    assert r["R.Out"][i] == pytest.approx(1.5, abs=1e-4)


def test_step_switches_at_step_time():
    c = sp.Circuit("st")
    c.add("STEP", "S", step_time=0.5, initial_value=0.0, final_value=1.0)
    r = _sim(c, 1.0)
    before = r["S.Out"][int(np.argmin(np.abs(r.time - 0.25)))]
    after = r["S.Out"][int(np.argmin(np.abs(r.time - 0.75)))]
    assert before == pytest.approx(0.0, abs=1e-9)
    assert after == pytest.approx(1.0, abs=1e-9)


def test_pulse_generator_duty_cycle():
    """50 % duty at 50 Hz (period 20 ms, width 0.5). The mean over 0.2 s is 0.5."""
    c = sp.Circuit("p")
    c.add("PULSE_GEN", "P", amplitude=1.0, period=0.02, width=0.5, delay=0.0)
    v = np.asarray(_sim(c, 0.2, dt=1e-5)["P.Out"])
    assert np.mean(v) == pytest.approx(0.5, abs=0.01)


def test_triangle_carrier_mean_is_midpoint():
    c = sp.Circuit("t")
    c.add("TRI", "T", frequency=100.0, min=0.0, max=1.0)
    v = np.asarray(_sim(c, 0.1, dt=1e-6)["T.Out"])
    assert np.mean(v) == pytest.approx(0.5, abs=0.01)
    assert v.max() == pytest.approx(1.0, abs=0.01)


def test_constant_source_value():
    c = sp.Circuit("c")
    c.add("CONST", "K", value=4.0)
    assert float(np.asarray(_sim(c, 0.001)["K.Out"])[-1]) == pytest.approx(4.0, abs=1e-9)


def test_rms_of_sine_is_amplitude_over_root_two():
    """Amplitude 1 at 50 Hz: RMS is 1/sqrt(2) = 0.7071 once a full cycle has been measured."""
    c = sp.Circuit("rms")
    c.add("SINE_WAVE", "S", amplitude=1.0, frequency=50.0, phase=0)
    c.add("RMS_VAL", "M", frequency=50.0)
    c.connect("S.Out", "M.In")
    r = _sim(c, 0.1, dt=1e-5)
    i = int(np.argmin(np.abs(r.time - 0.08)))
    assert r["M.Out"][i] == pytest.approx(1 / math.sqrt(2), abs=0.01)
