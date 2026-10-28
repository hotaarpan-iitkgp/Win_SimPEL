"""Phase 4, interconnected circuits: blocks wired the way they are used, checked against the
steady-state average the circuit theory predicts.

    PWM switch into a resistor:  average load voltage = D * V
    Buck converter (CCM, ideal):  average output voltage = D * Vin
"""
from __future__ import annotations

import warnings

import numpy as np
import pytest

import simpel as sp

warnings.simplefilter("ignore")


def _pwm_switch_resistor(duty: float):
    """10 V through a switch driven by a PWM at 10 kHz, into 100 Ohm."""
    c = sp.Circuit("pwm_r")
    c.add("V", "VS", value=10.0)
    c.ground("VS.B")
    c.add("S", "SW")
    c.add("R", "RL", value=100.0)
    c.add("CONST", "D", value=duty)
    c.add("PWM", "P", frequency=10e3, min=0.0, max=1.0)
    c.connect("VS.A", "SW.A")
    c.connect("SW.B", "RL.A")
    c.connect("D.Out", "P.In")
    c.connect("P.Out", "SW.Ctrl")
    c.ground("RL.B")
    return sp.Simulation(c, t_stop=0.02, dt=1e-6).run(check=False)


@pytest.mark.parametrize("duty", [0.25, 0.5, 0.75])
def test_pwm_switch_average_voltage_is_duty_times_supply(duty):
    r = _pwm_switch_resistor(duty)
    v = np.asarray(r["V_RL"], dtype=float)
    steady = v[r.time > 0.005]           # discard the first few cycles
    assert np.mean(steady) == pytest.approx(duty * 10.0, abs=0.1)


def _buck(duty: float, vin: float = 12.0):
    """Buck: switch from Vin to node SW, freewheel diode from ground to SW, L = 500 uH from SW
    to the output, C = 100 uF and 10 Ohm load at the output. L is above the critical value
    (about 125 uH here), so the inductor current never reaches zero: continuous conduction.
    The freewheel diode has Vd = 0.05 V."""
    c = sp.Circuit("buck")
    c.add("V", "VS", value=vin)
    c.ground("VS.B")
    c.add("S", "SW")
    c.add("D", "FD", Vd=0.05)
    c.add("L", "L1", L=500e-6)
    c.add("C", "C1", C=100e-6)
    c.add("R", "RL", value=10.0)
    c.add("CONST", "DUTY", value=duty)
    c.add("PWM", "P", frequency=20e3, min=0.0, max=1.0)
    c.connect("VS.A", "SW.A")
    c.connect("SW.B", "L1.A")
    c.connect("SW.B", "FD.B")
    c.ground("FD.A")
    c.connect("L1.B", "RL.A")
    c.connect("L1.B", "C1.A")
    c.ground("RL.B")
    c.ground("C1.B")
    c.connect("DUTY.Out", "P.In")
    c.connect("P.Out", "SW.Ctrl")
    return sp.Simulation(c, t_stop=0.04, dt=5e-7).run(check=False)


@pytest.mark.parametrize("duty", [0.5])
def test_buck_average_output_is_duty_times_input(duty):
    """CCM average: Vout = D Vin - (1 - D) Vd = 6.0 - 0.025 = 5.975 V."""
    r = _buck(duty)
    v = np.asarray(r["V_RL"], dtype=float)
    steady = v[r.time > 0.02]            # discard the start-up transient of the LC filter
    expected = duty * 12.0 - (1.0 - duty) * 0.05
    assert np.mean(steady) == pytest.approx(expected, abs=0.1)


def _rl_pwm(duty: float, vin: float = 10.0):
    """PWM switch from Vin into a series R-L load, with a freewheel diode across the load.
    Average load voltage: D Vin - (1 - D) Vd. R = 10 Ohm, L = 10 mH, tau = 1 ms, 10 kHz PWM."""
    c = sp.Circuit("rlpwm")
    c.add("V", "VS", value=vin)
    c.ground("VS.B")
    c.add("S", "SW")
    c.add("D", "FD", Vd=0.05)
    c.add("L", "L1", L=10e-3)
    c.add("R", "RL", value=10.0)
    c.add("CONST", "DUTY", value=duty)
    c.add("PWM", "P", frequency=10e3, min=0.0, max=1.0)
    c.connect("VS.A", "SW.A")
    c.connect("SW.B", "FD.B")
    c.connect("SW.B", "L1.A")
    c.ground("FD.A")
    c.connect("L1.B", "RL.A")
    c.ground("RL.B")
    c.connect("DUTY.Out", "P.In")
    c.connect("P.Out", "SW.Ctrl")
    return sp.Simulation(c, t_stop=0.03, dt=1e-6).run(check=False)


@pytest.mark.parametrize("duty", [0.5])
def test_rl_load_on_pwm_switch_average_voltage(duty):
    r = _rl_pwm(duty)
    v = np.asarray(r["V_RL"], dtype=float)
    steady = v[r.time > 0.015]
    assert np.mean(steady) == pytest.approx(duty * 10.0 - (1 - duty) * 0.05, abs=0.1)


def test_bus_creator_and_selector_route_signals():
    """Two constants packed onto a bus and unpacked again: each output equals its input."""
    c = sp.Circuit("bus")
    c.add("CONST", "A", value=2.0)
    c.add("CONST", "B", value=5.0)
    c.add("BUS_CREATOR", "BC", inputs=2)
    c.add("BUS_SELECTOR", "BS", signals="signal1,signal2")
    c.connect("A.Out", "BC.In1")
    c.connect("B.Out", "BC.In2")
    c.connect("BC.Bus", "BS.Bus")
    r = sp.Simulation(c, t_stop=1e-3, dt=1e-5).run(check=False)
    assert float(np.asarray(r["BS.Out1"])[-1]) == pytest.approx(2.0, abs=1e-9)
    assert float(np.asarray(r["BS.Out2"])[-1]) == pytest.approx(5.0, abs=1e-9)


def test_closed_loop_pid_buck_regulates_to_setpoint():
    """Buck of the previous tests with a PID loop: a voltmeter senses the output, the error to a
    6 V setpoint drives the PID, and its output clamped to 0..1 sets the PWM duty.
    Integral action drives the average output to the setpoint."""
    c = sp.Circuit("pidbuck")
    c.add("V", "VS", value=12.0)
    c.ground("VS.B")
    c.add("S", "SW")
    c.add("D", "FD", Vd=0.05)
    c.add("L", "L1", L=500e-6)
    c.add("C", "C1", C=100e-6)
    c.add("R", "RL", value=10.0)
    c.add("VM", "SENSE")
    c.add("CONST", "SP", value=6.0)
    c.add("SUBTRACT", "ERR")
    c.add("PID", "CTRL", Kp=0.05, Ki=50.0, Kd=0.0)
    c.add("SATURATION", "CLAMP", min=0.0, max=1.0)
    c.add("PWM", "P", frequency=20e3, min=0.0, max=1.0)
    c.connect("VS.A", "SW.A")
    c.connect("SW.B", "FD.B")
    c.connect("SW.B", "L1.A")
    c.ground("FD.A")
    c.connect("L1.B", "RL.A")
    c.connect("L1.B", "C1.A")
    c.connect("L1.B", "SENSE.A")
    c.ground("SENSE.B")
    c.ground("RL.B")
    c.ground("C1.B")
    c.connect("SP.Out", "ERR.In1")
    c.connect("SENSE.Out", "ERR.In2")
    c.connect("ERR.Out", "CTRL.In")
    c.connect("CTRL.Out", "CLAMP.In")
    c.connect("CLAMP.Out", "P.In")
    c.connect("P.Out", "SW.Ctrl")
    r = sp.Simulation(c, t_stop=0.08, dt=5e-7).run(check=False)
    v = np.asarray(r["V_RL"], dtype=float)
    steady = v[r.time > 0.06]
    assert np.mean(steady) == pytest.approx(6.0, abs=0.2)


def test_pll_locks_to_fifty_hertz_three_phase():
    """Balanced 50 Hz three-phase input. After lock-in the frequency output reads 50 Hz."""
    c = sp.Circuit("pll")
    c.add("PLL_3PH", "PLL", fn=50.0, Kp=1.0, Ki=10.0)
    for name, ang in (("A", 0.0), ("B", -120.0), ("C", 120.0)):
        c.add("SINE_WAVE", f"S{name}", amplitude=1.0, frequency=50.0, phase=ang)
        c.connect(f"S{name}.Out", f"PLL.{name}")
    r = sp.Simulation(c, t_stop=0.3, dt=1e-4).run(check=False)
    f = np.asarray(r["PLL.Freq"], dtype=float)
    assert np.mean(f[r.time > 0.25]) == pytest.approx(50.0, abs=0.5)


def test_transformer_and_bridge_rectifier_average():
    """Ideal 1:1 transformer from a 50 Hz, 5 V peak source, then a diode bridge into a resistor.
    The average of a full-wave rectified sine is 2 Vpk / pi, minus two diode drops (Vd = 0.05 V)."""
    c = sp.Circuit("bridge")
    c.add("AC_V", "VS", amplitude=5.0, frequency=50.0)
    c.ground("VS.B")
    c.add("IDEAL_XFMR", "T", primary_turns=100, secondary_turns=100)
    c.add("D", "D1", Vd=0.05)
    c.add("D", "D2", Vd=0.05)
    c.add("D", "D3", Vd=0.05)
    c.add("D", "D4", Vd=0.05)
    c.add("R", "RL", value=100.0)
    c.connect("VS.A", "T.P1A")
    c.ground("T.P1B")
    c.connect("D1.A", "T.S1A")
    c.connect("D2.A", "T.S1B")
    c.connect("D1.B", "RL.A")
    c.connect("D2.B", "RL.A")
    c.connect("D3.A", "RL.B")
    c.connect("D4.A", "RL.B")
    c.connect("D3.B", "T.S1A")
    c.connect("D4.B", "T.S1B")
    r = sp.Simulation(c, t_stop=0.1, dt=1e-5).run(check=False)
    v = np.asarray(r["V_RL"], dtype=float)
    steady = v[r.time > 0.03]
    expected = 2 * 5.0 / np.pi - 2 * 0.05
    assert np.mean(steady) == pytest.approx(expected, abs=0.15)


def test_toggle_state_machine_flips_on_each_rising_edge():
    """A D flip-flop with Q_bar fed back to D is a toggle: each rising clock edge flips Q.
    The clock has a 0.2 s period, so its rising edges at 0, 0.2 and 0.4 s flip Q, and Q changes
    every 0.2 s."""
    c = sp.Circuit("toggle")
    c.add("PULSE_GEN", "CLK", amplitude=1.0, period=0.2, width=0.5, delay=0.0)
    c.add("D_FLIP_FLOP", "FF")
    c.connect("CLK.Out", "FF.Ctrl")
    c.connect("FF.Q_bar", "FF.D")
    r = sp.Simulation(c, t_stop=0.6, dt=1e-5).run(check=False)

    def q(t):
        return float(np.asarray(r["FF.Q"])[int(np.argmin(np.abs(r.time - t)))])

    # Q holds between edges: [0, 0.2) and [0.4, 0.6) are one state, [0.2, 0.4) is the other
    assert q(0.1) != q(0.3)
    assert q(0.1) == pytest.approx(q(0.5), abs=1e-9)
