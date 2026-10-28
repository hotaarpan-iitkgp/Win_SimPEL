"""Electrical machines, each checked against an analytic result.

Conventions shared by the models. Phase currents flow from the terminal into the machine.
Back-EMF e_x and the torque are T_e = sum_x (EMF shape_x) i_x / (speed constant), so that
electrical power equals mechanical power. The mechanics are J dw/dt = T_e - T_L - B w.

DC motor:
    V = R_a i + L_a di/dt + K_e w,   J dw/dt = K_t i - T_L - B w
    steady state: w = (K_t V / R_a - T_L) / (B + K_t K_e / R_a)

PMSM / BLDC / synchronous (star, no neutral wire, so the phase currents sum to zero):
    a resistive star load R_L makes each phase current -e_x / R_tot, with R_tot = R_s + R_L.
    The mean of the summed squared EMF gives the torque constant c:
        PMSM (sine):      c = 1.5 K^2 / R_tot
        BLDC (trapezoid): c = (20/9) K^2 / R_tot   (the star neutral removes the mean EMF)
        synchronous:      same as PMSM with K = K_f i_field
    Steady generator speed with driving torque T_d: w = T_d / (c + B).

Induction machine: steady state from the T-equivalent circuit with
    Z_s = R_s + j w_e Lls,  Z_m = j w_e Lm,  Z_r = R_r / s + j w_e Llr.
    T = 3 |I_r|^2 (R_r / s) / w_sync  and  T = T_L + B w.

SRM: L(theta) = Lavg + Ldiff cos(theta). With the rotor at alignment (theta = 0) and the
rotor held by a large inertia, the phase is an RL circuit: i(t) = (V/R)(1 - exp(-t R / L)).
"""
from __future__ import annotations

import math
import warnings

import numpy as np
import pytest

import simpel as sp

warnings.simplefilter("ignore")

RA, LA, KE, KT, J, B = 1.0, 1e-3, 0.05, 0.05, 1e-4, 1e-4


# ───────────────────────────── helpers ─────────────────────────────


def _final(r, name: str) -> float:
    return float(np.asarray(r[name], dtype=float)[-1])


def _tail_mean(r, name: str, frac: float = 0.9) -> float:
    x = np.asarray(r[name], dtype=float)
    return float(x[int(frac * len(x)):].mean())


def _star(c: sp.Circuit, terms: list[str], ohms: float) -> None:
    """Resistor from each terminal to a common star node that is grounded."""
    names = [f"RS{i}" for i in range(3)]
    for n, t in zip(names, terms):
        c.add("R", n, value=ohms)
        c.connect(t, f"{n}.A")
    c.connect(f"{names[0]}.B", f"{names[1]}.B")
    c.connect(f"{names[0]}.B", f"{names[2]}.B")
    c.ground(f"{names[0]}.B")


def _field(c: sp.Circuit, volts: float) -> None:
    c.add("V", "VF", value=volts)
    c.ground("VF.B")
    c.connect("VF.A", "M.FA")
    c.connect("M.FB", "VF.B")


def _torque_load(c: sp.Circuit, tl: float) -> None:
    c.add("CONST", "TL", value=tl)
    c.connect("TL.Out", "M.TL")


def _line_voltage(c: sp.Circuit) -> None:
    c.add("VM", "VMAB")
    c.connect("VMAB.A", "M.A")
    c.connect("VMAB.B", "M.B")


# ───────────────────────────── DC motor ─────────────────────────────


def _dc_motor(volts: float = 24.0, load_torque: float = 0.0, t_stop: float = 0.4):
    c = sp.Circuit("dc")
    c.add("V", "VS", value=volts)
    c.ground("VS.B")
    c.add("DC_MOTOR", "M", Ra=RA, La=LA, Ke=KE, Kt=KT, J=J, B=B)
    c.connect("VS.A", "M.A")
    c.ground("M.B")
    c.add("CONST", "TL", value=load_torque)
    c.connect("TL.Out", "M.TL")
    return sp.Simulation(c, t_stop=t_stop, dt=1e-5).run(check=False)


def test_dc_no_load_speed_matches_steady_state_formula():
    """w = K_t V / (B R_a + K_t K_e) = 1.2 / 2.6e-3 = 461.5 rad/s."""
    expected = KT * 24.0 / (B * RA + KT * KE)
    assert _final(_dc_motor(), "M.Speed") == pytest.approx(expected, rel=0.01)


def test_dc_load_torque_reduces_speed_by_formula():
    """With T_L = 0.01 N m: w = (K_t V / R_a - T_L) / (B + K_t K_e / R_a) = 457.7 rad/s."""
    expected = (KT * 24.0 / RA - 0.01) / (B + KT * KE / RA)
    assert _final(_dc_motor(load_torque=0.01), "M.Speed") == pytest.approx(expected, rel=0.01)


def test_dc_armature_current_follows_back_emf():
    """i = (V - K_e w) / R_a. At no load that is about 0.923 A."""
    r = _dc_motor()
    w = _final(r, "M.Speed")
    expected = (24.0 - KE * w) / RA
    assert _final(r, "I_M_La") == pytest.approx(expected, rel=0.02)


def test_dc_steady_torque_balances_friction_at_no_load():
    """With no load, K_t i = B w at steady state."""
    r = _dc_motor()
    assert KT * _final(r, "I_M_La") == pytest.approx(B * _final(r, "M.Speed"), rel=0.02)


def test_dc_reversed_supply_reverses_rotation():
    assert _final(_dc_motor(volts=-24.0), "M.Speed") == pytest.approx(-461.54, rel=0.01)


def test_dc_motor_with_series_line_resistance():
    """A 1 ohm line in series with the armature makes R = R_a + 1 in the steady-state formula."""
    c = sp.Circuit("dcline")
    c.add("V", "VS", value=24.0)
    c.ground("VS.B")
    c.add("R", "RLINE", value=1.0)
    c.add("DC_MOTOR", "M", Ra=RA, La=LA, Ke=KE, Kt=KT, J=J, B=B)
    c.connect("VS.A", "RLINE.A")
    c.connect("RLINE.B", "M.A")
    c.ground("M.B")
    c.add("CONST", "TL", value=0.0)
    c.connect("TL.Out", "M.TL")
    r = sp.Simulation(c, t_stop=0.4, dt=1e-5).run(check=False)
    expected = KT * 24.0 / (B * (RA + 1.0) + KT * KE)
    assert _final(r, "M.Speed") == pytest.approx(expected, rel=0.01)


# ───────────────────────────── BLDC / PMSM ─────────────────────────────


def _pm_generator(kind: str, load_ohms: float = 1.0, t_drive: float = 0.03):
    c = sp.Circuit("pmgen")
    c.add(kind, "M", Rs=1.0, Ls=1e-3, Ke=0.05, P=4, J=1e-4, B=1e-4)
    _torque_load(c, -t_drive)
    _star(c, ["M.A", "M.B", "M.C"], load_ohms)
    return sp.Simulation(c, t_stop=2.0, dt=1e-5).run(check=False)


def test_pmsm_generator_speed_matches_torque_balance():
    """Sine EMF: c = 1.5 Ke^2 / R_tot, w = T_d / (c + B), R_tot = 2 ohm."""
    c_pm = 1.5 * KE**2 / 2.0
    expected = 0.03 / (c_pm + 1e-4)
    assert _tail_mean(_pm_generator("PMSM_MOTOR"), "M.Speed") == pytest.approx(expected, rel=0.03)


def test_bldc_generator_speed_matches_torque_balance():
    """Trapezoidal EMF with a floating star: c = (20/9) Ke^2 / R_tot, w = T_d / (c + B)."""
    c_bl = (20.0 / 9.0) * KE**2 / 2.0
    expected = 0.03 / (c_bl + 1e-4)
    assert _tail_mean(_pm_generator("BLDC_MOTOR"), "M.Speed") == pytest.approx(expected, rel=0.03)


def _pm_open_circuit(kind: str, w0: float):
    c = sp.Circuit("pmoc")
    c.add(kind, "M", Rs=1.0, Ls=1e-3, Ke=KE, P=4, J=1e-4, B=1e-4, omega0=w0)
    _torque_load(c, -1e-4 * w0)     # cancels friction, so the speed stays at w0 with no current
    _star(c, ["M.A", "M.B", "M.C"], 1e6)
    _line_voltage(c)
    return sp.Simulation(c, t_stop=0.02, dt=1e-6).run(check=False)


def test_pmsm_open_circuit_line_voltage_peak_is_sqrt3_ke_w():
    """Sine EMF: peak V_AB = sqrt(3) Ke w."""
    w0 = 200.0
    v_ab = np.asarray(_pm_open_circuit("PMSM_MOTOR", w0)["VMAB.Out"], dtype=float)
    assert np.max(np.abs(v_ab)) == pytest.approx(math.sqrt(3.0) * KE * w0, rel=0.02)


def test_bldc_open_circuit_line_voltage_peak_is_2_ke_w():
    """Trapezoidal EMF: the flat tops of two phases differ by 2, so peak V_AB = 2 Ke w."""
    w0 = 200.0
    v_ab = np.asarray(_pm_open_circuit("BLDC_MOTOR", w0)["VMAB.Out"], dtype=float)
    assert np.max(np.abs(v_ab)) == pytest.approx(2.0 * KE * w0, rel=0.02)


# ───────────────────────────── Synchronous (wound field) ─────────────────────────────


def test_sync_open_circuit_line_voltage_follows_field_current():
    """Field 24 V on 10 ohm gives i_f = 2.4 A, so peak V_AB = sqrt(3) K_f i_f w0."""
    w0, kf, rf, vf = 100.0, 0.05, 10.0, 24.0
    c = sp.Circuit("syncoc")
    c.add("SYNC_MOTOR", "M", Rs=1.0, Ls=1e-3, Rf=rf, Lf=0.5, Kf=kf, P=2, J=1e-4, B=1e-4, omega0=w0)
    _field(c, vf)
    _torque_load(c, -1e-4 * w0)
    _star(c, ["M.A", "M.B", "M.C"], 1e6)
    _line_voltage(c)
    r = sp.Simulation(c, t_stop=0.5, dt=1e-5).run(check=False)
    v_ab = np.asarray(r["VMAB.Out"], dtype=float)
    expected = math.sqrt(3.0) * kf * (vf / rf) * w0
    assert np.max(np.abs(v_ab[int(0.8 * len(v_ab)):])) == pytest.approx(expected, rel=0.02)


def test_sync_generator_speed_matches_torque_balance():
    """Sine EMF with K = K_f i_f: w = T_d / (1.5 K^2 / R_tot + B)."""
    kf, rf, vf = 0.05, 10.0, 24.0
    k = kf * (vf / rf)
    c = sp.Circuit("syncgen")
    c.add("SYNC_MOTOR", "M", Rs=1.0, Ls=1e-3, Rf=rf, Lf=0.5, Kf=kf, P=2, J=1e-4, B=1e-4)
    _field(c, vf)
    _torque_load(c, -0.03)
    _star(c, ["M.A", "M.B", "M.C"], 1.0)
    r = sp.Simulation(c, t_stop=2.0, dt=1e-5).run(check=False)
    expected = 0.03 / (1.5 * k**2 / 2.0 + 1e-4)
    assert _tail_mean(r, "M.Speed") == pytest.approx(expected, rel=0.03)


# ───────────────────────────── Switched reluctance ─────────────────────────────


def test_srm_phase_current_follows_rl_solution_at_alignment():
    """Phases A and B in series at theta = 0 (A aligned). The rotor is held by a large inertia.

    L_A(0) = Lmax = 10 mH and L_B(-120 deg) = Lavg - Ldiff/2 = 4 mH, so L_tot = 14 mH.
    R_tot = 1 ohm (R_s = 0.5 per phase). The 1 Mohm return on phase C is negligible.
    """
    c = sp.Circuit("srm")
    c.add("V", "VS", value=10.0)
    c.ground("VS.B")
    c.add("SRM_MOTOR", "M", Rs=0.5, Lmin=2e-3, Lmax=10e-3, Nr=4, J=1e6, B=0.0, omega0=0.0, theta0=0.0)
    _torque_load(c, 0.0)
    c.connect("VS.A", "M.A")
    c.connect("M.B", "VS.B")
    c.add("R", "RC", value=1e6)
    c.connect("M.C", "RC.A")
    c.ground("RC.B")
    r = sp.Simulation(c, t_stop=0.05, dt=1e-6).run(check=False)
    t = np.asarray(r.time, dtype=float)
    i = np.asarray(r["I_M_LA"], dtype=float)
    l_tot, r_tot = 14e-3, 1.0
    for t_probe in (0.01, 0.02, 0.04):
        k = int(round(t_probe / 1e-6))
        expected = 10.0 / r_tot * (1.0 - math.exp(-t_probe * r_tot / l_tot))
        assert i[k] == pytest.approx(expected, rel=0.01), f"t={t[k]}"


@pytest.mark.parametrize("theta0, sign", [(-math.pi / 4, +1), (2.1, -1)])
def test_srm_torque_direction_follows_inductance_slope(theta0, sign):
    """Torque is 1/2 i^2 dL/dtheta. Near theta = -45 deg the summed phase inductance rises,
    so the rotor accelerates forward. At 120 deg it falls, so the rotor accelerates backward."""
    c = sp.Circuit("srmsign")
    c.add("V", "VS", value=10.0)
    c.ground("VS.B")
    c.add("SRM_MOTOR", "M", Rs=0.5, Lmin=2e-3, Lmax=10e-3, Nr=4, J=1e-4, B=0.0, omega0=0.0, theta0=theta0)
    _torque_load(c, 0.0)
    c.connect("VS.A", "M.A")
    c.connect("M.B", "VS.B")
    c.add("R", "RC", value=1e6)
    c.connect("M.C", "RC.A")
    c.ground("RC.B")
    r = sp.Simulation(c, t_stop=0.01, dt=1e-6).run(check=False)
    w = _final(r, "M.Speed")
    assert np.sign(w) == sign and abs(w) > 1.0


# ───────────────────────────── Induction ─────────────────────────────

RS_IM, RR_IM, LLS_IM, LLR_IM, LM_IM, P_IM, B_IM = 1.115, 1.083, 0.005974, 0.005974, 0.1, 2, 1e-3
F_IM = 50.0
W_SYNC_E = 2 * math.pi * F_IM
W_SYNC_M = W_SYNC_E / P_IM
V_PHASE_RMS = 180.0 / math.sqrt(2.0)


def _induction(tl: float = 0.0, t_stop: float = 1.5, reversed_sequence: bool = False):
    c = sp.Circuit("ind")
    c.add("V_3PH", "VS", magnitude=180.0, frequency=F_IM, phase=0.0)
    c.ground("VS.N")
    c.add("INDUCTION_MOTOR", "M", P=P_IM, J=0.02, B=B_IM)
    c.connect("VS.A", "M.A")
    if reversed_sequence:
        c.connect("VS.C", "M.B")
        c.connect("VS.B", "M.C")
    else:
        c.connect("VS.B", "M.B")
        c.connect("VS.C", "M.C")
    _torque_load(c, tl)
    return sp.Simulation(c, t_stop=t_stop, dt=2e-5).run(check=False)


def _tmodel_torque(s: float) -> float:
    zs = RS_IM + 1j * W_SYNC_E * LLS_IM
    zm = 1j * W_SYNC_E * LM_IM
    zr = RR_IM / s + 1j * W_SYNC_E * LLR_IM
    zp = zm * zr / (zm + zr)
    i_s = V_PHASE_RMS / (zs + zp)
    i_r = i_s * zm / (zm + zr)
    return 3.0 * abs(i_r) ** 2 * (RR_IM / s) / W_SYNC_M


def _tmodel_slip(tl: float) -> float:
    """Bisection on slip for T_e(s) = T_L + B w(s)."""
    def f(s):
        return _tmodel_torque(s) - (tl + B_IM * W_SYNC_M * (1.0 - s))
    lo, hi = 1e-4, 1.0
    f_lo = f(lo)
    for _ in range(200):
        mid = 0.5 * (lo + hi)
        f_mid = f(mid)
        if (f_mid > 0) == (f_lo > 0):
            lo, f_lo = mid, f_mid
        else:
            hi = mid
    return 0.5 * (lo + hi)


def test_induction_no_load_speed_is_near_synchronous():
    w = _tail_mean(_induction(), "M.Speed", frac=0.95)
    assert w == pytest.approx(W_SYNC_M, rel=0.01)


def test_induction_loaded_speed_matches_equivalent_circuit():
    tl = 5.0
    expected = W_SYNC_M * (1.0 - _tmodel_slip(tl))
    assert _tail_mean(_induction(tl=tl, t_stop=2.0), "M.Speed", frac=0.95) == pytest.approx(expected, rel=0.01)


def test_induction_reversed_phase_sequence_reverses_rotation():
    assert _tail_mean(_induction(reversed_sequence=True), "M.Speed", frac=0.95) == pytest.approx(-W_SYNC_M, rel=0.01)
