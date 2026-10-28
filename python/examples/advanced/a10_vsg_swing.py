"""A10 - Virtual synchronous generator (VSG) on a stiff grid: swing-equation dynamics.

Split 800 V DC link, two-level IGBT bridge (PWM_3PH 10 kHz), L = 5 mH (0.05 ohm)
to a 325 V-peak / 50 Hz grid. The inverter emulates a synchronous machine:
    M dw/dt = P* - Pe - D (w - wg),   M = J w0
    theta = integral(w),   v* = DQ_ABC(d = E, q = 0, theta),  E = Vg
Pe is the instantaneous three-phase power (CSCRIPT). P* steps 0 -> 20 kW at 0.1 s.

Linearised small-signal theory (Pe = Pmax sin(delta), Pmax = 1.5 E Vg / X):
    ks = Pmax cos(delta_ss),   wn = sqrt(ks / M),   zeta = D / (2 sqrt(M ks))
    damped frequency wd = wn sqrt(1 - zeta^2)
    overshoot = exp(-zeta pi / sqrt(1 - zeta^2))
    steady state: P = P*, w = wg, delta = asin(P*/Pmax),
                  Q = 1.5 (E Vg cos(delta) - Vg^2) / X
J and D are chosen for wn = 2*pi*3 Hz and zeta = 0.3.
"""

from __future__ import annotations

import math

import numpy as np

import simpel as sp

try:
    from . import _adv as cm
    from .a08_droop_parallel_gfm import PQ_CODE
except ImportError:
    import _adv as cm
    from a08_droop_parallel_gfm import PQ_CODE

NAME = "a10_vsg_swing"
TITLE = "Virtual synchronous generator, swing-equation response"
CASES = [dict()]

VDC, VG, F0, FSW, RON = 800.0, 325.0, 50.0, 10e3, 5e-3
LF, RF = 5e-3, 0.05
W0 = 2 * math.pi * F0
X = W0 * LF
PMAX = 1.5 * VG * VG / X
P_STEP, T_STEP, T_STOP = 20e3, 0.1, 1.5   # sigma = zeta wn ~ 5.7/s: swing < 0.1 % by 1.4 s
DELTA_SS = math.asin(P_STEP / PMAX)
KS = PMAX * math.cos(DELTA_SS)
WN, ZETA = 2 * math.pi * 3.0, 0.3
M = KS / WN ** 2
D = 2 * ZETA * math.sqrt(M * KS)


def build() -> sp.Circuit:
    c = sp.Circuit(NAME)
    vp = c.add("V", "VdcP", value=VDC / 2)
    vn = c.add("V", "VdcN", value=VDC / 2)
    c.connect(vn.A, vp.B)
    c.ground(vn.A)
    grid = c.add("V_3PH", "Grid", magnitude=VG, frequency=F0)
    c.ground(grid.N)
    pwm = c.add("PWM_3PH", "PWM", fc=FSW)
    pq = c.add("CSCRIPT", "PQ", code=PQ_CODE)
    for j, ph in enumerate("ABC"):
        up = c.add("IGBT_DIODE", f"Q{ph}u", Ron=RON)
        lo = c.add("IGBT_DIODE", f"Q{ph}l", Ron=RON)
        lf = c.add("L", f"Lf{ph}", L=LF)
        rf = c.add("R", f"Rf{ph}", value=RF)
        am = c.add("AM", f"I{ph}")
        vm = c.add("VM", f"V{ph}")
        c.connect(vp.A, up.D)
        c.connect(up.S, lo.D, lf.A)
        c.connect(lo.S, vn.B)
        c.connect(lf.B, rf.A)
        c.connect(rf.B, am.A)
        c.connect(am.B, getattr(grid, ph), vm.A)
        c.ground(vm.B)
        inv = c.add("NOT", f"N{ph}")
        c.connect(getattr(pwm, f"Out{ph}"), up.G, inv.In)
        c.connect(inv.Out, lo.G)
        c.connect(vm.Out, getattr(pq, f"In{j + 1}"))
        c.connect(am.Out, getattr(pq, f"In{j + 4}"))
    pe = c.add("GAIN", "Pe", K=1)
    qe = c.add("GAIN", "Qe", K=1)
    c.connect(pq.Out1, pe.In); c.connect(pq.Out2, qe.In)

    # swing equation:  M dw/dt = P* - Pe - D (w - wg)
    pref = c.add("STEP", "Pref", step_time=T_STEP, initial_value=0, final_value=P_STEP)
    wg = c.add("CONST", "Wg", value=W0)
    dp = c.add("SUM", "dP", signs="+-")
    dw = c.add("SUM", "dW", signs="+-")
    damp = c.add("GAIN", "Damp", K=D)
    acc = c.add("SUM", "Acc", signs="+-")
    inv_m = c.add("GAIN", "InvM", K=1 / M)
    w = c.add("INTEGRATOR", "W", initial_condition=W0)
    th = c.add("INTEGRATOR", "Theta", initial_condition=-math.pi / 2)  # va_grid = Vg cos(wt - pi/2)
    c.connect(pref.Out, dp.In1); c.connect(pe.Out, dp.In2)
    c.connect(w.Out, dw.In1); c.connect(wg.Out, dw.In2)
    c.connect(dw.Out, damp.In)
    c.connect(dp.Out, acc.In1); c.connect(damp.Out, acc.In2)
    c.connect(acc.Out, inv_m.In)
    c.connect(inv_m.Out, w.In)
    c.connect(w.Out, th.In)

    e = c.add("CONST", "E", value=VG)
    zero = c.add("CONST", "Zero", value=0)
    dq = c.add("DQ_ABC", "Vref")
    c.connect(e.Out, dq.d); c.connect(zero.Out, dq.q); c.connect(th.Out, dq.Theta)
    for ph in "ABC":
        g = c.add("GAIN", f"M{ph}", K=1 / (VDC / 2))
        c.connect(getattr(dq, ph), g.In)
        c.connect(g.Out, getattr(pwm, ph))
    return c


def settings() -> dict:
    return dict(t_stop=T_STOP, dt=2e-6)


VDEV = 0.8  # device forward drop


def _steady_state_q() -> tuple[float, float]:
    """Grid-side Q and load angle for P_grid = P*, with the in-phase drops included.

    The bridge's forward drop is a square wave in phase with the current; its
    fundamental (4/pi) Vd acts like a resistance (4/pi) Vd / I. Together with Rf
    and Ron: Z = Req + jX,  I = (E e^{j d} - Vg) / Z,  S_grid = 1.5 Vg conj(I).
    """
    i_pk, d = P_STEP / (1.5 * VG), DELTA_SS
    for _ in range(50):
        z = complex(RF + RON + (4 / math.pi) * VDEV / i_pk, X)
        # solve Re(S(d)) = P* for d by Newton on the scalar function
        for _ in range(30):
            s = 1.5 * VG * np.conj((VG * np.exp(1j * d) - VG) / z)
            ds = 1.5 * VG * np.conj(1j * VG * np.exp(1j * d) / z)
            d -= (s.real - P_STEP) / ds.real
        i_pk = abs((VG * np.exp(1j * d) - VG) / z)
    return float(s.imag), d


def _fit_damped(t: np.ndarray, x: np.ndarray) -> tuple[float, float]:
    """Least-squares fit x = e^(-s t) (a sin(2 pi f t) + b cos(2 pi f t)); returns (f, s)."""
    best = (math.inf, 0.0, 0.0)
    for f in np.linspace(2.0, 4.0, 201):
        for s in np.linspace(1.0, 12.0, 111):
            e = np.exp(-s * t)
            a = np.vstack([e * np.sin(2 * math.pi * f * t), e * np.cos(2 * math.pi * f * t)]).T
            c = np.linalg.lstsq(a, x, rcond=None)[0]
            r = float(np.sum((a @ c - x) ** 2))
            if r < best[0]:
                best = (r, float(f), float(s))
    return best[1], best[2]


def checks(res: sp.Results) -> list[cm.Check]:
    t = res.time
    wd_th = WN * math.sqrt(1 - ZETA ** 2)
    os_th = math.exp(-ZETA * math.pi / math.sqrt(1 - ZETA ** 2))
    # Swing modes from the virtual rotor speed (1 ms decimation, after the step)
    k = (t > T_STEP + 0.01) & (t < T_STOP - 0.2)
    fd, sig = _fit_damped(t[k][::500] - T_STEP, (np.asarray(res["W.Out"]) - W0)[k][::500])
    wn = math.hypot(sig, 2 * math.pi * fd)
    # Overshoot of the electrical power (5 ms average removes PWM / 2f ripple)
    p = np.asarray(res["Pe.Out"])
    n = int(round(5e-3 / (t[1] - t[0])))
    pa = np.convolve(p, np.ones(n) / n, mode="same")
    m1 = (t > T_STEP) & (t < T_STEP + 2 * math.pi / wd_th)
    os_meas = (float(pa[m1].max()) - P_STEP) / P_STEP
    end = (T_STOP - 0.1, T_STOP)
    p_ss = cm.mean(res, "Pe.Out", *end)
    q_ss = cm.mean(res, "Qe.Out", *end)
    w_ss = cm.mean(res, "W.Out", *end)
    q_th, _ = _steady_state_q()
    return [
        cm.approx("Steady-state P [W] (= P*)", p_ss, P_STEP, rel=0.01),
        cm.approx("Steady-state f [Hz] (= grid)", w_ss / (2 * math.pi), F0, abs=0.002),
        cm.approx("Steady-state Q [var] (E behind Rf+Ron+drop + jX)", q_ss, q_th, rel=0.05),
        cm.approx("Damped swing frequency [Hz] (= wn sqrt(1-z^2)/2pi, fitted to w(t))", fd, wd_th / (2 * math.pi), rel=0.02),
        cm.approx("Damping ratio (= D / 2 sqrt(M ks), fitted)", sig / wn, ZETA, abs=0.03),
        cm.approx("Power overshoot [%] (= exp(-z pi/sqrt(1-z^2)))", 100 * os_meas, 100 * os_th, abs=4.0),
    ]


if __name__ == "__main__":
    cm.run_model(__import__(__name__))
