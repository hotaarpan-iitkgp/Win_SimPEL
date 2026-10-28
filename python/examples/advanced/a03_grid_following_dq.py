"""A03 - Grid-following inverter: PLL_3PH + dq current control, P and Q steps.

Split 800 V DC link (midpoint = grid neutral), two-level IGBT bridge, L filter
(5 mH, 0.1 ohm) to a 230 V-rms / 50 Hz grid (V_3PH magnitude = phase peak 325 V).

Control (all native blocks):
    PLL_3PH on the grid voltages -> theta (va = Vm*sin(theta))
    theta_dq = theta - pi/2, so ABC_DQ is cos-aligned: vd = Vm, vq = 0
    id* = 2 P* / (3 Vm),  iq* = -2 Q* / (3 Vm)
    vd* = vd + PI(id* - id) - wL iq,  vq* = vq + PI(iq* - iq) + wL id
    DQ_ABC -> /(Vdc/2) -> PWM_3PH (10 kHz) -> upper gates, NOT -> lower gates
    PI: Kp = wc L, Ki = Kp wc / 10 with wc = 2*pi*500 rad/s
Steps: P* 0 -> 10 kW at 50 ms, Q* 0 -> 5 kvar at 150 ms.

Theory checks: PLL locks at 50 Hz with vq ~ 0, P and Q track their references,
current amplitude = 2 S / (3 Vm), unity PF before the Q step, current THD,
id rise time ~ 2.2/wc, DC-to-AC efficiency.
"""

from __future__ import annotations

import math

import numpy as np

import simpel as sp

try:
    from . import _adv as cm
except ImportError:
    import _adv as cm

NAME = "a03_grid_following_dq"
TITLE = "Grid-following inverter, PLL + dq current control"
CASES = [dict()]

VDC, VM, F1, FSW = 800.0, 325.0, 50.0, 10e3
LF, RF, RON = 5e-3, 0.1, 5e-3
P_STEP, Q_STEP, T_P, T_Q, T_STOP = 10e3, 5e3, 0.05, 0.15, 0.25
W = 2 * math.pi * F1
WC = 2 * math.pi * 500
KP = WC * LF
KI = KP * WC / 10


def build() -> sp.Circuit:
    c = sp.Circuit(NAME)
    # ── power stage ──────────────────────────────────────────────────────
    vp = c.add("V", "VdcP", value=VDC / 2)
    vn = c.add("V", "VdcN", value=VDC / 2)
    c.connect(vn.A, vp.B)
    c.ground(vn.A)                         # DC midpoint = neutral
    grid = c.add("V_3PH", "Grid", magnitude=VM, frequency=F1)
    c.ground(grid.N)
    pwm = c.add("PWM_3PH", "PWM", fc=FSW)
    for ph in "ABC":
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

    # ── synchronisation ──────────────────────────────────────────────────
    pll = c.add("PLL_3PH", "PLL", fn=F1, Kp=2, Ki=200)
    c.connect("VA.Out", pll.A); c.connect("VB.Out", pll.B); c.connect("VC.Out", pll.C)
    half_pi = c.add("CONST", "HalfPi", value=math.pi / 2)
    th = c.add("SUM", "ThDq", signs="+-")
    c.connect(pll.Theta, th.In1); c.connect(half_pi.Out, th.In2)

    vdq = c.add("ABC_DQ", "Vdq")
    idq = c.add("ABC_DQ", "Idq")
    for ph in "ABC":
        c.connect(f"V{ph}.Out", getattr(vdq, ph))
        c.connect(f"I{ph}.Out", getattr(idq, ph))
    c.connect(th.Out, vdq.Theta, idq.Theta)

    # ── references ───────────────────────────────────────────────────────
    pref = c.add("STEP", "Pref", step_time=T_P, initial_value=0, final_value=P_STEP)
    qref = c.add("STEP", "Qref", step_time=T_Q, initial_value=0, final_value=Q_STEP)
    idref = c.add("GAIN", "IdRef", K=2 / (3 * VM))
    iqref = c.add("GAIN", "IqRef", K=-2 / (3 * VM))
    c.connect(pref.Out, idref.In); c.connect(qref.Out, iqref.In)

    # ── current loops with decoupling and grid feed-forward ─────────────
    def axis(name, ref, meas, ff, cross, cross_sign):
        err = c.add("SUM", f"E{name}", signs="+-")
        pi = c.add("CONT_PID", f"PI{name}", Kp=KP, Ki=KI, Kd=0)
        wl = c.add("GAIN", f"WL{name}", K=W * LF)
        s1 = c.add("SUM", f"S{name}", signs="+" + cross_sign)
        vref = c.add("SUM", f"V{name}ref", signs="++")
        c.connect(ref, err.In1); c.connect(meas, err.In2)
        c.connect(err.Out, pi.In)
        c.connect(cross, wl.In)
        c.connect(pi.Out, s1.In1); c.connect(wl.Out, s1.In2)
        c.connect(s1.Out, vref.In1); c.connect(ff, vref.In2)
        return vref

    vdref = axis("d", idref.Out, idq.d, vdq.d, idq.q, "-")
    vqref = axis("q", iqref.Out, idq.q, vdq.q, idq.d, "+")

    dq_abc = c.add("DQ_ABC", "Vabc")
    c.connect(vdref.Out, dq_abc.d); c.connect(vqref.Out, dq_abc.q); c.connect(th.Out, dq_abc.Theta)
    for ph in "ABC":
        g = c.add("GAIN", f"M{ph}", K=1 / (VDC / 2))
        c.connect(getattr(dq_abc, ph), g.In)
        c.connect(g.Out, getattr(pwm, ph))
    return c


def settings() -> dict:
    return dict(t_stop=T_STOP, dt=1e-6)


def _rise_time(res, name, t0, target, frac=0.9):
    t = res.time
    x = np.asarray(res[name])
    m = (t >= t0) & (x >= frac * target)
    return float(t[m][0] - t0) if m.any() else float("inf")


def checks(res: sp.Results) -> list[cm.Check]:
    v3, i3 = ("VA.Out", "VB.Out", "VC.Out"), ("IA.Out", "IB.Out", "IC.Out")
    p1, q1 = cm.inst_pq(res, v3, i3, T_Q - 0.04, T_Q)
    p2, q2 = cm.inst_pq(res, v3, i3, T_STOP - 0.04)
    s1, s2 = math.hypot(p1, q1), math.hypot(p2, q2)
    # Fundamental amplitude (the raw peak also contains the switching ripple)
    ia1 = res.fourier("IA.Out", f0=F1, t2=T_Q).fundamental
    ia2 = res.fourier("IA.Out", f0=F1).fundamental
    vd = cm.mean(res, "Vdq.d", 0.1, T_Q)
    vq = cm.mean(res, "Vdq.q", 0.1, T_Q)
    fr = cm.mean(res, "PLL.Freq", 0.1)
    fi = res.fourier("IA.Out", f0=F1, cycles=2)
    id_ref = 2 * P_STEP / (3 * VM)
    t_rise = _rise_time(res, "Idq.d", T_P, id_ref)
    p_dc = sum(VDC / 2 * -cm.mean(res, f"I_{n}", T_STOP - 0.04) for n in ("VdcP", "VdcN"))
    return [
        cm.approx("PLL frequency [Hz]", fr, F1, abs=0.05),
        cm.approx("Grid vd [V] (= Vm)", vd, VM, rel=0.01),
        cm.less("|Grid vq| [V] (PLL aligned)", abs(vq), 0.01 * VM),
        cm.approx("P before Q step [W]", p1, P_STEP, rel=0.02),
        cm.less("|Q| before Q step [var]", abs(q1), 0.02 * P_STEP),
        cm.greater("Power factor before Q step", p1 / s1, 0.995),
        cm.approx("P after Q step [W]", p2, P_STEP, rel=0.02),
        cm.approx("Q after Q step [var]", q2, Q_STEP, rel=0.02),
        cm.approx("Ia fundamental [A pk] (= 2S*/3Vm) before Q step", ia1, 2 * P_STEP / (3 * VM), rel=0.01),
        cm.approx("Ia fundamental [A pk] (= 2S*/3Vm) after Q step", ia2, 2 * math.hypot(P_STEP, Q_STEP) / (3 * VM), rel=0.01),
        cm.less("Ia THD [%]", fi.thd_percent, 5.0),
        cm.less("id 90 % rise time [ms] (~2.2/wc = 0.7 ms)", 1e3 * t_rise, 1.5),
        cm.between("DC->AC efficiency [%]", 100 * p2 / p_dc, 97.0, 100.0),
    ]


if __name__ == "__main__":
    cm.run_model(__import__(__name__))
