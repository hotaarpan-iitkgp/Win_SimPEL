"""A08 - Two grid-forming inverters with P-f / Q-V droop sharing an islanded load.

Each inverter: split 800 V DC link, two-level IGBT bridge (PWM_3PH 10 kHz), LC
filter (3 mH / 20 uF with 4 ohm damping), line 1.5 mH + 0.05 ohm to a common AC
bus. Bus load: star R = 10 ohm/phase, a second 20 ohm/phase set is switched in
at 0.35 s. No grid: the inverters set frequency and voltage themselves.

Droop controller (native blocks + one CSCRIPT for instantaneous p, q):
    p, q from capacitor voltages and line currents, 1st-order low-pass at 5 Hz
    w_i = w0 - mp_i P_i,   E_i = E0 - nq_i Q_i,   theta_i = integral(w_i)
    v_abc* = DQ_ABC(d = E_i, q = 0, theta_i),  modulation = v*/(Vdc/2)
Inverter 2 has twice the droop gains of inverter 1 (mp2 = 2 mp1, nq2 = 2 nq1).

Theory checks (steady state, before and after the load step):
    common frequency;  P1 / P2 = mp2 / mp1 = 2  (static P-f droop sharing)
    f = f0 - mp1 P1 / 2 pi, and the bus PLL reads the same frequency
    P1 + P2 = load power + line loss
    frequency deviation after the step grows by mp1 dP1 / 2 pi
"""

from __future__ import annotations

import math

import numpy as np

import simpel as sp

try:
    from . import _adv as cm
except ImportError:
    import _adv as cm

NAME = "a08_droop_parallel_gfm"
TITLE = "Two droop-controlled grid-forming inverters, islanded load step"
CASES = [dict()]

VDC, E0, F0, FSW, RON = 800.0, 325.0, 50.0, 10e3, 5e-3
LF, CF, RD, LL, RL = 3e-3, 20e-6, 4.0, 1.5e-3, 0.05
R1, R2, T_STEP, T_STOP = 10.0, 20.0, 0.35, 0.7
W0 = 2 * math.pi * F0
MP1 = 2 * math.pi * 0.5 / 10e3          # 0.5 Hz per 10 kW
NQ1 = 1e-3                               # V per var
FPQ = 5.0                                # p, q low-pass corner [Hz]

PQ_CODE = """
void step() {
    double va = inputs[0];
    double vb = inputs[1];
    double vc = inputs[2];
    double ia = inputs[3];
    double ib = inputs[4];
    double ic = inputs[5];
    outputs[0] = va * ia + vb * ib + vc * ic;
    outputs[1] = ((vb - vc) * ia + (vc - va) * ib + (va - vb) * ic) / sqrt(3.0);
}
"""


def _inverter(c: sp.Circuit, k: int, bus: dict, mp: float, nq: float) -> None:
    vp = c.add("V", f"VdcP{k}", value=VDC / 2)
    vn = c.add("V", f"VdcN{k}", value=VDC / 2)
    c.connect(vn.A, vp.B)
    c.ground(vn.A)
    pwm = c.add("PWM_3PH", f"PWM{k}", fc=FSW)
    pq = c.add("CSCRIPT", f"PQ{k}", code=PQ_CODE)
    for j, ph in enumerate("ABC"):
        up = c.add("IGBT_DIODE", f"Q{k}{ph}u", Ron=RON)
        lo = c.add("IGBT_DIODE", f"Q{k}{ph}l", Ron=RON)
        lf = c.add("L", f"Lf{k}{ph}", L=LF)
        cf = c.add("C", f"Cf{k}{ph}", C=CF)
        rd = c.add("R", f"Rd{k}{ph}", value=RD)
        am = c.add("AM", f"I{k}{ph}")
        ll = c.add("L", f"Ll{k}{ph}", L=LL)
        rl = c.add("R", f"Rl{k}{ph}", value=RL)
        vm = c.add("VM", f"V{k}{ph}")
        c.connect(vp.A, up.D)
        c.connect(up.S, lo.D, lf.A)
        c.connect(lo.S, vn.B)
        c.connect(lf.B, rd.A, am.A, vm.A)
        c.connect(rd.B, cf.A)
        c.ground(cf.B, vm.B)
        c.connect(am.B, ll.A)
        c.connect(ll.B, rl.A)
        c.connect(rl.B, bus[ph])
        inv = c.add("NOT", f"N{k}{ph}")
        c.connect(getattr(pwm, f"Out{ph}"), up.G, inv.In)
        c.connect(inv.Out, lo.G)
        c.connect(vm.Out, getattr(pq, f"In{j + 1}"))
        c.connect(am.Out, getattr(pq, f"In{j + 4}"))

    # FILTER_1ST's "type" parameter cannot be passed to Circuit.add (it collides with
    # add's own `type` argument); Lowpass is the default, otherwise set fp["type"].
    fp = c.add("FILTER_1ST", f"Pf{k}", fc=FPQ)
    fq = c.add("FILTER_1ST", f"Qf{k}", fc=FPQ)
    c.connect(pq.Out1, fp.In); c.connect(pq.Out2, fq.In)
    gp = c.add("GAIN", f"Mp{k}", K=mp)
    gq = c.add("GAIN", f"Nq{k}", K=nq)
    c.connect(fp.Out, gp.In); c.connect(fq.Out, gq.In)
    w0 = c.add("CONST", f"W0_{k}", value=W0)
    e0 = c.add("CONST", f"E0_{k}", value=E0)
    w = c.add("SUM", f"W{k}", signs="+-")
    e = c.add("SUM", f"E{k}", signs="+-")
    c.connect(w0.Out, w.In1); c.connect(gp.Out, w.In2)
    c.connect(e0.Out, e.In1); c.connect(gq.Out, e.In2)
    th = c.add("INTEGRATOR", f"Th{k}", initial_condition=0)
    c.connect(w.Out, th.In)
    zero = c.add("CONST", f"Z{k}", value=0)
    dq = c.add("DQ_ABC", f"Vref{k}")
    c.connect(e.Out, dq.d); c.connect(zero.Out, dq.q); c.connect(th.Out, dq.Theta)
    for ph in "ABC":
        g = c.add("GAIN", f"M{k}{ph}", K=1 / (VDC / 2))
        c.connect(getattr(dq, ph), g.In)
        c.connect(g.Out, getattr(pwm, ph))


def build() -> sp.Circuit:
    c = sp.Circuit(NAME)
    loads = {}
    sw = {}
    for ph in "ABC":
        r1 = c.add("R", f"Rload{ph}", value=R1)
        r2 = c.add("R", f"Rstep{ph}", value=R2)
        s = c.add("S", f"Sw{ph}", Ron=1e-3)
        c.connect(r1.A, r2.A)
        c.connect(r2.B, s.A)
        c.ground(r1.B, s.B)
        loads[ph] = r1.A
        sw[ph] = s
        vb = c.add("VM", f"Vbus{ph}")
        c.connect(vb.A, r1.A); c.ground(vb.B)
    step = c.add("STEP", "LoadStep", step_time=T_STEP, initial_value=0, final_value=1)
    c.connect(step.Out, *(s.Ctrl for s in sw.values()))
    _inverter(c, 1, loads, MP1, NQ1)
    _inverter(c, 2, loads, 2 * MP1, 2 * NQ1)
    pll = c.add("PLL_3PH", "BusPLL", fn=F0, Kp=2, Ki=200)
    c.connect("VbusA.Out", pll.A); c.connect("VbusB.Out", pll.B); c.connect("VbusC.Out", pll.C)
    return c


def settings() -> dict:
    return dict(t_stop=T_STOP, dt=1e-6)


def checks(res: sp.Results) -> list[cm.Check]:
    out = []
    w1, w2 = (T_STEP - 0.05, T_STEP), (T_STOP - 0.05, T_STOP)
    f_prev = None
    p1_prev = None
    for label, win, rl in (("before step", w1, (R1,)), ("after step", w2, (R1, R2))):
        p1 = cm.mean(res, "Pf1.Out", *win)
        p2 = cm.mean(res, "Pf2.Out", *win)
        f1 = cm.mean(res, "W1.Out", *win) / (2 * math.pi)
        f2 = cm.mean(res, "W2.Out", *win) / (2 * math.pi)
        # The PLL's Freq output ripples (PWM, load switching); average it over a
        # whole number of fundamental cycles or the mean is biased.
        n_cyc = math.floor((win[1] - win[0]) * f1)
        f_bus = cm.mean(res, "BusPLL.Freq", win[1] - n_cyc / f1, win[1])
        p_load = sum(
            float(np.mean(cm.window(res, f"V_R{n}{ph}", *win) ** 2)) / r
            for ph in "ABC" for n, r in zip(("load", "step"), rl)
        )
        p_line = sum(RL * float(np.mean(cm.window(res, f"I{k}{ph}.Out", *win) ** 2)) for k in (1, 2) for ph in "ABC")
        out += [
            cm.approx(f"Common frequency f1 - f2 [Hz] ({label})", f1 - f2, 0.0, abs=0.01),
            cm.approx(f"P1/P2 (= mp2/mp1 = 2) ({label})", p1 / p2, 2.0, rel=0.02),
            cm.approx(f"f = f0 - mp1 P1/2pi [Hz] ({label})", f1, F0 - MP1 * p1 / (2 * math.pi), abs=0.005),
            cm.approx(f"Bus PLL frequency = droop frequency [Hz] ({label})", f_bus, f1, abs=0.005),
            cm.approx(f"P1 + P2 = load + line loss [W] ({label})", p1 + p2, p_load + p_line, rel=0.01),
        ]
        if f_prev is not None:
            out.append(cm.approx("Frequency drop across the step [Hz] (= mp1 dP1/2pi)", f_prev - f1,
                                 MP1 * (p1 - p1_prev) / (2 * math.pi), rel=0.02))
        f_prev, p1_prev = f1, p1
    return out


if __name__ == "__main__":
    cm.run_model(__import__(__name__))
