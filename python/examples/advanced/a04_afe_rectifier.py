"""A04 - Active front end (3-phase PWM rectifier), cascaded Vdc / dq-current control.

230 V-rms grid (325 V phase peak) -> L filter (5 mH, 0.1 ohm) -> two-level IGBT
bridge -> split DC link (2 x 4.7 mF, midpoint tied to the grid neutral).
DC load 49 ohm (10 kW at 700 V); a second 49 ohm is switched in at 0.25 s.

Control:
    Vdc* ramps 560 -> 700 V in 50 ms (soft start from the diode-rectified level)
    Outer loop: PI(Vdc* - Vdc) -> id* (limited, clamping anti-windup), iq* = 0
    Inner loop: grid current i (grid -> converter) in a cos-aligned dq frame,
        vc_d = vg_d - PI(id* - id) + wL iq,   vc_q = vg_q - PI(iq* - iq) - wL id
    Modulation index = vc / (Vdc_measured / 2)  (DIVIDE block), PWM_3PH at 10 kHz
Voltage-loop tuning: dVdc/dt = (3 Vm / (2 Vdc C)) id = G id, wv = 2*pi*20 rad/s,
    Kp = wv / G, Ki = Kp wv / 4.

Theory checks: Vdc regulated before and after the load step, undershoot and
recovery, unity power factor, id = 2 P / (3 Vm), input-current THD, efficiency.
"""

from __future__ import annotations

import math

import numpy as np

import simpel as sp

try:
    from . import _adv as cm
except ImportError:
    import _adv as cm

NAME = "a04_afe_rectifier"
TITLE = "Active front end, Vdc outer loop + dq current loop"
CASES = [dict()]

VM, F1, FSW, LF, RF, RON = 325.0, 50.0, 10e3, 5e-3, 0.1, 5e-3
CH = 4.7e-3                     # each half of the DC link
CEQ = CH / 2
VDC0, VDC_REF, T_SOFT = 560.0, 700.0, 0.05
RLOAD, T_STEP, T_STOP = 49.0, 0.25, 0.45
W = 2 * math.pi * F1
WC = 2 * math.pi * 500
KPI, KII = WC * LF, WC * LF * WC / 10
G = 1.5 * VM / (VDC_REF * CEQ)
WV = 2 * math.pi * 20
KPV, KIV = WV / G, WV / G * WV / 4
ID_MAX = 80.0


def build() -> sp.Circuit:
    c = sp.Circuit(NAME)
    grid = c.add("V_3PH", "Grid", magnitude=VM, frequency=F1)
    c.ground(grid.N)
    cp = c.add("C", "Cp", C=CH, vC0=VDC0 / 2)
    cn = c.add("C", "Cn", C=CH, vC0=VDC0 / 2)
    c.connect(cp.B, cn.A)
    c.ground(cp.B)                           # DC midpoint = neutral
    r1 = c.add("R", "Rload", value=RLOAD)
    r2 = c.add("R", "Rstep", value=RLOAD)
    sw = c.add("S", "Sstep", Ron=1e-3)
    c.connect(cp.A, r1.A, r2.A)
    c.connect(r2.B, sw.A)
    c.connect(r1.B, sw.B, cn.B)
    vdc = c.add("VM", "Vdc")
    c.connect(vdc.A, cp.A); c.connect(vdc.B, cn.B)
    step = c.add("STEP", "LoadStep", step_time=T_STEP, initial_value=0, final_value=1)
    c.connect(step.Out, sw.Ctrl)

    pwm = c.add("PWM_3PH", "PWM", fc=FSW)
    for ph in "ABC":
        up = c.add("IGBT_DIODE", f"Q{ph}u", Ron=RON)
        lo = c.add("IGBT_DIODE", f"Q{ph}l", Ron=RON)
        am = c.add("AM", f"I{ph}")           # grid -> converter
        lf = c.add("L", f"Lf{ph}", L=LF)
        rf = c.add("R", f"Rf{ph}", value=RF)
        vm = c.add("VM", f"V{ph}")
        c.connect(getattr(grid, ph), am.A, vm.A)
        c.ground(vm.B)
        c.connect(am.B, rf.A)
        c.connect(rf.B, lf.A)
        c.connect(lf.B, up.S, lo.D)
        c.connect(up.D, cp.A)
        c.connect(lo.S, cn.B)
        inv = c.add("NOT", f"N{ph}")
        c.connect(getattr(pwm, f"Out{ph}"), up.G, inv.In)
        c.connect(inv.Out, lo.G)

    # synchronisation and transforms
    pll = c.add("PLL_3PH", "PLL", fn=F1, Kp=2, Ki=200)
    c.connect("VA.Out", pll.A); c.connect("VB.Out", pll.B); c.connect("VC.Out", pll.C)
    hp = c.add("CONST", "HalfPi", value=math.pi / 2)
    th = c.add("SUM", "ThDq", signs="+-")
    c.connect(pll.Theta, th.In1); c.connect(hp.Out, th.In2)
    vdq = c.add("ABC_DQ", "Vdq"); idq = c.add("ABC_DQ", "Idq")
    for ph in "ABC":
        c.connect(f"V{ph}.Out", getattr(vdq, ph)); c.connect(f"I{ph}.Out", getattr(idq, ph))
    c.connect(th.Out, vdq.Theta, idq.Theta)

    # outer DC-voltage loop
    ramp = c.add("RAMP", "Ramp", slope=(VDC_REF - VDC0) / T_SOFT, start_time=0, initial_output=VDC0)
    vref = c.add("SATURATION", "VdcRef", min=0, max=VDC_REF)
    ev = c.add("SUM", "Ev", signs="+-")
    piv = c.add("CONT_PID", "PIv", Kp=KPV, Ki=KIV, Kd=0, limit_output=True,
                upper_limit=ID_MAX, lower_limit=-ID_MAX, anti_windup=True)
    c.connect(ramp.Out, vref.In)
    c.connect(vref.Out, ev.In1); c.connect(vdc.Out, ev.In2)
    c.connect(ev.Out, piv.In)
    iq0 = c.add("CONST", "IqRef", value=0)

    def axis(name, ref, meas, vg, cross, cross_sign):
        e = c.add("SUM", f"E{name}", signs="+-")
        pi = c.add("CONT_PID", f"PI{name}", Kp=KPI, Ki=KII, Kd=0)
        wl = c.add("GAIN", f"WL{name}", K=W * LF)
        s = c.add("SUM", f"U{name}", signs="-" + cross_sign)   # -PI +/- wL*i_cross
        vc = c.add("SUM", f"Vc{name}", signs="++")
        c.connect(ref, e.In1); c.connect(meas, e.In2)
        c.connect(e.Out, pi.In)
        c.connect(cross, wl.In)
        c.connect(pi.Out, s.In1); c.connect(wl.Out, s.In2)
        c.connect(vg, vc.In1); c.connect(s.Out, vc.In2)
        return vc

    vcd = axis("d", piv.Out, idq.d, vdq.d, idq.q, "+")
    vcq = axis("q", iq0.Out, idq.q, vdq.q, idq.d, "-")
    dq_abc = c.add("DQ_ABC", "Vabc")
    c.connect(vcd.Out, dq_abc.d); c.connect(vcq.Out, dq_abc.q); c.connect(th.Out, dq_abc.Theta)
    half = c.add("GAIN", "HalfVdc", K=0.5)
    c.connect(vdc.Out, half.In)
    for ph in "ABC":
        dv = c.add("DIVIDE", f"M{ph}")
        c.connect(getattr(dq_abc, ph), dv.Num); c.connect(half.Out, dv.Den)
        c.connect(dv.Out, getattr(pwm, ph))
    return c


def settings() -> dict:
    return dict(t_stop=T_STOP, dt=1e-6)


def checks(res: sp.Results) -> list[cm.Check]:
    v3, i3 = ("VA.Out", "VB.Out", "VC.Out"), ("IA.Out", "IB.Out", "IC.Out")
    pre = res.stats("Vdc.Out", T_STEP - 0.04, T_STEP)
    post = res.stats("Vdc.Out", T_STOP - 0.04)
    dip = res.stats("Vdc.Out", T_STEP, T_STEP + 0.1)
    p1, q1 = cm.inst_pq(res, v3, i3, T_STEP - 0.04, T_STEP)
    p2, q2 = cm.inst_pq(res, v3, i3, T_STOP - 0.04)
    p_dc2 = post.mean ** 2 * (2 / RLOAD)
    id2 = cm.mean(res, "Idq.d", T_STOP - 0.04)
    fi = res.fourier("IA.Out", f0=F1, cycles=2)
    # recovery: per-cycle mean back within 1 % of Vdc*
    cyc = cm.cycle_mean(res, "Vdc.Out", F1, T_STEP)
    out = np.nonzero(np.abs(cyc - VDC_REF) > 0.01 * VDC_REF)[0]
    t_rec = 0.0 if not len(out) else (out[-1] + 1) / F1
    return [
        cm.approx("Vdc before step [V]", pre.mean, VDC_REF, rel=0.005),
        cm.approx("Vdc after step [V]", post.mean, VDC_REF, rel=0.005),
        cm.less("Vdc undershoot after 2x load step [%]", 100 * (VDC_REF - dip.min) / VDC_REF, 6.0),
        cm.less("Recovery to +/-1 % [ms]", 1e3 * t_rec, 100.0),
        cm.greater("Input PF before step", p1 / math.hypot(p1, q1), 0.995),
        cm.greater("Input PF after step", p2 / math.hypot(p2, q2), 0.995),
        cm.approx("id after step [A] (= 2P/3Vm)", id2, 2 * p2 / (3 * VM), rel=0.01),
        cm.less("Input current THD [%]", fi.thd_percent, 5.0),
        cm.between("AC->DC efficiency [%]", 100 * p_dc2 / p2, 96.0, 100.0),
    ]


if __name__ == "__main__":
    cm.run_model(__import__(__name__))
