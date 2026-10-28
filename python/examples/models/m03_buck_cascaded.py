"""Model 3 - Buck converter with cascaded voltage and average-current loops.

48 V -> 24 V at 50 kHz into 4 ohm (6 A). The outer PI voltage loop produces the
inductor-current reference, clamped to I_LIM = 8 A; the inner PI current loop
sets the duty cycle. At 15 ms the load drops to 2 ohm, which would need 12 A:
the converter goes into current limit and holds IL = 8 A, Vout = 8 A * 2 ohm.

Loop design (inner loop much faster than the outer one):
    inner: plant Vin/(s*L),  Kp_i = 2*pi*f_ci*L/Vin,  Ki_i = Kp_i*2*pi*f_ci/5   (f_ci = 5 kHz)
    outer: plant 1/(s*C),    Kp_v = 2*pi*f_cv*C,      Ki_v = Kp_v*2*pi*f_cv/5   (f_cv = 500 Hz)
"""

from __future__ import annotations

import math

import simpel as sp

try:
    from . import _common as cm
except ImportError:
    import _common as cm

NAME = "m03_buck_cascaded"
TITLE = "Buck, cascaded V + I loops, current limit"
CASES = [dict()]

VIN, VREF, FSW, L, C, R, RON, VD, I_LIM = 48.0, 24.0, 50e3, 200e-6, 220e-6, 4.0, 10e-3, 0.7, 8.0
T_STEP, T_STOP = 15e-3, 30e-3

F_CI, F_CV = 5e3, 500.0
KP_I = 2 * math.pi * F_CI * L / VIN
KI_I = KP_I * 2 * math.pi * F_CI / 5
KP_V = 2 * math.pi * F_CV * C
KI_V = KP_V * 2 * math.pi * F_CV / 5


def build() -> sp.Circuit:
    c = sp.Circuit(NAME)
    vin = c.add("V", "Vin", value=VIN)
    q1 = c.add("MOSFET", "Q1", Ron=RON)
    d1 = c.add("D", "D1", Vd=VD)
    l1 = c.add("L", "L1", L=L)
    isense = c.add("AM", "Isense")
    co = c.add("C", "Co", C=C)
    r1 = c.add("R", "Rload", value=R)
    r2 = c.add("R", "Rstep", value=R)
    s1 = c.add("S", "Sstep", Ron=1e-3)
    vsense = c.add("VM", "Vsense")
    c.connect(vin.A, q1.D)
    c.connect(q1.S, d1.B, l1.A)
    c.connect(l1.B, isense.A)
    c.connect(isense.B, co.A, r1.A, r2.A, vsense.A)
    c.connect(r2.B, s1.A)
    c.ground(vin.B, d1.A, co.B, r1.B, s1.B, vsense.B)

    vref = c.add("CONST", "Vref", value=VREF)
    ev = c.add("SUBTRACT", "Ev", signs="+-")
    pi_v = c.add("CONT_PID", "PIv", Kp=KP_V, Ki=KI_V, Kd=0)
    iref = c.add("SATURATION", "Iref", min=0, max=I_LIM)
    ei = c.add("SUBTRACT", "Ei", signs="+-")
    pi_i = c.add("CONT_PID", "PIi", Kp=KP_I, Ki=KI_I, Kd=0)
    duty = c.add("SATURATION", "Duty", min=0, max=0.95)
    tri = c.add("TRI_GEN", "Carrier", frequency=FSW, min=0, max=1)
    pwm = c.add("COMP", "PWM")
    c.connect(vref.Out, ev.In1)
    c.connect(vsense.Out, ev.In2)
    c.connect(ev.Out, pi_v.In)
    c.connect(pi_v.Out, iref.In)
    c.connect(iref.Out, ei.In1)
    c.connect(isense.Out, ei.In2)
    c.connect(ei.Out, pi_i.In)
    c.connect(pi_i.Out, duty.In)
    c.connect(duty.Out, pwm.In1)
    c.connect(tri.Out, pwm.In2)
    c.connect(pwm.Out, q1.G)

    step = c.add("STEP", "LoadStep", step_time=T_STEP, initial_value=0, final_value=1)
    c.connect(step.Out, s1.Ctrl)
    return c


def settings() -> dict:
    return dict(t_stop=T_STOP, dt=0.1e-6)


def checks(res: sp.Results) -> list[cm.Check]:
    t_pre = (T_STEP - 4e-3, T_STEP)
    t_post = (T_STOP - 5e-3, T_STOP)
    vo_pre = res.stats("V_Co", *t_pre).mean
    il_pre = res.stats("I_L1", *t_pre).mean
    vo_post = res.stats("V_Co", *t_post).mean
    il_post = res.stats("I_L1", *t_post).mean
    iref_post = res.stats("Iref.Out", *t_post).mean
    il_ripple = res.stats("I_L1", *t_pre).pk2pk
    d = (VREF + VD) / (VIN + VD)  # CCM duty with the diode drop
    dil = (VIN - VREF) * d / (L * FSW)
    r_post = R / 2
    return [
        cm.approx("Vout regulated [V]", vo_pre, VREF, rel=0.005),
        cm.approx("IL = Vout/R [A]", il_pre, VREF / R, rel=0.01),
        cm.approx("IL ripple pk-pk [A]", il_ripple, dil, rel=0.10),
        cm.approx("Iref clamped at I_LIM [A]", iref_post, I_LIM, rel=1e-6),
        cm.approx("IL in current limit [A]", il_post, I_LIM, rel=0.01),
        cm.approx("Vout in current limit [V] (= I_LIM*R)", vo_post, I_LIM * r_post, rel=0.01),
        cm.less("Peak IL at startup [A] (limit + ripple)", res.stats("I_L1", 0, T_STEP).max, I_LIM + dil),
    ]


if __name__ == "__main__":
    cm.run_model(__import__(__name__))
