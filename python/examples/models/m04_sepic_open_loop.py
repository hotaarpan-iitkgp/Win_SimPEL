"""Model 4 - SEPIC converter, open-loop PWM (step-up and step-down).

12 V input at 50 kHz, uncoupled inductors, 10 uF coupling capacitor.

Theory (CCM, diode drop Vd):
    Vout + Vd = D/(1-D) * Vin
    V(C1) mean = Vin                     (volt-second balance on L1 and L2)
    I(L2) mean = Iout,  I(L1) mean = Iin = Pin/Vin
    Pin ~= Pout * (Vout + Vd)/Vout       (diode is the dominant loss)
"""

from __future__ import annotations

import simpel as sp

try:
    from . import _common as cm
except ImportError:
    import _common as cm

NAME = "m04_sepic_open_loop"
TITLE = "SEPIC, open-loop PWM"
CASES = [dict(D=0.6), dict(D=0.4)]

VIN, FSW, L1, L2, C1, CO, R, RON, VD = 12.0, 50e3, 330e-6, 330e-6, 10e-6, 100e-6, 18.0, 10e-3, 0.7


def build(D: float = 0.6) -> sp.Circuit:
    c = sp.Circuit(NAME)
    vin = c.add("V", "Vin", value=VIN)
    l1 = c.add("L", "L1", L=L1)
    q1 = c.add("MOSFET", "Q1", Ron=RON)
    c1 = c.add("C", "C1", C=C1, esr=20e-3)
    l2 = c.add("L", "L2", L=L2)
    d1 = c.add("D", "D1", Vd=VD)
    co = c.add("C", "Co", C=CO)
    r1 = c.add("R", "Rload", value=R)
    c.connect(vin.A, l1.A)
    c.connect(l1.B, q1.D, c1.A)
    c.connect(c1.B, l2.A, d1.A)
    c.connect(d1.B, co.A, r1.A)
    c.ground(vin.B, q1.S, l2.B, co.B, r1.B)
    tri = c.add("TRI_GEN", "Carrier", frequency=FSW, min=0, max=1)
    duty = c.add("CONST", "Duty", value=D)
    pwm = c.add("COMP", "PWM")
    c.connect(duty.Out, pwm.In1)
    c.connect(tri.Out, pwm.In2)
    c.connect(pwm.Out, q1.G)
    return c


def settings(D: float = 0.6) -> dict:
    return dict(t_stop=40e-3, dt=0.1e-6)


def checks(res: sp.Results, D: float = 0.6) -> list[cm.Check]:
    t1 = 35e-3
    vo = res.stats("V_Co", t1).mean
    vo_th = D / (1 - D) * VIN - VD
    pin = VIN * res.stats("I_L1", t1).mean
    pout = res.stats("V_Rload", t1).rms ** 2 / R
    return [
        cm.approx("Vout [V] (= D/(1-D)*Vin - Vd)", vo, vo_th, rel=0.02),
        cm.approx("V(C1) mean [V] (= Vin)", res.stats("V_C1", t1).mean, VIN, rel=0.01),
        cm.approx("I(L2) mean [A] (= Iout; flows B->A)", -res.stats("I_L2", t1).mean, vo / R, rel=0.02),
        cm.approx("Efficiency (= Vout/(Vout+Vd))", pout / pin, vo / (vo + VD), abs=0.015),
    ]


if __name__ == "__main__":
    cm.run_model(__import__(__name__))
