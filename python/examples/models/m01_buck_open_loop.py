"""Model 1 - Buck converter, open-loop PWM.

48 V -> ~24 V at 20 kHz, D = 0.5, 5 ohm load (CCM).

Theory (CCM, diode drop Vd, switch resistance Ron):
    Vout = D*Vin - (1-D)*Vd - D*Ron*Iout
    dIL  = (Vin - Vout - Ron*Iout) * D / (L*fsw)          (peak-to-peak)
    dVo  = dIL / (8*fsw*C)                                 (peak-to-peak, ESR = 0)
"""

from __future__ import annotations

import simpel as sp

try:
    from . import _common as cm
except ImportError:  # run as a script
    import _common as cm

NAME = "m01_buck_open_loop"
TITLE = "Buck, open-loop PWM"
CASES = [dict(D=0.5), dict(D=0.25)]

VIN, FSW, L, C, R, RON, VD = 48.0, 20e3, 200e-6, 100e-6, 5.0, 10e-3, 0.7


def build(D: float = 0.5) -> sp.Circuit:
    c = sp.Circuit(NAME)
    v1 = c.add("V", "Vin", value=VIN)
    q1 = c.add("MOSFET", "Q1", Ron=RON)
    d1 = c.add("D", "D1", Vd=VD)
    l1 = c.add("L", "L1", L=L)
    c1 = c.add("C", "Co", C=C)
    r1 = c.add("R", "Rload", value=R)
    c.connect(v1.A, q1.D)
    c.connect(q1.S, d1.B, l1.A)
    c.connect(l1.B, c1.A, r1.A)
    c.ground(v1.B, d1.A, c1.B, r1.B)
    # PWM: gate = 1 while D > carrier (0..1), so the duty cycle is D.
    tri = c.add("TRI_GEN", "Carrier", frequency=FSW, min=0, max=1)
    duty = c.add("CONST", "Duty", value=D)
    cmp_ = c.add("COMP", "PWM")
    c.connect(duty.Out, cmp_.In1)
    c.connect(tri.Out, cmp_.In2)
    c.connect(cmp_.Out, q1.G)
    return c


def settings(D: float = 0.5) -> dict:
    return dict(t_stop=12e-3, dt=0.1e-6)


def expected(D: float) -> dict:
    vout = D * VIN - (1 - D) * VD
    for _ in range(5):  # include the small Ron drop
        iout = vout / R
        vout = D * VIN - (1 - D) * VD - D * RON * iout
    iout = vout / R
    dil = (VIN - vout - RON * iout) * D / (L * FSW)
    return dict(vout=vout, iout=iout, dil=dil, dvo=dil / (8 * FSW * C))


def checks(res: sp.Results, D: float = 0.5) -> list[cm.Check]:
    e = expected(D)
    t1 = 10e-3
    vo = res.stats("V_Co", t1)
    il = res.stats("I_L1", t1)
    return [
        cm.approx("Vout mean [V]", vo.mean, e["vout"], rel=0.01),
        cm.approx("IL mean [A]", il.mean, e["iout"], rel=0.01),
        cm.approx("IL ripple pk-pk [A]", il.pk2pk, e["dil"], rel=0.05),
        cm.approx("Vout ripple pk-pk [V]", vo.pk2pk, e["dvo"], rel=0.20),
        cm.greater("IL min > 0 (CCM) [A]", il.min, 0.0),
    ]


if __name__ == "__main__":
    cm.run_model(__import__(__name__))
