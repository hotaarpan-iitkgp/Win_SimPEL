"""Model 6 - Single-switch forward converter with a reset winding.

48 V input, three-winding transformer Np:Nr:Ns = 20:20:10 (n = Ns/Np = 0.5),
50 kHz, D = 0.4 (must stay below Np/(Np+Nr) = 0.5 for the core to reset).

The XFMR's magnetising inductance Lm sits across the primary (signal
I_T1_Lm). The reset winding (P2) is wired in reverse and returns the
magnetising energy to the input through D_reset.

Theory (CCM output filter, diode drops Vd):
    Vout          = n*D*Vin - Vd
    I(Lm) peak    = Vin*D / (Lm*fsw),   I(Lm) returns to 0 every cycle (core reset)
    Vds(reset)    = Vin*(1 + Np/Nr) + Vd*Np/Nr
    dI(Lo)        = (n*Vin - Vd - Vout)*D / (Lo*fsw)
"""

from __future__ import annotations

import simpel as sp

try:
    from . import _common as cm
except ImportError:
    import _common as cm

NAME = "m06_forward_reset"
TITLE = "Forward with reset winding, open-loop PWM"
CASES = [dict(D=0.4), dict(D=0.3)]

VIN, FSW, NP, NR, NS = 48.0, 50e3, 20, 20, 10
LM, LO, CO, R, RON, VD = 1e-3, 200e-6, 100e-6, 4.8, 10e-3, 0.7
N = NS / NP


def build(D: float = 0.4) -> sp.Circuit:
    c = sp.Circuit(NAME)
    vin = c.add("V", "Vin", value=VIN)
    x1 = c.add("XFMR", "T1", primary_turns=f"[{NP} {NR}]", secondary_turns=f"[{NS}]", polarity="non_inverted",
               Lm=LM)
    q1 = c.add("MOSFET", "Q1", Ron=RON)
    drst = c.add("D", "Dreset", Vd=VD)
    d1 = c.add("D", "D1", Vd=VD)
    d2 = c.add("D", "D2", Vd=VD)
    lo = c.add("L", "Lo", L=LO)
    co = c.add("C", "Co", C=CO)
    r1 = c.add("R", "Rload", value=R)
    # Primary
    c.connect(vin.A, x1.P1A, drst.B)
    c.connect(x1.P1B, q1.D)
    c.ground(vin.B, q1.S)
    # Reset winding, reversed: P2A at the input return, P2B to the reset diode
    c.ground(x1.P2A)
    c.connect(x1.P2B, drst.A)
    # Secondary: forward diode, freewheeling diode, LC filter
    c.connect(x1.S1A, d1.A)
    c.connect(d1.B, d2.B, lo.A)
    c.connect(lo.B, co.A, r1.A)
    c.ground(x1.S1B, d2.A, co.B, r1.B)
    tri = c.add("TRI_GEN", "Carrier", frequency=FSW, min=0, max=1)
    duty = c.add("CONST", "Duty", value=D)
    pwm = c.add("COMP", "PWM")
    c.connect(duty.Out, pwm.In1)
    c.connect(tri.Out, pwm.In2)
    c.connect(pwm.Out, q1.G)
    return c


def settings(D: float = 0.4) -> dict:
    return dict(t_stop=20e-3, dt=0.1e-6)


def checks(res: sp.Results, D: float = 0.4) -> list[cm.Check]:
    t1 = 16e-3
    vo = res.stats("V_Co", t1).mean
    im = res.stats("I_T1_Lm", t1)
    ilo = res.stats("I_Lo", t1)
    vds = res.stats("V_Q1", t1)
    im_pk = VIN * D / (LM * FSW)
    return [
        cm.approx("Vout [V] (= n*D*Vin - Vd)", vo, N * D * VIN - VD, rel=0.02),
        cm.approx("I(Lm) peak [A] (= Vin*D/(Lm*fsw))", im.max, im_pk, rel=0.03),
        cm.less("I(Lm) min [A] (core resets to 0)", abs(im.min), 0.01 * im_pk),
        cm.approx("Vds during reset [V]", vds.max, VIN * (1 + NP / NR) + VD * NP / NR, rel=0.02),
        cm.approx("I(Lo) mean [A] (= Iout)", ilo.mean, vo / R, rel=0.01),
        cm.approx("I(Lo) ripple pk-pk [A]", ilo.pk2pk, (N * VIN - VD - vo) * D / (LO * FSW), rel=0.06),
    ]


if __name__ == "__main__":
    cm.run_model(__import__(__name__))
