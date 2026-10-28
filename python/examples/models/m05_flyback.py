"""Model 5 - Flyback converter, open-loop PWM, continuous conduction.

48 V input, Np:Ns = 20:10 (n = Ns/Np = 0.5), 50 kHz, D = 0.4.

The XFMR's magnetising inductance Lm (referred to the primary) stores the
energy; its current is the signal I_T1_Lm. The default "inverted" polarity gives
the flyback dot arrangement: the secondary diode blocks while the switch is on
and conducts while it is off.

Theory (CCM, diode drop Vd):
    Vout + Vd   = n * D/(1-D) * Vin
    I(Lm) mean  = n * Iout / (1-D)       (secondary carries I(Lm)/n while off)
    dI(Lm)      = Vin * D / (Lm * fsw)                  (peak-to-peak)
    Vds(off)    = Vin + (Vout + Vd)/n
"""

from __future__ import annotations

import simpel as sp

try:
    from . import _common as cm
except ImportError:
    import _common as cm

NAME = "m05_flyback"
TITLE = "Flyback, open-loop PWM (CCM)"
CASES = [dict(D=0.4), dict(D=0.5)]

VIN, FSW, NP, NS, LM, CO, R, RON, VD = 48.0, 50e3, 20, 10, 500e-6, 100e-6, 10.0, 10e-3, 0.7
N = NS / NP


def build(D: float = 0.4) -> sp.Circuit:
    c = sp.Circuit(NAME)
    vin = c.add("V", "Vin", value=VIN)
    x1 = c.add("XFMR", "T1", primary_turns=f"[{NP}]", secondary_turns=f"[{NS}]", polarity="inverted", Lm=LM)
    q1 = c.add("MOSFET", "Q1", Ron=RON)
    d1 = c.add("D", "D1", Vd=VD)
    co = c.add("C", "Co", C=CO)
    r1 = c.add("R", "Rload", value=R)
    c.connect(vin.A, x1.P1A)
    c.connect(x1.P1B, q1.D)
    c.connect(x1.S1A, d1.A)
    c.connect(d1.B, co.A, r1.A)
    c.ground(vin.B, q1.S)
    c.ground(x1.S1B, co.B, r1.B)
    tri = c.add("TRI_GEN", "Carrier", frequency=FSW, min=0, max=1)
    duty = c.add("CONST", "Duty", value=D)
    pwm = c.add("COMP", "PWM")
    c.connect(duty.Out, pwm.In1)
    c.connect(tri.Out, pwm.In2)
    c.connect(pwm.Out, q1.G)
    return c


def settings(D: float = 0.4) -> dict:
    return dict(t_stop=30e-3, dt=0.1e-6)


def checks(res: sp.Results, D: float = 0.4) -> list[cm.Check]:
    t1 = 25e-3
    vo = res.stats("V_Co", t1).mean
    im = res.stats("I_T1_Lm", t1)
    vds = res.stats("V_Q1", t1)
    return [
        cm.approx("Vout [V] (= n*D/(1-D)*Vin - Vd)", vo, N * D / (1 - D) * VIN - VD, rel=0.02),
        cm.approx("I(Lm) mean [A] (= n*Iout/(1-D))", im.mean, N * (vo / R) / (1 - D), rel=0.03),
        cm.approx("I(Lm) ripple pk-pk [A]", im.pk2pk, VIN * D / (LM * FSW), rel=0.05),
        cm.approx("Vds off-state [V] (= Vin + (Vout+Vd)/n)", vds.max, VIN + (vo + VD) / N, rel=0.02),
        cm.greater("I(Lm) min > 0 (CCM) [A]", im.min, 0.0),
    ]


if __name__ == "__main__":
    cm.run_model(__import__(__name__))
