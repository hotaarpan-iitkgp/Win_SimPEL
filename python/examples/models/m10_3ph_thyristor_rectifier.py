"""Model 10 - Three-phase six-pulse thyristor bridge, firing-angle control.

400 V line-to-line, 50 Hz source (V_3PH magnitude = peak phase voltage, 325 V),
highly inductive load R = 10 ohm + L = 100 mH (continuous current). Each
thyristor gets a 120 deg gate pulse starting alpha after its natural
commutation point (T1 at wt = 30 deg + alpha, then every 60 deg in the order
T1 T2 T3 T4 T5 T6).

Theory (continuous conduction, no source inductance):
    Vd   = (3*sqrt(3)/pi) * Vph_peak * cos(alpha) - 2*Vt  = 1.35 * V_LL * cos(alpha) - 2*Vt
    Id   = Vd / R
    The DC ripple repeats six times per mains cycle (harmonics at 6k * f1 only).
    The line-current fundamental lags its phase voltage by alpha.
"""

from __future__ import annotations

import math

import numpy as np

import simpel as sp

try:
    from . import _common as cm
except ImportError:
    import _common as cm

NAME = "m10_3ph_thyristor_rectifier"
TITLE = "3-phase thyristor bridge, firing angle alpha"
CASES = [dict(alpha=30), dict(alpha=60)]

VPH_PK, F1, R, L, VT = 230 * math.sqrt(2), 50.0, 10.0, 100e-3, 0.8

# (thyristor, phase, upper?) in firing order; Tk fires at 30 + alpha + 60*(k-1) deg
_ORDER = [("T1", "A", True), ("T2", "C", False), ("T3", "B", True),
          ("T4", "A", False), ("T5", "C", True), ("T6", "B", False)]


def build(alpha: float = 30) -> sp.Circuit:
    c = sp.Circuit(NAME)
    src = c.add("V_3PH", "Vs", magnitude=VPH_PK, frequency=F1, connection="Y")
    c.ground(src.N)
    rl = c.add("R", "Rload", value=R)
    ll = c.add("L", "Lload", L=L)
    vd = c.add("VM", "Vd")
    pos, neg = [rl.A, vd.A], [ll.B, vd.B]
    c.connect(rl.B, ll.A)
    period = 1 / F1
    for k, (name, ph, upper) in enumerate(_ORDER):
        t = c.add("THYRISTOR", name, Vd=VT)
        if upper:
            c.connect(src.pin(ph), t.A)
            pos.append(t.K)
        else:
            c.connect(t.K, src.pin(ph))
            neg.append(t.A)
        angle = (30 + alpha + 60 * k) % 360
        g = c.add("PULSE_GEN", f"G{name[1]}", amplitude=1, period=period, width=1 / 3,
                  delay=angle / 360 * period)
        c.connect(g.Out, t.G)
    c.connect(*pos)
    c.connect(*neg)
    return c


def settings(alpha: float = 30) -> dict:
    return dict(t_stop=0.12, dt=2e-6)


def expected_vd(alpha: float) -> float:
    return 3 * math.sqrt(3) / math.pi * VPH_PK * math.cos(math.radians(alpha)) - 2 * VT


def checks(res: sp.Results, alpha: float = 30) -> list[cm.Check]:
    t1 = res.time[-1] - 2 / F1
    vd = res.stats("Vd.Out", t1).mean
    vd_th = expected_vd(alpha)
    fv = res.fourier("Vd.Out", f0=F1, max_harmonic=12)
    low = max(fv.harmonic(n) for n in range(1, 6))
    # The line current leaves the source at terminal A: I_Vs_A is the current into A.
    ia = res.fourier("I_Vs_A", f0=F1)
    va = res.fourier("V_Vs_A", f0=F1)
    lag = cm.wrap180(cm.phase_deg(va) - (cm.phase_deg(ia) + 180.0))
    return [
        cm.approx("Vd mean [V] (= 1.35*V_LL*cos(alpha) - 2Vt)", vd, vd_th, rel=0.01),
        cm.approx("Id mean [A] (= Vd/R)", res.stats("I_Rload", t1).mean, vd_th / R, rel=0.01),
        cm.greater("Id min > 0 (continuous conduction) [A]", res.stats("I_Lload", t1).min, 0.0),
        cm.less("Vd harmonics 1..5 / 6th (six-pulse) [%]", 100 * low / fv.harmonic(6), 1.0),
        cm.approx("Ia fundamental lags Va [deg] (= alpha)", lag, alpha, abs=1.5),
    ]


if __name__ == "__main__":
    cm.run_model(__import__(__name__))
