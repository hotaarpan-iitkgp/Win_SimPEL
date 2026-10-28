"""Model 9 - Three-phase two-level inverter, sinusoidal PWM, star RL load.

600 V DC, 50 Hz references 120 deg apart, 5 kHz carrier (mf = 100), load
R = 10 ohm + L = 10 mH per phase with a floating star point.

Theory (linear SPWM, ma <= 1):
    Van fundamental (peak) = ma * Vdc/2 - (4/pi)*Vdev        (device drop, small)
    Vab fundamental (peak) = sqrt(3) * Van,  leading Van by 30 deg
    Ia = Van / |R + j*w*L|,  lagging Van by atan(w*L/R)
    Three balanced phase currents, 120 deg apart.
"""

from __future__ import annotations

import math

import simpel as sp

try:
    from . import _common as cm
except ImportError:
    import _common as cm

NAME = "m09_3ph_inverter_spwm"
TITLE = "3-phase 2-level inverter, SPWM, RL load"
CASES = [dict(ma=0.8), dict(ma=0.4)]

VDC, F1, FC, R, L, RON, VDEV = 600.0, 50.0, 5e3, 10.0, 10e-3, 5e-3, 0.8
PHASES = {"A": 0.0, "B": -120.0, "C": 120.0}


def build(ma: float = 0.8) -> sp.Circuit:
    c = sp.Circuit(NAME)
    vdc = c.add("V", "Vdc", value=VDC)
    c.ground(vdc.B)
    tri = c.add("TRI_GEN", "Carrier", frequency=FC, min=-1, max=1)
    load_n = []
    for ph, ang in PHASES.items():
        top = c.add("IGBT_DIODE", f"Q{ph}1", Ron=RON)
        bot = c.add("IGBT_DIODE", f"Q{ph}2", Ron=RON)
        r = c.add("R", f"R{ph}", value=R)
        l = c.add("L", f"L{ph}", L=L)
        c.connect(vdc.A, top.D)
        c.connect(top.S, bot.D, r.A)
        c.ground(bot.S)
        c.connect(r.B, l.A)
        load_n.append(l.B)
        ref = c.add("SINE_WAVE", f"Ref{ph}", amplitude=ma, frequency=F1, phase=ang)
        cmp_ = c.add("COMP", f"Cmp{ph}")
        inv = c.add("NOT", f"Not{ph}")
        c.connect(ref.Out, cmp_.In1)
        c.connect(tri.Out, cmp_.In2)
        c.connect(cmp_.Out, top.G, inv.In)
        c.connect(inv.Out, bot.G)
    c.connect(*load_n)  # floating star point
    van = c.add("VM", "Van")
    vab = c.add("VM", "Vab")
    c.connect(van.A, "RA.A", vab.A)
    c.connect(van.B, "LA.B")
    c.connect(vab.B, "RB.A")
    return c


def settings(ma: float = 0.8) -> dict:
    return dict(t_stop=3 / F1, dt=1e-6)


def checks(res: sp.Results, ma: float = 0.8) -> list[cm.Check]:
    w = 2 * math.pi * F1
    van_th = ma * VDC / 2 - (4 / math.pi) * VDEV
    z = complex(R, w * L)
    i_th = van_th / abs(z)
    fvan = res.fourier("Van.Out", f0=F1)
    fvab = res.fourier("Vab.Out", f0=F1)
    fi = {ph: res.fourier(f"I_R{ph}", f0=F1) for ph in PHASES}
    amps = [f.fundamental for f in fi.values()]
    ang = {ph: cm.phase_deg(f) for ph, f in fi.items()}
    out = [
        cm.approx("Van fundamental [V pk] (= ma*Vdc/2)", fvan.fundamental, van_th, rel=0.01),
        cm.approx("Vab fundamental [V pk] (= sqrt3*Van)", fvab.fundamental, math.sqrt(3) * van_th, rel=0.01),
        cm.approx("Vab leads Van [deg]", cm.wrap180(cm.phase_deg(fvab) - cm.phase_deg(fvan)), 30.0, abs=1.0),
        cm.approx("Ia fundamental [A pk] (= Van/|Z|)", amps[0], i_th, rel=0.015),
        cm.approx("Ia lags Van [deg] (= atan(wL/R))", cm.wrap180(cm.phase_deg(fvan) - ang["A"]),
                  math.degrees(math.atan2(w * L, R)), abs=1.0),
        cm.less("Current imbalance (max/min - 1) [%]", 100 * (max(amps) / min(amps) - 1), 1.0),
        cm.approx("Ib lags Ia [deg]", cm.wrap180(ang["A"] - ang["B"]), 120.0, abs=1.0),
        cm.approx("Ic lags Ib [deg]", cm.wrap180(ang["B"] - ang["C"]), 120.0, abs=1.0),
        cm.less("Ia THD [%]", fi["A"].thd_percent, 5.0),
    ]
    return out


if __name__ == "__main__":
    cm.run_model(__import__(__name__))
