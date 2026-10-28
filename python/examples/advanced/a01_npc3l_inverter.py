"""A01 - Three-phase 3-level NPC inverter, phase-disposition PWM, split DC link.

600 V DC feeds two 2 mF capacitors through 50 mOhm; their midpoint O is the
neutral point. Each leg has four IGBTs with anti-parallel diodes (S1..S4) and two
clamp diodes. PD-PWM: an upper carrier (0..1) and a lower carrier (-1..0), both
at 5 kHz, compared with the 50 Hz references; S3 = not S1, S4 = not S2.
Load: star RL (10 ohm, 10 mH), floating star point.

Theory (linear range):
    Pole voltage vaO takes 3 levels {-Vdc/2, 0, +Vdc/2}; line voltage 5 levels.
    VaO fundamental (peak) = ma * Vdc/2;  Vab = sqrt3 * VaO, leading by 30 deg
    Ia = VaO1 / |R + j w L|, lagging by atan(wL/R)
    Neutral point: mean(Vc1 - Vc2) ~ 0 for a balanced load (PD-PWM self-balances)
"""

from __future__ import annotations

import math

import numpy as np

import simpel as sp

try:
    from . import _adv as cm
except ImportError:
    import _adv as cm

NAME = "a01_npc3l_inverter"
TITLE = "3-level NPC inverter, PD-PWM, split DC link"
CASES = [dict(ma=0.9), dict(ma=0.45)]

VDC, F1, FC, R, L, CDC, RON = 600.0, 50.0, 5e3, 10.0, 10e-3, 2e-3, 2e-3
PHASES = {"A": 0.0, "B": -120.0, "C": 120.0}


def build(ma: float = 0.9) -> sp.Circuit:
    c = sp.Circuit(NAME)
    vdc = c.add("V", "Vdc", value=VDC)
    rdc = c.add("R", "Rdc", value=0.05)
    c1 = c.add("C", "C1", C=CDC, vC0=VDC / 2)
    c2 = c.add("C", "C2", C=CDC, vC0=VDC / 2)
    c.connect(vdc.A, rdc.A)
    c.connect(rdc.B, c1.A)                 # P rail
    c.connect(c1.B, c2.A)                  # neutral point O
    c.ground(c2.B, vdc.B)                  # N rail is ground

    car_u = c.add("TRI_GEN", "CarU", frequency=FC, min=0, max=1)
    car_l = c.add("TRI_GEN", "CarL", frequency=FC, min=-1, max=0)
    load_n = []
    for ph, ang in PHASES.items():
        s = [c.add("IGBT_DIODE", f"S{k}{ph}", Ron=RON) for k in (1, 2, 3, 4)]
        dc1 = c.add("D", f"Dc1{ph}", Vd=0.0, Ron=1e-3)
        dc2 = c.add("D", f"Dc2{ph}", Vd=0.0, Ron=1e-3)
        c.connect(c1.A, s[0].D)            # P
        c.connect(s[0].S, s[1].D, dc1.B)   # x1
        c.connect(s[1].S, s[2].D)          # phase output
        c.connect(s[2].S, s[3].D, dc2.A)   # x2
        c.ground(s[3].S)                   # N
        c.connect(c1.B, dc1.A, dc2.B)      # clamp to O
        r = c.add("R", f"R{ph}", value=R)
        l = c.add("L", f"L{ph}", L=L)
        c.connect(s[1].S, r.A)
        c.connect(r.B, l.A)
        load_n.append(l.B)

        ref = c.add("SINE_WAVE", f"Ref{ph}", amplitude=ma, frequency=F1, phase=ang)
        cu = c.add("COMP", f"CmpU{ph}")
        cl = c.add("COMP", f"CmpL{ph}")
        n1 = c.add("NOT", f"NotU{ph}")
        n2 = c.add("NOT", f"NotL{ph}")
        c.connect(ref.Out, cu.In1, cl.In1)
        c.connect(car_u.Out, cu.In2)
        c.connect(car_l.Out, cl.In2)
        c.connect(cu.Out, s[0].G, n1.In)   # S1 / S3 complementary
        c.connect(n1.Out, s[2].G)
        c.connect(cl.Out, s[1].G, n2.In)   # S2 / S4 complementary
        c.connect(n2.Out, s[3].G)
    c.connect(*load_n)

    vao = c.add("VM", "VaO")
    vab = c.add("VM", "Vab")
    c.connect(vao.A, "RA.A", vab.A)
    c.connect(vao.B, c1.B)
    c.connect(vab.B, "RB.A")
    return c


def settings(ma: float = 0.9) -> dict:
    return dict(t_stop=4 / F1, dt=1e-6)


def checks(res: sp.Results, ma: float = 0.9) -> list[cm.Check]:
    w = 2 * math.pi * F1
    t_ss = 2 / F1
    van_th = ma * VDC / 2
    z = complex(R, w * L)
    fvao = res.fourier("VaO.Out", f0=F1)
    fvab = res.fourier("Vab.Out", f0=F1)
    fi = {ph: res.fourier(f"I_R{ph}", f0=F1) for ph in PHASES}
    amps = [f.fundamental for f in fi.values()]
    vao = cm.window(res, "VaO.Out", t_ss)
    vab = cm.window(res, "Vab.Out", t_ss)
    tol = 0.1 * VDC / 2
    n_pole = len(cm.levels(vao, tol))
    n_line = len(cm.levels(vab, tol))
    np_err = cm.mean(res, "V_C1", t_ss) - cm.mean(res, "V_C2", t_ss)
    expect_line_levels = 5 if ma > 0.5 else 3
    return [
        cm.approx("VaO fundamental [V pk] (= ma*Vdc/2)", fvao.fundamental, van_th, rel=0.015),
        cm.approx("Vab fundamental [V pk] (= sqrt3*VaO)", fvab.fundamental, math.sqrt(3) * van_th, rel=0.015),
        cm.approx("Vab leads VaO [deg]", cm.wrap180(cm.phase_deg(fvab) - cm.phase_deg(fvao)), 30.0, abs=1.0),
        cm.approx("Pole-voltage levels", n_pole, 3, abs=0),
        cm.approx(f"Line-voltage levels ({expect_line_levels} for ma {'>' if ma > 0.5 else '<'} 0.5)", n_line, expect_line_levels, abs=0),
        cm.approx("Ia fundamental [A pk] (= VaO1/|Z|)", amps[0], van_th / abs(z), rel=0.02),
        cm.approx("Ia lags VaO [deg] (= atan(wL/R))", cm.wrap180(cm.phase_deg(fvao) - cm.phase_deg(fi["A"])),
                  math.degrees(math.atan2(w * L, R)), abs=1.5),
        cm.less("Current imbalance [%]", 100 * (max(amps) / min(amps) - 1), 1.0),
        cm.less("Neutral-point offset |Vc1-Vc2| [% of Vdc]", 100 * abs(np_err) / VDC, 1.0),
        cm.less("Ia THD [%]", fi["A"].thd_percent, 5.0),
    ]


if __name__ == "__main__":
    cm.run_model(__import__(__name__))
