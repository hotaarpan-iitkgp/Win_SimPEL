"""A02 - Single-phase 7-level cascaded H-bridge (CHB), level-shifted PD-PWM.

Three H-bridge cells in series, each with its own isolated 100 V source. Six
carriers occupy the bands [-1,-2/3] ... [2/3,1] (phase disposition, 2 kHz).
Cell k (k = 1..3) compares the 50 Hz reference against band k (upper) and
band -k (lower):
    left leg  upper switch on  <=>  ref > carrier_up_k   -> cell = +Vdc
    right leg upper switch on  <=>  ref < carrier_lo_k   -> cell = -Vdc
    otherwise both lower switches on                      -> cell = 0
Load: R = 10 ohm, L = 20 mH.

Theory:
    Output voltage has 2N+1 = 7 levels (5 when ma < 2/3, 3 when ma < 1/3)
    Fundamental (peak) = ma * N * Vdc
    Sum of the three cell DC powers = load power + conduction loss
    With level-shifted PWM the inner cell (k = 1) carries the most power and the
    outer cell (k = 3) the least: P1 > P2 > P3.
"""

from __future__ import annotations

import math

import simpel as sp

try:
    from . import _adv as cm
except ImportError:
    import _adv as cm

NAME = "a02_chb7l_inverter"
TITLE = "7-level cascaded H-bridge, level-shifted PD-PWM"
CASES = [dict(ma=0.95), dict(ma=0.5)]

N, VCELL, F1, FC, R, L, RON = 3, 100.0, 50.0, 2e3, 10.0, 20e-3, 2e-3
VDEV = 0.8  # engine default forward drop of a switch / anti-parallel diode


def build(ma: float = 0.95) -> sp.Circuit:
    c = sp.Circuit(NAME)
    ref = c.add("SINE_WAVE", "Ref", amplitude=ma, frequency=F1, phase=0)
    prev = None  # node of the previous cell's right output
    first_left = None
    for k in range(1, N + 1):
        vs = c.add("V", f"Vdc{k}", value=VCELL)
        sw = {n: c.add("IGBT_DIODE", f"Q{n}_{k}", Ron=RON) for n in ("LU", "LL", "RU", "RL")}
        # DC bus of this (isolated) cell: + rail = Vdc.A, - rail = Vdc.B
        c.connect(vs.A, sw["LU"].D, sw["RU"].D)
        c.connect(vs.B, sw["LL"].S, sw["RL"].S)
        c.connect(sw["LU"].S, sw["LL"].D)  # left leg midpoint  (cell +)
        c.connect(sw["RU"].S, sw["RL"].D)  # right leg midpoint (cell -)
        if prev is None:
            first_left = sw["LU"].S
        else:
            c.connect(prev, sw["LU"].S)
        prev = sw["RU"].S

        lo, hi = (k - 1) / N, k / N
        cu = c.add("TRI_GEN", f"CarU{k}", frequency=FC, min=lo, max=hi)
        cl = c.add("TRI_GEN", f"CarL{k}", frequency=FC, min=-hi, max=-lo)
        cmp_l = c.add("COMP", f"CmpL{k}")   # left leg: ref > upper carrier
        cmp_r = c.add("COMP", f"CmpR{k}")   # right leg: lower carrier > ref
        nl = c.add("NOT", f"NotL{k}")
        nr = c.add("NOT", f"NotR{k}")
        c.connect(ref.Out, cmp_l.In1, cmp_r.In2)
        c.connect(cu.Out, cmp_l.In2)
        c.connect(cl.Out, cmp_r.In1)
        c.connect(cmp_l.Out, sw["LU"].G, nl.In)
        c.connect(nl.Out, sw["LL"].G)
        c.connect(cmp_r.Out, sw["RU"].G, nr.In)
        c.connect(nr.Out, sw["RL"].G)

    r = c.add("R", "Rload", value=R)
    l = c.add("L", "Lload", L=L)
    c.connect(first_left, r.A)
    c.connect(r.B, l.A)
    c.ground(l.B, prev)
    vo = c.add("VM", "Vout")
    c.connect(vo.A, first_left)
    c.ground(vo.B)
    return c


def settings(ma: float = 0.95) -> dict:
    return dict(t_stop=4 / F1, dt=2e-6)


def checks(res: sp.Results, ma: float = 0.95) -> list[cm.Check]:
    t_ss = 2 / F1
    w = 2 * math.pi * F1
    z = complex(R, w * L)
    phi = math.atan2(w * L, R)
    fv = res.fourier("Vout.Out", f0=F1)
    fi = res.fourier("I_Rload", f0=F1)
    # Two devices conduct in every cell at all times; their drop is a square wave in
    # phase with the current, which removes (4/pi)*2N*Vd*cos(phi) from the fundamental.
    v_th = ma * N * VCELL - (4 / math.pi) * 2 * N * VDEV * math.cos(phi)
    n_lv = len(cm.levels(cm.window(res, "Vout.Out", t_ss), 0.3 * VCELL))
    n_exp = 2 * math.ceil(ma * N - 1e-9) + 1
    # DC power delivered by each cell: V * (-I_source) (SPICE sign: I flows A->B inside)
    p_cell = [VCELL * -cm.mean(res, f"I_Vdc{k}", t_ss) for k in range(1, N + 1)]
    i_load = cm.window(res, "I_Rload", t_ss)
    p_load = R * float((i_load ** 2).mean())
    p_cond = 2 * N * VDEV * float(abs(i_load).mean())  # + Ron*I^2, negligible here
    return [
        cm.approx("Vout fundamental [V pk] (= ma*N*Vdc - device drops)", fv.fundamental, v_th, rel=0.01),
        cm.approx(f"Output levels (2*ceil(ma*N)+1 = {n_exp})", n_lv, n_exp, abs=0),
        cm.approx("Iload fundamental [A pk] (= V1/|Z|)", fi.fundamental, v_th / abs(z), rel=0.015),
        cm.less("Iload THD [%]", fi.thd_percent, 3.0),
        cm.approx("Sum of cell powers = load + conduction loss [W]", sum(p_cell), p_load + p_cond, rel=0.01),
        cm.greater("Inner cell carries most power: P1 - P2 [W]", p_cell[0] - p_cell[1], 0.0),
        cm.greater("P2 - P3 [W]", p_cell[1] - p_cell[2], -1e-6 if ma <= 2 / 3 else 0.0),
    ]


if __name__ == "__main__":
    cm.run_model(__import__(__name__))
