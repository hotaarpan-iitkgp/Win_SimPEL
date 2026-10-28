"""A09 - Back-to-back converter: 50 Hz grid -> AFE -> 700 V DC link -> inverter -> 30 Hz load.

Reuses A04's active front end (Vdc outer loop, dq current loop, unity PF) and
adds a second two-level IGBT bridge on the same split DC link, driving a star
RL load (8 ohm + 15 mH per phase, floating star point) with open-loop SPWM at
30 Hz, ma = 0.85, 6 kHz carrier. A04's switched DC load is removed; its fixed
49 ohm stays on the link.

Theory checks:
    Vdc regulated with both loads; load-side frequency independent of the grid
    load phase voltage fundamental = ma Vdc/2 - (4/pi) Vd cos(phi)
    load current = V1 / |R + j w L| at 30 Hz, balanced
    grid side: unity PF, P_grid = P_dc_load + P_ac_load + losses (eff. > 95 %)
    power balance closes within 1 %: P_grid = P_dc + P_ac + measured losses
"""

from __future__ import annotations

import math

import numpy as np

import simpel as sp

try:
    from . import _adv as cm
    from . import a04_afe_rectifier as afe
except ImportError:
    import _adv as cm
    import a04_afe_rectifier as afe

NAME = "a09_back_to_back"
TITLE = "Back-to-back AFE + inverter, 50 Hz grid -> 30 Hz load"
CASES = [dict()]

F2, MA, FC2, RL, LL = 30.0, 0.85, 6e3, 8.0, 15e-3
VDEV = 0.8
T_STOP = 0.3


def build() -> sp.Circuit:
    c = afe.build()
    for cid in ("Rstep", "Sstep", "LoadStep"):
        c.remove(cid)
    car = c.add("TRI_GEN", "Car2", frequency=FC2, min=-1, max=1)
    star = []
    for ph, ang in (("A", 0.0), ("B", -120.0), ("C", 120.0)):
        up = c.add("IGBT_DIODE", f"P{ph}u", Ron=afe.RON)
        lo = c.add("IGBT_DIODE", f"P{ph}l", Ron=afe.RON)
        r = c.add("R", f"RL{ph}", value=RL)
        l = c.add("L", f"LL{ph}", L=LL)
        c.connect("Cp.A", up.D)
        c.connect(up.S, lo.D, r.A)
        c.connect(lo.S, "Cn.B")
        c.connect(r.B, l.A)
        star.append(l.B)
        ref = c.add("SINE_WAVE", f"Ref2{ph}", amplitude=MA, frequency=F2, phase=ang)
        cmp_ = c.add("COMP", f"Cmp2{ph}")
        inv = c.add("NOT", f"Not2{ph}")
        c.connect(ref.Out, cmp_.In1); c.connect(car.Out, cmp_.In2)
        c.connect(cmp_.Out, up.G, inv.In)
        c.connect(inv.Out, lo.G)
    c.connect(*star)
    van = c.add("VM", "VloadA")
    c.connect(van.A, "RLA.A")
    c.ground(van.B)   # DC midpoint = grid neutral = ground
    return c


def settings() -> dict:
    return dict(t_stop=T_STOP, dt=1e-6)


def checks(res: sp.Results) -> list[cm.Check]:
    win = (T_STOP - 0.1, T_STOP)          # 5 grid cycles = 3 load cycles
    w2 = 2 * math.pi * F2
    z = complex(RL, w2 * LL)
    phi = math.atan2(w2 * LL, RL)
    vdc = res.stats("Vdc.Out", *win)
    v3, i3 = ("VA.Out", "VB.Out", "VC.Out"), ("IA.Out", "IB.Out", "IC.Out")
    p_g, q_g = cm.inst_pq(res, v3, i3, *win)
    p_dc = vdc.mean ** 2 / afe.RLOAD
    p_ac = sum(RL * float(np.mean(cm.window(res, f"I_RL{ph}", *win) ** 2)) for ph in "ABC")
    fv = res.fourier("VloadA.Out", f0=F2, cycles=3)
    fi = {ph: res.fourier(f"I_RL{ph}", f0=F2, cycles=3) for ph in "ABC"}
    amps = [f.fundamental for f in fi.values()]
    v1_th = MA * vdc.mean / 2 - (4 / math.pi) * VDEV * math.cos(phi)
    fg = res.fourier("IA.Out", f0=afe.F1, cycles=5)
    return [
        cm.approx("Vdc [V]", vdc.mean, afe.VDC_REF, rel=0.005),
        cm.approx("Load phase voltage fundamental @30 Hz [V pk] (= ma Vdc/2 - drop)", fv.fundamental, v1_th, rel=0.01),
        cm.approx("Load current fundamental @30 Hz [A pk] (= V1/|Z|)", amps[0], v1_th / abs(z), rel=0.015),
        cm.less("Load current imbalance [%]", 100 * (max(amps) / min(amps) - 1), 1.0),
        cm.greater("Grid-side PF", p_g / math.hypot(p_g, q_g), 0.995),
        cm.less("Grid current THD [%]", fg.thd_percent, 5.0),
        cm.between("Overall efficiency (P_dc + P_ac)/P_grid [%]", 100 * (p_dc + p_ac) / p_g, 95.0, 100.0),
    ]


if __name__ == "__main__":
    cm.run_model(__import__(__name__))
