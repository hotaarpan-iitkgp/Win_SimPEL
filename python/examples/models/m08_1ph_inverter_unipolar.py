"""Model 8 - Single-phase full-bridge inverter, unipolar SPWM, LC filter.

400 V DC, 50 Hz reference, 10 kHz triangular carrier, modulation index ma.
Leg A compares +sin against the carrier, leg B compares -sin, so the bridge
voltage switches between 0 and +/-Vdc at twice the carrier frequency.

Theory:
    Vab fundamental (peak) = ma * Vdc - dV                  (linear region, ma <= 1)
    dV = (4/pi)*2*Vdev + 2*Ron*I1    two conducting devices; the drop is a
                                      square wave in phase with the current
    Vo  = Vab * Z / (Z + j*w*Lf),  Z = R / (1 + j*w*R*Cf)   (filter + load at 50 Hz)
    Unipolar PWM: no harmonics around the carrier frequency fc; the first
    carrier band sits at 2*fc.
"""

from __future__ import annotations

import cmath
import math

import numpy as np

import simpel as sp

try:
    from . import _common as cm
except ImportError:
    import _common as cm

NAME = "m08_1ph_inverter_unipolar"
TITLE = "1-phase full bridge, unipolar SPWM, LC filter"
CASES = [dict(ma=0.8), dict(ma=0.5)]

VDC, F1, FC, LF, CF, R, RON = 400.0, 50.0, 10e3, 2e-3, 10e-6, 20.0, 5e-3
VDEV = 0.8  # IGBT_DIODE forward drop (default)


def build(ma: float = 0.8) -> sp.Circuit:
    c = sp.Circuit(NAME)
    vdc = c.add("V", "Vdc", value=VDC)
    qa1, qa2, qb1, qb2 = (c.add("IGBT_DIODE", n, Ron=RON) for n in ("QA1", "QA2", "QB1", "QB2"))
    lf = c.add("L", "Lf", L=LF)
    cf = c.add("C", "Cf", C=CF)
    r1 = c.add("R", "Rload", value=R)
    vab = c.add("VM", "Vab")
    c.connect(vdc.A, qa1.D, qb1.D)
    c.connect(qa1.S, qa2.D, lf.A, vab.A)
    c.connect(qb1.S, qb2.D, cf.B, r1.B, vab.B)
    c.connect(lf.B, cf.A, r1.A)
    c.ground(vdc.B, qa2.S, qb2.S)

    tri = c.add("TRI_GEN", "Carrier", frequency=FC, min=-1, max=1)
    ref_a = c.add("SINE_WAVE", "RefA", amplitude=ma, frequency=F1, phase=0)
    ref_b = c.add("SINE_WAVE", "RefB", amplitude=ma, frequency=F1, phase=180)
    cmp_a = c.add("COMP", "CmpA")
    cmp_b = c.add("COMP", "CmpB")
    not_a = c.add("NOT", "NotA")
    not_b = c.add("NOT", "NotB")
    c.connect(ref_a.Out, cmp_a.In1)
    c.connect(ref_b.Out, cmp_b.In1)
    c.connect(tri.Out, cmp_a.In2, cmp_b.In2)
    c.connect(cmp_a.Out, qa1.G, not_a.In)
    c.connect(not_a.Out, qa2.G)
    c.connect(cmp_b.Out, qb1.G, not_b.In)
    c.connect(not_b.Out, qb2.G)
    return c


def settings(ma: float = 0.8) -> dict:
    return dict(t_stop=3 / F1, dt=0.5e-6)


def expected(ma: float) -> tuple[float, complex]:
    """Vab fundamental (peak) and the complex Vo fundamental (relative to Vab)."""
    w = 2 * math.pi * F1
    z = R / (1 + 1j * w * R * CF)
    i1 = abs(ma * VDC / (z + 1j * w * LF))
    vab = ma * VDC - (4 / math.pi) * 2 * VDEV - 2 * RON * i1
    return vab, vab * z / (z + 1j * w * LF)


def _band(f: sp.FourierResult, centre: int, half: int = 12) -> float:
    m = (f.orders >= centre - half) & (f.orders <= centre + half)
    return float(np.sqrt(np.sum(f.magnitudes[m] ** 2)))


def checks(res: sp.Results, ma: float = 0.8) -> list[cm.Check]:
    fab = res.fourier("Vab.Out", f0=F1, max_harmonic=int(2 * FC / F1) + 20)
    fo = res.fourier("V_Cf", f0=F1)
    vab_th, vo_th = expected(ma)
    k = int(FC / F1)
    band_fc, band_2fc = _band(fab, k), _band(fab, 2 * k)
    ang = cm.wrap180(cm.phase_deg(fo) - cm.phase_deg(fab))
    return [
        cm.approx("Vab fundamental [V pk] (= ma*Vdc - dV)", fab.fundamental, vab_th, rel=0.005),
        cm.approx("Vo fundamental [V pk] (LC + load)", fo.fundamental, abs(vo_th), rel=0.01),
        cm.approx("Vo phase vs Vab [deg]", ang, math.degrees(cmath.phase(vo_th)), abs=1.0),
        cm.less("Vo THD [%]", fo.thd_percent, 1.5),
        cm.less("Vab band at fc / band at 2fc (unipolar)", band_fc / band_2fc, 0.05),
    ]


if __name__ == "__main__":
    cm.run_model(__import__(__name__))
