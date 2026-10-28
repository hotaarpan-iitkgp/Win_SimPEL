"""Model 7 - Dual active bridge (DAB), single phase-shift modulation.

Two full bridges (IGBTs with anti-parallel diodes) linked by a 1:1 transformer
and a 50 uH series inductance, 20 kHz, both DC sides held at 200 V by sources.
Each bridge makes a 50 % square wave; the secondary bridge lags the primary by
the phase shift phi. Power flows from the leading to the lagging bridge:

    P = Vin * Vout' * phi * (1 - |phi|/pi) / (2*pi*fsw*Lk)        (phi in radians)

Cases: phi = +30 deg and +60 deg (forward power) and -30 deg (reverse power).
"""

from __future__ import annotations

import math

import simpel as sp

try:
    from . import _common as cm
except ImportError:
    import _common as cm

NAME = "m07_dab_phase_shift"
TITLE = "Dual active bridge, phase-shift control"
CASES = [dict(phi=30), dict(phi=60), dict(phi=-30)]

V1, V2, FSW, LK, RS, NP, NS = 200.0, 200.0, 20e3, 50e-6, 0.05, 10, 10
RON, VDEV = 5e-3, 0.8  # device on-resistance and forward drop


def _bridge(c: sp.Circuit, prefix: str, dc_pos, gate_a, gate_b) -> tuple[sp.Pin, sp.Pin]:
    """Full bridge on a DC bus; returns the two leg midpoints. gate_a drives the
    upper switch of leg 1 and the lower switch of leg 2; gate_b the other pair."""
    q = [c.add("IGBT_DIODE", f"{prefix}{k}", Ron=RON) for k in range(1, 5)]
    c.connect(dc_pos, q[0].D, q[2].D)
    c.connect(q[0].S, q[1].D)  # leg 1 midpoint
    c.connect(q[2].S, q[3].D)  # leg 2 midpoint
    c.ground(q[1].S, q[3].S)
    c.connect(gate_a, q[0].G, q[3].G)
    c.connect(gate_b, q[1].G, q[2].G)
    return q[0].S, q[2].S


def build(phi: float = 30) -> sp.Circuit:
    c = sp.Circuit(NAME)
    v1 = c.add("V", "V1", value=V1)
    v2 = c.add("V", "V2", value=V2)
    c.ground(v1.B, v2.B)

    period = 1 / FSW
    p1 = c.add("PULSE_GEN", "Gp", amplitude=1, period=period, width=0.5, delay=0)
    delay2 = (phi % 360) / 360 * period
    p2 = c.add("PULSE_GEN", "Gs", amplitude=1, period=period, width=0.5, delay=delay2)
    n1 = c.add("NOT", "Gp_n")
    n2 = c.add("NOT", "Gs_n")
    c.connect(p1.Out, n1.In)
    c.connect(p2.Out, n2.In)

    a, b = _bridge(c, "Qp", v1.A, p1.Out, n1.Out)
    cc, d = _bridge(c, "Qs", v2.A, p2.Out, n2.Out)

    lk = c.add("L", "Lk", L=LK)
    rs = c.add("R", "Rs", value=RS)
    # Lm = 0: ideal transformer. The power equation neglects the magnetising
    # branch, and a lossless Lm would keep any start-up DC offset forever.
    x1 = c.add("XFMR", "T1", primary_turns=f"[{NP}]", secondary_turns=f"[{NS}]", polarity="non_inverted", Lm=0)
    c.connect(a, lk.A)
    c.connect(lk.B, rs.A)
    c.connect(rs.B, x1.P1A)
    c.connect(x1.P1B, b)
    c.connect(x1.S1A, cc)
    c.connect(x1.S1B, d)
    return c


def settings(phi: float = 30) -> dict:
    return dict(t_stop=4e-3, dt=50e-9)


def expected_power(phi: float) -> float:
    ph = math.radians(phi)
    return V1 * V2 * (NP / NS) * ph * (1 - abs(ph) / math.pi) / (2 * math.pi * FSW * LK)


def checks(res: sp.Results, phi: float = 30) -> list[cm.Check]:
    t1 = res.time[-1] - 1e-3  # last 20 switching periods
    # I_<source> is the current into terminal A: V1 delivers -V*I, V2 absorbs +V*I.
    p_in = -cm.mean_power(res, "V_V1", "I_V1", t1)
    p_out = cm.mean_power(res, "V_V2", "I_V2", t1)
    p_th = expected_power(phi)
    p_src, p_snk = (p_in, p_out) if phi > 0 else (-p_out, -p_in)
    ilk = res.stats("I_Lk", t1)
    return [
        cm.approx("Power delivered to V2 [W]", p_out, p_th, rel=0.05),
        cm.approx("Power drawn from V1 [W]", p_in, p_th, rel=0.05),
        cm.between("Efficiency of the sending side", p_snk / p_src, 0.95, 1.0),
        cm.less("I(Lk) DC offset [A]", abs(ilk.mean), 0.02 * ilk.abs_max),
    ]


if __name__ == "__main__":
    cm.run_model(__import__(__name__))
