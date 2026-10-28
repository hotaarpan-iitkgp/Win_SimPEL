"""Model 11 - Dual active bridge (DAB), closed-loop output voltage control.

The DAB of model 7 (two full bridges, 1:1 transformer, 50 uH series inductance,
20 kHz), except that the secondary DC side is now an output capacitor with a
resistive load. A PI controller sets the phase shift phi so the output follows
its reference, with a soft start and a load step.

Gate drive: the primary gate is the sign of sin(w t) and the secondary gate is
the sign of sin(w t - phi). The secondary therefore lags by phi and power flows
from the primary to the output for phi > 0. Both gates come from a ramp (w t)
and a sine block, so phi can change while the simulation runs.

Theory (ideal switches, n = 1, Ron and Rs neglected), phi in radians, Lk meaning
the effective inductance L_EFF defined below:
    P    = V1 * V2 * phi * (1 - |phi|/pi) / (2 pi f Lk)     (for V1 = V2)
    I_pk = [(V1 + V2) phi + (V1 - V2)(pi - phi)] / (2 w Lk)  (inductor peak)
    avg|I_Lk| = I_pk (pi - phi/2) / pi                       (average rectified)
At steady state P = V_out^2 / R_load, which fixes phi.

The voltage loop reads V_out through a 1 kHz low-pass. Without it the PI
follows the 2 f_sw ripple of the output capacitor, which modulates phi and
puts a DC offset on the inductor current. A DC-blocking capacitor in series
with Lk removes the remaining DC path, as in a real DAB. Its reactance at
f_sw is negligible, so the formulas above still apply.

Small-signal plant from phi to V_out: K / (1 + s tau), with
K = P'(phi) / (2 V / R) and tau = C R / 2. The PI gains are tuned for it.

Load step at 15 ms: a second 40 ohm resistor is switched in parallel (1 kW -> 2 kW).
"""

from __future__ import annotations

import math

import numpy as np

import simpel as sp

try:
    from . import _common as cm
except ImportError:
    import _common as cm

NAME = "m11_dab_closed_loop"
TITLE = "Dual active bridge, closed-loop output voltage, load step"
CASES = [dict()]

V1, VREF, FSW = 200.0, 200.0, 20e3
LK, RS, NP, NS = 50e-6, 0.05, 10, 10
RON = 5e-3
C_OUT, R_LOAD, R_STEP = 100e-6, 40.0, 40.0
C_BLK = 100e-6  # DC-blocking capacitor in series with Lk; reactance ~0.08 ohm at fsw
T_SOFT, T_STEP, T_STOP = 5e-3, 15e-3, 30e-3
KP, KI = 0.01, 3.0  # PI on (Vref - Vout); output is phi in rad
PHI_MAX = 1.0        # rad
V_SENSE_FC = 1e3     # Hz, voltage sense low-pass
# Series inductance seen at f_sw: Lk minus the blocking capacitor's reactance (~1.3 % here)
L_EFF = LK - 1 / ((2 * math.pi * FSW) ** 2 * C_BLK)


def _bridge(c: sp.Circuit, prefix: str, dc_pos, gate_a, gate_b):
    """Full bridge on a DC bus; returns the two leg midpoints. gate_a drives the
    upper switch of leg 1 and the lower switch of leg 2; gate_b the other pair.

    Ideal bidirectional switches (S) are used rather than IGBT_DIODE, because
    IGBT_DIODE has a built-in ~0.8 V forward drop. Two conducting devices per
    path would add about 3 V of conduction offset, which the SPS formulas do not
    include."""
    s1, s2, s3, s4 = (c.add("S", f"{prefix}{k}", Ron=RON) for k in range(1, 5))
    c.connect(dc_pos, s1.A, s3.A)
    c.connect(s1.B, s2.A)  # leg 1 midpoint
    c.connect(s3.B, s4.A)  # leg 2 midpoint
    c.ground(s2.B, s4.B)
    c.connect(gate_a, s1.Ctrl, s4.Ctrl)
    c.connect(gate_b, s2.Ctrl, s3.Ctrl)
    return s1.B, s3.B


def build() -> sp.Circuit:
    c = sp.Circuit(NAME)

    # Primary DC supply, output capacitor, load and a switched parallel load
    v1 = c.add("V", "V1", value=V1)
    co = c.add("C", "Co", C=C_OUT)
    rl = c.add("R", "Rload", value=R_LOAD)
    rstep = c.add("R", "Rstep", value=R_STEP)
    sw = c.add("S", "Sstep", Ron=1e-3)
    vm = c.add("VM", "Vsense")
    c.connect(co.A, rl.A, rstep.A, vm.A)
    c.connect(rstep.B, sw.A)
    c.ground(v1.B, co.B, rl.B, sw.B, vm.B)

    # Gate signals: primary = sign(sin(w t)), secondary = sign(sin(w t - phi))
    wt = c.add("RAMP", "Wt", slope=2 * math.pi * FSW, start_time=0, initial_output=0)
    sin_p = c.add("TRIG_FCN", "SinP", function="sin")
    ang_s = c.add("SUBTRACT", "AngS", signs="+-")
    sin_s = c.add("TRIG_FCN", "SinS", function="sin")
    zero = c.add("CONST", "Zero", value=0)
    gp = c.add("COMP", "GateP")
    gs = c.add("COMP", "GateS")
    gp_n = c.add("NOT", "GateP_n")
    gs_n = c.add("NOT", "GateS_n")
    c.connect(wt.Out, sin_p.In, ang_s.In1)
    c.connect(sin_p.Out, gp.In1)
    c.connect(ang_s.Out, sin_s.In)
    c.connect(sin_s.Out, gs.In1)
    c.connect(zero.Out, gp.In2, gs.In2)
    c.connect(gp.Out, gp_n.In)
    c.connect(gs.Out, gs_n.In)

    # Power stage: primary bridge -> Lk + Rs -> 1:1 transformer -> secondary bridge
    a, b = _bridge(c, "Qp", v1.A, gp.Out, gp_n.Out)
    lk = c.add("L", "Lk", L=LK)
    cblk = c.add("C", "Cblk", C=C_BLK)
    rser = c.add("R", "Rs", value=RS)
    xf = c.add("XFMR", "T1", primary_turns=f"[{NP}]", secondary_turns=f"[{NS}]",
               polarity="non_inverted", Lm=0)
    cc, d = _bridge(c, "Qs", co.A, gs.Out, gs_n.Out)
    c.connect(a, lk.A)
    c.connect(lk.B, cblk.A)
    c.connect(cblk.B, rser.A)
    c.connect(rser.B, xf.P1A)
    c.connect(xf.P1B, b)
    c.connect(xf.S1A, cc)
    c.connect(xf.S1B, d)

    # Voltage loop: soft-start reference -> error -> PI (phi, limited, anti-windup)
    ramp = c.add("RAMP", "VrefRamp", slope=VREF / T_SOFT, start_time=0, initial_output=0)
    ref = c.add("SATURATION", "Vref", min=0, max=VREF)
    err = c.add("SUBTRACT", "Err", signs="+-")
    pi = c.add("CONT_PID", "PI", Kp=KP, Ki=KI, limit_output="true",
               upper_limit=PHI_MAX, lower_limit=-PHI_MAX, anti_windup="true")
    c.connect(ramp.Out, ref.In)
    c.connect(ref.Out, err.In1)
    sense = c.add("FILTER_1ST", "VsenseLP", fc=V_SENSE_FC)  # default type: Lowpass
    c.connect(vm.Out, sense.In)
    c.connect(sense.Out, err.In2)
    c.connect(err.Out, pi.In)
    c.connect(pi.Out, ang_s.In2)

    # Load step: the parallel load is switched in at T_STEP
    step = c.add("STEP", "LoadStep", step_time=T_STEP, initial_value=0, final_value=1)
    c.connect(step.Out, sw.Ctrl)
    return c


def settings() -> dict:
    return dict(t_stop=T_STOP, dt=50e-9)  # 1000 steps per switching period


def phi_for_power(p: float) -> float:
    """Phase shift (rad, 0 to pi/2) that delivers ``p`` watts, from the ideal DAB law."""
    k = p * 2 * math.pi * FSW * L_EFF / (V1 * VREF)
    return math.pi * (1 - math.sqrt(1 - 4 * k / math.pi)) / 2


def expected_power(phi: float) -> float:
    return V1 * VREF * phi * (1 - abs(phi) / math.pi) / (2 * math.pi * FSW * L_EFF)


def _load_power(res: sp.Results, t1: float, t2: float) -> float:
    """Power into both load resistors (the step resistor may be switched in)."""
    v = cm.window(res, "V_Co", t1, t2)
    i = cm.window(res, "I_Rload", t1, t2) + cm.window(res, "I_Rstep", t1, t2)
    return float(np.mean(v * i))


def checks(res: sp.Results) -> list[cm.Check]:
    r_par = R_LOAD * R_STEP / (R_LOAD + R_STEP)
    pre0, pre1 = T_STEP - 5e-3, T_STEP        # last 5 ms before the load step
    post0, post1 = T_STOP - 5e-3, T_STOP      # last 5 ms of the run
    v_pre = res.stats("V_Co", pre0, pre1)
    v_post = res.stats("V_Co", post0, post1)
    phi_pre = res.stats("PI.Out", pre0, pre1).mean
    phi_post = res.stats("PI.Out", post0, post1).mean
    # Measured power at the source and at the load
    p_in = -cm.mean_power(res, "V_V1", "I_V1", pre0, pre1)
    p_load = _load_power(res, pre0, pre1)
    p_load_post = _load_power(res, post0, post1)
    # Inductor current over the 2.5 ms before the step, once the start-up has settled
    i_ss = cm.window(res, "I_Lk", T_STEP - 2.5e-3, T_STEP)
    dip = res.stats("V_Co", T_STEP, T_STEP + 5e-3)
    # SPS expressions with the measured V_out. The average rectified current over
    # a half period is I_pk * (pi - phi/2) / pi.
    v2 = v_pre.mean
    w = 2 * math.pi * FSW
    i_pk = ((V1 + v2) * phi_pre + (V1 - v2) * (math.pi - phi_pre)) / (2 * w * L_EFF)
    i_avg = i_pk * (math.pi - phi_pre / 2) / math.pi
    return [
        cm.approx("Vout before step [V]", v_pre.mean, VREF, rel=0.01),
        cm.approx("Vout after step [V]", v_post.mean, VREF, rel=0.01),
        cm.approx("phi before step [rad] (= ideal for 1 kW)", phi_pre, phi_for_power(VREF**2 / R_LOAD), rel=0.03),
        cm.approx("phi after step [rad] (= ideal for 2 kW)", phi_post, phi_for_power(VREF**2 / r_par), rel=0.03),
        cm.approx("P_load vs DAB law at measured phi [W]", p_load, expected_power(phi_pre), rel=0.03),
        cm.approx("P_load vs DAB law after step [W]", p_load_post, expected_power(phi_post), rel=0.03),
        cm.between("Power balance P_in / P_load", p_in / p_load, 0.98, 1.02),
        cm.approx("Mean rectified I_Lk [A] (SPS)", float(np.mean(np.abs(i_ss))), i_avg, rel=0.10),
        cm.less("I(Lk) DC offset [A]", abs(float(np.mean(i_ss))), 0.05 * float(np.mean(np.abs(i_ss)))),
        cm.greater("Vout min after step [V] (undershoot < 10 %)", dip.min, 0.9 * VREF),
    ]


if __name__ == "__main__":
    cm.run_model(__import__(__name__))
