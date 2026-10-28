"""Model 2 - Boost converter, closed-loop voltage-mode PID with a load step.

24 V -> 48 V at 100 kHz. The reference ramps from 24 V to 48 V over 5 ms (soft
start). At 30 ms a second 24 ohm resistor is switched in (2 A -> 4 A load).

Control: Vref - Vout -> CONT_PID -> SATURATION(0..0.85) -> PWM comparator.
The PID places its two zeros at the converter's LC corner w0 = (1-D)/sqrt(LC),
the classic "type III" voltage-mode boost compensation:
    Ki = wc / Gd0,  Kp = 2*Ki/w0,  Kd = Ki/w0^2,  Gd0 = Vin/(1-D)^2
with crossover wc = 2*pi*1.5 kHz, well below the right-half-plane zero.

Theory checks: Vout regulated at Vref before and after the step (integral
action), duty cycle D = 1 - Vin/(Vout + Vd), bounded undershoot and recovery.
"""

from __future__ import annotations

import math

import numpy as np

import simpel as sp

try:
    from . import _common as cm
except ImportError:
    import _common as cm

NAME = "m02_boost_pi"
TITLE = "Boost, PID voltage loop, load step"
CASES = [dict()]

VIN, VREF, FSW, L, C, R, RON, VD = 24.0, 48.0, 100e3, 50e-6, 220e-6, 24.0, 10e-3, 0.7
T_SOFT, T_STEP, T_STOP = 5e-3, 30e-3, 60e-3

D0 = 1 - VIN / VREF
W0 = (1 - D0) / math.sqrt(L * C)
GD0 = VIN / (1 - D0) ** 2
WC = 2 * math.pi * 1.5e3
KI = WC / GD0
KP = 2 * KI / W0
KD = KI / W0 ** 2
TF = 1 / (2 * math.pi * 15e3)


def build() -> sp.Circuit:
    c = sp.Circuit(NAME)
    vin = c.add("V", "Vin", value=VIN)
    l1 = c.add("L", "L1", L=L)
    q1 = c.add("MOSFET", "Q1", Ron=RON)
    d1 = c.add("D", "D1", Vd=VD)
    co = c.add("C", "Co", C=C, vC0=VIN - VD)  # pre-charged through the diode
    r1 = c.add("R", "Rload", value=R)
    r2 = c.add("R", "Rstep", value=R)
    s1 = c.add("S", "Sstep", Ron=1e-3)
    vm = c.add("VM", "Vsense")
    c.connect(vin.A, l1.A)
    c.connect(l1.B, q1.D, d1.A)
    c.connect(d1.B, co.A, r1.A, r2.A, vm.A)
    c.connect(r2.B, s1.A)
    c.ground(vin.B, q1.S, co.B, r1.B, s1.B, vm.B)

    # Soft-start reference: ramp from Vin, clamped at Vref.
    ramp = c.add("RAMP", "Ramp", slope=(VREF - VIN) / T_SOFT, start_time=0, initial_output=VIN)
    ref = c.add("SATURATION", "Vref", min=0, max=VREF)
    err = c.add("SUBTRACT", "Err", signs="+-")
    pid = c.add("CONT_PID", "PID", Kp=KP, Ki=KI, Kd=KD, Tf=TF)
    lim = c.add("SATURATION", "DutyLim", min=0, max=0.85)
    tri = c.add("TRI_GEN", "Carrier", frequency=FSW, min=0, max=1)
    pwm = c.add("COMP", "PWM")
    c.connect(ramp.Out, ref.In)
    c.connect(ref.Out, err.In1)
    c.connect(vm.Out, err.In2)
    c.connect(err.Out, pid.In)
    c.connect(pid.Out, lim.In)
    c.connect(lim.Out, pwm.In1)
    c.connect(tri.Out, pwm.In2)
    c.connect(pwm.Out, q1.G)

    step = c.add("STEP", "LoadStep", step_time=T_STEP, initial_value=0, final_value=1)
    c.connect(step.Out, s1.Ctrl)
    return c


def settings() -> dict:
    return dict(t_stop=T_STOP, dt=0.1e-6)


def _settle_time(res: sp.Results, t_event: float, target: float, band: float) -> float:
    """Time after ``t_event`` from which the switching-period average of Vout stays within ``band``."""
    t = res.time
    v = res["V_Co"]
    n = max(1, int(round((1 / FSW) / (t[1] - t[0]))))
    m = t >= t_event
    tv, vv = t[m], v[m]
    k = len(vv) // n
    avg = vv[: k * n].reshape(k, n).mean(axis=1)
    tav = tv[: k * n].reshape(k, n)[:, -1]
    out = np.nonzero(np.abs(avg - target) > band)[0]
    return 0.0 if not len(out) else float(tav[out[-1]] - t_event)


def checks(res: sp.Results) -> list[cm.Check]:
    pre = res.stats("V_Co", T_STEP - 5e-3, T_STEP)
    post = res.stats("V_Co", T_STOP - 10e-3)
    dip = res.stats("V_Co", T_STEP, T_STEP + 5e-3)
    duty_pre = res.stats("DutyLim.Out", T_STEP - 5e-3, T_STEP).mean
    duty_post = res.stats("DutyLim.Out", T_STOP - 10e-3).mean
    il_pre = res.stats("I_L1", T_STEP - 5e-3, T_STEP).mean
    il_post = res.stats("I_L1", T_STOP - 10e-3).mean
    # Ideal duty with the diode drop; switch/diode resistance raise it slightly.
    d_th = 1 - VIN / (VREF + VD)
    p_pre, p_post = VREF ** 2 / R, VREF ** 2 / (R / 2)
    return [
        cm.approx("Vout before step [V]", pre.mean, VREF, rel=0.005),
        cm.approx("Vout after step [V]", post.mean, VREF, rel=0.005),
        cm.approx("Duty before step", duty_pre, d_th, abs=0.01),
        cm.approx("Duty after step", duty_post, d_th, abs=0.015),
        cm.approx("IL before step [A] (= Pout/Vin)", il_pre, p_pre / VIN, rel=0.04),
        cm.approx("IL after step [A] (= Pout/Vin)", il_post, p_post / VIN, rel=0.04),
        cm.greater("Vout min after step [V] (undershoot < 5 %)", dip.min, 0.95 * VREF),
        cm.less("Recovery to +/-1 % [ms]", 1e3 * _settle_time(res, T_STEP, VREF, 0.01 * VREF), 3.0),
        cm.less("Startup overshoot [V]", res.stats("V_Co", 0, T_STEP).max - VREF, 0.05 * VREF),
    ]


if __name__ == "__main__":
    cm.run_model(__import__(__name__))
