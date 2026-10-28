"""A06 - PMSM direct torque control (DTC), hysteresis + Takahashi switching table.

Same machine and inverter as A05 (behavioural PMSM: R-L-CTRL_V per phase, rotor
dynamics in CSCRIPT). No current loops and no PWM: a single CSCRIPT, sampled at
40 kHz (timestep = 25 us), picks one of the six active voltage vectors directly.

Controller ("DTC" CSCRIPT):
    stator flux (current-model observer, encoder angle):
        psi_a = Ls i_a + psi_f cos(th),  psi_b = Ls i_b + psi_f sin(th)
    torque estimate  Te = 1.5 p (psi_a i_b - psi_b i_a)
    flux comparator   2-level, band +/-HF around psi*
    torque comparator 2-level, band +/-HT around T* (PMSM: no zero vectors)
    sector k = 1..6 of the flux vector (sector 1 centred on V1)
    vector = V(k+1) flux up / torque up, V(k+2) flux down / torque up,
             V(k-1) flux up / torque down, V(k-2) flux down / torque down
Speed PI (CONT_PID, limited +/-25 Nm) produces T*.
Profile: ramp to 1000 rpm in 0.1 s, 10 Nm load at 0.25 s.

Theory checks: speed tracking, estimated vs true torque, flux magnitude held at
psi*, flux / torque ripple bounded by the hysteresis band plus one sample of
slope, Te = TL + B w in steady state, average device switching frequency below
the fs/2 limit set by the sampling rate.
"""

from __future__ import annotations

import math

import numpy as np

import simpel as sp

try:
    from . import _adv as cm
    from .a05_pmsm_foc import B, J, KT, LS, MOTOR_CODE, P, PSI, RON, RS, T_RAMP, TL, VDC, W_REF
except ImportError:
    import _adv as cm
    from a05_pmsm_foc import B, J, KT, LS, MOTOR_CODE, P, PSI, RON, RS, T_RAMP, TL, VDC, W_REF

NAME = "a06_pmsm_dtc"
TITLE = "PMSM direct torque control, switching table"
CASES = [dict()]

# Longer than A05: the slow speed-loop pole (PI zero at ws/5 = 3 Hz, tau ~ 53 ms)
# needs ~4 tau after the ramp and after the load step to settle.
T_LOAD, T_STOP = 0.35, 0.65

TS = 25e-6                  # controller sample time
PSI_REF = 0.18              # stator flux reference [Wb]
HF, HT = 0.002, 0.25        # hysteresis half-bands [Wb], [Nm]
WS = 2 * math.pi * 15
KPW, KIW = WS * J, WS * J * WS / 5      # T* = PI(w* - w): plant 1/(J s)
T_MAX = 25.0

DTC_CODE = f"""
double LS = {LS};
double PSIF = {PSI};
double PP = {P};
double PSIREF = {PSI_REF};
double HF = {HF};
double HT = {HT};
double cf = 1.0;
double ct = 1.0;
void step() {{
    double ia = inputs[0];
    double ib = inputs[1];
    double ic = inputs[2];
    double th = inputs[3];
    double tref = inputs[4];
    double ial = (2.0 * ia - ib - ic) / 3.0;
    double ibe = (ib - ic) / sqrt(3.0);
    double pal = LS * ial + PSIF * cos(th);
    double pbe = LS * ibe + PSIF * sin(th);
    double pm = sqrt(pal * pal + pbe * pbe);
    double te = 1.5 * PP * (pal * ibe - pbe * ial);
    if (pm < PSIREF - HF) {{ cf = 1.0; }}
    if (pm > PSIREF + HF) {{ cf = -1.0; }}
    if (te < tref - HT) {{ ct = 1.0; }}
    if (te > tref + HT) {{ ct = -1.0; }}
    double ang = atan2(pbe, pal);
    double k = floor(fmod(ang + pi / 6.0 + 2.0 * pi, 2.0 * pi) / (pi / 3.0));
    double off = 1.0;
    if (cf > 0 && ct > 0) {{ off = 1.0; }}
    if (cf < 0 && ct > 0) {{ off = 2.0; }}
    if (cf > 0 && ct < 0) {{ off = -1.0; }}
    if (cf < 0 && ct < 0) {{ off = -2.0; }}
    double n = fmod(k + off + 6.0, 6.0);
    outputs[0] = (n == 0 || n == 1 || n == 5);
    outputs[1] = (n == 1 || n == 2 || n == 3);
    outputs[2] = (n == 3 || n == 4 || n == 5);
    outputs[3] = te;
    outputs[4] = pm;
    outputs[5] = k + 1.0;
}}
"""


def build() -> sp.Circuit:
    c = sp.Circuit(NAME)
    vdc = c.add("V", "Vdc", value=VDC)
    c.ground(vdc.B)
    motor = c.add("CSCRIPT", "Motor", code=MOTOR_CODE)
    dtc = c.add("CSCRIPT", "DTC", code=DTC_CODE, timestep=TS)
    tl = c.add("STEP", "TLoad", step_time=T_LOAD, initial_value=0, final_value=TL)
    c.connect(tl.Out, motor.In4)

    star = []
    for k, ph in enumerate("ABC"):
        up = c.add("IGBT_DIODE", f"Q{ph}u", Ron=RON)
        lo = c.add("IGBT_DIODE", f"Q{ph}l", Ron=RON)
        am = c.add("AM", f"I{ph}")
        rs = c.add("R", f"Rs{ph}", value=RS)
        ls = c.add("L", f"Ls{ph}", L=LS)
        emf = c.add("CTRL_V", f"E{ph}")
        c.connect(vdc.A, up.D)
        c.connect(up.S, lo.D, am.A)
        c.ground(lo.S)
        c.connect(am.B, rs.A); c.connect(rs.B, ls.A); c.connect(ls.B, emf.A)
        star.append(emf.B)
        c.connect(getattr(motor, f"Out{k + 1}"), emf.Ctrl)
        c.connect(am.Out, getattr(motor, f"In{k + 1}"), getattr(dtc, f"In{k + 1}"))
        inv = c.add("NOT", f"N{ph}")
        c.connect(getattr(dtc, f"Out{k + 1}"), up.G, inv.In)
        c.connect(inv.Out, lo.G)
    c.connect(*star)
    c.connect(motor.Out4, dtc.In4)

    g_wm = c.add("GAIN", "Speed", K=1); c.connect(motor.Out5, g_wm.In)
    g_te = c.add("GAIN", "Torque", K=1); c.connect(motor.Out6, g_te.In)
    g_tes = c.add("GAIN", "TorqueEst", K=1); c.connect(dtc.Out4, g_tes.In)
    g_psi = c.add("GAIN", "Flux", K=1); c.connect(dtc.Out5, g_psi.In)

    ramp = c.add("RAMP", "WRamp", slope=W_REF / T_RAMP, start_time=0, initial_output=0)
    wref = c.add("SATURATION", "WRef", min=0, max=W_REF)
    ew = c.add("SUM", "Ew", signs="+-")
    piw = c.add("CONT_PID", "PIw", Kp=KPW, Ki=KIW, Kd=0, limit_output=True,
                upper_limit=T_MAX, lower_limit=-T_MAX, anti_windup=True)
    c.connect(ramp.Out, wref.In)
    c.connect(wref.Out, ew.In1); c.connect(g_wm.Out, ew.In2)
    c.connect(ew.Out, piw.In)
    c.connect(piw.Out, dtc.In5)
    return c


def settings() -> dict:
    return dict(t_stop=T_STOP, dt=1e-6)


def checks(res: sp.Results) -> list[cm.Check]:
    pre = (T_LOAD - 0.05, T_LOAD)
    post = (T_STOP - 0.05, T_STOP)
    w_pre = cm.mean(res, "Speed.Out", *pre)
    w_post = cm.mean(res, "Speed.Out", *post)
    te = cm.window(res, "Torque.Out", *post)
    te_est = cm.window(res, "TorqueEst.Out", *post)
    psi = res.stats("Flux.Out", *post)
    # Sampled hysteresis overshoots its band by at most one sample of slope:
    #   |dpsi/dt| <= |v|max = (2/3) Vdc
    #   |dTe/dt| <= 1.5 p (psi_f / Ls) (|v|max + we psi_f)
    v_max = (2 / 3) * VDC
    psi_lim = 2 * HF + 2 * v_max * TS
    dte = 1.5 * P * (PSI / LS) * (v_max + P * W_REF * PSI) * TS
    te_lim = 2 * HT + 2 * dte
    gates = cm.window(res, "DTC.Out1", *post)
    f_sw = float(np.sum(np.diff(gates) > 0.5)) / (post[1] - post[0])
    return [
        cm.approx("Speed before load step [rad/s]", w_pre, W_REF, rel=0.005),
        cm.approx("Speed after load step [rad/s]", w_post, W_REF, rel=0.005),
        cm.approx("Mean Te = TL + B w [Nm]", float(te.mean()), TL + B * w_post, rel=0.02),
        cm.less("Torque estimate error |mean(Te_est - Te)| [Nm]", abs(float((te_est - te).mean())), 0.05 * TL),
        cm.approx("Flux magnitude [Wb] (= psi*)", psi.mean, PSI_REF, rel=0.01),
        cm.less(f"Flux ripple pk-pk [Wb] (< 2HF + one sample of slope = {psi_lim:.4f})", psi.pk2pk, psi_lim),
        cm.less(f"Torque ripple pk-pk [Nm] (< 2HT + one sample of slope = {te_lim:.2f})",
                float(np.ptp(te_est)), te_lim),
        cm.less(f"Leg A switching frequency [kHz] (< fs/2 = {0.5e-3 / TS:.0f})", 1e-3 * f_sw, 0.5e-3 / TS),
    ]


if __name__ == "__main__":
    cm.run_model(__import__(__name__))
