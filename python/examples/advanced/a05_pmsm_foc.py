"""A05 - Surface PMSM drive, field-oriented control (speed loop + dq current loops), SVPWM.

The engine's INDUCTION_MOTOR block is only three stator resistors, so the
machine is built from primitives - a good workout for CTRL_V and CSCRIPT:
    per phase:  terminal -> Rs -> Ls -> CTRL_V(e_x) -> floating star point
    CSCRIPT "Motor": rotor dynamics from the measured phase currents
        iq  = Park_q(ia, ib, ic; theta_e)            (amplitude-invariant, cos-aligned)
        Te  = 1.5 p psi_f iq
        J dwm/dt = Te - TL - B wm,   theta_e = integral(p wm)
        e_x = -p wm psi_f sin(theta_e - k*120 deg)   (d/dt of psi_f cos(.))
Drive: 300 V DC, two-level IGBT bridge, SVPWM 10 kHz (modulation v/(Vdc/2)).
Control: speed PI -> iq* (+/-30 A, anti-windup), id* = 0,
    vd* = PI_d(id* - id) - we Ls iq,   vq* = PI_q(iq* - iq) + we Ls id + we psi_f
Profile: speed ramp 0 -> 1000 rpm in 0.1 s, load torque 10 Nm at 0.25 s.

Theory checks: speed tracking (integral action), id ~ 0, iq = (TL + B wm)/Kt,
back-EMF amplitude = p wm psi_f, speed dip after the load step and recovery,
stator current fundamental frequency = p wm / 2 pi, mechanical / DC efficiency.
"""

from __future__ import annotations

import math

import numpy as np

import simpel as sp

try:
    from . import _adv as cm
except ImportError:
    import _adv as cm

NAME = "a05_pmsm_foc"
TITLE = "PMSM field-oriented control, SVPWM"
CASES = [dict()]

VDC, FSW, RON = 300.0, 10e3, 5e-3
RS, LS, PSI, P, J, B = 0.2, 2e-3, 0.175, 4, 0.01, 1e-3
KT = 1.5 * P * PSI
W_REF = 1000 * 2 * math.pi / 60          # rad/s mechanical
T_RAMP, T_LOAD, TL, T_STOP = 0.1, 0.25, 10.0, 0.4
WC = 2 * math.pi * 1000                    # current-loop bandwidth
KPI, KII = WC * LS, WC * RS
WS = 2 * math.pi * 20                      # speed-loop bandwidth
KPW, KIW = WS * J / KT, WS * J / KT * WS / 5
IQ_MAX = 30.0

MOTOR_CODE = f"""
double J = {J};
double B = {B};
double PP = {P};
double PSI = {PSI};
double KT = {KT};
double wm = 0.0;
double th = 0.0;
void step() {{
    double ia = inputs[0];
    double ib = inputs[1];
    double ic = inputs[2];
    double al = (2.0 * ia - ib - ic) / 3.0;
    double be = (ib - ic) / sqrt(3.0);
    double iq = -al * sin(th) + be * cos(th);
    double te = KT * iq;
    wm = wm + (te - inputs[3] - B * wm) / J * dt;
    th = fmod(th + PP * wm * dt, 2.0 * pi);
    double we = PP * wm;
    outputs[0] = -we * PSI * sin(th);
    outputs[1] = -we * PSI * sin(th - 2.0943951023931953);
    outputs[2] = -we * PSI * sin(th + 2.0943951023931953);
    outputs[3] = th;
    outputs[4] = wm;
    outputs[5] = te;
    outputs[6] = we;
}}
"""


def build() -> sp.Circuit:
    c = sp.Circuit(NAME)
    vdc = c.add("V", "Vdc", value=VDC)
    c.ground(vdc.B)
    sv = c.add("SVPWM", "SVM", fc=FSW)
    motor = c.add("CSCRIPT", "Motor", code=MOTOR_CODE)
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
        c.connect(am.B, rs.A)
        c.connect(rs.B, ls.A)
        c.connect(ls.B, emf.A)
        star.append(emf.B)
        c.connect(getattr(motor, f"Out{k + 1}"), emf.Ctrl)
        c.connect(am.Out, getattr(motor, f"In{k + 1}"))
        inv = c.add("NOT", f"N{ph}")
        c.connect(getattr(sv, f"Out{ph}"), up.G, inv.In)
        c.connect(inv.Out, lo.G)
    c.connect(*star)
    vea = c.add("VM", "EmfA")
    c.connect(vea.A, "EA.A"); c.connect(vea.B, "EA.B")

    # measured rotor quantities (ideal encoder)
    theta, wm, te, we = motor.Out4, motor.Out5, motor.Out6, motor.Out7
    g_wm = c.add("GAIN", "Speed", K=1)
    g_te = c.add("GAIN", "Torque", K=1)
    c.connect(wm, g_wm.In); c.connect(te, g_te.In)

    idq = c.add("ABC_DQ", "Idq")
    for ph in "ABC":
        c.connect(f"I{ph}.Out", getattr(idq, ph))
    c.connect(theta, idq.Theta)

    # speed loop
    ramp = c.add("RAMP", "WRamp", slope=W_REF / T_RAMP, start_time=0, initial_output=0)
    wref = c.add("SATURATION", "WRef", min=0, max=W_REF)
    ew = c.add("SUM", "Ew", signs="+-")
    piw = c.add("CONT_PID", "PIw", Kp=KPW, Ki=KIW, Kd=0, limit_output=True,
                upper_limit=IQ_MAX, lower_limit=-IQ_MAX, anti_windup=True)
    c.connect(ramp.Out, wref.In)
    c.connect(wref.Out, ew.In1); c.connect(g_wm.Out, ew.In2)
    c.connect(ew.Out, piw.In)
    id0 = c.add("CONST", "IdRef", value=0)

    # current loops with cross-coupling decoupling and EMF feed-forward
    wl_iq = c.add("PRODUCT_RECT", "WeIq");   c.connect(we, wl_iq.In1); c.connect(idq.q, wl_iq.In2)
    wl_id = c.add("PRODUCT_RECT", "WeId");   c.connect(we, wl_id.In1); c.connect(idq.d, wl_id.In2)
    g_lq = c.add("GAIN", "LsIq", K=LS);      c.connect(wl_iq.Out, g_lq.In)
    g_ld = c.add("GAIN", "LsId", K=LS);      c.connect(wl_id.Out, g_ld.In)
    g_emf = c.add("GAIN", "EmfFF", K=PSI);   c.connect(we, g_emf.In)

    ed = c.add("SUM", "Ed", signs="+-"); c.connect(id0.Out, ed.In1); c.connect(idq.d, ed.In2)
    eq = c.add("SUM", "Eq", signs="+-"); c.connect(piw.Out, eq.In1); c.connect(idq.q, eq.In2)
    pid_ = c.add("CONT_PID", "PId", Kp=KPI, Ki=KII, Kd=0); c.connect(ed.Out, pid_.In)
    piq = c.add("CONT_PID", "PIq", Kp=KPI, Ki=KII, Kd=0); c.connect(eq.Out, piq.In)
    vd = c.add("SUM", "Vd", signs="+-"); c.connect(pid_.Out, vd.In1); c.connect(g_lq.Out, vd.In2)
    vq1 = c.add("SUM", "Vq1", signs="++"); c.connect(piq.Out, vq1.In1); c.connect(g_ld.Out, vq1.In2)
    vq = c.add("SUM", "Vq", signs="++"); c.connect(vq1.Out, vq.In1); c.connect(g_emf.Out, vq.In2)

    dq_abc = c.add("DQ_ABC", "Vabc")
    c.connect(vd.Out, dq_abc.d); c.connect(vq.Out, dq_abc.q); c.connect(theta, dq_abc.Theta)
    for ph in "ABC":
        g = c.add("GAIN", f"M{ph}", K=1 / (VDC / 2))
        c.connect(getattr(dq_abc, ph), g.In)
        c.connect(g.Out, getattr(sv, ph))
    return c


def settings() -> dict:
    return dict(t_stop=T_STOP, dt=1e-6)


def checks(res: sp.Results) -> list[cm.Check]:
    pre = (T_LOAD - 0.05, T_LOAD)
    post = (T_STOP - 0.05, T_STOP)
    w_pre = cm.mean(res, "Speed.Out", *pre)
    w_post = cm.mean(res, "Speed.Out", *post)
    iq_post = cm.mean(res, "Idq.q", *post)
    id_post = cm.mean(res, "Idq.d", *post)
    te_post = cm.mean(res, "Torque.Out", *post)
    dip = W_REF - res.stats("Speed.Out", T_LOAD, T_LOAD + 0.1).min
    we = P * w_post
    f_el = we / (2 * math.pi)
    fe = res.fourier("EmfA.Out", f0=f_el, cycles=4)
    fi = res.fourier("IA.Out", f0=f_el, cycles=4)
    # recovery: speed back within 0.5 % of reference
    t = res.time
    s = np.asarray(res["Speed.Out"])
    m = t >= T_LOAD
    bad = np.nonzero(np.abs(s[m] - W_REF) > 0.005 * W_REF)[0]
    t_rec = 0.0 if not len(bad) else float(t[m][bad[-1]] - T_LOAD)
    p_dc = VDC * -cm.mean(res, "I_Vdc", *post)
    p_mech = te_post * w_post  # air-gap power
    t_rec_th = math.log(dip / (0.005 * W_REF)) / (WS / 5)
    return [
        cm.approx("Speed before load step [rad/s]", w_pre, W_REF, rel=0.005),
        cm.approx("Speed after load step [rad/s]", w_post, W_REF, rel=0.005),
        cm.less("|id| (id* = 0) [A]", abs(id_post), 0.5),
        cm.approx("iq = (TL + B w)/Kt [A]", iq_post, (TL + B * w_post) / KT, rel=0.02),
        cm.approx("Te = TL + B w [Nm]", te_post, TL + B * w_post, rel=0.02),
        cm.approx("Back-EMF fundamental [V pk] (= p w psi_f)", fe.fundamental, we * PSI, rel=0.01),
        cm.approx("Phase current fundamental [A pk] (= iq)", fi.fundamental, iq_post, rel=0.03),
        cm.less("Speed dip after 10 Nm step [% of ref]", 100 * dip / W_REF, 10.0),
        # The PI zero at wz = ws/5 leaves a slow pole near wz: the error decays as
        # e^(-wz t), so reaching the 0.5 % band takes ~ ln(dip/band)/wz.
        cm.less(f"Recovery to +/-0.5 % [ms] (~ln(dip/band)/wz = {1e3 * t_rec_th:.0f} ms, +30 %)",
                1e3 * t_rec, 1.3e3 * t_rec_th),
        cm.between("Air-gap power / DC power [%]", 100 * p_mech / p_dc, 90.0, 100.0),
    ]


if __name__ == "__main__":
    cm.run_model(__import__(__name__))
