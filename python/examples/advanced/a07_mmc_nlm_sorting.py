"""A07 - Single-phase-leg modular multilevel converter (MMC), N = 4 half-bridge
submodules per arm, nearest-level modulation (NLM) + sorting capacitor balancing.

DC link +/-1000 V (midpoint grounded). Each arm: 4 SMs in series + 5 mH arm
inductor. SM: two IGBTs with anti-parallel diodes and a 3 mF capacitor,
pre-charged to Vdc/N = 500 V. AC terminal -> R = 20 ohm + L = 20 mH -> ground.

Controller (one CSCRIPT, sampled at 10 kHz; code generated below because the
script language has no loops):
    n_up = round(N/2 (1 - m sin wt)),  n_lo = N - n_up         (NLM)
    rank_i = number of SMs in the same arm with a lower voltage (ties by index)
    arm current charging (> 0): insert the n lowest-voltage SMs
    arm current discharging:    insert the n highest-voltage SMs
Each output drives the SM's upper switch; NOT drives the bypass switch.

Theory checks:
    upper-arm voltage has N+1 = 5 levels; v_up + v_lo = Vdc on average
    internal EMF e = (v_lo - v_up)/2 has fundamental m Vdc/2
    load voltage phasor = e * Zload / (Zload + j w Larm/2)
    all 8 capacitors balanced around Vdc/N (spread of means < 1 %)
    DC power = load power + conduction loss
"""

from __future__ import annotations

import math

import numpy as np

import simpel as sp

try:
    from . import _adv as cm
except ImportError:
    import _adv as cm

NAME = "a07_mmc_nlm_sorting"
TITLE = "MMC leg, 4 SM/arm, NLM + sorting balancing"
# sort=False is a negative control: SMs are inserted in fixed index order, so the
# capacitors must drift apart. It shows the balancing check is meaningful.
CASES = [dict(m=0.9, sort=True), dict(m=0.9, sort=False)]

N, VDC, F1, CSM, LARM, R, L, RON = 4, 2000.0, 50.0, 3e-3, 5e-3, 20.0, 20e-3, 2e-3
TS = 1e-4
VDEV = 0.8


def controller_code(m: float, sort: bool = True) -> str:
    """CSCRIPT for NLM + sorting. inputs: 0..3 upper Vc, 4..7 lower Vc, 8 i_up, 9 i_lo."""
    if not sort:
        lines = [f"double M = {m};", f"double NN = {N}.0;", f"double W = {2 * math.pi * F1};", "void step() {",
                 "    double ref = M * sin(W * time);",
                 "    double nup = round(NN / 2.0 * (1.0 - ref));",
                 "    double nlo = NN - nup;"]
        for base, n in ((0, "nup"), (N, "nlo")):
            for i in range(N):
                lines.append(f"    outputs[{base + i}] = ({i} < {n}) + 0.0 * inputs[{base + i}] + 0.0 * inputs[{8 + base // N}];")
        lines += ["    outputs[8] = nup;", "    outputs[9] = nlo;", "}"]
        return "\n".join(lines)
    lines = [f"double M = {m};", f"double NN = {N}.0;", f"double W = {2 * math.pi * F1};", "void step() {",
             "    double ref = M * sin(W * time);",
             "    double nup = round(NN / 2.0 * (1.0 - ref));",
             "    double nlo = NN - nup;",
             "    double cu = inputs[8] > 0;",
             "    double cl = inputs[9] > 0;"]
    for arm, base, n, cflag in (("u", 0, "nup", "cu"), ("l", N, "nlo", "cl")):
        for i in range(N):
            terms = []
            for j in range(N):
                if j == i:
                    continue
                vi, vj = f"inputs[{base + i}]", f"inputs[{base + j}]"
                # j ranks below i if lower voltage, or equal voltage and lower index
                terms.append(f"({vj} < {vi})" if j > i else f"({vj} <= {vi})")
            lines.append(f"    double r{arm}{i} = {' + '.join(terms)};")
            lines.append(f"    outputs[{base + i}] = {cflag} * (r{arm}{i} < {n}) + (1 - {cflag}) * (r{arm}{i} >= NN - {n});")
    lines += ["    outputs[8] = nup;", "    outputs[9] = nlo;", "}"]
    return "\n".join(lines)


def build(m: float = 0.9, sort: bool = True) -> sp.Circuit:
    c = sp.Circuit(NAME)
    vp = c.add("V", "VdcP", value=VDC / 2)
    vn = c.add("V", "VdcN", value=VDC / 2)
    c.connect(vn.A, vp.B)
    c.ground(vn.A)
    ctl = c.add("CSCRIPT", "NLM", code=controller_code(m, sort), timestep=TS)

    def arm(prefix, top, k0):
        """String of N SMs starting at net ``top``; returns the bottom pin."""
        node = top
        for k in range(N):
            idx = k0 + k
            s1 = c.add("IGBT_DIODE", f"S{idx}a", Ron=RON)
            s2 = c.add("IGBT_DIODE", f"S{idx}b", Ron=RON)
            cap = c.add("C", f"C{idx}", C=CSM, vC0=VDC / N)
            vm = c.add("VM", f"Vc{idx}")
            # X = s1.S = s2.D ; Y = s2.S = cap.B ; cap.A = s1.D
            c.connect(cap.A, s1.D, vm.A)
            c.connect(cap.B, s2.S, vm.B)
            c.connect(s1.S, s2.D)
            c.connect(node, s1.S)
            node = s2.S
            c.connect(vm.Out, getattr(ctl, f"In{idx + 1}"))
            inv = c.add("NOT", f"Nb{idx}")
            c.connect(getattr(ctl, f"Out{idx + 1}"), s1.G, inv.In)
            c.connect(inv.Out, s2.G)
        return node

    # upper arm: +rail -> AMup -> SMs -> Lup -> AC
    am_u = c.add("AM", "Iup")
    c.connect(vp.A, am_u.A)
    bot_u = arm("u", am_u.B, 0)
    l_u = c.add("L", "Lup", L=LARM)
    c.connect(bot_u, l_u.A)
    # lower arm: AC -> Llo -> AMlo -> SMs -> -rail
    l_l = c.add("L", "Llo", L=LARM)
    am_l = c.add("AM", "Ilo")
    c.connect(l_u.B, l_l.A)
    c.connect(l_l.B, am_l.A)
    bot_l = arm("l", am_l.B, N)
    c.connect(bot_l, vn.B)
    c.connect(am_u.Out, ctl.In9)
    c.connect(am_l.Out, ctl.In10)

    rl = c.add("R", "Rload", value=R)
    ll = c.add("L", "Lload", L=L)
    c.connect(l_u.B, rl.A)
    c.connect(rl.B, ll.A)
    c.ground(ll.B)
    vo = c.add("VM", "Vout"); c.connect(vo.A, l_u.B); c.ground(vo.B)
    vau = c.add("VM", "Varm_up"); c.connect(vau.A, am_u.B); c.connect(vau.B, bot_u)
    val = c.add("VM", "Varm_lo"); c.connect(val.A, am_l.B); c.connect(val.B, bot_l)
    return c


def settings(m: float = 0.9, sort: bool = True) -> dict:
    return dict(t_stop=0.3, dt=2e-6)


def checks(res: sp.Results, m: float = 0.9, sort: bool = True) -> list[cm.Check]:
    if not sort:
        means = [cm.mean(res, f"Vc{i}.Out", 0.25) for i in range(2 * N)]
        spread = (max(means) - min(means)) / (VDC / N)
        return [cm.greater("Negative control, no sorting: capacitor spread [%]", 100 * spread, 10.0)]
    t_ss = 0.2
    w = 2 * math.pi * F1
    vc_nom = VDC / N
    vu = cm.window(res, "Varm_up.Out", t_ss)
    vl = cm.window(res, "Varm_lo.Out", t_ss)
    n_lv = len(cm.levels(vu, 0.4 * vc_nom))
    # internal EMF e = (v_lo - v_up)/2 (with v_up + v_lo ~ Vdc)
    t = res.time[res.time >= t_ss]
    e = 0.5 * (vl - vu)
    k = int(round(2 / F1 / (t[1] - t[0])))
    e2, t2 = e[-k:], t[-k:]
    e1 = 2 * abs(np.mean(e2 * np.exp(-1j * w * t2)))
    zl = complex(R, w * L)
    v_th = e1 * abs(zl / (zl + 1j * w * LARM / 2))
    fv = res.fourier("Vout.Out", f0=F1, cycles=2)
    # Ideal NLM staircase (sampled every TS, held): e = Vdc/N * (N/2 - n_up),
    # n_up = round(N/2 (1 - m sin wt)). With few levels its fundamental is not
    # exactly m Vdc/2, so compare with the staircase's own Fourier coefficient.
    ts = np.arange(0, 1 / F1, TS)
    n_up = np.round(N / 2 * (1 - m * np.sin(w * ts)))
    stair = vc_nom * (N / 2 - n_up)
    e1_th = 2 * abs(np.mean(stair * np.exp(-1j * w * (ts + TS / 2))))
    means = [cm.mean(res, f"Vc{i}.Out", t_ss) for i in range(2 * N)]
    spread = (max(means) - min(means)) / vc_nom
    i_load = cm.window(res, "I_Rload", t_ss)
    p_load = R * float(np.mean(i_load ** 2))
    p_dc = sum(VDC / 2 * -cm.mean(res, f"I_{n}", t_ss) for n in ("VdcP", "VdcN"))
    i_arm = np.abs(cm.window(res, "Iup.Out", t_ss)).mean() + np.abs(cm.window(res, "Ilo.Out", t_ss)).mean()
    p_cond = N * VDEV * i_arm
    return [
        cm.approx("Upper-arm voltage levels (N+1)", n_lv, N + 1, abs=0),
        cm.approx("mean(v_up + v_lo) [V] (= Vdc)", float(np.mean(vu + vl)), VDC, rel=0.01),
        cm.approx(f"Internal EMF fundamental [V pk] (= NLM staircase, m Vdc/2 = {m * VDC / 2:.0f})", e1, e1_th, rel=0.01),
        cm.approx("Load voltage fundamental [V pk] (EMF through Larm/2)", fv.fundamental, v_th, rel=0.01),
        cm.approx("Mean SM capacitor voltage [V] (= Vdc/N)", float(np.mean(means)), vc_nom, rel=0.01),
        cm.less("Capacitor balancing: spread of means [%]", 100 * spread, 1.0),
        cm.approx("DC power = load + conduction loss [W]", p_dc, p_load + p_cond, rel=0.02),
    ]


if __name__ == "__main__":
    cm.run_model(__import__(__name__))
