"""Generic block harness for the block verification plan (Phases 1 and 2).

Builds a test circuit for any block type, runs it through the simpel API and
collects metrics. Analytic checks live in test_functional*.py (Phase 3).
"""
from __future__ import annotations

import math
import re

import numpy as np

import simpel as sp

CTRL_SRC_T = 2.0e-6           # control-only circuits: fine time step
CTRL_T_STOP = 0.4             # two sine cycles at 5 Hz; long enough for delays of 20 ms

# Baseline parameters per block. They put the block in a regime where its parameters
# can act (e.g. limits the sine input actually reaches, a step inside the window).
# Phase 2 varies one parameter at a time on top of these baselines.
BASE_PARAMS = {
    "PULSE_GEN": {"period": 0.02, "width": 0.5},
    "STEP": {"step_time": 0.05},
    "DELAY": {"delay": 0.02},
    "TRANSPORT_DELAY": {"delay": 0.02},
    "TURN_ON_DELAY": {"delay": 0.02},
    "CONT_PID": {"Ki": 5.0, "Kd": 0.1, "limit_output": "true", "upper_limit": 0.5,
                 "lower_limit": -0.5},
    "SATURATION": {"min": -0.5, "max": 0.5},
    "DEAD_ZONE": {"start": -0.2, "end": 0.2},
    "RELAY": {"on_threshold": 0.5, "off_threshold": -0.5},
    "PWM_3PH": {"dead_time": 1e-5},
    "SVPWM": {"dead_time": 1e-5},
}

# Routing blocks have no meaningful output on their own. GOTO/FROM only carry a signal
# when both halves of a tag pair exist, and INPORT only exists inside a subsystem. Phase 1
# validates them as placed blocks. Their behaviour is checked in Phase 3 (GOTO/FROM pair:
# FROM followed by GAIN reproduces the GOTO input, verified in round 7).
ROUTING_ONLY = {"GOTO_SIG", "FROM_SIG", "INPORT"}

# Blocks whose output is constant by definition.
CONSTANT_BY_DESIGN = {"CONST", "CONSTANT"}
PWR_DT = 1.0e-6               # electrical circuits
PWR_T_STOP = 2.0e-3
# Rotating machines need a stimulus that makes the shaft move, and a window long enough for the
# mechanical dynamics. See _machine_circuit.
MACHINE_TYPES = {"BLDC_MOTOR", "PMSM_MOTOR", "SYNC_MOTOR", "INDUCTION_MOTOR", "IND_MOTOR"}
MACHINE_DT = 2.0e-5
MACHINE_T_STOP = 0.2

# Output pins for blocks whose names do not start with "Out".
OUTPUT_PINS = {
    "PARK": ("d", "q"), "PARK_TRANSFORM": ("d", "q"),
    "INV_PARK": ("Alpha", "Beta"), "INV_PARK_TRANSFORM": ("Alpha", "Beta"),
    "CLARKE": ("Alpha", "Beta"), "CLARKE_TRANSFORM": ("Alpha", "Beta"),
    "INV_CLARKE": ("A", "B", "C"), "ABC_DQ": ("d", "q"), "DQ_ABC": ("A", "B", "C"),
    "PLL_3PH": ("Theta", "Freq"), "PLL_LOOP": ("Theta", "Freq", "Cos", "Sin"),
    "PLL_1PH": ("Out",), "FOURIER_TRANS": ("Mag", "Phase"), "FOURIER_ANALYSIS": ("Mag", "Phase"),
    "PWM_MASTER": ("P1", "P1_n", "P2", "P2_n", "P3", "P3_n"),
    "D_FLIP_FLOP": ("Q", "Q_bar"), "PWM_3PH": ("OutA", "OutB", "OutC"),
    "SVPWM": ("OutA", "OutB", "OutC"), "BUS_CREATOR": ("Bus",), "BUS_SELECTOR": ("Out1", "Out2"),
    "TERMINATOR": (),
}

# Pins that are control inputs on electrical blocks (everything else is a power terminal).
CTRL_PIN_NAMES = {"G", "Ctrl", "Gate", "Trigger"}
CTRL_PIN_BY_TYPE = {"BJT": {"B"}}

# Parameters that describe structure (pin counts, operator strings, lists). They are not
# varied by Phase 2; Phase 3 covers them.
SKIP_PARAMS = {
    "inputs", "num_inputs", "channels", "terminals", "port_number", "num_carriers",
    "primary_turns", "secondary_turns", "indexing", "operators", "signs", "state",
    "type", "criteria", "function", "edge_type", "direction", "mode", "datatype",
    "operator", "tag", "code", "coefficients", "x_data", "y_data", "z_data", "num", "den",
    "A", "B", "C", "D", "x0", "initial_guess", "signals", "selected_signals", "target",
    "Gate_Signal_Label", "probe_target", "label", "trigger_type", "states_when_disabled",
    "connection", "polarity", "edge", "method", "criteria", "phase", "timestep",
}

_SUFFIX = {"k": 1e3, "K": 1e3, "M": 1e6, "meg": 1e6, "G": 1e9,
           "m": 1e-3, "u": 1e-6, "n": 1e-9, "p": 1e-12}


def parse_num(s):
    """Parse an engineering string ("10k", "1M", "100u", "2.5") or return None."""
    m = re.fullmatch(r"\s*([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)\s*(meg|[kKMGmunp])?\s*",
                     str(s))
    if not m:
        return None
    return float(m.group(1)) * _SUFFIX.get(m.group(2), 1.0)


def is_output(btype: str, pin: str) -> bool:
    if btype in OUTPUT_PINS:
        return pin in OUTPUT_PINS[btype]
    return pin.startswith("Out") or pin in ("Q", "Q_bar")


def pin_names(btype: str):
    return [p.name for p in sp.block_info(btype).pins]


# Operator blocks: each choice is tested under several constant input sets, because the
# choices only differ for some input pairs (for example "<" and "<=" differ only at equality).
OPERATOR_SCENARIOS = {
    "RELATIONAL_OPERATOR": [(1.0, 1.0), (0.2, 0.5), (0.5, 0.2)],
    "BITWISE_OP": [(3.0, 5.0), (6.0, 6.0), (5.0, 3.0)],
    "COMPARE_TO_CONSTANT": [(1.0,), (0.5,), (1.5,)],
}


def build_control(btype: str, params: dict, inputs=None) -> sp.Circuit:
    """Control/general block: one sine source per input pin, or constants if given."""
    c = sp.Circuit("t")
    c.add(btype, "U", **params)
    ins = [p for p in pin_names(btype) if not is_output(btype, p)]
    for i, pin in enumerate(ins):
        if inputs is None:
            c.add("SINE_WAVE", f"S{i}", amplitude=1.0, frequency=5.0 + 2.0 * i, phase=40.0 * i)
        else:
            c.add("CONST", f"S{i}", value=float(inputs[min(i, len(inputs) - 1)]))
        c.connect(f"S{i}.Out", f"U.{pin}")
    return c


# Load resistance per block. A thyristor only stays latched while its current is above the
# holding current (10 mA by default). The generic 1 kOhm load gives about 9 mA at 10 V, so it
# drops out. 100 Ohm gives about 90 mA.
LOAD_OHMS = {"THYRISTOR": 600.0}
# Gate level per block. 0.75 V sits between the default trigger (0.5 V) and the Vgt variant
# (1.0 V), so changing Vgt changes whether the device fires.
GATE_LEVEL = {"THYRISTOR": 0.75}


def _machine_circuit(btype: str, params: dict) -> sp.Circuit:
    """Machine driven the way it runs: a 3-phase supply for induction, a balanced resistive star
    load with a driving torque for the PM and synchronous machines, and a field source for the
    synchronous machine. The DC stimulus used for other electrical blocks produces no rotating
    field, so speed and inertia would have no effect on the signals."""
    c = sp.Circuit("m")
    c.add(btype, "U", **params)
    if btype in ("INDUCTION_MOTOR", "IND_MOTOR"):
        c.add("V_3PH", "VS", magnitude=180.0, frequency=50.0, phase=0.0)
        c.ground("VS.N")
        c.connect("VS.A", "U.A")
        c.connect("VS.B", "U.B")
        c.connect("VS.C", "U.C")
        tl = 0.0
    else:
        names = ["RS0", "RS1", "RS2"]
        for n, term in zip(names, ["U.A", "U.B", "U.C"]):
            c.add("R", n, value=1.0)
            c.connect(term, f"{n}.A")
        c.connect(f"{names[0]}.B", f"{names[1]}.B")
        c.connect(f"{names[0]}.B", f"{names[2]}.B")
        c.ground(f"{names[0]}.B")
        tl = -0.03
        if btype == "SYNC_MOTOR":
            c.add("V", "VF", value=24.0)
            c.ground("VF.B")
            c.connect("VF.A", "U.FA")
            c.connect("U.FB", "VF.B")
    c.add("CONST", "TLS", value=tl)
    c.connect("TLS.Out", "U.TL")
    return c


def build_power(btype: str, params: dict, ctrl_value: float = 1.0) -> sp.Circuit:
    """Electrical/magnetic block between a 10 V source and a 1 kOhm load."""
    if btype in MACHINE_TYPES:
        return _machine_circuit(btype, params)
    c = sp.Circuit("t")
    pins = pin_names(btype)
    ctrl_names = CTRL_PIN_BY_TYPE.get(btype, set()) | (CTRL_PIN_NAMES & set(pins))
    power = [p for p in pins if p not in ctrl_names]
    # Diode off state: reverse the bias by swapping the source and load terminals.
    if btype == "D" and ctrl_value == 0.0:
        power = power[::-1]
    c.add(btype, "U", **params)
    if len(power) < 2:
        raise ValueError(f"{btype}: fewer than two power pins")
    c.add("V", "VS", value=10.0)
    c.ground("VS.B")
    c.connect("VS.A", f"U.{power[0]}")
    c.add("R", "RL", value=LOAD_OHMS.get(btype, 1e3))
    c.connect(f"U.{power[1]}", "RL.A")
    c.ground("RL.B")
    # Component ids are indexed, not derived from pin names: pins such as "M+" and "M-"
    # sanitize to the same text and would collide.
    for k, p in enumerate(power[2:]):
        rid = f"RX{k}"
        c.add("R", rid, value=1e6)
        c.connect(f"U.{p}", f"{rid}.A")
        c.ground(f"{rid}.B")
    for k, p in enumerate(sorted(ctrl_names)):
        kid = f"K{k}"
        level = GATE_LEVEL.get(btype, ctrl_value) if ctrl_value == 1.0 else ctrl_value
        c.add("CONST", kid, value=level)
        c.connect(f"{kid}.Out", f"U.{p}")
    return c


def is_power_block(category: str, btype: str) -> bool:
    if category in ("electrical", "magnetic"):
        return True
    return False


# Parameters that only act while the switch is open. Phase 2 runs them with the control low.
OFF_STATE_PARAMS = {"Roff"}


def build(category: str, btype: str, params: dict, ctrl_value: float = 1.0, inputs=None) -> sp.Circuit:
    merged = {**BASE_PARAMS.get(btype, {}), **params}
    # Off state for the Roff variants. A manual double switch has no control pin, so its
    # state setting is what opens it.
    if btype == "MAN_DBL_SWITCH" and ctrl_value == 0.0:
        merged.setdefault("state", "open")
    if is_power_block(category, btype):
        return build_power(btype, merged, ctrl_value)
    return build_control(btype, merged, inputs)


def simulate(c: sp.Circuit, power: bool, btype: str | None = None) -> sp.Results:
    if power and btype in MACHINE_TYPES:
        return sp.Simulation(c, t_stop=MACHINE_T_STOP, dt=MACHINE_DT).run(check=False)
    if power:
        return sp.Simulation(c, t_stop=PWR_T_STOP, dt=PWR_DT).run(check=False)
    return sp.Simulation(c, t_stop=CTRL_T_STOP, dt=CTRL_SRC_T).run(check=False)


def signals(res: sp.Results, prefix: str | None = None) -> dict:
    out = {}
    for n in res.names:
        if prefix is None or n == prefix or n.startswith(prefix):
            out[n] = np.asarray(res[n], dtype=float)
    return out


def issue_codes(c: sp.Circuit):
    issues = c.validate(raise_on_error=False)
    return [(i.severity, i.code, i.message) for i in issues]
