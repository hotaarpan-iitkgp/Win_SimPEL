"""Phase 3, logic and comparison: truth tables and comparison answers.

Each case drives the block's input pins with CONST sources and checks the output at the end.
"""
from __future__ import annotations

import warnings

import numpy as np
import pytest

import simpel as sp

warnings.simplefilter("ignore")

OUT_PINS = {"Out", "Q", "Q_bar"}


def _block(btype: str, inputs, name: str = "U", **params) -> sp.Circuit:
    c = sp.Circuit("l")
    c.add(btype, name, **params)
    ins = [p.name for p in sp.block_info(btype).pins
           if p.name not in OUT_PINS and not p.name.startswith("Out")]
    for i, v in enumerate(inputs):
        c.add("CONST", f"K{i}", value=float(v))
        c.connect(f"K{i}.Out", f"{name}.{ins[i]}")
    return c


def _final(c: sp.Circuit, signal: str = "U.Out") -> float:
    r = sp.Simulation(c, t_stop=1e-3, dt=1e-6).run(check=False)
    return float(np.asarray(r[signal])[-1])


TRUTH = {
    "AND":  [0, 0, 0, 1],
    "OR":   [0, 1, 1, 1],
    "XOR":  [0, 1, 1, 0],
    "NAND": [1, 1, 1, 0],
    "NOR":  [1, 0, 0, 0],
    "XNOR": [1, 0, 0, 1],
}
PAIRS = [(0, 0), (0, 1), (1, 0), (1, 1)]


@pytest.mark.parametrize("op", sorted(TRUTH))
def test_logic_op_truth_table(op):
    for (a, b), expected in zip(PAIRS, TRUTH[op]):
        c = _block("LOGIC_OP", [a, b], operator=op)
        assert _final(c) == pytest.approx(expected, abs=1e-9), f"{op}({a},{b})"


@pytest.mark.parametrize("value, expected", [(0.0, 1.0), (1.0, 0.0)])
def test_not_gate(value, expected):
    assert _final(_block("NOT", [value])) == pytest.approx(expected, abs=1e-9)


@pytest.mark.parametrize("a, b, op, expected", [
    (0.2, 0.5, "<", 1.0),
    (0.2, 0.5, ">", 0.0),
    (0.5, 0.5, "<=", 1.0),
    (0.5, 0.5, "==", 1.0),
    (0.5, 0.5, "!=", 0.0),
    (0.5, 0.2, ">=", 1.0),
])
def test_relational_operator(a, b, op, expected):
    assert _final(_block("RELATIONAL_OPERATOR", [a, b], operator=op)) == pytest.approx(expected, abs=1e-9)


@pytest.mark.parametrize("value, op, expected", [
    (1.5, ">", 1.0),
    (0.5, ">", 0.0),
    (1.0, ">=", 1.0),
    (0.5, ">=", 0.0),
])
def test_compare_to_constant_default_threshold(value, op, expected):
    """The constant parameter is 1.0 by default."""
    assert _final(_block("COMPARE_TO_CONSTANT", [value], operator=op)) == pytest.approx(expected, abs=1e-9)
