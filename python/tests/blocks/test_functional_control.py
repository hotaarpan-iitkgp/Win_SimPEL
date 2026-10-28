"""Phase 3, control math: each block against a known analytic answer.

Each case is a constant-input circuit, so the answer is exact (or at a stated tolerance).
Pins are found from block_info, so the inputs are wired by name, not position.
Run from python/tests/blocks:  ..\\..\\..\\.venv\\Scripts\\python.exe -m pytest -q test_functional_control.py
"""
from __future__ import annotations

import math
import warnings

import numpy as np
import pytest

import simpel as sp

warnings.simplefilter("ignore")

OUT_PINS = {"Out", "Q", "Q_bar"}


def _block(btype: str, inputs, name: str = "U", **params) -> sp.Circuit:
    """Circuit with one block whose input pins are driven by CONST sources."""
    c = sp.Circuit("f")
    c.add(btype, name, **params)
    ins = [p.name for p in sp.block_info(btype).pins
           if p.name not in OUT_PINS and not p.name.startswith("Out")]
    for i, v in enumerate(inputs):
        c.add("CONST", f"K{i}", value=float(v))
        c.connect(f"K{i}.Out", f"{name}.{ins[i]}")
    return c


def _final(c: sp.Circuit, signal: str = "U.Out", t_stop: float = 1e-3, dt: float = 1e-6) -> float:
    r = sp.Simulation(c, t_stop=t_stop, dt=dt).run(check=False)
    return float(np.asarray(r[signal])[-1])


@pytest.mark.parametrize("btype, inputs, params, expected", [
    ("GAIN", [2.0], {"K": 2.5}, 5.0),
    ("SUBTRACT", [3.0, 1.0], {}, 2.0),
    ("SUM_RECT", [1.0, 2.0], {"signs": "++"}, 3.0),
    ("PRODUCT_RECT", [3.0, 4.0], {"operators": "**"}, 12.0),
    ("ABS", [-3.0], {}, 3.0),
    ("SIGN", [-2.0], {}, -1.0),
    ("ROUND", [2.6], {}, 3.0),
    ("MIN_MAX", [4.0, 1.0], {"function": "min"}, 1.0),
    ("MIN_MAX", [4.0, 1.0], {"function": "max"}, 4.0),
    ("TRIG_FCN", [math.pi / 2], {"function": "sin"}, 1.0),
    ("TRIG_FCN", [0.0], {"function": "cos"}, 1.0),
    ("OFFSET", [1.0], {"offset": 0.5}, 1.5),
    ("SATURATION", [3.0], {"min": -1.0, "max": 1.0}, 1.0),
    ("SATURATION", [-3.0], {"min": -1.0, "max": 1.0}, -1.0),
    ("SATURATION", [0.5], {"min": -1.0, "max": 1.0}, 0.5),
    ("DEAD_ZONE", [1.0], {"start": -0.5, "end": 0.5}, 0.5),
    ("DEAD_ZONE", [0.2], {"start": -0.5, "end": 0.5}, 0.0),
    ("LUT_1D", [1.5], {}, 3.0),
])
def test_static_block_matches_formula(btype, inputs, params, expected):
    c = _block(btype, inputs, **params)
    assert _final(c) == pytest.approx(expected, abs=1e-6)


def test_integrator_of_constant_is_ramp():
    """Integrating 2 from t = 0 gives 2 t. At t = 0.5 s the value is 1.0."""
    c = _block("INTEGRATOR", [2.0])
    assert _final(c, t_stop=0.5, dt=1e-5) == pytest.approx(1.0, abs=2e-3)


def test_transfer_function_first_order_lag():
    """H(s) = 1 / (s + 1), step of 1: y(t) = 1 - exp(-t). At t = 1 s: 0.6321."""
    c = _block("TRANSFER_FCN", [1.0], num="[1]", den="[1, 1]")
    assert _final(c, t_stop=1.0, dt=1e-4) == pytest.approx(1 - math.exp(-1), abs=5e-3)
