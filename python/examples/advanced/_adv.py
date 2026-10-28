"""Shared helpers for the advanced control examples.

Re-uses the check/run machinery of ``examples/models/_common.py`` (Check, approx,
less, greater, between, run_model) and adds a few power-electronics helpers.

Model module contract (same as examples/models):
    NAME, TITLE, CASES, build(**case) -> Circuit, settings(**case) -> dict,
    checks(res, **case) -> list[Check]
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

_MODELS = pathlib.Path(__file__).resolve().parents[1] / "models"
if str(_MODELS) not in sys.path:
    sys.path.insert(0, str(_MODELS))

from _common import (  # noqa: E402,F401  (re-exported)
    Check,
    approx,
    between,
    greater,
    less,
    phase_deg,
    run_case,
    run_model,
    window,
    wrap180,
)
import _common as _base  # noqa: E402

import simpel as sp  # noqa: E402

# Results and schematics of the advanced models go to examples/advanced/out.
_base.OUT_DIR = pathlib.Path(__file__).resolve().parent / "out"
OUT_DIR = _base.OUT_DIR


def mean(res: sp.Results, name: str, t1: float, t2: float | None = None) -> float:
    return float(np.mean(window(res, name, t1, t2)))


def inst_pq(res: sp.Results, v: tuple[str, str, str], i: tuple[str, str, str], t1: float, t2: float | None = None):
    """Mean instantaneous active and reactive power from abc waveforms.

    p = va*ia + vb*ib + vc*ic
    q = [(vb - vc)*ia + (vc - va)*ib + (va - vb)*ic] / sqrt(3)
    """
    va, vb, vc = (window(res, n, t1, t2) for n in v)
    ia, ib, ic = (window(res, n, t1, t2) for n in i)
    p = va * ia + vb * ib + vc * ic
    q = ((vb - vc) * ia + (vc - va) * ib + (va - vb) * ic) / math.sqrt(3)
    return float(np.mean(p)), float(np.mean(q))


def levels(x: np.ndarray, tol: float) -> list[float]:
    """Distinct plateau levels of a staircase waveform (clustered within ``tol``)."""
    xs = np.sort(np.asarray(x))
    out: list[float] = []
    for v in xs:
        if not out or v - out[-1] > tol:
            out.append(float(v))
    return out


def cycle_mean(res: sp.Results, name: str, f0: float, t1: float) -> np.ndarray:
    """Per-cycle means of ``name`` from t1 on (one value per 1/f0 period)."""
    t = res.time
    x = np.asarray(res[name])
    n = max(1, int(round((1 / f0) / (t[1] - t[0]))))
    m = t >= t1
    xv = x[m]
    k = len(xv) // n
    return xv[: k * n].reshape(k, n).mean(axis=1)
