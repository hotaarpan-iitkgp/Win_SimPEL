"""Shared helpers for the end-to-end converter models.

Each model module defines:

* ``NAME``, ``TITLE`` - identifiers for files and reports
* ``CASES`` - a list of keyword-argument dicts, one per simulated operating point
* ``build(**case) -> simpel.Circuit``
* ``settings(**case) -> dict`` - ``t_stop``, ``dt`` (and optionally ``solver``)
* ``checks(res, **case) -> list[Check]`` - theory-vs-simulation comparisons

``run_model(module)`` simulates every case, evaluates the checks and saves a
schematic ``.json`` and a ``.simres`` per case into ``out/``.
"""

from __future__ import annotations

import math
import os
import pathlib
import time
from dataclasses import dataclass, field

import numpy as np

import simpel as sp

OUT_DIR = pathlib.Path(__file__).resolve().parent / "out"


# ── checks ──────────────────────────────────────────────────────────────────
@dataclass(frozen=True)
class Check:
    name: str
    value: float
    expected: float | None
    limit: str  # human-readable tolerance, e.g. "+/-2 %" or "< 5"
    passed: bool

    def __str__(self) -> str:
        exp = "" if self.expected is None else f" (expected {self.expected:.4g}, {self.limit})"
        if self.expected is None:
            exp = f" ({self.limit})"
        return f"[{'PASS' if self.passed else 'FAIL'}] {self.name}: {self.value:.4g}{exp}"


def approx(name: str, value: float, expected: float, *, rel: float | None = None, abs: float | None = None) -> Check:  # noqa: A002
    """Pass when ``value`` is within ``rel`` (fraction of ``expected``) or ``abs``."""
    if rel is None and abs is None:
        raise ValueError("give rel or abs")
    tol = max(rel * math.fabs(expected) if rel is not None else 0.0, abs or 0.0)
    limit = f"+/-{rel * 100:g} %" if rel is not None and abs is None else f"+/-{tol:.3g}"
    return Check(name, float(value), float(expected), limit, bool(math.fabs(value - expected) <= tol))


def less(name: str, value: float, limit: float) -> Check:
    return Check(name, float(value), None, f"< {limit:.4g}", bool(value < limit))


def greater(name: str, value: float, limit: float) -> Check:
    return Check(name, float(value), None, f"> {limit:.4g}", bool(value > limit))


def between(name: str, value: float, lo: float, hi: float) -> Check:
    return Check(name, float(value), None, f"in [{lo:.4g}, {hi:.4g}]", bool(lo <= value <= hi))


# ── waveform helpers ────────────────────────────────────────────────────────
def window(res: sp.Results, name: str, t1: float, t2: float | None = None) -> np.ndarray:
    t2 = res.time[-1] if t2 is None else t2
    m = (res.time >= t1) & (res.time <= t2)
    return np.asarray(res[name])[m]


def mean_power(res: sp.Results, v: str, i: str, t1: float, t2: float | None = None) -> float:
    """Mean of v*i over [t1, t2] (fixed-step runs: plain sample mean)."""
    return float(np.mean(window(res, v, t1, t2) * window(res, i, t1, t2)))


def phase_deg(f: sp.FourierResult) -> float:
    """Phase of the fundamental as a sine angle in degrees (Fourier phases are cosine-based)."""
    return math.degrees(f.phases[list(f.orders).index(1)]) + 90.0


def wrap180(a: float) -> float:
    return (a + 180.0) % 360.0 - 180.0


# ── running ─────────────────────────────────────────────────────────────────
@dataclass
class CaseResult:
    model: str
    title: str
    case: dict
    checks: list[Check]
    wall_seconds: float
    steps: int
    files: list[str] = field(default_factory=list)

    @property
    def passed(self) -> bool:
        return all(c.passed for c in self.checks)

    @property
    def label(self) -> str:
        if not self.case:
            return self.model
        return self.model + "[" + ",".join(f"{k}={v}" for k, v in self.case.items()) + "]"


def case_suffix(case: dict) -> str:
    return "".join(f"_{k}{v}".replace("-", "m").replace(".", "p") for k, v in case.items())


def simulate(module, case: dict) -> sp.Results:
    c = module.build(**case)
    issues = [i for i in c.validate() if i.severity == "error"]
    if issues:
        raise sp.CircuitError("; ".join(i.message for i in issues))
    return sp.Simulation(c, **module.settings(**case)).run()


def run_case(module, case: dict, *, save: bool = True) -> CaseResult:
    c = module.build(**case)
    t0 = time.perf_counter()
    res = sp.Simulation(c, **module.settings(**case)).run()
    wall = time.perf_counter() - t0
    checks = module.checks(res, **case)
    out = CaseResult(module.NAME, module.TITLE, dict(case), checks, wall, len(res))
    if save:
        OUT_DIR.mkdir(exist_ok=True)
        stem = OUT_DIR / (module.NAME + case_suffix(case))
        c.save(str(stem) + ".json")
        res.save_for_gui(str(stem) + ".simres")
        out.files = [os.fspath(stem) + ".json", os.fspath(stem) + ".simres"]
    return out


def run_model(module, *, save: bool = True, verbose: bool = True) -> list[CaseResult]:
    results = []
    for case in module.CASES:
        r = run_case(module, case, save=save)
        results.append(r)
        if verbose:
            print(f"== {r.label}: {module.TITLE}  ({r.steps} samples, {r.wall_seconds:.2f} s)")
            for ch in r.checks:
                print("   ", ch)
    return results
