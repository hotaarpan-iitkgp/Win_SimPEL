"""Phase 3, magnetic: flux and MMF answers from the magnetic-circuit analogy.

Analogy used by the engine: MMF is a voltage (A-turns), flux is a current (Wb per second),
reluctance R is a resistor, permeance P = 1/R is a conductance. Flux through a reluctance is
Phi = F / R, and reluctances in series add. The air gap and core reluctances use
R = l / (mu0 * mu_r * A).
"""
from __future__ import annotations

import math
import warnings

import numpy as np
import pytest

import simpel as sp

warnings.simplefilter("ignore")

MU0 = 4e-7 * math.pi


def _flux(blocks, mmf: float = 100.0):
    """MMF source driving the given magnetic blocks in series. Returns (flux, MMF drop per block)."""
    c = sp.Circuit("mag")
    c.add("MMF_SRC", "F", F=mmf)
    prev = "F.A"
    names = []
    for i, (btype, params) in enumerate(blocks):
        name = f"U{i}"
        c.add(btype, name, **params)
        c.connect(prev, f"{name}.A")
        prev = f"{name}.B"
        names.append(name)
    c.connect(prev, "F.B")
    r = sp.Simulation(c, t_stop=2e-4, dt=1e-6).run(check=False)
    flux = float(np.asarray(r[f"I_{names[0]}"])[-1])
    drops = [float(np.asarray(r[f"V_{n}"])[-1]) for n in names]
    return flux, drops


def test_reluctance_flux_is_mmf_over_reluctance():
    """Rm = 1 MOhm, 100 A-turns: Phi = 100 / 1e6 = 1e-4 Wb/s."""
    flux, _ = _flux([("MAG_RESISTANCE", {"Rm": 1e6})])
    assert flux == pytest.approx(1e-4, rel=1e-3)


def test_mmf_drop_across_reluctance_equals_source():
    _, drops = _flux([("MAG_RESISTANCE", {"Rm": 1e6})], mmf=100.0)
    assert drops[0] == pytest.approx(100.0, rel=1e-3)


def test_series_reluctances_add():
    """Two 1 MOhm reluctances in series act as 2 MOhm: Phi = 100 / 2e6 = 5e-5."""
    flux, drops = _flux([("MAG_RESISTANCE", {"Rm": 1e6}), ("MAG_RESISTANCE", {"Rm": 1e6})])
    assert flux == pytest.approx(5e-5, rel=1e-3)
    assert drops[0] == pytest.approx(50.0, rel=1e-3)
    assert drops[1] == pytest.approx(50.0, rel=1e-3)


def test_permeance_flux_is_permeance_times_mmf():
    """Permeance P = 1 uWb/A-turn: Phi = P * F = 1e-6 * 100 = 1e-4 Wb/s. (Was zero at DC
    before the permeance blocks were changed from capacitors to conductances.)"""
    flux, _ = _flux([("MAG_PERMEANCE", {"P": 1e-6})])
    assert flux == pytest.approx(1e-4, rel=1e-3)


def test_leakage_path_permeance():
    """P = 10 nWb/A-turn: Phi = 10e-9 * 100 = 1e-6 Wb/s."""
    flux, _ = _flux([("LEAKAGE_PATH", {"P": 10e-9})])
    assert flux == pytest.approx(1e-6, rel=1e-3)


def test_linear_core_reluctance_from_geometry():
    """l = 0.1 m, mu_r = 1000, A = 1e-4 m2: R = l / (mu0 mu_r A), so Phi = F mu0 mu_r A / l."""
    expected = 100.0 * MU0 * 1000.0 * 1e-4 / 0.1
    flux, _ = _flux([("LINEAR_CORE", {"l": 0.1, "mu_r": 1000, "A": 1e-4})])
    assert flux == pytest.approx(expected, rel=0.01)


def test_air_gap_reluctance_from_geometry():
    """l = 1 m, A = 1e-4 m2 in air: Phi = F mu0 A / l. The flux is about 1.3e-8, which is
    small, so the tolerance is looser."""
    expected = 100.0 * MU0 * 1e-4 / 1.0
    flux, _ = _flux([("AIR_GAP", {"l": 1.0, "A": 1e-4})])
    assert flux == pytest.approx(expected, rel=0.05)


def test_permanent_magnet_mmf_offset_reduces_flux():
    """A permeance with an MMF offset F0 = 40 A-turns (permanent magnet) passes
    Phi = P (F - F0) = 1e-6 * (100 - 40) = 6e-5 Wb/s."""
    flux, _ = _flux([("MAG_PERMEANCE", {"P": 1e-6, "F0": 40.0})])
    assert flux == pytest.approx(6e-5, rel=1e-3)


def test_offset_equal_to_mmf_gives_no_flux():
    flux, _ = _flux([("MAG_PERMEANCE", {"P": 1e-6, "F0": 100.0})])
    assert abs(flux) < 1e-9
