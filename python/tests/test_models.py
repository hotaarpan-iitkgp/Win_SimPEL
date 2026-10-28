"""End-to-end converter models (python/examples/models): every operating point
is simulated and compared with theory; the saved schematic and results files
must reload to the same circuit and waveforms."""

import importlib
import pathlib
import sys

import numpy as np
import pytest

import simpel as sp

MODELS_DIR = pathlib.Path(__file__).resolve().parents[1] / "examples" / "models"
sys.path.insert(0, str(MODELS_DIR))

import run_all  # noqa: E402
import _common as cm  # noqa: E402

CASES = [(name, case) for name in run_all.MODELS for case in importlib.import_module(name).CASES]


def _id(p):
    name, case = p
    return name + cm.case_suffix(case)


@pytest.mark.parametrize("model_case", CASES, ids=[_id(p) for p in CASES])
def test_model_matches_theory(model_case):
    name, case = model_case
    r = cm.run_case(importlib.import_module(name), case, save=False)
    failed = [str(c) for c in r.checks if not c.passed]
    assert not failed, f"{r.label}:\n  " + "\n  ".join(failed)


@pytest.mark.parametrize("name", run_all.MODELS)
def test_model_files_round_trip(name, tmp_path):
    mod = importlib.import_module(name)
    case = mod.CASES[0]
    c = mod.build(**case)
    assert [i for i in c.validate() if i.severity == "error"] == []
    p = tmp_path / f"{name}.json"
    c.save(p)
    again = sp.Circuit.load(p)
    dt = mod.settings(**case)["dt"]
    r1 = sp.Simulation(c, t_stop=200 * dt, dt=dt).run()
    r2 = sp.Simulation(again, t_stop=200 * dt, dt=dt).run()
    assert r1.netlist == r2.netlist
    q = tmp_path / f"{name}.simres"
    r1.save_for_gui(q)
    back = sp.load_results(q)
    assert np.array_equal(back.time, r1.time)
    for k in r1.names:
        assert np.array_equal(back[k], r1[k])
