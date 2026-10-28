"""Simulate all eleven converter models, check them against theory and save the
results for the GUI.

    python run_all.py            # all models
    python run_all.py m05 m07    # models whose name starts with these prefixes

For every operating point this writes ``out/<model>_<case>.json`` (schematic,
open with File > Open) and ``out/<model>_<case>.simres`` (waveforms, open with
File > Open Results or ``Win_SimPEL --results <file>``), then prints a summary
table. The exit code is non-zero if any check fails.
"""

from __future__ import annotations

import importlib
import pathlib
import sys

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import _common as cm  # noqa: E402

MODELS = [
    "m01_buck_open_loop",
    "m02_boost_pi",
    "m03_buck_cascaded",
    "m04_sepic_open_loop",
    "m05_flyback",
    "m06_forward_reset",
    "m07_dab_phase_shift",
    "m08_1ph_inverter_unipolar",
    "m09_3ph_inverter_spwm",
    "m10_3ph_thyristor_rectifier",
    "m11_dab_closed_loop",
]


def main(prefixes: list[str]) -> int:
    names = [m for m in MODELS if not prefixes or any(m.startswith(p) for p in prefixes)]
    rows = []
    for name in names:
        mod = importlib.import_module(name)
        rows += cm.run_model(mod)
    print()
    print(f"{'case':44} {'checks':>7} {'samples':>9} {'time [s]':>9}  result")
    for r in rows:
        n_ok = sum(c.passed for c in r.checks)
        print(f"{r.label:44} {n_ok:>3}/{len(r.checks):<3} {r.steps:>9} {r.wall_seconds:>9.2f}  {'PASS' if r.passed else 'FAIL'}")
    failed = [r for r in rows if not r.passed]
    print(f"\n{len(rows) - len(failed)}/{len(rows)} cases passed; files in {cm.OUT_DIR}")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
