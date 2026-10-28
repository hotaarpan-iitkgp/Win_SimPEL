"""Run every advanced control example, check it against theory and save the
results for the GUI (examples/advanced/out/*.json + *.simres).

    python run_all.py            # all models
    python run_all.py a03 a05    # models whose name starts with these prefixes

Exit code is non-zero if any check fails.
"""

from __future__ import annotations

import importlib
import pathlib
import sys

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import _adv as cm  # noqa: E402

MODELS = [
    "a01_npc3l_inverter",
    "a02_chb7l_inverter",
    "a03_grid_following_dq",
    "a04_afe_rectifier",
    "a05_pmsm_foc",
    "a06_pmsm_dtc",
    "a07_mmc_nlm_sorting",
    "a08_droop_parallel_gfm",
    "a09_back_to_back",
    "a10_vsg_swing",
]


def main(prefixes: list[str]) -> int:
    names = [m for m in MODELS if not prefixes or any(m.startswith(p) for p in prefixes)]
    rows = []
    for name in names:
        rows += cm.run_model(importlib.import_module(name))
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
