"""Run the block verification plan (Phases 1 and 2) over every block.

    python driver.py            # all blocks
    python driver.py V R        # blocks whose names start with these prefixes

Each block runs in a separate worker process with a timeout, so a hang is recorded
as HANG and does not stop the run. Results: results/<BLOCK>.json and results/summary.json.
"""
from __future__ import annotations

import json
import os
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor

HERE = os.path.dirname(os.path.abspath(__file__))
RESULTS = os.path.join(HERE, "results")
PY = sys.executable
TIMEOUT_S = 240
WORKERS = 4

sys.path.insert(0, HERE)
import simpel as sp  # noqa: E402

CATEGORIES = ["control", "electrical", "general", "magnetic"]


def all_blocks(prefixes):
    seen = []
    for cat in CATEGORIES:
        for t in sp.blocks(cat):
            if prefixes and not any(t.startswith(p) for p in prefixes):
                continue
            seen.append((cat, t))
    return seen


def safe(name: str) -> str:
    return "".join(ch if ch.isalnum() or ch in "-_" else "_" for ch in name)


def run_one(item):
    cat, btype = item
    out = os.path.join(RESULTS, safe(btype) + ".json")
    try:
        p = subprocess.run([PY, os.path.join(HERE, "worker.py"), cat, btype, out],
                           capture_output=True, text=True, timeout=TIMEOUT_S, cwd=HERE)
        if p.returncode != 0 or not os.path.exists(out):
            rec = {"block": btype, "category": cat,
                   "phase1": {"status": "CRASH", "reason": (p.stderr or p.stdout)[-600:]}}
        else:
            with open(out, encoding="utf-8") as f:
                rec = json.load(f)
    except subprocess.TimeoutExpired:
        rec = {"block": btype, "category": cat,
               "phase1": {"status": "HANG", "reason": f"no result after {TIMEOUT_S} s"}}
    with open(out, "w", encoding="utf-8") as f:
        json.dump(rec, f, indent=1, default=str)
    return rec


def summarise(recs):
    rows = []
    for r in recs:
        p1 = r.get("phase1", {}).get("status", "?")
        p2 = r.get("phase2") or []
        effect = sum(1 for x in p2 if x.get("status") == "EFFECT")
        noeff = [x["param"] for x in p2 if x.get("status") == "NO_EFFECT"]
        errs = [x["param"] for x in p2 if x.get("status") == "ERROR"]
        rows.append({"block": r["block"], "category": r["category"], "role": r.get("role"),
                     "phase1": p1, "reason": r.get("phase1", {}).get("reason", ""),
                     "sens_effect": effect, "sens_no_effect": noeff, "sens_error": errs})
    return rows


def main(argv):
    os.makedirs(RESULTS, exist_ok=True)
    items = all_blocks(argv)
    print(f"{len(items)} blocks", flush=True)
    recs = []
    with ThreadPoolExecutor(max_workers=WORKERS) as ex:
        for i, rec in enumerate(ex.map(run_one, items), 1):
            recs.append(rec)
            print(f"[{i:3}/{len(items)}] {rec['block']:<26} {rec.get('phase1', {}).get('status')}", flush=True)
    rows = summarise(recs)
    with open(os.path.join(RESULTS, "summary.json"), "w", encoding="utf-8") as f:
        json.dump(rows, f, indent=1)
    counts = {}
    for r in rows:
        counts[r["phase1"]] = counts.get(r["phase1"], 0) + 1
    print("phase 1:", counts)
    print("phase 2 NO_EFFECT parameters:", sum(len(r["sens_no_effect"]) for r in rows))
    print("phase 2 ERROR parameters:", sum(len(r["sens_error"]) for r in rows))


if __name__ == "__main__":
    main(sys.argv[1:])
