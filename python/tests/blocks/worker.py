"""Run Phase 1 (smoke) and Phase 2 (parameter sensitivity) for ONE block.

    python worker.py <category> <BLOCK> <out.json>

Runs in its own process so a hang or crash only affects that block (the driver
enforces the timeout).
"""
from __future__ import annotations

import json
import math
import os
import sys
import traceback
import warnings

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import numpy as np  # noqa: E402

import simpel as sp  # noqa: E402
import harness as h  # noqa: E402

warnings.simplefilter("ignore")


def phase1(category: str, btype: str, role: str) -> dict:
    res = {"role": role}
    if role == "unsupported":
        c = sp.Circuit("t")
        c.add(btype, "U")
        codes = [i for i in h.issue_codes(c)]
        res["issues"] = codes
        res["status"] = "EXPECTED_UNSUPPORTED" if any(x[1] == "unsupported_block" for x in codes) else "FAIL"
        if res["status"] == "FAIL":
            res["reason"] = "unsupported block is not reported as unsupported_block"
        return res
    if (role in ("structural", "observer") or not sp.block_info(btype).simulated
            or btype in h.ROUTING_ONLY):
        c = sp.Circuit("t")
        c.add(btype, "U")
        res["issues"] = h.issue_codes(c)
        res["status"] = "PLACED"
        return res

    try:
        c = h.build(category, btype, {})
    except Exception as e:  # noqa: BLE001
        return {**res, "status": "FAIL", "reason": f"build: {e}"}
    issues = h.issue_codes(c)
    res["issues"] = issues
    errors = [x for x in issues if x[0] == "error"]
    try:
        power = h.is_power_block(category, btype)
        r = h.simulate(c, power, btype)
    except Exception as e:  # noqa: BLE001
        return {**res, "status": "FAIL", "reason": f"simulate: {e}",
                "trace": traceback.format_exc(limit=3)}
    sigs = h.signals(r)
    bad = [n for n, v in sigs.items() if not np.all(np.isfinite(v))]
    if bad:
        return {**res, "status": "FAIL", "reason": f"non-finite signals: {bad[:5]}"}
    if errors:
        return {**res, "status": "FAIL", "reason": f"validation error: {errors[0][1]}: {errors[0][2]}"}
    prefix = "U" if not power else None
    active = [n for n, v in sigs.items()
              if (n == "U" or n.startswith("U.")) or (power and (n.startswith("V_") or n.startswith("I_")))]
    # Control blocks are driven by sine inputs, so their outputs must vary. Electrical
    # circuits are often DC, so there the requirement is only that something is non-zero.
    if power:
        alive = any(v.size and np.max(np.abs(v)) > 1e-9 for n, v in sigs.items() if n in active)
        if not alive:
            return {**res, "status": "FAIL", "reason": "all electrical signals are zero"}
    elif btype in h.CONSTANT_BY_DESIGN:
        res["note"] = "constant by design"
    elif not any(v.size and np.ptp(v) > 1e-9 for n, v in sigs.items() if n in active):
        return {**res, "status": "FAIL", "reason": "control output does not vary with its inputs"}
    res["status"] = "PASS"
    res["signal_count"] = len(sigs)
    return res


def _variants(btype: str):
    """Yield (param, value, default, note) for Phase 2.

    Variants are compared with the block's BASELINE value (harness.BASE_PARAMS if it
    overrides the parameter, else the engine default), so a variant never equals the
    baseline by accident.
    """
    bi = sp.block_info(btype)
    baseline = h.BASE_PARAMS.get(btype, {})
    for p in bi.params:
        name = p.name
        default = str(baseline.get(name, p.default))
        choices = getattr(p, "choices", None)
        if choices:
            for ch in choices:
                if str(ch) != default:
                    yield name, str(ch), default, "choice"
            continue
        if name in h.SKIP_PARAMS:
            yield name, None, default, "skipped: structural/string parameter"
            continue
        num = h.parse_num(default)
        if num is None:
            yield name, None, default, "skipped: non-numeric parameter"
            continue
        new = 1.0 if num == 0 else 2.0 * num
        yield name, repr(new), default, "x2"


def _run(category: str, btype: str, params: dict, ctrl: float = 1.0, inputs=None):
    c = h.build(category, btype, params, ctrl_value=ctrl, inputs=inputs)
    power = h.is_power_block(category, btype)
    r = h.simulate(c, power, btype)
    return h.signals(r, None if power else "U")


def _diff(a: dict, b: dict):
    worst, where = 0.0, None
    scale = max([1.0] + [float(np.max(np.abs(v))) for v in a.values() if v.size])
    for n in sorted(set(a) & set(b)):
        x, y = a[n], b[n]
        if x.shape != y.shape:
            continue
        d = float(np.max(np.abs(x - y))) if x.size else 0.0
        if not math.isfinite(d):
            d = float("inf")
        if d > worst:
            worst, where = d, n
    return worst, where, scale


def phase2(category: str, btype: str) -> list:
    base = _run(category, btype, {})
    # Off-state baseline: the control is held low, so parameters such as Roff are exercised.
    base_off = _run(category, btype, {}, ctrl=0.0) if h.is_power_block(category, btype) else None
    out = []
    if not base or not any(n == "U" or n.startswith("U.") for n in base):
        if not h.is_power_block(category, btype):
            return [{"note": "block has no outputs; sensitivity not observable here (Phase 3)"}]
    for name, value, default, note in _variants(btype):
        row = {"param": name, "default": default, "variant": value}
        if value is None:
            row.update(status="SKIPPED", note=note)
            out.append(row)
            continue
        if name == "operator" and btype in h.OPERATOR_SCENARIOS:
            # A choice counts as EFFECT if any constant-input scenario separates it from the default.
            effect, failed = False, None
            for sc in h.OPERATOR_SCENARIOS[btype]:
                try:
                    b_sc = _run(category, btype, {}, inputs=sc)
                    v_sc = _run(category, btype, {name: value}, inputs=sc)
                except Exception as e:  # noqa: BLE001
                    failed = str(e)[:200]
                    break
                worst, _, scale = _diff(b_sc, v_sc)
                if worst > 1e-9 * scale:
                    effect = True
                    break
            if failed:
                row.update(status="ERROR", note=failed)
            else:
                row.update(status="EFFECT" if effect else "NO_EFFECT", note="constant-input scenarios")
            out.append(row)
            continue
        off_state = name in h.OFF_STATE_PARAMS and base_off is not None
        ref = base_off if off_state else base
        try:
            var = _run(category, btype, {name: (value if not _is_num(value) else float(value))},
                       ctrl=0.0 if off_state else 1.0)
        except Exception as e:  # noqa: BLE001
            row.update(status="ERROR", note=str(e)[:200])
            out.append(row)
            continue
        if off_state:
            row["state"] = "off"
        worst, where, scale = _diff(ref, var)
        if worst > 1e-9 * scale:
            row.update(status="EFFECT", max_diff=worst, signal=where)
        else:
            row.update(status="NO_EFFECT", note=note)
        out.append(row)
    return out


def _is_num(v):
    try:
        float(v)
        return True
    except (TypeError, ValueError):
        return False


def main():
    category, btype, out_path = sys.argv[1], sys.argv[2], sys.argv[3]
    role = sp.block_info(btype).role
    result = {"block": btype, "category": category, "role": role}
    try:
        result["phase1"] = phase1(category, btype, role)
    except Exception as e:  # noqa: BLE001
        result["phase1"] = {"status": "FAIL", "reason": f"harness: {e}",
                            "trace": traceback.format_exc(limit=3)}
    if result["phase1"].get("status") == "PASS" and role == "model":
        try:
            result["phase2"] = phase2(category, btype)
        except Exception as e:  # noqa: BLE001
            result["phase2"] = [{"status": "ERROR", "note": f"sweep: {e}"}]
    with open(out_path, "w", encoding="utf-8") as f:
        json.dump(result, f, indent=1, default=str)


if __name__ == "__main__":
    main()
