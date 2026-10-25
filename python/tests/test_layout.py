import json
import pathlib

import numpy as np
import pytest

import simpel as sp

from test_circuit import build_buck

DESIGNS = pathlib.Path(__file__).resolve().parents[2] / "tests" / "golden" / "designs"


def boxes(doc):
    return {c["id"]: (c["x"], c["y"]) for c in doc["components"]}


def test_auto_layout_on_save(tmp_path):
    c = build_buck()
    p = tmp_path / "buck.json"
    c.save(p)  # built in Python: laid out by default
    pos = boxes(json.loads(p.read_text()))
    assert pos["V1"][0] < pos["Q1"][0] < pos["L1"][0] < pos["C1"][0]
    assert pos["TRI1"][0] < pos["CMP1"][0]
    assert pos["CMP1"][1] > pos["Q1"][1]  # control below the power stage
    assert all(x % 20 == 0 and y % 20 == 0 for x, y in pos.values())
    assert len(set(pos.values())) == len(pos)
    # Layout never changes the simulated circuit.
    a = sp.Simulation(c, t_stop=1e-3, dt=1e-6).run()
    b = sp.Simulation(sp.Circuit.load(p), t_stop=1e-3, dt=1e-6).run()
    assert a.netlist == b.netlist and np.array_equal(a["V_C1"], b["V_C1"])


def test_loaded_design_keeps_its_drawing(tmp_path):
    src = DESIGNS / "Buck_converter.json"
    c = sp.Circuit.load(src)
    original = boxes(json.loads(src.read_text(encoding="utf-8")))
    kept = boxes(json.loads(c.to_json()))
    assert all(kept[k] == pytest.approx(v) for k, v in original.items())
    relaid = boxes(json.loads(c.to_json(layout="auto")))
    assert relaid != kept
    with pytest.raises(ValueError):
        c.to_json(layout="pretty")


def test_saved_wires_are_routed(tmp_path):
    c = build_buck()
    p = tmp_path / "buck.json"
    c.save(p)
    doc = json.loads(p.read_text())
    routes = [w["route"] for w in doc["wires"] if "route" in w]
    assert routes and len(routes) == len(doc["wires"])
    for r in routes:  # orthogonal polylines
        assert all(a["x"] == b["x"] or a["y"] == b["y"] for a, b in zip(r, r[1:]))
    placed_only = json.loads(c.to_json(layout="place"))
    assert not any("route" in w for w in placed_only["wires"])
    svg = sp.Simulation(sp.Circuit.load(p), t_stop=1e-4).run(check=False).schematic_svg()
    assert svg.count("<polyline") >= len(routes)
