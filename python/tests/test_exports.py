import csv
import pathlib
import xml.etree.ElementTree as ET

import numpy as np
import pytest

import simpel as sp

from test_circuit import build_buck

DESIGNS = pathlib.Path(__file__).resolve().parents[2] / "tests" / "golden" / "designs"


@pytest.fixture(scope="module")
def res():
    return sp.Simulation(build_buck(), t_stop=2e-3, dt=1e-6).run()


def test_csv_round_trip(res, tmp_path):
    p = tmp_path / "w.csv"
    res.export_csv(p, ["V_C1", "I_L1"])
    rows = list(csv.reader(open(p, newline="")))
    assert rows[0] == ["time", "V_C1", "I_L1"]
    data = np.array(rows[1:], dtype=float)
    assert data.shape == (len(res), 3)
    assert np.allclose(data[:, 0], res.time, rtol=1e-11, atol=0)
    assert np.allclose(data[:, 1], res["V_C1"], rtol=1e-11, atol=1e-300)
    res.export_csv(p, precision=17)  # all signals, exact
    rows = list(csv.reader(open(p, newline="")))
    assert rows[0][1:] == res.names
    col = rows[0].index("V_C1")
    assert np.array_equal(np.array([r[col] for r in rows[1:]], dtype=float), res["V_C1"])
    with pytest.raises(KeyError):
        res.export_csv(p, ["nope"])


def test_svg_outputs(res, tmp_path):
    svg = res.svg(["V_C1", "I_L1"], title="Buck")
    ET.fromstring(svg)  # well-formed XML
    assert "V_C1" in svg and "I_L1" in svg
    p = tmp_path / "w.svg"
    res.export_svg(p, ["V_C1"], t1=1e-3)
    assert p.read_text(encoding="utf-8").startswith("<")
    sch = res.schematic_svg()
    ET.fromstring(sch)
    assert "Q1" in sch and "CMP1" in sch


def test_html_report(res, tmp_path):
    p = tmp_path / "r.html"
    res.export_html(p, plots=[["V_C1", "I_L1"], ["CMP1.Out"]])
    html = p.read_text(encoding="utf-8")
    assert "<svg" in html and "V_C1" in html and "CMP1.Out" in html
    with pytest.raises(KeyError):
        res.export_html(p, plots=[["V_C"]])


def test_html_report_uses_design_scopes(tmp_path):
    r = sp.Simulation(DESIGNS / "test01_buck_pid.json", t_stop=1e-3).run(check=False)
    p = tmp_path / "r.html"
    r.export_html(p)
    assert p.stat().st_size > 10000


def test_decimated_plot_keeps_spikes(res):
    mpl = pytest.importorskip("matplotlib")
    mpl.use("Agg")
    ax = res.plot(["I_L1"], max_points=200)
    (line,) = ax.get_lines()
    y = line.get_ydata()
    assert len(y) <= 200
    assert np.max(y) == pytest.approx(np.max(res["I_L1"]))
    assert np.min(y) == pytest.approx(np.min(res["I_L1"]))
    axes = res.plot(["V_C1", "I_L1"], separate=True, t1=1e-3)
    assert len(axes) == 2
    mpl.pyplot.close("all")
