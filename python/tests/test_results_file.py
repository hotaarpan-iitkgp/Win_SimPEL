import pathlib
import subprocess
import time

import numpy as np
import pytest

import simpel as sp

from test_circuit import build_buck

ROOT = pathlib.Path(__file__).resolve().parents[2]
DESIGNS = ROOT / "tests" / "golden" / "designs"
GUI = ROOT / "build" / "circuitsim_pro_win.exe"
CLI = ROOT / "build" / "simpel_cli.exe"


def test_round_trip_is_bitwise(tmp_path):
    res = sp.Simulation(build_buck(), t_stop=2e-3, dt=1e-6, step="adaptive").run()
    p = tmp_path / "buck.simres"
    res.save_for_gui(p)
    back = sp.load_results(p)
    assert np.array_equal(back.time, res.time)
    assert back.names == res.names
    for n in res.names:
        assert np.array_equal(back[n], res[n]), n
    assert back.netlist == res.netlist
    assert back.step == "adaptive" and back.t_stop == pytest.approx(res.t_stop)
    assert back.stats("V_C1", t1=1e-3) == res.stats("V_C1", t1=1e-3)
    assert "Q1" in back.schematic_svg()


def test_cancelled_results_keep_the_flag(tmp_path):
    res = sp.Simulation(build_buck(), t_stop=1.0, dt=1e-7).run(lambda t, ts: t < 1e-3)
    p = tmp_path / "partial.simres"
    res.save_for_gui(p)
    assert sp.load_results(p).cancelled


def test_reads_files_written_by_the_cli(tmp_path):
    if not CLI.is_file():
        pytest.skip("simpel_cli not built")
    p = tmp_path / "cli.simres"
    design = DESIGNS / "Buck_converter.json"
    subprocess.run([str(CLI), "run", str(design), "-t", "2e-3", "--simres", str(p)], check=True,
                   capture_output=True)
    back = sp.load_results(p)
    ref = sp.run_design_file(design, t_stop=2e-3)
    assert np.array_equal(back.time, ref["time"])
    for k, v in ref["signals"].items():
        assert np.array_equal(back[k], v)


def test_bad_files(tmp_path):
    with pytest.raises(FileNotFoundError):
        sp.load_results(tmp_path / "nope.simres")
    bad = tmp_path / "bad.simres"
    bad.write_bytes(b"SIMPELRS" + b"\x01\x00\x00\x00" + b"\xff" * 8)
    with pytest.raises(RuntimeError, match="not a valid .simres"):
        sp.load_results(bad)


def test_open_in_gui_locating(monkeypatch, tmp_path):
    monkeypatch.setenv("SIMPEL_GUI", str(tmp_path / "missing.exe"))
    assert sp.find_gui() is None
    res = sp.Simulation(build_buck(), t_stop=1e-4).run()
    with pytest.raises(FileNotFoundError, match="SIMPEL_GUI"):
        sp.open_in_gui(res)


@pytest.mark.skipif(not GUI.is_file(), reason="GUI not built")
def test_open_in_gui_starts_the_application():
    res = sp.Simulation(build_buck(), t_stop=1e-3).run()
    proc = sp.open_in_gui(res, gui=GUI)
    try:
        time.sleep(3)
        assert proc.poll() is None  # still running: it opened the file
    finally:
        proc.kill()
        proc.wait()
