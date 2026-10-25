"""Handing results over to the desktop application."""

from __future__ import annotations

import os
import shutil
import subprocess
import tempfile
from typing import Union

from .circuit import Circuit
from .simulation import Results

__all__ = ["open_in_gui", "find_gui"]

GUI_NAMES = ("circuitsim_pro_win.exe", "circuitsim_pro_win")


def find_gui() -> str | None:
    """Path of the desktop application: ``SIMPEL_GUI`` if set, else the first
    ``circuitsim_pro_win`` on ``PATH``. ``None`` if neither exists."""
    env = os.environ.get("SIMPEL_GUI")
    if env:
        return env if os.path.isfile(env) else None
    for n in GUI_NAMES:
        p = shutil.which(n)
        if p:
            return p
    return None


def open_in_gui(what: Union[Results, Circuit, str, os.PathLike], *, gui: str | os.PathLike | None = None,
                wait: bool = False) -> subprocess.Popen:
    """Open results (a :class:`Results`, or a ``.simres`` path) or a circuit
    (a :class:`Circuit`, or a schematic ``.json`` path) in the desktop
    application. Results and circuits are written to a temporary file first.

    The application is located through ``gui``, the ``SIMPEL_GUI`` environment
    variable or ``PATH``. Returns the process; ``wait=True`` waits for it to close.
    """
    exe = os.fspath(gui) if gui is not None else find_gui()
    if not exe or not os.path.isfile(exe):
        raise FileNotFoundError(
            "SimPEL desktop application not found: pass gui=..., set SIMPEL_GUI to circuitsim_pro_win.exe, "
            "or put it on PATH")
    tmpdir = tempfile.mkdtemp(prefix="simpel_")
    if isinstance(what, Results):
        path = os.path.join(tmpdir, "results.simres")
        what.save_for_gui(path)
        args = ["--results", path]
    elif isinstance(what, Circuit):
        path = os.path.join(tmpdir, f"{what.name or 'circuit'}.json")
        what.save(path)
        args = ["--open", path]
    else:
        path = os.fspath(what)
        if not os.path.isfile(path):
            raise FileNotFoundError(path)
        args = ["--results", path] if path.lower().endswith(".simres") else ["--open", path]
    proc = subprocess.Popen([exe, *args])
    if wait:
        proc.wait()
    return proc
