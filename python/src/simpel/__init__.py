"""SimPEL: Python API for the SimPEL power-electronics circuit simulator.

    import simpel as sp
    c = sp.Circuit("buck")
    vin = c.add("V", "V1", value=48)
    ...
    res = sp.Simulation(c, t_stop=5e-3, dt=1e-7).run()
    res.stats("V_C1", t1=4e-3).mean
"""

from __future__ import annotations

import os
from typing import Mapping

# Loss models shipped with the package (simpel/data/thermal_models) are used unless
# SIMPEL_LOSS_MODELS_DIR says otherwise; it must be set before the library loads.
_PKG_MODELS = os.path.join(os.path.dirname(__file__), "data", "thermal_models")
if os.path.isdir(_PKG_MODELS) and not os.environ.get("SIMPEL_LOSS_MODELS_DIR"):
    os.environ["SIMPEL_LOSS_MODELS_DIR"] = _PKG_MODELS

from . import _core  # noqa: E402
from .blocks import BlockInfo, ParamInfo, PinInfo, block_info, blocks, categories
from .circuit import Circuit, CircuitError, Component, Issue, ParameterWarning, Pin, ValidationWarning
from .simulation import (
    FourierResult, LossReport, LossResult, Results, SignalStats, Simulation, load_results, loss_models,
    set_loss_model_dir,
)
from .gui import find_gui, open_in_gui
from .sweep import SweepPoint, SweepResults, sweep

__all__ = [
    "__version__", "core_version", "run_design_file",
    "blocks", "block_info", "categories", "BlockInfo", "ParamInfo", "PinInfo",
    "Circuit", "Component", "Pin", "Issue", "CircuitError", "ValidationWarning", "ParameterWarning",
    "Simulation", "Results", "SignalStats", "FourierResult", "LossResult", "LossReport",
    "loss_models", "set_loss_model_dir",
    "sweep", "SweepResults", "SweepPoint",
    "load_results", "open_in_gui", "find_gui",
]

__version__ = "0.1.0"


def core_version() -> str:
    """Version of the compiled C++ core."""
    return _core.version()


def run_design_file(
    path: str | os.PathLike,
    *,
    t_stop: float | None = None,
    dt: float | None = None,
    step: str | None = None,
    solver: str | None = None,
    params: Mapping[str, object] | None = None,
) -> dict:
    """Simulate a schematic ``.json`` file exactly as the GUI would and return the
    raw result dict. Prefer ``Simulation(path, ...).run()``, which returns
    :class:`Results`.

    ``params`` maps ``"CompId.param"`` to a value (numbers or engineering strings
    such as ``"10u"``).
    """
    p = os.fspath(path)
    if not os.path.isfile(p):
        raise FileNotFoundError(p)
    plist = [(str(k), _fmt(v)) for k, v in (params or {}).items()]
    try:
        return _core.run_design_file(p, t_stop=t_stop, dt=dt, step=step, solver=solver, params=plist)
    except RuntimeError as e:
        raise ValueError(str(e)) from None


def _fmt(v: object) -> str:
    if isinstance(v, bool):
        return "true" if v else "false"
    if isinstance(v, float):
        return repr(v)
    return str(v)
