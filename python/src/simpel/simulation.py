"""Running simulations and analysing results."""

from __future__ import annotations

import os
import sys
import time as _time
import warnings
from dataclasses import dataclass, field
from typing import Callable, Iterable, Mapping, Union

import numpy as np

from . import _core
from .circuit import Circuit, ValidationWarning

__all__ = [
    "Simulation", "Results", "SignalStats", "FourierResult", "LossResult", "LossReport",
    "loss_models", "set_loss_model_dir", "load_results",
]

ProgressFn = Callable[[float, float], object]


@dataclass(frozen=True)
class SignalStats:
    mean: float
    rms: float
    min: float
    max: float
    abs_max: float
    pk2pk: float
    t1: float
    t2: float


@dataclass(frozen=True)
class FourierResult:
    """Exact piecewise-linear Fourier series over one window ``[t1, t2]``; the
    window length is the fundamental period."""

    f0: float
    dc: float
    fundamental: float  # peak amplitude of harmonic 1
    thd: float  # ratio, e.g. 0.05 for 5 %
    orders: np.ndarray
    magnitudes: np.ndarray  # peak amplitudes
    phases: np.ndarray  # radians
    t1: float
    t2: float

    @property
    def thd_percent(self) -> float:
        return 100.0 * self.thd

    def harmonic(self, n: int) -> float:
        """Peak amplitude of harmonic ``n`` (0 = DC)."""
        if n == 0:
            return self.dc
        idx = np.nonzero(self.orders == n)[0]
        if not len(idx):
            raise IndexError(f"harmonic {n} not computed (max {int(self.orders.max()) if len(self.orders) else 0})")
        return float(self.magnitudes[idx[0]])


@dataclass(frozen=True)
class LossResult:
    component: str
    model: str
    conduction: float  # W
    turn_on: float  # W
    turn_off: float  # W
    total: float  # W
    hard_turn_ons: int
    soft_turn_ons: int
    turn_offs: int


@dataclass(frozen=True)
class LossReport:
    t_start: float
    duration: float
    tj: float
    devices: tuple[LossResult, ...] = field(default_factory=tuple)

    @property
    def total(self) -> float:
        return sum(d.total for d in self.devices)

    def __getitem__(self, component: str) -> LossResult:
        for d in self.devices:
            if d.component == component:
                return d
        raise KeyError(f"no loss result for {component!r}; analysed: {[d.component for d in self.devices]}")

    def to_pandas(self):
        pd = _pandas()
        return pd.DataFrame([d.__dict__ for d in self.devices]).set_index("component")


def loss_models() -> list[str]:
    """Names usable as a switch's ``thermal_model`` parameter."""
    return list(_core.loss_models())


def set_loss_model_dir(*dirs: str | os.PathLike) -> int:
    """Use only the loss models found (recursively) in ``dirs``. Returns how many
    were loaded. By default the directories in ``SIMPEL_LOSS_MODELS_DIR`` and the
    GUI's search locations are used."""
    return _core.load_loss_models([os.fspath(d) for d in dirs])


def load_results(path: str | os.PathLike) -> "Results":
    """Read a ``.simres`` file written by :meth:`Results.save_for_gui`, the GUI
    (File > Save Results) or ``simpel_cli run --simres``."""
    import json

    p = os.fspath(path)
    if not os.path.isfile(p):
        raise FileNotFoundError(p)
    d = _core.read_results(p)
    meta = json.loads(d["meta_json"])
    settings = json.loads(d["settings_json"])
    t = d["time"]
    raw = {
        "time": t,
        "signals": d["signals"],
        "netlist": d["netlist"],
        "wall_seconds": float(meta.get("wall_seconds", 0.0)),
        "t_stop": float(meta.get("t_stop", settings.get("t_stop", t[-1] if len(t) else 0.0))),
        "dt": float(meta.get("dt", settings.get("dt", 0.0))),
        "solver": meta.get("solver", settings.get("solver", "")),
        "step": meta.get("step", settings.get("step", "fixed")),
        "cancelled": bool(meta.get("cancelled", False)),
        "design_json": d["design_json"],
    }
    return Results(raw, settings)


def _pandas():
    try:
        import pandas as pd
    except ImportError:  # pragma: no cover
        raise ImportError("this needs pandas: pip install pandas") from None
    return pd


class Results:
    """Waveforms of a finished (or cancelled) run.

    ``res["V_C1"]`` is a NumPy array aligned with ``res.time``. Arrays share
    memory with the simulator's output; they are not copied.
    """

    def __init__(self, raw: dict, settings: dict):
        self.time: np.ndarray = raw["time"]
        self._signals: dict[str, np.ndarray] = dict(raw["signals"])
        self.netlist: str = raw["netlist"]
        self.wall_seconds: float = raw["wall_seconds"]
        self.t_stop: float = raw["t_stop"]
        self.dt: float = raw["dt"]
        self.solver: str = raw["solver"]
        self.step: str = "adaptive" if raw["step"] == "variable" else raw["step"]
        self.cancelled: bool = raw["cancelled"]
        self.settings: dict = dict(settings)
        self._design_json: str = raw["design_json"]

    # ── signal access ───────────────────────────────────────────────────────
    @property
    def names(self) -> list[str]:
        return sorted(self._signals)

    @property
    def signals(self) -> Mapping[str, np.ndarray]:
        return dict(self._signals)

    def __getitem__(self, name: str) -> np.ndarray:
        try:
            return self._signals[name]
        except KeyError:
            import difflib

            close = difflib.get_close_matches(name, self._signals, n=3, cutoff=0.6)
            hint = f"; did you mean {close}?" if close else ""
            raise KeyError(f"no signal {name!r}{hint}") from None

    def __contains__(self, name: object) -> bool:
        return name in self._signals

    def __len__(self) -> int:
        return len(self.time)

    def to_pandas(self, names: Iterable[str] | None = None):
        """DataFrame indexed by time (column ``time`` as the index)."""
        pd = _pandas()
        cols = list(names) if names is not None else self.names
        data = {n: self[n] for n in cols}
        df = pd.DataFrame(data, index=pd.Index(self.time, name="time"))
        return df

    def to_dict(self, names: Iterable[str] | None = None) -> dict[str, np.ndarray]:
        cols = list(names) if names is not None else self.names
        out = {"time": self.time}
        out.update({n: self[n] for n in cols})
        return out

    # ── analyses ────────────────────────────────────────────────────────────
    def _window(self, t1: float | None, t2: float | None) -> tuple[float, float]:
        if not len(self.time):
            raise ValueError("no samples")
        a = float(self.time[0]) if t1 is None else float(t1)
        b = float(self.time[-1]) if t2 is None else float(t2)
        if not b > a:
            raise ValueError(f"empty window [{a}, {b}]")
        return a, b

    def stats(self, name: str, t1: float | None = None, t2: float | None = None) -> SignalStats:
        """Mean, RMS, min, max, peak-to-peak of a signal over ``[t1, t2]``
        (default: the whole run), integrating the piecewise-linear waveform so
        non-uniform (adaptive) time steps are weighted correctly."""
        a, b = self._window(t1, t2)
        d = _core.signal_stats(self.time, self[name], a, b)
        if not d["valid"]:
            raise ValueError(f"no samples of {name!r} in [{a}, {b}]")
        return SignalStats(d["mean"], d["rms"], d["min"], d["max"], d["abs_max"], d["pk2pk"], a, b)

    def fourier(self, name: str, f0: float | None = None, *, cycles: int = 1, t1: float | None = None,
                t2: float | None = None, max_harmonic: int = 50) -> FourierResult:
        """Harmonic analysis. Give ``f0`` to analyse the last ``cycles`` periods
        of the run, or an explicit window ``t1``..``t2`` whose length is the
        fundamental period."""
        if f0 is not None:
            if t1 is not None:
                raise ValueError("give either f0 or t1, not both")
            if f0 <= 0 or cycles < 1:
                raise ValueError("f0 must be > 0 and cycles >= 1")
            b = float(self.time[-1]) if t2 is None else float(t2)
            a = b - cycles / f0
            if a < float(self.time[0]) - 1e-15:
                raise ValueError(f"{cycles} cycle(s) of {f0} Hz do not fit in the run")
        else:
            a, b = self._window(t1, t2)
        d = _core.fourier(self.time, self[name], a, b, max_harmonic * cycles if f0 is not None else max_harmonic)
        if not d["valid"]:
            raise ValueError(f"Fourier analysis of {name!r} over [{a}, {b}] failed (too few samples?)")
        orders, mags, phases = d["orders"], d["magnitudes"], d["phases"]
        thd = d["thd"]
        if f0 is not None and cycles > 1:
            # The window spans several periods: harmonic k of f0 is order k*cycles.
            keep = (orders % cycles) == 0
            orders, mags, phases = orders[keep] / cycles, mags[keep], phases[keep]
            fund = mags[orders == 1]
            fund = float(fund[0]) if len(fund) else 0.0
            rest = mags[orders > 1]
            thd = float(np.sqrt(np.sum(rest ** 2)) / fund) if fund > 0 else 0.0
            return FourierResult(f0, d["dc"], fund, thd, orders[:max_harmonic], mags[:max_harmonic],
                                 phases[:max_harmonic], a, b)
        return FourierResult(d["f0"], d["dc"], d["fundamental"], thd, orders, mags, phases, a, b)

    def losses(self, t_start: float | None = None, duration: float | None = None, *, tj: float = 125.0,
               models: Mapping[str, str] | None = None) -> LossReport:
        """Semiconductor conduction and switching losses from the device models
        named by each switch's ``thermal_model`` parameter (or ``models``,
        ``{"Q1": "<model>"}``), averaged over ``[t_start, t_start + duration]``
        (default: the last fifth of the run) at junction temperature ``tj``."""
        t0, t_end = float(self.time[0]), float(self.time[-1])
        if duration is None:
            duration = (t_end - t0) / 5.0 if t_start is None else t_end - float(t_start)
        if t_start is None:
            t_start = t_end - duration
        if duration <= 0:
            raise ValueError("duration must be > 0")
        design = self._design_json
        if models:
            import json

            d = json.loads(design)
            ids = {c["id"]: c for c in d["components"]}
            available = set(loss_models())
            for cid, model in models.items():
                if cid not in ids:
                    raise KeyError(f"no component {cid!r}")
                m = model if model in available or model.endswith(".json") else model + ".json"
                if m not in available:
                    raise KeyError(f"unknown loss model {model!r} ({len(available)} available; see simpel.loss_models())")
                ids[cid].setdefault("parameters", {})["thermal_model"] = m
            design = json.dumps(d)
        wanted = {k: v for k, v in self._signals.items() if k.startswith(("V_", "I_")) or "." in k or "_gate" in k}
        raw = _core.losses(design, self.time, wanted, float(t_start), float(duration), float(tj))
        return LossReport(float(t_start), float(duration), float(tj), tuple(LossResult(**r) for r in raw))

    # ── exports ─────────────────────────────────────────────────────────────
    def _check_names(self, names: Iterable[str] | None) -> list[str]:
        cols = list(names) if names is not None else []
        for n in cols:
            self[n]  # raises KeyError with suggestions
        return cols

    def export_csv(self, path: str | os.PathLike, names: Iterable[str] | None = None, *, precision: int = 12) -> None:
        """Write ``time`` and the signals (default: all, sorted) as CSV with
        ``precision`` significant digits."""
        cols = self._check_names(names)
        _core.export_csv(os.fspath(path), self.time, self._signals, cols, precision)

    def svg(self, names: Iterable[str] | None = None, *, title: str = "", t1: float | None = None,
            t2: float | None = None, panes: int | None = None, dark: bool = False) -> str:
        """Waveform plot as SVG text, in the GUI's scope style. One pane per signal
        unless ``panes`` is given. Default signals: the first four."""
        cols = self._check_names(names) or self.names[:4]
        n = len(cols) if panes is None else int(panes)
        a = -1.0 if t1 is None else float(t1)
        b = -1.0 if t2 is None else float(t2)
        return _core.scope_svg(self.time, {k: self._signals[k] for k in cols}, cols, cols, title or "Waveforms",
                               n, dark, a, b)

    def export_svg(self, path: str | os.PathLike, names: Iterable[str] | None = None, **kwargs) -> None:
        """Write :meth:`svg` to a file."""
        with open(os.fspath(path), "w", encoding="utf-8", newline="\n") as f:
            f.write(self.svg(names, **kwargs))

    def schematic_svg(self, *, dark: bool = False) -> str:
        """The simulated circuit's schematic as SVG text."""
        return _core.schematic_svg(self._design_json, dark)

    def export_html(self, path: str | os.PathLike, plots: Iterable[Iterable[str]] | None = None, *,
                    pdf: bool = False) -> None:
        """Write the GUI's HTML report: schematic, waveform plots, design and
        netlist. ``plots`` is a list of signal groups, one plot each; by default
        the circuit's oscilloscope blocks (or probes) decide. ``pdf`` additionally
        prints it to PDF with a locally installed Edge or Chrome (Windows)."""
        groups = [self._check_names(g) for g in plots] if plots is not None else []
        if plots is not None and not any(groups):
            raise ValueError("plots must contain at least one signal")
        _core.export_html(os.fspath(path), self._design_json, self.netlist, self.time, self._signals,
                          [g for g in groups if g], pdf)

    def plot(self, names: Iterable[str] | str | None = None, *, t1: float | None = None, t2: float | None = None,
             ax=None, max_points: int = 4000, separate: bool = False, **line_kwargs):
        """Plot signals with matplotlib. Long traces are min-max decimated to
        ``max_points`` per signal, which keeps switching spikes visible.
        ``separate`` gives each signal its own axes. Returns the axes (a list
        when ``separate``)."""
        try:
            import matplotlib.pyplot as plt
        except ImportError:  # pragma: no cover
            raise ImportError("plotting needs matplotlib: pip install matplotlib") from None
        cols = [names] if isinstance(names, str) else (self._check_names(names) or self.names[:4])
        a, b = self._window(t1, t2)
        lo = int(np.searchsorted(self.time, a, side="left"))
        hi = int(np.searchsorted(self.time, b, side="right"))
        t = self.time[lo:hi]
        if separate:
            if ax is not None:
                raise ValueError("ax cannot be combined with separate=True")
            fig, axes = plt.subplots(len(cols), 1, sharex=True, squeeze=False)
            axes = list(axes[:, 0])
        else:
            axes = [ax if ax is not None else plt.subplots()[1]] * len(cols)
        for name, axis in zip(cols, axes):
            y = self[name][lo:hi]
            td, yd = _core.decimate(t, y, max_points) if len(t) > max_points else (t, y)
            axis.plot(td, yd, label=name, **line_kwargs)
            axis.grid(True, alpha=0.3)
            if separate:
                axis.set_ylabel(name)
        axes[-1].set_xlabel("time [s]")
        if not separate:
            axes[0].legend()
        return axes if separate else axes[0]

    # ── results files ───────────────────────────────────────────────────────
    def save_for_gui(self, path: str | os.PathLike) -> None:
        """Write a ``.simres`` file: the design, the netlist, the settings and every
        waveform, bit for bit. The GUI opens it with File > Open Results (or
        ``circuitsim_pro_win.exe --results <file>``) without re-simulating;
        :func:`simpel.load_results` reads it back."""
        import json

        meta = {"wall_seconds": self.wall_seconds, "cancelled": self.cancelled, "solver": self.solver,
                "step": self.step, "t_stop": self.t_stop, "dt": self.dt, "source": "python"}
        settings = {k: v for k, v in self.settings.items() if isinstance(v, (int, float, str, bool))}
        _core.write_results(os.fspath(path), self._design_json, self.netlist, json.dumps(settings), json.dumps(meta),
                            self.time, [(k, self._signals[k]) for k in sorted(self._signals)])

    def __repr__(self) -> str:
        state = " (cancelled)" if self.cancelled else ""
        t_end = float(self.time[-1]) if len(self.time) else 0.0
        return (f"<Results{state}: {len(self.time)} samples to t={t_end:.6g} s, {len(self._signals)} signals, "
                f"{self.solver}/{self.step}, {self.wall_seconds:.3f} s>")


class Simulation:
    """A transient simulation of a circuit.

    >>> sim = Simulation(c, t_stop=5e-3, dt=1e-7, step="adaptive")
    >>> res = sim.run()

    Settings not given are taken from the circuit (a loaded design keeps its own)
    and otherwise default to the GUI's.
    """

    _KEYS = ("t_stop", "dt", "solver", "step", "rel_tol", "abs_tol_v", "abs_tol_i", "h_min", "h_max",
             "lu_cache", "diode_model")

    def __init__(self, circuit: Union[Circuit, str, os.PathLike], *, t_stop: float | None = None,
                 dt: float | None = None, step: str | None = None, solver: str | None = None,
                 rel_tol: float | None = None, abs_tol_v: float | None = None, abs_tol_i: float | None = None,
                 h_min: float | None = None, h_max: float | None = None, lu_cache: bool | None = None,
                 diode_model: str | None = None):
        self.circuit = circuit if isinstance(circuit, Circuit) else Circuit.load(circuit)
        given = dict(t_stop=t_stop, dt=dt, step=step, solver=solver, rel_tol=rel_tol, abs_tol_v=abs_tol_v,
                     abs_tol_i=abs_tol_i, h_min=h_min, h_max=h_max, lu_cache=lu_cache, diode_model=diode_model)
        if step is not None and step not in ("fixed", "adaptive", "variable"):
            raise ValueError("step must be 'fixed' or 'adaptive'")
        self.settings: dict = {**self.circuit.settings, **{k: v for k, v in given.items() if v is not None}}
        if self.settings.get("step") == "variable":
            self.settings["step"] = "adaptive"
        self._cancel = _core.CancelToken()

    def cancel(self) -> None:
        """Stop a running :meth:`run` (callable from another thread). The run
        returns the samples produced so far, with ``cancelled`` set."""
        self._cancel.set()

    def run(self, progress: ProgressFn | bool | None = None, *, check: bool = True) -> Results:
        """Simulate. ``progress(t, t_stop)`` is called about 30 times a second
        (return ``False`` to stop early); ``progress=True`` prints a progress line
        to stderr. With ``check`` validation errors raise and warnings are
        issued as :class:`simpel.ValidationWarning`. Ctrl+C interrupts the run."""
        if check:
            for issue in self.circuit.validate(raise_on_error=True):
                warnings.warn(issue.message, ValidationWarning, stacklevel=2)
        cb = _text_progress() if progress is True else (progress or None)
        self._cancel.clear()
        try:
            raw = _core.simulate(self.circuit._c, {k: self.settings[k] for k in self._KEYS if k in self.settings},
                                 cb, self._cancel)
        finally:
            if progress is True:
                sys.stderr.write("\n")
        return Results(raw, self.settings)

    def __repr__(self) -> str:
        s = self.settings
        return f"<Simulation of {self.circuit.name!r}: t_stop={s.get('t_stop')}, dt={s.get('dt')}, {s.get('step')}>"


def _text_progress() -> ProgressFn:
    start = _time.perf_counter()

    def cb(t: float, t_stop: float) -> None:
        frac = 0.0 if t_stop <= 0 else min(1.0, t / t_stop)
        sys.stderr.write(f"\r  {100 * frac:5.1f} %  t={t:.4g} s  {(_time.perf_counter() - start):.1f} s elapsed")
        sys.stderr.flush()

    return cb
