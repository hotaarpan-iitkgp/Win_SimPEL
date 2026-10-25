"""Parallel parameter sweeps."""

from __future__ import annotations

import itertools
from dataclasses import dataclass
from typing import Any, Callable, Iterable, Iterator, Mapping, Sequence

from . import _core
from .circuit import Circuit, CircuitError, _fmt
from .simulation import Results, Simulation

__all__ = ["sweep", "SweepResults", "SweepPoint"]


@dataclass(frozen=True)
class SweepPoint:
    index: int
    params: dict  # {"Q1.Ron": value, ...} as given
    results: Results | None
    error: str | None

    @property
    def ok(self) -> bool:
        return self.results is not None


class SweepResults(Sequence[SweepPoint]):
    """Outcome of :func:`sweep`: one :class:`SweepPoint` per parameter set, in
    order. Failed points have ``results is None`` and an ``error``."""

    def __init__(self, points: list[SweepPoint], names: list[str], wall_seconds: float):
        self._points = points
        self.param_names = names
        self.wall_seconds = wall_seconds

    def __getitem__(self, i):  # type: ignore[override]
        return self._points[i]

    def __len__(self) -> int:
        return len(self._points)

    def __iter__(self) -> Iterator[SweepPoint]:
        return iter(self._points)

    @property
    def failed(self) -> list[SweepPoint]:
        return [p for p in self._points if not p.ok]

    def table(self, **metrics: Callable[[Results], Any]):
        """DataFrame with one row per point: the swept parameters plus one column
        per metric, e.g. ``table(vout=lambda r: r.stats("V_C1", t1=4e-3).mean)``.
        Failed points get NaN metrics and their error in column ``error``."""
        from .simulation import _pandas

        pd = _pandas()
        rows = []
        for p in self._points:
            row = dict(p.params)
            for k, fn in metrics.items():
                row[k] = fn(p.results) if p.ok else float("nan")
            row["error"] = p.error
            rows.append(row)
        df = pd.DataFrame(rows)
        if not df["error"].notna().any():
            df = df.drop(columns="error")
        return df

    def __repr__(self) -> str:
        bad = len(self.failed)
        return f"<SweepResults: {len(self)} points ({bad} failed), {self.wall_seconds:.2f} s>"


def sweep(circuit: Circuit | str, params: Mapping[str, Iterable[Any]] | Iterable[Mapping[str, Any]], *,
          settings: Mapping[str, Any] | None = None, workers: int | None = None, mode: str = "grid",
          signals: Iterable[str] | None = None, progress: Callable[[int, int], object] | bool | None = None,
          **sim_kwargs) -> SweepResults:
    """Simulate ``circuit`` for many parameter sets in parallel.

    ``params`` maps ``"Comp.param"`` to a list of values: ``mode="grid"`` runs
    every combination, ``mode="zip"`` pairs the lists element by element. Or
    pass a list of dicts, one per point. ``settings`` (e.g. ``sim.settings``)
    and keyword arguments such as ``t_stop`` are the simulation settings, as for
    :class:`Simulation`. ``signals`` keeps only those waveforms (saves memory on
    big sweeps). ``workers`` defaults to the number of CPU threads.
    ``progress(done, total)`` returning ``False`` stops the sweep; points not
    started are reported as failed with error ``"cancelled"``.

    Results are bitwise identical to running each point on its own.
    """
    import time

    c = circuit if isinstance(circuit, Circuit) else Circuit.load(circuit)
    sim = Simulation(c, **{**dict(settings or {}), **sim_kwargs})

    if isinstance(params, Mapping):
        names = list(params)
        lists = [list(v) if not isinstance(v, (str, bytes)) else [v] for v in params.values()]
        if mode == "grid":
            combos = list(itertools.product(*lists))
        elif mode == "zip":
            if len({len(x) for x in lists}) > 1:
                raise ValueError("mode='zip' needs equally long value lists")
            combos = list(zip(*lists))
        else:
            raise ValueError("mode must be 'grid' or 'zip'")
        point_dicts = [dict(zip(names, combo)) for combo in combos]
    else:
        point_dicts = [dict(p) for p in params]
        names = list(dict.fromkeys(k for p in point_dicts for k in p))
    if not point_dicts:
        raise ValueError("no sweep points")

    for name in names:
        comp, dot, pname = name.partition(".")
        if not dot or not pname:
            raise ValueError(f"sweep parameter {name!r} must be 'Component.param'")
        if comp not in c:
            raise CircuitError(f"sweep parameter {name!r}: no component {comp!r}")
        if not c._c.param_affects_netlist(comp, pname):
            from .blocks import block_info

            info = block_info(c[comp].type)
            raise CircuitError(f"sweep parameter {name!r} has no effect on {info.type}; "
                               f"known parameters: {[p.name for p in info.params]}")

    keep = list(signals) if signals is not None else []
    cb = _text_progress() if progress is True else (progress or None)
    raw_points = [[(k, _fmt(v)) for k, v in p.items()] for p in point_dicts]
    t0 = time.perf_counter()
    try:
        raw = _core.sweep(c._c, {k: sim.settings[k] for k in Simulation._KEYS if k in sim.settings}, raw_points,
                          workers or 0, keep, cb, _core.CancelToken())
    finally:
        if progress is True:
            import sys

            sys.stderr.write("\n")
    wall = time.perf_counter() - t0
    out = [SweepPoint(i, point_dicts[i], Results(r, sim.settings) if r is not None else None, err)
           for i, (r, err) in enumerate(raw)]
    return SweepResults(out, names, wall)


def _text_progress() -> Callable[[int, int], None]:
    import sys

    def cb(done: int, total: int) -> None:
        sys.stderr.write(f"\r  {done}/{total} points")
        sys.stderr.flush()

    return cb
