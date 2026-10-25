"""Building circuits programmatically."""

from __future__ import annotations

import difflib
import os
import warnings
from dataclasses import dataclass
from typing import Iterator, Union

from . import _core
from .blocks import block_info

__all__ = ["Circuit", "Component", "Pin", "Issue", "CircuitError", "ValidationWarning", "ParameterWarning"]

CircuitError = _core.CircuitError  # ValueError subclass raised for invalid construction


class ValidationWarning(UserWarning):
    """A validation finding that does not stop a simulation."""


class ParameterWarning(UserWarning):
    """A parameter that the solver never reads (usually a misspelt name)."""


def _fmt(v: object) -> str:
    if isinstance(v, bool):
        return "true" if v else "false"
    if isinstance(v, float):
        return repr(v)
    if isinstance(v, (int, str)):
        return str(v)
    raise TypeError(f"parameter values must be numbers, strings or booleans, not {type(v).__name__}")


@dataclass(frozen=True)
class Pin:
    """A pin of a component, e.g. ``q1.D``. Obtained from a :class:`Component`."""

    component: str
    name: str
    kind: str = "power"  # "power" or "control"

    def __str__(self) -> str:
        return f"{self.component}.{self.name}"

    def __repr__(self) -> str:
        return f"Pin({self.component}.{self.name})"


@dataclass(frozen=True)
class Issue:
    severity: str  # "error" or "warning"
    code: str
    message: str
    component: str = ""
    pin: str = ""

    def __str__(self) -> str:
        return f"{self.severity}: {self.message}"


PinLike = Union[Pin, str]


class Component:
    """Handle to a block in a :class:`Circuit`.

    Pins are attributes (``q1.D``, case-insensitive) or ``q1.pin("D")``;
    parameters are items (``q1["Ron"] = 5e-3``).
    """

    __slots__ = ("_circuit", "_id")

    def __init__(self, circuit: "Circuit", id: str):  # noqa: A002
        object.__setattr__(self, "_circuit", circuit)
        object.__setattr__(self, "_id", id)

    @property
    def id(self) -> str:
        return self._id

    @property
    def type(self) -> str:
        return self._circuit._c.component(self._id)["type"]

    @property
    def params(self) -> dict[str, str]:
        """Current parameter values (a copy)."""
        return dict(self._circuit._c.component(self._id)["params"])

    @property
    def pins(self) -> list[Pin]:
        return [Pin(self._id, p["name"], p["kind"]) for p in self._circuit._c.pins(self._id)]

    def pin(self, name: str) -> Pin:
        try:
            canon = self._circuit._c.resolve_pin(self._id, name)
        except ValueError as e:
            raise AttributeError(str(e)) from None
        for p in self.pins:
            if p.name == canon:
                return p
        raise AttributeError(name)  # pragma: no cover

    def __getattr__(self, name: str) -> Pin:
        if name.startswith("_"):
            raise AttributeError(name)
        return self.pin(name)

    def __setattr__(self, name: str, value: object) -> None:
        raise AttributeError("set parameters with component[name] = value")

    def __getitem__(self, name: str) -> str:
        params = self.params
        if name not in params:
            raise KeyError(f"{self._id} has no parameter {name!r}; set: {sorted(params)}")
        return params[name]

    def __setitem__(self, name: str, value: object) -> None:
        self._circuit._c.set_param(self._id, name, _fmt(value))
        self._circuit._warn_ineffective(self._id, [name])

    def __dir__(self) -> list[str]:
        return sorted(set(super().__dir__()) | {p.name for p in self.pins})

    def __repr__(self) -> str:
        return f"<{self.type} {self._id}>"


class Circuit:
    """A circuit described by connectivity.

    >>> c = Circuit("buck")
    >>> vin = c.add("V", "V1", value=48)
    >>> q1 = c.add("MOSFET", "Q1", Ron=10e-3)
    >>> c.connect(vin.A, q1.D)
    >>> c.ground(vin.B)
    """

    def __init__(self, name: str = "circuit", *, _core_circuit: "_core.Circuit | None" = None):
        self._c = _core_circuit if _core_circuit is not None else _core.Circuit(name)
        # Loaded designs keep their drawing; built ones are laid out when saved.
        self._loaded = _core_circuit is not None

    # ── construction ────────────────────────────────────────────────────────
    @classmethod
    def load(cls, path: str | os.PathLike, name: str | None = None) -> "Circuit":
        """Open a schematic ``.json`` saved by the GUI (or by :meth:`save`)."""
        p = os.fspath(path)
        if not os.path.isfile(p):
            raise FileNotFoundError(p)
        stem = os.path.splitext(os.path.basename(p))[0]
        return cls(_core_circuit=_core.Circuit.from_design_file(p, name or stem))

    @property
    def name(self) -> str:
        return self._c.name

    def add(self, type: str, id: str | None = None, **params: object) -> Component:  # noqa: A002
        """Add a block. ``type`` is a registry type (see :func:`simpel.blocks`);
        ``id`` defaults to the next free ``<prefix><n>``; keyword arguments set
        parameters (numbers or engineering strings such as ``"10u"``)."""
        cid = self._c.add(type, id or "", {k: _fmt(v) for k, v in params.items()})
        self._warn_ineffective(cid, list(params))
        return Component(self, cid)

    def remove(self, component: Component | str) -> None:
        self._c.remove(component.id if isinstance(component, Component) else component)

    def connect(self, *pins: PinLike) -> None:
        """Put all given pins on one net. Pins are ``Pin`` objects or ``"Q1.D"``.
        For control signals give the driving output first or anywhere: the
        connection is made from the net's single output to each input."""
        if len(pins) < 2:
            raise CircuitError("connect needs at least two pins")
        self._c.connect([self._pin_tuple(p) for p in pins])

    def ground(self, *pins: PinLike) -> None:
        """Connect pins to ground (a GND block is added on first use)."""
        if not pins:
            raise CircuitError("ground needs at least one pin")
        self._c.ground([self._pin_tuple(p) for p in pins])

    # ── inspection ──────────────────────────────────────────────────────────
    def __getitem__(self, id: str) -> Component:  # noqa: A002
        if not self._c.has(id):
            raise KeyError(f"no component {id!r}; components: {self._c.ids()}")
        return Component(self, id)

    def __contains__(self, id: object) -> bool:  # noqa: A002
        return isinstance(id, str) and self._c.has(id)

    def __iter__(self) -> Iterator[Component]:
        return (Component(self, i) for i in self._c.ids())

    def __len__(self) -> int:
        return len(self._c.ids())

    @property
    def components(self) -> list[Component]:
        return list(self)

    def validate(self, *, raise_on_error: bool = False) -> list[Issue]:
        """Check the circuit. Returns errors and warnings; with
        ``raise_on_error`` an error raises :class:`CircuitError`."""
        issues = [Issue(d["severity"], d["code"], d["message"], d["component"], d["pin"]) for d in self._c.validate()]
        if raise_on_error:
            errors = [i for i in issues if i.severity == "error"]
            if errors:
                raise CircuitError("circuit has errors:\n  " + "\n  ".join(i.message for i in errors))
        return issues

    @property
    def settings(self) -> dict:
        """Simulation settings stored with the circuit (from a loaded file, or the
        defaults). :class:`simpel.Simulation` arguments override them."""
        return self._c.settings()

    def save(self, path: str | os.PathLike, layout: str | None = None) -> None:
        """Save as a schematic ``.json`` the GUI can open.

        ``layout="auto"`` places the blocks (power stage left to right from the
        sources, control below it, signals flowing left to right) and routes the
        wires around them; ``"place"`` places without routing; ``"keep"`` keeps
        the current drawing. The default is ``"auto"`` for circuits built in
        Python and ``"keep"`` for loaded designs. Layout changes the drawing
        only, never the simulated circuit."""
        self._c.save(os.fspath(path), self._layout(layout))

    def to_json(self, layout: str | None = None) -> str:
        """The schematic ``.json`` text (see :meth:`save` for ``layout``)."""
        return self._c.design_json(self._layout(layout))

    def _layout(self, layout: str | None) -> str:
        if layout is None:
            return "keep" if self._loaded else "auto"
        if layout not in ("auto", "place", "keep"):
            raise ValueError("layout must be 'auto', 'place' or 'keep'")
        return layout

    def __repr__(self) -> str:
        return f"<Circuit {self.name!r}: {len(self)} components>"

    # ── internals ───────────────────────────────────────────────────────────
    def _pin_tuple(self, p: PinLike) -> tuple[str, str]:
        if isinstance(p, Pin):
            return (p.component, p.name)
        if isinstance(p, str) and "." in p:
            comp, _, pin = p.partition(".")
            return (comp, pin)
        raise TypeError(f"expected a Pin or 'Component.Pin' string, got {p!r}")

    def _warn_ineffective(self, cid: str, names: list[str]) -> None:
        if not names:
            return
        bad = [n for n in self._c.ineffective_params(cid) if n in names]
        if not bad:
            return
        info = block_info(self._c.component(cid)["type"])
        known = [p.name for p in info.params]
        for n in bad:
            msg = f"{cid}: parameter {n!r} has no effect on {info.type}"
            close = difflib.get_close_matches(n, known, n=1, cutoff=0.5)
            if not close:
                close = [k for k in known if k.lower() == n.lower()]
            msg += f"; did you mean {close[0]!r}?" if close else f"; known parameters: {known}"
            warnings.warn(msg, ParameterWarning, stacklevel=3)
