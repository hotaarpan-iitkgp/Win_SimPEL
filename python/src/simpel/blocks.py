"""Block registry access: what blocks exist, their parameters and pins."""

from __future__ import annotations

from dataclasses import dataclass, field

from . import _core

__all__ = ["BlockInfo", "ParamInfo", "PinInfo", "blocks", "block_info", "categories"]

#: Block roles, see data/blocks schema:
#:   "model"       has a solver model and is simulated
#:   "structural"  shapes connectivity only (ground, ports, labels, subsystems)
#:   "observer"    reads results (scopes, probes, loss analyser)
#:   "unsupported" accepted in designs but not simulated yet
ROLES = ("model", "structural", "observer", "unsupported")


@dataclass(frozen=True)
class ParamInfo:
    name: str
    default: str
    unit: str = ""
    description: str = ""
    choices: tuple[str, ...] = ()  # allowed values; empty = free-form


@dataclass(frozen=True)
class PinInfo:
    name: str
    kind: str  # "power" (electrical / magnetic) or "control"
    x: float = 0.0
    y: float = 0.0


@dataclass(frozen=True)
class BlockInfo:
    type: str
    name: str
    category: str
    subcategory: str
    description: str = ""
    role: str = "model"
    legacy: bool = False
    aliases: tuple[str, ...] = field(default_factory=tuple)
    params: tuple[ParamInfo, ...] = field(default_factory=tuple)
    pins: tuple[PinInfo, ...] = field(default_factory=tuple)
    in_palette: bool = False
    #: Allowed values per lower-case parameter name. May include alternative
    #: spellings used by older designs that are not in ``params``.
    choices: dict = field(default_factory=dict, compare=False, hash=False)

    @property
    def simulated(self) -> bool:
        return self.role == "model"

    def param(self, name: str) -> ParamInfo:
        for p in self.params:
            if p.name == name:
                return p
        raise KeyError(f"{self.type} has no parameter {name!r}; known: {[p.name for p in self.params]}")

    def pin(self, name: str) -> PinInfo:
        for p in self.pins:
            if p.name == name:
                return p
        raise KeyError(f"{self.type} has no pin {name!r}; known: {[p.name for p in self.pins]}")

    def __str__(self) -> str:
        head = f"{self.type}: {self.name}  [{self.category} / {self.subcategory}]"
        if self.role != "model":
            head += f"  ({self.role})"
        if self.legacy:
            head += "  (legacy name)"
        lines = [head]
        if self.params:
            lines.append("  params: " + ", ".join(f"{p.name}={p.default}" for p in self.params))
        if self.pins:
            lines.append("  pins:   " + ", ".join(f"{p.name}({p.kind[0]})" for p in self.pins))
        return "\n".join(lines)


def blocks(category: str | None = None, *, role: str | None = None, include_legacy: bool = False) -> list[str]:
    """Registered block type names.

    ``category``: ``"general"``, ``"control"``, ``"electrical"`` or ``"magnetic"``.
    ``role``: one of :data:`ROLES`, e.g. ``"model"`` for simulated blocks only.
    Legacy type names (kept so old designs load) are excluded unless
    ``include_legacy`` is true.
    """
    if category is not None and category not in categories():
        raise ValueError(f"unknown category {category!r}; expected one of {categories()}")
    if role is not None and role not in ROLES:
        raise ValueError(f"unknown role {role!r}; expected one of {list(ROLES)}")
    return [
        b["type"]
        for b in _core.block_types()
        if (category is None or b["category"] == category)
        and (role is None or b["role"] == role)
        and (include_legacy or not b["legacy"])
    ]


def categories() -> list[str]:
    seen: list[str] = []
    for b in _core.block_types():
        if b["category"] not in seen:
            seen.append(b["category"])
    return seen


def block_info(type: str) -> BlockInfo:  # noqa: A002 - mirrors the JSON field name
    """Full description of one block type (an alias resolves to its block).
    Raises ``KeyError`` with suggestions for unknown types."""
    d = _core.block_info(type)
    return BlockInfo(
        type=d["type"],
        name=d["name"],
        category=d["category"],
        subcategory=d["subcategory"],
        description=d["description"],
        role=d["role"],
        legacy=d["legacy"],
        aliases=tuple(d["aliases"]),
        params=tuple(
            ParamInfo(p["name"], p["default"], p["unit"], p["description"],
                      tuple(d["choices"].get(p["name"].lower(), ())))
            for p in d["params"]
        ),
        pins=tuple(PinInfo(p["name"], p["kind"], p["x"], p["y"]) for p in d["pins"]),
        in_palette=bool(d["palette"]),
        choices={k: tuple(v) for k, v in d["choices"].items()},
    )
