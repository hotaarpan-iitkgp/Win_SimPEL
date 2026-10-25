"""The public API is a contract (docs/api-stability.md): this test fails when a
public name or signature changes. After an intended change, regenerate the
snapshot with SIMPEL_UPDATE_API=1 and record the change in the changelog."""

import dataclasses
import inspect
import os
import pathlib

import simpel as sp

SNAPSHOT = pathlib.Path(__file__).with_name("api_surface.txt")


def _sig(obj) -> str:
    try:
        return str(inspect.signature(obj))
    except (TypeError, ValueError):
        return "(...)"


def surface() -> list[str]:
    lines = []
    for name in sorted(sp.__all__):
        obj = getattr(sp, name)
        if inspect.isclass(obj):
            if dataclasses.is_dataclass(obj):
                fields = ", ".join(f"{f.name}: {f.type}" for f in dataclasses.fields(obj))
                lines.append(f"class {name}({fields})")
            else:
                lines.append(f"class {name}{_sig(obj.__init__) if '__init__' in obj.__dict__ else ''}")
            for attr, val in sorted(vars(obj).items()):
                if attr.startswith("_") and attr not in ("__getitem__", "__len__", "__iter__", "__contains__"):
                    continue
                if isinstance(val, property):
                    lines.append(f"  {name}.{attr} [property]")
                elif isinstance(val, (staticmethod, classmethod)):
                    lines.append(f"  {name}.{attr}{_sig(val.__func__)} [{type(val).__name__}]")
                elif inspect.isfunction(val):
                    lines.append(f"  {name}.{attr}{_sig(val)}")
        elif callable(obj):
            lines.append(f"def {name}{_sig(obj)}")
        else:
            lines.append(f"{name} = {type(obj).__name__}")
    return lines


def test_public_api_is_unchanged():
    got = surface()
    if os.environ.get("SIMPEL_UPDATE_API"):
        SNAPSHOT.write_text("\n".join(got) + "\n", encoding="utf-8")
    want = SNAPSHOT.read_text(encoding="utf-8").splitlines()
    removed = [l for l in want if l not in got]
    added = [l for l in got if l not in want]
    assert not removed and not added, (
        "public API changed (see docs/api-stability.md)\n  removed:\n    " + "\n    ".join(removed)
        + "\n  added:\n    " + "\n    ".join(added))
