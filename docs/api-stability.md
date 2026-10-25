# API and file-format stability

SimPEL follows semantic versioning (`MAJOR.MINOR.PATCH`, currently 0.1.0). The core
version (`simpel.core_version()`, `src/engine/Version.hpp`) and the Python package
version (`simpel.__version__`, `python/pyproject.toml`) are always equal; a test
checks it.

Until 1.0, a MINOR release may break the contract below, but only with a changelog
entry and, where feasible, one release of `DeprecationWarning` first. From 1.0,
breaking changes need a MAJOR release.

## What is covered

| Surface | Contract | Guarded by |
|---|---|---|
| Python names in `simpel.__all__` and their signatures | stable | `python/tests/test_api_surface.py` against `api_surface.txt` |
| Schematic `.json` written by `Circuit.save`, the GUI and `simpel_cli` | readable by every later version; `simulationSettings` keys stable | `tests/test_designio.cpp`, golden round trip |
| `.simres` format version 1 | readable by every later version; a new layout gets a new version number, which older readers reject with a clear message | `tests/test_resultsfile.cpp` |
| Block registry files, `"schema": 1` | stable; block `type` names and their `aliases` are never removed, only deprecated | `tests/registry` |
| `simpel_cli` commands and options | stable | `cli_*` CTest cases |
| Numerical results | bitwise reproducible for the same build, design and settings, including parallel sweeps; may change between versions only with a changelog entry | `tests/golden` (63 designs), `tests/REFERENCE.txt` |

Adding a function, a keyword argument with a default, a block, a parameter, a
signal or a field in a file header is not a breaking change.

## What is not covered

- `simpel._core` and anything starting with `_`.
- Constructing `Results`, `Component`, `SweepResults` or `SweepPoint` directly:
  obtain them from `Simulation.run`, `Circuit.add`/`Circuit[...]` and `sweep`.
- The C++ headers under `src/` (the core library is linked statically by the
  GUI, the CLI and the extension; it is not a public C++ SDK yet).
- Text of error and warning messages, signal ordering in exported files beyond
  "sorted by name", schematic coordinates produced by `layout="auto"`.
- Results across different compilers or CPU architectures: floating-point
  rounding may differ in the last bits.

## Changing the public API

1. Make the change; run `SIMPEL_UPDATE_API=1 pytest tests/test_api_surface.py`
   from `python/` to refresh the snapshot.
2. Review the snapshot diff: every removed or changed line is a break.
3. Note it in the changelog with the version it ships in.

Changes to numerical results go through `golden_runner --update` and
`test_emission --update`; the diff of the expectation files is the record of what
changed.
