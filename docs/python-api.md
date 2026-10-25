# Python API reference

Install, quick start and examples: `python/README.md`. Stability rules:
`docs/api-stability.md`. Every function and class has a docstring
(`help(simpel.Simulation)`).

## Blocks

| | |
|---|---|
| `blocks(category=None, *, role=None, include_legacy=False)` | registered block type names |
| `categories()` | `["control", "electrical", "general", "magnetic"]` |
| `block_info(type)` | `BlockInfo`: name, category, role, aliases, `params` (`ParamInfo`: default, unit, choices), `pins` (`PinInfo`: name, kind) |

`role` is `"model"` (simulated), `"structural"`, `"observer"` or `"unsupported"`
(placed but not simulated). Defining new blocks: `docs/adding-a-block.md`.

## Circuits

| | |
|---|---|
| `Circuit(name)` / `Circuit.load(path)` | new circuit / open a schematic `.json` |
| `c.add(type, id=None, **params) -> Component` | parameters as numbers or strings such as `"10u"`; warns (`ParameterWarning`) for a parameter the solver never reads |
| `c.connect(*pins)`, `c.ground(*pins)` | pins as `comp.PinName` (case-insensitive) or `"Comp.Pin"` |
| `c[id]`, `id in c`, `len(c)`, iteration | components |
| `comp["Ron"] = 5e-3`, `comp.params`, `comp.pins` | edit / inspect |
| `c.validate(raise_on_error=False) -> list[Issue]` | `multiple_drivers` (error), `unconnected_pin`, `unconnected_input`, `unsupported_block`, `ineffective_param`, `no_ground`, `empty_circuit` |
| `c.save(path, layout=None)`, `c.to_json(layout=None)` | `layout`: `"auto"` (place + route; default for built circuits), `"place"`, `"keep"` (default for loaded designs) |
| `c.settings` | settings stored with the circuit |

## Simulation

`Simulation(circuit_or_path, *, t_stop, dt, step="fixed"|"adaptive", solver,
rel_tol, abs_tol_v, abs_tol_i, h_min, h_max, lu_cache, diode_model)`; unspecified
settings come from the circuit, then the GUI defaults.

`sim.run(progress=None, *, check=True) -> Results`: `progress(t, t_stop)` about 30
times a second (return `False` to stop) or `True` for a text bar; `check` raises on
validation errors and warns (`ValidationWarning`) otherwise. `sim.cancel()` stops a
run from another thread; Ctrl+C interrupts. A stopped run returns its samples so far
with `res.cancelled` set.

## Results

| | |
|---|---|
| `res.time`, `res[name]`, `res.names`, `res.signals` | NumPy arrays (shared, not copied) |
| `res.to_pandas(names)`, `res.to_dict(names)` | tables |
| `res.stats(name, t1, t2) -> SignalStats` | mean, rms, min, max, abs_max, pk2pk; exact for non-uniform steps |
| `res.fourier(name, f0=None, *, cycles=1, t1, t2, max_harmonic=50) -> FourierResult` | thd, fundamental, dc, `harmonic(n)`, orders / magnitudes / phases |
| `res.losses(t_start, duration, *, tj=125, models=None) -> LossReport` | per device conduction / switching losses (`thermal_model` parameter or `models={"Q1": ...}`) |
| `res.export_csv(path, names, precision=12)` | CSV |
| `res.svg(...)`, `res.export_svg(path, names, ...)`, `res.schematic_svg()` | GUI-style SVG |
| `res.export_html(path, plots=None, *, pdf=False)` | the GUI's report |
| `res.plot(names, *, t1, t2, ax, max_points=4000, separate=False)` | matplotlib with min-max decimation |
| `res.save_for_gui(path)`, `load_results(path)` | `.simres`, bitwise |

`loss_models()` lists device models; `set_loss_model_dir(*dirs)` replaces the
search (default: the models shipped in the package, or `SIMPEL_LOSS_MODELS_DIR`).

## Sweeps

`sweep(circuit, params, *, settings=None, workers=None, mode="grid"|"zip",
signals=None, progress=None, **sim_kwargs) -> SweepResults`: `params` is
`{"Comp.param": [values]}` or a list of dicts. Points run in parallel in C++;
results are bitwise identical to separate runs. `SweepResults` is a sequence of
`SweepPoint` (`params`, `results`, `error`, `ok`) with `failed` and
`table(**metrics)` (DataFrame).

## Desktop application

`open_in_gui(results_or_circuit_or_path, *, gui=None, wait=False)` starts
`circuitsim_pro_win.exe` (from `gui=`, `SIMPEL_GUI` or `PATH`) with
`--results <file.simres>` or `--open <file.json>`. `find_gui()` reports what it
would use.

## Command line

`simpel_cli run <design.json> [-t s] [-dt s] [--step-type fixed|variable]
[--solver name] [-p Comp.param=value]... [--netlist f] [--csv f] [--signals a,b]
[--svg f] [--html f] [--schematic f] [--simres f]`, `simpel_cli netlist`,
`simpel_cli signals`, `simpel_cli info <file.simres>`, `simpel_cli version`.
