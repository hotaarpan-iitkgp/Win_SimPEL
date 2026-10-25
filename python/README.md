# simpel

Python API for the SimPEL circuit simulator. The compiled core is the same C++
engine the desktop application uses, so a design simulates identically here and in
the GUI.

Build and install into the active environment (Windows, MinGW-w64 toolchain):

```powershell
powershell -File build_mingw.ps1 -Python path\to\venv\Scripts\python.exe
```

## Building a circuit

```python
import simpel as sp

c = sp.Circuit("buck")
v1  = c.add("V", "V1", value=48)
q1  = c.add("MOSFET", "Q1", Ron=10e-3, thermal_model="C2M0025120D.json")
d1  = c.add("D", "D1")
l1  = c.add("L", "L1", L="100u")
c1  = c.add("C", "C1", C=100e-6)
r1  = c.add("R", "R1", value=5)
tri = c.add("TRI_GEN", "TRI1", frequency="20k")
k   = c.add("CONST", "K1", value=0.5)
cmp = c.add("COMP", "CMP1")

c.connect(v1.A, q1.D)
c.connect(q1.S, d1.B, l1.A)          # any number of pins on one net
c.connect(l1.B, c1.A, r1.A)
c.ground(v1.B, d1.A, c1.B, r1.B)
c.connect(tri.Out, cmp.In1)
c.connect(k.Out, cmp.In2)
c.connect(cmp.Out, q1.G)

for issue in c.validate():
    print(issue)
```

- Block types, parameters and pins: `sp.blocks()`, `sp.block_info("MOSFET")`.
- Pins are attributes (case-insensitive) or strings: `q1.D`, `"Q1.D"`.
- A parameter the solver never reads (e.g. a typo, `Rn=` for `Ron=`) raises a
  `sp.ParameterWarning` with a suggestion.
- `c.save("buck.json")` writes a file the GUI opens; `sp.Circuit.load(path)` opens
  GUI designs.

## Simulating

```python
sim = sp.Simulation(c, t_stop=5e-3, dt=1e-7, step="adaptive")
res = sim.run(progress=True)              # or progress=lambda t, t_stop: ...
res["V_C1"], res.time                     # NumPy arrays, no copy
df = res.to_pandas(["V_C1", "I_L1"])
```

`progress(t, t_stop)` returning `False` stops the run; `sim.cancel()` does the same
from another thread; Ctrl+C interrupts. A stopped run returns what it computed so
far with `res.cancelled == True`.

## Analyses

```python
res.stats("V_C1", t1=4e-3).mean           # mean, rms, min, max, pk2pk
res.fourier("V_R1", f0=50).thd            # exact for non-uniform steps
res.losses(t_start=4e-3, duration=1e-3, tj=125).total    # needs device models
```

Loss models are looked up in `SIMPEL_LOSS_MODELS_DIR` (`;`-separated) and the GUI's
search locations, or set explicitly with `sp.set_loss_model_dir(path)`;
`sp.loss_models()` lists them.

## Exports and plots

```python
res.export_csv("buck.csv", ["V_C1", "I_L1"])         # default: all signals, 12 digits
res.export_svg("vout.svg", ["V_C1"], t1=4e-3)        # GUI scope style
res.export_html("report.html", plots=[["V_C1", "I_L1"], ["CMP1.Out"]])
res.schematic_svg()
res.plot(["V_C1", "I_L1"], separate=True)            # matplotlib, min-max decimated
```

## Parameter sweeps

```python
sw = sp.sweep(c, {"L1.L": ["50u", "100u", "200u"], "R1.value": [2.5, 5, 10]},
              t_stop=5e-3, dt=1e-7, workers=8, signals=["V_C1"])
df = sw.table(vout=lambda r: r.stats("V_C1", t1=4e-3).mean)
```

Every combination (`mode="grid"`) or element-wise pairs (`mode="zip"`), or a list of
dicts. Points run in parallel on a C++ thread pool; results are bitwise identical to
running each point on its own. A swept parameter the solver never reads is an error.

## Saving for the GUI

`c.save("buck.json")` lays the circuit out automatically when it was built in Python
(power stage left to right from the sources, control below, signals left to right)
and routes every wire around the blocks on the 20 px grid (A*, a port of the web
tool's router); a loaded design keeps its drawing. Pass `layout="auto"`,
`"place"` (no routing) or `"keep"` to choose. Layout changes the drawing only,
never the simulated circuit. In the GUI a routed wire keeps its route until one of
its blocks moves; clicking a selected routed wire turns it back into an ordinary,
draggable wire.
