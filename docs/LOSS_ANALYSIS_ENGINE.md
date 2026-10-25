# Thermal & Loss Analysis Engine Architecture & Implementation Guide

## 1. Executive Summary & Design Philosophy

The **Loss Analysis Engine** in CircuitSim Pro (Win_SimPEL) is a high-performance, native C++ analytical engine designed to calculate semiconductor switching and conduction losses with PLECS-grade accuracy.

### Core Philosophy: Decoupled Simulation & Event-Driven Analysis
In traditional circuit simulators (e.g. SPICE), attempting to capture thermal switching losses through physical semiconductor models (e.g., charge-carrier physics, parasitic capacitances) incurs severe penalties:
- Extremely small numerical time steps ($dt \sim 10^{-11}\text{ s}$ to $10^{-9}\text{ s}$) are required to resolve sub-nanosecond $V$-$I$ overlapping intervals.
- Severe numerical convergence failures occur during sharp switching transitions.
- Simulation times increase from seconds to hours.

**Win_SimPEL adopts the industry-standard PLECS approach:**
1. **Idealized / Piecewise-Linear Electrical Simulation:** The circuit solver runs with fast, robust, and stable macro-switch models ($R_{\text{on}}$, $R_{\text{off}}$) with standard simulation time steps ($dt \sim 10^{-7}\text{ s}$ to $10^{-6}\text{ s}$).
2. **Post-Processing Analytical Event Extraction:** Upon completing the simulation (or over a user-selected steady-state analysis window $[t_{\text{start}}, t_{\text{start}} + \Delta T]$), the **Loss Analysis Engine** scans high-speed telemetry waveforms, detects switching transition events (turn-on, turn-off, reverse recovery), and queries multi-dimensional Look-Up Tables (LUTs) constructed from manufacturer datasheets.
3. **Continuous Conduction Loss Integration:** Conduction losses are computed at each time step using temperature-dependent $V_{\text{drop}}(I, T_j)$ forward characteristics, taking into account gate state (channel conduction vs. body diode reverse conduction).

---

## 2. Architecture & Component Interaction

The loss analysis subsystem consists of three primary modules:

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                              MainWindow (UI)                                │
│   [+ Loss Analysis] Toolbar  │  Property Inspector  │  Thermal Model Viewer │
└───────────────────────┬─────────────────────────────────────────────────────┘
                        │ triggers analysis over [t_start, t_start + duration]
                        ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                    CircuitSimEngine::LossAnalysisEngine                     │
│                                                                             │
│  1. Slices Telemetry [t_start, t_end] (v, i, gate)                          │
│  2. Traces Gate Signals dynamically across netlist                          │
│  3. Detects Turn-On (Hard vs ZVS) and Turn-Off Events                       │
│  4. Queries 3D LUTs for E_on(V, I, Tj) and E_off(V, I, Tj)                  │
│  5. Numerically integrates conduction loss: ∫ V_drop(i, Tj) * i * dt        │
└───────────────────────┬─────────────────────────────────────────────────────┘
                        │ queries component models
                        ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                    CircuitSimEngine::LossModelLibrary                       │
│   - Singleton catalog manager                                               │
│   - Background asynchronous directory scanner (std::thread)                 │
│   - Thread-safe storage (std::mutex libraryMutex)                           │
│   - Auto-resolves models from: loss_models/, data/thermal_models/           │
└───────────────────────┬─────────────────────────────────────────────────────┘
                        │ parses and evaluates
                        ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                        CircuitSimEngine::LossModel                          │
│   - Interpolator2D: Conduction Drop V_drop(I, Tj) [Bilinear]                │
│   - Interpolator3D: Switching Energies E_on/E_off(V, I, Tj) [Trilinear]     │
│   - Extrapolation clamps, 1-point axis safety, negative-axis handling       │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## 3. Multi-Dimensional LUT Interpolator (`LossModel.hpp` / `.cpp`)

The interpolators evaluate non-linear semiconductor characteristics across operating currents, blocking voltages, and junction temperatures.

### 3.1 1D Weight Calculation with Linear Extrapolation (`getWeights`)
For an input $x$ against an axis vector $[a_0, a_1, \dots, a_{N-1}]$:
- If $x \le a_0$: indices $(0, 1)$ are chosen, fraction $f = \frac{x - a_0}{a_1 - a_0}$ (permitting linear extrapolation below range).
- If $x \ge a_{N-1}$: indices $(N-2, N-1)$ are chosen, fraction $f = \frac{x - a_{N-2}}{a_{N-1} - a_{N-2}}$ (permitting linear extrapolation above range).
- If $a_0 < x < a_{N-1}$: binary search (`std::upper_bound`) finds the bounding interval $[a_k, a_{k+1}]$ in $O(\log N)$, and $f = \frac{x - a_k}{a_{k+1} - a_k}$.
- **Edge-Case Guard:** If an axis contains only 1 element ($N=1$), index is clamped to $0$ and fraction to $0.0$, preventing vector out-of-bounds crashes.

### 3.2 2D Bilinear Interpolator (`Interpolator2D`)
Used for conduction forward voltage drop $V_{\text{drop}} = f(I, T_j)$:
$$\begin{aligned}
c_0 &= V(i_0, t_0) + f_i \cdot \left[V(i_1, t_0) - V(i_0, t_0)\right] \\
c_1 &= V(i_0, t_1) + f_i \cdot \left[V(i_1, t_1) - V(i_0, t_1)\right] \\
V_{\text{drop}}(I, T_j) &= c_0 + f_t \cdot (c_1 - c_0)
\end{aligned}$$

### 3.3 3D Trilinear Interpolator (`Interpolator3D`)
Used for turn-on energy $E_{\text{on}} = f(V, I, T_j)$ and turn-off energy $E_{\text{off}} = f(V, I, T_j)$:
1. Interpolate across voltage dimension ($V$):
   $$\begin{aligned}
   c_{00} &= \text{LUT}(v_0, i_0, t_0) \cdot (1 - f_v) + \text{LUT}(v_1, i_0, t_0) \cdot f_v \\
   c_{01} &= \text{LUT}(v_0, i_0, t_1) \cdot (1 - f_v) + \text{LUT}(v_1, i_0, t_1) \cdot f_v \\
   c_{10} &= \text{LUT}(v_0, i_1, t_0) \cdot (1 - f_v) + \text{LUT}(v_1, i_1, t_0) \cdot f_v \\
   c_{11} &= \text{LUT}(v_0, i_1, t_1) \cdot (1 - f_v) + \text{LUT}(v_1, i_1, t_1) \cdot f_v
   \end{aligned}$$
2. Interpolate across current dimension ($I$):
   $$\begin{aligned}
   c_0 &= c_{00} + f_i \cdot (c_{10} - c_{00}) \\
   c_1 &= c_{01} + f_i \cdot (c_{11} - c_{01})
   \end{aligned}$$
3. Interpolate across temperature dimension ($T_j$):
   $$E = c_0 + f_t \cdot (c_1 - c_0)$$

### 3.4 Negative-Axis Convention Handling
Certain thermal models (such as Wolfspeed/Cree SiC MOSFET body diodes) specify third-quadrant conduction curves using negative currents ($I_{\text{SD}} < 0$).
`LossModel::getConductionVoltageDrop` first evaluates with positive $|I|$. If the result is near zero and the table's current axis is negative ($i_{\text{axis}}[0] < 0$), it automatically evaluates with $-|I|$ and returns the absolute voltage drop $|V_{\text{drop}}|$, ensuring universal compatibility with both first- and third-quadrant conventions.

---

## 4. Library Management & Asynchronous Parallel Loading (`LossModelLibrary`)

To prevent application startup delay when parsing dozens of large thermal model files, `LossModelLibrary` utilizes an **asynchronous worker thread**:

- **Immediate Startup:** In `MainWindow::MainWindow()`, `loadAllModelsAsync()` is called instead of blocking synchronous I/O. The desktop application window opens in milliseconds.
- **Lock-Free File I/O:** Reading files and deserializing 3D JSON arrays occur in parallel worker threads **outside the mutex**.
- **Fine-Grained Mutex Synchronization:** The internal catalog (`std::map<std::string, std::shared_ptr<LossModel>> models`, `model_names`, `loaded_dirs`) is guarded by `std::mutex libraryMutex`. Lock duration is strictly limited to microseconds during pointer insertion.
- **Incremental Discovery:** As each valid thermal model is parsed, it is registered so dropdown menus in the Property Inspector update progressively.
- **Background Rescan:** The UI's "Rescan Library" button calls `refreshAsync()`, running directory re-scans in the background without freezing the GUI.

---

## 5. Event-Driven Loss Calculation Algorithm (`LossAnalysisEngine`)

When a user requests a loss analysis via the `LOSS_ANALYZER` block or Property Inspector:

### 5.1 Telemetry Slicing
The engine extracts the evaluation time window:
$$t \in [t_{\text{start}}, t_{\text{start}} + \text{duration}]$$
Using `std::lower_bound`, indices `idx_start` and `idx_end` are determined. The voltage $V_{\text{comp}}(t)$, current $I_{\text{comp}}(t)$, and gate signal $G_{\text{comp}}(t)$ are extracted as sliced contiguous vectors.

### 5.2 Dynamic Gate Signal Resolution (`findGateSignal`)
The engine dynamically discovers the gate control signal through a multi-stage search:
1. Direct component parameters (`"control_signal"`, `"Gate_Signal_Label"`, `"Control"`).
2. Wire netlist tracing: checks all wires connected to terminals `"G"`, `"Ctrl"`, `"Gate"`, or `"In"`.
3. Standard node name heuristics (`"<ID>.G"`, `"<ID>_gate"`).

### 5.3 Gate-Controlled Switching Analysis (MOSFETs, IGBTs, etc.)
Using a gate threshold $V_{\text{th}} = 0.5\text{ V}$:

#### A. Turn-On Event Detection ($G_{k-1} \le V_{\text{th}}$ and $G_k > V_{\text{th}}$)
1. **Pre-switching Voltage:** Extract blocking voltage $V_{\text{sw}} = |v_k|$.
2. **Inrush Current Peak:** Look ahead across a 20-sample window ($[k, k + 20]$) to find peak channel current:
   $$I_{\text{peak}} = \max_{m \in [k, k+20]} |i_m|$$
3. **Hard vs. Soft Switching (ZVS) Classification:**
   - **Hard Turn-On:** If $V_{\text{sw}} > 5.0\text{ V}$ and $I_{\text{peak}} > 0.05\text{ A}$, energy $E_{\text{on}} = \text{LUT}_{3D}(V_{\text{sw}}, I_{\text{peak}}, T_j)$ is added, and `num_hard_turn_on` increments.
   - **Zero-Voltage Switching (ZVS Soft Turn-On):** If $V_{\text{sw}} \le 5.0\text{ V}$, the device turned on under zero/low voltage. Energy is looked up at $V_{\text{sw}}$, and `num_soft_turn_on` increments.

#### B. Turn-Off Event Detection ($G_{k-1} > V_{\text{th}}$ and $G_k \le V_{\text{th}}$)
1. **Conduction Current:** Look back 5 samples ($[k - 5, k]$) to find current prior to channel interruption:
   $$I_{\text{sw}} = \max_{m \in [k-5, k]} |i_m|$$
2. **Blocking Voltage Peak:** Look ahead 20 samples ($[k, k + 20]$) to capture the transient drain-source/collector-emitter voltage:
   $$V_{\text{block}} = \max_{m \in [k, k+20]} |v_m|$$
3. **Energy Lookup:** If $I_{\text{sw}} > 0.05\text{ A}$, energy $E_{\text{off}} = \text{LUT}_{3D}(V_{\text{block}}, I_{\text{sw}}, T_j)$ is added, and `num_turn_off` increments.

#### C. Continuous Conduction Loss Integration
For each time step $\Delta t_k = t_{k+1} - t_k$:
- If $G_k > V_{\text{th}}$ (Gate ON): Channel is conducting. Voltage drop $V_{\text{drop}} = \text{LUT}_{\text{cond\_on}}(|i_k|, T_j)$.
- If $G_k \le V_{\text{th}}$ (Gate OFF): Channel is off, but reverse current may conduct through the antiparallel/body diode if $|v_k| < 5.0\text{ V}$. Voltage drop $V_{\text{drop}} = \text{LUT}_{\text{cond\_off}}(|i_k|, T_j)$.
- Energy contribution:
  $$E_{\text{cond}} += V_{\text{drop}} \cdot |i_k| \cdot \Delta t_k$$

### 5.4 Passive Diode / Reverse Recovery Heuristics (Devices without Gate)
For passive diodes (or switches without an active gate):
1. **Turn-Off (Reverse Recovery) Detection:**
   - Detects negative voltage transition when diode transitions from forward bias to reverse blocking ($V_{\text{drop}} \to -V_{\text{block}}$).
   - Looks back 10 samples to extract forward current $I_F$.
   - Looks ahead 10 samples to extract reverse blocking voltage $V_R$.
   - Adds reverse recovery energy: $E_{\text{sw\_off}} += \text{LUT}_{\text{turn\_off}}(V_R, I_F, T_j)$.
2. **Diode Conduction Loss:**
   - Detected when $|i_k| > 0.05\text{ A}$ and forward voltage $|v_k| < 5.0\text{ V}$.
   - Evaluates conduction drop from diode LUT $\text{LUT}_{\text{cond}}(|i_k|, T_j)$ and integrates $V_{\text{drop}} \cdot |i_k| \cdot \Delta t_k$.

### 5.5 Average Power Output
$$\begin{aligned}
P_{\text{cond}} &= \frac{E_{\text{cond}}}{T_{\text{eval}}} \\
P_{\text{sw\_on}} &= \frac{E_{\text{sw\_on}}}{T_{\text{eval}}} \\
P_{\text{sw\_off}} &= \frac{E_{\text{sw\_off}}}{T_{\text{eval}}} \\
P_{\text{total}} &= P_{\text{cond}} + P_{\text{sw\_on}} + P_{\text{sw\_off}}
\end{aligned}$$

---

## 6. Schematic & UI Integration

### 6.1 The `LOSS_ANALYZER` Schematic Block
- **Component Identifier:** `LOSS_ANALYZER`
- **Parameters:**
  - `start_time`: Time to begin averaging (default `0.0` s). Setting this to steady-state time (e.g., `0.015` s) discards startup inrush transients.
  - `duration`: Time window length (default `0.02` s, matching a 50 Hz fundamental period).
  - `tj`: Global semiconductor junction temperature in °C (default `125.0` °C).
- **Execution:** Clicking **"Show Loss Report"** in the block's property inspector runs the engine and displays the tabular report.

### 6.2 Direct Toolbar Button (`+ Loss Analysis`)
- Positioned on the top control bar next to **Component Pane**.
- One-click adds a `LOSS_ANALYZER` block centered in the current viewport, snapped to the 20px grid, and automatically selects it.

### 6.3 Batch Thermal Model Assignment
- When multiple MOSFETs/IGBTs/Diodes are selected simultaneously, the Property Inspector exposes an aggregated **Thermal & Loss Model (Batch)** dropdown.
- If components have divergent models, it displays `--- (Mixed / Multiple Models) ---`.
- Selecting a model applies it across all selected switches simultaneously.

### 6.4 Tabbed Thermal Model Inspector
Available via **Tools -> Loss Thermal Model Inspector...** or by clicking **"Inspect Loss Model"** on any switch:
- **Tab 1 (Metadata):** Part number, device type (SiC MOSFET, IGBT, etc.), gate charge $Q_g$, driver voltage $V_{\text{drv}}$, leakage current $I_{\text{leakage}}$.
- **Tab 2 (Turn-On Loss $E_{\text{on}}$):** 3D table displaying $E_{\text{on}}$ across $V$, $I$, and $T_j$.
- **Tab 3 (Turn-Off Loss $E_{\text{off}}$):** 3D table displaying $E_{\text{off}}$ across $V$, $I$, and $T_j$.
- **Tab 4 (Conduction Loss):** 2D tables displaying $V_{\text{drop}}$ versus $I_D$ at multiple temperatures for both Gate-ON (channel) and Gate-OFF (body diode).

---

## 7. Thermal Model JSON Schema Specification

Thermal models are stored as `.json` files in `loss_models/` or `data/thermal_models/`:

```json
{
  "metadata": {
    "part_number": "C2M0025120D",
    "manufacturer": "Wolfspeed",
    "type": "SiC_MOSFET",
    "package": "TO-247-3",
    "V_ds_max_V": 1200.0,
    "I_d_continuous_A": 90.0,
    "R_ds_on_typical_mOhm": 25.0,
    "Qg_nC": 161.0,
    "V_drv_V": 20.0,
    "I_leakage_uA": 1.0
  },
  "turn_on_loss": {
    "v_axis": [400.0, 600.0, 800.0],
    "i_axis": [10.0, 30.0, 60.0, 90.0],
    "t_axis": [25.0, 125.0, 150.0],
    "data": [
      [
        [0.00015, 0.00018, 0.00019],
        [0.00045, 0.00052, 0.00055],
        [0.00095, 0.00110, 0.00118],
        [0.00150, 0.00175, 0.00185]
      ],
      [
        [0.00025, 0.00030, 0.00032],
        [0.00075, 0.00088, 0.00092],
        [0.00160, 0.00185, 0.00198],
        [0.00255, 0.00295, 0.00315]
      ],
      [
        [0.00038, 0.00045, 0.00048],
        [0.00110, 0.00130, 0.00138],
        [0.00235, 0.00275, 0.00292],
        [0.00375, 0.00435, 0.00465]
      ]
    ]
  },
  "turn_off_loss": {
    "v_axis": [400.0, 600.0, 800.0],
    "i_axis": [10.0, 30.0, 60.0, 90.0],
    "t_axis": [25.0, 125.0, 150.0],
    "data": [
      [
        [0.00008, 0.00009, 0.00010],
        [0.00025, 0.00028, 0.00030],
        [0.00055, 0.00062, 0.00066],
        [0.00090, 0.00102, 0.00108]
      ],
      [
        [0.00014, 0.00016, 0.00017],
        [0.00042, 0.00048, 0.00051],
        [0.00092, 0.00105, 0.00112],
        [0.00150, 0.00172, 0.00182]
      ],
      [
        [0.00020, 0.00023, 0.00025],
        [0.00062, 0.00071, 0.00075],
        [0.00138, 0.00158, 0.00168],
        [0.00225, 0.00258, 0.00274]
      ]
    ]
  },
  "conduction_loss_on": {
    "i_axis": [0.0, 10.0, 25.0, 50.0, 75.0, 100.0],
    "t_axis": [25.0, 125.0, 150.0],
    "data": [
      [0.00, 0.00, 0.00],
      [0.25, 0.38, 0.42],
      [0.63, 0.98, 1.10],
      [1.30, 2.10, 2.38],
      [2.05, 3.35, 3.82],
      [2.90, 4.80, 5.50]
    ]
  },
  "conduction_loss_off": {
    "i_axis": [0.0, 10.0, 25.0, 50.0, 75.0, 100.0],
    "t_axis": [25.0, 125.0, 150.0],
    "data": [
      [0.00, 0.00, 0.00],
      [2.80, 2.45, 2.35],
      [3.30, 2.95, 2.85],
      [3.95, 3.65, 3.55],
      [4.60, 4.35, 4.25],
      [5.25, 5.05, 4.95]
    ]
  }
}
```
