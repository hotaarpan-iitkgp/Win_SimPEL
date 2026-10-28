# Block Verification Plan (SimPEL Python API, C++ engine)

Scope: `c++ version/` only. Every block is exercised through `simpel` (`sp.Circuit`,
`sp.Simulation`, `res[...]`). Bugs are fixed in `c++ version/src/engine/*`, then the
Python build is refreshed with `python/build_mingw.ps1`.

## Inventory (Phase 0, done)

196 blocks from `sp.blocks()`, see `inventory.tsv`:

| role        | count | meaning                                              |
|-------------|------:|------------------------------------------------------|
| model       | 167   | simulated; must pass every phase                      |
| unsupported | 18    | placed but not simulated; must be reported, not crash |
| structural  | 8     | GND, ports, subsystem; validated only                 |
| observer    | 3     | PROBE, SCOPE, LOSS_ANALYZER; placed in a circuit      |

## Phase 1 - Smoke (every block)

For each block, a test circuit is generated automatically:
- control/general blocks: one SINE_WAVE source per input pin (distinct frequency and phase).
- electrical/magnetic blocks: a 10 V source on the first power pin, a 1 kOhm load on the
  second, 1 MOhm to ground on any other power pin, a CONST 1.0 on each control pin.
- unsupported blocks: must produce an `unsupported_block` validation issue.
- structural/observer blocks: placed and validated only.

Pass criteria, model blocks:
1. circuit builds and validates with no error-severity issue
2. simulation completes (no exception, no hang past the per-block timeout)
3. every recorded signal is finite
4. at least one signal varies (`ptp > 1e-9`)

## Phase 2 - Parameter sensitivity

For each block, the block is simulated with engine defaults (baseline) and again with each
parameter changed:
- numeric parameters: value x 2 (or 1.0 when the default is 0)
- enumerated parameters: every listed choice
- structural/string/list parameters (pin counts, operators, code, sign strings) are
  reported as `SKIPPED`; they are covered in Phase 3.

A parameter is `EFFECT` when some recorded signal of the block changes, and `NO_EFFECT`
otherwise. Every `NO_EFFECT` is a finding: either the parameter is ignored by the engine
(a bug, the class found earlier with `fc`, `Kp`, `fn`) or the harness cannot observe it,
which is then recorded with a note.

## Phase 3 - Functional verification (analytic answers)

Pytest modules, one per category, each case a small circuit with a known answer:
- control math (gain, sum/product signs, divide, abs/sign/round/min_max, trig, math)
- control dynamics (integrator, derivative, transfer function, state space, filters,
  moving average, delay, memory, rate limiter, saturation, dead zone, relay, quantizer)
- logic and switching (gates, comparators, flip-flops, edge detect, monostable, signal
  switches, multiport switch, goto/from)
- sources and measurement (sine, ramp, step, pulse, PWM, triangle, RMS, THD, Fourier)
- electrical (R/L/C transients, diode, switches, transformer ratio, mutual inductance)
- magnetic (MMF, permeance, reluctance)

## Phase 4 - Interconnected circuits

Blocks composed into the topologies they are normally used in, with parameters changed
mid-run: RC step, RL with PWM switch, buck with PI + PWM, PLL locked to a 3-phase source,
transformer + bridge rectifier, bus creator/selector routing, state machine with edge
detection.

## Phase 5 - Fix loop

For each failure: reproduce in a minimal circuit, find the root cause in `src/engine`,
fix it there (no Python-side workaround), rebuild, rerun the failing phase, add a
regression test in `python/tests/test_block_regressions.py`.
Stop condition: all phases pass, or a failure is recorded with the reason it is out of
scope.

## Running

    cd "c++ version/python/tests/blocks"
    ..\..\..\.venv\Scripts\python.exe driver.py          # phases 1 and 2, all blocks
    ..\..\..\.venv\Scripts\python.exe driver.py V R      # only blocks starting with these

Results: `results/<BLOCK>.json` per block and `results/summary.json`.

## Status checkpoint

Phase 0: done. Inventory of 196 blocks in `inventory.tsv`.

Phase 1 (current build, full run): PASS 159, FAIL 8, EXPECTED_UNSUPPORTED 18, PLACED 11.
Open FAILs: LUT_2D, LUT_3D, DLL, FMU, FOURIER_SERIES, GOTO_SIG, FROM_SIG, INPORT.
All report "control output does not vary with its inputs". Root cause not yet triaged. Some
may be harness limits (GOTO/FROM need a matching tag, DLL/FMU need an external file, INPORT
needs a subsystem context) rather than engine bugs.

Phase 2 (current build): 0 ERROR, 33 NO_EFFECT (down from 43 after the MATH_FCN fix).
Remaining NO_EFFECT groups: Roff/Ron on switch models at default load (likely not observable
in the harness circuit), THYRISTOR gate/holding params, PWM_MASTER frequency/min/max,
VAR_R/VAR_L, TRI amplitude, OPAMP/E_COMP gain, DISCRETE_PID ts, choice params on
BITWISE_OP, COMPARE_TO_CONSTANT, RELATIONAL_OPERATOR.

Fix loop (Phase 5): rounds 1-5 applied fixes for MOV_AVG, EDGE_DETECT, MONOSTABLE, PWM,
PWM_3PH, D_FLIP_FLOP, MULTIPORT_SWITCH, FOURIER_ANALYSIS, PERIODIC_IMP_AVG, THD_VAL,
MATH_FCN. Regression suite: 21 passed, 1 xfailed (PWM_MASTER, still open).

Phase 3 (functional, analytic): not started.
Phase 4 (interconnected circuits): not started.

## Round 7 update (supersedes the Phase 1 counts above)

Engine fix: `parseM` in `src/engine/CircuitSimulator.cpp` (LUT_2D) now accepts the nested
`[[a, b], [c, d]]` table form. The default table was being read as one flattened row, so
LUT_2D output was always zero. Regression test added: `test_lut_2d_parses_nested_table_default`.

Harness fix: GOTO_SIG, FROM_SIG and INPORT are now routing-only (`harness.ROUTING_ONLY`) and
are placed-and-validated in Phase 1. GOTO/FROM was verified separately: FROM feeding a GAIN
reproduces the GOTO input (ptp 2 for a unit sine).

Build note: `python/build_mingw.ps1` installs into whatever `python` is on PATH unless
`-Python` is given. Pass the venv interpreter explicitly:
`powershell -ExecutionPolicy Bypass -File python\build_mingw.ps1 -Python ..\..\..\.venv\Scripts\python.exe`
(run from `c++ version`). Round 6 installed into the global Python by mistake.

Phase 1 (round 7, full run): PASS 160, FAIL 4, PLACED 14, EXPECTED_UNSUPPORTED 18.
Open FAILs: LUT_3D, DLL, FMU, FOURIER_SERIES. The engine has no evaluation branch for any of
them (the type is parsed in NetlistParser.cpp and then the output stays at zero). They are
unimplemented, not mis-wired. Decision needed: implement, or mark `simulated: false` so they
report `unsupported_block`.

Open item (not fixed): LUT_2D has one input pin. Its netlist sets the second axis to the
literal `"0.0"` (control.json, `input2`), so the second axis is always zero. Decision needed:
add a second input pin, or document the single-input behaviour.

Phase 2 (round 7): 0 ERROR, 33 NO_EFFECT, unchanged.
Regression suite: 22 passed, 1 xfailed (PWM_MASTER).

## Round 11 update (supersedes earlier Phase 1 and 2 counts)

Phase 1 (full run): PASS 162, FAIL 2 (DLL, FMU, out of scope by decision), PLACED 14,
EXPECTED_UNSUPPORTED 18.

Fixed this round:
- LUT_2D: second input pin In2 is real (Terminals.cpp, control.json). Table parsing accepts
  nested rows. Regression: second axis used, bilinear result 1.0.
- LUT_3D: implemented. Three inputs In, In2, In3; trilinear over axes x, y, z and a nested cube
  `table`. Regression: linear table gives exact result 1.5.
- FOURIER_SERIES: implemented as a truncated Fourier series of the last fundamental cycle,
  params f, harmonics, ts. Output evaluated at the current time (not held). Regression: a
  fundamental plus third harmonic with one harmonic kept gives the fundamental back.
- Operator defaults now match the JSON: RELATIONAL_OPERATOR "<", COMPARE_TO_CONSTANT ">=",
  BITWISE_OP "XOR". Before, the engine fallback was "==" or "AND" and they disagreed with the
  declared defaults.
- TRI: min, max and amplitude declare "" (unset). Amplitude A gives -A..+A; explicit min or max
  overrides; nothing set gives 0..1. The registry used to fill min=0, max=1 on every TRI, so
  amplitude could never take effect.
- DISCRETE_PID: ts > 0 makes the controller sampled (state updates per ts, output held). The
  netlist emits ts=0.01 for DISCRETE_PID by default.

Phase 2 (full run): 0 ERROR, 31 NO_EFFECT.
Remaining NO_EFFECT, by cause:
- Roff / Ron on switch models (IGCT, MOSFET, vg-FET, BJT, JFET, S, BREAKER, DBL_SWITCH,
  MAN_SWITCH, MAN_DBL_SWITCH, MAN_TRPL_SWITCH, SR_SWITCH): the harness drives the control high,
  so the switch is always on and the off or on resistance is not seen. Harness limit. Fix:
  add an off-state variant to the harness.
- OPAMP gain, E_COMP gain: the output is saturated or set by the feedback network, so open-loop
  gain is not observable in this circuit. Harness limit.
- Not counted above, but still open: PWM_MASTER (xfail, P1 not populated), and the VAR_R /
  VAR_L value parameters, which are ignored by design when a control signal is wired (the
  nominal value is only the fallback).

Regression suite: 30 passed, 1 xfailed (PWM_MASTER).

## Round 16 update: discrete blocks and Phase 3 started

Discrete-time blocks: all six are model blocks now. UNIT_DELAY, ZOH, DISCRETE_INT,
DISCRETE_MEAN, DISCRETE_TF and DISCRETE_SS sample every ts and hold between samples. Each has a
regression test. DISCRETE_INT supports forward Euler only; the method string is not read.

Thyristor: traced to the harness, not the engine. The default 1 kOhm load at 10 V carries
about 9 mA, below the 10 mA holding current, so the device correctly turns off. Harness now uses
600 Ohm load and a 0.75 V gate. All five thyristor parameters show EFFECT.

Operator choices: RELATIONAL_OPERATOR, BITWISE_OP and COMPARE_TO_CONSTANT are tested under
several constant input sets. All choices show EFFECT.

Phase 3 (analytic functional tests), started. Modules:
- test_functional_control.py: static block formulas, integrator, transfer function
- test_functional_logic.py: logic truth tables, NOT, relational and compare operators
- test_functional_sources.py: sine, ramp, step, pulse, triangle, constant, RMS
- test_functional_electrical.py: divider, RC, RL, forward and reverse diode, closed switch
Status: 51 passed. Not yet written: magnetic (MMF, permeance, reluctance), and the
interconnected circuits of Phase 4.

## Round 17 update: Phase 3 magnetic, Phase 4 started

Magnetic (test_functional_magnetic.py, 7 tests): fixed a netlist defect. AIR_GAP, LINEAR_CORE,
MAG_PERMEANCE and LEAKAGE_PATH were emitted as capacitors, which carry no DC flux. They are now
resistors of value 1/P, so flux = P * MMF. Open item: F0 (MMF offset, permanent magnet) is still
ignored by these four blocks. It needs a series MMF source in the netlist.

Phase 4 (test_interconnected.py, 4 tests): PWM switch into a resistor at 25, 50 and 75 % duty
gives D * V. Buck converter in continuous conduction (L = 500 uH, above the 125 uH critical value)
gives D * Vin - (1 - D) * Vd. Remaining Phase 4 cases: RL load with PWM switch and freewheel
diode, PI-controlled buck, transformer with bridge rectifier, bus creator/selector routing,
state machine with edge detection, PLL locked to a 3-phase source.

Note on the buck test: at L = 100 uH the converter is in discontinuous conduction, so the output
is not D * Vin. The test was corrected to use a continuous-conduction inductor.

## Round 18 update: Phase 4 complete

test_interconnected.py now has 10 cases, all passing (117 block tests in total). Covered topologies:
PWM switch into a resistor (3 duties), RL load on a PWM switch with freewheel diode, buck in
continuous conduction, closed-loop PID buck regulating to 6 V, PLL locked to a 50 Hz three-phase
source, ideal transformer into a diode bridge, bus creator and selector routing, and a toggle
state machine (D flip-flop with Q_bar fed back). The RC step is in test_functional_electrical.py.

Engine defects found and fixed this round:
- BUS_CREATOR / BUS_SELECTOR: a bus carried only its first input. The netlist now wires each
  selector output to the matching creator input (unity gain).
- D_FLIP_FLOP: Q_bar was never registered when the netlist named the output, so feedback from
  Q_bar read 0 and the toggle never switched.

Open item: the four permeance blocks still ignore F0 (MMF offset).

## Round 19: IC macromodels removed from the tool

Removed IC_555, IC_7400, IC_7404, IC_7408, IC_7432, IC_LM317, IC_LM7805 and IC_PC817 from the
block set. They were unsupported placeholders with no solver model. Removed from:
data/blocks/electrical.json, tests/registry/emission_snapshot.txt, and the saved driver results.
The block inventory is regenerated: 188 blocks, not 196.

Phase 1 now: PASS 168, FAIL 2 (DLL, FMU), PLACED 14, EXPECTED_UNSUPPORTED 4 (DISPLAY_VAL,
INTERNAL_VAR, PI_SECTION, LINE_3PH). Phase 2: 0 errors, 3 no-effect (VAR_R, VAR_L by design, and
DISCRETE_MEAN initial_value, which only affects the output before the first sample).
Block tests: 119 passed.

The driver logs from earlier rounds still mention the IC blocks. They are history, so I left them.
