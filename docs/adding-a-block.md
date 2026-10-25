# Adding or changing a block

Blocks are described in `data/blocks/<category>.json` (`general`, `control`,
`electrical`, `magnetic`). That file is the single source of truth for the block's
name, category, parameters and defaults, palette button, inspector dropdowns, type
mapping and, for most simple blocks, its netlist entry. The JSON is compiled into
the core, so rebuild after editing it. The schema is documented at the top of
`src/engine/BlockRegistry.hpp`.

To try changes without rebuilding, point `SIMPEL_BLOCKS_DIR` at a directory of
block JSON files; the GUI, `simpel_cli` and the Python module then load those
instead of the embedded copy.

## 1. A variant of an existing model (JSON only)

If the new block behaves like a block the solver already implements (an offset, a
gain, a filter with different defaults, a renamed source), no C++ is needed:

```json
{
  "type": "BIAS_ADD", "name": "Bias Adder", "subcategory": "Math", "ui_type": "Unknown",
  "params": [{"name": "bias", "default": "2.5", "unit": "", "description": "Added to the input"}],
  "pins": [{"name": "In",  "x": -30, "y": 0, "dx": -1, "dy": 0, "kind": "control"},
           {"name": "Out", "x":  30, "y": 0, "dx":  1, "dy": 0, "kind": "control"}],
  "netlist": {"section": "control_loops.gains",
              "fields": {"output": "{id}.Out", "original_type": "OFFSET",
                         "offset": {"param": "bias", "default": 0.0},
                         "input": {"signal": "In"}, "input1": {"signal": "In"},
                         "input2": "0.0", "K": 1.0}},
  "palette": [{"order": 1000, "text": "Bias Adder (BIAS_ADD)", "label": "Bias", "prefix": "BIAS",
               "ui_type": "Unknown", "basic": false, "params": [["bias", "2.5"]]}]
}
```

- `pins`: position relative to the block centre at rotation 0, outward direction,
  and `kind` (`control` for signals, `power` for electrical/magnetic terminals). Or
  `"pins_from": "OFFSET"` to reuse another block's layout.
- `netlist.fields`: `"text"` (with `{id}` / `{type}` substituted), a literal number,
  `{"signal": "<pin>"}`, `[{"signal": ...}, ...]`, `{"param": "<name>", "default": ...,
  "as": "number|int|string"}` and `{"nodes": true}` for electrical blocks. `id` is
  always emitted. `original_type` selects the solver model.
- `palette.order` is the button's global position and must be unique.
- `choices` gives inspector dropdowns: `{"mode": ["a", "b"]}` (lower-case keys).

`tests/registry/test_custom_block.cpp` is this example, run end to end.

## 2. A block with new behaviour

Do step 1, then:

1. **Solver model.** Add the behaviour in `src/engine/CircuitSimulator.cpp`
   (control blocks: `evaluateControls`; electrical blocks: the MNA stamps) under a
   new `CircuitSimEngine::ComponentType` in `CircuitSimulator.hpp`.
2. **Netlist type name.** Map your `original_type` string to that enum in
   `stringToComponentType` in `src/engine/NetlistParser.cpp`. The parser copies every
   netlist field into the model's `parameters`, and `inputs` arrays into
   `input_0`, `In1`, ... so most blocks need no further parser code.
3. **Netlist entry.** The registry `netlist` spec is usually enough. Write a branch in
   `src/engine/NetlistGenerator.cpp` only if the entry needs logic: several inputs
   with fallbacks, parameters that depend on each other, multiple netlist elements
   (three-phase sources, transformers), or a variable pin count. Hand-written
   branches take precedence over the spec.
4. **Schematic symbol.** Optional: add a drawing in
   `SchematicCanvas::drawComponentShape`. Without one the block is drawn as a box.

## 3. Roles

`role` says what the block does in a simulation: `model` (default, emitted into the
netlist), `structural` (ground, ports, labels, subsystems), `observer` (scopes,
probes, the loss analyser) or `unsupported` (placeable but not simulated).
`tests/registry/test_registry.cpp` fails if a role disagrees with what the netlist
generator actually emits, so a `model` block without a netlist entry is caught.

## 4. Checks to run

```
cmake --build build
cd build && ctest
```

- `registry`: schema, palette, type mapping, roles, inspector choices.
- `emission`: hashes the netlist of every block with default, non-default and wired
  inputs against `tests/registry/emission_snapshot.txt`. A new block adds lines:
  review them and refresh with `test_emission tests/registry/emission_snapshot.txt
  --update`. A changed line for an existing block means its emission changed.
  The snapshot also lists `unmapped` parameters: parameters whose value never
  reaches the netlist. Either the generator reads a different name, or the value
  is transformed; check each new one.
- `golden`: netlists and waveforms of the design corpus, bitwise. Refresh with
  `golden_runner --update` only for an intended change.
- `custom_block`: the JSON-only path end to end.

## 5. Renaming a type or parameter

Old designs keep working if the old type name is kept in `aliases`. Parameters
are stored by name in design files, so renaming one needs a migration in
`src/engine/DesignIO.cpp` (see the resistor `resistance` / `R` -> `value` alias
there).
