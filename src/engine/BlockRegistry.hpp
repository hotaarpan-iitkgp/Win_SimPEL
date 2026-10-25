#pragma once

// The block registry: one description of every block type, loaded from
// data/blocks/<category>.json. It is the source of truth for block names,
// categories, parameters and their defaults, and the component palette.
// Pin layouts are not duplicated in the data: they are computed from
// getTerminals() on a default instance, so they cannot drift from what the
// schematic and the netlist builder use.
//
// The JSON files are compiled into the core, so the GUI, CLI and Python module
// work without locating a data directory. Setting the environment variable
// SIMPEL_BLOCKS_DIR to a directory of *.json files makes instance() load those
// instead (useful when developing new blocks).
//
// File format (schema 1):
// {
//   "schema": 1, "category": "electrical", "description": "...",
//   "blocks": [
//     { "type": "R", "name": "Resistor", "subcategory": "Passive Components",
//       "description": "...",                       optional
//       "params": [ {"name": "value", "default": "10", "unit": "Ohm", "description": "..."} ],
//       "palette": [                                 zero or more palette buttons
//         { "order": 120, "text": "Resistor (R)", "label": "R", "prefix": "R",
//           "ui_type": "Resistor", "subcategory": "Passive Components", "basic": true,
//           "params": [["value", "10"]] } ] } ] }
// "order" is the global position of the button in the palette (unique across files).
// "ui_type" names a CircuitSim::ComponentType enumerator.
//
// Further block fields:
//   "ui_type"   required. ComponentType of instances of this block; drives
//               stringToComponentType().
//   "aliases"   other type strings designs may use for this block (legacy and
//               web-tool spellings). They map to the same ui_type. Unique across
//               all files and distinct from every block type.
//   "role"      "model" (default: emitted into the solver netlist), "structural"
//               (shapes connectivity only: ground, ports, labels, subsystems),
//               "observer" (reads results: scopes, probes, analysers) or
//               "unsupported" (accepted in designs but has no solver model yet).
//   "legacy"    true for type names kept only so old designs load; hidden from
//               the palette and from the Python API's default listings.
//   "choices"   dropdown values for the property inspector, keyed by lower-case
//               parameter name: {"operator": ["<", "<=", ">"]}.
//   "netlist"   how to emit the block into the solver netlist when NetlistGenerator
//               has no hand-written branch for it:
//               {"section": "control_loops.gains",
//                "fields": {"output": "{id}.Out", "original_type": "OFFSET",
//                           "offset": {"param": "offset", "default": 0.0},
//                           "input": {"signal": "In"}, "K": 1.0}}
//               See NetlistField for the field forms. "id" is always emitted.
//   "pins_from" reuse the pin layout of another block type.
//   "pins"      explicit pin layout for blocks the pin code does not know:
//               [{"name": "In", "x": -30, "y": 0, "dx": -1, "dy": 0, "kind": "control"}]
//               ("kind" is "control" or "power").
// Pin layouts hard-coded in getTerminals() take precedence over "pins_from"/"pins".

#include "Components.hpp"

#include <map>
#include <string>
#include <utility>
#include <vector>

namespace CircuitSim {

// Name <-> value for every ComponentType enumerator (generated from Components.hpp).
bool componentTypeFromName(const std::string& name, ComponentType& out);
const char* componentTypeName(ComponentType t);   // "Unknown" for out-of-range
int componentTypeCount();

struct BlockParam {
    std::string name;
    std::string defaultValue;
    std::string unit;          // may be empty
    std::string description;   // may be empty
};

struct PaletteEntry {
    int order = 0;
    std::string text;          // button text
    std::string label;         // default instance label
    std::string prefix;        // id prefix ("R" -> R1, R2, ...)
    std::string type;          // block type (rawTypeStr)
    ComponentType uiType = ComponentType::Unknown;
    std::string uiTypeName;
    std::string category;
    std::string subcategory;
    bool basic = false;
    std::vector<std::pair<std::string, std::string>> params;   // ordered defaults
};

struct BlockPin {
    std::string name;
    float relX = 0, relY = 0;  // position relative to the block centre, rotation 0
    float dirX = 0, dirY = 0;  // outward stub direction
    bool isControl = false;    // control-signal pin (otherwise electrical / magnetic)
};

// One field of a registry-described netlist entry ("netlist": {"fields": {...}}).
struct NetlistField {
    enum class Kind {
        Text,         // "key": "text"            {id} and {type} are substituted
        Number,       // "key": 1.0 / 2           literal number (int stays int)
        Signal,       // "key": {"signal": "In"}  signal driving input pin "In", "0.0" if none
        SignalArray,  // "key": [{"signal": "A"}, {"signal": "B"}]
        Param,        // "key": {"param": "delay", "default": 0.1, "as": "number|int|string"}
        Nodes         // "key": {"nodes": true}   the block's electrical nodes
    };
    std::string key;
    Kind kind = Kind::Text;
    std::string text;                  // Text template, Param name
    double number = 0.0;               // Number value, numeric Param default
    bool isInt = false;                // Number is an integer literal / Param "as": "int"
    bool asString = false;             // Param "as": "string" (raw parameter text)
    std::string defaultText;           // string Param default
    std::vector<std::string> signals;  // Signal (one) / SignalArray pins
};

struct NetlistSpec {
    bool present = false;
    std::string stage;                 // "control_loops" or "physical_stage"
    std::string section;               // e.g. "gains", "inductors"
    std::vector<NetlistField> fields;
};

enum class BlockRole { Model, Structural, Observer, Unsupported };
const char* blockRoleName(BlockRole r);

struct BlockDef {
    std::string type;
    std::string name;
    std::string category;
    std::string subcategory;
    std::string description;
    std::string sourceFile;
    ComponentType uiType = ComponentType::Unknown;
    std::string uiTypeName;
    std::vector<std::string> aliases;
    BlockRole role = BlockRole::Model;
    bool legacy = false;
    std::string pinsFrom;               // empty = none
    std::vector<BlockPin> declaredPins; // from "pins"
    std::vector<BlockParam> params;
    // Allowed values for parameters edited with a dropdown, keyed by lower-case
    // parameter name ("choices" in the JSON). A key may name a parameter that is
    // not in `params`: legacy designs use several spellings of the same setting.
    std::map<std::string, std::vector<std::string>> choices;
    // How the block is written into the solver netlist when the netlist generator
    // has no hand-written emitter for it ("netlist" in the JSON).
    NetlistSpec netlist;
    std::vector<int> paletteEntries;   // indices into BlockRegistry::palette()

    const BlockParam* findParam(const std::string& n) const;
    // Dropdown values for parameter `name` (any case), or null.
    const std::vector<std::string>* choicesFor(const std::string& name) const;
};

// The pins of an actual component instance, as the schematic shows them: its
// terminal layout (which may depend on its parameters), or the generic pins from
// setupComponentPins when it has none.
std::vector<struct BlockPin> instancePins(const ComponentInstance& comp);

// Pin layout the registry supplies for `comp` when the hard-coded layouts in
// getTerminals() have none ("pins_from" or "pins"). Empty if neither is set.
std::vector<TerminalDef> registryTerminals(const ComponentInstance& comp);

class BlockRegistry {
public:
    // Process-wide registry: embedded data, or SIMPEL_BLOCKS_DIR if set. Loaded on
    // first use (thread-safe). Throws std::runtime_error if the data is invalid.
    static const BlockRegistry& instance();

    // (file name, JSON text) pairs. Throws std::runtime_error naming the file and
    // the problem on any schema violation.
    static BlockRegistry fromJsonTexts(const std::vector<std::pair<std::string, std::string>>& files);
    static BlockRegistry fromDirectory(const std::string& dir);
    static BlockRegistry fromEmbedded();

    const std::vector<BlockDef>& blocks() const { return blocks_; }
    const std::vector<PaletteEntry>& palette() const { return palette_; }   // sorted by order
    const BlockDef* find(const std::string& type) const;      // canonical type only
    const BlockDef* resolve(const std::string& name) const;   // type or alias
    // Block with a "netlist" spec for `name`: exact type or alias first, then a
    // case-insensitive match (the netlist generator has always compared upper-cased
    // type names). Null if none.
    const BlockDef* netlistBlockFor(const std::string& name) const;
    std::vector<std::string> categories() const;   // in first-appearance order

    // Closest known type names (case-insensitive), best first. For error messages.
    std::vector<std::string> suggest(const std::string& type, size_t maxResults = 3) const;

    // A component of `type` with the registry defaults, `overrides` applied on top,
    // and pins set up exactly as the schematic does it. Throws if type is unknown.
    ComponentInstance makeInstance(const std::string& type, const std::string& id,
                                   const std::map<std::string, std::string>& overrides = {}) const;

    // Pins of a `type` instance with the given parameters (defaults if null).
    // Some blocks (C-Script, sum/product, multi-winding transformers, ...) derive
    // their pins from parameters.
    std::vector<BlockPin> pins(const std::string& type,
                               const std::map<std::string, std::string>* params = nullptr) const;

private:
    std::vector<BlockDef> blocks_;
    std::vector<PaletteEntry> palette_;
    std::map<std::string, size_t> index_;
    std::map<std::string, size_t> aliasIndex_;
    std::map<std::string, size_t> netlistUpperIndex_;   // UPPER(type/alias) -> block, spec'd blocks only
};

} // namespace CircuitSim
