#pragma once

// Programmatic circuit construction: blocks are added by registry type, wired by
// pin name, and turned into a CircuitDesign the existing pipeline simulates. The
// description is connectivity only; positions are placeholders until layout.
//
//   Circuit c("buck");
//   c.add("V", "V1", {{"value", "48"}});
//   c.add("MOSFET", "Q1", {{"Ron", "10m"}});
//   c.connect({{"V1", "A"}, {"Q1", "D"}});
//   c.ground({{"V1", "B"}});
//   auto issues = c.validate();
//
// Connections form nets. When a design is produced, each net becomes a star of
// wires from one hub pin: the net's control-signal driver if it has exactly one
// (the netlist generator reads a control input's signal from the pin at the other
// end of its wire), otherwise its first pin.

#include "BlockRegistry.hpp"
#include "Components.hpp"

#include <map>
#include <string>
#include <vector>

namespace CircuitSim {

struct PinRef {
    std::string component;
    std::string pin;
};

struct ValidationIssue {
    enum class Severity { Error, Warning };
    Severity severity = Severity::Warning;
    std::string code;        // stable identifier, e.g. "unconnected_pin"
    std::string component;   // may be empty
    std::string pin;         // may be empty
    std::string message;
};

class Circuit {
public:
    explicit Circuit(std::string name = "circuit");

    // Takes the components and wires of an existing design; new connections are
    // added on top. Settings are kept and returned by design().
    static Circuit fromDesign(const CircuitDesign& design, std::string name = "circuit");

    const std::string& name() const { return name_; }

    // Adds a block with registry defaults plus `params`. An empty id picks the
    // next free "<prefix><n>". Throws std::invalid_argument for an unknown type
    // (with suggestions), a malformed or duplicate id. Returns the id used.
    std::string add(const std::string& type, const std::string& id = "",
                    const std::map<std::string, std::string>& params = {});
    void remove(const std::string& id);   // also drops its connections

    bool has(const std::string& id) const;
    const ComponentInstance& component(const std::string& id) const;   // throws std::out_of_range
    std::vector<std::string> componentIds() const;                      // in insertion order
    void setParam(const std::string& id, const std::string& name, const std::string& value);

    // Pins of a component (names as the schematic uses them).
    std::vector<BlockPin> pins(const std::string& id) const;
    // Canonical pin name for `pin` on component `id`: exact match first, then a
    // case-insensitive one. Throws std::invalid_argument listing the valid pins.
    std::string resolvePin(const std::string& id, const std::string& pin) const;

    // Puts all given pins on one net (at least two). Throws on unknown
    // components or pins.
    void connect(const std::vector<PinRef>& pins);
    // Connects the pins to the circuit's ground (a GND block, added on first use).
    void ground(const std::vector<PinRef>& pins);

    std::vector<ValidationIssue> validate() const;

    // Parameters set on a component that change nothing in the solver netlist
    // (typically a misspelt name). Parameters the registry lists, and the few the
    // GUI and loss analysis use outside the netlist, are never reported.
    std::vector<std::string> ineffectiveParams(const std::string& id) const;
    // Whether changing parameter `name` of component `id` changes the solver
    // netlist (two probe values are compared; the circuit is not modified).
    // Registry parameters and the GUI / loss-analysis ones count as effective.
    bool paramAffectsNetlist(const std::string& id, const std::string& name) const;

    CircuitDesign design() const;
    SolverSettings& settings() { return settings_; }
    const SolverSettings& settings() const { return settings_; }

private:
    std::string name_;
    std::vector<ComponentInstance> comps_;
    std::vector<WireInstance> baseWires_;                 // from fromDesign()
    std::vector<std::pair<std::string, std::string>> links_;  // "comp:pin" pairs
    SolverSettings settings_;
    PlotConfig plotConfig_;
    std::string groundId_;

    ComponentInstance* find(const std::string& id);
    const ComponentInstance* find(const std::string& id) const;
    std::string pinKey(const PinRef& p) const;   // validates, returns "comp:pin"
    void placeNew(ComponentInstance& c) const;
};

} // namespace CircuitSim
