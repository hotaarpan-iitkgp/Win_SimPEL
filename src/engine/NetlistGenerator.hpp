#pragma once

#include "Components.hpp"
#include <string>

namespace CircuitSim {

// Builds the solver netlist JSON (the format NetlistParser consumes) for a design.
// Node names are derived internally from a copy of the design, so the caller's
// design is not modified.
std::string generateNetlistJson(const CircuitDesign& design);

} // namespace CircuitSim
