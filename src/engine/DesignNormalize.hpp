#pragma once

#include "Components.hpp"

namespace CircuitSim {

// Clears junction flags whose target wire no longer exists.
void cleanupOrphanedJunctions(CircuitDesign& design);

// Where several wires end directly on the same pin, keeps the first one on the pin
// and re-attaches the others as junctions onto it at the pin's stub point.
void consolidateOverlappingWires(CircuitDesign& design);

// Both of the above, in the order SchematicCanvas::setCircuit applies them.
void normaliseDesignWires(CircuitDesign& design);

} // namespace CircuitSim
