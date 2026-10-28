#pragma once

#include "Components.hpp"
#include <unordered_map>
#include <string>
#include <vector>

namespace CircuitSim {

class NetlistBuilder {
public:
    // Assigns comp.nodes. Every net that contains a GND block's terminal is node "0"
    // regardless of wire order or direction; without a connected GND the negative
    // terminal of the first voltage source (then a 3-phase source's neutral) is used.
    static void buildNodesForCircuit(CircuitDesign& design);
};

} // namespace CircuitSim
