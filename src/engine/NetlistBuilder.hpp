#pragma once

#include "Components.hpp"
#include <unordered_map>
#include <string>
#include <vector>

namespace CircuitSim {

class NetlistBuilder {
public:
    static void buildNodesForCircuit(CircuitDesign& design);
};

} // namespace CircuitSim
