#pragma once

#include "Components.hpp"
#include <unordered_map>
#include <string>
#include <vector>

namespace CircuitSim {

class NetlistBuilder {
public:
    static void buildNodesForCircuit(CircuitDesign& design);

    // buildNodesForCircuit takes the first GND block's "Gnd" pin as the reference
    // node only if that pin is *not* the representative of its connected set in the
    // union-find it builds; when it is, the reference falls through to a phantom
    // pin and the netlist generator's own ground fallback decides. Which pin ends up
    // as representative depends on the order and direction of the wires. Tools that
    // rewrite wires (the router) use this to keep the outcome unchanged.
    // Returns false when there is no GND block; *groundId is set otherwise.
    static bool firstGroundPinIsSetRoot(const CircuitDesign& design, std::string* groundId = nullptr);
};

} // namespace CircuitSim
