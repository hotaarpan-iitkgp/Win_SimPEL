#pragma once

#include "CircuitSimulator.hpp"
#include <string>
#include <vector>

namespace CircuitSimEngine {

class NetlistParser {
public:
    static bool parseJsonString(const std::string& jsonContent, 
                                std::vector<ComponentModel>& outPhysical, 
                                std::vector<ComponentModel>& outControl, 
                                SimulationConfig& outConfig);
};

} // namespace CircuitSimEngine
