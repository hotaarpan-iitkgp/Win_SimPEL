#pragma once

#include "engine/Components.hpp"
#include "engine/CircuitSimulator.hpp"
#include "imgui.h"
#include <string>

namespace CircuitSim {

class NetlistSourceView {
private:
    char jsonBuffer[65536] = "";
    bool isNetlistValid = true;
    std::string netlistStatusMsg = "Valid Netlist";
    std::string lastGeneratedJson = "";

public:
    NetlistSourceView() = default;

    void updateFromCircuit(const CircuitDesign& design);
    void render(const char* title, CircuitDesign& design, CircuitSimulator& simulator);
};

} // namespace CircuitSim
