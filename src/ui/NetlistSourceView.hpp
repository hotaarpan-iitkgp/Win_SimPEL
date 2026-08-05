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
    int numPanes = 1;
    bool autoFitNext = false;
    bool isAdaptiveZoomEnabled = true;

public:
    NetlistSourceView() = default;

    void triggerAutoFit() { autoFitNext = true; }
    static std::string generateNetlistJson(const CircuitDesign& design);
    void updateFromCircuit(const CircuitDesign& design);
    void render(const char* title, CircuitDesign& design, CircuitSimEngine::CircuitSimulator& simulator);
};

} // namespace CircuitSim
