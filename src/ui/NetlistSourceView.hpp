#pragma once

#include "engine/Components.hpp"
#include "engine/CircuitSimulator.hpp"
#include "imgui.h"
#include <string>
#include <array>

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

    // Per-pane deferred zoom: SetNextAxesLimits must be called BEFORE BeginPlot
    static constexpr int MAX_PANES = 4;
    std::array<bool,   MAX_PANES> hasPendingZoom = {};
    std::array<double, MAX_PANES> pendingXMin    = {};
    std::array<double, MAX_PANES> pendingXMax    = {};
    std::array<double, MAX_PANES> pendingYMin    = {};
    std::array<double, MAX_PANES> pendingYMax    = {};

public:
    NetlistSourceView() = default;

    void triggerAutoFit() { autoFitNext = true; }
    static std::string generateNetlistJson(const CircuitDesign& design);
    void updateFromCircuit(const CircuitDesign& design);
    void render(const char* title, CircuitDesign& design, CircuitSimEngine::CircuitSimulator& simulator);
};

} // namespace CircuitSim
