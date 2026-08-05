#pragma once

#include "engine/Components.hpp"
#include "engine/CircuitSimulator.hpp"
#include "imgui.h"
#include <string>
#include <array>
#include "OscilloscopeView.hpp"

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
    bool isDarkMode = true;
    float traceLineWidth = 2.0f;
    float splitRatio = 0.10f;

    // Per-pane deferred zoom: SetNextAxisLimits must be called BEFORE BeginPlot
    static constexpr int MAX_PANES = 4;
    std::array<WaveformPendingZoom, MAX_PANES> pendingZoom = {};

public:
    NetlistSourceView() = default;

    void setDarkMode(bool dark) { isDarkMode = dark; }
    void setTraceLineWidth(float width) { traceLineWidth = width; }
    float getTraceLineWidth() const { return traceLineWidth; }

    void triggerAutoFit() { autoFitNext = true; }
    static std::string generateNetlistJson(const CircuitDesign& design);
    void updateFromCircuit(const CircuitDesign& design);
    void render(const char* title, CircuitDesign& design, CircuitSimEngine::CircuitSimulator& simulator);
};

} // namespace CircuitSim
