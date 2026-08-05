#pragma once

#include "engine/CircuitSimulator.hpp"
#include "imgui.h"
#include "implot.h"
#include <string>
#include <vector>
#include <array>

namespace CircuitSim {

class OscilloscopeView {
private:
    int numPanes = 1;
    bool autoFitNext = false;
    bool isAdaptiveZoomEnabled = true;

    // Per-pane deferred zoom: SetNextAxesLimits must be called BEFORE BeginPlot,
    // so we store the zoom result and apply it on the next frame.
    static constexpr int MAX_PANES = 4;
    std::array<bool,   MAX_PANES> hasPendingZoom = {};
    std::array<double, MAX_PANES> pendingXMin    = {};
    std::array<double, MAX_PANES> pendingXMax    = {};
    std::array<double, MAX_PANES> pendingYMin    = {};
    std::array<double, MAX_PANES> pendingYMax    = {};

public:
    OscilloscopeView() = default;

    void triggerAutoFit() { autoFitNext = true; }
    void render(const char* title, CircuitSimEngine::CircuitSimulator& simulator);
};

} // namespace CircuitSim
