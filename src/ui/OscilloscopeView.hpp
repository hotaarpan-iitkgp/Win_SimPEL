#pragma once

#include "engine/CircuitSimulator.hpp"
#include "imgui.h"
#include "implot.h"
#include <string>
#include <vector>

namespace CircuitSim {

class OscilloscopeView {
private:
    int numPanes = 1;
    bool autoFitNext = false;
    bool isAdaptiveZoomEnabled = true;

public:
    OscilloscopeView() = default;

    void triggerAutoFit() { autoFitNext = true; }
    void render(const char* title, CircuitSimEngine::CircuitSimulator& simulator);
};

} // namespace CircuitSim
