#pragma once

#include "engine/CircuitSimulator.hpp"
#include "imgui.h"
#include "implot.h"
#include <string>
#include <vector>

namespace CircuitSim {

class OscilloscopeView {
public:
    OscilloscopeView() = default;

    void render(const char* title, CircuitSimulator& simulator);
};

} // namespace CircuitSim
