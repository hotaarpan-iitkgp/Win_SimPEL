#pragma once

#include "SchematicCanvas.hpp"
#include "OscilloscopeView.hpp"
#include "engine/CircuitSimulator.hpp"
#include "imgui.h"

namespace CircuitSim {

class MainWindow {
private:
    SchematicCanvas canvas;
    OscilloscopeView scopeView;
    CircuitSimulator simulator;

    void renderMenuBar();
    void renderControlBar();
    void renderComponentPalette();
    void renderPropertyInspector();

public:
    MainWindow();

    void loadPresetTemplate(const std::string& name);
    void render();
};

} // namespace CircuitSim
