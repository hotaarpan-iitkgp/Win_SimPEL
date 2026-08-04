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

    bool showSimParamsModal = false;
    char simStopTimeBuf[64] = "0.02";
    char simStepSizeBuf[64] = "1u";
    int simSolverIdx = 0;

    void renderMenuBar();
    void renderControlBar();
    void renderComponentPalette();
    void renderPropertyInspector();
    void renderSimParamsModal();

public:
    MainWindow();

    void loadPresetTemplate(const std::string& name);
    void render();
};

} // namespace CircuitSim
