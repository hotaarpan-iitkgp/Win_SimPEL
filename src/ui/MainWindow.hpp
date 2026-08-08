#pragma once

#include "SchematicCanvas.hpp"
#include "OscilloscopeView.hpp"
#include "ScopeWindow.hpp"
#include "NetlistSourceView.hpp"
#include "engine/CircuitSimulator.hpp"
#include "imgui.h"
#include <vector>
#include <memory>
#include <thread>
#include <atomic>

namespace CircuitSim {

enum class WorkspaceMode { SchematicCAD, WaveformNetlist };

class MainWindow {
private:
    SchematicCanvas canvas;
    OscilloscopeView scopeView;
    NetlistSourceView netlistSourceView;
    CircuitSimEngine::CircuitSimulator simulator;

    // Open scope popup windows (PLECS/MATLAB-style)
    std::vector<ScopeWindow> openScopeWindows;

    WorkspaceMode activeWorkspace = WorkspaceMode::SchematicCAD;
    bool isDarkMode = true;
    bool showComponentPalette = true;
    bool showDetailedLibrary = false;
    char searchPaletteBuf[128] = "";

    bool showSimParamsModal = false;
    char simStopTimeBuf[64] = "0.02";
    char simStepSizeBuf[64] = "1u";
    int simSolverIdx = 0;

    // Background simulation thread
    std::thread simThread;
    std::atomic<bool> simRunning{false};

    void applyDarkTheme();
    void applyLightTheme();

    void renderMenuBar();
    void renderControlBar();
    void renderComponentPalette();
    void renderPropertyInspector();
    void renderSimParamsModal();

    // Scope window helpers
    void handleScopeOpenRequest();
    std::vector<std::string> traceScopeInputSignals(const std::string& scopeId, int numChannels);

public:
    MainWindow();
    ~MainWindow() { if (simThread.joinable()) simThread.join(); }

    void startSimulation();
    void loadPresetTemplate(const std::string& name);
    void render();
};

} // namespace CircuitSim
