#pragma once

#include "engine/CircuitSimulator.hpp"
#include "engine/Components.hpp"
#include "engine/FourierAnalysis.hpp"
#include "imgui.h"
#include "implot.h"
#include <string>
#include <vector>
#include <array>

namespace CircuitSim {

// Zoom types for the scope window (mirrors OscilloscopeView)
enum class ScopeZoomMode {
    Adaptive = 0,
    X_Only   = 1,
    Y_Only   = 2,
    Box_2D   = 3,
    Disabled = 4
};

enum ScopeZoomType { SZ_NONE = 0, SZ_X_ONLY, SZ_Y_ONLY, SZ_BOX_2D };

struct ScopePendingZoom {
    bool hasPending = false;
    ScopeZoomType type = SZ_NONE;
    double xMin = 0.0, xMax = 0.0;
    double yMin = 0.0, yMax = 0.0;
};

struct ScopeDragState {
    bool isDragging = false;
    ImPlotPoint startPt = {0.0, 0.0};
    ImPlotPoint currentPt = {0.0, 0.0};
    ImVec2 startPx = {0.0f, 0.0f};
    ImVec2 currentPx = {0.0f, 0.0f};
};

struct ScopeCursorState {
    bool showCursors = false;
    double cursor1Time = 0.0;
    double cursor2Time = 0.0;
    bool initialized = false;
    bool snapToSample = false;
    bool lockBoundary = true;
    bool showHarmonicsWindow = false;
    int maxHarmonics = 50;
};

// A standalone PLECS/MATLAB-style scope popup window.
// Each instance corresponds to one SCOPE component on the schematic.
// Supports subplots (one per channel), zoom modes, and real-time updates.
class ScopeWindow {
private:
    std::string scopeId;
    std::string windowTitle;
    int numChannels = 2;
    bool isOpen = true;
    bool isDarkMode = true;
    float traceLineWidth = 2.0f;

    // Layout mode
    bool useSubplots = true;   // true = each channel in its own subplot row
    int numPanes = 1;          // how many subplot rows to show (auto or manual)

    // Zoom & Cursor state
    ScopeZoomMode activeZoomMode = ScopeZoomMode::Disabled;
    bool autoFitNext = true;
    ScopeCursorState cursorState;

    // Per-pane zoom & drag state (max 8 channels)
    static constexpr int MAX_PANES = 8;
    std::array<ScopePendingZoom, MAX_PANES> pendingZoom = {};
    std::array<ScopeDragState, MAX_PANES> customDragState = {};

    // Minimize/maximize state
    bool isMinimized = false;
    bool isMaximized = false;
    ImVec2 savedWindowSize = {700.0f, 450.0f};
    ImVec2 savedWindowPos = {0.0f, 0.0f};
    bool hasSavedPosSize = false;

    // Signal keys and labels for each input channel
    std::vector<std::string> channelSignalKeys;
    std::vector<std::string> channelLabels;

    void renderToolbar(const CircuitSimEngine::TelemetryData& data);
    void renderPlots(const CircuitSimEngine::TelemetryData& data);
    void renderZoomOverlay(int paneIdx);
    void renderCursorOverlay(int paneIdx, const CircuitSimEngine::TelemetryData& data);
    void renderDataPanel(const CircuitSimEngine::TelemetryData& data);
    void renderHarmonicsWindow(const CircuitSimEngine::TelemetryData& data);

    double interpolateSignal(const std::vector<double>& timeHist, const std::vector<double>& signalData, double targetT, bool snap) const;

public:
    ScopeWindow() = default;
    ScopeWindow(const std::string& scopeCompId, int channels,
                const std::vector<std::string>& signalKeys,
                const std::vector<std::string>& labels);

    void setDarkMode(bool dark) { isDarkMode = dark; }
    bool isWindowOpen() const { return isOpen; }
    const std::string& getScopeId() const { return scopeId; }

    void render(CircuitSimEngine::CircuitSimulator& simulator);
};

} // namespace CircuitSim
