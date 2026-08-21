#pragma once

#include "engine/CircuitSimulator.hpp"
#include "engine/FourierAnalysis.hpp"
#include "ui/TracePlotter.hpp"
#include "ScopeWindow.hpp"
#include "imgui.h"
#include "implot.h"
#include <string>
#include <vector>
#include <array>

namespace CircuitSim {

enum WaveformZoomType { ZOOM_NONE = 0, ZOOM_X_ONLY, ZOOM_Y_ONLY, ZOOM_BOX_2D };

enum class ActiveZoomMode {
    Adaptive = 0, // Smart detection
    X_Only   = 1, // Explicit 1D X-Axis Zoom (Time Range)
    Y_Only   = 2, // Explicit 1D Y-Axis Zoom (Amplitude Range)
    Box_2D   = 3, // Explicit 2D Box Zoom
    Disabled = 4  // Zoom disabled (normal pan)
};

struct WaveformPendingZoom {
    bool hasPending = false;
    WaveformZoomType type = ZOOM_NONE;
    double xMin = 0.0, xMax = 0.0;
    double yMin = 0.0, yMax = 0.0;
};

struct CustomZoomDragState {
    bool isDragging = false;
    ImPlotPoint startPt = {0.0, 0.0};
    ImPlotPoint currentPt = {0.0, 0.0};
    ImVec2 startPx = {0.0f, 0.0f};
    ImVec2 currentPx = {0.0f, 0.0f};
};

class OscilloscopeView {
private:
    int numPanes = 1;
    bool autoFitNext = false;
    ActiveZoomMode activeZoomMode = ActiveZoomMode::Adaptive;
    bool isDarkMode = true;
    float traceLineWidth = 2.0f;
    bool isCollapsed = false;
    float savedDockHeight = 0.0f; // Stores height before collapse

    // Cursor & Interpolation state
    ScopeCursorState cursorState;
    InterpolationMode globalPlotMode = InterpolationMode::AutoHybrid;

    // Telemetry cache optimization to avoid 100k-point vector copies every frame
    uint64_t lastTelemetryVer = 0;
    CircuitSimEngine::TelemetryData cachedTelemetry;

    // Per-pane deferred zoom & custom gesture tracking
    static constexpr int MAX_PANES = 4;
    std::array<WaveformPendingZoom, MAX_PANES> pendingZoom = {};
    std::array<CustomZoomDragState, MAX_PANES> customDragState = {};

    void renderCursorOverlay(int paneIdx, const CircuitSimEngine::TelemetryData& data);
    void renderDataPanel(const CircuitSimEngine::TelemetryData& data);
    void renderHarmonicsWindow(const CircuitSimEngine::TelemetryData& data);
    double interpolateSignal(const std::vector<double>& timeHist, const std::vector<double>& signalData, double targetT, bool snap) const;

public:
    OscilloscopeView() = default;

    void setDarkMode(bool dark) { isDarkMode = dark; }
    void setTraceLineWidth(float width) { traceLineWidth = width; }
    float getTraceLineWidth() const { return traceLineWidth; }

    void triggerAutoFit() { autoFitNext = true; }
    void render(const char* title, CircuitSimEngine::CircuitSimulator& simulator, const CircuitDesign* design = nullptr);
};

} // namespace CircuitSim
