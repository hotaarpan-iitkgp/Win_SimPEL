#pragma once

#include "engine/CircuitSimulator.hpp"
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

class OscilloscopeView {
private:
    int numPanes = 1;
    bool autoFitNext = false;
    ActiveZoomMode activeZoomMode = ActiveZoomMode::Adaptive;
    bool isDarkMode = true;
    float traceLineWidth = 2.0f;

    // Per-pane deferred zoom: SetNextAxisLimits must be called BEFORE BeginPlot
    static constexpr int MAX_PANES = 4;
    std::array<WaveformPendingZoom, MAX_PANES> pendingZoom = {};

public:
    OscilloscopeView() = default;

    void setDarkMode(bool dark) { isDarkMode = dark; }
    void setTraceLineWidth(float width) { traceLineWidth = width; }
    float getTraceLineWidth() const { return traceLineWidth; }

    void triggerAutoFit() { autoFitNext = true; }
    void render(const char* title, CircuitSimEngine::CircuitSimulator& simulator);
};

} // namespace CircuitSim
