#pragma once

#include "engine/CircuitSimulator.hpp"
#include "imgui.h"
#include "implot.h"
#include <string>
#include <vector>
#include <array>

namespace CircuitSim {

enum WaveformZoomType { ZOOM_NONE = 0, ZOOM_X_ONLY, ZOOM_Y_ONLY, ZOOM_BOX_2D };

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
    bool isAdaptiveZoomEnabled = true;
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
