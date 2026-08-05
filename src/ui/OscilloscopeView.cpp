#include "OscilloscopeView.hpp"
#include "implot.h"
#include <iostream>
#include <algorithm>

namespace CircuitSim {

struct SignalCategory {
    std::string title;
    std::string yLabel;
    std::vector<std::pair<std::string, std::vector<double>>> variables;
};

void OscilloscopeView::render(const char* title, CircuitSimEngine::CircuitSimulator& simulator) {
    ImGui::Begin(title);
    
    CircuitSimEngine::TelemetryData data = simulator.getTelemetryCopy();
    
    if (data.timeHistory.empty()) {
        ImGui::TextColored(ImVec4(0.7f, 0.7f, 0.7f, 1.0f), "No simulation waveform data available. Press PLAY to run simulation.");
        ImGui::End();
        return;
    }

    SignalCategory voltageCat{"Voltage Waveforms (V)", "Voltage (V)", {}};
    SignalCategory currentCat{"Current Waveforms (I)", "Current (A)", {}};
    SignalCategory controlCat{"Control & Pulse Signals", "Signal (V / State)", {}};
    SignalCategory otherCat{"Other Signals", "Magnitude", {}};

    for (const auto& pair : data.voltages) {
        const std::string& name = pair.first;
        const std::vector<double>& vals = pair.second;
        if (vals.empty()) continue;

        // Skip internal raw MNA matrix node voltages (node_1, node_2, 0, etc.)
        if (name.rfind("node_", 0) == 0 || name == "0" || name == "node_0") continue;

        if (name.rfind("I_", 0) == 0) {
            currentCat.variables.push_back({name, vals});
        } else if (name.rfind("V_", 0) == 0) {
            voltageCat.variables.push_back({name, vals});
        } else if (name.find(".Out") != std::string::npos || name.rfind("Ctrl_", 0) == 0 || name.rfind("PULSE", 0) != std::string::npos || name.rfind("PWM", 0) != std::string::npos) {
            controlCat.variables.push_back({name, vals});
        } else {
            otherCat.variables.push_back({name, vals});
        }
    }

    std::vector<SignalCategory> categories;
    if (!voltageCat.variables.empty()) categories.push_back(voltageCat);
    if (!currentCat.variables.empty()) categories.push_back(currentCat);
    if (!controlCat.variables.empty()) categories.push_back(controlCat);
    if (!otherCat.variables.empty()) categories.push_back(otherCat);

    if (categories.empty()) {
        ImGui::TextColored(ImVec4(0.7f, 0.7f, 0.7f, 1.0f), "No active component signals selected for plotting.");
        ImGui::End();
        return;
    }

    bool doFitThisFrame = autoFitNext;
    if (autoFitNext) autoFitNext = false;
    if (ImGui::Button("Fit Waveforms / Reset Zoom")) doFitThisFrame = true;
    ImGui::SameLine();

    if (isAdaptiveZoomEnabled) {
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.12f, 0.45f, 0.80f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.20f, 0.55f, 0.90f, 1.0f));
        if (ImGui::Button("🔍 Adaptive Box Zoom: ON")) {
            isAdaptiveZoomEnabled = false;
        }
        ImGui::PopStyleColor(2);
    } else {
        if (ImGui::Button("🔍 Adaptive Box Zoom: OFF")) {
            isAdaptiveZoomEnabled = true;
        }
    }
    ImGui::SameLine();

    if (ImGui::Button("➕ Add Plot Pane")) {
        numPanes = std::min(numPanes + 1, 4);
    }
    ImGui::SameLine();
    if (numPanes > 1) {
        if (ImGui::Button("➖ Remove Plot Pane")) {
            numPanes = std::max(numPanes - 1, 1);
        }
        ImGui::SameLine();
    }

    ImGui::TextDisabled("|");
    ImGui::SameLine();

    ImGui::SetNextItemWidth(100.0f);
    ImGui::SliderFloat("Line Width", &traceLineWidth, 1.0f, 6.0f, "%.1f px");
    ImGui::SameLine();

    ImGui::TextColored(isDarkMode ? ImVec4(0.7f, 0.8f, 0.9f, 1.0f) : ImVec4(0.3f, 0.4f, 0.5f, 1.0f), "| Tip: Right-click plot to add/remove subplots, drag box to zoom.");

    int renderPanes = std::min(numPanes, (int)categories.size());
    if (renderPanes < 1) renderPanes = 1;

    static const ImVec4 DARK_MODE_COLORS[] = {
        ImVec4(0.00f, 0.95f, 1.00f, 1.00f), // Neon Cyan
        ImVec4(0.10f, 1.00f, 0.45f, 1.00f), // Bright Emerald Green
        ImVec4(1.00f, 0.88f, 0.00f, 1.00f), // Vivid Gold Yellow
        ImVec4(1.00f, 0.25f, 0.60f, 1.00f), // Bright Neon Pink
        ImVec4(1.00f, 0.50f, 0.10f, 1.00f), // Bright Coral Orange
        ImVec4(0.70f, 0.40f, 1.00f, 1.00f), // Bright Electric Violet
        ImVec4(0.40f, 0.90f, 1.00f, 1.00f), // Bright Sky Blue
        ImVec4(0.75f, 1.00f, 0.20f, 1.00f)  // Electric Lime
    };

    static const ImVec4 LIGHT_MODE_COLORS[] = {
        ImVec4(0.05f, 0.35f, 0.75f, 1.00f), // Deep Royal Navy
        ImVec4(0.02f, 0.50f, 0.25f, 1.00f), // Dark Forest Green
        ImVec4(0.80f, 0.12f, 0.12f, 1.00f), // Deep Crimson Red
        ImVec4(0.50f, 0.15f, 0.75f, 1.00f), // Deep Dark Violet
        ImVec4(0.85f, 0.30f, 0.05f, 1.00f), // Rich Dark Orange
        ImVec4(0.05f, 0.50f, 0.55f, 1.00f), // Dark Teal
        ImVec4(0.45f, 0.25f, 0.08f, 1.00f), // Dark Warm Brown
        ImVec4(0.12f, 0.18f, 0.28f, 1.00f)  // Deep Charcoal Slate
    };

    const auto& palette = isDarkMode ? DARK_MODE_COLORS : LIGHT_MODE_COLORS;
    size_t numColors = sizeof(DARK_MODE_COLORS) / sizeof(DARK_MODE_COLORS[0]);

    if (ImPlot::BeginSubplots("Oscilloscope Subplots", renderPanes, 1, ImVec2(-1, -1), ImPlotSubplotFlags_LinkCols)) {
        for (int i = 0; i < renderPanes; ++i) {
            const auto& cat = categories[i % categories.size()];

            // Pending zoom from previous frame (before BeginPlot)
            if (hasPendingZoom[i]) {
                ImPlot::SetNextAxesLimits(pendingXMin[i], pendingXMax[i],
                                          pendingYMin[i], pendingYMax[i],
                                          ImGuiCond_Always);
                hasPendingZoom[i] = false;
            } else if (doFitThisFrame) {
                // Manual fit with 8% Y padding for breathing room
                double xMin = data.timeHistory.empty() ? 0.0 : data.timeHistory.front();
                double xMax = data.timeHistory.empty() ? 1.0 : data.timeHistory.back();
                double yMin =  1e30, yMax = -1e30;
                for (const auto& vp : cat.variables) {
                    for (double v : vp.second) {
                        if (v < yMin) yMin = v;
                        if (v > yMax) yMax = v;
                    }
                }
                if (yMin > yMax) { yMin = -1.0; yMax = 1.0; }
                double yRange = yMax - yMin;
                double yPad = (yRange > 1e-9) ? yRange * 0.08 : 0.5;
                ImPlot::SetNextAxesLimits(xMin, xMax, yMin - yPad, yMax + yPad, ImGuiCond_Always);
            }

            if (ImPlot::BeginPlot(cat.title.c_str(), ImVec2(-1, -1),
                                   isAdaptiveZoomEnabled ? ImPlotFlags_NoMenus : ImPlotFlags_None)) {

                // Override mouse button bindings based on zoom mode
                if (isAdaptiveZoomEnabled) {
                    ImPlot::GetInputMap().Select       = ImGuiMouseButton_Left;  // LMB = rubber-band
                    ImPlot::GetInputMap().SelectCancel = ImGuiMouseButton_Right;
                    ImPlot::GetInputMap().Pan          = ImGuiMouseButton_Right; // RMB = pan
                } else {
                    ImPlot::GetInputMap().Select       = ImGuiMouseButton_Right;
                    ImPlot::GetInputMap().SelectCancel = ImGuiMouseButton_Left;
                    ImPlot::GetInputMap().Pan          = ImGuiMouseButton_Left;
                }

                ImPlot::SetupAxes("Time (s)", cat.yLabel.c_str());

                // Commit zoom only on mouse RELEASE — ImPlot draws the rubber-band during drag
                if (isAdaptiveZoomEnabled && ImPlot::IsPlotSelected() && ImGui::IsMouseReleased(ImGuiMouseButton_Left)) {
                    ImPlotRect sel = ImPlot::GetPlotSelection();
                    ImPlotRect cur = ImPlot::GetPlotLimits();
                    ImPlot::CancelPlotSelection();

                    double dxSel = std::abs(sel.X.Max - sel.X.Min);
                    double dySel = std::abs(sel.Y.Max - sel.Y.Min);
                    double dxCur = std::abs(cur.X.Max - cur.X.Min);
                    double dyCur = std::abs(cur.Y.Max - cur.Y.Min);

                    double nx = (dxCur > 1e-15) ? dxSel / dxCur : 0.0;
                    double ny = (dyCur > 1e-15) ? dySel / dyCur : 0.0;

                    // Store current limits as baseline
                    pendingXMin[i] = cur.X.Min; pendingXMax[i] = cur.X.Max;
                    pendingYMin[i] = cur.Y.Min; pendingYMax[i] = cur.Y.Max;

                    if (nx > 2.5 * ny) {
                        // Wide horizontal → X-axis zoom only
                        pendingXMin[i] = sel.X.Min;
                        pendingXMax[i] = sel.X.Max;
                    } else if (ny > 2.5 * nx) {
                        // Tall vertical → Y-axis zoom only
                        pendingYMin[i] = sel.Y.Min;
                        pendingYMax[i] = sel.Y.Max;
                    } else {
                        // Roughly square → full box zoom
                        pendingXMin[i] = sel.X.Min; pendingXMax[i] = sel.X.Max;
                        pendingYMin[i] = sel.Y.Min; pendingYMax[i] = sel.Y.Max;
                    }
                    hasPendingZoom[i] = true; // will be applied before BeginPlot next frame
                }

                // Right-Click Context Menu
                if (!isAdaptiveZoomEnabled && ImGui::BeginPopupContextItem("PlotContextMenu")) {
                    if (ImGui::MenuItem("➕ Add Subplot Pane Below")) {
                        numPanes = std::min(numPanes + 1, 4);
                    }
                    if (numPanes > 1) {
                        if (ImGui::MenuItem("➖ Remove Subplot Pane")) {
                            numPanes = std::max(numPanes - 1, 1);
                        }
                    }
                    if (ImGui::MenuItem("Fit Waveforms / Reset Zoom")) {
                        ImPlot::SetNextAxesToFit();
                    }
                    ImGui::EndPopup();
                }

                int varIdx = 0;
                for (const auto& varPair : cat.variables) {
                    const std::string& varName = varPair.first;
                    const std::vector<double>& vals = varPair.second;
                    int count = (int)std::min(data.timeHistory.size(), vals.size());
                    if (count > 0) {
                        ImPlotSpec spec;
                        spec.LineColor = palette[varIdx % numColors];
                        spec.LineWeight = traceLineWidth;
                        ImPlot::PlotLine(varName.c_str(), data.timeHistory.data(), vals.data(), count, spec);
                    }
                    varIdx++;
                }
                ImPlot::EndPlot();
            }
        }
        ImPlot::EndSubplots();
    }

    ImGui::End();
}

} // namespace CircuitSim
