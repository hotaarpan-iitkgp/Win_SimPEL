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
    if (autoFitNext) {
        autoFitNext = false;
    }

    if (ImGui::Button("Fit Waveforms / Reset Zoom") || doFitThisFrame) {
        ImPlot::SetNextAxesToFit();
    }
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
    ImGui::TextColored(ImVec4(0.6f, 0.6f, 0.6f, 1.0f), "| Tip: Right-click plot to add/remove subplots, drag box to zoom.");

    int renderPanes = std::min(numPanes, (int)categories.size());
    if (renderPanes < 1) renderPanes = 1;

    if (ImPlot::BeginSubplots("Oscilloscope Subplots", renderPanes, 1, ImVec2(-1, -1), ImPlotSubplotFlags_LinkCols)) {
        for (int i = 0; i < renderPanes; ++i) {
            const auto& cat = categories[i % categories.size()];
            if (doFitThisFrame) {
                ImPlot::SetNextAxesToFit();
            }
            // Configure ImPlot mouse bindings based on zoom mode:
            //   Adaptive zoom ON  → LMB drag = box select, RMB drag = pan
            //   Adaptive zoom OFF → LMB drag = pan (ImPlot default), RMB = context menu

            if (ImPlot::BeginPlot(cat.title.c_str(), ImVec2(-1, -1),
                                   isAdaptiveZoomEnabled ? ImPlotFlags_NoMenus : ImPlotFlags_None)) {

                // Override mouse button bindings when adaptive zoom is on
                if (isAdaptiveZoomEnabled) {
                    ImPlot::GetInputMap().Select       = ImGuiMouseButton_Left;   // LMB = draw selection box
                    ImPlot::GetInputMap().SelectCancel = ImGuiMouseButton_Right;
                    ImPlot::GetInputMap().Pan          = ImGuiMouseButton_Right;  // RMB = pan
                } else {
                    // Restore defaults
                    ImPlot::GetInputMap().Select       = ImGuiMouseButton_Right;
                    ImPlot::GetInputMap().SelectCancel = ImGuiMouseButton_Left;
                    ImPlot::GetInputMap().Pan          = ImGuiMouseButton_Left;
                }

                ImPlot::SetupAxes("Time (s)", cat.yLabel.c_str());

                // Adaptive Box Zoom (PLECS / Plotly style) — commit on selection release
                if (isAdaptiveZoomEnabled && ImPlot::IsPlotSelected()) {
                    ImPlotRect sel = ImPlot::GetPlotSelection();
                    ImPlotRect cur = ImPlot::GetPlotLimits();
                    ImPlot::CancelPlotSelection();

                    double dxSel = std::abs(sel.X.Max - sel.X.Min);
                    double dySel = std::abs(sel.Y.Max - sel.Y.Min);
                    double dxCur = std::abs(cur.X.Max - cur.X.Min);
                    double dyCur = std::abs(cur.Y.Max - cur.Y.Min);

                    // Normalize drag dimensions against current view to detect intent
                    double nx = (dxCur > 1e-15) ? dxSel / dxCur : 0.0;
                    double ny = (dyCur > 1e-15) ? dySel / dyCur : 0.0;

                    double newXMin = cur.X.Min, newXMax = cur.X.Max;
                    double newYMin = cur.Y.Min, newYMax = cur.Y.Max;

                    if (nx > 2.5 * ny) {
                        // Wide horizontal drag → X-axis zoom only (most common waveform use)
                        newXMin = sel.X.Min;
                        newXMax = sel.X.Max;
                    } else if (ny > 2.5 * nx) {
                        // Tall vertical drag → Y-axis zoom only
                        newYMin = sel.Y.Min;
                        newYMax = sel.Y.Max;
                    } else {
                        // Roughly square → full box zoom
                        newXMin = sel.X.Min; newXMax = sel.X.Max;
                        newYMin = sel.Y.Min; newYMax = sel.Y.Max;
                    }

                    ImPlot::SetNextAxesLimits(newXMin, newXMax, newYMin, newYMax, ImGuiCond_Always);
                }


                // Right-Click Context Menu on Plot itself
                if (ImGui::BeginPopupContextItem("PlotContextMenu")) {
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

                for (const auto& varPair : cat.variables) {
                    const std::string& varName = varPair.first;
                    const std::vector<double>& vals = varPair.second;
                    int count = (int)std::min(data.timeHistory.size(), vals.size());
                    if (count > 0) {
                        ImPlot::PlotLine(varName.c_str(), data.timeHistory.data(), vals.data(), count);
                    }
                }
                ImPlot::EndPlot();
            }
        }
        ImPlot::EndSubplots();
    }

    ImGui::End();
}

} // namespace CircuitSim
