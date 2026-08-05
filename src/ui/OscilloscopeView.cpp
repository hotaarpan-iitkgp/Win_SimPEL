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
            if (ImPlot::BeginPlot(cat.title.c_str())) {
                ImPlot::SetupAxes("Time (s)", cat.yLabel.c_str());

                // Adaptive Box Zoom (PLECS / Plotly style)
                if (isAdaptiveZoomEnabled && ImPlot::IsPlotSelected()) {
                    ImPlotRect selectRect = ImPlot::GetPlotSelection();
                    ImPlotRect currentLimits = ImPlot::GetPlotLimits();
                    ImPlot::CancelPlotSelection();

                    double dxPlot = std::abs(selectRect.X.Max - selectRect.X.Min);
                    double dyPlot = std::abs(selectRect.Y.Max - selectRect.Y.Min);

                    double currentDx = std::abs(currentLimits.X.Max - currentLimits.X.Min);
                    double currentDy = std::abs(currentLimits.Y.Max - currentLimits.Y.Min);

                    double normDx = dxPlot / (currentDx > 1e-15 ? currentDx : 1e-15);
                    double normDy = dyPlot / (currentDy > 1e-15 ? currentDy : 1e-15);

                    double newXMin = currentLimits.X.Min;
                    double newXMax = currentLimits.X.Max;
                    double newYMin = currentLimits.Y.Min;
                    double newYMax = currentLimits.Y.Max;

                    if (normDx > 2.0 * normDy) {
                        newXMin = selectRect.X.Min;
                        newXMax = selectRect.X.Max;
                    } else if (normDy > 2.0 * normDx) {
                        newYMin = selectRect.Y.Min;
                        newYMax = selectRect.Y.Max;
                    } else {
                        newXMin = selectRect.X.Min;
                        newXMax = selectRect.X.Max;
                        newYMin = selectRect.Y.Min;
                        newYMax = selectRect.Y.Max;
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
