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

    ImGui::TextDisabled("|");
    ImGui::SameLine();

    // Dedicated Zoom Mode Buttons
    if (activeZoomMode == ActiveZoomMode::Adaptive) {
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.10f, 0.45f, 0.85f, 1.0f));
        if (ImGui::Button("🔍 Adaptive")) activeZoomMode = ActiveZoomMode::Disabled;
        ImGui::PopStyleColor();
    } else {
        if (ImGui::Button("🔍 Adaptive")) activeZoomMode = ActiveZoomMode::Adaptive;
    }
    ImGui::SameLine();

    if (activeZoomMode == ActiveZoomMode::X_Only) {
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.00f, 0.65f, 0.85f, 1.0f));
        if (ImGui::Button("↔ X-Axis Zoom")) activeZoomMode = ActiveZoomMode::Disabled;
        ImGui::PopStyleColor();
    } else {
        if (ImGui::Button("↔ X-Axis Zoom")) activeZoomMode = ActiveZoomMode::X_Only;
    }
    ImGui::SameLine();

    if (activeZoomMode == ActiveZoomMode::Y_Only) {
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.70f, 0.20f, 0.80f, 1.0f));
        if (ImGui::Button("↕ Y-Axis Zoom")) activeZoomMode = ActiveZoomMode::Disabled;
        ImGui::PopStyleColor();
    } else {
        if (ImGui::Button("↕ Y-Axis Zoom")) activeZoomMode = ActiveZoomMode::Y_Only;
    }
    ImGui::SameLine();

    if (activeZoomMode == ActiveZoomMode::Box_2D) {
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.15f, 0.65f, 0.35f, 1.0f));
        if (ImGui::Button("⤢ Box Zoom")) activeZoomMode = ActiveZoomMode::Disabled;
        ImGui::PopStyleColor();
    } else {
        if (ImGui::Button("⤢ Box Zoom")) activeZoomMode = ActiveZoomMode::Box_2D;
    }
    ImGui::SameLine();

    ImGui::TextDisabled("|");
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

    ImGui::Separator();

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

    bool isZoomActive = (activeZoomMode != ActiveZoomMode::Disabled);

    if (ImPlot::BeginSubplots("Oscilloscope Subplots", renderPanes, 1, ImVec2(-1, -1), ImPlotSubplotFlags_LinkCols)) {
        for (int i = 0; i < renderPanes; ++i) {
            const auto& cat = categories[i % categories.size()];

            // Pending zoom from previous frame (before BeginPlot)
            if (pendingZoom[i].hasPending) {
                if (pendingZoom[i].type == ZOOM_X_ONLY) {
                    ImPlot::SetNextAxisLimits(ImAxis_X1, pendingZoom[i].xMin, pendingZoom[i].xMax, ImGuiCond_Always);
                } else if (pendingZoom[i].type == ZOOM_Y_ONLY) {
                    ImPlot::SetNextAxisLimits(ImAxis_Y1, pendingZoom[i].yMin, pendingZoom[i].yMax, ImGuiCond_Always);
                } else if (pendingZoom[i].type == ZOOM_BOX_2D) {
                    ImPlot::SetNextAxisLimits(ImAxis_X1, pendingZoom[i].xMin, pendingZoom[i].xMax, ImGuiCond_Always);
                    ImPlot::SetNextAxisLimits(ImAxis_Y1, pendingZoom[i].yMin, pendingZoom[i].yMax, ImGuiCond_Always);
                }
                pendingZoom[i].hasPending = false;
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

            if (isZoomActive) {
                ImPlot::PushStyleColor(ImPlotCol_Selection, ImVec4(0.0f, 0.0f, 0.0f, 0.0f));
            }

            if (ImPlot::BeginPlot(cat.title.c_str(), ImVec2(-1, -1),
                                   isZoomActive ? ImPlotFlags_NoMenus : ImPlotFlags_None)) {

                // Override mouse button bindings based on zoom mode
                if (isZoomActive) {
                    ImPlot::GetInputMap().Select       = ImGuiMouseButton_Left;  // LMB = rubber-band
                    ImPlot::GetInputMap().SelectCancel = ImGuiMouseButton_Right;
                    ImPlot::GetInputMap().Pan          = ImGuiMouseButton_Right; // RMB = pan
                } else {
                    ImPlot::GetInputMap().Select       = ImGuiMouseButton_Right;
                    ImPlot::GetInputMap().SelectCancel = ImGuiMouseButton_Left;
                    ImPlot::GetInputMap().Pan          = ImGuiMouseButton_Left;
                }

                ImPlot::SetupAxes("Time (s)", cat.yLabel.c_str());

                bool isMouseDown = ImGui::IsMouseDown(ImGuiMouseButton_Left);
                bool isMouseReleased = ImGui::IsMouseReleased(ImGuiMouseButton_Left);

                // Box / X / Y Zoom rubber-band and release commit
                if (isZoomActive && (ImPlot::IsPlotSelected() || isMouseDown || isMouseReleased)) {
                    ImPlotRect sel = ImPlot::GetPlotSelection();

                    // Convert selection coordinates to screen pixels
                    ImVec2 pMin = ImPlot::PlotToPixels(ImPlotPoint(sel.X.Min, sel.Y.Max));
                    ImVec2 pMax = ImPlot::PlotToPixels(ImPlotPoint(sel.X.Max, sel.Y.Min));

                    ImVec2 plotPos = ImPlot::GetPlotPos();
                    ImVec2 plotSize = ImPlot::GetPlotSize();
                    float pLeft = plotPos.x;
                    float pRight = plotPos.x + plotSize.x;
                    float pTop = plotPos.y;
                    float pBottom = plotPos.y + plotSize.y;

                    float x1 = std::min(pMin.x, pMax.x);
                    float x2 = std::max(pMin.x, pMax.x);
                    float y1 = std::min(pMin.y, pMax.y);
                    float y2 = std::max(pMin.y, pMax.y);

                    float dxPx = x2 - x1;
                    float dyPx = y2 - y1;

                    WaveformZoomType currentDragType = ZOOM_BOX_2D;
                    if (activeZoomMode == ActiveZoomMode::X_Only) {
                        currentDragType = ZOOM_X_ONLY;
                    } else if (activeZoomMode == ActiveZoomMode::Y_Only) {
                        currentDragType = ZOOM_Y_ONLY;
                    } else if (activeZoomMode == ActiveZoomMode::Box_2D) {
                        currentDragType = ZOOM_BOX_2D;
                    } else if (activeZoomMode == ActiveZoomMode::Adaptive) {
                        if (dyPx <= 0.10f * dxPx || dyPx <= 12.0f) {
                            currentDragType = ZOOM_X_ONLY;
                        } else if (dxPx <= 0.10f * dyPx || dxPx <= 12.0f) {
                            currentDragType = ZOOM_Y_ONLY;
                        } else {
                            currentDragType = ZOOM_BOX_2D;
                        }
                    }

                    // Only render visual rubber-band while mouse is actively dragging
                    if (isMouseDown && (dxPx > 3.0f || dyPx > 3.0f)) {
                        ImDrawList* drawList = ImPlot::GetPlotDrawList();

                        if (currentDragType == ZOOM_X_ONLY) {
                            // --- X-AXIS ZOOM: HORIZONTAL SPAN BAND ---
                            drawList->AddRectFilled(ImVec2(x1, pTop), ImVec2(x2, pBottom), IM_COL32(0, 220, 255, 35));
                            drawList->AddLine(ImVec2(x1, pTop), ImVec2(x1, pBottom), IM_COL32(0, 220, 255, 255), 2.0f);
                            drawList->AddLine(ImVec2(x2, pTop), ImVec2(x2, pBottom), IM_COL32(0, 220, 255, 255), 2.0f);

                            const char* tag = " [ ↔ X-Zoom (Time Only) ] ";
                            ImVec2 txtSz = ImGui::CalcTextSize(tag);
                            float midX = (x1 + x2) * 0.5f;
                            drawList->AddRectFilled(ImVec2(midX - txtSz.x * 0.5f - 4, pTop + 6), ImVec2(midX + txtSz.x * 0.5f + 4, pTop + 6 + txtSz.y + 2), IM_COL32(0, 150, 200, 230), 4.0f);
                            drawList->AddText(ImVec2(midX - txtSz.x * 0.5f, pTop + 7), IM_COL32(255, 255, 255, 255), tag);
                        } else if (currentDragType == ZOOM_Y_ONLY) {
                            // --- Y-AXIS ZOOM: VERTICAL SPAN BAND ---
                            drawList->AddRectFilled(ImVec2(pLeft, y1), ImVec2(pRight, y2), IM_COL32(220, 0, 255, 35));
                            drawList->AddLine(ImVec2(pLeft, y1), ImVec2(pRight, y1), IM_COL32(220, 0, 255, 255), 2.0f);
                            drawList->AddLine(ImVec2(pLeft, y2), ImVec2(pRight, y2), IM_COL32(220, 0, 255, 255), 2.0f);

                            const char* tag = " [ ↕ Y-Zoom (Amp Only) ] ";
                            ImVec2 txtSz = ImGui::CalcTextSize(tag);
                            float midY = (y1 + y2) * 0.5f;
                            drawList->AddRectFilled(ImVec2(pLeft + 6, midY - txtSz.y * 0.5f - 2), ImVec2(pLeft + 6 + txtSz.x + 8, midY + txtSz.y * 0.5f + 2), IM_COL32(160, 0, 180, 230), 4.0f);
                            drawList->AddText(ImVec2(pLeft + 10, midY - txtSz.y * 0.5f), IM_COL32(255, 255, 255, 255), tag);
                        } else {
                            // --- 2D BOX ZOOM: RECTANGLE ---
                            drawList->AddRectFilled(ImVec2(x1, y1), ImVec2(x2, y2), IM_COL32(60, 255, 120, 35));
                            drawList->AddRect(ImVec2(x1, y1), ImVec2(x2, y2), IM_COL32(60, 255, 120, 255), 0, 0, 2.0f);

                            const char* tag = " [ ⤢ 2D Box Zoom ] ";
                            ImVec2 txtSz = ImGui::CalcTextSize(tag);
                            drawList->AddRectFilled(ImVec2(x1 + 4, y1 + 4), ImVec2(x1 + 12 + txtSz.x, y1 + 6 + txtSz.y), IM_COL32(30, 160, 80, 230), 4.0f);
                            drawList->AddText(ImVec2(x1 + 8, y1 + 5), IM_COL32(255, 255, 255, 255), tag);
                        }
                    }

                    // END CONDITION COMMIT: Execute zoom based on the exact classification at mouse release!
                    if (isMouseReleased && (dxPx > 5.0f || dyPx > 5.0f)) {
                        ImPlot::CancelPlotSelection();

                        pendingZoom[i].type = currentDragType;
                        pendingZoom[i].xMin = sel.X.Min; pendingZoom[i].xMax = sel.X.Max;
                        pendingZoom[i].yMin = sel.Y.Min; pendingZoom[i].yMax = sel.Y.Max;
                        pendingZoom[i].hasPending = true;
                    }
                }

                // Right-Click Context Menu
                if (!isZoomActive && ImGui::BeginPopupContextItem("PlotContextMenu")) {
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
            if (isZoomActive) {
                ImPlot::PopStyleColor();
            }
        }
        ImPlot::EndSubplots();
    }

    ImGui::End();
}

} // namespace CircuitSim
