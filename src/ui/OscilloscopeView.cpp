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

            if (ImPlot::BeginPlot(cat.title.c_str(), ImVec2(-1, -1),
                                   isZoomActive ? ImPlotFlags_NoMenus : ImPlotFlags_None)) {

                // Assign Select to Middle Mouse Button (valid index 2) to avoid ImGui IM_ASSERT(button >= 0 && button < 5) crash
                if (isZoomActive) {
                    ImPlot::GetInputMap().Select       = ImGuiMouseButton_Middle;
                    ImPlot::GetInputMap().SelectCancel = ImGuiMouseButton_Right;
                    ImPlot::GetInputMap().Pan          = ImGuiMouseButton_Right; // RMB = Pan
                } else {
                    ImPlot::GetInputMap().Select       = ImGuiMouseButton_Right;
                    ImPlot::GetInputMap().SelectCancel = ImGuiMouseButton_Left;
                    ImPlot::GetInputMap().Pan          = ImGuiMouseButton_Left;
                }

                ImPlot::SetupAxes("Time (s)", cat.yLabel.c_str());

                // --- DEDICATED SEPARATE ZOOM MODULE (Bypasses ImPlot 2D Box engine) ---
                if (isZoomActive) {
                    bool isHovered = ImPlot::IsPlotHovered();
                    bool isMouseDown = ImGui::IsMouseDown(ImGuiMouseButton_Left);
                    bool isMouseReleased = ImGui::IsMouseReleased(ImGuiMouseButton_Left);
                    ImVec2 mousePx = ImGui::GetMousePos();

                    // 1. Start custom drag gesture
                    if (isHovered && ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
                        customDragState[i].isDragging = true;
                        customDragState[i].startPt = ImPlot::GetPlotMousePos();
                        customDragState[i].startPx = mousePx;
                    }

                    // 2. Active custom drag gesture: render visual selection area
                    if (customDragState[i].isDragging && isMouseDown) {
                        customDragState[i].currentPt = ImPlot::GetPlotMousePos();
                        customDragState[i].currentPx = mousePx;

                        ImVec2 pStart = customDragState[i].startPx;
                        ImVec2 pCurr = customDragState[i].currentPx;

                        ImVec2 plotPos = ImPlot::GetPlotPos();
                        ImVec2 plotSize = ImPlot::GetPlotSize();
                        float pLeft = plotPos.x;
                        float pRight = plotPos.x + plotSize.x;
                        float pTop = plotPos.y;
                        float pBottom = plotPos.y + plotSize.y;

                        float x1 = std::min(pStart.x, pCurr.x);
                        float x2 = std::max(pStart.x, pCurr.x);
                        float y1 = std::min(pStart.y, pCurr.y);
                        float y2 = std::max(pStart.y, pCurr.y);

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

                        if (dxPx > 3.0f || dyPx > 3.0f) {
                            ImDrawList* drawList = ImPlot::GetPlotDrawList();

                            if (currentDragType == ZOOM_X_ONLY) {
                                // --- PURE X-AXIS SELECTION AREA (FULL HEIGHT CYAN BAND) ---
                                drawList->AddRectFilled(ImVec2(x1, pTop), ImVec2(x2, pBottom), IM_COL32(0, 220, 255, 40));
                                drawList->AddLine(ImVec2(x1, pTop), ImVec2(x1, pBottom), IM_COL32(0, 220, 255, 255), 2.0f);
                                drawList->AddLine(ImVec2(x2, pTop), ImVec2(x2, pBottom), IM_COL32(0, 220, 255, 255), 2.0f);

                                const char* tag = " [ ↔ X-Axis Zoom (Time Only) ] ";
                                ImVec2 txtSz = ImGui::CalcTextSize(tag);
                                float midX = (x1 + x2) * 0.5f;
                                drawList->AddRectFilled(ImVec2(midX - txtSz.x * 0.5f - 4, pTop + 6), ImVec2(midX + txtSz.x * 0.5f + 4, pTop + 6 + txtSz.y + 2), IM_COL32(0, 150, 200, 230), 4.0f);
                                drawList->AddText(ImVec2(midX - txtSz.x * 0.5f, pTop + 7), IM_COL32(255, 255, 255, 255), tag);
                            } else if (currentDragType == ZOOM_Y_ONLY) {
                                // --- PURE Y-AXIS SELECTION AREA (FULL WIDTH MAGENTA BAND) ---
                                drawList->AddRectFilled(ImVec2(pLeft, y1), ImVec2(pRight, y2), IM_COL32(220, 0, 255, 40));
                                drawList->AddLine(ImVec2(pLeft, y1), ImVec2(pRight, y1), IM_COL32(220, 0, 255, 255), 2.0f);
                                drawList->AddLine(ImVec2(pLeft, y2), ImVec2(pRight, y2), IM_COL32(220, 0, 255, 255), 2.0f);

                                const char* tag = " [ ↕ Y-Axis Zoom (Amp Only) ] ";
                                ImVec2 txtSz = ImGui::CalcTextSize(tag);
                                float midY = (y1 + y2) * 0.5f;
                                drawList->AddRectFilled(ImVec2(pLeft + 6, midY - txtSz.y * 0.5f - 2), ImVec2(pLeft + 6 + txtSz.x + 8, midY + txtSz.y * 0.5f + 2), IM_COL32(160, 0, 180, 230), 4.0f);
                                drawList->AddText(ImVec2(pLeft + 10, midY - txtSz.y * 0.5f), IM_COL32(255, 255, 255, 255), tag);
                            } else {
                                // --- 2D BOX SELECTION AREA ---
                                drawList->AddRectFilled(ImVec2(x1, y1), ImVec2(x2, y2), IM_COL32(60, 255, 120, 40));
                                drawList->AddRect(ImVec2(x1, y1), ImVec2(x2, y2), IM_COL32(60, 255, 120, 255), 0, 0, 2.0f);

                                const char* tag = " [ ⤢ 2D Box Zoom ] ";
                                ImVec2 txtSz = ImGui::CalcTextSize(tag);
                                drawList->AddRectFilled(ImVec2(x1 + 4, y1 + 4), ImVec2(x1 + 12 + txtSz.x, y1 + 6 + txtSz.y), IM_COL32(30, 160, 80, 230), 4.0f);
                                drawList->AddText(ImVec2(x1 + 8, y1 + 5), IM_COL32(255, 255, 255, 255), tag);
                            }
                        }
                    }

                    // 3. Mouse release: commit zoom limits from custom module
                    if (customDragState[i].isDragging && isMouseReleased) {
                        customDragState[i].isDragging = false;
                        customDragState[i].currentPt = ImPlot::GetPlotMousePos();
                        customDragState[i].currentPx = mousePx;

                        float dxPx = std::abs(customDragState[i].currentPx.x - customDragState[i].startPx.x);
                        float dyPx = std::abs(customDragState[i].currentPx.y - customDragState[i].startPx.y);

                        if (dxPx > 5.0f || dyPx > 5.0f) {
                            WaveformZoomType finalZoomType = ZOOM_BOX_2D;
                            if (activeZoomMode == ActiveZoomMode::X_Only) {
                                finalZoomType = ZOOM_X_ONLY;
                            } else if (activeZoomMode == ActiveZoomMode::Y_Only) {
                                finalZoomType = ZOOM_Y_ONLY;
                            } else if (activeZoomMode == ActiveZoomMode::Box_2D) {
                                finalZoomType = ZOOM_BOX_2D;
                            } else if (activeZoomMode == ActiveZoomMode::Adaptive) {
                                if (dyPx <= 0.10f * dxPx || dyPx <= 12.0f) {
                                    finalZoomType = ZOOM_X_ONLY;
                                } else if (dxPx <= 0.10f * dyPx || dxPx <= 12.0f) {
                                    finalZoomType = ZOOM_Y_ONLY;
                                } else {
                                    finalZoomType = ZOOM_BOX_2D;
                                }
                            }

                            pendingZoom[i].type = finalZoomType;
                            pendingZoom[i].xMin = std::min(customDragState[i].startPt.x, customDragState[i].currentPt.x);
                            pendingZoom[i].xMax = std::max(customDragState[i].startPt.x, customDragState[i].currentPt.x);
                            pendingZoom[i].yMin = std::min(customDragState[i].startPt.y, customDragState[i].currentPt.y);
                            pendingZoom[i].yMax = std::max(customDragState[i].startPt.y, customDragState[i].currentPt.y);
                            pendingZoom[i].hasPending = true;
                        }
                    }
                }

                // Right-Click Context Menu (only when zoom mode is off)
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
        }
        ImPlot::EndSubplots();
    }

    ImGui::End();
}

} // namespace CircuitSim
