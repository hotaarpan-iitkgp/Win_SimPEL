#include "ScopeWindow.hpp"
#include "imgui_internal.h"
#include "implot.h"
#include <algorithm>
#include <cmath>
#include <cfloat>

namespace CircuitSim {

static const ImVec4 SCOPE_DARK_COLORS[] = {
    ImVec4(0.00f, 0.95f, 1.00f, 1.00f),
    ImVec4(0.10f, 1.00f, 0.45f, 1.00f),
    ImVec4(1.00f, 0.88f, 0.00f, 1.00f),
    ImVec4(1.00f, 0.25f, 0.60f, 1.00f),
    ImVec4(1.00f, 0.50f, 0.10f, 1.00f),
    ImVec4(0.70f, 0.40f, 1.00f, 1.00f),
    ImVec4(0.40f, 0.90f, 1.00f, 1.00f),
    ImVec4(0.75f, 1.00f, 0.20f, 1.00f)
};

static const ImVec4 SCOPE_LIGHT_COLORS[] = {
    ImVec4(0.05f, 0.35f, 0.75f, 1.00f),
    ImVec4(0.02f, 0.50f, 0.25f, 1.00f),
    ImVec4(0.80f, 0.12f, 0.12f, 1.00f),
    ImVec4(0.50f, 0.15f, 0.75f, 1.00f),
    ImVec4(0.85f, 0.30f, 0.05f, 1.00f),
    ImVec4(0.05f, 0.50f, 0.55f, 1.00f),
    ImVec4(0.45f, 0.25f, 0.08f, 1.00f),
    ImVec4(0.12f, 0.18f, 0.28f, 1.00f)
};

ScopeWindow::ScopeWindow(const std::string& scopeCompId, int channels,
                         const std::vector<std::string>& signalKeys,
                         const std::vector<std::string>& labels)
    : scopeId(scopeCompId)
    , numChannels(channels)
    , channelSignalKeys(signalKeys)
    , channelLabels(labels)
{
    windowTitle = "Scope: " + scopeId + "###ScopeWin_" + scopeId;
    isOpen = true;
    autoFitNext = true;
    numPanes = std::max(1, numChannels); // Default: one subplot per channel
    useSubplots = (numChannels > 1);
}

void ScopeWindow::render(CircuitSimEngine::CircuitSimulator& simulator) {
    if (!isOpen) return;

    // Handle minimize/maximize sizing
    if (isMinimized) {
        ImGui::SetNextWindowSize(ImVec2(300, ImGui::GetFrameHeight() + 8.0f), ImGuiCond_Always);
    } else {
        ImGui::SetNextWindowSize(ImVec2(700, 450), ImGuiCond_FirstUseEver);
    }

    ImGuiWindowFlags flags = ImGuiWindowFlags_None;
    if (isMinimized) flags |= ImGuiWindowFlags_NoResize;

    if (!ImGui::Begin(windowTitle.c_str(), &isOpen, flags)) {
        ImGui::End();
        return;
    }

    // Render minimize/maximize buttons in the TITLE BAR (left of the X close button)
    {
        ImGuiWindow* win = ImGui::GetCurrentWindow();
        float titleBarHeight = ImGui::GetFrameHeight();
        float btnSize = titleBarHeight - 4.0f;
        float closeButtonWidth = titleBarHeight; // X button occupies ~1 frame height
        float windowWidth = ImGui::GetWindowWidth();
        float padding = ImGui::GetStyle().FramePadding.x;

        // Position buttons in title bar row, to the left of the close button
        // Close button is at rightmost. We place our buttons to its left.
        float btnX = windowWidth - closeButtonWidth - (btnSize + 4.0f) * 2 - padding;
        float btnY = win->DC.CursorPos.y - ImGui::GetCursorPosY() + 2.0f; // title bar Y

        ImVec2 savedCursor = ImGui::GetCursorPos();

        if (isMinimized) {
            // Show restore button only
            ImGui::SetCursorPos(ImVec2(btnX + btnSize + 4.0f, btnY));
            if (ImGui::InvisibleButton("##ScopeRestore", ImVec2(btnSize, btnSize))) {
                isMinimized = false;
                if (hasSavedPosSize) {
                    ImGui::SetWindowSize(savedWindowSize);
                    ImGui::SetWindowPos(savedWindowPos);
                }
            }
            // Draw restore icon (□)
            ImVec2 rMin = ImGui::GetItemRectMin();
            ImVec2 rMax = ImGui::GetItemRectMax();
            ImDrawList* dl = ImGui::GetWindowDrawList();
            dl->AddRect(ImVec2(rMin.x+3,rMin.y+3), ImVec2(rMax.x-3,rMax.y-3), 
                ImGui::IsItemHovered() ? IM_COL32(255,255,255,255) : IM_COL32(180,180,180,255), 0, 0, 1.5f);
            if (ImGui::IsItemHovered()) ImGui::SetTooltip("Restore");
        } else {
            // Minimize button
            ImGui::SetCursorPos(ImVec2(btnX, btnY));
            if (ImGui::InvisibleButton("##ScopeMin", ImVec2(btnSize, btnSize))) {
                savedWindowSize = ImGui::GetWindowSize();
                savedWindowPos = ImGui::GetWindowPos();
                hasSavedPosSize = true;
                isMinimized = true;
            }
            {
                ImVec2 rMin = ImGui::GetItemRectMin();
                ImVec2 rMax = ImGui::GetItemRectMax();
                ImDrawList* dl = ImGui::GetWindowDrawList();
                float midY = (rMin.y + rMax.y) * 0.5f + 3.0f;
                dl->AddLine(ImVec2(rMin.x+4, midY), ImVec2(rMax.x-4, midY),
                    ImGui::IsItemHovered() ? IM_COL32(255,255,255,255) : IM_COL32(180,180,180,255), 1.5f);
            }
            if (ImGui::IsItemHovered()) ImGui::SetTooltip("Minimize");

            // Maximize button
            ImGui::SetCursorPos(ImVec2(btnX + btnSize + 4.0f, btnY));
            if (ImGui::InvisibleButton("##ScopeMax", ImVec2(btnSize, btnSize))) {
                savedWindowSize = ImGui::GetWindowSize();
                savedWindowPos = ImGui::GetWindowPos();
                hasSavedPosSize = true;
                ImGuiViewport* vp = ImGui::GetMainViewport();
                ImGui::SetWindowPos(vp->Pos);
                ImGui::SetWindowSize(vp->Size);
            }
            {
                ImVec2 rMin = ImGui::GetItemRectMin();
                ImVec2 rMax = ImGui::GetItemRectMax();
                ImDrawList* dl = ImGui::GetWindowDrawList();
                dl->AddRect(ImVec2(rMin.x+3,rMin.y+3), ImVec2(rMax.x-3,rMax.y-3),
                    ImGui::IsItemHovered() ? IM_COL32(255,255,255,255) : IM_COL32(180,180,180,255), 0, 0, 1.5f);
            }
            if (ImGui::IsItemHovered()) ImGui::SetTooltip("Maximize");
        }

        ImGui::SetCursorPos(savedCursor);
    }

    if (isMinimized) {
        ImGui::End();
        return;
    }

    // Get live telemetry
    CircuitSimEngine::TelemetryData data = simulator.getTelemetryCopy();

    if (data.timeHistory.empty()) {
        ImGui::TextColored(ImVec4(0.7f, 0.7f, 0.7f, 1.0f),
            "Waiting for simulation data... Press PLAY to run simulation.");
        ImGui::End();
        return;
    }

    renderToolbar(data);
    ImGui::Separator();
    renderPlots(data);

    ImGui::End();
}

void ScopeWindow::renderToolbar(const CircuitSimEngine::TelemetryData& data) {
    bool doFit = false;
    if (ImGui::Button("Fit / Reset Zoom")) { doFit = true; autoFitNext = true; }
    ImGui::SameLine();
    ImGui::TextDisabled("|");
    ImGui::SameLine();

    // Zoom mode buttons
    if (activeZoomMode == ScopeZoomMode::Adaptive) {
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.10f, 0.45f, 0.85f, 1.0f));
        if (ImGui::Button("Adaptive##sz")) activeZoomMode = ScopeZoomMode::Disabled;
        ImGui::PopStyleColor();
    } else {
        if (ImGui::Button("Adaptive##sz")) activeZoomMode = ScopeZoomMode::Adaptive;
    }
    ImGui::SameLine();

    if (activeZoomMode == ScopeZoomMode::X_Only) {
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.00f, 0.65f, 0.85f, 1.0f));
        if (ImGui::Button("X-Zoom##sz")) activeZoomMode = ScopeZoomMode::Disabled;
        ImGui::PopStyleColor();
    } else {
        if (ImGui::Button("X-Zoom##sz")) activeZoomMode = ScopeZoomMode::X_Only;
    }
    ImGui::SameLine();

    if (activeZoomMode == ScopeZoomMode::Y_Only) {
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.70f, 0.20f, 0.80f, 1.0f));
        if (ImGui::Button("Y-Zoom##sz")) activeZoomMode = ScopeZoomMode::Disabled;
        ImGui::PopStyleColor();
    } else {
        if (ImGui::Button("Y-Zoom##sz")) activeZoomMode = ScopeZoomMode::Y_Only;
    }
    ImGui::SameLine();

    if (activeZoomMode == ScopeZoomMode::Box_2D) {
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.15f, 0.65f, 0.35f, 1.0f));
        if (ImGui::Button("Box Zoom##sz")) activeZoomMode = ScopeZoomMode::Disabled;
        ImGui::PopStyleColor();
    } else {
        if (ImGui::Button("Box Zoom##sz")) activeZoomMode = ScopeZoomMode::Box_2D;
    }
    ImGui::SameLine();

    ImGui::TextDisabled("|");
    ImGui::SameLine();

    // Subplot pane controls
    if (ImGui::Button("+Pane##sp")) {
        numPanes = std::min(numPanes + 1, MAX_PANES);
    }
    ImGui::SameLine();
    if (numPanes > 1) {
        if (ImGui::Button("-Pane##sp")) {
            numPanes = std::max(numPanes - 1, 1);
        }
        ImGui::SameLine();
    }

    ImGui::TextDisabled("|");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(90.0f);
    ImGui::SliderFloat("Width##sw", &traceLineWidth, 1.0f, 6.0f, "%.1f px");

    ImGui::SameLine();
    ImGui::TextDisabled("| %d pts | t=%.4fs | %d Ch",
        (int)data.timeHistory.size(),
        data.timeHistory.empty() ? 0.0 : data.timeHistory.back(),
        numChannels);
}

void ScopeWindow::renderPlots(const CircuitSimEngine::TelemetryData& data) {
    const double* timeData = data.timeHistory.data();
    int numPoints = (int)data.timeHistory.size();

    const auto& palette = isDarkMode ? SCOPE_DARK_COLORS : SCOPE_LIGHT_COLORS;
    constexpr size_t numColors = 8;

    bool isZoomActive = (activeZoomMode != ScopeZoomMode::Disabled);
    bool doFitThisFrame = autoFitNext;
    if (autoFitNext) autoFitNext = false;

    int renderPanes = std::min(numPanes, numChannels);
    if (renderPanes < 1) renderPanes = 1;

    // Determine layout: subplots if multiple panes, single plot otherwise
    if (renderPanes == 1) {
        // Single plot with all channels overlaid
        if (doFitThisFrame) {
            double xMin = data.timeHistory.front();
            double xMax = data.timeHistory.back();
            ImPlot::SetNextAxesLimits(xMin, xMax, -1.0, 1.0, ImGuiCond_Always);
        }
        if (pendingZoom[0].hasPending) {
            if (pendingZoom[0].type == SZ_X_ONLY)
                ImPlot::SetNextAxisLimits(ImAxis_X1, pendingZoom[0].xMin, pendingZoom[0].xMax, ImGuiCond_Always);
            else if (pendingZoom[0].type == SZ_Y_ONLY)
                ImPlot::SetNextAxisLimits(ImAxis_Y1, pendingZoom[0].yMin, pendingZoom[0].yMax, ImGuiCond_Always);
            else if (pendingZoom[0].type == SZ_BOX_2D) {
                ImPlot::SetNextAxisLimits(ImAxis_X1, pendingZoom[0].xMin, pendingZoom[0].xMax, ImGuiCond_Always);
                ImPlot::SetNextAxisLimits(ImAxis_Y1, pendingZoom[0].yMin, pendingZoom[0].yMax, ImGuiCond_Always);
            }
            pendingZoom[0].hasPending = false;
        }

        ImPlotFlags plotFlags = isZoomActive ? ImPlotFlags_NoMenus : ImPlotFlags_None;
        if (ImPlot::BeginPlot("##ScopeSingle", ImVec2(-1, -1), plotFlags)) {
            if (isZoomActive) {
                ImPlot::GetInputMap().Select = ImGuiMouseButton_Middle;
                ImPlot::GetInputMap().SelectCancel = ImGuiMouseButton_Right;
                ImPlot::GetInputMap().Pan = ImGuiMouseButton_Right;
            } else {
                ImPlot::GetInputMap().Select = ImGuiMouseButton_Right;
                ImPlot::GetInputMap().SelectCancel = ImGuiMouseButton_Left;
                ImPlot::GetInputMap().Pan = ImGuiMouseButton_Left;
            }
            ImPlot::SetupAxes("Time (s)", "Amplitude");

            if (isZoomActive) renderZoomOverlay(0);

            for (int ch = 0; ch < numChannels && ch < (int)channelSignalKeys.size(); ++ch) {
                const std::string& sigKey = channelSignalKeys[ch];
                if (sigKey.empty()) continue;
                auto it = data.voltages.find(sigKey);
                if (it == data.voltages.end()) {
                    // Try alternate keys
                    std::string alt = sigKey;
                    if (alt.size() > 4 && alt.substr(alt.size()-4) == ".Out") {
                        alt = alt.substr(0, alt.size()-4);
                        it = data.voltages.find(alt);
                    }
                    if (it == data.voltages.end()) it = data.voltages.find("V_" + sigKey);
                    if (it == data.voltages.end()) it = data.voltages.find("I_" + sigKey);
                }
                if (it != data.voltages.end() && !it->second.empty()) {
                    int pts = std::min(numPoints, (int)it->second.size());
                    ImPlotSpec spec;
                    spec.LineColor = palette[ch % numColors];
                    spec.LineWeight = traceLineWidth;
                    std::string lbl = (ch < (int)channelLabels.size()) ? channelLabels[ch] : sigKey;
                    ImPlot::PlotLine(lbl.c_str(), timeData, it->second.data(), pts, spec);
                }
            }
            ImPlot::EndPlot();
        }
    } else {

        // Multiple subplots — one per channel, linked X-axis
        if (ImPlot::BeginSubplots("##ScopeSubplots", renderPanes, 1, ImVec2(-1, -1), ImPlotSubplotFlags_LinkCols)) {
            for (int i = 0; i < renderPanes; ++i) {
                int ch = i % numChannels;

                // Apply pending zoom
                if (pendingZoom[i].hasPending) {
                    if (pendingZoom[i].type == SZ_X_ONLY)
                        ImPlot::SetNextAxisLimits(ImAxis_X1, pendingZoom[i].xMin, pendingZoom[i].xMax, ImGuiCond_Always);
                    else if (pendingZoom[i].type == SZ_Y_ONLY)
                        ImPlot::SetNextAxisLimits(ImAxis_Y1, pendingZoom[i].yMin, pendingZoom[i].yMax, ImGuiCond_Always);
                    else if (pendingZoom[i].type == SZ_BOX_2D) {
                        ImPlot::SetNextAxisLimits(ImAxis_X1, pendingZoom[i].xMin, pendingZoom[i].xMax, ImGuiCond_Always);
                        ImPlot::SetNextAxisLimits(ImAxis_Y1, pendingZoom[i].yMin, pendingZoom[i].yMax, ImGuiCond_Always);
                    }
                    pendingZoom[i].hasPending = false;
                } else if (doFitThisFrame) {
                    double xMin = data.timeHistory.front();
                    double xMax = data.timeHistory.back();
                    ImPlot::SetNextAxesLimits(xMin, xMax, -1.0, 1.0, ImGuiCond_Always);
                }

                std::string plotTitle = (ch < (int)channelLabels.size()) ? channelLabels[ch] : ("Ch" + std::to_string(ch+1));
                ImPlotFlags pflags = isZoomActive ? ImPlotFlags_NoMenus : ImPlotFlags_None;

                if (ImPlot::BeginPlot(plotTitle.c_str(), ImVec2(-1, -1), pflags)) {
                    if (isZoomActive) {
                        ImPlot::GetInputMap().Select = ImGuiMouseButton_Middle;
                        ImPlot::GetInputMap().SelectCancel = ImGuiMouseButton_Right;
                        ImPlot::GetInputMap().Pan = ImGuiMouseButton_Right;
                    } else {
                        ImPlot::GetInputMap().Select = ImGuiMouseButton_Right;
                        ImPlot::GetInputMap().SelectCancel = ImGuiMouseButton_Left;
                        ImPlot::GetInputMap().Pan = ImGuiMouseButton_Left;
                    }
                    ImPlot::SetupAxes("Time (s)", "Amplitude");

                    if (isZoomActive) renderZoomOverlay(i);

                    const std::string& sigKey = (ch < (int)channelSignalKeys.size()) ? channelSignalKeys[ch] : "";
                    if (!sigKey.empty()) {
                        auto it = data.voltages.find(sigKey);
                        if (it == data.voltages.end()) {
                            std::string alt = sigKey;
                            if (alt.size() > 4 && alt.substr(alt.size()-4) == ".Out") {
                                alt = alt.substr(0, alt.size()-4);
                                it = data.voltages.find(alt);
                            }
                            if (it == data.voltages.end()) it = data.voltages.find("V_" + sigKey);
                            if (it == data.voltages.end()) it = data.voltages.find("I_" + sigKey);
                        }
                        if (it != data.voltages.end() && !it->second.empty()) {
                            int pts = std::min(numPoints, (int)it->second.size());
                            ImPlotSpec spec;
                            spec.LineColor = palette[ch % numColors];
                            spec.LineWeight = traceLineWidth;
                            std::string lbl = (ch < (int)channelLabels.size()) ? channelLabels[ch] : sigKey;
                            ImPlot::PlotLine(lbl.c_str(), timeData, it->second.data(), pts, spec);
                        }
                    }
                    ImPlot::EndPlot();
                }
            }
            ImPlot::EndSubplots();
        }
    }
}

void ScopeWindow::renderZoomOverlay(int paneIdx) {
    bool isHovered = ImPlot::IsPlotHovered();
    bool isMouseDown = ImGui::IsMouseDown(ImGuiMouseButton_Left);
    bool isMouseReleased = ImGui::IsMouseReleased(ImGuiMouseButton_Left);
    ImVec2 mousePx = ImGui::GetMousePos();

    // Start drag
    if (isHovered && ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
        customDragState[paneIdx].isDragging = true;
        customDragState[paneIdx].startPt = ImPlot::GetPlotMousePos();
        customDragState[paneIdx].startPx = mousePx;
    }

    // Active drag: render visual
    if (customDragState[paneIdx].isDragging && isMouseDown) {
        customDragState[paneIdx].currentPt = ImPlot::GetPlotMousePos();
        customDragState[paneIdx].currentPx = mousePx;

        ImVec2 pStart = customDragState[paneIdx].startPx;
        ImVec2 pCurr = customDragState[paneIdx].currentPx;

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

        ScopeZoomType currentDragType = SZ_BOX_2D;
        if (activeZoomMode == ScopeZoomMode::X_Only) currentDragType = SZ_X_ONLY;
        else if (activeZoomMode == ScopeZoomMode::Y_Only) currentDragType = SZ_Y_ONLY;
        else if (activeZoomMode == ScopeZoomMode::Box_2D) currentDragType = SZ_BOX_2D;
        else if (activeZoomMode == ScopeZoomMode::Adaptive) {
            if (dyPx <= 0.10f * dxPx || dyPx <= 12.0f) currentDragType = SZ_X_ONLY;
            else if (dxPx <= 0.10f * dyPx || dxPx <= 12.0f) currentDragType = SZ_Y_ONLY;
            else currentDragType = SZ_BOX_2D;
        }

        if (dxPx > 3.0f || dyPx > 3.0f) {
            ImDrawList* dl = ImPlot::GetPlotDrawList();
            if (currentDragType == SZ_X_ONLY) {
                dl->AddRectFilled(ImVec2(x1, pTop), ImVec2(x2, pBottom), IM_COL32(0, 220, 255, 40));
                dl->AddLine(ImVec2(x1, pTop), ImVec2(x1, pBottom), IM_COL32(0, 220, 255, 255), 2.0f);
                dl->AddLine(ImVec2(x2, pTop), ImVec2(x2, pBottom), IM_COL32(0, 220, 255, 255), 2.0f);
            } else if (currentDragType == SZ_Y_ONLY) {
                dl->AddRectFilled(ImVec2(pLeft, y1), ImVec2(pRight, y2), IM_COL32(220, 0, 255, 40));
                dl->AddLine(ImVec2(pLeft, y1), ImVec2(pRight, y1), IM_COL32(220, 0, 255, 255), 2.0f);
                dl->AddLine(ImVec2(pLeft, y2), ImVec2(pRight, y2), IM_COL32(220, 0, 255, 255), 2.0f);
            } else {
                dl->AddRectFilled(ImVec2(x1, y1), ImVec2(x2, y2), IM_COL32(60, 255, 120, 40));
                dl->AddRect(ImVec2(x1, y1), ImVec2(x2, y2), IM_COL32(60, 255, 120, 255), 0, 0, 2.0f);
            }
        }
    }

    // Release: commit zoom
    if (customDragState[paneIdx].isDragging && isMouseReleased) {
        customDragState[paneIdx].isDragging = false;
        customDragState[paneIdx].currentPt = ImPlot::GetPlotMousePos();
        customDragState[paneIdx].currentPx = mousePx;

        float dxPx = std::abs(customDragState[paneIdx].currentPx.x - customDragState[paneIdx].startPx.x);
        float dyPx = std::abs(customDragState[paneIdx].currentPx.y - customDragState[paneIdx].startPx.y);

        if (dxPx > 5.0f || dyPx > 5.0f) {
            ScopeZoomType finalType = SZ_BOX_2D;
            if (activeZoomMode == ScopeZoomMode::X_Only) finalType = SZ_X_ONLY;
            else if (activeZoomMode == ScopeZoomMode::Y_Only) finalType = SZ_Y_ONLY;
            else if (activeZoomMode == ScopeZoomMode::Box_2D) finalType = SZ_BOX_2D;
            else if (activeZoomMode == ScopeZoomMode::Adaptive) {
                if (dyPx <= 0.10f * dxPx || dyPx <= 12.0f) finalType = SZ_X_ONLY;
                else if (dxPx <= 0.10f * dyPx || dxPx <= 12.0f) finalType = SZ_Y_ONLY;
                else finalType = SZ_BOX_2D;
            }

            pendingZoom[paneIdx].type = finalType;
            pendingZoom[paneIdx].xMin = std::min(customDragState[paneIdx].startPt.x, customDragState[paneIdx].currentPt.x);
            pendingZoom[paneIdx].xMax = std::max(customDragState[paneIdx].startPt.x, customDragState[paneIdx].currentPt.x);
            pendingZoom[paneIdx].yMin = std::min(customDragState[paneIdx].startPt.y, customDragState[paneIdx].currentPt.y);
            pendingZoom[paneIdx].yMax = std::max(customDragState[paneIdx].startPt.y, customDragState[paneIdx].currentPt.y);
            pendingZoom[paneIdx].hasPending = true;
        }
    }
}

} // namespace CircuitSim
