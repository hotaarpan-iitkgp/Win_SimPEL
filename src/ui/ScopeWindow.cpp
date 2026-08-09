#include "ScopeWindow.hpp"
#include "engine/SignalAnalysis.hpp"
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

    ImGuiWindowFlags flags = ImGuiWindowFlags_NoDocking; // Force separate OS window via viewports
    if (isMinimized) flags |= ImGuiWindowFlags_NoResize;

    if (!ImGui::Begin(windowTitle.c_str(), &isOpen, flags)) {
        ImGui::End();
        return;
    }

    // Render minimize & maximize buttons directly in the TITLE BAR (left of the 'X' close button)
    {
        ImGuiWindow* win = ImGui::GetCurrentWindow();
        float titleBarHeight = win->TitleBarHeight;
        if (titleBarHeight > 0.0f) {
            float btnW = 18.0f;
            float btnH = 18.0f;
            float spacing = 3.0f;
            float rightMargin = 28.0f; // distance from right edge to place before close X button

            float startX = win->Pos.x + win->Size.x - rightMargin - (btnW * 2 + spacing);
            float startY = win->Pos.y + (titleBarHeight - btnH) * 0.5f;

            ImVec2 savedCursor = ImGui::GetCursorPos();

            // Push clip rect covering the title bar so buttons are not clipped by InnerClipRect
            ImGui::PushClipRect(win->Pos, ImVec2(win->Pos.x + win->Size.x, win->Pos.y + titleBarHeight), false);

            ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(0, 0));
            ImGui::PushStyleVar(ImGuiStyleVar_FrameRounding, 3.0f);
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.20f, 0.25f, 0.35f, 0.85f));
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.35f, 0.45f, 0.65f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(0.15f, 0.35f, 0.75f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.95f, 0.95f, 0.95f, 1.0f));

            // 1. MINIMIZE BUTTON (-)
            ImGui::SetCursorScreenPos(ImVec2(startX, startY));
            if (ImGui::Button("-##ScopeMinBtn", ImVec2(btnW, btnH))) {
                if (isMinimized) {
                    isMinimized = false;
                    if (hasSavedPosSize) {
                        ImGui::SetWindowSize(savedWindowSize);
                        ImGui::SetWindowPos(savedWindowPos);
                    }
                } else {
                    savedWindowSize = ImGui::GetWindowSize();
                    savedWindowPos = ImGui::GetWindowPos();
                    hasSavedPosSize = true;
                    isMinimized = true;
                }
            }
            if (ImGui::IsItemHovered()) ImGui::SetTooltip(isMinimized ? "Restore Window" : "Minimize Window");

            // 2. MAXIMIZE / RESTORE BUTTON (+ / ❐)
            ImGui::SetCursorScreenPos(ImVec2(startX + btnW + spacing, startY));
            const char* maxSymbol = isMaximized ? "❐" : "+";
            if (ImGui::Button((std::string(maxSymbol) + "##ScopeMaxBtn").c_str(), ImVec2(btnW, btnH))) {
                if (isMaximized) {
                    isMaximized = false;
                    if (hasSavedPosSize) {
                        ImGui::SetWindowSize(savedWindowSize);
                        ImGui::SetWindowPos(savedWindowPos);
                    }
                } else {
                    if (!isMinimized) {
                        savedWindowSize = ImGui::GetWindowSize();
                        savedWindowPos = ImGui::GetWindowPos();
                        hasSavedPosSize = true;
                    }
                    isMinimized = false;
                    isMaximized = true;

                    ImGuiViewport* vp = ImGui::GetMainViewport();
                    ImGui::SetWindowPos(vp->WorkPos);
                    ImGui::SetWindowSize(vp->WorkSize);
                }
            }
            if (ImGui::IsItemHovered()) ImGui::SetTooltip(isMaximized ? "Restore Down" : "Maximize Window");

            ImGui::PopStyleColor(4);
            ImGui::PopStyleVar(2);
            ImGui::PopClipRect();

            ImGui::SetCursorPos(savedCursor);
        }
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
    if (cursorState.showCursors) {
        ImGui::Separator();
        renderDataPanel(data);
    }
    if (cursorState.showHarmonicsWindow) {
        renderHarmonicsWindow(data);
    }

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

    // Dual Cursors controls
    if (cursorState.showCursors) {
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.00f, 0.70f, 0.50f, 1.0f));
        if (ImGui::Button("Cursors (||)##cur")) cursorState.showCursors = false;
        ImGui::PopStyleColor();

        ImGui::SameLine();
        if (cursorState.snapToSample) {
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.15f, 0.65f, 0.35f, 1.0f));
            if (ImGui::Button("Snap ON##cur")) cursorState.snapToSample = false;
            ImGui::PopStyleColor();
        } else {
            if (ImGui::Button("Snap OFF##cur")) cursorState.snapToSample = true;
        }

        ImGui::SameLine();
        if (cursorState.lockBoundary) {
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.85f, 0.45f, 0.00f, 1.0f));
            if (ImGui::Button("Lock (t1<=t2)##cur")) cursorState.lockBoundary = false;
            ImGui::PopStyleColor();
        } else {
            if (ImGui::Button("Free Order##cur")) cursorState.lockBoundary = true;
        }

        ImGui::SameLine();
        if (cursorState.showHarmonicsWindow) {
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.70f, 0.20f, 0.80f, 1.0f));
            if (ImGui::Button("Spectrum (FFT)##cur")) cursorState.showHarmonicsWindow = false;
            ImGui::PopStyleColor();
        } else {
            if (ImGui::Button("Spectrum (FFT)##cur")) cursorState.showHarmonicsWindow = true;
        }
    } else {
        if (ImGui::Button("Cursors (||)##cur")) cursorState.showCursors = true;
    }

    ImGui::SameLine();
    ImGui::TextDisabled("|");
    ImGui::SameLine();

    // Plot Interpolation Mode selector
    const char* modeNames[] = { "Hybrid (Auto)", "Linear", "Stairs" };
    int currentModeIdx = (int)globalPlotMode;
    ImGui::SetNextItemWidth(110.0f);
    if (ImGui::Combo("##PlotModeCombo", &currentModeIdx, modeNames, 3)) {
        globalPlotMode = (InterpolationMode)currentModeIdx;
    }
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("Interpolation Mode:\n- Hybrid: Step at switching events (e.g. V_ds, V_L), Linear elsewhere\n- Linear: Continuous linear interpolation\n- Stairs: Step plot (e.g. Gate Pulses)");
    }

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
            double xMin = 0.0;
            double xMax = data.timeHistory.empty() ? 1.0 : data.timeHistory.back();
            double yMin = 1e30, yMax = -1e30;
            for (int ch = 0; ch < numChannels && ch < (int)channelSignalKeys.size(); ++ch) {
                const std::string& sigKey = channelSignalKeys[ch];
                if (sigKey.empty()) continue;
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
                if (it != data.voltages.end()) {
                    for (double v : it->second) {
                        if (v < yMin) yMin = v;
                        if (v > yMax) yMax = v;
                    }
                }
            }
            if (yMin > yMax) { yMin = -1.0; yMax = 1.0; }
            double ySpan = yMax - yMin;
            double yPad = (ySpan > 1e-12) ? (0.10 * ySpan) : ((std::abs(yMin) > 1e-6) ? (0.10 * std::abs(yMin)) : 1.0);
            ImPlot::SetNextAxesLimits(xMin, xMax, yMin - yPad, yMax + yPad, ImGuiCond_Always);
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
            renderCursorOverlay(0, data);

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

                    InterpolationMode mode = globalPlotMode;
                    if (globalPlotMode == InterpolationMode::AutoHybrid) {
                        mode = detectDefaultInterpolationMode(sigKey);
                    }

                    if (mode == InterpolationMode::AlwaysStairs) {
                        ImPlot::PlotStairs(lbl.c_str(), timeData, it->second.data(), pts, spec);
                    } else if (mode == InterpolationMode::AutoHybrid) {
                        std::vector<double> rawT(timeData, timeData + pts);
                        std::vector<double> rawY(it->second.begin(), it->second.begin() + pts);
                        std::vector<double> hT, hY;
                        buildHybridVertices(rawT, rawY, hT, hY);
                        ImPlot::PlotLine(lbl.c_str(), hT.data(), hY.data(), (int)hT.size(), spec);
                    } else {
                        ImPlot::PlotLine(lbl.c_str(), timeData, it->second.data(), pts, spec);
                    }
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
                    double xMin = 0.0;
                    double xMax = data.timeHistory.empty() ? 1.0 : data.timeHistory.back();
                    double yMin = 1e30, yMax = -1e30;
                    if (ch < (int)channelSignalKeys.size()) {
                        const std::string& sigKey = channelSignalKeys[ch];
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
                        if (it != data.voltages.end()) {
                            for (double v : it->second) {
                                if (v < yMin) yMin = v;
                                if (v > yMax) yMax = v;
                            }
                        }
                    }
                    if (yMin > yMax) { yMin = -1.0; yMax = 1.0; }
                    double ySpan = yMax - yMin;
                    double yPad = (ySpan > 1e-12) ? (0.10 * ySpan) : ((std::abs(yMin) > 1e-6) ? (0.10 * std::abs(yMin)) : 1.0);
                    ImPlot::SetNextAxesLimits(xMin, xMax, yMin - yPad, yMax + yPad, ImGuiCond_Always);
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
                    renderCursorOverlay(i, data);

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

                            InterpolationMode mode = globalPlotMode;
                            if (globalPlotMode == InterpolationMode::AutoHybrid) {
                                mode = detectDefaultInterpolationMode(sigKey);
                            }

                            if (mode == InterpolationMode::AlwaysStairs) {
                                ImPlot::PlotStairs(lbl.c_str(), timeData, it->second.data(), pts, spec);
                            } else if (mode == InterpolationMode::AutoHybrid) {
                                std::vector<double> rawT(timeData, timeData + pts);
                                std::vector<double> rawY(it->second.begin(), it->second.begin() + pts);
                                std::vector<double> hT, hY;
                                buildHybridVertices(rawT, rawY, hT, hY);
                                ImPlot::PlotLine(lbl.c_str(), hT.data(), hY.data(), (int)hT.size(), spec);
                            } else {
                                ImPlot::PlotLine(lbl.c_str(), timeData, it->second.data(), pts, spec);
                            }
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

double ScopeWindow::interpolateSignal(const std::vector<double>& timeHist, const std::vector<double>& signalData, double targetT, bool snap) const {
    if (timeHist.empty() || signalData.empty()) return 0.0;
    int n = std::min((int)timeHist.size(), (int)signalData.size());
    if (n == 0) return 0.0;

    if (targetT <= timeHist.front()) return signalData.front();
    if (targetT >= timeHist.back()) return signalData[n - 1];

    auto it = std::lower_bound(timeHist.begin(), timeHist.begin() + n, targetT);
    int idx = (int)std::distance(timeHist.begin(), it);

    if (idx == 0) return signalData[0];
    if (idx >= n) return signalData[n - 1];

    if (snap) {
        double d1 = std::abs(timeHist[idx - 1] - targetT);
        double d2 = std::abs(timeHist[idx] - targetT);
        return (d1 < d2) ? signalData[idx - 1] : signalData[idx];
    }

    double t0 = timeHist[idx - 1];
    double t1 = timeHist[idx];
    double y0 = signalData[idx - 1];
    double y1 = signalData[idx];

    if (std::abs(t1 - t0) < 1e-12) return y0;
    double alpha = (targetT - t0) / (t1 - t0);
    return y0 + alpha * (y1 - y0);
}

void ScopeWindow::renderCursorOverlay(int paneIdx, const CircuitSimEngine::TelemetryData& data) {
    if (!cursorState.showCursors || data.timeHistory.empty()) return;

    if (!cursorState.initialized) {
        double tMin = data.timeHistory.front();
        double tMax = data.timeHistory.back();
        double span = (tMax > tMin) ? (tMax - tMin) : 1.0;
        cursorState.cursor1Time = tMin + 0.20 * span;
        cursorState.cursor2Time = tMin + 0.80 * span;
        cursorState.initialized = true;
    }

    ImVec4 c1Color = isDarkMode ? ImVec4(0.00f, 0.90f, 1.00f, 0.90f) : ImVec4(0.00f, 0.45f, 0.85f, 0.90f);
    ImVec4 c2Color = isDarkMode ? ImVec4(1.00f, 0.70f, 0.00f, 0.90f) : ImVec4(0.85f, 0.45f, 0.00f, 0.90f);

    bool moved1 = ImPlot::DragLineX(1001, &cursorState.cursor1Time, c1Color, 2.0f);
    bool moved2 = ImPlot::DragLineX(1002, &cursorState.cursor2Time, c2Color, 2.0f);

    if (cursorState.lockBoundary) {
        if (cursorState.cursor1Time > cursorState.cursor2Time) {
            if (moved1) cursorState.cursor1Time = cursorState.cursor2Time;
            else if (moved2) cursorState.cursor2Time = cursorState.cursor1Time;
            else cursorState.cursor1Time = cursorState.cursor2Time;
        }
    }

    if (cursorState.snapToSample && (moved1 || moved2)) {
        auto snapVal = [&](double& t) {
            auto it = std::lower_bound(data.timeHistory.begin(), data.timeHistory.end(), t);
            if (it != data.timeHistory.end()) {
                if (it != data.timeHistory.begin()) {
                    auto prev = it - 1;
                    if (std::abs(*prev - t) < std::abs(*it - t)) t = *prev;
                    else t = *it;
                } else {
                    t = *it;
                }
            }
        };
        if (moved1) snapVal(cursorState.cursor1Time);
        if (moved2) snapVal(cursorState.cursor2Time);
    }

    // Render badge labels "I" and "II"
    ImDrawList* dl = ImPlot::GetPlotDrawList();
    ImVec2 plotPos = ImPlot::GetPlotPos();
    ImVec2 plotSize = ImPlot::GetPlotSize();

    float px1 = ImPlot::PlotToPixels(ImPlotPoint(cursorState.cursor1Time, 0)).x;
    float px2 = ImPlot::PlotToPixels(ImPlotPoint(cursorState.cursor2Time, 0)).x;

    float tagY = plotPos.y + plotSize.y - 18.0f;
    if (px1 >= plotPos.x && px1 <= plotPos.x + plotSize.x) {
        dl->AddRectFilled(ImVec2(px1 - 10, tagY), ImVec2(px1 + 10, tagY + 16), IM_COL32(0, 180, 220, 220), 3.0f);
        dl->AddText(ImVec2(px1 - 3, tagY + 1), IM_COL32(255, 255, 255, 255), "I");
    }
    if (px2 >= plotPos.x && px2 <= plotPos.x + plotSize.x) {
        dl->AddRectFilled(ImVec2(px2 - 12, tagY), ImVec2(px2 + 12, tagY + 16), IM_COL32(220, 140, 0, 220), 3.0f);
        dl->AddText(ImVec2(px2 - 6, tagY + 1), IM_COL32(255, 255, 255, 255), "II");
    }
}

void ScopeWindow::renderDataPanel(const CircuitSimEngine::TelemetryData& data) {
    if (!cursorState.showCursors || data.timeHistory.empty()) return;

    double t1 = cursorState.cursor1Time;
    double t2 = cursorState.cursor2Time;
    double dt = t2 - t1;
    double freq = (std::abs(dt) > 1e-12) ? (1.0 / std::abs(dt)) : 0.0;

    ImGui::PushStyleVar(ImGuiStyleVar_ChildRounding, 4.0f);
    ImGui::BeginChild("ScopeDataPanel", ImVec2(0, 160), true, ImGuiWindowFlags_None);

    ImGui::TextColored(ImVec4(0.00f, 0.85f, 1.00f, 1.00f), "Data & Time-Span Mathematical Analysis");
    ImGui::SameLine();
    ImGui::TextDisabled("(PLECS Scope Metrics Over [t1, t2])");

    static ImGuiTableFlags tflags = ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_Resizable | ImGuiTableFlags_SizingFixedFit;
    if (ImGui::BeginTable("##CursorDataTable", 10, tflags)) {
        ImGui::TableSetupColumn("Name", ImGuiTableColumnFlags_WidthStretch);
        ImGui::TableSetupColumn("Cursor 1", ImGuiTableColumnFlags_WidthFixed, 80.0f);
        ImGui::TableSetupColumn("Cursor 2", ImGuiTableColumnFlags_WidthFixed, 80.0f);
        ImGui::TableSetupColumn("Delta", ImGuiTableColumnFlags_WidthFixed, 80.0f);
        ImGui::TableSetupColumn("THD %", ImGuiTableColumnFlags_WidthFixed, 75.0f);
        ImGui::TableSetupColumn("Mean", ImGuiTableColumnFlags_WidthFixed, 80.0f);
        ImGui::TableSetupColumn("RMS", ImGuiTableColumnFlags_WidthFixed, 80.0f);
        ImGui::TableSetupColumn("Min", ImGuiTableColumnFlags_WidthFixed, 80.0f);
        ImGui::TableSetupColumn("Max", ImGuiTableColumnFlags_WidthFixed, 80.0f);
        ImGui::TableSetupColumn("AbsMax", ImGuiTableColumnFlags_WidthFixed, 80.0f);
        ImGui::TableHeadersRow();

        // Row 1: Time
        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        ImGui::TextUnformatted("Time (s)");
        ImGui::TableSetColumnIndex(1);
        ImGui::Text("%.6f", t1);
        ImGui::TableSetColumnIndex(2);
        ImGui::Text("%.6f", t2);
        ImGui::TableSetColumnIndex(3);
        ImGui::Text("%.6f", dt);
        ImGui::TableSetColumnIndex(4);
        if (freq > 0.0) {
            if (freq >= 1e6) ImGui::Text("%.3f MHz", freq / 1e6);
            else if (freq >= 1e3) ImGui::Text("%.3f kHz", freq / 1e3);
            else ImGui::Text("%.2f Hz", freq);
        } else {
            ImGui::TextUnformatted("N/A");
        }
        ImGui::TableSetColumnIndex(5); ImGui::TextDisabled("-");
        ImGui::TableSetColumnIndex(6); ImGui::TextDisabled("-");
        ImGui::TableSetColumnIndex(7); ImGui::TextDisabled("-");
        ImGui::TableSetColumnIndex(8); ImGui::TextDisabled("-");
        ImGui::TableSetColumnIndex(9); ImGui::TextDisabled("-");

        // Channel signal rows
        const auto& palette = isDarkMode ? SCOPE_DARK_COLORS : SCOPE_LIGHT_COLORS;
        constexpr size_t numColors = 8;

        for (int ch = 0; ch < numChannels && ch < (int)channelSignalKeys.size(); ++ch) {
            const std::string& sigKey = channelSignalKeys[ch];
            if (sigKey.empty()) continue;

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
                CircuitSimEngine::SignalStats st = CircuitSimEngine::computeSignalStats(data.timeHistory, it->second, t1, t2);
                CircuitSimEngine::FourierResult fr = CircuitSimEngine::computeFourierSpectrum(data.timeHistory, it->second, t1, t2, 30);

                std::string label = (ch < (int)channelLabels.size()) ? channelLabels[ch] : sigKey;

                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0);
                ImGui::ColorButton("##cDot", palette[ch % numColors], ImGuiColorEditFlags_NoTooltip, ImVec2(10, 10));
                ImGui::SameLine();
                ImGui::TextUnformatted(label.c_str());

                ImGui::TableSetColumnIndex(1); ImGui::Text("%.5g", st.yAtT1);
                ImGui::TableSetColumnIndex(2); ImGui::Text("%.5g", st.yAtT2);
                ImGui::TableSetColumnIndex(3); ImGui::Text("%.5g", st.yAtT2 - st.yAtT1);
                ImGui::TableSetColumnIndex(4);
                if (fr.isValid) ImGui::Text("%.2f%%", fr.thdPercent);
                else ImGui::TextDisabled("-");
                ImGui::TableSetColumnIndex(5); ImGui::Text("%.5g", st.mean);
                ImGui::TableSetColumnIndex(6); ImGui::Text("%.5g", st.rms);
                ImGui::TableSetColumnIndex(7); ImGui::Text("%.5g", st.minVal);
                ImGui::TableSetColumnIndex(8); ImGui::Text("%.5g", st.maxVal);
                ImGui::TableSetColumnIndex(9); ImGui::Text("%.5g", st.absMaxVal);
            }
        }
        ImGui::EndTable();
    }
    ImGui::EndChild();
    ImGui::PopStyleVar();
}

void ScopeWindow::renderHarmonicsWindow(const CircuitSimEngine::TelemetryData& data) {
    if (!cursorState.showHarmonicsWindow) return;

    double t1 = cursorState.cursor1Time;
    double t2 = cursorState.cursor2Time;

    std::string winTitle = "Harmonic Spectrum (FFT / DFT): " + scopeId + "###HarmWin_" + scopeId;
    ImGui::SetNextWindowSize(ImVec2(750, 480), ImGuiCond_FirstUseEver);

    if (!ImGui::Begin(winTitle.c_str(), &cursorState.showHarmonicsWindow)) {
        ImGui::End();
        return;
    }

    if (data.timeHistory.empty()) {
        ImGui::TextDisabled("No waveform data available.");
        ImGui::End();
        return;
    }

    const auto& palette = isDarkMode ? SCOPE_DARK_COLORS : SCOPE_LIGHT_COLORS;
    constexpr size_t numColors = 8;

    double f0 = (std::abs(t2 - t1) > 1e-12) ? (1.0 / std::abs(t2 - t1)) : 0.0;
    ImGui::TextColored(ImVec4(0.00f, 0.90f, 1.00f, 1.00f), "Fundamental-Aligned Fourier Spectrum");
    ImGui::SameLine();
    ImGui::TextDisabled("| f0 = %.2f Hz (T = %.6fs)", f0, std::abs(t2 - t1));

    ImGui::SameLine();
    ImGui::TextDisabled("|");
    ImGui::SameLine();

    ImGui::SetNextItemWidth(120.0f);
    ImGui::InputInt("Max Harmonics (N)##maxH", &cursorState.maxHarmonics, 10, 100);
    if (cursorState.maxHarmonics < 5) cursorState.maxHarmonics = 5;
    if (cursorState.maxHarmonics > 2000) cursorState.maxHarmonics = 2000;

    int renderPanes = std::min(numPanes, numChannels);
    if (renderPanes < 1) renderPanes = 1;

    int maxN = cursorState.maxHarmonics;

    if (renderPanes == 1) {
        // Single plot matching Scope single plot view
        if (ImPlot::BeginPlot("##HarmonicSpectrumSingle", ImVec2(-1, -1))) {
            ImPlot::SetupAxes("Harmonic Order (n)", "Peak Magnitude (V_n)");
            ImPlot::SetupAxisLimits(ImAxis_X1, 0.5, (double)maxN + 0.5, ImGuiCond_Always);

            for (int ch = 0; ch < numChannels && ch < (int)channelSignalKeys.size(); ++ch) {
                const std::string& sigKey = channelSignalKeys[ch];
                if (sigKey.empty()) continue;

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
                    CircuitSimEngine::FourierResult fr = CircuitSimEngine::computeFourierSpectrum(data.timeHistory, it->second, t1, t2, maxN);
                    if (fr.isValid && !fr.harmonicOrders.empty()) {
                        char buf[128];
                        snprintf(buf, sizeof(buf), "%s (THD=%.2f%%, V1=%.4g)",
                                 (ch < (int)channelLabels.size()) ? channelLabels[ch].c_str() : sigKey.c_str(),
                                 fr.thdPercent, fr.fundamentalMag);

                        ImPlotSpec spec;
                        spec.LineColor = palette[ch % numColors];
                        spec.FillColor = palette[ch % numColors];

                        ImPlot::PlotBars(buf, fr.harmonicOrders.data(), fr.harmonicMags.data(), (int)fr.harmonicOrders.size(), 0.5, spec);
                    }
                }
            }
            ImPlot::EndPlot();
        }
    } else {
        // Equal number of subplots as Scope (one per channel pane)
        if (ImPlot::BeginSubplots("##HarmonicsSubplots", renderPanes, 1, ImVec2(-1, -1), ImPlotSubplotFlags_LinkCols)) {
            for (int i = 0; i < renderPanes; ++i) {
                int ch = i % numChannels;
                const std::string& sigKey = (ch < (int)channelSignalKeys.size()) ? channelSignalKeys[ch] : "";
                std::string paneTitle = (ch < (int)channelLabels.size()) ? channelLabels[ch] : ("Ch" + std::to_string(ch+1));

                if (ImPlot::BeginPlot(paneTitle.c_str(), ImVec2(-1, -1))) {
                    ImPlot::SetupAxes("Harmonic Order (n)", "Magnitude");
                    ImPlot::SetupAxisLimits(ImAxis_X1, 0.5, (double)maxN + 0.5, ImGuiCond_Always);

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
                            CircuitSimEngine::FourierResult fr = CircuitSimEngine::computeFourierSpectrum(data.timeHistory, it->second, t1, t2, maxN);
                            if (fr.isValid && !fr.harmonicOrders.empty()) {
                                char buf[128];
                                snprintf(buf, sizeof(buf), "%s (THD=%.2f%%, V1=%.4g)",
                                         paneTitle.c_str(), fr.thdPercent, fr.fundamentalMag);

                                ImPlotSpec spec;
                                spec.LineColor = palette[ch % numColors];
                                spec.FillColor = palette[ch % numColors];

                                ImPlot::PlotBars(buf, fr.harmonicOrders.data(), fr.harmonicMags.data(), (int)fr.harmonicOrders.size(), 0.5, spec);
                            }
                        }
                    }
                    ImPlot::EndPlot();
                }
            }
            ImPlot::EndSubplots();
        }
    }

    ImGui::End();
}

} // namespace CircuitSim
