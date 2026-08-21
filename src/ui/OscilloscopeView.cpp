#include "OscilloscopeView.hpp"
#include "engine/SignalAnalysis.hpp"
#include "imgui_internal.h"
#include "implot.h"
#include <iostream>
#include <algorithm>
#include <map>
#include <cfloat>

#include <set>
#include <sstream>
#include "engine/Components.hpp"

namespace CircuitSim {

struct SignalCategory {
    std::string title;
    std::string yLabel;
    std::vector<std::pair<std::string, std::vector<double>>> variables;
};

static std::set<std::string> extractProbedSet(const CircuitDesign* design) {
    std::set<std::string> probedSet;
    if (!design) return probedSet;

    for (const auto& comp : design->components) {
        if (comp.parameters.count("probe_signal") && comp.parameters.at("probe_signal") == "1") {
            probedSet.insert("V_" + comp.id);
            probedSet.insert("I_" + comp.id);
            probedSet.insert(comp.id);
        }
        if (comp.parameters.count("plotV") && comp.parameters.at("plotV") == "1") {
            probedSet.insert("V_" + comp.id);
        }
        if (comp.parameters.count("plotI") && comp.parameters.at("plotI") == "1") {
            probedSet.insert("I_" + comp.id);
        }
        if (comp.parameters.count("selected_signals") && !comp.parameters.at("selected_signals").empty()) {
            std::stringstream ss(comp.parameters.at("selected_signals"));
            std::string item;
            while (std::getline(ss, item, ',')) {
                if (!item.empty()) {
                    probedSet.insert(item);
                    if (item.rfind("V_", 0) != 0 && item.rfind("I_", 0) != 0) {
                        probedSet.insert("V_" + item);
                        probedSet.insert("I_" + item);
                    }
                }
            }
        }
    }

    for (const auto& wire : design->wires) {
        if (!wire.from.isWireJunction && !wire.from.compId.empty()) probedSet.insert(wire.from.compId);
        if (!wire.to.isWireJunction && !wire.to.compId.empty()) probedSet.insert(wire.to.compId);
    }

    return probedSet;
}

static bool isSignalProbed(const std::string& name, const std::set<std::string>& probedSet) {
    if (probedSet.empty()) {
        // Strict default: if no explicit probe configured, only auto-select primary output & sensor signals
        if (name.rfind("VM", 0) == 0 || name.rfind("V_VM", 0) == 0 ||
            name.rfind("AM", 0) == 0 || name.rfind("I_AM", 0) == 0 ||
            name.find(".Out") != std::string::npos || name.find("SCOPE") != std::string::npos ||
            name.find("PROBE") != std::string::npos || name.rfind("V_C1", 0) == 0 ||
            name.rfind("V_R_load", 0) == 0 || name.rfind("V_out", 0) == 0 || name.rfind("V_Vout", 0) == 0) {
            return true;
        }
        return false;
    }

    if (probedSet.count(name) > 0) return true;

    std::string base = name;
    if (base.rfind("V_", 0) == 0 || base.rfind("I_", 0) == 0) {
        base = base.substr(2);
        if (probedSet.count(base) > 0) return true;
    }

    for (const auto& p : probedSet) {
        if (p.empty()) continue;
        if (name == p || name == ("V_" + p) || name == ("I_" + p)) return true;
        if (name.find(p) != std::string::npos || p.find(name) != std::string::npos) return true;
    }
    return false;
}

void OscilloscopeView::render(const char* title, CircuitSimEngine::CircuitSimulator& simulator, const CircuitDesign* design) {
    ImGui::Begin(title);

    // Handle dock node size for collapse/expand (must be after Begin so window exists)
    ImGuiWindow* dockWin = ImGui::GetCurrentWindow();
    if (dockWin && dockWin->DockNode) {
        ImGuiDockNode* node = dockWin->DockNode;
        if (isCollapsed) {
            // Force the dock node to minimal height (just the tab bar)
            float minH = ImGui::GetFrameHeight() + 8.0f;
            node->SizeRef.y = minH;
        } else if (savedDockHeight > 0.0f) {
            // Restore saved height after expanding
            node->SizeRef.y = savedDockHeight;
            savedDockHeight = 0.0f;
        }
    }

    // Collapse/Expand toggle button positioned at the beginning of content
    {
        const char* btnLabel = isCollapsed ? "\xe2\x96\xb6##OscToggle" : "\xe2\x96\xbc##OscToggle"; // ▶ (collapsed) or ▼ (expanded)
        if (ImGui::SmallButton(btnLabel)) {
            if (!isCollapsed) {
                // Collapsing: save current dock height
                if (dockWin->DockNode) {
                    savedDockHeight = dockWin->DockNode->SizeRef.y;
                    if (savedDockHeight <= 0.0f) savedDockHeight = dockWin->Size.y;
                }
            }
            isCollapsed = !isCollapsed;
        }
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip(isCollapsed ? "Expand Oscilloscope" : "Collapse Oscilloscope");
        }
        ImGui::SameLine();
    }

    // If collapsed, show title hint on the same line then end early
    if (isCollapsed) {
        ImGui::TextDisabled("(Oscilloscope collapsed — click \xe2\x96\xb6 to expand)");
        ImGui::End();
        return;
    }
    
    uint64_t currentVer = simulator.getTelemetryVersion();
    if (currentVer != lastTelemetryVer || cachedTelemetry.timeHistory.empty()) {
        cachedTelemetry = simulator.getTelemetryCopy();
        lastTelemetryVer = currentVer;
    }
    const auto& data = cachedTelemetry;
    
    if (data.timeHistory.empty()) {
        ImGui::TextColored(ImVec4(0.7f, 0.7f, 0.7f, 1.0f), "No simulation waveform data available. Press PLAY to run simulation.");
        ImGui::End();
        return;
    }

    std::set<std::string> probedSet = extractProbedSet(design);

    // Synchronize enabledSignals map with data.voltages
    bool hasAnySelected = false;
    for (const auto& pair : data.voltages) {
        const std::string& name = pair.first;
        if (name.rfind("node_", 0) == 0 || name == "0" || name == "node_0") continue;
        if (enabledSignals.count(name) == 0) {
            enabledSignals[name] = isSignalProbed(name, probedSet);
        }
        if (enabledSignals[name]) hasAnySelected = true;
    }

    // Safety fallback: if no signals were auto-selected, enable the first 2 variables
    if (!hasAnySelected) {
        int count = 0;
        for (const auto& pair : data.voltages) {
            const std::string& name = pair.first;
            if (name.rfind("node_", 0) == 0 || name == "0" || name == "node_0") continue;
            enabledSignals[name] = true;
            if (++count >= 2) break;
        }
    }

    SignalCategory voltageCat{"Voltage Waveforms (V)", "Voltage (V)", {}};
    SignalCategory currentCat{"Current Waveforms (I)", "Current (A)", {}};
    SignalCategory controlCat{"Control & Scope Signals", "Signal (V / State)", {}};
    SignalCategory otherCat{"Other Signals", "Magnitude", {}};

    for (const auto& pair : data.voltages) {
        const std::string& name = pair.first;
        const std::vector<double>& vals = pair.second;
        if (vals.empty()) continue;

        // Skip internal raw MNA matrix node voltages (node_1, node_2, 0, etc.)
        if (name.rfind("node_", 0) == 0 || name == "0" || name == "node_0") continue;

        // Filter: only plot signals that are enabled by user / probed
        if (!enabledSignals[name]) continue;

        if (name.rfind("I_", 0) == 0 || name.rfind("AM", 0) == 0) {
            currentCat.variables.push_back({name, vals});
        } else if (name.rfind("V_", 0) == 0 || name.rfind("VM", 0) == 0) {
            voltageCat.variables.push_back({name, vals});
        } else if (name.find(".Out") != std::string::npos || name.find(".In") != std::string::npos || 
                   name.rfind("Ctrl_", 0) == 0 || name.find("GAIN") != std::string::npos ||
                   name.find("SCOPE") != std::string::npos || name.find("PROBE") != std::string::npos ||
                   name.find("PULSE") != std::string::npos || name.find("PWM") != std::string::npos ||
                   name.find("TRI") != std::string::npos || name.find("PID") != std::string::npos ||
                   name.find("SUM") != std::string::npos || name.find("PROD") != std::string::npos ||
                   name.find("COMP") != std::string::npos || name.find("MATH") != std::string::npos ||
                   name.find("EDGE") != std::string::npos || name.find("KEY") != std::string::npos) {
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

    // Interactive Signal Selection Dropdown
    int selCount = 0;
    int totCount = 0;
    for (const auto& pair : data.voltages) {
        const std::string& name = pair.first;
        if (name.rfind("node_", 0) == 0 || name == "0" || name == "node_0") continue;
        totCount++;
        if (enabledSignals[name]) selCount++;
    }

    char filterBuf[64];
    snprintf(filterBuf, sizeof(filterBuf), "📊 Signals (%d/%d)##sigFilterOsc", selCount, totCount);
    ImGui::SetNextItemWidth(140.0f);
    if (ImGui::BeginCombo("##SignalComboOsc", filterBuf, ImGuiComboFlags_HeightLarge)) {
        if (ImGui::SmallButton("Select All")) {
            for (const auto& pair : data.voltages) enabledSignals[pair.first] = true;
        }
        ImGui::SameLine();
        if (ImGui::SmallButton("Deselect All")) {
            for (const auto& pair : data.voltages) enabledSignals[pair.first] = false;
        }
        ImGui::Separator();

        for (const auto& pair : data.voltages) {
            const std::string& name = pair.first;
            if (name.rfind("node_", 0) == 0 || name == "0" || name == "node_0") continue;
            bool enabled = enabledSignals[name];
            if (ImGui::Checkbox(name.c_str(), &enabled)) {
                enabledSignals[name] = enabled;
            }
        }
        ImGui::EndCombo();
    }
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("Filter which variables are plotted on screen.\nCheck or uncheck variables to customize waveforms.");
    }
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

    ImGui::SameLine();
    ImGui::TextDisabled("|");
    ImGui::SameLine();

    // Dual Cursors controls
    if (cursorState.showCursors) {
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.00f, 0.70f, 0.50f, 1.0f));
        if (ImGui::Button("Cursors (||)##cur_osc")) cursorState.showCursors = false;
        ImGui::PopStyleColor();

        ImGui::SameLine();
        if (cursorState.snapToSample) {
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.15f, 0.65f, 0.35f, 1.0f));
            if (ImGui::Button("Snap ON##cur_osc")) cursorState.snapToSample = false;
            ImGui::PopStyleColor();
        } else {
            if (ImGui::Button("Snap OFF##cur_osc")) cursorState.snapToSample = true;
        }

        ImGui::SameLine();
        if (cursorState.lockBoundary) {
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.85f, 0.45f, 0.00f, 1.0f));
            if (ImGui::Button("Lock (t1<=t2)##cur_osc")) cursorState.lockBoundary = false;
            ImGui::PopStyleColor();
        } else {
            if (ImGui::Button("Free Order##cur_osc")) cursorState.lockBoundary = true;
        }

        ImGui::SameLine();
        if (cursorState.showHarmonicsWindow) {
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.70f, 0.20f, 0.80f, 1.0f));
            if (ImGui::Button("Spectrum (FFT)##cur_osc")) cursorState.showHarmonicsWindow = false;
            ImGui::PopStyleColor();
        } else {
            if (ImGui::Button("Spectrum (FFT)##cur_osc")) cursorState.showHarmonicsWindow = true;
        }
    } else {
        if (ImGui::Button("Cursors (||)##cur_osc")) cursorState.showCursors = true;
    }

    ImGui::SameLine();
    ImGui::TextDisabled("|");
    ImGui::SameLine();

    // Plot Interpolation Mode selector
    const char* modeNamesOsc[] = { "Hybrid (Auto)", "Linear", "Stairs" };
    int currentModeIdxOsc = (int)globalPlotMode;
    ImGui::SetNextItemWidth(110.0f);
    if (ImGui::Combo("##PlotModeComboOsc", &currentModeIdxOsc, modeNamesOsc, 3)) {
        globalPlotMode = (InterpolationMode)currentModeIdxOsc;
    }
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("Interpolation Mode:\n- Hybrid: Step at switching events (e.g. V_ds, V_L), Linear elsewhere\n- Linear: Continuous linear interpolation\n- Stairs: Step plot (e.g. Gate Pulses)");
    }

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

    ImPlot::PushStyleVar(ImPlotStyleVar_PlotPadding, ImVec2(4.0f, 2.0f));
    ImPlot::PushStyleVar(ImPlotStyleVar_LabelPadding, ImVec2(2.0f, 2.0f));

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
                // Fit with 10% Y padding for exact centering above and below
                double xMin = 0.0;
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
                double yPad = (yRange > 1e-12) ? (yRange * 0.10) : ((std::abs(yMin) > 1e-6) ? (0.10 * std::abs(yMin)) : 1.0);
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

                bool isBottomPlot = (i == renderPanes - 1);
                if (!isBottomPlot) {
                    ImPlot::SetupAxes(nullptr, cat.yLabel.c_str(), ImPlotAxisFlags_NoTickLabels | ImPlotAxisFlags_NoLabel, 0);
                } else {
                    ImPlot::SetupAxes("Time (s)", cat.yLabel.c_str(), 0, 0);
                }
                renderCursorOverlay(i, data);

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
                thread_local std::vector<double> s_decT;
                thread_local std::vector<double> s_decY;
                thread_local std::vector<double> s_hT;
                thread_local std::vector<double> s_hY;

                for (const auto& varPair : cat.variables) {
                    const std::string& varName = varPair.first;
                    const std::vector<double>& vals = varPair.second;
                    int count = (int)std::min(data.timeHistory.size(), vals.size());
                    if (count > 0) {
                        ImPlotSpec spec;
                        spec.LineColor = palette[varIdx % numColors];
                        spec.LineWeight = traceLineWidth;

                        InterpolationMode mode = globalPlotMode;
                        if (globalPlotMode == InterpolationMode::AutoHybrid) {
                            mode = detectDefaultInterpolationMode(varName);
                        }

                        const double* pT = data.timeHistory.data();
                        const double* pY = vals.data();
                        int drawCount = count;

                        if (count > 2000) {
                            decimateMinMax(pT, pY, count, 2000, s_decT, s_decY);
                            pT = s_decT.data();
                            pY = s_decY.data();
                            drawCount = (int)s_decT.size();
                        }

                        if (mode == InterpolationMode::AlwaysStairs) {
                            ImPlot::PlotStairs(varName.c_str(), pT, pY, drawCount, spec);
                        } else if (mode == InterpolationMode::AutoHybrid) {
                            s_hT.assign(pT, pT + drawCount);
                            s_hY.assign(pY, pY + drawCount);
                            std::vector<double> hOutT, hOutY;
                            buildHybridVertices(s_hT, s_hY, hOutT, hOutY);
                            ImPlot::PlotLine(varName.c_str(), hOutT.data(), hOutY.data(), (int)hOutT.size(), spec);
                        } else {
                            ImPlot::PlotLine(varName.c_str(), pT, pY, drawCount, spec);
                        }
                    }
                    varIdx++;
                }
                ImPlot::EndPlot();
            }
        }
        ImPlot::EndSubplots();
    }
    ImPlot::PopStyleVar(2);

    if (cursorState.showCursors) {
        ImGui::Separator();
        renderDataPanel(data);
    }
    if (cursorState.showHarmonicsWindow) {
        renderHarmonicsWindow(data);
    }

    ImGui::End();
}

double OscilloscopeView::interpolateSignal(const std::vector<double>& timeHist, const std::vector<double>& signalData, double targetT, bool snap) const {
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

void OscilloscopeView::renderCursorOverlay(int paneIdx, const CircuitSimEngine::TelemetryData& data) {
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

    bool moved1 = ImPlot::DragLineX(2001, &cursorState.cursor1Time, c1Color, 2.0f);
    bool moved2 = ImPlot::DragLineX(2002, &cursorState.cursor2Time, c2Color, 2.0f);

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

void OscilloscopeView::renderDataPanel(const CircuitSimEngine::TelemetryData& data) {
    if (!cursorState.showCursors || data.timeHistory.empty()) return;

    double t1 = cursorState.cursor1Time;
    double t2 = cursorState.cursor2Time;
    double dt = t2 - t1;
    double freq = (std::abs(dt) > 1e-12) ? (1.0 / std::abs(dt)) : 0.0;

    ImGui::PushStyleVar(ImGuiStyleVar_ChildRounding, 4.0f);
    ImGui::BeginChild("OscDataPanel", ImVec2(0, 160), true, ImGuiWindowFlags_None);

    ImGui::TextColored(ImVec4(0.00f, 0.85f, 1.00f, 1.00f), "Data & Time-Span Mathematical Analysis");
    ImGui::SameLine();
    ImGui::TextDisabled("(PLECS Scope Metrics Over [t1, t2])");

    static ImGuiTableFlags tflags = ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_Resizable | ImGuiTableFlags_SizingFixedFit;
    if (ImGui::BeginTable("##OscCursorDataTable", 10, tflags)) {
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

        // Active signals
        static const ImVec4 DARK_MODE_COLORS[] = {
            ImVec4(0.00f, 0.95f, 1.00f, 1.00f), ImVec4(0.10f, 1.00f, 0.45f, 1.00f),
            ImVec4(1.00f, 0.88f, 0.00f, 1.00f), ImVec4(1.00f, 0.25f, 0.60f, 1.00f),
            ImVec4(1.00f, 0.50f, 0.10f, 1.00f), ImVec4(0.70f, 0.40f, 1.00f, 1.00f)
        };
        static const ImVec4 LIGHT_MODE_COLORS[] = {
            ImVec4(0.05f, 0.35f, 0.75f, 1.00f), ImVec4(0.02f, 0.50f, 0.25f, 1.00f),
            ImVec4(0.80f, 0.12f, 0.12f, 1.00f), ImVec4(0.50f, 0.15f, 0.75f, 1.00f)
        };
        const auto& palette = isDarkMode ? DARK_MODE_COLORS : LIGHT_MODE_COLORS;

        int sIdx = 0;
        for (const auto& pair : data.voltages) {
            const std::string& name = pair.first;
            const std::vector<double>& vals = pair.second;
            if (vals.empty() || name.rfind("node_", 0) == 0 || name == "0") continue;

            CircuitSimEngine::SignalStats st = CircuitSimEngine::computeSignalStats(data.timeHistory, vals, t1, t2);
            CircuitSimEngine::FourierResult fr = CircuitSimEngine::computeFourierSpectrum(data.timeHistory, vals, t1, t2, cursorState.maxHarmonics);

            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::ColorButton("##cDotOsc", palette[sIdx % 6], ImGuiColorEditFlags_NoTooltip, ImVec2(10, 10));
            ImGui::SameLine();
            ImGui::TextUnformatted(name.c_str());

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

            sIdx++;
        }
        ImGui::EndTable();
    }
    ImGui::EndChild();
    ImGui::PopStyleVar();
}

void OscilloscopeView::renderHarmonicsWindow(const CircuitSimEngine::TelemetryData& data) {
    if (!cursorState.showHarmonicsWindow) return;

    double t1 = cursorState.cursor1Time;
    double t2 = cursorState.cursor2Time;

    ImGui::SetNextWindowSize(ImVec2(750, 480), ImGuiCond_FirstUseEver);

    if (!ImGui::Begin("Oscilloscope Harmonic Spectrum (FFT / DFT)###OscHarmWin", &cursorState.showHarmonicsWindow)) {
        ImGui::End();
        return;
    }

    if (data.timeHistory.empty()) {
        ImGui::TextDisabled("No waveform data available.");
        ImGui::End();
        return;
    }

    static const ImVec4 DARK_MODE_COLORS[] = {
        ImVec4(0.00f, 0.95f, 1.00f, 1.00f), ImVec4(0.10f, 1.00f, 0.45f, 1.00f),
        ImVec4(1.00f, 0.88f, 0.00f, 1.00f), ImVec4(1.00f, 0.25f, 0.60f, 1.00f)
    };
    static const ImVec4 LIGHT_MODE_COLORS[] = {
        ImVec4(0.05f, 0.35f, 0.75f, 1.00f), ImVec4(0.02f, 0.50f, 0.25f, 1.00f)
    };
    const auto& palette = isDarkMode ? DARK_MODE_COLORS : LIGHT_MODE_COLORS;

    double f0 = (std::abs(t2 - t1) > 1e-12) ? (1.0 / std::abs(t2 - t1)) : 0.0;
    ImGui::TextColored(ImVec4(0.00f, 0.90f, 1.00f, 1.00f), "Fundamental-Aligned Fourier Spectrum");
    ImGui::SameLine();
    ImGui::TextDisabled("| f0 = %.2f Hz (T = %.6fs)", f0, std::abs(t2 - t1));

    ImGui::SameLine();
    ImGui::TextDisabled("|");
    ImGui::SameLine();

    ImGui::SetNextItemWidth(120.0f);
    ImGui::InputInt("Max Harmonics (N)##maxH_osc", &cursorState.maxHarmonics, 10, 100);
    if (cursorState.maxHarmonics < 5) cursorState.maxHarmonics = 5;
    if (cursorState.maxHarmonics > 2000) cursorState.maxHarmonics = 2000;

    int renderPanes = numPanes;
    if (renderPanes < 1) renderPanes = 1;

    int maxN = cursorState.maxHarmonics;

    // Collect active signal vectors
    std::vector<std::pair<std::string, std::vector<double>>> activeSigs;
    for (const auto& pair : data.voltages) {
        if (pair.second.empty() || pair.first.rfind("node_", 0) == 0 || pair.first == "0") continue;
        activeSigs.push_back(pair);
    }

    if (renderPanes == 1 || activeSigs.empty()) {
        if (ImPlot::BeginPlot("##OscHarmonicSpectrumSingle", ImVec2(-1, -1))) {
            ImPlot::SetupAxes("Harmonic Order (n)", "Peak Magnitude (V_n)");
            ImPlot::SetupAxisLimits(ImAxis_X1, 0.5, (double)maxN + 0.5, ImGuiCond_Always);

            int sigIdx = 0;
            for (const auto& pair : activeSigs) {
                CircuitSimEngine::FourierResult fr = CircuitSimEngine::computeFourierSpectrum(data.timeHistory, pair.second, t1, t2, maxN);
                if (fr.isValid && !fr.harmonicOrders.empty()) {
                    char buf[128];
                    snprintf(buf, sizeof(buf), "%s (THD=%.2f%%, V1=%.4g)", pair.first.c_str(), fr.thdPercent, fr.fundamentalMag);

                    ImPlotSpec spec;
                    spec.LineColor = palette[sigIdx % 4];
                    spec.FillColor = palette[sigIdx % 4];

                    ImPlot::PlotBars(buf, fr.harmonicOrders.data(), fr.harmonicMags.data(), (int)fr.harmonicOrders.size(), 0.5, spec);
                }
                sigIdx++;
            }
            ImPlot::EndPlot();
        }
    } else {
        // Equal subplots matching Oscilloscope view panes
        if (ImPlot::BeginSubplots("##OscHarmonicsSubplots", renderPanes, 1, ImVec2(-1, -1), ImPlotSubplotFlags_LinkCols)) {
            for (int i = 0; i < renderPanes; ++i) {
                const auto& pair = activeSigs[i % activeSigs.size()];
                std::string paneTitle = pair.first;

                if (ImPlot::BeginPlot(paneTitle.c_str(), ImVec2(-1, -1))) {
                    ImPlot::SetupAxes("Harmonic Order (n)", "Magnitude");
                    ImPlot::SetupAxisLimits(ImAxis_X1, 0.5, (double)maxN + 0.5, ImGuiCond_Always);

                    CircuitSimEngine::FourierResult fr = CircuitSimEngine::computeFourierSpectrum(data.timeHistory, pair.second, t1, t2, maxN);
                    if (fr.isValid && !fr.harmonicOrders.empty()) {
                        char buf[128];
                        snprintf(buf, sizeof(buf), "%s (THD=%.2f%%, V1=%.4g)", paneTitle.c_str(), fr.thdPercent, fr.fundamentalMag);

                        ImPlotSpec spec;
                        spec.LineColor = palette[i % 4];
                        spec.FillColor = palette[i % 4];

                        ImPlot::PlotBars(buf, fr.harmonicOrders.data(), fr.harmonicMags.data(), (int)fr.harmonicOrders.size(), 0.5, spec);
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
