#include "NetlistSourceView.hpp"
#include "engine/NetlistBuilder.hpp"
#include "implot.h"
#include <nlohmann/json.hpp>
#include <windows.h>
#include <commdlg.h>
#include <fstream>
#include <iostream>
#include <algorithm>

#include "engine/ExpressionEvaluator.hpp"
#include "../engine/CScriptEngine.hpp"
#include <unordered_set>
#include <set>
#include <sstream>
#include <cmath>

using json = nlohmann::json;

namespace CircuitSim {

void NetlistSourceView::updateFromCircuit(const CircuitDesign& design) {
    lastGeneratedJson = generateNetlistJson(design);
    strncpy(jsonBuffer, lastGeneratedJson.c_str(), sizeof(jsonBuffer) - 1);
    isNetlistValid = true;
    netlistStatusMsg = "Valid Netlist";
}

void NetlistSourceView::render(const char* title, CircuitDesign& design, CircuitSimEngine::CircuitSimulator& simulator) {
    if (jsonBuffer[0] == '\0') {
        updateFromCircuit(design);
    }

    ImGuiViewport* viewport = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(ImVec2(viewport->WorkPos.x, viewport->WorkPos.y + 26.0f), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(viewport->WorkSize.x, viewport->WorkSize.y - 26.0f), ImGuiCond_Always);
    ImGui::SetNextWindowViewport(viewport->ID);

    ImGui::Begin(title, nullptr, ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoDocking);

    float availWidth = ImGui::GetContentRegionAvail().x;
    float splitterWidth = 8.0f;
    float leftWidth = availWidth * splitRatio;
    if (leftWidth < 120.0f) leftWidth = 120.0f;
    if (availWidth - leftWidth - splitterWidth < 200.0f) {
        leftWidth = availWidth - splitterWidth - 200.0f;
    }

    // ─── LEFT PANEL: RAW NETLIST SOURCE CODE EDITOR ───
    ImGui::BeginChild("NetlistLeftPanel", ImVec2(leftWidth, 0), true);
    
    ImGui::TextColored(ImVec4(0.2f, 0.8f, 1.0f, 1.0f), "Raw Netlist Source Code (JSON Schema)");
    ImGui::SameLine();
    if (isNetlistValid) {
        ImGui::TextColored(ImVec4(0.2f, 0.9f, 0.4f, 1.0f), "[Valid Netlist]");
    } else {
        ImGui::TextColored(ImVec4(1.0f, 0.3f, 0.3f, 1.0f), "[JSON Error]");
    }

    ImGui::Spacing();

    if (ImGui::Button("Copy JSON")) {
        ImGui::SetClipboardText(jsonBuffer);
    }
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("Copy complete schematic design JSON to clipboard");
    }
    ImGui::SameLine();
    if (ImGui::Button("Download Netlist")) {
        OPENFILENAMEA ofn;
        char szFile[260] = "circuit_netlist.json";
        ZeroMemory(&ofn, sizeof(ofn));
        ofn.lStructSize = sizeof(ofn);
        ofn.hwndOwner = NULL;
        ofn.lpstrFile = szFile;
        ofn.nMaxFile = sizeof(szFile);
        ofn.lpstrFilter = "JSON Files (*.json)\0*.json\0All Files (*.*)\0*.*\0";
        ofn.nFilterIndex = 1;
        ofn.lpstrFileTitle = NULL;
        ofn.nMaxFileTitle = 0;
        ofn.lpstrInitialDir = NULL;
        ofn.Flags = OFN_PATHMUSTEXIST | OFN_OVERWRITEPROMPT;

        if (GetSaveFileNameA(&ofn) == TRUE) {
            std::ofstream out(ofn.lpstrFile);
            if (out.is_open()) {
                out << jsonBuffer;
            }
        }
    }

    if (!isNetlistValid) {
        ImGui::TextColored(ImVec4(1.0f, 0.3f, 0.3f, 1.0f), "%s", netlistStatusMsg.c_str());
    }

    ImGui::Spacing();
    ImGui::InputTextMultiline("##NetlistJsonEditor", jsonBuffer, sizeof(jsonBuffer), ImVec2(-1, -1), ImGuiInputTextFlags_AllowTabInput);

    ImGui::EndChild();

    ImGui::SameLine(0, 0);

    // ─── INTERACTIVE DRAGGABLE SPLITTER BAR ───
    ImGui::PushStyleColor(ImGuiCol_Button, isDarkMode ? ImVec4(0.20f, 0.27f, 0.38f, 0.50f) : ImVec4(0.75f, 0.80f, 0.88f, 0.80f));
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.05f, 0.65f, 0.91f, 0.85f));
    ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(0.02f, 0.75f, 1.00f, 1.00f));

    ImGui::Button("##SplitterBar", ImVec2(splitterWidth, -1));
    if (ImGui::IsItemActive()) {
        float mouseDeltaX = ImGui::GetIO().MouseDelta.x;
        splitRatio += mouseDeltaX / availWidth;
        if (splitRatio < 0.05f) splitRatio = 0.05f;
        if (splitRatio > 0.85f) splitRatio = 0.85f;
    }
    if (ImGui::IsItemHovered() || ImGui::IsItemActive()) {
        ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeEW);
    }

    ImGui::PopStyleColor(3);

    ImGui::SameLine(0, 0);

    // ─── RIGHT PANEL: WAVEFORM SOLVER PLOTTER ───
    ImGui::BeginChild("WaveformRightPanel", ImVec2(0, 0), true);

    ImGui::TextColored(ImVec4(0.2f, 0.8f, 1.0f, 1.0f), "Interactive Waveform Solver Plotter");
    double pct = simulator.getProgressPercent();
    double cpuSec = simulator.getComputeTimeSeconds();
    ImGui::TextColored(ImVec4(0.7f, 0.7f, 0.7f, 1.0f), "| Progress: %.1f%% | Compute Time: %.3f s", pct, cpuSec);

    ImGui::Spacing();

    // Incremental refresh; see ScopeWindow for rationale.
    uint64_t curTelemetryVer = simulator.getTelemetryVersion();
    uint64_t curTelemetryGen = simulator.getTelemetryGeneration();
    if (curTelemetryVer != lastTelemetryVer || curTelemetryGen != lastTelemetryGen) {
        cachedCount = simulator.syncTelemetryInto(cachedTelemetry, cachedCount, lastTelemetryGen);
        lastTelemetryVer = curTelemetryVer;
    }
    const CircuitSimEngine::TelemetryData& data = cachedTelemetry;
    if (data.timeHistory.empty()) {
        ImGui::TextColored(ImVec4(0.7f, 0.7f, 0.7f, 1.0f), "No simulation telemetry data. Click PLAY or Start Simulation (Ctrl+T) to plot waveforms.");
    } else {
        struct SignalCat {
            std::string title;
            std::string yLabel;
            std::vector<std::pair<std::string, std::vector<double>>> variables;
        };

        SignalCat voltageCat{"Voltage Waveforms (V)", "Voltage (V)", {}};
        SignalCat currentCat{"Current Waveforms (I)", "Current (A)", {}};
        SignalCat controlCat{"Control & Pulse Signals", "Signal (V / State)", {}};
        SignalCat otherCat{"Other Signals", "Magnitude", {}};

        std::set<std::string> probedSet;

        // 1. Add all signals explicitly checked in Property Inspector
        for (const auto& plot : design.plotConfig.plots) {
            for (const auto& var : plot.variables) {
                if (!var.empty()) {
                    probedSet.insert(var);
                    if (var.rfind("V_", 0) == 0 || var.rfind("I_", 0) == 0) {
                        probedSet.insert(var.substr(2));
                    }
                }
            }
        }

        // 2. Add signals from components with explicit probe parameters
        for (const auto& comp : design.components) {
            if (comp.parameters.count("plotV") && comp.parameters.at("plotV") == "1") {
                probedSet.insert("V_" + comp.id);
            }
            if (comp.parameters.count("plotI") && comp.parameters.at("plotI") == "1") {
                probedSet.insert("I_" + comp.id);
            }
            if (comp.parameters.count("probe_signal") && comp.parameters.at("probe_signal") == "1") {
                probedSet.insert("V_" + comp.id);
                probedSet.insert("I_" + comp.id);
                probedSet.insert(comp.id);
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

        // 3. Add signals connected specifically to SCOPE or PROBE block terminals
        std::set<std::string> scopeProbeCompIds;
        for (const auto& comp : design.components) {
            std::string t = comp.rawTypeStr;
            std::transform(t.begin(), t.end(), t.begin(), ::toupper);
            if (t == "SCOPE" || t == "PROBE" || t == "UNIFIEDPROBE") {
                scopeProbeCompIds.insert(comp.id);
            }
        }

        for (const auto& wire : design.wires) {
            if (!wire.from.isWireJunction && scopeProbeCompIds.count(wire.from.compId)) {
                if (!wire.to.isWireJunction && !wire.to.compId.empty()) {
                    probedSet.insert(wire.to.compId);
                    probedSet.insert("V_" + wire.to.compId);
                    probedSet.insert("I_" + wire.to.compId);
                }
            }
            if (!wire.to.isWireJunction && scopeProbeCompIds.count(wire.to.compId)) {
                if (!wire.from.isWireJunction && !wire.from.compId.empty()) {
                    probedSet.insert(wire.from.compId);
                    probedSet.insert("V_" + wire.from.compId);
                    probedSet.insert("I_" + wire.from.compId);
                }
            }
        }

        auto isSigProbed = [&](const std::string& name) -> bool {
            if (probedSet.empty()) return false;
            if (probedSet.count(name) > 0) return true;
            std::string base = name;
            if (base.rfind("V_", 0) == 0 || base.rfind("I_", 0) == 0) {
                base = base.substr(2);
                if (probedSet.count(base) > 0) return true;
            }
            for (const auto& p : probedSet) {
                if (p.empty()) continue;
                if (name == p || name == ("V_" + p) || name == ("I_" + p)) return true;
            }
            return false;
        };

        // Synchronize enabledSignals map
        for (const auto& pair : data.voltages) {
            const std::string& name = pair.first;
            if (name.rfind("node_", 0) == 0 || name == "0" || name == "node_0") continue;
            enabledSignals[name] = isSigProbed(name);
        }

        for (const auto& pair : data.voltages) {
            const std::string& name = pair.first;
            const std::vector<double>& vals = pair.second;
            if (vals.empty()) continue;

            // Skip internal raw MNA matrix node voltages (node_1, node_2, 0, etc.)
            if (name.rfind("node_", 0) == 0 || name == "0" || name == "node_0") continue;

            // Filter: only plot signals selected by user / probed
            if (!enabledSignals[name]) continue;

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

        std::vector<SignalCat> categories;
        if (!voltageCat.variables.empty()) categories.push_back(voltageCat);
        if (!currentCat.variables.empty()) categories.push_back(currentCat);
        if (!controlCat.variables.empty()) categories.push_back(controlCat);
        if (!otherCat.variables.empty()) categories.push_back(otherCat);

        if (!categories.empty()) {
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
            snprintf(filterBuf, sizeof(filterBuf), "📊 Signals (%d/%d)##sigFilterNet", selCount, totCount);
            ImGui::SetNextItemWidth(140.0f);
            if (ImGui::BeginCombo("##SignalComboNet", filterBuf, ImGuiComboFlags_HeightLarge)) {
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

            // Plot Interpolation Mode selector
            const char* modeNamesNet[] = { "Hybrid (Auto)", "Linear", "Stairs" };
            int currentModeIdxNet = (int)globalPlotMode;
            ImGui::SetNextItemWidth(110.0f);
            if (ImGui::Combo("##PlotModeComboNet", &currentModeIdxNet, modeNamesNet, 3)) {
                globalPlotMode = (InterpolationMode)currentModeIdxNet;
            }
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip("Interpolation Mode:\n- Hybrid: Step at switching events (e.g. V_ds, V_L), Linear elsewhere\n- Linear: Continuous linear interpolation\n- Stairs: Step plot (e.g. Gate Pulses)");
            }

            ImGui::SameLine();

            ImGui::SetNextItemWidth(100.0f);
            ImGui::SliderFloat("Line Width", &traceLineWidth, 1.0f, 6.0f, "%.1f px");

            int renderPanes = std::min(numPanes, (int)categories.size());
            if (renderPanes < 1) renderPanes = 1;

            bool isZoomActive = (activeZoomMode != ActiveZoomMode::Disabled);

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

            if (ImPlot::BeginSubplots("Waveform Subplots", renderPanes, 1, ImVec2(-1, -1), ImPlotSubplotFlags_LinkCols)) {
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

                                InterpolationMode mode = globalPlotMode;
                                if (globalPlotMode == InterpolationMode::AutoHybrid) {
                                    mode = detectDefaultInterpolationMode(varName);
                                }

                                if (mode == InterpolationMode::AlwaysStairs) {
                                    ImPlot::PlotStairs(varName.c_str(), data.timeHistory.data(), vals.data(), count, spec);
                                } else if (mode == InterpolationMode::AutoHybrid) {
                                    std::vector<double> rawT(data.timeHistory.begin(), data.timeHistory.begin() + count);
                                    std::vector<double> rawY(vals.begin(), vals.begin() + count);
                                    std::vector<double> hT, hY;
                                    buildHybridVertices(rawT, rawY, hT, hY);
                                    ImPlot::PlotLine(varName.c_str(), hT.data(), hY.data(), (int)hT.size(), spec);
                                } else {
                                    ImPlot::PlotLine(varName.c_str(), data.timeHistory.data(), vals.data(), count, spec);
                                }
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
        }
    }

    ImGui::EndChild();

    ImGui::End();
}

} // namespace CircuitSim
