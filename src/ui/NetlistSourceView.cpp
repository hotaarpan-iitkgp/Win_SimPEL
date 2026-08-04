#include "NetlistSourceView.hpp"
#include "engine/NetlistBuilder.hpp"
#include "implot.h"
#include <nlohmann/json.hpp>
#include <windows.h>
#include <commdlg.h>
#include <fstream>
#include <iostream>
#include <algorithm>

using json = nlohmann::json;

namespace CircuitSim {

void NetlistSourceView::updateFromCircuit(const CircuitDesign& design) {
    CircuitDesign tempDesign = design;
    NetlistBuilder::buildNodesForCircuit(tempDesign);

    json root;

    // 1. Simulation Parameters
    json simParamsObj;
    simParamsObj["stop_time"] = tempDesign.settings.stopTime;
    simParamsObj["step_size"] = tempDesign.settings.stepSize;
    simParamsObj["solver"] = tempDesign.settings.solverType;
    simParamsObj["step_type"] = tempDesign.settings.stepType;
    root["simulation_parameters"] = simParamsObj;

    // 2. Physical Stage (Electrical Components & Resolved Nodes) and Control Loops
    json physArr = json::array();
    json ctrlArr = json::array();

    for (const auto& comp : tempDesign.components) {
        std::string t = comp.rawTypeStr;
        std::transform(t.begin(), t.end(), t.begin(), ::toupper);

        bool isElectrical = (t == "R" || t == "L" || t == "C" || t == "V" || t == "AC_V" || t == "I" || t == "S" || t == "D" || t == "MOSFET" || t == "VM" || t == "AM" || t == "GND");

        json cObj;
        cObj["id"] = comp.id;
        cObj["type"] = comp.rawTypeStr.empty() ? "UNKNOWN" : comp.rawTypeStr;
        if (!comp.label.empty()) cObj["label"] = comp.label;

        json paramsObj = json::object();
        for (const auto& pair : comp.parameters) {
            paramsObj[pair.first] = pair.second;
        }
        cObj["parameters"] = paramsObj;

        if (isElectrical) {
            json nodesArr = json::array();
            for (const auto& n : comp.nodes) {
                nodesArr.push_back(n);
            }
            cObj["nodes"] = nodesArr;
            physArr.push_back(cObj);
        } else {
            ctrlArr.push_back(cObj);
        }
    }

    root["physical_stage"] = physArr;
    root["control_loops"] = ctrlArr;

    // 3. Plot Configuration
    json plotConfigObj = json::object();
    json plotsArr = json::array();
    for (const auto& p : tempDesign.plotConfig.plots) {
        json pObj;
        pObj["title"] = p.title;
        json varsArr = json::array();
        for (const auto& v : p.variables) varsArr.push_back(v);
        pObj["variables"] = varsArr;
        plotsArr.push_back(pObj);
    }
    plotConfigObj["plots"] = plotsArr;
    root["plotConfiguration"] = plotConfigObj;

    lastGeneratedJson = root.dump(2);
    strncpy(jsonBuffer, lastGeneratedJson.c_str(), sizeof(jsonBuffer) - 1);
    isNetlistValid = true;
    netlistStatusMsg = "Valid Netlist";
}

void NetlistSourceView::render(const char* title, CircuitDesign& design, CircuitSimulator& simulator) {
    if (jsonBuffer[0] == '\0') {
        updateFromCircuit(design);
    }

    ImGuiIO& io = ImGui::GetIO();
    ImGui::SetNextWindowPos(ImVec2(0, 70), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(io.DisplaySize.x, io.DisplaySize.y - 75), ImGuiCond_Always);

    ImGui::Begin(title, nullptr, ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoTitleBar);

    float availWidth = ImGui::GetContentRegionAvail().x;
    float leftWidth = availWidth * 0.45f;
    float rightWidth = availWidth - leftWidth - 10.0f;

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

    if (ImGui::Button("Sync from Schematic")) {
        updateFromCircuit(design);
    }
    ImGui::SameLine();
    if (ImGui::Button("Apply Netlist Changes")) {
        try {
            json root = json::parse(jsonBuffer);
            isNetlistValid = true;
            netlistStatusMsg = "Valid Netlist";

            auto parseCompList = [](const json& arr, std::vector<ComponentInstance>& outList) {
                for (const auto& item : arr) {
                    ComponentInstance comp;
                    if (item.contains("id")) comp.id = item["id"].get<std::string>();
                    if (item.contains("type")) comp.rawTypeStr = item["type"].get<std::string>();
                    if (item.contains("label")) comp.label = item["label"].get<std::string>();
                    if (item.contains("nodes") && item["nodes"].is_array()) {
                        for (const auto& n : item["nodes"]) comp.nodes.push_back(n.get<std::string>());
                    }
                    if (item.contains("parameters") && item["parameters"].is_object()) {
                        for (auto& [pK, pV] : item["parameters"].items()) {
                            if (pV.is_string()) comp.parameters[pK] = pV.get<std::string>();
                            else if (pV.is_number()) comp.parameters[pK] = std::to_string(pV.get<double>());
                        }
                    }
                    outList.push_back(comp);
                }
            };

            if (root.contains("physical_stage") || root.contains("control_loops")) {
                design.components.clear();
                if (root.contains("physical_stage") && root["physical_stage"].is_array()) {
                    parseCompList(root["physical_stage"], design.components);
                }
                if (root.contains("control_loops") && root["control_loops"].is_array()) {
                    parseCompList(root["control_loops"], design.components);
                }
            } else if (root.contains("components") && root["components"].is_array()) {
                design.components.clear();
                parseCompList(root["components"], design.components);
            }

            if (root.contains("simulation_parameters") && root["simulation_parameters"].is_object()) {
                const auto& sp = root["simulation_parameters"];
                if (sp.contains("stop_time")) {
                    if (sp["stop_time"].is_number()) design.settings.stopTime = sp["stop_time"].get<double>();
                    else if (sp["stop_time"].is_string()) {
                        try { design.settings.stopTime = std::stod(sp["stop_time"].get<std::string>()); } catch (...) {}
                    }
                }
                if (sp.contains("step_size")) {
                    if (sp["step_size"].is_number()) design.settings.stepSize = sp["step_size"].get<double>();
                    else if (sp["step_size"].is_string()) {
                        try { design.settings.stepSize = std::stod(sp["step_size"].get<std::string>()); } catch (...) {}
                    }
                }
                if (sp.contains("solver") && sp["solver"].is_string()) {
                    design.settings.solverType = sp["solver"].get<std::string>();
                }
            }
        } catch (const std::exception& e) {
            isNetlistValid = false;
            netlistStatusMsg = std::string("JSON Syntax Error: ") + e.what();
        }
    }
    ImGui::SameLine();
    if (ImGui::Button("Copy Netlist")) {
        ImGui::SetClipboardText(jsonBuffer);
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
        ofn.lpstrDefExt = "json";
        ofn.Flags = OFN_PATHMUSTEXIST | OFN_OVERWRITEPROMPT;

        if (GetSaveFileNameA(&ofn) == TRUE) {
            std::ofstream outFile(ofn.lpstrFile);
            if (outFile.is_open()) {
                outFile << jsonBuffer;
                outFile.close();
            }
        }
    }

    if (!isNetlistValid) {
        ImGui::TextColored(ImVec4(1.0f, 0.3f, 0.3f, 1.0f), "%s", netlistStatusMsg.c_str());
    }

    ImGui::Spacing();
    ImGui::InputTextMultiline("##NetlistJsonEditor", jsonBuffer, sizeof(jsonBuffer), ImVec2(-1, -1), ImGuiInputTextFlags_AllowTabInput);

    ImGui::EndChild();

    ImGui::SameLine();

    // ─── RIGHT PANEL: WAVEFORM SOLVER PLOTTER ───
    ImGui::BeginChild("WaveformRightPanel", ImVec2(rightWidth, 0), true);

    ImGui::TextColored(ImVec4(0.2f, 0.8f, 1.0f, 1.0f), "Interactive Waveform Solver Plotter");
    ImGui::SameLine();
    ImGui::TextColored(ImVec4(0.7f, 0.7f, 0.7f, 1.0f), "| Sim Time: %.5f s", simulator.getCurrentTime());

    ImGui::Spacing();

    TelemetryData data = simulator.getTelemetryCopy();
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

        std::vector<SignalCat> categories;
        if (!voltageCat.variables.empty()) categories.push_back(voltageCat);
        if (!currentCat.variables.empty()) categories.push_back(currentCat);
        if (!controlCat.variables.empty()) categories.push_back(controlCat);
        if (!otherCat.variables.empty()) categories.push_back(otherCat);

        if (!categories.empty()) {
            if (ImGui::Button("Fit Waveforms / Reset Zoom")) {
                ImPlot::SetNextAxesToFit();
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
            ImGui::TextColored(ImVec4(0.6f, 0.6f, 0.6f, 1.0f), "| Tip: Right-click plot to add/remove subplots, drag box to zoom, scroll wheel to zoom axis.");

            int renderPanes = std::min(numPanes, (int)categories.size());
            if (renderPanes < 1) renderPanes = 1;

            if (ImPlot::BeginSubplots("Waveform Subplots", renderPanes, 1, ImVec2(-1, -1), ImPlotSubplotFlags_LinkCols)) {
                for (int i = 0; i < renderPanes; ++i) {
                    const auto& cat = categories[i % categories.size()];
                    if (ImPlot::BeginPlot(cat.title.c_str())) {
                        ImPlot::SetupAxes("Time (s)", cat.yLabel.c_str());

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
        }
    }

    ImGui::EndChild();

    ImGui::End();
}

} // namespace CircuitSim
