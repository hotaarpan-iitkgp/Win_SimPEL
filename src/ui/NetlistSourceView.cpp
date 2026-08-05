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

#include "engine/ExpressionEvaluator.hpp"

std::string NetlistSourceView::generateNetlistJson(const CircuitDesign& design) {
    CircuitDesign tempDesign = design;
    NetlistBuilder::buildNodesForCircuit(tempDesign);

    json root;

    json physStageObj;
    physStageObj["resistors"] = json::array();
    physStageObj["inductors"] = json::array();
    physStageObj["capacitors"] = json::array();
    physStageObj["voltage_sources"] = json::array();
    physStageObj["current_sources"] = json::array();
    physStageObj["switches"] = json::array();
    physStageObj["diodes"] = json::array();
    physStageObj["analog_switches"] = json::array();
    physStageObj["transformers"] = json::array();
    physStageObj["voltmeters"] = json::array();
    physStageObj["ammeters"] = json::array();
    physStageObj["custom_eblocks"] = json::array();

    json ctrlLoopsObj;
    ctrlLoopsObj["constants"] = json::array();
    ctrlLoopsObj["gains"] = json::array();
    ctrlLoopsObj["pi_controllers"] = json::array();
    ctrlLoopsObj["pid_controllers"] = json::array();
    ctrlLoopsObj["summing_junctions"] = json::array();
    ctrlLoopsObj["pwm_generators"] = json::array();
    ctrlLoopsObj["triangle_carriers"] = json::array();
    ctrlLoopsObj["comparators"] = json::array();
    ctrlLoopsObj["logic_gates"] = json::array();
    ctrlLoopsObj["product_blocks"] = json::array();
    ctrlLoopsObj["custom_functions"] = json::array();
    ctrlLoopsObj["custom_scripts"] = json::array();
    ctrlLoopsObj["signals_routing"] = json::array();
    ctrlLoopsObj["plls"] = json::array();
    ctrlLoopsObj["probes"] = json::array();
    ctrlLoopsObj["pwm_masters"] = json::array();

    for (const auto& comp : tempDesign.components) {
        std::string t = comp.rawTypeStr;
        std::transform(t.begin(), t.end(), t.begin(), ::toupper);

        json cObj;
        cObj["id"] = comp.id;

        // Format nodes into "node_0", "node_1", etc.
        json formattedNodes = json::array();
        for (const auto& n : comp.nodes) {
            if (n == "0" || n == "node_0") formattedNodes.push_back("node_0");
            else if (n.rfind("node_", 0) == 0) formattedNodes.push_back(n);
            else formattedNodes.push_back("node_" + n);
        }

        // Parse all component parameters using ExpressionEvaluator::parseScientific
        std::unordered_map<std::string, double> parsedParams;
        for (const auto& pair : comp.parameters) {
            parsedParams[pair.first] = CircuitSimEngine::ExpressionEvaluator::parseScientific(pair.second);
        }

        if (t == "R" || t == "RESISTOR") {
            cObj["nodes"] = formattedNodes;
            cObj["value"] = parsedParams.count("value") ? parsedParams["value"] : 10.0;
            cObj["esr"] = parsedParams.count("esr") ? parsedParams["esr"] : 0.0;
            cObj["src_type"] = "static";
            physStageObj["resistors"].push_back(cObj);
        } else if (t == "L" || t == "INDUCTOR") {
            cObj["nodes"] = formattedNodes;
            cObj["L"] = parsedParams.count("L") ? parsedParams["L"] : 0.0001;
            cObj["esr"] = parsedParams.count("esr") ? parsedParams["esr"] : 0.05;
            cObj["iL0"] = parsedParams.count("iL0") ? parsedParams["iL0"] : 0.0;
            physStageObj["inductors"].push_back(cObj);
        } else if (t == "C" || t == "CAPACITOR") {
            cObj["nodes"] = formattedNodes;
            cObj["C"] = parsedParams.count("C") ? parsedParams["C"] : 0.0001;
            cObj["esr"] = parsedParams.count("esr") ? parsedParams["esr"] : 0.01;
            cObj["vC0"] = parsedParams.count("vC0") ? parsedParams["vC0"] : 0.0;
            physStageObj["capacitors"].push_back(cObj);
        } else if (t == "V" || t == "VOLTAGESOURCE") {
            cObj["nodes"] = formattedNodes;
            cObj["value"] = parsedParams.count("value") ? parsedParams["value"] : 100.0;
            cObj["src_type"] = "dc";
            physStageObj["voltage_sources"].push_back(cObj);
        } else if (t == "D" || t == "DIODE") {
            cObj["type"] = "Diode";
            cObj["nodes"] = formattedNodes;
            cObj["Vd"] = parsedParams.count("Vd") ? parsedParams["Vd"] : 0.7;
            double rOn = parsedParams.count("Ron") ? parsedParams["Ron"] : 0.001;
            double rOff = parsedParams.count("Roff") ? parsedParams["Roff"] : 1000000.0;
            if (rOff < rOn * 1e4 || rOff <= 1.0) rOff = 1000000.0;
            cObj["Ron"] = rOn;
            cObj["Roff"] = rOff;
            physStageObj["diodes"].push_back(cObj);
        } else if (t == "MOSFET" || t == "S" || t == "IGBT" || t == "VG-FET") {
            cObj["type"] = "MOSFET";
            
            json pNodes = json::array();
            if (formattedNodes.size() >= 2) {
                pNodes.push_back(formattedNodes[0]);
                pNodes.push_back(formattedNodes[1]);
            } else {
                pNodes = formattedNodes;
            }
            cObj["nodes"] = pNodes;

            cObj["control_node"] = comp.id + ".G";

            std::string ctrlSig = "";
            for (const auto& w : tempDesign.wires) {
                if (w.to.compId == comp.id && (w.to.terminal == "G" || w.to.terminal == "Ctrl")) {
                    ctrlSig = w.from.compId + "." + w.from.terminal;
                    break;
                }
            }
            if (ctrlSig.empty()) ctrlSig = "PULSE_GEN1.Out";
            cObj["control_signal"] = ctrlSig;

            double rOn = parsedParams.count("Ron") ? parsedParams["Ron"] : 0.01;
            double rOff = parsedParams.count("Roff") ? parsedParams["Roff"] : 1000000.0;
            if (rOff < rOn * 1e4 || rOff <= 1.0) rOff = 1000000.0;
            cObj["Ron"] = rOn;
            cObj["Roff"] = rOff;
            cObj["Vd"] = parsedParams.count("Vd") ? parsedParams["Vd"] : 0.8;
            cObj["Iholding"] = parsedParams.count("Iholding") ? parsedParams["Iholding"] : 0.01;
            cObj["Vgt"] = parsedParams.count("Vgt") ? parsedParams["Vgt"] : 0.5;
            physStageObj["analog_switches"].push_back(cObj);
        } else if (t == "PULSE" || t == "PULSE_GEN" || t == "CONST" || t == "CONSTANT") {
            cObj["output"] = comp.id + ".Out";
            cObj["original_type"] = comp.rawTypeStr;
            cObj["amplitude"] = parsedParams.count("amplitude") ? parsedParams["amplitude"] : 1.0;
            cObj["period"] = parsedParams.count("period") ? parsedParams["period"] : 0.0001;
            cObj["width"] = parsedParams.count("width") ? parsedParams["width"] : 0.5;
            cObj["delay"] = parsedParams.count("delay") ? parsedParams["delay"] : 0.0;
            cObj["value"] = 1.0;
            ctrlLoopsObj["constants"].push_back(cObj);
        }
    }

    // ─── Ground Detection & Contiguous Node Re-indexing Pass ───
    std::string groundNodeStr = "";

    auto checkGround = [&](const json& compList) {
        for (const auto& item : compList) {
            if (item.contains("nodes") && item["nodes"].is_array()) {
                for (const auto& nVal : item["nodes"]) {
                    std::string nStr = nVal.get<std::string>();
                    if (nStr == "0" || nStr == "node_0") {
                        groundNodeStr = nStr;
                        return;
                    }
                }
            }
        }
    };

    checkGround(physStageObj["voltage_sources"]);
    if (groundNodeStr.empty()) checkGround(physStageObj["resistors"]);
    if (groundNodeStr.empty()) checkGround(physStageObj["capacitors"]);
    if (groundNodeStr.empty()) checkGround(physStageObj["diodes"]);
    if (groundNodeStr.empty()) checkGround(physStageObj["inductors"]);
    if (groundNodeStr.empty()) checkGround(physStageObj["analog_switches"]);

    // If no explicit "0" or "node_0" was found, default Ground to negative pin (nodes[1]) of primary Voltage Source V1
    if (groundNodeStr.empty()) {
        for (const auto& item : physStageObj["voltage_sources"]) {
            if (item.contains("nodes") && item["nodes"].is_array() && item["nodes"].size() >= 2) {
                groundNodeStr = item["nodes"][1].get<std::string>();
                break;
            }
        }
    }

    std::unordered_map<std::string, std::string> nodeRemap;
    if (!groundNodeStr.empty()) {
        nodeRemap[groundNodeStr] = "node_0";
    }
    nodeRemap["0"] = "node_0";
    nodeRemap["node_0"] = "node_0";

    int physNodeCounter = 1;

    auto compactNodes = [&](json& compList) {
        for (auto& item : compList) {
            if (item.contains("nodes") && item["nodes"].is_array()) {
                json newNodes = json::array();
                for (const auto& nVal : item["nodes"]) {
                    std::string nStr = nVal.get<std::string>();
                    if (nodeRemap.find(nStr) == nodeRemap.end()) {
                        nodeRemap[nStr] = "node_" + std::to_string(physNodeCounter++);
                    }
                    newNodes.push_back(nodeRemap[nStr]);
                }
                item["nodes"] = newNodes;
            }
        }
    };

    compactNodes(physStageObj["voltage_sources"]);
    compactNodes(physStageObj["analog_switches"]);
    compactNodes(physStageObj["diodes"]);
    compactNodes(physStageObj["inductors"]);
    compactNodes(physStageObj["capacitors"]);
    compactNodes(physStageObj["resistors"]);
    compactNodes(physStageObj["current_sources"]);
    compactNodes(physStageObj["switches"]);
    compactNodes(physStageObj["transformers"]);
    compactNodes(physStageObj["voltmeters"]);
    compactNodes(physStageObj["ammeters"]);

    root["physical_stage"] = physStageObj;
    root["control_loops"] = ctrlLoopsObj;

    json simParamsObj;
    simParamsObj["stop_time"] = (tempDesign.settings.stopTime > 0.0) ? tempDesign.settings.stopTime : 0.01;
    simParamsObj["step_size"] = (tempDesign.settings.stepSize > 0.0) ? tempDesign.settings.stepSize : 1e-5;
    if (simParamsObj["stop_time"].get<double>() <= 0.0) simParamsObj["stop_time"] = 0.01;
    if (simParamsObj["step_size"].get<double>() <= 0.0) simParamsObj["step_size"] = 1e-5;

    simParamsObj["solver"] = tempDesign.settings.solverType.empty() ? "euler" : tempDesign.settings.solverType;
    simParamsObj["step_type"] = tempDesign.settings.stepType.empty() ? "fixed" : tempDesign.settings.stepType;
    simParamsObj["solverMethod"] = "non-ideal";
    simParamsObj["engine"] = "auto";
    simParamsObj["enable_lu_cache"] = true;

    json wantedVars = json::array();
    for (const auto& p : tempDesign.plotConfig.plots) {
        for (const auto& v : p.variables) {
            wantedVars.push_back(v);
        }
    }
    if (wantedVars.empty()) {
        wantedVars = json::array({"COMP1.Minus", "COMP1.Out", "COMP1.Plus", "CONST1.Out", "I_L1", "I_D1", "V_C1"});
    }
    simParamsObj["wanted_variables"] = wantedVars;
    root["simulation_parameters"] = simParamsObj;

    root["probes"] = json::array();

    return root.dump(2);
}

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

    CircuitSimEngine::TelemetryData data = simulator.getTelemetryCopy();
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
