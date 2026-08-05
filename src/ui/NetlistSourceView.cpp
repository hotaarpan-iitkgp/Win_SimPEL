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
#include <unordered_set>
#include <cmath>

using json = nlohmann::json;

namespace CircuitSim {

static double roundToDigits(double val, int digits = 9) {
    if (val == 0.0) return 0.0;
    double scale = std::pow(10.0, digits);
    return std::round(val * scale) / scale;
}

static json formatJSStyleDouble(double val) {
    if (val == 0.0) return 0;
    if (val == std::floor(val) && std::abs(val) < 1e15) {
        return (long long)val;
    }
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%.17g", val);
    std::string s(buf);
    try {
        return json::parse(s);
    } catch (...) {
        return val;
    }
}

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

    std::unordered_set<std::string> validCompIds;
    for (const auto& comp : tempDesign.components) {
        validCompIds.insert(comp.id);
    }

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

        // Parse all component parameters using ExpressionEvaluator::parseScientific and roundToDigits
        std::unordered_map<std::string, double> parsedParams;
        for (const auto& pair : comp.parameters) {
            double rawVal = CircuitSimEngine::ExpressionEvaluator::parseScientific(pair.second);
            parsedParams[pair.first] = roundToDigits(rawVal, 9);
        }

        if (t == "R" || t == "RESISTOR") {
            cObj["nodes"] = formattedNodes;
            cObj["value"] = formatJSStyleDouble(parsedParams.count("value") ? parsedParams["value"] : 10.0);
            cObj["esr"] = formatJSStyleDouble(parsedParams.count("esr") ? parsedParams["esr"] : 0.0);
            cObj["src_type"] = "static";
            physStageObj["resistors"].push_back(cObj);
        } else if (t == "L" || t == "INDUCTOR") {
            cObj["nodes"] = formattedNodes;
            cObj["L"] = formatJSStyleDouble(parsedParams.count("L") ? parsedParams["L"] : 0.0001);
            cObj["esr"] = formatJSStyleDouble(parsedParams.count("esr") ? parsedParams["esr"] : 0.05);
            cObj["iL0"] = formatJSStyleDouble(parsedParams.count("iL0") ? parsedParams["iL0"] : 0.0);
            physStageObj["inductors"].push_back(cObj);
        } else if (t == "C" || t == "CAPACITOR") {
            cObj["nodes"] = formattedNodes;
            cObj["C"] = formatJSStyleDouble(parsedParams.count("C") ? parsedParams["C"] : 0.0001);
            cObj["esr"] = formatJSStyleDouble(parsedParams.count("esr") ? parsedParams["esr"] : 0.01);
            cObj["vC0"] = formatJSStyleDouble(parsedParams.count("vC0") ? parsedParams["vC0"] : 0.0);
            physStageObj["capacitors"].push_back(cObj);
        } else if (t == "V" || t == "VOLTAGESOURCE") {
            cObj["nodes"] = formattedNodes;
            cObj["value"] = formatJSStyleDouble(parsedParams.count("value") ? parsedParams["value"] : 100.0);
            cObj["src_type"] = "dc";
            physStageObj["voltage_sources"].push_back(cObj);
        } else if (t == "D" || t == "DIODE") {
            cObj["type"] = "Diode";
            cObj["nodes"] = formattedNodes;
            cObj["Vd"] = formatJSStyleDouble(parsedParams.count("Vd") ? parsedParams["Vd"] : 0.7);
            double rOn = parsedParams.count("Ron") ? parsedParams["Ron"] : 0.001;
            double rOff = parsedParams.count("Roff") ? parsedParams["Roff"] : 1000000.0;
            if (rOff < rOn * 1e4 || rOff <= 1.0) rOff = 1000000.0;
            cObj["Ron"] = formatJSStyleDouble(rOn);
            cObj["Roff"] = formatJSStyleDouble(rOff);
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
            cObj["Ron"] = formatJSStyleDouble(rOn);
            cObj["Roff"] = formatJSStyleDouble(rOff);
            cObj["Vd"] = formatJSStyleDouble(parsedParams.count("Vd") ? parsedParams["Vd"] : 0.8);
            cObj["Iholding"] = formatJSStyleDouble(parsedParams.count("Iholding") ? parsedParams["Iholding"] : 0.01);
            cObj["Vgt"] = formatJSStyleDouble(parsedParams.count("Vgt") ? parsedParams["Vgt"] : 0.5);
            physStageObj["analog_switches"].push_back(cObj);
        } else if (t == "PULSE" || t == "PULSE_GEN" || t == "CONST" || t == "CONSTANT") {
            cObj["output"] = comp.id + ".Out";
            cObj["original_type"] = comp.rawTypeStr;
            cObj["amplitude"] = formatJSStyleDouble(parsedParams.count("amplitude") ? parsedParams["amplitude"] : 1.0);
            cObj["period"] = formatJSStyleDouble(parsedParams.count("period") ? parsedParams["period"] : 0.0001);
            cObj["width"] = formatJSStyleDouble(parsedParams.count("width") ? parsedParams["width"] : 0.5);
            cObj["delay"] = formatJSStyleDouble(parsedParams.count("delay") ? parsedParams["delay"] : 0.0);
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
    double rawStopTime = (tempDesign.settings.stopTime > 0.0) ? tempDesign.settings.stopTime : 0.01;
    double rawStepSize = (tempDesign.settings.stepSize > 0.0) ? tempDesign.settings.stepSize : 1e-5;

    simParamsObj["stop_time"] = formatJSStyleDouble(rawStopTime);
    simParamsObj["step_size"] = formatJSStyleDouble(rawStepSize);

    simParamsObj["solver"] = tempDesign.settings.solverType.empty() ? "euler" : tempDesign.settings.solverType;
    simParamsObj["step_type"] = tempDesign.settings.stepType.empty() ? "fixed" : tempDesign.settings.stepType;
    simParamsObj["solverMethod"] = "non-ideal";
    simParamsObj["engine"] = "auto";
    simParamsObj["enable_lu_cache"] = true;

    // ─── Component-Aware Wanted Variables Generation ───
    json wantedVars = json::array();
    std::unordered_set<std::string> addedVars;

    // Include plotConfig variables ONLY if their component actually exists in schematic
    for (const auto& p : tempDesign.plotConfig.plots) {
        for (const auto& v : p.variables) {
            std::string compBase = v;
            if (compBase.rfind("I_", 0) == 0 || compBase.rfind("V_", 0) == 0) {
                compBase = compBase.substr(2);
            } else if (compBase.find('.') != std::string::npos) {
                compBase = compBase.substr(0, compBase.find('.'));
            }
            if (validCompIds.count(compBase) && !addedVars.count(v)) {
                wantedVars.push_back(v);
                addedVars.insert(v);
            }
        }
    }

    // Dynamic Discovery: Add default telemetry variables for components present in schematic
    for (const auto& comp : tempDesign.components) {
        std::string t = comp.rawTypeStr;
        std::transform(t.begin(), t.end(), t.begin(), ::toupper);

        if (t == "C" || t == "CAPACITOR") {
            std::string varName = "V_" + comp.id;
            if (!addedVars.count(varName)) {
                wantedVars.push_back(varName);
                addedVars.insert(varName);
            }
        } else if (t == "L" || t == "INDUCTOR") {
            std::string varName = "I_" + comp.id;
            if (!addedVars.count(varName)) {
                wantedVars.push_back(varName);
                addedVars.insert(varName);
            }
        } else if (t == "D" || t == "DIODE") {
            std::string varName = "I_" + comp.id;
            if (!addedVars.count(varName)) {
                wantedVars.push_back(varName);
                addedVars.insert(varName);
            }
        } else if (t == "MOSFET" || t == "S" || t == "IGBT" || t == "VG-FET") {
            std::string varName = "I_" + comp.id;
            if (!addedVars.count(varName)) {
                wantedVars.push_back(varName);
                addedVars.insert(varName);
            }
        } else if (t == "PULSE" || t == "PULSE_GEN" || t == "PWM" || t == "CONST" || t == "CONSTANT") {
            std::string varName = comp.id + ".Out";
            if (!addedVars.count(varName)) {
                wantedVars.push_back(varName);
                addedVars.insert(varName);
            }
        }
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
    ImGui::SetNextWindowPos(ImVec2(0, 52), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(io.DisplaySize.x, io.DisplaySize.y - 54), ImGuiCond_Always);

    ImGui::Begin(title, nullptr, ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoTitleBar);

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
            bool doFitThisFrame = autoFitNext;
            if (autoFitNext) autoFitNext = false;
            if (ImGui::Button("Fit Waveforms / Reset Zoom")) doFitThisFrame = true;
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

            ImGui::TextDisabled("|");
            ImGui::SameLine();

            ImGui::SetNextItemWidth(100.0f);
            ImGui::SliderFloat("Line Width", &traceLineWidth, 1.0f, 6.0f, "%.1f px");
            ImGui::SameLine();

            ImGui::TextColored(isDarkMode ? ImVec4(0.7f, 0.8f, 0.9f, 1.0f) : ImVec4(0.3f, 0.4f, 0.5f, 1.0f), "| Tip: Right-click plot to add/remove subplots, drag box to zoom.");

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

            if (ImPlot::BeginSubplots("Waveform Subplots", renderPanes, 1, ImVec2(-1, -1), ImPlotSubplotFlags_LinkCols)) {
                for (int i = 0; i < renderPanes; ++i) {
                    const auto& cat = categories[i % categories.size()];

                    // Pending zoom from previous frame (before BeginPlot)
                    if (hasPendingZoom[i]) {
                        ImPlot::SetNextAxesLimits(pendingXMin[i], pendingXMax[i],
                                                  pendingYMin[i], pendingYMax[i],
                                                  ImGuiCond_Always);
                        hasPendingZoom[i] = false;
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

                    if (isAdaptiveZoomEnabled) {
                        ImPlot::PushStyleColor(ImPlotCol_Selection, ImVec4(0.0f, 0.0f, 0.0f, 0.0f));
                    }

                    if (ImPlot::BeginPlot(cat.title.c_str(), ImVec2(-1, -1),
                                           isAdaptiveZoomEnabled ? ImPlotFlags_NoMenus : ImPlotFlags_None)) {

                        // Override mouse button bindings based on zoom mode
                        if (isAdaptiveZoomEnabled) {
                            ImPlot::GetInputMap().Select       = ImGuiMouseButton_Left;
                            ImPlot::GetInputMap().SelectCancel = ImGuiMouseButton_Right;
                            ImPlot::GetInputMap().Pan          = ImGuiMouseButton_Right;
                        } else {
                            ImPlot::GetInputMap().Select       = ImGuiMouseButton_Right;
                            ImPlot::GetInputMap().SelectCancel = ImGuiMouseButton_Left;
                            ImPlot::GetInputMap().Pan          = ImGuiMouseButton_Left;
                        }

                        ImPlot::SetupAxes("Time (s)", cat.yLabel.c_str());

                        // Adaptive Box Zoom real-time visual rubber-band and release commit
                        if (isAdaptiveZoomEnabled && ImPlot::IsPlotSelected()) {
                            ImPlotRect sel = ImPlot::GetPlotSelection();
                            ImPlotRect cur = ImPlot::GetPlotLimits();

                            double dxSel = std::abs(sel.X.Max - sel.X.Min);
                            double dySel = std::abs(sel.Y.Max - sel.Y.Min);
                            double dxCur = std::abs(cur.X.Max - cur.X.Min);
                            double dyCur = std::abs(cur.Y.Max - cur.Y.Min);

                            double nx = (dxCur > 1e-15) ? dxSel / dxCur : 0.0;
                            double ny = (dyCur > 1e-15) ? dySel / dyCur : 0.0;

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

                            ImDrawList* drawList = ImPlot::GetPlotDrawList();

                            // Tolerance ratio 1.4 for responsive X / Y zoom detection
                            if (nx > 1.4 * ny) {
                                // --- X-AXIS ZOOM: HORIZONTAL SPAN BAND ---
                                drawList->AddRectFilled(ImVec2(x1, pTop), ImVec2(x2, pBottom), IM_COL32(0, 220, 255, 35));
                                drawList->AddLine(ImVec2(x1, pTop), ImVec2(x1, pBottom), IM_COL32(0, 220, 255, 255), 2.0f);
                                drawList->AddLine(ImVec2(x2, pTop), ImVec2(x2, pBottom), IM_COL32(0, 220, 255, 255), 2.0f);

                                const char* tag = " [ ↔ X-Zoom ] ";
                                ImVec2 txtSz = ImGui::CalcTextSize(tag);
                                float midX = (x1 + x2) * 0.5f;
                                drawList->AddRectFilled(ImVec2(midX - txtSz.x * 0.5f - 4, pTop + 6), ImVec2(midX + txtSz.x * 0.5f + 4, pTop + 6 + txtSz.y + 2), IM_COL32(0, 150, 200, 230), 4.0f);
                                drawList->AddText(ImVec2(midX - txtSz.x * 0.5f, pTop + 7), IM_COL32(255, 255, 255, 255), tag);
                            } else if (ny > 1.4 * nx) {
                                // --- Y-AXIS ZOOM: VERTICAL SPAN BAND ---
                                drawList->AddRectFilled(ImVec2(pLeft, y1), ImVec2(pRight, y2), IM_COL32(220, 0, 255, 35));
                                drawList->AddLine(ImVec2(pLeft, y1), ImVec2(pRight, y1), IM_COL32(220, 0, 255, 255), 2.0f);
                                drawList->AddLine(ImVec2(pLeft, y2), ImVec2(pRight, y2), IM_COL32(220, 0, 255, 255), 2.0f);

                                const char* tag = " [ ↕ Y-Zoom ] ";
                                ImVec2 txtSz = ImGui::CalcTextSize(tag);
                                float midY = (y1 + y2) * 0.5f;
                                drawList->AddRectFilled(ImVec2(pLeft + 6, midY - txtSz.y * 0.5f - 2), ImVec2(pLeft + 6 + txtSz.x + 8, midY + txtSz.y * 0.5f + 2), IM_COL32(160, 0, 180, 230), 4.0f);
                                drawList->AddText(ImVec2(pLeft + 10, midY - txtSz.y * 0.5f), IM_COL32(255, 255, 255, 255), tag);
                            } else {
                                // --- 2D BOX ZOOM: RECTANGLE ---
                                drawList->AddRectFilled(ImVec2(x1, y1), ImVec2(x2, y2), IM_COL32(60, 255, 120, 35));
                                drawList->AddRect(ImVec2(x1, y1), ImVec2(x2, y2), IM_COL32(60, 255, 120, 255), 0, 0, 2.0f);

                                const char* tag = " [ ⤢ Box Zoom ] ";
                                ImVec2 txtSz = ImGui::CalcTextSize(tag);
                                drawList->AddRectFilled(ImVec2(x1 + 4, y1 + 4), ImVec2(x1 + 12 + txtSz.x, y1 + 6 + txtSz.y), IM_COL32(30, 160, 80, 230), 4.0f);
                                drawList->AddText(ImVec2(x1 + 8, y1 + 5), IM_COL32(255, 255, 255, 255), tag);
                            }

                            if (ImGui::IsMouseReleased(ImGuiMouseButton_Left)) {
                                ImPlot::CancelPlotSelection();

                                pendingXMin[i] = cur.X.Min; pendingXMax[i] = cur.X.Max;
                                pendingYMin[i] = cur.Y.Min; pendingYMax[i] = cur.Y.Max;

                                if (nx > 1.4 * ny) {
                                    pendingXMin[i] = sel.X.Min;
                                    pendingXMax[i] = sel.X.Max;
                                } else if (ny > 1.4 * nx) {
                                    pendingYMin[i] = sel.Y.Min;
                                    pendingYMax[i] = sel.Y.Max;
                                } else {
                                    pendingXMin[i] = sel.X.Min; pendingXMax[i] = sel.X.Max;
                                    pendingYMin[i] = sel.Y.Min; pendingYMax[i] = sel.Y.Max;
                                }
                                hasPendingZoom[i] = true;
                            }
                        }

                        // Right-Click Context Menu (only when zoom mode is off)
                        if (!isAdaptiveZoomEnabled && ImGui::BeginPopupContextItem("PlotContextMenu")) {
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
                    if (isAdaptiveZoomEnabled) {
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
