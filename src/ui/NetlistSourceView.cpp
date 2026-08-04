#include "NetlistSourceView.hpp"
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
    json root;
    json compsArr = json::array();
    for (const auto& comp : design.components) {
        json cObj;
        cObj["id"] = comp.id;
        cObj["type"] = comp.rawTypeStr.empty() ? "UNKNOWN" : comp.rawTypeStr;
        cObj["label"] = comp.label;
        cObj["x"] = comp.x;
        cObj["y"] = comp.y;
        cObj["rotation"] = comp.rotation;
        json paramsObj = json::object();
        for (const auto& pair : comp.parameters) {
            paramsObj[pair.first] = pair.second;
        }
        cObj["parameters"] = paramsObj;
        compsArr.push_back(cObj);
    }
    root["components"] = compsArr;

    json wiresArr = json::array();
    for (const auto& w : design.wires) {
        json wObj;
        wObj["id"] = w.id;
        json fromObj;
        fromObj["type"] = "pin";
        fromObj["compId"] = w.from.compId;
        fromObj["terminal"] = w.from.terminal;
        wObj["from"] = fromObj;

        json toObj;
        if (w.to.isWireJunction) {
            toObj["type"] = "wire";
            toObj["wireId"] = w.to.targetWireId;
            toObj["x"] = w.to.junctionX;
            toObj["y"] = w.to.junctionY;
        } else {
            toObj["type"] = "pin";
            toObj["compId"] = w.to.compId;
            toObj["terminal"] = w.to.terminal;
        }
        wObj["to"] = toObj;
        wiresArr.push_back(wObj);
    }
    root["wires"] = wiresArr;

    json plotConfigObj = json::object();
    json plotsArr = json::array();
    for (const auto& p : design.plotConfig.plots) {
        json pObj;
        pObj["title"] = p.title;
        json varsArr = json::array();
        for (const auto& v : p.variables) varsArr.push_back(v);
        pObj["variables"] = varsArr;
        plotsArr.push_back(pObj);
    }
    plotConfigObj["plots"] = plotsArr;
    root["plotConfiguration"] = plotConfigObj;

    json simSettingsObj = json::object();
    simSettingsObj["stopTime"] = std::to_string(design.settings.stopTime);
    simSettingsObj["stepSize"] = std::to_string(design.settings.stepSize);
    simSettingsObj["solver"] = design.settings.solverType;
    simSettingsObj["stepType"] = design.settings.stepType;
    root["simulationSettings"] = simSettingsObj;

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

            if (root.contains("components") && root["components"].is_array()) {
                design.components.clear();
                for (const auto& item : root["components"]) {
                    ComponentInstance comp;
                    if (item.contains("id")) comp.id = item["id"].get<std::string>();
                    if (item.contains("type")) comp.rawTypeStr = item["type"].get<std::string>();
                    if (item.contains("label")) comp.label = item["label"].get<std::string>();
                    if (item.contains("x")) comp.x = item["x"].get<float>();
                    if (item.contains("y")) comp.y = item["y"].get<float>();
                    if (item.contains("rotation")) comp.rotation = item["rotation"].get<int>();

                    if (item.contains("parameters") && item["parameters"].is_object()) {
                        for (auto& [pK, pV] : item["parameters"].items()) {
                            if (pV.is_string()) comp.parameters[pK] = pV.get<std::string>();
                            else if (pV.is_number()) comp.parameters[pK] = std::to_string(pV.get<double>());
                        }
                    }
                    design.components.push_back(comp);
                }
            }

            if (root.contains("wires") && root["wires"].is_array()) {
                design.wires.clear();
                for (const auto& wItem : root["wires"]) {
                    WireInstance wire;
                    if (wItem.contains("id")) wire.id = wItem["id"].get<std::string>();
                    if (wItem.contains("from") && wItem["from"].is_object()) {
                        wire.from.compId = wItem["from"].value("compId", "");
                        wire.from.terminal = wItem["from"].value("terminal", "");
                    }
                    if (wItem.contains("to") && wItem["to"].is_object()) {
                        std::string toType = wItem["to"].value("type", "pin");
                        if (toType == "wire") {
                            wire.to.isWireJunction = true;
                            wire.to.targetWireId = wItem["to"].value("wireId", "");
                            wire.to.junctionX = wItem["to"].value("x", 0.0f);
                            wire.to.junctionY = wItem["to"].value("y", 0.0f);
                        } else {
                            wire.to.isWireJunction = false;
                            wire.to.compId = wItem["to"].value("compId", "");
                            wire.to.terminal = wItem["to"].value("terminal", "");
                        }
                    }
                    design.wires.push_back(wire);
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
        if (ImPlot::BeginPlot("Simulated Waveforms (MNA Solver)", ImVec2(-1, -1))) {
            ImPlot::SetupAxes("Time (s)", "Voltage / Current (V/A)");

            for (const auto& pair : data.voltages) {
                const std::string& varName = pair.first;
                const std::vector<double>& vals = pair.second;

                int count = (int)std::min(data.timeHistory.size(), vals.size());
                if (count > 0) {
                    ImPlot::PlotLine(varName.c_str(), data.timeHistory.data(), vals.data(), count);
                }
            }

            ImPlot::EndPlot();
        }
    }

    ImGui::EndChild();

    ImGui::End();
}

} // namespace CircuitSim
