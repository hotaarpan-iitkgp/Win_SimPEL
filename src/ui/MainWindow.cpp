#include "MainWindow.hpp"
#include "engine/NetlistBuilder.hpp"
#include "imgui.h"
#include <iostream>
#include <string>
#include <cstdlib>
#include <fstream>
#include <windows.h>
#include <commdlg.h>
#include "engine/NetlistParser.hpp"
#include "nlohmann/json.hpp"
#include <algorithm>

using json = nlohmann::json;

namespace CircuitSim {

void MainWindow::applyDarkTheme() {
    ImGuiStyle& style = ImGui::GetStyle();
    ImGui::StyleColorsDark();

    style.WindowRounding = 6.0f;
    style.ChildRounding = 6.0f;
    style.FrameRounding = 5.0f;
    style.PopupRounding = 6.0f;
    style.ScrollbarRounding = 4.0f;
    style.GrabRounding = 4.0f;
    style.TabRounding = 6.0f;
    style.WindowBorderSize = 1.0f;
    style.FrameBorderSize = 1.0f;
    style.ItemSpacing = ImVec2(8, 6);

    ImVec4* colors = style.Colors;
    colors[ImGuiCol_WindowBg]             = ImVec4(0.06f, 0.09f, 0.16f, 1.00f);
    colors[ImGuiCol_ChildBg]              = ImVec4(0.11f, 0.15f, 0.23f, 1.00f);
    colors[ImGuiCol_PopupBg]              = ImVec4(0.09f, 0.13f, 0.20f, 0.98f);
    colors[ImGuiCol_Border]               = ImVec4(0.20f, 0.27f, 0.38f, 0.70f);
    colors[ImGuiCol_FrameBg]              = ImVec4(0.12f, 0.17f, 0.27f, 1.00f);
    colors[ImGuiCol_FrameBgHovered]       = ImVec4(0.18f, 0.25f, 0.38f, 1.00f);
    colors[ImGuiCol_FrameBgActive]        = ImVec4(0.24f, 0.32f, 0.47f, 1.00f);
    colors[ImGuiCol_TitleBg]              = ImVec4(0.06f, 0.09f, 0.16f, 1.00f);
    colors[ImGuiCol_TitleBgActive]        = ImVec4(0.11f, 0.15f, 0.23f, 1.00f);
    colors[ImGuiCol_MenuBarBg]            = ImVec4(0.08f, 0.12f, 0.19f, 1.00f);
    colors[ImGuiCol_ScrollbarBg]          = ImVec4(0.06f, 0.09f, 0.16f, 0.50f);
    colors[ImGuiCol_ScrollbarGrab]        = ImVec4(0.20f, 0.27f, 0.38f, 1.00f);
    colors[ImGuiCol_ScrollbarGrabHovered] = ImVec4(0.28f, 0.37f, 0.52f, 1.00f);
    colors[ImGuiCol_CheckMark]            = ImVec4(0.05f, 0.65f, 0.91f, 1.00f);
    colors[ImGuiCol_SliderGrab]           = ImVec4(0.05f, 0.65f, 0.91f, 1.00f);
    colors[ImGuiCol_SliderGrabActive]     = ImVec4(0.02f, 0.75f, 1.00f, 1.00f);
    colors[ImGuiCol_Button]               = ImVec4(0.14f, 0.20f, 0.31f, 1.00f);
    colors[ImGuiCol_ButtonHovered]        = ImVec4(0.05f, 0.65f, 0.91f, 0.85f);
    colors[ImGuiCol_ButtonActive]         = ImVec4(0.02f, 0.52f, 0.78f, 1.00f);
    colors[ImGuiCol_Header]               = ImVec4(0.14f, 0.20f, 0.31f, 1.00f);
    colors[ImGuiCol_HeaderHovered]        = ImVec4(0.20f, 0.28f, 0.42f, 1.00f);
    colors[ImGuiCol_HeaderActive]         = ImVec4(0.05f, 0.65f, 0.91f, 0.60f);
    colors[ImGuiCol_Tab]                  = ImVec4(0.11f, 0.15f, 0.23f, 1.00f);
    colors[ImGuiCol_TabHovered]           = ImVec4(0.05f, 0.65f, 0.91f, 0.80f);
    colors[ImGuiCol_TabActive]            = ImVec4(0.16f, 0.22f, 0.33f, 1.00f);
    colors[ImGuiCol_Text]                 = ImVec4(0.95f, 0.96f, 0.98f, 1.00f);
    colors[ImGuiCol_TextDisabled]         = ImVec4(0.50f, 0.58f, 0.70f, 1.00f);
}

void MainWindow::applyLightTheme() {
    ImGuiStyle& style = ImGui::GetStyle();
    ImGui::StyleColorsLight();

    style.WindowRounding = 6.0f;
    style.ChildRounding = 6.0f;
    style.FrameRounding = 5.0f;
    style.PopupRounding = 6.0f;
    style.ScrollbarRounding = 4.0f;
    style.GrabRounding = 4.0f;
    style.TabRounding = 6.0f;
    style.WindowBorderSize = 1.0f;
    style.FrameBorderSize = 1.0f;
    style.ItemSpacing = ImVec2(8, 6);

    ImVec4* colors = style.Colors;
    colors[ImGuiCol_WindowBg]             = ImVec4(0.92f, 0.94f, 0.97f, 1.00f); // Sleek slate grey #e9eef5
    colors[ImGuiCol_ChildBg]              = ImVec4(0.95f, 0.97f, 0.99f, 1.00f); // Clean light grey panel #f1f5f9
    colors[ImGuiCol_PopupBg]              = ImVec4(1.00f, 1.00f, 1.00f, 0.98f);
    colors[ImGuiCol_Border]               = ImVec4(0.78f, 0.83f, 0.89f, 1.00f); // Crisp slate border
    colors[ImGuiCol_BorderShadow]         = ImVec4(0.00f, 0.00f, 0.00f, 0.04f);
    colors[ImGuiCol_FrameBg]              = ImVec4(0.88f, 0.92f, 0.96f, 1.00f);
    colors[ImGuiCol_FrameBgHovered]       = ImVec4(0.80f, 0.86f, 0.93f, 1.00f);
    colors[ImGuiCol_FrameBgActive]        = ImVec4(0.72f, 0.80f, 0.90f, 1.00f);
    colors[ImGuiCol_TitleBg]              = ImVec4(0.88f, 0.92f, 0.96f, 1.00f);
    colors[ImGuiCol_TitleBgActive]        = ImVec4(0.82f, 0.87f, 0.93f, 1.00f);
    colors[ImGuiCol_MenuBarBg]            = ImVec4(0.95f, 0.97f, 0.99f, 1.00f);
    colors[ImGuiCol_ScrollbarBg]          = ImVec4(0.92f, 0.94f, 0.97f, 0.60f);
    colors[ImGuiCol_ScrollbarGrab]        = ImVec4(0.75f, 0.80f, 0.86f, 1.00f);
    colors[ImGuiCol_ScrollbarGrabHovered] = ImVec4(0.60f, 0.68f, 0.78f, 1.00f);
    colors[ImGuiCol_CheckMark]            = ImVec4(0.02f, 0.52f, 0.78f, 1.00f);
    colors[ImGuiCol_SliderGrab]           = ImVec4(0.02f, 0.52f, 0.78f, 1.00f);
    colors[ImGuiCol_SliderGrabActive]     = ImVec4(0.01f, 0.60f, 0.88f, 1.00f);
    colors[ImGuiCol_Button]               = ImVec4(0.86f, 0.90f, 0.95f, 1.00f);
    colors[ImGuiCol_ButtonHovered]        = ImVec4(0.78f, 0.84f, 0.92f, 1.00f);
    colors[ImGuiCol_ButtonActive]         = ImVec4(0.70f, 0.78f, 0.88f, 1.00f);
    colors[ImGuiCol_Header]               = ImVec4(0.82f, 0.87f, 0.93f, 1.00f); // High contrast light slate header
    colors[ImGuiCol_HeaderHovered]        = ImVec4(0.74f, 0.81f, 0.89f, 1.00f);
    colors[ImGuiCol_HeaderActive]         = ImVec4(0.66f, 0.75f, 0.85f, 1.00f);
    colors[ImGuiCol_Tab]                  = ImVec4(0.88f, 0.92f, 0.96f, 1.00f);
    colors[ImGuiCol_TabHovered]           = ImVec4(0.02f, 0.52f, 0.78f, 0.80f);
    colors[ImGuiCol_TabActive]            = ImVec4(0.97f, 0.98f, 1.00f, 1.00f);
    colors[ImGuiCol_Text]                 = ImVec4(0.06f, 0.09f, 0.16f, 1.00f); // Bold high contrast dark charcoal
    colors[ImGuiCol_TextDisabled]         = ImVec4(0.35f, 0.42f, 0.50f, 1.00f);
}

MainWindow::MainWindow() {
    applyDarkTheme();
    loadPresetTemplate("buck_converter");
}

void MainWindow::startSimulation() {
    NetlistBuilder::buildNodesForCircuit(canvas.getCircuitRef());

    std::string jsonNetlist = NetlistSourceView::generateNetlistJson(canvas.getCircuit());

    std::vector<CircuitSimEngine::ComponentModel> physComps;
    std::vector<CircuitSimEngine::ComponentModel> ctrlComps;
    CircuitSimEngine::SimulationConfig simCfg;

    CircuitSimEngine::NetlistParser::parseJsonString(jsonNetlist, physComps, ctrlComps, simCfg);

    simulator.setup(physComps, ctrlComps, simCfg);
    CircuitSimEngine::SimulationOutput output = simulator.runTransient();
    simulator.setTelemetryOutput(output);

    scopeView.triggerAutoFit();
    netlistSourceView.triggerAutoFit();
}

void MainWindow::loadPresetTemplate(const std::string& name) {
    CircuitDesign cd;
    cd.settings.stopTime = 0.01;
    cd.settings.stepSize = 1e-5;
    cd.settings.solverType = "euler";
    cd.settings.stepType = "fixed";

    // 1. MOSFET1
    ComponentInstance m1;
    m1.id = "MOSFET1"; m1.label = "MOSFET1"; m1.type = ComponentType::MOSFET; m1.rawTypeStr = "MOSFET";
    m1.x = 380; m1.y = 60; m1.rotation = 0;
    m1.parameters["Ron"] = "10m"; m1.parameters["Roff"] = "1M";
    setupComponentPins(m1);
    cd.components.push_back(m1);

    // 2. D1
    ComponentInstance d1;
    d1.id = "D1"; d1.label = "D1"; d1.type = ComponentType::Diode; d1.rawTypeStr = "D";
    d1.x = 380; d1.y = 220; d1.rotation = 180;
    d1.parameters["Vd"] = "0.7"; d1.parameters["Ron"] = "1m"; d1.parameters["Roff"] = "1M";
    setupComponentPins(d1);
    cd.components.push_back(d1);

    // 3. V1
    ComponentInstance v1;
    v1.id = "V1"; v1.label = "V1"; v1.type = ComponentType::VoltageSource; v1.rawTypeStr = "V";
    v1.x = 220; v1.y = 160; v1.rotation = 0;
    v1.parameters["value"] = "100";
    setupComponentPins(v1);
    cd.components.push_back(v1);

    // 4. L1
    ComponentInstance l1;
    l1.id = "L1"; l1.label = "L1"; l1.type = ComponentType::Inductor; l1.rawTypeStr = "L";
    l1.x = 480; l1.y = 140; l1.rotation = 270;
    l1.parameters["L"] = "100u"; l1.parameters["esr"] = "50m"; l1.parameters["iL0"] = "0";
    setupComponentPins(l1);
    cd.components.push_back(l1);

    // 5. C1
    ComponentInstance c1;
    c1.id = "C1"; c1.label = "C1"; c1.type = ComponentType::Capacitor; c1.rawTypeStr = "C";
    c1.x = 560; c1.y = 220; c1.rotation = 0;
    c1.parameters["C"] = "100u"; c1.parameters["esr"] = "10m"; c1.parameters["vC0"] = "0";
    setupComponentPins(c1);
    cd.components.push_back(c1);

    // 6. R1
    ComponentInstance r1;
    r1.id = "R1"; r1.label = "R1"; r1.type = ComponentType::Resistor; r1.rawTypeStr = "R";
    r1.x = 620; r1.y = 220; r1.rotation = 0;
    r1.parameters["value"] = "10"; r1.parameters["esr"] = "0";
    setupComponentPins(r1);
    cd.components.push_back(r1);

    // 7. PULSE_GEN1
    ComponentInstance pulse1;
    pulse1.id = "PULSE_GEN1"; pulse1.label = "PULSE_GEN1"; pulse1.type = ComponentType::PulseGenerator; pulse1.rawTypeStr = "PULSE_GEN";
    pulse1.x = 280; pulse1.y = 60; pulse1.rotation = 0;
    pulse1.parameters["amplitude"] = "1"; pulse1.parameters["period"] = "1/10000"; pulse1.parameters["width"] = "0.5"; pulse1.parameters["delay"] = "0";
    setupComponentPins(pulse1);
    cd.components.push_back(pulse1);

    // Wires
    WireInstance w1; w1.id = "W1"; w1.from = {"PULSE_GEN1", "Out"}; w1.to = {"MOSFET1", "G"}; cd.wires.push_back(w1);
    WireInstance w2; w2.id = "W2"; w2.from = {"D1", "B"}; w2.to = {"MOSFET1", "S"}; cd.wires.push_back(w2);
    WireInstance w3; w3.id = "W3"; w3.from = {"V1", "A"}; w3.to = {"MOSFET1", "D"}; cd.wires.push_back(w3);
    WireInstance w4; w4.id = "W4"; w4.from = {"V1", "B"}; w4.to = {"D1", "A"}; cd.wires.push_back(w4);

    WireInstance w5; w5.id = "W5"; w5.from = {"L1", "A"}; w5.to.isWireJunction = true; w5.to.targetWireId = "W2"; w5.to.junctionX = 380.0f; w5.to.junctionY = 140.0f; cd.wires.push_back(w5);
    WireInstance w6; w6.id = "W6"; w6.from = {"C1", "A"}; w6.to = {"L1", "B"}; cd.wires.push_back(w6);
    WireInstance w7; w7.id = "W7"; w7.from = {"C1", "B"}; w7.to.isWireJunction = true; w7.to.targetWireId = "W4"; w7.to.junctionX = 380.0f; w7.to.junctionY = 280.0f; cd.wires.push_back(w7);
    WireInstance w8; w8.id = "W8"; w8.from = {"R1", "A"}; w8.to.isWireJunction = true; w8.to.targetWireId = "W6"; w8.to.junctionX = 560.0f; w8.to.junctionY = 140.0f; cd.wires.push_back(w8);
    WireInstance w9; w9.id = "W9"; w9.from = {"R1", "B"}; w9.to.isWireJunction = true; w9.to.targetWireId = "W7"; w9.to.junctionX = 560.0f; w9.to.junctionY = 280.0f; cd.wires.push_back(w9);

    PlotChannelConfig pChan;
    pChan.title = "Waveform analysis";
    pChan.variables = {"I_L1", "I_D1", "V_C1"};
    cd.plotConfig.plots = {pChan};

    NetlistBuilder::buildNodesForCircuit(cd);
    canvas.setCircuit(cd);
    simulator.loadCircuit(cd);
}

static std::string openFileDialog() {
    char szFile[260] = {0};
    OPENFILENAMEA ofn = {0};
    ofn.lStructSize = sizeof(ofn);
    ofn.lpstrFilter = "JSON Schematic (*.json)\0*.json\0All Files (*.*)\0*.*\0";
    ofn.lpstrFile = szFile;
    ofn.nMaxFile = sizeof(szFile);
    ofn.Flags = OFN_PATHMUSTEXIST | OFN_FILEMUSTEXIST;
    if (GetOpenFileNameA(&ofn)) return std::string(szFile);
    return "";
}

static std::string saveFileDialog() {
    char szFile[260] = {0};
    OPENFILENAMEA ofn = {0};
    ofn.lStructSize = sizeof(ofn);
    ofn.lpstrFilter = "JSON Schematic (*.json)\0*.json\0All Files (*.*)\0*.*\0";
    ofn.lpstrFile = szFile;
    ofn.nMaxFile = sizeof(szFile);
    ofn.Flags = OFN_PATHMUSTEXIST | OFN_OVERWRITEPROMPT;
    if (GetSaveFileNameA(&ofn)) {
        std::string res(szFile);
        if (res.find(".json") == std::string::npos) res += ".json";
        return res;
    }
    return "";
}

void MainWindow::renderMenuBar() {
    if (ImGui::BeginMainMenuBar()) {
        if (ImGui::BeginMenu("File")) {
            if (ImGui::MenuItem("New Workspace")) { canvas.setCircuit(CircuitDesign()); simulator.loadCircuit(CircuitDesign()); }
            if (ImGui::MenuItem("Open Schematic (.json)")) {
                std::string path = openFileDialog();
                if (!path.empty()) {
                    std::ifstream f(path);
                    if (f.is_open()) {
                        json j = json::parse(f);
                        CircuitDesign cd;
                        if (j.contains("components") && j["components"].is_array()) {
                            for (const auto& cItem : j["components"]) {
                                ComponentInstance comp;
                                comp.id = cItem.value("id", "");
                                comp.rawTypeStr = cItem.value("type", "R");
                                comp.type = stringToComponentType(comp.rawTypeStr);
                                comp.label = cItem.value("label", comp.id);
                                comp.x = cItem.value("x", 0.0f);
                                comp.y = cItem.value("y", 0.0f);
                                comp.rotation = cItem.value("rotation", 0);
                                if (cItem.contains("parameters") && cItem["parameters"].is_object()) {
                                    for (auto& [k, v] : cItem["parameters"].items()) {
                                        if (v.is_string()) comp.parameters[k] = v.get<std::string>();
                                    }
                                }
                                setupComponentPins(comp);
                                cd.components.push_back(comp);
                            }
                        }
                        if (j.contains("wires") && j["wires"].is_array()) {
                            for (const auto& wItem : j["wires"]) {
                                WireInstance wire;
                                wire.id = wItem.value("id", "");
                                if (wItem.contains("from") && wItem["from"].is_object()) {
                                    wire.from.compId = wItem["from"].value("compId", "");
                                    wire.from.terminal = wItem["from"].value("terminal", "");
                                }
                                if (wItem.contains("to") && wItem["to"].is_object()) {
                                    const auto& toObj = wItem["to"];
                                    std::string toComp = toObj.value("compId", "");
                                    std::string wireId = toObj.value("wireId", "");
                                    std::string toType = toObj.value("type", "pin");

                                    std::string targetW = !wireId.empty() ? wireId : toComp;

                                    if (toType == "wire" || toType == "junction" || (!targetW.empty() && (targetW[0] == 'w' || targetW[0] == 'W') && targetW.find(".") == std::string::npos)) {
                                        wire.to.isWireJunction = true;
                                        wire.to.targetWireId = targetW;
                                        wire.to.junctionX = toObj.value("x", 0.0f);
                                        wire.to.junctionY = toObj.value("y", 0.0f);
                                    } else {
                                        wire.to.isWireJunction = false;
                                        wire.to.compId = toComp;
                                        wire.to.terminal = toObj.value("terminal", "");
                                    }
                                }
                                cd.wires.push_back(wire);
                            }
                        }
                        if (j.contains("simulationSettings") && j["simulationSettings"].is_object()) {
                            const auto& ss = j["simulationSettings"];
                            if (ss.contains("stopTime")) {
                                if (ss["stopTime"].is_number()) cd.settings.stopTime = ss["stopTime"].get<double>();
                                else if (ss["stopTime"].is_string()) cd.settings.stopTime = CircuitSimEngine::ExpressionEvaluator::parseScientific(ss["stopTime"].get<std::string>());
                            }
                            if (ss.contains("stepSize")) {
                                if (ss["stepSize"].is_number()) cd.settings.stepSize = ss["stepSize"].get<double>();
                                else if (ss["stepSize"].is_string()) cd.settings.stepSize = CircuitSimEngine::ExpressionEvaluator::parseScientific(ss["stepSize"].get<std::string>());
                            }
                        }
                        if (j.contains("simulation_parameters") && j["simulation_parameters"].is_object()) {
                            const auto& sp = j["simulation_parameters"];
                            if (sp.contains("stop_time")) {
                                if (sp["stop_time"].is_number()) cd.settings.stopTime = sp["stop_time"].get<double>();
                                else if (sp["stop_time"].is_string()) cd.settings.stopTime = CircuitSimEngine::ExpressionEvaluator::parseScientific(sp["stop_time"].get<std::string>());
                            }
                            if (sp.contains("step_size")) {
                                if (sp["step_size"].is_number()) cd.settings.stepSize = sp["step_size"].get<double>();
                                else if (sp["step_size"].is_string()) cd.settings.stepSize = CircuitSimEngine::ExpressionEvaluator::parseScientific(sp["step_size"].get<std::string>());
                            }
                        }
                        canvas.setCircuit(cd);
                        simulator.loadCircuit(cd);
                    }
                }
            }
            if (ImGui::MenuItem("Save Schematic (.json)")) {
                std::string path = saveFileDialog();
                if (!path.empty()) {
                    json j;
                    json compArray = json::array();
                    const auto& cd = canvas.getCircuit();
                    for (const auto& comp : cd.components) {
                        json cObj;
                        cObj["id"] = comp.id;
                        cObj["type"] = comp.rawTypeStr;
                        cObj["label"] = comp.label;
                        cObj["x"] = comp.x;
                        cObj["y"] = comp.y;
                        cObj["rotation"] = comp.rotation;
                        json pObj = json::object();
                        for (const auto& [k, v] : comp.parameters) pObj[k] = v;
                        cObj["parameters"] = pObj;
                        compArray.push_back(cObj);
                    }
                    j["components"] = compArray;
                    json wireArray = json::array();
                    for (const auto& wire : cd.wires) {
                        json wObj;
                        wObj["id"] = wire.id;
                        json fObj; fObj["type"] = "pin"; fObj["compId"] = wire.from.compId; fObj["terminal"] = wire.from.terminal;
                        json tObj;
                        if (wire.to.isWireJunction) {
                            tObj["type"] = "junction";
                            tObj["compId"] = wire.to.targetWireId;
                            tObj["terminal"] = "";
                            tObj["x"] = wire.to.junctionX;
                            tObj["y"] = wire.to.junctionY;
                        } else {
                            tObj["type"] = "pin";
                            tObj["compId"] = wire.to.compId;
                            tObj["terminal"] = wire.to.terminal;
                        }
                        wObj["from"] = fObj; wObj["to"] = tObj;
                        wireArray.push_back(wObj);
                    }
                    j["wires"] = wireArray;
                    std::ofstream out(path);
                    if (out.is_open()) out << j.dump(2);
                }
            }
            ImGui::Separator();
            if (ImGui::MenuItem("Exit")) { exit(0); }
            ImGui::EndMenu();
        }
        if (ImGui::BeginMenu("Edit")) {
            if (ImGui::MenuItem("Undo", "Ctrl+Z")) { canvas.undo(); }
            if (ImGui::MenuItem("Redo", "Ctrl+Y")) { canvas.redo(); }
            ImGui::Separator();
            if (ImGui::MenuItem("Copy", "Ctrl+C")) { canvas.copySelected(); }
            if (ImGui::MenuItem("Paste", "Ctrl+V")) { canvas.pasteSelected(); }
            if (ImGui::MenuItem("Duplicate", "Ctrl+D")) { canvas.duplicateSelected(); }
            ImGui::Separator();
            if (ImGui::MenuItem("Flip Horizontal (H)")) { canvas.flipHorizontal(); }
            if (ImGui::MenuItem("Flip Vertical (V)")) { canvas.flipVertical(); }
            ImGui::EndMenu();
        }
        if (ImGui::BeginMenu("View")) {
            if (ImGui::MenuItem("Fit to Screen (F)")) { canvas.fitToScreen(); }
            if (ImGui::MenuItem("Component Pane", nullptr, showComponentPalette)) {
                showComponentPalette = !showComponentPalette;
            }
            ImGui::Separator();
            if (ImGui::BeginMenu("Appearance")) {
                if (ImGui::MenuItem("Dark Mode (Sleek Slate)", nullptr, isDarkMode)) {
                    isDarkMode = true;
                    applyDarkTheme();
                    canvas.setDarkMode(true);
                }
                if (ImGui::MenuItem("Light Mode (Clean Studio)", nullptr, !isDarkMode)) {
                    isDarkMode = false;
                    applyLightTheme();
                    canvas.setDarkMode(false);
                }
                ImGui::EndMenu();
            }
            ImGui::Separator();
            if (ImGui::BeginMenu("Workspace")) {
                if (ImGui::MenuItem("Schematic CAD", nullptr, activeWorkspace == WorkspaceMode::SchematicCAD)) {
                    activeWorkspace = WorkspaceMode::SchematicCAD;
                }
                if (ImGui::MenuItem("Waveform & Netlist", nullptr, activeWorkspace == WorkspaceMode::WaveformNetlist)) {
                    activeWorkspace = WorkspaceMode::WaveformNetlist;
                }
                ImGui::EndMenu();
            }
            ImGui::EndMenu();
        }
        if (ImGui::BeginMenu("Simulation")) {
            if (ImGui::MenuItem("Start", "Ctrl+T")) {
                startSimulation();
            }
            if (ImGui::MenuItem("Pause", "Space")) {
                simulator.pause();
            }
            if (ImGui::MenuItem("Simulation parameters...", "Ctrl+E")) {
                showSimParamsModal = true;
            }
            ImGui::EndMenu();
        }
        if (ImGui::BeginMenu("Templates")) {
            if (ImGui::MenuItem("Buck Converter")) { loadPresetTemplate("buck_converter"); }
            ImGui::EndMenu();
        }
        ImGui::EndMainMenuBar();
    }

    // Global Hotkeys for Simulation Menu
    ImGuiIO& io = ImGui::GetIO();
    if (!io.WantCaptureKeyboard) {
        if (io.KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_T)) {
            startSimulation();
        }
        if (io.KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_E)) {
            showSimParamsModal = true;
        }
    }
}

void MainWindow::renderControlBar() {
    ImGuiViewport* viewport = ImGui::GetMainViewport();
    float menuBarHeight = ImGui::GetFrameHeight();
    
    ImGui::SetNextWindowPos(ImVec2(viewport->Pos.x, viewport->Pos.y + menuBarHeight));
    ImGui::SetNextWindowSize(ImVec2(viewport->Size.x, 26.0f));
    
    ImGuiWindowFlags windowFlags = ImGuiWindowFlags_NoTitleBar | 
                                   ImGuiWindowFlags_NoResize | 
                                   ImGuiWindowFlags_NoMove | 
                                   ImGuiWindowFlags_NoScrollbar | 
                                   ImGuiWindowFlags_NoSavedSettings | 
                                   ImGuiWindowFlags_NoDocking |
                                   ImGuiWindowFlags_NoNavFocus;

    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(10.0f, 3.0f));
    ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
    ImVec4 toolbarBg = isDarkMode ? ImVec4(0.11f, 0.15f, 0.23f, 1.0f) : ImVec4(0.92f, 0.94f, 0.97f, 1.0f);
    ImGui::PushStyleColor(ImGuiCol_WindowBg, toolbarBg);

    if (ImGui::Begin("##TopToolbar", nullptr, windowFlags)) {
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0, 0, 0, 0)); // No button outlines
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.25f, 0.25f, 0.32f, 0.6f));
        ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(0.35f, 0.35f, 0.42f, 0.8f));
        ImGui::PushStyleVar(ImGuiStyleVar_FrameBorderSize, 0.0f);

        if (showComponentPalette) ImGui::PushStyleColor(ImGuiCol_Text, isDarkMode ? ImVec4(0.3f, 0.8f, 1.0f, 1.0f) : ImVec4(0.02f, 0.52f, 0.78f, 1.0f));
        if (ImGui::Button("Component Pane")) {
            showComponentPalette = !showComponentPalette;
        }
        if (showComponentPalette) ImGui::PopStyleColor();
        ImGui::SameLine(0, 15);

        ImGui::TextDisabled("|");
        ImGui::SameLine(0, 15);

        if (ImGui::Button("Play")) {
            startSimulation();
        }
        ImGui::SameLine(0, 15);

        if (ImGui::Button("Pause")) {
            simulator.pause();
        }
        ImGui::SameLine(0, 15);

        if (ImGui::Button("Reset")) {
            simulator.reset();
        }
        ImGui::SameLine(0, 15);

        if (ImGui::Button("Fit Schematic")) {
            canvas.fitToScreen();
        }
        ImGui::SameLine(0, 20);

        ImGui::TextDisabled("|");
        ImGui::SameLine(0, 20);

        ImGui::Text("Sim Time: %.5f s", simulator.getCurrentTime());
        ImGui::SameLine(0, 20);

        ImGui::TextDisabled("|");
        ImGui::SameLine(0, 20);

        bool isCadActive = (activeWorkspace == WorkspaceMode::SchematicCAD);
        if (isCadActive) ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.3f, 0.8f, 1.0f, 1.0f));
        if (ImGui::Button("Schematic CAD")) {
            activeWorkspace = WorkspaceMode::SchematicCAD;
        }
        if (isCadActive) ImGui::PopStyleColor();

        ImGui::SameLine(0, 15);

        bool isNetlistActive = (activeWorkspace == WorkspaceMode::WaveformNetlist);
        if (isNetlistActive) ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.4f, 0.9f, 0.5f, 1.0f));
        if (ImGui::Button("Waveform & Netlist")) {
            activeWorkspace = WorkspaceMode::WaveformNetlist;
            netlistSourceView.updateFromCircuit(canvas.getCircuit());
        }
        if (isNetlistActive) ImGui::PopStyleColor();

        ImGui::PopStyleVar(); // FrameBorderSize
        ImGui::PopStyleColor(3); // Button hover/active/bg
    }
    ImGui::End();
    ImGui::PopStyleColor(); // WindowBg
    ImGui::PopStyleVar(2); // WindowPadding, WindowBorderSize
}

void MainWindow::renderComponentPalette() {
    if (!ImGui::Begin("Component Pane", &showComponentPalette)) {
        ImGui::End();
        return;
    }
    
    auto getUniqueId = [&](const std::string& prefix) -> std::string {
        const auto& comps = canvas.getCircuit().components;
        int idx = 1;
        while (true) {
            std::string cand = prefix + std::to_string(idx);
            bool found = false;
            for (const auto& c : comps) {
                if (c.id == cand) {
                    found = true;
                    break;
                }
            }
            if (!found) return cand;
            idx++;
        }
    };

    if (ImGui::CollapsingHeader("⚡ Electrical Components", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::Indent(8.0f);
        if (ImGui::Button("Resistor (R)")) {
            ComponentInstance comp;
            comp.id = getUniqueId("R");
            comp.label = "Resistor"; comp.type = ComponentType::Resistor; comp.rawTypeStr = "R";
            comp.parameters["value"] = "1k";
            canvas.addComponent(comp);
        }
        if (ImGui::Button("Capacitor (C)")) {
            ComponentInstance comp;
            comp.id = getUniqueId("C");
            comp.label = "Capacitor"; comp.type = ComponentType::Capacitor; comp.rawTypeStr = "C";
            comp.parameters["C"] = "10u";
            canvas.addComponent(comp);
        }
        if (ImGui::Button("Inductor (L)")) {
            ComponentInstance comp;
            comp.id = getUniqueId("L");
            comp.label = "Inductor"; comp.type = ComponentType::Inductor; comp.rawTypeStr = "L";
            comp.parameters["L"] = "1m";
            canvas.addComponent(comp);
        }
        if (ImGui::Button("DC Voltage Source (V)")) {
            ComponentInstance comp;
            comp.id = getUniqueId("V");
            comp.label = "DC Source"; comp.type = ComponentType::VoltageSource; comp.rawTypeStr = "V";
            comp.parameters["value"] = "12";
            canvas.addComponent(comp);
        }
        if (ImGui::Button("AC Voltage Source")) {
            ComponentInstance comp;
            comp.id = getUniqueId("ACV");
            comp.label = "AC Source"; comp.type = ComponentType::ACVoltageSource; comp.rawTypeStr = "AC_V";
            comp.parameters["Vm"] = "325"; comp.parameters["f"] = "50";
            canvas.addComponent(comp);
        }
        if (ImGui::Button("Current Source (I)")) {
            ComponentInstance comp;
            comp.id = getUniqueId("I");
            comp.label = "I Source"; comp.type = ComponentType::CurrentSource; comp.rawTypeStr = "I";
            comp.parameters["value"] = "1";
            canvas.addComponent(comp);
        }
        if (ImGui::Button("Diode (D)")) {
            ComponentInstance comp;
            comp.id = getUniqueId("D");
            comp.label = "Diode"; comp.type = ComponentType::Diode; comp.rawTypeStr = "D";
            comp.parameters["Vf"] = "0.7"; comp.parameters["Ron"] = "10m";
            canvas.addComponent(comp);
        }
        if (ImGui::Button("MOSFET")) {
            ComponentInstance comp;
            comp.id = getUniqueId("MOSFET");
            comp.label = "MOSFET"; comp.type = ComponentType::MOSFET; comp.rawTypeStr = "MOSFET";
            comp.parameters["Ron"] = "10m"; comp.parameters["Roff"] = "1M";
            canvas.addComponent(comp);
        }
        if (ImGui::Button("Switch (S)")) {
            ComponentInstance comp;
            comp.id = getUniqueId("S");
            comp.label = "Switch"; comp.type = ComponentType::Switch; comp.rawTypeStr = "S";
            comp.parameters["Ron"] = "10m"; comp.parameters["Roff"] = "1M";
            canvas.addComponent(comp);
        }
        if (ImGui::Button("Ground (GND)")) {
            ComponentInstance comp;
            comp.id = getUniqueId("GND");
            comp.label = "GND"; comp.type = ComponentType::Unknown; comp.rawTypeStr = "GND";
            canvas.addComponent(comp);
        }
        if (ImGui::Button("Voltmeter (VM)")) {
            ComponentInstance comp;
            comp.id = getUniqueId("VM");
            comp.label = "Voltmeter"; comp.type = ComponentType::Voltmeter; comp.rawTypeStr = "VM";
            canvas.addComponent(comp);
        }
        if (ImGui::Button("Ammeter (AM)")) {
            ComponentInstance comp;
            comp.id = getUniqueId("AM");
            comp.label = "Ammeter"; comp.type = ComponentType::Ammeter; comp.rawTypeStr = "AM";
            canvas.addComponent(comp);
        }
        ImGui::Unindent(8.0f);
        ImGui::Spacing();
    }

    if (ImGui::CollapsingHeader("🎛️ Control & Math Blocks", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::Indent(8.0f);
        if (ImGui::Button("Gain (K)")) {
            ComponentInstance comp;
            comp.id = getUniqueId("GAIN");
            comp.label = "Gain"; comp.type = ComponentType::Gain; comp.rawTypeStr = "GAIN";
            comp.parameters["K"] = "1.0";
            canvas.addComponent(comp);
        }
        if (ImGui::Button("PID Controller")) {
            ComponentInstance comp;
            comp.id = getUniqueId("PID");
            comp.label = "PID"; comp.type = ComponentType::PI_Controller; comp.rawTypeStr = "PID";
            comp.parameters["Kp"] = "1.0"; comp.parameters["Ki"] = "10"; comp.parameters["Kd"] = "0";
            canvas.addComponent(comp);
        }
        if (ImGui::Button("Comparator")) {
            ComponentInstance comp;
            comp.id = getUniqueId("COMP");
            comp.label = "Comparator"; comp.type = ComponentType::Comparator; comp.rawTypeStr = "COMP";
            canvas.addComponent(comp);
        }
        if (ImGui::Button("PWM Generator")) {
            ComponentInstance comp;
            comp.id = getUniqueId("PWM");
            comp.label = "PWM"; comp.type = ComponentType::PWM_Generator; comp.rawTypeStr = "PWM";
            comp.parameters["frequency"] = "20000";
            canvas.addComponent(comp);
        }
        if (ImGui::Button("Pulse Generator")) {
            ComponentInstance comp;
            comp.id = getUniqueId("PULSE_GEN");
            comp.label = "Pulse Gen"; comp.type = ComponentType::PulseGenerator; comp.rawTypeStr = "PULSE_GEN";
            comp.parameters["amplitude"] = "1";
            comp.parameters["period"] = "1";
            comp.parameters["width"] = "0.5";
            comp.parameters["delay"] = "0";
            canvas.addComponent(comp);
        }
        if (ImGui::Button("Triangle Carrier")) {
            ComponentInstance comp;
            comp.id = getUniqueId("TRI");
            comp.label = "Triangle"; comp.type = ComponentType::Triangle_Carrier; comp.rawTypeStr = "TRI";
            canvas.addComponent(comp);
        }
        if (ImGui::Button("Sum (SUM_RECT)")) {
            ComponentInstance comp;
            comp.id = getUniqueId("SUM");
            comp.label = "Sum"; comp.type = ComponentType::SummingJunction; comp.rawTypeStr = "SUM_RECT";
            canvas.addComponent(comp);
        }
        if (ImGui::Button("Sum Round (SUM_ROUND)")) {
            ComponentInstance comp;
            comp.id = getUniqueId("SUM");
            comp.label = "Sum (Round)"; comp.type = ComponentType::SummingJunction; comp.rawTypeStr = "SUM_ROUND";
            canvas.addComponent(comp);
        }
        if (ImGui::Button("Product (PRODUCT_RECT)")) {
            ComponentInstance comp;
            comp.id = getUniqueId("PRODUCT");
            comp.label = "Product"; comp.type = ComponentType::Product; comp.rawTypeStr = "PRODUCT_RECT";
            canvas.addComponent(comp);
        }
        if (ImGui::Button("AND Gate")) {
            ComponentInstance comp;
            comp.id = getUniqueId("AND");
            comp.label = "AND"; comp.type = ComponentType::AND_Gate; comp.rawTypeStr = "AND";
            canvas.addComponent(comp);
        }
        if (ImGui::Button("OR Gate")) {
            ComponentInstance comp;
            comp.id = getUniqueId("OR");
            comp.label = "OR"; comp.type = ComponentType::OR_Gate; comp.rawTypeStr = "OR";
            canvas.addComponent(comp);
        }
        if (ImGui::Button("NOT Gate")) {
            ComponentInstance comp;
            comp.id = getUniqueId("NOT");
            comp.label = "NOT"; comp.type = ComponentType::NOT_Gate; comp.rawTypeStr = "NOT";
            canvas.addComponent(comp);
        }
        if (ImGui::Button("C-Script (CSCRIPT)")) {
            ComponentInstance comp;
            comp.id = getUniqueId("CSCRIPT");
            comp.label = "C-Script"; comp.type = ComponentType::CustomScript; comp.rawTypeStr = "CSCRIPT";
            comp.parameters["code"] = "// Step code\noutputs[0] = inputs[0] * 2.0;\n";
            canvas.addComponent(comp);
        }
        if (ImGui::Button("Subsystem Block")) {
            ComponentInstance comp;
            comp.id = getUniqueId("SUBSYSTEM");
            comp.label = "Subsystem"; comp.type = ComponentType::Unknown; comp.rawTypeStr = "SUBSYSTEM";
            canvas.addComponent(comp);
        }
        ImGui::Unindent(8.0f);
        ImGui::Spacing();
    }

    if (ImGui::CollapsingHeader("📊 Scope & Probe", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::Indent(8.0f);
        if (ImGui::Button("Oscilloscope (SCOPE)")) {
            ComponentInstance comp;
            comp.id = getUniqueId("SCOPE");
            comp.label = "Oscilloscope"; comp.type = ComponentType::Unknown; comp.rawTypeStr = "SCOPE";
            comp.parameters["channels"] = "2";
            canvas.addComponent(comp);
        }
        if (ImGui::Button("Active Probe (PROBE)")) {
            ComponentInstance comp;
            comp.id = getUniqueId("PROBE");
            comp.label = "Active Probe"; comp.type = ComponentType::Unknown; comp.rawTypeStr = "PROBE";
            comp.parameters["target"] = "";
            comp.parameters["selected_signals"] = "";
            canvas.addComponent(comp);
        }
        ImGui::Unindent(8.0f);
        ImGui::Spacing();
    }

    ImGui::End();
}

void MainWindow::renderPropertyInspector() {
    ImGui::Begin("Property Inspector");
    
    ComponentInstance* comp = canvas.getSelectedComponent();
    if (!comp) {
        ImGui::TextColored(ImVec4(0.6f, 0.6f, 0.6f, 1.0f), "Select a component on canvas to view and edit properties.");
        ImGui::End();
        return;
    }

    ImGui::Text("Selected: %s (%s)", comp->id.c_str(), comp->label.c_str());
    ImGui::Separator();
    
    char labelBuf[128];
    strncpy(labelBuf, comp->label.c_str(), sizeof(labelBuf));
    if (ImGui::InputText("Label", labelBuf, sizeof(labelBuf))) {
        comp->label = labelBuf;
    }

    ImGui::Spacing();
    ImGui::TextColored(ImVec4(0.2f, 0.8f, 1.0f, 1.0f), "Parameters:");
    
    for (auto& pair : comp->parameters) {
        if (pair.first == "probe_signal" || pair.first == "plotI" || pair.first == "plotV" || pair.first == "target" || pair.first == "selected_signals") continue;
        char valBuf[256];
        strncpy(valBuf, pair.second.c_str(), sizeof(valBuf));
        if (ImGui::InputText(pair.first.c_str(), valBuf, sizeof(valBuf))) {
            pair.second = valBuf;
        }
    }

    ImGui::Spacing();
    ImGui::Separator();
    ImGui::TextColored(ImVec4(0.2f, 0.8f, 1.0f, 1.0f), "Signals to Plot / Probe:");
    ImGui::Spacing();

    // Determine signals to plot/probe based on component type (matching Web Tool)
    std::vector<std::string> availableSignals;
    std::string t = comp->rawTypeStr;
    std::transform(t.begin(), t.end(), t.begin(), ::toupper);

    bool isElectrical = (t == "R" || t == "L" || t == "C" || t == "V" || t == "AC_V" || t == "I" || t == "S" || t == "D" || t == "MOSFET" || t == "VM" || t == "AM" || t == "GND");
    if (isElectrical) {
        availableSignals.push_back("V_" + comp->id);
        availableSignals.push_back("I_" + comp->id);
        if (t == "MOSFET" || t == "S") {
            availableSignals.push_back("Ctrl_" + comp->id);
        }
    } else if (t != "SCOPE" && t != "PROBE") {
        // Control / Math component signals
        availableSignals.push_back(comp->id + ".Out");
        if (t == "CSCRIPT") {
            availableSignals.push_back(comp->id + ".Out1");
            availableSignals.push_back(comp->id + ".Out2");
            availableSignals.push_back(comp->id + ".Out3");
            availableSignals.push_back(comp->id + ".Out4");
        } else if (t == "SUM_RECT" || t == "SUM_ROUND" || t == "PRODUCT_RECT" || t == "COMP" || t == "AND" || t == "OR") {
            availableSignals.push_back(comp->id + ".A");
            availableSignals.push_back(comp->id + ".B");
        }
    }

    auto& cd = canvas.getCircuitRef();
    if (cd.plotConfig.plots.empty()) {
        cd.plotConfig.plots.push_back({ "Waveform Analysis", {} });
    }
    auto& plotVars = cd.plotConfig.plots[0].variables;

    for (const auto& sigName : availableSignals) {
        auto it = std::find(plotVars.begin(), plotVars.end(), sigName);
        bool isChecked = (it != plotVars.end());

        if (ImGui::Checkbox(sigName.c_str(), &isChecked)) {
            if (isChecked) {
                if (it == plotVars.end()) plotVars.push_back(sigName);
            } else {
                if (it != plotVars.end()) plotVars.erase(it);
            }
            canvas.syncProbeSignals();
        }
    }

    ImGui::End();
}

void MainWindow::renderSimParamsModal() {
    if (showSimParamsModal) {
        ImGui::OpenPopup("Simulation Parameters Modal");
        if (ImGui::BeginPopupModal("Simulation Parameters Modal", &showSimParamsModal, ImGuiWindowFlags_AlwaysAutoResize)) {
            if (ImGui::IsWindowAppearing()) {
                const auto& cd = canvas.getCircuit();
                std::snprintf(simStopTimeBuf, sizeof(simStopTimeBuf), "%.17g", cd.settings.stopTime);
                std::snprintf(simStepSizeBuf, sizeof(simStepSizeBuf), "%.17g", cd.settings.stepSize);
                if (cd.settings.solverType == "trapezoidal") simSolverIdx = 1;
                else if (cd.settings.solverType == "rk4") simSolverIdx = 2;
                else simSolverIdx = 0;
            }

            ImGui::TextColored(ImVec4(0.2f, 0.8f, 1.0f, 1.0f), "Solver & Simulation Configuration");
            ImGui::Separator();
            ImGui::Spacing();

            ImGui::InputText("Stop Time (s)", simStopTimeBuf, sizeof(simStopTimeBuf));
            ImGui::InputText("Step Size (s)", simStepSizeBuf, sizeof(simStepSizeBuf));
            
            const char* solverItems[] = { "Euler (Fixed Step)", "Trapezoidal (Gear/BE)", "Runge-Kutta 4th Order (RK4)" };
            ImGui::Combo("Solver Method", &simSolverIdx, solverItems, IM_ARRAYSIZE(solverItems));

            ImGui::Spacing();
            ImGui::Separator();
            ImGui::Spacing();

            if (ImGui::Button("OK", ImVec2(120, 30))) {
                auto& cd = canvas.getCircuitRef();
                try { cd.settings.stopTime = CircuitSimEngine::ExpressionEvaluator::parseScientific(simStopTimeBuf); } catch (...) {}
                try { cd.settings.stepSize = CircuitSimEngine::ExpressionEvaluator::parseScientific(simStepSizeBuf); } catch (...) {}
                cd.settings.solverType = (simSolverIdx == 0) ? "euler" : ((simSolverIdx == 1) ? "trapezoidal" : "rk4");
                
                showSimParamsModal = false;
                ImGui::CloseCurrentPopup();
            }
            ImGui::SameLine();
            if (ImGui::Button("Cancel", ImVec2(100, 30))) {
                showSimParamsModal = false;
                ImGui::CloseCurrentPopup();
            }
            ImGui::EndPopup();
        }
    }
}

void MainWindow::render() {
    renderMenuBar();
    renderControlBar();

    if (activeWorkspace == WorkspaceMode::SchematicCAD) {
        renderComponentPalette();
        renderPropertyInspector();
        canvas.render("Schematic Editor Canvas", ImVec2(800, 600));
        scopeView.render("Real-Time Oscilloscope Waveforms", simulator);
    } else {
        netlistSourceView.render("Waveform Solver & Raw Netlist Workspace", canvas.getCircuitRef(), simulator);
    }

    renderSimParamsModal();
}

} // namespace CircuitSim
