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
#include <unordered_set>
#include <set>

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

    ImPlotStyle& plotStyle = ImPlot::GetStyle();
    ImVec4* pColors = plotStyle.Colors;
    pColors[ImPlotCol_FrameBg]      = ImVec4(0.09f, 0.13f, 0.20f, 1.00f);
    pColors[ImPlotCol_PlotBg]       = ImVec4(0.06f, 0.09f, 0.16f, 1.00f);
    pColors[ImPlotCol_PlotBorder]   = ImVec4(0.20f, 0.27f, 0.38f, 1.00f);
    pColors[ImPlotCol_LegendBg]     = ImVec4(0.09f, 0.13f, 0.20f, 0.85f);
    pColors[ImPlotCol_LegendBorder] = ImVec4(0.20f, 0.27f, 0.38f, 1.00f);
    pColors[ImPlotCol_LegendText]   = ImVec4(0.95f, 0.96f, 0.98f, 1.00f);
    pColors[ImPlotCol_TitleText]    = ImVec4(0.35f, 0.75f, 1.00f, 1.00f);
    pColors[ImPlotCol_AxisText]     = ImVec4(0.85f, 0.90f, 0.96f, 1.00f);
    pColors[ImPlotCol_AxisGrid]     = ImVec4(0.18f, 0.24f, 0.35f, 1.00f);
    pColors[ImPlotCol_Selection]    = ImVec4(0.00f, 0.75f, 1.00f, 0.30f);
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

    ImPlotStyle& plotStyle = ImPlot::GetStyle();
    ImVec4* pColors = plotStyle.Colors;
    pColors[ImPlotCol_FrameBg]      = ImVec4(0.95f, 0.97f, 0.99f, 1.00f);
    pColors[ImPlotCol_PlotBg]       = ImVec4(0.98f, 0.97f, 0.90f, 1.00f); // Warm light background
    pColors[ImPlotCol_PlotBorder]   = ImVec4(0.65f, 0.72f, 0.80f, 1.00f);
    pColors[ImPlotCol_LegendBg]     = ImVec4(1.00f, 1.00f, 1.00f, 0.92f);
    pColors[ImPlotCol_LegendBorder] = ImVec4(0.70f, 0.75f, 0.82f, 1.00f);
    pColors[ImPlotCol_LegendText]   = ImVec4(0.05f, 0.08f, 0.15f, 1.00f); // Bold dark charcoal text
    pColors[ImPlotCol_TitleText]    = ImVec4(0.02f, 0.40f, 0.65f, 1.00f);
    pColors[ImPlotCol_AxisText]     = ImVec4(0.05f, 0.08f, 0.15f, 1.00f); // Bold dark charcoal text
    pColors[ImPlotCol_AxisGrid]     = ImVec4(0.78f, 0.82f, 0.88f, 1.00f);
    pColors[ImPlotCol_Selection]    = ImVec4(0.02f, 0.52f, 0.78f, 0.30f);
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
                                        else if (v.is_number()) comp.parameters[k] = std::to_string(v.get<double>());
                                        else if (v.is_boolean()) comp.parameters[k] = v.get<bool>() ? "true" : "false";
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

                            // Sanitize wire IDs and fix duplicate wire targets
                            std::unordered_set<std::string> seenWIds;
                            int maxWNum = 0;
                            for (const auto& w : cd.wires) {
                                if (w.id.size() > 1 && (w.id[0] == 'w' || w.id[0] == 'W')) {
                                    try {
                                        int num = std::stoi(w.id.substr(1));
                                        if (num > maxWNum) maxWNum = num;
                                    } catch (...) {}
                                }
                            }
                            for (auto& w : cd.wires) {
                                if (w.id.empty() || seenWIds.count(w.id)) {
                                    maxWNum++;
                                    std::string newId = "w" + std::to_string(maxWNum);
                                    std::string oldId = w.id;
                                    w.id = newId;
                                    for (auto& tw : cd.wires) {
                                        if (tw.to.isWireJunction && tw.to.targetWireId == oldId) {
                                            tw.to.targetWireId = newId;
                                        }
                                    }
                                }
                                seenWIds.insert(w.id);
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
                        canvas.fitToScreen();
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

        bool pushedStyle = showComponentPalette;
        if (pushedStyle) ImGui::PushStyleColor(ImGuiCol_Text, isDarkMode ? ImVec4(0.3f, 0.8f, 1.0f, 1.0f) : ImVec4(0.02f, 0.52f, 0.78f, 1.0f));
        if (ImGui::Button("Component Pane")) {
            showComponentPalette = !showComponentPalette;
        }
        if (pushedStyle) ImGui::PopStyleColor();
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
        ImGui::SameLine(0, 10);

        // Adaptive Box Zoom toggle
        bool zoomActive = canvas.isAdaptiveZoomMode();
        if (zoomActive) ImGui::PushStyleColor(ImGuiCol_Text, isDarkMode ? ImVec4(0.3f, 1.0f, 0.5f, 1.0f) : ImVec4(0.05f, 0.55f, 0.15f, 1.0f));
        if (ImGui::Button(zoomActive ? "[Box Zoom ON]" : "Box Zoom")) {
            canvas.toggleAdaptiveZoom();
        }
        if (zoomActive) ImGui::PopStyleColor();
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
    if (!showComponentPalette) return;

    if (ImGui::Begin("Component Pane", &showComponentPalette)) {
        
        // 1. Library Selector Tabs (Basic Lib vs Detailed Lib)
        float availW = ImGui::GetContentRegionAvail().x;
        float tabW = availW * 0.5f - 4.0f;

        if (!showDetailedLibrary) {
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.12f, 0.25f, 0.45f, 1.0f));
            ImGui::Button("Basic Lib", ImVec2(tabW, 26));
            ImGui::PopStyleColor();
            ImGui::SameLine();
            if (ImGui::Button("Detailed Lib", ImVec2(tabW, 26))) {
                showDetailedLibrary = true;
            }
        } else {
            if (ImGui::Button("Basic Lib", ImVec2(tabW, 26))) {
                showDetailedLibrary = false;
            }
            ImGui::SameLine();
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.12f, 0.25f, 0.45f, 1.0f));
            ImGui::Button("Detailed Lib", ImVec2(tabW, 26));
            ImGui::PopStyleColor();
        }

        ImGui::Spacing();
        ImGui::Separator();
        ImGui::Spacing();

        // 2. Search Bar
        ImGui::InputTextWithHint("##PaletteSearch", "Search components...", searchPaletteBuf, sizeof(searchPaletteBuf));
        std::string searchQuery = searchPaletteBuf;
        std::transform(searchQuery.begin(), searchQuery.end(), searchQuery.begin(), ::tolower);

        ImGui::Spacing();

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

        // Helper lambda to render component button
        auto renderCompButton = [&](const char* buttonText, const std::string& prefix, const std::string& label, ComponentType type, const std::string& rawTypeStr, const std::vector<std::pair<std::string, std::string>>& defaultParams = {}) {
            ImGui::PushID(rawTypeStr.c_str());
            if (ImGui::Button(buttonText)) {
                ComponentInstance comp;
                comp.id = getUniqueId(prefix);
                comp.label = label;
                comp.type = type;
                comp.rawTypeStr = rawTypeStr;
                for (const auto& p : defaultParams) {
                    comp.parameters[p.first] = p.second;
                }
                canvas.addComponent(comp);
            }
            ImGui::PopID();
        };

        // Helper metadata for library component categorization matching Web Tool
        struct ComponentMeta {
            const char* buttonText;
            const char* label;
            const char* prefix;
            ComponentType type;
            const char* rawTypeStr;
            const char* category;       // "general", "control", "electrical"
            const char* subcategory;    // Subheading matching webtool
            std::vector<std::pair<std::string, std::string>> defaultParams;
            bool isBasic = false;       // Only original basic library blocks are true
        };

        static const std::vector<ComponentMeta> allComponents = {
            // General Domain
            { "Subsystem Block", "Subsystem", "SUBSYSTEM", ComponentType::Unknown, "SUBSYSTEM", "general", "Ports and Subsystems", {}, true },
            { "Active Probe (PROBE)", "Active Probe", "PROBE", ComponentType::Unknown, "PROBE", "general", "Signal Routing", {{"target", ""}, {"selected_signals", ""}}, true },
            { "Oscilloscope (SCOPE)", "Oscilloscope", "SCOPE", ComponentType::Unknown, "SCOPE", "general", "Visualization & Logging", {{"channels", "2"}}, true },

            // Control Domain (Sources)
            { "Constant (CONST)", "Constant", "CONST", ComponentType::Constant, "CONST", "control", "Sources", {{"value", "1.0"}}, true },
            { "Clock (CLOCK)", "Clock", "CLOCK", ComponentType::Clock, "CLOCK", "control", "Sources", {}, false },
            { "Initial Condition (INIT_COND)", "Initial Condition", "INIT_COND", ComponentType::InitialCondition, "INIT_COND", "control", "Sources", {{"initial_value", "0"}}, false },
            { "Pulse Generator (PULSE_GEN)", "Pulse Gen", "PULSE_GEN", ComponentType::PulseGenerator, "PULSE_GEN", "control", "Sources", {{"amplitude", "1"}, {"period", "1"}, {"width", "0.5"}, {"delay", "0"}}, true },
            { "Ramp (RAMP)", "Ramp", "RAMP", ComponentType::Ramp, "RAMP", "control", "Sources", {{"slope", "1"}, {"start_time", "0"}, {"initial_output", "0"}}, false },
            { "Random Numbers (RANDOM_NUM)", "Random Numbers", "RANDOM_NUM", ComponentType::RandomNumbers, "RANDOM_NUM", "control", "Sources", {{"mean", "0"}, {"std", "1"}}, false },
            { "Sine Wave (SINE_WAVE)", "Sine Wave", "SINE_WAVE", ComponentType::SineWave, "SINE_WAVE", "control", "Sources", {{"amplitude", "1"}, {"frequency", "50"}, {"phase", "0"}}, false },
            { "Step (STEP)", "Step", "STEP", ComponentType::Step, "STEP", "control", "Sources", {{"step_time", "1"}, {"initial_value", "0"}, {"final_value", "1"}}, false },
            { "Triangular Wave Generator (TRI_GEN)", "Triangle", "TRI", ComponentType::Triangle_Carrier, "TRI_GEN", "control", "Sources", {{"frequency", "10k"}, {"min", "0"}, {"max", "1"}}, true },
            { "White Noise (WHITE_NOISE)", "White Noise", "WHITE_NOISE", ComponentType::WhiteNoise, "WHITE_NOISE", "control", "Sources", {{"psd", "0.1"}}, false },

            // Functions & Tables
            { "Gain Scalar (GAIN)", "Gain", "GAIN", ComponentType::Gain, "GAIN", "control", "Functions & Tables", {{"K", "2.5"}}, true },
            { "Product (PROD)", "Product", "PROD", ComponentType::Product, "PRODUCT_RECT", "control", "Functions & Tables", {{"operators", "*/"}}, true },
            { "Trigonometric Function (TRIG_FCN)", "Trig", "TRIG_FCN", ComponentType::TrigFunction, "TRIG_FCN", "control", "Functions & Tables", {{"function", "sin"}}, false },
            { "Math Function (MATH_FCN)", "Math Function", "MATH_FCN", ComponentType::MathFunction, "MATH_FCN", "control", "Functions & Tables", {{"function", "exp"}}, true },
            { "Abs (ABS)", "Abs", "ABS", ComponentType::Abs, "ABS", "control", "Functions & Tables", {}, false },
            { "Sign (SIGN)", "Sign", "SIGN", ComponentType::Sign, "SIGN", "control", "Functions & Tables", {}, false },
            { "Round (ROUND)", "Round", "ROUND", ComponentType::Round, "ROUND", "control", "Functions & Tables", {{"mode", "nearest"}}, false },
            { "Min/Max (MIN_MAX)", "MinMax", "MIN_MAX", ComponentType::MinMax, "MIN_MAX", "control", "Functions & Tables", {{"function", "min"}}, false },
            { "1D Look-Up Table (LUT_1D)", "LUT 1D", "LUT_1D", ComponentType::LUT_1D, "LUT_1D", "control", "Functions & Tables", {{"x_data", "[0, 1, 2]"}, {"y_data", "[0, 2, 4]"}}, false },
            { "2D Look-Up Table (LUT_2D)", "LUT 2D", "LUT_2D", ComponentType::LUT_2D, "LUT_2D", "control", "Functions & Tables", {{"x_data", "[0, 1]"}, {"y_data", "[0, 1]"}, {"z_data", "[[0, 1], [1, 2]]"}}, false },
            { "3D Look-Up Table (LUT_3D)", "LUT 3D", "LUT_3D", ComponentType::LUT_3D, "LUT_3D", "control", "Functions & Tables", {}, false },
            { "C-Script (CSCRIPT)", "C-Script", "CSCRIPT", ComponentType::CustomScript, "CSCRIPT", "control", "Functions & Tables", {{"code", "// Step code\noutputs[0] = inputs[0] * 2.0;\n"}}, true },
            { "DLL (DLL)", "DLL", "DLL", ComponentType::DLL, "DLL", "control", "Functions & Tables", {}, false },
            { "FMU (FMU)", "FMU", "FMU", ComponentType::FMU, "FMU", "control", "Functions & Tables", {}, false },
            { "Fourier Series (FOURIER_SERIES)", "Fourier", "FOURIER_SERIES", ComponentType::FourierSeries, "FOURIER_SERIES", "control", "Functions & Tables", {}, false },

            // Continuous
            { "Integrator (INTEGRATOR)", "1/s", "INTEGRATOR", ComponentType::Integrator, "INTEGRATOR", "control", "Continuous", {{"initial_condition", "0"}}, false },
            { "Derivative (DERIVATIVE)", "s", "DERIVATIVE", ComponentType::Derivative, "DERIVATIVE", "control", "Continuous", {}, false },
            { "Transfer Function (TRANSFER_FCN)", "G(s)", "TRANSFER_FCN", ComponentType::TransferFunction, "TRANSFER_FCN", "control", "Continuous", {{"num", "[1]"}, {"den", "[1, 1]"}}, false },
            { "State Space (STATE_SPACE)", "State Space", "STATE_SPACE", ComponentType::StateSpace, "STATE_SPACE", "control", "Continuous", {{"A", "[-1]"}, {"B", "[1]"}, {"C", "[1]"}, {"D", "[0]"}, {"x0", "0"}}, false },
            { "PI Controller (PID)", "PID", "PID", ComponentType::PI_Controller, "PID", "control", "Continuous", {{"Kp", "2.5"}, {"Ki", "50.0"}, {"Kd", "0"}}, true },
            { "Continuous PID Controller (CONT_PID)", "PID(s)", "CONT_PID", ComponentType::ContinuousPID, "CONT_PID", "control", "Continuous", {{"Kp", "1.0"}, {"Ki", "0.0"}, {"Kd", "0.0"}, {"Tf", "0.01"}}, false },
            { "Single-Phase PLL (PLL_1PH)", "PLL 1Ph", "PLL_1PH", ComponentType::PLL_1PH, "PLL_1PH", "control", "Continuous", {{"fn", "50.0"}, {"Kp", "20.0"}, {"Ki", "1000.0"}}, false },
            // Delays
            { "Delay (DELAY)", "Delay", "DELAY", ComponentType::Delay, "DELAY", "control", "Delays", {{"delay", "0.1"}}, false },
            { "Transport Delay (TRANSPORT_DELAY)", "e^-sT", "TRANSPORT_DELAY", ComponentType::TransportDelay, "TRANSPORT_DELAY", "control", "Delays", {{"delay", "0.1"}}, false },
            { "Turn-on Delay (TURN_ON_DELAY)", "Ton", "TURN_ON_DELAY", ComponentType::TurnOnDelay, "TURN_ON_DELAY", "control", "Delays", {{"delay", "0.05"}}, false },
            { "Memory (MEMORY_BLOCK)", "Mem", "MEMORY_BLOCK", ComponentType::MemoryBlock, "MEMORY_BLOCK", "control", "Delays", {{"initial_value", "0.0"}}, false },

            // Discontinuous
            { "Quantizer (QUANTIZER)", "Quantize", "QUANTIZER", ComponentType::Quantizer, "QUANTIZER", "control", "Discontinuous", {{"step_size", "0.5"}, {"mode", "round"}}, false },
            { "Signal Switch (SIGNAL_SWITCH)", "Switch", "SIGNAL_SWITCH", ComponentType::SignalSwitch, "SIGNAL_SWITCH", "control", "Discontinuous", {{"threshold", "0.5"}, {"criteria", "u2 >= threshold"}}, false },
            { "Manual Signal Switch (MANUAL_SWITCH)", "Manual Sw", "MANUAL_SWITCH", ComponentType::ManualSwitch, "MANUAL_SWITCH", "control", "Discontinuous", {{"state", "Input 1"}}, false },
            { "Multiport Signal Switch (MULTIPORT_SWITCH)", "Multiport Sw", "MULTIPORT_SWITCH", ComponentType::MultiportSwitch, "MULTIPORT_SWITCH", "control", "Discontinuous", {{"inputs", "3"}, {"indexing", "1-based"}}, false },
            { "Hit Crossing (HIT_CROSSING)", "Hit Cross", "HIT_CROSSING", ComponentType::HitCrossing, "HIT_CROSSING", "control", "Discontinuous", {{"offset", "0.0"}, {"direction", "either"}}, false },
            { "Saturation (SATURATION)", "Sat", "SATURATION", ComponentType::Saturation, "SATURATION", "control", "Discontinuous", {{"min", "-10"}, {"max", "10"}}, false },
            { "Dead Zone (DEAD_ZONE)", "Dead", "DEAD_ZONE", ComponentType::DeadZone, "DEAD_ZONE", "control", "Discontinuous", {{"start", "-0.5"}, {"end", "0.5"}}, false },
            { "Rate Limiter (RATE_LIMITER)", "Rate", "RATE_LIMITER", ComponentType::RateLimiter, "RATE_LIMITER", "control", "Discontinuous", {{"up", "10"}, {"down", "-10"}}, false },
            { "Relay (RELAY)", "Relay", "RELAY", ComponentType::Relay, "RELAY", "control", "Discontinuous", {{"on_threshold", "1"}, {"off_threshold", "-1"}}, false },
            { "Comparator (COMP)", "COMP", "COMP", ComponentType::Comparator, "COMP", "control", "Discontinuous", {}, true },

            // Logical & Bitwise sub-library
            { "AND Gate", "AND", "AND", ComponentType::AND_Gate, "AND", "control", "Logical & Bitwise", {}, true },
            { "OR Gate", "OR", "OR", ComponentType::OR_Gate, "OR", "control", "Logical & Bitwise", {}, true },
            { "NOT Gate", "NOT", "NOT", ComponentType::NOT_Gate, "NOT", "control", "Logical & Bitwise", {}, true },
            { "Edge Detector (EDGE_DETECT)", "Edge", "EDGE_DETECT", ComponentType::EdgeDetect, "EDGE_DETECT", "control", "Logical & Bitwise", {{"edge", "rising"}, {"pulse_width", "1e-3"}}, true },
            { "Logical Operator (LOGIC_OP)", "Logic", "LOGIC_OP", ComponentType::LogicOp, "LOGIC_OP", "control", "Logical & Bitwise", {{"operator", "AND"}}, false },
            { "Bitwise Operator (BITWISE_OP)", "Bitwise", "BITWISE_OP", ComponentType::BitwiseOp, "BITWISE_OP", "control", "Logical & Bitwise", {{"operator", "AND"}}, false },
            { "Combinational Logic (COMB_LOGIC)", "Truth", "COMB_LOGIC", ComponentType::CombLogic, "COMB_LOGIC", "control", "Logical & Bitwise", {{"truth_table", "00:0,01:1,10:1,11:0"}}, false },
            { "Monostable (MONOSTABLE)", "Mono", "MONOSTABLE", ComponentType::Monostable, "MONOSTABLE", "control", "Logical & Bitwise", {{"duration", "0.1"}, {"edge", "rising"}}, false },
            { "Monoflop (MONOFLOP)", "Monoflop", "MONOFLOP", ComponentType::Monoflop, "MONOFLOP", "control", "Logical & Bitwise", {{"duration", "0.1"}, {"trigger_edge", "rising"}, {"retriggerable", "false"}}, false },
            { "Relational Operator (RELATIONAL_OPERATOR)", "RelOp", "RELATIONAL_OPERATOR", ComponentType::RelationalOp, "RELATIONAL_OPERATOR", "control", "Logical & Bitwise", {{"operator", "=="}}, false },
            { "Compare to Constant (COMPARE_TO_CONSTANT)", "CmpConst", "COMPARE_TO_CONSTANT", ComponentType::CompareToConstant, "COMPARE_TO_CONSTANT", "control", "Logical & Bitwise", {{"operator", "=="}, {"constant", "0.0"}}, false },
            { "D Flip-Flop (D_FLIP_FLOP)", "D-FF", "D_FLIP_FLOP", ComponentType::DFlipFlop, "D_FLIP_FLOP", "control", "Logical & Bitwise", {{"initial_state", "0.0"}, {"trigger_edge", "rising"}}, false },
            { "JK Flip-Flop (JK_FLIP_FLOP)", "JK-FF", "JK_FLIP_FLOP", ComponentType::JKFlipFlop, "JK_FLIP_FLOP", "control", "Logical & Bitwise", {{"initial_state", "0.0"}, {"trigger_edge", "rising"}}, false },
            { "Shift Register (SHIFT_REG)", "Shift", "SHIFT_REG", ComponentType::ShiftReg, "SHIFT_REG", "control", "Logical & Bitwise", {{"length", "4"}}, false },

            // Modulators sub-library
            { "PWM Generator", "PWM", "PWM", ComponentType::PWM_Generator, "PWM", "control", "Modulators", {{"frequency", "20000"}}, true },
            { "Master PWM (PWM_MASTER)", "Master PWM", "PWM_MASTER", ComponentType::MasterPWM, "PWM_MASTER", "control", "Modulators", {{"num_carriers", "3"}, {"fc", "10k"}, {"dead_time", "1u"}}, true },
            { "PWM (3-Phase) (PWM_3PH)", "PWM 3Ph", "PWM_3PH", ComponentType::PWM_3PH, "PWM_3PH", "control", "Modulators", {{"frequency", "10k"}}, false },
            { "Space Vector PWM (SVPWM)", "SVPWM", "SVPWM", ComponentType::SVPWM, "SVPWM", "control", "Modulators", {{"frequency", "10k"}}, false },

            // Signal Transforms sub-library
            { "Clarke Transform (CLARKE)", "abc-αβ", "CLARKE", ComponentType::Clarke, "CLARKE", "control", "Signal Transforms", {}, false },
            { "Park Transform (PARK)", "αβ-dq", "PARK", ComponentType::Park, "PARK", "control", "Signal Transforms", {}, false },
            { "Inverse Clarke Transform (INV_CLARKE)", "αβ-abc", "INV_CLARKE", ComponentType::InvClarke, "INV_CLARKE", "control", "Signal Transforms", {}, false },
            { "Inverse Park Transform (INV_PARK)", "dq-αβ", "INV_PARK", ComponentType::InvPark, "INV_PARK", "control", "Signal Transforms", {}, false },

            // Filters & Measurements sub-library
            { "Periodic Average (PER_AVG)", "Avg T", "PER_AVG", ComponentType::PerAvg, "PER_AVG", "control", "Filters & Measurements", {{"period", "0.02"}}, false },
            { "Periodic Impulse Average (PERIODIC_IMP_AVG)", "Imp Avg", "PERIODIC_IMP_AVG", ComponentType::PeriodicImpAvg, "PERIODIC_IMP_AVG", "control", "Filters & Measurements", {{"initial_value", "0.0"}}, false },
            { "Fourier Transform (FOURIER_TRANS)", "Fourier", "FOURIER_TRANS", ComponentType::FourierTrans, "FOURIER_TRANS", "control", "Filters & Measurements", {{"f", "50.0"}, {"harmonic", "1"}, {"ts", "100u"}}, false },
            { "Moving Average (MOV_AVG)", "MovAvg", "MOV_AVG", ComponentType::MovAvg, "MOV_AVG", "control", "Filters & Measurements", {{"window", "10"}}, false },
            { "First-Order Filter (FILTER_1ST)", "Filt1st", "FILTER_1ST", ComponentType::Filter1st, "FILTER_1ST", "control", "Filters & Measurements", {{"type", "Lowpass"}, {"fc", "1k"}}, false },
            { "Second-Order Filter (FILTER_2ND)", "Filt2nd", "FILTER_2ND", ComponentType::Filter2nd, "FILTER_2ND", "control", "Filters & Measurements", {{"type", "Lowpass"}, {"fc", "1k"}, {"Q", "0.707"}}, false },
            { "Fourier Analysis (FOURIER_ANALYSIS)", "Fourier", "FOURIER_ANALYSIS", ComponentType::FourierAnalysis, "FOURIER_ANALYSIS", "control", "Filters & Measurements", {{"f", "50.0"}}, false },
            { "RMS (RMS_VAL)", "RMS", "RMS_VAL", ComponentType::RmsVal, "RMS_VAL", "control", "Filters & Measurements", {{"frequency", "50"}}, false },
            { "THD (THD_VAL)", "THD", "THD_VAL", ComponentType::ThdVal, "THD_VAL", "control", "Filters & Measurements", {{"frequency", "50"}}, false },
            { "PLL (Phase-Locked Loop) (PLL_LOOP)", "PLL", "PLL_LOOP", ComponentType::PllLoop, "PLL_LOOP", "control", "Filters & Measurements", {{"fn", "50.0"}, {"Kp", "20.0"}, {"Ki", "1000.0"}}, false },

            // State Machines sub-library
            { "State Machine (STATE_MACHINE)", "State", "STATE_MACHINE", ComponentType::StateMachine, "STATE_MACHINE", "control", "State Machines", {}, false },

            // Math sub-library
            { "Offset (OFFSET)", "OFFSET", "OFFSET", ComponentType::Offset, "OFFSET", "control", "Math", {{"offset", "0.0"}}, false },
            { "Summing Block (SUM)", "SUM", "SUM", ComponentType::SummingJunction, "SUM", "control", "Math", {{"signs", "++"}}, false },
            { "Sum (round) (SUM_ROUND)", "SUM", "SUM_ROUND", ComponentType::SummingJunction, "SUM_ROUND", "control", "Math", {{"inputs", "2"}, {"signs", "++"}}, true },
            { "Sum (rectangular) (SUM_RECT)", "SUM", "SUM_RECT", ComponentType::SummingJunction, "SUM_RECT", "control", "Math", {{"inputs", "2"}, {"signs", "++"}}, true },
            { "Subtract (SUBTRACT)", "SUB", "SUBTRACT", ComponentType::SummingJunction, "SUBTRACT", "control", "Math", {{"signs", "+-"}}, false },
            { "Product (rectangular) (PRODUCT_RECT)", "PROD", "PRODUCT_RECT", ComponentType::Product, "PRODUCT_RECT", "control", "Math", {{"inputs", "2"}, {"operators", "**"}}, true },
            { "Signum (SIGNUM)", "sgn", "SIGNUM", ComponentType::Signum, "SIGNUM", "control", "Math", {}, false },
            { "Divide (DIVIDE)", "DIV", "DIVIDE", ComponentType::Divide, "DIVIDE", "control", "Math", {{"operators", "*/"}}, false },
            { "Data Type Conversion (DATATYPE_CONV)", "Cast", "DATATYPE_CONV", ComponentType::DataTypeConv, "DATATYPE_CONV", "control", "Math", {{"datatype", "boolean"}}, false },

            // Electrical Domain Connectivity sub-library
            { "Electrical Port (E_PORT)", "E-Port", "E_PORT", ComponentType::ElectricalPort, "E_PORT", "electrical", "Connectivity", {}, false },
            { "Electrical Label (E_LABEL)", "E-Tag", "E_LABEL", ComponentType::ElectricalLabel, "E_LABEL", "electrical", "Connectivity", {{"label", "A"}}, false },
            { "Ground (GND)", "GND", "GND", ComponentType::Unknown, "GND", "electrical", "Connectivity", {}, true },

            // Electrical Domain Sources sub-library
            { "DC Voltage Source (V)", "DC_V", "V", ComponentType::VoltageSource, "V", "electrical", "Sources", {{"value", "24"}}, true },
            { "DC Current Source (I)", "DC_I", "I", ComponentType::CurrentSource, "I", "electrical", "Sources", {{"value", "1"}}, true },
            { "AC Voltage Source (AC_V)", "AC_V", "AC_V", ComponentType::ACVoltageSource, "AC_V", "electrical", "Sources", {{"amplitude", "12"}, {"frequency", "50"}, {"phase", "0"}}, true },
            { "AC Current Source (AC_I)", "AC_I", "AC_I", ComponentType::ACCurrentSource, "AC_I", "electrical", "Sources", {{"amplitude", "1"}, {"frequency", "50"}, {"phase", "0"}}, false },
            { "Controlled Voltage Source (CTRL_V)", "Src V", "CTRL_V", ComponentType::ControlledVoltageSource, "CTRL_V", "electrical", "Sources", {}, false },
            { "Controlled Current Source (CTRL_I)", "Src I", "CTRL_I", ComponentType::ControlledCurrentSource, "CTRL_I", "electrical", "Sources", {}, false },
            { "3-Phase Voltage Source (V_3PH)", "3Ph V", "V3PH", ComponentType::ThreePhaseSource, "V_3PH", "electrical", "Sources", {{"magnitude", "230"}, {"frequency", "50"}, {"phase", "0"}, {"connection", "Y"}}, true },
            { "3-Phase Current Source (I_3PH)", "3Ph I", "I3PH", ComponentType::ThreePhaseCurrentSource, "I_3PH", "electrical", "Sources", {}, false },

            // Electrical Domain Meters (Sensors) sub-library
            { "Voltmeter (VM)", "VM", "VM", ComponentType::Voltmeter, "VM", "electrical", "Meters (Sensors)", {}, true },
            { "Ammeter (AM)", "AM", "AM", ComponentType::Ammeter, "AM", "electrical", "Meters (Sensors)", {}, true },
            { "Voltage Meter (3-Phase) (VM_3PH)", "3VM", "VM_3PH", ComponentType::Voltmeter3Ph, "VM_3PH", "electrical", "Meters (Sensors)", {}, false },
            { "Current Meter (3-Phase) (AM_3PH)", "3AM", "AM_3PH", ComponentType::Ammeter3Ph, "AM_3PH", "electrical", "Meters (Sensors)", {}, false },

            // Electrical Domain Passive Components sub-library
            { "Resistor (R)", "R", "R", ComponentType::Resistor, "R", "electrical", "Passive Components", {{"value", "10"}}, true },
            { "Inductor (L)", "L", "L", ComponentType::Inductor, "L", "electrical", "Passive Components", {{"L", "10m"}, {"esr", "0"}}, true },
            { "Capacitor (C)", "C", "C", ComponentType::Capacitor, "C", "electrical", "Passive Components", {{"C", "100u"}, {"esr", "0"}}, true },
            { "Variable Resistor (VAR_R)", "var R", "VAR_R", ComponentType::VariableResistor, "VAR_R", "electrical", "Passive Components", {{"value", "10"}}, false },
            { "Variable Inductor (VAR_L)", "var L", "VAR_L", ComponentType::VariableInductor, "VAR_L", "electrical", "Passive Components", {{"L", "10m"}}, false },
            { "Variable Capacitor (VAR_C)", "var C", "VAR_C", ComponentType::VariableCapacitor, "VAR_C", "electrical", "Passive Components", {{"C", "100u"}}, false },
            { "Saturable Inductor (SAT_L)", "sat L", "SAT_L", ComponentType::SaturableInductor, "SAT_L", "electrical", "Passive Components", {{"L", "10m"}}, false },
            { "Saturable Capacitor (SAT_C)", "sat C", "SAT_C", ComponentType::SaturableCapacitor, "SAT_C", "electrical", "Passive Components", {{"C", "100u"}}, false },
            { "Pi-Section Line (PI_SECTION)", "Pi Line", "PI_SECTION", ComponentType::PiSectionLine, "PI_SECTION", "electrical", "Passive Components", {}, false },
            { "Transmission Line (3ph) (LINE_3PH)", "3Ph Line", "LINE_3PH", ComponentType::TransmissionLine3Ph, "LINE_3PH", "electrical", "Passive Components", {}, false },
            { "Piece-wise Linear Resistor (PWL_R)", "pwl R", "PWL_R", ComponentType::PWLResistor, "PWL_R", "electrical", "Passive Components", {{"value", "10"}}, false },
            { "Electrical Algebraic Component (E_ALGEBRAIC)", "E-Alg", "E_ALGEBRAIC", ComponentType::ElectricalAlgebraic, "E_ALGEBRAIC", "electrical", "Passive Components", {}, false },

            // Signals Routing sub-library
            { "Signal Goto (GOTO_SIG)", "Goto", "GOTO_SIG", ComponentType::GotoSignal, "GOTO_SIG", "control", "Signals Routing", {{"tag", "S1"}}, false },
            { "Signal From (FROM_SIG)", "From", "FROM_SIG", ComponentType::FromSignal, "FROM_SIG", "control", "Signals Routing", {{"tag", "S1"}}, false },

            // Electrical Domain Power Semiconductors (Ideal Behavioral Switches) sub-library
            { "Diode (D)", "D", "D", ComponentType::Diode, "D", "electrical", "Power Semiconductors (Ideal Behavioral Switches)", {{"Vd", "0.7"}, {"Ron", "1m"}, {"Roff", "1M"}}, true },
            { "Thyristor (THYRISTOR)", "SCR", "THYRISTOR", ComponentType::Thyristor, "THYRISTOR", "electrical", "Power Semiconductors (Ideal Behavioral Switches)", {{"Vd", "0.8"}, {"Ron", "1m"}, {"Roff", "1M"}, {"Iholding", "10m"}, {"Vgt", "0.5"}}, true },
            { "GTO (Gate Turn-Off) (GTO)", "GTO", "GTO", ComponentType::GTO, "GTO", "electrical", "Power Semiconductors (Ideal Behavioral Switches)", {{"Vd", "0.8"}, {"Ron", "1m"}, {"Roff", "1M"}}, false },
            { "IGBT (IGBT)", "IGBT", "IGBT", ComponentType::Switch, "IGBT", "electrical", "Power Semiconductors (Ideal Behavioral Switches)", {{"Ron", "10m"}, {"Roff", "1M"}}, true },
            { "IGBT with Diode (IGBT_DIODE)", "IGBT+D", "IGBT_DIODE", ComponentType::IGBTDiode, "IGBT_DIODE", "electrical", "Power Semiconductors (Ideal Behavioral Switches)", {{"Ron", "10m"}, {"Roff", "1M"}}, false },
            { "IGCT (IGCT)", "IGCT", "IGCT", ComponentType::IGCT, "IGCT", "electrical", "Power Semiconductors (Ideal Behavioral Switches)", {{"Vd", "0.8"}, {"Ron", "1m"}, {"Roff", "1M"}}, false },
            { "MOSFET (MOSFET)", "MOSFET", "MOSFET", ComponentType::MOSFET, "MOSFET", "electrical", "Power Semiconductors (Ideal Behavioral Switches)", {{"Ron", "10m"}, {"Roff", "1M"}}, true },
            { "Voltage-Gated FET (vg-FET)", "vg-FET", "vg-FET", ComponentType::VGFET, "vg-FET", "electrical", "Power Semiconductors (Ideal Behavioral Switches)", {{"Gate_Signal_Label", "S1"}, {"Ron", "10m"}, {"Roff", "1M"}}, true },
            { "BJT (BJT)", "BJT", "BJT", ComponentType::BJT, "BJT", "electrical", "Power Semiconductors (Ideal Behavioral Switches)", {{"Vd", "0.7"}, {"Ron", "1m"}, {"Roff", "1M"}}, false },
            { "JFET (JFET)", "JFET", "JFET", ComponentType::JFET, "JFET", "electrical", "Power Semiconductors (Ideal Behavioral Switches)", {{"Ron", "10m"}, {"Roff", "1M"}}, false },

            // Electrical Domain Switches sub-library
            { "Switch (S)", "S", "S", ComponentType::Switch, "S", "electrical", "Switches", {{"Ron", "10m"}, {"Roff", "1M"}}, true },
            { "Breaker (BREAKER)", "Breaker", "BREAKER", ComponentType::Breaker, "BREAKER", "electrical", "Switches", {{"Ron", "10m"}, {"Roff", "1M"}}, false },
            { "Double Switch (DBL_SWITCH)", "2-Way", "DBL_SWITCH", ComponentType::DoubleSwitch, "DBL_SWITCH", "electrical", "Switches", {{"Ron", "10m"}, {"Roff", "1M"}}, false },
            { "Manual Switch (MAN_SWITCH)", "Man Sw", "MAN_SWITCH", ComponentType::ElectricalManualSwitch, "MAN_SWITCH", "electrical", "Switches", {{"Ron", "10m"}, {"Roff", "1M"}}, false },
            { "Manual Double Switch (MAN_DBL_SWITCH)", "Man 2Sw", "MAN_DBL_SWITCH", ComponentType::ManualDoubleSwitch, "MAN_DBL_SWITCH", "electrical", "Switches", {{"Ron", "10m"}, {"Roff", "1M"}}, false },
            { "Manual Triple Switch (MAN_TRPL_SWITCH)", "Man 3Sw", "MAN_TRPL_SWITCH", ComponentType::ManualTripleSwitch, "MAN_TRPL_SWITCH", "electrical", "Switches", {{"Ron", "10m"}, {"Roff", "1M"}}, false },
            { "Set/Reset Switch (SR_SWITCH)", "SR Sw", "SR_SWITCH", ComponentType::SRSwitch, "SR_SWITCH", "electrical", "Switches", {{"Ron", "10m"}, {"Roff", "1M"}}, false },
            // Electrical Domain Transformers sub-library
            { "Ideal Transformer (IDEAL_XFMR)", "Ideal XFMR", "IDEAL_XFMR", ComponentType::IdealTransformer, "IDEAL_XFMR", "electrical", "Transformers", {{"primary_turns", "100"}, {"secondary_turns", "100"}}, false },
            { "Linear Transformer (2-Winding) (XFMR_2W)", "XFMR 2W", "XFMR_2W", ComponentType::Transformer2W, "XFMR_2W", "electrical", "Transformers", {{"primary_turns", "100"}, {"secondary_turns", "100"}}, true },
            { "Linear Transformer (3-Winding) (XFMR_3W)", "XFMR 3W", "XFMR_3W", ComponentType::Transformer3W, "XFMR_3W", "electrical", "Transformers", {{"primary_turns", "100"}, {"secondary_turns", "100, 100"}}, false },
            { "Mutual Inductor (2-Winding) (MUTUAL_2W)", "Mut 2W", "MUTUAL_2W", ComponentType::MutualInductor2W, "MUTUAL_2W", "electrical", "Transformers", {{"primary_turns", "100"}, {"secondary_turns", "100"}}, false },
            { "Mutual Inductor (3-Winding) (MUTUAL_3W)", "Mut 3W", "MUTUAL_3W", ComponentType::MutualInductor3W, "MUTUAL_3W", "electrical", "Transformers", {{"primary_turns", "100"}, {"secondary_turns", "100, 100"}}, false },
            { "Saturable Transformer (SAT_XFMR)", "Sat XFMR", "SAT_XFMR", ComponentType::SaturableTransformer, "SAT_XFMR", "electrical", "Transformers", {{"primary_turns", "100"}, {"secondary_turns", "100"}}, false },
            { "Transformers (3-Phase 2-Winding) (XFMR_3PH_2W)", "3Ph 2W XFMR", "XFMR_3PH_2W", ComponentType::Transformer3Ph2W, "XFMR_3PH_2W", "electrical", "Transformers", {}, false },
            // Electrical Domain Electrical Machines sub-library
            { "Induction Motor (1-Phase/3-Phase) (INDUCTION_MOTOR)", "Ind Motor 1Ph/3Ph", "INDUCTION_MOTOR", ComponentType::InductionMotor, "INDUCTION_MOTOR", "electrical", "Electrical Machines", {{"Rs", "1.115"}}, false },
            { "Induction Motor (IND_MOTOR)", "Ind Motor", "IND_MOTOR", ComponentType::InductionMotor, "IND_MOTOR", "electrical", "Electrical Machines", {{"Rs", "1.115"}}, false },

            // Electrical Domain Electronics sub-library
            { "Operational Amplifier (OPAMP)", "OpAmp", "OPAMP", ComponentType::OpAmp, "OPAMP", "electrical", "Electronics", {{"gain", "1e5"}}, false },
            { "Comparator (Electrical) (E_COMP)", "E-Comp", "E_COMP", ComponentType::EComp, "E_COMP", "electrical", "Electronics", {{"gain", "1e5"}}, false },

            // Electrical Domain Custom Machine/Load Models sub-library
            { "Generalized Electrical Block (GEN_EBLOCK)", "Gen E-Block", "GEN_EBLOCK", ComponentType::GenEBlock, "GEN_EBLOCK", "electrical", "Custom Machine/Load Models", {{"terminals", "3"}}, true },
        };

        // Filter search results if search query is non-empty
        if (!searchQuery.empty()) {
            ImGui::TextColored(ImVec4(0.2f, 0.8f, 1.0f, 1.0f), "🔍 Search Results");
            ImGui::Separator();
            ImGui::Spacing();
            int matchCount = 0;
            for (const auto& item : allComponents) {
                std::string textLower = item.buttonText;
                std::string rawLower = item.rawTypeStr;
                std::string subcatLower = item.subcategory;
                std::transform(textLower.begin(), textLower.end(), textLower.begin(), ::tolower);
                std::transform(rawLower.begin(), rawLower.end(), rawLower.begin(), ::tolower);
                std::transform(subcatLower.begin(), subcatLower.end(), subcatLower.begin(), ::tolower);

                if (textLower.find(searchQuery) != std::string::npos ||
                    rawLower.find(searchQuery) != std::string::npos ||
                    subcatLower.find(searchQuery) != std::string::npos) {
                    
                    matchCount++;
                    renderCompButton(item.buttonText, item.prefix, item.label, item.type, item.rawTypeStr, item.defaultParams);
                    ImGui::SameLine();
                    ImGui::TextDisabled("[%s]", item.subcategory);
                }
            }
            if (matchCount == 0) {
                ImGui::TextDisabled("No matching components found.");
            }
        }
        else if (!showDetailedLibrary) {
            // BASIC LIBRARY VIEW (Only simple original basic blocks)
            if (ImGui::CollapsingHeader("⚡ Power Stage", ImGuiTreeNodeFlags_DefaultOpen)) {
                ImGui::PushID("basic_power");
                ImGui::Indent(8.0f);
                for (const auto& item : allComponents) {
                    if (item.isBasic && std::string(item.category) == "electrical") {
                        renderCompButton(item.buttonText, item.prefix, item.label, item.type, item.rawTypeStr, item.defaultParams);
                    }
                }
                ImGui::Unindent(8.0f);
                ImGui::Spacing();
                ImGui::PopID();
            }

            if (ImGui::CollapsingHeader("🎛️ Control Loops", ImGuiTreeNodeFlags_DefaultOpen)) {
                ImGui::PushID("basic_control");
                ImGui::Indent(8.0f);
                for (const auto& item : allComponents) {
                    if (item.isBasic && std::string(item.category) == "control") {
                        renderCompButton(item.buttonText, item.prefix, item.label, item.type, item.rawTypeStr, item.defaultParams);
                    }
                }
                ImGui::Unindent(8.0f);
                ImGui::Spacing();
                ImGui::PopID();
            }

            if (ImGui::CollapsingHeader("📊 Scope & Probes", ImGuiTreeNodeFlags_DefaultOpen)) {
                ImGui::PushID("basic_general");
                ImGui::Indent(8.0f);
                for (const auto& item : allComponents) {
                    if (item.isBasic && std::string(item.category) == "general") {
                        renderCompButton(item.buttonText, item.prefix, item.label, item.type, item.rawTypeStr, item.defaultParams);
                    }
                }
                ImGui::Unindent(8.0f);
                ImGui::Spacing();
                ImGui::PopID();
            }
        }
        else {
            // DETAILED LIBRARY VIEW (Categorized with ALL Web Tool Headings & Subheadings)
            
            auto renderSubheading = [&](const char* subcatName, const std::vector<const ComponentMeta*>& compList) {
                if (ImGui::TreeNodeEx(subcatName, ImGuiTreeNodeFlags_DefaultOpen)) {
                    ImGui::Indent(4.0f);
                    if (compList.empty()) {
                        ImGui::TextDisabled("(No blocks available yet)");
                    } else {
                        for (const auto* item : compList) {
                            renderCompButton(item->buttonText, item->prefix, item->label, item->type, item->rawTypeStr, item->defaultParams);
                        }
                    }
                    ImGui::Unindent(4.0f);
                    ImGui::TreePop();
                }
                ImGui::Spacing();
            };

            // 1. GENERAL BLOCKS (Category: general)
            if (ImGui::CollapsingHeader("⚙️ General Blocks", ImGuiTreeNodeFlags_DefaultOpen)) {
                ImGui::PushID("cat_general");
                ImGui::Indent(8.0f);
                
                std::vector<const ComponentMeta*> portsSubsys, sigRouting, visLogging, execControl;
                for (const auto& item : allComponents) {
                    if (std::string(item.category) == "general") {
                        std::string sub = item.subcategory;
                        if (sub == "Ports and Subsystems") portsSubsys.push_back(&item);
                        else if (sub == "Signal Routing") sigRouting.push_back(&item);
                        else if (sub == "Visualization & Logging") visLogging.push_back(&item);
                        else if (sub == "Execution Control & Tools") execControl.push_back(&item);
                    }
                }

                renderSubheading("Ports and Subsystems", portsSubsys);
                renderSubheading("Signal Routing", sigRouting);
                renderSubheading("Visualization & Logging", visLogging);
                renderSubheading("Execution Control & Tools", execControl);

                ImGui::Unindent(8.0f);
                ImGui::Spacing();
                ImGui::PopID();
            }

            // 2. CONTROL BLOCKS (Category: control)
            if (ImGui::CollapsingHeader("🎛️ Control Blocks", ImGuiTreeNodeFlags_DefaultOpen)) {
                ImGui::PushID("cat_control");
                ImGui::Indent(8.0f);
                
                std::vector<const ComponentMeta*> sources, fnTables, continuous, delays, discrete, discontinuous, logicBitwise, modulators, sigTransforms, filtersMeas, stateMachines, math;
                for (const auto& item : allComponents) {
                    if (std::string(item.category) == "control") {
                        std::string sub = item.subcategory;
                        if (sub == "Sources") sources.push_back(&item);
                        else if (sub == "Functions & Tables") fnTables.push_back(&item);
                        else if (sub == "Continuous") continuous.push_back(&item);
                        else if (sub == "Delays") delays.push_back(&item);
                        else if (sub == "Discrete-Time Dynamics") discrete.push_back(&item);
                        else if (sub == "Discontinuous") discontinuous.push_back(&item);
                        else if (sub == "Logical & Bitwise") logicBitwise.push_back(&item);
                        else if (sub == "Modulators") modulators.push_back(&item);
                        else if (sub == "Signal Transforms") sigTransforms.push_back(&item);
                        else if (sub == "Filters & Measurements") filtersMeas.push_back(&item);
                        else if (sub == "State Machines") stateMachines.push_back(&item);
                        else if (sub == "Math") math.push_back(&item);
                    }
                }

                renderSubheading("Sources", sources);
                renderSubheading("Functions & Tables", fnTables);
                renderSubheading("Continuous", continuous);
                renderSubheading("Delays", delays);
                renderSubheading("Discrete-Time Dynamics", discrete);
                renderSubheading("Discontinuous", discontinuous);
                renderSubheading("Logical & Bitwise", logicBitwise);
                renderSubheading("Modulators", modulators);
                renderSubheading("Signal Transforms", sigTransforms);
                renderSubheading("Filters & Measurements", filtersMeas);
                renderSubheading("State Machines", stateMachines);
                renderSubheading("Math", math);

                ImGui::Unindent(8.0f);
                ImGui::Spacing();
                ImGui::PopID();
            }

            // 3. ELECTRICAL BLOCKS (Category: electrical)
            if (ImGui::CollapsingHeader("⚡ Electrical Blocks", ImGuiTreeNodeFlags_DefaultOpen)) {
                ImGui::PushID("cat_electrical");
                ImGui::Indent(8.0f);
                
                std::vector<const ComponentMeta*> connect, elecSources, meters, passives, semiSwitches, switches, transformers, machines, electronics, ics, customLoads;
                for (const auto& item : allComponents) {
                    if (std::string(item.category) == "electrical") {
                        std::string sub = item.subcategory;
                        if (sub == "Connectivity") connect.push_back(&item);
                        else if (sub == "Sources") elecSources.push_back(&item);
                        else if (sub == "Meters (Sensors)") meters.push_back(&item);
                        else if (sub == "Passive Components") passives.push_back(&item);
                        else if (sub == "Power Semiconductors (Ideal Behavioral Switches)") semiSwitches.push_back(&item);
                        else if (sub == "Switches") switches.push_back(&item);
                        else if (sub == "Transformers") transformers.push_back(&item);
                        else if (sub == "Electrical Machines") machines.push_back(&item);
                        else if (sub == "Electronics") electronics.push_back(&item);
                        else if (sub == "Integrated Circuits (ICs)") ics.push_back(&item);
                        else if (sub == "Custom Machine/Load Models") customLoads.push_back(&item);
                    }
                }

                renderSubheading("Connectivity", connect);
                renderSubheading("Sources", elecSources);
                renderSubheading("Meters (Sensors)", meters);
                renderSubheading("Passive Components", passives);
                renderSubheading("Power Semiconductors (Ideal Behavioral Switches)", semiSwitches);
                renderSubheading("Switches", switches);
                renderSubheading("Transformers", transformers);
                renderSubheading("Electrical Machines", machines);
                renderSubheading("Electronics", electronics);
                renderSubheading("Integrated Circuits (ICs)", ics);
                renderSubheading("Custom Machine/Load Models", customLoads);

                ImGui::Unindent(8.0f);
                ImGui::Spacing();
                ImGui::PopID();
            }
        }
    }
    ImGui::End();
}

void MainWindow::renderPropertyInspector() {
    ComponentInstance* comp = canvas.getSelectedComponent();
    if (!comp) return;

    if (ImGui::Begin("Property Inspector")) {
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
        char valBuf[256] = {0};
        strncpy(valBuf, pair.second.c_str(), sizeof(valBuf) - 1);
        std::string inputLabel = pair.first + "##" + comp->id;
        if (ImGui::InputText(inputLabel.c_str(), valBuf, sizeof(valBuf))) {
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
        }
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
    scopeView.setDarkMode(isDarkMode);
    netlistSourceView.setDarkMode(isDarkMode);

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
