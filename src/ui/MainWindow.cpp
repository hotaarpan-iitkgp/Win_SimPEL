#include "MainWindow.hpp"
#include "SVGExporter.hpp"
#include "engine/NetlistBuilder.hpp"
#include "imgui.h"
#include "imgui_internal.h"
#include <iostream>
#include <string>
#include <cstdlib>
#include <fstream>
#include <functional>
#include <windows.h>
#include <commdlg.h>
#include <shlobj.h>
#include <filesystem>
#include "engine/NetlistParser.hpp"
#include "engine/CScriptEngine.hpp"
#include "engine/ExpressionEvaluator.hpp"
#include "nlohmann/json.hpp"
#include <algorithm>
#include <unordered_set>
#include <unordered_map>
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
    // Don't start if already running
    if (simRunning.load()) return;

    // Join previous thread if any
    if (simThread.joinable()) simThread.join();

    NetlistBuilder::buildNodesForCircuit(canvas.getCircuitRef());

    std::string jsonNetlist = NetlistSourceView::generateNetlistJson(canvas.getCircuit());

    std::vector<CircuitSimEngine::ComponentModel> physComps;
    std::vector<CircuitSimEngine::ComponentModel> ctrlComps;
    CircuitSimEngine::SimulationConfig simCfg;

    CircuitSimEngine::NetlistParser::parseJsonString(jsonNetlist, physComps, ctrlComps, simCfg);

    simulator.setup(physComps, ctrlComps, simCfg);

    // Clear previous telemetry and trigger auto-fit
    simulator.reset();
    scopeView.triggerAutoFit();
    netlistSourceView.triggerAutoFit();

    // Run simulation on background thread for live/dynamic plotting
    simRunning.store(true);
    simThread = std::thread([this]() {
        CircuitSimEngine::SimulationOutput output = simulator.runTransient();
        simulator.setTelemetryOutput(output); // Final complete result
        simRunning.store(false);
    });
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

static std::string openFolderDialog() {
    char szDir[MAX_PATH] = { 0 };
    BROWSEINFOA bi = { 0 };
    bi.lpszTitle = "Select Folder Containing Schematic (.json) Files";
    bi.ulFlags = BIF_RETURNONLYFSDIRS | BIF_NEWDIALOGSTYLE;
    LPITEMIDLIST pidl = SHBrowseForFolderA(&bi);
    if (pidl != 0) {
        if (SHGetPathFromIDListA(pidl, szDir)) {
            CoTaskMemFree(pidl);
            return std::string(szDir);
        }
        CoTaskMemFree(pidl);
    }
    return "";
}

static void parseWireEndpointJSON(
    const json& epObj,
    WireEndpoint& ep,
    const std::function<std::string(const std::string&, const std::string&)>& resolveTerm = nullptr
) {
    if (!epObj.is_object()) return;

    std::string compId = epObj.value("compId", epObj.value("componentId", ""));
    std::string wireId = epObj.value("wireId", epObj.value("targetWireId", ""));
    std::string epType = epObj.value("type", "");
    std::string terminal = epObj.value("terminal", epObj.value("terminalName", ""));

    std::string targetW = !wireId.empty() ? wireId : compId;

    bool isJunction = (epType == "wire" || epType == "junction" || epObj.value("isWireJunction", false));
    if (!isJunction && !targetW.empty()) {
        if ((targetW[0] == 'w' || targetW[0] == 'W') && targetW.find('.') == std::string::npos) {
            isJunction = true;
        }
    }

    if (isJunction) {
        ep.isWireJunction = true;
        ep.targetWireId = targetW;
        ep.junctionX = epObj.value("x", epObj.value("junctionX", 0.0f));
        ep.junctionY = epObj.value("y", epObj.value("junctionY", 0.0f));
        ep.compId = "";
        ep.terminal = "";
    } else {
        ep.isWireJunction = false;
        ep.compId = compId;
        ep.terminal = resolveTerm ? resolveTerm(compId, terminal) : terminal;
        ep.targetWireId = "";
        ep.junctionX = 0.0f;
        ep.junctionY = 0.0f;
    }
}

static json serializeWireEndpointJSON(const WireEndpoint& ep) {
    json obj;
    bool isJunc = ep.isWireJunction || (!ep.targetWireId.empty() && ep.compId.empty()) ||
                  (!ep.compId.empty() && (ep.compId[0] == 'w' || ep.compId[0] == 'W') && ep.compId.find('.') == std::string::npos);

    if (isJunc) {
        std::string targetId = !ep.targetWireId.empty() ? ep.targetWireId : ep.compId;
        obj["type"] = "junction";
        obj["compId"] = targetId;
        obj["terminal"] = "";
        obj["x"] = ep.junctionX;
        obj["y"] = ep.junctionY;
    } else {
        obj["type"] = "pin";
        obj["compId"] = ep.compId;
        obj["terminal"] = ep.terminal;
    }
    return obj;
}

static void sanitizeCircuitWires(CircuitDesign& cd) {
    // 1. Remove invalid, dangling, or incomplete wire entries
    std::vector<WireInstance> cleanWires;
    std::unordered_map<std::string, const ComponentInstance*> compMap;
    for (const auto& c : cd.components) compMap[c.id] = &c;

    for (const auto& w : cd.wires) {
        bool fromValid = w.from.isWireJunction ? !w.from.targetWireId.empty() : !w.from.compId.empty();
        bool toValid   = w.to.isWireJunction   ? !w.to.targetWireId.empty()   : !w.to.compId.empty();

        if (!fromValid || !toValid) continue;

        if (!w.from.isWireJunction && !compMap.count(w.from.compId)) continue;
        if (!w.to.isWireJunction && !compMap.count(w.to.compId)) continue;

        if (!w.from.isWireJunction && !w.to.isWireJunction && w.from.compId == w.to.compId && w.from.terminal == w.to.terminal) {
            continue;
        }

        cleanWires.push_back(w);
    }
    cd.wires = cleanWires;

    // 2. Remove duplicate or reverse-parallel wire segments connecting the exact same endpoints
    std::vector<WireInstance> dedupedWires;
    std::unordered_set<std::string> seenEndpoints;

    for (const auto& w : cd.wires) {
        std::string ep1 = w.from.isWireJunction ? ("j:" + w.from.targetWireId) : ("p:" + w.from.compId + "." + w.from.terminal);
        std::string ep2 = w.to.isWireJunction   ? ("j:" + w.to.targetWireId)   : ("p:" + w.to.compId + "." + w.to.terminal);

        std::string forwardKey = ep1 + "<->" + ep2;
        std::string reverseKey = ep2 + "<->" + ep1;

        if (seenEndpoints.count(forwardKey) || seenEndpoints.count(reverseKey)) {
            continue;
        }
        seenEndpoints.insert(forwardKey);
        seenEndpoints.insert(reverseKey);
        dedupedWires.push_back(w);
    }
    cd.wires = dedupedWires;

    // 3. Fix uninitialized junction coordinates (0, 0)
    std::unordered_map<std::string, const WireInstance*> wireLookup;
    for (const auto& w : cd.wires) wireLookup[w.id] = &w;

    auto resolvePinPos = [&](const std::string& compId, const std::string& termName, float& outX, float& outY) -> bool {
        auto it = compMap.find(compId);
        if (it == compMap.end()) return false;
        const auto* comp = it->second;
        for (const auto& pin : comp->pins) {
            if (pin.name == termName) {
                outX = comp->x + pin.relativeX;
                outY = comp->y + pin.relativeY;
                return true;
            }
        }
        outX = comp->x; outY = comp->y;
        return true;
    };

    for (auto& w : cd.wires) {
        if (w.from.isWireJunction && w.from.junctionX == 0.0f && w.from.junctionY == 0.0f) {
            auto it = wireLookup.find(w.from.targetWireId);
            if (it != wireLookup.end()) {
                float fx = 0, fy = 0, tx = 0, ty = 0;
                bool fOk = resolvePinPos(it->second->from.compId, it->second->from.terminal, fx, fy);
                bool tOk = resolvePinPos(it->second->to.compId, it->second->to.terminal, tx, ty);
                if (fOk && tOk) {
                    w.from.junctionX = (fx + tx) * 0.5f;
                    w.from.junctionY = (fy + ty) * 0.5f;
                } else if (fOk) {
                    w.from.junctionX = fx; w.from.junctionY = fy;
                } else if (tOk) {
                    w.from.junctionX = tx; w.from.junctionY = ty;
                }
            }
        }
        if (w.to.isWireJunction && w.to.junctionX == 0.0f && w.to.junctionY == 0.0f) {
            auto it = wireLookup.find(w.to.targetWireId);
            if (it != wireLookup.end()) {
                float fx = 0, fy = 0, tx = 0, ty = 0;
                bool fOk = resolvePinPos(it->second->from.compId, it->second->from.terminal, fx, fy);
                bool tOk = resolvePinPos(it->second->to.compId, it->second->to.terminal, tx, ty);
                if (fOk && tOk) {
                    w.to.junctionX = (fx + tx) * 0.5f;
                    w.to.junctionY = (fy + ty) * 0.5f;
                } else if (fOk) {
                    w.to.junctionX = fx; w.to.junctionY = fy;
                } else if (tOk) {
                    w.to.junctionX = tx; w.to.junctionY = ty;
                }
            }
        }
    }
}

static std::string buildSchematicJsonString(const CircuitDesign& cd) {
    json j;
    json compArray = json::array();
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
        wObj["from"] = serializeWireEndpointJSON(wire.from);
        wObj["to"] = serializeWireEndpointJSON(wire.to);
        wireArray.push_back(wObj);
    }
    j["wires"] = wireArray;
    return j.dump(2);
}

void MainWindow::renderMenuBar() {
    if (ImGui::BeginMainMenuBar()) {
        if (ImGui::BeginMenu("File")) {
            if (ImGui::MenuItem("New Workspace")) { currentLoadedJsonName = ""; canvas.setCircuit(CircuitDesign()); simulator.loadCircuit(CircuitDesign()); }
            if (ImGui::MenuItem("Open Schematic (.json)")) {
                std::string path = openFileDialog();
                if (!path.empty()) {
                    currentLoadedJsonName = path;
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
                                if (wItem.contains("from")) parseWireEndpointJSON(wItem["from"], wire.from);
                                if (wItem.contains("to")) parseWireEndpointJSON(wItem["to"], wire.to);
                                cd.wires.push_back(wire);
                            }

                            sanitizeCircuitWires(cd);
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
                    currentLoadedJsonName = path;
                    std::string jsonStr = buildSchematicJsonString(canvas.getCircuit());
                    std::ofstream out(path);
                    if (out.is_open()) out << jsonStr;
                }
            }
            if (ImGui::MenuItem("Export Schematic as SVG (.svg)")) {
                std::string defaultName = getProjectBaseName() + "_schematic.svg";
                std::string path = SVGExporter::saveSVGFileDialog("Export Schematic as SVG", defaultName);
                if (!path.empty()) {
                    SVGExporter::exportSchematicToSVG(canvas.getCircuit(), path, canvas.isDarkModeActive());
                }
            }
            if (ImGui::MenuItem("Export Report (HTML / PDF)...")) {
                isBatchExportMode = false;
                showExportOptionsModal = true;
            }
            ImGui::Separator();
            if (ImGui::MenuItem("Batch Simulate Folder (.json)...")) {
                std::string folder = openFolderDialog();
                if (!folder.empty()) {
                    batchSimulateFolder(folder);
                }
            }
            if (ImGui::MenuItem("Batch Export Reports for Folder (HTML / PDF)...")) {
                std::string folder = openFolderDialog();
                if (!folder.empty()) {
                    exportTargetFolder = folder;
                    isBatchExportMode = true;
                    showExportOptionsModal = true;
                }
            }
            ImGui::Separator();
            if (ImGui::MenuItem("Copy Schematic JSON")) {
                std::string jsonStr = buildSchematicJsonString(canvas.getCircuit());
                ImGui::SetClipboardText(jsonStr.c_str());
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
            if (ImGui::MenuItem("Demo Circuits Pane", nullptr, showDemoPane)) {
                showDemoPane = !showDemoPane;
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
    ImGui::SetNextWindowViewport(viewport->ID);
    
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

        if (ImGui::Button("Copy JSON")) {
            std::string jsonStr = buildSchematicJsonString(canvas.getCircuit());
            ImGui::SetClipboardText(jsonStr.c_str());
        }
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Copy complete schematic design JSON to clipboard");
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

        double pct = simulator.getProgressPercent();
        double cpuSec = simulator.getComputeTimeSeconds();
        if (simRunning.load()) {
            ImGui::TextColored(ImVec4(0.3f, 0.8f, 1.0f, 1.0f), "Progress: %.1f%% | Compute Time: %.3f s", pct, cpuSec);
        } else {
            ImGui::TextColored(ImVec4(0.2f, 0.9f, 0.4f, 1.0f), "Progress: %.1f%% | Compute Time: %.3f s", pct, cpuSec);
        }
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

        // Helper lambda to render component button with clean graphic icon preview
        auto renderCompButton = [&](const char* buttonText, const std::string& prefix, const std::string& label, ComponentType type, const std::string& rawTypeStr, const std::vector<std::pair<std::string, std::string>>& defaultParams = {}, float forcedWidth = -1.0f) {
            ImGui::PushID(rawTypeStr.c_str());

            float availW = ImGui::GetContentRegionAvail().x;
            float itemW = (forcedWidth > 0.0f) ? forcedWidth : availW;
            bool isTwoCol = (forcedWidth > 0.0f && forcedWidth < availW * 0.8f);

            float rowHeight = 46.0f;
            float iconSize = isTwoCol ? 34.0f : 36.0f;
            ImVec2 cursorPos = ImGui::GetCursorScreenPos();
            ImDrawList* drawList = ImGui::GetWindowDrawList();

            bool isDark = canvas.isDarkModeActive();
            ImU32 iconBgCol = isDark ? IM_COL32(15, 23, 42, 240) : IM_COL32(241, 245, 249, 255);
            ImU32 iconBorderCol = isDark ? IM_COL32(56, 189, 248, 180) : IM_COL32(14, 165, 233, 200);

            // Parse buttonText into main label and sub label
            std::string rawBtn = buttonText;
            std::string mainText = rawBtn;
            std::string subText = "";

            size_t lastParenOpen = rawBtn.find_last_of('(');
            size_t lastParenClose = rawBtn.find_last_of(')');
            if (lastParenOpen != std::string::npos && lastParenClose != std::string::npos && lastParenClose > lastParenOpen) {
                mainText = rawBtn.substr(0, lastParenOpen);
                subText = rawBtn.substr(lastParenOpen + 1, lastParenClose - lastParenOpen - 1);
                while (!mainText.empty() && (mainText.back() == ' ' || mainText.back() == '\t')) {
                    mainText.pop_back();
                }
            }

            // Push button styles
            ImGui::PushStyleVar(ImGuiStyleVar_FrameRounding, 6.0f);
            bool clicked = ImGui::Button("##card", ImVec2(itemW, rowHeight));
            ImGui::PopStyleVar(1);

            // Draw Icon Badge inside left padding of the row button
            float iconMarginY = (rowHeight - iconSize) * 0.5f;
            ImVec2 iconMin = ImVec2(cursorPos.x + 4.0f, cursorPos.y + iconMarginY);
            ImVec2 iconMax = ImVec2(iconMin.x + iconSize, iconMin.y + iconSize);
            ImVec2 iconCenter = ImVec2(iconMin.x + iconSize * 0.5f, iconMin.y + iconSize * 0.5f);

            drawList->AddRectFilled(iconMin, iconMax, iconBgCol, 6.0f);
            drawList->AddRect(iconMin, iconMax, iconBorderCol, 6.0f, 0, 1.2f);

            // Clip all drawing strictly inside the icon badge
            drawList->PushClipRect(iconMin, iconMax, true);

            ComponentInstance tempComp;
            tempComp.rawTypeStr = rawTypeStr;
            tempComp.label = label;
            tempComp.rotation = 0;
            for (const auto& p : defaultParams) tempComp.parameters[p.first] = p.second;

            ImU32 iconColor = isDark ? IM_COL32(240, 240, 240, 240) : IM_COL32(30, 30, 30, 255);
            float shapeScale = isTwoCol ? 0.35f : 0.38f;
            SchematicCanvas::drawComponentShape(drawList, tempComp, iconCenter, shapeScale, iconColor, isDark);

            drawList->PopClipRect();

            // Render Text Area next to the icon badge
            float textStartX = cursorPos.x + 4.0f + iconSize + 6.0f;
            float maxTextWidth = (cursorPos.x + itemW - 4.0f) - textStartX;

            ImU32 textColorMain = isDark ? IM_COL32(240, 245, 250, 255) : IM_COL32(15, 23, 42, 255);
            ImU32 textColorSub  = isDark ? IM_COL32(56, 189, 248, 220)  : IM_COL32(14, 165, 233, 220);

            auto fitText = [](const std::string& str, float maxW) -> std::string {
                if (maxW <= 10.0f) return "";
                if (ImGui::CalcTextSize(str.c_str()).x <= maxW) return str;
                std::string res = str;
                while (res.length() > 1) {
                    res.pop_back();
                    std::string cand = res + "..";
                    if (ImGui::CalcTextSize(cand.c_str()).x <= maxW) return cand;
                }
                return res;
            };

            std::string dispMain = fitText(mainText, maxTextWidth);
            std::string dispSub = fitText(subText, maxTextWidth);

            if (!subText.empty() && maxTextWidth > 30.0f) {
                // 2-line rendering (Main Label on top, Code / Sub-label on bottom)
                ImVec2 mainPos = ImVec2(textStartX, cursorPos.y + (rowHeight * 0.5f - 16.0f));
                ImVec2 subPos  = ImVec2(textStartX, cursorPos.y + (rowHeight * 0.5f + 1.0f));

                drawList->AddText(mainPos, textColorMain, dispMain.c_str());
                drawList->AddText(subPos, textColorSub, dispSub.c_str());
            } else {
                // Single line vertically centered text
                ImVec2 textPos = ImVec2(textStartX, cursorPos.y + (rowHeight - ImGui::GetTextLineHeight()) * 0.5f);
                drawList->AddText(textPos, textColorMain, dispMain.c_str());
            }

            if (clicked) {
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
            // General Domain - Ports and Subsystems
            { "Subsystem Block", "Subsystem", "SUBSYSTEM", ComponentType::Subsystem, "SUBSYSTEM", "general", "Ports and Subsystems", {}, true },
            { "Inport (INPORT)", "Inport", "INPORT", ComponentType::Inport, "INPORT", "general", "Ports and Subsystems", {{"port_number", "1"}}, true },
            { "Outport (OUTPORT)", "Outport", "OUTPORT", ComponentType::Outport, "OUTPORT", "general", "Ports and Subsystems", {{"port_number", "1"}}, true },
            { "Physical Inport (PIN)", "Phys Inport", "PIN", ComponentType::PhysicalInport, "PIN", "general", "Ports and Subsystems", {{"port_number", "1"}}, true },
            { "Physical Outport (POUT)", "Phys Outport", "POUT", ComponentType::PhysicalOutport, "POUT", "general", "Ports and Subsystems", {{"port_number", "1"}}, true },
            { "Enable Port (ENABLE_PORT)", "Enable", "ENABLE_PORT", ComponentType::EnablePort, "ENABLE_PORT", "general", "Ports and Subsystems", {{"states_when_disabled", "held"}}, true },
            { "Trigger Port (TRIGGER_PORT)", "Trigger", "TRIGGER_PORT", ComponentType::TriggerPort, "TRIGGER_PORT", "general", "Ports and Subsystems", {{"trigger_type", "rising"}}, true },
            { "Bus Creator (BUS_CREATOR)", "Bus Creator", "BUS_CREATOR", ComponentType::BusCreator, "BUS_CREATOR", "general", "Ports and Subsystems", {{"inputs", "2"}}, true },
            { "Bus Selector (BUS_SELECTOR)", "Bus Selector", "BUS_SELECTOR", ComponentType::BusSelector, "BUS_SELECTOR", "general", "Ports and Subsystems", {{"signals", "signal1,signal2"}}, true },
            { "Terminator (TERMINATOR)", "Terminator", "TERMINATOR", ComponentType::Terminator, "TERMINATOR", "general", "Ports and Subsystems", {}, true },

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
            { "Sum (SUM_RECT)", "Sum", "SUM_RECT", ComponentType::SummingJunction, "SUM_RECT", "control", "Functions & Tables", {{"inputs", "++"}}, true },
            { "Product (PROD)", "Product", "PROD", ComponentType::Product, "PRODUCT_RECT", "control", "Functions & Tables", {{"operators", "*/"}}, true },
            { "Trigonometric Function (TRIG_FCN)", "Trig", "TRIG_FCN", ComponentType::TrigFunction, "TRIG_FCN", "control", "Functions & Tables", {{"function", "sin"}}, false },
            { "Math Function (MATH_FCN)", "Math Function", "MATH_FCN", ComponentType::MathFunction, "MATH_FCN", "control", "Functions & Tables", {{"function", "exp"}}, true },
            { "Abs (ABS)", "Abs", "ABS", ComponentType::Abs, "ABS", "control", "Functions & Tables", {}, false },
            { "Sign (SIGN)", "Sign", "SIGN", ComponentType::Sign, "SIGN", "control", "Functions & Tables", {}, false },
            { "Round (ROUND)", "Round", "ROUND", ComponentType::Round, "ROUND", "control", "Functions & Tables", {{"mode", "nearest"}}, false },
            { "Min/Max (MIN_MAX)", "MinMax", "MIN_MAX", ComponentType::MinMax, "MIN_MAX", "control", "Functions & Tables", {{"function", "min"}, {"num_inputs", "2"}}, false },
            { "Polynomial (POLYNOMIAL)", "Polynomial", "POLYNOMIAL", ComponentType::Polynomial, "POLYNOMIAL", "control", "Functions & Tables", {{"coefficients", "[1, 0]"}}, true },
            { "1D Look-Up Table (LUT_1D)", "LUT 1D", "LUT_1D", ComponentType::LUT_1D, "LUT_1D", "control", "Functions & Tables", {{"x_data", "[0, 1, 2]"}, {"y_data", "[0, 2, 4]"}}, false },
            { "2D Look-Up Table (LUT_2D)", "LUT 2D", "LUT_2D", ComponentType::LUT_2D, "LUT_2D", "control", "Functions & Tables", {{"x_data", "[0, 1]"}, {"y_data", "[0, 1]"}, {"z_data", "[[0, 1], [1, 2]]"}}, false },
            { "3D Look-Up Table (LUT_3D)", "LUT 3D", "LUT_3D", ComponentType::LUT_3D, "LUT_3D", "control", "Functions & Tables", {}, false },
            { "C-Script (CSCRIPT)", "C-Script", "CSCRIPT", ComponentType::CustomScript, "CSCRIPT", "control", "Functions & Tables", {{"code", "// Step code\noutputs[0] = inputs[0] * 2.0;\n"}}, true },
            { "Algebraic Constraint (ALGEBRAIC_CONSTRAINT)", "Alg Constraint", "ALGEBRAIC_CONSTRAINT", ComponentType::AlgebraicConstraint, "ALGEBRAIC_CONSTRAINT", "control", "Functions & Tables", {{"initial_guess", "0.0"}}, true },
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
            { "AND Gate (AND)", "AND", "AND", ComponentType::AND_Gate, "AND", "control", "Logical & Bitwise", {{"inputs", "2"}}, true },
            { "OR Gate (OR)", "OR", "OR", ComponentType::OR_Gate, "OR", "control", "Logical & Bitwise", {{"inputs", "2"}}, true },
            { "NOT Gate (NOT)", "NOT", "NOT", ComponentType::NOT_Gate, "NOT", "control", "Logical & Bitwise", {}, true },
            { "NAND Gate (NAND)", "NAND", "NAND", ComponentType::AND_Gate, "NAND", "control", "Logical & Bitwise", {{"inputs", "2"}}, false },
            { "NOR Gate (NOR)", "NOR", "NOR", ComponentType::OR_Gate, "NOR", "control", "Logical & Bitwise", {{"inputs", "2"}}, false },
            { "XOR Gate (XOR)", "XOR", "XOR", ComponentType::LogicOp, "XOR", "control", "Logical & Bitwise", {{"inputs", "2"}}, false },
            { "XNOR Gate (XNOR)", "XNOR", "XNOR", ComponentType::LogicOp, "XNOR", "control", "Logical & Bitwise", {{"inputs", "2"}}, false },
            { "Compare To Constant (COMP_CONST)", "Compare", "COMP_CONST", ComponentType::CompareToConstant, "COMP_CONST", "control", "Logical & Bitwise", {{"operator", "=="}, {"const", "0"}}, false },

            // Modulators sub-library
            { "PWM Generator (PWM)", "PWM", "PWM", ComponentType::PWM_Generator, "PWM", "control", "Modulators", {{"frequency", "10k"}, {"min", "0"}, {"max", "1"}}, true },
            { "Master PWM (PWM_MASTER)", "Master PWM", "PWM_MASTER", ComponentType::MasterPWM, "PWM_MASTER", "control", "Modulators", {{"frequency", "10k"}, {"min", "0"}, {"max", "1"}}, true },
            { "Edge Detector (EDGE_DETECT)", "Edge", "EDGE_DETECT", ComponentType::EdgeDetect, "EDGE_DETECT", "control", "Modulators", {{"edge_type", "rising"}}, true },

            // Signal Transforms sub-library
            { "ABC to dq Transform (PARK_TRANSFORM)", "αβ-dq", "PARK_TRANSFORM", ComponentType::Park, "PARK_TRANSFORM", "control", "Signal Transforms", {}, false },
            { "dq to ABC Transform (INV_PARK_TRANSFORM)", "dq-αβ", "INV_PARK_TRANSFORM", ComponentType::InvPark, "INV_PARK_TRANSFORM", "control", "Signal Transforms", {}, false },
            { "Clarke Transform (CLARKE_TRANSFORM)", "abc-αβ", "CLARKE_TRANSFORM", ComponentType::Clarke, "CLARKE_TRANSFORM", "control", "Signal Transforms", {}, false },
            { "Inverse Clarke Transform (INV_CLARKE)", "αβ-abc", "INV_CLARKE", ComponentType::InvClarke, "INV_CLARKE", "control", "Signal Transforms", {}, false },
            { "Park Transform (PARK)", "abc-dq", "PARK", ComponentType::Park, "PARK", "control", "Signal Transforms", {}, false },
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
            { "Summing Block (SUM)", "SUM", "SUM_RECT", ComponentType::SummingJunction, "SUM_RECT", "control", "Math", {{"signs", "++"}}, false },
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

        // Helper lambda to render a list of components in a responsive 2-column or 1-column grid
        auto renderComponentGrid = [&](const std::vector<const ComponentMeta*>& compList) {
            if (compList.empty()) return;
            float availW = ImGui::GetContentRegionAvail().x;
            int numCols = (availW >= 260.0f) ? 2 : 1;
            float gap = 6.0f;
            float itemW = (numCols == 2) ? std::floor((availW - gap) * 0.5f) : availW;

            int colIdx = 0;
            for (size_t i = 0; i < compList.size(); ++i) {
                if (numCols == 2 && colIdx == 1) {
                    ImGui::SameLine(0, gap);
                }
                renderCompButton(compList[i]->buttonText, compList[i]->prefix, compList[i]->label, compList[i]->type, compList[i]->rawTypeStr, compList[i]->defaultParams, itemW);
                colIdx = (colIdx + 1) % numCols;
            }
            ImGui::Spacing();
        };

        // Filter search results if search query is non-empty
        if (!searchQuery.empty()) {
            ImGui::TextColored(ImVec4(0.2f, 0.8f, 1.0f, 1.0f), "🔍 Search Results");
            ImGui::Separator();
            ImGui::Spacing();
            std::vector<const ComponentMeta*> searchResults;
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
                    searchResults.push_back(&item);
                }
            }
            if (searchResults.empty()) {
                ImGui::TextDisabled("No matching components found.");
            } else {
                renderComponentGrid(searchResults);
            }
        }
        else if (!showDetailedLibrary) {
            // BASIC LIBRARY VIEW (Only simple original basic blocks)
            if (ImGui::CollapsingHeader("[PWR] Power Stage")) {
                ImGui::PushID("basic_power");
                ImGui::Indent(4.0f);
                std::vector<const ComponentMeta*> basicPower;
                for (const auto& item : allComponents) {
                    if (item.isBasic && std::string(item.category) == "electrical") basicPower.push_back(&item);
                }
                renderComponentGrid(basicPower);
                ImGui::Unindent(4.0f);
                ImGui::PopID();
            }

            if (ImGui::CollapsingHeader("[CTRL] Control Loops")) {
                ImGui::PushID("basic_control");
                ImGui::Indent(4.0f);
                std::vector<const ComponentMeta*> basicControl;
                for (const auto& item : allComponents) {
                    if (item.isBasic && std::string(item.category) == "control") basicControl.push_back(&item);
                }
                renderComponentGrid(basicControl);
                ImGui::Unindent(4.0f);
                ImGui::PopID();
            }

            if (ImGui::CollapsingHeader("[SCOPE] Scope & Probes")) {
                ImGui::PushID("basic_general");
                ImGui::Indent(4.0f);
                std::vector<const ComponentMeta*> basicGeneral;
                for (const auto& item : allComponents) {
                    if (item.isBasic && std::string(item.category) == "general") basicGeneral.push_back(&item);
                }
                renderComponentGrid(basicGeneral);
                ImGui::Unindent(4.0f);
                ImGui::PopID();
            }
        }
        else {
            // DETAILED LIBRARY VIEW (Categorized with ALL Web Tool Headings & Subheadings)
            
            auto renderSubheading = [&](const char* subcatName, const std::vector<const ComponentMeta*>& compList) {
                if (ImGui::TreeNodeEx(subcatName, ImGuiTreeNodeFlags_None)) {
                    ImGui::Indent(4.0f);
                    if (compList.empty()) {
                        ImGui::TextDisabled("(No blocks available yet)");
                    } else {
                        renderComponentGrid(compList);
                    }
                    ImGui::Unindent(4.0f);
                    ImGui::TreePop();
                }
                ImGui::Spacing();
            };

            // 1. GENERAL BLOCKS (Category: general)
            if (ImGui::CollapsingHeader("[GEN] General Blocks")) {
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
            if (ImGui::CollapsingHeader("[CTRL] Control Blocks")) {
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
            if (ImGui::CollapsingHeader("[ELEC] Electrical Blocks")) {
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

void MainWindow::loadSchematicFromJson(const json& j) {
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
        std::unordered_map<std::string, std::string> compTypeMap;
        for (const auto& c : cd.components) {
            compTypeMap[c.id] = c.rawTypeStr;
        }

        auto resolveTerminalName = [&](const std::string& compId, const std::string& term) -> std::string {
            auto it = compTypeMap.find(compId);
            if (it != compTypeMap.end()) {
                std::string t = it->second;
                if (t == "SCOPE" || t == "Oscilloscope") {
                    std::string lowerTerm = term;
                    std::transform(lowerTerm.begin(), lowerTerm.end(), lowerTerm.begin(), ::tolower);
                    if (lowerTerm.rfind("ch", 0) == 0 && lowerTerm.length() > 2) {
                        return "In" + lowerTerm.substr(2);
                    }
                    if (lowerTerm.rfind("in", 0) == 0 && lowerTerm.length() > 2) {
                        return "In" + lowerTerm.substr(2);
                    }
                }
            }
            return term;
        };

        for (const auto& wItem : j["wires"]) {
            WireInstance wire;
            wire.id = wItem.value("id", "");
            if (wItem.contains("from")) parseWireEndpointJSON(wItem["from"], wire.from, resolveTerminalName);
            if (wItem.contains("to")) parseWireEndpointJSON(wItem["to"], wire.to, resolveTerminalName);
            cd.wires.push_back(wire);
        }
        sanitizeCircuitWires(cd);
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

    NetlistBuilder::buildNodesForCircuit(cd);
    canvas.setCircuit(cd);
    simulator.loadCircuit(cd);
    canvas.fitToScreen();
    scopeView.triggerAutoFit();
}

void MainWindow::batchSimulateFolder(const std::string& folderPath) {
    if (folderPath.empty()) return;
    int processed = 0, succeeded = 0;
    try {
        for (const auto& entry : std::filesystem::directory_iterator(folderPath)) {
            if (entry.is_regular_file() && entry.path().extension() == ".json") {
                processed++;
                std::ifstream f(entry.path().string());
                if (!f.is_open()) continue;
                try {
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
                            if (wItem.contains("from")) parseWireEndpointJSON(wItem["from"], wire.from);
                            if (wItem.contains("to")) parseWireEndpointJSON(wItem["to"], wire.to);
                            cd.wires.push_back(wire);
                        }
                        sanitizeCircuitWires(cd);
                    }
                    NetlistBuilder::buildNodesForCircuit(cd);
                    std::string jsonNetlist = NetlistSourceView::generateNetlistJson(cd);
                    std::vector<CircuitSimEngine::ComponentModel> physComps;
                    std::vector<CircuitSimEngine::ComponentModel> ctrlComps;
                    CircuitSimEngine::SimulationConfig simCfg;
                    CircuitSimEngine::NetlistParser::parseJsonString(jsonNetlist, physComps, ctrlComps, simCfg);

                    CircuitSimEngine::CircuitSimulator batchSim;
                    batchSim.setup(physComps, ctrlComps, simCfg);
                    batchSim.reset();
                    batchSim.runTransient();
                    succeeded++;
                } catch (...) {}
            }
        }
    } catch (...) {}

    std::string msg = "Batch Simulation Complete!\n\nSuccessfully simulated " + std::to_string(succeeded) + " of " + std::to_string(processed) + " schematic files in:\n" + folderPath + "\n\nWould you like to configure and export reports now?";
    int res = MessageBoxA(NULL, msg.c_str(), "Batch Simulation Complete", MB_YESNO | MB_ICONINFORMATION);
    if (res == IDYES) {
        exportTargetFolder = folderPath;
        isBatchExportMode = true;
        showExportOptionsModal = true;
    }
}

void MainWindow::batchExportHtmlFolder(const std::string& folderPath) {
    exportTargetFolder = folderPath;
    isBatchExportMode = true;
    showExportOptionsModal = true;
}

void MainWindow::executeBatchExportWithOptions(const std::string& folderPath, const SVGExporter::ReportExportOptions& options) {
    if (folderPath.empty()) return;
    int processed = 0, succeeded = 0;
    std::vector<SVGExporter::CircuitReportItem> allReports;

    try {
        for (const auto& entry : std::filesystem::directory_iterator(folderPath)) {
            if (entry.is_regular_file() && entry.path().extension() == ".json") {
                processed++;
                std::ifstream f(entry.path().string());
                if (!f.is_open()) continue;
                try {
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
                    std::unordered_map<std::string, std::string> compTypeMap;
                    for (const auto& c : cd.components) compTypeMap[c.id] = c.rawTypeStr;

                    auto resolveTerminalName = [&](const std::string& compId, const std::string& term) -> std::string {
                        auto it = compTypeMap.find(compId);
                        if (it != compTypeMap.end()) {
                            std::string t = it->second;
                            if (t == "SCOPE" || t == "Oscilloscope") {
                                std::string lowerTerm = term;
                                std::transform(lowerTerm.begin(), lowerTerm.end(), lowerTerm.begin(), ::tolower);
                                if (lowerTerm.rfind("ch", 0) == 0 && lowerTerm.length() > 2) return "In" + lowerTerm.substr(2);
                                if (lowerTerm.rfind("in", 0) == 0 && lowerTerm.length() > 2) return "In" + lowerTerm.substr(2);
                            }
                        }
                        return term;
                    };

                    if (j.contains("wires") && j["wires"].is_array()) {
                        for (const auto& wItem : j["wires"]) {
                            WireInstance wire;
                            wire.id = wItem.value("id", "");
                            if (wItem.contains("from")) parseWireEndpointJSON(wItem["from"], wire.from, resolveTerminalName);
                            if (wItem.contains("to")) parseWireEndpointJSON(wItem["to"], wire.to, resolveTerminalName);
                            cd.wires.push_back(wire);
                        }
                        sanitizeCircuitWires(cd);
                    }
                    NetlistBuilder::buildNodesForCircuit(cd);
                    std::string jsonNetlist = NetlistSourceView::generateNetlistJson(cd);
                    std::vector<CircuitSimEngine::ComponentModel> physComps;
                    std::vector<CircuitSimEngine::ComponentModel> ctrlComps;
                    CircuitSimEngine::SimulationConfig simCfg;
                    CircuitSimEngine::NetlistParser::parseJsonString(jsonNetlist, physComps, ctrlComps, simCfg);

                    CircuitSimEngine::CircuitSimulator batchSim;
                    batchSim.setup(physComps, ctrlComps, simCfg);
                    batchSim.reset();
                    CircuitSimEngine::SimulationOutput output = batchSim.runTransient();
                    batchSim.setTelemetryOutput(output);
                    auto telemetry = batchSim.getTelemetryCopy();

                    std::vector<SVGExporter::ScopeReportData> scopesData;
                    for (const auto& comp : cd.components) {
                        if (comp.type == ComponentType::Oscilloscope || comp.rawTypeStr == "SCOPE") {
                            int numChannels = 2;
                            if (comp.parameters.count("channels")) {
                                try { numChannels = std::stoi(comp.parameters.at("channels")); } catch(...) {}
                            }
                            std::vector<std::string> sigKeys = SVGExporter::traceScopeInputSignals(cd, comp.id, numChannels);
                            std::vector<std::string> validKeys, validLabels;
                            for (const auto& k : sigKeys) {
                                if (!k.empty() && (telemetry.voltages.count(k) || !telemetry.timeHistory.empty())) {
                                    validKeys.push_back(k); validLabels.push_back(k);
                                }
                            }
                            if (!validKeys.empty()) {
                                SVGExporter::ScopeReportData srd;
                                srd.scopeId = comp.id;
                                srd.scopeTitle = (comp.label.empty() || comp.label == comp.id) ? comp.id : (comp.label + " (" + comp.id + ")");
                                srd.signalKeys = validKeys;
                                srd.signalLabels = validLabels;
                                srd.numPanes = (int)validKeys.size();
                                scopesData.push_back(srd);
                            }
                        }
                    }

                    std::string outHtmlPath = entry.path().parent_path().string() + "/" + entry.path().stem().string() + "_report.html";
                    std::string schematicJson = buildSchematicJsonString(cd);
                    bool exportOk = true;
                    if (options.exportIndividual) {
                        exportOk = SVGExporter::exportFullReportToHTML(cd, telemetry, scopesData, schematicJson, jsonNetlist, outHtmlPath, false, options);
                    }
                    if (exportOk) {
                        succeeded++;
                        SVGExporter::CircuitReportItem rItem;
                        rItem.jsonName = entry.path().filename().string();
                        rItem.design = cd;
                        rItem.telemetry = telemetry;
                        rItem.scopesData = scopesData;
                        rItem.schematicJson = schematicJson;
                        rItem.netlistJson = jsonNetlist;
                        allReports.push_back(rItem);
                    }
                } catch (...) {}
            }
        }
    } catch (...) {}

    if (options.exportMerged && !allReports.empty()) {
        std::string mergedHtmlPath = folderPath + "/_all_simulation_reports_merged.html";
        SVGExporter::exportMergedReportToHTML(allReports, mergedHtmlPath, options);
    }

    std::string formatName = (options.format == SVGExporter::ReportExportFormat::HTML) ? "HTML" : ((options.format == SVGExporter::ReportExportFormat::PDF) ? "PDF" : "HTML + PDF");
    std::string msg = "Batch " + formatName + " Export Complete!\n\nSuccessfully exported reports for " + std::to_string(succeeded) + " of " + std::to_string(processed) + " circuits to:\n" + folderPath;
    MessageBoxA(NULL, msg.c_str(), "Batch Export Complete", MB_OK | MB_ICONINFORMATION);
}

void MainWindow::executeSingleExportWithOptions(const SVGExporter::ReportExportOptions& options) {
    std::string defaultName = getProjectBaseName() + "_report.html";
    std::string path = SVGExporter::saveHTMLFileDialog("Export Report", defaultName);
    if (path.empty()) return;

    NetlistBuilder::buildNodesForCircuit(canvas.getCircuitRef());
    std::string jsonNetlist = NetlistSourceView::generateNetlistJson(canvas.getCircuit());

    std::vector<CircuitSimEngine::ComponentModel> physComps;
    std::vector<CircuitSimEngine::ComponentModel> ctrlComps;
    CircuitSimEngine::SimulationConfig simCfg;
    CircuitSimEngine::NetlistParser::parseJsonString(jsonNetlist, physComps, ctrlComps, simCfg);

    simulator.setup(physComps, ctrlComps, simCfg);
    simulator.reset();
    CircuitSimEngine::SimulationOutput output = simulator.runTransient();
    simulator.setTelemetryOutput(output);

    auto telemetry = simulator.getTelemetryCopy();

    std::vector<SVGExporter::ScopeReportData> scopesData;

    for (const auto& comp : canvas.getCircuit().components) {
        if (comp.type == ComponentType::Oscilloscope || comp.rawTypeStr == "SCOPE") {
            int numChannels = 2;
            if (comp.parameters.count("channels")) {
                try { numChannels = std::stoi(comp.parameters.at("channels")); } catch(...) {}
            }

            std::vector<std::string> sigKeys = traceScopeInputSignals(comp.id, numChannels);

            std::vector<std::string> validKeys;
            std::vector<std::string> validLabels;
            for (const auto& k : sigKeys) {
                if (!k.empty() && (telemetry.voltages.count(k) || !telemetry.timeHistory.empty())) {
                    validKeys.push_back(k);
                    validLabels.push_back(k);
                }
            }

            if (!validKeys.empty()) {
                SVGExporter::ScopeReportData srd;
                srd.scopeId = comp.id;
                srd.scopeTitle = (comp.label.empty() || comp.label == comp.id) ? comp.id : (comp.label + " (" + comp.id + ")");
                srd.signalKeys = validKeys;
                srd.signalLabels = validLabels;
                srd.numPanes = (int)validKeys.size();
                scopesData.push_back(srd);
            }
        }
    }

    if (scopesData.empty()) {
        std::vector<std::string> probeKeys;
        for (const auto& comp : canvas.getCircuit().components) {
            if (comp.rawTypeStr == "PROBE") {
                if (comp.parameters.count("selected_signals") && !comp.parameters.at("selected_signals").empty()) {
                    probeKeys.push_back(comp.parameters.at("selected_signals"));
                } else if (comp.parameters.count("target") && !comp.parameters.at("target").empty()) {
                    std::string pType = comp.parameters.count("probe_type") ? comp.parameters.at("probe_type") : "Voltage";
                    if (pType == "Current" || pType == "I") probeKeys.push_back("I_" + comp.parameters.at("target"));
                    else probeKeys.push_back("V_" + comp.parameters.at("target"));
                }
            }
        }

        std::vector<std::string> validKeys;
        std::vector<std::string> validLabels;
        for (const auto& k : probeKeys) {
            if (!k.empty() && telemetry.voltages.count(k)) {
                validKeys.push_back(k);
                validLabels.push_back(k);
            }
        }

        if (!validKeys.empty()) {
            SVGExporter::ScopeReportData srd;
            srd.scopeId = "Probes";
            srd.scopeTitle = "Probe Waveforms";
            srd.signalKeys = validKeys;
            srd.signalLabels = validLabels;
            srd.numPanes = (int)validKeys.size();
            scopesData.push_back(srd);
        }
    }

    std::string schematicJson = buildSchematicJsonString(canvas.getCircuit());

    SVGExporter::exportFullReportToHTML(
        canvas.getCircuit(),
        telemetry,
        scopesData,
        schematicJson,
        jsonNetlist,
        path,
        false,
        options
    );

    std::string formatName = (options.format == SVGExporter::ReportExportFormat::HTML) ? "HTML" : ((options.format == SVGExporter::ReportExportFormat::PDF) ? "PDF" : "HTML + PDF");
    std::string msg = "Report exported successfully (" + formatName + ") to:\n" + path;
    MessageBoxA(NULL, msg.c_str(), "Report Export Complete", MB_OK | MB_ICONINFORMATION);
}

void MainWindow::renderExportOptionsModal() {
    if (showExportOptionsModal) {
        ImGui::OpenPopup("Report Export Configuration");
    }

    ImVec2 center = ImGui::GetMainViewport()->GetCenter();
    ImGui::SetNextWindowPos(center, ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
    ImGui::SetNextWindowSize(ImVec2(520, 0), ImGuiCond_Appearing);

    if (ImGui::BeginPopupModal("Report Export Configuration", &showExportOptionsModal, ImGuiWindowFlags_AlwaysAutoResize)) {
        ImGui::TextColored(ImVec4(0.01f, 0.52f, 0.78f, 1.0f), isBatchExportMode ? "Batch Report Export Settings" : "Circuit Report Export Settings");
        ImGui::Separator();
        ImGui::Spacing();

        if (isBatchExportMode) {
            ImGui::Text("Target Folder:");
            char folderBuf[512];
            std::strncpy(folderBuf, exportTargetFolder.c_str(), sizeof(folderBuf));
            folderBuf[sizeof(folderBuf) - 1] = '\0';
            ImGui::SetNextItemWidth(380);
            if (ImGui::InputText("##Folder", folderBuf, sizeof(folderBuf))) {
                exportTargetFolder = folderBuf;
            }
            ImGui::SameLine();
            if (ImGui::Button("Browse...")) {
                std::string f = openFolderDialog();
                if (!f.empty()) exportTargetFolder = f;
            }
            ImGui::Spacing();
            ImGui::Separator();
            ImGui::Spacing();
        }

        // 1. Export Format Combo
        ImGui::Text("Export Format:");
        const char* formatItems[] = { "HTML Document (.html)", "PDF Document (.pdf)", "Both (HTML + PDF Documents)" };
        int currentFormatIdx = (int)currentExportOptions.format;
        ImGui::SetNextItemWidth(320);
        if (ImGui::Combo("##ExportFormat", &currentFormatIdx, formatItems, IM_ARRAYSIZE(formatItems))) {
            currentExportOptions.format = (SVGExporter::ReportExportFormat)currentFormatIdx;
        }
        ImGui::Spacing();

        // 2. Batch Export Modes (only in batch mode)
        if (isBatchExportMode) {
            ImGui::TextColored(ImVec4(0.9f, 0.9f, 0.9f, 1.0f), "Batch Generation Scope:");
            ImGui::Checkbox("Export Individual Circuit Reports (<name>_report)", &currentExportOptions.exportIndividual);
            ImGui::Checkbox("Export Master Merged Report (_all_simulation_reports_merged)", &currentExportOptions.exportMerged);
            ImGui::Spacing();
        }

        // 3. Content Inclusion
        ImGui::TextColored(ImVec4(0.9f, 0.9f, 0.9f, 1.0f), "Report Content Inclusion:");
        ImGui::Checkbox("Include Schematic SVG Diagram", &currentExportOptions.includeSchematicSvg);
        ImGui::Checkbox("Include Oscilloscope Waveforms", &currentExportOptions.includeWaveforms);
        ImGui::Checkbox("Include Schematic JSON Structure", &currentExportOptions.includeSchematicJson);
        ImGui::Checkbox("Include Netlist JSON Specification", &currentExportOptions.includeNetlistJson);
        ImGui::Spacing();

        // 4. JSON Layout
        if (currentExportOptions.includeSchematicJson || currentExportOptions.includeNetlistJson) {
            ImGui::TextColored(ImVec4(0.9f, 0.9f, 0.9f, 1.0f), "JSON Section Layout:");
            ImGui::Checkbox("Make JSON Sections Collapsible (<details>)", &currentExportOptions.jsonCollapsible);
            if (currentExportOptions.jsonCollapsible) {
                ImGui::Indent(20.0f);
                ImGui::Checkbox("Expand JSON Sections by default", &currentExportOptions.jsonDefaultExpanded);
                ImGui::Unindent(20.0f);
            }
            ImGui::Spacing();
        }

        ImGui::Separator();
        ImGui::Spacing();

        // Action Buttons
        if (ImGui::Button("Export Reports", ImVec2(140, 32))) {
            showExportOptionsModal = false;
            ImGui::CloseCurrentPopup();

            if (isBatchExportMode) {
                if (!exportTargetFolder.empty()) {
                    executeBatchExportWithOptions(exportTargetFolder, currentExportOptions);
                }
            } else {
                executeSingleExportWithOptions(currentExportOptions);
            }
        }
        ImGui::SameLine();
        if (ImGui::Button("Cancel", ImVec2(100, 32))) {
            showExportOptionsModal = false;
            ImGui::CloseCurrentPopup();
        }

        ImGui::EndPopup();
    }
}

bool MainWindow::loadDemoJsonFile(const std::string& filename) {
    std::vector<std::string> searchPaths = {
        "working jsons/" + filename,
        "../working jsons/" + filename,
        "../../working jsons/" + filename,
        "d:/01-Soft Dev Projects/circuitsim-pro/working jsons/" + filename,
        filename
    };

    for (const auto& path : searchPaths) {
        std::ifstream f(path);
        if (f.is_open()) {
            try {
                json j = json::parse(f);
                currentLoadedJsonName = filename;
                loadSchematicFromJson(j);
                return true;
            } catch (...) {}
        }
    }
    return false;
}

std::string MainWindow::getProjectBaseName() const {
    if (currentLoadedJsonName.empty()) return "circuit";
    std::string name = currentLoadedJsonName;
    size_t lastSlash = name.find_last_of("/\\");
    if (lastSlash != std::string::npos) {
        name = name.substr(lastSlash + 1);
    }
    if (name.size() > 5 && (name.substr(name.size() - 5) == ".json" || name.substr(name.size() - 5) == ".JSON")) {
        name = name.substr(0, name.size() - 5);
    }
    std::string clean;
    clean.reserve(name.size());
    for (char c : name) {
        if (std::isalnum((unsigned char)c) || c == '_' || c == '-') clean += c;
        else if (c == ' ') clean += '_';
    }
    return clean.empty() ? "circuit" : clean;
}

void MainWindow::renderDemoPane() {
    if (!showDemoPane) return;

    if (ImGui::FindWindowByName("Component Pane")) {
        ImGuiID compDockId = ImGui::FindWindowByName("Component Pane")->DockId;
        if (compDockId != 0) {
            ImGui::SetNextWindowDockID(compDockId, ImGuiCond_FirstUseEver);
        }
    }

    if (ImGui::Begin("Demo Circuits Pane", &showDemoPane)) {
        ImGui::TextColored(ImVec4(0.2f, 0.8f, 1.0f, 1.0f), "📂 Working Example Schematics");
        ImGui::TextDisabled("Click any example to load into the workspace");
        ImGui::Spacing();

        ImGui::InputTextWithHint("##DemoSearch", "Search example circuits...", searchDemoBuf, sizeof(searchDemoBuf));
        std::string searchQuery = searchDemoBuf;
        std::transform(searchQuery.begin(), searchQuery.end(), searchQuery.begin(), ::tolower);

        ImGui::Spacing();
        ImGui::Separator();
        ImGui::Spacing();

        struct DemoItem {
            std::string label;
            std::string filename;
            std::string category;
            std::string badge;
            std::string desc;
        };

        static const std::vector<DemoItem> demoCatalog = {
            // DC-DC Converters
            { "Buck Converter (Basic)", "Buck_converter.json", "DC-DC Converters", "Buck", "Fundamental open-loop Buck DC-DC step-down converter with pulse generator." },
            { "Buck Converter (Closed Loop Voltage Mode)", "scenario1_buck_converter.json", "DC-DC Converters", "PI Closed Loop", "Regulated Buck converter with discrete PI controller and voltage error feedback." },
            { "Buck Converter (Dual Loop Control)", "buck closed dual loop.json", "DC-DC Converters", "Dual Loop", "Cascaded voltage and current dual-loop controlled Buck converter." },
            { "Boost Converter (Closed Loop)", "boost closed loop.json", "DC-DC Converters", "Boost Closed Loop", "Closed-loop step-up DC-DC boost converter." },
            { "Flyback Converter (Closed Loop)", "Flyback Converter Closed loop.json", "DC-DC Converters", "Flyback", "Isolated flyback converter with coupled inductor transformer." },
            { "Forward Converter (Ideal Transformer)", "Forward Converter.json", "DC-DC Converters", "Forward", "Single-switch forward converter with ideal transformer." },
            { "Forward Converter (Closed Loop)", "Forward Converter closed loop.json", "DC-DC Converters", "Forward Closed Loop", "Regulated forward converter with feedback control." },
            { "Forward Converter (Multiwinding)", "forward_converter_multiwinding_fixed.json", "DC-DC Converters", "Multi-Secondary", "Forward converter with multi-winding transformer." },
            { "SEPIC Converter (Open Loop)", "SEPIC open loop.json", "DC-DC Converters", "SEPIC", "Single-Ended Primary-Inductor Converter in open loop." },
            { "SEPIC Converter (CScript & Wireless)", "sepic closed loop script routing test.json", "DC-DC Converters", "CScript + Wireless", "SEPIC converter regulated via CScript C-code block and wireless GOTO/FROM tags." },
            { "Zeta Converter (PID Closed Loop)", "zeta_converter_pid_closed_loop.json", "DC-DC Converters", "Zeta PID", "Zeta converter with closed-loop PID regulation." },
            { "Dual Active Bridge (DAB) Converter", "Dual Active Bridge converter.json", "DC-DC Converters", "DAB", "Bidirectional isolated Dual Active Bridge converter." },

            // Inverters & Motor Drives
            { "Single-Phase Hysteresis Inverter", "1ph_inverter_hysteresis_basic_blocks.json", "Inverters & Drives", "Hysteresis", "Single-phase full-bridge VSI with hysteresis current controller." },
            { "Single-Phase Hysteresis Current Control", "1ph_inverter_hysteresis_current_control.json", "Inverters & Drives", "Current Control", "Hysteresis band current regulated single-phase inverter." },
            { "Single-Phase Unipolar Inverter", "1ph_inverter_unipolar.json", "Inverters & Drives", "Unipolar PWM", "Unipolar SPWM switched single-phase full-bridge inverter." },
            { "3-Phase Inverter (Sin PWM)", "3Phase Inverter Sin PWM.json", "Inverters & Drives", "3Ph SPWM", "3-Phase sinusoidal PWM voltage source inverter." },
            { "3-Phase Inverter (SPWM Fixed)", "three_phase_inverter_basic_spwm_fixed.json", "Inverters & Drives", "3Ph Fixed", "Balanced 3-phase AC output inverter with LC filter." },
            { "3-Phase Inverter (Min-Max Injection / SVPWM)", "Three Phase VSI min max injection PWM svpwm type.json", "Inverters & Drives", "SVPWM", "Space vector PWM via min-max zero-sequence harmonic injection." },
            { "3-Level 3-Phase T-Type VSI", "3-Level 3_phase T-type VSI min max injection PWM svpwm type.json", "Inverters & Drives", "3-Level T-Type", "Multi-level T-Type neutral-point-clamped 3-phase VSI." },
            { "Cascaded H-Bridge 5-Level Inverter", "Cascaded H-bridge 5-level inverter single phase.json", "Inverters & Drives", "5-Level CHB", "5-Level single-phase cascaded H-bridge inverter." },
            { "3-Phase VSI (vgFET Switches)", "vgFET 3Phase VSI.json", "Inverters & Drives", "vgFET", "3-Phase VSI powered by voltage-gated FET switches." },

            // Control & Signal Routing Demos
            { "3-Phase VSI (Wireless GOTO/FROM)", "3P VSI goto from block test.json", "Control & Signal Routing", "Wireless Tags", "3-Phase inverter featuring wireless GOTO_SIG and FROM_SIG signal routing." },
            { "SEPIC Closed Loop (Script & GOTO)", "sepic closed loop with script block and wireless routing.json", "Control & Signal Routing", "CScript + Tags", "SEPIC converter with CScript block and GOTO/FROM wireless tags." }
        };

        std::map<std::string, std::vector<const DemoItem*>> categorized;
        for (const auto& item : demoCatalog) {
            if (!searchQuery.empty()) {
                std::string t = item.label + " " + item.filename + " " + item.desc + " " + item.badge;
                std::transform(t.begin(), t.end(), t.begin(), ::tolower);
                if (t.find(searchQuery) == std::string::npos) continue;
            }
            categorized[item.category].push_back(&item);
        }

        if (categorized.empty()) {
            ImGui::TextColored(ImVec4(0.7f, 0.7f, 0.7f, 1.0f), "No example circuits match '%s'", searchDemoBuf);
        } else {
            for (const auto& [catName, items] : categorized) {
                if (ImGui::CollapsingHeader(catName.c_str(), ImGuiTreeNodeFlags_DefaultOpen)) {
                    for (const auto* item : items) {
                        ImGui::PushID(item->filename.c_str());

                        // Render badge tag
                        ImGui::TextColored(ImVec4(0.0f, 0.75f, 0.95f, 1.0f), "[%s]", item->badge.c_str());
                        ImGui::SameLine();

                        if (ImGui::Selectable(item->label.c_str(), false)) {
                            loadDemoJsonFile(item->filename);
                        }

                        if (ImGui::IsItemHovered()) {
                            ImGui::SetTooltip("%s\nFilename: %s\n\n(Click to load into workspace)", item->desc.c_str(), item->filename.c_str());
                        }

                        ImGui::PopID();
                        ImGui::Spacing();
                    }
                }
            }
        }
    }
    ImGui::End();
}

void MainWindow::renderBatchPropertyInspector(const std::vector<ComponentInstance*>& selectedComps) {
    if (selectedComps.empty()) return;

    if (!ImGui::Begin("Property Inspector")) {
        ImGui::End();
        return;
    }

    // 1. Build comma-separated label string
    std::string labelsStr = "";
    for (size_t i = 0; i < selectedComps.size(); ++i) {
        if (i > 0) labelsStr += ", ";
        labelsStr += selectedComps[i]->label.empty() ? selectedComps[i]->id : selectedComps[i]->label;
    }

    ImGui::TextColored(ImVec4(0.2f, 0.8f, 1.0f, 1.0f), "Selected (%d): %s", (int)selectedComps.size(), labelsStr.c_str());
    
    // Check if all selected components are of the SAME component type
    bool sameType = true;
    std::string firstType = selectedComps[0]->rawTypeStr;
    for (const auto* c : selectedComps) {
        if (c->rawTypeStr != firstType) {
            sameType = false;
            break;
        }
    }

    if (sameType) {
        ImGui::TextDisabled("Type: %s (Batch Edit Mode)", firstType.c_str());
    } else {
        ImGui::TextDisabled("Type: Mixed Component Selection");
    }
    ImGui::Separator();
    ImGui::Spacing();

    // ── 2. BATCH SIGNAL PROBING / PLOTTING ──
    std::vector<ComponentInstance*> electricalComps;
    for (auto* c : selectedComps) {
        std::string t = c->rawTypeStr;
        std::transform(t.begin(), t.end(), t.begin(), ::toupper);
        bool isElectrical = (t == "R" || t == "L" || t == "C" || t == "V" || t == "AC_V" || t == "I" || t == "S" || t == "D" || t == "MOSFET" || t == "VM" || t == "AM" || t == "GND");
        if (isElectrical) {
            electricalComps.push_back(c);
        }
    }

    if (!electricalComps.empty()) {
        ImGui::TextColored(ImVec4(0.0f, 0.9f, 0.7f, 1.0f), "⚡ Batch Signals to Plot / Probe (%d Electrical):", (int)electricalComps.size());
        ImGui::Spacing();

        auto& cd = canvas.getCircuitRef();
        if (cd.plotConfig.plots.empty()) {
            cd.plotConfig.plots.push_back({ "Waveform Analysis", {} });
        }
        auto& plotVars = cd.plotConfig.plots[0].variables;

        // Count how many have Voltage and Current checked
        int numVChecked = 0;
        int numIChecked = 0;
        for (const auto* c : electricalComps) {
            std::string vSig = "V_" + c->id;
            std::string iSig = "I_" + c->id;
            if (std::find(plotVars.begin(), plotVars.end(), vSig) != plotVars.end() ||
                (c->parameters.count("plotV") && c->parameters.at("plotV") == "1")) {
                numVChecked++;
            }
            if (std::find(plotVars.begin(), plotVars.end(), iSig) != plotVars.end() ||
                (c->parameters.count("plotI") && c->parameters.at("plotI") == "1")) {
                numIChecked++;
            }
        }

        bool allV = (numVChecked == (int)electricalComps.size());
        bool allI = (numIChecked == (int)electricalComps.size());

        // Batch Voltage Checkbox
        bool batchV = allV;
        std::string vLabel = "Voltage (V) [Batch All " + std::to_string(electricalComps.size()) + "]";
        if (numVChecked > 0 && numVChecked < (int)electricalComps.size()) {
            vLabel += " (Partial " + std::to_string(numVChecked) + "/" + std::to_string(electricalComps.size()) + ")";
        }
        if (ImGui::Checkbox(vLabel.c_str(), &batchV)) {
            for (auto* c : electricalComps) {
                std::string vSig = "V_" + c->id;
                if (batchV) {
                    if (std::find(plotVars.begin(), plotVars.end(), vSig) == plotVars.end()) {
                        plotVars.push_back(vSig);
                    }
                    c->parameters["plotV"] = "1";
                    c->parameters["probe_signal"] = "1";
                } else {
                    auto eraseIt = std::find(plotVars.begin(), plotVars.end(), vSig);
                    if (eraseIt != plotVars.end()) plotVars.erase(eraseIt);
                    c->parameters["plotV"] = "0";
                    bool anyLeft = false;
                    for (const auto& v : plotVars) {
                        if (v.find(c->id) != std::string::npos) { anyLeft = true; break; }
                    }
                    if (!anyLeft) c->parameters["probe_signal"] = "0";
                }
            }
        }

        // Batch Current Checkbox
        bool batchI = allI;
        std::string iLabel = "Current (I) [Batch All " + std::to_string(electricalComps.size()) + "]";
        if (numIChecked > 0 && numIChecked < (int)electricalComps.size()) {
            iLabel += " (Partial " + std::to_string(numIChecked) + "/" + std::to_string(electricalComps.size()) + ")";
        }
        if (ImGui::Checkbox(iLabel.c_str(), &batchI)) {
            for (auto* c : electricalComps) {
                std::string iSig = "I_" + c->id;
                if (batchI) {
                    if (std::find(plotVars.begin(), plotVars.end(), iSig) == plotVars.end()) {
                        plotVars.push_back(iSig);
                    }
                    c->parameters["plotI"] = "1";
                    c->parameters["probe_signal"] = "1";
                } else {
                    auto eraseIt = std::find(plotVars.begin(), plotVars.end(), iSig);
                    if (eraseIt != plotVars.end()) plotVars.erase(eraseIt);
                    c->parameters["plotI"] = "0";
                    bool anyLeft = false;
                    for (const auto& v : plotVars) {
                        if (v.find(c->id) != std::string::npos) { anyLeft = true; break; }
                    }
                    if (!anyLeft) c->parameters["probe_signal"] = "0";
                }
            }
        }

        // Quick Batch Probing Buttons
        ImGui::Spacing();
        if (ImGui::Button("Probe All V", ImVec2(90, 24))) {
            for (auto* c : electricalComps) {
                std::string vSig = "V_" + c->id;
                if (std::find(plotVars.begin(), plotVars.end(), vSig) == plotVars.end()) {
                    plotVars.push_back(vSig);
                }
                c->parameters["plotV"] = "1";
                c->parameters["probe_signal"] = "1";
            }
        }
        ImGui::SameLine();
        if (ImGui::Button("Probe All I", ImVec2(90, 24))) {
            for (auto* c : electricalComps) {
                std::string iSig = "I_" + c->id;
                if (std::find(plotVars.begin(), plotVars.end(), iSig) == plotVars.end()) {
                    plotVars.push_back(iSig);
                }
                c->parameters["plotI"] = "1";
                c->parameters["probe_signal"] = "1";
            }
        }
        ImGui::SameLine();
        if (ImGui::Button("Clear Probes", ImVec2(90, 24))) {
            for (auto* c : electricalComps) {
                std::string vSig = "V_" + c->id;
                std::string iSig = "I_" + c->id;
                auto itV = std::find(plotVars.begin(), plotVars.end(), vSig);
                if (itV != plotVars.end()) plotVars.erase(itV);
                auto itI = std::find(plotVars.begin(), plotVars.end(), iSig);
                if (itI != plotVars.end()) plotVars.erase(itI);
                c->parameters["plotV"] = "0";
                c->parameters["plotI"] = "0";
                c->parameters["probe_signal"] = "0";
            }
        }
        ImGui::Separator();
        ImGui::Spacing();
    }

    // ── 3. BATCH PARAMETER EDITING (FOR SAME TYPE SELECTION) ──
    if (sameType) {
        ImGui::TextColored(ImVec4(0.2f, 0.8f, 1.0f, 1.0f), "🛠 Batch Parameter Editor:");
        ImGui::TextDisabled("Single value (e.g. 100) or comma/space separated (e.g. 10, 20, 50)");
        ImGui::Spacing();

        // Collect all common parameter keys present in the first component
        std::vector<std::string> paramKeys;
        for (const auto& pair : selectedComps[0]->parameters) {
            std::string p = pair.first;
            if (p == "probe_signal" || p == "plotI" || p == "plotV" || p == "target" || p == "selected_signals" || p == "probe_type" || p == "num_inputs" || p == "num_outputs" || p == "timestep" || p == "code") continue;
            paramKeys.push_back(p);
        }

        for (const auto& key : paramKeys) {
            std::string displayVal = "";
            bool allIdentical = true;
            std::string firstVal = selectedComps[0]->parameters.count(key) ? selectedComps[0]->parameters.at(key) : "";

            for (size_t i = 0; i < selectedComps.size(); ++i) {
                std::string curV = selectedComps[i]->parameters.count(key) ? selectedComps[i]->parameters.at(key) : "";
                if (i > 0) displayVal += ", ";
                displayVal += curV;
                if (curV != firstVal) allIdentical = false;
            }

            char valBuf[512] = {0};
            if (allIdentical) {
                strncpy(valBuf, firstVal.c_str(), sizeof(valBuf) - 1);
            } else {
                strncpy(valBuf, displayVal.c_str(), sizeof(valBuf) - 1);
            }

            std::string inputLabel = key + "##batch_" + key;
            if (ImGui::InputText(inputLabel.c_str(), valBuf, sizeof(valBuf))) {
                std::string rawInput(valBuf);
                std::vector<std::string> tokens;
                std::string currentTok = "";
                for (char ch : rawInput) {
                    if (ch == ',' || ch == ' ') {
                        if (!currentTok.empty()) {
                            tokens.push_back(currentTok);
                            currentTok = "";
                        }
                    } else {
                        currentTok += ch;
                    }
                }
                if (!currentTok.empty()) tokens.push_back(currentTok);

                if (tokens.size() == 1) {
                    for (auto* c : selectedComps) {
                        c->parameters[key] = tokens[0];
                    }
                } else if (tokens.size() > 1) {
                    for (size_t i = 0; i < selectedComps.size(); ++i) {
                        if (i < tokens.size()) {
                            selectedComps[i]->parameters[key] = tokens[i];
                        } else {
                            selectedComps[i]->parameters[key] = tokens.back();
                        }
                    }
                }
            }
        }
    }

    ImGui::End();
}

void MainWindow::renderPropertyInspector() {
    auto selectedComps = canvas.getSelectedComponents();
    if (selectedComps.empty()) return;

    if (selectedComps.size() > 1) {
        renderBatchPropertyInspector(selectedComps);
        return;
    }

    ComponentInstance* comp = selectedComps[0];
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
    std::string t = comp->rawTypeStr;
    std::transform(t.begin(), t.end(), t.begin(), ::toupper);

    if (t == "PROBE" || t == "UNIFIEDPROBE") {
        ImGui::TextColored(ImVec4(0.2f, 0.8f, 1.0f, 1.0f), "Probe Settings:");
        
        // Target Component Combo
        std::vector<std::string> allTargetIds;
        for (const auto& c : canvas.getCircuitRef().components) {
            if (c.id != comp->id && c.rawTypeStr != "PROBE") {
                allTargetIds.push_back(c.id);
            }
        }

        std::string currentTarget = comp->parameters.count("target") ? comp->parameters.at("target") : "";
        if (currentTarget.empty() && !allTargetIds.empty()) currentTarget = allTargetIds[0];

        if (ImGui::BeginCombo("Target Component##probe", currentTarget.c_str())) {
            for (const auto& targetId : allTargetIds) {
                bool isSelected = (currentTarget == targetId);
                if (ImGui::Selectable(targetId.c_str(), isSelected)) {
                    currentTarget = targetId;
                    comp->parameters["target"] = targetId;
                    std::string pType = comp->parameters.count("probe_type") ? comp->parameters.at("probe_type") : "Voltage";
                    if (pType == "Current" || pType == "I") comp->parameters["selected_signals"] = "I_" + targetId;
                    else comp->parameters["selected_signals"] = "V_" + targetId;
                }
            }
            ImGui::EndCombo();
        }

        // Measure Type Combo
        std::string currentType = comp->parameters.count("probe_type") ? comp->parameters.at("probe_type") : "Voltage";
        const char* probeTypes[] = { "Voltage (V)", "Current (I)" };
        int typeIdx = (currentType == "Current" || currentType == "I") ? 1 : 0;
        if (ImGui::Combo("Measure Type##probe", &typeIdx, probeTypes, 2)) {
            std::string newType = (typeIdx == 1) ? "Current" : "Voltage";
            comp->parameters["probe_type"] = newType;
            if (!currentTarget.empty()) {
                comp->parameters["selected_signals"] = (typeIdx == 1) ? ("I_" + currentTarget) : ("V_" + currentTarget);
            }
        }

        // Custom Signal String fallback
        std::string selSig = comp->parameters.count("selected_signals") ? comp->parameters.at("selected_signals") : "";
        char sigBuf[256] = {0};
        strncpy(sigBuf, selSig.c_str(), sizeof(sigBuf) - 1);
        if (ImGui::InputText("Signal Key##probe", sigBuf, sizeof(sigBuf))) {
            comp->parameters["selected_signals"] = sigBuf;
        }
    }

    if (t == "CSCRIPT") {
        if (ImGui::Button("Open C-Script IDE Editor", ImVec2(-1, 28))) {
            canvas.openCScriptModalForComp(comp->id);
        }
        ImGui::Spacing();

        std::string tsStr = comp->parameters.count("timestep") ? comp->parameters.at("timestep") : "0";
        char tsBuf[64] = {0};
        strncpy(tsBuf, tsStr.c_str(), sizeof(tsBuf) - 1);
        if (ImGui::InputText("Timestep (0=Cont)##cscript", tsBuf, sizeof(tsBuf))) {
            comp->parameters["timestep"] = tsBuf;
        }

        // Custom Parameters Section (Auto-Discovered from C-Script Code)
        std::string codeStr = comp->parameters.count("code") ? comp->parameters.at("code") : "";
        auto discParams = CircuitSimEngine::CScriptEngine::discoverParamsFromCode(codeStr);
        std::set<std::string> discParamNames;
        for (const auto& dp : discParams) discParamNames.insert(dp.name);

        if (!discParams.empty()) {
            ImGui::Spacing();
            ImGui::TextColored(ImVec4(0.2f, 0.8f, 1.0f, 1.0f), "Custom Parameters (Auto-Discovered):");
            for (const auto& dp : discParams) {
                std::string curValStr = comp->parameters.count(dp.name) ? comp->parameters.at(dp.name) : dp.rawValStr;
                char pBuf[64] = {0};
                strncpy(pBuf, curValStr.c_str(), sizeof(pBuf) - 1);
                std::string labelStr = dp.name + " (" + dp.typeStr + ")##" + comp->id;
                if (ImGui::InputText(labelStr.c_str(), pBuf, sizeof(pBuf))) {
                    comp->parameters[dp.name] = pBuf;
                    try {
                        double nVal = CircuitSimEngine::ExpressionEvaluator::parseScientific(pBuf);
                        std::string updatedCode = CircuitSimEngine::CScriptEngine::updateParamInCode(codeStr, dp.name, nVal);
                        comp->parameters["code"] = updatedCode;
                        codeStr = updatedCode;
                    } catch (...) {}
                }
            }
        }
    }

    std::set<std::string> handledCScriptParams;
    if (t == "CSCRIPT") {
        std::string codeStr = comp->parameters.count("code") ? comp->parameters.at("code") : "";
        auto discParams = CircuitSimEngine::CScriptEngine::discoverParamsFromCode(codeStr);
        for (const auto& dp : discParams) handledCScriptParams.insert(dp.name);
    }

    for (auto& pair : comp->parameters) {
        if (pair.first == "probe_signal" || pair.first == "plotI" || pair.first == "plotV" || pair.first == "target" || pair.first == "selected_signals" || pair.first == "probe_type" || pair.first == "num_inputs" || pair.first == "num_outputs" || pair.first == "timestep") continue;
        if (handledCScriptParams.count(pair.first)) continue;

        if (pair.first == "code") {
            char codeBuf[4096] = {0};
            strncpy(codeBuf, pair.second.c_str(), sizeof(codeBuf) - 1);
            if (ImGui::InputTextMultiline("Code##cscript", codeBuf, sizeof(codeBuf), ImVec2(-1, 120))) {
                pair.second = codeBuf;
            }
            continue;
        }

        // ── Dropdown Combos for String Parameters with Known Discrete Options ──
        std::string pKey = pair.first;
        std::transform(pKey.begin(), pKey.end(), pKey.begin(), ::tolower);

        std::vector<std::string> comboChoices;

        if (pKey == "function" || pKey == "func" || pKey == "fcn" || pKey == "expression") {
            if (t == "ROUND") {
                comboChoices = {"round", "floor", "ceil", "fix"};
            } else if (t == "MIN_MAX" || t == "MIN" || t == "MAX") {
                comboChoices = {"min", "max"};
            } else if (t == "TRIG_FCN" || t == "TRIG" || t == "FCN" || t == "MATH_FCN" || t == "MATH_FUNC" || t == "MATH") {
                comboChoices = {"sin", "cos", "tan", "asin", "acos", "atan", "atan2", "sinh", "cosh", "tanh", "exp", "log", "log10", "sqrt", "abs", "square", "pow", "reciprocal"};
            } else if (t == "LOGIC_OP" || t == "COMB_LOGIC") {
                comboChoices = {"AND", "OR", "NAND", "NOR", "XOR", "NXOR", "NOT"};
            }
        } else if (pKey == "operator" || pKey == "op" || pKey == "relop") {
            if (t == "RELATIONAL_OPERATOR" || t == "COMP" || t == "COMPARE_TO_CONSTANT") {
                comboChoices = {"==", "~=", "<", "<=", ">", ">="};
            } else if (t == "BITWISE_OP") {
                comboChoices = {"AND", "OR", "XOR", "NOT", "SHIFT_LEFT", "SHIFT_RIGHT"};
            } else if (t == "LOGIC_OP" || t == "COMB_LOGIC") {
                comboChoices = {"AND", "OR", "NAND", "NOR", "XOR", "NXOR", "NOT"};
            }
        } else if (pKey == "edgetype" || pKey == "edge" || pKey == "direction" || pKey == "hit_direction" || pKey == "dir") {
            if (t == "EDGE_DETECT" || t == "HIT_CROSSING") {
                comboChoices = {"either", "rising", "falling"};
            }
        } else if (pKey == "datatype" || pKey == "output_type") {
            if (t == "DATATYPE_CONV") {
                comboChoices = {"double", "float", "int32", "int16", "int8", "uint32", "uint16", "uint8", "boolean"};
            }
        } else if (pKey == "filter_type" || (pKey == "type" && (t == "FILTER_1ST" || t == "FILTER_2ND"))) {
            comboChoices = {"lowpass", "highpass", "bandpass", "bandstop"};
        } else if (pKey == "alignment" || pKey == "mode" || pKey == "rounding_mode") {
            if (t == "ROUND") {
                comboChoices = {"nearest", "floor", "ceil", "fix"};
            } else if (t == "PWM" || t == "PWM_3PH" || t == "SVPWM") {
                comboChoices = {"edge", "center", "symmetric", "asymmetric"};
            } else if (t == "FOURIER_ANALYSIS" || t == "FOURIER_TRANS") {
                comboChoices = {"magnitude_phase", "real_imag", "harmonic_series"};
            }
        }

        if (!comboChoices.empty()) {
            std::string currentVal = pair.second;
            std::string comboLabel = pair.first + "##" + comp->id;
            if (ImGui::BeginCombo(comboLabel.c_str(), currentVal.c_str())) {
                for (const auto& choice : comboChoices) {
                    bool isSelected = (currentVal == choice);
                    if (ImGui::Selectable(choice.c_str(), isSelected)) {
                        pair.second = choice;
                    }
                    if (isSelected) ImGui::SetItemDefaultFocus();
                }
                ImGui::EndCombo();
            }
            continue;
        }

        char valBuf[256] = {0};
        strncpy(valBuf, pair.second.c_str(), sizeof(valBuf) - 1);
        std::string inputLabel = pair.first + "##" + comp->id;
        if (ImGui::InputText(inputLabel.c_str(), valBuf, sizeof(valBuf))) {
            pair.second = valBuf;
            if (t == "CSCRIPT") {
                try {
                    double nVal = CircuitSimEngine::ExpressionEvaluator::parseScientific(valBuf);
                    std::string codeStr = comp->parameters.count("code") ? comp->parameters.at("code") : "";
                    std::string updatedCode = CircuitSimEngine::CScriptEngine::updateParamInCode(codeStr, pair.first, nVal);
                    comp->parameters["code"] = updatedCode;
                } catch (...) {}
            }
        }
    }

    ImGui::Spacing();
    ImGui::Separator();
    ImGui::TextColored(ImVec4(0.2f, 0.8f, 1.0f, 1.0f), "Signals to Plot / Probe:");
    ImGui::Spacing();

    // Determine signals to plot/probe based on component type (matching Web Tool)
    std::vector<std::string> availableSignals;
    // t is already defined and upper-cased above

    bool isElectrical = (t == "R" || t == "L" || t == "C" || t == "V" || t == "AC_V" || t == "I" || t == "S" || t == "D" || t == "MOSFET" || t == "VM" || t == "AM" || t == "GND");
    if (isElectrical) {
        availableSignals.push_back("V_" + comp->id);
        availableSignals.push_back("I_" + comp->id);
        if (t == "MOSFET" || t == "S") {
            availableSignals.push_back("Ctrl_" + comp->id);
        }
    } else if (t != "SCOPE" && t != "PROBE") {
        // Control / Math component signals
        if (t == "CSCRIPT") {
            std::string codeStr = comp->parameters.count("code") ? comp->parameters.at("code") : "";
            std::vector<CircuitSimEngine::CScriptPort> discIn, discOut;
            CircuitSimEngine::CScriptEngine::discoverPorts(codeStr, discIn, discOut);
            if (!discOut.empty()) {
                for (const auto& op : discOut) {
                    availableSignals.push_back(comp->id + "." + op.name);
                }
            } else {
                availableSignals.push_back(comp->id + ".Out1");
            }
        } else {
            availableSignals.push_back(comp->id + ".Out");
            if (t == "SUM_RECT" || t == "SUM_ROUND" || t == "PRODUCT_RECT" || t == "COMP" || t == "AND" || t == "OR") {
                availableSignals.push_back(comp->id + ".A");
                availableSignals.push_back(comp->id + ".B");
            }
        }
    }

    auto& cd = canvas.getCircuitRef();
    if (cd.plotConfig.plots.empty()) {
        cd.plotConfig.plots.push_back({ "Waveform Analysis", {} });
    }
    auto& plotVars = cd.plotConfig.plots[0].variables;

    for (const auto& sigName : availableSignals) {
        auto it = std::find(plotVars.begin(), plotVars.end(), sigName);
        bool isChecked = (it != plotVars.end()) ||
                         (sigName.rfind("V_", 0) == 0 && comp->parameters.count("plotV") && comp->parameters.at("plotV") == "1") ||
                         (sigName.rfind("I_", 0) == 0 && comp->parameters.count("plotI") && comp->parameters.at("plotI") == "1");

        if (ImGui::Checkbox(sigName.c_str(), &isChecked)) {
            if (isChecked) {
                if (std::find(plotVars.begin(), plotVars.end(), sigName) == plotVars.end()) {
                    plotVars.push_back(sigName);
                }
                if (sigName.rfind("V_", 0) == 0) comp->parameters["plotV"] = "1";
                if (sigName.rfind("I_", 0) == 0) comp->parameters["plotI"] = "1";
                comp->parameters["probe_signal"] = "1";
            } else {
                auto eraseIt = std::find(plotVars.begin(), plotVars.end(), sigName);
                if (eraseIt != plotVars.end()) plotVars.erase(eraseIt);
                if (sigName.rfind("V_", 0) == 0) comp->parameters["plotV"] = "0";
                if (sigName.rfind("I_", 0) == 0) comp->parameters["plotI"] = "0";

                bool anyLeft = false;
                for (const auto& v : plotVars) {
                    if (v.find(comp->id) != std::string::npos) { anyLeft = true; break; }
                }
                if (!anyLeft) comp->parameters["probe_signal"] = "0";
            }
            scopeView.triggerAutoFit();
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
        renderDemoPane();
        renderPropertyInspector();
        canvas.render("Schematic Editor Canvas", ImVec2(800, 600));

        // Handle scope open requests from double-click on SCOPE components
        handleScopeOpenRequest();
    } else {
        netlistSourceView.render("Waveform Solver & Raw Netlist Workspace", canvas.getCircuitRef(), simulator);
    }

    // Render all open scope popup windows (they are independent of workspace mode)
    for (auto& sw : openScopeWindows) {
        sw.setDarkMode(isDarkMode);
        sw.render(simulator, getProjectBaseName());
    }
    // Remove closed scope windows
    openScopeWindows.erase(
        std::remove_if(openScopeWindows.begin(), openScopeWindows.end(),
            [](const ScopeWindow& sw) { return !sw.isWindowOpen(); }),
        openScopeWindows.end());

    renderSimParamsModal();
    renderExportOptionsModal();
}

void MainWindow::handleScopeOpenRequest() {
    if (!canvas.scopeOpenRequest.pending) return;
    canvas.scopeOpenRequest.pending = false;

    const std::string& scopeId = canvas.scopeOpenRequest.scopeId;
    int numCh = canvas.scopeOpenRequest.numChannels;

    // Check if this scope is already open — if so, bring it to focus
    for (auto& sw : openScopeWindows) {
        if (sw.getScopeId() == scopeId) {
            ImGui::SetWindowFocus(("Scope: " + scopeId + "###ScopeWin_" + scopeId).c_str());
            return;
        }
    }

    // Trace wires to find what signals are connected to this SCOPE's input pins
    std::vector<std::string> signalKeys = traceScopeInputSignals(scopeId, numCh);

    // Build labels from signal keys (use short readable names)
    std::vector<std::string> labels;
    for (int ch = 0; ch < numCh; ++ch) {
        if (ch < (int)signalKeys.size() && !signalKeys[ch].empty()) {
            labels.push_back("Ch" + std::to_string(ch + 1) + ": " + signalKeys[ch]);
        } else {
            labels.push_back("Ch" + std::to_string(ch + 1) + " (unconnected)");
        }
    }

    openScopeWindows.emplace_back(scopeId, numCh, signalKeys, labels);
}

std::vector<std::string> MainWindow::traceScopeInputSignals(const std::string& scopeId, int numChannels) {
    return SVGExporter::traceScopeInputSignals(canvas.getCircuit(), scopeId, numChannels);
}

} // namespace CircuitSim
