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
#include "engine/LossModelLibrary.hpp"
#include "engine/DesignIO.hpp"
#include "engine/BlockRegistry.hpp"
#include "engine/NetlistGenerator.hpp"
#include "engine/ResultsFile.hpp"

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
    CircuitSimEngine::LossModelLibrary::getInstance().loadAllModelsAsync();
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

bool MainWindow::openResultsFile(const std::string& path, std::string* error) {
    if (simRunning.load()) {
        if (error) *error = "A simulation is running; stop it first.";
        return false;
    }
    try {
        ResultsFileData data = readResultsFile(path);
        DesignIO::LoadOptions lo;
        lo.normaliseWires = false;   // SchematicCanvas::setCircuit applies it
        CircuitDesign cd = DesignIO::loadDesign(data.design, lo);
        NetlistBuilder::buildNodesForCircuit(cd);
        canvas.setCircuit(cd);
        simulator.loadCircuit(cd);
        canvas.fitToScreen();

        CircuitSimEngine::SimulationOutput out;
        out.time = std::move(data.time);
        for (auto& [name, values] : data.signals) out.voltages[name] = std::move(values);
        const double stopTime = data.settings.value("t_stop", out.time.empty() ? 0.0 : out.time.back());
        simulator.presentResults(out, stopTime, data.meta.value("wall_seconds", 0.0));
        scopeView.triggerAutoFit();
        netlistSourceView.triggerAutoFit();
        currentLoadedJsonName = path;
        return true;
    } catch (const std::exception& e) {
        if (error) *error = e.what();
        return false;
    }
}

bool MainWindow::openSchematicFile(const std::string& path, std::string* error) {
    try {
        std::ifstream f(path);
        if (!f.is_open()) throw std::runtime_error("cannot open " + path);
        json j = json::parse(f);
        loadSchematicFromJson(j);
        currentLoadedJsonName = path;
        return true;
    } catch (const std::exception& e) {
        if (error) *error = e.what();
        return false;
    }
}

bool MainWindow::saveResultsFile(const std::string& path, std::string* error) {
    if (simRunning.load()) {
        if (error) *error = "A simulation is running; wait for it to finish.";
        return false;
    }
    try {
        CircuitDesign cd = canvas.getCircuit();
        NetlistBuilder::buildNodesForCircuit(cd);
        const std::string netlist = generateNetlistJson(cd);
        auto tel = simulator.getTelemetryCopy();
        if (tel.timeHistory.empty()) {
            if (error) *error = "There are no results yet: run a simulation first.";
            return false;
        }
        std::vector<std::string> names;
        for (const auto& kv : tel.voltages) names.push_back(kv.first);
        std::sort(names.begin(), names.end());
        std::vector<ResultsSignalView> views;
        for (const auto& n : names) views.push_back({n, tel.voltages[n].data(), tel.voltages[n].size()});
        const auto& st = cd.settings;
        json settings = {{"t_stop", st.stopTime}, {"dt", st.stepSize}, {"solver", st.solverType},
                         {"step", st.stepType == "variable" ? std::string("adaptive") : st.stepType}};
        json meta = {{"wall_seconds", simulator.getComputeTimeSeconds()}, {"source", "gui"}, {"cancelled", false}};
        writeResultsFile(path, DesignIO::saveDesign(canvas.getCircuit()), netlist, settings, meta,
                         tel.timeHistory.data(), tel.timeHistory.size(), views);
        return true;
    } catch (const std::exception& e) {
        if (error) *error = e.what();
        return false;
    }
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

static std::string resultsFileDialog(bool save) {
    char szFile[260] = {0};
    OPENFILENAMEA ofn = {0};
    ofn.lStructSize = sizeof(ofn);
    ofn.lpstrFilter = "SimPEL Results (*.simres)\0*.simres\0All Files (*.*)\0*.*\0";
    ofn.lpstrFile = szFile;
    ofn.nMaxFile = sizeof(szFile);
    if (save) {
        ofn.Flags = OFN_PATHMUSTEXIST | OFN_OVERWRITEPROMPT;
        if (!GetSaveFileNameA(&ofn)) return "";
        std::string res(szFile);
        if (res.size() < 7 || res.substr(res.size() - 7) != ".simres") res += ".simres";
        return res;
    }
    ofn.Flags = OFN_PATHMUSTEXIST | OFN_FILEMUSTEXIST;
    return GetOpenFileNameA(&ofn) ? std::string(szFile) : std::string();
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

// Design file I/O lives in engine/DesignIO. The GUI loads without wire normalisation
// because SchematicCanvas::setCircuit applies it.
static DesignIO::LoadOptions guiLoadOptions() {
    DesignIO::LoadOptions o;
    o.normaliseWires = false;
    return o;
}

static std::string buildSchematicJsonString(const CircuitDesign& cd) {
    return DesignIO::saveDesignString(cd);
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
                        CircuitDesign cd = DesignIO::loadDesign(j, guiLoadOptions());
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
            if (ImGui::MenuItem("Open Results (.simres)...")) {
                std::string path = resultsFileDialog(false);
                std::string err;
                if (!path.empty() && !openResultsFile(path, &err))
                    MessageBoxA(NULL, err.c_str(), "Open Results", MB_OK | MB_ICONERROR);
            }
            if (ImGui::MenuItem("Save Results (.simres)...", nullptr, false, !simRunning.load())) {
                std::string path = resultsFileDialog(true);
                std::string err;
                if (!path.empty() && !saveResultsFile(path, &err))
                    MessageBoxA(NULL, err.c_str(), "Save Results", MB_OK | MB_ICONERROR);
            }
            ImGui::Separator();
            if (ImGui::MenuItem("Export Schematic as SVG (.svg)")) {
                std::string defaultName = getProjectBaseName() + "_schematic.svg";
                std::string path = FileDialogs::saveSVGFileDialog("Export Schematic as SVG", defaultName);
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
            if (ImGui::MenuItem("Stop")) {
                simulator.pause();
            }
            if (ImGui::MenuItem("Simulation parameters...", "Ctrl+E")) {
                showSimParamsModal = true;
            }
            ImGui::EndMenu();
        }
        if (ImGui::BeginMenu("Tools")) {
            if (ImGui::MenuItem("Add Loss Analysis Block to Schematic")) {
                addLossAnalyzerBlock();
            }
            if (ImGui::MenuItem("Loss Thermal Model Inspector...")) {
                openThermalModelViewer("");
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
        ImGui::SameLine(0, 10);

        ImGui::PushStyleColor(ImGuiCol_Text, isDarkMode ? ImVec4(1.0f, 0.65f, 0.35f, 1.0f) : ImVec4(0.85f, 0.35f, 0.05f, 1.0f));
        if (ImGui::Button("+ Loss Analysis")) {
            addLossAnalyzerBlock();
        }
        ImGui::PopStyleColor();
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Directly add a Thermal & Loss Analysis block (LOSS_ANALYZER) to the schematic");
        }
        ImGui::SameLine(0, 15);

        ImGui::TextDisabled("|");
        ImGui::SameLine(0, 15);

        if (ImGui::Button("Play")) {
            startSimulation();
        }
        ImGui::SameLine(0, 15);

        if (ImGui::Button("Stop")) {
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

        // The palette comes from the block registry (data/blocks/*.json, "palette"
        // entries), in their "order". The registry outlives this static, so the
        // c_str() pointers stay valid.
        static const std::vector<ComponentMeta> allComponents = []() {
            std::vector<ComponentMeta> v;
            for (const auto& e : BlockRegistry::instance().palette()) {
                v.push_back({ e.text.c_str(), e.label.c_str(), e.prefix.c_str(), e.uiType, e.type.c_str(),
                              e.category.c_str(), e.subcategory.c_str(), e.params, e.basic });
            }
            return v;
        }();

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
                // Include the library category so e.g. "magnetic" surfaces every block
                // in the Magnetic library, not just the ones with it in their name.
                std::string catLower = item.category;
                std::transform(textLower.begin(), textLower.end(), textLower.begin(), ::tolower);
                std::transform(rawLower.begin(), rawLower.end(), rawLower.begin(), ::tolower);
                std::transform(subcatLower.begin(), subcatLower.end(), subcatLower.begin(), ::tolower);
                std::transform(catLower.begin(), catLower.end(), catLower.begin(), ::tolower);

                if (textLower.find(searchQuery) != std::string::npos ||
                    rawLower.find(searchQuery) != std::string::npos ||
                    subcatLower.find(searchQuery) != std::string::npos ||
                    catLower.find(searchQuery) != std::string::npos) {
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

            // Magnetic blocks are all advanced, so this section stays hidden unless one
            // is promoted to the basic set. Rendering it conditionally avoids showing an
            // empty header while still keeping the basic view complete.
            {
                std::vector<const ComponentMeta*> basicMagnetic;
                for (const auto& item : allComponents) {
                    if (item.isBasic && std::string(item.category) == "magnetic") basicMagnetic.push_back(&item);
                }
                if (!basicMagnetic.empty() && ImGui::CollapsingHeader("[MAG] Magnetic")) {
                    ImGui::PushID("basic_magnetic");
                    ImGui::Indent(4.0f);
                    renderComponentGrid(basicMagnetic);
                    ImGui::Unindent(4.0f);
                    ImGui::PopID();
                }
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

            // 4. MAGNETIC BLOCKS (Category: magnetic)
            // Separate top-level library: the magnetic domain uses MMF and flux rate
            // rather than voltage and current, so its blocks are not interchangeable
            // with the electrical ones.
            if (ImGui::CollapsingHeader("[MAG] Magnetic Blocks")) {
                ImGui::PushID("cat_magnetic");
                ImGui::Indent(8.0f);

                std::vector<const ComponentMeta*> magSources, magComponents;
                for (const auto& item : allComponents) {
                    if (std::string(item.category) == "magnetic") {
                        std::string sub = item.subcategory;
                        if (sub == "Sources") magSources.push_back(&item);
                        else if (sub == "Components") magComponents.push_back(&item);
                    }
                }

                renderSubheading("Sources", magSources);
                renderSubheading("Components", magComponents);

                ImGui::Unindent(8.0f);
                ImGui::Spacing();
                ImGui::PopID();
            }
        }
    }
    ImGui::End();
}

void MainWindow::loadSchematicFromJson(const json& j) {
    CircuitDesign cd = DesignIO::loadDesign(j, guiLoadOptions());

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
                    CircuitDesign cd = DesignIO::loadDesign(j);
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
                    CircuitDesign cd = DesignIO::loadDesign(j);
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
    std::string path = FileDialogs::saveHTMLFileDialog("Export Report", defaultName);
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
            if (p == "probe_signal" || p == "plotI" || p == "plotV" || p == "target" || p == "selected_signals" || p == "probe_type" || p == "num_inputs" || p == "num_outputs" || p == "timestep" || p == "code" || p == "thermal_model") continue;
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

        // ── BATCH THERMAL & LOSS MODEL (FOR POWER SWITCHES) ──
        bool allPowerSwitches = true;
        for (const auto* c : selectedComps) {
            bool isSw = (c->type == ComponentType::MOSFET || c->type == ComponentType::IGBT ||
                         c->type == ComponentType::IGBTDiode || c->type == ComponentType::Diode ||
                         c->type == ComponentType::Thyristor || c->type == ComponentType::GTO ||
                         c->type == ComponentType::IGCT || c->type == ComponentType::BJT ||
                         c->type == ComponentType::JFET);
            if (!isSw) {
                allPowerSwitches = false;
                break;
            }
        }

        if (allPowerSwitches) {
            ImGui::Spacing();
            ImGui::Separator();
            ImGui::Spacing();
            ImGui::TextColored(ImVec4(0.2f, 0.8f, 1.0f, 1.0f), "Thermal & Loss Model (Batch):");
            ImGui::SameLine();
            if (CircuitSimEngine::LossModelLibrary::getInstance().isLoading()) {
                ImGui::TextColored(ImVec4(1.0f, 0.75f, 0.2f, 1.0f), "[Loading (%d)...]", (int)CircuitSimEngine::LossModelLibrary::getInstance().getLoadedCount());
            } else {
                if (ImGui::SmallButton("Rescan##batch_switch_models")) {
                    CircuitSimEngine::LossModelLibrary::getInstance().refreshAsync();
                }
            }

            // Check if all selected have the same model assigned
            bool allSameModel = true;
            std::string firstModel = selectedComps[0]->parameters.count("thermal_model") 
                                         ? selectedComps[0]->parameters.at("thermal_model") 
                                         : "Ideal (No Loss)";
            for (const auto* c : selectedComps) {
                std::string m = c->parameters.count("thermal_model") 
                                    ? c->parameters.at("thermal_model") 
                                    : "Ideal (No Loss)";
                if (m != firstModel) {
                    allSameModel = false;
                    break;
                }
            }

            std::string previewText = allSameModel ? firstModel : "--- (Mixed / Multiple Models) ---";
            auto models = CircuitSimEngine::LossModelLibrary::getInstance().getAvailableModels();

            if (ImGui::BeginCombo("##batch_thermal_model", previewText.c_str())) {
                bool isIdealSelected = (allSameModel && firstModel == "Ideal (No Loss)");
                if (ImGui::Selectable("Ideal (No Loss)", isIdealSelected)) {
                    for (auto* c : selectedComps) {
                        c->parameters["thermal_model"] = "Ideal (No Loss)";
                    }
                }
                for (const auto& m : models) {
                    bool isSelected = (allSameModel && firstModel == m);
                    if (ImGui::Selectable(m.c_str(), isSelected)) {
                        for (auto* c : selectedComps) {
                            c->parameters["thermal_model"] = m;
                        }
                    }
                }
                ImGui::EndCombo();
            }

            if (allSameModel && firstModel != "Ideal (No Loss)" && !firstModel.empty()) {
                if (ImGui::Button("Inspect Loss Model (JSON / Tables)##batch_view_model", ImVec2(-1, 26))) {
                    openThermalModelViewer(firstModel);
                }
            } else if (!allSameModel) {
                ImGui::TextDisabled("(Select a single model to inspect tables)");
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

    if (t == "LOSS_ANALYZER") {
        ImGui::TextColored(ImVec4(0.2f, 0.8f, 1.0f, 1.0f), "Thermal & Loss Analysis Settings:");
        
        auto availModels = CircuitSimEngine::LossModelLibrary::getInstance().getAvailableModels();
        if (CircuitSimEngine::LossModelLibrary::getInstance().isLoading()) {
            ImGui::TextColored(ImVec4(1.0f, 0.75f, 0.2f, 1.0f), "Scanning models in background (%d loaded)...", (int)availModels.size());
        } else {
            ImGui::TextDisabled("Models detected: %d", (int)availModels.size());
            ImGui::SameLine();
            if (ImGui::SmallButton("Rescan Library##loss_analyzer")) {
                CircuitSimEngine::LossModelLibrary::getInstance().refreshAsync();
            }
        }
        
        std::string startTime = comp->parameters.count("start_time") ? comp->parameters.at("start_time") : "0.0";
        char stBuf[64] = {0};
        strncpy(stBuf, startTime.c_str(), sizeof(stBuf) - 1);
        if (ImGui::InputText("Start Time (s)", stBuf, sizeof(stBuf))) {
            comp->parameters["start_time"] = stBuf;
        }

        std::string duration = comp->parameters.count("duration") ? comp->parameters.at("duration") : "0.02";
        char dBuf[64] = {0};
        strncpy(dBuf, duration.c_str(), sizeof(dBuf) - 1);
        if (ImGui::InputText("Duration (s)", dBuf, sizeof(dBuf))) {
            comp->parameters["duration"] = dBuf;
        }

        std::string tj = comp->parameters.count("tj") ? comp->parameters.at("tj") : "125.0";
        char tjBuf[64] = {0};
        strncpy(tjBuf, tj.c_str(), sizeof(tjBuf) - 1);
        if (ImGui::InputText("Junction Temp Tj (C)", tjBuf, sizeof(tjBuf))) {
            comp->parameters["tj"] = tjBuf;
        }

        if (ImGui::Button("Show Loss Report", ImVec2(-1, 28))) {
            calculateLosses(comp->id);
            showLossReportWindow = true;
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

    bool isPowerSwitch = (comp->type == ComponentType::MOSFET || comp->type == ComponentType::IGBT ||
                          comp->type == ComponentType::IGBTDiode || comp->type == ComponentType::Diode ||
                          comp->type == ComponentType::Thyristor || comp->type == ComponentType::GTO ||
                          comp->type == ComponentType::IGCT || comp->type == ComponentType::BJT ||
                          comp->type == ComponentType::JFET);

    if (isPowerSwitch) {
        ImGui::Spacing();
        ImGui::TextColored(ImVec4(0.2f, 0.8f, 1.0f, 1.0f), "Thermal & Loss Model:");
        ImGui::SameLine();
        if (CircuitSimEngine::LossModelLibrary::getInstance().isLoading()) {
            ImGui::TextColored(ImVec4(1.0f, 0.75f, 0.2f, 1.0f), "[Loading (%d)...]", (int)CircuitSimEngine::LossModelLibrary::getInstance().getLoadedCount());
        } else {
            if (ImGui::SmallButton("Rescan##switch_models")) {
                CircuitSimEngine::LossModelLibrary::getInstance().refreshAsync();
            }
        }

        auto models = CircuitSimEngine::LossModelLibrary::getInstance().getAvailableModels();
        std::string currentModel = comp->parameters.count("thermal_model") ? comp->parameters.at("thermal_model") : "Ideal (No Loss)";
        
        if (ImGui::BeginCombo("##thermal_model", currentModel.c_str())) {
            bool isIdeal = (currentModel == "Ideal (No Loss)");
            if (ImGui::Selectable("Ideal (No Loss)", isIdeal)) {
                comp->parameters["thermal_model"] = "Ideal (No Loss)";
            }
            for (const auto& m : models) {
                bool isSelected = (currentModel == m);
                if (ImGui::Selectable(m.c_str(), isSelected)) {
                    comp->parameters["thermal_model"] = m;
                }
            }
            ImGui::EndCombo();
        }

        if (currentModel != "Ideal (No Loss)" && !currentModel.empty()) {
            if (ImGui::Button("Inspect Loss Model (JSON / Tables)##btn_view_model", ImVec2(-1, 26))) {
                openThermalModelViewer(currentModel);
            }
        }
    }

    // ── Measurement / Probe selection (Voltage and/or Current) ──
    {
        bool isElectrical =
            comp->type == ComponentType::Resistor || comp->type == ComponentType::Capacitor ||
            comp->type == ComponentType::Inductor || comp->type == ComponentType::VoltageSource ||
            comp->type == ComponentType::ACVoltageSource || comp->type == ComponentType::CurrentSource ||
            comp->type == ComponentType::ACCurrentSource || comp->type == ComponentType::Diode ||
            comp->type == ComponentType::Switch || comp->type == ComponentType::MOSFET ||
            comp->type == ComponentType::Thyristor || comp->type == ComponentType::ThreePhaseSource ||
            comp->type == ComponentType::ThreePhaseCurrentSource ||
            comp->type == ComponentType::IGBT || comp->type == ComponentType::IGBTDiode ||
            comp->type == ComponentType::GTO || comp->type == ComponentType::IGCT ||
            comp->type == ComponentType::BJT || comp->type == ComponentType::JFET ||
            comp->type == ComponentType::VGFET ||
            // Magnetic blocks: "voltage" is MMF [A-t] and "current" is flux rate [Wb/s]
            comp->type == ComponentType::Winding || comp->type == ComponentType::MagneticPermeance ||
            comp->type == ComponentType::LinearCore || comp->type == ComponentType::AirGap ||
            comp->type == ComponentType::LeakageFluxPath || comp->type == ComponentType::MagneticResistance ||
            comp->type == ComponentType::MMFSource || comp->type == ComponentType::MMFSourceControlled ||
            comp->type == ComponentType::VariableResistor || comp->type == ComponentType::VariableCapacitor ||
            comp->type == ComponentType::VariableInductor;

        if (isElectrical) {
            ImGui::Spacing();
            ImGui::TextColored(ImVec4(0.2f, 0.8f, 1.0f, 1.0f), "Measure / Probe:");

            bool plotV = comp->parameters.count("plotV") && comp->parameters.at("plotV") == "1";
            bool plotI = comp->parameters.count("plotI") && comp->parameters.at("plotI") == "1";

            if (ImGui::Checkbox("Voltage (V)##measV", &plotV)) {
                comp->parameters["plotV"] = plotV ? "1" : "0";
                canvas.syncProbeSignals();
            }
            ImGui::SameLine();
            if (ImGui::Checkbox("Current (I)##measI", &plotI)) {
                comp->parameters["plotI"] = plotI ? "1" : "0";
                canvas.syncProbeSignals();
            }

            if (comp->type == ComponentType::ThreePhaseSource ||
                comp->type == ComponentType::ThreePhaseCurrentSource) {
                ImGui::TextDisabled("Emitted per phase (e.g. V_%s_A, I_%s_A)", comp->id.c_str(), comp->id.c_str());
            }
            ImGui::Spacing();
        }
    }

    std::set<std::string> handledCScriptParams;
    if (t == "CSCRIPT") {
        std::string codeStr = comp->parameters.count("code") ? comp->parameters.at("code") : "";
        auto discParams = CircuitSimEngine::CScriptEngine::discoverParamsFromCode(codeStr);
        for (const auto& dp : discParams) handledCScriptParams.insert(dp.name);
    }

    const BlockDef* inspectorBlock = BlockRegistry::instance().resolve(comp->rawTypeStr);
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
        // Allowed values come from the block registry ("choices" in data/blocks).
        if (inspectorBlock) {
            if (const auto* ch = inspectorBlock->choicesFor(pKey)) comboChoices = *ch;
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

void MainWindow::calculateLosses(const std::string& compId) {
    if (simRunning.load()) return; // Cannot calculate while running

    // Find the loss analyzer component
    const ComponentInstance* analyzer = nullptr;
    for (const auto& c : canvas.getCircuitRef().components) {
        if (c.id == compId) {
            analyzer = &c;
            break;
        }
    }
    if (!analyzer) return;

    double t_start = 0.0, duration = 0.02, tj = 125.0;
    try { t_start = std::stod(analyzer->parameters.at("start_time")); } catch(...) {}
    try { duration = std::stod(analyzer->parameters.at("duration")); } catch(...) {}
    try { tj = std::stod(analyzer->parameters.at("tj")); } catch(...) {}

    lossResults = CircuitSimEngine::LossAnalysisEngine::runAnalysis(
        simulator, canvas.getCircuitRef(), t_start, duration, tj
    );
}

void MainWindow::addLossAnalyzerBlock() {
    activeWorkspace = WorkspaceMode::SchematicCAD;
    ComponentInstance comp;
    const auto& comps = canvas.getCircuitRef().components;
    int idx = 1;
    while (true) {
        std::string cand = "LOSS_ANALYZER" + std::to_string(idx);
        bool found = false;
        for (const auto& c : comps) {
            if (c.id == cand) {
                found = true;
                break;
            }
        }
        if (!found) {
            comp.id = cand;
            break;
        }
        idx++;
    }
    comp.label = "Loss Analyzer";
    comp.type = ComponentType::LossAnalyzer;
    comp.rawTypeStr = "LOSS_ANALYZER";
    comp.parameters["start_time"] = "0.0";
    comp.parameters["duration"] = "0.02";
    comp.parameters["tj"] = "125.0";

    canvas.addComponent(comp);
}

void MainWindow::renderLossReportWindow() {
    if (!showLossReportWindow) return;

    ImGui::SetNextWindowSize(ImVec2(800, 500), ImGuiCond_FirstUseEver);
    if (ImGui::Begin("Loss Analysis Report", &showLossReportWindow)) {
        if (lossResults.empty()) {
            ImGui::Text("No loss data available. Ensure simulation is run and thermal models are assigned.");
        } else {
            if (ImGui::BeginTable("LossTable", 8, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_Resizable)) {
                ImGui::TableSetupColumn("Component");
                ImGui::TableSetupColumn("Model");
                ImGui::TableSetupColumn("P_cond (W)");
                ImGui::TableSetupColumn("P_sw_on (W)");
                ImGui::TableSetupColumn("P_sw_off (W)");
                ImGui::TableSetupColumn("P_total (W)");
                ImGui::TableSetupColumn("Hard On Count");
                ImGui::TableSetupColumn("Soft On Count");
                ImGui::TableHeadersRow();

                for (const auto& res : lossResults) {
                    ImGui::TableNextRow();
                    ImGui::TableNextColumn(); ImGui::Text("%s", res.componentId.c_str());
                    ImGui::TableNextColumn(); 
                    ImGui::Text("%s", res.modelName.c_str());
                    ImGui::SameLine();
                    if (ImGui::SmallButton(("Inspect##" + res.componentId).c_str())) {
                        openThermalModelViewer(res.modelName);
                    }
                    ImGui::TableNextColumn(); ImGui::Text("%.3f", res.P_cond);
                    ImGui::TableNextColumn(); ImGui::Text("%.3f", res.P_sw_on);
                    ImGui::TableNextColumn(); ImGui::Text("%.3f", res.P_sw_off);
                    ImGui::TableNextColumn(); ImGui::Text("%.3f", res.P_total);
                    ImGui::TableNextColumn(); ImGui::Text("%d", res.num_hard_turn_on);
                    ImGui::TableNextColumn(); ImGui::Text("%d", res.num_soft_turn_on);
                }
                ImGui::EndTable();
            }

            ImGui::Spacing();
            ImGui::Separator();
            ImGui::Spacing();

            if (ImPlot::BeginPlot("Power Loss Breakdown", ImVec2(-1, 300))) {
                ImPlot::SetupAxes("Component", "Power (W)", ImPlotAxisFlags_AutoFit, ImPlotAxisFlags_AutoFit);
                
                std::vector<double> xs, pcond, psw_on, psw_off;
                std::vector<const char*> labels;
                for (size_t i = 0; i < lossResults.size(); ++i) {
                    xs.push_back((double)i);
                    pcond.push_back(lossResults[i].P_cond);
                    psw_on.push_back(lossResults[i].P_sw_on);
                    psw_off.push_back(lossResults[i].P_sw_off);
                    labels.push_back(lossResults[i].componentId.c_str());
                }
                
                ImPlot::SetupAxisTicks(ImAxis_X1, xs.data(), (int)xs.size(), labels.data());

                // Stacked bars: Plot total (p2), then partial (p1), then base (pcond)
                std::vector<double> p1, p2;
                for (size_t i = 0; i < xs.size(); ++i) {
                    double pc = std::max(0.0, pcond[i]);
                    double p_on = std::max(0.0, psw_on[i]);
                    double p_off = std::max(0.0, psw_off[i]);
                    p1.push_back(pc + p_on);
                    p2.push_back(pc + p_on + p_off);
                }
                ImPlot::PlotBars("P_sw_off", xs.data(), p2.data(), (int)xs.size(), 0.5);
                ImPlot::PlotBars("P_sw_on", xs.data(), p1.data(), (int)xs.size(), 0.5);
                ImPlot::PlotBars("P_cond", xs.data(), pcond.data(), (int)xs.size(), 0.5);

                ImPlot::EndPlot();
            }
        }
    }
    ImGui::End();
}

void MainWindow::openThermalModelViewer(const std::string& modelKey) {
    if (!modelKey.empty()) {
        viewingModelKey = modelKey;
    }
    showThermalModelViewer = true;
}

static void renderModelMetadataTab(const CircuitSimEngine::LossModel& model) {
    ImGui::TextColored(ImVec4(0.3f, 0.8f, 1.0f, 1.0f), "Device Specifications & Parameters:");
    ImGui::Spacing();

    if (ImGui::BeginTable("##MetaTable", 2, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_Resizable)) {
        ImGui::TableSetupColumn("Property / Parameter", ImGuiTableColumnFlags_WidthFixed, 220.0f);
        ImGui::TableSetupColumn("Value", ImGuiTableColumnFlags_WidthStretch);
        ImGui::TableHeadersRow();

        auto addRow = [](const char* prop, const std::string& val) {
            ImGui::TableNextRow();
            ImGui::TableNextColumn(); ImGui::TextUnformatted(prop);
            ImGui::TableNextColumn(); ImGui::TextUnformatted(val.c_str());
        };

        addRow("Part Number", model.part_number);
        addRow("Device Type", model.device_type);
        addRow("Gate Charge Qg", std::to_string(model.Qg_nC) + " nC");
        addRow("Leakage Current I_leakage", std::to_string(model.I_leakage_uA) + " uA");
        addRow("Gate Drive Voltage V_drv", std::to_string(model.V_drv_V) + " V");
        addRow("File Path", model.filepath);

        for (const auto& kv : model.metadataMap) {
            if (kv.first != "part_number" && kv.first != "type" && 
                kv.first != "Qg_nC" && kv.first != "I_leakage_uA" && kv.first != "V_drv_V") {
                addRow(kv.first.c_str(), kv.second);
            }
        }

        ImGui::EndTable();
    }
}

static void render3DLossTable(const CircuitSimEngine::Interpolator3D& lut, const char* label) {
    if (!lut.valid || lut.data.empty() || lut.v_axis.empty() || lut.i_axis.empty() || lut.t_axis.empty()) {
        ImGui::Spacing();
        ImGui::TextDisabled("No %s data table available in this JSON model.", label);
        return;
    }

    static int selectedV = 0;
    if (selectedV >= (int)lut.v_axis.size()) selectedV = 0;

    ImGui::Text("Voltage Slice (V_ds):");
    ImGui::SameLine();
    for (size_t v = 0; v < lut.v_axis.size(); ++v) {
        char vbuf[32];
        snprintf(vbuf, sizeof(vbuf), "%.1f V##%s_%zu", lut.v_axis[v], label, v);
        if (ImGui::RadioButton(vbuf, selectedV == (int)v)) {
            selectedV = (int)v;
        }
        if (v + 1 < lut.v_axis.size()) ImGui::SameLine();
    }

    ImGui::Spacing();
    ImGui::TextDisabled("Values displayed in milliJoules (mJ) at V_ds = %.1f V:", lut.v_axis[selectedV]);

    size_t Ni = lut.i_axis.size();
    size_t Nt = lut.t_axis.size();
    int totalCols = (int)Nt + 1;

    std::string tableId = std::string("##3DLossTable_") + label;
    if (ImGui::BeginTable(tableId.c_str(), totalCols, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_ScrollX | ImGuiTableFlags_ScrollY | ImGuiTableFlags_Resizable, ImVec2(0, 360))) {
        ImGui::TableSetupColumn("Current I (A)", ImGuiTableColumnFlags_WidthFixed, 120.0f);
        for (size_t t = 0; t < Nt; ++t) {
            char tHeader[32];
            snprintf(tHeader, sizeof(tHeader), "%.1f °C", lut.t_axis[t]);
            ImGui::TableSetupColumn(tHeader, ImGuiTableColumnFlags_WidthFixed, 100.0f);
        }
        ImGui::TableHeadersRow();

        for (size_t i = 0; i < Ni; ++i) {
            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ImGui::Text("%.2f A", lut.i_axis[i]);

            for (size_t t = 0; t < Nt; ++t) {
                ImGui::TableNextColumn();
                size_t idx = selectedV * (Ni * Nt) + i * Nt + t;
                if (idx < lut.data.size()) {
                    double val_mJ = lut.data[idx] * 1000.0;
                    ImGui::Text("%.4f mJ", val_mJ);
                } else {
                    ImGui::Text("-");
                }
            }
        }
        ImGui::EndTable();
    }
}

static void render2DTable(const CircuitSimEngine::Interpolator2D& lut, const char* title) {
    ImGui::TextColored(ImVec4(0.2f, 0.8f, 1.0f, 1.0f), "%s:", title);
    if (!lut.valid || lut.data.empty() || lut.i_axis.empty() || lut.t_axis.empty()) {
        ImGui::TextDisabled("No table data available.");
        return;
    }

    size_t Ni = lut.i_axis.size();
    size_t Nt = lut.t_axis.size();
    int totalCols = (int)Nt + 1;

    std::string tableId = std::string("##2DCondTable_") + title;
    if (ImGui::BeginTable(tableId.c_str(), totalCols, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_ScrollX | ImGuiTableFlags_ScrollY | ImGuiTableFlags_Resizable, ImVec2(0, 200))) {
        ImGui::TableSetupColumn("Current I (A)", ImGuiTableColumnFlags_WidthFixed, 120.0f);
        for (size_t t = 0; t < Nt; ++t) {
            char tHeader[32];
            snprintf(tHeader, sizeof(tHeader), "%.1f °C", lut.t_axis[t]);
            ImGui::TableSetupColumn(tHeader, ImGuiTableColumnFlags_WidthFixed, 100.0f);
        }
        ImGui::TableHeadersRow();

        for (size_t i = 0; i < Ni; ++i) {
            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ImGui::Text("%.2f A", lut.i_axis[i]);

            for (size_t t = 0; t < Nt; ++t) {
                ImGui::TableNextColumn();
                size_t idx = i * Nt + t;
                if (idx < lut.data.size()) {
                    ImGui::Text("%.4f V", lut.data[idx]);
                } else {
                    ImGui::Text("-");
                }
            }
        }
        ImGui::EndTable();
    }
}

void MainWindow::renderThermalModelViewerModal() {
    if (!showThermalModelViewer) return;

    ImGui::SetNextWindowSize(ImVec2(850, 600), ImGuiCond_FirstUseEver);
    if (ImGui::Begin("Thermal & Loss Model Inspector", &showThermalModelViewer)) {
        auto availModels = CircuitSimEngine::LossModelLibrary::getInstance().getAvailableModels();
        bool isLoading = CircuitSimEngine::LossModelLibrary::getInstance().isLoading();

        if (viewingModelKey.empty() && !availModels.empty()) {
            viewingModelKey = availModels[0];
        }

        ImGui::Text("Selected Model:");
        ImGui::SameLine();
        ImGui::SetNextItemWidth(380.0f);
        if (ImGui::BeginCombo("##inspect_model_combo", viewingModelKey.c_str())) {
            for (const auto& m : availModels) {
                bool isSelected = (viewingModelKey == m);
                if (ImGui::Selectable(m.c_str(), isSelected)) {
                    viewingModelKey = m;
                }
                if (isSelected) ImGui::SetItemDefaultFocus();
            }
            ImGui::EndCombo();
        }

        ImGui::SameLine();
        if (isLoading) {
            ImGui::TextColored(ImVec4(1.0f, 0.75f, 0.2f, 1.0f), "[Loading (%d)...]", (int)availModels.size());
        } else {
            if (ImGui::Button("Rescan Library##insp_rescan")) {
                CircuitSimEngine::LossModelLibrary::getInstance().refreshAsync();
            }
        }

        auto model = CircuitSimEngine::LossModelLibrary::getInstance().getModel(viewingModelKey);
        if (!model) {
            if (isLoading) {
                ImGui::TextColored(ImVec4(1.0f, 0.75f, 0.2f, 1.0f), "Thermal loss models are currently loading in the background...");
            } else {
                ImGui::TextColored(ImVec4(1, 0.4f, 0.4f, 1), "Model not found or failed to load: %s", viewingModelKey.c_str());
            }
            ImGui::End();
            return;
        }

        ImGui::Separator();
        ImGui::Spacing();

        if (ImGui::BeginTabBar("##ThermalModelTabs")) {
            // ── TAB 1: METADATA ──
            if (ImGui::BeginTabItem("Metadata")) {
                renderModelMetadataTab(*model);
                ImGui::EndTabItem();
            }

            // ── TAB 2: TURN-ON LOSS ──
            if (ImGui::BeginTabItem("Turn-On Loss (E_on)")) {
                render3DLossTable(model->getTurnOnLUT(), "E_on");
                ImGui::EndTabItem();
            }

            // ── TAB 3: TURN-OFF LOSS ──
            if (ImGui::BeginTabItem("Turn-Off Loss (E_off)")) {
                render3DLossTable(model->getTurnOffLUT(), "E_off");
                ImGui::EndTabItem();
            }

            // ── TAB 4: CONDUCTION LOSS ──
            if (ImGui::BeginTabItem("Conduction Losses (V_drop)")) {
                render2DTable(model->getCondOnLUT(), "Gate ON (Channel Conduction Voltage Drop)");
                ImGui::Spacing();
                ImGui::Separator();
                ImGui::Spacing();
                render2DTable(model->getCondOffLUT(), "Gate OFF (Body Diode Conduction Voltage Drop)");
                ImGui::EndTabItem();
            }

            ImGui::EndTabBar();
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
                simStepTypeIdx = (cd.settings.stepType == "variable" ||
                                  cd.settings.stepType == "adaptive") ? 1 : 0;
                std::snprintf(simRelTolBuf,  sizeof(simRelTolBuf),  "%.17g", cd.settings.relTol);
                std::snprintf(simAbsTolVBuf, sizeof(simAbsTolVBuf), "%.17g", cd.settings.absTolV);
                std::snprintf(simAbsTolIBuf, sizeof(simAbsTolIBuf), "%.17g", cd.settings.absTolI);
                std::snprintf(simHMaxBuf,    sizeof(simHMaxBuf),    "%.17g", cd.settings.hMax);
                std::snprintf(simHMinBuf,    sizeof(simHMinBuf),    "%.17g", cd.settings.hMin);
            }

            ImGui::TextColored(ImVec4(0.2f, 0.8f, 1.0f, 1.0f), "Solver & Simulation Configuration");
            ImGui::Separator();
            ImGui::Spacing();

            ImGui::InputText("Stop Time (s)", simStopTimeBuf, sizeof(simStopTimeBuf));
            ImGui::InputText("Step Size (s)", simStepSizeBuf, sizeof(simStepSizeBuf));
            
            const char* solverItems[] = { "Euler (Fixed Step)", "Trapezoidal (Gear/BE)", "Runge-Kutta 4th Order (RK4)" };
            ImGui::Combo("Solver Method", &simSolverIdx, solverItems, IM_ARRAYSIZE(solverItems));

            const char* stepTypeItems[] = { "Fixed step", "Variable step (error controlled)" };
            ImGui::Combo("Step Type", &simStepTypeIdx, stepTypeItems, IM_ARRAYSIZE(stepTypeItems));

            if (simStepTypeIdx == 1) {
                ImGui::Spacing();
                ImGui::TextColored(ImVec4(0.2f, 0.8f, 1.0f, 1.0f), "Variable Step Tolerances");
                ImGui::Separator();
                ImGui::TextWrapped(
                    "Step size is chosen from the local error in the capacitor voltages and "
                    "inductor currents, and every switching edge is landed on exactly so duty "
                    "cycle stays accurate no matter how large the step grows. Here 'Step Size' "
                    "above is the starting step, not a limit.");
                ImGui::Spacing();
                ImGui::InputText("Relative tolerance", simRelTolBuf, sizeof(simRelTolBuf));
                ImGui::InputText("Absolute tolerance - voltage (V)", simAbsTolVBuf, sizeof(simAbsTolVBuf));
                ImGui::InputText("Absolute tolerance - current (A)", simAbsTolIBuf, sizeof(simAbsTolIBuf));
                ImGui::InputText("Max step (s, 0 = auto)", simHMaxBuf, sizeof(simHMaxBuf));
                ImGui::InputText("Min step (s, 0 = auto)", simHMinBuf, sizeof(simHMinBuf));
                if (ImGui::IsItemHovered()) {
                    ImGui::SetTooltip("Auto caps the largest step at a quarter of the shortest "
                                      "carrier period in the circuit.");
                }
            }

            ImGui::Spacing();
            ImGui::Separator();
            ImGui::Spacing();

            if (ImGui::Button("OK", ImVec2(120, 30))) {
                auto& cd = canvas.getCircuitRef();
                try { cd.settings.stopTime = CircuitSimEngine::ExpressionEvaluator::parseScientific(simStopTimeBuf); } catch (...) {}
                try { cd.settings.stepSize = CircuitSimEngine::ExpressionEvaluator::parseScientific(simStepSizeBuf); } catch (...) {}
                cd.settings.solverType = (simSolverIdx == 0) ? "euler" : ((simSolverIdx == 1) ? "trapezoidal" : "rk4");
                cd.settings.stepType = (simStepTypeIdx == 1) ? "variable" : "fixed";
                if (simStepTypeIdx == 1) {
                    auto readField = [](const char* buf, double& dst, double fallback) {
                        try {
                            const double v = CircuitSimEngine::ExpressionEvaluator::parseScientific(buf);
                            dst = (v >= 0.0) ? v : fallback;
                        } catch (...) { dst = fallback; }
                    };
                    readField(simRelTolBuf,  cd.settings.relTol,  1e-3);
                    readField(simAbsTolVBuf, cd.settings.absTolV, 1e-3);
                    readField(simAbsTolIBuf, cd.settings.absTolI, 1e-6);
                    readField(simHMaxBuf,    cd.settings.hMax,    0.0);
                    readField(simHMinBuf,    cd.settings.hMin,    0.0);
                    if (cd.settings.relTol <= 0.0) cd.settings.relTol = 1e-3;
                }
                
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
    renderLossReportWindow();
    renderThermalModelViewerModal();
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
