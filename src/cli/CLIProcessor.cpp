#include "cli/CLIProcessor.hpp"
#include "ui/SVGExporter.hpp"
#include "ui/NetlistSourceView.hpp"
#include "engine/NetlistBuilder.hpp"
#include "engine/ExpressionEvaluator.hpp"
#include <nlohmann/json.hpp>
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <cmath>
#include <chrono>
#include <thread>
#include <filesystem>

namespace CircuitSim {

using json = nlohmann::json;

void CLIProcessor::printHelp() {
    std::cout << "========================================================================\n";
    std::cout << "               CircuitSim Pro - Command Line Interface (CLI)            \n";
    std::cout << "========================================================================\n\n";
    std::cout << "Usage:\n";
    std::cout << "  circuitsim_pro_win.exe [options]\n\n";
    std::cout << "Options:\n";
    std::cout << "  -s, --simulate <file.json>    Path to JSON schematic file to simulate\n";
    std::cout << "  -h, --headless                Run in pure console mode without GUI\n";
    std::cout << "  -t, --tstop <seconds>         Override simulation stop time (e.g. 0.05)\n";
    std::cout << "  -d, --dt <step_size>          Override simulation timestep (e.g. 1u, 10n)\n";
    std::cout << "  -m, --solver <type>           Override solver method (euler, trapezoidal, rk4)\n";
    std::cout << "  -p, --param <Comp.Param=Val>  Override component parameter (e.g. V1.value=250)\n";
    std::cout << "      --export-html <out.html>  Export standalone interactive HTML report\n";
    std::cout << "  -o, --export-svg <out.svg>    Export waveform plots as vector SVG\n";
    std::cout << "      --export-schematic-svg    Export schematic layout diagram as light-mode SVG\n";
    std::cout << "      --export-csv <out.csv>    Export time-series voltage/current telemetry to CSV\n";
    std::cout << "      --batch-dir <folder>      Batch simulate all .json files in target directory\n";
    std::cout << "      --out-dir <folder>        Output directory for batch exports (default: .)\n";
    std::cout << "      --help                    Display CLI help menu and exit\n\n";
    std::cout << "Examples:\n";
    std::cout << "  circuitsim_pro_win.exe -s \"working jsons/Buck_converter.json\" -t 0.02 --export-html \"report.html\"\n";
    std::cout << "  circuitsim_pro_win.exe -s \"working jsons/1ph_inverter.json\" -p \"V1.value=400\" -o \"waves.svg\"\n";
    std::cout << "========================================================================\n";
}

bool CLIProcessor::parseArgs(int argc, char** argv, CLIOptions& opts) {
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];

        if (arg == "--help") {
            opts.showHelp = true;
            return true;
        } else if (arg == "-s" || arg == "--simulate") {
            if (i + 1 < argc) opts.inputJsonPath = argv[++i];
        } else if (arg == "-h" || arg == "--headless") {
            opts.isHeadless = true;
        } else if (arg == "-t" || arg == "--tstop") {
            if (i + 1 < argc) {
                try { opts.stopTime = CircuitSimEngine::ExpressionEvaluator::parseScientific(argv[++i]); } catch (...) {}
            }
        } else if (arg == "-d" || arg == "--dt") {
            if (i + 1 < argc) {
                try { opts.stepSize = CircuitSimEngine::ExpressionEvaluator::parseScientific(argv[++i]); } catch (...) {}
            }
        } else if (arg == "-m" || arg == "--solver") {
            if (i + 1 < argc) opts.solverType = argv[++i];
        } else if (arg == "-p" || arg == "--param") {
            if (i + 1 < argc) {
                std::string spec = argv[++i];
                size_t eqPos = spec.find('=');
                if (eqPos != std::string::npos) {
                    opts.paramOverrides.push_back({spec.substr(0, eqPos), spec.substr(eqPos + 1)});
                }
            }
        } else if (arg == "--export-html") {
            if (i + 1 < argc) opts.exportHtmlPath = argv[++i];
        } else if (arg == "-o" || arg == "--export-svg") {
            if (i + 1 < argc) opts.exportSvgPath = argv[++i];
        } else if (arg == "--export-schematic-svg") {
            if (i + 1 < argc) opts.exportSchematicSvgPath = argv[++i];
        } else if (arg == "--export-csv") {
            if (i + 1 < argc) opts.exportCsvPath = argv[++i];
        } else if (arg == "--batch-dir") {
            if (i + 1 < argc) opts.batchDir = argv[++i];
        } else if (arg == "--out-dir") {
            if (i + 1 < argc) opts.outDir = argv[++i];
        }
    }
    return !opts.inputJsonPath.empty() || !opts.batchDir.empty() || opts.showHelp;
}

static bool loadJsonSchematic(const std::string& path, CircuitDesign& cd) {
    std::ifstream f(path);
    if (!f.is_open()) return false;
    try {
        json j = json::parse(f);
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
                    }
                }
                cd.components.push_back(comp);
            }
        }
        if (j.contains("wires") && j["wires"].is_array()) {
            for (const auto& wItem : j["wires"]) {
                WireInstance wire;
                wire.id = wItem.value("id", "");
                auto parseEp = [](const json& epObj, WireEndpoint& ep) {
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
                        ep.terminal = terminal;
                        ep.targetWireId = "";
                        ep.junctionX = 0.0f;
                        ep.junctionY = 0.0f;
                    }
                };
                if (wItem.contains("from")) parseEp(wItem["from"], wire.from);
                if (wItem.contains("to")) parseEp(wItem["to"], wire.to);
                if (wItem.contains("manualPath") && wItem["manualPath"].is_array()) {
                    for (const auto& ptObj : wItem["manualPath"]) {
                        wire.manualPath.push_back({ptObj.value("x", 0.0f), ptObj.value("y", 0.0f)});
                    }
                }
                cd.wires.push_back(wire);
            }
        }
        if (j.contains("simulation_settings") && j["simulation_settings"].is_object()) {
            const auto& ss = j["simulation_settings"];
            if (ss.contains("solverType")) cd.settings.solverType = ss["solverType"].get<std::string>();
            if (ss.contains("stopTime")) {
                if (ss["stopTime"].is_number()) cd.settings.stopTime = ss["stopTime"].get<double>();
                else if (ss["stopTime"].is_string()) cd.settings.stopTime = CircuitSimEngine::ExpressionEvaluator::parseScientific(ss["stopTime"].get<std::string>());
            }
            if (ss.contains("stepSize")) {
                if (ss["stepSize"].is_number()) cd.settings.stepSize = ss["stepSize"].get<double>();
                else if (ss["stepSize"].is_string()) cd.settings.stepSize = CircuitSimEngine::ExpressionEvaluator::parseScientific(ss["stepSize"].get<std::string>());
            }
        }
        NetlistBuilder::buildNodesForCircuit(cd);
        return true;
    } catch (...) {
        return false;
    }
}

int CLIProcessor::runCLI(int argc, char** argv) {
    CLIOptions opts;
    if (!parseArgs(argc, argv, opts) || opts.showHelp) {
        printHelp();
        return 0;
    }

    std::cout << "[CLI] CircuitSim Pro - Native Execution Engine Started.\n";

    // Handle single schematic simulation
    if (!opts.inputJsonPath.empty()) {
        CircuitDesign cd;
        std::cout << "[CLI] Loading circuit schematic: " << opts.inputJsonPath << " ...\n";
        if (!loadJsonSchematic(opts.inputJsonPath, cd)) {
            std::cerr << "[CLI ERROR] Failed to load JSON schematic from: " << opts.inputJsonPath << "\n";
            return 1;
        }

        // Apply simulation parameter overrides
        if (opts.stopTime > 0.0) cd.settings.stopTime = opts.stopTime;
        if (opts.stepSize > 0.0) cd.settings.stepSize = opts.stepSize;
        if (!opts.solverType.empty()) cd.settings.solverType = opts.solverType;

        // Apply component parameter overrides (-p CompID.Param=Val)
        for (const auto& [target, valStr] : opts.paramOverrides) {
            size_t dotPos = target.find('.');
            if (dotPos != std::string::npos) {
                std::string compId = target.substr(0, dotPos);
                std::string paramName = target.substr(dotPos + 1);
                for (auto& c : cd.components) {
                    if (c.id == compId) {
                        c.parameters[paramName] = valStr;
                        std::cout << "[CLI] Overrode parameter: " << compId << "." << paramName << " = " << valStr << "\n";
                    }
                }
            }
        }

        std::cout << "[CLI] Initializing simulation engine (Solver: " << cd.settings.solverType
                  << ", tstop: " << cd.settings.stopTime << "s, dt: " << cd.settings.stepSize << "s)...\n";

        CircuitSimEngine::CircuitSimulator simulator;
        simulator.loadCircuit(cd);

        auto startTime = std::chrono::high_resolution_clock::now();
        CircuitSimEngine::SimulationOutput simOutput = simulator.runTransient();
        simulator.setTelemetryOutput(simOutput);

        auto endTime = std::chrono::high_resolution_clock::now();
        double elapsedMs = std::chrono::duration<double, std::milli>(endTime - startTime).count();

        std::cout << "[CLI SUCCESS] Simulation completed cleanly in " << std::fixed << std::setprecision(2) << elapsedMs << " ms.\n";

        auto telemetry = simulator.getTelemetryCopy();

        // Export HTML Report
        if (!opts.exportHtmlPath.empty()) {
            std::cout << "[CLI] Exporting Interactive HTML Report: " << opts.exportHtmlPath << " ...\n";
            if (exportHTMLReport(cd, telemetry, opts.exportHtmlPath)) {
                std::cout << "[CLI SUCCESS] Interactive HTML Report saved to: " << opts.exportHtmlPath << "\n";
            } else {
                std::cerr << "[CLI ERROR] Failed to export HTML Report to: " << opts.exportHtmlPath << "\n";
            }
        }

        // Export Waveform SVG
        if (!opts.exportSvgPath.empty()) {
            std::cout << "[CLI] Exporting Scope Waveforms SVG: " << opts.exportSvgPath << " ...\n";
            auto scopesData = SVGExporter::buildScopeReportDataForDesign(cd, telemetry);
            if (!scopesData.empty()) {
                const auto& s = scopesData.front();
                if (SVGExporter::exportScopeToSVG(telemetry, s.signalKeys, s.signalLabels, s.scopeTitle, opts.exportSvgPath, s.numPanes, false)) {
                    std::cout << "[CLI SUCCESS] Waveforms SVG saved to: " << opts.exportSvgPath << "\n";
                }
            }
        }

        // Export Schematic SVG
        if (!opts.exportSchematicSvgPath.empty()) {
            std::cout << "[CLI] Exporting Schematic SVG: " << opts.exportSchematicSvgPath << " ...\n";
            if (SVGExporter::exportSchematicToSVG(cd, opts.exportSchematicSvgPath, false)) {
                std::cout << "[CLI SUCCESS] Schematic SVG saved to: " << opts.exportSchematicSvgPath << "\n";
            }
        }

        // Export CSV Telemetry Data
        if (!opts.exportCsvPath.empty()) {
            std::cout << "[CLI] Exporting CSV Telemetry Data: " << opts.exportCsvPath << " ...\n";
            if (exportCSVData(telemetry, opts.exportCsvPath)) {
                std::cout << "[CLI SUCCESS] Telemetry CSV saved to: " << opts.exportCsvPath << "\n";
            }
        }
    }

    return 0;
}

bool CLIProcessor::exportCSVData(const CircuitSimEngine::TelemetryData& telemetry, const std::string& filename) {
    std::ofstream out(filename);
    if (!out.is_open()) return false;

    const auto& timeVec = telemetry.timeHistory;
    if (timeVec.empty()) return false;

    std::vector<std::string> keys;
    for (const auto& pair : telemetry.voltages) {
        if (pair.first.rfind("node_", 0) != 0 && pair.first != "0" && pair.first != "node_0") {
            keys.push_back(pair.first);
        }
    }

    out << "Time(s)";
    for (const auto& k : keys) out << "," << k;
    out << "\n";

    size_t numRows = timeVec.size();
    for (size_t i = 0; i < numRows; ++i) {
        out << std::setprecision(8) << timeVec[i];
        for (const auto& k : keys) {
            const auto& vec = telemetry.voltages.at(k);
            double val = (i < vec.size()) ? vec[i] : 0.0;
            out << "," << std::setprecision(6) << val;
        }
        out << "\n";
    }
    out.close();
    return true;
}

// ============================================================================
// STANDALONE INTERACTIVE HTML REPORT EXPORTER
// ============================================================================

bool CLIProcessor::exportHTMLReport(const CircuitDesign& design, const CircuitSimEngine::TelemetryData& telemetry, const std::string& filename) {
    CircuitDesign tempDesign = design;
    NetlistBuilder::buildNodesForCircuit(tempDesign);
    std::string jsonNetlist = NetlistSourceView::generateNetlistJson(tempDesign);

    auto scopesData = SVGExporter::buildScopeReportDataForDesign(tempDesign, telemetry);
    
    json j;
    json compArray = json::array();
    for (const auto& comp : tempDesign.components) {
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
    for (const auto& wire : tempDesign.wires) {
        json wObj;
        wObj["id"] = wire.id;
        json fObj;
        if (wire.from.isWireJunction) {
            fObj["type"] = "junction";
            fObj["compId"] = wire.from.targetWireId;
            fObj["terminal"] = "";
            fObj["x"] = wire.from.junctionX;
            fObj["y"] = wire.from.junctionY;
        } else {
            fObj["type"] = "pin";
            fObj["compId"] = wire.from.compId;
            fObj["terminal"] = wire.from.terminal;
        }
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
        wObj["from"] = fObj;
        wObj["to"] = tObj;
        wireArray.push_back(wObj);
    }
    j["wires"] = wireArray;
    std::string schematicJson = j.dump(2);

    return SVGExporter::exportFullReportToHTML(tempDesign, telemetry, scopesData, schematicJson, jsonNetlist, filename, false);
}

} // namespace CircuitSim
