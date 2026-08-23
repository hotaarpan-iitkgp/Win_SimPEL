#include <windows.h>
#include <GL/gl.h>
#include "imgui.h"
#include "imgui_internal.h"
#include "imgui_impl_win32.h"
#include "imgui_impl_opengl3.h"
#include "implot.h"
#include "ui/MainWindow.hpp"
#include "ui/SVGExporter.hpp"
#include "ui/NetlistSourceView.hpp"
#include "engine/NetlistBuilder.hpp"
#include "engine/NetlistParser.hpp"
#include "engine/CircuitSimulator.hpp"
#include <nlohmann/json.hpp>
#include <iostream>
#include <fstream>
#include <filesystem>
#include <unordered_map>
#include <unordered_set>
#include <exception>
#include <csignal>
#include <chrono>
#include <ctime>

static std::string buildSchematicJsonString(const CircuitSim::CircuitDesign& cd) {
    nlohmann::json j;
    nlohmann::json compArray = nlohmann::json::array();
    for (const auto& comp : cd.components) {
        nlohmann::json cObj;
        cObj["id"] = comp.id;
        cObj["type"] = comp.rawTypeStr;
        cObj["label"] = comp.label;
        cObj["x"] = comp.x;
        cObj["y"] = comp.y;
        cObj["rotation"] = comp.rotation;
        nlohmann::json pObj = nlohmann::json::object();
        for (const auto& [k, v] : comp.parameters) pObj[k] = v;
        cObj["parameters"] = pObj;
        compArray.push_back(cObj);
    }
    j["components"] = compArray;
    nlohmann::json wireArray = nlohmann::json::array();
    for (const auto& wire : cd.wires) {
        nlohmann::json wObj;
        wObj["id"] = wire.id;
        nlohmann::json fObj;
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
        nlohmann::json tObj;
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
    return j.dump(2);
}

static void parseWireEndpointJSON(
    const nlohmann::json& epObj,
    CircuitSim::WireEndpoint& ep,
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

static void sanitizeCircuitWires(CircuitSim::CircuitDesign& cd) {
    std::vector<CircuitSim::WireInstance> cleanWires;
    std::unordered_map<std::string, const CircuitSim::ComponentInstance*> compMap;
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

    std::vector<CircuitSim::WireInstance> dedupedWires;
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
}

static CircuitSim::CircuitDesign loadSchematicFromJson(const nlohmann::json& j) {
    CircuitSim::CircuitDesign cd;
    if (j.contains("components") && j["components"].is_array()) {
        for (const auto& cItem : j["components"]) {
            CircuitSim::ComponentInstance comp;
            comp.id = cItem.value("id", "");
            comp.rawTypeStr = cItem.value("type", "R");
            comp.type = CircuitSim::stringToComponentType(comp.rawTypeStr);
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
            CircuitSim::setupComponentPins(comp);
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

    if (j.contains("wires") && j["wires"].is_array()) {
        for (const auto& wItem : j["wires"]) {
            CircuitSim::WireInstance wire;
            wire.id = wItem.value("id", "");
            if (wItem.contains("from")) parseWireEndpointJSON(wItem["from"], wire.from, resolveTerminalName);
            if (wItem.contains("to")) parseWireEndpointJSON(wItem["to"], wire.to, resolveTerminalName);
            cd.wires.push_back(wire);
        }
        sanitizeCircuitWires(cd);
    }
    return cd;
}

static bool processSingleJsonFile(
    const std::string& inputFile,
    std::string htmlFile,
    std::string svgFile,
    const std::vector<std::pair<std::string, std::string>>& paramOverrides,
    double tstopOverride,
    double stepOverride,
    CircuitSim::SVGExporter::CircuitReportItem* outReportItem = nullptr,
    const CircuitSim::SVGExporter::ReportExportOptions& exportOptions = CircuitSim::SVGExporter::ReportExportOptions())
{
    if (htmlFile.empty()) {
        size_t dotPos = inputFile.find_last_of('.');
        if (dotPos != std::string::npos) {
            htmlFile = inputFile.substr(0, dotPos) + "_report.html";
        } else {
            htmlFile = inputFile + "_report.html";
        }
    }

    std::ifstream inFile(inputFile);
    if (!inFile.is_open()) {
        std::cerr << "[CLI ERROR] Failed to open input file: " << inputFile << std::endl;
        return false;
    }
    std::string jsonContent((std::istreambuf_iterator<char>(inFile)), std::istreambuf_iterator<char>());
    inFile.close();

    CircuitSim::CircuitDesign design;
    try {
        nlohmann::json j = nlohmann::json::parse(jsonContent);
        design = loadSchematicFromJson(j);
    } catch (const std::exception& e) {
        std::cerr << "[CLI ERROR] Failed to parse input schematic JSON (" << inputFile << "): " << e.what() << std::endl;
        return false;
    }

    for (const auto& overridePair : paramOverrides) {
        std::string compId = overridePair.first;
        std::string paramKey = overridePair.second;
        size_t dotPos = compId.find('.');
        if (dotPos != std::string::npos) {
            paramKey = compId.substr(dotPos + 1);
            compId = compId.substr(0, dotPos);
        }

        bool foundComp = false;
        for (auto& comp : design.components) {
            if (comp.id == compId) {
                comp.parameters[paramKey] = overridePair.second;
                if (paramKey == "R" || paramKey == "C" || paramKey == "L" || paramKey == "value") {
                    comp.parameters[paramKey] = overridePair.second;
                    comp.parameters["value"] = overridePair.second;
                }
                std::cout << "[CLI] Overrode " << compId << "." << paramKey << " = " << overridePair.second << std::endl;
                foundComp = true;
                break;
            }
        }
        if (!foundComp) {
            std::cout << "[CLI Warning] Component ID '" << compId << "' not found for parameter override." << std::endl;
        }
    }

    CircuitSim::NetlistBuilder::buildNodesForCircuit(design);
    std::string jsonNetlist = CircuitSim::NetlistSourceView::generateNetlistJson(design);

    std::vector<CircuitSimEngine::ComponentModel> physComps;
    std::vector<CircuitSimEngine::ComponentModel> ctrlComps;
    CircuitSimEngine::SimulationConfig simCfg;
    CircuitSimEngine::NetlistParser::parseJsonString(jsonNetlist, physComps, ctrlComps, simCfg);

    if (tstopOverride > 0.0) simCfg.stopTime = tstopOverride;
    if (stepOverride > 0.0) simCfg.stepSize = stepOverride;

    CircuitSimEngine::CircuitSimulator simulator;
    simulator.setup(physComps, ctrlComps, simCfg);
    simulator.reset();

    std::cout << "[CLI] Running Transient Simulation for '" << inputFile << "' (tstop = " << simCfg.stopTime << "s)..." << std::endl;
    CircuitSimEngine::SimulationOutput output = simulator.runTransient();
    simulator.setTelemetryOutput(output);

    auto telemetry = simulator.getTelemetryCopy();

    std::vector<CircuitSim::SVGExporter::ScopeReportData> scopesData;

    for (const auto& comp : design.components) {
        if (comp.type == CircuitSim::ComponentType::Oscilloscope || comp.rawTypeStr == "SCOPE") {
            int numChannels = 2;
            if (comp.parameters.count("channels")) {
                try { numChannels = std::stoi(comp.parameters.at("channels")); } catch(...) {}
            }

            std::vector<std::string> sigKeys = CircuitSim::SVGExporter::traceScopeInputSignals(design, comp.id, numChannels);

            std::vector<std::string> validKeys;
            std::vector<std::string> validLabels;
            for (const auto& k : sigKeys) {
                if (!k.empty() && (telemetry.voltages.count(k) || !telemetry.timeHistory.empty())) {
                    validKeys.push_back(k);
                    validLabels.push_back(k);
                }
            }

            if (!validKeys.empty()) {
                CircuitSim::SVGExporter::ScopeReportData srd;
                srd.scopeId = comp.id;
                srd.scopeTitle = comp.label.empty() ? comp.id : (comp.label + " (" + comp.id + ")");
                srd.signalKeys = validKeys;
                srd.signalLabels = validLabels;
                srd.numPanes = (int)validKeys.size();
                scopesData.push_back(srd);
            }
        }
    }

    if (scopesData.empty()) {
        std::vector<std::string> probeKeys;
        for (const auto& comp : design.components) {
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
            CircuitSim::SVGExporter::ScopeReportData srd;
            srd.scopeId = "Probes";
            srd.scopeTitle = "Probe Waveforms";
            srd.signalKeys = validKeys;
            srd.signalLabels = validLabels;
            srd.numPanes = (int)validKeys.size();
            scopesData.push_back(srd);
        }
    }

    std::string schematicJson = buildSchematicJsonString(design);

    std::string formatStr = (exportOptions.format == CircuitSim::SVGExporter::ReportExportFormat::PDF) ? "PDF" : ((exportOptions.format == CircuitSim::SVGExporter::ReportExportFormat::BOTH_HTML_PDF) ? "HTML & PDF" : "HTML");
    std::cout << "[CLI] Exporting " << formatStr << " Report to '" << htmlFile << "'..." << std::endl;
    bool success = true;
    if (exportOptions.exportIndividual) {
        success = CircuitSim::SVGExporter::exportFullReportToHTML(
            design,
            telemetry,
            scopesData,
            schematicJson,
            jsonNetlist,
            htmlFile,
            false /* Light Mode Only */,
            exportOptions
        );
    }

    if (svgFile.size() > 0) {
        std::cout << "[CLI] Exporting Schematic SVG to '" << svgFile << "'..." << std::endl;
        CircuitSim::SVGExporter::exportSchematicToSVG(design, svgFile, false);
    }

    if (success) {
        try {
            std::string absPath = std::filesystem::absolute(htmlFile).string();
            std::cout << "[CLI SUCCESS] Report exported successfully to:\n  " << absPath << "\n";
        } catch (...) {
            std::cout << "[CLI SUCCESS] Report exported successfully to '" << htmlFile << "'!\n";
        }
        if (outReportItem) {
            std::string jName = "Circuit.json";
            try {
                jName = std::filesystem::path(inputFile).filename().string();
            } catch(...) {}
            outReportItem->jsonName = jName;
            outReportItem->design = design;
            outReportItem->telemetry = telemetry;
            outReportItem->scopesData = scopesData;
            outReportItem->schematicJson = schematicJson;
            outReportItem->netlistJson = jsonNetlist;
        }
        return true;
    } else {
        std::cerr << "[CLI ERROR] Failed to export report for '" << inputFile << "'.\n";
        return false;
    }
}

static bool runHeadlessCLI(int argc, char** argv) {
    bool hasCliArg = false;
    std::string inputFile;
    std::string inputDir;
    std::string htmlFile;
    std::string svgFile;
    std::vector<std::pair<std::string, std::string>> paramOverrides;
    double tstopOverride = -1.0;
    double stepOverride = -1.0;
    CircuitSim::SVGExporter::ReportExportOptions cliExportOptions;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "-h" || arg == "--help") {
            hasCliArg = true;
            break;
        }
        if ((arg == "-i" || arg == "--input") && i + 1 < argc) {
            inputFile = argv[++i];
            hasCliArg = true;
        } else if ((arg == "-dir" || arg == "--dir" || arg == "--input-dir") && i + 1 < argc) {
            inputDir = argv[++i];
            hasCliArg = true;
        } else if ((arg == "-o" || arg == "--output" || arg == "--html") && i + 1 < argc) {
            htmlFile = argv[++i];
            hasCliArg = true;
        } else if (arg == "--pdf") {
            cliExportOptions.format = CircuitSim::SVGExporter::ReportExportFormat::PDF;
            hasCliArg = true;
        } else if (arg == "--both") {
            cliExportOptions.format = CircuitSim::SVGExporter::ReportExportFormat::BOTH_HTML_PDF;
            hasCliArg = true;
        } else if (arg == "--no-json") {
            cliExportOptions.includeSchematicJson = false;
            cliExportOptions.includeNetlistJson = false;
            hasCliArg = true;
        } else if (arg == "--expanded-json") {
            cliExportOptions.jsonDefaultExpanded = true;
            hasCliArg = true;
        } else if (arg == "--static-json") {
            cliExportOptions.jsonCollapsible = false;
            hasCliArg = true;
        } else if (arg == "--merged-only") {
            cliExportOptions.exportIndividual = false;
            cliExportOptions.exportMerged = true;
            hasCliArg = true;
        } else if (arg == "--individual-only") {
            cliExportOptions.exportMerged = false;
            cliExportOptions.exportIndividual = true;
            hasCliArg = true;
        } else if ((arg == "-s" || arg == "--export-svg") && i + 1 < argc) {
            svgFile = argv[++i];
            hasCliArg = true;
        } else if ((arg == "-p" || arg == "--param") && i + 1 < argc) {
            std::string paramSpec = argv[++i];
            size_t eqPos = paramSpec.find('=');
            if (eqPos != std::string::npos) {
                std::string key = paramSpec.substr(0, eqPos);
                std::string val = paramSpec.substr(eqPos + 1);
                paramOverrides.push_back({key, val});
            }
            hasCliArg = true;
        } else if ((arg == "-t" || arg == "--tstop") && i + 1 < argc) {
            try { tstopOverride = std::stod(argv[++i]); } catch (...) {}
            hasCliArg = true;
        } else if ((arg == "-dt" || arg == "--step") && i + 1 < argc) {
            try { stepOverride = std::stod(argv[++i]); } catch (...) {}
            hasCliArg = true;
        }
    }

    if (!hasCliArg) return false;

    if (AttachConsole(ATTACH_PARENT_PROCESS)) {
        FILE* fp;
        freopen_s(&fp, "CONOUT$", "w", stdout);
        freopen_s(&fp, "CONOUT$", "w", stderr);
    }

    std::cout << "\n========================================================\n";
    std::cout << " CircuitSim Pro CLI Engine (Headless Mode)\n";
    std::cout << "========================================================\n";

    if (inputFile.empty() && inputDir.empty()) {
        std::cout << "Usage: circuitsim_pro_win.exe -i <input.json> [options]\n";
        std::cout << "   or: circuitsim_pro_win.exe -dir <folder> [options]\n\n";
        std::cout << "Options:\n";
        std::cout << "  -i, --input <file>          Input schematic JSON file path\n";
        std::cout << "  -dir, --input-dir <folder>  Batch process all .json files in directory\n";
        std::cout << "  -o, --html, --output <file> Output report path (Default: <input>_report.html)\n";
        std::cout << "  --pdf                       Export PDF report using headless print\n";
        std::cout << "  --both                      Export both HTML and PDF reports\n";
        std::cout << "  --no-json                   Exclude JSON structures from report\n";
        std::cout << "  --expanded-json             Start collapsible JSON sections expanded\n";
        std::cout << "  --static-json               Make JSON sections static (uncollapsible)\n";
        std::cout << "  --merged-only               Batch: only export merged master report\n";
        std::cout << "  --individual-only           Batch: only export individual reports\n";
        std::cout << "  -p, --param CompId.Param=Val  Override component parameter (e.g. -p L1.L=200u -p R1.R=20)\n";
        std::cout << "  -s, --export-svg <file>     Export standalone schematic SVG\n";
        std::cout << "  -t, --tstop <seconds>       Override transient simulation stop time\n";
        std::cout << "  -dt, --step <seconds>       Override maximum simulation step size\n";
        std::cout << "  -h, --help                  Show CLI usage guide\n";
        std::cout << "========================================================\n\n";
        std::exit(0);
    }

    if (!inputDir.empty()) {
        std::cout << "[CLI] Batch Exporting directory '" << inputDir << "'...\n";
        int totalProcessed = 0;
        int totalSuccess = 0;
        std::vector<CircuitSim::SVGExporter::CircuitReportItem> allReports;

        try {
            for (const auto& entry : std::filesystem::directory_iterator(inputDir)) {
                if (entry.is_regular_file() && entry.path().extension() == ".json") {
                    std::string filePath = entry.path().string();
                    std::string outHtml = entry.path().parent_path().string() + "/" + entry.path().stem().string() + "_report.html";
                    std::cout << "\n--------------------------------------------------------\n";
                    std::cout << "[CLI Batch " << (totalProcessed + 1) << "] Processing: " << entry.path().filename().string() << std::endl;
                    totalProcessed++;
                    CircuitSim::SVGExporter::CircuitReportItem reportItem;
                    if (processSingleJsonFile(filePath, outHtml, "", paramOverrides, tstopOverride, stepOverride, &reportItem, cliExportOptions)) {
                        totalSuccess++;
                        allReports.push_back(reportItem);
                    }
                }
            }
        } catch (const std::exception& e) {
            std::cerr << "[CLI ERROR] Directory iteration error: " << e.what() << std::endl;
        }

        // Generate Master Merged Report
        if (cliExportOptions.exportMerged && !allReports.empty()) {
            std::string mergedHtmlPath = inputDir + "/_all_simulation_reports_merged.html";
            std::cout << "\n--------------------------------------------------------\n";
            std::cout << "[CLI] Generating Master Merged Report (" << allReports.size() << " schematics)..." << std::endl;
            if (CircuitSim::SVGExporter::exportMergedReportToHTML(allReports, mergedHtmlPath, cliExportOptions)) {
                std::string absMerged = std::filesystem::absolute(mergedHtmlPath).string();
                std::cout << "[CLI SUCCESS] Master Merged Report exported successfully to:\n  " << absMerged << "\n";
            }
        }

        std::cout << "\n========================================================\n";
        std::cout << "[CLI BATCH COMPLETE] Successfully processed " << totalSuccess << " of " << totalProcessed << " JSON files.\n";
        std::cout << "========================================================\n\n";
        std::exit(0);
    }

    bool success = processSingleJsonFile(inputFile, htmlFile, svgFile, paramOverrides, tstopOverride, stepOverride, nullptr, cliExportOptions);
    if (success) {
        std::exit(0);
    } else {
        std::exit(1);
    }

    return true;
}

static void logCrash(const std::string& errorMsg) {
    std::string fullMsg = "\n========================================================\n";
    fullMsg += "CRITICAL ERROR / APPLICATION CRASH DETECTED:\n";
    fullMsg += errorMsg + "\n";
    fullMsg += "========================================================\n";
    
    // 1. Output to PowerShell / Console
    std::cerr << fullMsg << std::endl;
    std::cout << fullMsg << std::endl;
    std::fflush(stderr);
    std::fflush(stdout);

    // 2. Write to crash_log.txt in execution directory
    try {
        std::ofstream logFile("crash_log.txt", std::ios::app);
        if (logFile.is_open()) {
            auto now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
            logFile << "[" << std::ctime(&now) << "] " << fullMsg << "\n";
            logFile.close();
        }
    } catch (...) {}
}

static LONG WINAPI customUnhandledExceptionFilter(EXCEPTION_POINTERS* pExceptionInfo) {
    char buf[512];
    DWORD code = pExceptionInfo ? pExceptionInfo->ExceptionRecord->ExceptionCode : 0;
    void* addr = pExceptionInfo ? pExceptionInfo->ExceptionRecord->ExceptionAddress : nullptr;
    snprintf(buf, sizeof(buf), "Unhandled Win32 SEH Exception: 0x%08X at Address 0x%p", (unsigned int)code, addr);
    logCrash(buf);
    MessageBoxA(NULL, buf, "CircuitSim Pro - Fatal Crash Handler", MB_ICONERROR | MB_OK);
    return EXCEPTION_EXECUTE_HANDLER;
}

static void customTerminateHandler() {
    logCrash("Unhandled C++ std::exception / terminate() called.");
    std::abort();
}

static void customSignalHandler(int sig) {
    char buf[256];
    snprintf(buf, sizeof(buf), "Fatal Signal Received: %d", sig);
    logCrash(buf);
    std::exit(sig);
}

// Forward declare Win32 message handler
extern IMGUI_IMPL_API LRESULT ImGui_ImplWin32_WndProcHandler(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam);

LRESULT WINAPI WndProc(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam) {
    if (ImGui_ImplWin32_WndProcHandler(hWnd, msg, wParam, lParam))
        return true;

    switch (msg) {
        case WM_SIZE:
            if (wParam != SIZE_MINIMIZED) {
                // Handle resize
            }
            return 0;
        case WM_SYSCOMMAND:
            if ((wParam & 0xfff0) == SC_KEYMENU) // Disable ALT application menu
                return 0;
            break;
        case WM_DESTROY:
            PostQuitMessage(0);
            return 0;
    }
    return DefWindowProc(hWnd, msg, wParam, lParam);
}

static HICON createCircuitSimIcon(int size) {
    std::vector<DWORD> pixels(size * size);
    float center = size / 2.0f;
    float radius = size * 0.44f;

    for (int y = 0; y < size; ++y) {
        for (int x = 0; x < size; ++x) {
            float dx = x - center;
            float dy = y - center;
            float dist = std::sqrt(dx * dx + dy * dy);

            if (dist <= radius) {
                if (dist >= radius - 2.5f) {
                    pixels[y * size + x] = 0xFFF8BD38; // Cyan/Gold border (BGRA)
                } else {
                    pixels[y * size + x] = 0xFF2A170F; // Dark navy background
                }
            } else {
                pixels[y * size + x] = 0x00000000; // Transparent
            }
        }
    }

    int midX = size / 2;
    int upperY = (int)(center - size * 0.18f);
    int lowerY = (int)(center + size * 0.22f);
    int arm = std::max(2, size / 7);

    // Draw '+' symbol
    for (int d = -arm; d <= arm; ++d) {
        if (upperY >= 0 && upperY < size && midX + d >= 0 && midX + d < size)
            pixels[upperY * size + (midX + d)] = 0xFF00E6FF;
        if (upperY + d >= 0 && upperY + d < size && midX >= 0 && midX < size)
            pixels[(upperY + d) * size + midX] = 0xFF00E6FF;
    }

    // Draw '-' symbol
    for (int d = -arm; d <= arm; ++d) {
        if (lowerY >= 0 && lowerY < size && midX + d >= 0 && midX + d < size)
            pixels[lowerY * size + (midX + d)] = 0xFF00E6FF;
    }

    HBITMAP hColor = CreateBitmap(size, size, 1, 32, pixels.data());
    HBITMAP hMask = CreateBitmap(size, size, 1, 1, NULL);

    ICONINFO ii = {0};
    ii.fIcon = TRUE;
    ii.hbmColor = hColor;
    ii.hbmMask = hMask;

    HICON hIcon = CreateIconIndirect(&ii);
    DeleteObject(hColor);
    DeleteObject(hMask);
    return hIcon;
}

int main(int argc, char** argv) {
    SetUnhandledExceptionFilter(customUnhandledExceptionFilter);
    std::set_terminate(customTerminateHandler);
    std::signal(SIGSEGV, customSignalHandler);
    std::signal(SIGABRT, customSignalHandler);
    std::signal(SIGFPE, customSignalHandler);
    std::signal(SIGILL, customSignalHandler);

    if (runHeadlessCLI(argc, argv)) {
        return 0;
    }

    HICON hIconBig = createCircuitSimIcon(32);
    HICON hIconSmall = createCircuitSimIcon(16);

    // Register Win32 Window Class
    WNDCLASSEXW wc = { sizeof(WNDCLASSEXW), CS_OWNDC, WndProc, 0L, 0L, GetModuleHandle(NULL), hIconBig, NULL, NULL, NULL, L"CircuitSimProWinClass", hIconSmall };
    RegisterClassExW(&wc);
    HWND hwnd = CreateWindowW(wc.lpszClassName, L"CircuitSim Pro - Native High Performance C++ Windows Desktop Tool", WS_OVERLAPPEDWINDOW, 100, 100, 1400, 900, NULL, NULL, wc.hInstance, NULL);

    SendMessage(hwnd, WM_SETICON, ICON_BIG, (LPARAM)hIconBig);
    SendMessage(hwnd, WM_SETICON, ICON_SMALL, (LPARAM)hIconSmall);

    // Initialize OpenGL Context
    PIXELFORMATDESCRIPTOR pfd = {
        sizeof(PIXELFORMATDESCRIPTOR), 1, PFD_DRAW_TO_WINDOW | PFD_SUPPORT_OPENGL | PFD_DOUBLEBUFFER,
        PFD_TYPE_RGBA, 32, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 24, 8, 0, PFD_MAIN_PLANE, 0, 0, 0, 0
    };
    HDC hdc = GetDC(hwnd);
    int pf = ChoosePixelFormat(hdc, &pfd);
    SetPixelFormat(hdc, pf, &pfd);
    HGLRC hglrc = wglCreateContext(hdc);
    wglMakeCurrent(hdc, hglrc);

    ShowWindow(hwnd, SW_SHOWDEFAULT);
    UpdateWindow(hwnd);

    // Setup Dear ImGui context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImPlot::CreateContext();

    ImGuiIO& io = ImGui::GetIO(); (void)io;
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
    io.ConfigFlags |= ImGuiConfigFlags_ViewportsEnable;
    io.ConfigViewportsNoDecoration = false;
    io.ConfigViewportsNoTaskBarIcon = false;

    // Load Crisp High-Quality TrueType Vector Font (Segoe UI)
    if (GetFileAttributesA("C:\\Windows\\Fonts\\segoeui.ttf") != INVALID_FILE_ATTRIBUTES) {
        io.Fonts->AddFontFromFileTTF("C:\\Windows\\Fonts\\segoeui.ttf", 16.0f);
    } else if (GetFileAttributesA("C:\\Windows\\Fonts\\arial.ttf") != INVALID_FILE_ATTRIBUTES) {
        io.Fonts->AddFontFromFileTTF("C:\\Windows\\Fonts\\arial.ttf", 15.0f);
    } else {
        io.Fonts->AddFontDefault();
    }

    ImGui::StyleColorsDark();

    // When viewports are enabled, tweak WindowRounding/WindowBg so platform windows look consistent
    ImGuiStyle& style = ImGui::GetStyle();
    if (io.ConfigFlags & ImGuiConfigFlags_ViewportsEnable) {
        style.WindowRounding = 0.0f;
        style.Colors[ImGuiCol_WindowBg].w = 1.0f;
    }

    // Global HGLRC context reference for multi-viewport rendering
    static HGLRC s_hglrc = hglrc;
    static HDC s_mainHdc = hdc;

    // Initialize ImGui Win32 & OpenGL3 Backends
    ImGui_ImplWin32_Init(hwnd);
    ImGui_ImplOpenGL3_Init("#version 130");

    // Setup platform renderer callback for secondary viewport OS windows
    if (io.ConfigFlags & ImGuiConfigFlags_ViewportsEnable) {
        ImGuiPlatformIO& platform_io = ImGui::GetPlatformIO();
        platform_io.Renderer_RenderWindow = [](ImGuiViewport* viewport, void*) {
            HWND hwnd = (HWND)viewport->PlatformHandle;
            if (!hwnd) return;
            HDC hdc = ::GetDC(hwnd);
            if (hdc) {
                PIXELFORMATDESCRIPTOR pfd = {
                    sizeof(PIXELFORMATDESCRIPTOR), 1, PFD_DRAW_TO_WINDOW | PFD_SUPPORT_OPENGL | PFD_DOUBLEBUFFER,
                    PFD_TYPE_RGBA, 32, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 24, 8, 0, PFD_MAIN_PLANE, 0, 0, 0, 0
                };
                int pf = ::ChoosePixelFormat(hdc, &pfd);
                if (pf) {
                    ::SetPixelFormat(hdc, pf, &pfd);
                }
                ::wglMakeCurrent(hdc, s_hglrc);
                ::glViewport(0, 0, (GLsizei)viewport->Size.x, (GLsizei)viewport->Size.y);
                ::glClearColor(0.12f, 0.12f, 0.14f, 1.0f);
                ::glClear(GL_COLOR_BUFFER_BIT);
                if (viewport->DrawData) {
                    ImGui_ImplOpenGL3_RenderDrawData(viewport->DrawData);
                }
                ::SwapBuffers(hdc);
                ::ReleaseDC(hwnd, hdc);
            }
        };
    }

    CircuitSim::MainWindow mainWindow;

    // Main render loop
    bool done = false;
    while (!done) {
        MSG msg;
        while (PeekMessage(&msg, NULL, 0U, 0U, PM_REMOVE)) {
            TranslateMessage(&msg);
            DispatchMessage(&msg);
            if (msg.message == WM_QUIT) done = true;
        }
        if (done) break;

        // Start Dear ImGui Frame
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplWin32_NewFrame();
        ImGui::NewFrame();

        // Enable full workspace docking window
        ImGuiDockNodeFlags dockspace_flags = ImGuiDockNodeFlags_None;
        ImGuiWindowFlags window_flags = ImGuiWindowFlags_MenuBar | ImGuiWindowFlags_NoDocking;
        ImGuiViewport* viewport = ImGui::GetMainViewport();
        ImGui::SetNextWindowPos(viewport->WorkPos);
        ImGui::SetNextWindowSize(viewport->WorkSize);
        ImGui::SetNextWindowViewport(viewport->ID);
        ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
        ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
        window_flags |= ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove;
        window_flags |= ImGuiWindowFlags_NoBringToFrontOnFocus | ImGuiWindowFlags_NoNavFocus;

        ImGui::Begin("MainDockSpace", nullptr, window_flags);
        ImGui::PopStyleVar(2);

        ImGuiID dockspace_id = ImGui::GetID("MyDockSpace");
        ImGui::DockSpace(dockspace_id, ImVec2(0.0f, 0.0f), dockspace_flags);

        static bool first_layout = true;
        if (first_layout) {
            first_layout = false;
            ImGui::DockBuilderRemoveNode(dockspace_id);
            ImGui::DockBuilderAddNode(dockspace_id, ImGuiDockNodeFlags_DockSpace);
            ImGui::DockBuilderSetNodeSize(dockspace_id, viewport->WorkSize);

            ImGuiID dock_main_id = dockspace_id;
            ImGuiID dock_left_id = ImGui::DockBuilderSplitNode(dock_main_id, ImGuiDir_Left, 0.20f, nullptr, &dock_main_id);
            ImGuiID dock_right_id = ImGui::DockBuilderSplitNode(dock_main_id, ImGuiDir_Right, 0.22f, nullptr, &dock_main_id);
            ImGuiID dock_top_id = ImGui::DockBuilderSplitNode(dock_main_id, ImGuiDir_Up, 0.08f, nullptr, &dock_main_id);

            ImGuiID dock_left_top_id = dock_left_id;
            ImGuiID dock_left_bottom_id = ImGui::DockBuilderSplitNode(dock_left_top_id, ImGuiDir_Down, 0.45f, nullptr, &dock_left_top_id);

            ImGui::DockBuilderDockWindow("Component Pane", dock_left_top_id);
            ImGui::DockBuilderDockWindow("Demo Circuits Pane", dock_left_bottom_id);
            ImGui::DockBuilderDockWindow("Simulation Control", dock_top_id);
            ImGui::DockBuilderDockWindow("Property Inspector", dock_right_id);
            ImGui::DockBuilderDockWindow("Schematic Editor Canvas", dock_main_id);

            ImGui::DockBuilderFinish(dockspace_id);
        }

        try {
            mainWindow.render();
        } catch (const std::exception& e) {
            logCrash(std::string("Exception caught in main render loop: ") + e.what());
        } catch (...) {
            logCrash("Unknown exception caught in main render loop.");
        }

        ImGui::End();

        // Rendering
        ImGui::Render();
        RECT clientRect;
        GetClientRect(hwnd, &clientRect);
        int fbW = clientRect.right - clientRect.left;
        int fbH = clientRect.bottom - clientRect.top;
        glViewport(0, 0, fbW, fbH);
        glClearColor(0.96f, 0.95f, 0.81f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        // Multi-viewport: update and render additional platform windows
        if (io.ConfigFlags & ImGuiConfigFlags_ViewportsEnable) {
            ImGui::UpdatePlatformWindows();
            ImGui::RenderPlatformWindowsDefault();
            wglMakeCurrent(hdc, hglrc); // Restore main context after rendering sub-viewports
        }

        SwapBuffers(hdc);

        // High-precision frame pacing cap (60 FPS / ~16.6ms target) to prevent 100% CPU core lockup
        static auto lastFrameTime = std::chrono::high_resolution_clock::now();
        auto currentFrameTime = std::chrono::high_resolution_clock::now();
        std::chrono::duration<float, std::milli> elapsed = currentFrameTime - lastFrameTime;
        if (elapsed.count() < 16.0f) {
            DWORD sleepMs = (DWORD)(16.0f - elapsed.count());
            if (sleepMs > 0 && sleepMs <= 16) ::Sleep(sleepMs);
        }
        lastFrameTime = std::chrono::high_resolution_clock::now();
    }

    // Cleanup
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplWin32_Shutdown();
    ImPlot::DestroyContext();
    ImGui::DestroyContext();

    wglMakeCurrent(NULL, NULL);
    wglDeleteContext(hglrc);
    ReleaseDC(hwnd, hdc);
    DestroyWindow(hwnd);
    UnregisterClassW(wc.lpszClassName, wc.hInstance);

    return 0;
}
