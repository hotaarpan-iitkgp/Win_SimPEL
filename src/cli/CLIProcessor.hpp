#pragma once

#include "engine/CircuitSimulator.hpp"
#include "engine/Components.hpp"
#include "ui/SVGExporter.hpp"
#include <string>
#include <vector>
#include <map>

namespace CircuitSim {

struct CLIOptions {
    std::string inputJsonPath;
    bool isHeadless = false;
    double stopTime = -1.0;
    double stepSize = -1.0;
    std::string solverType = "";
    std::vector<std::pair<std::string, std::string>> paramOverrides;
    std::string exportHtmlPath;
    std::string exportSvgPath;
    std::string exportSchematicSvgPath;
    std::string exportCsvPath;
    std::string batchDir;
    std::string outDir = ".";
    bool showHelp = false;
};

class CLIProcessor {
public:
    static bool parseArgs(int argc, char** argv, CLIOptions& opts);
    static int runCLI(int argc, char** argv);
    static void printHelp();

    static bool exportHTMLReport(const CircuitDesign& design,
                                 const CircuitSimEngine::TelemetryData& telemetry,
                                 const std::string& filename);

    static bool exportCSVData(const CircuitSimEngine::TelemetryData& telemetry,
                              const std::string& filename);
};

} // namespace CircuitSim
