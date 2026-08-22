#pragma once

#include "engine/Components.hpp"
#include "engine/CircuitSimulator.hpp"
#include <string>
#include <vector>

namespace CircuitSim {

class SVGExporter {
public:
    // Export full schematic canvas to an SVG file or string
    static bool exportSchematicToSVG(const CircuitDesign& design, const std::string& filename, bool isDarkMode = true);
    static bool exportSchematicToSVGString(const CircuitDesign& design, std::string& outSVG, bool isDarkMode = true);

    // Export Scope / Oscilloscope plots & subplots to an SVG file or string
    static bool exportScopeToSVG(
        const CircuitSimEngine::TelemetryData& telemetry,
        const std::vector<std::string>& signalKeys,
        const std::vector<std::string>& labels,
        const std::string& scopeTitle,
        const std::string& filename,
        int numPanes = 1,
        bool isDarkMode = true,
        double timeMin = -1.0, double timeMax = -1.0
    );
    static bool exportScopeToSVGString(
        const CircuitSimEngine::TelemetryData& telemetry,
        const std::vector<std::string>& signalKeys,
        const std::vector<std::string>& labels,
        const std::string& scopeTitle,
        std::string& outSVG,
        int numPanes = 1,
        bool isDarkMode = true,
        double timeMin = -1.0, double timeMax = -1.0
    );

    struct ScopeReportData {
        std::string scopeId;
        std::string scopeTitle;
        std::vector<std::string> signalKeys;
        std::vector<std::string> signalLabels;
        int numPanes = 1;
    };

    // Export complete Light Mode HTML report combining Schematic SVG, Scope SVGs (per Scope block), Schematic JSON, and Netlist JSON
    static bool exportFullReportToHTML(
        const CircuitDesign& design,
        const CircuitSimEngine::TelemetryData& telemetry,
        const std::vector<ScopeReportData>& scopesData,
        const std::string& schematicJson,
        const std::string& netlistJson,
        const std::string& filename,
        bool isDarkMode = false
    );

    // Helper: Windows Save File Dialog for SVG & HTML
    static std::string saveSVGFileDialog(const std::string& title = "Export to SVG", const std::string& defaultName = "export.svg");
    static std::string saveHTMLFileDialog(const std::string& title = "Export Report to HTML", const std::string& defaultName = "report.html");
};

} // namespace CircuitSim
