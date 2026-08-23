#pragma once

#include "engine/Components.hpp"
#include "engine/CircuitSimulator.hpp"
#include <string>
#include <vector>

namespace CircuitSim {

struct ScopeReportData {
    std::string scopeId;
    std::string scopeTitle;
    std::vector<std::string> signalKeys;
    std::vector<std::string> signalLabels;
    int numPanes = 1;
};

struct CircuitReportItem {
    std::string jsonName;
    CircuitDesign design;
    CircuitSimEngine::TelemetryData telemetry;
    std::vector<ScopeReportData> scopesData;
    std::string schematicJson;
    std::string netlistJson;
};

enum class ReportExportFormat {
    HTML = 0,
    PDF = 1,
    BOTH_HTML_PDF = 2
};

struct ReportExportOptions {
    ReportExportFormat format = ReportExportFormat::HTML;
    bool exportIndividual = true;      // Export individual circuit reports
    bool exportMerged = true;          // Export master merged report
    bool includeSchematicSvg = true;   // Include schematic diagram SVG
    bool includeWaveforms = true;      // Include oscilloscope waveforms
    bool includeSchematicJson = true;  // Include Schematic JSON structure
    bool includeNetlistJson = true;    // Include Netlist JSON specification
    bool jsonCollapsible = true;       // Make JSON sections collapsible (<details>)
    bool jsonDefaultExpanded = false;  // Collapsible JSON starts expanded (<details open>)
};

class SVGExporter {
public:
    using ScopeReportData = CircuitSim::ScopeReportData;
    using CircuitReportItem = CircuitSim::CircuitReportItem;
    using ReportExportFormat = CircuitSim::ReportExportFormat;
    using ReportExportOptions = CircuitSim::ReportExportOptions;

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

    // Export complete Light Mode HTML report combining Schematic SVG, Scope SVGs (per Scope block), Schematic JSON, and Netlist JSON
    static bool exportFullReportToHTML(
        const CircuitDesign& design,
        const CircuitSimEngine::TelemetryData& telemetry,
        const std::vector<ScopeReportData>& scopesData,
        const std::string& schematicJson,
        const std::string& netlistJson,
        const std::string& filename,
        bool isDarkMode = false,
        const ReportExportOptions& options = ReportExportOptions()
    );

    // Export merged Light Mode HTML report combining all circuit reports into one master document
    static bool exportMergedReportToHTML(
        const std::vector<CircuitReportItem>& reports,
        const std::string& filename,
        const ReportExportOptions& options = ReportExportOptions()
    );

    // Converts HTML file to PDF using Windows Edge / Chrome headless mode
    static bool convertHtmlToPdf(const std::string& htmlPath, const std::string& pdfPath);

    // Scope signal tracing helper
    static std::vector<std::string> traceScopeInputSignals(const CircuitDesign& design, const std::string& scopeId, int numChannels);

    // Helper: Windows Save File Dialog for SVG & HTML
    static std::string saveSVGFileDialog(const std::string& title = "Export to SVG", const std::string& defaultName = "export.svg");
    static std::string saveHTMLFileDialog(const std::string& title = "Export Report to HTML", const std::string& defaultName = "report.html");
};

} // namespace CircuitSim
