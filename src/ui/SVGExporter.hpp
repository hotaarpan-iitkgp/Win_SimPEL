#pragma once

#include "engine/Components.hpp"
#include "engine/CircuitSimulator.hpp"
#include <string>
#include <vector>

namespace CircuitSim {

class SVGExporter {
public:
    // Export full schematic canvas to an SVG file
    static bool exportSchematicToSVG(const CircuitDesign& design, const std::string& filename, bool isDarkMode = true);

    // Export Scope / Oscilloscope plots & subplots to a single SVG file
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

    // Helper: Windows Save File Dialog for SVG
    static std::string saveSVGFileDialog(const std::string& title = "Export to SVG", const std::string& defaultName = "export.svg");
};

} // namespace CircuitSim
