#pragma once

// The exporter itself is in the core (engine/SVGExporter.hpp). The UI adds the
// Windows save-file dialogs used by the export menus.
#include "engine/SVGExporter.hpp"

#include <string>

namespace CircuitSim {
namespace FileDialogs {

std::string saveSVGFileDialog(const std::string& title = "Export to SVG", const std::string& defaultName = "export.svg");
std::string saveHTMLFileDialog(const std::string& title = "Export Report to HTML", const std::string& defaultName = "report.html");

} // namespace FileDialogs
} // namespace CircuitSim
