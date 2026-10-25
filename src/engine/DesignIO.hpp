#pragma once

// Schematic design files (.json) <-> CircuitDesign.
//
// One loader and one writer shared by the GUI, the headless CLI and the Python API.
// The loader is the union of what the GUI's and the CLI's private loaders used to
// accept (both component parameter keys, legacy wire formats, copy-paste wire id
// remapping, scope pin aliases, web-tool parameter aliases) plus every simulation
// settings block that has been written over time:
//   "settings"              lowest precedence (camelCase or snake_case keys)
//   "simulationSettings"    camelCase, what the writer emits
//   "simulation_parameters" snake_case, highest precedence (emitted netlist style)
// The writer always emits "simulationSettings" so a saved design reopens with the
// same solver configuration.

#include "Components.hpp"

#include <nlohmann/json.hpp>
#include <string>

namespace CircuitSim {
namespace DesignIO {

struct LoadOptions {
    // Apply the wire normalisation the schematic canvas applies on load
    // (normaliseDesignWires). Headless callers want this so they simulate exactly
    // what the GUI would; the GUI itself gets it from SchematicCanvas::setCircuit.
    bool normaliseWires = true;
};

CircuitDesign loadDesign(const nlohmann::json& j, const LoadOptions& opts = LoadOptions());

// Throws std::runtime_error if the file cannot be opened, nlohmann::json::exception
// if it is not valid JSON.
CircuitDesign loadDesignFile(const std::string& path, const LoadOptions& opts = LoadOptions());

// Applies the settings blocks of `j` onto `settings` (and `plot` if non-null).
void readSettings(const nlohmann::json& j, SolverSettings& settings, PlotConfig* plot = nullptr);

nlohmann::json saveDesign(const CircuitDesign& design);
std::string saveDesignString(const CircuitDesign& design);   // pretty-printed, indent 2
// Throws std::runtime_error if the file cannot be written.
void saveDesignFile(const CircuitDesign& design, const std::string& path);

} // namespace DesignIO
} // namespace CircuitSim
