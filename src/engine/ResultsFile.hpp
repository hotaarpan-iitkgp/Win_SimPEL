#pragma once

// .simres: a simulation result the GUI can open without re-simulating.
//
// Layout (all integers little-endian):
//   8 bytes   magic "SIMPELRS"
//   u32       format version (1)
//   u64       header length H
//   H bytes   UTF-8 JSON header:
//             {"format": "simpel-results", "version": 1, "core_version": "...",
//              "design": {...schematic, as saved by DesignIO...},
//              "netlist": "<solver netlist text>",
//              "settings": {...}, "meta": {...},
//              "samples": N, "signals": [{"name": "V_C1", "length": N}, ...]}
//   8*N bytes time, IEEE-754 binary64 little-endian
//   then each signal's samples in header order, 8 bytes each.
// Values are stored bit for bit, so a reloaded result is identical to the run.
// Readers reject a file whose sizes do not add up (truncated or corrupt).

#include <nlohmann/json.hpp>

#include <cstddef>
#include <string>
#include <utility>
#include <vector>

namespace CircuitSim {

constexpr unsigned kResultsFileVersion = 1;

struct ResultsSignalView {
    std::string name;
    const double* data = nullptr;
    size_t size = 0;
};

struct ResultsFileData {
    nlohmann::json design;     // schematic (DesignIO::saveDesign format)
    std::string netlist;
    nlohmann::json settings = nlohmann::json::object();
    nlohmann::json meta = nlohmann::json::object();   // wall_seconds, cancelled, solver, ...
    std::string coreVersion;
    std::vector<double> time;
    std::vector<std::pair<std::string, std::vector<double>>> signals;   // file order
};

// Throws std::runtime_error on I/O errors or duplicate / empty signal names.
void writeResultsFile(const std::string& path, const nlohmann::json& design, const std::string& netlist,
                      const nlohmann::json& settings, const nlohmann::json& meta,
                      const double* time, size_t samples, const std::vector<ResultsSignalView>& signals);

// Throws std::runtime_error with the reason for anything that is not a valid file.
ResultsFileData readResultsFile(const std::string& path);

// True if the file starts with the .simres magic (cheap check for "open" dialogs).
bool looksLikeResultsFile(const std::string& path);

} // namespace CircuitSim
