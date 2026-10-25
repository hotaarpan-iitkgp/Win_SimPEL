#pragma once

// Data exports that need no rendering: CSV of waveforms. SVG / HTML / PDF are in
// SVGExporter.hpp; plotting decimation in Decimate.hpp.

#include "Decimate.hpp"

#include <string>
#include <unordered_map>
#include <vector>

namespace CircuitSim {

// Writes `time` and the chosen signals as CSV: a header row "time,<name>,..." and
// one row per sample, numbers in %.<precision>g (shortest form, no padding). With
// `names` empty, every signal is written in sorted order. A signal shorter than
// `time` leaves its remaining cells empty. Returns false and sets *error on an
// unknown signal name or an I/O failure.
bool exportCSV(const std::vector<double>& time,
               const std::unordered_map<std::string, std::vector<double>>& signals,
               const std::vector<std::string>& names,
               const std::string& path,
               int precision = 12,
               std::string* error = nullptr);

} // namespace CircuitSim
