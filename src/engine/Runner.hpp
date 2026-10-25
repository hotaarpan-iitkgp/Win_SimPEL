#pragma once

// Headless "design in, waveforms out" pipeline: exactly what the GUI does when a
// design is simulated (node building, netlist emission, parse, transient run),
// without any UI. Shared by simpel_cli and the Python module.

#include "Components.hpp"

#include <atomic>
#include <functional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace CircuitSim {

struct RunOverrides {
    double tStop = -1.0;            // <0 keeps the design's value
    double dt = -1.0;               // <0 keeps the design's value
    std::string stepType;           // "", "fixed" or "variable"
    std::string solver;             // "", "euler", "trapezoidal", ...
    // "CompId.param" -> value, applied before netlist emission. Setting R/L/C/value
    // also sets "value", matching the CLI's -p option.
    std::vector<std::pair<std::string, std::string>> params;
};

// Optional run control. Both members may be left empty.
struct RunControl {
    // Polled about every 33 ms of wall time; setting it stops the run, which then
    // returns the samples produced so far with RunResult::cancelled set.
    const std::atomic<bool>* cancel = nullptr;
    // Called on the simulating thread with (time, stop time) at the same cadence
    // and once at the end.
    std::function<void(double, double)> progress;
};

struct RunResult {
    std::string netlistJson;
    std::vector<double> time;
    std::unordered_map<std::string, std::vector<double>> signals;
    double wallSeconds = 0.0;
    double stopTime = 0.0;
    double stepSize = 0.0;
    std::string solver;
    std::string stepType;
    bool cancelled = false;
    CircuitDesign design;           // the design as simulated (overrides applied, nodes built)

    std::vector<std::string> signalNames() const;   // sorted
};

// Applies `CompId.param=value` overrides. Returns the ids that were not found.
std::vector<std::string> applyParamOverrides(CircuitDesign& design,
                                             const std::vector<std::pair<std::string, std::string>>& params);

// Throws std::runtime_error on an invalid design or netlist.
RunResult runDesign(CircuitDesign design, const RunOverrides& ov = RunOverrides(),
                    const RunControl* control = nullptr);
RunResult runDesignFile(const std::string& path, const RunOverrides& ov = RunOverrides(),
                        const RunControl* control = nullptr);

} // namespace CircuitSim
