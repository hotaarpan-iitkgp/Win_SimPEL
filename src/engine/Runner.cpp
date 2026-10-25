#include "Runner.hpp"

#include "CircuitSimulator.hpp"
#include "DesignIO.hpp"
#include "NetlistBuilder.hpp"
#include "NetlistGenerator.hpp"
#include "NetlistParser.hpp"

#include <algorithm>
#include <chrono>
#include <stdexcept>

namespace CircuitSim {

std::vector<std::string> RunResult::signalNames() const {
    std::vector<std::string> k;
    k.reserve(signals.size());
    for (const auto& kv : signals) k.push_back(kv.first);
    std::sort(k.begin(), k.end());
    return k;
}

std::vector<std::string> applyParamOverrides(CircuitDesign& design,
                                             const std::vector<std::pair<std::string, std::string>>& params) {
    std::vector<std::string> missing;
    for (const auto& [spec, value] : params) {
        size_t dot = spec.find('.');
        if (dot == std::string::npos || dot == 0 || dot + 1 >= spec.size())
            throw std::runtime_error("parameter override must be CompId.param: '" + spec + "'");
        std::string compId = spec.substr(0, dot);
        std::string key = spec.substr(dot + 1);
        bool found = false;
        for (auto& comp : design.components) {
            if (comp.id != compId) continue;
            comp.parameters[key] = value;
            if (key == "R" || key == "C" || key == "L" || key == "value") comp.parameters["value"] = value;
            found = true;
            break;
        }
        if (!found) missing.push_back(compId);
    }
    return missing;
}

RunResult runDesign(CircuitDesign design, const RunOverrides& ov, const RunControl* control) {
    if (!ov.stepType.empty()) design.settings.stepType = ov.stepType;
    if (!ov.solver.empty()) design.settings.solverType = ov.solver;
    auto missing = applyParamOverrides(design, ov.params);
    if (!missing.empty()) {
        std::string msg = "parameter override for unknown component(s):";
        for (const auto& m : missing) msg += " " + m;
        throw std::runtime_error(msg);
    }

    RunResult r;
    NetlistBuilder::buildNodesForCircuit(design);
    r.netlistJson = generateNetlistJson(design);

    std::vector<CircuitSimEngine::ComponentModel> phys, ctrl;
    CircuitSimEngine::SimulationConfig cfg;
    CircuitSimEngine::NetlistParser::parseJsonString(r.netlistJson, phys, ctrl, cfg);
    if (ov.tStop > 0.0) cfg.stopTime = ov.tStop;
    if (ov.dt > 0.0) cfg.stepSize = ov.dt;
    if (!(cfg.stopTime > 0.0) || !(cfg.stepSize > 0.0))
        throw std::runtime_error("stop time and step size must be positive");

    r.stopTime = cfg.stopTime;
    r.stepSize = cfg.stepSize;
    r.solver = cfg.solver;
    r.stepType = cfg.step_type;

    const auto t0 = std::chrono::steady_clock::now();
    CircuitSimEngine::CircuitSimulator sim;
    sim.setup(phys, ctrl, cfg);
    sim.reset();
    if (control && (control->cancel || control->progress)) {
        const RunControl ctl = *control;
        sim.setProgressCallback([&sim, ctl](double t, double tStop) {
            if (ctl.cancel && ctl.cancel->load(std::memory_order_relaxed)) sim.requestCancel();
            if (ctl.progress) ctl.progress(t, tStop);
            // The callback may itself request cancellation through the flag.
            if (ctl.cancel && ctl.cancel->load(std::memory_order_relaxed)) sim.requestCancel();
        });
    }
    CircuitSimEngine::SimulationOutput out = sim.runTransient();
    r.cancelled = sim.wasCancelled();
    sim.setTelemetryOutput(out);
    CircuitSimEngine::TelemetryData tel = sim.getTelemetryCopy();
    r.wallSeconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();

    r.time = std::move(tel.timeHistory);
    r.signals = std::move(tel.voltages);
    r.design = std::move(design);
    return r;
}

RunResult runDesignFile(const std::string& path, const RunOverrides& ov, const RunControl* control) {
    return runDesign(DesignIO::loadDesignFile(path), ov, control);
}

} // namespace CircuitSim
