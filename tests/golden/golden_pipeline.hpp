// Shared pipeline for the golden-output harness. See golden_runner.cpp.
#pragma once

#include "reference_loaders.hpp"

#include "engine/CircuitSimulator.hpp"
#include "engine/NetlistBuilder.hpp"
#include "engine/NetlistParser.hpp"
#include "engine/NetlistGenerator.hpp"
#include "engine/DesignNormalize.hpp"
#include "engine/DesignIO.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace golden {

struct Entry {
    std::string name;       // expectation file stem
    std::string design;     // path relative to <root>/designs
    double tstop = -1.0;    // override, <0 = keep the design's value
    double dt = -1.0;       // override, <0 = keep the design's value
    long maxSteps = 20000;  // caps stopTime at maxSteps * stepSize to bound runtime
    std::string stepType;   // non-empty: force SolverSettings::stepType (e.g. "variable")
};

inline std::string trim(const std::string& s) {
    size_t a = s.find_first_not_of(" \t\r\n");
    if (a == std::string::npos) return "";
    size_t b = s.find_last_not_of(" \t\r\n");
    return s.substr(a, b - a + 1);
}

// manifest.txt: one entry per line, `name | design path | key=value ...`; '#' comments.
inline std::vector<Entry> readManifest(const std::filesystem::path& p) {
    std::vector<Entry> out;
    std::ifstream f(p);
    std::string line;
    while (std::getline(f, line)) {
        line = trim(line);
        if (line.empty() || line[0] == '#') continue;
        std::vector<std::string> parts;
        std::stringstream ss(line);
        std::string part;
        while (std::getline(ss, part, '|')) parts.push_back(trim(part));
        if (parts.size() < 2) continue;
        Entry e;
        e.name = parts[0];
        e.design = parts[1];
        if (parts.size() > 2) {
            std::stringstream os(parts[2]);
            std::string kv;
            while (os >> kv) {
                size_t eq = kv.find('=');
                if (eq == std::string::npos) continue;
                std::string k = kv.substr(0, eq), v = kv.substr(eq + 1);
                if (k == "tstop") e.tstop = std::stod(v);
                else if (k == "dt") e.dt = std::stod(v);
                else if (k == "maxsteps") e.maxSteps = std::stol(v);
                else if (k == "step") e.stepType = v;
            }
        }
        out.push_back(e);
    }
    return out;
}

inline uint64_t fnv1a(const void* data, size_t n, uint64_t h = 1469598103934665603ULL) {
    const unsigned char* p = static_cast<const unsigned char*>(data);
    for (size_t i = 0; i < n; ++i) { h ^= p[i]; h *= 1099511628211ULL; }
    return h;
}

inline std::string hex64(uint64_t v) {
    char b[17];
    std::snprintf(b, sizeof b, "%016llx", (unsigned long long)v);
    return b;
}

inline std::string g17(double v) {
    char b[40];
    std::snprintf(b, sizeof b, "%.17g", v);
    return b;
}

inline std::string safeName(std::string s) {
    for (char& c : s) if (c == ' ' || c == '\t') c = '~';
    return s;
}

// ---- Pipeline stages. Each stage calls the *current* production code, so that a
// refactor which changes behaviour shows up as a digest mismatch. ----

// Stage 1: JSON -> CircuitDesign through the production loader. The expectations
// were generated with the frozen pre-refactor GUI loader (golden_ref_gui), so a
// netlist_gui match proves DesignIO is equivalent on the corpus.
inline CircuitSim::CircuitDesign loadDesignGui(const nlohmann::json& j) {
    CircuitSim::DesignIO::LoadOptions o;
    o.normaliseWires = false;   // applied separately below, as the canvas does
    return CircuitSim::DesignIO::loadDesign(j, o);
}

// Stage 2: the normalisation SchematicCanvas::setCircuit applies on open.
inline CircuitSim::CircuitDesign normaliseLikeCanvas(CircuitSim::CircuitDesign cd) {
    CircuitSim::normaliseDesignWires(cd);
    return cd;
}

// Stage 3: node building + netlist emission (MainWindow::startSimulation).
inline std::string emitNetlist(CircuitSim::CircuitDesign& cd) {
    CircuitSim::NetlistBuilder::buildNodesForCircuit(cd);
    return CircuitSim::generateNetlistJson(cd);
}

inline std::vector<std::string> runEntry(const std::filesystem::path& root, const Entry& e) {
    std::vector<std::string> lines;
    lines.push_back("design " + safeName(e.design));
    try {
        std::ifstream f(root / "designs" / e.design);
        if (!f.is_open()) { lines.push_back("error cannot-open"); return lines; }
        nlohmann::json j = nlohmann::json::parse(f);

        // Headless CLI path (main.cpp processSingleJsonFile): netlist only.
        {
            CircuitSim::CircuitDesign cli = golden_ref_cli::loadSchematicFromJson(j);
            std::string nl = emitNetlist(cli);
            lines.push_back("netlist_cli " + hex64(fnv1a(nl.data(), nl.size())) + " " + std::to_string(nl.size()));
        }

        CircuitSim::CircuitDesign cd = normaliseLikeCanvas(loadDesignGui(j));
        if (!e.stepType.empty()) cd.settings.stepType = e.stepType;
        std::string netlist = emitNetlist(cd);
        lines.push_back("netlist_gui " + hex64(fnv1a(netlist.data(), netlist.size())) + " " + std::to_string(netlist.size()));

        // Save -> reload must reproduce the same netlist (settings included).
        {
            CircuitSim::CircuitDesign again = CircuitSim::DesignIO::loadDesign(
                nlohmann::json::parse(CircuitSim::DesignIO::saveDesignString(cd)));
            std::string nl2 = emitNetlist(again);
            lines.push_back(std::string("roundtrip ") + (nl2 == netlist ? "same" : "DIFFERENT"));
        }

        std::vector<CircuitSimEngine::ComponentModel> phys, ctrl;
        CircuitSimEngine::SimulationConfig cfg;
        CircuitSimEngine::NetlistParser::parseJsonString(netlist, phys, ctrl, cfg);
        if (e.tstop > 0) cfg.stopTime = e.tstop;
        if (e.dt > 0) cfg.stepSize = e.dt;
        if (e.maxSteps > 0 && cfg.stepSize > 0 && cfg.stopTime / cfg.stepSize > (double)e.maxSteps)
            cfg.stopTime = cfg.stepSize * (double)e.maxSteps;
        lines.push_back("config tstop=" + g17(cfg.stopTime) + " dt=" + g17(cfg.stepSize) +
                        " solver=" + cfg.solver + " step=" + cfg.step_type +
                        " phys=" + std::to_string(phys.size()) + " ctrl=" + std::to_string(ctrl.size()));

        CircuitSimEngine::CircuitSimulator sim;
        sim.setup(phys, ctrl, cfg);
        sim.reset();
        CircuitSimEngine::SimulationOutput out = sim.runTransient();
        sim.setTelemetryOutput(out);
        CircuitSimEngine::TelemetryData tel = sim.getTelemetryCopy();

        const auto& t = tel.timeHistory;
        lines.push_back("time " + std::to_string(t.size()) + " " + hex64(fnv1a(t.data(), t.size() * sizeof(double))) +
                        " " + (t.empty() ? std::string("-") : g17(t.back())));
        std::vector<std::string> keys;
        keys.reserve(tel.voltages.size());
        for (const auto& kv : tel.voltages) keys.push_back(kv.first);
        std::sort(keys.begin(), keys.end());
        for (const auto& k : keys) {
            const auto& v = tel.voltages.at(k);
            lines.push_back("sig " + safeName(k) + " " + std::to_string(v.size()) + " " +
                            hex64(fnv1a(v.data(), v.size() * sizeof(double))) + " " +
                            (v.empty() ? std::string("-") : g17(v.back())));
        }
    } catch (const std::exception& ex) {
        lines.push_back(std::string("error ") + safeName(ex.what()));
    } catch (...) {
        lines.push_back("error unknown");
    }
    return lines;
}

} // namespace golden
