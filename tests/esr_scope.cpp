// Scope check for the capacitor/inductor ESR state fix.
//
// The fix changes what is stored as the reactive state when a series ESR is
// present. With esr == 0 the expression reduces to the previous one exactly, so
// zero-ESR circuits must stay bitwise identical to the old engine. Compiled
// against both the baseline and the current engine, the printed digests let that
// be checked directly.
//
// usage: esr_scope <esr>        e.g. 0  or  0.01

#include "engine/CircuitSimulator.hpp"
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

using namespace CircuitSimEngine;

static ComponentModel mk(const std::string& id, ComponentType t,
                         const std::vector<std::string>& nodes,
                         const std::vector<std::pair<std::string, std::string>>& params) {
    ComponentModel c; c.id = id; c.type = t; c.nodes = nodes;
    for (const auto& p : params) c.parameters[p.first] = p.second;
    return c;
}

// FNV-1a over the raw bytes of every produced sample: any bit difference shows.
static uint64_t digest(const SimulationOutput& o) {
    uint64_t h = 1469598103934665603ULL;
    auto feed = [&](double d) {
        unsigned char b[sizeof(double)];
        std::memcpy(b, &d, sizeof(double));
        for (size_t i = 0; i < sizeof(double); ++i) { h ^= b[i]; h *= 1099511628211ULL; }
    };
    for (double t : o.time) feed(t);
    auto addMap = [&](const std::unordered_map<std::string, std::vector<double>>& m) {
        std::vector<std::string> keys;
        for (const auto& kv : m) keys.push_back(kv.first);
        std::sort(keys.begin(), keys.end());
        for (const auto& k : keys) {
            for (char c : k) { h ^= (unsigned char)c; h *= 1099511628211ULL; }
            for (double d : m.at(k)) feed(d);
        }
    };
    addMap(o.voltages); addMap(o.signals); addMap(o.inductors);
    addMap(o.voltmeters); addMap(o.ammeters); addMap(o.custom_plots);
    return h;
}

int main(int argc, char** argv) {
    const char* esr = (argc > 1) ? argv[1] : "0";

    for (double dt : {1e-6, 1e-7}) {
        for (const char* solver : {"euler", "trapezoidal"}) {
            std::vector<ComponentModel> phys = {
                mk("V1",      ComponentType::VoltageSource, {"node_1", "node_0"}, {{"value", "100"}}),
                mk("MOSFET1", ComponentType::MOSFET,        {"node_1", "node_2"},
                   {{"Ron", "0.01"}, {"Roff", "1000000"}, {"Vd", "0.8"}, {"Vgt", "0.5"},
                    {"Iholding", "0.01"}, {"control_signal", "PULSE_GEN1.Out"}}),
                mk("D1",      ComponentType::Diode,         {"node_0", "node_2"},
                   {{"Vd", "0.7"}, {"Ron", "0.001"}, {"Roff", "1000000"}}),
                mk("L1",      ComponentType::Inductor,      {"node_2", "node_3"},
                   {{"L", "0.0001"}, {"esr", esr}, {"iL0", "0"}}),
                mk("C1",      ComponentType::Capacitor,     {"node_3", "node_0"},
                   {{"C", "0.0001"}, {"esr", esr}, {"vC0", "0"}}),
                mk("R1",      ComponentType::Resistor,      {"node_3", "node_0"}, {{"value", "10"}}),
            };
            std::vector<ComponentModel> ctrl = {
                mk("PULSE_GEN1", ComponentType::PulseGenerator, {},
                   {{"period", "1e-5"}, {"width", "0.5"}, {"amplitude", "1"}, {"delay", "0"}}),
            };

            SimulationConfig cfg;
            cfg.stopTime = 0.002; cfg.stepSize = dt;
            cfg.solver = solver; cfg.step_type = "fixed";

            CircuitSimulator sim;
            sim.setup(phys, ctrl, cfg);
            sim.reset();
            SimulationOutput o = sim.runTransient();
            printf("esr=%-6s dt=%-8.g solver=%-12s digest=%016llx samples=%zu\n",
                   esr, dt, solver, (unsigned long long)digest(o), o.time.size());
        }
    }
    return 0;
}
