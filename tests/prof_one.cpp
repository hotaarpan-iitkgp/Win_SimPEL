// Single-case profile driver: runs only the 20 ms @ 1 us default case so the phase
// breakdown printed by the solver's temporary instrumentation is unambiguous.
#include "engine/CircuitSimulator.hpp"
#include <chrono>
#include <cstdio>
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

int main(int argc, char** argv) {
    double tStop = (argc > 1) ? atof(argv[1]) : 20e-3;
    double dt    = (argc > 2) ? atof(argv[2]) : 1e-6;

    std::vector<ComponentModel> phys = {
        mk("V1",      ComponentType::VoltageSource, {"node_1", "0"},      {{"value", "24"}}),
        mk("MOSFET1", ComponentType::MOSFET,        {"node_1", "node_2"},
           {{"Ron", "10m"}, {"Roff", "1M"}, {"control_signal", "COMP1.Out"}}),
        mk("D1",      ComponentType::Diode,         {"0", "node_2"},
           {{"Vd", "0.7"}, {"Ron", "1m"}, {"Roff", "1M"}}),
        mk("L1",      ComponentType::Inductor,      {"node_2", "node_3"},
           {{"L", "100u"}, {"esr", "50m"}, {"iL0", "0"}}),
        mk("C1",      ComponentType::Capacitor,     {"node_3", "0"},
           {{"C", "100u"}, {"esr", "10m"}, {"vC0", "0"}}),
        mk("R1",      ComponentType::Resistor,      {"node_3", "0"},      {{"value", "10"}}),
        mk("VM1",     ComponentType::Voltmeter,     {"node_3", "0"},      {}),
    };
    std::vector<ComponentModel> ctrl = {
        mk("CONST1",    ComponentType::Constant,        {}, {{"value", "5"}}),
        mk("SUBTRACT1", ComponentType::SummingJunction, {},
           {{"signs", "+-"}, {"In1", "CONST1.Out"}, {"In2", "VM1.Out"}}),
        mk("PID1",      ComponentType::PI_Controller,   {},
           {{"Kp", "0.1"}, {"Ki", "60"}, {"Kd", "0"}, {"In", "SUBTRACT1.Out"},
            {"limit_output", "true"}, {"lower_limit", "0.01"}, {"upper_limit", "0.9"},
            {"anti_windup", "true"}}),
        mk("TRI1",      ComponentType::Triangle_Carrier, {},
           {{"frequency", "100k"}, {"min", "0"}, {"max", "1"}}),
        mk("COMP1",     ComponentType::Comparator,      {},
           {{"In1", "PID1.Out"}, {"In2", "TRI1.Out"}, {"hysteresis", "0"}}),
    };

    SimulationConfig cfg;
    cfg.stopTime = tStop; cfg.stepSize = dt; cfg.solver = "euler"; cfg.step_type = "fixed";

    CircuitSimulator sim;
    sim.setup(phys, ctrl, cfg);
    auto t0 = std::chrono::high_resolution_clock::now();
    SimulationOutput out = sim.runTransient();
    auto t1 = std::chrono::high_resolution_clock::now();

    printf("tStop=%g dt=%g  records=%zu  wall=%.1f ms\n", tStop, dt, out.time.size(),
           std::chrono::duration<double, std::milli>(t1 - t0).count());
    return 0;
}
