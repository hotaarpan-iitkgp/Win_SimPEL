// Scaling harness: M independent buck stages sharing one PWM comparator.
// Matrix dimension grows ~linearly with M, and every stage switches every cycle, so
// this isolates how the numerical core (assemble / factorize / solve) scales with
// circuit size and with switching-event density.

#include "engine/CircuitSimulator.hpp"
#include <chrono>
#include <cstdio>
#include <string>
#include <vector>

using namespace CircuitSimEngine;
using Clock = std::chrono::high_resolution_clock;

static ComponentModel mk(const std::string& id, ComponentType t,
                         const std::vector<std::string>& nodes,
                         const std::vector<std::pair<std::string, std::string>>& params) {
    ComponentModel c; c.id = id; c.type = t; c.nodes = nodes;
    for (const auto& p : params) c.parameters[p.first] = p.second;
    return c;
}

int main() {
    printf("Scaling: M buck stages, 2 ms @ 1 us (2000 steps), shared 100 kHz PWM\n");
    printf("=====================================================================\n\n");
    printf("%6s %10s %12s %12s %14s\n", "stages", "extraDim", "run ms", "us/step", "us/step/stage");
    printf("---------------------------------------------------------------------\n");

    for (int M : {1, 2, 4, 8, 16, 32}) {
        std::vector<ComponentModel> phys, ctrl;

        phys.push_back(mk("V1", ComponentType::VoltageSource, {"node_in", "0"}, {{"value", "24"}}));

        for (int i = 0; i < M; ++i) {
            std::string s = std::to_string(i);
            std::string sw = "sw" + s, ou = "ou" + s;
            phys.push_back(mk("M" + s, ComponentType::MOSFET, {"node_in", sw},
                {{"Ron", "10m"}, {"Roff", "1M"}, {"control_signal", "COMP1.Out"}}));
            phys.push_back(mk("D" + s, ComponentType::Diode, {"0", sw},
                {{"Vd", "0.7"}, {"Ron", "1m"}, {"Roff", "1M"}}));
            phys.push_back(mk("L" + s, ComponentType::Inductor, {sw, ou},
                {{"L", "100u"}, {"esr", "50m"}}));
            phys.push_back(mk("C" + s, ComponentType::Capacitor, {ou, "0"},
                {{"C", "100u"}, {"esr", "10m"}}));
            phys.push_back(mk("R" + s, ComponentType::Resistor, {ou, "0"}, {{"value", "10"}}));
        }

        ctrl = {
            mk("CONST1", ComponentType::Constant,         {}, {{"value", "0.5"}}),
            mk("TRI1",   ComponentType::Triangle_Carrier, {}, {{"frequency", "100k"}, {"min", "0"}, {"max", "1"}}),
            mk("COMP1",  ComponentType::Comparator,       {}, {{"In1", "CONST1.Out"}, {"In2", "TRI1.Out"}}),
        };

        SimulationConfig cfg;
        cfg.stopTime = 2e-3; cfg.stepSize = 1e-6; cfg.solver = "euler"; cfg.step_type = "fixed";

        CircuitSimulator sim;
        sim.setup(phys, ctrl, cfg);
        auto t0 = Clock::now();
        SimulationOutput out = sim.runTransient();
        auto t1 = Clock::now();

        double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
        int steps = (int)out.time.size();
        double usStep = steps ? ms * 1000.0 / steps : 0.0;
        // nodes: node_in + M*(sw,ou) + gnd ; unknowns also include 1 vsource + M inductors
        int extraDim = 1 + 2 * M + 1 + M;
        printf("%6d %10d %12.1f %12.2f %14.3f\n", M, extraDim, ms, usStep, usStep / M);
        fflush(stdout);
    }
    return 0;
}
