// Performance harness for the closed-loop buck converter from the user's test JSON.
//
//   V1(24V) -- MOSFET1 -- L1 -- node_3 -- C1 || R1 -- gnd
//                 |
//                D1 (freewheel)
//
//   Control: CONST(5) - VM1  ->  SUBTRACT -> PID -> COMP vs TRI(100 kHz) -> MOSFET gate
//
// Reports wall time for setup and for the transient run, plus derived per-step cost.

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
    ComponentModel c;
    c.id = id;
    c.type = t;
    c.nodes = nodes;
    for (const auto& p : params) c.parameters[p.first] = p.second;
    return c;
}

static void buildBuck(std::vector<ComponentModel>& phys, std::vector<ComponentModel>& ctrl) {
    phys = {
        mk("V1",      ComponentType::VoltageSource, {"node_1", "0"},           {{"value", "24"}}),
        mk("MOSFET1", ComponentType::MOSFET,        {"node_1", "node_2"},
           {{"Ron", "10m"}, {"Roff", "1M"}, {"control_signal", "COMP1.Out"}}),
        mk("D1",      ComponentType::Diode,         {"0", "node_2"},
           {{"Vd", "0.7"}, {"Ron", "1m"}, {"Roff", "1M"}}),
        mk("L1",      ComponentType::Inductor,      {"node_2", "node_3"},
           {{"L", "100u"}, {"esr", "50m"}, {"iL0", "0"}}),
        mk("C1",      ComponentType::Capacitor,     {"node_3", "0"},
           {{"C", "100u"}, {"esr", "10m"}, {"vC0", "0"}}),
        mk("R1",      ComponentType::Resistor,      {"node_3", "0"},           {{"value", "10"}}),
        mk("VM1",     ComponentType::Voltmeter,     {"node_3", "0"},           {}),
    };

    ctrl = {
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
}

struct RunStats {
    double setupMs;
    double runMs;
    int    records;
    size_t signalCount;
};

static RunStats runOnce(double tStop, double dt, const char* solver, const char* stepType) {
    std::vector<ComponentModel> phys, ctrl;
    buildBuck(phys, ctrl);

    SimulationConfig cfg;
    cfg.stopTime  = tStop;
    cfg.stepSize  = dt;
    cfg.solver    = solver;
    cfg.step_type = stepType;

    CircuitSimulator sim;

    auto t0 = Clock::now();
    sim.setup(phys, ctrl, cfg);
    auto t1 = Clock::now();
    SimulationOutput out = sim.runTransient();
    auto t2 = Clock::now();

    RunStats s;
    s.setupMs = std::chrono::duration<double, std::milli>(t1 - t0).count();
    s.runMs   = std::chrono::duration<double, std::milli>(t2 - t1).count();
    s.records = (int)out.time.size();
    s.signalCount = out.voltages.size() + out.signals.size() + out.custom_plots.size()
                  + out.inductors.size() + out.voltmeters.size() + out.ammeters.size();
    return s;
}

int main() {
    printf("Closed-loop buck converter (100 kHz carrier) - solver performance\n");
    printf("=================================================================\n\n");

    struct Case { const char* name; double tStop; double dt; };
    const Case cases[] = {
        { "2 ms  @ 1us  (2k steps)",    2e-3,  1e-6 },
        { "20 ms @ 1us  (20k steps)",   20e-3, 1e-6 },   // the tool's default
        { "20 ms @ 500ns (40k steps)",  20e-3, 5e-7 },
        { "100 ms @ 1us (100k steps)",  100e-3,1e-6 },
    };

    printf("%-28s %10s %12s %10s %14s %10s\n",
           "case", "setup ms", "run ms", "records", "us/step", "signals");
    printf("-----------------------------------------------------------------------------------------\n");

    for (const auto& c : cases) {
        RunStats s = runOnce(c.tStop, c.dt, "euler", "fixed");
        double usPerStep = (s.records > 0) ? (s.runMs * 1000.0 / s.records) : 0.0;
        printf("%-28s %10.1f %12.1f %10d %14.2f %10zu\n",
               c.name, s.setupMs, s.runMs, s.records, usPerStep, s.signalCount);
        fflush(stdout);
    }

    printf("\nReal-time factor for the 20 ms default case:\n");
    {
        RunStats s = runOnce(20e-3, 1e-6, "euler", "fixed");
        printf("  simulated 20 ms of circuit time in %.1f ms wall time  ->  %.2fx %s than real time\n",
               s.runMs, (s.runMs / 20.0), (s.runMs > 20.0) ? "SLOWER" : "faster");
    }
    return 0;
}
