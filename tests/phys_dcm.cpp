// Step-refinement convergence probe for the user's open-loop buck netlist.
// Compiles against both the current engine and the pre-Phase-1 baseline, so the
// same sweep can be run on each to tell a regression from a pre-existing issue.
//
// usage: phys_dcm <pulsePeriod> <dt> [stopTime]

#include "engine/CircuitSimulator.hpp"
#include <cmath>
#include <cstdio>
#include <cstdlib>
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
    double period = (argc > 1) ? atof(argv[1]) : 1e-4;
    double dt     = (argc > 2) ? atof(argv[2]) : 1e-6;
    double tStop  = (argc > 3) ? atof(argv[3]) : 0.01;

    char per[64];
    snprintf(per, sizeof(per), "%.17g", period);

    std::vector<ComponentModel> phys = {
        mk("V1",      ComponentType::VoltageSource, {"node_1", "node_0"}, {{"value", "100"}}),
        mk("MOSFET1", ComponentType::MOSFET,        {"node_1", "node_2"},
           {{"Ron", "0.01"}, {"Roff", "1000000"}, {"Vd", "0.8"}, {"Vgt", "0.5"},
            {"Iholding", "0.01"}, {"control_signal", "PULSE_GEN1.Out"}}),
        mk("D1",      ComponentType::Diode,         {"node_0", "node_2"},
           {{"Vd", "0.7"}, {"Ron", "0.001"}, {"Roff", "1000000"}}),
        mk("L1",      ComponentType::Inductor,      {"node_2", "node_3"},
           {{"L", "0.0001"}, {"esr", "0.05"}, {"iL0", "0"}}),
        mk("C1",      ComponentType::Capacitor,     {"node_3", "node_0"},
           {{"C", "0.0001"}, {"esr", "0.01"}, {"vC0", "0"}}),
        mk("R1",      ComponentType::Resistor,      {"node_3", "node_0"}, {{"value", "10"}}),
    };
    std::vector<ComponentModel> ctrl = {
        mk("PULSE_GEN1", ComponentType::PulseGenerator, {},
           {{"period", per}, {"width", "0.5"}, {"amplitude", "1"}, {"delay", "0"}}),
    };

    SimulationConfig cfg;
    cfg.stopTime = tStop; cfg.stepSize = dt; cfg.solver = "euler"; cfg.step_type = "fixed";

    CircuitSimulator sim;
    sim.setup(phys, ctrl, cfg);
    sim.reset();
    SimulationOutput o = sim.runTransient();

    auto stat = [&](const char* key, double& mean, double& ripple, double& mn, double& mx) {
        mean = ripple = 0; mn = 0; mx = 0;
        const std::vector<double>* p = nullptr;
        auto it = o.custom_plots.find(key);
        if (it != o.custom_plots.end()) p = &it->second;
        if (!p) { auto i2 = o.voltages.find(key); if (i2 != o.voltages.end()) p = &i2->second; }
        if (!p || p->empty()) return;
        size_t n = p->size(), from = n - n / 10;
        double lo = 1e300, hi = -1e300, sum = 0;
        for (size_t i = from; i < n; ++i) {
            double v = (*p)[i];
            sum += v; if (v < lo) lo = v; if (v > hi) hi = v;
        }
        mean = sum / (double)(n - from); ripple = hi - lo; mn = lo; mx = hi;
    };

    double vm, vr, vlo, vhi, im, ir, ilo, ihi;
    stat("V_C1", vm, vr, vlo, vhi);
    stat("I_L1", im, ir, ilo, ihi);

    printf("period=%.3g dt=%.3g samples=%zu tEnd=%.6g | Vout mean=%.4f ripple=%.4f min=%.3f max=%.3f | IL mean=%.4f min=%.3f max=%.3f\n",
           period, dt, o.time.size(), o.time.empty() ? 0.0 : o.time.back(),
           vm, vr, vlo, vhi, im, ilo, ihi);

    // Optional trace dump: phys_dcm <period> <dt> <tStop> <tFrom> <tTo> <stride>
    if (argc > 6) {
        double tFrom = atof(argv[4]), tTo = atof(argv[5]);
        long stride = atol(argv[6]);
        if (stride < 1) stride = 1;
        auto get = [&](const char* k) -> const std::vector<double>* {
            auto it = o.custom_plots.find(k);
            if (it != o.custom_plots.end()) return &it->second;
            auto i2 = o.voltages.find(k);
            if (i2 != o.voltages.end()) return &i2->second;
            auto i3 = o.signals.find(k);
            if (i3 != o.signals.end()) return &i3->second;
            return nullptr;
        };
        const std::vector<double>* gate = get("PULSE_GEN1.Out");
        const std::vector<double>* vc   = get("V_C1");
        const std::vector<double>* il   = get("I_L1");
        const std::vector<double>* n2   = get("node_2");
        const std::vector<double>* imos = get("I_MOSFET1");
        const std::vector<double>* id   = get("I_D1");
        printf("%14s %6s %12s %12s %12s %12s %12s\n",
               "t", "gate", "V_C1", "I_L1", "node_2", "I_MOSFET1", "I_D1");
        for (size_t i = 0; i < o.time.size(); i += (size_t)stride) {
            double t = o.time[i];
            if (t < tFrom || t > tTo) continue;
            auto at = [&](const std::vector<double>* v) -> double {
                return (v && i < v->size()) ? (*v)[i] : 0.0;
            };
            printf("%14.9f %6.2f %12.5f %12.5f %12.5f %12.5f %12.5f\n",
                   t, at(gate), at(vc), at(il), at(n2), at(imos), at(id));
        }
    }
    return 0;
}
