// Re-run integrity test.
//
// The GUI keeps ONE CircuitSimulator as a MainWindow member and calls
// setup() -> reset() -> runTransient() again for every Run. This test reproduces
// that exact pattern and asserts that a run on a reused simulator is bitwise
// identical to the same run on a freshly constructed one, including when the
// step size changes between runs.
//
// It also reports steady-state output voltage / ripple for the user's buck
// netlist at 1e-6 and 1e-8 so the physics can be sanity-checked independently
// of the plotting path.

#include "engine/CircuitSimulator.hpp"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <functional>
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

// The user's netlist: 100 V in, open-loop PULSE_GEN gate, 50% duty.
static void buildCase(double pulsePeriod,
                      std::vector<ComponentModel>& phys,
                      std::vector<ComponentModel>& ctrl) {
    char per[64];
    snprintf(per, sizeof(per), "%.17g", pulsePeriod);

    phys = {
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
    ctrl = {
        mk("PULSE_GEN1", ComponentType::PulseGenerator, {},
           {{"period", per}, {"width", "0.5"}, {"amplitude", "1"}, {"delay", "0"}}),
    };
}

static SimulationConfig cfgFor(double dt) {
    SimulationConfig c;
    c.stopTime = 0.01;
    c.stepSize = dt;
    c.solver = "euler";
    c.step_type = "fixed";
    return c;
}

// Flatten an output into one comparable stream so "identical" means identical.
static std::vector<double> flatten(const SimulationOutput& o) {
    std::vector<double> v = o.time;
    auto add = [&](const std::unordered_map<std::string, std::vector<double>>& m) {
        std::vector<std::string> keys;
        for (const auto& kv : m) keys.push_back(kv.first);
        std::sort(keys.begin(), keys.end());
        for (const auto& k : keys) {
            v.push_back((double)std::hash<std::string>{}(k));
            const auto& s = m.at(k);
            v.insert(v.end(), s.begin(), s.end());
        }
    };
    add(o.voltages); add(o.signals); add(o.inductors);
    add(o.voltmeters); add(o.ammeters); add(o.custom_plots);
    return v;
}

static bool bitwiseEqual(const std::vector<double>& a, const std::vector<double>& b) {
    if (a.size() != b.size()) return false;
    for (size_t i = 0; i < a.size(); ++i) {
        if (std::memcmp(&a[i], &b[i], sizeof(double)) != 0) return false;
    }
    return true;
}

struct Stats { double mean; double ripple; double last; size_t n; };

// Steady-state statistics over the final 10% of the run.
static Stats statsOf(const SimulationOutput& o, const std::string& key) {
    Stats s{0, 0, 0, 0};
    const std::vector<double>* p = nullptr;
    auto find = [&](const std::unordered_map<std::string, std::vector<double>>& m) {
        auto it = m.find(key);
        if (it != m.end()) p = &it->second;
    };
    find(o.custom_plots); if (!p) find(o.voltages); if (!p) find(o.signals);
    if (!p || p->empty()) return s;
    size_t n = p->size();
    size_t from = n - n / 10;
    double mn = 1e300, mx = -1e300, sum = 0;
    for (size_t i = from; i < n; ++i) {
        double v = (*p)[i];
        sum += v; if (v < mn) mn = v; if (v > mx) mx = v;
    }
    s.n = n; s.mean = sum / (double)(n - from); s.ripple = mx - mn; s.last = p->back();
    return s;
}

int main() {
    int pass = 0, fail = 0;
    auto check = [&](const char* what, bool ok) {
        printf("  %-62s %s\n", what, ok ? "PASS" : "FAIL");
        if (ok) ++pass; else ++fail;
    };

    printf("Re-run integrity (simulator reused across runs, as the GUI does)\n");
    printf("================================================================\n\n");

    std::vector<ComponentModel> phys, ctrl;

    // --- Reference outputs, each from a pristine simulator -------------------
    buildCase(1e-5, phys, ctrl);
    SimulationOutput refA;   // period 1e-5, dt 1e-6
    { CircuitSimulator s; s.setup(phys, ctrl, cfgFor(1e-6)); s.reset(); refA = s.runTransient(); }

    SimulationOutput refB;   // period 1e-5, dt 1e-7
    { CircuitSimulator s; s.setup(phys, ctrl, cfgFor(1e-7)); s.reset(); refB = s.runTransient(); }

    std::vector<ComponentModel> phys2, ctrl2;
    buildCase(1e-4, phys2, ctrl2);
    SimulationOutput refC;   // period 1e-4, dt 1e-6
    { CircuitSimulator s; s.setup(phys2, ctrl2, cfgFor(1e-6)); s.reset(); refC = s.runTransient(); }

    // --- Same simulator, four consecutive runs with changing step size -------
    CircuitSimulator sim;
    SimulationOutput r1, r2, r3, r4;

    sim.setup(phys, ctrl, cfgFor(1e-6));  sim.reset();  r1 = sim.runTransient();
    sim.setTelemetryOutput(r1);
    sim.setup(phys, ctrl, cfgFor(1e-7));  sim.reset();  r2 = sim.runTransient();
    sim.setTelemetryOutput(r2);
    sim.setup(phys2, ctrl2, cfgFor(1e-6)); sim.reset(); r3 = sim.runTransient();
    sim.setTelemetryOutput(r3);
    sim.setup(phys, ctrl, cfgFor(1e-6));  sim.reset();  r4 = sim.runTransient();
    sim.setTelemetryOutput(r4);

    printf("Reused simulator vs fresh simulator (bitwise):\n");
    check("run 1  dt=1e-6  period=1e-5", bitwiseEqual(flatten(r1), flatten(refA)));
    check("run 2  dt=1e-7  period=1e-5  (step size changed)", bitwiseEqual(flatten(r2), flatten(refB)));
    check("run 3  dt=1e-6  period=1e-4  (circuit + step changed)", bitwiseEqual(flatten(r3), flatten(refC)));
    check("run 4  dt=1e-6  period=1e-5  (back to run 1 settings)", bitwiseEqual(flatten(r4), flatten(refA)));

    // --- Telemetry generation / cache invalidation ---------------------------
    printf("\nTelemetry cache invalidation:\n");
    {
        CircuitSimulator s;
        s.setup(phys, ctrl, cfgFor(1e-6)); s.reset();
        SimulationOutput o = s.runTransient();
        s.setTelemetryOutput(o);

        TelemetryData view;              // stand-in for a scope window's cache
        uint64_t gen = (uint64_t)-1;
        size_t have = s.syncTelemetryInto(view, 0, gen);
        check("first sync mirrors the whole history",
              have == o.time.size() && view.timeHistory.size() == o.time.size());

        // Second run with a SMALLER step: more samples than the cached run.
        uint64_t genBefore = s.getTelemetryGeneration();
        s.setup(phys, ctrl, cfgFor(1e-7));
        s.reset();
        check("reset() starts a new telemetry generation",
              s.getTelemetryGeneration() != genBefore);

        SimulationOutput o2 = s.runTransient();
        s.setTelemetryOutput(o2);
        check("run 2 really does produce more samples than run 1",
              o2.time.size() > o.time.size());

        have = s.syncTelemetryInto(view, have, gen);
        check("stale cache discarded, not appended to",
              view.timeHistory.size() == o2.time.size() && have == o2.time.size());
        check("cached curve matches run 2 from its first sample",
              !view.timeHistory.empty() && view.timeHistory[0] == o2.time[0]);

        // Equal-length case: same stop time and step, different component values.
        std::vector<ComponentModel> physAlt = phys;
        for (auto& c : physAlt) if (c.id == "R1") c.parameters["value"] = "5";
        s.setup(physAlt, ctrl, cfgFor(1e-7));
        s.reset();
        SimulationOutput o3 = s.runTransient();
        s.setTelemetryOutput(o3);
        have = s.syncTelemetryInto(view, have, gen);
        bool sameLen = (o3.time.size() == o2.time.size());
        const std::vector<double>& vc3 = o3.custom_plots.at("V_C1");
        bool viewHasRun3 = view.voltages.count("V_C1") &&
                           view.voltages.at("V_C1").size() == vc3.size() &&
                           view.voltages.at("V_C1").back() == vc3.back();
        check("equal sample count across runs still refreshes the plot",
              sameLen && viewHasRun3);
    }

    // --- Physics sanity: step size must not change the answer much -----------
    printf("\nPhysics: open-loop buck, 100 V in, 50%% duty, R=10 ohm\n");
    printf("  %-34s %10s %10s %10s\n", "case", "Vout mean", "ripple", "samples");
    struct Row { const char* name; double period; double dt; };
    const Row rows[] = {
        { "period 1e-4 (10 kHz), dt 1e-6",  1e-4, 1e-6 },
        { "period 1e-4 (10 kHz), dt 1e-7",  1e-4, 1e-7 },
        { "period 1e-4 (10 kHz), dt 1e-8",  1e-4, 1e-8 },
        { "period 1e-5 (100 kHz), dt 1e-6", 1e-5, 1e-6 },
        { "period 1e-5 (100 kHz), dt 1e-7", 1e-5, 1e-7 },
    };
    std::vector<Stats> got;
    for (const auto& r : rows) {
        std::vector<ComponentModel> p, c;
        buildCase(r.period, p, c);
        CircuitSimulator s;
        s.setup(p, c, cfgFor(r.dt)); s.reset();
        SimulationOutput o = s.runTransient();
        Stats st = statsOf(o, "V_C1");
        got.push_back(st);
        printf("  %-34s %10.3f %10.4f %10zu\n", r.name, st.mean, st.ripple, st.n);
        fflush(stdout);
    }

    printf("\n");
    // Analytical expectations for 50%% duty on a 100 V rail, R = 10 ohm:
    //
    //  10 kHz: L_crit = (1-D)R/(2f) = 250 uH > L = 100 uH  ->  DCM.
    //          K = 2Lf/R = 0.2, M = 2/(1 + sqrt(1 + 4K/D^2)) = 0.656  ->  65.6 V.
    //  100 kHz: L_crit = 25 uH < 100 uH  ->  CCM  ->  M = D  ->  50 V,
    //          less the diode drop and the L/C ESR losses.
    const double expectedDCM = 65.6;
    const double expectedCCM = 50.0;
    for (size_t i = 0; i < 3; ++i) {
        char buf[128];
        snprintf(buf, sizeof(buf), "10 kHz case %zu mean Vout ~ %.1f V (DCM)", i + 1, expectedDCM);
        check(buf, std::fabs(got[i].mean - expectedDCM) < 2.5);
    }
    for (size_t i = 3; i < 5; ++i) {
        char buf[128];
        snprintf(buf, sizeof(buf), "100 kHz case %zu mean Vout ~ %.1f V (CCM)", i + 1, expectedCCM);
        check(buf, std::fabs(got[i].mean - expectedCCM) < 2.5);
    }

    // The whole point: refining the step must converge, not wander.
    check("dt 1e-6 -> 1e-7 changes mean Vout by < 1 V (10 kHz)",
          std::fabs(got[0].mean - got[1].mean) < 1.0);
    check("dt 1e-7 -> 1e-8 changes mean Vout by < 1 V (10 kHz)",
          std::fabs(got[1].mean - got[2].mean) < 1.0);
    check("dt 1e-6 -> 1e-7 changes mean Vout by < 1 V (100 kHz)",
          std::fabs(got[3].mean - got[4].mean) < 1.0);
    check("ripple does not grow when the step shrinks (10 kHz)",
          got[2].ripple < got[0].ripple * 1.5);

    // Truncation guard: every run must reach the requested stop time.
    for (size_t i = 0; i < got.size(); ++i) {
        char buf[128];
        snprintf(buf, sizeof(buf), "case %zu ran to completion (%zu samples)", i + 1, got[i].n);
        check(buf, got[i].n == (size_t)llround(std::ceil(0.01 / rows[i].dt)));
    }

    printf("\n------------------------------------------------------------\n");
    printf("%d/%d checks passed\n", pass, pass + fail);
    return fail == 0 ? 0 : 1;
}
