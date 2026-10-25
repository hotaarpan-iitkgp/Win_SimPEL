// Verifies the incremental telemetry path is lossless:
//   1. Live-published telemetry matches the authoritative SimulationOutput exactly.
//   2. syncTelemetryInto() called repeatedly mid-run reconstructs the identical
//      series that a single full getTelemetryCopy() would have produced.
//   3. A reset mid-consumption re-syncs cleanly rather than corrupting the cache.

#include "engine/CircuitSimulator.hpp"
#include <cstdio>
#include <cmath>
#include <string>
#include <thread>
#include <vector>

using namespace CircuitSimEngine;

static ComponentModel mk(const std::string& id, ComponentType t,
                         const std::vector<std::string>& nodes,
                         const std::vector<std::pair<std::string, std::string>>& params) {
    ComponentModel c; c.id = id; c.type = t; c.nodes = nodes;
    for (const auto& p : params) c.parameters[p.first] = p.second;
    return c;
}

static int checks = 0, failures = 0;
static void expect(const char* what, bool ok, const char* detail = "") {
    checks++;
    if (!ok) failures++;
    printf("  %-58s %s%s%s\n", what, ok ? "PASS" : "FAIL",
           (!ok && *detail) ? "  " : "", (!ok) ? detail : "");
}

int main() {
    std::vector<ComponentModel> phys = {
        mk("V1",      ComponentType::VoltageSource, {"node_1", "0"},      {{"value", "24"}}),
        mk("MOSFET1", ComponentType::MOSFET,        {"node_1", "node_2"},
           {{"Ron", "10m"}, {"Roff", "1M"}, {"control_signal", "COMP1.Out"}}),
        mk("D1",      ComponentType::Diode,         {"0", "node_2"},
           {{"Vd", "0.7"}, {"Ron", "1m"}, {"Roff", "1M"}}),
        mk("L1",      ComponentType::Inductor,      {"node_2", "node_3"}, {{"L", "100u"}, {"esr", "50m"}}),
        mk("C1",      ComponentType::Capacitor,     {"node_3", "0"},      {{"C", "100u"}, {"esr", "10m"}}),
        mk("R1",      ComponentType::Resistor,      {"node_3", "0"},      {{"value", "10"}}),
        mk("VM1",     ComponentType::Voltmeter,     {"node_3", "0"},      {}),
    };
    std::vector<ComponentModel> ctrl = {
        mk("CONST1", ComponentType::Constant,         {}, {{"value", "0.4"}}),
        mk("TRI1",   ComponentType::Triangle_Carrier, {}, {{"frequency", "100k"}, {"min", "0"}, {"max", "1"}}),
        mk("COMP1",  ComponentType::Comparator,       {}, {{"In1", "CONST1.Out"}, {"In2", "TRI1.Out"}}),
    };

    SimulationConfig cfg;
    cfg.stopTime = 20e-3; cfg.stepSize = 1e-6; cfg.solver = "euler"; cfg.step_type = "fixed";

    printf("Incremental telemetry correctness\n");
    printf("=================================\n\n");

    // ---- 1. live-published telemetry vs the authoritative output ----
    CircuitSimulator sim;
    sim.setup(phys, ctrl, cfg);
    SimulationOutput out = sim.runTransient();

    // Snapshot the live telemetry *before* setTelemetryOutput overwrites it.
    TelemetryData live = sim.getTelemetryCopy();

    printf("A) live publish tracks the authoritative output\n");
    expect("live sample count <= output count", live.timeHistory.size() <= out.time.size());
    bool prefixOk = true;
    for (size_t i = 0; i < live.timeHistory.size(); ++i) {
        if (live.timeHistory[i] != out.time[i]) { prefixOk = false; break; }
    }
    expect("live time series is an exact prefix of out.time", prefixOk);

    bool sigPrefixOk = true;
    std::string badKey;
    for (const auto& kv : live.voltages) {
        const std::vector<double>* src = nullptr;
        if (out.voltages.count(kv.first))          src = &out.voltages.at(kv.first);
        else if (out.signals.count(kv.first))      src = &out.signals.at(kv.first);
        else if (out.custom_plots.count(kv.first)) src = &out.custom_plots.at(kv.first);
        else if (out.inductors.count(kv.first))    src = &out.inductors.at(kv.first);
        else if (out.voltmeters.count(kv.first))   src = &out.voltmeters.at(kv.first);
        else if (out.ammeters.count(kv.first))     src = &out.ammeters.at(kv.first);
        if (!src) { sigPrefixOk = false; badKey = kv.first + " (missing)"; break; }
        for (size_t i = 0; i < kv.second.size() && i < src->size(); ++i) {
            if (kv.second[i] != (*src)[i]) { sigPrefixOk = false; badKey = kv.first; break; }
        }
        if (!sigPrefixOk) break;
    }
    expect("every live signal is an exact prefix of its output series", sigPrefixOk, badKey.c_str());

    // ---- 2. incremental sync reproduces the full copy exactly ----
    printf("\nB) syncTelemetryInto() is lossless\n");
    sim.setTelemetryOutput(out);          // final full-resolution publish
    TelemetryData whole = sim.getTelemetryCopy();

    // Consume in several chunks, as the UI would across frames.
    TelemetryData inc;
    size_t have = 0;
    uint64_t gen = sim.getTelemetryGeneration();
    for (int round = 0; round < 5; ++round) {
        have = sim.syncTelemetryInto(inc, have, gen);
    }

    expect("time series length matches full copy",
           inc.timeHistory.size() == whole.timeHistory.size());
    expect("signal count matches full copy",
           inc.voltages.size() == whole.voltages.size());

    bool identical = (inc.timeHistory == whole.timeHistory);
    expect("time series bitwise identical to full copy", identical);

    bool allSigIdentical = true;
    std::string mismatch;
    for (const auto& kv : whole.voltages) {
        auto it = inc.voltages.find(kv.first);
        if (it == inc.voltages.end()) { allSigIdentical = false; mismatch = kv.first + " (absent)"; break; }
        if (it->second != kv.second)  { allSigIdentical = false; mismatch = kv.first; break; }
    }
    expect("all signals bitwise identical to full copy", allSigIdentical, mismatch.c_str());

    // ---- 3. reset mid-consumption recovers ----
    printf("\nC) reset() mid-consumption re-syncs cleanly\n");
    sim.reset();
    size_t afterReset = sim.syncTelemetryInto(inc, have, gen);
    expect("consumer count drops to 0 after reset", afterReset == 0);
    expect("stale cached samples were discarded", inc.timeHistory.empty());
    expect("reset() advanced the telemetry generation", gen == sim.getTelemetryGeneration());

    printf("\n------------------------------------------------------------\n");
    printf("%d/%d checks passed\n", checks - failures, checks);
    return failures == 0 ? 0 : 1;
}
