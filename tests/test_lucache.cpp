// LU factorization cache equivalence test.
//
// The cache must be a pure performance change: for the same circuit, running with
// the cache on and off has to produce bitwise identical output. This compares full
// output digests across circuits, solvers and step sizes, and reports the hit rate
// so a regression in cache effectiveness is visible too.

#include "engine/CircuitSimulator.hpp"
#include <algorithm>
#include <cstdio>
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

static uint64_t digest(const SimulationOutput& o) {
    uint64_t h = 1469598103934665603ULL;
    auto feed = [&](double d) {
        uint64_t b;
        std::memcpy(&b, &d, sizeof(b));
        h ^= b; h *= 1099511628211ULL; h ^= h >> 29;
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

// M independent buck stages, all switched by one comparator. M scales the matrix.
static void buildStages(int M, bool variableLoad,
                        std::vector<ComponentModel>& phys,
                        std::vector<ComponentModel>& ctrl) {
    phys.clear(); ctrl.clear();
    phys.push_back(mk("V1", ComponentType::VoltageSource, {"node_in", "0"}, {{"value", "24"}}));
    for (int i = 0; i < M; ++i) {
        std::string s = std::to_string(i), sw = "sw" + s, ou = "ou" + s;
        phys.push_back(mk("M" + s, ComponentType::MOSFET, {"node_in", sw},
            {{"Ron", "10m"}, {"Roff", "1M"}, {"control_signal", "COMP1.Out"}}));
        phys.push_back(mk("D" + s, ComponentType::Diode, {"0", sw},
            {{"Vd", "0.7"}, {"Ron", "1m"}, {"Roff", "1M"}}));
        phys.push_back(mk("L" + s, ComponentType::Inductor, {sw, ou},
            {{"L", "100u"}, {"esr", "50m"}}));
        phys.push_back(mk("C" + s, ComponentType::Capacitor, {ou, "0"},
            {{"C", "100u"}, {"esr", "10m"}}));
        if (variableLoad && i == 0) {
            // A signal-controlled resistor (VAR_R) makes K change on almost every
            // step, which is the worst case for the cache: near-zero hit rate, so
            // this is where a correctness bug or a slowdown would show up.
            phys.push_back(mk("R" + s, ComponentType::Resistor, {ou, "0"},
                {{"value", "10"}, {"src_type", "variable"}, {"control_signal", "TRI1.Out"}}));
        } else {
            phys.push_back(mk("R" + s, ComponentType::Resistor, {ou, "0"}, {{"value", "10"}}));
        }
    }
    ctrl = {
        mk("CONST1", ComponentType::Constant,         {}, {{"value", "0.5"}}),
        mk("TRI1",   ComponentType::Triangle_Carrier, {}, {{"frequency", "100k"}, {"min", "0"}, {"max", "1"}}),
        mk("COMP1",  ComponentType::Comparator,       {}, {{"In1", "CONST1.Out"}, {"In2", "TRI1.Out"}}),
    };
}

struct Result { uint64_t dig; long long fact; long long hits; size_t entries; double ms; };

static Result run(int M, bool variableLoad, const char* solver, double dt, bool cache) {
    std::vector<ComponentModel> phys, ctrl;
    buildStages(M, variableLoad, phys, ctrl);

    SimulationConfig cfg;
    // Long enough that wall-clock noise does not dominate the comparison; the
    // smallest cases finish in a couple of ms at 1 ms of simulated time.
    cfg.stopTime = 5e-3; cfg.stepSize = dt;
    cfg.solver = solver; cfg.step_type = "fixed";
    cfg.enableLUCache = cache;

    CircuitSimulator sim;
    sim.setup(phys, ctrl, cfg);
    sim.reset();
    auto t0 = std::chrono::high_resolution_clock::now();
    SimulationOutput o = sim.runTransient();
    auto t1 = std::chrono::high_resolution_clock::now();

    Result r;
    r.dig = digest(o);
    r.fact = sim.getFactorizeCount();
    r.hits = sim.getLUCacheHitCount();
    r.entries = sim.getLUCacheEntryCount();
    r.ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    return r;
}

int main() {
    int pass = 0, fail = 0;
    auto check = [&](const std::string& what, bool ok) {
        printf("  %-58s %s\n", what.c_str(), ok ? "PASS" : "FAIL");
        if (ok) ++pass; else ++fail;
    };

    printf("LU factorization cache: equivalence and effectiveness\n");
    printf("=====================================================\n\n");

    struct Case { int M; bool varLoad; const char* solver; double dt; const char* name; };
    const Case cases[] = {
        { 1,  false, "euler",       1e-6, "1 stage,  euler"       },
        { 1,  false, "trapezoidal", 1e-6, "1 stage,  trapezoidal" },
        { 1,  false, "rk4",         1e-6, "1 stage,  rk4"         },
        { 4,  false, "euler",       1e-6, "4 stages, euler"       },
        { 8,  false, "trapezoidal", 1e-6, "8 stages, trapezoidal" },
        { 16, false, "euler",       1e-6, "16 stages, euler"      },
        { 32, false, "euler",       1e-6, "32 stages, euler"      },
        { 4,  false, "euler",       1e-7, "4 stages, euler, dt=100ns" },
        { 4,  true,  "euler",       1e-6, "4 stages, VARIABLE load (worst case)" },
        // Large and thrashing at the same time: this is the only regime where the
        // factorization itself still dominates, so it is the case that would justify
        // a sparse factorization or a low-rank update.
        { 32, true,  "euler",       1e-6, "32 stages, VARIABLE load (large+thrash)" },
    };

    printf("Bitwise equivalence, cache on vs cache off:\n");
    std::vector<Result> on, off;
    for (const auto& c : cases) {
        // Best of 3 for the timing: the small cases run in a couple of milliseconds,
        // where single-shot wall time is dominated by scheduling noise.
        Result a = run(c.M, c.varLoad, c.solver, c.dt, false);
        Result b = run(c.M, c.varLoad, c.solver, c.dt, true);
        for (int rep = 1; rep < 3; ++rep) {
            Result a2 = run(c.M, c.varLoad, c.solver, c.dt, false);
            Result b2 = run(c.M, c.varLoad, c.solver, c.dt, true);
            if (a2.ms < a.ms) a.ms = a2.ms;
            if (b2.ms < b.ms) b.ms = b2.ms;
            if (a2.dig != a.dig || b2.dig != b.dig) { a.dig = 1; b.dig = 2; }  // run-to-run instability
        }
        off.push_back(a); on.push_back(b);
        check(std::string(c.name), a.dig == b.dig);
    }

    printf("\nEffectiveness (cache enabled):\n");
    printf("  %-38s %10s %10s %8s %9s\n", "case", "factorize", "hits", "entries", "hit rate");
    for (size_t i = 0; i < on.size(); ++i) {
        long long total = on[i].fact + on[i].hits;
        double rate = total ? (100.0 * (double)on[i].hits / (double)total) : 0.0;
        printf("  %-38s %10lld %10lld %8zu %8.1f%%\n",
               cases[i].name, on[i].fact, on[i].hits, on[i].entries, rate);
    }

    printf("\nSpeed (same circuit, cache off -> on):\n");
    printf("  %-38s %10s %10s %8s\n", "case", "off ms", "on ms", "gain");
    for (size_t i = 0; i < on.size(); ++i) {
        printf("  %-38s %10.1f %10.1f %7.2fx\n",
               cases[i].name, off[i].ms, on[i].ms,
               on[i].ms > 0 ? off[i].ms / on[i].ms : 0.0);
    }

    // A switching converter with fixed passives must hit nearly every time: the
    // only misses should be the handful of distinct topologies.
    printf("\n");
    for (size_t i = 0; i < on.size(); ++i) {
        if (cases[i].varLoad) continue;
        long long total = on[i].fact + on[i].hits;
        double rate = total ? (100.0 * (double)on[i].hits / (double)total) : 0.0;
        check(std::string(cases[i].name) + ": hit rate > 90%", rate > 90.0);
    }
    // The continuously varying case must be recognised as a near-total miss. Note
    // this is asserted on counters rather than wall time: a timing assertion here
    // flaked under background load, and a flaky test is worse than no test. The
    // measured times are reported above for inspection instead.
    for (size_t i = 0; i < on.size(); ++i) {
        if (!cases[i].varLoad) continue;
        long long total = on[i].fact + on[i].hits;
        double rate = total ? (100.0 * (double)on[i].hits / (double)total) : 0.0;
        check("variable load really does miss (hit rate < 25%)", rate < 25.0);
        // The adaptive bail-out should stop the cache growing without bound and
        // stop paying for lookups: entries stay at the cap, hits stay negligible.
        check("variable load: cache stays within its entry cap", on[i].entries <= 64);
        check("variable load: bail-out keeps hits from dominating", on[i].hits < on[i].fact / 10);
    }
    // The cache must never make the factorization count grow.
    for (size_t i = 0; i < on.size(); ++i) {
        check(std::string(cases[i].name) + ": factorizations not increased",
              on[i].fact <= off[i].fact);
    }

    printf("\n------------------------------------------------------------\n");
    printf("%d/%d checks passed\n", pass, pass + fail);
    return fail == 0 ? 0 : 1;
}
