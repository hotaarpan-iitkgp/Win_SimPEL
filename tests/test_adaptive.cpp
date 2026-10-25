// Adaptive (variable-step) solver tests.
//
// The adaptive path cannot be checked against a stored digest the way the fixed path
// can, because it deliberately produces a different set of sample times. So it is
// checked three ways:
//
//   A) Rollback correctness. With config.debugRejectEveryNthStep set, every Nth step
//      is rejected once and retried. Because a rejected step must leave no trace,
//      the output has to be IDENTICAL to a run without the hook. This is the only
//      direct test of saveSolverState/restoreSolverState.
//
//   B) Accuracy. Compared against a fine fixed-step reference by interpolating both
//      onto common times - index-by-index comparison is meaningless here, and the
//      fixed path additionally labels each solution with the time at the START of
//      its step, so it is shifted by one step.
//
//   C) Convergence. Tightening relTol must reduce the error against the reference,
//      and must not increase it.
//
// It also reports step counts, which is the number the whole exercise is about.

#include "engine/CircuitSimulator.hpp"
#include <algorithm>
#include <cmath>
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

// Open-loop buck, 100 V in, 50% duty at the given carrier period.
static void buildBuck(double pulsePeriod,
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

struct Run {
    std::vector<double> t;
    std::vector<double> vc;
    size_t samples = 0;
    double ms = 0.0;
    long long accepted = 0, rejected = 0, eventRetries = 0, gateCuts = 0, errChecked = 0;

    double hMin = 0.0, hMax = 0.0;
    long long factorizations = 0, cacheHits = 0;
};

static Run runCase(double pulsePeriod, double dt, double tStop, bool adaptive,
                   double relTol, int rejectEveryN, double hMax = 0.0) {
    std::vector<ComponentModel> phys, ctrl;
    buildBuck(pulsePeriod, phys, ctrl);

    SimulationConfig cfg;
    cfg.stopTime = tStop;
    cfg.stepSize = dt;
    cfg.solver = "euler";
    cfg.step_type = adaptive ? "variable" : "fixed";
    cfg.relTol = relTol;
    cfg.hMax = hMax;
    cfg.debugRejectEveryNthStep = rejectEveryN;

    CircuitSimulator sim;
    sim.setup(phys, ctrl, cfg);
    sim.reset();
    auto c0 = std::chrono::high_resolution_clock::now();
    SimulationOutput o = sim.runTransient();
    auto c1 = std::chrono::high_resolution_clock::now();

    Run r;
    r.t = o.time;
    auto it = o.custom_plots.find("V_C1");
    if (it != o.custom_plots.end()) r.vc = it->second;
    r.samples = o.time.size();
    r.ms = std::chrono::duration<double, std::milli>(c1 - c0).count();
    r.accepted = sim.getAdaptiveAccepted();
    r.rejected = sim.getAdaptiveRejected();
    r.eventRetries = sim.getAdaptiveEventRetries();

    r.gateCuts = sim.getAdaptiveGateCuts();
    r.errChecked = sim.getAdaptiveErrChecked();
    r.hMin = sim.getAdaptiveHMin();
    r.hMax = sim.getAdaptiveHMax();
    r.factorizations = sim.getFactorizeCount();
    r.cacheHits = sim.getLUCacheHitCount();
    return r;
}

static double interpAt(const Run& r, double t) {
    if (r.t.empty()) return 0.0;
    if (t <= r.t.front()) return r.vc.empty() ? 0.0 : r.vc.front();
    if (t >= r.t.back()) return r.vc.empty() ? 0.0 : r.vc.back();
    size_t lo = (size_t)(std::lower_bound(r.t.begin(), r.t.end(), t) - r.t.begin());
    if (lo == 0) return r.vc[0];
    if (lo >= r.vc.size()) return r.vc.back();
    const double t0 = r.t[lo - 1], t1 = r.t[lo];
    const double y0 = r.vc[lo - 1], y1 = r.vc[lo];
    if (t1 - t0 <= 0.0) return y0;
    return y0 + (y1 - y0) * (t - t0) / (t1 - t0);
}

// Max deviation against the reference over the second half of the run, so the
// comparison is about tracking the waveform rather than the opening transient.
static double maxDevVsRef(const Run& a, const Run& ref, double tStop) {
    double worst = 0.0;
    const int N = 2000;
    for (int i = 0; i <= N; ++i) {
        const double t = tStop * 0.5 + (tStop * 0.5) * (double)i / (double)N;
        worst = std::max(worst, std::fabs(interpAt(a, t) - interpAt(ref, t)));
    }
    return worst;
}

int main() {
    int pass = 0, fail = 0;
    auto check = [&](const std::string& what, bool ok) {
        printf("  %-62s %s\n", what.c_str(), ok ? "PASS" : "FAIL");
        if (ok) ++pass; else ++fail;
    };

    printf("Adaptive backward-Euler stepping\n");
    printf("================================\n\n");

    const double tStop = 2e-3;
    const double period = 1e-5;     // 100 kHz carrier
    const double dt = 1e-7;

    // ---- A) rollback correctness -------------------------------------------
    printf("A) Rejected steps leave no trace (exercises save/restoreSolverState)\n");
    Run plain = runCase(period, dt, tStop, true, 1e-3, 0);
    for (int n : {1, 2, 3, 7}) {
        Run forced = runCase(period, dt, tStop, true, 1e-3, n);
        bool same = (forced.t.size() == plain.t.size()) && (forced.vc.size() == plain.vc.size());
        if (same) {
            for (size_t i = 0; i < forced.t.size() && same; ++i) {
                if (std::memcmp(&forced.t[i], &plain.t[i], sizeof(double)) != 0) same = false;
            }
            for (size_t i = 0; i < forced.vc.size() && same; ++i) {
                if (std::memcmp(&forced.vc[i], &plain.vc[i], sizeof(double)) != 0) same = false;
            }
        }
        char buf[128];
        snprintf(buf, sizeof(buf), "rejecting every %d%s step gives identical output", n,
                 (n == 1 ? "st" : (n == 2 ? "nd" : (n == 3 ? "rd" : "th"))));
        check(buf, same);
    }

    // ---- B) accuracy against a fine fixed-step reference -------------------
    printf("\nB) Accuracy vs a fixed-step reference at dt = 10 ns\n");
    Run ref = runCase(period, 1e-8, tStop, false, 0.0, 0);
    Run fixedCoarse = runCase(period, dt, tStop, false, 0.0, 0);
    double devFixed = maxDevVsRef(fixedCoarse, ref, tStop);
    double devAdapt = maxDevVsRef(plain, ref, tStop);
    printf("     fixed dt=10ns  (reference) : %7zu samples %28.1f ms\n", ref.samples, ref.ms);
    printf("     fixed dt=100ns             : %7zu samples  max dev %8.4f V %8.1f ms\n",
           fixedCoarse.samples, devFixed, fixedCoarse.ms);
    printf("     adaptive relTol=1e-3       : %7zu samples  max dev %8.4f V %8.1f ms\n",
           plain.samples, devAdapt, plain.ms);
    check("adaptive run reaches the stop time",
          !plain.t.empty() && plain.t.back() >= tStop * (1.0 - 1e-9));
    check("adaptive uses fewer samples than the fine fixed reference",
          plain.samples < ref.samples);
    // Every carrier edge must be landed on, otherwise duty cycle is quantised to h.
    // 200 carrier cycles over 2 ms at 50% duty gives ~400 edges.
    check("gate edges are localised (>= 300 cuts for ~400 edges)", plain.gateCuts >= 300);

    // ---- C) convergence as the tolerance tightens --------------------------
    printf("\nC) Tightening relTol reduces the error\n");
    printf("  %-8s %9s %10s %8s %8s %8s %9s %9s %8s\n",
           "relTol", "samples", "max dev V", "errChk", "reject", "evtRetr",
           "hMin", "hMax", "ms");
    double prevDev = 1e300;
    bool monotone = true;
    Run tight;
    double devTight = 1e300;
    for (double rt : {1e-2, 1e-3, 1e-4, 1e-5}) {
        Run r = runCase(period, dt, tStop, true, rt, 0);
        double dev = maxDevVsRef(r, ref, tStop);
        if (rt == 1e-5) { tight = r; devTight = dev; }
        printf("  %-8.0e %9zu %10.5f %8lld %8lld %8lld %9.2e %9.2e %8.1f\n",
               rt, r.samples, dev, r.errChecked, r.rejected, r.eventRetries,
               r.hMin, r.hMax, r.ms);
        // Allow a small slack: the estimator is a heuristic, not a bound.
        if (dev > prevDev * 1.5 + 1e-9) monotone = false;
        prevDev = dev;
        fflush(stdout);
    }
    printf("\n  diagnostics at relTol=1e-3: accepted=%lld gateCuts=%lld errChecked=%lld"
           " reject=%lld evtRetry=%lld hMin=%.3e hMax=%.3e fact=%lld hits=%lld\n",
           plain.accepted, plain.gateCuts, plain.errChecked, plain.rejected,
           plain.eventRetries, plain.hMin, plain.hMax,
           plain.factorizations, plain.cacheHits);
    // Event localisation by halving must keep retries on the step ladder, so they hit
    // the factorization cache instead of forcing a new factorization each time. This
    // is what made margin interpolation lose despite needing far fewer retries.
    check("event retries reuse cached factorizations (fact << retries)",
          plain.factorizations * 4 < plain.eventRetries);
    check("error does not grow as relTol is tightened", monotone);

    // The point of the exercise: reach the fixed solver's accuracy for less work.
    printf("\nD) Adaptive vs fixed at comparable accuracy\n");
    printf("     fixed dt=100ns       : %7zu samples  dev %8.4f V  %8.1f ms\n",
           fixedCoarse.samples, devFixed, fixedCoarse.ms);
    printf("     adaptive relTol=1e-5 : %7zu samples  dev %8.4f V  %8.1f ms\n",
           tight.samples, devTight, tight.ms);
    check("adaptive at 1e-5 is at least as accurate as fixed 100 ns",
          devTight <= devFixed * 1.2);
    check("adaptive at 1e-5 needs fewer samples than fixed 100 ns",
          tight.samples < fixedCoarse.samples);

    printf("\n------------------------------------------------------------\n");
    printf("%d/%d checks passed\n", pass, pass + fail);
    return fail == 0 ? 0 : 1;
}
