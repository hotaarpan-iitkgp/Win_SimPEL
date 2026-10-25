#pragma once

// Parallel parameter sweeps: one design, many parameter sets, simulated on a pool
// of worker threads. Every point gets its own simulator; results are returned in
// point order and are bitwise identical to running the points one by one
// (tests/test_sweep.cpp checks this, including random-source blocks).
//
// Thread safety of the core, as audited for this: the block registry is built
// once (thread-safe static) and read-only afterwards; the loss model library is
// mutex-protected; netlist generation, parsing and simulation keep all state in
// their own objects. The only shared mutable state was the random-number blocks'
// generators (thread_local, so results depended on the thread's history); they
// are now per simulator and re-seeded on every run.

#include "Runner.hpp"

#include <atomic>
#include <functional>
#include <string>
#include <vector>

namespace CircuitSim {

struct SweepPointResult {
    bool ok = false;
    bool skipped = false;     // not started because the sweep was cancelled
    std::string error;        // set when !ok && !skipped
    RunResult result;
};

struct SweepOptions {
    int workers = 0;                        // <= 0: hardware concurrency
    std::vector<std::string> keepSignals;   // empty: keep all signals
    const std::atomic<bool>* cancel = nullptr;
    // Called from worker threads after each point (done count, total). Must be
    // thread-safe.
    std::function<void(size_t, size_t)> onPointDone;
};

// `points[i]` are the parameter overrides of point i on top of `base` (and `ov`'s
// time settings). Never throws for a failing point: its error is recorded.
std::vector<SweepPointResult> runSweep(const CircuitDesign& base,
                                       const RunOverrides& ov,
                                       const std::vector<std::vector<std::pair<std::string, std::string>>>& points,
                                       const SweepOptions& opts = SweepOptions());

int defaultWorkerCount();

} // namespace CircuitSim
