// Parallel sweep: results bitwise identical to serial runs, independent of worker
// count and of what ran before on a thread (random-source blocks included);
// failing points reported; cancellation; speed-up.
// Usage: test_sweep <tests/golden/designs>
#include "engine/Circuit.hpp"
#include "engine/DesignIO.hpp"
#include "engine/Runner.hpp"
#include "engine/Sweep.hpp"

#include <atomic>
#include <chrono>
#include <cstdio>
#include <string>
#include <thread>

using namespace CircuitSim;

static int g_pass = 0, g_fail = 0;
static void check(bool ok, const std::string& what) {
    if (ok) ++g_pass; else { ++g_fail; std::printf("FAIL: %s\n", what.c_str()); }
}
static bool same(const RunResult& a, const RunResult& b) {
    return a.netlistJson == b.netlistJson && a.time == b.time && a.signals == b.signals;
}

static CircuitDesign noisy() {
    Circuit c("noise");
    c.add("RANDOM_NUM", "RN1", {{"mean", "1"}, {"std", "0.5"}});
    c.add("WHITE_NOISE", "WN1", {{"psd", "0.01"}});
    c.add("GAIN", "G1", {{"K", "2"}});
    c.connect({{"RN1", "Out"}, {"G1", "In"}});
    c.settings().stopTime = 1e-3;
    c.settings().stepSize = 1e-6;
    return c.design();
}

int main(int argc, char** argv) {
    const std::string designs = argc > 1 ? argv[1] : "tests/golden/designs";
    const CircuitDesign buck = DesignIO::loadDesignFile(designs + "/Buck_converter.json");
    RunOverrides ov;
    ov.tStop = 5e-3;

    std::vector<std::vector<std::pair<std::string, std::string>>> points;
    for (const char* l : {"50u", "100u", "200u", "400u"})
        for (const char* r : {"5", "10", "20"})
            points.push_back({{"L1.L", l}, {"R1.R", r}});

    // 1. Serial reference.
    std::vector<RunResult> ref;
    auto t0 = std::chrono::steady_clock::now();
    for (const auto& p : points) {
        RunOverrides pov = ov;
        pov.params = p;
        ref.push_back(runDesign(buck, pov));
    }
    const double serialS = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();

    // 2. Parallel, several worker counts.
    for (int w : {1, 3, 8}) {
        SweepOptions o;
        o.workers = w;
        std::atomic<size_t> calls{0};
        o.onPointDone = [&](size_t, size_t) { calls.fetch_add(1); };
        auto t1 = std::chrono::steady_clock::now();
        auto res = runSweep(buck, ov, points, o);
        const double s = std::chrono::duration<double>(std::chrono::steady_clock::now() - t1).count();
        bool all = res.size() == points.size();
        for (size_t i = 0; all && i < res.size(); ++i) all = res[i].ok && same(res[i].result, ref[i]);
        check(all, "workers=" + std::to_string(w) + ": all points bitwise equal to serial runs");
        check(calls.load() == points.size(), "progress called once per point");
        std::printf("  workers=%d: %.3f s (serial %.3f s, speed-up %.2fx)\n", w, s, serialS, serialS / s);
    }
    {
        // Distinct parameters really produce distinct waveforms.
        check(ref[0].signals.at("I_L1") != ref[11].signals.at("I_L1"), "sweep points differ");
    }

    // 3. Random sources: reproducible run to run on one thread, and in a pool.
    {
        const CircuitDesign d = noisy();
        RunResult a = runDesign(d), b = runDesign(d);
        check(same(a, b), "random blocks: second run on the same thread is identical");
        bool varied = false;
        const auto& v = a.signals.at("RN1.Out");
        for (size_t i = 1; i < v.size(); ++i) varied |= v[i] != v[0];
        check(varied, "random blocks actually produce noise");
        std::vector<std::vector<std::pair<std::string, std::string>>> np(6, std::vector<std::pair<std::string, std::string>>{});
        SweepOptions o;
        o.workers = 3;
        auto res = runSweep(d, RunOverrides(), np, o);
        bool all = true;
        for (const auto& r : res) all &= r.ok && same(r.result, a);
        check(all, "random blocks: identical across pool threads");
        // A run on a fresh thread matches too.
        RunResult c;
        std::thread([&] { c = runDesign(d); }).join();
        check(same(a, c), "random blocks: identical on a fresh thread");
    }

    // 4. Errors and signal filtering.
    {
        std::vector<std::vector<std::pair<std::string, std::string>>> p = {
            {{"L1.L", "100u"}}, {{"NOPE.R", "1"}}, {{"L1.L", "200u"}}};
        SweepOptions o;
        o.workers = 2;
        o.keepSignals = {"V_C1", "I_L1", "not_a_signal"};
        auto res = runSweep(buck, ov, p, o);
        check(res[0].ok && res[2].ok && !res[1].ok, "failing point isolated");
        check(res[1].error.find("NOPE") != std::string::npos, "failing point has its error");
        check(res[0].result.signals.size() == 2 && res[0].result.signals.count("V_C1"), "keepSignals filters");
    }

    // 5. Cancellation: remaining points are skipped.
    {
        std::vector<std::vector<std::pair<std::string, std::string>>> p(40, {{"L1.L", "100u"}});
        RunOverrides longRun;
        longRun.tStop = 0.05;
        std::atomic<bool> cancel{false};
        SweepOptions o;
        o.workers = 2;
        o.cancel = &cancel;
        o.onPointDone = [&](size_t done, size_t) { if (done >= 2) cancel.store(true); };
        auto res = runSweep(buck, longRun, p, o);
        size_t skipped = 0;
        for (const auto& r : res) skipped += r.skipped;
        check(skipped >= 30, "cancel skips the remaining points (" + std::to_string(skipped) + " skipped)");
    }

    std::printf("%d/%d checks passed\n", g_pass, g_pass + g_fail);
    return g_fail == 0 ? 0 : 1;
}
