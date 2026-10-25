#include "Sweep.hpp"

#include <algorithm>
#include <exception>
#include <thread>

namespace CircuitSim {

int defaultWorkerCount() {
    unsigned n = std::thread::hardware_concurrency();
    return n == 0 ? 2 : (int)n;
}

std::vector<SweepPointResult> runSweep(const CircuitDesign& base,
                                       const RunOverrides& ov,
                                       const std::vector<std::vector<std::pair<std::string, std::string>>>& points,
                                       const SweepOptions& opts) {
    const size_t n = points.size();
    std::vector<SweepPointResult> out(n);
    if (n == 0) return out;

    int workers = opts.workers > 0 ? opts.workers : defaultWorkerCount();
    workers = (int)std::min<size_t>((size_t)workers, n);

    std::atomic<size_t> next{0};
    std::atomic<size_t> done{0};
    auto cancelled = [&] { return opts.cancel && opts.cancel->load(std::memory_order_relaxed); };

    auto work = [&]() {
        for (;;) {
            const size_t i = next.fetch_add(1);
            if (i >= n) return;
            SweepPointResult& r = out[i];
            if (cancelled()) {
                r.skipped = true;
                r.error = "cancelled";
            } else {
                try {
                    RunOverrides pov = ov;
                    pov.params.insert(pov.params.end(), points[i].begin(), points[i].end());
                    RunControl ctl;
                    ctl.cancel = opts.cancel;
                    r.result = runDesign(base, pov, opts.cancel ? &ctl : nullptr);
                    if (!opts.keepSignals.empty()) {
                        decltype(r.result.signals) kept;
                        for (const auto& k : opts.keepSignals) {
                            auto it = r.result.signals.find(k);
                            if (it != r.result.signals.end()) kept.emplace(k, std::move(it->second));
                        }
                        r.result.signals = std::move(kept);
                    }
                    r.ok = true;
                } catch (const std::exception& e) {
                    r.error = e.what();
                } catch (...) {
                    r.error = "unknown error";
                }
            }
            const size_t d = done.fetch_add(1) + 1;
            if (opts.onPointDone) opts.onPointDone(d, n);
        }
    };

    if (workers <= 1) {
        work();
    } else {
        std::vector<std::thread> pool;
        pool.reserve((size_t)workers);
        for (int w = 0; w < workers; ++w) pool.emplace_back(work);
        for (auto& t : pool) t.join();
    }
    return out;
}

} // namespace CircuitSim
