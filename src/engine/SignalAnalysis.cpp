#include "SignalAnalysis.hpp"

namespace CircuitSimEngine {

SignalStats computeSignalStats(const std::vector<double>& timeHist,
                               const std::vector<double>& signalData,
                               double t1, double t2) {
    SignalStats stats;
    if (timeHist.empty() || signalData.empty()) return stats;
    int n = (int)std::min(timeHist.size(), signalData.size());
    if (n == 0) return stats;

    double tStart = std::min(t1, t2);
    double tEnd = std::max(t1, t2);

    // Clamp to valid time domain bounds
    double minDomainT = timeHist.front();
    double maxDomainT = timeHist.back();

    tStart = std::clamp(tStart, minDomainT, maxDomainT);
    tEnd = std::clamp(tEnd, minDomainT, maxDomainT);

    double totalTime = tEnd - tStart;
    if (totalTime <= 1e-15) {
        // Point measurement if time window is zero
        auto it = std::lower_bound(timeHist.begin(), timeHist.begin() + n, tStart);
        int idx = (int)std::distance(timeHist.begin(), it);
        idx = std::clamp(idx, 0, n - 1);
        double val = signalData[idx];
        stats.mean = val;
        stats.rms = std::abs(val);
        stats.minVal = val;
        stats.maxVal = val;
        stats.absMaxVal = std::abs(val);
        stats.pk2pk = 0.0;
        stats.yAtT1 = val;
        stats.yAtT2 = val;
        stats.isValid = true;
        return stats;
    }

    // Helper for linear interpolation at an arbitrary time point t
    auto getInterpolatedValue = [&](double tTarget) -> double {
        if (tTarget <= timeHist.front()) return signalData.front();
        if (tTarget >= timeHist.back()) return signalData[n - 1];

        auto it = std::lower_bound(timeHist.begin(), timeHist.begin() + n, tTarget);
        int idx = (int)std::distance(timeHist.begin(), it);
        if (idx == 0) return signalData[0];
        if (idx >= n) return signalData[n - 1];

        double t0 = timeHist[idx - 1];
        double t_next = timeHist[idx];
        double y0 = signalData[idx - 1];
        double y_next = signalData[idx];

        if (std::abs(t_next - t0) < 1e-15) return y0;
        double alpha = (tTarget - t0) / (t_next - t0);
        return y0 + alpha * (y_next - y0);
    };

    double yStart = getInterpolatedValue(tStart);
    double yEnd = getInterpolatedValue(tEnd);

    // Construct ordered sequence of sub-points (t_k, y_k)
    struct Pt { double t; double y; };
    std::vector<Pt> points;
    points.reserve(n + 2);

    points.push_back({tStart, yStart});

    // Find interior simulation sample points strictly between tStart and tEnd
    auto itStart = std::upper_bound(timeHist.begin(), timeHist.begin() + n, tStart);
    auto itEnd = std::lower_bound(timeHist.begin(), timeHist.begin() + n, tEnd);

    for (auto it = itStart; it < itEnd; ++it) {
        int idx = (int)std::distance(timeHist.begin(), it);
        points.push_back({timeHist[idx], signalData[idx]});
    }

    points.push_back({tEnd, yEnd});

    // Compute trapezoidal Mean, exact piecewise linear RMS, and Extrema
    double sumAreaMean = 0.0;
    double sumAreaRmsSq = 0.0;

    double minVal = points[0].y;
    double maxVal = points[0].y;
    double absMaxVal = std::abs(points[0].y);

    for (size_t i = 0; i < points.size(); ++i) {
        minVal = std::min(minVal, points[i].y);
        maxVal = std::max(maxVal, points[i].y);
        absMaxVal = std::max(absMaxVal, std::abs(points[i].y));
    }

    for (size_t i = 0; i < points.size() - 1; ++i) {
        double dt = points[i+1].t - points[i].t;
        if (dt <= 1e-15) continue;

        double ya = points[i].y;
        double yb = points[i+1].y;

        // Trapezoidal integral for mean: (ya + yb) / 2 * dt
        sumAreaMean += 0.5 * (ya + yb) * dt;

        // Exact integral for square of piecewise linear function: (ya^2 + ya*yb + yb^2) / 3 * dt
        sumAreaRmsSq += (ya * ya + ya * yb + yb * yb) / 3.0 * dt;
    }

    stats.mean = sumAreaMean / totalTime;
    stats.rms = std::sqrt(std::max(0.0, sumAreaRmsSq / totalTime));
    stats.minVal = minVal;
    stats.maxVal = maxVal;
    stats.absMaxVal = absMaxVal;
    stats.pk2pk = maxVal - minVal;
    stats.yAtT1 = (t1 <= t2) ? yStart : yEnd;
    stats.yAtT2 = (t1 <= t2) ? yEnd : yStart;
    stats.isValid = true;

    return stats;
}

} // namespace CircuitSimEngine
