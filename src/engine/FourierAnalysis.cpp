#include "FourierAnalysis.hpp"

namespace CircuitSimEngine {

static constexpr double PI = 3.14159265358979323846;

FourierResult computeFourierSpectrum(const std::vector<double>& timeHist,
                                     const std::vector<double>& signalData,
                                     double t1, double t2,
                                     int maxHarmonics) {
    FourierResult result;
    if (timeHist.empty() || signalData.empty()) return result;
    int n = (int)std::min(timeHist.size(), signalData.size());
    if (n == 0) return result;

    double tStart = std::min(t1, t2);
    double tEnd = std::max(t1, t2);

    // Clamp to valid time domain bounds
    double minDomainT = timeHist.front();
    double maxDomainT = timeHist.back();

    tStart = std::clamp(tStart, minDomainT, maxDomainT);
    tEnd = std::clamp(tEnd, minDomainT, maxDomainT);

    double T = tEnd - tStart;
    if (T <= 1e-12) return result; // Fundamental period must be > 0

    double f0 = 1.0 / T;
    double omega0 = 2.0 * PI * f0;

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

    // Construct ordered sequence of sub-points relative to tStart: u = t - tStart in [0, T]
    struct SubPt { double u; double y; };
    std::vector<SubPt> points;
    points.reserve(n + 2);

    points.push_back({0.0, yStart});

    auto itStart = std::upper_bound(timeHist.begin(), timeHist.begin() + n, tStart);
    auto itEnd = std::lower_bound(timeHist.begin(), timeHist.begin() + n, tEnd);

    for (auto it = itStart; it < itEnd; ++it) {
        int idx = (int)std::distance(timeHist.begin(), it);
        double u = timeHist[idx] - tStart;
        points.push_back({u, signalData[idx]});
    }

    points.push_back({T, yEnd});

    // 1. Calculate DC Component (a0 / 2)
    double dcArea = 0.0;
    for (size_t i = 0; i < points.size() - 1; ++i) {
        double du = points[i+1].u - points[i].u;
        if (du <= 1e-15) continue;
        dcArea += 0.5 * (points[i].y + points[i+1].y) * du;
    }
    double v0 = dcArea / T;

    // 2. Calculate Harmonics n = 1 .. maxHarmonics using exact piecewise linear integration
    int numHarmonics = std::clamp(maxHarmonics, 1, 100);
    result.harmonicOrders.reserve(numHarmonics);
    result.harmonicMags.reserve(numHarmonics);
    result.harmonicPhases.reserve(numHarmonics);

    double sumHarmonicsSq = 0.0;

    for (int h = 1; h <= numHarmonics; ++h) {
        double omega = h * omega0;
        double omegaSq = omega * omega;

        double sumIc = 0.0;
        double sumIs = 0.0;

        for (size_t i = 0; i < points.size() - 1; ++i) {
            double u_a = points[i].u;
            double u_b = points[i+1].u;
            double du = u_b - u_a;
            if (du <= 1e-15) continue;

            double ya = points[i].y;
            double yb = points[i+1].y;
            double m = (yb - ya) / du;

            double sinA = std::sin(omega * u_a);
            double sinB = std::sin(omega * u_b);
            double cosA = std::cos(omega * u_a);
            double cosB = std::cos(omega * u_b);

            if (std::abs(m) < 1e-14) {
                sumIc += (ya / omega) * (sinB - sinA);
                sumIs += (ya / omega) * (cosA - cosB);
            } else {
                sumIc += (yb * sinB - ya * sinA) / omega + m * (cosB - cosA) / omegaSq;
                sumIs += (ya * cosA - yb * cosB) / omega + m * (sinB - sinA) / omegaSq;
            }
        }

        double an = (2.0 / T) * sumIc;
        double bn = (2.0 / T) * sumIs;
        double vn = std::sqrt(an * an + bn * bn);
        double phase = std::atan2(-bn, an);

        result.harmonicOrders.push_back((double)h);
        result.harmonicMags.push_back(vn);
        result.harmonicPhases.push_back(phase);

        if (h >= 2) {
            sumHarmonicsSq += vn * vn;
        }
    }

    double v1 = result.harmonicMags.empty() ? 0.0 : result.harmonicMags[0];
    double thdRatio = (v1 > 1e-12) ? (std::sqrt(sumHarmonicsSq) / v1) : 0.0;

    result.fundamentalFreq = f0;
    result.fundamentalMag = v1;
    result.thdRatio = thdRatio;
    result.thdPercent = thdRatio * 100.0;
    result.dcOffset = v0;
    result.maxHarmonic = numHarmonics;
    result.isValid = true;

    return result;
}

} // namespace CircuitSimEngine
