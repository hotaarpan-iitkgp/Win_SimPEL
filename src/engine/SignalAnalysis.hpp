#pragma once

#include <vector>
#include <cmath>
#include <algorithm>
#include <cfloat>

namespace CircuitSimEngine {

struct SignalStats {
    double mean = 0.0;
    double rms = 0.0;
    double minVal = 0.0;
    double maxVal = 0.0;
    double absMaxVal = 0.0;
    double pk2pk = 0.0;
    double yAtT1 = 0.0;
    double yAtT2 = 0.0;
    bool isValid = false;
};

// Computes non-uniform time-step statistics over interval [t1, t2]
// Uses trapezoidal integration for Mean and exact piecewise linear integration for RMS
SignalStats computeSignalStats(const std::vector<double>& timeHist,
                               const std::vector<double>& signalData,
                               double t1, double t2);

} // namespace CircuitSimEngine
