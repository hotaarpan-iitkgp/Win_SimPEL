#pragma once

#include <vector>
#include <cmath>
#include <algorithm>
#include <cfloat>

namespace CircuitSimEngine {

struct FourierResult {
    double fundamentalFreq = 0.0; // f0 = 1 / (t2 - t1)
    double fundamentalMag = 0.0;  // V1 (Peak magnitude of fundamental)
    double thdRatio = 0.0;        // THD decimal fraction
    double thdPercent = 0.0;      // THD % (e.g. 5.2%)
    double dcOffset = 0.0;        // V0 (DC component)
    int maxHarmonic = 50;

    std::vector<double> harmonicOrders; // 1, 2, ..., N
    std::vector<double> harmonicMags;   // V1, V2, ..., VN
    std::vector<double> harmonicPhases; // phi1, phi2, ..., phiN

    bool isValid = false;
};

// Continuous piecewise-linear exact Fourier spectrum and THD analyzer over interval [t1, t2]
// Uses explicit fundamental period T = |t2 - t1| (f0 = 1/T)
// Operates on non-uniform variable time-step data without zero-padding or FFT resampling
FourierResult computeFourierSpectrum(const std::vector<double>& timeHist,
                                     const std::vector<double>& signalData,
                                     double t1, double t2,
                                     int maxHarmonics = 50);

} // namespace CircuitSimEngine
