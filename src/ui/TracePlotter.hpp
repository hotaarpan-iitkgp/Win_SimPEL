#pragma once

#include <vector>
#include <string>
#include <algorithm>
#include <cmath>

namespace CircuitSim {

enum class InterpolationMode {
    AutoHybrid,   // Step interpolation at switching events, linear elsewhere
    AlwaysLinear, // Standard linear interpolation
    AlwaysStairs  // Always step interpolation (for Gate Pulses / PWM / Logic)
};

// Determines default interpolation mode based on signal key / component type
InterpolationMode detectDefaultInterpolationMode(const std::string& signalKey);

// Builds hybrid vertices for plotting signals with switching jumps
// Inserts step corner (t_{k+1}, y_k) ONLY at detected switching event steps
void buildHybridVertices(const std::vector<double>& rawT,
                         const std::vector<double>& rawY,
                         std::vector<double>& outT,
                         std::vector<double>& outY,
                         double jumpRelThreshold = 0.025);

// Min-Max Decimation / LOD Downsampling to cap rendered points at maxPoints (e.g. 2000)
// Preserves all local min/max peaks, switching spikes, and extreme values while reducing render load
void decimateMinMax(const double* rawT,
                    const double* rawY,
                    size_t count,
                    size_t maxPoints,
                    std::vector<double>& outT,
                    std::vector<double>& outY);

} // namespace CircuitSim
