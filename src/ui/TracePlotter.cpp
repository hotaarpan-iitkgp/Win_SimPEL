#include "TracePlotter.hpp"
#include <cctype>

namespace CircuitSim {

static std::string toUpper(std::string str) {
    for (char& c : str) c = (char)std::toupper((unsigned char)c);
    return str;
}

InterpolationMode detectDefaultInterpolationMode(const std::string& signalKey) {
    std::string key = toUpper(signalKey);

    // Gate Pulses, Digital Control, PWM, Square wave generators MUST always be step interpolation
    if (key.find("PULSE") != std::string::npos ||
        key.find("PWM") != std::string::npos ||
        key.find("GATE") != std::string::npos ||
        key.find("CLOCK") != std::string::npos ||
        key.find("CLK") != std::string::npos ||
        key.find("LOGIC") != std::string::npos ||
        key.find("SQUARE") != std::string::npos ||
        key.find("DIGITAL") != std::string::npos ||
        key.find("STEP") != std::string::npos) {
        return InterpolationMode::AlwaysStairs;
    }

    // Electrical signals use AutoHybrid (switching events get step interpolation, continuous regions get linear)
    return InterpolationMode::AutoHybrid;
}

void buildHybridVertices(const std::vector<double>& rawT,
                         const std::vector<double>& rawY,
                         std::vector<double>& outT,
                         std::vector<double>& outY,
                         double jumpRelThreshold) {
    outT.clear();
    outY.clear();

    size_t n = std::min(rawT.size(), rawY.size());
    if (n == 0) return;

    outT.reserve(n * 2);
    outY.reserve(n * 2);

    // Find signal span (yMax - yMin)
    double yMin = rawY[0];
    double yMax = rawY[0];
    for (size_t i = 1; i < n; ++i) {
        if (rawY[i] < yMin) yMin = rawY[i];
        if (rawY[i] > yMax) yMax = rawY[i];
    }

    double ySpan = yMax - yMin;
    if (ySpan < 1e-12) {
        outT = rawT;
        outY = rawY;
        return;
    }

    double tDomain = rawT.back() - rawT.front();
    double dtEventThresh = std::max(1e-6, 1e-4 * tDomain);

    for (size_t k = 0; k < n; ++k) {
        outT.push_back(rawT[k]);
        outY.push_back(rawY[k]);

        if (k < n - 1) {
            double dt = rawT[k+1] - rawT[k];
            double dy = std::abs(rawY[k+1] - rawY[k]);

            // Detect switching event jump
            bool isSwitchingJump = (dt <= dtEventThresh && (dy / ySpan) >= jumpRelThreshold) ||
                                   (dt > 0.0 && (dy / dt) > (1e6 * ySpan));

            if (isSwitchingJump) {
                // Insert vertical step corner at (t_{k+1}, y_k)
                outT.push_back(rawT[k+1]);
                outY.push_back(rawY[k]);
            }
        }
    }
}

void decimateMinMax(const double* rawT,
                    const double* rawY,
                    size_t count,
                    size_t maxPoints,
                    std::vector<double>& outT,
                    std::vector<double>& outY) {
    outT.clear();
    outY.clear();

    if (count == 0) return;

    if (count <= maxPoints || maxPoints < 4) {
        outT.assign(rawT, rawT + count);
        outY.assign(rawY, rawY + count);
        return;
    }

    size_t numBuckets = maxPoints / 2;
    size_t bucketSize = count / numBuckets;
    if (bucketSize < 1) bucketSize = 1;

    outT.reserve(numBuckets * 2);
    outY.reserve(numBuckets * 2);

    for (size_t b = 0; b < numBuckets; ++b) {
        size_t startIdx = b * bucketSize;
        size_t endIdx = (b == numBuckets - 1) ? count : (b + 1) * bucketSize;
        if (startIdx >= count) break;

        size_t minIdx = startIdx;
        size_t maxIdx = startIdx;
        double minVal = rawY[startIdx];
        double maxVal = rawY[startIdx];

        for (size_t i = startIdx + 1; i < endIdx; ++i) {
            double v = rawY[i];
            if (v < minVal) { minVal = v; minIdx = i; }
            if (v > maxVal) { maxVal = v; maxIdx = i; }
        }

        if (minIdx <= maxIdx) {
            outT.push_back(rawT[minIdx]); outY.push_back(rawY[minIdx]);
            if (minIdx != maxIdx) {
                outT.push_back(rawT[maxIdx]); outY.push_back(rawY[maxIdx]);
            }
        } else {
            outT.push_back(rawT[maxIdx]); outY.push_back(rawY[maxIdx]);
            outT.push_back(rawT[minIdx]); outY.push_back(rawY[minIdx]);
        }
    }
}

} // namespace CircuitSim
