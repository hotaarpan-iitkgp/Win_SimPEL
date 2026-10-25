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

} // namespace CircuitSim
