#include "Decimate.hpp"

namespace CircuitSim {

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
