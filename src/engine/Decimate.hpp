#pragma once

#include <cstddef>
#include <vector>

namespace CircuitSim {

// Min-max decimation for plotting: splits the samples into maxPoints/2 buckets and
// keeps each bucket's minimum and maximum (in time order), so switching spikes and
// extremes survive while the point count is capped. Moved from ui/TracePlotter.
void decimateMinMax(const double* rawT,
                    const double* rawY,
                    size_t count,
                    size_t maxPoints,
                    std::vector<double>& outT,
                    std::vector<double>& outY);

} // namespace CircuitSim
