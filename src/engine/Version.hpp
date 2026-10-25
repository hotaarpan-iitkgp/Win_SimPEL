#pragma once

// SimPEL core version. Bump the minor version when the Python API or the design /
// results file formats gain features, the major version when they break.
#define SIMPEL_VERSION_MAJOR 0
#define SIMPEL_VERSION_MINOR 1
#define SIMPEL_VERSION_PATCH 0
#define SIMPEL_VERSION_STRING "0.1.0"

namespace CircuitSim {
inline const char* simpelVersion() { return SIMPEL_VERSION_STRING; }
} // namespace CircuitSim
