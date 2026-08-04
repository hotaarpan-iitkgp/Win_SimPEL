#pragma once

#include "engine/Components.hpp"
#include <string>

namespace CircuitSim {

class ConfiguratorDialog {
public:
    static bool showConfiguratorModal(ComponentInstance& comp, bool* openFlag);
};

} // namespace CircuitSim
