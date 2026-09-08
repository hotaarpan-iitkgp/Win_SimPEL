#pragma once

#include "engine/CircuitSimulator.hpp"
#include "engine/LossModelLibrary.hpp"
#include "engine/Components.hpp"
#include <vector>
#include <string>
#include <map>

namespace CircuitSimEngine {

struct SwitchLossResult {
    std::string componentId;
    std::string modelName;
    double P_cond = 0.0;
    double P_sw_on = 0.0;
    double P_sw_off = 0.0;
    double P_total = 0.0;
    
    // Detailed events
    int num_hard_turn_on = 0;
    int num_soft_turn_on = 0;
    int num_turn_off = 0;
};

class LossAnalysisEngine {
public:
    // Run post-processing on the entire circuit telemetry for the specified window
    static std::vector<SwitchLossResult> runAnalysis(
        const CircuitSimulator& sim,
        const CircuitSim::CircuitDesign& design,
        double t_start, 
        double duration, 
        double global_tj
    );

private:
    static SwitchLossResult analyzeComponent(
        const std::string& compId,
        const CircuitSim::ComponentInstance& comp,
        std::shared_ptr<LossModel> model,
        const std::vector<double>& t_slice,
        const std::vector<double>& v_slice,
        const std::vector<double>& i_slice,
        const std::vector<double>& g_slice,
        double tj,
        double duration
    );
};

} // namespace CircuitSimEngine
