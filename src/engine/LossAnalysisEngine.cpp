#include "LossAnalysisEngine.hpp"
#include <cmath>
#include <algorithm>
#include <iostream>

namespace CircuitSimEngine {

std::vector<SwitchLossResult> LossAnalysisEngine::runAnalysis(
    const CircuitSimulator& sim,
    const CircuitSim::CircuitDesign& design,
    double t_start, 
    double duration, 
    double global_tj
) {
    std::vector<SwitchLossResult> results;
    
    auto telemetry = const_cast<CircuitSimulator*>(&sim)->getTelemetryCopy();
    
    // get time array
    const auto& t_all = telemetry.timeHistory;
    if (t_all.empty()) return results;
    
    // slice time array
    double t_end = t_start + duration;
    auto it_start = std::lower_bound(t_all.begin(), t_all.end(), t_start);
    auto it_end = std::lower_bound(t_all.begin(), t_all.end(), t_end);
    
    size_t idx_start = std::distance(t_all.begin(), it_start);
    size_t idx_end = std::distance(t_all.begin(), it_end);
    if (idx_end <= idx_start) return results;
    
    std::vector<double> t_slice(it_start, it_end);
    
    // find power switches
    for (const auto& comp : design.components) {
        bool isPowerSwitch = (comp.type == CircuitSim::ComponentType::MOSFET || comp.type == CircuitSim::ComponentType::IGBT ||
                              comp.type == CircuitSim::ComponentType::IGBTDiode || comp.type == CircuitSim::ComponentType::Diode ||
                              comp.type == CircuitSim::ComponentType::Thyristor || comp.type == CircuitSim::ComponentType::GTO ||
                              comp.type == CircuitSim::ComponentType::IGCT || comp.type == CircuitSim::ComponentType::BJT ||
                              comp.type == CircuitSim::ComponentType::JFET);
                              
        if (!isPowerSwitch) continue;
        
        auto it_tm = comp.parameters.find("thermal_model");
        if (it_tm == comp.parameters.end() || it_tm->second == "Ideal (No Loss)") continue;
        
        auto model = LossModelLibrary::getInstance().getModel(it_tm->second);
        if (!model) continue;
        
        std::string vKey = "V_" + comp.id;
        std::string iKey = "I_" + comp.id;
        // The gate logic signal name
        std::string gKey;
        if (comp.parameters.count("Gate_Signal_Label")) {
            gKey = comp.parameters.at("Gate_Signal_Label");
        } else {
            // some default?
        }
        
        if (telemetry.voltages.count(vKey) && telemetry.voltages.count(iKey)) {
            const auto& v_all = telemetry.voltages.at(vKey);
            const auto& i_all = telemetry.voltages.at(iKey);
            
            std::vector<double> v_slice(v_all.begin() + idx_start, v_all.begin() + idx_end);
            std::vector<double> i_slice(i_all.begin() + idx_start, i_all.begin() + idx_end);
            std::vector<double> g_slice;
            
            if (!gKey.empty() && telemetry.voltages.count(gKey)) {
                const auto& g_all = telemetry.voltages.at(gKey);
                g_slice.assign(g_all.begin() + idx_start, g_all.begin() + idx_end);
            } else {
                g_slice.resize(v_slice.size(), 0.0);
            }
            
            auto res = analyzeComponent(comp.id, comp, model, t_slice, v_slice, i_slice, g_slice, global_tj, duration);
            results.push_back(res);
        }
    }
    
    return results;
}

SwitchLossResult LossAnalysisEngine::analyzeComponent(
    const std::string& compId,
    const CircuitSim::ComponentInstance& comp,
    std::shared_ptr<LossModel> model,
    const std::vector<double>& t,
    const std::vector<double>& v,
    const std::vector<double>& i,
    const std::vector<double>& g,
    double tj,
    double duration
) {
    SwitchLossResult res;
    res.componentId = compId;
    res.modelName = comp.parameters.at("thermal_model");
    
    size_t N = t.size();
    if (N < 2) return res;
    
    double E_cond = 0.0;
    double E_sw_on = 0.0;
    double E_sw_off = 0.0;
    
    // Conduction Loss
    for (size_t k = 0; k < N - 1; ++k) {
        double dt = t[k+1] - t[k];
        double current = i[k];
        if (std::abs(current) < 1e-4) continue;
        
        bool is_gate_on = (g[k] > 0.5);
        double v_drop = model->getConductionVoltageDrop(std::abs(current), tj, is_gate_on);
        E_cond += v_drop * std::abs(current) * dt;
    }
    
    // Switching Loss Events
    bool prev_gate = (g[0] > 0.5);
    for (size_t k = 1; k < N; ++k) {
        bool cur_gate = (g[k] > 0.5);
        
        if (cur_gate && !prev_gate) {
            // Turn ON event
            double v_block = (v[k-1] > 0) ? v[k-1] : 0.0;
            double i_on = std::abs(i[k]); // simplistic
            if (v_block < 5.0 || i_on < 0.0) {
                // Soft turn-on
                res.num_soft_turn_on++;
            } else {
                E_sw_on += model->getTurnOnEnergy(v_block, i_on, tj);
                res.num_hard_turn_on++;
            }
        } 
        else if (!cur_gate && prev_gate) {
            // Turn OFF event
            double v_block = (k+1 < N) ? v[k+1] : v[k]; // Next V
            double i_off = std::abs(i[k-1]);
            E_sw_off += model->getTurnOffEnergy(v_block, i_off, tj);
            res.num_turn_off++;
        }
        
        prev_gate = cur_gate;
    }
    
    if (duration > 0.0) {
        res.P_cond = E_cond / duration;
        res.P_sw_on = E_sw_on / duration;
        res.P_sw_off = E_sw_off / duration;
        res.P_total = res.P_cond + res.P_sw_on + res.P_sw_off;
    }
    
    return res;
}

} // namespace CircuitSimEngine
