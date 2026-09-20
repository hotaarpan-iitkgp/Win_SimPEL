#include "LossAnalysisEngine.hpp"
#include <cmath>
#include <algorithm>
#include <iostream>

namespace CircuitSimEngine {

static std::vector<double> findGateSignal(
    const CircuitSim::ComponentInstance& comp,
    const CircuitSim::CircuitDesign& design,
    const std::unordered_map<std::string, std::vector<double>>& voltages
) {
    std::vector<std::string> candidates;

    // 1. Direct parameters
    if (comp.parameters.count("control_signal")) candidates.push_back(comp.parameters.at("control_signal"));
    if (comp.parameters.count("Gate_Signal_Label")) candidates.push_back(comp.parameters.at("Gate_Signal_Label"));
    if (comp.parameters.count("Control")) candidates.push_back(comp.parameters.at("Control"));
    if (comp.parameters.count("tag")) candidates.push_back(comp.parameters.at("tag"));

    // 2. Trace schematic wires connected to gate terminal
    for (const auto& wire : design.wires) {
        if (wire.to.compId == comp.id && (wire.to.terminal == "G" || wire.to.terminal == "Ctrl" || wire.to.terminal == "Gate" || wire.to.terminal == "In")) {
            candidates.push_back(wire.from.compId + "." + wire.from.terminal);
            candidates.push_back(wire.from.compId + ".Out");
            candidates.push_back(wire.from.compId);
        }
        if (wire.from.compId == comp.id && (wire.from.terminal == "G" || wire.from.terminal == "Ctrl" || wire.from.terminal == "Gate" || wire.from.terminal == "In")) {
            candidates.push_back(wire.to.compId + "." + wire.to.terminal);
            candidates.push_back(wire.to.compId + ".Out");
            candidates.push_back(wire.to.compId);
        }
    }

    candidates.push_back(comp.id + ".G");
    candidates.push_back(comp.id + "_gate");

    for (const auto& key : candidates) {
        if (key.empty()) continue;
        auto it = voltages.find(key);
        if (it != voltages.end() && !it->second.empty()) {
            return it->second;
        }
    }

    return {};
}

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
    if (idx_end <= idx_start) {
        // Fallback: if user specified start_time beyond sim duration, use the last portion
        if (t_all.size() > 10) {
            idx_start = (size_t)(t_all.size() * 0.5);
            idx_end = t_all.size();
            duration = t_all.back() - t_all[idx_start];
        } else {
            return results;
        }
    }
    
    std::vector<double> t_slice(t_all.begin() + idx_start, t_all.begin() + idx_end);
    
    // Process each semiconductor in the design
    for (const auto& comp : design.components) {
        bool isPowerSwitch = (comp.type == CircuitSim::ComponentType::MOSFET || comp.type == CircuitSim::ComponentType::IGBT ||
                              comp.type == CircuitSim::ComponentType::IGBTDiode || comp.type == CircuitSim::ComponentType::Diode ||
                              comp.type == CircuitSim::ComponentType::Thyristor || comp.type == CircuitSim::ComponentType::GTO ||
                              comp.type == CircuitSim::ComponentType::IGCT || comp.type == CircuitSim::ComponentType::BJT ||
                              comp.type == CircuitSim::ComponentType::JFET);
                              
        if (!isPowerSwitch) continue;
        
        auto it_tm = comp.parameters.find("thermal_model");
        if (it_tm == comp.parameters.end() || it_tm->second == "Ideal (No Loss)" || it_tm->second.empty()) continue;
        
        auto model = LossModelLibrary::getInstance().getModel(it_tm->second);
        if (!model) continue;
        
        std::string vKey = "V_" + comp.id;
        std::string iKey = "I_" + comp.id;
        
        if (telemetry.voltages.count(vKey) && telemetry.voltages.count(iKey)) {
            const auto& v_all = telemetry.voltages.at(vKey);
            const auto& i_all = telemetry.voltages.at(iKey);
            
            size_t actual_end = std::min({idx_end, v_all.size(), i_all.size()});
            if (actual_end <= idx_start) continue;

            std::vector<double> v_slice(v_all.begin() + idx_start, v_all.begin() + actual_end);
            std::vector<double> i_slice(i_all.begin() + idx_start, i_all.begin() + actual_end);
            std::vector<double> t_curr_slice(t_all.begin() + idx_start, t_all.begin() + actual_end);

            std::vector<double> g_all = findGateSignal(comp, design, telemetry.voltages);
            std::vector<double> g_slice;
            if (!g_all.empty() && g_all.size() >= actual_end) {
                g_slice.assign(g_all.begin() + idx_start, g_all.begin() + actual_end);
            }
            
            auto res = analyzeComponent(comp.id, comp, model, t_curr_slice, v_slice, i_slice, g_slice, global_tj, duration);
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
    
    size_t N = std::min({t.size(), v.size(), i.size()});
    if (N < 2) return res;
    
    double actual_duration = (duration > 0.0) ? duration : (t.back() - t.front());
    if (actual_duration <= 0.0) return res;

    bool has_gate = (g.size() >= N);
    if (has_gate) {
        // Check if gate has any actual transitions or variation
        double g_min = *std::min_element(g.begin(), g.begin() + N);
        double g_max = *std::max_element(g.begin(), g.begin() + N);
        if (std::abs(g_max - g_min) < 0.3) {
            // Gate is completely flat or inactive
            if (comp.type == CircuitSim::ComponentType::Diode) {
                has_gate = false;
            }
        }
    }

    double E_cond = 0.0;
    double E_sw_on = 0.0;
    double E_sw_off = 0.0;

    const double GATE_THRESHOLD = 0.5;

    if (has_gate) {
        // ── Methodology Step 2: Event Extraction via Gate Voltage ──
        bool prev_gate = (g[0] > GATE_THRESHOLD);

        for (size_t k = 1; k < N; ++k) {
            bool cur_gate = (g[k] > GATE_THRESHOLD);

            if (cur_gate && !prev_gate) {
                // TURN-ON EVENT
                double v_sw = std::abs(v[k]);
                size_t look_ahead = std::min(N, k + 20);
                double i_peak = 0.0;
                for (size_t m = k; m < look_ahead; ++m) {
                    if (std::abs(i[m]) > i_peak) i_peak = std::abs(i[m]);
                }

                if (v_sw > 5.0 && i_peak > 0.05) {
                    // Hard Turn-On
                    E_sw_on += model->getTurnOnEnergy(v_sw, i_peak, tj);
                    res.num_hard_turn_on++;
                } else if (v_sw <= 5.0) {
                    // ZVS Soft Turn-On
                    E_sw_on += model->getTurnOnEnergy(v_sw, i_peak, tj);
                    res.num_soft_turn_on++;
                }
            }
            else if (!cur_gate && prev_gate) {
                // TURN-OFF EVENT
                size_t look_back_start = (k >= 5) ? (k - 5) : 0;
                double i_sw = 0.0;
                for (size_t m = look_back_start; m <= k; ++m) {
                    if (std::abs(i[m]) > i_sw) i_sw = std::abs(i[m]);
                }

                size_t look_ahead = std::min(N, k + 20);
                double v_block = 0.0;
                for (size_t m = k; m < look_ahead; ++m) {
                    if (std::abs(v[m]) > v_block) v_block = std::abs(v[m]);
                }

                if (i_sw > 0.05) {
                    E_sw_off += model->getTurnOffEnergy(v_block, i_sw, tj);
                    res.num_turn_off++;
                }
            }

            prev_gate = cur_gate;
        }

        // ── Methodology Step 1 & 5: Conduction Loss (Gate Controlled) ──
        for (size_t k = 0; k < N - 1; ++k) {
            double dt = t[k+1] - t[k];
            double cur_i = std::abs(i[k]);
            if (cur_i < 1e-3) continue;

            bool is_gate_on = (g[k] > GATE_THRESHOLD);
            double v_drop = 0.0;

            if (is_gate_on) {
                // Channel conduction
                if (model->hasConductionTable(true)) {
                    v_drop = model->getConductionVoltageDrop(cur_i, tj, true);
                } else {
                    v_drop = std::abs(v[k]);
                }
            } else {
                // Gate OFF: check if body diode or reverse current is conducting
                if (std::abs(v[k]) < 5.0) {
                    if (model->hasConductionTable(false)) {
                        v_drop = model->getConductionVoltageDrop(cur_i, tj, false);
                    } else {
                        v_drop = std::abs(v[k]);
                    }
                }
            }

            E_cond += v_drop * cur_i * dt;
        }

    } else {
        // ── Fallback for Passive Diodes or Devices without explicit Gate ──
        // 1. Detect Turn-Off (Reverse Recovery) from Vds
        double v_max = -1e9, v_min = 1e9;
        for (size_t k = 0; k < N; ++k) {
            if (v[k] > v_max) v_max = v[k];
            if (v[k] < v_min) v_min = v[k];
        }
        double v_range = v_max - v_min;

        if (v_range >= 5.0) {
            double v_mid = (v_max + v_min) * 0.5;
            bool state_high = (v[0] > v_mid);

            for (size_t k = 1; k < N - 1; ++k) {
                if (state_high && v[k] < (v_min + 0.2 * v_range)) {
                    // Transition to reverse bias -> Reverse Recovery
                    size_t look_back_start = (k >= 10) ? (k - 10) : 0;
                    double i_before = 0.0;
                    for (size_t m = look_back_start; m < k; ++m) {
                        if (std::abs(i[m]) > i_before) i_before = std::abs(i[m]);
                    }

                    size_t look_ahead = std::min(N, k + 10);
                    double v_after = 0.0;
                    for (size_t m = k; m < look_ahead; ++m) {
                        if (std::abs(v[m]) > v_after) v_after = std::abs(v[m]);
                    }

                    if (i_before > 0.05) {
                        E_sw_off += model->getTurnOffEnergy(v_after, i_before, tj);
                        res.num_turn_off++;
                    }
                    state_high = false;
                } else if (!state_high && v[k] > (v_max - 0.2 * v_range)) {
                    state_high = true;
                }
            }
        }

        // 2. Diode Conduction Loss
        for (size_t k = 0; k < N - 1; ++k) {
            double dt = t[k+1] - t[k];
            double cur_i = std::abs(i[k]);
            double cur_v = std::abs(v[k]);

            // Diode is conducting when current is present and voltage drop is low (< 5V)
            if (cur_i > 0.05 && cur_v < 5.0) {
                double v_drop = 0.0;
                if (model->hasConductionTable(false)) {
                    v_drop = model->getConductionVoltageDrop(cur_i, tj, false);
                } else if (model->hasConductionTable(true)) {
                    v_drop = model->getConductionVoltageDrop(cur_i, tj, true);
                } else {
                    // Fallback to direct V*I from simulation waveform
                    v_drop = cur_v;
                }

                E_cond += v_drop * cur_i * dt;
            }
        }
    }
    
    res.P_cond = E_cond / actual_duration;
    res.P_sw_on = E_sw_on / actual_duration;
    res.P_sw_off = E_sw_off / actual_duration;
    res.P_total = res.P_cond + res.P_sw_on + res.P_sw_off;
    
    return res;
}

} // namespace CircuitSimEngine
