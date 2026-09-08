#include "LossModel.hpp"
#include <fstream>
#include <algorithm>
#include <iostream>

namespace CircuitSimEngine {

static void getWeights(double x, const std::vector<double>& axis, size_t& idx, double& frac) {
    if (axis.empty()) { idx = 0; frac = 0.0; return; }
    if (axis.size() == 1) { idx = 0; frac = 0.0; return; }
    
    auto it = std::lower_bound(axis.begin(), axis.end(), x);
    if (it == axis.begin()) {
        idx = 0;
    } else if (it == axis.end()) {
        idx = axis.size() - 2;
    } else {
        idx = std::distance(axis.begin(), it) - 1;
    }
    frac = (x - axis[idx]) / (axis[idx+1] - axis[idx]);
}

double Interpolator2D::evaluate(double i, double t) const {
    if (!valid) return 0.0;
    
    size_t i_idx, t_idx;
    double i_f, t_f;
    getWeights(i, i_axis, i_idx, i_f);
    getWeights(t, t_axis, t_idx, t_f);
    
    size_t Nt = t_axis.size();
    
    double v00 = data[i_idx * Nt + t_idx];
    double v01 = data[i_idx * Nt + t_idx + 1];
    double v10 = data[(i_idx + 1) * Nt + t_idx];
    double v11 = data[(i_idx + 1) * Nt + t_idx + 1];
    
    double c0 = v00 + i_f * (v10 - v00);
    double c1 = v01 + i_f * (v11 - v01);
    
    return c0 + t_f * (c1 - c0);
}

double Interpolator3D::evaluate(double v, double i, double t) const {
    if (!valid) return 0.0;
    
    size_t v_idx, i_idx, t_idx;
    double v_f, i_f, t_f;
    getWeights(v, v_axis, v_idx, v_f);
    getWeights(i, i_axis, i_idx, i_f);
    getWeights(t, t_axis, t_idx, t_f);
    
    size_t Ni = i_axis.size();
    size_t Nt = t_axis.size();
    
    auto getVal = [&](size_t dv, size_t di, size_t dt) {
        return data[(v_idx + dv) * (Ni * Nt) + (i_idx + di) * Nt + (t_idx + dt)];
    };
    
    double c00 = getVal(0,0,0) * (1 - v_f) + getVal(1,0,0) * v_f;
    double c01 = getVal(0,0,1) * (1 - v_f) + getVal(1,0,1) * v_f;
    double c10 = getVal(0,1,0) * (1 - v_f) + getVal(1,1,0) * v_f;
    double c11 = getVal(0,1,1) * (1 - v_f) + getVal(1,1,1) * v_f;
    
    double c0 = c00 + i_f * (c10 - c00);
    double c1 = c01 + i_f * (c11 - c01);
    
    return c0 + t_f * (c1 - c0);
}

LossModel::LossModel(const std::string& filepath) {
    try {
        std::ifstream file(filepath);
        if (!file.is_open()) return;
        
        nlohmann::json j;
        file >> j;
        
        if (j.contains("metadata")) {
            auto meta = j["metadata"];
            part_number = meta.value("part_number", "Unknown");
            device_type = meta.value("type", "Unknown");
            Qg_nC = meta.value("Qg_nC", 0.0);
            I_leakage_uA = meta.value("I_leakage_uA", 0.0);
            V_drv_V = meta.value("V_drv_V", 15.0);
        }
        
        if (j.contains("turn_on_loss") && !j["turn_on_loss"].is_null()) {
            parseBlock3D(j["turn_on_loss"], turn_on_interp);
        }
        if (j.contains("turn_off_loss") && !j["turn_off_loss"].is_null()) {
            parseBlock3D(j["turn_off_loss"], turn_off_interp);
        }
        if (j.contains("conduction_loss_on") && !j["conduction_loss_on"].is_null()) {
            parseBlock2D(j["conduction_loss_on"], cond_on_interp);
        }
        if (j.contains("conduction_loss_off") && !j["conduction_loss_off"].is_null()) {
            parseBlock2D(j["conduction_loss_off"], cond_off_interp);
        }
        
        valid = true;
    } catch (const std::exception& e) {
        std::cerr << "LossModel parse error (" << filepath << "): " << e.what() << "\n";
    }
}

void LossModel::parseBlock3D(const nlohmann::json& block, Interpolator3D& interp) {
    try {
        interp.v_axis = block["v_axis"].get<std::vector<double>>();
        interp.i_axis = block["i_axis"].get<std::vector<double>>();
        interp.t_axis = block["t_axis"].get<std::vector<double>>();
        
        auto data_json = block["data"];
        size_t Nv = interp.v_axis.size();
        size_t Ni = interp.i_axis.size();
        size_t Nt = interp.t_axis.size();
        
        interp.data.reserve(Nv * Ni * Nt);
        for (size_t v = 0; v < Nv; ++v) {
            for (size_t i = 0; i < Ni; ++i) {
                for (size_t t = 0; t < Nt; ++t) {
                    interp.data.push_back(data_json[v][i][t].get<double>());
                }
            }
        }
        interp.valid = true;
    } catch (...) { interp.valid = false; }
}

void LossModel::parseBlock2D(const nlohmann::json& block, Interpolator2D& interp) {
    try {
        interp.i_axis = block["i_axis"].get<std::vector<double>>();
        interp.t_axis = block["t_axis"].get<std::vector<double>>();
        
        auto data_json = block["data"];
        size_t Ni = interp.i_axis.size();
        size_t Nt = interp.t_axis.size();
        
        interp.data.reserve(Ni * Nt);
        for (size_t i = 0; i < Ni; ++i) {
            for (size_t t = 0; t < Nt; ++t) {
                interp.data.push_back(data_json[i][t].get<double>());
            }
        }
        interp.valid = true;
    } catch (...) { interp.valid = false; }
}

double LossModel::getTurnOnEnergy(double v, double i, double t) const {
    return turn_on_interp.evaluate(v, i, t);
}

double LossModel::getTurnOffEnergy(double v, double i, double t) const {
    return turn_off_interp.evaluate(v, i, t);
}

double LossModel::getConductionVoltageDrop(double i, double t, bool gate_on) const {
    if (gate_on && cond_on_interp.valid) return cond_on_interp.evaluate(i, t);
    if (!gate_on && cond_off_interp.valid) return cond_off_interp.evaluate(i, t);
    return 0.0;
}

} // namespace CircuitSimEngine
