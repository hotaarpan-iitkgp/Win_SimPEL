#include "LossModel.hpp"
#include <fstream>
#include <algorithm>
#include <iostream>
#include <cmath>

namespace CircuitSimEngine {

static void getWeights(double x, const std::vector<double>& axis, size_t& idx0, size_t& idx1, double& frac) {
    if (axis.empty()) {
        idx0 = 0;
        idx1 = 0;
        frac = 0.0;
        return;
    }
    if (axis.size() == 1) {
        idx0 = 0;
        idx1 = 0;
        frac = 0.0;
        return;
    }
    
    if (x <= axis.front()) {
        idx0 = 0;
        idx1 = 1;
        double denom = axis[1] - axis[0];
        frac = (std::abs(denom) > 1e-12) ? (x - axis[0]) / denom : 0.0;
    } else if (x >= axis.back()) {
        idx0 = axis.size() - 2;
        idx1 = axis.size() - 1;
        double denom = axis[idx1] - axis[idx0];
        frac = (std::abs(denom) > 1e-12) ? (x - axis[idx0]) / denom : 0.0;
    } else {
        auto it = std::upper_bound(axis.begin(), axis.end(), x);
        idx1 = std::distance(axis.begin(), it);
        idx0 = (idx1 > 0) ? (idx1 - 1) : 0;
        double denom = axis[idx1] - axis[idx0];
        frac = (std::abs(denom) > 1e-12) ? (x - axis[idx0]) / denom : 0.0;
    }
}

double Interpolator2D::evaluate(double i, double t) const {
    if (!valid || data.empty() || i_axis.empty() || t_axis.empty()) return 0.0;
    
    size_t i0, i1, t0, t1;
    double i_f, t_f;
    getWeights(i, i_axis, i0, i1, i_f);
    getWeights(t, t_axis, t0, t1, t_f);
    
    size_t Nt = t_axis.size();
    
    auto getVal = [&](size_t i_idx, size_t t_idx) -> double {
        size_t idx = i_idx * Nt + t_idx;
        if (idx < data.size()) return data[idx];
        return 0.0;
    };
    
    double v00 = getVal(i0, t0);
    double v01 = getVal(i0, t1);
    double v10 = getVal(i1, t0);
    double v11 = getVal(i1, t1);
    
    double c0 = v00 + i_f * (v10 - v00);
    double c1 = v01 + i_f * (v11 - v01);
    
    return c0 + t_f * (c1 - c0);
}

double Interpolator3D::evaluate(double v, double i, double t) const {
    if (!valid || data.empty() || v_axis.empty() || i_axis.empty() || t_axis.empty()) return 0.0;
    
    size_t v0, v1, i0, i1, t0, t1;
    double v_f, i_f, t_f;
    getWeights(v, v_axis, v0, v1, v_f);
    getWeights(i, i_axis, i0, i1, i_f);
    getWeights(t, t_axis, t0, t1, t_f);
    
    size_t Ni = i_axis.size();
    size_t Nt = t_axis.size();
    
    auto getVal = [&](size_t v_idx, size_t i_idx, size_t t_idx) -> double {
        size_t idx = v_idx * (Ni * Nt) + i_idx * Nt + t_idx;
        if (idx < data.size()) return data[idx];
        return 0.0;
    };
    
    double c00 = getVal(v0, i0, t0) * (1.0 - v_f) + getVal(v1, i0, t0) * v_f;
    double c01 = getVal(v0, i0, t1) * (1.0 - v_f) + getVal(v1, i0, t1) * v_f;
    double c10 = getVal(v0, i1, t0) * (1.0 - v_f) + getVal(v1, i1, t0) * v_f;
    double c11 = getVal(v0, i1, t1) * (1.0 - v_f) + getVal(v1, i1, t1) * v_f;
    
    double c0 = c00 + i_f * (c10 - c00);
    double c1 = c01 + i_f * (c11 - c01);
    
    return c0 + t_f * (c1 - c0);
}

LossModel::LossModel(const std::string& filepath) {
    this->filepath = filepath;
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

            for (auto& el : meta.items()) {
                if (el.value().is_string()) {
                    metadataMap[el.key()] = el.value().get<std::string>();
                } else {
                    metadataMap[el.key()] = el.value().dump();
                }
            }
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
        if (!block.contains("v_axis") || !block.contains("i_axis") || !block.contains("t_axis") || !block.contains("data")) {
            interp.valid = false;
            return;
        }
        interp.v_axis = block["v_axis"].get<std::vector<double>>();
        interp.i_axis = block["i_axis"].get<std::vector<double>>();
        interp.t_axis = block["t_axis"].get<std::vector<double>>();
        
        auto data_json = block["data"];
        if (!data_json.is_array()) { interp.valid = false; return; }

        size_t Nv = interp.v_axis.size();
        size_t Ni = interp.i_axis.size();
        size_t Nt = interp.t_axis.size();
        
        interp.data.clear();
        interp.data.reserve(Nv * Ni * Nt);
        for (size_t v = 0; v < Nv && v < data_json.size(); ++v) {
            const auto& row_v = data_json[v];
            if (!row_v.is_array()) continue;
            for (size_t i = 0; i < Ni && i < row_v.size(); ++i) {
                const auto& row_i = row_v[i];
                if (!row_i.is_array()) continue;
                for (size_t t = 0; t < Nt && t < row_i.size(); ++t) {
                    interp.data.push_back(row_i[t].get<double>());
                }
            }
        }
        interp.valid = (interp.data.size() == (Nv * Ni * Nt) && !interp.data.empty());
    } catch (...) { interp.valid = false; }
}

void LossModel::parseBlock2D(const nlohmann::json& block, Interpolator2D& interp) {
    try {
        if (!block.contains("i_axis") || !block.contains("t_axis") || !block.contains("data")) {
            interp.valid = false;
            return;
        }
        interp.i_axis = block["i_axis"].get<std::vector<double>>();
        interp.t_axis = block["t_axis"].get<std::vector<double>>();
        
        auto data_json = block["data"];
        if (!data_json.is_array()) { interp.valid = false; return; }

        size_t Ni = interp.i_axis.size();
        size_t Nt = interp.t_axis.size();
        
        interp.data.clear();
        interp.data.reserve(Ni * Nt);
        for (size_t i = 0; i < Ni && i < data_json.size(); ++i) {
            const auto& row_i = data_json[i];
            if (!row_i.is_array()) continue;
            for (size_t t = 0; t < Nt && t < row_i.size(); ++t) {
                interp.data.push_back(row_i[t].get<double>());
            }
        }
        interp.valid = (interp.data.size() == (Ni * Nt) && !interp.data.empty());
    } catch (...) { interp.valid = false; }
}

double LossModel::getTurnOnEnergy(double v, double i, double t) const {
    return turn_on_interp.evaluate(v, i, t);
}

double LossModel::getTurnOffEnergy(double v, double i, double t) const {
    return turn_off_interp.evaluate(v, i, t);
}

bool LossModel::hasConductionTable(bool gate_on) const {
    return gate_on ? cond_on_interp.valid : cond_off_interp.valid;
}

double LossModel::getConductionVoltageDrop(double i, double t, bool gate_on) const {
    const auto& interp = gate_on ? cond_on_interp : cond_off_interp;
    if (!interp.valid) return 0.0;
    
    // First try standard positive evaluation
    double val = interp.evaluate(std::abs(i), t);
    
    // If returned zero or negligible, check if table i_axis is negative-indexed (PLECS body diode convention)
    if (std::abs(val) < 1e-4 && !interp.i_axis.empty() && interp.i_axis.front() < 0.0) {
        val = interp.evaluate(-std::abs(i), t);
    }
    return std::abs(val);
}

} // namespace CircuitSimEngine
