#pragma once

#include <vector>
#include <string>
#include <nlohmann/json.hpp>

namespace CircuitSimEngine {

class Interpolator2D {
public:
    std::vector<double> i_axis;
    std::vector<double> t_axis;
    std::vector<double> data; // i_idx * Nt + t_idx

    bool valid = false;

    double evaluate(double i, double t) const;
};

class Interpolator3D {
public:
    std::vector<double> v_axis;
    std::vector<double> i_axis;
    std::vector<double> t_axis;
    std::vector<double> data; // v_idx * (Ni * Nt) + i_idx * Nt + t_idx

    bool valid = false;

    double evaluate(double v, double i, double t) const;
};

class LossModel {
public:
    LossModel() = default;
    LossModel(const std::string& filepath);
    bool isValid() const { return valid; }

    std::string filepath;
    std::string part_number;
    std::string device_type;
    double Qg_nC = 0.0;
    double I_leakage_uA = 0.0;
    double V_drv_V = 15.0;
    std::map<std::string, std::string> metadataMap;

    double getTurnOnEnergy(double v, double i, double t) const;
    double getTurnOffEnergy(double v, double i, double t) const;
    double getConductionVoltageDrop(double i, double t, bool gate_on) const;
    bool hasConductionTable(bool gate_on) const;

    const Interpolator3D& getTurnOnLUT() const { return turn_on_interp; }
    const Interpolator3D& getTurnOffLUT() const { return turn_off_interp; }
    const Interpolator2D& getCondOnLUT() const { return cond_on_interp; }
    const Interpolator2D& getCondOffLUT() const { return cond_off_interp; }

private:
    bool valid = false;
    Interpolator3D turn_on_interp;
    Interpolator3D turn_off_interp;
    Interpolator2D cond_on_interp;
    Interpolator2D cond_off_interp;

    void parseBlock3D(const nlohmann::json& block, Interpolator3D& interp);
    void parseBlock2D(const nlohmann::json& block, Interpolator2D& interp);
};

} // namespace CircuitSimEngine
