// FROZEN ORACLE: the property-inspector dropdown rules exactly as they were in
// ui/MainWindow.cpp renderPropertyInspector() before the block registry's
// "choices" field replaced them. `t` is the upper-cased block type, `pKey` the
// lower-cased parameter name. Do not edit.
#pragma once
#include <string>
#include <vector>
namespace golden_ref_inspector {
inline std::vector<std::string> legacyChoices(const std::string& t, const std::string& pKey) {
        std::vector<std::string> comboChoices;


        if (pKey == "function" || pKey == "func" || pKey == "fcn" || pKey == "expression") {
            if (t == "ROUND") {
                comboChoices = {"round", "floor", "ceil", "fix"};
            } else if (t == "MIN_MAX" || t == "MIN" || t == "MAX") {
                comboChoices = {"min", "max"};
            } else if (t == "TRIG_FCN" || t == "TRIG" || t == "FCN" || t == "MATH_FCN" || t == "MATH_FUNC" || t == "MATH") {
                comboChoices = {"sin", "cos", "tan", "asin", "acos", "atan", "atan2", "sinh", "cosh", "tanh", "exp", "log", "log10", "sqrt", "abs", "square", "pow", "reciprocal"};
            } else if (t == "LOGIC_OP" || t == "COMB_LOGIC") {
                comboChoices = {"AND", "OR", "NAND", "NOR", "XOR", "NXOR", "NOT"};
            }
        } else if (pKey == "operator" || pKey == "op" || pKey == "relop" || pKey == "relational_operator" || pKey == "condition" || pKey == "comparison") {
            if (t == "RELATIONAL_OPERATOR" || t == "COMP" || t == "COMPARE" || t == "COMP_CONST" || t == "COMPARE_TO_CONSTANT" || t == "COMPARE_TO_ZERO" || t == "Comparator" || t == "CompareToConstant" || t == "CompareToZero") {
                comboChoices = {"==", "~=", "!=", "<", "<=", ">", ">="};
            } else if (t == "BITWISE_OP") {
                comboChoices = {"AND", "OR", "XOR", "NOT", "SHIFT_LEFT", "SHIFT_RIGHT"};
            } else if (t == "LOGIC_OP" || t == "COMB_LOGIC") {
                comboChoices = {"AND", "OR", "NAND", "NOR", "XOR", "NXOR", "NOT"};
            }
        } else if (pKey == "edgetype" || pKey == "edge" || pKey == "edge_type" || pKey == "direction" || pKey == "hit_direction" || pKey == "dir" || pKey == "detection_mode" || pKey == "type" || pKey == "mode") {
            if (t == "EDGE_DETECT") {
                comboChoices = {"rising", "falling", "either"};
            } else if (t == "HIT_CROSSING") {
                comboChoices = {"either", "rising", "falling"};
            }
        } else if (pKey == "datatype" || pKey == "output_type") {
            if (t == "DATATYPE_CONV") {
                comboChoices = {"double", "float", "int32", "int16", "int8", "uint32", "uint16", "uint8", "boolean"};
            }
        } else if (pKey == "filter_type" || (pKey == "type" && (t == "FILTER_1ST" || t == "FILTER_2ND"))) {
            comboChoices = {"lowpass", "highpass", "bandpass", "bandstop"};
        } else if (pKey == "alignment" || pKey == "mode" || pKey == "rounding_mode") {
            if (t == "ROUND") {
                comboChoices = {"nearest", "floor", "ceil", "fix"};
            } else if (t == "PWM" || t == "PWM_3PH" || t == "SVPWM") {
                comboChoices = {"edge", "center", "symmetric", "asymmetric"};
            } else if (t == "FOURIER_ANALYSIS" || t == "FOURIER_TRANS") {
                comboChoices = {"magnitude_phase", "real_imag", "harmonic_series"};
            }
        }
        return comboChoices;
}
inline std::vector<std::string> legacyKeys() {
    return {"function", "func", "fcn", "expression", "operator", "op", "relop", "relational_operator",
            "condition", "comparison", "edgetype", "edge", "edge_type", "direction", "hit_direction", "dir",
            "detection_mode", "type", "mode", "datatype", "output_type", "filter_type", "alignment",
            "rounding_mode"};
}
} // namespace golden_ref_inspector
