#include "NetlistParser.hpp"
#include <nlohmann/json.hpp>
#include <iostream>
#include <unordered_map>
#include <set>
#include <algorithm>

using json = nlohmann::json;

namespace CircuitSimEngine {

static std::vector<double> parseTurnsArrayStr(const std::string& str) {
    std::vector<double> res;
    if (str.empty()) { res.push_back(100.0); return res; }
    std::string clean = str;
    clean.erase(std::remove(clean.begin(), clean.end(), '['), clean.end());
    clean.erase(std::remove(clean.begin(), clean.end(), ']'), clean.end());
    std::replace(clean.begin(), clean.end(), ',', ' ');
    std::stringstream ss(clean);
    double val;
    while (ss >> val) {
        res.push_back(val);
    }
    if (res.empty()) res.push_back(100.0);
    return res;
}

static ComponentType stringToComponentType(const std::string& typeStr, const std::string& fallbackCategory = "") {
    std::string t = typeStr;
    if (t.empty()) t = fallbackCategory;

    if (t == "Resistor" || t == "R" || t == "resistors" || t == "VariableResistor") return ComponentType::Resistor;
    if (t == "Capacitor" || t == "C" || t == "capacitors") return ComponentType::Capacitor;
    if (t == "Inductor" || t == "L" || t == "inductors") return ComponentType::Inductor;
    if (t == "VoltageSource" || t == "V" || t == "DCVoltageSource" || t == "voltage_sources" || t == "dc_sources" || t == "DC_V") return ComponentType::VoltageSource;
    if (t == "ACVoltageSource" || t == "ac_sources" || t == "AC_V" || t == "ac" || t == "AC") return ComponentType::ACVoltageSource;
    if (t == "ControlledVoltageSource" || t == "CTRL_V") return ComponentType::ControlledVoltageSource;
    if (t == "ThreePhaseSource" || t == "V_3PH") return ComponentType::ThreePhaseSource;
    if (t == "CurrentSource" || t == "I" || t == "current_sources" || t == "DC_I") return ComponentType::CurrentSource;
    if (t == "ACCurrentSource" || t == "AC_I") return ComponentType::ACCurrentSource;
    if (t == "ControlledCurrentSource" || t == "CTRL_I") return ComponentType::ControlledCurrentSource;
    if (t == "ThreePhaseCurrentSource" || t == "I_3PH") return ComponentType::ThreePhaseCurrentSource;
    if (t == "ElectricalPort" || t == "E_PORT") return ComponentType::ElectricalPort;
    if (t == "ElectricalLabel" || t == "E_LABEL") return ComponentType::ElectricalLabel;
    if (t == "VM_3PH" || t == "Voltmeter3Ph") return ComponentType::Voltmeter3Ph;
    if (t == "AM_3PH" || t == "Ammeter3Ph") return ComponentType::Ammeter3Ph;
    if (t == "VAR_R" || t == "VariableResistor") return ComponentType::VariableResistor;
    if (t == "VAR_L" || t == "VariableInductor") return ComponentType::VariableInductor;
    if (t == "VAR_C" || t == "VariableCapacitor") return ComponentType::VariableCapacitor;
    if (t == "SAT_L" || t == "SaturableInductor") return ComponentType::SaturableInductor;
    if (t == "SAT_C" || t == "SaturableCapacitor") return ComponentType::SaturableCapacitor;
    if (t == "PI_SECTION" || t == "PiSectionLine") return ComponentType::PiSectionLine;
    if (t == "LINE_3PH" || t == "TransmissionLine3Ph") return ComponentType::TransmissionLine3Ph;
    if (t == "PWL_R" || t == "PWLResistor") return ComponentType::PWLResistor;
    if (t == "E_ALGEBRAIC" || t == "ElectricalAlgebraic") return ComponentType::ElectricalAlgebraic;
    if (t == "THYRISTOR" || t == "SCR" || t == "Thyristor") return ComponentType::Thyristor;
    if (t == "GTO") return ComponentType::GTO;
    if (t == "IGBT_DIODE" || t == "IGBTDiode") return ComponentType::IGBTDiode;
    if (t == "IGCT") return ComponentType::IGCT;
    if (t == "BJT") return ComponentType::BJT;
    if (t == "JFET") return ComponentType::JFET;
    if (t == "BREAKER" || t == "Breaker") return ComponentType::Breaker;
    if (t == "DBL_SWITCH" || t == "DoubleSwitch") return ComponentType::DoubleSwitch;
    if (t == "MAN_SWITCH" || t == "ManualSwitch") return ComponentType::ElectricalManualSwitch;
    if (t == "MAN_DBL_SWITCH" || t == "ManualDoubleSwitch") return ComponentType::ManualDoubleSwitch;
    if (t == "MAN_TRPL_SWITCH" || t == "ManualTripleSwitch") return ComponentType::ManualTripleSwitch;
    if (t == "SR_SWITCH" || t == "SRSwitch") return ComponentType::SRSwitch;
    if (t == "TRPL_SWITCH" || t == "TripleSwitch") return ComponentType::TripleSwitch;
    if (t == "IDEAL_XFMR" || t == "IdealTransformer") return ComponentType::IdealTransformer;
    if (t == "XFMR_2W" || t == "Transformer2W") return ComponentType::Transformer2W;
    if (t == "XFMR_3W" || t == "Transformer3W") return ComponentType::Transformer3W;
    if (t == "MUTUAL_2W" || t == "MutualInductor2W") return ComponentType::MutualInductor2W;
    if (t == "MUTUAL_3W" || t == "MutualInductor3W") return ComponentType::MutualInductor3W;
    if (t == "SAT_XFMR" || t == "SaturableTransformer") return ComponentType::SaturableTransformer;
    if (t == "XFMR_3PH_2W" || t == "Transformer3Ph2W") return ComponentType::Transformer3Ph2W;
    if (t == "XFMR_3PH_3W" || t == "Transformer3Ph3W") return ComponentType::Transformer3Ph3W;
    if (t == "INDUCTION_MOTOR" || t == "IND_MOTOR" || t == "InductionMotor") return ComponentType::InductionMotor;
    if (t == "GOTO_SIG" || t == "GOTO" || t == "GotoSignal") return ComponentType::GotoSignal;
    if (t == "FROM_SIG" || t == "FROM" || t == "FromSignal") return ComponentType::FromSignal;
    if (t == "MOSFET" || t == "vg-FET" || t == "VGFET" || t == "Mosfet" || t == "MOSFET_DIODE" || t == "IGBT" || t == "IGBT_DIODE" || t == "mosfets") return ComponentType::MOSFET;
    if (t == "Diode" || t == "D" || t == "diodes") return ComponentType::Diode;
    if (t == "Switch" || t == "S" || t == "switches" || t == "analog_switches") return ComponentType::Switch;
    if (t == "Voltmeter" || t == "VM" || t == "voltmeters") return ComponentType::Voltmeter;
    if (t == "Ammeter" || t == "AM" || t == "ammeters") return ComponentType::Ammeter;
    if (t == "UnifiedProbe" || t == "PROBE" || t == "probes") return ComponentType::UnifiedProbe;
    if (t == "Oscilloscope" || t == "SCOPE") return ComponentType::Oscilloscope;
    if (t == "PulseGenerator" || t == "PULSE" || t == "PULSE_GEN" || t == "pulse_generators") return ComponentType::PulseGenerator;
    if (t == "Constant" || t == "CONST" || t == "constants") return ComponentType::Constant;
    if (t == "Clock" || t == "CLOCK") return ComponentType::Clock;
    if (t == "InitialCondition" || t == "INIT_COND") return ComponentType::InitialCondition;
    if (t == "Ramp" || t == "RAMP") return ComponentType::Ramp;
    if (t == "RandomNumbers" || t == "RANDOM_NUM") return ComponentType::RandomNumbers;
    if (t == "SineWave" || t == "SINE_WAVE") return ComponentType::SineWave;
    if (t == "Step" || t == "STEP") return ComponentType::Step;
    if (t == "Triangle_Carrier" || t == "TRIANGLE" || t == "TRI" || t == "TRI_GEN" || t == "triangle_carriers") return ComponentType::Triangle_Carrier;
    if (t == "WhiteNoise" || t == "WHITE_NOISE") return ComponentType::WhiteNoise;
    if (t == "TrigFunction" || t == "TRIG_FCN") return ComponentType::TrigFunction;
    if (t == "Abs" || t == "ABS") return ComponentType::Abs;
    if (t == "Sign" || t == "SIGN") return ComponentType::Sign;
    if (t == "Round" || t == "ROUND") return ComponentType::Round;
    if (t == "MinMax" || t == "MIN_MAX") return ComponentType::MinMax;
    if (t == "LUT_1D") return ComponentType::LUT_1D;
    if (t == "LUT_2D") return ComponentType::LUT_2D;
    if (t == "LUT_3D") return ComponentType::LUT_3D;
    if (t == "DLL") return ComponentType::DLL;
    if (t == "FMU") return ComponentType::FMU;
    if (t == "FourierSeries" || t == "FOURIER_SERIES") return ComponentType::FourierSeries;
    if (t == "Integrator" || t == "INTEGRATOR" || t == "integrators") return ComponentType::Integrator;
    if (t == "Derivative" || t == "DERIVATIVE" || t == "derivatives") return ComponentType::Derivative;
    if (t == "TransferFunction" || t == "TRANSFER_FCN" || t == "transfer_functions") return ComponentType::TransferFunction;
    if (t == "StateSpace" || t == "STATE_SPACE" || t == "state_space_blocks") return ComponentType::StateSpace;
    if (t == "ContinuousPID" || t == "CONT_PID" || t == "PID" || t == "DISCRETE_PID" || t == "pid_controllers") return ComponentType::ContinuousPID;
    if (t == "PLL_1PH") return ComponentType::PLL_1PH;
    if (t == "PLL_3PH") return ComponentType::PLL_3PH;
    if (t == "Delay" || t == "DELAY") return ComponentType::Delay;
    if (t == "TransportDelay" || t == "TRANSPORT_DELAY") return ComponentType::TransportDelay;
    if (t == "TurnOnDelay" || t == "TURN_ON_DELAY") return ComponentType::TurnOnDelay;
    if (t == "MemoryBlock" || t == "MEMORY" || t == "MEMORY_BLOCK") return ComponentType::MemoryBlock;
    if (t == "Quantizer" || t == "QUANTIZER") return ComponentType::Quantizer;
    if (t == "SignalSwitch" || t == "SIGNAL_SWITCH") return ComponentType::SignalSwitch;
    if (t == "ManualSwitch" || t == "MANUAL_SWITCH") return ComponentType::ManualSwitch;
    if (t == "MultiportSwitch" || t == "MULTIPORT_SWITCH") return ComponentType::MultiportSwitch;
    if (t == "HitCrossing" || t == "HIT_CROSSING") return ComponentType::HitCrossing;
    if (t == "Saturation" || t == "SATURATION") return ComponentType::Saturation;
    if (t == "DeadZone" || t == "DEAD_ZONE") return ComponentType::DeadZone;
    if (t == "RateLimiter" || t == "RATE_LIMITER") return ComponentType::RateLimiter;
    if (t == "Relay" || t == "RELAY") return ComponentType::Relay;
    if (t == "Comparator" || t == "COMP" || t == "comparators") return ComponentType::Comparator;
    if (t == "LogicOp" || t == "LOGIC_OP" || t == "logic_gates") return ComponentType::LogicOp;
    if (t == "NOT" || t == "not" || t == "NOT_OP" || t == "NOT_Gate" || t == "Not") return ComponentType::NOT_Gate;
    if (t == "AND" || t == "and" || t == "AND_OP" || t == "AND_Gate" || t == "And") return ComponentType::AND_Gate;
    if (t == "OR" || t == "or" || t == "OR_OP" || t == "OR_Gate" || t == "Or") return ComponentType::OR_Gate;
    if (t == "NAND" || t == "nand" || t == "NAND_OP" || t == "NAND_Gate") return ComponentType::LogicOp;
    if (t == "NOR" || t == "nor" || t == "NOR_OP" || t == "NOR_Gate") return ComponentType::LogicOp;
    if (t == "XOR" || t == "xor" || t == "XOR_OP" || t == "XOR_Gate") return ComponentType::LogicOp;
    if (t == "XNOR" || t == "xnor" || t == "NXOR" || t == "nxor" || t == "XNOR_OP" || t == "XNOR_Gate") return ComponentType::LogicOp;
    if (t == "BitwiseOp" || t == "BITWISE_OP") return ComponentType::BitwiseOp;
    if (t == "CombLogic" || t == "COMB_LOGIC") return ComponentType::CombLogic;
    if (t == "EdgeDetect" || t == "EDGE_DETECT") return ComponentType::EdgeDetect;
    if (t == "Monostable" || t == "MONOSTABLE") return ComponentType::Monostable;
    if (t == "Monoflop" || t == "MONOFLOP") return ComponentType::Monoflop;
    if (t == "RelationalOp" || t == "RELATIONAL_OPERATOR") return ComponentType::RelationalOp;
    if (t == "CompareToConstant" || t == "COMPARE_TO_CONSTANT" || t == "COMP_CONST") return ComponentType::CompareToConstant;
    if (t == "DFlipFlop" || t == "D_FLIP_FLOP") return ComponentType::DFlipFlop;
    if (t == "JKFlipFlop" || t == "JK_FLIP_FLOP") return ComponentType::JKFlipFlop;
    if (t == "ShiftReg" || t == "SHIFT_REG") return ComponentType::ShiftReg;
    if (t == "PWM_3PH") return ComponentType::PWM_3PH;
    if (t == "SVPWM") return ComponentType::SVPWM;
    if (t == "CLARKE" || t == "Clarke" || t == "CLARKE_TRANSFORM" || t == "CLARKE_TRANS") return ComponentType::Clarke;
    if (t == "PARK" || t == "Park" || t == "PARK_TRANSFORM" || t == "PARK_TRANS") return ComponentType::Park;
    if (t == "INV_CLARKE" || t == "InvClarke" || t == "INV_CLARKE_TRANSFORM" || t == "INV_CLARKE_TRANS") return ComponentType::InvClarke;
    if (t == "INV_PARK" || t == "InvPark" || t == "INV_PARK_TRANSFORM" || t == "INV_PARK_TRANS") return ComponentType::InvPark;
    if (t == "DQ_TO_ABC" || t == "DQ_ABC" || t == "INV_PARK_3PH" || t == "PARK_INV_3PH" || t == "DqToAbc") return ComponentType::DqToAbc;
    if (t == "ABC_TO_DQ" || t == "ABC_DQ" || t == "PARK_3PH" || t == "PARK_3PHASE" || t == "AbcToDq") return ComponentType::AbcToDq;
    if (t == "PER_AVG" || t == "PerAvg") return ComponentType::PerAvg;
    if (t == "PERIODIC_IMP_AVG" || t == "PeriodicImpAvg") return ComponentType::PeriodicImpAvg;
    if (t == "FOURIER_TRANS" || t == "FourierTrans") return ComponentType::FourierTrans;
    if (t == "MOV_AVG" || t == "MovAvg") return ComponentType::MovAvg;
    if (t == "FILTER_1ST" || t == "Filter1st") return ComponentType::Filter1st;
    if (t == "FILTER_2ND" || t == "Filter2nd") return ComponentType::Filter2nd;
    if (t == "FOURIER_ANALYSIS" || t == "FourierAnalysis") return ComponentType::FourierAnalysis;
    if (t == "RMS_VAL" || t == "RmsVal") return ComponentType::RmsVal;
    if (t == "THD_VAL" || t == "ThdVal") return ComponentType::ThdVal;
    if (t == "PLL_LOOP" || t == "PllLoop") return ComponentType::PllLoop;
    if (t == "OFFSET" || t == "Offset") return ComponentType::Offset;
    if (t == "SIGNUM" || t == "Signum" || t == "SIGN" || t == "Sign" || t == "SGN") return ComponentType::Signum;
    if (t == "DIVIDE" || t == "Divide") return ComponentType::Divide;
    if (t == "DATATYPE_CONV" || t == "DataTypeConv") return ComponentType::DataTypeConv;
    if (t == "STATE_MACHINE" || t == "StateMachine") return ComponentType::StateMachine;
    if (t == "Gain" || t == "GAIN" || t == "gains") return ComponentType::Gain;
    if (t == "SummingJunction" || t == "SUM" || t == "SUM_RECT" || t == "SUM_ROUND" || t == "SUBTRACT" || t == "SUB" || t == "summing_junctions") return ComponentType::SummingJunction;
    if (t == "Product" || t == "PRODUCT" || t == "PROD" || t == "PRODUCT_RECT" || t == "product_blocks") return ComponentType::Product;
    if (t == "PWM_MASTER" || t == "Master_PWM" || t == "PwmMaster" || t == "PWM_Master" || t == "pwm_masters") return ComponentType::PWM_MASTER;
    if (t == "PWM_Generator" || t == "PWM" || t == "pwm_generators") return ComponentType::PWM_Generator;
    if (t == "Triangle_Carrier" || t == "TRIANGLE" || t == "TRI" || t == "TRI_GEN" || t == "triangle_carriers") return ComponentType::Triangle_Carrier;
    if (t == "PI_Controller" || t == "PI" || t == "PID" || t == "CONT_PID" || t == "DISCRETE_PID" || t == "pi_controllers" || t == "pid_controllers") return ComponentType::ContinuousPID;
    if (t == "Comparator" || t == "COMP" || t == "E_COMP" || t == "comparators") return ComponentType::Comparator;
    if (t == "CustomScript" || t == "CSCRIPT" || t == "custom_scripts") return ComponentType::CustomScript;
    if (t == "INPORT" || t == "IN" || t == "ports") return ComponentType::Inport;
    if (t == "OUTPORT" || t == "OUT") return ComponentType::Outport;
    if (t == "PHYSICAL_INPORT" || t == "PIN") return ComponentType::PhysicalInport;
    if (t == "PHYSICAL_OUTPORT" || t == "POUT") return ComponentType::PhysicalOutport;
    if (t == "ENABLE_PORT") return ComponentType::EnablePort;
    if (t == "TRIGGER_PORT") return ComponentType::TriggerPort;
    if (t == "BUS_CREATOR" || t == "buses") return ComponentType::BusCreator;
    if (t == "BUS_SELECTOR") return ComponentType::BusSelector;
    if (t == "TERMINATOR" || t == "terminators") return ComponentType::Terminator;
    if (t == "POLYNOMIAL" || t == "functions") return ComponentType::Polynomial;
    if (t == "MATH_FCN" || t == "MathFcn" || t == "MATH_FUNCTION" || t == "MathFunction") return ComponentType::MathFunction;
    if (t == "ALGEBRAIC_CONSTRAINT") return ComponentType::AlgebraicConstraint;
    if (t == "XFMR" || t == "Transformer" || t == "transformers") return ComponentType::Transformer;
    return ComponentType::Unknown;
}

static void parseComponentItem(const json& item, const std::string& defaultCategoryType, std::vector<ComponentModel>& outList) {
    if (!item.is_object()) return;
    ComponentModel comp;
    if (item.contains("id")) comp.id = item["id"].get<std::string>();
    
    std::string itemType = item.contains("type") ? item["type"].get<std::string>() : defaultCategoryType;
    if (item.contains("original_type") && item["original_type"].is_string()) {
        std::string origType = item["original_type"].get<std::string>();
        if (!origType.empty()) itemType = origType;
    }
    // Detect AC voltage source via src_type field (web-tool exports "src_type":"ac")
    if (item.contains("src_type") && item["src_type"].is_string()) {
        std::string srcType = item["src_type"].get<std::string>();
        if (srcType == "ac" || srcType == "AC") itemType = "AC_V";
    }
    // Also detect AC via top-level type field "ac" when inside voltage_sources category
    if ((itemType == "ac" || itemType == "AC") ||
        (defaultCategoryType == "voltage_sources" && (itemType == "ac" || itemType == "AC"))) {
        itemType = "AC_V";
    }
    comp.type = stringToComponentType(itemType, defaultCategoryType);
    if (item.contains("label")) comp.label = item["label"].get<std::string>();

    if (item.contains("nodes") && item["nodes"].is_array()) {
        for (const auto& n : item["nodes"]) {
            if (n.is_string()) comp.nodes.push_back(n.get<std::string>());
            else if (n.is_number()) comp.nodes.push_back(std::to_string(n.get<int>()));
        }
    }

    if (item.contains("primary_windings") && item["primary_windings"].is_array()) {
        std::string pTurnsStr = "[";
        for (const auto& w : item["primary_windings"]) {
            if (w.contains("nodes") && w["nodes"].is_array()) {
                for (const auto& n : w["nodes"]) {
                    if (n.is_string()) comp.nodes.push_back(n.get<std::string>());
                }
            }
            if (w.contains("turns")) {
                if (w["turns"].is_number()) pTurnsStr += std::to_string(w["turns"].get<double>()) + " ";
                else if (w["turns"].is_string()) pTurnsStr += w["turns"].get<std::string>() + " ";
            }
        }
        pTurnsStr += "]";
        comp.parameters["primary_turns"] = pTurnsStr;
    }

    if (item.contains("secondary_windings") && item["secondary_windings"].is_array()) {
        std::string sTurnsStr = "[";
        for (const auto& w : item["secondary_windings"]) {
            if (w.contains("nodes") && w["nodes"].is_array()) {
                for (const auto& n : w["nodes"]) {
                    if (n.is_string()) comp.nodes.push_back(n.get<std::string>());
                }
            }
            if (w.contains("turns")) {
                if (w["turns"].is_number()) sTurnsStr += std::to_string(w["turns"].get<double>()) + " ";
                else if (w["turns"].is_string()) sTurnsStr += w["turns"].get<std::string>() + " ";
            }
        }
        sTurnsStr += "]";
        comp.parameters["secondary_turns"] = sTurnsStr;
    }

    if (item.contains("channels") && item["channels"].is_object()) {
        int inCount = 0;
        int outCount = 0;
        for (auto& [chKey, chVal] : item["channels"].items()) {
            if (chVal.is_string()) {
                std::string sVal = chVal.get<std::string>();
                if (chKey == "In" || chKey.rfind("In", 0) == 0 || chKey == "Plus" || chKey == "Minus" || chKey == "Ctrl") {
                    comp.parameters["input_" + std::to_string(inCount++)] = sVal;
                    comp.parameters[chKey] = sVal;
                } else if (chKey == "Out" || chKey.rfind("Out", 0) == 0) {
                    comp.parameters["output_" + std::to_string(outCount++)] = sVal;
                    comp.parameters[chKey] = sVal;
                }
            }
        }
    }

    std::string itemTypeLower = itemType;
    std::transform(itemTypeLower.begin(), itemTypeLower.end(), itemTypeLower.begin(), ::toupper);
    if (itemTypeLower == "NAND") comp.parameters["operator"] = "NAND";
    else if (itemTypeLower == "NOR") comp.parameters["operator"] = "NOR";
    else if (itemTypeLower == "XOR") comp.parameters["operator"] = "XOR";
    else if (itemTypeLower == "XNOR" || itemTypeLower == "NXOR") comp.parameters["operator"] = "XNOR";
    else if (itemTypeLower == "NOT") comp.parameters["operator"] = "NOT";
    else if (itemTypeLower == "AND") comp.parameters["operator"] = "AND";
    else if (itemTypeLower == "OR") comp.parameters["operator"] = "OR";

    if (item.contains("inputs") && item["inputs"].is_array()) {
        int idx = 0;
        for (const auto& inp : item["inputs"]) {
            if (inp.is_string()) {
                std::string sVal = inp.get<std::string>();
                comp.parameters["input_" + std::to_string(idx)] = sVal;
                comp.parameters["In" + std::to_string(idx + 1)] = sVal;
                if (idx == 0) {
                    comp.parameters["In"] = sVal;
                    comp.parameters["input"] = sVal;
                    comp.parameters["input_a"] = sVal;
                }
                if (idx == 1) {
                    comp.parameters["input_b"] = sVal;
                }
                idx++;
            }
        }
    }

    if (item.contains("outputs") && item["outputs"].is_array()) {
        int idx = 0;
        for (const auto& out : item["outputs"]) {
            if (out.is_string()) {
                comp.parameters["output_" + std::to_string(idx)] = out.get<std::string>();
                idx++;
            }
        }
    }

    if (item.contains("output") && item["output"].is_string()) {
        comp.parameters["output"] = item["output"].get<std::string>();
    }

    if (item.contains("control_signal") && item["control_signal"].is_string()) {
        comp.parameters["control_signal"] = item["control_signal"].get<std::string>();
    }

    auto parseParamsMap = [&](const json& pObj) {
        for (auto& [k, v] : pObj.items()) {
            std::string valStr = v.is_string() ? v.get<std::string>() : (v.is_number() ? std::to_string(v.get<double>()) : (v.is_boolean() ? (v.get<bool>() ? "true" : "false") : v.dump()));
            comp.parameters[k] = valStr;

            // Alias mappings for parameter key discrepancies
            if (k == "start_of_dead_zone") comp.parameters["start"] = valStr;
            else if (k == "end_of_dead_zone") comp.parameters["end"] = valStr;
            else if (k == "lower_limit") comp.parameters["min"] = valStr;
            else if (k == "upper_limit") comp.parameters["max"] = valStr;
            else if (k == "cutoff_freq") comp.parameters["fc"] = valStr;
            else if (k == "damping") comp.parameters["zeta"] = valStr;
            else if (k == "pulse_duration") comp.parameters["duration"] = valStr;
        }
    };
    if (item.contains("parameters") && item["parameters"].is_object()) parseParamsMap(item["parameters"]);
    if (item.contains("params") && item["params"].is_object()) parseParamsMap(item["params"]);

    for (auto& [k, v] : item.items()) {
        if (k == "id" || k == "nodes" || k == "type" || k == "parameters" || k == "label" || k == "inputs" || k == "outputs") continue;
        std::string valStr = v.is_string() ? v.get<std::string>() : (v.is_number() ? std::to_string(v.get<double>()) : (v.is_boolean() ? (v.get<bool>() ? "true" : "false") : v.dump()));
        comp.parameters[k] = valStr;

        if (k == "start_of_dead_zone") comp.parameters["start"] = valStr;
        else if (k == "end_of_dead_zone") comp.parameters["end"] = valStr;
        else if (k == "lower_limit") comp.parameters["min"] = valStr;
        else if (k == "upper_limit") comp.parameters["max"] = valStr;
        else if (k == "cutoff_freq") comp.parameters["fc"] = valStr;
        else if (k == "damping") comp.parameters["zeta"] = valStr;
        else if (k == "pulse_duration") comp.parameters["duration"] = valStr;
    }

    outList.push_back(comp);
}

// Disjoint set helper for raw schematic wire graph discovery
struct DisjointSet {
    std::unordered_map<std::string, std::string> parent;

    std::string find(const std::string& i) {
        if (parent.find(i) == parent.end()) parent[i] = i;
        if (parent[i] == i) return i;
        return parent[i] = find(parent[i]);
    }

    void unite(const std::string& i, const std::string& j) {
        std::string rootI = find(i);
        std::string rootJ = find(j);
        if (rootI != rootJ) parent[rootI] = rootJ;
    }
};

static std::string endpointToPinKey(const json& ep) {
    if (!ep.is_object()) return "";
    std::string cId = ep.contains("compId") ? ep["compId"].get<std::string>() : (ep.contains("comp") ? ep["comp"].get<std::string>() : "");
    std::string term = ep.contains("terminal") ? ep["terminal"].get<std::string>() : (ep.contains("term") ? ep["term"].get<std::string>() : "");
    if (!cId.empty() && !term.empty()) return cId + "." + term;
    if (ep.contains("type") && ep["type"] == "wire") {
        std::string wId = ep.contains("wireId") ? ep["wireId"].get<std::string>() : "";
        return "WIRE:" + wId;
    }
    return "";
}

static void parseWireEndpointKeys(const json& w, std::string& kFrom, std::string& kTo) {
    kFrom = ""; kTo = "";
    if (w.contains("from")) kFrom = endpointToPinKey(w["from"]);
    if (w.contains("to")) kTo = endpointToPinKey(w["to"]);

    if (kFrom.empty()) {
        std::string cId = w.contains("fromComp") ? w["fromComp"].get<std::string>() : "";
        std::string term = w.contains("fromTerm") ? w["fromTerm"].get<std::string>() : (w.contains("fromTerminal") ? w["fromTerminal"].get<std::string>() : "");
        if (!cId.empty() && !term.empty()) kFrom = cId + "." + term;
    }
    if (kTo.empty()) {
        std::string cId = w.contains("toComp") ? w["toComp"].get<std::string>() : "";
        std::string term = w.contains("toTerm") ? w["toTerm"].get<std::string>() : (w.contains("toTerminal") ? w["toTerminal"].get<std::string>() : "");
        if (!cId.empty() && !term.empty()) kTo = cId + "." + term;
    }
}

bool NetlistParser::parseJsonString(const std::string& jsonContent, 
                                std::vector<ComponentModel>& outPhysical, 
                                std::vector<ComponentModel>& outControl, 
                                SimulationConfig& outConfig) {
    try {
        auto root = json::parse(jsonContent);

        // Parse simulation parameters
        if (root.contains("simulation_parameters")) {
            auto simParams = root["simulation_parameters"];
            if (simParams.contains("t_stop")) {
                if (simParams["t_stop"].is_number()) outConfig.stopTime = simParams["t_stop"].get<double>();
                else if (simParams["t_stop"].is_string()) outConfig.stopTime = ExpressionEvaluator::parseScientific(simParams["t_stop"].get<std::string>());
            }
            if (simParams.contains("stop_time")) {
                if (simParams["stop_time"].is_number()) outConfig.stopTime = simParams["stop_time"].get<double>();
                else if (simParams["stop_time"].is_string()) outConfig.stopTime = ExpressionEvaluator::parseScientific(simParams["stop_time"].get<std::string>());
            }
            if (simParams.contains("stopTime")) {
                if (simParams["stopTime"].is_number()) outConfig.stopTime = simParams["stopTime"].get<double>();
                else if (simParams["stopTime"].is_string()) outConfig.stopTime = ExpressionEvaluator::parseScientific(simParams["stopTime"].get<std::string>());
            }
            if (simParams.contains("dt")) {
                if (simParams["dt"].is_number()) outConfig.stepSize = simParams["dt"].get<double>();
                else if (simParams["dt"].is_string()) outConfig.stepSize = ExpressionEvaluator::parseScientific(simParams["dt"].get<std::string>());
            }
            if (simParams.contains("step_size")) {
                if (simParams["step_size"].is_number()) outConfig.stepSize = simParams["step_size"].get<double>();
                else if (simParams["step_size"].is_string()) outConfig.stepSize = ExpressionEvaluator::parseScientific(simParams["step_size"].get<std::string>());
            }
            if (simParams.contains("stepSize")) {
                if (simParams["stepSize"].is_number()) outConfig.stepSize = simParams["stepSize"].get<double>();
                else if (simParams["stepSize"].is_string()) outConfig.stepSize = ExpressionEvaluator::parseScientific(simParams["stepSize"].get<std::string>());
            }
            if (simParams.contains("solver") && simParams["solver"].is_string()) {
                outConfig.solver = simParams["solver"].get<std::string>();
            }
            if (simParams.contains("solverMethod") && simParams["solverMethod"].is_string()) {
                outConfig.solverMethod = simParams["solverMethod"].get<std::string>();
            }
            if (simParams.contains("step_type") && simParams["step_type"].is_string()) {
                outConfig.step_type = simParams["step_type"].get<std::string>();
            }
        }

        bool hasPreExtractedStage = false;
        if (root.contains("physical_stage") && !root["physical_stage"].empty()) hasPreExtractedStage = true;
        if (root.contains("control_loops") && !root["control_loops"].empty()) hasPreExtractedStage = true;

        if (hasPreExtractedStage) {
            if (root.contains("physical_stage")) {
                const auto& stage = root["physical_stage"];
                if (stage.is_array()) {
                    for (const auto& item : stage) parseComponentItem(item, "", outPhysical);
                } else if (stage.is_object()) {
                    for (auto& [catKey, catVal] : stage.items()) {
                        if (catVal.is_array()) {
                            for (const auto& item : catVal) parseComponentItem(item, catKey, outPhysical);
                        }
                    }
                }
            }

            if (root.contains("control_loops")) {
                const auto& ctrl = root["control_loops"];
                if (ctrl.is_array()) {
                    for (const auto& item : ctrl) parseComponentItem(item, "", outControl);
                } else if (ctrl.is_object()) {
                    for (auto& [catKey, catVal] : ctrl.items()) {
                        if (catVal.is_array()) {
                            for (const auto& item : catVal) parseComponentItem(item, catKey, outControl);
                        }
                    }
                }
            }

            if (root.contains("probes") && root["probes"].is_array()) {
                for (const auto& item : root["probes"]) parseComponentItem(item, "probes", outControl);
            }
        } else if (root.contains("components") && root["components"].is_array() && root.contains("wires") && root["wires"].is_array()) {
            // Raw Schematic Export (components + wires) - Run Disjoint-Set Wire Graph Discovery
            std::vector<ComponentModel> rawComps;
            for (const auto& item : root["components"]) {
                parseComponentItem(item, "", rawComps);
            }

            DisjointSet dset;
            for (const auto& w : root["wires"]) {
                std::string kFrom, kTo;
                parseWireEndpointKeys(w, kFrom, kTo);
                if (!kFrom.empty() && !kTo.empty()) {
                    dset.unite(kFrom, kTo);
                }
            }

            // Map pin keys to root representative
            std::unordered_map<std::string, std::string> rootToNodeName;
            int nodeCounter = 1;

            // Identify ground reference pins (terminal B of V1, or GND)
            for (const auto& c : rawComps) {
                if (c.type == ComponentType::VoltageSource) {
                    std::string gndRoot = dset.find(c.id + ".B");
                    if (!gndRoot.empty() && rootToNodeName.find(gndRoot) == rootToNodeName.end()) {
                        rootToNodeName[gndRoot] = "0";
                    }
                }
            }

            auto getNodeNameForPin = [&](const std::string& pinKey) -> std::string {
                std::string r = dset.find(pinKey);
                if (rootToNodeName.find(r) != rootToNodeName.end()) return rootToNodeName[r];
                std::string nName = "node_" + std::to_string(nodeCounter++);
                rootToNodeName[r] = nName;
                return nName;
            };

            // Map GOTO, FROM, and direct wire connections to switch gates & control inputs
            std::unordered_map<std::string, std::string> gotoTagToSignalKey;
            for (const auto& w : root["wires"]) {
                std::string kFrom, kTo;
                parseWireEndpointKeys(w, kFrom, kTo);
                
                for (const auto& c : rawComps) {
                    if (c.parameters.count("tag")) {
                        std::string tag = c.parameters.at("tag");
                        if (kTo == c.id + ".In") {
                            gotoTagToSignalKey[tag] = kFrom;
                        }
                    }
                }

                size_t dotPos = kTo.find('.');
                if (dotPos != std::string::npos) {
                    std::string compId = kTo.substr(0, dotPos);
                    std::string pinName = kTo.substr(dotPos + 1);
                    for (auto& c : rawComps) {
                        if (c.id == compId) {
                            c.parameters[pinName] = kFrom;
                            if (pinName == "A" || pinName == "In1") {
                                c.parameters["In1"] = kFrom; c.parameters["In"] = kFrom; c.parameters["A"] = kFrom; c.parameters["input_0"] = kFrom; c.parameters["input_a"] = kFrom;
                            } else if (pinName == "B" || pinName == "In2") {
                                c.parameters["In2"] = kFrom; c.parameters["B"] = kFrom; c.parameters["input_1"] = kFrom; c.parameters["input_b"] = kFrom;
                            }
                            if (pinName == "G" || pinName == "Ctrl" || pinName == "Gate" || pinName == "Control" || pinName == "control") {
                                c.parameters["control_signal"] = kFrom;
                                c.parameters["Control"] = kFrom;
                                c.parameters["Ctrl"] = kFrom;
                            }
                        }
                    }
                }
            }

            // Build netlist for each component
            for (auto& c : rawComps) {
                if (c.type == ComponentType::Resistor || c.type == ComponentType::Capacitor || 
                    c.type == ComponentType::Inductor || c.type == ComponentType::VoltageSource || 
                    c.type == ComponentType::ACVoltageSource || c.type == ComponentType::CurrentSource || 
                    c.type == ComponentType::Diode || c.type == ComponentType::Switch || 
                    c.type == ComponentType::MOSFET || c.type == ComponentType::BJT || 
                    c.type == ComponentType::JFET || c.type == ComponentType::Thyristor || 
                    c.type == ComponentType::IGBTDiode || c.type == ComponentType::Voltmeter || 
                    c.type == ComponentType::Ammeter) {
                    
                    std::string pA = c.id + ".A";
                    std::string pB = c.id + ".B";
                    if (c.type == ComponentType::Switch || c.type == ComponentType::MOSFET || c.type == ComponentType::IGBTDiode) { 
                        pA = c.id + ".D"; pB = c.id + ".S"; 
                    } else if (c.type == ComponentType::BJT || c.type == ComponentType::JFET) {
                        pA = c.id + ".C"; pB = c.id + ".E";
                    }
                    
                    c.nodes.push_back(getNodeNameForPin(pA));
                    c.nodes.push_back(getNodeNameForPin(pB));

                    if ((c.type == ComponentType::Switch || c.type == ComponentType::MOSFET) && c.parameters.count("Gate_Signal_Label")) {
                        std::string gateTag = c.parameters["Gate_Signal_Label"];
                        if (gotoTagToSignalKey.count(gateTag)) {
                            c.parameters["control_signal"] = gotoTagToSignalKey[gateTag];
                        }
                    }

                    outPhysical.push_back(c);
                }
                else if (c.type == ComponentType::Transformer || c.type == ComponentType::IdealTransformer || 
                         c.type == ComponentType::Transformer2W || c.type == ComponentType::Transformer3W ||
                         c.type == ComponentType::MutualInductor2W || c.type == ComponentType::MutualInductor3W ||
                         c.type == ComponentType::SaturableTransformer || c.type == ComponentType::Transformer3Ph2W || 
                         c.type == ComponentType::Transformer3Ph3W) {
                    if (c.nodes.empty()) {
                        std::string pStr = c.parameters.count("primary_turns") ? c.parameters.at("primary_turns") : "[100]";
                        std::string sStr = c.parameters.count("secondary_turns") ? c.parameters.at("secondary_turns") : "[100]";
                        auto pTurns = parseTurnsArrayStr(pStr);
                        auto sTurns = parseTurnsArrayStr(sStr);

                        for (size_t i = 0; i < pTurns.size(); ++i) {
                            std::string pA = c.id + ".P" + std::to_string(i + 1) + "A";
                            std::string pB = c.id + ".P" + std::to_string(i + 1) + "B";
                            if (pTurns.size() == 1) {
                                std::string rA = dset.find(pA);
                                if (rA.empty()) pA = c.id + ".P1";
                                std::string rB = dset.find(pB);
                                if (rB.empty()) pB = c.id + ".P2";
                            }
                            c.nodes.push_back(getNodeNameForPin(pA));
                            c.nodes.push_back(getNodeNameForPin(pB));
                        }

                        for (size_t j = 0; j < sTurns.size(); ++j) {
                            std::string sA = c.id + ".S" + std::to_string(j + 1) + "A";
                            std::string sB = c.id + ".S" + std::to_string(j + 1) + "B";
                            if (sTurns.size() == 1) {
                                std::string rA = dset.find(sA);
                                if (rA.empty()) sA = c.id + ".S1";
                                std::string rB = dset.find(sB);
                                if (rB.empty()) sB = c.id + ".S2";
                            }
                            c.nodes.push_back(getNodeNameForPin(sA));
                            c.nodes.push_back(getNodeNameForPin(sB));
                        }
                    }

                    outPhysical.push_back(c);
                }
                else {
                    // Control blocks
                    outControl.push_back(c);
                }
            }
        } else if (root.contains("components") && root["components"].is_array()) {
            for (const auto& item : root["components"]) parseComponentItem(item, "", outPhysical);
        }

        return true;
    } catch (const std::exception& e) {
        std::cerr << "Netlist Json Parse Error: " << e.what() << std::endl;
        return false;
    }
}

} // namespace CircuitSimEngine
