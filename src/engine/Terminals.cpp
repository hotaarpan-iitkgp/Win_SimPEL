// Pin layout of every block: name, position relative to the block centre, and the
// outward direction of the pin stub. Shared by the netlist builder (engine) and the
// schematic canvas (UI). Moved verbatim from ui/SchematicCanvas.cpp.
#include "Components.hpp"
#include "CScriptEngine.hpp"
#include "BlockRegistry.hpp"
#include <algorithm>
#include <cmath>
#include <sstream>
#include <string>
#include <vector>

namespace CircuitSim {

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

static void getCSCRIPTPorts(const ComponentInstance& comp, std::vector<CircuitSimEngine::CScriptPort>& inPorts, std::vector<CircuitSimEngine::CScriptPort>& outPorts) {
    std::string code = comp.parameters.count("code") ? comp.parameters.at("code") : "";
    CircuitSimEngine::CScriptEngine::discoverPorts(code, inPorts, outPorts);

    int manualIn = std::max(1, comp.numInputPins);
    if (comp.parameters.count("num_inputs")) {
        try { manualIn = std::max(1, std::stoi(comp.parameters.at("num_inputs"))); } catch (...) {}
    }
    if ((int)inPorts.size() < manualIn) {
        for (int i = (int)inPorts.size(); i < manualIn; ++i) {
            inPorts.push_back({"In" + std::to_string(i + 1), false, i});
        }
    }

    int manualOut = 1;
    if (comp.parameters.count("num_outputs")) {
        try { manualOut = std::max(1, std::stoi(comp.parameters.at("num_outputs"))); } catch (...) {}
    }
    if ((int)outPorts.size() < manualOut) {
        for (int j = (int)outPorts.size(); j < manualOut; ++j) {
            outPorts.push_back({"Out" + std::to_string(j + 1), true, j});
        }
    }
}

static int parseMathBlockPins(const ComponentInstance& comp, std::vector<std::string>& outSigns) {
    outSigns.clear();
    std::string t = comp.rawTypeStr;
    std::transform(t.begin(), t.end(), t.begin(), ::toupper);

    std::string signsStr = comp.parameters.count("signs") ? comp.parameters.at("signs") : "";
    if (signsStr.empty() && comp.parameters.count("inputs")) {
        signsStr = comp.parameters.at("inputs");
    }
    if (signsStr.empty() && comp.parameters.count("num_inputs")) {
        signsStr = comp.parameters.at("num_inputs");
    }

    int nPins = 2;
    bool isNumeric = !signsStr.empty();
    for (char c : signsStr) {
        if (!std::isdigit((unsigned char)c)) { isNumeric = false; break; }
    }

    if (isNumeric) {
        try { nPins = std::clamp(std::stoi(signsStr), 1, 32); } catch (...) { nPins = 2; }
        std::string defaultSign = (t == "PROD" || t == "PRODUCT_RECT") ? "*" : "+";
        for (int i = 0; i < nPins; ++i) outSigns.push_back(defaultSign);
    } else if (!signsStr.empty()) {
        nPins = (int)signsStr.length();
        for (char c : signsStr) {
            outSigns.push_back(std::string(1, c));
        }
    } else {
        nPins = 2;
        std::string defaultSign = (t == "PROD" || t == "PRODUCT_RECT") ? "*" : "+";
        outSigns.assign(nPins, defaultSign);
    }

    return std::max(1, nPins);
}

std::vector<TerminalDef> getTerminals(const ComponentInstance& comp) {
    const std::string& t = comp.rawTypeStr;
    
    if (t == "R" || t == "L" || t == "C" || t == "V" || t == "DC_V" || t == "DC_V1" || t == "VoltageSource" ||
        t == "I" || t == "DC_I" || t == "DC_I1" || t == "CurrentSource" ||
        t == "D" || t == "Diode" || t == "AC_V" || t == "AC_I" ||
        t == "SAT_L" || t == "SAT_C" || t == "PWL_R") {
        return {{"A", 0, -40, 0, -1, false}, {"B", 0, 40, 0, 1, false}};
    }
    if (t == "VAR_R" || t == "VariableResistor" ||
        t == "VAR_L" || t == "VariableInductor" ||
        t == "VAR_C" || t == "VariableCapacitor") {
        // Third pin is the signal input that drives the element value.
        return {{"A", 0, -40, 0, -1, false}, {"B", 0, 40, 0, 1, false}, {"Ctrl", -30, 0, -1, 0, true}};
    }
    if (t == "VM" || t == "AM") {
        return {{"A", 0, -40, 0, -1, false}, {"B", 0, 40, 0, 1, false}, {"Out", 20, 0, 1, 0, true}};
    }
    if (t == "MOSFET" || t == "vg-FET") {
        return {{"D", 0, -40, 0, -1, false}, {"S", 0, 40, 0, 1, false}, {"G", -20, 0, -1, 0, true}};
    }
    if (t == "CTRL_V" || t == "ControlledVoltageSource" || t == "CTRL_I" || t == "ControlledCurrentSource") {
        // Without this entry getTerminals() returned {} for controlled sources, so
        // NetlistBuilder assigned them no nodes at all and they were stamped nowhere.
        return {{"A", 0, -40, 0, -1, false}, {"B", 0, 40, 0, 1, false}, {"Ctrl", -30, 0, -1, 0, true}};
    }
    if (t == "S") {
        return {{"A", 0, -40, 0, -1, false}, {"B", 0, 40, 0, 1, false}, {"Ctrl", -20, 0, -1, 0, true}};
    }
    if (t == "GND") {
        return {{"Gnd", 0, -20, 0, -1, false}};
    }
    if (t == "CONST" || t == "TRI" || t == "TRI_GEN" || t == "PULSE" || t == "PULSE_GEN" || t == "STEP" || t == "RAMP" || t == "SINE_WAVE" || t == "CLOCK" || t == "RANDOM_NUM" || t == "WHITE_NOISE") {
        return {{"Out", 20, 0, 1, 0, true}};
    }
    if (t == "DC_MOTOR") {
        // Electrical armature terminals A and B. TL takes the load torque signal and Speed
        // outputs the shaft speed.
        return {{"A", 0, -40, 0, -1, false}, {"B", 0, 40, 0, 1, false}, {"TL", -30, 0, -1, 0, true}, {"Speed", 30, 0, 1, 0, true}};
    }
    if (t == "LUT_2D") {
        // Two signal inputs: first and second table axis.
        return {{"In", -20, -10, -1, 0, true}, {"In2", -20, 10, -1, 0, true}, {"Out", 20, 0, 1, 0, true}};
    }
    if (t == "LUT_3D") {
        // Three signal inputs: x, y and z table axes.
        return {{"In", -20, -20, -1, 0, true}, {"In2", -20, 0, -1, 0, true}, {"In3", -20, 20, -1, 0, true}, {"Out", 20, 0, 1, 0, true}};
    }
    if (t == "GAIN" || t == "PID" || t == "PWM" || t == "FCN" || t == "MATH_FCN" || t == "POLYNOMIAL" || t == "ALGEBRAIC_CONSTRAINT" || t == "NOT" || t == "INIT_COND" ||
        t == "TRIG_FCN" || t == "ABS" || t == "SIGN" || t == "ROUND" || t == "LUT_1D" || t == "LUT_2D" || t == "LUT_3D" || t == "DLL" || t == "FMU" || t == "FOURIER_SERIES" ||
        t == "INTEGRATOR" || t == "DERIVATIVE" || t == "TRANSFER_FCN" || t == "STATE_SPACE" || t == "CONT_PID" || t == "DISCRETE_PID" || t == "PLL_1PH" ||
        t == "DELAY" || t == "TRANSPORT_DELAY" || t == "TURN_ON_DELAY" || t == "MEMORY_BLOCK" || t == "UNIT_DELAY" || t == "ZOH" || t == "DISCRETE_INT" || t == "DISCRETE_MEAN" || t == "DISCRETE_TF" || t == "DISCRETE_SS" ||
        t == "QUANTIZER" || t == "HIT_CROSSING" || t == "SATURATION" || t == "DEAD_ZONE" || t == "RATE_LIMITER" || t == "RELAY") {
        return {{"In", -20, 0, -1, 0, true}, {"Out", 20, 0, 1, 0, true}};
    }
    if (t == "SIGNAL_SWITCH") {
        return {{"In1", -20, -10, -1, 0, true}, {"Ctrl", 0, -20, 0, -1, true}, {"In2", -20, 10, -1, 0, true}, {"Out", 20, 0, 1, 0, true}};
    }
    if (t == "MANUAL_SWITCH") {
        return {{"In1", -20, -10, -1, 0, true}, {"In2", -20, 10, -1, 0, true}, {"Out", 20, 0, 1, 0, true}};
    }
    if (t == "MULTIPORT_SWITCH") {
        return {{"Ctrl", 0, -20, 0, -1, true}, {"In1", -20, -10, -1, 0, true}, {"In2", -20, 0, -1, 0, true}, {"In3", -20, 10, -1, 0, true}, {"Out", 20, 0, 1, 0, true}};
    }
    if (t == "PLL_3PH") {
        return {{"A", -20, -10, -1, 0, true}, {"B", -20, 0, -1, 0, true}, {"C", -20, 10, -1, 0, true}, {"Theta", 20, -10, 1, 0, true}, {"Freq", 20, 10, 1, 0, true}};
    }
    if (t == "SUM" || t == "SUM_ROUND" || t == "SUM_RECT" || t == "SUBTRACT" || t == "SUB" ||
        t == "PROD" || t == "PRODUCT_RECT" || t == "MIN_MAX" || t == "MIN" || t == "MAX") {
        std::vector<std::string> signs;
        int nInputs = parseMathBlockPins(comp, signs);

        float spacing = 18.0f;
        float totalH = std::max(40.0f, (nInputs - 1) * spacing + 20.0f);
        float halfH = totalH * 0.5f;

        std::vector<TerminalDef> terms;
        for (int i = 0; i < nInputs; ++i) {
            float relY = -halfH + 10.0f + i * spacing;
            std::string pName = "In" + std::to_string(i + 1);
            terms.push_back({pName, -25.0f, relY, -1, 0, true});
        }

        terms.push_back({"Out", 25.0f, 0.0f, 1, 0, true});

        return terms;
    }
    if (t == "COMP" || t == "AND" || t == "OR" || t == "NAND" || t == "NOR" || t == "XOR" || t == "XNOR" ||
        t == "LOGIC_OP" || t == "BITWISE_OP" || t == "COMB_LOGIC" || t == "RELATIONAL_OPERATOR") {
        return {{"In1", -20, -10, -1, 0, true}, {"In2", -20, 10, -1, 0, true}, {"Out", 20, 0, 1, 0, true}};
    }
    if (t == "EDGE_DETECT" || t == "MONOSTABLE" || t == "MONOFLOP" || t == "COMP_CONST" || t == "COMPARE_TO_CONSTANT") {
        return {{"In", -20, 0, -1, 0, true}, {"Out", 20, 0, 1, 0, true}};
    }
    if (t == "D_FLIP_FLOP") {
        return {{"D", -20, -10, -1, 0, true}, {"Ctrl", 0, -20, 0, -1, true}, {"Q", 20, -10, 1, 0, true}, {"Q_bar", 20, 10, 1, 0, true}};
    }
    if (t == "JK_FLIP_FLOP") {
        return {{"J", -20, -15, -1, 0, true}, {"K", -20, 0, -1, 0, true}, {"Ctrl", 0, -20, 0, -1, true}, {"Q", 20, -10, 1, 0, true}, {"Q_bar", 20, 10, 1, 0, true}};
    }
    if (t == "SHIFT_REG") {
        return {{"D", -20, -10, -1, 0, true}, {"Ctrl", 0, -20, 0, -1, true}, {"Out", 20, 0, 1, 0, true}};
    }
    if (t == "CLARKE") {
        return {{"A", -20, -15, -1, 0, true}, {"B", -20, 0, -1, 0, true}, {"C", -20, 15, -1, 0, true}, {"Alpha", 20, -10, 1, 0, true}, {"Beta", 20, 10, 1, 0, true}};
    }
    if (t == "INV_CLARKE") {
        return {{"Alpha", -20, -10, -1, 0, true}, {"Beta", -20, 10, -1, 0, true}, {"A", 20, -15, 1, 0, true}, {"B", 20, 0, 1, 0, true}, {"C", 20, 15, 1, 0, true}};
    }
    if (t == "PARK") {
        return {{"Alpha", -20, -15, -1, 0, true}, {"Beta", -20, 0, -1, 0, true}, {"Theta", -20, 15, -1, 0, true}, {"d", 20, -10, 1, 0, true}, {"q", 20, 10, 1, 0, true}};
    }
    if (t == "INV_PARK") {
        return {{"d", -20, -15, -1, 0, true}, {"q", -20, 0, -1, 0, true}, {"Theta", -20, 15, -1, 0, true}, {"Alpha", 20, -10, 1, 0, true}, {"Beta", 20, 10, 1, 0, true}};
    }
    if (t == "DQ_TO_ABC" || t == "DQ_ABC" || t == "INV_PARK_3PH") {
        return {{"d", -20, -15, -1, 0, true}, {"q", -20, 0, -1, 0, true}, {"Theta", -20, 15, -1, 0, true}, {"A", 20, -15, 1, 0, true}, {"B", 20, 0, 1, 0, true}, {"C", 20, 15, 1, 0, true}};
    }
    if (t == "ABC_TO_DQ" || t == "ABC_DQ" || t == "PARK_3PH") {
        return {{"A", -20, -15, -1, 0, true}, {"B", -20, 0, -1, 0, true}, {"C", -20, 15, -1, 0, true}, {"Theta", -20, 30, -1, 0, true}, {"d", 20, -10, 1, 0, true}, {"q", 20, 10, 1, 0, true}};
    }
    if (t == "PWM_3PH" || t == "SVPWM") {
        return {{"A", -20, -15, -1, 0, true}, {"B", -20, 0, -1, 0, true}, {"C", -20, 15, -1, 0, true}, {"OutA", 20, -15, 1, 0, true}, {"OutB", 20, 0, 1, 0, true}, {"OutC", 20, 15, 1, 0, true}};
    }
    if (t == "PER_AVG" || t == "MOV_AVG" || t == "FILTER_1ST" || t == "FILTER_2ND" || t == "RMS_VAL" || t == "THD_VAL" || t == "OFFSET" || t == "SIGNUM" || t == "DATATYPE_CONV" || t == "STATE_MACHINE") {
        return {{"In", -20, 0, -1, 0, true}, {"Out", 20, 0, 1, 0, true}};
    }
    if (t == "DIVIDE") {
        return {{"Num", -20, -10, -1, 0, true}, {"Den", -20, 10, -1, 0, true}, {"Out", 20, 0, 1, 0, true}};
    }
    if (t == "PERIODIC_IMP_AVG") {
        return {{"In", -20, -10, -1, 0, true}, {"Trig", -20, 10, -1, 0, true}, {"Out", 20, 0, 1, 0, true}};
    }
    if (t == "FOURIER_TRANS" || t == "FOURIER_ANALYSIS") {
        return {{"In", -20, 0, -1, 0, true}, {"Mag", 20, -10, 1, 0, true}, {"Phase", 20, 10, 1, 0, true}};
    }
    if (t == "PLL_LOOP") {
        return {{"In", -20, 0, -1, 0, true}, {"Theta", 20, -15, 1, 0, true}, {"Freq", 20, -5, 1, 0, true}, {"Cos", 20, 5, 1, 0, true}, {"Sin", 20, 15, 1, 0, true}};
    }
    if (t == "E_PORT" || t == "E_LABEL") {
        return {{"A", 0, 0, 0, 0, true}};
    }
    if (t == "CTRL_V" || t == "CTRL_I") {
        return {{"A", 0, -20, 0, -1, true}, {"B", 0, 20, 0, 1, true}, {"Ctrl", -20, 0, -1, 0, true}};
    }
    if (t == "V_3PH") {
        return {{"A", -20, -15, -1, 0, true}, {"B", -20, 0, -1, 0, true}, {"C", -20, 15, -1, 0, true}, {"N", 20, 0, 1, 0, true}};
    }
    if (t == "I_3PH") {
        return {{"A", 20, -15, 1, 0, true}, {"B", 20, 0, 1, 0, true}, {"C", 20, 15, 1, 0, true}, {"Ctrl", -20, 0, -1, 0, true}};
    }
    if (t == "VM" || t == "AM") {
        return {{"A", -20, 0, -1, 0, true}, {"B", 20, 0, 1, 0, true}, {"Out", 0, -20, 0, -1, true}};
    }
    if (t == "VM_3PH") {
        return {{"A", -20, -15, -1, 0, true}, {"B", -20, 0, -1, 0, true}, {"C", -20, 15, -1, 0, true}, {"N", 20, 0, 1, 0, true}, {"Out", 0, -20, 0, -1, true}};
    }
    if (t == "AM_3PH") {
        return {{"A_in", -20, -15, -1, 0, true}, {"B_in", -20, 0, -1, 0, true}, {"C_in", -20, 15, -1, 0, true}, {"A_out", 20, -15, 1, 0, true}, {"B_out", 20, 0, 1, 0, true}, {"C_out", 20, 15, 1, 0, true}, {"Out", 0, -20, 0, -1, true}};
    }
    if (t == "VAR_R" || t == "VAR_L" || t == "VAR_C" || t == "SAT_L" || t == "SAT_C") {
        return {{"A", -20, 0, -1, 0, true}, {"B", 20, 0, 1, 0, true}, {"Ctrl", 0, -20, 0, -1, true}};
    }
    if (t == "PI_SECTION" || t == "PWL_R" || t == "E_ALGEBRAIC") {
        return {{"A", -20, 0, -1, 0, true}, {"B", 20, 0, 1, 0, true}};
    }
    if (t == "LINE_3PH") {
        return {{"A_in", -20, -15, -1, 0, true}, {"B_in", -20, 0, -1, 0, true}, {"C_in", -20, 15, -1, 0, true}, {"A_out", 20, -15, 1, 0, true}, {"B_out", 20, 0, 1, 0, true}, {"C_out", 20, 15, 1, 0, true}};
    }
    // ── Magnetic domain blocks ──
    // Winding bridges the two domains: E+/E- are electrical, M+/M- are magnetic.
    if (t == "WINDING") {
        return {{"E+", -25, -25, -1, 0, false}, {"E-", -25, 25, -1, 0, false},
                {"M+",  25, -25,  1, 0, false}, {"M-",  25, 25,  1, 0, false}};
    }
    // Two-terminal magnetic elements.
    if (t == "MAG_PERMEANCE" || t == "LINEAR_CORE" || t == "AIR_GAP" ||
        t == "LEAKAGE_PATH" || t == "MAG_RESISTANCE" || t == "MMF_SRC") {
        return {{"A", 0, -40, 0, -1, false}, {"B", 0, 40, 0, 1, false}};
    }
    if (t == "MMF_SRC_CTRL") {
        return {{"A", 0, -40, 0, -1, false}, {"B", 0, 40, 0, 1, false},
                {"Ctrl", -30, 0, -1, 0, true}};
    }

    // Power semiconductors. The two power terminals must be electrical (isControl =
    // false) — only the gate/base is a control-domain pin. Marking the power terminals
    // as control broke domain isolation and coloured power wires as signal lines.
    // Geometry matches the +/-40 body span used by the other two-terminal devices.
    if (t == "THYRISTOR" || t == "SCR" || t == "GTO" || t == "IGCT") {
        return {{"A", 0, -40, 0, -1, false}, {"K", 0, 40, 0, 1, false}, {"G", -20, 10, -1, 0, true}};
    }
    if (t == "BJT") {
        return {{"C", 0, -40, 0, -1, false}, {"E", 0, 40, 0, 1, false}, {"B", -20, 0, -1, 0, true}};
    }
    if (t == "JFET" || t == "IGBT" || t == "IGBT_DIODE") {
        return {{"D", 0, -40, 0, -1, false}, {"S", 0, 40, 0, 1, false}, {"G", -20, 0, -1, 0, true}};
    }
    if (t == "BREAKER" || t == "SR_SWITCH") {
        return {{"A", -20, 0, -1, 0, true}, {"B", 20, 0, 1, 0, true}, {"Ctrl", 0, -20, 0, -1, true}};
    }
    if (t == "DBL_SWITCH" || t == "MAN_SWITCH") {
        return {{"Common", -20, 0, -1, 0, true}, {"A", 20, -10, 1, 0, true}, {"B", 20, 10, 1, 0, true}};
    }
    if (t == "MAN_DBL_SWITCH") {
        return {{"A1", -20, -10, -1, 0, true}, {"A2", -20, 10, -1, 0, true}, {"B1", 20, -10, 1, 0, true}, {"B2", 20, 10, 1, 0, true}};
    }
    if (t == "MAN_TRPL_SWITCH" || t == "TRPL_SWITCH") {
        return {{"A1", -20, -15, -1, 0, true}, {"A2", -20, 0, -1, 0, true}, {"A3", -20, 15, -1, 0, true}, {"B1", 20, -15, 1, 0, true}, {"B2", 20, 0, 1, 0, true}, {"B3", 20, 15, 1, 0, true}, {"Ctrl", 0, -20, 0, -1, true}};
    }

    if (t == "IDEAL_XFMR" || t == "XFMR_2W" || t == "MUTUAL_2W" || t == "SAT_XFMR" || t == "Transformer" || t == "IDEAL_TRANSFORMER" || t == "TRANSFORMER" || t == "XFMR") {
        std::string pStr = comp.parameters.count("primary_turns") ? comp.parameters.at("primary_turns") : "[100]";
        std::string sStr = comp.parameters.count("secondary_turns") ? comp.parameters.at("secondary_turns") : "[100]";
        auto pTurns = parseTurnsArrayStr(pStr);
        auto sTurns = parseTurnsArrayStr(sStr);

        int np = (int)pTurns.size();
        int ns = (int)sTurns.size();

        std::vector<TerminalDef> terms;

        // Primary terminals (Left side, x = -25)
        for (int i = 0; i < np; ++i) {
            float yCenter = (np > 1) ? (-25.0f * (np - 1) + 50.0f * i) : 0.0f;
            std::string nameA = (np == 1) ? "P1A" : ("P" + std::to_string(i + 1) + "A");
            std::string nameB = (np == 1) ? "P1B" : ("P" + std::to_string(i + 1) + "B");
            terms.push_back({nameA, -25.0f, yCenter - 14.0f, -1.0f, 0.0f, true});
            terms.push_back({nameB, -25.0f, yCenter + 14.0f, -1.0f, 0.0f, true});
        }

        // Secondary terminals (Right side, x = +25)
        for (int j = 0; j < ns; ++j) {
            float yCenter = (ns > 1) ? (-25.0f * (ns - 1) + 50.0f * j) : 0.0f;
            std::string nameA = (ns == 1) ? "S1A" : ("S" + std::to_string(j + 1) + "A");
            std::string nameB = (ns == 1) ? "S1B" : ("S" + std::to_string(j + 1) + "B");
            terms.push_back({nameA, 25.0f, yCenter - 14.0f, 1.0f, 0.0f, true});
            terms.push_back({nameB, 25.0f, yCenter + 14.0f, 1.0f, 0.0f, true});
        }

        return terms;
    }
    if (t == "XFMR_3W" || t == "MUTUAL_3W") {
        return {{"P1", -20, -10, -1, 0, true}, {"P2", -20, 10, -1, 0, true}, {"S1_1", 20, -15, 1, 0, true}, {"S1_2", 20, -5, 1, 0, true}, {"S2_1", 20, 5, 1, 0, true}, {"S2_2", 20, 15, 1, 0, true}};
    }
    if (t == "XFMR_3PH_2W" || t == "XFMR_3PH_3W") {
        return {{"A", -20, -15, -1, 0, true}, {"B", -20, 0, -1, 0, true}, {"C", -20, 15, -1, 0, true}, {"a", 20, -15, 1, 0, true}, {"b", 20, 0, 1, 0, true}, {"c", 20, 15, 1, 0, true}};
    }
    if (t == "OPAMP" || t == "E_COMP") {
        return {{"Plus", -20, -10, -1, 0, true}, {"Minus", -20, 10, -1, 0, true}, {"Out", 20, 0, 1, 0, true}};
    }
    if (t == "GOTO_SIG" || t == "GOTO") {
        return {{"In", -15, 0, -1, 0, true}};
    }
    if (t == "FROM_SIG" || t == "FROM") {
        return {{"Out", 15, 0, 1, 0, true}};
    }
    if (t == "VGFET") {
        // "vg-FET" is handled alongside MOSFET near the top of this function; this entry
        // covers the alternate spelling and must stay identical to it.
        return {{"D", 0, -40, 0, -1, false}, {"S", 0, 40, 0, 1, false}, {"G", -20, 0, -1, 0, true}};
    }
    if (t == "INDUCTION_MOTOR" || t == "IND_MOTOR" || t == "BLDC_MOTOR" || t == "PMSM_MOTOR" || t == "SRM_MOTOR") {
        // Three stator phases A, B, C (electrical). TL takes the load torque signal and Speed
        // outputs the shaft speed.
        return {{"A", -30, -20, -1, 0, false}, {"B", -30, 0, -1, 0, false}, {"C", -30, 20, -1, 0, false},
                {"TL", 0, -40, 0, -1, true}, {"Speed", 30, 0, 1, 0, true}};
    }
    if (t == "SYNC_MOTOR") {
        // Stator phases A, B, C and the field winding FA, FB (electrical).
        return {{"A", -30, -20, -1, 0, false}, {"B", -30, 0, -1, 0, false}, {"C", -30, 20, -1, 0, false},
                {"FA", 30, -20, 1, 0, false}, {"FB", 30, 20, 1, 0, false},
                {"TL", 0, -40, 0, -1, true}, {"Speed", 30, 0, 1, 0, true}};
    }
    if (t == "GEN_EBLOCK") {
        int n = 3;
        if (comp.parameters.count("terminals")) {
            try { n = std::stoi(comp.parameters.at("terminals")); } catch (...) {}
        }
        if (n < 1) n = 1;
        if (n > 8) n = 8;
        std::vector<TerminalDef> terms;
        for (int i = 0; i < n; ++i) {
            float yOff = (n > 1) ? (-15.0f * (n - 1) + 30.0f * i) : 0.0f;
            terms.push_back({"T" + std::to_string(i + 1), -30.0f, yOff, -1.0f, 0.0f, true});
        }
        return terms;
    }
    if (t == "SCOPE") {
        int numCh = 2;
        if (comp.parameters.count("channels")) {
            try { numCh = std::stoi(comp.parameters.at("channels")); } catch (...) {}
        }
        if (numCh < 1) numCh = 1;
        if (numCh > 8) numCh = 8;

        std::vector<TerminalDef> terms;
        for (int i = 0; i < numCh; ++i) {
            float yOff = (numCh > 1) ? (-10.0f * (numCh - 1) + 20.0f * i) : 0.0f;
            std::string termName = "In" + std::to_string(i + 1);
            terms.push_back({termName, -16.0f, yOff, -1.0f, 0.0f, true});
        }
        return terms;
    }
    if (t == "PROBE") {
        std::string sigStr = comp.parameters.count("selected_signals") ? comp.parameters.at("selected_signals") : "";
        std::vector<std::string> sigs;
        std::stringstream ss(sigStr);
        std::string item;
        while (std::getline(ss, item, ',')) { if (!item.empty()) sigs.push_back(item); }

        std::vector<TerminalDef> terms;
        if (sigs.empty()) {
            terms.push_back({"Out", 30.0f, 0.0f, 1.0f, 0.0f, true});
        } else {
            int n = (int)sigs.size();
            for (int i = 0; i < n; ++i) {
                float yOff = (n > 1) ? (-10.0f * (n - 1) + 20.0f * i) : 0.0f;
                terms.push_back({sigs[i], 30.0f, yOff, 1.0f, 0.0f, true});
            }
        }
        return terms;
    }
    if (t == "MUX") {
        return {{"In1", -20, -12, -1, 0, true}, {"In2", -20, 12, -1, 0, true}, {"Out", 20, 0, 1, 0, true}};
    }
    if (t == "INPORT" || t == "IN") {
        return {{"Out", 20, 0, 1, 0, true}};
    }
    if (t == "OUTPORT" || t == "OUT") {
        return {{"In", -20, 0, -1, 0, true}};
    }
    if (t == "PHYSICAL_INPORT" || t == "PIN") {
        return {{"A", 20, 0, 1, 0, true}};
    }
    if (t == "PHYSICAL_OUTPORT" || t == "POUT") {
        return {{"A", -20, 0, -1, 0, true}};
    }
    if (t == "ENABLE_PORT") {
        return {{"Enable", 0, -20, 0, -1, true}};
    }
    if (t == "TRIGGER_PORT") {
        return {{"Trigger", 0, -20, 0, -1, true}};
    }
    if (t == "BUS_CREATOR") {
        int n = 2;
        if (comp.parameters.count("inputs")) {
            try { n = std::stoi(comp.parameters.at("inputs")); } catch (...) {}
        }
        if (n < 1) n = 1; if (n > 8) n = 8;
        std::vector<TerminalDef> terms;
        for (int i = 0; i < n; ++i) {
            float yOff = (n > 1) ? (-15.0f * (n - 1) / 2.0f + 15.0f * i) : 0.0f;
            terms.push_back({"In" + std::to_string(i + 1), -20.0f, yOff, -1.0f, 0.0f, true});
        }
        terms.push_back({"Bus", 20.0f, 0.0f, 1.0f, 0.0f, true});
        return terms;
    }
    if (t == "BUS_SELECTOR") {
        std::vector<TerminalDef> terms;
        terms.push_back({"Bus", -20.0f, 0.0f, -1.0f, 0.0f, true});
        terms.push_back({"Out1", 20.0f, -10.0f, 1.0f, 0.0f, true});
        terms.push_back({"Out2", 20.0f, 10.0f, 1.0f, 0.0f, true});
        return terms;
    }
    if (t == "TERMINATOR") {
        return {{"In", -15, 0, -1, 0, true}};
    }
    if (t == "POLYNOMIAL" || t == "ALGEBRAIC_CONSTRAINT") {
        return {{"In", -20, 0, -1, 0, true}, {"Out", 20, 0, 1, 0, true}};
    }

    if (t == "CSCRIPT" || t == "CUSTOMSCRIPT") {
        std::vector<CircuitSimEngine::CScriptPort> inPorts, outPorts;
        getCSCRIPTPorts(comp, inPorts, outPorts);

        int numIn = (int)inPorts.size();
        int numOut = (int)outPorts.size();

        float hw = 45.0f;
        std::vector<TerminalDef> terms;

        for (int i = 0; i < numIn; ++i) {
            float yOff = (numIn > 1) ? (-18.0f * (numIn - 1) / 2.0f + 18.0f * i) : 0.0f;
            terms.push_back({inPorts[i].name, -hw, yOff, -1.0f, 0.0f, true});
        }
        for (int j = 0; j < numOut; ++j) {
            float yOff = (numOut > 1) ? (-18.0f * (numOut - 1) / 2.0f + 18.0f * j) : 0.0f;
            terms.push_back({outPorts[j].name, hw, yOff, 1.0f, 0.0f, true});
        }
        return terms;
    }
    // No built-in layout: use the one declared in the block registry, if any.
    return registryTerminals(comp);
}

} // namespace CircuitSim
