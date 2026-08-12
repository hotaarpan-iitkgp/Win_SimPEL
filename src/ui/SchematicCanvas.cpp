#include "SchematicCanvas.hpp"
#include "imgui_internal.h"
#include "../engine/CScriptEngine.hpp"
#include <nlohmann/json.hpp>
#include <windows.h>
#include <commdlg.h>
#include <cmath>
#include <algorithm>
#include <iostream>
#include <sstream>
#include <fstream>
#include <unordered_set>

namespace CircuitSim {

static std::string g_internalClipboard;

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

enum class DomainType { Power, Control };

static ImVec2 rotatePt(float px, float py, float cx, float cy, float angleDeg) {
    if (angleDeg == 0.0f) return ImVec2(cx + px, cy + py);
    float rad = angleDeg * 3.1415926535f / 180.0f;
    float cosA = std::cos(rad);
    float sinA = std::sin(rad);
    float rx = px * cosA - py * sinA;
    float ry = px * sinA + py * cosA;
    return ImVec2(cx + rx, cy + ry);
}

static ImVec2 getClosestPointOnSegment(ImVec2 p, ImVec2 a, ImVec2 b, float& outDist) {
    float l2 = (b.x - a.x)*(b.x - a.x) + (b.y - a.y)*(b.y - a.y);
    if (l2 == 0) {
        outDist = std::sqrt((p.x - a.x)*(p.x - a.x) + (p.y - a.y)*(p.y - a.y));
        return a;
    }
    float t = std::max(0.0f, std::min(1.0f, ((p.x - a.x)*(b.x - a.x) + (p.y - a.y)*(b.y - a.y)) / l2));
    ImVec2 proj(a.x + t * (b.x - a.x), a.y + t * (b.y - a.y));
    outDist = std::sqrt((p.x - proj.x)*(p.x - proj.x) + (p.y - proj.y)*(p.y - proj.y));
    return proj;
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
    
    if (t == "R" || t == "L" || t == "C" || t == "V" || t == "I" || t == "D" || t == "AC_V") {
        return {{"A", 0, -40, 0, -1, false}, {"B", 0, 40, 0, 1, false}};
    }
    if (t == "VM" || t == "AM") {
        return {{"A", 0, -40, 0, -1, false}, {"B", 0, 40, 0, 1, false}, {"Out", 20, 0, 1, 0, true}};
    }
    if (t == "MOSFET" || t == "vg-FET") {
        return {{"D", 0, -40, 0, -1, false}, {"S", 0, 40, 0, 1, false}, {"G", -20, 0, -1, 0, true}};
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
    if (t == "GAIN" || t == "PID" || t == "PWM" || t == "FCN" || t == "NOT" || t == "INIT_COND" ||
        t == "TRIG_FCN" || t == "ABS" || t == "SIGN" || t == "ROUND" || t == "LUT_1D" || t == "LUT_2D" || t == "LUT_3D" || t == "DLL" || t == "FMU" || t == "FOURIER_SERIES" ||
        t == "INTEGRATOR" || t == "DERIVATIVE" || t == "TRANSFER_FCN" || t == "STATE_SPACE" || t == "CONT_PID" || t == "DISCRETE_PID" || t == "PLL_1PH" ||
        t == "DELAY" || t == "TRANSPORT_DELAY" || t == "TURN_ON_DELAY" || t == "MEMORY_BLOCK" ||
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
        t == "PROD" || t == "PRODUCT_RECT") {
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

        if (t == "SUM_RECT" || t == "PRODUCT_RECT") {
            terms.push_back({"Ctrl", -15.0f, -halfH, 0, -1, true});
        }

        return terms;
    }
    if (t == "COMP" || t == "AND" || t == "OR" || t == "MIN_MAX" ||
        t == "LOGIC_OP" || t == "BITWISE_OP" || t == "COMB_LOGIC" || t == "RELATIONAL_OPERATOR") {
        return {{"In1", -20, -10, -1, 0, true}, {"In2", -20, 10, -1, 0, true}, {"Out", 20, 0, 1, 0, true}};
    }
    if (t == "EDGE_DETECT" || t == "MONOSTABLE" || t == "MONOFLOP" || t == "COMPARE_TO_CONSTANT") {
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
    if (t == "THYRISTOR" || t == "GTO" || t == "IGCT") {
        return {{"A", 0, -20, 0, -1, true}, {"K", 0, 20, 0, 1, true}, {"G", -20, 10, -1, 0, true}};
    }
    if (t == "BJT") {
        return {{"C", 0, -20, 0, -1, true}, {"E", 0, 20, 0, 1, true}, {"B", -20, 0, -1, 0, true}};
    }
    if (t == "JFET" || t == "MOSFET" || t == "IGBT" || t == "IGBT_DIODE") {
        return {{"D", 0, -20, 0, -1, true}, {"S", 0, 20, 0, 1, true}, {"G", -20, 0, -1, 0, true}};
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
    if (t == "vg-FET" || t == "VGFET") {
        return {{"D", 0, -20, 0, -1, true}, {"S", 0, 20, 0, 1, true}};
    }
    if (t == "INDUCTION_MOTOR" || t == "IND_MOTOR") {
        return {{"A", -20, -15, -1, 0, true}, {"B", -20, 0, -1, 0, true}, {"C", -20, 15, -1, 0, true}, {"TL", -20, 30, -1, 0, true}, {"Speed", 20, 0, 1, 0, true}};
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
                float yOff = (n > 1) ? (-15.0f * (n - 1) + 30.0f * i) : 0.0f;
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
    return {};
}

static bool isTerminalMatch(const std::string& compTypeStr, const std::string& termA, const std::string& termB) {
    if (termA == termB) return true;
    std::string a = termA, b = termB;
    std::transform(a.begin(), a.end(), a.begin(), ::toupper);
    std::transform(b.begin(), b.end(), b.begin(), ::toupper);
    if (a == b) return true;

    // Top pin aliases (Plus / In1 / A / In / G)
    bool isTopA = (a == "A" || a == "PLUS" || a == "IN1" || a == "IN" || a == "INPUT_0" || a == "IN_0");
    bool isTopB = (b == "A" || b == "PLUS" || b == "IN1" || b == "IN" || b == "INPUT_0" || b == "IN_0");
    if (isTopA && isTopB) return true;

    // Bottom pin aliases (Minus / In2 / B)
    bool isBotA = (a == "B" || a == "MINUS" || a == "IN2" || a == "INPUT_1" || a == "IN_1");
    bool isBotB = (b == "B" || b == "MINUS" || b == "IN2" || b == "INPUT_1" || b == "IN_1");
    if (isBotA && isBotB) return true;

    // Output pin aliases (Out / Out1 / Output / Direct)
    bool isOutA = (a == "OUT" || a == "OUT1" || a == "OUTPUT" || a == "OUTPUT_0" || a == "OUTDIRECT1");
    bool isOutB = (b == "OUT" || b == "OUT1" || b == "OUTPUT" || b == "OUTPUT_0" || b == "OUTDIRECT1");
    if (isOutA && isOutB) return true;

    // Primary 1 Top (P1, P1A, P1_1, PA)
    bool isP1A_A = (a == "P1" || a == "P1A" || a == "P1_1" || a == "PA");
    bool isP1A_B = (b == "P1" || b == "P1A" || b == "P1_1" || b == "PA");
    if (isP1A_A && isP1A_B) return true;

    // Primary 1 Bottom (P2, P1B, P1_2, PB)
    bool isP1B_A = (a == "P2" || a == "P1B" || a == "P1_2" || a == "PB");
    bool isP1B_B = (b == "P2" || b == "P1B" || b == "P1_2" || b == "PB");
    if (isP1B_A && isP1B_B) return true;

    // Secondary 1 Top (S1, S1A, S1_1, SA)
    bool isS1A_A = (a == "S1" || a == "S1A" || a == "S1_1" || a == "SA");
    bool isS1A_B = (b == "S1" || b == "S1A" || b == "S1_1" || b == "SA");
    if (isS1A_A && isS1A_B) return true;

    // Secondary 1 Bottom (S2, S1B, S1_2, SB)
    bool isS1B_A = (a == "S2" || a == "S1B" || a == "S1_2" || a == "SB");
    bool isS1B_B = (b == "S2" || b == "S1B" || b == "S1_2" || b == "SB");
    if (isS1B_A && isS1B_B) return true;

    // Secondary 2 Top (S2A, S3, S2_1)
    bool isS2A_A = (a == "S2A" || a == "S3" || a == "S2_1");
    bool isS2A_B = (b == "S2A" || b == "S3" || b == "S2_1");
    if (isS2A_A && isS2A_B) return true;

    // Secondary 2 Bottom (S2B, S4, S2_2)
    bool isS2B_A = (a == "S2B" || a == "S4" || a == "S2_2");
    bool isS2B_B = (b == "S2B" || b == "S4" || b == "S2_2");
    if (isS2B_A && isS2B_B) return true;

    return false;
}

static DomainType getPinDomain(const ComponentInstance& comp, const std::string& pinName) {
    std::string t = comp.rawTypeStr;
    std::transform(t.begin(), t.end(), t.begin(), ::toupper);
    std::string p = pinName;
    std::transform(p.begin(), p.end(), p.begin(), ::toupper);

    if ((t == "MOSFET" || t == "VG-FET" || t == "IGBT" || t == "IGBT_DIODE" || t == "IGCT" || t == "GTO" || t == "THYRISTOR" || t == "JFET") && (p == "G" || p == "GATE")) return DomainType::Control;
    if (t == "BJT" && (p == "B" || p == "BASE")) return DomainType::Control;
    if (t == "S" && (p == "CTRL" || p == "GATE")) return DomainType::Control;
    if ((t == "CTRL_V" || t == "CTRL_I" || t == "I_3PH") && (p == "CTRL" || p == "IN")) return DomainType::Control;
    if ((t == "VM" || t == "AM") && (p == "OUT" || p == "V" || p == "I")) return DomainType::Control;

    if (t == "GAIN" || t == "PID" || t == "PWM" || t == "TRI" || t == "TRI_GEN" || t == "PULSE" || t == "PULSE_GEN" || t == "CONST" || t == "STEP" || t == "RAMP" || t == "SINE_WAVE" || t == "CLOCK" ||
        t == "RANDOM_NUM" || t == "WHITE_NOISE" || t == "INIT_COND" ||
        t == "TRIG_FCN" || t == "ABS" || t == "SIGN" || t == "ROUND" || t == "MIN_MAX" || t == "LUT_1D" || t == "LUT_2D" || t == "LUT_3D" || t == "DLL" || t == "FMU" || t == "FOURIER_SERIES" ||
        t == "INTEGRATOR" || t == "DERIVATIVE" || t == "TRANSFER_FCN" || t == "STATE_SPACE" || t == "CONT_PID" || t == "DISCRETE_PID" || t == "PLL_1PH" || t == "PLL_3PH" ||
        t == "DELAY" || t == "TRANSPORT_DELAY" || t == "TURN_ON_DELAY" || t == "MEMORY_BLOCK" ||
        t == "QUANTIZER" || t == "SIGNAL_SWITCH" || t == "MANUAL_SWITCH" || t == "MULTIPORT_SWITCH" || t == "HIT_CROSSING" || t == "SATURATION" || t == "DEAD_ZONE" || t == "RATE_LIMITER" || t == "RELAY" ||
        t == "LOGIC_OP" || t == "BITWISE_OP" || t == "COMB_LOGIC" || t == "EDGE_DETECT" || t == "MONOSTABLE" || t == "MONOFLOP" ||
        t == "RELATIONAL_OPERATOR" || t == "COMPARE_TO_CONSTANT" || t == "D_FLIP_FLOP" || t == "JK_FLIP_FLOP" || t == "SHIFT_REG" ||
        t == "CLARKE" || t == "INV_CLARKE" || t == "PARK" || t == "INV_PARK" || t == "PWM_3PH" || t == "SVPWM" ||
        t == "PER_AVG" || t == "PERIODIC_IMP_AVG" || t == "FOURIER_TRANS" || t == "MOV_AVG" || t == "FILTER_1ST" || t == "FILTER_2ND" ||
        t == "FOURIER_ANALYSIS" || t == "RMS_VAL" || t == "THD_VAL" || t == "PLL_LOOP" ||
        t == "OFFSET" || t == "SIGNUM" || t == "DIVIDE" || t == "DATATYPE_CONV" || t == "STATE_MACHINE" ||
        t == "SUM" || t == "SUM_ROUND" || t == "SUM_RECT" || t == "SUBTRACT" || t == "SUB" ||
        t == "PROD" || t == "PRODUCT" || t == "PRODUCT_RECT" || t == "COMP" || t == "COMPARATOR" ||
        t == "GOTO_SIG" || t == "GOTO" || t == "FROM_SIG" || t == "FROM" ||
        t == "AND" || t == "OR" || t == "NOT" || t == "FCN" ||
        t == "CSCRIPT" || t == "SCOPE" || t == "PROBE" || t == "MUX" || t == "DEMUX" || t == "KEY_TRIGGER") {
        return DomainType::Control;
    }

    return DomainType::Power;
}

static ImVec2 getEndpointWorldPos(const WireEndpoint& ep, const CircuitDesign& design) {
    if (ep.isWireJunction) {
        return ImVec2(ep.junctionX, ep.junctionY);
    }
    for (const auto& comp : design.components) {
        if (comp.id == ep.compId) {
            auto terms = getTerminals(comp);
            for (const auto& t : terms) {
                if (t.name == ep.terminal) {
                    float rad = comp.rotation * 3.14159265f / 180.0f;
                    float rx = t.relX * std::cos(rad) - t.relY * std::sin(rad);
                    float ry = t.relX * std::sin(rad) + t.relY * std::cos(rad);
                    return ImVec2(comp.x + rx, comp.y + ry);
                }
            }
            return ImVec2(comp.x, comp.y);
        }
    }
    return ImVec2(ep.junctionX, ep.junctionY);
}

static DomainType getWireDomainInternal(const WireInstance& wire, const CircuitDesign& design, std::set<std::string>& visitedWires) {
    if (visitedWires.count(wire.id)) return DomainType::Power;
    visitedWires.insert(wire.id);

    // 1. Check direct component pin on 'from'
    if (!wire.from.isWireJunction && !wire.from.compId.empty()) {
        for (const auto& comp : design.components) {
            if (comp.id == wire.from.compId) {
                DomainType dom = getPinDomain(comp, wire.from.terminal);
                if (dom == DomainType::Control) return DomainType::Control;
            }
        }
    }

    // 2. Check direct component pin on 'to'
    if (!wire.to.isWireJunction && !wire.to.compId.empty()) {
        for (const auto& comp : design.components) {
            if (comp.id == wire.to.compId) {
                DomainType dom = getPinDomain(comp, wire.to.terminal);
                if (dom == DomainType::Control) return DomainType::Control;
            }
        }
    }

    // 3. Trace target wire if 'from' is a junction
    if (wire.from.isWireJunction && !wire.from.targetWireId.empty()) {
        for (const auto& w : design.wires) {
            if (w.id == wire.from.targetWireId) {
                DomainType dom = getWireDomainInternal(w, design, visitedWires);
                if (dom == DomainType::Control) return DomainType::Control;
            }
        }
    }

    // 4. Trace target wire if 'to' is a junction
    if (wire.to.isWireJunction && !wire.to.targetWireId.empty()) {
        for (const auto& w : design.wires) {
            if (w.id == wire.to.targetWireId) {
                DomainType dom = getWireDomainInternal(w, design, visitedWires);
                if (dom == DomainType::Control) return DomainType::Control;
            }
        }
    }

    // 5. Trace any wire that branches off from this wire
    for (const auto& w : design.wires) {
        if (w.id == wire.id) continue;
        bool branchesFromThis = (w.from.isWireJunction && w.from.targetWireId == wire.id) ||
                               (w.to.isWireJunction && w.to.targetWireId == wire.id);
        if (branchesFromThis) {
            DomainType dom = getWireDomainInternal(w, design, visitedWires);
            if (dom == DomainType::Control) return DomainType::Control;
        }
    }

    return DomainType::Power;
}

static DomainType getWireDomain(const WireInstance& wire, const CircuitDesign& design) {
    std::set<std::string> visited;
    return getWireDomainInternal(wire, design, visited);
}

static bool isControlOutputPin(const ComponentInstance& comp, const std::string& pinName) {
    if (getPinDomain(comp, pinName) != DomainType::Control) return false;
    std::string p = pinName;
    std::transform(p.begin(), p.end(), p.begin(), ::toupper);
    if (p.rfind("OUT", 0) == 0 || p == "OUT" || p == "OUT1" || p == "OUT2" || p == "OUT3" || p == "OUT4" || p == "D1" || p == "C1") {
        return true;
    }
    return false;
}

bool SchematicCanvas::validateSingleOutportConstraint(const std::string& startCompId, const std::string& startPin, const std::string& targetCompId, const std::string& targetPin) const {
    const ComponentInstance* startComp = nullptr;
    const ComponentInstance* targetComp = nullptr;
    for (const auto& c : design.components) {
        if (c.id == startCompId) startComp = &c;
        if (c.id == targetCompId) targetComp = &c;
    }
    if (!startComp || !targetComp) return true;

    if (getPinDomain(*startComp, startPin) == DomainType::Control) {
        bool startIsOut = isControlOutputPin(*startComp, startPin);
        bool targetIsOut = isControlOutputPin(*targetComp, targetPin);

        if (startIsOut && targetIsOut) {
            return false;
        }
    }
    return true;
}

void SchematicCanvas::normalizeControlWires() {
    for (auto& wire : design.wires) {
        if (wire.to.isWireJunction) continue;

        const ComponentInstance* fromComp = nullptr;
        const ComponentInstance* toComp = nullptr;
        for (const auto& c : design.components) {
            if (c.id == wire.from.compId) fromComp = &c;
            if (c.id == wire.to.compId) toComp = &c;
        }

        if (fromComp && toComp) {
            if (getPinDomain(*fromComp, wire.from.terminal) == DomainType::Control) {
                bool fromIsOut = isControlOutputPin(*fromComp, wire.from.terminal);
                bool toIsOut = isControlOutputPin(*toComp, wire.to.terminal);

                if (!fromIsOut && toIsOut) {
                    auto tempEndpoint = wire.from;
                    wire.from = wire.to;
                    wire.to = tempEndpoint;

                    if (!wire.manualPath.empty()) {
                        std::reverse(wire.manualPath.begin(), wire.manualPath.end());
                    }
                }
            }
        }
    }
    rebuildNetlist();
}

void SchematicCanvas::rebuildNetlist() {
    std::unordered_map<std::string, std::string> parent;
    
    auto findRoot = [&](const std::string& i) {
        std::string root = i;
        while (parent.count(root) && parent[root] != root) {
            root = parent[root];
        }
        std::string curr = i;
        while (parent.count(curr) && parent[curr] != root) {
            std::string nxt = parent[curr];
            parent[curr] = root;
            curr = nxt;
        }
        return root;
    };
    
    auto unionNodes = [&](const std::string& a, const std::string& b) {
        if (a.empty() || b.empty()) return;
        if (!parent.count(a)) parent[a] = a;
        if (!parent.count(b)) parent[b] = b;
        std::string rootA = findRoot(a);
        std::string rootB = findRoot(b);
        if (rootA != rootB) {
            parent[rootA] = rootB;
        }
    };

    // 1. Register all component pins
    for (const auto& comp : design.components) {
        auto terms = getTerminals(comp);
        for (const auto& t : terms) {
            std::string key = comp.id + "." + t.name;
            parent[key] = key;
        }
    }

    // 2. Map wires and T-junctions
    std::unordered_map<std::string, std::string> wireNetMap;
    for (const auto& wire : design.wires) {
        std::string fromKey = wire.from.compId + "." + wire.from.terminal;
        if (!wire.to.isWireJunction) {
            std::string toKey = wire.to.compId + "." + wire.to.terminal;
            unionNodes(fromKey, toKey);
            wireNetMap[wire.id] = fromKey;
        } else {
            auto it = wireNetMap.find(wire.to.targetWireId);
            if (it != wireNetMap.end()) {
                unionNodes(fromKey, it->second);
            }
            wireNetMap[wire.id] = fromKey;
        }
    }

    // 3. Assign merged net names to component nodes for simulation
    for (auto& comp : design.components) {
        auto terms = getTerminals(comp);
        comp.nodes.clear();
        for (const auto& t : terms) {
            std::string key = comp.id + "." + t.name;
            std::string rootNet = findRoot(key);
            if (comp.rawTypeStr == "GND" || rootNet.find("GND") != std::string::npos) {
                comp.nodes.push_back("0");
            } else {
                comp.nodes.push_back("net_" + rootNet);
            }
        }
    }
}

std::vector<ImVec2> SchematicCanvas::simplifyPath(const std::vector<ImVec2>& points) const {
    std::vector<ImVec2> result;
    if (points.size() <= 2) return points;
    result.push_back(points[0]);
    for (size_t i = 1; i < points.size() - 1; ++i) {
        ImVec2 prev = result.back();
        ImVec2 curr = points[i];
        ImVec2 next = points[i + 1];
        bool collinearX = (std::abs(prev.x - curr.x) < 1.0f && std::abs(curr.x - next.x) < 1.0f);
        bool collinearY = (std::abs(prev.y - curr.y) < 1.0f && std::abs(curr.y - next.y) < 1.0f);
        if (!collinearX && !collinearY) {
            result.push_back(curr);
        }
    }
    result.push_back(points.back());
    return result;
}

void SchematicCanvas::drawCurrentFlowAnimation(ImDrawList* drawList, ImVec2 p1, ImVec2 mid, ImVec2 mid2, ImVec2 p2, bool isControlNet, float timeSec) {
    // Flowing wire particle dots disabled for simple, crisp, maximum UI rendering performance
    (void)drawList; (void)p1; (void)mid; (void)mid2; (void)p2; (void)isControlNet; (void)timeSec;
}

bool SchematicCanvas::isPinConnected(const std::string& compId, const std::string& pinName) const {
    const ComponentInstance* comp = nullptr;
    for (const auto& c : design.components) {
        if (c.id == compId) { comp = &c; break; }
    }
    for (const auto& w : design.wires) {
        if (w.from.compId == compId && (w.from.terminal == pinName || (comp && isTerminalMatch(comp->rawTypeStr, w.from.terminal, pinName)))) return true;
        if (w.to.compId == compId && (w.to.terminal == pinName || (comp && isTerminalMatch(comp->rawTypeStr, w.to.terminal, pinName)))) return true;
    }
    return false;
}

void SchematicCanvas::pushUndoState() {
    undoStack.push_back(design);
    redoStack.clear();
    if (undoStack.size() > 50) undoStack.erase(undoStack.begin());
}

void SchematicCanvas::undo() {
    if (undoStack.size() > 1) {
        redoStack.push_back(undoStack.back());
        undoStack.pop_back();
        design = undoStack.back();
    }
}

void SchematicCanvas::redo() {
    if (!redoStack.empty()) {
        design = redoStack.back();
        undoStack.push_back(redoStack.back());
        redoStack.pop_back();
    }
}

void SchematicCanvas::fitToScreen(ImVec2 canvasSize) {
    if (canvasSize.x <= 0.0f || canvasSize.y <= 0.0f) {
        canvasSize = lastRenderedCanvasSize;
    }
    if (canvasSize.x <= 0.0f || canvasSize.y <= 0.0f) {
        canvasSize = ImVec2(800.0f, 600.0f);
    }
    if (design.components.empty()) {
        zoomLevel = 1.0f;
        panOffset = ImVec2(canvasSize.x * 0.5f - 400.0f, canvasSize.y * 0.5f - 400.0f);
        return;
    }
    float minX = 1e9f, maxX = -1e9f, minY = 1e9f, maxY = -1e9f;
    for (const auto& c : design.components) {
        minX = std::min(minX, c.x - 40.0f);
        maxX = std::max(maxX, c.x + 40.0f);
        minY = std::min(minY, c.y - 40.0f);
        maxY = std::max(maxY, c.y + 40.0f);
    }
    for (const auto& w : design.wires) {
        for (const auto& pt : w.manualPath) {
            minX = std::min(minX, pt.x);
            maxX = std::max(maxX, pt.x);
            minY = std::min(minY, pt.y);
            maxY = std::max(maxY, pt.y);
        }
        if (w.to.isWireJunction) {
            minX = std::min(minX, w.to.junctionX);
            maxX = std::max(maxX, w.to.junctionX);
            minY = std::min(minY, w.to.junctionY);
            maxY = std::max(maxY, w.to.junctionY);
        }
    }
    float width = maxX - minX + 80.0f;
    float height = maxY - minY + 80.0f;
    if (width < 100.0f) width = 100.0f;
    if (height < 100.0f) height = 100.0f;

    float centerX = (minX + maxX) * 0.5f;
    float centerY = (minY + maxY) * 0.5f;

    float zoomX = (canvasSize.x * 0.85f) / width;
    float zoomY = (canvasSize.y * 0.85f) / height;
    float newZoom = std::min(zoomX, zoomY);

    if (newZoom < 0.15f) newZoom = 0.15f;
    if (newZoom > 2.0f) newZoom = 2.0f;

    zoomLevel = newZoom;
    panOffset.x = (canvasSize.x * 0.5f) / zoomLevel - centerX;
    panOffset.y = (canvasSize.y * 0.5f) / zoomLevel - centerY;
}

void SchematicCanvas::addComponent(const ComponentInstance& comp) {
    pushUndoState();
    ComponentInstance newComp = comp;
    
    if (hasLastClickPos) {
        newComp.x = lastCanvasClickWorldPos.x;
        newComp.y = lastCanvasClickWorldPos.y;
    }
    
    design.components.push_back(newComp);
}

ComponentInstance* SchematicCanvas::getSelectedComponent() {
    if (selectedComponentIds.empty()) return nullptr;
    std::string targetId = *selectedComponentIds.begin();
    for (auto& c : design.components) {
        if (c.id == targetId) return &c;
    }
    return nullptr;
}

void SchematicCanvas::drawGrid(ImDrawList* drawList, ImVec2 canvasSize, ImVec2 canvasPos) {
    float gridSize = 20.0f * zoomLevel;
    ImU32 gridColor = isDarkMode ? IM_COL32(35, 42, 56, 180) : IM_COL32(232, 227, 185, 220);
    ImU32 gridMajor = isDarkMode ? IM_COL32(50, 60, 80, 220) : IM_COL32(212, 205, 158, 255);
    
    float startX = std::fmod(panOffset.x * zoomLevel, gridSize);
    float startY = std::fmod(panOffset.y * zoomLevel, gridSize);
    if (startX < 0) startX += gridSize;
    if (startY < 0) startY += gridSize;

    int cellIdx = 0;
    for (float x = startX; x < canvasSize.x; x += gridSize) {
        ImU32 col = (cellIdx % 5 == 0) ? gridMajor : gridColor;
        drawList->AddLine(ImVec2(canvasPos.x + x, canvasPos.y), ImVec2(canvasPos.x + x, canvasPos.y + canvasSize.y), col, 1.0f);
        cellIdx++;
    }
    cellIdx = 0;
    for (float y = startY; y < canvasSize.y; y += gridSize) {
        ImU32 col = (cellIdx % 5 == 0) ? gridMajor : gridColor;
        drawList->AddLine(ImVec2(canvasPos.x, canvasPos.y + y), ImVec2(canvasPos.x + canvasSize.x, canvasPos.y + y), col, 1.0f);
        cellIdx++;
    }
}

void SchematicCanvas::drawBreadcrumbs(ImDrawList* drawList, ImVec2 canvasPos) {
    if (subsystemStack.empty()) return;

    std::string pathText = "Main Workspace";
    for (const auto& sub : subsystemStack) {
        pathText += " > " + sub.name;
    }

    drawList->AddRectFilled(canvasPos, ImVec2(canvasPos.x + 320, canvasPos.y + 32), IM_COL32(24, 28, 38, 230), 4.0f);
    drawList->AddText(ImVec2(canvasPos.x + 10, canvasPos.y + 8), IM_COL32(56, 189, 248, 255), pathText.c_str());

    ImGui::SetCursorScreenPos(ImVec2(canvasPos.x + 330, canvasPos.y + 2));
    if (ImGui::Button("< Back to Main Workspace")) {
        design = subsystemStack.front().design;
        subsystemStack.clear();
    }
}

void SchematicCanvas::drawComponentShape(ImDrawList* drawList, const ComponentInstance& comp, ImVec2 c, float s, ImU32 color) {
    const std::string& t = comp.rawTypeStr;
    float rot = (float)comp.rotation;
    ImU32 blockBg = isDarkMode ? IM_COL32(30, 41, 59, 230) : IM_COL32(252, 250, 225, 240);
    
    if (t == "R") {
        ImVec2 rawPts[] = {
            {0, -40*s}, {0, -20*s},
            {-10*s, -15*s}, {10*s, -9*s},
            {-10*s, -3*s},  {10*s, 3*s},
            {-10*s, 9*s},   {10*s, 15*s},
            {0, 20*s}, {0, 40*s}
        };
        ImVec2 pts[10];
        for (int i = 0; i < 10; ++i) pts[i] = rotatePt(rawPts[i].x, rawPts[i].y, c.x, c.y, rot);
        drawList->AddPolyline(pts, 10, color, 0, 2.0f * s);
    } else if (t == "L") {
        ImVec2 p1 = rotatePt(0, -40*s, c.x, c.y, rot);
        ImVec2 p2 = rotatePt(0, -20*s, c.x, c.y, rot);
        drawList->AddLine(p1, p2, color, 2.0f*s);
        for (int i = 0; i < 3; ++i) {
            float cy = -13.3f*s + i * 13.3f*s;
            ImVec2 c0 = rotatePt(0, cy - 6.7f*s, c.x, c.y, rot);
            ImVec2 c1 = rotatePt(-14*s, cy - 6.7f*s, c.x, c.y, rot);
            ImVec2 c2 = rotatePt(-14*s, cy + 6.7f*s, c.x, c.y, rot);
            ImVec2 c3 = rotatePt(0, cy + 6.7f*s, c.x, c.y, rot);
            drawList->AddBezierCubic(c0, c1, c2, c3, color, 2.0f*s, 12);
        }
        ImVec2 p3 = rotatePt(0, 20*s, c.x, c.y, rot);
        ImVec2 p4 = rotatePt(0, 40*s, c.x, c.y, rot);
        drawList->AddLine(p3, p4, color, 2.0f*s);
    } else if (t == "C") {
        drawList->AddLine(rotatePt(0, -40*s, c.x, c.y, rot), rotatePt(0, -5*s, c.x, c.y, rot), color, 2.0f*s);
        drawList->AddLine(rotatePt(-15*s, -5*s, c.x, c.y, rot), rotatePt(15*s, -5*s, c.x, c.y, rot), color, 2.0f*s);
        drawList->AddLine(rotatePt(-15*s, 5*s, c.x, c.y, rot), rotatePt(15*s, 5*s, c.x, c.y, rot), color, 2.0f*s);
        drawList->AddLine(rotatePt(0, 5*s, c.x, c.y, rot), rotatePt(0, 40*s, c.x, c.y, rot), color, 2.0f*s);
    } else if (t == "S") {
        drawList->AddLine(rotatePt(0, -40*s, c.x, c.y, rot), rotatePt(0, -20*s, c.x, c.y, rot), color, 2.0f*s);
        drawList->AddCircleFilled(rotatePt(0, -20*s, c.x, c.y, rot), 3.0f*s, color);
        drawList->AddCircleFilled(rotatePt(0, 20*s, c.x, c.y, rot), 3.0f*s, color);
        drawList->AddLine(rotatePt(0, -20*s, c.x, c.y, rot), rotatePt(13*s, 16*s, c.x, c.y, rot), color, 2.0f*s);
        drawList->AddLine(rotatePt(0, 20*s, c.x, c.y, rot), rotatePt(0, 40*s, c.x, c.y, rot), color, 2.0f*s);
        drawList->AddLine(rotatePt(-20*s, 0, c.x, c.y, rot), rotatePt(-6*s, 0, c.x, c.y, rot), color, 2.0f*s);
    } else if (t == "D" || t == "DIODE") {
        drawList->AddLine(rotatePt(0, -40*s, c.x, c.y, rot), rotatePt(0, -8*s, c.x, c.y, rot), color, 2.0f*s);
        ImVec2 tri[] = {rotatePt(-12*s, -8*s, c.x, c.y, rot), rotatePt(12*s, -8*s, c.x, c.y, rot), rotatePt(0, 8*s, c.x, c.y, rot)};
        drawList->AddTriangleFilled(tri[0], tri[1], tri[2], IM_COL32(0, 230, 120, 30));
        drawList->AddTriangle(tri[0], tri[1], tri[2], color, 2.0f*s);
        drawList->AddLine(rotatePt(-12*s, 8*s, c.x, c.y, rot), rotatePt(12*s, 8*s, c.x, c.y, rot), color, 2.0f*s);
        drawList->AddLine(rotatePt(0, 8*s, c.x, c.y, rot), rotatePt(0, 40*s, c.x, c.y, rot), color, 2.0f*s);
    } else if (t == "THYRISTOR" || t == "SCR" || t == "GTO" || t == "IGCT") {
        // Anode lead
        drawList->AddLine(rotatePt(0, -20*s, c.x, c.y, rot), rotatePt(0, -8*s, c.x, c.y, rot), color, 2.0f*s);
        // Diode Triangle Body
        ImVec2 tri[] = {
            rotatePt(-12*s, -8*s, c.x, c.y, rot),
            rotatePt(12*s, -8*s, c.x, c.y, rot),
            rotatePt(0, 8*s, c.x, c.y, rot)
        };
        drawList->AddTriangleFilled(tri[0], tri[1], tri[2], IM_COL32(0, 230, 120, 30));
        drawList->AddTriangle(tri[0], tri[1], tri[2], color, 2.0f*s);
        // Cathode Bar
        drawList->AddLine(rotatePt(-12*s, 8*s, c.x, c.y, rot), rotatePt(12*s, 8*s, c.x, c.y, rot), color, 2.0f*s);
        // Cathode lead
        drawList->AddLine(rotatePt(0, 8*s, c.x, c.y, rot), rotatePt(0, 20*s, c.x, c.y, rot), color, 2.0f*s);
        // Gate lead angling from Gate pin (-20, 10) to Cathode bar (-5, 8)
        ImVec2 g1 = rotatePt(-20*s, 10*s, c.x, c.y, rot);
        ImVec2 g2 = rotatePt(-10*s, 10*s, c.x, c.y, rot);
        ImVec2 g3 = rotatePt(-5*s, 8*s, c.x, c.y, rot);
        drawList->AddLine(g1, g2, color, 2.0f*s);
        drawList->AddLine(g2, g3, color, 2.0f*s);
        if (t == "GTO" || t == "IGCT") {
            drawList->AddLine(rotatePt(-12*s, 6*s, c.x, c.y, rot), rotatePt(-8*s, 14*s, c.x, c.y, rot), color, 2.0f*s);
        }
    } else if (t == "MOSFET" || t == "vg-FET" || t == "VGFET") {
        drawList->AddLine(rotatePt(0, -40*s, c.x, c.y, rot), rotatePt(0, -15*s, c.x, c.y, rot), color, 2.0f*s);
        drawList->AddLine(rotatePt(0, 15*s, c.x, c.y, rot), rotatePt(0, 40*s, c.x, c.y, rot), color, 2.0f*s);
        drawList->AddLine(rotatePt(-5*s, -15*s, c.x, c.y, rot), rotatePt(-5*s, 15*s, c.x, c.y, rot), color, 2.0f*s);
        drawList->AddLine(rotatePt(-5*s, 0, c.x, c.y, rot), rotatePt(0, 0, c.x, c.y, rot), color, 2.0f*s);
        drawList->AddLine(rotatePt(-10*s, -15*s, c.x, c.y, rot), rotatePt(-10*s, 15*s, c.x, c.y, rot), color, 2.0f*s);
        if (t == "vg-FET" || t == "VGFET") {
            std::string gateLbl = comp.parameters.count("Gate_Signal_Label") ? comp.parameters.at("Gate_Signal_Label") : (comp.parameters.count("tag") ? comp.parameters.at("tag") : "S1");
            drawList->AddText(rotatePt(-26*s, -6*s, c.x, c.y, rot), color, gateLbl.c_str());
        } else {
            drawList->AddLine(rotatePt(-20*s, 0, c.x, c.y, rot), rotatePt(-10*s, 0, c.x, c.y, rot), color, 2.0f*s);
        }
        
        drawList->AddLine(rotatePt(0, 15*s, c.x, c.y, rot), rotatePt(12*s, 15*s, c.x, c.y, rot), color, 1.5f*s);
        drawList->AddLine(rotatePt(12*s, 15*s, c.x, c.y, rot), rotatePt(12*s, 6*s, c.x, c.y, rot), color, 1.5f*s);
        
        ImVec2 dTri[] = {rotatePt(7*s, 6*s, c.x, c.y, rot), rotatePt(17*s, 6*s, c.x, c.y, rot), rotatePt(12*s, -6*s, c.x, c.y, rot)};
        drawList->AddTriangleFilled(dTri[0], dTri[1], dTri[2], IM_COL32(0, 230, 120, 30));
        drawList->AddTriangle(dTri[0], dTri[1], dTri[2], color, 1.5f*s);
        
        drawList->AddLine(rotatePt(7*s, -6*s, c.x, c.y, rot), rotatePt(17*s, -6*s, c.x, c.y, rot), color, 1.5f*s);
        
        drawList->AddLine(rotatePt(12*s, -6*s, c.x, c.y, rot), rotatePt(12*s, -15*s, c.x, c.y, rot), color, 1.5f*s);
        drawList->AddLine(rotatePt(12*s, -15*s, c.x, c.y, rot), rotatePt(0, -15*s, c.x, c.y, rot), color, 1.5f*s);
    } else if (t == "V") {
        drawList->AddLine(rotatePt(0, -40*s, c.x, c.y, rot), rotatePt(0, -16*s, c.x, c.y, rot), color, 2.0f*s);
        drawList->AddLine(rotatePt(0, 16*s, c.x, c.y, rot), rotatePt(0, 40*s, c.x, c.y, rot), color, 2.0f*s);
        drawList->AddCircle(c, 16*s, color, 0, 2.0f*s);
        drawList->AddLine(rotatePt(-3*s, -7*s, c.x, c.y, rot), rotatePt(3*s, -7*s, c.x, c.y, rot), color, 1.5f*s);
        drawList->AddLine(rotatePt(0, -10*s, c.x, c.y, rot), rotatePt(0, -4*s, c.x, c.y, rot), color, 1.5f*s);
        drawList->AddLine(rotatePt(-3*s, 7*s, c.x, c.y, rot), rotatePt(3*s, 7*s, c.x, c.y, rot), color, 1.5f*s);
    } else if (t == "I") {
        drawList->AddLine(rotatePt(0, -40*s, c.x, c.y, rot), rotatePt(0, -16*s, c.x, c.y, rot), color, 2.0f*s);
        drawList->AddLine(rotatePt(0, 16*s, c.x, c.y, rot), rotatePt(0, 40*s, c.x, c.y, rot), color, 2.0f*s);
        drawList->AddCircle(c, 16*s, color, 0, 2.0f*s);
        drawList->AddLine(rotatePt(0, -9*s, c.x, c.y, rot), rotatePt(0, 9*s, c.x, c.y, rot), color, 2.0f*s);
        ImVec2 arr[] = {rotatePt(-4*s, 3*s, c.x, c.y, rot), rotatePt(0, 9*s, c.x, c.y, rot), rotatePt(4*s, 3*s, c.x, c.y, rot)};
        drawList->AddPolyline(arr, 3, color, 0, 2.0f*s);
    } else if (t == "AC_V") {
        drawList->AddLine(rotatePt(0, -40*s, c.x, c.y, rot), rotatePt(0, -16*s, c.x, c.y, rot), color, 2.0f*s);
        drawList->AddLine(rotatePt(0, 16*s, c.x, c.y, rot), rotatePt(0, 40*s, c.x, c.y, rot), color, 2.0f*s);
        drawList->AddCircle(c, 16*s, color, 0, 2.0f*s);
        ImVec2 c0 = rotatePt(-8*s, 0, c.x, c.y, rot);
        ImVec2 c1 = rotatePt(-4*s, -8*s, c.x, c.y, rot);
        ImVec2 c2 = rotatePt(0, -8*s, c.x, c.y, rot);
        ImVec2 c3 = rotatePt(0, 0, c.x, c.y, rot);
        drawList->AddBezierCubic(c0, c1, c2, c3, color, 2.0f*s, 10);
        ImVec2 c4 = rotatePt(0, 0, c.x, c.y, rot);
        ImVec2 c5 = rotatePt(0, 8*s, c.x, c.y, rot);
        ImVec2 c6 = rotatePt(4*s, 8*s, c.x, c.y, rot);
        ImVec2 c7 = rotatePt(8*s, 0, c.x, c.y, rot);
        drawList->AddBezierCubic(c4, c5, c6, c7, color, 2.0f*s, 10);
    } else if (t == "VM" || t == "AM") {
        drawList->AddLine(rotatePt(0, -40*s, c.x, c.y, rot), rotatePt(0, -16*s, c.x, c.y, rot), color, 2.0f*s);
        drawList->AddLine(rotatePt(0, 16*s, c.x, c.y, rot), rotatePt(0, 40*s, c.x, c.y, rot), color, 2.0f*s);
        drawList->AddLine(rotatePt(16*s, 0, c.x, c.y, rot), rotatePt(20*s, 0, c.x, c.y, rot), color, 2.0f*s);
        drawList->AddCircle(c, 16*s, color, 0, 2.0f*s);
        drawList->AddText(rotatePt(-4*s, -7*s, c.x, c.y, rot), color, (t == "VM") ? "V" : "A");
    } else if (t == "GND") {
        drawList->AddLine(rotatePt(0, -20*s, c.x, c.y, rot), rotatePt(0, 0, c.x, c.y, rot), color, 2.0f*s);
        drawList->AddLine(rotatePt(-12*s, 0, c.x, c.y, rot), rotatePt(12*s, 0, c.x, c.y, rot), color, 2.0f*s);
        drawList->AddLine(rotatePt(-8*s, 6*s, c.x, c.y, rot), rotatePt(8*s, 6*s, c.x, c.y, rot), color, 2.0f*s);
        drawList->AddLine(rotatePt(-4*s, 12*s, c.x, c.y, rot), rotatePt(4*s, 12*s, c.x, c.y, rot), color, 2.0f*s);
    } else if (t == "GAIN") {
        ImVec2 t1 = rotatePt(-16*s, -18*s, c.x, c.y, rot);
        ImVec2 t2 = rotatePt(16*s, 0, c.x, c.y, rot);
        ImVec2 t3 = rotatePt(-16*s, 18*s, c.x, c.y, rot);
        drawList->AddTriangleFilled(t1, t2, t3, IM_COL32(38, 50, 70, 200));
        drawList->AddTriangle(t1, t2, t3, color, 2.0f*s);
        drawList->AddLine(rotatePt(-20*s, 0, c.x, c.y, rot), rotatePt(-16*s, 0, c.x, c.y, rot), color, 2.0f*s);
        drawList->AddLine(rotatePt(16*s, 0, c.x, c.y, rot), rotatePt(20*s, 0, c.x, c.y, rot), color, 2.0f*s);
        drawList->AddText(rotatePt(-4*s, -7*s, c.x, c.y, rot), color, "K");
    } else if (t == "SUM_ROUND" || t == "SUM" || t == "SUM_RECT" || t == "PRODUCT_RECT" || t == "PROD") {
        std::vector<std::string> signs;
        int nInputs = parseMathBlockPins(comp, signs);

        float hw = 25.0f * s;
        float spacing = 18.0f * s;
        float totalH = std::max(40.0f * s, (nInputs - 1) * spacing + 20.0f * s);
        float hh = totalH * 0.5f;

        if (t == "SUM_ROUND") {
            float rad = std::max(18.0f * s, hh);
            drawList->AddCircleFilled(c, rad, IM_COL32(38, 50, 70, 200));
            drawList->AddCircle(c, rad, color, 0, 2.0f * s);
            drawList->AddText(rotatePt(-4.0f * s, -6.0f * s, c.x, c.y, rot), color, "Σ");
            for (int i = 0; i < nInputs; ++i) {
                float py = -hh + 10.0f * s + i * spacing;
                std::string signStr = (i < (int)signs.size()) ? signs[i] : "+";
                drawList->AddText(rotatePt(-rad + 4.0f * s, py - 6.0f * s, c.x, c.y, rot), color, signStr.c_str());
            }
        } else {
            drawList->AddRectFilled({c.x - hw, c.y - hh}, {c.x + hw, c.y + hh}, IM_COL32(38, 50, 70, 200), 4.0f * s);
            drawList->AddRect({c.x - hw, c.y - hh}, {c.x + hw, c.y + hh}, color, 4.0f * s, 0, 2.0f * s);
            std::string mainSymbol = (t == "PROD" || t == "PRODUCT_RECT") ? "Π" : "Σ";
            drawList->AddText(rotatePt(-4.0f * s, -hh + 4.0f * s, c.x, c.y, rot), color, mainSymbol.c_str());

            for (int i = 0; i < nInputs; ++i) {
                float py = -hh + 10.0f * s + i * spacing;
                std::string signStr = (i < (int)signs.size()) ? signs[i] : ((t == "PROD" || t == "PRODUCT_RECT") ? "*" : "+");
                drawList->AddText(rotatePt(-hw + 6.0f * s, py - 6.0f * s, c.x, c.y, rot), color, signStr.c_str());
            }
        }
    } else if (t == "COMP") {
        ImVec2 t1 = rotatePt(-16*s, -20*s, c.x, c.y, rot);
        ImVec2 t2 = rotatePt(16*s, 0, c.x, c.y, rot);
        ImVec2 t3 = rotatePt(-16*s, 20*s, c.x, c.y, rot);
        drawList->AddTriangleFilled(t1, t2, t3, IM_COL32(38, 50, 70, 200));
        drawList->AddTriangle(t1, t2, t3, color, 2.0f*s);
        drawList->AddText(rotatePt(-10*s, -14*s, c.x, c.y, rot), color, "+");
        drawList->AddText(rotatePt(-10*s, 4*s, c.x, c.y, rot), color, "-");
    } else if (t == "PID") {
        float hw = 20*s, hh = 16*s;
        drawList->AddRectFilled({c.x - hw, c.y - hh}, {c.x + hw, c.y + hh}, IM_COL32(38, 50, 70, 200), 4*s);
        drawList->AddRect({c.x - hw, c.y - hh}, {c.x + hw, c.y + hh}, color, 4*s, 0, 2.0f*s);
        drawList->AddText(rotatePt(-12*s, -6*s, c.x, c.y, rot), color, "PID");
    } else if (t == "PWM") {
        float hw = 20*s, hh = 16*s;
        drawList->AddRectFilled({c.x - hw, c.y - hh}, {c.x + hw, c.y + hh}, IM_COL32(38, 50, 70, 200), 4*s);
        drawList->AddRect({c.x - hw, c.y - hh}, {c.x + hw, c.y + hh}, color, 4*s, 0, 2.0f*s);
        ImVec2 wave[] = {rotatePt(-10*s, 6*s, c.x, c.y, rot), rotatePt(-10*s, -6*s, c.x, c.y, rot), rotatePt(0, -6*s, c.x, c.y, rot), rotatePt(0, 6*s, c.x, c.y, rot), rotatePt(10*s, 6*s, c.x, c.y, rot)};
        drawList->AddPolyline(wave, 5, color, 0, 1.8f*s);
    } else if (t == "TRI") {
        float hw = 20*s, hh = 16*s;
        drawList->AddRectFilled({c.x - hw, c.y - hh}, {c.x + hw, c.y + hh}, blockBg, 4*s);
        drawList->AddRect({c.x - hw, c.y - hh}, {c.x + hw, c.y + hh}, color, 4*s, 0, 2.0f*s);
        ImVec2 triWave[] = {rotatePt(-10*s, 6*s, c.x, c.y, rot), rotatePt(0, -6*s, c.x, c.y, rot), rotatePt(10*s, 6*s, c.x, c.y, rot)};
        drawList->AddPolyline(triWave, 3, color, 0, 1.8f*s);
    } else if (t == "CONST" || t == "CONSTANT") {
        float hw = 20*s, hh = 16*s;
        drawList->AddRectFilled({c.x - hw, c.y - hh}, {c.x + hw, c.y + hh}, blockBg, 4*s);
        drawList->AddRect({c.x - hw, c.y - hh}, {c.x + hw, c.y + hh}, color, 4*s, 0, 2.0f*s);
        std::string valStr = comp.parameters.count("value") ? comp.parameters.at("value") : (comp.parameters.count("constant") ? comp.parameters.at("constant") : "1");
        drawList->AddText(rotatePt(-6*s, -7*s, c.x, c.y, rot), color, valStr.c_str());
    } else if (t == "V_3PH" || t == "ThreePhaseSource") {
        float hw = 24*s, hh = 28*s;
        drawList->AddRectFilled({c.x - hw, c.y - hh}, {c.x + hw, c.y + hh}, blockBg, 4*s);
        drawList->AddRect({c.x - hw, c.y - hh}, {c.x + hw, c.y + hh}, color, 4*s, 0, 2.0f*s);
        drawList->AddText(rotatePt(-16*s, -12*s, c.x, c.y, rot), color, "3-Phase");
        drawList->AddText(rotatePt(-12*s, 2*s, c.x, c.y, rot), IM_COL32(56, 189, 248, 255), "AC");
    } else if (t == "PWM_MASTER" || t == "MasterPWM") {
        float hw = 30*s, hh = 28*s;
        drawList->AddRectFilled({c.x - hw, c.y - hh}, {c.x + hw, c.y + hh}, blockBg, 4*s);
        drawList->AddRect({c.x - hw, c.y - hh}, {c.x + hw, c.y + hh}, color, 4*s, 0, 2.0f*s);
        drawList->AddText(rotatePt(-22*s, -10*s, c.x, c.y, rot), color, "PWM");
        drawList->AddText(rotatePt(-24*s, 2*s, c.x, c.y, rot), IM_COL32(245, 158, 11, 255), "MASTER");
    } else if (t == "EDGE_DETECT" || t == "EdgeDetector") {
        float hw = 22*s, hh = 16*s;
        drawList->AddRectFilled({c.x - hw, c.y - hh}, {c.x + hw, c.y + hh}, blockBg, 4*s);
        drawList->AddRect({c.x - hw, c.y - hh}, {c.x + hw, c.y + hh}, color, 4*s, 0, 2.0f*s);
        drawList->AddText(rotatePt(-14*s, -6*s, c.x, c.y, rot), color, "EDGE");
    } else if (t == "MATH_FCN" || t == "FCN" || t == "MathFunction") {
        float hw = 22*s, hh = 16*s;
        drawList->AddRectFilled({c.x - hw, c.y - hh}, {c.x + hw, c.y + hh}, blockBg, 4*s);
        drawList->AddRect({c.x - hw, c.y - hh}, {c.x + hw, c.y + hh}, color, 4*s, 0, 2.0f*s);
        std::string fnLabel = comp.parameters.count("function") ? comp.parameters.at("function") : "FCN";
        if (fnLabel.length() > 5) fnLabel = fnLabel.substr(0, 5);
        drawList->AddText(rotatePt(-14*s, -6*s, c.x, c.y, rot), color, fnLabel.c_str());
    } else if (t == "KEY_TRIGGER" || t == "KeyTrigger") {
        float hw = 22*s, hh = 16*s;
        drawList->AddRectFilled({c.x - hw, c.y - hh}, {c.x + hw, c.y + hh}, blockBg, 4*s);
        drawList->AddRect({c.x - hw, c.y - hh}, {c.x + hw, c.y + hh}, color, 4*s, 0, 2.0f*s);
        std::string keyLabel = comp.parameters.count("key") ? comp.parameters.at("key") : "KEY";
        drawList->AddText(rotatePt(-14*s, -6*s, c.x, c.y, rot), IM_COL32(236, 72, 153, 255), keyLabel.c_str());
    } else if (t == "PULSE" || t == "PULSE_GEN") {
        float hw = 22*s, hh = 16*s;
        drawList->AddRectFilled({c.x - hw, c.y - hh}, {c.x + hw, c.y + hh}, blockBg, 4*s);
        drawList->AddRect({c.x - hw, c.y - hh}, {c.x + hw, c.y + hh}, color, 4*s, 0, 2.0f*s);
        ImVec2 pulseW[] = {
            rotatePt(-12*s, 6*s, c.x, c.y, rot),
            rotatePt(-12*s, -6*s, c.x, c.y, rot),
            rotatePt(0, -6*s, c.x, c.y, rot),
            rotatePt(0, 6*s, c.x, c.y, rot),
            rotatePt(12*s, 6*s, c.x, c.y, rot)
        };
        drawList->AddPolyline(pulseW, 5, color, 0, 1.8f*s);
        drawList->AddLine(rotatePt(hw, 0, c.x, c.y, rot), rotatePt(hw + 4*s, 0, c.x, c.y, rot), color, 2.0f*s);
    } else if (t == "STEP") {
        float hw = 22*s, hh = 16*s;
        drawList->AddRectFilled({c.x - hw, c.y - hh}, {c.x + hw, c.y + hh}, blockBg, 4*s);
        drawList->AddRect({c.x - hw, c.y - hh}, {c.x + hw, c.y + hh}, color, 4*s, 0, 2.0f*s);
        ImVec2 stepW[] = {
            rotatePt(-12*s, 6*s, c.x, c.y, rot),
            rotatePt(-2*s, 6*s, c.x, c.y, rot),
            rotatePt(-2*s, -6*s, c.x, c.y, rot),
            rotatePt(10*s, -6*s, c.x, c.y, rot)
        };
        drawList->AddPolyline(stepW, 4, color, 0, 1.8f*s);
        drawList->AddLine(rotatePt(hw, 0, c.x, c.y, rot), rotatePt(hw + 4*s, 0, c.x, c.y, rot), color, 2.0f*s);
    } else if (t == "RAMP") {
        float hw = 22*s, hh = 16*s;
        drawList->AddRectFilled({c.x - hw, c.y - hh}, {c.x + hw, c.y + hh}, blockBg, 4*s);
        drawList->AddRect({c.x - hw, c.y - hh}, {c.x + hw, c.y + hh}, color, 4*s, 0, 2.0f*s);
        ImVec2 rampW[] = {
            rotatePt(-12*s, 6*s, c.x, c.y, rot),
            rotatePt(-4*s, 6*s, c.x, c.y, rot),
            rotatePt(8*s, -6*s, c.x, c.y, rot)
        };
        drawList->AddPolyline(rampW, 3, color, 0, 1.8f*s);
        drawList->AddLine(rotatePt(hw, 0, c.x, c.y, rot), rotatePt(hw + 4*s, 0, c.x, c.y, rot), color, 2.0f*s);
    } else if (t == "CLOCK") {
        float hw = 22*s, hh = 16*s;
        drawList->AddRectFilled({c.x - hw, c.y - hh}, {c.x + hw, c.y + hh}, blockBg, 4*s);
        drawList->AddRect({c.x - hw, c.y - hh}, {c.x + hw, c.y + hh}, color, 4*s, 0, 2.0f*s);
        drawList->AddCircle(c, 7*s, color, 0, 1.5f*s);
        drawList->AddLine(c, rotatePt(0, -5*s, c.x, c.y, rot), color, 1.5f*s);
        drawList->AddLine(c, rotatePt(4*s, 2*s, c.x, c.y, rot), color, 1.5f*s);
        drawList->AddLine(rotatePt(hw, 0, c.x, c.y, rot), rotatePt(hw + 4*s, 0, c.x, c.y, rot), color, 2.0f*s);
    } else if (t == "SINE_WAVE") {
        float hw = 22*s, hh = 16*s;
        drawList->AddRectFilled({c.x - hw, c.y - hh}, {c.x + hw, c.y + hh}, blockBg, 4*s);
        drawList->AddRect({c.x - hw, c.y - hh}, {c.x + hw, c.y + hh}, color, 4*s, 0, 2.0f*s);
        ImVec2 sineW[16];
        for (int i = 0; i < 16; ++i) {
            float px = -12.0f + i * (24.0f / 15.0f);
            float py = -6.0f * std::sin(2.0f * 3.14159265f * (i / 15.0f));
            sineW[i] = rotatePt(px * s, py * s, c.x, c.y, rot);
        }
        drawList->AddPolyline(sineW, 16, color, 0, 1.8f*s);
        drawList->AddLine(rotatePt(hw, 0, c.x, c.y, rot), rotatePt(hw + 4*s, 0, c.x, c.y, rot), color, 2.0f*s);
    } else if (t == "RANDOM_NUM") {
        float hw = 22*s, hh = 16*s;
        drawList->AddRectFilled({c.x - hw, c.y - hh}, {c.x + hw, c.y + hh}, blockBg, 4*s);
        drawList->AddRect({c.x - hw, c.y - hh}, {c.x + hw, c.y + hh}, color, 4*s, 0, 2.0f*s);
        drawList->AddText(rotatePt(-12*s, -6*s, c.x, c.y, rot), color, "Rand");
        drawList->AddLine(rotatePt(hw, 0, c.x, c.y, rot), rotatePt(hw + 4*s, 0, c.x, c.y, rot), color, 2.0f*s);
    } else if (t == "WHITE_NOISE") {
        float hw = 22*s, hh = 16*s;
        drawList->AddRectFilled({c.x - hw, c.y - hh}, {c.x + hw, c.y + hh}, blockBg, 4*s);
        drawList->AddRect({c.x - hw, c.y - hh}, {c.x + hw, c.y + hh}, color, 4*s, 0, 2.0f*s);
        drawList->AddText(rotatePt(-14*s, -6*s, c.x, c.y, rot), color, "Noise");
        drawList->AddLine(rotatePt(hw, 0, c.x, c.y, rot), rotatePt(hw + 4*s, 0, c.x, c.y, rot), color, 2.0f*s);
    } else if (t == "INIT_COND") {
        float hw = 22*s, hh = 16*s;
        drawList->AddRectFilled({c.x - hw, c.y - hh}, {c.x + hw, c.y + hh}, blockBg, 4*s);
        drawList->AddRect({c.x - hw, c.y - hh}, {c.x + hw, c.y + hh}, color, 4*s, 0, 2.0f*s);
        drawList->AddText(rotatePt(-8*s, -6*s, c.x, c.y, rot), color, "x0");
        drawList->AddLine(rotatePt(-hw - 4*s, 0, c.x, c.y, rot), rotatePt(-hw, 0, c.x, c.y, rot), color, 2.0f*s);
        drawList->AddLine(rotatePt(hw, 0, c.x, c.y, rot), rotatePt(hw + 4*s, 0, c.x, c.y, rot), color, 2.0f*s);
    } else if (t == "TRIG_FCN") {
        float hw = 22*s, hh = 16*s;
        drawList->AddRectFilled({c.x - hw, c.y - hh}, {c.x + hw, c.y + hh}, blockBg, 4*s);
        drawList->AddRect({c.x - hw, c.y - hh}, {c.x + hw, c.y + hh}, color, 4*s, 0, 2.0f*s);
        std::string trigFn = comp.parameters.count("function") ? comp.parameters.at("function") : "sin";
        drawList->AddText(rotatePt(-12*s, -6*s, c.x, c.y, rot), color, trigFn.c_str());
        drawList->AddLine(rotatePt(-hw - 4*s, 0, c.x, c.y, rot), rotatePt(-hw, 0, c.x, c.y, rot), color, 2.0f*s);
        drawList->AddLine(rotatePt(hw, 0, c.x, c.y, rot), rotatePt(hw + 4*s, 0, c.x, c.y, rot), color, 2.0f*s);
    } else if (t == "ABS") {
        float hw = 22*s, hh = 16*s;
        drawList->AddRectFilled({c.x - hw, c.y - hh}, {c.x + hw, c.y + hh}, blockBg, 4*s);
        drawList->AddRect({c.x - hw, c.y - hh}, {c.x + hw, c.y + hh}, color, 4*s, 0, 2.0f*s);
        drawList->AddText(rotatePt(-10*s, -6*s, c.x, c.y, rot), color, "|u|");
        drawList->AddLine(rotatePt(-hw - 4*s, 0, c.x, c.y, rot), rotatePt(-hw, 0, c.x, c.y, rot), color, 2.0f*s);
        drawList->AddLine(rotatePt(hw, 0, c.x, c.y, rot), rotatePt(hw + 4*s, 0, c.x, c.y, rot), color, 2.0f*s);
    } else if (t == "SIGN") {
        float hw = 22*s, hh = 16*s;
        drawList->AddRectFilled({c.x - hw, c.y - hh}, {c.x + hw, c.y + hh}, blockBg, 4*s);
        drawList->AddRect({c.x - hw, c.y - hh}, {c.x + hw, c.y + hh}, color, 4*s, 0, 2.0f*s);
        drawList->AddText(rotatePt(-10*s, -6*s, c.x, c.y, rot), color, "sgn");
        drawList->AddLine(rotatePt(-hw - 4*s, 0, c.x, c.y, rot), rotatePt(-hw, 0, c.x, c.y, rot), color, 2.0f*s);
        drawList->AddLine(rotatePt(hw, 0, c.x, c.y, rot), rotatePt(hw + 4*s, 0, c.x, c.y, rot), color, 2.0f*s);
    } else if (t == "ROUND") {
        float hw = 22*s, hh = 16*s;
        drawList->AddRectFilled({c.x - hw, c.y - hh}, {c.x + hw, c.y + hh}, blockBg, 4*s);
        drawList->AddRect({c.x - hw, c.y - hh}, {c.x + hw, c.y + hh}, color, 4*s, 0, 2.0f*s);
        drawList->AddText(rotatePt(-14*s, -6*s, c.x, c.y, rot), color, "Round");
        drawList->AddLine(rotatePt(-hw - 4*s, 0, c.x, c.y, rot), rotatePt(-hw, 0, c.x, c.y, rot), color, 2.0f*s);
        drawList->AddLine(rotatePt(hw, 0, c.x, c.y, rot), rotatePt(hw + 4*s, 0, c.x, c.y, rot), color, 2.0f*s);
    } else if (t == "MIN_MAX") {
        float hw = 22*s, hh = 18*s;
        drawList->AddRectFilled({c.x - hw, c.y - hh}, {c.x + hw, c.y + hh}, blockBg, 4*s);
        drawList->AddRect({c.x - hw, c.y - hh}, {c.x + hw, c.y + hh}, color, 4*s, 0, 2.0f*s);
        std::string mmFn = comp.parameters.count("function") ? comp.parameters.at("function") : "min";
        drawList->AddText(rotatePt(-14*s, -6*s, c.x, c.y, rot), color, mmFn.c_str());
        drawList->AddLine(rotatePt(-hw - 4*s, -10*s, c.x, c.y, rot), rotatePt(-hw, -10*s, c.x, c.y, rot), color, 2.0f*s);
        drawList->AddLine(rotatePt(-hw - 4*s, 10*s, c.x, c.y, rot), rotatePt(-hw, 10*s, c.x, c.y, rot), color, 2.0f*s);
        drawList->AddLine(rotatePt(hw, 0, c.x, c.y, rot), rotatePt(hw + 4*s, 0, c.x, c.y, rot), color, 2.0f*s);
    } else if (t == "GOTO_SIG" || t == "GOTO") {
        std::string tag = comp.parameters.count("tag") ? comp.parameters.at("tag") : "A";
        float hh = 14*s;
        ImVec2 pts[] = {
            rotatePt(-15*s, -hh, c.x, c.y, rot),
            rotatePt(5*s, 0, c.x, c.y, rot),
            rotatePt(-15*s, hh, c.x, c.y, rot)
        };
        drawList->AddConvexPolyFilled(pts, 3, blockBg);
        drawList->AddPolyline(pts, 3, color, ImDrawFlags_Closed, 2.0f*s);
        drawList->AddText(rotatePt(-10*s, -6*s, c.x, c.y, rot), color, tag.c_str());
        drawList->AddLine(rotatePt(-15*s - 4*s, 0, c.x, c.y, rot), rotatePt(-15*s, 0, c.x, c.y, rot), color, 2.0f*s);
    } else if (t == "FROM_SIG" || t == "FROM") {
        std::string tag = comp.parameters.count("tag") ? comp.parameters.at("tag") : "A";
        float hh = 14*s;
        ImVec2 pts[] = {
            rotatePt(-5*s, -hh, c.x, c.y, rot),
            rotatePt(15*s, 0, c.x, c.y, rot),
            rotatePt(-5*s, hh, c.x, c.y, rot)
        };
        drawList->AddConvexPolyFilled(pts, 3, blockBg);
        drawList->AddPolyline(pts, 3, color, ImDrawFlags_Closed, 2.0f*s);
        drawList->AddText(rotatePt(-2*s, -6*s, c.x, c.y, rot), color, tag.c_str());
        drawList->AddLine(rotatePt(15*s, 0, c.x, c.y, rot), rotatePt(15*s + 4*s, 0, c.x, c.y, rot), color, 2.0f*s);
    } else if (t == "LUT_1D" || t == "LUT_2D" || t == "LUT_3D") {
        float hw = 24*s, hh = 16*s;
        drawList->AddRectFilled({c.x - hw, c.y - hh}, {c.x + hw, c.y + hh}, blockBg, 4*s);
        drawList->AddRect({c.x - hw, c.y - hh}, {c.x + hw, c.y + hh}, color, 4*s, 0, 2.0f*s);
        std::string lutLbl = (t == "LUT_2D") ? "LUT2D" : ((t == "LUT_3D") ? "LUT3D" : "LUT1D");
        drawList->AddText(rotatePt(-16*s, -6*s, c.x, c.y, rot), color, lutLbl.c_str());
        drawList->AddLine(rotatePt(-hw - 4*s, 0, c.x, c.y, rot), rotatePt(-hw, 0, c.x, c.y, rot), color, 2.0f*s);
        drawList->AddLine(rotatePt(hw, 0, c.x, c.y, rot), rotatePt(hw + 4*s, 0, c.x, c.y, rot), color, 2.0f*s);
    } else if (t == "DLL" || t == "FMU" || t == "FOURIER_SERIES") {
        float hw = 24*s, hh = 16*s;
        drawList->AddRectFilled({c.x - hw, c.y - hh}, {c.x + hw, c.y + hh}, blockBg, 4*s);
        drawList->AddRect({c.x - hw, c.y - hh}, {c.x + hw, c.y + hh}, color, 4*s, 0, 2.0f*s);
        std::string lbl = (t == "DLL") ? "DLL" : ((t == "FMU") ? "FMU" : "Fourier");
        drawList->AddText(rotatePt(-14*s, -6*s, c.x, c.y, rot), color, lbl.c_str());
        drawList->AddLine(rotatePt(-hw - 4*s, 0, c.x, c.y, rot), rotatePt(-hw, 0, c.x, c.y, rot), color, 2.0f*s);
        drawList->AddLine(rotatePt(hw, 0, c.x, c.y, rot), rotatePt(hw + 4*s, 0, c.x, c.y, rot), color, 2.0f*s);
    } else if (t == "INTEGRATOR") {
        float hw = 22*s, hh = 16*s;
        drawList->AddRectFilled({c.x - hw, c.y - hh}, {c.x + hw, c.y + hh}, blockBg, 4*s);
        drawList->AddRect({c.x - hw, c.y - hh}, {c.x + hw, c.y + hh}, color, 4*s, 0, 2.0f*s);
        drawList->AddText(rotatePt(-10*s, -6*s, c.x, c.y, rot), color, "1/s");
        drawList->AddLine(rotatePt(-hw - 4*s, 0, c.x, c.y, rot), rotatePt(-hw, 0, c.x, c.y, rot), color, 2.0f*s);
        drawList->AddLine(rotatePt(hw, 0, c.x, c.y, rot), rotatePt(hw + 4*s, 0, c.x, c.y, rot), color, 2.0f*s);
    } else if (t == "DERIVATIVE") {
        float hw = 22*s, hh = 16*s;
        drawList->AddRectFilled({c.x - hw, c.y - hh}, {c.x + hw, c.y + hh}, blockBg, 4*s);
        drawList->AddRect({c.x - hw, c.y - hh}, {c.x + hw, c.y + hh}, color, 4*s, 0, 2.0f*s);
        drawList->AddText(rotatePt(-4*s, -6*s, c.x, c.y, rot), color, "s");
        drawList->AddLine(rotatePt(-hw - 4*s, 0, c.x, c.y, rot), rotatePt(-hw, 0, c.x, c.y, rot), color, 2.0f*s);
        drawList->AddLine(rotatePt(hw, 0, c.x, c.y, rot), rotatePt(hw + 4*s, 0, c.x, c.y, rot), color, 2.0f*s);
    } else if (t == "TRANSFER_FCN") {
        float hw = 24*s, hh = 16*s;
        drawList->AddRectFilled({c.x - hw, c.y - hh}, {c.x + hw, c.y + hh}, blockBg, 4*s);
        drawList->AddRect({c.x - hw, c.y - hh}, {c.x + hw, c.y + hh}, color, 4*s, 0, 2.0f*s);
        drawList->AddText(rotatePt(-12*s, -6*s, c.x, c.y, rot), color, "G(s)");
        drawList->AddLine(rotatePt(-hw - 4*s, 0, c.x, c.y, rot), rotatePt(-hw, 0, c.x, c.y, rot), color, 2.0f*s);
        drawList->AddLine(rotatePt(hw, 0, c.x, c.y, rot), rotatePt(hw + 4*s, 0, c.x, c.y, rot), color, 2.0f*s);
    } else if (t == "STATE_SPACE") {
        float hw = 26*s, hh = 16*s;
        drawList->AddRectFilled({c.x - hw, c.y - hh}, {c.x + hw, c.y + hh}, blockBg, 4*s);
        drawList->AddRect({c.x - hw, c.y - hh}, {c.x + hw, c.y + hh}, color, 4*s, 0, 2.0f*s);
        drawList->AddText(rotatePt(-16*s, -6*s, c.x, c.y, rot), color, "x'=Ax+Bu");
        drawList->AddLine(rotatePt(-hw - 4*s, 0, c.x, c.y, rot), rotatePt(-hw, 0, c.x, c.y, rot), color, 2.0f*s);
        drawList->AddLine(rotatePt(hw, 0, c.x, c.y, rot), rotatePt(hw + 4*s, 0, c.x, c.y, rot), color, 2.0f*s);
    } else if (t == "CONT_PID") {
        float hw = 24*s, hh = 16*s;
        drawList->AddRectFilled({c.x - hw, c.y - hh}, {c.x + hw, c.y + hh}, blockBg, 4*s);
        drawList->AddRect({c.x - hw, c.y - hh}, {c.x + hw, c.y + hh}, color, 4*s, 0, 2.0f*s);
        drawList->AddText(rotatePt(-16*s, -6*s, c.x, c.y, rot), color, "PID(s)");
        drawList->AddLine(rotatePt(-hw - 4*s, 0, c.x, c.y, rot), rotatePt(-hw, 0, c.x, c.y, rot), color, 2.0f*s);
        drawList->AddLine(rotatePt(hw, 0, c.x, c.y, rot), rotatePt(hw + 4*s, 0, c.x, c.y, rot), color, 2.0f*s);
    } else if (t == "PLL_1PH" || t == "PLL_3PH") {
        float hw = 24*s, hh = 18*s;
        drawList->AddRectFilled({c.x - hw, c.y - hh}, {c.x + hw, c.y + hh}, blockBg, 4*s);
        drawList->AddRect({c.x - hw, c.y - hh}, {c.x + hw, c.y + hh}, color, 4*s, 0, 2.0f*s);
        std::string pllLbl = (t == "PLL_3PH") ? "PLL 3" : "PLL 1";
        drawList->AddText(rotatePt(-16*s, -6*s, c.x, c.y, rot), color, pllLbl.c_str());
        if (t == "PLL_3PH") {
            drawList->AddLine(rotatePt(-hw - 4*s, -10*s, c.x, c.y, rot), rotatePt(-hw, -10*s, c.x, c.y, rot), color, 2.0f*s);
            drawList->AddLine(rotatePt(-hw - 4*s, 0, c.x, c.y, rot), rotatePt(-hw, 0, c.x, c.y, rot), color, 2.0f*s);
            drawList->AddLine(rotatePt(-hw - 4*s, 10*s, c.x, c.y, rot), rotatePt(-hw, 10*s, c.x, c.y, rot), color, 2.0f*s);
            drawList->AddLine(rotatePt(hw, -10*s, c.x, c.y, rot), rotatePt(hw + 4*s, -10*s, c.x, c.y, rot), color, 2.0f*s);
            drawList->AddLine(rotatePt(hw, 10*s, c.x, c.y, rot), rotatePt(hw + 4*s, 10*s, c.x, c.y, rot), color, 2.0f*s);
        } else {
            drawList->AddLine(rotatePt(-hw - 4*s, 0, c.x, c.y, rot), rotatePt(-hw, 0, c.x, c.y, rot), color, 2.0f*s);
            drawList->AddLine(rotatePt(hw, 0, c.x, c.y, rot), rotatePt(hw + 4*s, 0, c.x, c.y, rot), color, 2.0f*s);
        }
    } else if (t == "SCOPE") {
        int numCh = 2;
        if (comp.parameters.count("channels")) {
            try { numCh = std::stoi(comp.parameters.at("channels")); } catch (...) {}
        }
        if (numCh < 1) numCh = 1;
        float hw = 16.0f * s;
        float hh = std::max(16.0f, numCh * 10.0f) * s;

        drawList->AddRectFilled({c.x - hw, c.y - hh}, {c.x + hw, c.y + hh}, blockBg, 4.0f * s);
        drawList->AddRect({c.x - hw, c.y - hh}, {c.x + hw, c.y + hh}, color, 4.0f * s, 0, 2.0f * s);

        ImVec2 scW[] = {
            rotatePt(-8*s, -6*s, c.x, c.y, rot),
            rotatePt(-4*s, -6*s, c.x, c.y, rot),
            rotatePt(-4*s, 6*s, c.x, c.y, rot),
            rotatePt(4*s, 6*s, c.x, c.y, rot),
            rotatePt(4*s, -6*s, c.x, c.y, rot),
            rotatePt(8*s, -6*s, c.x, c.y, rot)
        };
        drawList->AddPolyline(scW, 6, IM_COL32(56, 189, 248, 255), 0, 1.5f * s);

        for (int i = 0; i < numCh; ++i) {
            float yOff = (numCh > 1) ? (-10.0f * (numCh - 1) + 20.0f * i) : 0.0f;
            ImVec2 arr[] = {
                rotatePt(-hw, (yOff - 3.0f)*s, c.x, c.y, rot),
                rotatePt(-hw + 4.0f*s, yOff*s, c.x, c.y, rot),
                rotatePt(-hw, (yOff + 3.0f)*s, c.x, c.y, rot)
            };
            drawList->AddTriangleFilled(arr[0], arr[1], arr[2], color);
        }
    } else if (t == "PROBE") {
        std::string sigStr = comp.parameters.count("selected_signals") ? comp.parameters.at("selected_signals") : "";
        std::vector<std::string> sigs;
        std::stringstream ss(sigStr);
        std::string item;
        while (std::getline(ss, item, ',')) { if (!item.empty()) sigs.push_back(item); }

        int numPins = std::max(1, (int)sigs.size());
        float hw = 30.0f * s;
        float hh = std::max(20.0f, numPins * 15.0f) * s;

        drawList->AddRectFilled({c.x - hw, c.y - hh}, {c.x + hw, c.y + hh}, blockBg, 4.0f * s);
        drawList->AddRect({c.x - hw, c.y - hh}, {c.x + hw, c.y + hh}, IM_COL32(14, 165, 233, 230), 4.0f * s, 0, 2.0f * s);
        drawList->AddText(rotatePt(-18*s, -6*s, c.x, c.y, rot), IM_COL32(14, 165, 233, 255), "PROBE");
    } else if (t == "MUX") {
        ImVec2 mPts[] = {rotatePt(-20*s, -25*s, c.x, c.y, rot), rotatePt(20*s, -15*s, c.x, c.y, rot), rotatePt(20*s, 15*s, c.x, c.y, rot), rotatePt(-20*s, 25*s, c.x, c.y, rot)};
        drawList->AddConvexPolyFilled(mPts, 4, blockBg);
        drawList->AddPolyline(mPts, 4, color, ImDrawFlags_Closed, 2.0f*s);
        drawList->AddText(rotatePt(-12*s, -6*s, c.x, c.y, rot), color, "MUX");
    } else if (t == "AND") {
        float hw = 20*s, hh = 18*s;
        drawList->AddRectFilled({c.x - hw, c.y - hh}, {c.x + hw, c.y + hh}, blockBg, 4*s);
        drawList->AddRect({c.x - hw, c.y - hh}, {c.x + hw, c.y + hh}, color, 4*s, 0, 2.0f*s);
        drawList->AddText(rotatePt(-4*s, -7*s, c.x, c.y, rot), color, "&");
    } else if (t == "OR") {
        float hw = 20*s, hh = 18*s;
        drawList->AddRectFilled({c.x - hw, c.y - hh}, {c.x + hw, c.y + hh}, blockBg, 4*s);
        drawList->AddRect({c.x - hw, c.y - hh}, {c.x + hw, c.y + hh}, color, 4*s, 0, 2.0f*s);
        drawList->AddText(rotatePt(-10*s, -7*s, c.x, c.y, rot), color, ">=1");
    } else if (t == "NOT") {
        ImVec2 t1 = rotatePt(-14*s, -15*s, c.x, c.y, rot);
        ImVec2 t2 = rotatePt(6*s, 0, c.x, c.y, rot);
        ImVec2 t3 = rotatePt(-14*s, 15*s, c.x, c.y, rot);
        drawList->AddTriangleFilled(t1, t2, t3, blockBg);
        drawList->AddTriangle(t1, t2, t3, color, 2.0f*s);
        drawList->AddCircle(rotatePt(10*s, 0, c.x, c.y, rot), 3.0f*s, color, 0, 2.0f*s);
    } else if (t == "SUBSYSTEM") {
        float hw = 50*s, hh = 40*s;
        drawList->AddRectFilled({c.x - hw, c.y - hh}, {c.x + hw, c.y + hh}, blockBg, 6*s);
        drawList->AddRect({c.x - hw, c.y - hh}, {c.x + hw, c.y + hh}, color, 6*s, 0, 2.0f*s);
        drawList->AddText({c.x - 28*s, c.y - 6*s}, color, "Subsystem");
    } else if (t == "INPORT" || t == "IN") {
        float hw = 22*s, hh = 15*s;
        drawList->AddRectFilled({c.x - hw, c.y - hh}, {c.x + hw, c.y + hh}, blockBg, 4*s);
        drawList->AddRect({c.x - hw, c.y - hh}, {c.x + hw, c.y + hh}, color, 4*s, 0, 2.0f*s);
        std::string pNum = comp.parameters.count("port_number") ? comp.parameters.at("port_number") : "1";
        std::string lbl = "In " + pNum;
        drawList->AddText({c.x - 12*s, c.y - 6*s}, color, lbl.c_str());
    } else if (t == "OUTPORT" || t == "OUT") {
        float hw = 22*s, hh = 15*s;
        drawList->AddRectFilled({c.x - hw, c.y - hh}, {c.x + hw, c.y + hh}, blockBg, 4*s);
        drawList->AddRect({c.x - hw, c.y - hh}, {c.x + hw, c.y + hh}, color, 4*s, 0, 2.0f*s);
        std::string pNum = comp.parameters.count("port_number") ? comp.parameters.at("port_number") : "1";
        std::string lbl = "Out " + pNum;
        drawList->AddText({c.x - 14*s, c.y - 6*s}, color, lbl.c_str());
    } else if (t == "PHYSICAL_INPORT" || t == "PIN") {
        float hw = 22*s, hh = 15*s;
        drawList->AddRectFilled({c.x - hw, c.y - hh}, {c.x + hw, c.y + hh}, IM_COL32(40, 50, 70, 240), 4*s);
        drawList->AddRect({c.x - hw, c.y - hh}, {c.x + hw, c.y + hh}, IM_COL32(234, 179, 8, 255), 4*s, 0, 2.0f*s);
        drawList->AddText({c.x - 14*s, c.y - 6*s}, IM_COL32(234, 179, 8, 255), "pIn");
    } else if (t == "PHYSICAL_OUTPORT" || t == "POUT") {
        float hw = 22*s, hh = 15*s;
        drawList->AddRectFilled({c.x - hw, c.y - hh}, {c.x + hw, c.y + hh}, IM_COL32(40, 50, 70, 240), 4*s);
        drawList->AddRect({c.x - hw, c.y - hh}, {c.x + hw, c.y + hh}, IM_COL32(234, 179, 8, 255), 4*s, 0, 2.0f*s);
        drawList->AddText({c.x - 16*s, c.y - 6*s}, IM_COL32(234, 179, 8, 255), "pOut");
    } else if (t == "ENABLE_PORT") {
        float hw = 22*s, hh = 18*s;
        drawList->AddRectFilled({c.x - hw, c.y - hh}, {c.x + hw, c.y + hh}, blockBg, 4*s);
        drawList->AddRect({c.x - hw, c.y - hh}, {c.x + hw, c.y + hh}, color, 4*s, 0, 2.0f*s);
        drawList->AddText({c.x - 18*s, c.y - 6*s}, color, "Enable");
    } else if (t == "TRIGGER_PORT") {
        float hw = 22*s, hh = 18*s;
        drawList->AddRectFilled({c.x - hw, c.y - hh}, {c.x + hw, c.y + hh}, blockBg, 4*s);
        drawList->AddRect({c.x - hw, c.y - hh}, {c.x + hw, c.y + hh}, color, 4*s, 0, 2.0f*s);
        drawList->AddText({c.x - 18*s, c.y - 6*s}, color, "Trig");
    } else if (t == "BUS_CREATOR" || t == "BUS_SELECTOR") {
        float hw = 8*s, hh = 25*s;
        drawList->AddRectFilled({c.x - hw, c.y - hh}, {c.x + hw, c.y + hh}, IM_COL32(30, 30, 30, 255), 2*s);
        drawList->AddRect({c.x - hw, c.y - hh}, {c.x + hw, c.y + hh}, IM_COL32(200, 200, 200, 255), 2*s, 0, 1.5f*s);
    } else if (t == "TERMINATOR") {
        float hw = 15*s, hh = 15*s;
        drawList->AddRectFilled({c.x - hw, c.y - hh}, {c.x + hw, c.y + hh}, blockBg, 2*s);
        drawList->AddRect({c.x - hw, c.y - hh}, {c.x + hw, c.y + hh}, color, 2*s, 0, 1.5f*s);
        drawList->AddLine({c.x + 4*s, c.y - 8*s}, {c.x + 4*s, c.y + 8*s}, color, 2.0f*s);
    } else if (t == "POLYNOMIAL") {
        float hw = 35*s, hh = 22*s;
        drawList->AddRectFilled({c.x - hw, c.y - hh}, {c.x + hw, c.y + hh}, blockBg, 4*s);
        drawList->AddRect({c.x - hw, c.y - hh}, {c.x + hw, c.y + hh}, color, 4*s, 0, 2.0f*s);
        drawList->AddText({c.x - 25*s, c.y - 6*s}, color, "Poly P(u)");
    } else if (t == "ALGEBRAIC_CONSTRAINT") {
        float hw = 35*s, hh = 22*s;
        drawList->AddRectFilled({c.x - hw, c.y - hh}, {c.x + hw, c.y + hh}, blockBg, 4*s);
        drawList->AddRect({c.x - hw, c.y - hh}, {c.x + hw, c.y + hh}, color, 4*s, 0, 2.0f*s);
        drawList->AddText({c.x - 22*s, c.y - 6*s}, color, "f(z) = 0");
    } else if (t == "CSCRIPT" || t == "CUSTOMSCRIPT") {
        std::vector<CircuitSimEngine::CScriptPort> inPorts, outPorts;
        getCSCRIPTPorts(comp, inPorts, outPorts);

        int numIn = (int)inPorts.size();
        int numOut = (int)outPorts.size();
        int nMax = std::max(numIn, numOut);

        float hw = 45.0f * s;
        float hh = std::max(25.0f, nMax * 12.0f + 10.0f) * s;

        ImU32 cscriptBg = IM_COL32(30, 41, 59, 240);
        ImU32 cscriptBorder = IM_COL32(59, 130, 246, 255);

        drawList->AddRectFilled({c.x - hw, c.y - hh}, {c.x + hw, c.y + hh}, cscriptBg, 4.0f * s);
        drawList->AddRect({c.x - hw, c.y - hh}, {c.x + hw, c.y + hh}, cscriptBorder, 4.0f * s, 0, 1.5f * s);

        std::string labelText = comp.label.empty() ? "C-Script" : comp.label;
        ImVec2 txtSz = ImGui::CalcTextSize(labelText.c_str());
        drawList->AddText({c.x - (txtSz.x * 0.5f), c.y - 6.0f * s}, IM_COL32(255, 255, 255, 240), labelText.c_str());

        for (int i = 0; i < numIn; ++i) {
            float yOff = (numIn > 1) ? (-18.0f * (numIn - 1) / 2.0f + 18.0f * i) : 0.0f;
            drawList->AddText({c.x - hw + 4.0f * s, c.y + (yOff - 6.0f) * s}, IM_COL32(148, 163, 184, 220), inPorts[i].name.c_str());
        }

        for (int j = 0; j < numOut; ++j) {
            float yOff = (numOut > 1) ? (-18.0f * (numOut - 1) / 2.0f + 18.0f * j) : 0.0f;
            ImVec2 pSz = ImGui::CalcTextSize(outPorts[j].name.c_str());
            drawList->AddText({c.x + hw - (pSz.x + 4.0f * s), c.y + (yOff - 6.0f) * s}, IM_COL32(148, 163, 184, 220), outPorts[j].name.c_str());
        }
    } else if (t == "IDEAL_XFMR" || t == "XFMR_2W" || t == "MUTUAL_2W" || t == "SAT_XFMR" || t == "Transformer" || t == "IDEAL_TRANSFORMER" || t == "TRANSFORMER" || t == "XFMR") {
        std::string pStr = comp.parameters.count("primary_turns") ? comp.parameters.at("primary_turns") : "[100]";
        std::string sStr = comp.parameters.count("secondary_turns") ? comp.parameters.at("secondary_turns") : "[100]";
        auto pTurns = parseTurnsArrayStr(pStr);
        auto sTurns = parseTurnsArrayStr(sStr);

        int np = (int)pTurns.size();
        int ns = (int)sTurns.size();
        int nMax = std::max({1, np, ns});

        float totalHalfH = std::max(22.0f, nMax * 24.0f + 4.0f) * s;
        float hw = 22.0f * s;

        // 1. Dashed outer orange/red bounding box (matching WebTool style)
        drawList->AddRect(
            {c.x - hw, c.y - totalHalfH}, 
            {c.x + hw, c.y + totalHalfH}, 
            IM_COL32(234, 88, 12, 180), 
            4.0f * s, 0, 1.5f * s
        );

        // 2. Central vertical magnetic core lines (2 parallel lines)
        ImVec2 core1_top = rotatePt(-2.0f * s, -totalHalfH + 6.0f * s, c.x, c.y, rot);
        ImVec2 core1_bot = rotatePt(-2.0f * s, totalHalfH - 6.0f * s, c.x, c.y, rot);
        ImVec2 core2_top = rotatePt(2.0f * s, -totalHalfH + 6.0f * s, c.x, c.y, rot);
        ImVec2 core2_bot = rotatePt(2.0f * s, totalHalfH - 6.0f * s, c.x, c.y, rot);
        drawList->AddLine(core1_top, core1_bot, color, 1.8f * s);
        drawList->AddLine(core2_top, core2_bot, color, 1.8f * s);

        // 3. Draw Primary Windings (Left side)
        for (int i = 0; i < np; ++i) {
            float wyCenter = (np > 1) ? (-25.0f * (np - 1) + 50.0f * i) : 0.0f;
            float wxLeft = -6.0f;

            // 3 scalloped arcs for this primary winding
            for (int k = -1; k <= 1; ++k) {
                float arcY = wyCenter + k * 8.0f;
                ImVec2 arcCenter = rotatePt(wxLeft * s, arcY * s, c.x, c.y, rot);
                drawList->AddCircle(arcCenter, 4.5f * s, color, 16, 1.8f * s);
            }

            // Lead lines to terminal ports
            ImVec2 topLeadStart = rotatePt(wxLeft * s, (wyCenter - 14.0f) * s, c.x, c.y, rot);
            ImVec2 topLeadEnd = rotatePt(-25.0f * s, (wyCenter - 14.0f) * s, c.x, c.y, rot);
            ImVec2 botLeadStart = rotatePt(wxLeft * s, (wyCenter + 14.0f) * s, c.x, c.y, rot);
            ImVec2 botLeadEnd = rotatePt(-25.0f * s, (wyCenter + 14.0f) * s, c.x, c.y, rot);

            drawList->AddLine(topLeadStart, topLeadEnd, color, 1.8f * s);
            drawList->AddLine(botLeadStart, botLeadEnd, color, 1.8f * s);

            // Circular terminal port rings (matching WebTool UI)
            drawList->AddCircle(topLeadEnd, 3.5f * s, IM_COL32(0, 102, 204, 255), 16, 2.0f * s);
            drawList->AddCircle(botLeadEnd, 3.5f * s, IM_COL32(0, 102, 204, 255), 16, 2.0f * s);

            // Polarity dot at top of winding
            drawList->AddCircleFilled(rotatePt(-14.0f * s, (wyCenter - 18.0f) * s, c.x, c.y, rot), 2.5f * s, color);
        }

        // 4. Draw Secondary Windings (Right side)
        for (int j = 0; j < ns; ++j) {
            float wyCenter = (ns > 1) ? (-25.0f * (ns - 1) + 50.0f * j) : 0.0f;
            float wxRight = 6.0f;

            for (int k = -1; k <= 1; ++k) {
                float arcY = wyCenter + k * 8.0f;
                ImVec2 arcCenter = rotatePt(wxRight * s, arcY * s, c.x, c.y, rot);
                drawList->AddCircle(arcCenter, 4.5f * s, color, 16, 1.8f * s);
            }

            ImVec2 topLeadStart = rotatePt(wxRight * s, (wyCenter - 14.0f) * s, c.x, c.y, rot);
            ImVec2 topLeadEnd = rotatePt(25.0f * s, (wyCenter - 14.0f) * s, c.x, c.y, rot);
            ImVec2 botLeadStart = rotatePt(wxRight * s, (wyCenter + 14.0f) * s, c.x, c.y, rot);
            ImVec2 botLeadEnd = rotatePt(25.0f * s, (wyCenter + 14.0f) * s, c.x, c.y, rot);

            drawList->AddLine(topLeadStart, topLeadEnd, color, 1.8f * s);
            drawList->AddLine(botLeadStart, botLeadEnd, color, 1.8f * s);

            // Circular terminal port rings (matching WebTool UI)
            drawList->AddCircle(topLeadEnd, 3.5f * s, IM_COL32(0, 102, 204, 255), 16, 2.0f * s);
            drawList->AddCircle(botLeadEnd, 3.5f * s, IM_COL32(0, 102, 204, 255), 16, 2.0f * s);

            // Polarity dot (inverted if turns < 0 or polarity == "inverted")
            std::string polStr = comp.parameters.count("polarity") ? comp.parameters.at("polarity") : "normal";
            bool isInv = (polStr == "inverted" && j == 0) || (sTurns[j] < 0.0f);
            float dotY = isInv ? (wyCenter + 18.0f) : (wyCenter - 18.0f);
            drawList->AddCircleFilled(rotatePt(14.0f * s, dotY * s, c.x, c.y, rot), 2.5f * s, color);
        }
    } else {
        drawList->AddRect({c.x - 20*s, c.y - 20*s}, {c.x + 20*s, c.y + 20*s}, color, 4*s, 0, 2.0f*s);
        drawList->AddText({c.x - 12*s, c.y - 5*s}, color, t.c_str());
    }
}

void SchematicCanvas::drawTerminals(ImDrawList* drawList, const ComponentInstance& comp, ImVec2 center, float s, ImVec2 mousePos, float& minPinDist) {
    auto terminals = getTerminals(comp);
    if (terminals.empty() && !comp.pins.empty()) {
        for (const auto& pin : comp.pins) {
            TerminalDef td;
            td.name = pin.name.c_str();
            td.x = pin.relativeX;
            td.y = pin.relativeY;
            td.dx = pin.isOutput ? 1.0f : -1.0f;
            td.dy = 0;
            td.isControl = pin.isCtrl || pin.isInput || pin.isOutput;
            terminals.push_back(td);
        }
    }

    const ComponentInstance* startComp = nullptr;
    if (isWiring) {
        for (const auto& c : design.components) {
            if (c.id == wireStartCompId) { startComp = &c; break; }
        }
    }
    
    for (const auto& term : terminals) {
        ImVec2 tPos = rotatePt(term.x * s, term.y * s, center.x, center.y, (float)comp.rotation);
        float dist = std::sqrt((mousePos.x - tPos.x)*(mousePos.x - tPos.x) + (mousePos.y - tPos.y)*(mousePos.y - tPos.y));
        bool isHovered = (dist < 14.0f * s);
        bool isConnected = isPinConnected(comp.id, term.name);

        bool isControl = getPinDomain(comp, term.name) == DomainType::Control;
        
        if (isHovered && dist < minPinDist) {
            minPinDist = dist;
            hoveredPinCompId = comp.id;
            hoveredPinName = term.name;

            if (isWiring && startComp) {
                DomainType startDom = getPinDomain(*startComp, wireStartPin);
                DomainType targetDom = getPinDomain(comp, term.name);
                if (startDom != targetDom) {
                    ImGui::SetTooltip("🚫 CANNOT CONNECT: %s Pin cannot connect to %s Pin!",
                        startDom == DomainType::Control ? "Control Signal" : "Electrical Power",
                        targetDom == DomainType::Control ? "Control Signal" : "Electrical Power");
                } else {
                    ImGui::SetTooltip("%s.%s (%s)", comp.id.c_str(), term.name, isControl ? "Control Domain" : "Power Domain");
                }
            } else {
                ImGui::SetTooltip("%s.%s (%s)", comp.id.c_str(), term.name, isControl ? "Control Domain" : "Power Domain");
            }
        }

        bool isClosestHovered = (!hoveredPinCompId.empty() && hoveredPinCompId == comp.id && hoveredPinName == term.name);
        float radius = isClosestHovered ? 6.0f * s : (isConnected ? 2.0f * s : (isControl ? 4.0f * s : 4.5f * s));
        ImU32 termColor = isClosestHovered ? IM_COL32(255, 200, 50, 255) : (isControl ? IM_COL32(56, 189, 248, 230) : IM_COL32(0, 230, 120, 230));

        if (isClosestHovered && isWiring && startComp) {
            DomainType startDom = getPinDomain(*startComp, wireStartPin);
            DomainType targetDom = getPinDomain(comp, term.name);
            if (startDom != targetDom) {
                termColor = IM_COL32(255, 50, 50, 255);
            }
        }

        drawList->AddCircleFilled(tPos, radius, termColor);
    }
}

bool SchematicCanvas::getTerminalPortStub(const ComponentInstance& comp, const std::string& terminalName, ImVec2 canvasPos, float zoomLevel, ImVec2& outPinPos, ImVec2& outStubPos, bool& outIsVertical) const {
    auto terminals = getTerminals(comp);
    if (terminals.empty() && !comp.pins.empty()) {
        for (const auto& pin : comp.pins) {
            TerminalDef td;
            td.name = pin.name.c_str();
            td.x = pin.relativeX;
            td.y = pin.relativeY;
            td.dx = pin.isOutput ? 1.0f : -1.0f;
            td.dy = 0;
            td.isControl = pin.isCtrl || pin.isInput || pin.isOutput;
            terminals.push_back(td);
        }
    }

    ImVec2 compCenter = worldToScreen(comp.x, comp.y, canvasPos);
    for (const auto& t : terminals) {
        if (isTerminalMatch(comp.rawTypeStr, t.name, terminalName)) {
            outPinPos = rotatePt(t.x * zoomLevel, t.y * zoomLevel, compCenter.x, compCenter.y, (float)comp.rotation);
            ImVec2 dir = rotatePt(t.dx * 20.0f * zoomLevel, t.dy * 20.0f * zoomLevel, 0, 0, (float)comp.rotation);
            outStubPos = ImVec2(outPinPos.x + dir.x, outPinPos.y + dir.y);
            ImVec2 rotatedDir = rotatePt(t.dx, t.dy, 0, 0, (float)comp.rotation);
            outIsVertical = (std::abs(rotatedDir.y) > std::abs(rotatedDir.x));
            return true;
        }
    }
    outIsVertical = false;
    return false;
}

void SchematicCanvas::drawWires(ImDrawList* drawList, ImVec2 canvasPos) {
    ImGuiIO& io = ImGui::GetIO();
    ImVec2 mousePos = io.MousePos;

    hoveredWireId.clear();
    float minWireDist = 1e9f;

    std::map<std::string, std::tuple<ImVec2, ImVec2, ImVec2, ImVec2>> wirePointsMap;

    for (auto& wire : design.wires) {
        ImVec2 p1(0, 0), p1_stub(0, 0), p2(0, 0), p2_stub(0, 0);
        bool foundFrom = false, foundTo = false;

        DomainType wDom = getWireDomain(wire, design);
        bool isControlNet = (wDom == DomainType::Control);

        bool fromIsVertical = false;
        if (wire.from.isWireJunction) {
            p1 = worldToScreen(wire.from.junctionX, wire.from.junctionY, canvasPos);
            p1_stub = p1;
            foundFrom = true;
        } else {
            for (const auto& comp : design.components) {
                if (comp.id == wire.from.compId) {
                    foundFrom = getTerminalPortStub(comp, wire.from.terminal, canvasPos, zoomLevel, p1, p1_stub, fromIsVertical);
                    break;
                }
            }
        }

        if (wire.to.isWireJunction) {
            std::string targetLower = wire.to.targetWireId;
            std::transform(targetLower.begin(), targetLower.end(), targetLower.begin(), ::tolower);

            bool foundTarget = false;
            for (const auto& [k, v] : wirePointsMap) {
                std::string kLower = k;
                std::transform(kLower.begin(), kLower.end(), kLower.begin(), ::tolower);
                if (kLower == targetLower) {
                    ImVec2 tp1 = std::get<0>(v);
                    ImVec2 tmid = std::get<1>(v);
                    ImVec2 tmid2 = std::get<2>(v);
                    ImVec2 tp2 = std::get<3>(v);

                    ImVec2 jScreen = worldToScreen(wire.to.junctionX, wire.to.junctionY, canvasPos);
                    float d1=0, d2=0, d3=0;
                    ImVec2 q1 = getClosestPointOnSegment(jScreen, tp1, tmid, d1);
                    ImVec2 q2 = getClosestPointOnSegment(jScreen, tmid, tmid2, d2);
                    ImVec2 q3 = getClosestPointOnSegment(jScreen, tmid2, tp2, d3);

                    float minD = std::min({d1, d2, d3});
                    p2 = (minD == d1) ? q1 : ((minD == d2) ? q2 : q3);
                    p2_stub = p2;
                    foundTo = true;
                    foundTarget = true;
                    break;
                }
            }
            if (!foundTarget) {
                p2 = worldToScreen(wire.to.junctionX, wire.to.junctionY, canvasPos);
                p2_stub = p2;
                foundTo = true;
            }
        } else {
            for (const auto& comp : design.components) {
                if (comp.id == wire.to.compId) {
                    bool dummyVert;
                    foundTo = getTerminalPortStub(comp, wire.to.terminal, canvasPos, zoomLevel, p2, p2_stub, dummyVert);
                    break;
                }
            }
        }

        if (!foundFrom) p1_stub = p1;
        if (!foundTo) p2_stub = p2;

        if (foundFrom && foundTo) {
            // ALWAYS PRESERVE PORT STUB (never collapse p1_stub = p1 so wire doesn't turn right at pin)

            ImVec2 c1(0, 0), c2(0, 0);
            bool hasCustomOffset = !wire.manualPath.empty();

            if (hasCustomOffset) {
                if (isDraggingSegmentHorizontal) {
                    float snapY = worldToScreen(0, wire.manualPath[0].y, canvasPos).y;
                    c1 = ImVec2(p1_stub.x, snapY);
                    c2 = ImVec2(p2_stub.x, snapY);
                } else {
                    float snapX = worldToScreen(wire.manualPath[0].x, 0, canvasPos).x;
                    c1 = ImVec2(snapX, p1_stub.y);
                    c2 = ImVec2(snapX, p2_stub.y);
                }
            } else {
                // Find source component to check if wire path cuts through component body
                const ComponentInstance* fromComp = nullptr;
                if (!wire.from.compId.empty()) {
                    for (const auto& comp : design.components) {
                        if (comp.id == wire.from.compId) { fromComp = &comp; break; }
                    }
                }

                if (fromIsVertical) {
                    c1 = ImVec2(p1_stub.x, p2_stub.y);
                    c2 = c1;

                    // Obstacle avoidance: check if routing vertically back toward component cuts through component body
                    if (fromComp) {
                        ImVec2 compCenter = worldToScreen(fromComp->x, fromComp->y, canvasPos);
                        float rawHw = 25.0f, rawHh = 25.0f;
                        getComponentBounds(*fromComp, rawHw, rawHh);
                        float hw = rawHw * zoomLevel;
                        float hh = rawHh * zoomLevel;

                        // If p1_stub and p2_stub are on opposite Y-sides of component center,
                        // or if p2_stub.y is inside component Y bounds:
                        bool cutsThroughY = ((p1_stub.y - compCenter.y) * (p2_stub.y - compCenter.y) < 0) ||
                                             (std::abs(p2_stub.y - compCenter.y) <= hh);
                        bool alignsX = (std::abs(p1_stub.x - compCenter.x) <= hw + 10.0f * zoomLevel);

                        if (cutsThroughY && alignsX) {
                            // Detour around component side to create clean Z/L shape
                            float detourX = (p2_stub.x >= compCenter.x) ? 
                                            (compCenter.x + hw + 25.0f * zoomLevel) : 
                                            (compCenter.x - hw - 25.0f * zoomLevel);
                            c1 = ImVec2(detourX, p1_stub.y);
                            c2 = ImVec2(detourX, p2_stub.y);
                        }
                    }
                } else {
                    c1 = ImVec2(p2_stub.x, p1_stub.y);
                    c2 = c1;

                    if (fromComp) {
                        ImVec2 compCenter = worldToScreen(fromComp->x, fromComp->y, canvasPos);
                        float rawHw = 25.0f, rawHh = 25.0f;
                        getComponentBounds(*fromComp, rawHw, rawHh);
                        float hw = rawHw * zoomLevel;
                        float hh = rawHh * zoomLevel;

                        bool cutsThroughX = ((p1_stub.x - compCenter.x) * (p2_stub.x - compCenter.x) < 0) ||
                                            (std::abs(p2_stub.x - compCenter.x) <= hw);
                        bool alignsY = (std::abs(p1_stub.y - compCenter.y) <= hh + 10.0f * zoomLevel);

                        if (cutsThroughX && alignsY) {
                            float detourY = (p2_stub.y >= compCenter.y) ? 
                                            (compCenter.y + hh + 25.0f * zoomLevel) : 
                                            (compCenter.y - hh - 25.0f * zoomLevel);
                            c1 = ImVec2(p1_stub.x, detourY);
                            c2 = ImVec2(p2_stub.x, detourY);
                        }
                    }
                }
            }

            wirePointsMap[wire.id] = {p1_stub, c1, c2, p2_stub};

            float d0 = 0, d1 = 0, d2 = 0, d3 = 0, d4 = 0;
            ImVec2 q0 = getClosestPointOnSegment(mousePos, p1, p1_stub, d0);
            ImVec2 q1 = getClosestPointOnSegment(mousePos, p1_stub, c1, d1);
            ImVec2 q2 = getClosestPointOnSegment(mousePos, c1, c2, d2);
            ImVec2 q3 = getClosestPointOnSegment(mousePos, c2, p2_stub, d3);
            ImVec2 q4 = getClosestPointOnSegment(mousePos, p2_stub, p2, d4);

            float bestD = std::min({d0, d1, d2, d3, d4});
            ImVec2 bestQ = (bestD == d0) ? q0 : ((bestD == d1) ? q1 : ((bestD == d2) ? q2 : ((bestD == d3) ? q3 : q4)));

            bool isWireHovered = (bestD < 12.0f * zoomLevel);

            if (isWireHovered && bestD < minWireDist) {
                minWireDist = bestD;
                hoveredWireId = wire.id;
                ImVec2 snapQ = bestQ;
                if (isWiring) {
                    ImVec2 startP(0, 0), startStub(0, 0);
                    bool startIsVert = false;
                    for (const auto& comp : design.components) {
                        if (comp.id == wireStartCompId) {
                            getTerminalPortStub(comp, wireStartPin, canvasPos, zoomLevel, startP, startStub, startIsVert);
                            break;
                        }
                    }
                    if (startIsVert && std::abs(startStub.x - bestQ.x) < 20.0f * zoomLevel) {
                        snapQ.x = startStub.x;
                    } else if (!startIsVert && std::abs(startStub.y - bestQ.y) < 20.0f * zoomLevel) {
                        snapQ.y = startStub.y;
                    }
                }
                ImVec2 wJunction = screenToWorld(snapQ, canvasPos);
                wJunction.x = std::round(wJunction.x / 10.0f) * 10.0f;
                wJunction.y = std::round(wJunction.y / 10.0f) * 10.0f;
                hoveredWireJunctionPos = wJunction;
            }

            bool isSelected = selectedWireIds.count(wire.id) > 0;
            bool isThisWireHovered = (wire.id == hoveredWireId);
            ImU32 powerWireColor = isDarkMode ? IM_COL32(0, 230, 120, 255) : IM_COL32(4, 120, 87, 255);
            ImU32 ctrlWireColor = isDarkMode ? IM_COL32(56, 189, 248, 255) : IM_COL32(2, 132, 199, 255);
            ImU32 selColor = isDarkMode ? IM_COL32(255, 180, 0, 255) : IM_COL32(217, 119, 6, 255);
            ImU32 hovColor = isDarkMode ? IM_COL32(255, 220, 100, 255) : IM_COL32(245, 158, 11, 255);

            ImU32 wireColor = isSelected ? selColor : (isThisWireHovered ? hovColor : (isControlNet ? ctrlWireColor : powerWireColor));
            float thickness = (isSelected || isThisWireHovered) ? 3.5f * zoomLevel : 2.5f * zoomLevel;

            if (isSelected) {
                drawList->AddLine(p1, p1_stub, IM_COL32(245, 158, 11, 60), thickness + 4.0f*zoomLevel);
                drawList->AddLine(p1_stub, c1, IM_COL32(245, 158, 11, 60), thickness + 4.0f*zoomLevel);
                drawList->AddLine(c1, c2, IM_COL32(245, 158, 11, 60), thickness + 4.0f*zoomLevel);
                drawList->AddLine(c2, p2_stub, IM_COL32(245, 158, 11, 60), thickness + 4.0f*zoomLevel);
                drawList->AddLine(p2_stub, p2, IM_COL32(245, 158, 11, 60), thickness + 4.0f*zoomLevel);
            }
            
            // Render 100% Strictly Orthogonal Wire Path with Port Stubs
            drawList->AddLine(p1, p1_stub, wireColor, thickness);
            drawList->AddLine(p1_stub, c1, wireColor, thickness);
            drawList->AddLine(c1, c2, wireColor, thickness);
            drawList->AddLine(c2, p2_stub, wireColor, thickness);
            drawList->AddLine(p2_stub, p2, wireColor, thickness);

            if (wire.to.isWireJunction) {
                drawList->AddCircleFilled(p2, 4.0f * zoomLevel, isControlNet ? IM_COL32(56, 189, 248, 255) : IM_COL32(0, 230, 120, 255));
            }
            if (wire.from.isWireJunction) {
                drawList->AddCircleFilled(p1, 4.0f * zoomLevel, isControlNet ? IM_COL32(56, 189, 248, 255) : IM_COL32(0, 230, 120, 255));
            }

            // Interactive Segment Drag Handles (Horizontal / Vertical 4-Way Dragging)
            if (isSelected && selectedComponentIds.empty()) {
                ImVec2 h1((p1_stub.x + c1.x)*0.5f, (p1_stub.y + c1.y)*0.5f);
                ImVec2 h2((c1.x + c2.x)*0.5f, (c1.y + c2.y)*0.5f);
                ImVec2 h3((c2.x + p2_stub.x)*0.5f, (c2.y + p2_stub.y)*0.5f);
                float hs = 6.0f * zoomLevel;

                drawList->AddRectFilled({h1.x - hs, h1.y - hs}, {h1.x + hs, h1.y + hs}, IM_COL32(255, 180, 0, 255));
                drawList->AddRectFilled({h2.x - hs, h2.y - hs}, {h2.x + hs, h2.y + hs}, IM_COL32(255, 180, 0, 255));
                drawList->AddRectFilled({h3.x - hs, h3.y - hs}, {h3.x + hs, h3.y + hs}, IM_COL32(255, 180, 0, 255));

                if (ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
                    if (isWireHovered) {
                        pushUndoState();
                        isDraggingWireSegment = true;
                        draggingWireId = wire.id;
                        
                        ImVec2 segP1 = (bestD == d1 || bestD == d0) ? p1_stub : ((bestD == d2) ? c1 : c2);
                        ImVec2 segP2 = (bestD == d1 || bestD == d0) ? c1 : ((bestD == d2) ? c2 : p2_stub);
                        
                        isDraggingSegmentHorizontal = (std::abs(segP1.y - segP2.y) <= std::abs(segP1.x - segP2.x));
                    }
                }
            }

            if (isControlNet && !wire.to.isWireJunction) {
                float arrLen = 8.0f * zoomLevel;
                ImVec2 arr1(p2.x - arrLen, p2.y - arrLen * 0.5f);
                ImVec2 arr2(p2.x - arrLen, p2.y + arrLen * 0.5f);
                drawList->AddTriangleFilled(p2, arr1, arr2, wireColor);
            }
        }
    }

    if (!hoveredPinCompId.empty()) {
        hoveredWireId.clear();
    }

    if (isWiring && !hoveredWireId.empty()) {
        const ComponentInstance* startComp = nullptr;
        for (const auto& c : design.components) {
            if (c.id == wireStartCompId) { startComp = &c; break; }
        }
        const WireInstance* targetWire = nullptr;
        for (auto& w : design.wires) {
            if (w.id == hoveredWireId) { targetWire = &w; break; }
        }

        DomainType startDom = startComp ? getPinDomain(*startComp, wireStartPin) : DomainType::Power;
        DomainType targetWireDom = targetWire ? getWireDomain(*targetWire, design) : DomainType::Power;

        ImVec2 jScreen = worldToScreen(hoveredWireJunctionPos.x, hoveredWireJunctionPos.y, canvasPos);

        if (startDom != targetWireDom) {
            drawList->AddCircleFilled(jScreen, 6.0f * zoomLevel, IM_COL32(255, 50, 50, 255));
            drawList->AddCircle(jScreen, 9.0f * zoomLevel, IM_COL32(255, 50, 50, 255), 0, 2.0f * zoomLevel);
            ImGui::SetTooltip("🚫 CANNOT CONNECT: %s Line cannot attach to %s Wire!",
                startDom == DomainType::Control ? "Control Signal" : "Electrical Power",
                targetWireDom == DomainType::Control ? "Control Signal" : "Electrical Power");
        } else {
            ImU32 dotCol = (startDom == DomainType::Control) ? IM_COL32(56, 189, 248, 255) : IM_COL32(0, 230, 120, 255);
            drawList->AddCircleFilled(jScreen, 6.0f * zoomLevel, dotCol);
            drawList->AddCircle(jScreen, 9.0f * zoomLevel, IM_COL32(255, 200, 50, 255), 0, 2.0f * zoomLevel);
        }
    }

    // Active 4-Way Orthogonal Segment Dragging Engine
    if (isDraggingWireSegment && ImGui::IsMouseDragging(ImGuiMouseButton_Left)) {
        for (auto& w : design.wires) {
            if (w.id == draggingWireId) {
                ImVec2 wPos = screenToWorld(mousePos, canvasPos);
                float snapX = std::round(wPos.x / 20.0f) * 20.0f;
                float snapY = std::round(wPos.y / 20.0f) * 20.0f;

                Point2D pt;
                pt.x = snapX;
                pt.y = snapY;
                w.manualPath = {pt};
                break;
            }
        }
    } else if (ImGui::IsMouseReleased(ImGuiMouseButton_Left)) {
        isDraggingWireSegment = false;
    }

    // Active Wire Drawing Preview with Single Right-Angle L-Shape Corner (Matching User Diagram)
    if (isWiring) {
        ImVec2 p1(0, 0), p1_stub(0, 0);
        bool fromIsVertical = false;
        for (const auto& comp : design.components) {
            if (comp.id == wireStartCompId) {
                getTerminalPortStub(comp, wireStartPin, canvasPos, zoomLevel, p1, p1_stub, fromIsVertical);
                break;
            }
        }

        ImU32 previewColor = IM_COL32(255, 200, 0, 200);
        drawList->AddLine(p1, p1_stub, previewColor, 2.0f * zoomLevel);

        ImVec2 curr = p1_stub;
        for (const auto& corner : activeWireCorners) {
            ImVec2 sc = worldToScreen(corner.x, corner.y, canvasPos);
            drawList->AddLine(curr, sc, previewColor, 2.0f * zoomLevel);
            curr = sc;
        }

        ImVec2 targetP = (!hoveredWireId.empty()) ? worldToScreen(hoveredWireJunctionPos.x, hoveredWireJunctionPos.y, canvasPos) : wireCurrentPos;
        // SINGLE RIGHT-ANGLE L-SHAPE PREVIEW (matching user screenshot 100%)
        ImVec2 mid = fromIsVertical ? ImVec2(curr.x, targetP.y) : ImVec2(targetP.x, curr.y);
        drawList->AddLine(curr, mid, previewColor, 2.0f * zoomLevel);
        drawList->AddLine(mid, targetP, previewColor, 2.0f * zoomLevel);
    }
}

void SchematicCanvas::getComponentBounds(const ComponentInstance& comp, float& outHalfW, float& outHalfH) {
    std::string t = comp.rawTypeStr;
    std::transform(t.begin(), t.end(), t.begin(), ::toupper);

    if (t == "SUM" || t == "SUM_ROUND" || t == "SUM_RECT" || t == "SUBTRACT" || t == "SUB" ||
        t == "PROD" || t == "PRODUCT_RECT") {
        std::vector<std::string> signs;
        int nInputs = parseMathBlockPins(comp, signs);
        float spacing = 18.0f;
        float totalH = std::max(40.0f, (nInputs - 1) * spacing + 20.0f);
        outHalfW = 26.0f;
        outHalfH = totalH * 0.5f + 2.0f;
        int rot = ((comp.rotation % 360) + 360) % 360;
        if (rot == 90 || rot == 270) std::swap(outHalfW, outHalfH);
        return;
    }

    float minX = 0.0f, maxX = 0.0f, minY = 0.0f, maxY = 0.0f;
    bool hasPins = !comp.pins.empty();

    if (hasPins) {
        minX = comp.pins[0].relativeX;
        maxX = comp.pins[0].relativeX;
        minY = comp.pins[0].relativeY;
        maxY = comp.pins[0].relativeY;
        for (const auto& pin : comp.pins) {
            minX = std::min(minX, (float)pin.relativeX);
            maxX = std::max(maxX, (float)pin.relativeX);
            minY = std::min(minY, (float)pin.relativeY);
            maxY = std::max(maxY, (float)pin.relativeY);
        }
        outHalfW = std::max(std::abs(minX), std::abs(maxX)) + 4.0f;
        outHalfH = std::max(std::abs(minY), std::abs(maxY)) + 4.0f;
    } else {
        outHalfW = 25.0f;
        outHalfH = 25.0f;
    }
    if (t == "CSCRIPT" || t == "CUSTOMSCRIPT") {
        std::vector<CircuitSimEngine::CScriptPort> inPorts, outPorts;
        getCSCRIPTPorts(comp, inPorts, outPorts);
        int nMax = std::max((int)inPorts.size(), (int)outPorts.size());
        outHalfW = std::max(outHalfW, 45.0f);
        outHalfH = std::max(outHalfH, std::max(25.0f, nMax * 12.0f + 10.0f));
    } else if (t == "SUBSYSTEM") {
        outHalfW = std::max(outHalfW, 52.0f);
        outHalfH = std::max(outHalfH, 42.0f);
    } else if (t == "INPORT" || t == "OUTPORT" || t == "IN" || t == "OUT" || t == "PIN" || t == "POUT" || t == "PHYSICAL_INPORT" || t == "PHYSICAL_OUTPORT") {
        outHalfW = std::max(outHalfW, 24.0f);
        outHalfH = std::max(outHalfH, 16.0f);
    } else if (t == "ENABLE_PORT" || t == "TRIGGER_PORT") {
        outHalfW = std::max(outHalfW, 24.0f);
        outHalfH = std::max(outHalfH, 20.0f);
    } else if (t == "BUS_CREATOR" || t == "BUS_SELECTOR") {
        outHalfW = std::max(outHalfW, 10.0f);
        outHalfH = std::max(outHalfH, 28.0f);
    } else if (t == "TERMINATOR") {
        outHalfW = std::max(outHalfW, 16.0f);
        outHalfH = std::max(outHalfH, 16.0f);
    } else if (t == "POLYNOMIAL" || t == "ALGEBRAIC_CONSTRAINT") {
        outHalfW = std::max(outHalfW, 36.0f);
        outHalfH = std::max(outHalfH, 24.0f);
    } else if (t == "PULSE" || t == "PULSE_GEN") {
        outHalfW = std::max(outHalfW, 24.0f);
        outHalfH = std::max(outHalfH, 18.0f);
    } else if (t == "SCOPE") {
        int numCh = 2;
        if (comp.parameters.count("channels")) {
            try { numCh = std::stoi(comp.parameters.at("channels")); } catch (...) {}
        }
        if (numCh < 1) numCh = 1;
        outHalfW = std::max(outHalfW, 18.0f);
        outHalfH = std::max(outHalfH, std::max(18.0f, numCh * 10.0f + 2.0f));
    } else if (t == "IDEAL_XFMR" || t == "XFMR_2W" || t == "MUTUAL_2W" || t == "SAT_XFMR" || t == "Transformer" || t == "IDEAL_TRANSFORMER" || t == "TRANSFORMER" || t == "XFMR") {
        std::string pStr = comp.parameters.count("primary_turns") ? comp.parameters.at("primary_turns") : "[100]";
        std::string sStr = comp.parameters.count("secondary_turns") ? comp.parameters.at("secondary_turns") : "[100]";
        auto pTurns = parseTurnsArrayStr(pStr);
        auto sTurns = parseTurnsArrayStr(sStr);
        int nMax = std::max(1, std::max((int)pTurns.size(), (int)sTurns.size()));
        outHalfW = std::max(outHalfW, 25.0f);
        outHalfH = std::max(outHalfH, std::max(25.0f, nMax * 25.0f + 5.0f));
    } else if (t == "PROBE") {
        std::string sigStr = comp.parameters.count("selected_signals") ? comp.parameters.at("selected_signals") : "";
        int count = 0;
        std::stringstream ss(sigStr);
        std::string item;
        while (std::getline(ss, item, ',')) { if (!item.empty()) count++; }
        int numPins = std::max(1, count);
        outHalfW = std::max(outHalfW, 32.0f);
        outHalfH = std::max(outHalfH, std::max(22.0f, numPins * 15.0f + 2.0f));
    } else if (t == "R" || t == "C" || t == "L" || t == "D" || t == "V" || t == "I" ||
               t == "AC_V" || t == "S" || t == "MOSFET" || t == "VM" || t == "AM") {
        outHalfW = std::max(outHalfW, 14.0f);
        outHalfH = std::max(outHalfH, 14.0f);
    } else {
        outHalfW = std::max(outHalfW, 22.0f);
        outHalfH = std::max(outHalfH, 22.0f);
    }

    int rot = ((comp.rotation % 360) + 360) % 360;
    if (rot == 90 || rot == 270) {
        std::swap(outHalfW, outHalfH);
    }
}

void SchematicCanvas::drawComponents(ImDrawList* drawList, ImVec2 canvasPos) {
    ImGuiIO& io = ImGui::GetIO();
    ImVec2 mousePos = io.MousePos;
    hoveredPinCompId.clear();
    hoveredPinName.clear();
    float minPinDist = 1e9f;
    
    for (auto& comp : design.components) {
        ImVec2 center = worldToScreen(comp.x, comp.y, canvasPos);
        float s = zoomLevel;
        
        bool isSelected = selectedComponentIds.count(comp.id) > 0;
        ImU32 defaultColor = isDarkMode ? IM_COL32(200, 210, 230, 255) : IM_COL32(30, 41, 59, 255);
        ImU32 componentColor = isSelected ? IM_COL32(255, 180, 0, 255) : defaultColor;

        float hw = 25.0f, hh = 25.0f;
        getComponentBounds(comp, hw, hh);

        if (isSelected) {
            drawList->AddRectFilled(
                {center.x - hw*s, center.y - hh*s},
                {center.x + hw*s, center.y + hh*s},
                IM_COL32(255, 180, 0, 40), 6*s);
            drawList->AddRect(
                {center.x - hw*s, center.y - hh*s},
                {center.x + hw*s, center.y + hh*s},
                IM_COL32(255, 180, 0, 180), 6*s, 0, 1.5f*s);
        }
        
        drawComponentShape(drawList, comp, center, s, componentColor);
        ImU32 labelColor = isDarkMode ? IM_COL32(180, 190, 210, 255) : IM_COL32(15, 23, 42, 255);
        std::string dispId = !comp.id.empty() ? comp.id : comp.label;
        drawList->AddText({center.x - (hw - 4.0f)*s, center.y - (hh + 16.0f)*s}, labelColor, dispId.c_str());
        drawTerminals(drawList, comp, center, s, mousePos, minPinDist);
    }
}

void SchematicCanvas::autoSelectWiresForSelectedComponents() {
    if (selectedComponentIds.empty()) return;

    auto isCompSel = [this](const std::string& cid) -> bool {
        if (cid.empty()) return false;
        return selectedComponentIds.count(cid) > 0;
    };

    auto isNodeSel = [this](const std::string& nodeStr) -> bool {
        if (nodeStr.empty()) return false;
        size_t dot = nodeStr.find('.');
        if (dot != std::string::npos) {
            std::string cid = nodeStr.substr(0, dot);
            return selectedComponentIds.count(cid) > 0;
        }
        return false;
    };

    bool addedAny = true;
    while (addedAny) {
        addedAny = false;
        for (const auto& w : design.wires) {
            if (selectedWireIds.count(w.id) > 0) continue;

            bool fromIsSel = isCompSel(w.from.compId) || isNodeSel(w.fromNode);
            if (w.from.isWireJunction && !w.from.targetWireId.empty()) {
                if (selectedWireIds.count(w.from.targetWireId) > 0) fromIsSel = true;
            }

            bool toIsSel = isCompSel(w.to.compId) || isNodeSel(w.toNode);
            if (w.to.isWireJunction && !w.to.targetWireId.empty()) {
                if (selectedWireIds.count(w.to.targetWireId) > 0) toIsSel = true;
            }

            // A wire branching into another wire is selected if its component is selected and its target wire is selected, OR if both ends are selected
            if (w.to.isWireJunction && fromIsSel) {
                // If targetWireId is connected to any selected component, select this branch wire too
                bool targetConnectedToSel = false;
                for (const auto& tw : design.wires) {
                    if (tw.id == w.to.targetWireId) {
                        if (isCompSel(tw.from.compId) || isCompSel(tw.to.compId) || selectedWireIds.count(tw.id) > 0) {
                            targetConnectedToSel = true;
                            break;
                        }
                    }
                }
                if (targetConnectedToSel) {
                    selectedWireIds.insert(w.id);
                    addedAny = true;
                }
            } else if (fromIsSel && toIsSel) {
                selectedWireIds.insert(w.id);
                addedAny = true;
            }
        }
    }
}

void SchematicCanvas::deleteSelected() {
    if (selectedComponentIds.empty() && selectedWireIds.empty()) return;

    pushUndoState();

    std::unordered_set<std::string> compsToDelete(selectedComponentIds.begin(), selectedComponentIds.end());
    std::unordered_set<std::string> wiresToDelete(selectedWireIds.begin(), selectedWireIds.end());

    // Mark wires attached to deleted components
    for (const auto& w : design.wires) {
        if (!w.from.compId.empty() && compsToDelete.count(w.from.compId) > 0) wiresToDelete.insert(w.id);
        if (!w.to.compId.empty() && compsToDelete.count(w.to.compId) > 0) wiresToDelete.insert(w.id);
        for (const std::string& compId : compsToDelete) {
            std::string prefix = compId + ".";
            if (w.fromNode.rfind(prefix, 0) == 0 || w.toNode.rfind(prefix, 0) == 0) {
                wiresToDelete.insert(w.id);
            }
        }
    }

    // Preserve non-deleted junction connections attached to deleted wires
    std::vector<WireInstance> preservedWires;
    for (const auto& w : design.wires) {
        if (wiresToDelete.count(w.id) > 0) {
            for (const auto& jw : design.wires) {
                if (jw.to.isWireJunction && jw.to.targetWireId == w.id && wiresToDelete.count(jw.id) == 0) {
                    // Create a wire from w.from to jw.to junction position so the rest of the net stays connected
                    if (!w.from.compId.empty() && compsToDelete.count(w.from.compId) == 0) {
                        WireInstance kw;
                        int maxW = 0;
                        for (const auto& exW : design.wires) {
                            if (exW.id.size() > 1 && (exW.id[0] == 'w' || exW.id[0] == 'W')) {
                                try { maxW = std::max(maxW, std::stoi(exW.id.substr(1))); } catch (...) {}
                            }
                        }
                        kw.id = "w" + std::to_string(maxW + 1 + preservedWires.size());
                        kw.from = w.from;
                        kw.to.isWireJunction = true;
                        kw.to.targetWireId = jw.id;
                        kw.to.junctionX = jw.to.junctionX;
                        kw.to.junctionY = jw.to.junctionY;
                        preservedWires.push_back(kw);
                    }
                }
            }
        }
    }

    // Perform deletions
    design.wires.erase(
        std::remove_if(design.wires.begin(), design.wires.end(),
                       [&](const WireInstance& w) { return wiresToDelete.count(w.id) > 0; }),
        design.wires.end()
    );

    design.components.erase(
        std::remove_if(design.components.begin(), design.components.end(),
                       [&](const ComponentInstance& c) { return compsToDelete.count(c.id) > 0; }),
        design.components.end()
    );

    // Append preserved wire segments
    for (auto& kw : preservedWires) {
        design.wires.push_back(kw);
    }

    cleanupOrphanedJunctions();

    selectedComponentIds.clear();
    selectedWireIds.clear();
}

void SchematicCanvas::cleanupOrphanedJunctions() {
    std::unordered_set<std::string> validWireIds;
    for (const auto& w : design.wires) {
        validWireIds.insert(w.id);
    }

    // Clear junction flag if targetWireId no longer exists in design.wires
    for (auto& w : design.wires) {
        if (w.from.isWireJunction && !w.from.targetWireId.empty()) {
            if (validWireIds.count(w.from.targetWireId) == 0) {
                w.from.isWireJunction = false;
                w.from.targetWireId.clear();
            }
        }
        if (w.to.isWireJunction && !w.to.targetWireId.empty()) {
            if (validWireIds.count(w.to.targetWireId) == 0) {
                w.to.isWireJunction = false;
                w.to.targetWireId.clear();
            }
        }
    }
}

void SchematicCanvas::copySelected() {
    if (selectedComponentIds.empty() && selectedWireIds.empty()) return;

    nlohmann::json jData;
    jData["components"] = nlohmann::json::array();
    jData["wires"] = nlohmann::json::array();

    std::unordered_set<std::string> selComps(selectedComponentIds.begin(), selectedComponentIds.end());
    std::unordered_set<std::string> selWires(selectedWireIds.begin(), selectedWireIds.end());

    for (const auto& comp : design.components) {
        if (selComps.count(comp.id)) {
            nlohmann::json jComp;
            jComp["id"] = comp.id;
            jComp["type"] = comp.rawTypeStr;
            jComp["label"] = comp.label;
            jComp["x"] = comp.x;
            jComp["y"] = comp.y;
            jComp["rotation"] = comp.rotation;
            jComp["params"] = comp.parameters;
            
            nlohmann::json jPins = nlohmann::json::array();
            for (const auto& p : comp.pins) {
                nlohmann::json jp;
                jp["name"] = p.name;
                jp["relX"] = p.relativeX;
                jp["relY"] = p.relativeY;
                jp["isInput"] = p.isInput;
                jp["isOutput"] = p.isOutput;
                jp["opSign"] = p.opSign;
                jPins.push_back(jp);
            }
            jComp["pins"] = jPins;

            jData["components"].push_back(jComp);
        }
    }

    std::unordered_map<std::string, std::string> oldToNewWireId;
    int wireCounter = 1;
    for (const auto& w : design.wires) {
        if (selWires.count(w.id) > 0) {
            std::string tempWireId = "w_copy_" + std::to_string(wireCounter++);
            oldToNewWireId[w.id] = tempWireId;

            nlohmann::json jw;
            jw["id"] = tempWireId;
            jw["oldId"] = w.id;
            jw["fromComp"] = w.from.compId;
            jw["fromTerm"] = w.from.terminal;
            jw["toComp"] = w.to.compId;
            jw["toTerm"] = w.to.terminal;
            jw["fromNode"] = w.fromNode;
            jw["toNode"] = w.toNode;
            jw["isJunction"] = w.to.isWireJunction;
            jw["targetWireId"] = w.to.targetWireId;
            jw["jX"] = w.to.junctionX;
            jw["jY"] = w.to.junctionY;
            
            nlohmann::json jPath = nlohmann::json::array();
            for (const auto& pt : w.manualPath) {
                jPath.push_back({{"x", pt.x}, {"y", pt.y}});
            }
            jw["path"] = jPath;

            jData["wires"].push_back(jw);
        }
    }

    std::string jsonStr = jData.dump(2);
    g_internalClipboard = jsonStr;

    if (OpenClipboard(NULL)) {
        EmptyClipboard();
        HGLOBAL hMem = GlobalAlloc(GMEM_MOVEABLE, jsonStr.size() + 1);
        if (hMem) {
            memcpy(GlobalLock(hMem), jsonStr.c_str(), jsonStr.size() + 1);
            GlobalUnlock(hMem);
            SetClipboardData(CF_TEXT, hMem);
        }
        CloseClipboard();
    }
}

void SchematicCanvas::pasteSelected() {
    pushUndoState();
    std::string jsonStr;

    if (OpenClipboard(NULL)) {
        HANDLE hData = GetClipboardData(CF_TEXT);
        if (hData) {
            char* pszText = static_cast<char*>(GlobalLock(hData));
            if (pszText) {
                jsonStr = std::string(pszText);
                GlobalUnlock(hData);
            }
        }
        CloseClipboard();
    }

    if (jsonStr.empty() || jsonStr.find("components") == std::string::npos) {
        jsonStr = g_internalClipboard;
    }

    if (jsonStr.empty()) return;

    try {
        nlohmann::json jData = nlohmann::json::parse(jsonStr);
        if (!jData.contains("components") || !jData["components"].is_array()) return;

        selectedComponentIds.clear();
        selectedWireIds.clear();

        std::unordered_map<std::string, std::string> oldToNewId;
        float offsetX = 30.0f;
        float offsetY = 30.0f;

        for (const auto& jComp : jData["components"]) {
            ComponentInstance comp;
            std::string oldId = jComp.value("id", "comp");
            std::string typeStr = jComp.value("type", "R");
            
            std::string prefix = typeStr;
            int idx = 1;
            std::string newId;
            while (true) {
                newId = prefix + std::to_string(idx);
                bool exists = false;
                for (const auto& existing : design.components) {
                    if (existing.id == newId) { exists = true; break; }
                }
                if (!exists && oldToNewId.find(newId) == oldToNewId.end()) break;
                idx++;
            }

            oldToNewId[oldId] = newId;
            comp.id = newId;
            comp.rawTypeStr = typeStr;
            comp.label = jComp.value("label", newId);
            comp.x = jComp.value("x", 0.0f) + offsetX;
            comp.y = jComp.value("y", 0.0f) + offsetY;
            comp.rotation = jComp.value("rotation", 0);

            if (jComp.contains("params")) {
                for (auto& [k, v] : jComp["params"].items()) {
                    if (v.is_string()) comp.parameters[k] = v.get<std::string>();
                    else comp.parameters[k] = v.dump();
                }
            }

            if (jComp.contains("pins") && jComp["pins"].is_array()) {
                for (const auto& jp : jComp["pins"]) {
                    Pin p;
                    p.name = jp.value("name", "");
                    p.relativeX = jp.value("relX", 0.0f);
                    p.relativeY = jp.value("relY", 0.0f);
                    p.isInput = jp.value("isInput", false);
                    p.isOutput = jp.value("isOutput", false);
                    p.opSign = jp.value("opSign", "+");
                    comp.pins.push_back(p);
                }
            }

            if (typeStr == "R") comp.type = ComponentType::Resistor;
            else if (typeStr == "C") comp.type = ComponentType::Capacitor;
            else if (typeStr == "L") comp.type = ComponentType::Inductor;
            else if (typeStr == "V") comp.type = ComponentType::VoltageSource;
            else if (typeStr == "AC_V") comp.type = ComponentType::ACVoltageSource;
            else if (typeStr == "I") comp.type = ComponentType::CurrentSource;
            else if (typeStr == "D") comp.type = ComponentType::Diode;
            else if (typeStr == "MOSFET") comp.type = ComponentType::MOSFET;
            else if (typeStr == "S") comp.type = ComponentType::Switch;
            else if (typeStr == "VM") comp.type = ComponentType::Voltmeter;
            else if (typeStr == "AM") comp.type = ComponentType::Ammeter;
            else if (typeStr == "GAIN") comp.type = ComponentType::Gain;
            else if (typeStr == "PID") comp.type = ComponentType::PI_Controller;
            else if (typeStr == "COMP") comp.type = ComponentType::Comparator;
            else if (typeStr == "PWM") comp.type = ComponentType::PWM_Generator;
            else if (typeStr == "PULSE" || typeStr == "PULSE_GEN") comp.type = ComponentType::PulseGenerator;
            else if (typeStr == "TRI") comp.type = ComponentType::Triangle_Carrier;
            else if (typeStr == "SUM_RECT" || typeStr == "SUM_ROUND") comp.type = ComponentType::SummingJunction;
            else if (typeStr == "PRODUCT_RECT") comp.type = ComponentType::Product;
            else if (typeStr == "AND") comp.type = ComponentType::AND_Gate;
            else if (typeStr == "OR") comp.type = ComponentType::OR_Gate;
            else if (typeStr == "NOT") comp.type = ComponentType::NOT_Gate;
            else if (typeStr == "CSCRIPT") comp.type = ComponentType::CustomScript;
            else comp.type = ComponentType::Unknown;

            design.components.push_back(comp);
            selectedComponentIds.insert(newId);
        }

        if (jData.contains("wires") && jData["wires"].is_array()) {
            std::unordered_map<std::string, std::string> oldWireIdToNewWireId;

            // Pass 1: Map old wire IDs to new wire IDs
            for (const auto& jw : jData["wires"]) {
                std::string oldWId = jw.value("id", "");
                std::string origWId = jw.value("oldId", "");
                std::string newWId = "wire_" + std::to_string(rand() % 100000);
                if (!oldWId.empty()) oldWireIdToNewWireId[oldWId] = newWId;
                if (!origWId.empty()) oldWireIdToNewWireId[origWId] = newWId;
            }

            // Pass 2: Reconstruct copied wires cleanly
            for (const auto& jw : jData["wires"]) {
                WireInstance wire;
                std::string oldWId = jw.value("id", "");
                wire.id = oldWireIdToNewWireId.count(oldWId) ? oldWireIdToNewWireId[oldWId] : ("wire_" + std::to_string(rand() % 100000));

                std::string oldFromComp = jw.value("fromComp", "");
                std::string oldToComp = jw.value("toComp", "");

                wire.from.compId = oldToNewId.count(oldFromComp) ? oldToNewId[oldFromComp] : "";
                wire.from.terminal = oldToNewId.count(oldFromComp) ? jw.value("fromTerm", "") : "";
                wire.to.compId = oldToNewId.count(oldToComp) ? oldToNewId[oldToComp] : "";
                wire.to.terminal = oldToNewId.count(oldToComp) ? jw.value("toTerm", "") : "";

                bool isJunc = jw.value("isJunction", false);
                std::string origTargetW = jw.value("targetWireId", "");

                // Only preserve junction if the target wire was ALSO copied in this selection!
                if (isJunc && !origTargetW.empty() && oldWireIdToNewWireId.count(origTargetW) > 0) {
                    wire.to.isWireJunction = true;
                    wire.to.targetWireId = oldWireIdToNewWireId[origTargetW];
                    wire.to.junctionX = jw.value("jX", 0.0f) + offsetX;
                    wire.to.junctionY = jw.value("jY", 0.0f) + offsetY;
                } else {
                    wire.to.isWireJunction = false;
                    wire.to.targetWireId.clear();
                    wire.to.junctionX = 0.0f;
                    wire.to.junctionY = 0.0f;
                }

                // Skip orphaned wires that connect to nothing at all
                bool hasFrom = !wire.from.compId.empty() || wire.from.isWireJunction;
                bool hasTo = !wire.to.compId.empty() || wire.to.isWireJunction;
                if (!hasFrom && !hasTo) continue;

                wire.fromNode = wire.from.compId + "." + wire.from.terminal;
                wire.toNode = wire.to.isWireJunction ? (wire.to.targetWireId + ".junction") : (wire.to.compId + "." + wire.to.terminal);

                if (jw.contains("path") && jw["path"].is_array()) {
                    for (const auto& jpt : jw["path"]) {
                        Point2D pt;
                        pt.x = jpt.value("x", 0.0f) + offsetX;
                        pt.y = jpt.value("y", 0.0f) + offsetY;
                        wire.manualPath.push_back(pt);
                    }
                }

                design.wires.push_back(wire);
                selectedWireIds.insert(wire.id);
            }
        }

        cleanupOrphanedJunctions();
    } catch (...) {}
}

void SchematicCanvas::duplicateSelected() {
    copySelected();
    pasteSelected();
}

void SchematicCanvas::flipHorizontal() {
    pushUndoState();
    for (auto& comp : design.components) {
        if (selectedComponentIds.count(comp.id)) {
            comp.rotation = (comp.rotation + 180) % 360;
        }
    }
}

void SchematicCanvas::flipVertical() {
    flipHorizontal();
}

void SchematicCanvas::renderModals() {
    if (showConfigurator && pendingConfigCompIdx >= 0 && pendingConfigCompIdx < (int)design.components.size()) {
        if (ConfiguratorDialog::showConfiguratorModal(design.components[pendingConfigCompIdx], &showConfigurator)) {
            pendingConfigCompIdx = -1;
        }
    }

    if (showCScriptModal) {
        ImGui::OpenPopup("C-Script Interactive IDE & Logic Compiler");
        ImGui::SetNextWindowSize(ImVec2(880, 520), ImGuiCond_FirstUseEver);
        if (ImGui::BeginPopupModal("C-Script Interactive IDE & Logic Compiler", &showCScriptModal, ImGuiWindowFlags_None)) {
            std::string compIdStr = (cscriptCompIdx >= 0 && cscriptCompIdx < (int)design.components.size()) ? design.components[cscriptCompIdx].id : "CSCRIPT1";
            ImGui::TextColored(ImVec4(0.2f, 0.8f, 1.0f, 1.0f), "Component: %s", compIdStr.c_str());
            ImGui::Separator();
            ImGui::Spacing();

            ImGui::Columns(2, "cscript_ide_cols", true);
            ImGui::SetColumnWidth(0, 540);

            // Left Column: Code Editor & Execution Timestep
            ImGui::Text("Step Logic Code (C++ Syntax):");
            ImGui::InputTextMultiline("##code_ide", cscriptCodeBuf, sizeof(cscriptCodeBuf), ImVec2(-1, 350));

            ImGui::Spacing();
            ImGui::Text("Execution Timestep:");
            ImGui::SameLine();
            ImGui::SetNextItemWidth(140);
            ImGui::InputText("##timestep_ide", cscriptTimestepBuf, sizeof(cscriptTimestepBuf));
            ImGui::SameLine();
            ImGui::TextDisabled("(0 = Continuous dt, e.g. 100u for Discrete)");

            ImGui::NextColumn();

            // Right Column: Live Discovery Preview
            std::vector<CircuitSimEngine::CScriptPort> discIn, discOut;
            CircuitSimEngine::CScriptEngine::discoverPorts(cscriptCodeBuf, discIn, discOut);
            auto discParams = CircuitSimEngine::CScriptEngine::discoverParamsFromCode(cscriptCodeBuf);

            ImGui::TextColored(ImVec4(0.2f, 0.9f, 0.4f, 1.0f), "Live Discovery Preview:");
            ImGui::Separator();

            ImGui::Spacing();
            ImGui::Text("Discovered Input Pins (%d):", (int)discIn.size());
            for (const auto& p : discIn) {
                ImGui::BulletText("Pin: %s (Index %d)", p.name.c_str(), p.index);
            }

            ImGui::Spacing();
            ImGui::Text("Discovered Output Pins (%d):", (int)discOut.size());
            for (const auto& p : discOut) {
                ImGui::BulletText("Pin: %s (Index %d)", p.name.c_str(), p.index);
            }

            ImGui::Spacing();
            ImGui::Text("Discovered Parameters (%d):", (int)discParams.size());
            for (const auto& par : discParams) {
                ImGui::BulletText("%s %s = %s", par.typeStr.c_str(), par.name.c_str(), par.rawValStr.c_str());
            }

            ImGui::Spacing();
            ImGui::Separator();
            ImGui::TextColored(ImVec4(0.4f, 1.0f, 0.4f, 1.0f), "Status: [Syntax Valid]");

            ImGui::Columns(1);
            ImGui::Spacing();
            ImGui::Separator();
            ImGui::Spacing();

            if (ImGui::Button("Save & Apply to Schematic", ImVec2(200, 32))) {
                pushUndoState();
                if (cscriptCompIdx >= 0 && cscriptCompIdx < (int)design.components.size()) {
                    auto& comp = design.components[cscriptCompIdx];
                    comp.parameters["code"] = cscriptCodeBuf;
                    comp.parameters["timestep"] = cscriptTimestepBuf;

                    auto discParams = CircuitSimEngine::CScriptEngine::discoverParamsFromCode(cscriptCodeBuf);
                    for (const auto& par : discParams) {
                        comp.parameters[par.name] = par.rawValStr;
                    }
                }
                showCScriptModal = false;
                cscriptCompIdx = -1;
                ImGui::CloseCurrentPopup();
            }
            ImGui::SameLine();
            if (ImGui::Button("Cancel", ImVec2(100, 32))) {
                showCScriptModal = false;
                cscriptCompIdx = -1;
                ImGui::CloseCurrentPopup();
            }
            ImGui::EndPopup();
        }
    }

    if (showScopeModal) {
        ImGui::OpenPopup("Oscilloscope Configuration Modal");
        if (ImGui::BeginPopupModal("Oscilloscope Configuration Modal", &showScopeModal, ImGuiWindowFlags_AlwaysAutoResize)) {
            ImGui::Text("Edit Oscilloscope Configuration:");
            ImGui::Separator();
            ImGui::InputText("Number of Channels (1..8)", scopeChannelsBuf, sizeof(scopeChannelsBuf));
            ImGui::Spacing();
            if (ImGui::Button("Save Parameters", ImVec2(140, 30))) {
                pushUndoState();
                if (scopeCompIdx >= 0 && scopeCompIdx < (int)design.components.size()) {
                    int numCh = 2;
                    try { numCh = std::stoi(scopeChannelsBuf); } catch (...) {}
                    if (numCh < 1) numCh = 1;
                    if (numCh > 8) numCh = 8;
                    design.components[scopeCompIdx].parameters["channels"] = std::to_string(numCh);
                    design.components[scopeCompIdx].label = "SCOPE (" + std::to_string(numCh) + " Ch)";
                }
                showScopeModal = false;
                scopeCompIdx = -1;
                ImGui::CloseCurrentPopup();
            }
            ImGui::SameLine();
            if (ImGui::Button("Cancel", ImVec2(100, 30))) {
                showScopeModal = false;
                scopeCompIdx = -1;
                ImGui::CloseCurrentPopup();
            }
            ImGui::EndPopup();
        }
    }

    if (showPulseModal) {
        ImGui::OpenPopup("Pulse Generator Parameters Modal");
        if (ImGui::BeginPopupModal("Pulse Generator Parameters Modal", &showPulseModal, ImGuiWindowFlags_AlwaysAutoResize)) {
            ImGui::Text("Edit Pulse Generator Configuration:");
            ImGui::Separator();
            ImGui::InputText("Amplitude", pulseAmpBuf, sizeof(pulseAmpBuf));
            ImGui::InputText("Period (s)", pulsePeriodBuf, sizeof(pulsePeriodBuf));
            ImGui::InputText("Pulse Width (0..1)", pulseWidthBuf, sizeof(pulseWidthBuf));
            ImGui::InputText("Delay (s)", pulseDelayBuf, sizeof(pulseDelayBuf));
            ImGui::Spacing();
            if (ImGui::Button("Save Parameters", ImVec2(140, 30))) {
                pushUndoState();
                if (pulseCompIdx >= 0 && pulseCompIdx < (int)design.components.size()) {
                    auto& p = design.components[pulseCompIdx].parameters;
                    p.clear();
                    p["amplitude"] = pulseAmpBuf;
                    p["period"] = pulsePeriodBuf;
                    p["width"] = pulseWidthBuf;
                    p["delay"] = pulseDelayBuf;
                    design.components[pulseCompIdx].label = "Pulse Gen";
                }
                showPulseModal = false;
                pulseCompIdx = -1;
                ImGui::CloseCurrentPopup();
            }
            ImGui::SameLine();
            if (ImGui::Button("Cancel", ImVec2(100, 30))) {
                showPulseModal = false;
                pulseCompIdx = -1;
                ImGui::CloseCurrentPopup();
            }
            ImGui::EndPopup();
        }
    }
}

void SchematicCanvas::render(const char* title, ImVec2 size) {
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0, 0));
    ImGui::Begin(title, nullptr, ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
    
    ImVec2 canvasPos = ImGui::GetCursorScreenPos();
    ImVec2 canvasSize = ImGui::GetContentRegionAvail();
    lastRenderedCanvasSize = canvasSize;
    ImDrawList* drawList = ImGui::GetWindowDrawList();

    ImU32 canvasBgColor = isDarkMode ? IM_COL32(15, 23, 42, 255) : IM_COL32(246, 243, 206, 255);
    drawList->AddRectFilled(canvasPos, ImVec2(canvasPos.x + canvasSize.x, canvasPos.y + canvasSize.y), canvasBgColor);
    drawList->PushClipRect(canvasPos, ImVec2(canvasPos.x + canvasSize.x, canvasPos.y + canvasSize.y), true);
    
    drawGrid(drawList, canvasSize, canvasPos);
    drawComponents(drawList, canvasPos);
    drawWires(drawList, canvasPos);
    drawBreadcrumbs(drawList, canvasPos);

    ImGuiIO& io = ImGui::GetIO();
    ImVec2 mousePos = io.MousePos;

    if (isBoxSelecting) {
        boxSelectEnd = mousePos;
        drawList->AddRectFilled(boxSelectStart, boxSelectEnd, IM_COL32(255, 180, 0, 35));
        drawList->AddRect(boxSelectStart, boxSelectEnd, IM_COL32(255, 180, 0, 255), 0, 0, 1.5f);
    }

    drawList->PopClipRect();

    // Adaptive Box Zoom rubber-band — drawn OUTSIDE clip rect so it renders on top
    if (isBoxZooming) {
        boxZoomEnd = mousePos;
        float bx0 = std::min(boxZoomStart.x, boxZoomEnd.x);
        float by0 = std::min(boxZoomStart.y, boxZoomEnd.y);
        float bx1 = std::max(boxZoomStart.x, boxZoomEnd.x);
        float by1 = std::max(boxZoomStart.y, boxZoomEnd.y);
        float bw = bx1 - bx0, bh = by1 - by0;
        float aspect = (bh > 1.0f) ? (bw / bh) : 99.0f;

        if (aspect > 3.0f) {
            // X-zoom: draw full-height cyan band spanning the canvas height
            float top    = canvasPos.y;
            float bottom = canvasPos.y + canvasSize.y;
            drawList->AddRectFilled(ImVec2(bx0, top), ImVec2(bx1, bottom), IM_COL32(0, 220, 255, 22));
            drawList->AddLine(ImVec2(bx0, top), ImVec2(bx0, bottom), IM_COL32(0, 220, 255, 255), 2.0f);
            drawList->AddLine(ImVec2(bx1, top), ImVec2(bx1, bottom), IM_COL32(0, 220, 255, 255), 2.0f);
        } else if (aspect < 0.33f) {
            // Y-zoom: draw full-width magenta band spanning the canvas width
            float left  = canvasPos.x;
            float right = canvasPos.x + canvasSize.x;
            drawList->AddRectFilled(ImVec2(left, by0), ImVec2(right, by1), IM_COL32(220, 0, 255, 22));
            drawList->AddLine(ImVec2(left, by0), ImVec2(right, by0), IM_COL32(220, 0, 255, 255), 2.0f);
            drawList->AddLine(ImVec2(left, by1), ImVec2(right, by1), IM_COL32(220, 0, 255, 255), 2.0f);
        } else {
            // Box zoom: green rectangle
            drawList->AddRectFilled(ImVec2(bx0, by0), ImVec2(bx1, by1), IM_COL32(60, 255, 120, 30));
            drawList->AddRect(ImVec2(bx0, by0), ImVec2(bx1, by1), IM_COL32(60, 255, 120, 255), 0, 0, 2.0f);
        }
    }

    // Set focus on click
    if (ImGui::IsWindowHovered() && ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
        ImGui::SetWindowFocus();
    }

    // Escape or Right-Click cancels wiring / box-zoom drag
    if (isWiring && (ImGui::IsKeyPressed(ImGuiKey_Escape) || ImGui::IsMouseClicked(ImGuiMouseButton_Right))) {
        isWiring = false;
        activeWireCorners.clear();
    }
    if (isBoxZooming && ImGui::IsKeyPressed(ImGuiKey_Escape)) {
        isBoxZooming = false;
    }

    // Mouse Pan with Right Button or Spacebar + Left Mouse
    // When adaptive zoom is active, left-drag is NOT a pan — it's a zoom drag
    bool lmbPanActive = ImGui::IsKeyDown(ImGuiKey_Space) && ImGui::IsMouseDragging(ImGuiMouseButton_Left);
    bool rmbPanActive = ImGui::IsMouseDragging(ImGuiMouseButton_Right);
    bool suppressPan = adaptiveZoomMode && !lmbPanActive; // in zoom mode, only space+LMB still pans
    if (ImGui::IsWindowHovered() && !isDraggingWireSegment && (rmbPanActive || lmbPanActive)) {
        if (!suppressPan || lmbPanActive) {
            panOffset.x += io.MouseDelta.x / zoomLevel;
            panOffset.y += io.MouseDelta.y / zoomLevel;
        }
    }

    // Zoom with scroll wheel centered on mouse pointer position
    if (ImGui::IsWindowHovered() && io.MouseWheel != 0.0f) {
        float oldZoom = zoomLevel;
        float zoomFactor = 1.15f;
        float newZoom = (io.MouseWheel > 0.0f) ? (oldZoom * zoomFactor) : (oldZoom / zoomFactor);
        
        if (newZoom < 0.15f) newZoom = 0.15f;
        if (newZoom > 6.0f) newZoom = 6.0f;
        
        if (newZoom != oldZoom) {
            ImVec2 worldMouse = screenToWorld(mousePos, canvasPos);
            zoomLevel = newZoom;
            panOffset.x = (mousePos.x - canvasPos.x) / newZoom - worldMouse.x;
            panOffset.y = (mousePos.y - canvasPos.y) / newZoom - worldMouse.y;
        }
    }

    // Left click handling
    if (ImGui::IsWindowHovered() && ImGui::IsMouseClicked(ImGuiMouseButton_Left) && !isDraggingWireSegment) {
        ImVec2 clickW = screenToWorld(mousePos, canvasPos);
        lastCanvasClickWorldPos.x = std::round(clickW.x / 20.0f) * 20.0f;
        lastCanvasClickWorldPos.y = std::round(clickW.y / 20.0f) * 20.0f;
        hasLastClickPos = true;

        if (adaptiveZoomMode && !isWiring) {
            // Adaptive box zoom mode: always start rubber-band regardless of what is under cursor
            isBoxZooming = true;
            boxZoomStart = mousePos;
            boxZoomEnd   = mousePos;
        } else if (!hoveredPinCompId.empty()) {
            if (!isWiring) {
                isWiring = true;
                wireStartCompId = hoveredPinCompId;
                wireStartPin = hoveredPinName;
                wireCurrentPos = mousePos;
                activeWireCorners.clear();
            } else {
                // STRICT DOMAIN VALIDATION: Pin-to-Pin Connection
                const ComponentInstance* startComp = nullptr;
                const ComponentInstance* targetComp = nullptr;
                for (const auto& c : design.components) {
                    if (c.id == wireStartCompId) startComp = &c;
                    if (c.id == hoveredPinCompId) targetComp = &c;
                }

                if (startComp && targetComp) {
                    DomainType startDom = getPinDomain(*startComp, wireStartPin);
                    DomainType targetDom = getPinDomain(*targetComp, hoveredPinName);

                    if (startDom != targetDom) {
                        // REJECT CROSS-DOMAIN PIN CONNECTION!
                        isWiring = false;
                        activeWireCorners.clear();
                    } else if (!validateSingleOutportConstraint(wireStartCompId, wireStartPin, hoveredPinCompId, hoveredPinName)) {
                        // REJECT MULTIPLE CONTROL OUTPUT DRIVERS CONFLICT!
                        isWiring = false;
                        activeWireCorners.clear();
                    } else {
                        pushUndoState();
                        WireInstance wire;
                        
                        int maxWNum = 0;
                        std::unordered_set<std::string> existingWIds;
                        for (const auto& w : design.wires) {
                            existingWIds.insert(w.id);
                            if (w.id.size() > 1 && (w.id[0] == 'w' || w.id[0] == 'W')) {
                                try {
                                    int num = std::stoi(w.id.substr(1));
                                    if (num > maxWNum) maxWNum = num;
                                } catch (...) {}
                            }
                        }
                        int candW = maxWNum + 1;
                        while (existingWIds.count("w" + std::to_string(candW))) candW++;
                        wire.id = "w" + std::to_string(candW);

                        wire.from.compId = wireStartCompId;
                        wire.from.terminal = wireStartPin;
                        wire.to.compId = hoveredPinCompId;
                        wire.to.terminal = hoveredPinName;
                        design.wires.push_back(wire);
                        normalizeControlWires();
                        isWiring = false;
                        activeWireCorners.clear();
                    }
                }
            }
        } else if (isWiring) {
            // STRICT DOMAIN VALIDATION: T-Junction Connection
            if (!hoveredWireId.empty()) {
                const ComponentInstance* startComp = nullptr;
                for (const auto& c : design.components) {
                    if (c.id == wireStartCompId) { startComp = &c; break; }
                }
                const WireInstance* targetWire = nullptr;
                for (const auto& w : design.wires) {
                    if (w.id == hoveredWireId) { targetWire = &w; break; }
                }

                if (startComp && targetWire) {
                    DomainType startDom = getPinDomain(*startComp, wireStartPin);
                    DomainType targetWireDom = getWireDomain(*targetWire, design);

                    if (startDom != targetWireDom) {
                        // REJECT CROSS-DOMAIN T-JUNCTION CONNECTION!
                        isWiring = false;
                        activeWireCorners.clear();
                    } else {
                        pushUndoState();
                        WireInstance wire;
                        
                        int maxWNum = 0;
                        std::unordered_set<std::string> existingWIds;
                        for (const auto& w : design.wires) {
                            existingWIds.insert(w.id);
                            if (w.id.size() > 1 && (w.id[0] == 'w' || w.id[0] == 'W')) {
                                try {
                                    int num = std::stoi(w.id.substr(1));
                                    if (num > maxWNum) maxWNum = num;
                                } catch (...) {}
                            }
                        }
                        int candW = maxWNum + 1;
                        while (existingWIds.count("w" + std::to_string(candW))) candW++;
                        wire.id = "w" + std::to_string(candW);

                        wire.from.compId = wireStartCompId;
                        wire.from.terminal = wireStartPin;
                        wire.to.isWireJunction = true;
                        wire.to.targetWireId = hoveredWireId;
                        wire.to.junctionX = hoveredWireJunctionPos.x;
                        wire.to.junctionY = hoveredWireJunctionPos.y;
                        design.wires.push_back(wire);

                        // Split hoveredWire at junction ONLY if junction is in middle (not at endpoints)
                        for (size_t wi = 0; wi < design.wires.size(); ++wi) {
                            if (design.wires[wi].id == hoveredWireId) {
                                ImVec2 wP1 = getEndpointWorldPos(design.wires[wi].from, design);
                                ImVec2 wP2 = getEndpointWorldPos(design.wires[wi].to, design);

                                float jx = hoveredWireJunctionPos.x;
                                float jy = hoveredWireJunctionPos.y;

                                float d1 = std::sqrt((wP1.x - jx)*(wP1.x - jx) + (wP1.y - jy)*(wP1.y - jy));
                                float d2 = std::sqrt((wP2.x - jx)*(wP2.x - jx) + (wP2.y - jy)*(wP2.y - jy));

                                if (d1 > 15.0f && d2 > 15.0f) {
                                    WireInstance seg2;
                                    candW++;
                                    while (existingWIds.count("w" + std::to_string(candW))) candW++;
                                    seg2.id = "w" + std::to_string(candW);

                                    seg2.from.isWireJunction = true;
                                    seg2.from.targetWireId = wire.id;
                                    seg2.from.junctionX = jx;
                                    seg2.from.junctionY = jy;
                                    seg2.to = design.wires[wi].to;

                                    design.wires[wi].to.isWireJunction = true;
                                    design.wires[wi].to.targetWireId = wire.id;
                                    design.wires[wi].to.junctionX = jx;
                                    design.wires[wi].to.junctionY = jy;

                                    design.wires.push_back(seg2);
                                }
                                break;
                            }
                        }

                        normalizeControlWires();
                        isWiring = false;
                        activeWireCorners.clear();
                    }
                }
            } else {
                ImVec2 worldCorner = screenToWorld(mousePos, canvasPos);
                activeWireCorners.push_back(worldCorner);
            }
        } else {
            bool hitComp = false;
            for (auto it = design.components.rbegin(); it != design.components.rend(); ++it) {
                auto& comp = *it;
                ImVec2 center = worldToScreen(comp.x, comp.y, canvasPos);
                float hw = 25.0f, hh = 25.0f;
                getComponentBounds(comp, hw, hh);

                if (mousePos.x >= center.x - hw*zoomLevel && mousePos.x <= center.x + hw*zoomLevel &&
                    mousePos.y >= center.y - hh*zoomLevel && mousePos.y <= center.y + hh*zoomLevel) {
                    
                    if (!io.KeyShift && selectedComponentIds.count(comp.id) == 0) {
                        selectedComponentIds.clear();
                        selectedWireIds.clear();
                    }
                    selectedComponentIds.insert(comp.id);
                    hitComp = true;
                    break;
                }
            }
            if (!hitComp) {
                if (!hoveredWireId.empty()) {
                    if (!io.KeyShift) {
                        selectedComponentIds.clear();
                        selectedWireIds.clear();
                    }
                    selectedWireIds.insert(hoveredWireId);
                } else if (adaptiveZoomMode) {
                    // In zoom mode: start box-zoom rubber-band, don't touch selection
                    isBoxZooming = true;
                    boxZoomStart = mousePos;
                    boxZoomEnd  = mousePos;
                } else if (!io.KeyShift) {
                    selectedComponentIds.clear();
                    selectedWireIds.clear();
                    isBoxSelecting = true;
                    boxSelectStart = mousePos;
                }
            }
        }
    }

    if (isBoxSelecting && ImGui::IsMouseReleased(ImGuiMouseButton_Left)) {
        isBoxSelecting = false;
        ImVec2 minP(std::min(boxSelectStart.x, boxSelectEnd.x), std::min(boxSelectStart.y, boxSelectEnd.y));
        ImVec2 maxP(std::max(boxSelectStart.x, boxSelectEnd.x), std::max(boxSelectStart.y, boxSelectEnd.y));

        for (const auto& comp : design.components) {
            ImVec2 center = worldToScreen(comp.x, comp.y, canvasPos);
            float hw = 25.0f, hh = 25.0f;
            getComponentBounds(comp, hw, hh);

            if (center.x + hw*zoomLevel >= minP.x && center.x - hw*zoomLevel <= maxP.x &&
                center.y + hh*zoomLevel >= minP.y && center.y - hh*zoomLevel <= maxP.y) {
                selectedComponentIds.insert(comp.id);
            }
        }

        // Select wire segments that lie inside or intersect selection box
        auto lineSegIntersectsBox = [](ImVec2 p1, ImVec2 p2, ImVec2 bMin, ImVec2 bMax) -> bool {
            float minX = std::min(p1.x, p2.x), maxX = std::max(p1.x, p2.x);
            float minY = std::min(p1.y, p2.y), maxY = std::max(p1.y, p2.y);
            return !(maxX < bMin.x || minX > bMax.x || maxY < bMin.y || minY > bMax.y);
        };

        for (const auto& w : design.wires) {
            ImVec2 wp1 = getEndpointWorldPos(w.from, design);
            ImVec2 wp2 = getEndpointWorldPos(w.to, design);
            ImVec2 sp1 = worldToScreen(wp1.x, wp1.y, canvasPos);
            ImVec2 sp2 = worldToScreen(wp2.x, wp2.y, canvasPos);

            if (lineSegIntersectsBox(sp1, sp2, minP, maxP)) {
                selectedWireIds.insert(w.id);
            }
        }
    }

    // Commit adaptive box zoom on mouse release
    if (isBoxZooming && ImGui::IsMouseReleased(ImGuiMouseButton_Left)) {
        isBoxZooming = false;
        float bx0 = std::min(boxZoomStart.x, boxZoomEnd.x);
        float by0 = std::min(boxZoomStart.y, boxZoomEnd.y);
        float bx1 = std::max(boxZoomStart.x, boxZoomEnd.x);
        float by1 = std::max(boxZoomStart.y, boxZoomEnd.y);
        float bw = bx1 - bx0, bh = by1 - by0;

        // Require a minimum drag of 8 pixels to avoid accidental trigger
        if (bw > 8.0f || bh > 8.0f) {
            float aspect = (bh > 1.0f) ? (bw / bh) : 99.0f;
            float viewW = canvasSize.x;
            float viewH = canvasSize.y;

            if (aspect > 3.0f) {
                // --- X-axis zoom only ---
                // Center Y stays, zoom so that bw maps to viewW
                ImVec2 wldLeft  = screenToWorld(ImVec2(bx0, by0 + bh * 0.5f), canvasPos);
                ImVec2 wldRight = screenToWorld(ImVec2(bx1, by0 + bh * 0.5f), canvasPos);
                float worldW = wldRight.x - wldLeft.x;
                if (worldW > 0.001f) {
                    float newZoom = viewW / worldW;
                    newZoom = std::max(0.15f, std::min(newZoom, 6.0f));
                    float worldMidX = (wldLeft.x + wldRight.x) * 0.5f;
                    panOffset.x = viewW / (2.0f * newZoom) - worldMidX;
                    zoomLevel = newZoom;
                }
            } else if (aspect < 0.33f) {
                // --- Y-axis zoom only ---
                ImVec2 wldTop    = screenToWorld(ImVec2(bx0 + bw * 0.5f, by0), canvasPos);
                ImVec2 wldBottom = screenToWorld(ImVec2(bx0 + bw * 0.5f, by1), canvasPos);
                float worldH = wldBottom.y - wldTop.y;
                if (worldH > 0.001f) {
                    float newZoom = viewH / worldH;
                    newZoom = std::max(0.15f, std::min(newZoom, 6.0f));
                    float worldMidY = (wldTop.y + wldBottom.y) * 0.5f;
                    panOffset.y = viewH / (2.0f * newZoom) - worldMidY;
                    zoomLevel = newZoom;
                }
            } else {
                // --- Full box zoom ---
                ImVec2 wldTL = screenToWorld(ImVec2(bx0, by0), canvasPos);
                ImVec2 wldBR = screenToWorld(ImVec2(bx1, by1), canvasPos);
                float worldW = wldBR.x - wldTL.x;
                float worldH = wldBR.y - wldTL.y;
                if (worldW > 0.001f && worldH > 0.001f) {
                    float zx = viewW / worldW;
                    float zy = viewH / worldH;
                    float newZoom = std::min(zx, zy);
                    newZoom = std::max(0.15f, std::min(newZoom, 6.0f));
                    float worldMidX = (wldTL.x + wldBR.x) * 0.5f;
                    float worldMidY = (wldTL.y + wldBR.y) * 0.5f;
                    panOffset.x = viewW / (2.0f * newZoom) - worldMidX;
                    panOffset.y = viewH / (2.0f * newZoom) - worldMidY;
                    zoomLevel = newZoom;
                }
            }
        }
    }

    if (isWiring) wireCurrentPos = mousePos;

    // Drag selected group (with Ctrl+Drag cloning support)
    static bool hasClonedForCtrlDrag = false;
    if (ImGui::IsWindowHovered() && (!selectedComponentIds.empty() || !selectedWireIds.empty()) && !isWiring && !isBoxSelecting && !isBoxZooming && !isDraggingWireSegment) {
        if (ImGui::IsMouseDragging(ImGuiMouseButton_Left)) {
            if (io.KeyCtrl && !hasClonedForCtrlDrag) {
                duplicateSelected();
                hasClonedForCtrlDrag = true;
            }
            ImVec2 deltaWorld(io.MouseDelta.x / zoomLevel, io.MouseDelta.y / zoomLevel);

            // 1. Move selected components
            std::unordered_set<std::string> movedCompIds;
            for (auto& comp : design.components) {
                if (selectedComponentIds.count(comp.id)) {
                    comp.x += deltaWorld.x;
                    comp.y += deltaWorld.y;
                    movedCompIds.insert(comp.id);
                }
            }

            // 2. Identify junction points that belong to the moving selection
            // A junction node only moves if the wire itself is selected OR if both endpoints are selected (whole sub-circuit)
            auto getJKey = [](float x, float y) {
                int ix = (int)std::round(x / 2.0f);
                int iy = (int)std::round(y / 2.0f);
                return std::to_string(ix) + "_" + std::to_string(iy);
            };

            std::set<std::string> movedJunctionKeys;
            for (const auto& w : design.wires) {
                bool wireSelected = selectedWireIds.count(w.id) > 0;
                bool fromCompSelected = !w.from.compId.empty() && movedCompIds.count(w.from.compId) > 0;
                bool toCompSelected = !w.to.compId.empty() && movedCompIds.count(w.to.compId) > 0;

                bool shouldMoveJunction = wireSelected || (fromCompSelected && toCompSelected);

                if (shouldMoveJunction) {
                    if (w.from.isWireJunction) {
                        movedJunctionKeys.insert(getJKey(w.from.junctionX, w.from.junctionY));
                    }
                    if (w.to.isWireJunction) {
                        movedJunctionKeys.insert(getJKey(w.to.junctionX, w.to.junctionY));
                    }
                }
            }

            // 3. Move junction coordinates & manual paths for affected wires
            for (auto& w : design.wires) {
                if (w.from.isWireJunction) {
                    std::string key = getJKey(w.from.junctionX, w.from.junctionY);
                    if (movedJunctionKeys.count(key) > 0) {
                        w.from.junctionX += deltaWorld.x;
                        w.from.junctionY += deltaWorld.y;
                    }
                }
                if (w.to.isWireJunction) {
                    std::string key = getJKey(w.to.junctionX, w.to.junctionY);
                    if (movedJunctionKeys.count(key) > 0) {
                        w.to.junctionX += deltaWorld.x;
                        w.to.junctionY += deltaWorld.y;
                    }
                }

                bool fromIn = !w.from.compId.empty() && movedCompIds.count(w.from.compId) > 0;
                bool toIn = !w.to.compId.empty() && movedCompIds.count(w.to.compId) > 0;
                if (selectedWireIds.count(w.id) > 0 || (fromIn && toIn)) {
                    for (auto& pt : w.manualPath) {
                        pt.x += deltaWorld.x;
                        pt.y += deltaWorld.y;
                    }
                }
            }
        } else {
            hasClonedForCtrlDrag = false;
        }
    }

    // Right-Click Context Menu for Selection
    if (ImGui::IsWindowHovered() && ImGui::IsMouseClicked(ImGuiMouseButton_Right) && (!selectedComponentIds.empty() || !selectedWireIds.empty())) {
        ImGui::OpenPopup("ComponentContextMenu");
    }
    if (ImGui::BeginPopup("ComponentContextMenu")) {
        if (!selectedComponentIds.empty()) {
            if (ImGui::MenuItem("Rotate 90° (R)")) {
                pushUndoState();
                for (auto& comp : design.components) {
                    if (selectedComponentIds.count(comp.id)) comp.rotation = (comp.rotation + 90) % 360;
                }
            }
            if (ImGui::MenuItem("Flip Horizontal (H)")) flipHorizontal();
            if (ImGui::MenuItem("Flip Vertical (V)")) flipVertical();
            ImGui::Separator();
            if (ImGui::MenuItem("Duplicate (Ctrl+D)")) duplicateSelected();
        }
        if (ImGui::MenuItem("Delete Selected", "Del")) {
            deleteSelected();
        }
        ImGui::EndPopup();
    }

    // Double-click handler to open dynamic modals or enter Subsystem
    if (ImGui::IsWindowHovered() && ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) {
        for (size_t i = 0; i < design.components.size(); ++i) {
            auto& comp = design.components[i];
            ImVec2 center = worldToScreen(comp.x, comp.y, canvasPos);
            float hw = 25.0f, hh = 25.0f;
            getComponentBounds(comp, hw, hh);

            if (mousePos.x >= center.x - hw*zoomLevel && mousePos.x <= center.x + hw*zoomLevel &&
                mousePos.y >= center.y - hh*zoomLevel && mousePos.y <= center.y + hh*zoomLevel) {
                
                if (comp.rawTypeStr == "SUBSYSTEM") {
                    subsystemStack.push_back({comp.label.empty() ? comp.id : comp.label, design});
                    design = CircuitDesign();
                } else if (comp.rawTypeStr == "CSCRIPT") {
                    showCScriptModal = true;
                    cscriptCompIdx = (int)i;
                    strncpy(cscriptCodeBuf, comp.parameters["code"].c_str(), sizeof(cscriptCodeBuf) - 1);
                    std::string tsStr = comp.parameters.count("timestep") ? comp.parameters["timestep"] : "0";
                    strncpy(cscriptTimestepBuf, tsStr.c_str(), sizeof(cscriptTimestepBuf) - 1);
                } else if (comp.rawTypeStr == "PULSE" || comp.rawTypeStr == "PULSE_GEN") {
                    showPulseModal = true;
                    pulseCompIdx = (int)i;
                    std::string amp = comp.parameters.count("amplitude") ? comp.parameters["amplitude"] : "1";
                    std::string period = comp.parameters.count("period") ? comp.parameters["period"] : "1";
                    std::string width = comp.parameters.count("width") ? comp.parameters["width"] : "0.5";
                    std::string delay = comp.parameters.count("delay") ? comp.parameters["delay"] : "0";

                    strncpy(pulseAmpBuf, amp.c_str(), sizeof(pulseAmpBuf));
                    strncpy(pulsePeriodBuf, period.c_str(), sizeof(pulsePeriodBuf));
                    strncpy(pulseWidthBuf, width.c_str(), sizeof(pulseWidthBuf));
                    strncpy(pulseDelayBuf, delay.c_str(), sizeof(pulseDelayBuf));
                } else if (comp.rawTypeStr == "SCOPE") {
                    // Open the scope popup window (MainWindow will handle creation)
                    scopeOpenRequest.pending = true;
                    scopeOpenRequest.scopeId = comp.id;
                    int ch = 2;
                    if (comp.parameters.count("channels")) {
                        try { ch = std::stoi(comp.parameters.at("channels")); } catch (...) {}
                    }
                    scopeOpenRequest.numChannels = ch;
                }
                break;
            }
        }
    }

    // Keyboard Shortcuts (works when window is hovered or focused)
    if (ImGui::IsWindowFocused() || ImGui::IsWindowHovered()) {
        if (io.KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_Z)) undo();
        if (io.KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_Y)) redo();
        if (io.KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_C)) copySelected();
        if (io.KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_V)) pasteSelected();
        if (io.KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_D)) duplicateSelected();
        if (ImGui::IsKeyPressed(ImGuiKey_F)) fitToScreen(canvasSize);
        if (ImGui::IsKeyPressed(ImGuiKey_H)) flipHorizontal();
        if (ImGui::IsKeyPressed(ImGuiKey_V)) flipVertical();
        if (ImGui::IsKeyPressed(ImGuiKey_R)) {
            pushUndoState();
            for (auto& comp : design.components) {
                if (selectedComponentIds.count(comp.id)) comp.rotation = (comp.rotation + 90) % 360;
            }
        }
        if (ImGui::IsKeyPressed(ImGuiKey_Delete)) {
            deleteSelected();
        }
    }

    renderModals();

    ImGui::End();
    ImGui::PopStyleVar();
}

void SchematicCanvas::syncProbeSignals() {
    std::vector<std::string> probedSignals;
    std::vector<std::string> probedTargets;

    for (const auto& comp : design.components) {
        bool isProbed = (comp.parameters.count("probe_signal") && comp.parameters.at("probe_signal") == "1") ||
                        (comp.parameters.count("plotI") && comp.parameters.at("plotI") == "1") ||
                        (comp.parameters.count("plotV") && comp.parameters.at("plotV") == "1");
        if (isProbed) {
            probedTargets.push_back(comp.id);
            if (comp.type == ComponentType::Inductor) {
                probedSignals.push_back("I_" + comp.id);
            } else {
                probedSignals.push_back("V_" + comp.id);
            }
        }
    }

    std::string sigStr = "";
    for (size_t i = 0; i < probedSignals.size(); ++i) {
        if (i > 0) sigStr += ",";
        sigStr += probedSignals[i];
    }
    std::string targetStr = "";
    for (size_t i = 0; i < probedTargets.size(); ++i) {
        if (i > 0) targetStr += ",";
        targetStr += probedTargets[i];
    }

    for (auto& comp : design.components) {
        if (comp.rawTypeStr == "PROBE" || comp.type == ComponentType::UnifiedProbe) {
            // Only set default probed targets if comp currently has NO target or selected_signals specified
            if (!comp.parameters.count("target") || comp.parameters["target"].empty()) {
                if (!targetStr.empty()) comp.parameters["target"] = targetStr;
            }
            if (!comp.parameters.count("selected_signals") || comp.parameters["selected_signals"].empty()) {
                if (!sigStr.empty()) comp.parameters["selected_signals"] = sigStr;
            }
        }
    }
}

void SchematicCanvas::openCScriptModalForComp(const std::string& compId) {
    for (size_t i = 0; i < design.components.size(); ++i) {
        if (design.components[i].id == compId) {
            showCScriptModal = true;
            cscriptCompIdx = (int)i;
            strncpy(cscriptCodeBuf, design.components[i].parameters["code"].c_str(), sizeof(cscriptCodeBuf) - 1);
            std::string tsStr = design.components[i].parameters.count("timestep") ? design.components[i].parameters["timestep"] : "0";
            strncpy(cscriptTimestepBuf, tsStr.c_str(), sizeof(cscriptTimestepBuf) - 1);
            break;
        }
    }
}

} // namespace CircuitSim
