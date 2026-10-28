#include "CircuitSimulator.hpp"
#include <cmath>
#include <cctype>
#include <cstdlib>
#include <cstdio>
#include <iostream>
#include <algorithm>
#include <sstream>
#include <random>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

namespace CircuitSimEngine {

// Lower-cases an ASCII string. Used at setup time to pre-normalise function/mode
// selector strings so the per-timestep evaluation can compare them directly instead
// of copying and transforming on every step.
static std::string toLowerAscii(std::string s) {
    for (char& c : s) c = (char)::tolower((unsigned char)c);
    return s;
}

// Normalized trapezoidal back-EMF shape of a BLDC phase, theta in radians (any value). The shape
// is +1 over 120 deg, -1 over the next 120 deg, and changes linearly over 60 deg at each edge.
static double trapezoidShape(double theta) {
    const double twoPi = 6.283185307179586;
    double d = std::fmod(theta, twoPi);
    if (d < 0.0) d += twoPi;                  // [0, 2 pi)
    double deg = d * 180.0 / 3.141592653589793;
    if (deg >= 180.0) deg -= 360.0;           // [-180, 180)
    if (deg < -150.0) deg += 360.0;           // falling edge of the negative half, now (150, 210)
    if (deg > 150.0) return 1.0 - (deg - 150.0) / 30.0;
    if (deg > 30.0) return 1.0;
    if (deg >= -30.0) return deg / 30.0;      // rising edge
    return -1.0;
}

// Effective value of a passive element for the current timestep. Signal-controlled
// elements (VAR_R / VAR_L / VAR_C) read their value from the bound control signal,
// floored to keep the MNA stamp well conditioned. Elements with no control signal
// wired up fall back to their nominal parameter value.
static inline double liveElementValue(const FastCompiledComponent& fc, double floorVal) {
    if (!fc.isVariable || fc.ctrlSigPtr == nullptr) return fc.val;
    double v = *fc.ctrlSigPtr;
    if (!(v == v)) return fc.nominalVal;      // NaN guard
    return (v < floorVal) ? floorVal : v;
}

// PWM_3PH output ports, in registration order: G1..G6, gA1,gA2,gB1,gB2,gC1,gC2,
// OutA,OutB,OutC, Out1,Out2,Out3. Upper gates follow the comparison, lower gates
// are its complement (no dead time in this block).
// Dead-time gating for PWM_3PH. raw[k] is the requested state of gate k (order:
// A-upper, A-lower, B-upper, B-lower, C-upper, C-lower). A gate may switch on only
// deadTime after its own rising edge, so the two switches of a leg never conduct
// together. Turn-off is immediate. The rising-edge times live in the component's
// PWM master state arrays, updated once per accepted step (commit).
// Parses "[[a, b], [c, d]]", "[a, b; c, d]" or "[a b]" into rows of numbers.
static std::vector<std::vector<double>> parseNumRows(const std::string& src) {
    std::string s;
    s.reserve(src.size());
    int depth = 0;
    for (char c : src) {
        if (c == '[') {
            ++depth;
            if (depth == 2 && !s.empty()) s += ';';
        } else if (c == ']') {
            --depth;
        } else if (c == ',') {
            s += ' ';
        } else {
            s += c;
        }
    }
    std::vector<std::vector<double>> rows;
    std::stringstream ss(s);
    std::string line;
    while (std::getline(ss, line, ';')) {
        std::stringstream ls(line);
        std::vector<double> row;
        double v;
        while (ls >> v) row.push_back(v);
        if (!row.empty()) rows.push_back(row);
    }
    return rows;
}

static void pwm3PhGates(FastCompiledComponent& fc, double t, double deadTime, bool commit,
                        const double raw[6], double gated[6]) {
    if (fc.pwmMasterLastTargetDirect.size() < 6) {
        fc.pwmMasterLastTargetDirect.assign(6, 0);
        fc.pwmMasterLastTransDirect.assign(6, -1.0);
    }
    if (commit && t > fc.lastTime) {
        for (int k = 0; k < 6; ++k) {
            const int r = raw[k] > 0.5 ? 1 : 0;
            if (r != fc.pwmMasterLastTargetDirect[k]) {
                fc.pwmMasterLastTargetDirect[k] = r;
                if (r) fc.pwmMasterLastTransDirect[k] = t;   // rising edge
            }
        }
        fc.lastTime = t;
    }
    for (int k = 0; k < 6; ++k) {
        const bool on = raw[k] > 0.5;
        const double since = (fc.pwmMasterLastTransDirect[k] >= 0.0) ? (t - fc.pwmMasterLastTransDirect[k]) : 1e300;
        const bool blocked = on && deadTime > 0.0 && since < deadTime - 1e-15;
        gated[k] = (on && !blocked) ? 1.0 : 0.0;
    }
}

// Writes the 18 PWM_3PH output ports, in registration order: G1..G6, gA1..gC2,
// OutA..OutC, Out1..Out3. Upper and lower gates are the gated values.
static inline void writePwm3PhGates(const FastCompiledComponent& fc, std::vector<double>& sigs,
                                    const double g[6]) {
    const double vals[18] = {g[0], g[1], g[2], g[3], g[4], g[5],
                             g[0], g[1], g[2], g[3], g[4], g[5],
                             g[0], g[2], g[4], g[0], g[2], g[4]};
    for (size_t k = 0; k < 18 && k < fc.outputSigIndices.size(); ++k) {
        int idx = fc.outputSigIndices[k];
        if (idx >= 0 && idx < (int)sigs.size()) sigs[idx] = vals[k];
    }
}

// Control blocks whose outputs are recorded one vector per port. A CSCRIPT with a
// single output belongs here too: it used to fall through to the "<id>.Out" path,
// a key the script never writes, so its only output read 0 in the results.
static inline bool hasPerPortOutputs(const FastCompiledComponent& fc) {
    if (fc.type == ComponentType::CustomScript) return !fc.outputSigKeys.empty();
    return fc.outputSigKeys.size() > 1;
}

void CircuitSimulator::setup(const std::vector<ComponentModel>& physComps, 
                            const std::vector<ComponentModel>& ctrlComps, 
                            const SimulationConfig& simCfg) {
    components = physComps;
    controlBlocks = ctrlComps;
    config = simCfg;

    buildIndexMaps();
}

double CircuitSimulator::evaluateParam(const ComponentModel& comp, const std::string& key, double defaultVal) {
    auto it = comp.parameters.find(key);
    if (it != comp.parameters.end() && !it->second.empty()) {
        std::string s = it->second;
        s.erase(std::remove(s.begin(), s.end(), '['), s.end());
        s.erase(std::remove(s.begin(), s.end(), ']'), s.end());
        return ExpressionEvaluator::parseScientific(s);
    }
    return defaultVal;
}

static std::string getParamString(const ComponentModel& comp, const std::string& key, const std::string& defaultStr = "") {
    auto it = comp.parameters.find(key);
    if (it != comp.parameters.end()) return it->second;
    return defaultStr;
}

static bool isTransformerType(ComponentType t) {
    return t == ComponentType::Transformer ||
           t == ComponentType::IdealTransformer ||
           t == ComponentType::Transformer2W ||
           t == ComponentType::Transformer3W ||
           t == ComponentType::MutualInductor2W ||
           t == ComponentType::MutualInductor3W ||
           t == ComponentType::SaturableTransformer ||
           t == ComponentType::Transformer3Ph2W ||
           t == ComponentType::Transformer3Ph3W;
}

static std::vector<double> parseTurnsVector(const std::string& str, double defaultVal = 100.0) {
    if (str.empty()) return { defaultVal };
    std::string clean = str;
    for (char& c : clean) {
        if (c == '[' || c == ']' || c == ';' || c == ',') c = ' ';
    }
    std::stringstream ss(clean);
    std::vector<double> res;
    double val;
    while (ss >> val) {
        if (val > 0) res.push_back(val);
    }
    if (res.empty()) res.push_back(defaultVal);
    return res;
}

void CircuitSimulator::buildIndexMaps() {
    nodeToIdx.clear();
    vSourceToIdx.clear();
    inductorToIdx.clear();
    capVoltagesPrev.clear();
    indCurrentsPrev.clear();
    diodeStatePrev.clear();
    switchStatePrev.clear();
    piIntegratorState.clear();
    controlSignalsCurrent.clear();
    cscriptEngines.clear();

    signalKeyToIdx.clear();
    flatControlSignals.clear();
    flatCapVoltages.clear();
    flatCapCurrents.clear();
    flatIndCurrents.clear();
    flatDiodeStates.clear();
    flatSwitchStates.clear();
    flatPiIntegratorState.clear();

    fastPhysComps.clear();
    fastCtrlComps.clear();

    auto getOrCreateSignalIdx = [&](const std::string& key) -> int {
        if (key.empty()) return -1;
        auto it = signalKeyToIdx.find(key);
        if (it != signalKeyToIdx.end()) return it->second;
        int idx = (int)flatControlSignals.size();
        flatControlSignals.push_back(0.0);
        signalKeyToIdx[key] = idx;
        return idx;
    };

    int nIdx = 1; // Node "0" or GND is index 0
    int numXfmrWindings = 0;

    for (const auto& comp : components) {
        for (const auto& n : comp.nodes) {
            if (n != "0" && n != "node_0" && !n.empty() && nodeToIdx.find(n) == nodeToIdx.end()) {
                nodeToIdx[n] = nIdx++;
            }
        }

        if (comp.type == ComponentType::VoltageSource || 
            comp.type == ComponentType::ACVoltageSource || 
            comp.type == ComponentType::ControlledVoltageSource ||
            comp.type == ComponentType::Ammeter) {
            if (vSourceToIdx.find(comp.id) == vSourceToIdx.end()) {
                vSourceToIdx[comp.id] = (int)vSourceToIdx.size();
            }
        } else if (comp.type == ComponentType::Inductor) {
            if (inductorToIdx.find(comp.id) == inductorToIdx.end()) {
                double i0 = evaluateParam(comp, "iL0", 0.0);
                inductorToIdx[comp.id] = (int)inductorToIdx.size();
                indCurrentsPrev[comp.id] = i0;
            }
        } else if (comp.type == ComponentType::Capacitor) {
            double v0 = evaluateParam(comp, "vC0", 0.0);
            capVoltagesPrev[comp.id] = v0;
        } else if (comp.type == ComponentType::Diode || comp.type == ComponentType::Thyristor || comp.type == ComponentType::MOSFET ||
                   comp.type == ComponentType::IGBT || comp.type == ComponentType::GTO ||
                   comp.type == ComponentType::IGCT || comp.type == ComponentType::IGBTDiode ||
                   comp.type == ComponentType::BJT || comp.type == ComponentType::JFET) {
            diodeStatePrev[comp.id] = 0.0; // Initially OFF
        } else if (comp.type == ComponentType::Switch) {
            switchStatePrev[comp.id] = 0.0;
        } else if (isTransformerType(comp.type)) {
            int wCount = (int)comp.nodes.size() / 2;
            if (wCount < 2) wCount = 2;
            numXfmrWindings += wCount;
        } else if (comp.type == ComponentType::Winding) {
            // Gyrator needs two branch unknowns: the electrical port current and the
            // magnetic port flux rate. They share the transformer extra-unknown pool.
            numXfmrWindings += 2;
        }
    }

    numNodes = nIdx - 1;
    totalDim = numNodes + (int)vSourceToIdx.size() + (int)inductorToIdx.size() + numXfmrWindings;

    K.assign(totalDim * totalDim, 0.0);
    K_static.assign(totalDim * totalDim, 0.0);
    // Dynamic stamp position tracking starts empty for each new circuit.
    dynStampIdx.clear();
    dynStampSeen.assign((size_t)totalDim * (size_t)totalDim, 0);
    dynStampBaseCopied = false;
    // Reset stale change-detection state so the first assembly always factorizes.
    matrixKChanged = true;
    trapModeStampValid = false;
    // Do not carry a pending backward-Euler damping window into the next run, but
    // start each run with one: the trapezoidal history terms (capacitor current,
    // inductor voltage) start at zero, which is wrong whenever the circuit is not at
    // rest at t = 0, and trapezoidal never damps that error away.
    forceBackwardEulerSteps = 2;
    B.assign(totalDim, 0.0);
    X.assign(totalDim, 0.0);

    LU_cached.assign(totalDim * totalDim, 0.0);
    x_buf.assign(totalDim, 0.0);
    p_cached.assign(totalDim, 0);

    // Size the factorization cache from the matrix dimension so the memory it can
    // occupy stays bounded regardless of circuit size. Entries are reserved up
    // front: LU_active points into this vector, so it must never reallocate.
    luCache.clear();
    luCacheClock = 0;
    luFactorizeCount = 0;
    luCacheHitCount = 0;
    luWindowLookups = 0;
    luWindowHits = 0;
    luCacheSuspended = false;
    luSuspendCountdown = 0;
    LU_active = nullptr;
    p_active = nullptr;
    sparse_active = nullptr;
    if (totalDim > 0) {
        const size_t nn = (size_t)totalDim * (size_t)totalDim;
        // K + LU + perm, plus the compressed triangular form. The latter is bounded
        // by the dense triangles, so budget for the worst case rather than hope.
        const size_t bytesPerEntry = nn * sizeof(double) * 2 + (size_t)totalDim * sizeof(int)
                                   + nn * (sizeof(double) + sizeof(int));
        constexpr size_t kBudgetBytes = 48ull << 20;   // 48 MB ceiling for the cache
        size_t maxEntries = kBudgetBytes / (bytesPerEntry ? bytesPerEntry : 1);
        if (maxEntries < 2) maxEntries = 2;    // always worth holding the hot pair
        if (maxEntries > 64) maxEntries = 64;  // beyond this the linear probe costs more than it saves
        luCacheMaxEntries = maxEntries;
        luCache.reserve(luCacheMaxEntries);
    } else {
        luCacheMaxEntries = 0;
    }

    scriptInValsBuf.assign(20, 0.0);
    matrixKChanged = true;

    int currentWindingOffset = 0;

    // Compile physical components into fast primitive structures
    for (const auto& comp : components) {
        FastCompiledComponent fc;
        fc.id = comp.id;
        fc.type = comp.type;

        fc.n1 = (comp.nodes.size() > 0 && nodeToIdx.count(comp.nodes[0])) ? nodeToIdx[comp.nodes[0]] - 1 : -1;
        fc.n2 = (comp.nodes.size() > 1 && nodeToIdx.count(comp.nodes[1])) ? nodeToIdx[comp.nodes[1]] - 1 : -1;
        fc.n3 = (comp.nodes.size() > 2 && nodeToIdx.count(comp.nodes[2])) ? nodeToIdx[comp.nodes[2]] - 1 : -1;
        fc.n4 = (comp.nodes.size() > 3 && nodeToIdx.count(comp.nodes[3])) ? nodeToIdx[comp.nodes[3]] - 1 : -1;

        if (vSourceToIdx.count(comp.id)) fc.vIdx = numNodes + vSourceToIdx[comp.id];
        if (inductorToIdx.count(comp.id)) fc.lIdx = numNodes + (int)vSourceToIdx.size() + inductorToIdx[comp.id];

        if (isTransformerType(comp.type)) {
            std::string pStr = getParamString(comp, "primary_turns", "[100]");
            std::string sStr = getParamString(comp, "secondary_turns", "[100]");
            auto pTurns = parseTurnsVector(pStr, 100.0);
            auto sTurns = parseTurnsVector(sStr, 100.0);

            std::vector<double> allTurns;
            for (double t : pTurns) allTurns.push_back(t);
            for (double t : sTurns) allTurns.push_back(t);

            int wCount = (int)comp.nodes.size() / 2;
            if (wCount < 2) wCount = 2;
            for (int k = 0; k < wCount; ++k) {
                FastCompiledComponent::WindingInfo wi;
                wi.n1 = (2 * k < (int)comp.nodes.size() && nodeToIdx.count(comp.nodes[2 * k])) ? nodeToIdx[comp.nodes[2 * k]] - 1 : -1;
                wi.n2 = (2 * k + 1 < (int)comp.nodes.size() && nodeToIdx.count(comp.nodes[2 * k + 1])) ? nodeToIdx[comp.nodes[2 * k + 1]] - 1 : -1;
                wi.wIdx = numNodes + (int)vSourceToIdx.size() + (int)inductorToIdx.size() + currentWindingOffset++;
                wi.turns = (k < (int)allTurns.size()) ? allTurns[k] : 100.0;
                fc.windings.push_back(wi);
            }
            if (fc.windings.size() > 0) fc.wIdx0 = fc.windings[0].wIdx;
            if (fc.windings.size() > 1) fc.wIdx1 = fc.windings[1].wIdx;
            fc.polarity = getParamString(comp, "polarity", "");
        }
        else if (comp.type == ComponentType::Winding) {
            // wIdx0 = electrical port current I_e, wIdx1 = magnetic port flux rate PhiDot.
            int base = numNodes + (int)vSourceToIdx.size() + (int)inductorToIdx.size();
            fc.wIdx0 = base + currentWindingOffset++;
            fc.wIdx1 = base + currentWindingOffset++;
        }

        fc.val = evaluateParam(comp, "val", 0.0);
        if (fc.val == 0.0 && comp.parameters.count("value")) fc.val = evaluateParam(comp, "value", 0.0);
        if (fc.val == 0.0 && comp.parameters.count("v")) fc.val = evaluateParam(comp, "v", 0.0);
        if (fc.val == 0.0 && comp.parameters.count("V")) fc.val = evaluateParam(comp, "V", 0.0);
        if (fc.val == 0.0 && comp.parameters.count("amplitude")) fc.val = evaluateParam(comp, "amplitude", 0.0);
        // Web-tool alias for resistance
        if (fc.val == 0.0 && comp.parameters.count("resistance")) fc.val = evaluateParam(comp, "resistance", 0.0);
        if (comp.type == ComponentType::Winding) {
            // Number of turns acts as the gyration resistance.
            fc.val = evaluateParam(comp, "N", 0.0);
            if (fc.val == 0.0) fc.val = evaluateParam(comp, "turns", 0.0);
            if (fc.val == 0.0) fc.val = evaluateParam(comp, "value", 1.0);
            if (std::abs(fc.val) < 1e-12) fc.val = 1.0; // a zero-turn winding is meaningless
        }
        if (fc.type == ComponentType::Capacitor) {
            fc.val = evaluateParam(comp, "C", 1e-6);
            fc.stateIdx = (int)flatCapVoltages.size();
            flatCapVoltages.push_back(capVoltagesPrev[comp.id]);
            flatCapCurrents.push_back(0.0);
        }
        else if (comp.type == ComponentType::Inductor) {
            fc.val = evaluateParam(comp, "L", 1e-3);
            fc.stateIdx = (int)flatIndCurrents.size();
            flatIndCurrents.push_back(indCurrentsPrev[comp.id]);
            flatIndVoltages.push_back(0.0);
        }
        else if (comp.type == ComponentType::Diode || comp.type == ComponentType::Thyristor || comp.type == ComponentType::MOSFET ||
                 comp.type == ComponentType::IGBT || comp.type == ComponentType::GTO ||
                 comp.type == ComponentType::IGCT || comp.type == ComponentType::IGBTDiode ||
                 comp.type == ComponentType::BJT || comp.type == ComponentType::JFET) {
            fc.stateIdx = (int)flatDiodeStates.size();
            flatDiodeStates.push_back(0.0);
        }
        else if (comp.type == ComponentType::Switch) {
            fc.stateIdx = (int)flatSwitchStates.size();
            flatSwitchStates.push_back(0.0);
        }

        fc.esr = evaluateParam(comp, "esr", 0.0);
        fc.Ron = evaluateParam(comp, "Ron", 0.01);
        fc.Roff = evaluateParam(comp, "Roff", 1e6);
        fc.Vvd = evaluateParam(comp, "Vd", 0.8);
        fc.Iholding = evaluateParam(comp, "Iholding", 0.01);
        fc.Vgt = evaluateParam(comp, "Vgt", 0.5);
        fc.freq = evaluateParam(comp, "freq", 50.0);
        if (comp.type == ComponentType::ACVoltageSource) {
            // Web-tool netlist uses "amplitude" and "frequency" keys
            if (comp.parameters.count("amplitude")) fc.val = evaluateParam(comp, "amplitude", 1.0);
            if (comp.parameters.count("frequency")) fc.freq = evaluateParam(comp, "frequency", 50.0);
            if (comp.parameters.count("phase")) fc.delay = evaluateParam(comp, "phase", 0.0);
            // Also support "value" as amplitude alias (Windows tool schematic)
            if (!comp.parameters.count("amplitude") && comp.parameters.count("value")) fc.val = evaluateParam(comp, "value", 100.0);
        }
        if (comp.type == ComponentType::ControlledVoltageSource || comp.type == ComponentType::ControlledCurrentSource) {
            fc.gain = evaluateParam(comp, "gain", 1.0);
            if (!comp.parameters.count("gain") && comp.parameters.count("K")) fc.gain = evaluateParam(comp, "K", 1.0);
            if (!comp.parameters.count("gain") && comp.parameters.count("k")) fc.gain = evaluateParam(comp, "k", 1.0);
            if (!comp.parameters.count("gain") && comp.parameters.count("value")) fc.gain = evaluateParam(comp, "value", 1.0);
            if (comp.parameters.count("control_signal")) fc.ctrlSigKey = getParamString(comp, "control_signal", "");
        }

        fc.vPlotKey = "V_" + comp.id;
        fc.iPlotKey = "I_" + comp.id;
        fc.ctrlSigKey = getParamString(comp, "control_signal", "");

        // Signal-controlled passive element (VAR_R / VAR_L / VAR_C). The netlist marks
        // these with src_type == "variable"; the nominal parameter is retained as the
        // value used whenever no control signal is actually wired up.
        if (getParamString(comp, "src_type", "") == "variable" &&
            (fc.type == ComponentType::Resistor ||
             fc.type == ComponentType::Capacitor ||
             fc.type == ComponentType::Inductor)) {
            fc.isVariable = true;

            // getIncomingSignal() yields the literal "0.0" when the Ctrl pin is left
            // unconnected. Treat any bare numeric key as "no signal" so the element
            // keeps its nominal value instead of collapsing to zero.
            const std::string& k = fc.ctrlSigKey;
            bool numericOnly = !k.empty();
            for (char c : k) {
                if (!std::isdigit((unsigned char)c) && c != '.' && c != '-' && c != '+') {
                    numericOnly = false;
                    break;
                }
            }
            if (k.empty() || numericOnly) fc.ctrlSigKey.clear();
        }
        fc.nominalVal = fc.val;

        fc.vPlotSignalIdx = getOrCreateSignalIdx(fc.vPlotKey);
        fc.iPlotSignalIdx = getOrCreateSignalIdx(fc.iPlotKey);
        fc.ctrlSigSignalIdx = getOrCreateSignalIdx(fc.ctrlSigKey);
        fc.outSignalIdx = getOrCreateSignalIdx(comp.id + ".Out");
        fc.compSelfSignalIdx = getOrCreateSignalIdx(comp.id);

        fastPhysComps.push_back(fc);
    }

    // Compile control components into fast primitive structures
    for (const auto& ctrlComp : controlBlocks) {
        FastCompiledComponent fc;
        fc.id = ctrlComp.id;
        fc.type = ctrlComp.type;

        fc.val = evaluateParam(ctrlComp, "value", 1.0);
        if (ctrlComp.parameters.count("constant")) fc.val = evaluateParam(ctrlComp, "constant", 1.0);
        else if (ctrlComp.parameters.count("const")) fc.val = evaluateParam(ctrlComp, "const", 1.0);
        else if (ctrlComp.parameters.count("val")) fc.val = evaluateParam(ctrlComp, "val", 1.0);

        fc.freq = evaluateParam(ctrlComp, "frequency", 10000.0);
        if (ctrlComp.parameters.count("freq")) fc.freq = evaluateParam(ctrlComp, "freq", 10000.0);
        fc.minVal = evaluateParam(ctrlComp, "min", 0.0);
        if (ctrlComp.parameters.count("minVal")) fc.minVal = evaluateParam(ctrlComp, "minVal", 0.0);
        fc.maxVal = evaluateParam(ctrlComp, "max", 1.0);
        if (ctrlComp.parameters.count("maxVal")) fc.maxVal = evaluateParam(ctrlComp, "maxVal", 1.0);

        if (ctrlComp.parameters.count("gain")) fc.gain = evaluateParam(ctrlComp, "gain", 1.0);
        else if (ctrlComp.parameters.count("k")) fc.gain = evaluateParam(ctrlComp, "k", 1.0);
        else if (ctrlComp.parameters.count("K")) fc.gain = evaluateParam(ctrlComp, "K", 1.0);

        if (ctrlComp.parameters.count("Kp")) fc.Kp = evaluateParam(ctrlComp, "Kp", 1.0);
        if (ctrlComp.parameters.count("Ki")) fc.Ki = evaluateParam(ctrlComp, "Ki", 0.0);
        if (ctrlComp.parameters.count("period")) fc.period = evaluateParam(ctrlComp, "period", 0.0001);
        if (ctrlComp.parameters.count("width")) fc.width = evaluateParam(ctrlComp, "width", 0.5);
        if (ctrlComp.parameters.count("delay")) fc.delay = evaluateParam(ctrlComp, "delay", 0.0);
        if (ctrlComp.parameters.count("amplitude")) fc.amplitude = evaluateParam(ctrlComp, "amplitude", 1.0);

        if (ctrlComp.type == ComponentType::Triangle_Carrier) {
            fc.minVal = evaluateParam(ctrlComp, "min", 0.0);
            if (ctrlComp.parameters.count("minVal")) fc.minVal = evaluateParam(ctrlComp, "minVal", fc.minVal);
            if (ctrlComp.parameters.count("v_min")) fc.minVal = evaluateParam(ctrlComp, "v_min", fc.minVal);
            if (ctrlComp.parameters.count("min_val")) fc.minVal = evaluateParam(ctrlComp, "min_val", fc.minVal);

            fc.maxVal = evaluateParam(ctrlComp, "max", 1.0);
            if (ctrlComp.parameters.count("maxVal")) fc.maxVal = evaluateParam(ctrlComp, "maxVal", fc.maxVal);
            if (ctrlComp.parameters.count("v_max")) fc.maxVal = evaluateParam(ctrlComp, "v_max", fc.maxVal);
            if (ctrlComp.parameters.count("max_val")) fc.maxVal = evaluateParam(ctrlComp, "max_val", fc.maxVal);

            if (ctrlComp.parameters.count("frequency")) fc.freq = evaluateParam(ctrlComp, "frequency", 10000.0);
            if (ctrlComp.parameters.count("freq")) fc.freq = evaluateParam(ctrlComp, "freq", fc.freq);
            if (ctrlComp.parameters.count("period")) {
                double p = evaluateParam(ctrlComp, "period", 0.0001);
                if (p > 0.0) fc.freq = 1.0 / p;
            }

            if (ctrlComp.parameters.count("phase")) fc.delay = evaluateParam(ctrlComp, "phase", 0.0);
            if (ctrlComp.parameters.count("phase_deg")) fc.delay = evaluateParam(ctrlComp, "phase_deg", fc.delay);

            if (ctrlComp.parameters.count("amplitude") && !ctrlComp.parameters.count("min") && !ctrlComp.parameters.count("v_min")) {
                double amp = evaluateParam(ctrlComp, "amplitude", 1.0);
                fc.minVal = -amp;
                fc.maxVal = amp;
            }

            fc.polarity = getParamString(ctrlComp, "phase_source", "internal");
            fc.vPlotKey = getParamString(ctrlComp, "freq_source", "internal");
        } else if (ctrlComp.type == ComponentType::Step) {
            fc.delay = evaluateParam(ctrlComp, "step_time", 1.0);
            fc.minVal = evaluateParam(ctrlComp, "initial_value", 0.0);
            fc.maxVal = evaluateParam(ctrlComp, "final_value", 1.0);
        } else if (ctrlComp.type == ComponentType::Ramp) {
            fc.gain = evaluateParam(ctrlComp, "slope", 1.0);
            fc.delay = evaluateParam(ctrlComp, "start_time", 0.0);
            fc.val = evaluateParam(ctrlComp, "initial_output", 0.0);
        } else if (ctrlComp.type == ComponentType::SineWave) {
            fc.amplitude = evaluateParam(ctrlComp, "amplitude", 1.0);
            fc.freq = evaluateParam(ctrlComp, "frequency", 50.0);
            fc.delay = evaluateParam(ctrlComp, "phase", 0.0); // phase degrees
            fc.val = evaluateParam(ctrlComp, "bias", 0.0);
            if (!ctrlComp.parameters.count("bias")) {
                fc.val = evaluateParam(ctrlComp, "offset", 
                         evaluateParam(ctrlComp, "dc_offset", 
                         evaluateParam(ctrlComp, "dc", 0.0)));
            }
        } else if (ctrlComp.type == ComponentType::RandomNumbers) {
            fc.val = evaluateParam(ctrlComp, "mean", 0.0);
            fc.gain = evaluateParam(ctrlComp, "std", 1.0);
        } else if (ctrlComp.type == ComponentType::WhiteNoise) {
            fc.val = evaluateParam(ctrlComp, "psd", 0.1);
        } else if (ctrlComp.type == ComponentType::InitialCondition) {
            fc.val = evaluateParam(ctrlComp, "initial_value", 0.0);
            if (ctrlComp.parameters.count("x0")) fc.val = evaluateParam(ctrlComp, "x0", 0.0);
        } else if (ctrlComp.type == ComponentType::TrigFunction) {
            // Normalised to lower case here so evaluateControls() does not have to copy
            // and transform the string on every timestep.
            fc.polarity = toLowerAscii(getParamString(ctrlComp, "function", "sin"));
        } else if (ctrlComp.type == ComponentType::Round) {
            // NOTE: deliberately NOT normalised. The live Round branch compares
            // fc.polarity case-sensitively, so lower-casing here would silently change
            // results for modes written in upper case (see report: latent Round bug).
            fc.polarity = getParamString(ctrlComp, "mode", "nearest");
        } else if (ctrlComp.type == ComponentType::MinMax) {
            std::string funcStr = getParamString(ctrlComp, "function", "min");
            if (funcStr.empty()) funcStr = getParamString(ctrlComp, "func", "min");
            if (funcStr.empty()) funcStr = getParamString(ctrlComp, "mode", "min");
            fc.polarity = funcStr;

            std::string nStr = getParamString(ctrlComp, "num_inputs", "");
            if (nStr.empty()) nStr = getParamString(ctrlComp, "inputs", "");
            if (nStr.empty()) nStr = getParamString(ctrlComp, "number_of_inputs", "");

            int nPins = 2;
            if (!nStr.empty()) {
                try { nPins = std::clamp(std::stoi(nStr), 1, 32); } catch (...) { nPins = 2; }
            }

            fc.inputSigKeys.clear();
            fc.inputSigIndices.clear();
            for (int i = 0; i < nPins; ++i) {
                std::string inK = getParamString(ctrlComp, "In" + std::to_string(i + 1), "");
                if (inK.empty()) inK = getParamString(ctrlComp, "input_" + std::to_string(i), "");
                if (inK.empty() && i == 0) inK = getParamString(ctrlComp, "In", "");
                fc.inputSigKeys.push_back(inK);
                fc.inputSigIndices.push_back(getOrCreateSignalIdx(inK));
            }
        } else if (ctrlComp.type == ComponentType::PWM_Generator) {
            fc.freq = evaluateParam(ctrlComp, "carrier_freq", 10000.0);
            if (!ctrlComp.parameters.count("carrier_freq") && ctrlComp.parameters.count("frequency")) fc.freq = evaluateParam(ctrlComp, "frequency", 10000.0);
            if (!ctrlComp.parameters.count("carrier_freq") && !ctrlComp.parameters.count("frequency") && ctrlComp.parameters.count("fc")) fc.freq = evaluateParam(ctrlComp, "fc", 10000.0);
            if (!ctrlComp.parameters.count("carrier_freq") && !ctrlComp.parameters.count("frequency") && !ctrlComp.parameters.count("fc") && ctrlComp.parameters.count("freq")) fc.freq = evaluateParam(ctrlComp, "freq", 10000.0);

            fc.minVal = evaluateParam(ctrlComp, "min_val", 0.0);
            if (!ctrlComp.parameters.count("min_val") && ctrlComp.parameters.count("min")) fc.minVal = evaluateParam(ctrlComp, "min", 0.0);

            fc.maxVal = evaluateParam(ctrlComp, "max_val", 1.0);
            if (!ctrlComp.parameters.count("max_val") && ctrlComp.parameters.count("max")) fc.maxVal = evaluateParam(ctrlComp, "max", 1.0);

            fc.delayDuration = evaluateParam(ctrlComp, "dead_time", 0.0);
            if (!ctrlComp.parameters.count("dead_time") && ctrlComp.parameters.count("deadtime")) fc.delayDuration = evaluateParam(ctrlComp, "deadtime", 0.0);
        } else if (ctrlComp.type == ComponentType::SVPWM || ctrlComp.type == ComponentType::PWM_3PH) {
            fc.freq = evaluateParam(ctrlComp, "carrier_freq", 10000.0);
            if (!ctrlComp.parameters.count("carrier_freq") && ctrlComp.parameters.count("frequency")) fc.freq = evaluateParam(ctrlComp, "frequency", 10000.0);
            if (!ctrlComp.parameters.count("carrier_freq") && !ctrlComp.parameters.count("frequency") && ctrlComp.parameters.count("fc")) fc.freq = evaluateParam(ctrlComp, "fc", 10000.0);

            fc.delayDuration = evaluateParam(ctrlComp, "dead_time", 1e-6);
            if (!ctrlComp.parameters.count("dead_time") && ctrlComp.parameters.count("deadtime")) fc.delayDuration = evaluateParam(ctrlComp, "deadtime", 1e-6);
            if (!ctrlComp.parameters.count("dead_time") && !ctrlComp.parameters.count("deadtime") && ctrlComp.parameters.count("dt")) fc.delayDuration = evaluateParam(ctrlComp, "dt", 1e-6);

            fc.minVal = evaluateParam(ctrlComp, "min", -1.0);
            fc.maxVal = evaluateParam(ctrlComp, "max", 1.0);
        } else if (ctrlComp.type == ComponentType::PWM_MASTER) {
            int numCarriers = (int)evaluateParam(ctrlComp, "num_carriers", 3.0);
            if (!ctrlComp.parameters.count("num_carriers") && ctrlComp.parameters.count("N")) {
                numCarriers = (int)evaluateParam(ctrlComp, "N", 3.0);
            }
            if (numCarriers < 1) numCarriers = 1;

            fc.freq = evaluateParam(ctrlComp, "fc", 10000.0);
            if (!ctrlComp.parameters.count("fc") && ctrlComp.parameters.count("carrier_freq")) fc.freq = evaluateParam(ctrlComp, "carrier_freq", 10000.0);
            if (!ctrlComp.parameters.count("fc") && !ctrlComp.parameters.count("carrier_freq") && ctrlComp.parameters.count("frequency")) fc.freq = evaluateParam(ctrlComp, "frequency", 10000.0);
            if (!ctrlComp.parameters.count("fc") && !ctrlComp.parameters.count("carrier_freq") && !ctrlComp.parameters.count("frequency") && ctrlComp.parameters.count("freq")) fc.freq = evaluateParam(ctrlComp, "freq", 10000.0);

            fc.delayDuration = evaluateParam(ctrlComp, "dead_time", 1e-6);
            if (!ctrlComp.parameters.count("dead_time") && ctrlComp.parameters.count("deadtime")) fc.delayDuration = evaluateParam(ctrlComp, "deadtime", 1e-6);
            if (!ctrlComp.parameters.count("dead_time") && !ctrlComp.parameters.count("deadtime") && ctrlComp.parameters.count("dead_Time")) fc.delayDuration = evaluateParam(ctrlComp, "dead_Time", 1e-6);

            bool isCommonMod = (getParamString(ctrlComp, "common_modulation", "false") == "true");

            fc.numInputs = numCarriers;
            // Carrier range: the triangle runs from min to max, as in PWM_Generator.
            fc.minVal = evaluateParam(ctrlComp, "min", 0.0);
            fc.maxVal = evaluateParam(ctrlComp, "max", 1.0);
            fc.pwmMasterInIndices.assign(numCarriers, -1);
            fc.pwmMasterExtPhaseIndices.assign(numCarriers, -1);
            fc.pwmMasterOutDirectIndices.assign(numCarriers, -1);
            fc.pwmMasterOutComplIndices.assign(numCarriers, -1);
            fc.pwmMasterPhaseDeg.assign(numCarriers, 0.0);
            fc.pwmMasterLevelOffset.assign(numCarriers, 0.0);
            fc.pwmMasterPhaseExt.assign(numCarriers, false);
            fc.pwmMasterLastTargetDirect.assign(numCarriers, 0);
            fc.pwmMasterLastTargetCompl.assign(numCarriers, 0);
            fc.pwmMasterLastTransDirect.assign(numCarriers, 0.0);
            fc.pwmMasterLastTransCompl.assign(numCarriers, 0.0);
            fc.pwmMasterDirectOut.assign(numCarriers, 0.0);
            fc.pwmMasterComplOut.assign(numCarriers, 0.0);

            std::string configStr = getParamString(ctrlComp, "config", "[]");
            if (!configStr.empty() && configStr != "[]") {
                try {
                    auto cfgJson = json::parse(configStr);
                    if (cfgJson.is_array()) {
                        for (const auto& cItem : cfgJson) {
                            if (cItem.is_object() && cItem.contains("id")) {
                                int cId = 0;
                                if (cItem["id"].is_number()) cId = cItem["id"].template get<int>();
                                else if (cItem["id"].is_string()) cId = std::stoi(cItem["id"].template get<std::string>());
                                if (cId >= 1 && cId <= numCarriers) {
                                    int idx = cId - 1;
                                    if (cItem.contains("phase_source") && cItem["phase_source"] == "external") {
                                        fc.pwmMasterPhaseExt[idx] = true;
                                    }
                                    if (cItem.contains("phase")) {
                                        if (cItem["phase"].is_string()) fc.pwmMasterPhaseDeg[idx] = std::stod(cItem["phase"].template get<std::string>());
                                        else if (cItem["phase"].is_number()) fc.pwmMasterPhaseDeg[idx] = cItem["phase"].template get<double>();
                                    }
                                    if (cItem.contains("level_shift") && cItem["level_shift"].is_boolean() && cItem["level_shift"].template get<bool>()) {
                                        if (cItem.contains("level_offset")) {
                                            if (cItem["level_offset"].is_string()) fc.pwmMasterLevelOffset[idx] = std::stod(cItem["level_offset"].template get<std::string>());
                                            else if (cItem["level_offset"].is_number()) fc.pwmMasterLevelOffset[idx] = cItem["level_offset"].template get<double>();
                                        }
                                    }
                                }
                            }
                        }
                    }
                } catch (...) {}
            }

            std::string commonInKey = getParamString(ctrlComp, "In", "");
            if (commonInKey.empty()) commonInKey = getParamString(ctrlComp, "In1", "");

            for (int i = 0; i < numCarriers; ++i) {
                int chIdx = i + 1;
                std::string inK = isCommonMod ? commonInKey : getParamString(ctrlComp, "In" + std::to_string(chIdx), "");
                if (inK.empty() && !isCommonMod) inK = getParamString(ctrlComp, "input_" + std::to_string(chIdx), "");
                if (inK.empty() && !isCommonMod) inK = getParamString(ctrlComp, "input" + std::to_string(chIdx), "");
                if (inK.empty() && i == 0) inK = commonInKey;

                if (!inK.empty()) {
                    fc.pwmMasterInIndices[i] = getOrCreateSignalIdx(inK);
                }

                if (fc.pwmMasterPhaseExt[i]) {
                    std::string extK = getParamString(ctrlComp, "ExtPhase" + std::to_string(chIdx), "");
                    if (!extK.empty()) fc.pwmMasterExtPhaseIndices[i] = getOrCreateSignalIdx(extK);
                }

                std::string dKey = ctrlComp.id + ".P" + std::to_string(chIdx);
                std::string cKey = ctrlComp.id + ".P" + std::to_string(chIdx) + "_n";

                fc.pwmMasterOutDirectIndices[i] = getOrCreateSignalIdx(dKey);
                fc.pwmMasterOutComplIndices[i] = getOrCreateSignalIdx(cKey);

                // Keys and indices must stay in step: the recorder walks outputSigIndices,
                // so a key with no index was never written and came back empty.
                const std::string portKeys[5] = {
                    dKey, cKey,
                    ctrlComp.id + ".OutDirect" + std::to_string(chIdx),
                    ctrlComp.id + ".OutCompl" + std::to_string(chIdx),
                    ctrlComp.id + ".Out" + std::to_string(chIdx)};
                for (const auto& pk : portKeys) {
                    fc.outputSigKeys.push_back(pk);
                    fc.outputSigIndices.push_back(getOrCreateSignalIdx(pk));
                }
            }
        } else if (ctrlComp.type == ComponentType::LUT_1D) {
            fc.polarity = getParamString(ctrlComp, "x", "[0, 1]");
            if (ctrlComp.parameters.count("x_data")) fc.polarity = getParamString(ctrlComp, "x_data", "[0, 1]");
            fc.vPlotKey = getParamString(ctrlComp, "y", "[0, 1]");
            if (ctrlComp.parameters.count("y_data")) fc.vPlotKey = getParamString(ctrlComp, "y_data", "[0, 1]");
        } else if (ctrlComp.type == ComponentType::Integrator) {
            fc.val = evaluateParam(ctrlComp, "initial_condition", 0.0);
            if (ctrlComp.parameters.count("initial_value")) fc.val = evaluateParam(ctrlComp, "initial_value", 0.0);
            if (ctrlComp.parameters.count("x0")) fc.val = evaluateParam(ctrlComp, "x0", 0.0);
            fc.gain = evaluateParam(ctrlComp, "K", 1.0);
            if (ctrlComp.parameters.count("gain")) fc.gain = evaluateParam(ctrlComp, "gain", 1.0);
            if (ctrlComp.parameters.count("k")) fc.gain = evaluateParam(ctrlComp, "k", 1.0);
        } else if (ctrlComp.type == ComponentType::TransferFunction) {
            fc.polarity = getParamString(ctrlComp, "num", "");
            if (fc.polarity.empty()) fc.polarity = getParamString(ctrlComp, "numerator", "");
            if (fc.polarity.empty()) fc.polarity = getParamString(ctrlComp, "n", "[1]");

            fc.vPlotKey = getParamString(ctrlComp, "den", "");
            if (fc.vPlotKey.empty()) fc.vPlotKey = getParamString(ctrlComp, "denominator", "");
            if (fc.vPlotKey.empty()) fc.vPlotKey = getParamString(ctrlComp, "d", "[1, 1]");

            fc.gain = evaluateParam(ctrlComp, "K", 1.0);
            if (ctrlComp.parameters.count("gain")) fc.gain = evaluateParam(ctrlComp, "gain", 1.0);
            if (ctrlComp.parameters.count("k")) fc.gain = evaluateParam(ctrlComp, "k", 1.0);
        } else if (ctrlComp.type == ComponentType::ContinuousPID) {
            fc.gain = evaluateParam(ctrlComp, "Kp", 1.0);
            // ts > 0 turns the controller into a sampled one (DISCRETE_PID); 0 is continuous.
            fc.delay = evaluateParam(ctrlComp, "ts", 0.0);
            // Ki and Kd travel as text (read back with atof). std::to_string keeps only
            // six decimals, which rounded small gains (Kd = 5e-7 became 0), so print
            // them round-trip exact instead.
            auto exactText = [](double x) {
                char buf[32];
                std::snprintf(buf, sizeof buf, "%.17g", x);
                return std::string(buf);
            };
            fc.vAlphaKey = exactText(evaluateParam(ctrlComp, "Ki", 0.0));
            fc.vBetaKey = exactText(evaluateParam(ctrlComp, "Kd", 0.0));
            fc.minVal = evaluateParam(ctrlComp, "Tf", 0.01);
            // These were emitted into the netlist but never read, so a limited PID
            // ran unlimited and wound up.
            auto isOn = [&](const char* key) {
                std::string v = toLowerAscii(getParamString(ctrlComp, key, "false"));
                return v == "true" || v == "1" || v == "on" || v == "yes";
            };
            fc.pidLimitOutput = isOn("limit_output");
            fc.pidAntiWindup = isOn("anti_windup");
            fc.pidUpper = evaluateParam(ctrlComp, "upper_limit", 1.0);
            fc.pidLower = evaluateParam(ctrlComp, "lower_limit", -1.0);
            if (fc.pidLower > fc.pidUpper) std::swap(fc.pidLower, fc.pidUpper);
        } else if (ctrlComp.type == ComponentType::PLL_1PH || ctrlComp.type == ComponentType::PLL_3PH) {
            fc.freq = evaluateParam(ctrlComp, "fn", 50.0);
            fc.gain = evaluateParam(ctrlComp, "Kp", 20.0);
            fc.maxVal = evaluateParam(ctrlComp, "Ki", 1000.0);
        } else if (ctrlComp.type == ComponentType::Delay || ctrlComp.type == ComponentType::TransportDelay) {
            fc.delayDuration = evaluateParam(ctrlComp, "delay", 0.1);
        } else if (ctrlComp.type == ComponentType::TurnOnDelay) {
            fc.delayDuration = evaluateParam(ctrlComp, "delay", 0.05);
        } else if (ctrlComp.type == ComponentType::MemoryBlock) {
            fc.val = evaluateParam(ctrlComp, "initial_value", 0.0);
        } else if (ctrlComp.type == ComponentType::Quantizer) {
            if (ctrlComp.parameters.count("interval")) fc.minVal = evaluateParam(ctrlComp, "interval", 0.5);
            else if (ctrlComp.parameters.count("step_size")) fc.minVal = evaluateParam(ctrlComp, "step_size", 0.5);
            else if (ctrlComp.parameters.count("step")) fc.minVal = evaluateParam(ctrlComp, "step", 0.5);
            else if (ctrlComp.parameters.count("quantization_interval")) fc.minVal = evaluateParam(ctrlComp, "quantization_interval", 0.5);
            else if (ctrlComp.parameters.count("q")) fc.minVal = evaluateParam(ctrlComp, "q", 0.5);
            else fc.minVal = evaluateParam(ctrlComp, "step_size", 0.5);
            fc.polarity = getParamString(ctrlComp, "mode", "round");
        } else if (ctrlComp.type == ComponentType::SignalSwitch) {
            fc.thresholdVal = evaluateParam(ctrlComp, "threshold", 0.5);
            fc.polarity = getParamString(ctrlComp, "criteria", "u2 >= threshold");
        } else if (ctrlComp.type == ComponentType::ManualSwitch) {
            fc.polarity = getParamString(ctrlComp, "state", "Input 1");
        } else if (ctrlComp.type == ComponentType::MultiportSwitch) {
            fc.polarity = getParamString(ctrlComp, "indexing", "1-based");
            fc.val = evaluateParam(ctrlComp, "inputs", 3.0);
            // Data inputs In1..InN come from the netlist "inputs" array (the parser
            // exposes them as In1..InN). They were never bound, so the selected input
            // always read 0 and the switch output was constant zero.
            const int nData = std::max(1, (int)fc.val);
            for (int k = 1; k <= nData; ++k) {
                std::string inK = getParamString(ctrlComp, "In" + std::to_string(k), "");
                fc.inputSigKeys.push_back(inK);
                fc.inputSigIndices.push_back(getOrCreateSignalIdx(inK));
            }
        } else if (ctrlComp.type == ComponentType::HitCrossing) {
            fc.thresholdVal = evaluateParam(ctrlComp, "hit_threshold", 0.0);
            if (!ctrlComp.parameters.count("hit_threshold") && ctrlComp.parameters.count("offset")) fc.thresholdVal = evaluateParam(ctrlComp, "offset", 0.0);
            if (!ctrlComp.parameters.count("hit_threshold") && !ctrlComp.parameters.count("offset") && ctrlComp.parameters.count("threshold")) fc.thresholdVal = evaluateParam(ctrlComp, "threshold", 0.0);
            fc.polarity = getParamString(ctrlComp, "direction", "either");
        } else if (ctrlComp.type == ComponentType::Saturation) {
            auto getParamVal = [&](const std::vector<std::string>& keys, double defaultVal) -> double {
                for (const auto& k : keys) {
                    if (ctrlComp.parameters.count(k)) {
                        try { return std::stod(ctrlComp.parameters.at(k)); } catch(...) {}
                    }
                }
                return defaultVal;
            };

            fc.minVal = getParamVal({"lower_limit", "min_limit", "min", "minVal", "v_min", "min_val", "lower", "low"}, -10.0);
            fc.maxVal = getParamVal({"upper_limit", "max_limit", "max", "maxVal", "v_max", "max_val", "upper", "high"}, 10.0);
        } else if (ctrlComp.type == ComponentType::DeadZone) {
            auto getParamVal = [&](const std::vector<std::string>& keys, double defaultVal) -> double {
                for (const auto& k : keys) {
                    if (ctrlComp.parameters.count(k)) {
                        try { return std::stod(ctrlComp.parameters.at(k)); } catch(...) {}
                    }
                }
                return defaultVal;
            };

            fc.minVal = getParamVal({"start_of_dead_zone", "dead_zone_start", "start_dead_zone", "start", "min", "lower_limit", "low", "lower"}, -0.5);
            fc.maxVal = getParamVal({"end_of_dead_zone", "dead_zone_end", "end_dead_zone", "end", "max", "upper_limit", "high", "upper"}, 0.5);
        } else if (ctrlComp.type == ComponentType::RateLimiter) {
            auto getParamVal = [&](const std::vector<std::string>& keys, double defaultVal) -> double {
                for (const auto& k : keys) {
                    if (ctrlComp.parameters.count(k)) {
                        try { return std::stod(ctrlComp.parameters.at(k)); } catch(...) {}
                    }
                }
                return defaultVal;
            };

            fc.rateUp = getParamVal({"rising_slew_rate", "slew_rate_rising", "rising_rate", "rate_up", "slew_rate_up", "rate_rising", "up", "rising"}, 10.0);
            fc.rateDown = getParamVal({"falling_slew_rate", "slew_rate_falling", "falling_rate", "rate_down", "slew_rate_down", "rate_falling", "down", "falling"}, -10.0);
        } else if (ctrlComp.type == ComponentType::Filter1st || ctrlComp.type == ComponentType::Filter2nd) {
            auto getParamVal = [&](const std::vector<std::string>& keys, double defaultVal) -> double {
                for (const auto& k : keys) {
                    if (ctrlComp.parameters.count(k)) {
                        try { return std::stod(ctrlComp.parameters.at(k)); } catch(...) {}
                    }
                }
                return defaultVal;
            };

            fc.freq = getParamVal({"cutoff_freq", "cutoff", "fc", "freq", "frequency", "f"}, 100.0);
            fc.gain = getParamVal({"damping", "damping_ratio", "zeta", "Q", "q"}, 0.707);
        } else if (ctrlComp.type == ComponentType::MovAvg) {
            // Window length in samples. This was never set, so the runtime always used
            // its default of 4 samples and the "window" parameter had no effect.
            fc.shiftLength = std::max(1, (int)std::llround(evaluateParam(ctrlComp, "window", 10.0)));
        } else if (ctrlComp.type == ComponentType::PerAvg) {
            fc.delayDuration = evaluateParam(ctrlComp, "period", 0.02);
            if (!ctrlComp.parameters.count("period") && ctrlComp.parameters.count("T")) fc.delayDuration = evaluateParam(ctrlComp, "T", 0.02);
            if (!ctrlComp.parameters.count("period") && !ctrlComp.parameters.count("T") && ctrlComp.parameters.count("duration")) fc.delayDuration = evaluateParam(ctrlComp, "duration", 0.02);
            if (!ctrlComp.parameters.count("period") && !ctrlComp.parameters.count("T") && !ctrlComp.parameters.count("duration") && ctrlComp.parameters.count("time")) fc.delayDuration = evaluateParam(ctrlComp, "time", 0.02);
        } else if (ctrlComp.type == ComponentType::LUT_1D) {
            fc.polarity = getParamString(ctrlComp, "table_x", "[0, 1]");
            if (!ctrlComp.parameters.count("table_x") && ctrlComp.parameters.count("x")) fc.polarity = getParamString(ctrlComp, "x", "[0, 1]");
            fc.vPlotKey = getParamString(ctrlComp, "table_y", "[0, 1]");
            if (!ctrlComp.parameters.count("table_y") && ctrlComp.parameters.count("y")) fc.vPlotKey = getParamString(ctrlComp, "y", "[0, 1]");
        } else if (ctrlComp.type == ComponentType::LUT_2D) {
            fc.polarity = getParamString(ctrlComp, "table_x", "[0, 1]");
            if (!ctrlComp.parameters.count("table_x") && ctrlComp.parameters.count("x")) fc.polarity = getParamString(ctrlComp, "x", "[0, 1]");

            fc.vPlotKey = getParamString(ctrlComp, "table_y", "[0, 1]");
            if (!ctrlComp.parameters.count("table_y") && ctrlComp.parameters.count("y")) fc.vPlotKey = getParamString(ctrlComp, "y", "[0, 1]");

            fc.vAlphaKey = getParamString(ctrlComp, "table_z", "[[0, 1], [1, 2]]");
            if (!ctrlComp.parameters.count("table_z") && ctrlComp.parameters.count("z")) fc.vAlphaKey = getParamString(ctrlComp, "z", "[[0, 1], [1, 2]]");
            if (!ctrlComp.parameters.count("table_z") && !ctrlComp.parameters.count("z") && ctrlComp.parameters.count("table_data")) fc.vAlphaKey = getParamString(ctrlComp, "table_data", "[[0, 1], [1, 2]]");
        } else if (ctrlComp.type == ComponentType::UnitDelay || ctrlComp.type == ComponentType::ZoH) {
            // Sample period. The output changes only at sample instants and is held in between.
            fc.delay = evaluateParam(ctrlComp, "ts", 0.02);
        } else if (ctrlComp.type == ComponentType::DcMotor) {
            // stateVal is the shaft speed. inputSigIndices[0] is the armature current signal and
            // outputSigIndices[0] receives the back-EMF Ke * speed.
            fc.dcKt = evaluateParam(ctrlComp, "Kt", 0.05);
            fc.dcKe = evaluateParam(ctrlComp, "Ke", 0.05);
            fc.dcJ = evaluateParam(ctrlComp, "J", 1e-4);
            fc.dcB = evaluateParam(ctrlComp, "B", 1e-4);
            fc.stateVal = evaluateParam(ctrlComp, "omega0", 0.0);
            fc.inputSigIndices.assign(1, getOrCreateSignalIdx(getParamString(ctrlComp, "current", "")));
            fc.outputSigIndices.assign(1, getOrCreateSignalIdx(ctrlComp.id + ".Emf"));
        } else if (ctrlComp.type == ComponentType::BldcMotor || ctrlComp.type == ComponentType::PmsmMotor ||
                   ctrlComp.type == ComponentType::SynchronousMotor) {
            // Three-phase machine with back-EMF e_x = k * omega * shape(theta - 2*pi*x/3).
            // inputSigIndices: phase currents a, b, c (and the field current for SYNC_MOTOR).
            // outputSigIndices: the phase EMFs. stateVal is the speed, mTheta the electrical angle.
            fc.mPoles = evaluateParam(ctrlComp, "P", 2.0);
            fc.mKe = evaluateParam(ctrlComp, "Ke", 0.05);
            fc.mKf = evaluateParam(ctrlComp, "Kf", 0.05);
            fc.mJ = evaluateParam(ctrlComp, "J", 1e-4);
            fc.mB = evaluateParam(ctrlComp, "B", 1e-4);
            fc.stateVal = evaluateParam(ctrlComp, "omega0", 0.0);
            fc.mTheta = evaluateParam(ctrlComp, "theta0", 0.0);
            fc.inputSigIndices.clear();
            fc.inputSigIndices.push_back(getOrCreateSignalIdx(getParamString(ctrlComp, "current_a", "")));
            fc.inputSigIndices.push_back(getOrCreateSignalIdx(getParamString(ctrlComp, "current_b", "")));
            fc.inputSigIndices.push_back(getOrCreateSignalIdx(getParamString(ctrlComp, "current_c", "")));
            if (ctrlComp.type == ComponentType::SynchronousMotor)
                fc.inputSigIndices.push_back(getOrCreateSignalIdx(getParamString(ctrlComp, "current_f", "")));
            fc.outputSigIndices.clear();
            fc.outputSigIndices.push_back(getOrCreateSignalIdx(ctrlComp.id + ".Ea"));
            fc.outputSigIndices.push_back(getOrCreateSignalIdx(ctrlComp.id + ".Eb"));
            fc.outputSigIndices.push_back(getOrCreateSignalIdx(ctrlComp.id + ".Ec"));
        } else if (ctrlComp.type == ComponentType::SrMotor) {
            // Switched reluctance: L_x(theta) = Lavg + Ldiff cos(theta - 2*pi*x/3), theta = Nr * theta_m.
            // outputSigIndices: EMF a, b, c (0..2), then the phase inductances a, b, c (3..5).
            const double lmin = evaluateParam(ctrlComp, "Lmin", 2e-3);
            const double lmax = evaluateParam(ctrlComp, "Lmax", 10e-3);
            fc.mPoles = evaluateParam(ctrlComp, "Nr", 4.0);
            fc.mLavg = 0.5 * (lmin + lmax);
            fc.mLdiff = 0.5 * (lmax - lmin);
            fc.mJ = evaluateParam(ctrlComp, "J", 1e-4);
            fc.mB = evaluateParam(ctrlComp, "B", 1e-4);
            fc.stateVal = evaluateParam(ctrlComp, "omega0", 0.0);
            fc.mTheta = evaluateParam(ctrlComp, "theta0", 0.0);
            fc.inputSigIndices.clear();
            fc.inputSigIndices.push_back(getOrCreateSignalIdx(getParamString(ctrlComp, "current_a", "")));
            fc.inputSigIndices.push_back(getOrCreateSignalIdx(getParamString(ctrlComp, "current_b", "")));
            fc.inputSigIndices.push_back(getOrCreateSignalIdx(getParamString(ctrlComp, "current_c", "")));
            fc.outputSigIndices.clear();
            for (const char* sfx : {".Ea", ".Eb", ".Ec", ".La", ".Lb", ".Lc"})
                fc.outputSigIndices.push_back(getOrCreateSignalIdx(ctrlComp.id + sfx));
        } else if (ctrlComp.type == ComponentType::InductionMotor) {
            // Squirrel-cage induction machine in the stationary alpha-beta frame. The rotor flux
            // linkage (mPsiA, mPsiB) is the state. Stator EMFs are (Lm / Lr) d psi_r / dt.
            fc.mPoles = evaluateParam(ctrlComp, "P", 2.0);
            fc.mRr = evaluateParam(ctrlComp, "Rr", 1.083);
            fc.mLr = evaluateParam(ctrlComp, "Lr", 0.105974);
            fc.mLm = evaluateParam(ctrlComp, "Lm", 0.1);
            fc.mJ = evaluateParam(ctrlComp, "J", 0.02);
            fc.mB = evaluateParam(ctrlComp, "B", 1e-3);
            fc.stateVal = evaluateParam(ctrlComp, "omega0", 0.0);
            fc.mPsiA = 0.0;
            fc.mPsiB = 0.0;
            fc.inputSigIndices.clear();
            fc.inputSigIndices.push_back(getOrCreateSignalIdx(getParamString(ctrlComp, "current_a", "")));
            fc.inputSigIndices.push_back(getOrCreateSignalIdx(getParamString(ctrlComp, "current_b", "")));
            fc.inputSigIndices.push_back(getOrCreateSignalIdx(getParamString(ctrlComp, "current_c", "")));
            fc.outputSigIndices.clear();
            fc.outputSigIndices.push_back(getOrCreateSignalIdx(ctrlComp.id + ".Ea"));
            fc.outputSigIndices.push_back(getOrCreateSignalIdx(ctrlComp.id + ".Eb"));
            fc.outputSigIndices.push_back(getOrCreateSignalIdx(ctrlComp.id + ".Ec"));
        } else if (ctrlComp.type == ComponentType::DiscreteTF) {
            // y[k] = (sum_i b_i u[k-i] - sum_{i>=1} a_i y[k-i]) / a0. dtfU and dtfY hold the past
            // inputs and outputs, most recent first.
            fc.delay = evaluateParam(ctrlComp, "ts", 0.01);
            if (fc.delay <= 0.0) fc.delay = 0.01;
            auto numRows = parseNumRows(getParamString(ctrlComp, "num", "[0.1]"));
            auto denRows = parseNumRows(getParamString(ctrlComp, "den", "[1 -0.9]"));
            fc.dtfNum = numRows.empty() ? std::vector<double>{0.1} : numRows[0];
            fc.dtfDen = denRows.empty() ? std::vector<double>{1.0} : denRows[0];
            fc.dtfU.assign(fc.dtfNum.size() > 1 ? fc.dtfNum.size() - 1 : 0, 0.0);
            fc.dtfY.assign(fc.dtfDen.size() > 1 ? fc.dtfDen.size() - 1 : 0, 0.0);
        } else if (ctrlComp.type == ComponentType::DiscreteSS) {
            // x[k+1] = A x[k] + B u[k], y[k] = C x[k] + D u[k]. A is n x n, B, C and x0 have length n.
            fc.delay = evaluateParam(ctrlComp, "ts", 0.01);
            if (fc.delay <= 0.0) fc.delay = 0.01;
            auto aRows = parseNumRows(getParamString(ctrlComp, "A", "[0.5]"));
            const size_t n = aRows.empty() ? 1 : aRows.size();
            auto firstRowPadded = [](const std::vector<std::vector<double>>& rows, size_t len) {
                std::vector<double> v(len, 0.0);
                if (!rows.empty())
                    for (size_t i = 0; i < len && i < rows[0].size(); ++i) v[i] = rows[0][i];
                return v;
            };
            fc.dssA.assign(n * n, 0.0);
            for (size_t i = 0; i < n && i < aRows.size(); ++i)
                for (size_t j = 0; j < n && j < aRows[i].size(); ++j) fc.dssA[i * n + j] = aRows[i][j];
            fc.dssB = firstRowPadded(parseNumRows(getParamString(ctrlComp, "B", "[1]")), n);
            fc.dssC = firstRowPadded(parseNumRows(getParamString(ctrlComp, "C", "[1]")), n);
            auto dRows = parseNumRows(getParamString(ctrlComp, "D", "[0]"));
            fc.dssD = (dRows.empty() || dRows[0].empty()) ? 0.0 : dRows[0][0];
            fc.dssX = firstRowPadded(parseNumRows(getParamString(ctrlComp, "x0", "[0]")), n);
        } else if (ctrlComp.type == ComponentType::DiscreteInt) {
            // Forward Euler: y[k] = y[k-1] + ts * u[k-1]. prevOut holds y, stateVal the last input.
            fc.delay = evaluateParam(ctrlComp, "ts", 0.01);
            if (fc.delay <= 0.0) fc.delay = 0.01;
            fc.prevOut = evaluateParam(ctrlComp, "initial_value", 1.0);
        } else if (ctrlComp.type == ComponentType::DiscreteMean) {
            // Mean of the samples taken in the last `period`. shiftLength is that window in samples.
            fc.delay = evaluateParam(ctrlComp, "ts", 0.001);
            if (fc.delay <= 0.0) fc.delay = 0.001;
            const double meanPeriod = evaluateParam(ctrlComp, "period", 0.02);
            fc.shiftLength = std::max(1, (int)std::llround(meanPeriod / fc.delay));
            fc.prevOut = evaluateParam(ctrlComp, "initial_value", 1.0);
        } else if (ctrlComp.type == ComponentType::Polynomial) {
            // coefficients, highest power first: "[1, 0]" is u. Parsed once here.
            std::string cs = getParamString(ctrlComp, "coefficients", "[1, 0]");
            for (char& c : cs) if (c == '[' || c == ']' || c == ',') c = ' ';
            std::stringstream csStream(cs);
            double cv = 0.0;
            while (csStream >> cv) fc.polyCoeffs.push_back(cv);
        } else if (ctrlComp.type == ComponentType::AlgebraicConstraint) {
            // initial_guess seeds the unknown. gain is the correction step per sample.
            fc.stateVal = evaluateParam(ctrlComp, "initial_guess", 0.0);
            fc.gain = evaluateParam(ctrlComp, "gain", 0.01);
        } else if (ctrlComp.type == ComponentType::LUT_3D) {
            // Axes go in polarity (x), vPlotKey (y) and vBetaKey (z). The cube
            // table[i][j][k] goes in vAlphaKey. Keys come from the netlist (x, y, z, table).
            const std::string kLut3Default = "[[[0, 1], [1, 2]], [[1, 2], [2, 3]]]";
            fc.polarity = getParamString(ctrlComp, "x", "[0, 1]");
            fc.vPlotKey = getParamString(ctrlComp, "y", "[0, 1]");
            fc.vBetaKey = getParamString(ctrlComp, "z", "[0, 1]");
            fc.vAlphaKey = getParamString(ctrlComp, "table", kLut3Default);
        } else if (ctrlComp.type == ComponentType::FourierSeries) {
            // Truncated Fourier series of the input over the last fundamental cycle.
            // f = fundamental, harmonics = number of harmonics kept, ts = sample interval.
            fc.freq = evaluateParam(ctrlComp, "f", 50.0);
            fc.shiftLength = std::max(1, (int)std::llround(evaluateParam(ctrlComp, "harmonics", 10.0)));
            fc.delay = evaluateParam(ctrlComp, "ts", 1e-4);
        } else if (ctrlComp.type == ComponentType::MathFunction) {
            fc.polarity = getParamString(ctrlComp, "function", "exp");
            if (!ctrlComp.parameters.count("function") && ctrlComp.parameters.count("func")) fc.polarity = getParamString(ctrlComp, "func", "exp");
            if (!ctrlComp.parameters.count("function") && !ctrlComp.parameters.count("func") && ctrlComp.parameters.count("fcn")) fc.polarity = getParamString(ctrlComp, "fcn", "exp");
            if (!ctrlComp.parameters.count("function") && !ctrlComp.parameters.count("func") && !ctrlComp.parameters.count("fcn") && ctrlComp.parameters.count("operator")) fc.polarity = getParamString(ctrlComp, "operator", "exp");
            // Normalised once here; evaluateControls() compares it directly.
            fc.polarity = toLowerAscii(fc.polarity);
        } else if (ctrlComp.type == ComponentType::Relay) {
            auto getParamVal = [&](const std::vector<std::string>& keys, double defaultVal) -> double {
                for (const auto& k : keys) {
                    if (ctrlComp.parameters.count(k)) {
                        try { return std::stod(ctrlComp.parameters.at(k)); } catch(...) {}
                    }
                }
                return defaultVal;
            };

            fc.onThresh = getParamVal({"switch_on_point", "on_point", "switch_on", "on_threshold", "high_threshold", "upper_threshold", "on"}, 1.0);
            fc.offThresh = getParamVal({"switch_off_point", "off_point", "switch_off", "off_threshold", "low_threshold", "lower_threshold", "off"}, -1.0);
            fc.outValOn = getParamVal({"output_on", "on_output", "on_val", "output_high", "on_value"}, 1.0);
            fc.outValOff = getParamVal({"output_off", "off_output", "off_val", "output_low", "off_value"}, 0.0);
        } else if (ctrlComp.type == ComponentType::LogicOp || ctrlComp.type == ComponentType::BitwiseOp) {
            std::string op = getParamString(ctrlComp, "operator", "");
            if (op.empty()) op = getParamString(ctrlComp, "op", "");
            if (op.empty()) op = getParamString(ctrlComp, "logic_operator", "");
            if (op.empty()) {
                std::string orig = getParamString(ctrlComp, "original_type", "");
                if (orig == "NAND" || orig == "nand") op = "NAND";
                else if (orig == "NOR" || orig == "nor") op = "NOR";
                else if (orig == "XOR" || orig == "xor") op = "XOR";
                else if (orig == "XNOR" || orig == "xnor" || orig == "NXOR") op = "XNOR";
                else if (orig == "NOT" || orig == "not") op = "NOT";
                else if (orig == "AND" || orig == "and") op = "AND";
                else if (orig == "OR" || orig == "or") op = "OR";
                else if (orig == "BITWISE_OP") op = "XOR";   // JSON default for BITWISE_OP
                else op = "AND";
            }
            std::transform(op.begin(), op.end(), op.begin(), ::toupper);
            fc.polarity = op;
            if (ctrlComp.type == ComponentType::BitwiseOp) fc.bitMask = (long long)evaluateParam(ctrlComp, "bit_mask", -1.0);
        } else if (ctrlComp.type == ComponentType::CombLogic) {
            fc.polarity = getParamString(ctrlComp, "truth_table", "");
        } else if (ctrlComp.type == ComponentType::EdgeDetect) {
            std::string em = getParamString(ctrlComp, "edge", "");
            if (em.empty()) em = getParamString(ctrlComp, "edge_type", "");
            if (em.empty()) em = getParamString(ctrlComp, "detection_mode", "");
            if (em.empty()) em = getParamString(ctrlComp, "trigger_edge", "");
            if (em.empty()) em = getParamString(ctrlComp, "mode", "rising");
            std::transform(em.begin(), em.end(), em.begin(), ::tolower);
            if (em.find("fall") != std::string::npos || em == "neg" || em == "negative") fc.edgeMode = "falling";
            else if (em.find("both") != std::string::npos || em.find("either") != std::string::npos) fc.edgeMode = "either";
            else fc.edgeMode = "rising";

            fc.pulseDuration = evaluateParam(ctrlComp, "pulse_width", 1e-3);
            if (!ctrlComp.parameters.count("pulse_width") && ctrlComp.parameters.count("duration")) fc.pulseDuration = evaluateParam(ctrlComp, "duration", 1e-3);
            if (!ctrlComp.parameters.count("pulse_width") && !ctrlComp.parameters.count("duration") && ctrlComp.parameters.count("width")) fc.pulseDuration = evaluateParam(ctrlComp, "width", 1e-3);
        } else if (ctrlComp.type == ComponentType::Monostable || ctrlComp.type == ComponentType::Monoflop) {
            fc.pulseDuration = evaluateParam(ctrlComp, "duration", 0.01);
            if (!ctrlComp.parameters.count("duration") && ctrlComp.parameters.count("pulse_duration")) fc.pulseDuration = evaluateParam(ctrlComp, "pulse_duration", 0.01);
            if (!ctrlComp.parameters.count("duration") && !ctrlComp.parameters.count("pulse_duration") && ctrlComp.parameters.count("pulse_width")) fc.pulseDuration = evaluateParam(ctrlComp, "pulse_width", 0.01);
            if (!ctrlComp.parameters.count("duration") && !ctrlComp.parameters.count("pulse_duration") && !ctrlComp.parameters.count("pulse_width") && ctrlComp.parameters.count("width")) fc.pulseDuration = evaluateParam(ctrlComp, "width", 0.01);

            std::string em = getParamString(ctrlComp, "trigger_edge", "");
            if (em.empty()) em = getParamString(ctrlComp, "edge", "rising");
            std::transform(em.begin(), em.end(), em.begin(), ::tolower);
            if (em.find("fall") != std::string::npos || em == "neg" || em == "negative") fc.edgeMode = "falling";
            else if (em.find("both") != std::string::npos || em.find("either") != std::string::npos) fc.edgeMode = "either";
            else fc.edgeMode = "rising";

            fc.retriggerable = (getParamString(ctrlComp, "retriggerable", "false") == "true");
        } else if (ctrlComp.type == ComponentType::RelationalOp) {
            fc.polarity = getParamString(ctrlComp, "operator", "<");   // JSON default is "<"
        } else if (ctrlComp.type == ComponentType::CompareToConstant) {
            // Fallback is ">=", the JSON default. It used to be "==", so an unset operator
            // compared for equality, which a continuous input almost never satisfies.
            fc.polarity = getParamString(ctrlComp, "operator", ">=");
            if (!ctrlComp.parameters.count("operator") && ctrlComp.parameters.count("op")) fc.polarity = getParamString(ctrlComp, "op", ">=");
            if (!ctrlComp.parameters.count("operator") && !ctrlComp.parameters.count("op") && ctrlComp.parameters.count("relop")) fc.polarity = getParamString(ctrlComp, "relop", ">=");

            fc.thresholdVal = evaluateParam(ctrlComp, "threshold", 0.0);
            if (!ctrlComp.parameters.count("threshold") && ctrlComp.parameters.count("constant")) fc.thresholdVal = evaluateParam(ctrlComp, "constant", 0.0);
            if (!ctrlComp.parameters.count("threshold") && !ctrlComp.parameters.count("constant") && ctrlComp.parameters.count("const")) fc.thresholdVal = evaluateParam(ctrlComp, "const", 0.0);
            if (!ctrlComp.parameters.count("threshold") && !ctrlComp.parameters.count("constant") && !ctrlComp.parameters.count("const") && ctrlComp.parameters.count("value")) fc.thresholdVal = evaluateParam(ctrlComp, "value", 0.0);
            if (!ctrlComp.parameters.count("threshold") && !ctrlComp.parameters.count("constant") && !ctrlComp.parameters.count("const") && !ctrlComp.parameters.count("value") && ctrlComp.parameters.count("val")) fc.thresholdVal = evaluateParam(ctrlComp, "val", 0.0);
        } else if (ctrlComp.type == ComponentType::DFlipFlop) {
            fc.q_state = evaluateParam(ctrlComp, "initial_state", 0.0) > 0.5 ? 1.0 : 0.0;
            fc.edgeMode = getParamString(ctrlComp, "trigger_edge", "rising");
        } else if (ctrlComp.type == ComponentType::JKFlipFlop) {
            fc.q_state = evaluateParam(ctrlComp, "initial_state", 0.0) > 0.5 ? 1.0 : 0.0;
            fc.edgeMode = getParamString(ctrlComp, "trigger_edge", "rising");
        } else if (ctrlComp.type == ComponentType::ShiftReg) {
            fc.shiftLength = (int)evaluateParam(ctrlComp, "length", 4.0);
            fc.shiftBuffer.assign(fc.shiftLength, 0.0);
        } else if (ctrlComp.type == ComponentType::Offset) {
            fc.thresholdVal = evaluateParam(ctrlComp, "offset", 0.0);
        } else if (ctrlComp.type == ComponentType::DataTypeConv) {
            fc.polarity = getParamString(ctrlComp, "datatype", "boolean");
        } else if (ctrlComp.type == ComponentType::SummingJunction || ctrlComp.type == ComponentType::Product) {
            std::string sStr = getParamString(ctrlComp, "signs", "");
            if (sStr.empty()) sStr = getParamString(ctrlComp, "inputs", "");
            if (sStr.empty()) sStr = getParamString(ctrlComp, "operators", "");
            if (sStr.empty()) sStr = getParamString(ctrlComp, "num_inputs", "");

            int nPins = 2;
            std::string polarityStr = "";
            bool isNumeric = !sStr.empty();
            for (char c : sStr) {
                if (!std::isdigit((unsigned char)c)) { isNumeric = false; break; }
            }

            if (isNumeric) {
                try { nPins = std::clamp(std::stoi(sStr), 1, 32); } catch (...) { nPins = 2; }
                std::string defaultSign = (ctrlComp.type == ComponentType::Product) ? "*" : "+";
                for (int i = 0; i < nPins; ++i) polarityStr += defaultSign;
            } else if (!sStr.empty()) {
                nPins = (int)sStr.length();
                polarityStr = sStr;
            } else {
                nPins = 2;
                polarityStr = (ctrlComp.type == ComponentType::Product) ? "**" : "++";
            }

            fc.polarity = polarityStr;
            fc.inputSigKeys.clear();
            fc.inputSigIndices.clear();

            for (int i = 0; i < nPins; ++i) {
                std::string inK = getParamString(ctrlComp, "In" + std::to_string(i + 1), "");
                if (inK.empty()) inK = getParamString(ctrlComp, "input_" + std::to_string(i), "");
                if (inK.empty() && i == 0) inK = getParamString(ctrlComp, "In", "");
                if (inK.empty() && i == 0) inK = getParamString(ctrlComp, "Num", "");
                if (inK.empty() && i == 1) inK = getParamString(ctrlComp, "Den", "");
                
                fc.inputSigKeys.push_back(inK);
                fc.inputSigIndices.push_back(getOrCreateSignalIdx(inK));
            }
        }

        if (ctrlComp.type == ComponentType::PI_Controller) {
            fc.stateIdx = (int)flatPiIntegratorState.size();
            flatPiIntegratorState.push_back(0.0);
            piIntegratorState[ctrlComp.id] = 0.0;
        }

        fc.in0Key = getParamString(ctrlComp, "In", "");
        if (fc.in0Key.empty()) fc.in0Key = getParamString(ctrlComp, "In1", "");
        if (fc.in0Key.empty()) fc.in0Key = getParamString(ctrlComp, "Num", "");
        if (fc.in0Key.empty()) fc.in0Key = getParamString(ctrlComp, "Plus", "");
        if (fc.in0Key.empty()) fc.in0Key = getParamString(ctrlComp, "input_0", "");
        if (fc.in0Key.empty()) fc.in0Key = getParamString(ctrlComp, "input_a", "");
        if (fc.in0Key.empty()) fc.in0Key = getParamString(ctrlComp, "input", "");   // e.g. GAIN "input": "TRI1.Out"
        if (fc.in0Key.empty()) fc.in0Key = getParamString(ctrlComp, "input1", "");
        if (fc.in0Key.empty()) fc.in0Key = getParamString(ctrlComp, "A", "");
        if (fc.in0Key.empty()) fc.in0Key = getParamString(ctrlComp, "in_a", "");

        fc.in1Key = getParamString(ctrlComp, "In2", "");
        if (fc.in1Key.empty()) fc.in1Key = getParamString(ctrlComp, "Den", "");
        if (fc.in1Key.empty()) fc.in1Key = getParamString(ctrlComp, "Minus", "");
        if (fc.in1Key.empty()) fc.in1Key = getParamString(ctrlComp, "input_1", "");
        if (fc.in1Key.empty()) fc.in1Key = getParamString(ctrlComp, "input_b", "");
        if (fc.in1Key.empty()) fc.in1Key = getParamString(ctrlComp, "input2", "");  // secondary input alias
        if (fc.in1Key.empty()) fc.in1Key = getParamString(ctrlComp, "B", "");
        if (fc.in1Key.empty()) fc.in1Key = getParamString(ctrlComp, "in_b", "");
        fc.outKey = getParamString(ctrlComp, "output", "");
        if (fc.outKey.empty()) fc.outKey = ctrlComp.id + ".Out";
        fc.targetKey = getParamString(ctrlComp, "target", "");
        fc.ctrlSigKey = getParamString(ctrlComp, "selected_signals", "");
        if (fc.ctrlSigKey.empty()) fc.ctrlSigKey = getParamString(ctrlComp, "control_signal", "");
        if (fc.ctrlSigKey.empty()) fc.ctrlSigKey = getParamString(ctrlComp, "Control", "");
        if (fc.ctrlSigKey.empty()) fc.ctrlSigKey = getParamString(ctrlComp, "Ctrl", "");
        if (fc.ctrlSigKey.empty()) fc.ctrlSigKey = getParamString(ctrlComp, "control", "");

        if (ctrlComp.type == ComponentType::Clarke) {
            fc.outKey = ctrlComp.id + ".Alpha";
            std::string inA = getParamString(ctrlComp, "input_a", ""); if (inA.empty()) inA = getParamString(ctrlComp, "input_A", ""); if (inA.empty()) inA = getParamString(ctrlComp, "A", ""); if (inA.empty()) inA = getParamString(ctrlComp, "Va", ""); if (inA.empty()) inA = getParamString(ctrlComp, "a", ""); if (inA.empty()) inA = getParamString(ctrlComp, "In1", "");
            std::string inB = getParamString(ctrlComp, "input_b", ""); if (inB.empty()) inB = getParamString(ctrlComp, "input_B", ""); if (inB.empty()) inB = getParamString(ctrlComp, "B", ""); if (inB.empty()) inB = getParamString(ctrlComp, "Vb", ""); if (inB.empty()) inB = getParamString(ctrlComp, "b", ""); if (inB.empty()) inB = getParamString(ctrlComp, "In2", "");
            std::string inC = getParamString(ctrlComp, "input_c", ""); if (inC.empty()) inC = getParamString(ctrlComp, "input_C", ""); if (inC.empty()) inC = getParamString(ctrlComp, "C", ""); if (inC.empty()) inC = getParamString(ctrlComp, "Vc", ""); if (inC.empty()) inC = getParamString(ctrlComp, "c", ""); if (inC.empty()) inC = getParamString(ctrlComp, "In3", "");
            fc.inputSigIndices.push_back(getOrCreateSignalIdx(inA));
            fc.inputSigIndices.push_back(getOrCreateSignalIdx(inB));
            fc.inputSigIndices.push_back(getOrCreateSignalIdx(inC));
            fc.outputSigKeys.push_back(ctrlComp.id + ".Alpha");
            fc.outputSigKeys.push_back(ctrlComp.id + ".Valpha");
            fc.outputSigKeys.push_back(ctrlComp.id + ".alpha");
            fc.outputSigKeys.push_back(ctrlComp.id + ".Beta");
            fc.outputSigKeys.push_back(ctrlComp.id + ".Vbeta");
            fc.outputSigKeys.push_back(ctrlComp.id + ".beta");
            for (const auto& k : fc.outputSigKeys) {
                fc.outputSigIndices.push_back(getOrCreateSignalIdx(k));
            }
        } else if (ctrlComp.type == ComponentType::InvClarke) {
            fc.outKey = ctrlComp.id + ".A";
            std::string inAlpha = getParamString(ctrlComp, "input_alpha", ""); if (inAlpha.empty()) inAlpha = getParamString(ctrlComp, "Alpha", ""); if (inAlpha.empty()) inAlpha = getParamString(ctrlComp, "Valpha", ""); if (inAlpha.empty()) inAlpha = getParamString(ctrlComp, "alpha", ""); if (inAlpha.empty()) inAlpha = getParamString(ctrlComp, "In1", "");
            std::string inBeta = getParamString(ctrlComp, "input_beta", ""); if (inBeta.empty()) inBeta = getParamString(ctrlComp, "Beta", ""); if (inBeta.empty()) inBeta = getParamString(ctrlComp, "Vbeta", ""); if (inBeta.empty()) inBeta = getParamString(ctrlComp, "beta", ""); if (inBeta.empty()) inBeta = getParamString(ctrlComp, "In2", "");
            fc.inputSigIndices.push_back(getOrCreateSignalIdx(inAlpha));
            fc.inputSigIndices.push_back(getOrCreateSignalIdx(inBeta));
            fc.outputSigKeys.push_back(ctrlComp.id + ".A");
            fc.outputSigKeys.push_back(ctrlComp.id + ".Va");
            fc.outputSigKeys.push_back(ctrlComp.id + ".a");
            fc.outputSigKeys.push_back(ctrlComp.id + ".B");
            fc.outputSigKeys.push_back(ctrlComp.id + ".Vb");
            fc.outputSigKeys.push_back(ctrlComp.id + ".b");
            fc.outputSigKeys.push_back(ctrlComp.id + ".C");
            fc.outputSigKeys.push_back(ctrlComp.id + ".Vc");
            fc.outputSigKeys.push_back(ctrlComp.id + ".c");
            for (const auto& k : fc.outputSigKeys) {
                fc.outputSigIndices.push_back(getOrCreateSignalIdx(k));
            }
        } else if (ctrlComp.type == ComponentType::Park) {
            fc.outKey = ctrlComp.id + ".d";
            std::string inAlpha = getParamString(ctrlComp, "input_alpha", ""); if (inAlpha.empty()) inAlpha = getParamString(ctrlComp, "Alpha", ""); if (inAlpha.empty()) inAlpha = getParamString(ctrlComp, "Valpha", ""); if (inAlpha.empty()) inAlpha = getParamString(ctrlComp, "alpha", ""); if (inAlpha.empty()) inAlpha = getParamString(ctrlComp, "In1", "");
            std::string inBeta = getParamString(ctrlComp, "input_beta", ""); if (inBeta.empty()) inBeta = getParamString(ctrlComp, "Beta", ""); if (inBeta.empty()) inBeta = getParamString(ctrlComp, "Vbeta", ""); if (inBeta.empty()) inBeta = getParamString(ctrlComp, "beta", ""); if (inBeta.empty()) inBeta = getParamString(ctrlComp, "In2", "");
            std::string inTheta = getParamString(ctrlComp, "input_theta", ""); if (inTheta.empty()) inTheta = getParamString(ctrlComp, "Theta", ""); if (inTheta.empty()) inTheta = getParamString(ctrlComp, "theta", ""); if (inTheta.empty()) inTheta = getParamString(ctrlComp, "wt", ""); if (inTheta.empty()) inTheta = getParamString(ctrlComp, "In3", "");
            std::string inA = getParamString(ctrlComp, "input_a", ""); if (inA.empty()) inA = getParamString(ctrlComp, "input_A", ""); if (inA.empty()) inA = getParamString(ctrlComp, "A", ""); if (inA.empty()) inA = getParamString(ctrlComp, "Va", "");
            std::string inB = getParamString(ctrlComp, "input_b", ""); if (inB.empty()) inB = getParamString(ctrlComp, "input_B", ""); if (inB.empty()) inB = getParamString(ctrlComp, "B", ""); if (inB.empty()) inB = getParamString(ctrlComp, "Vb", "");
            std::string inC = getParamString(ctrlComp, "input_c", ""); if (inC.empty()) inC = getParamString(ctrlComp, "input_C", ""); if (inC.empty()) inC = getParamString(ctrlComp, "C", ""); if (inC.empty()) inC = getParamString(ctrlComp, "Vc", "");
            fc.inputSigIndices.push_back(getOrCreateSignalIdx(inAlpha));
            fc.inputSigIndices.push_back(getOrCreateSignalIdx(inBeta));
            fc.inputSigIndices.push_back(getOrCreateSignalIdx(inTheta));
            fc.inputSigIndices.push_back(getOrCreateSignalIdx(inA));
            fc.inputSigIndices.push_back(getOrCreateSignalIdx(inB));
            fc.inputSigIndices.push_back(getOrCreateSignalIdx(inC));
            fc.outputSigKeys.push_back(ctrlComp.id + ".d");
            fc.outputSigKeys.push_back(ctrlComp.id + ".Vd");
            fc.outputSigKeys.push_back(ctrlComp.id + ".D");
            fc.outputSigKeys.push_back(ctrlComp.id + ".q");
            fc.outputSigKeys.push_back(ctrlComp.id + ".Vq");
            fc.outputSigKeys.push_back(ctrlComp.id + ".Q");
            for (const auto& k : fc.outputSigKeys) {
                fc.outputSigIndices.push_back(getOrCreateSignalIdx(k));
            }
        } else if (ctrlComp.type == ComponentType::InvPark) {
            fc.outKey = ctrlComp.id + ".Alpha";
            std::string inD = getParamString(ctrlComp, "input_d", ""); if (inD.empty()) inD = getParamString(ctrlComp, "d", ""); if (inD.empty()) inD = getParamString(ctrlComp, "Vd", ""); if (inD.empty()) inD = getParamString(ctrlComp, "D", ""); if (inD.empty()) inD = getParamString(ctrlComp, "d_in", ""); if (inD.empty()) inD = getParamString(ctrlComp, "In1", "");
            std::string inQ = getParamString(ctrlComp, "input_q", ""); if (inQ.empty()) inQ = getParamString(ctrlComp, "q", ""); if (inQ.empty()) inQ = getParamString(ctrlComp, "Vq", ""); if (inQ.empty()) inQ = getParamString(ctrlComp, "Q", ""); if (inQ.empty()) inQ = getParamString(ctrlComp, "q_in", ""); if (inQ.empty()) inQ = getParamString(ctrlComp, "In2", "");
            std::string inTheta = getParamString(ctrlComp, "input_theta", ""); if (inTheta.empty()) inTheta = getParamString(ctrlComp, "Theta", ""); if (inTheta.empty()) inTheta = getParamString(ctrlComp, "theta", ""); if (inTheta.empty()) inTheta = getParamString(ctrlComp, "wt", ""); if (inTheta.empty()) inTheta = getParamString(ctrlComp, "In3", "");
            fc.inputSigIndices.push_back(getOrCreateSignalIdx(inD));
            fc.inputSigIndices.push_back(getOrCreateSignalIdx(inQ));
            fc.inputSigIndices.push_back(getOrCreateSignalIdx(inTheta));
            fc.outputSigKeys.push_back(ctrlComp.id + ".Alpha");
            fc.outputSigKeys.push_back(ctrlComp.id + ".Valpha");
            fc.outputSigKeys.push_back(ctrlComp.id + ".alpha");
            fc.outputSigKeys.push_back(ctrlComp.id + ".Beta");
            fc.outputSigKeys.push_back(ctrlComp.id + ".Vbeta");
            fc.outputSigKeys.push_back(ctrlComp.id + ".beta");
            fc.outputSigKeys.push_back(ctrlComp.id + ".A");
            fc.outputSigKeys.push_back(ctrlComp.id + ".Va");
            fc.outputSigKeys.push_back(ctrlComp.id + ".B");
            fc.outputSigKeys.push_back(ctrlComp.id + ".Vb");
            fc.outputSigKeys.push_back(ctrlComp.id + ".C");
            fc.outputSigKeys.push_back(ctrlComp.id + ".Vc");
            for (const auto& k : fc.outputSigKeys) {
                fc.outputSigIndices.push_back(getOrCreateSignalIdx(k));
            }
        } else if (ctrlComp.type == ComponentType::DqToAbc) {
            fc.outKey = ctrlComp.id + ".A";
            std::string inD = getParamString(ctrlComp, "input_d", ""); if (inD.empty()) inD = getParamString(ctrlComp, "d", ""); if (inD.empty()) inD = getParamString(ctrlComp, "Vd", ""); if (inD.empty()) inD = getParamString(ctrlComp, "D", ""); if (inD.empty()) inD = getParamString(ctrlComp, "In1", "");
            std::string inQ = getParamString(ctrlComp, "input_q", ""); if (inQ.empty()) inQ = getParamString(ctrlComp, "q", ""); if (inQ.empty()) inQ = getParamString(ctrlComp, "Vq", ""); if (inQ.empty()) inQ = getParamString(ctrlComp, "Q", ""); if (inQ.empty()) inQ = getParamString(ctrlComp, "In2", "");
            std::string inTheta = getParamString(ctrlComp, "input_theta", ""); if (inTheta.empty()) inTheta = getParamString(ctrlComp, "Theta", ""); if (inTheta.empty()) inTheta = getParamString(ctrlComp, "theta", ""); if (inTheta.empty()) inTheta = getParamString(ctrlComp, "wt", ""); if (inTheta.empty()) inTheta = getParamString(ctrlComp, "In3", "");
            fc.inputSigIndices.push_back(getOrCreateSignalIdx(inD));
            fc.inputSigIndices.push_back(getOrCreateSignalIdx(inQ));
            fc.inputSigIndices.push_back(getOrCreateSignalIdx(inTheta));
            fc.outputSigKeys.push_back(ctrlComp.id + ".A");
            fc.outputSigKeys.push_back(ctrlComp.id + ".Va");
            fc.outputSigKeys.push_back(ctrlComp.id + ".a");
            fc.outputSigKeys.push_back(ctrlComp.id + ".Out1");
            fc.outputSigKeys.push_back(ctrlComp.id + ".OutA");
            fc.outputSigKeys.push_back(ctrlComp.id + ".B");
            fc.outputSigKeys.push_back(ctrlComp.id + ".Vb");
            fc.outputSigKeys.push_back(ctrlComp.id + ".b");
            fc.outputSigKeys.push_back(ctrlComp.id + ".Out2");
            fc.outputSigKeys.push_back(ctrlComp.id + ".OutB");
            fc.outputSigKeys.push_back(ctrlComp.id + ".C");
            fc.outputSigKeys.push_back(ctrlComp.id + ".Vc");
            fc.outputSigKeys.push_back(ctrlComp.id + ".c");
            fc.outputSigKeys.push_back(ctrlComp.id + ".Out3");
            fc.outputSigKeys.push_back(ctrlComp.id + ".OutC");
            for (const auto& k : fc.outputSigKeys) {
                fc.outputSigIndices.push_back(getOrCreateSignalIdx(k));
            }
        } else if (ctrlComp.type == ComponentType::AbcToDq) {
            fc.outKey = ctrlComp.id + ".d";
            std::string inA = getParamString(ctrlComp, "input_a", ""); if (inA.empty()) inA = getParamString(ctrlComp, "A", ""); if (inA.empty()) inA = getParamString(ctrlComp, "Va", ""); if (inA.empty()) inA = getParamString(ctrlComp, "In1", "");
            std::string inB = getParamString(ctrlComp, "input_b", ""); if (inB.empty()) inB = getParamString(ctrlComp, "B", ""); if (inB.empty()) inB = getParamString(ctrlComp, "Vb", ""); if (inB.empty()) inB = getParamString(ctrlComp, "In2", "");
            std::string inC = getParamString(ctrlComp, "input_c", ""); if (inC.empty()) inC = getParamString(ctrlComp, "C", ""); if (inC.empty()) inC = getParamString(ctrlComp, "Vc", ""); if (inC.empty()) inC = getParamString(ctrlComp, "In3", "");
            std::string inTheta = getParamString(ctrlComp, "input_theta", ""); if (inTheta.empty()) inTheta = getParamString(ctrlComp, "Theta", ""); if (inTheta.empty()) inTheta = getParamString(ctrlComp, "theta", ""); if (inTheta.empty()) inTheta = getParamString(ctrlComp, "wt", ""); if (inTheta.empty()) inTheta = getParamString(ctrlComp, "In4", "");
            fc.inputSigIndices.push_back(getOrCreateSignalIdx(inA));
            fc.inputSigIndices.push_back(getOrCreateSignalIdx(inB));
            fc.inputSigIndices.push_back(getOrCreateSignalIdx(inC));
            fc.inputSigIndices.push_back(getOrCreateSignalIdx(inTheta));
            fc.outputSigKeys.push_back(ctrlComp.id + ".d");
            fc.outputSigKeys.push_back(ctrlComp.id + ".Vd");
            fc.outputSigKeys.push_back(ctrlComp.id + ".D");
            fc.outputSigKeys.push_back(ctrlComp.id + ".Out1");
            fc.outputSigKeys.push_back(ctrlComp.id + ".q");
            fc.outputSigKeys.push_back(ctrlComp.id + ".Vq");
            fc.outputSigKeys.push_back(ctrlComp.id + ".Q");
            fc.outputSigKeys.push_back(ctrlComp.id + ".Out2");
            for (const auto& k : fc.outputSigKeys) {
                fc.outputSigIndices.push_back(getOrCreateSignalIdx(k));
            }
        } else if (ctrlComp.type == ComponentType::PWM_3PH || ctrlComp.type == ComponentType::SVPWM) {
            fc.outKey = ctrlComp.id + ".G1";
            // The runtime used to infer alpha/beta input from "two inputs are bound",
            // which is always true for an A/B/C block, so phase references were
            // re-mixed as if they were alpha/beta. Decide it from the netlist keys.
            fc.svpwmAlphaBeta = !getParamString(ctrlComp, "Valpha", "").empty() ||
                                !getParamString(ctrlComp, "Alpha", "").empty() ||
                                !getParamString(ctrlComp, "input_alpha", "").empty();
            std::string inA = getParamString(ctrlComp, "Valpha", ""); 
            if (inA.empty()) inA = getParamString(ctrlComp, "Alpha", ""); 
            if (inA.empty()) inA = getParamString(ctrlComp, "input_alpha", ""); 
            if (inA.empty()) inA = getParamString(ctrlComp, "Va", ""); 
            if (inA.empty()) inA = getParamString(ctrlComp, "A", ""); 
            if (inA.empty()) inA = getParamString(ctrlComp, "In1", ""); 
            if (inA.empty()) inA = getParamString(ctrlComp, "input_a", "");

            std::string inB = getParamString(ctrlComp, "Vbeta", ""); 
            if (inB.empty()) inB = getParamString(ctrlComp, "Beta", ""); 
            if (inB.empty()) inB = getParamString(ctrlComp, "input_beta", ""); 
            if (inB.empty()) inB = getParamString(ctrlComp, "Vb", ""); 
            if (inB.empty()) inB = getParamString(ctrlComp, "B", ""); 
            if (inB.empty()) inB = getParamString(ctrlComp, "In2", ""); 
            if (inB.empty()) inB = getParamString(ctrlComp, "input_b", "");

            std::string inC = getParamString(ctrlComp, "Vc", ""); 
            if (inC.empty()) inC = getParamString(ctrlComp, "C", ""); 
            if (inC.empty()) inC = getParamString(ctrlComp, "In3", ""); 
            if (inC.empty()) inC = getParamString(ctrlComp, "input_c", "");

            fc.inputSigIndices.push_back(getOrCreateSignalIdx(inA));
            fc.inputSigIndices.push_back(getOrCreateSignalIdx(inB));
            fc.inputSigIndices.push_back(getOrCreateSignalIdx(inC));

            std::vector<std::string> gateKeys = {
                "G1", "G2", "G3", "G4", "G5", "G6",
                "gA1", "gA2", "gB1", "gB2", "gC1", "gC2",
                "OutA", "OutB", "OutC", "Out1", "Out2", "Out3"
            };
            for (const auto& gk : gateKeys) {
                std::string k = ctrlComp.id + "." + gk;
                fc.outputSigKeys.push_back(k);
                fc.outputSigIndices.push_back(getOrCreateSignalIdx(k));
            }
        } else if (ctrlComp.type == ComponentType::FourierTrans || ctrlComp.type == ComponentType::FourierAnalysis) {
            // Parameters the runtime reads: freq = fundamental f, shiftLength = harmonic
            // order, delay = sample interval ts. None of them were set before, so the
            // generic defaults (10 kHz frequency, 1 harmonic of 4) were used instead.
            fc.freq = evaluateParam(ctrlComp, "f", 50.0);
            fc.shiftLength = std::max(1, (int)std::llround(evaluateParam(ctrlComp, "harmonic", 1.0)));
            fc.delay = evaluateParam(ctrlComp, "ts", 1e-4);
            fc.outKey = ctrlComp.id + ".Mag";
            fc.outputSigKeys.push_back(ctrlComp.id + ".Mag");
            fc.outputSigKeys.push_back(ctrlComp.id + ".Phase");
            fc.outputSigIndices.push_back(getOrCreateSignalIdx(ctrlComp.id + ".Mag"));
            fc.outputSigIndices.push_back(getOrCreateSignalIdx(ctrlComp.id + ".Phase"));
        } else if (ctrlComp.type == ComponentType::PllLoop) {
            // fn/Kp/Ki are emitted by the registry but were never read here, so the
            // loop started from the generic "frequency" default (10 kHz) with Kp = 1.
            fc.freq = evaluateParam(ctrlComp, "fn", 50.0);
            fc.gain = evaluateParam(ctrlComp, "Kp", 20.0);
            fc.maxVal = evaluateParam(ctrlComp, "Ki", 1000.0);
            fc.outKey = ctrlComp.id + ".Theta";
            fc.outputSigKeys.push_back(ctrlComp.id + ".Theta");
            fc.outputSigKeys.push_back(ctrlComp.id + ".Freq");
            fc.outputSigKeys.push_back(ctrlComp.id + ".Cos");
            fc.outputSigKeys.push_back(ctrlComp.id + ".Sin");
            fc.outputSigIndices.push_back(getOrCreateSignalIdx(ctrlComp.id + ".Theta"));
            fc.outputSigIndices.push_back(getOrCreateSignalIdx(ctrlComp.id + ".Freq"));
            fc.outputSigIndices.push_back(getOrCreateSignalIdx(ctrlComp.id + ".Cos"));
            fc.outputSigIndices.push_back(getOrCreateSignalIdx(ctrlComp.id + ".Sin"));
        } else if (ctrlComp.type == ComponentType::PLL_1PH || ctrlComp.type == ComponentType::PLL_3PH) {
            // Both PLLs were stubs: they free-ran at fn and wrote only "<id>.Out", so
            // the Theta/Freq pins the netlist routes downstream were never driven.
            // Register the four outputs the generator emits (Theta, Freq, Cos, Sin).
            // PLL_1PH exposes Theta on its single "Out" pin, so keep outKey there.
            if (ctrlComp.type == ComponentType::PLL_3PH) {
                fc.outKey = ctrlComp.id + ".Theta";
                fc.inputSigKeys.clear();
                fc.inputSigIndices.clear();
                for (int k = 0; k < 3; ++k) {
                    std::string inK = getParamString(ctrlComp, "In" + std::to_string(k + 1), "");
                    if (inK.empty()) inK = getParamString(ctrlComp, "input_" + std::to_string(k), "");
                    if (inK.empty()) inK = getParamString(ctrlComp, std::string(1, (char)('A' + k)), "");
                    fc.inputSigKeys.push_back(inK);
                    fc.inputSigIndices.push_back(getOrCreateSignalIdx(inK));
                }
            }
            for (const char* port : {".Theta", ".Freq", ".Cos", ".Sin"}) {
                fc.outputSigKeys.push_back(ctrlComp.id + port);
                fc.outputSigIndices.push_back(getOrCreateSignalIdx(ctrlComp.id + port));
            }
        } else if (ctrlComp.type == ComponentType::PeriodicImpAvg) {
            // The trigger is emitted as control_signal (registry: signal "Trig"). The
            // signal index is computed earlier in this function from the generic key, so
            // it must be recomputed here too; otherwise the trigger was never bound and
            // the output stayed at its default value.
            std::string trigKey = getParamString(ctrlComp, "control_signal", "");
            if (trigKey.empty()) trigKey = getParamString(ctrlComp, "Trig", "");
            fc.ctrlSigKey = trigKey;
            fc.ctrlSigSignalIdx = getOrCreateSignalIdx(trigKey);
            fc.val = evaluateParam(ctrlComp, "initial_value", 0.0);
        }

        if (ctrlComp.type == ComponentType::UnifiedProbe) {
            std::string targetComp = getParamString(ctrlComp, "target", "");
            std::string selSigs = getParamString(ctrlComp, "selected_signals", "");
            std::string probeType = getParamString(ctrlComp, "probe_type", "Voltage");
            std::string probeSig = getParamString(ctrlComp, "probe_signal", "");

            std::string srcKey = "";
            if (!selSigs.empty()) {
                srcKey = selSigs;
                size_t comma = srcKey.find(',');
                if (comma != std::string::npos) srcKey = srcKey.substr(0, comma);
            } else if (!probeSig.empty()) {
                srcKey = probeSig;
            } else if (!targetComp.empty()) {
                if (targetComp.rfind("V_", 0) == 0 || targetComp.rfind("I_", 0) == 0) {
                    srcKey = targetComp;
                } else if (probeType == "Current" || probeType == "I") {
                    srcKey = "I_" + targetComp;
                } else {
                    srcKey = "V_" + targetComp;
                }
            }

            fc.ctrlSigKey = srcKey;
            fc.outKey = ctrlComp.id + ".Out";

            // Register output signal indices so PROBE outputs to .Out, PROBE ID, custom_plots, and terminal pin names
            fc.outputSigKeys.push_back(ctrlComp.id + ".Out");
            fc.outputSigKeys.push_back(ctrlComp.id);
            if (!selSigs.empty()) fc.outputSigKeys.push_back(ctrlComp.id + "." + selSigs);
            if (!srcKey.empty()) fc.outputSigKeys.push_back(ctrlComp.id + "." + srcKey);

            fc.outputSigIndices.clear();
            for (const auto& k : fc.outputSigKeys) {
                fc.outputSigIndices.push_back(getOrCreateSignalIdx(k));
            }
            if (!srcKey.empty()) {
                fc.ctrlSigSignalIdx = getOrCreateSignalIdx(srcKey);
                fc.targetSignalIdx = getOrCreateSignalIdx(srcKey);
            }
        }

        if (ctrlComp.type == ComponentType::DFlipFlop || ctrlComp.type == ComponentType::JKFlipFlop) {
            // Register Q and Q_bar in outputSigIndices. This runs whether or not the netlist
            // named the output: the netlist sets outKey to Q, which used to skip the Q_bar
            // registration, so feedback from Q_bar read 0 and the flip-flop never toggled.
            if (fc.outKey.empty()) fc.outKey = ctrlComp.id + ".Q";
            fc.outputSigIndices.push_back(getOrCreateSignalIdx(ctrlComp.id + ".Q"));
            fc.outputSigIndices.push_back(getOrCreateSignalIdx(ctrlComp.id + ".Q_bar"));
        } else if (fc.outKey.empty()) {
            fc.outKey = ctrlComp.id + ".Out";
        }

        fc.in0SignalIdx = getOrCreateSignalIdx(fc.in0Key);
        fc.in1SignalIdx = getOrCreateSignalIdx(fc.in1Key);
        if (ctrlComp.type == ComponentType::LUT_3D) {
            // LUT_3D reads In, In2 and In3 as u1, u2 and u3 through inputSigIndices.
            fc.inputSigIndices = {fc.in0SignalIdx, fc.in1SignalIdx, getOrCreateSignalIdx(getParamString(ctrlComp, "In3", ""))};
        }
        fc.outSignalIdx = getOrCreateSignalIdx(fc.outKey);
        if (ctrlComp.type != ComponentType::UnifiedProbe) {
            fc.targetSignalIdx = getOrCreateSignalIdx(fc.targetKey);
            fc.ctrlSigSignalIdx = getOrCreateSignalIdx(fc.ctrlSigKey);
        }
        getOrCreateSignalIdx(fc.id);

        if (ctrlComp.type == ComponentType::CustomScript) {
            std::string code = getParamString(ctrlComp, "code", "");
            cscriptEngines[ctrlComp.id].setup(code, ctrlComp.parameters);

            std::vector<CScriptPort> discIn, discOut;
            CScriptEngine::discoverPorts(code, discIn, discOut);

            // Ports come back in order of first appearance in the code, but the
            // runtime addresses them by position (inputs[k] / outputs[k]). A script
            // that touched inputs[1] before inputs[0] therefore had its inputs
            // swapped. Re-order by script index and fill gaps so position == index.
            auto byIndex = [](std::vector<CScriptPort> ports, const char* prefix) {
                int maxIdx = -1;
                for (const auto& p : ports) maxIdx = std::max(maxIdx, p.index);
                std::vector<CScriptPort> ordered(maxIdx + 1);
                std::vector<bool> filled(maxIdx + 1, false);
                for (const auto& p : ports) {
                    if (p.index >= 0 && p.index <= maxIdx && !filled[p.index]) {
                        ordered[p.index] = p;
                        filled[p.index] = true;
                    }
                }
                for (int k = 0; k <= maxIdx; ++k) {
                    if (!filled[k]) ordered[k] = {std::string(prefix) + std::to_string(k + 1), false, k};
                }
                return ordered;
            };
            discIn = byIndex(discIn, "In");
            discOut = byIndex(discOut, "Out");

            for (size_t i = 0; i < discIn.size(); ++i) {
                std::string pName = discIn[i].name;
                std::string inK = getParamString(ctrlComp, pName, "");
                if (inK.empty()) inK = getParamString(ctrlComp, "In" + std::to_string(i + 1), "");
                if (inK.empty()) inK = getParamString(ctrlComp, "input_" + std::to_string(i), "");
                if (inK.empty()) inK = ctrlComp.id + "." + pName;

                fc.inputSigKeys.push_back(inK);
                fc.inputSigIndices.push_back(getOrCreateSignalIdx(inK));
            }

            for (size_t j = 0; j < discOut.size(); ++j) {
                std::string pName = discOut[j].name;
                std::string outK = getParamString(ctrlComp, pName, "");
                if (outK.empty()) outK = getParamString(ctrlComp, "Out" + std::to_string(j + 1), "");
                if (outK.empty()) outK = getParamString(ctrlComp, "output_" + std::to_string(j), "");
                if (outK.empty()) outK = ctrlComp.id + "." + pName;

                fc.outputSigKeys.push_back(outK);
                fc.outputSigIndices.push_back(getOrCreateSignalIdx(outK));
            }

            for (const auto& [vName, vVal] : cscriptEngines[ctrlComp.id].getAllVars()) {
                std::string pKey = ctrlComp.id + "." + vName;
                fc.customScriptVarNames.push_back(vName);
                fc.customPlotVarKeys.push_back(pKey);
                fc.customPlotVarIndices.push_back(getOrCreateSignalIdx(pKey));
            }
        }

        fastCtrlComps.push_back(fc);
    }

    // Bind raw pointers to flatControlSignals vector
    for (auto& fc : fastPhysComps) {
        if (fc.ctrlSigSignalIdx >= 0 && fc.ctrlSigSignalIdx < (int)flatControlSignals.size()) fc.ctrlSigPtr = &flatControlSignals[fc.ctrlSigSignalIdx];
    }

    fastGateCtrlComps.clear();
    for (auto& fc : fastCtrlComps) {
        if (fc.in0SignalIdx >= 0 && fc.in0SignalIdx < (int)flatControlSignals.size()) fc.in0Ptr = &flatControlSignals[fc.in0SignalIdx];
        if (fc.in1SignalIdx >= 0 && fc.in1SignalIdx < (int)flatControlSignals.size()) fc.in1Ptr = &flatControlSignals[fc.in1SignalIdx];
        if (fc.outSignalIdx >= 0 && fc.outSignalIdx < (int)flatControlSignals.size()) fc.outPtr = &flatControlSignals[fc.outSignalIdx];
        if (fc.ctrlSigSignalIdx >= 0 && fc.ctrlSigSignalIdx < (int)flatControlSignals.size()) fc.ctrlSigPtr = &flatControlSignals[fc.ctrlSigSignalIdx];
        if (fc.targetSignalIdx >= 0 && fc.targetSignalIdx < (int)flatControlSignals.size()) fc.targetPtr = &flatControlSignals[fc.targetSignalIdx];

        if (fc.type == ComponentType::Triangle_Carrier ||
            fc.type == ComponentType::PWM_Generator ||
            fc.type == ComponentType::PWM_MASTER ||
            fc.type == ComponentType::PWM_3PH ||
            fc.type == ComponentType::SVPWM ||
            fc.type == ComponentType::PulseGenerator ||
            fc.type == ComponentType::Comparator ||
            fc.type == ComponentType::Relay ||
            fc.type == ComponentType::HitCrossing) {
            fastGateCtrlComps.push_back(&fc);
        }
    }

    // Pre-stamp Static Conductance Matrix K_static
    std::fill(K_static.begin(), K_static.end(), 0.0);

    // Add gmin shunt to ground on every node to prevent singular floating subgraphs
    for (int i = 0; i < numNodes; ++i) {
        K_static[i * totalDim + i] += 1e-12;
    }
    for (const auto& fc : fastPhysComps) {
        int n1 = fc.n1;
        int n2 = fc.n2;

        if (fc.type == ComponentType::Resistor) {
            // Signal-controlled resistors change every timestep, so they must not be
            // baked into the static matrix â€” assembleMNA() stamps them instead.
            if (fc.isVariable) continue;

            double Rtotal = fc.val + fc.esr;
            if (Rtotal < 1e-6) Rtotal = 1e-6;
            double g = 1.0 / Rtotal;

            if (n1 >= 0) K_static[n1 * totalDim + n1] += g;
            if (n2 >= 0) K_static[n2 * totalDim + n2] += g;
            if (n1 >= 0 && n2 >= 0) {
                K_static[n1 * totalDim + n2] -= g;
                K_static[n2 * totalDim + n1] -= g;
            }
        }
        else if (fc.type == ComponentType::VoltageSource || 
                 fc.type == ComponentType::ACVoltageSource || 
                 fc.type == ComponentType::ControlledVoltageSource ||
                 fc.type == ComponentType::Ammeter) {
            int vIdx = fc.vIdx;
            if (n1 >= 0) {
                K_static[n1 * totalDim + vIdx] += 1.0;
                K_static[vIdx * totalDim + n1] += 1.0;
            }
            if (n2 >= 0) {
                K_static[n2 * totalDim + vIdx] -= 1.0;
                K_static[vIdx * totalDim + n2] -= 1.0;
            }
        }
        else if (fc.type == ComponentType::Inductor) {
            int lIdx = fc.lIdx;
            if (n1 >= 0) {
                K_static[n1 * totalDim + lIdx] += 1.0;
                K_static[lIdx * totalDim + n1] -= 1.0;
            }
            if (n2 >= 0) {
                K_static[n2 * totalDim + lIdx] -= 1.0;
                K_static[lIdx * totalDim + n2] += 1.0;
            }
        }
        else if (fc.type == ComponentType::Winding) {
            // â”€â”€ Electrical <-> magnetic gyrator (PLECS permeance-capacitance analogy) â”€â”€
            //   v_elec = N * PhiDot        (Faraday)
            //   i_elec = F / N             (Ampere)
            // n1,n2 = electrical +,-   n3,n4 = magnetic +,-
            // rIe carries the electrical port current, rIm carries the flux rate.
            // The magnetic-side KCL sign is inverted relative to the electrical side:
            // that anti-symmetry is what makes this a gyrator rather than a transformer,
            // and it is what turns a magnetic capacitance into an electrical inductance
            // (L = N^2 * P) instead of the reciprocal.
            int rIe = fc.wIdx0;
            int rIm = fc.wIdx1;
            if (rIe < 0 || rIm < 0) continue;

            double N = fc.val;
            if (std::abs(N) < 1e-12) N = 1.0;

            int n3 = fc.n3;
            int n4 = fc.n4;

            // KCL: I_e leaves n1 and enters n2; PhiDot enters n3 and leaves n4.
            if (n1 >= 0) K_static[n1 * totalDim + rIe] += 1.0;
            if (n2 >= 0) K_static[n2 * totalDim + rIe] -= 1.0;
            if (n3 >= 0) K_static[n3 * totalDim + rIm] -= 1.0;
            if (n4 >= 0) K_static[n4 * totalDim + rIm] += 1.0;

            // Row rIe:  I_e - (V(n3) - V(n4)) / N = 0
            K_static[rIe * totalDim + rIe] += 1.0;
            if (n3 >= 0) K_static[rIe * totalDim + n3] -= 1.0 / N;
            if (n4 >= 0) K_static[rIe * totalDim + n4] += 1.0 / N;

            // Row rIm:  (V(n1) - V(n2)) - N * PhiDot = 0
            if (n1 >= 0) K_static[rIm * totalDim + n1] += 1.0;
            if (n2 >= 0) K_static[rIm * totalDim + n2] -= 1.0;
            K_static[rIm * totalDim + rIm] -= N;
        }
        else if (isTransformerType(fc.type)) {
            if (fc.windings.empty()) continue;

            // 1. KCL contributions of all windings (rows n1..n2, column wIdx)
            for (const auto& w : fc.windings) {
                if (w.n1 >= 0) K_static[w.n1 * totalDim + w.wIdx] += 1.0;
                if (w.n2 >= 0) K_static[w.n2 * totalDim + w.wIdx] -= 1.0;
            }

            const auto& w0 = fc.windings[0];
            std::string polStr = fc.polarity;

            auto getEffectiveTurns = [&](const FastCompiledComponent::WindingInfo& w, size_t idx) -> double {
                double t = w.turns;
                if (polStr == "inverted" && idx >= 1) {
                    return -std::abs(t);
                }
                return t;
            };

            double n0 = getEffectiveTurns(w0, 0);

            // 2. Ampere's Law (MMF balance) in row w0.wIdx: Sum(N_k * I_wk) = 0
            for (size_t k = 0; k < fc.windings.size(); ++k) {
                const auto& w = fc.windings[k];
                double nk = getEffectiveTurns(w, k);
                K_static[w0.wIdx * totalDim + w.wIdx] += nk;
            }

            // 3. Faraday's Law (Voltage ratio) in row w_k.wIdx: N_k * (V0_1 - V0_2) - N0 * (Vk_1 - Vk_2) = 0
            for (size_t k = 1; k < fc.windings.size(); ++k) {
                const auto& wk = fc.windings[k];
                double nk = getEffectiveTurns(wk, k);
                if (w0.n1 >= 0) K_static[wk.wIdx * totalDim + w0.n1] += nk;
                if (w0.n2 >= 0) K_static[wk.wIdx * totalDim + w0.n2] -= nk;
                if (wk.n1 >= 0) K_static[wk.wIdx * totalDim + wk.n1] -= n0;
                if (wk.n2 >= 0) K_static[wk.wIdx * totalDim + wk.n2] += n0;
            }
        }
    }
}

// Cache prefilter: FNV-1a over the matrix diagonal only, with an extra shift-xor
// per word so structured matrices still spread out.
//
// Hashing all n^2 entries was itself the dominant solver cost once factorizations
// were being reused (33% of runtime at n=98). The diagonal is O(n) and is a strong
// discriminator here, because every conductance stamp - switches, diodes, the
// companion models - lands on it. It is only a prefilter: a candidate is still
// confirmed with a full bitwise compare, so a weak hash costs time, never accuracy.
static inline uint64_t hashDiagonal(const double* K, int n) {
    uint64_t h = 1469598103934665603ULL ^ (uint64_t)n;
    const size_t stride = (size_t)n + 1;
    const double* p = K;
    for (int i = 0; i < n; ++i, p += stride) {
        uint64_t bits;
        std::memcpy(&bits, p, sizeof(bits));
        h ^= bits;
        h *= 1099511628211ULL;
        h ^= h >> 29;
    }
    return h;
}

void CircuitSimulator::buildSparseTriangular(int n, SparseTriangular& out) const {
    out.valid = false;
    out.Lptr.assign((size_t)n + 1, 0);
    out.Uptr.assign((size_t)n + 1, 0);
    out.Lidx.clear(); out.Lval.clear();
    out.Uidx.clear(); out.Uval.clear();
    out.Udiag.assign((size_t)n, 0.0);

    const double* LU = LU_cached.data();

    // Count first so the index/value arrays are allocated exactly once.
    size_t lnz = 0, unz = 0;
    for (int i = 0; i < n; ++i) {
        const double* row = LU + (size_t)i * n;
        for (int j = 0; j < i; ++j) if (row[j] != 0.0) ++lnz;
        for (int j = i + 1; j < n; ++j) if (row[j] != 0.0) ++unz;
    }

    // If the factors came out nearly dense there is nothing to gain, and the
    // indirection would only add memory traffic.
    const size_t triangleSize = (size_t)n * (size_t)(n - 1) / 2;
    if (triangleSize > 0 && (lnz + unz) * 10 > triangleSize * 2 * 7) return;   // > 70% full

    out.Lidx.resize(lnz); out.Lval.resize(lnz);
    out.Uidx.resize(unz); out.Uval.resize(unz);

    size_t lp = 0, up = 0;
    for (int i = 0; i < n; ++i) {
        const double* row = LU + (size_t)i * n;
        out.Lptr[i] = (int)lp;
        for (int j = 0; j < i; ++j) {
            if (row[j] != 0.0) { out.Lidx[lp] = j; out.Lval[lp] = row[j]; ++lp; }
        }
        out.Uptr[i] = (int)up;
        for (int j = i + 1; j < n; ++j) {
            if (row[j] != 0.0) { out.Uidx[up] = j; out.Uval[up] = row[j]; ++up; }
        }
        out.Udiag[i] = row[i];
    }
    out.Lptr[n] = (int)lp;
    out.Uptr[n] = (int)up;
    out.valid = true;
}

void CircuitSimulator::prepareFactorization(int n) {
    if (n <= 0) return;

    const size_t nn = (size_t)n * (size_t)n;

    // Rolling-window policy constants for the adaptive bail-out.
    constexpr long long kWindow        = 1024;  // lookups per review
    constexpr long long kMinHitPercent = 25;    // below this, caching is not paying
    constexpr long long kRetryAfter    = 20000; // factorizations before trying again

    if (!config.enableLUCache || luCacheMaxEntries == 0 || luCacheSuspended) {
        factorizeLU(n);
        ++luFactorizeCount;
        LU_active = LU_cached.data();
        p_active = p_cached.data();
        // No compression here: without a cache entry to amortise it, building the
        // compressed form costs about as much as the dense solve it would replace.
        sparse_active = nullptr;
        if (luCacheSuspended && --luSuspendCountdown <= 0) {
            luCacheSuspended = false;
            luWindowLookups = 0;
            luWindowHits = 0;
        }
        return;
    }

    ++luWindowLookups;
    const uint64_t key = hashDiagonal(K.data(), n);

    for (LUCacheEntry& e : luCache) {
        if (e.key != key) continue;
        // Confirm rather than trust the hash, so correctness never depends on it.
        if (std::memcmp(e.K.data(), K.data(), nn * sizeof(double)) != 0) continue;
        e.lastUsed = ++luCacheClock;
        ++luCacheHitCount;
        ++luWindowHits;
        LU_active = e.LU.data();
        p_active = e.perm.data();
        sparse_active = e.sparse.valid ? &e.sparse : nullptr;
        return;
    }

    factorizeLU(n);
    ++luFactorizeCount;

    if (luWindowLookups >= kWindow) {
        if (luWindowHits * 100 < luWindowLookups * kMinHitPercent) {
            luCacheSuspended = true;
            luSuspendCountdown = kRetryAfter;
        }
        luWindowLookups = 0;
        luWindowHits = 0;
    }

    LUCacheEntry* slot = nullptr;
    if (luCache.size() < luCacheMaxEntries) {
        luCache.emplace_back();
        slot = &luCache.back();
        slot->K.resize(nn);
        slot->LU.resize(nn);
        slot->perm.resize((size_t)n);
    } else {
        // Evict least recently used.
        uint64_t oldest = UINT64_MAX;
        for (LUCacheEntry& e : luCache) {
            if (e.lastUsed < oldest) { oldest = e.lastUsed; slot = &e; }
        }
    }

    slot->key = key;
    slot->lastUsed = ++luCacheClock;
    std::memcpy(slot->K.data(), K.data(), nn * sizeof(double));
    std::memcpy(slot->LU.data(), LU_cached.data(), nn * sizeof(double));
    std::memcpy(slot->perm.data(), p_cached.data(), (size_t)n * sizeof(int));
    buildSparseTriangular(n, slot->sparse);

    LU_active = slot->LU.data();
    p_active = slot->perm.data();
    sparse_active = slot->sparse.valid ? &slot->sparse : nullptr;
}

bool CircuitSimulator::factorizeLU(int n) {
    if (n <= 0) return true;

    std::copy(K.begin(), K.end(), LU_cached.begin());
    for (int i = 0; i < n; i++) p_cached[i] = i;

    for (int i = 0; i < n; i++) {
        double maxA = 0.0;
        int maxRow = i;
        for (int k = i; k < n; k++) {
            double absA = std::fabs(LU_cached[k * n + i]);
            if (absA > maxA) {
                maxA = absA;
                maxRow = k;
            }
        }
        if (maxA < 1e-15) {
            LU_cached[i * n + i] += 1e-9;
            maxA = 1e-9;
        }

        if (maxRow != i) {
            std::swap(p_cached[i], p_cached[maxRow]);
            for (int k = 0; k < n; k++) {
                std::swap(LU_cached[i * n + k], LU_cached[maxRow * n + k]);
            }
        }

        double pivotInv = 1.0 / LU_cached[i * n + i];
        for (int j = i + 1; j < n; j++) {
            LU_cached[j * n + i] *= pivotInv;
            double factor = LU_cached[j * n + i];
            for (int k = i + 1; k < n; k++) {
                LU_cached[j * n + k] -= factor * LU_cached[i * n + k];
            }
        }
    }

    return true;
}

bool CircuitSimulator::solveLUSubstitution(int n) {
    if (n <= 0) return true;

    // Normally set by prepareFactorization(); this only guards against a solve
    // being reached before any factorization has been made.
    if (!LU_active || !p_active) {
        factorizeLU(n);
        ++luFactorizeCount;
        LU_active = LU_cached.data();
        p_active = p_cached.data();
        sparse_active = nullptr;
    }

    const int* perm = p_active;
    double* x = x_buf.data();

    for (int i = 0; i < n; i++) {
        x[i] = B[perm[i]];
    }

    if (sparse_active) {
        // Same arithmetic in the same order as below, with the structural zeros
        // omitted, so the result is bit-for-bit the same.
        const SparseTriangular& s = *sparse_active;
        const int* Lptr = s.Lptr.data();
        const int* Lidx = s.Lidx.data();
        const double* Lval = s.Lval.data();
        for (int i = 0; i < n; i++) {
            double acc = x[i];
            const int end = Lptr[i + 1];
            for (int k = Lptr[i]; k < end; ++k) acc -= Lval[k] * x[Lidx[k]];
            x[i] = acc;
        }

        const int* Uptr = s.Uptr.data();
        const int* Uidx = s.Uidx.data();
        const double* Uval = s.Uval.data();
        const double* Udiag = s.Udiag.data();
        for (int i = n - 1; i >= 0; i--) {
            double acc = x[i];
            const int end = Uptr[i + 1];
            for (int k = Uptr[i]; k < end; ++k) acc -= Uval[k] * x[Uidx[k]];
            x[i] = acc / Udiag[i];
        }
    } else {
        const double* LU = LU_active;

        // Forward substitution L*y = b
        for (int i = 0; i < n; i++) {
            double acc = x[i];
            const double* row = LU + (size_t)i * n;
            for (int j = 0; j < i; j++) acc -= row[j] * x[j];
            x[i] = acc;
        }

        // Backward substitution U*x = y
        for (int i = n - 1; i >= 0; i--) {
            double acc = x[i];
            const double* row = LU + (size_t)i * n;
            for (int j = i + 1; j < n; j++) acc -= row[j] * x[j];
            x[i] = acc / row[i];
        }
    }

    std::copy(x_buf.begin(), x_buf.end(), X.begin());
    return true;
}

bool CircuitSimulator::solveLUFast(int n) {
    prepareFactorization(n);
    return solveLUSubstitution(n);
}

void CircuitSimulator::evaluateGateControls(double currentTime, double dtStep) {
    // Gate for every write to per-block state. Always false when probing gates.
    const bool commitState = false;
    const double dtNow = (dtStep > 0.0) ? dtStep : ((config.stepSize > 0.0) ? config.stepSize : 1e-5);

    for (int pass = 0; pass < 2; ++pass) {
        for (auto* fcPtr : fastGateCtrlComps) {
            auto& fc = *fcPtr;
            double val = 0.0;

            if (fc.type == ComponentType::Triangle_Carrier) {
                bool extPhase = (fc.polarity == "external");
                bool extFreq = (fc.vPlotKey == "external");

                double phase_deg = extPhase ? (fc.in1Ptr ? *fc.in1Ptr : 0.0) : fc.delay;
                double freq = extFreq ? (fc.in0Ptr ? *fc.in0Ptr : (fc.freq > 0 ? fc.freq : 10000.0)) : (fc.freq > 0 ? fc.freq : 10000.0);

                double min = fc.minVal;
                double max = fc.maxVal;

                double t_norm = std::fmod(currentTime * freq + phase_deg / 360.0, 1.0);
                if (t_norm < 0.0) t_norm += 1.0;

                val = (t_norm < 0.5) ? min + (max - min) * (t_norm / 0.5) : max - (max - min) * ((t_norm - 0.5) / 0.5);
            }
            else if (fc.type == ComponentType::PWM_Generator) {
                double vMod = fc.in0Ptr ? *fc.in0Ptr : 0.0;
                double fcHz = (fc.freq > 0.0) ? fc.freq : 10000.0;
                double minVal = fc.minVal;
                double maxVal = fc.maxVal;
                if (maxVal <= minVal) maxVal = minVal + 1.0;
                double deadTime = fc.delayDuration;
                double Tc = 1.0 / fcHz;

                double tLocal = std::fmod(currentTime, Tc);
                if (tLocal < 0.0) tLocal += Tc;

                double triVal = (tLocal < Tc / 2.0) 
                                ? minVal + (maxVal - minVal) * (tLocal / (Tc / 2.0))
                                : maxVal - (maxVal - minVal) * ((tLocal - Tc / 2.0) / (Tc / 2.0));

                int targetDirect = (vMod >= triVal) ? 1 : 0;

                if (deadTime > 0.0) {
                    int targetCompl = (targetDirect == 0) ? 1 : 0;
                    if (pass == 0 && commitState && currentTime > fc.lastTime) {
                        if (fc.pwmMasterLastTargetDirect.empty()) {
                            fc.pwmMasterLastTargetDirect.assign(1, 0);
                            fc.pwmMasterLastTargetCompl.assign(1, 0);
                            fc.pwmMasterLastTransDirect.assign(1, 0.0);
                            fc.pwmMasterLastTransCompl.assign(1, 0.0);
                        }
                        if (targetDirect == 1 && fc.pwmMasterLastTargetDirect[0] == 0) {
                            fc.pwmMasterLastTransDirect[0] = currentTime;
                        }
                        if (targetCompl == 1 && fc.pwmMasterLastTargetCompl[0] == 0) {
                            fc.pwmMasterLastTransCompl[0] = currentTime;
                        }
                        fc.pwmMasterLastTargetDirect[0] = targetDirect;
                        fc.pwmMasterLastTargetCompl[0] = targetCompl;
                        fc.lastTime = currentTime;
                    }
                    double transT = (!fc.pwmMasterLastTransDirect.empty()) ? fc.pwmMasterLastTransDirect[0] : 0.0;
                    val = (targetDirect == 1 && (currentTime - transT >= deadTime - 1e-12)) ? 1.0 : 0.0;
                } else {
                    val = (targetDirect == 1) ? 1.0 : 0.0;
                }
            }
            else if (fc.type == ComponentType::PWM_MASTER) {
                int N = fc.numInputs;
                double fcHz = (fc.freq > 0.0) ? fc.freq : 10000.0;
                double deadTime = fc.delayDuration;
                double Tc = 1.0 / fcHz;

                for (int i = 0; i < N; ++i) {
                    double vMod = (fc.pwmMasterInIndices[i] >= 0 && fc.pwmMasterInIndices[i] < (int)flatControlSignals.size()) 
                                  ? flatControlSignals[fc.pwmMasterInIndices[i]] : 0.0;

                    double phaseDeg = fc.pwmMasterPhaseDeg[i];
                    if (fc.pwmMasterPhaseExt[i] && fc.pwmMasterExtPhaseIndices[i] >= 0 && fc.pwmMasterExtPhaseIndices[i] < (int)flatControlSignals.size()) {
                        phaseDeg = flatControlSignals[fc.pwmMasterExtPhaseIndices[i]];
                    }

                    double lOffset = fc.pwmMasterLevelOffset[i];
                    double tOffset = (phaseDeg / 360.0) * Tc;
                    double tLocal = std::fmod(currentTime - tOffset, Tc);
                    if (tLocal < 0.0) tLocal += Tc;

                    double triVal = (tLocal < Tc / 2.0) 
                                    ? (tLocal / (Tc / 2.0)) 
                                    : (1.0 - (tLocal - Tc / 2.0) / (Tc / 2.0));
                    double vCarrier = fc.minVal + (fc.maxVal - fc.minVal) * triVal + lOffset;

                    int targetDirect = (vMod >= vCarrier) ? 1 : 0;
                    int targetCompl = (targetDirect == 0) ? 1 : 0;

                    if (pass == 0 && commitState && currentTime > fc.lastTime) {
                        if (targetDirect == 1 && fc.pwmMasterLastTargetDirect[i] == 0) {
                            fc.pwmMasterLastTransDirect[i] = currentTime;
                        }
                        if (targetCompl == 1 && fc.pwmMasterLastTargetCompl[i] == 0) {
                            fc.pwmMasterLastTransCompl[i] = currentTime;
                        }
                        fc.pwmMasterLastTargetDirect[i] = targetDirect;
                        fc.pwmMasterLastTargetCompl[i] = targetCompl;
                    }

                    double outD = (targetDirect == 1 && (currentTime - fc.pwmMasterLastTransDirect[i] >= deadTime - 1e-12)) ? 1.0 : 0.0;
                    double outC = (targetCompl == 1 && (currentTime - fc.pwmMasterLastTransCompl[i] >= deadTime - 1e-12)) ? 1.0 : 0.0;

                    int dIdx = fc.pwmMasterOutDirectIndices[i];
                    int cIdx = fc.pwmMasterOutComplIndices[i];
                    if (dIdx >= 0 && dIdx < (int)flatControlSignals.size()) flatControlSignals[dIdx] = outD;
                    if (cIdx >= 0 && cIdx < (int)flatControlSignals.size()) flatControlSignals[cIdx] = outC;

                    int chIdx = i + 1;
                    auto itD2 = signalKeyToIdx.find(fc.id + ".OutDirect" + std::to_string(chIdx));
                    if (itD2 != signalKeyToIdx.end() && itD2->second < (int)flatControlSignals.size()) flatControlSignals[itD2->second] = outD;
                    auto itC2 = signalKeyToIdx.find(fc.id + ".OutCompl" + std::to_string(chIdx));
                    if (itC2 != signalKeyToIdx.end() && itC2->second < (int)flatControlSignals.size()) flatControlSignals[itC2->second] = outC;
                    auto itD3 = signalKeyToIdx.find(fc.id + ".Out" + std::to_string(chIdx));
                    if (itD3 != signalKeyToIdx.end() && itD3->second < (int)flatControlSignals.size()) flatControlSignals[itD3->second] = outD;
                }
                if (pass == 0 && commitState && currentTime > fc.lastTime) {
                    fc.lastTime = currentTime;
                }
                val = (N > 0 && fc.pwmMasterOutDirectIndices[0] >= 0 && fc.pwmMasterOutDirectIndices[0] < (int)flatControlSignals.size()) 
                      ? flatControlSignals[fc.pwmMasterOutDirectIndices[0]] : 0.0;
            }
            else if (fc.type == ComponentType::PWM_3PH) {
                double freq = (fc.freq > 0.0) ? fc.freq : 10000.0;
                double period = 1.0 / freq;
                double phaseIn = std::fmod(currentTime, period) / period;
                if (phaseIn < 0.0) phaseIn += 1.0;
                double v_car = (phaseIn < 0.5) ? (4.0 * phaseIn - 1.0) : (3.0 - 4.0 * phaseIn); // -1 to +1 triangle wave

                double va = (fc.inputSigIndices.size() > 0 && fc.inputSigIndices[0] >= 0 && fc.inputSigIndices[0] < (int)flatControlSignals.size()) ? flatControlSignals[fc.inputSigIndices[0]] : (fc.in0Ptr ? *fc.in0Ptr : 0.0);
                double vb = (fc.inputSigIndices.size() > 1 && fc.inputSigIndices[1] >= 0 && fc.inputSigIndices[1] < (int)flatControlSignals.size()) ? flatControlSignals[fc.inputSigIndices[1]] : (fc.in1Ptr ? *fc.in1Ptr : 0.0);
                double vc = (fc.inputSigIndices.size() > 2 && fc.inputSigIndices[2] >= 0 && fc.inputSigIndices[2] < (int)flatControlSignals.size()) ? flatControlSignals[fc.inputSigIndices[2]] : 0.0;

                double outA = (va > v_car) ? 1.0 : 0.0;
                double outB = (vb > v_car) ? 1.0 : 0.0;
                double outC = (vc > v_car) ? 1.0 : 0.0;

                // Ports are registered as G1..G6, gA1..gC2, OutA..OutC, Out1..Out3 (in
                // that order). Writing outputSigIndices[0..2] put phase A/B/C on G1/G2/G3
                // and left OutA/OutB/OutC permanently at 0. Write each port explicitly;
                // the even gates are the complementary lower switches.
                {
                    const double raw[6] = {outA, 1.0 - outA, outB, 1.0 - outB, outC, 1.0 - outC};
                    double g[6];
                    pwm3PhGates(fc, currentTime, fc.delayDuration, commitState && pass == 0, raw, g);
                    writePwm3PhGates(fc, flatControlSignals, g);
                    outA = g[0]; outB = g[2]; outC = g[4];
                }
                val = outA;
            }
            else if (fc.type == ComponentType::SVPWM) {
                double fcHz = (fc.freq > 0.0) ? fc.freq : 10000.0;
                double deadTime = fc.delayDuration;
                double minVal = (fc.minVal != 0.0) ? fc.minVal : -1.0;
                double maxVal = (fc.maxVal != 0.0) ? fc.maxVal : 1.0;
                double Tc = 1.0 / fcHz;

                double in1 = (fc.inputSigIndices.size() > 0 && fc.inputSigIndices[0] >= 0 && fc.inputSigIndices[0] < (int)flatControlSignals.size()) ? flatControlSignals[fc.inputSigIndices[0]] : 0.0;
                double in2 = (fc.inputSigIndices.size() > 1 && fc.inputSigIndices[1] >= 0 && fc.inputSigIndices[1] < (int)flatControlSignals.size()) ? flatControlSignals[fc.inputSigIndices[1]] : 0.0;
                double in3 = (fc.inputSigIndices.size() > 2 && fc.inputSigIndices[2] >= 0 && fc.inputSigIndices[2] < (int)flatControlSignals.size()) ? flatControlSignals[fc.inputSigIndices[2]] : 0.0;

                double vA = 0.0, vB = 0.0, vC = 0.0;
                if (fc.svpwmAlphaBeta) {
                    double valpha = in1;
                    double vbeta = in2;
                    vA = valpha;
                    vB = -0.5 * valpha + (std::sqrt(3.0) / 2.0) * vbeta;
                    vC = -0.5 * valpha - (std::sqrt(3.0) / 2.0) * vbeta;
                } else {
                    vA = in1;
                    vB = in2;
                    vC = in3;
                }

                double v_max = std::max(vA, std::max(vB, vC));
                double v_min = std::min(vA, std::min(vB, vC));
                double v_offset = -0.5 * (v_max + v_min);

                double v_refA = vA + v_offset;
                double v_refB = vB + v_offset;
                double v_refC = vC + v_offset;

                double tLocal = std::fmod(currentTime, Tc);
                if (tLocal < 0.0) tLocal += Tc;
                double triVal = (tLocal < Tc / 2.0) 
                                ? minVal + (maxVal - minVal) * (tLocal / (Tc / 2.0))
                                : maxVal - (maxVal - minVal) * ((tLocal - Tc / 2.0) / (Tc / 2.0));

                double raw_gA1 = (v_refA > triVal) ? 1.0 : 0.0;
                double raw_gA2 = (v_refA <= triVal) ? 1.0 : 0.0;
                double raw_gB1 = (v_refB > triVal) ? 1.0 : 0.0;
                double raw_gB2 = (v_refB <= triVal) ? 1.0 : 0.0;
                double raw_gC1 = (v_refC > triVal) ? 1.0 : 0.0;
                double raw_gC2 = (v_refC <= triVal) ? 1.0 : 0.0;

                if (pass == 0 && commitState && currentTime > fc.lastTime) {
                    if (fc.pwmMasterLastTransDirect.size() < 6) {
                        fc.pwmMasterLastTransDirect.assign(6, -1.0);
                        fc.pwmMasterLastTargetDirect.assign(6, 0);
                    }
                    if (fc.pwmMasterLastTargetDirect[0] > 0 && raw_gA1 <= 0.5) fc.pwmMasterLastTransDirect[0] = currentTime;
                    if (fc.pwmMasterLastTargetDirect[1] > 0 && raw_gA2 <= 0.5) fc.pwmMasterLastTransDirect[1] = currentTime;
                    if (fc.pwmMasterLastTargetDirect[2] > 0 && raw_gB1 <= 0.5) fc.pwmMasterLastTransDirect[2] = currentTime;
                    if (fc.pwmMasterLastTargetDirect[3] > 0 && raw_gB2 <= 0.5) fc.pwmMasterLastTransDirect[3] = currentTime;
                    if (fc.pwmMasterLastTargetDirect[4] > 0 && raw_gC1 <= 0.5) fc.pwmMasterLastTransDirect[4] = currentTime;
                    if (fc.pwmMasterLastTargetDirect[5] > 0 && raw_gC2 <= 0.5) fc.pwmMasterLastTransDirect[5] = currentTime;

                    fc.pwmMasterLastTargetDirect[0] = (raw_gA1 > 0.5) ? 1 : 0;
                    fc.pwmMasterLastTargetDirect[1] = (raw_gA2 > 0.5) ? 1 : 0;
                    fc.pwmMasterLastTargetDirect[2] = (raw_gB1 > 0.5) ? 1 : 0;
                    fc.pwmMasterLastTargetDirect[3] = (raw_gB2 > 0.5) ? 1 : 0;
                    fc.pwmMasterLastTargetDirect[4] = (raw_gC1 > 0.5) ? 1 : 0;
                    fc.pwmMasterLastTargetDirect[5] = (raw_gC2 > 0.5) ? 1 : 0;
                    fc.lastTime = currentTime;
                }

                double gA1 = raw_gA1;
                if (gA1 > 0.5 && deadTime > 0 && fc.pwmMasterLastTransDirect.size() >= 6 && fc.pwmMasterLastTransDirect[1] >= 0 && (currentTime - fc.pwmMasterLastTransDirect[1]) < deadTime) gA1 = 0.0;
                double gA2 = raw_gA2;
                if (gA2 > 0.5 && deadTime > 0 && fc.pwmMasterLastTransDirect.size() >= 6 && fc.pwmMasterLastTransDirect[0] >= 0 && (currentTime - fc.pwmMasterLastTransDirect[0]) < deadTime) gA2 = 0.0;

                double gB1 = raw_gB1;
                if (gB1 > 0.5 && deadTime > 0 && fc.pwmMasterLastTransDirect.size() >= 6 && fc.pwmMasterLastTransDirect[3] >= 0 && (currentTime - fc.pwmMasterLastTransDirect[3]) < deadTime) gB1 = 0.0;
                double gB2 = raw_gB2;
                if (gB2 > 0.5 && deadTime > 0 && fc.pwmMasterLastTransDirect.size() >= 6 && fc.pwmMasterLastTransDirect[2] >= 0 && (currentTime - fc.pwmMasterLastTransDirect[2]) < deadTime) gB2 = 0.0;

                double gC1 = raw_gC1;
                if (gC1 > 0.5 && deadTime > 0 && fc.pwmMasterLastTransDirect.size() >= 6 && fc.pwmMasterLastTransDirect[5] >= 0 && (currentTime - fc.pwmMasterLastTransDirect[5]) < deadTime) gC1 = 0.0;
                double gC2 = raw_gC2;
                if (gC2 > 0.5 && deadTime > 0 && fc.pwmMasterLastTransDirect.size() >= 6 && fc.pwmMasterLastTransDirect[4] >= 0 && (currentTime - fc.pwmMasterLastTransDirect[4]) < deadTime) gC2 = 0.0;

                std::vector<std::pair<std::string, double>> outputs = {
                    {"G1", gA1}, {"G2", gA2}, {"G3", gB1}, {"G4", gB2}, {"G5", gC1}, {"G6", gC2},
                    {"OutA", gA1}, {"OutB", gB1}, {"OutC", gC1},
                    {"gA1", gA1}, {"gA2", gA2}, {"gB1", gB1}, {"gB2", gB2}, {"gC1", gC1}, {"gC2", gC2},
                    {"Out1", gA1}, {"Out2", gB1}, {"Out3", gC1}
                };

                for (const auto& p : outputs) {
                    auto it = signalKeyToIdx.find(fc.id + "." + p.first);
                    if (it != signalKeyToIdx.end() && it->second >= 0 && it->second < (int)flatControlSignals.size()) {
                        flatControlSignals[it->second] = p.second;
                    }
                }
                val = gA1;
            }
            else if (fc.type == ComponentType::PulseGenerator) {
                double p = (fc.period > 0.0) ? fc.period : 0.0001;
                double w = (fc.width > 0.0 && fc.width <= 1.0) ? fc.width : 0.5;
                double d = fc.delay;
                double amp = (fc.amplitude != 0.0) ? fc.amplitude : 1.0;

                double tRel = currentTime - d;
                if (tRel < 0.0) {
                    val = 0.0;
                } else {
                    double phase = std::fmod(tRel, p);
                    if (phase < 0.0) phase += p;
                    val = (phase < p * w) ? amp : 0.0;
                }
            }
            else if (fc.type == ComponentType::Comparator) {
                double inA = fc.in0Ptr ? *fc.in0Ptr : 0.0;
                double inB = fc.in1Ptr ? *fc.in1Ptr : 0.0;
                val = (inA > inB) ? 1.0 : 0.0;
            }
            else if (fc.type == ComponentType::Relay) {
                double inVal = (fc.inputSigIndices.empty() || fc.inputSigIndices[0] < 0) ? (fc.in0Ptr ? *fc.in0Ptr : 0.0) : flatControlSignals[fc.inputSigIndices[0]];
                if (commitState) {
                    if (inVal >= fc.onThresh) fc.relayState = 1;
                    else if (inVal <= fc.offThresh) fc.relayState = 0;
                }
                val = (fc.relayState == 1) ? fc.outValOn : fc.outValOff;
            }
            else if (fc.type == ComponentType::HitCrossing) {
                double inVal = fc.in0Ptr ? *fc.in0Ptr : 0.0;
                double offset = fc.thresholdVal;
                double hit = 0.0;

                if (currentTime <= 0.0) {
                    val = 0.0;
                    fc.stateVal = inVal;
                    fc.nextStateVal = inVal;
                    fc.lastTime = 0.0;
                } else {
                    if (commitState && currentTime > fc.lastTime) {
                        fc.stateVal = fc.nextStateVal;
                        fc.lastTime = currentTime;
                    }
                    double prev = fc.stateVal;
                    if (fc.polarity == "rising") {
                        if (prev < offset && inVal >= offset) hit = 1.0;
                    } else if (fc.polarity == "falling") {
                        if (prev > offset && inVal <= offset) hit = 1.0;
                    } else {
                        if ((prev < offset && inVal >= offset) || (prev > offset && inVal <= offset)) hit = 1.0;
                    }
                    if (pass == 0 && commitState) {
                        fc.nextStateVal = inVal;
                    }
                    val = hit;
                }
            }

            if (fc.iPlotSignalIdx >= 0 && fc.iPlotSignalIdx < (int)flatControlSignals.size()) {
                flatControlSignals[fc.iPlotSignalIdx] = val;
            }
            if (fc.outSignalIdx >= 0 && fc.outSignalIdx < (int)flatControlSignals.size()) {
                flatControlSignals[fc.outSignalIdx] = val;
            }
            if (fc.compSelfSignalIdx >= 0 && fc.compSelfSignalIdx < (int)flatControlSignals.size()) {
                flatControlSignals[fc.compSelfSignalIdx] = val;
            }
        }
    }
}


void CircuitSimulator::evaluateControls(double currentTime, double dtStep, bool commit) {
    // Gate for every write to per-block state. When false this call is a pure
    // evaluation: flatControlSignals is produced, nothing is advanced.
    const bool commitState = commit;
    // The step actually being taken. Under fixed-step operation this equals
    // config.stepSize, so behaviour is unchanged; under variable-step control it is
    // the accepted step and every rate-dependent block below must use it.
    const double dtNow = (dtStep > 0.0) ? dtStep : ((config.stepSize > 0.0) ? config.stepSize : 1e-5);
    for (auto& fc : fastPhysComps) {
        int n1 = fc.n1, n2 = fc.n2;
        double v1 = (n1 >= 0 && n1 < totalDim) ? X[n1] : 0.0;
        double v2 = (n2 >= 0 && n2 < totalDim) ? X[n2] : 0.0;
        double vDiff = v1 - v2;
        if (std::abs(vDiff) < 1e-12 && (fc.type == ComponentType::VoltageSource || fc.type == ComponentType::ACVoltageSource || fc.type == ComponentType::ControlledVoltageSource)) {
            vDiff = fc.val;
        }

        if (fc.vPlotSignalIdx >= 0 && fc.vPlotSignalIdx < (int)flatControlSignals.size()) {
            flatControlSignals[fc.vPlotSignalIdx] = vDiff;
        }

        double iComp = 0.0;
        if (fc.type == ComponentType::Resistor) {
            double Rtotal = liveElementValue(fc, 1e-6) + fc.esr;
            if (Rtotal < 1e-6) Rtotal = 1e-6;
            iComp = vDiff / Rtotal;
        } else if (fc.type == ComponentType::Inductor) {
            if (fc.lIdx >= 0 && fc.lIdx < totalDim) iComp = X[fc.lIdx];
        } else if (fc.type == ComponentType::VoltageSource || fc.type == ComponentType::ACVoltageSource || fc.type == ComponentType::ControlledVoltageSource || fc.type == ComponentType::Ammeter) {
            if (fc.vIdx >= 0 && fc.vIdx < totalDim) iComp = X[fc.vIdx];
        } else if (fc.type == ComponentType::CurrentSource) {
            iComp = fc.val;
        } else if (fc.type == ComponentType::ACCurrentSource) {
            double phaseRad = fc.delay * 3.141592653589793 / 180.0;
            iComp = fc.val * std::sin(2.0 * 3.141592653589793 * fc.freq * currentTime + phaseRad);
        } else if (fc.type == ComponentType::ControlledCurrentSource) {
            // Same signal selection and gain as the MNA stamp, so I_<id> reports the
            // current actually injected.
            double ctrlVal = (fc.ctrlSigPtr && *fc.ctrlSigPtr != 0.0) ? *fc.ctrlSigPtr : (fc.in0Ptr ? *fc.in0Ptr : (fc.ctrlSigPtr ? *fc.ctrlSigPtr : 0.0));
            iComp = fc.gain * ctrlVal;
        } else if (fc.type == ComponentType::Diode) {
            double state = (fc.stateIdx >= 0 && fc.stateIdx < (int)flatDiodeStates.size()) ? flatDiodeStates[fc.stateIdx] : 0.0;
            double R = (state > 0.5) ? fc.Ron : fc.Roff;
            iComp = (state > 0.5) ? ((vDiff - fc.Vvd) / R) : (vDiff / R);
        } else if (fc.type == ComponentType::Winding) {
            // Electrical port current of the gyrator.
            if (fc.wIdx0 >= 0 && fc.wIdx0 < totalDim) iComp = X[fc.wIdx0];
        } else if (fc.type == ComponentType::IGBT || fc.type == ComponentType::GTO ||
                   fc.type == ComponentType::IGCT || fc.type == ComponentType::BJT ||
                   fc.type == ComponentType::IGBTDiode) {
            double state = (fc.stateIdx >= 0 && fc.stateIdx < (int)flatDiodeStates.size()) ? flatDiodeStates[fc.stateIdx] : 0.0;
            double R = (state > 0.5) ? fc.Ron : fc.Roff;
            if (R < 1e-6) R = 1e-6;
            if (state > 0.5) {
                double offset = (vDiff >= 0.0) ? fc.Vvd : -fc.Vvd;
                iComp = (vDiff - offset) / R;
            } else {
                iComp = vDiff / R;
            }
        } else if (fc.type == ComponentType::JFET) {
            double state = (fc.stateIdx >= 0 && fc.stateIdx < (int)flatDiodeStates.size()) ? flatDiodeStates[fc.stateIdx] : 0.0;
            double R = (state > 0.5) ? fc.Ron : fc.Roff;
            if (R < 1e-6) R = 1e-6;
            iComp = vDiff / R;
        } else if (fc.type == ComponentType::Switch) {
            double ctrlVal = fc.ctrlSigPtr ? *fc.ctrlSigPtr : 0.0;
            iComp = vDiff / ((ctrlVal > 0.5) ? fc.Ron : fc.Roff);
        }

        if (fc.iPlotSignalIdx >= 0 && fc.iPlotSignalIdx < (int)flatControlSignals.size()) {
            flatControlSignals[fc.iPlotSignalIdx] = iComp;
        }
        if (fc.outSignalIdx >= 0 && fc.outSignalIdx < (int)flatControlSignals.size()) {
            flatControlSignals[fc.outSignalIdx] = (fc.type == ComponentType::Ammeter) ? iComp : vDiff;
        }
        if (fc.compSelfSignalIdx >= 0 && fc.compSelfSignalIdx < (int)flatControlSignals.size()) {
            flatControlSignals[fc.compSelfSignalIdx] = (fc.type == ComponentType::Ammeter) ? iComp : vDiff;
        }
    }

    for (int pass = 0; pass < 2; ++pass) {
        for (auto& fc : fastCtrlComps) {
            double val = 0.0;

            if (fc.type == ComponentType::Constant) {
                val = fc.val;
            }
            else if (fc.type == ComponentType::Clock) {
                val = currentTime;
            }
            else if (fc.type == ComponentType::InitialCondition) {
                double inVal = fc.in0Ptr ? *fc.in0Ptr : 0.0;
                val = (currentTime == 0.0) ? fc.val : inVal;
            }
            else if (fc.type == ComponentType::Ramp) {
                val = (currentTime >= fc.delay) ? fc.val + fc.gain * (currentTime - fc.delay) : fc.val;
            }
            else if (fc.type == ComponentType::SineWave) {
                val = fc.val + fc.amplitude * std::sin(2.0 * 3.141592653589793 * fc.freq * currentTime + fc.delay * 3.141592653589793 / 180.0);
            }
            else if (fc.type == ComponentType::Step) {
                val = (currentTime >= fc.delay) ? fc.maxVal : fc.minVal;
            }
            else if (fc.type == ComponentType::RandomNumbers) {
                std::mt19937& gen = rngRandomNumbers_;
                std::normal_distribution<double> dist(fc.val, (fc.gain > 0.0 ? fc.gain : 1.0));
                val = dist(gen);
            }
            else if (fc.type == ComponentType::WhiteNoise) {
                std::mt19937& gen = rngWhiteNoise_;
                std::normal_distribution<double> dist(0.0, 1.0);
                double dt = dtNow;
                val = std::sqrt(fc.val / dt) * dist(gen);
            }
            else if (fc.type == ComponentType::Abs) {
                double inVal = fc.in0Ptr ? *fc.in0Ptr : 0.0;
                val = std::abs(inVal);
            }
            else if (fc.type == ComponentType::Sign) {
                double inVal = fc.in0Ptr ? *fc.in0Ptr : 0.0;
                val = (inVal > 0.0) ? 1.0 : ((inVal < 0.0) ? -1.0 : 0.0);
            }
            else if (fc.type == ComponentType::TrigFunction) {
                double inVal = fc.in0Ptr ? *fc.in0Ptr : 0.0;
                double inVal2 = fc.in1Ptr ? *fc.in1Ptr : 0.0;
                // Already lower-cased at setup; avoids a per-step allocation + transform.
                const std::string& f = fc.polarity;
                if (f == "cos") val = std::cos(inVal);
                else if (f == "tan") val = std::tan(inVal);
                else if (f == "asin") val = std::asin(std::max(-1.0, std::min(1.0, inVal)));
                else if (f == "acos") val = std::acos(std::max(-1.0, std::min(1.0, inVal)));
                else if (f == "atan") val = std::atan(inVal);
                else if (f == "atan2") val = std::atan2(inVal, inVal2);
                else if (f == "sinh") val = std::sinh(inVal);
                else if (f == "cosh") val = std::cosh(inVal);
                else if (f == "tanh") val = std::tanh(inVal);
                else if (f == "exp") val = std::exp(inVal);
                else if (f == "log" || f == "ln") val = std::log(std::max(1e-15, inVal));
                else if (f == "log10") val = std::log10(std::max(1e-15, inVal));
                else if (f == "sqrt") val = std::sqrt(std::max(0.0, inVal));
                else if (f == "abs") val = std::abs(inVal);
                else if (f == "square" || f == "sqr") val = inVal * inVal;
                else if (f == "pow") val = std::pow(inVal, inVal2);
                else if (f == "reciprocal" || f == "1/x") val = (std::abs(inVal) < 1e-15) ? 1e15 : (1.0 / inVal);
                else val = std::sin(inVal);
            }
            else if (fc.type == ComponentType::Round) {
                double inVal = fc.in0Ptr ? *fc.in0Ptr : 0.0;
                if (fc.polarity == "floor") val = std::floor(inVal);
                else if (fc.polarity == "ceil") val = std::ceil(inVal);
                else val = std::round(inVal);
            }
            else if (fc.type == ComponentType::MinMax) {
                size_t nIn = std::max((size_t)1, fc.inputSigIndices.size());
                double resVal = 0.0;
                bool first = true;
                bool isMax = (fc.polarity == "max");

                for (size_t i = 0; i < nIn; ++i) {
                    double vIn = 0.0;
                    if (i < fc.inputSigIndices.size() && fc.inputSigIndices[i] >= 0 && fc.inputSigIndices[i] < (int)flatControlSignals.size()) {
                        vIn = flatControlSignals[fc.inputSigIndices[i]];
                    } else if (i == 0 && fc.in0Ptr) {
                        vIn = *fc.in0Ptr;
                    } else if (i == 1 && fc.in1Ptr) {
                        vIn = *fc.in1Ptr;
                    }

                    if (first) {
                        resVal = vIn;
                        first = false;
                    } else {
                        if (isMax) resVal = std::max(resVal, vIn);
                        else resVal = std::min(resVal, vIn);
                    }
                }
                val = resVal;
            }
            else if (fc.type == ComponentType::LUT_1D) {
                double inVal = fc.in0Ptr ? *fc.in0Ptr : 0.0;
                auto parseVec = [](std::string s) -> std::vector<double> {
                    s.erase(std::remove(s.begin(), s.end(), '['), s.end());
                    s.erase(std::remove(s.begin(), s.end(), ']'), s.end());
                    std::stringstream ss(s);
                    std::string token;
                    std::vector<double> vec;
                    while (std::getline(ss, token, ',')) {
                        if (!token.empty()) {
                            try { vec.push_back(std::stod(token)); } catch (...) {}
                        }
                    }
                    return vec;
                };
                std::vector<double> vx = parseVec(fc.polarity);
                std::vector<double> vy = parseVec(fc.vPlotKey);
                if (vx.size() < 2 || vy.size() < vx.size()) {
                    val = vy.empty() ? 0.0 : vy[0];
                } else if (inVal <= vx[0]) {
                    val = vy[0];
                } else if (inVal >= vx.back()) {
                    val = vy[vx.size() - 1];
                } else {
                    size_t idx = 0;
                    for (size_t i = 0; i < vx.size() - 1; ++i) {
                        if (inVal >= vx[i] && inVal <= vx[i + 1]) {
                            idx = i;
                            break;
                        }
                    }
                    double x0 = vx[idx], x1 = vx[idx + 1];
                    double y0 = vy[idx], y1 = vy[idx + 1];
                    val = y0 + (y1 - y0) * (inVal - x0) / (x1 - x0);
                }
            }
            else if (fc.type == ComponentType::Integrator) {
                double inVal = fc.in0Ptr ? *fc.in0Ptr : 0.0;
                double dt = dtNow;
                double gainK = (fc.gain != 0.0) ? fc.gain : 1.0;
                if (currentTime == 0.0) {
                    if (pass == 0 && commitState) fc.stateVal = fc.val;
                } else if (pass == 0 && commitState) {
                    fc.stateVal += gainK * inVal * dt;
                }
                val = fc.stateVal;
            }
            else if (fc.type == ComponentType::Derivative) {
                double inVal = fc.in0Ptr ? *fc.in0Ptr : 0.0;
                double dt = dtNow;
                if (currentTime <= 0.0) {
                    val = 0.0;
                    fc.stateVal = inVal;
                    fc.nextStateVal = inVal;
                    fc.lastTime = 0.0;
                } else {
                    if (commitState && currentTime > fc.lastTime) {
                        fc.stateVal = fc.nextStateVal;
                        fc.lastTime = currentTime;
                    }
                    val = (inVal - fc.stateVal) / dt;
                    if (pass == 0 && commitState) {
                        fc.nextStateVal = inVal;
                    }
                }
            }
            else if (fc.type == ComponentType::TransferFunction) {
                double inVal = fc.in0Ptr ? *fc.in0Ptr : 0.0;
                double dt = dtNow;

                if (currentTime == 0.0) {
                    fc.stateVector.clear();
                }

                auto parseVec = [](std::string s) -> std::vector<double> {
                    std::vector<double> vec;
                    if (s.empty()) return vec;
                    for (char& c : s) {
                        if (c == '[' || c == ']' || c == ',' || c == ';' || c == '\t' || c == '\r' || c == '\n') {
                            c = ' ';
                        }
                    }
                    std::stringstream ss(s);
                    double v = 0.0;
                    while (ss >> v) vec.push_back(v);
                    return vec;
                };

                std::vector<double> num = parseVec(fc.polarity);
                std::vector<double> den = parseVec(fc.vPlotKey);
                double gainK = (fc.gain != 0.0) ? fc.gain : 1.0;
                for (double& nCoeff : num) nCoeff *= gainK;

                if (num.empty()) num = {1.0};
                if (den.empty()) den = {1.0, 1.0};

                size_t n = den.size() - 1;
                if (n < 1) {
                    val = (num[0] / den[0]) * inVal;
                } else {
                    double an = den[0] != 0.0 ? den[0] : 1.0;
                    std::vector<double> denNorm(den.size()), numNorm(den.size(), 0.0);
                    for (size_t i = 0; i < den.size(); ++i) denNorm[i] = den[i] / an;
                    for (size_t i = 0; i < num.size(); ++i) numNorm[numNorm.size() - num.size() + i] = num[i] / an;

                    if (fc.stateVector.size() != n) fc.stateVector.assign(n, 0.0);

                    auto getXDot = [&](const std::vector<double>& xCurr, double uVal) -> std::vector<double> {
                        std::vector<double> xD(n, 0.0);
                        for (size_t k = 0; k < n - 1; ++k) xD[k] = xCurr[k + 1];
                        double sumA = 0.0;
                        for (size_t k = 0; k < n; ++k) sumA += denNorm[n - k] * xCurr[k];
                        xD[n - 1] = -sumA + uVal;
                        return xD;
                    };

                    // 4th-order Runge-Kutta integration for state vector
                    std::vector<double> k1 = getXDot(fc.stateVector, inVal);

                    std::vector<double> x2(n);
                    for (size_t k = 0; k < n; ++k) x2[k] = fc.stateVector[k] + 0.5 * dt * k1[k];
                    std::vector<double> k2 = getXDot(x2, inVal);

                    std::vector<double> x3(n);
                    for (size_t k = 0; k < n; ++k) x3[k] = fc.stateVector[k] + 0.5 * dt * k2[k];
                    std::vector<double> k3 = getXDot(x3, inVal);

                    std::vector<double> x4(n);
                    for (size_t k = 0; k < n; ++k) x4[k] = fc.stateVector[k] + dt * k3[k];
                    std::vector<double> k4 = getXDot(x4, inVal);

                    if (pass == 0 && commitState) {
                        for (size_t k = 0; k < n; ++k) {
                            fc.stateVector[k] += (dt / 6.0) * (k1[k] + 2.0 * k2[k] + 2.0 * k3[k] + k4[k]);
                        }
                    }

                    double bn = numNorm[0];
                    double sumC = 0.0;
                    for (size_t k = 0; k < n; ++k) {
                        sumC += (numNorm[n - k] - bn * denNorm[n - k]) * fc.stateVector[k];
                    }
                    val = sumC + bn * inVal;
                }
            }
            else if (fc.type == ComponentType::ContinuousPID) {
                double inVal = fc.in0Ptr ? *fc.in0Ptr : 0.0;
                // ts > 0 makes this a sampled controller (DISCRETE_PID): the state updates once
                // per ts and the output is held in between. ts = 0 keeps the continuous form.
                const bool sampled = fc.delay > 0.0;
                const double ts = fc.delay;
                const bool sampleNow = !sampled || (currentTime + 1e-12 >= fc.nextSampleTime);
                const bool doCommit = (pass == 0 && commitState && sampleNow);
                double dt = sampled ? ts : dtNow;
                double Kp = fc.gain;
                double Ki = std::atof(fc.vAlphaKey.c_str());
                double Kd = std::atof(fc.vBetaKey.c_str());
                double Tf = (fc.minVal > 0.0) ? fc.minVal : 0.01;

                if (currentTime == 0.0) {
                    if (pass == 0 && commitState) {
                        fc.stateVal = 0.0;
                        fc.filterState = 0.0;
                    }
                } else if (doCommit) {
                    const double newFilter = (Tf / (Tf + dt)) * fc.filterState + (dt / (Tf + dt)) * inVal;
                    const double dI = Ki * inVal * dt;
                    double newI = fc.stateVal + dI;
                    if (fc.pidLimitOutput && fc.pidAntiWindup) {
                        // Clamping anti-windup: the integrator may move outwards only as
                        // far as the value that puts the output on the limit (never
                        // backwards, so a reduced limit does not kick the state).
                        const double rest = Kp * inVal + (Kd / (Tf + dt)) * (inVal - newFilter);
                        if (dI > 0.0) newI = std::max(fc.stateVal, std::min(newI, fc.pidUpper - rest));
                        else if (dI < 0.0) newI = std::min(fc.stateVal, std::max(newI, fc.pidLower - rest));
                    }
                    fc.stateVal = newI;
                    fc.filterState = newFilter;
                }
                double dTerm = (Kd / (Tf + dt)) * (inVal - fc.filterState);
                val = Kp * inVal + fc.stateVal + dTerm;
                if (fc.pidLimitOutput) val = std::min(std::max(val, fc.pidLower), fc.pidUpper);
                if (sampled) {
                    if (doCommit) {
                        fc.prevOut = val;
                        fc.nextSampleTime = currentTime + ts;
                    } else if (!sampleNow) {
                        val = fc.prevOut;   // between samples: hold the last sampled output
                    }
                }
            }
            else if (fc.type == ComponentType::PLL_1PH || fc.type == ComponentType::PLL_3PH) {
                // Phase-locked loop. Theta is aligned so that the (phase-A) input is
                // Vm*sin(Theta). State: fc.stateVal = theta, fc.filterState = PI
                // integrator (rad/s deviation from 2*pi*fn).
                //  PLL_3PH: synchronous-reference-frame PLL. Clarke (amplitude
                //           invariant) then err = alpha*cos(th) + beta*sin(th)
                //           = Vm*sin(theta_grid - th): no 2f ripple for a balanced set.
                //  PLL_1PH: multiplier phase detector, err = v*cos(th)
                //           = (Vm/2)*sin(theta_grid - th) + 2f ripple (as PLL_LOOP).
                // The detector is not normalised, so the loop gains scale with the
                // input amplitude (the textbook form; the defaults suit volt-level inputs).
                constexpr double kTwoPi = 2.0 * 3.141592653589793;
                const double dt = dtNow;
                const double fn = (fc.freq > 0.0) ? fc.freq : 50.0;
                const double w0 = kTwoPi * fn;
                const double Kp = fc.gain;
                const double Ki = fc.maxVal;
                auto sig = [&](size_t k) {
                    int idx = (k < fc.inputSigIndices.size()) ? fc.inputSigIndices[k] : -1;
                    return (idx >= 0 && idx < (int)flatControlSignals.size()) ? flatControlSignals[idx] : 0.0;
                };
                if (currentTime == 0.0 && pass == 0 && commitState) {
                    fc.stateVal = 0.0;
                    fc.filterState = 0.0;
                    fc.pllHasOmega = false;
                }
                const double th = fc.stateVal;
                double err;
                if (fc.type == ComponentType::PLL_3PH) {
                    const double va = sig(0), vb = sig(1), vc = sig(2);
                    const double alpha = (2.0 / 3.0) * (va - 0.5 * vb - 0.5 * vc);
                    const double beta = (vb - vc) / std::sqrt(3.0);
                    err = alpha * std::cos(th) + beta * std::sin(th);
                } else {
                    const double v = fc.in0Ptr ? *fc.in0Ptr : 0.0;
                    err = v * std::cos(th);
                }
                const double omega = w0 + Kp * err + fc.filterState;
                if (pass == 0 && commitState && currentTime > 0.0) {
                    fc.filterState += Ki * err * dt;
                    double thNew = std::fmod(th + omega * dt, kTwoPi);
                    if (thNew < 0.0) thNew += kTwoPi;
                    fc.stateVal = thNew;
                    fc.pllOmega = omega;
                    fc.pllHasOmega = true;
                }
                // Later passes of the same step see the advanced theta, so recomputing
                // omega there biases Freq by Kp*Vm*omega*dt. Report the committed value.
                const double omegaOut = fc.pllHasOmega ? fc.pllOmega : omega;
                const double outs[4] = {th, omegaOut / kTwoPi, std::cos(th), std::sin(th)};
                for (size_t k = 0; k < 4 && k < fc.outputSigIndices.size(); ++k) {
                    int idx = fc.outputSigIndices[k];
                    if (idx >= 0 && idx < (int)flatControlSignals.size()) flatControlSignals[idx] = outs[k];
                }
                val = th;
            }
            else if (fc.type == ComponentType::Delay || fc.type == ComponentType::TransportDelay) {
                double inVal = fc.in0Ptr ? *fc.in0Ptr : 0.0;
                double delayDuration = (fc.delayDuration > 0.0) ? fc.delayDuration : 0.1;
                
                if (commitState && (fc.delayHistory.empty() || currentTime > fc.delayHistory.back().t)) {
                    fc.delayHistory.push_back({currentTime, inVal});
                }
                
                double targetT = currentTime - delayDuration;
                if (fc.delayHistory.empty()) {
                    val = inVal;
                } else if (targetT <= fc.delayHistory.front().t) {
                    val = fc.delayHistory.front().val;
                } else if (targetT >= fc.delayHistory.back().t) {
                    val = fc.delayHistory.back().val;
                } else {
                    size_t idx = 0;
                    for (size_t i = 0; i < fc.delayHistory.size() - 1; ++i) {
                        if (targetT >= fc.delayHistory[i].t && targetT <= fc.delayHistory[i + 1].t) {
                            idx = i;
                            break;
                        }
                    }
                    auto pt0 = fc.delayHistory[idx];
                    auto pt1 = fc.delayHistory[idx + 1];
                    double dtInterval = pt1.t - pt0.t;
                    if (dtInterval > 1e-15) {
                        val = pt0.val + (pt1.val - pt0.val) * (targetT - pt0.t) / dtInterval;
                    } else {
                        val = pt0.val;
                    }
                }
            }
            else if (fc.type == ComponentType::TurnOnDelay) {
                double inVal = fc.in0Ptr ? *fc.in0Ptr : 0.0;
                double delayDuration = (fc.delayDuration > 0.0) ? fc.delayDuration : 0.05;
                bool isHigh = (inVal > 0.5);
                if (commitState) {
                    if (isHigh) {
                        if (!fc.prevInputHigh) {
                            fc.highStartTime = currentTime;
                        }
                    } else {
                        fc.highStartTime = -1.0;
                    }
                    fc.prevInputHigh = isHigh;
                }
                
                if (isHigh && fc.highStartTime >= 0.0 && (currentTime - fc.highStartTime) >= delayDuration) {
                    val = 1.0;
                } else {
                    val = 0.0;
                }
            }
            else if (fc.type == ComponentType::MemoryBlock) {
                double inVal = fc.in0Ptr ? *fc.in0Ptr : 0.0;
                if (currentTime == 0.0) {
                    fc.prevVal = fc.val;
                    fc.currentVal = inVal;
                    fc.lastTime = currentTime;
                } else if (commitState && currentTime > fc.lastTime) {
                    fc.prevVal = fc.currentVal;
                    fc.currentVal = inVal;
                    fc.lastTime = currentTime;
                }
                val = fc.prevVal;
            }
            else if (fc.type == ComponentType::Quantizer) {
                double inVal = (fc.inputSigIndices.size() > 0 && fc.inputSigIndices[0] >= 0 && fc.inputSigIndices[0] < (int)flatControlSignals.size()) ? flatControlSignals[fc.inputSigIndices[0]] : (fc.in0Ptr ? *fc.in0Ptr : 0.0);
                double step = (fc.minVal > 0.0) ? fc.minVal : 0.5;
                double ratio = inVal / step;
                double q = 0.0;
                if (fc.polarity == "floor") q = std::floor(ratio);
                else if (fc.polarity == "ceil") q = std::ceil(ratio);
                else q = std::round(ratio);
                val = q * step;
            }
            else if (fc.type == ComponentType::SignalSwitch) {
                double in1 = fc.in0Ptr ? *fc.in0Ptr : 0.0;
                double ctrl = fc.ctrlSigPtr ? *fc.ctrlSigPtr : 0.0;
                double in2 = fc.in1Ptr ? *fc.in1Ptr : 0.0;
                double thresh = fc.thresholdVal;
                bool pass = false;
                if (fc.polarity == "u2 > threshold") pass = (ctrl > thresh);
                else if (fc.polarity == "u2 != 0") pass = (ctrl != 0.0);
                else pass = (ctrl >= thresh);
                val = pass ? in1 : in2;
            }
            else if (fc.type == ComponentType::ManualSwitch) {
                double in1 = fc.in0Ptr ? *fc.in0Ptr : 0.0;
                double in2 = fc.in1Ptr ? *fc.in1Ptr : 0.0;
                val = (fc.polarity == "Input 1") ? in1 : in2;
            }
            else if (fc.type == ComponentType::MultiportSwitch) {
                double ctrl = fc.ctrlSigPtr ? *fc.ctrlSigPtr : 0.0;
                int ctrlIdx = (int)std::round(ctrl);
                int targetIdx = (fc.polarity == "0-based") ? (ctrlIdx + 1) : ctrlIdx;
                if (targetIdx >= 1 && targetIdx <= (int)fc.inputSigIndices.size()) {
                    int sigIdx = fc.inputSigIndices[targetIdx - 1];
                    val = (sigIdx >= 0 && sigIdx < (int)flatControlSignals.size()) ? flatControlSignals[sigIdx] : 0.0;
                } else {
                    val = 0.0;
                }
            }
            else if (fc.type == ComponentType::HitCrossing) {
                double inVal = fc.in0Ptr ? *fc.in0Ptr : 0.0;
                double offset = fc.thresholdVal;
                double hit = 0.0;

                if (currentTime <= 0.0) {
                    val = 0.0;
                    fc.stateVal = inVal;
                    fc.nextStateVal = inVal;
                    fc.lastTime = 0.0;
                } else {
                    if (commitState && currentTime > fc.lastTime) {
                        fc.stateVal = fc.nextStateVal;
                        fc.lastTime = currentTime;
                    }
                    double prev = fc.stateVal;
                    if (fc.polarity == "rising") {
                        if (prev < offset && inVal >= offset) hit = 1.0;
                    } else if (fc.polarity == "falling") {
                        if (prev > offset && inVal <= offset) hit = 1.0;
                    } else {
                        if ((prev < offset && inVal >= offset) || (prev > offset && inVal <= offset)) hit = 1.0;
                    }
                    if (pass == 0 && commitState) {
                        fc.nextStateVal = inVal;
                    }
                    val = hit;
                }
            }
            else if (fc.type == ComponentType::Saturation) {
                double inVal = (fc.inputSigIndices.empty() || fc.inputSigIndices[0] < 0) ? (fc.in0Ptr ? *fc.in0Ptr : 0.0) : flatControlSignals[fc.inputSigIndices[0]];
                val = std::max(fc.minVal, std::min(fc.maxVal, inVal));
            }
            else if (fc.type == ComponentType::DeadZone) {
                double inVal = (fc.inputSigIndices.empty() || fc.inputSigIndices[0] < 0) ? (fc.in0Ptr ? *fc.in0Ptr : 0.0) : flatControlSignals[fc.inputSigIndices[0]];
                if (inVal > fc.maxVal) val = inVal - fc.maxVal;
                else if (inVal < fc.minVal) val = inVal - fc.minVal;
                else val = 0.0;
            }
            else if (fc.type == ComponentType::RateLimiter) {
                double inVal = fc.in0Ptr ? *fc.in0Ptr : 0.0;
                double dt = dtNow;
                if (currentTime == 0.0) {
                    fc.prevOut = inVal;
                    val = inVal;
                } else {
                    double rate = (inVal - fc.prevOut) / dt;
                    double clampedRate = std::max(fc.rateDown, std::min(fc.rateUp, rate));
                    val = fc.prevOut + clampedRate * dt;
                    if (commitState && currentTime > fc.lastTime) {
                        fc.prevOut = val;
                        fc.lastTime = currentTime;
                    }
                }
            }
            else if (fc.type == ComponentType::Relay) {
                double inVal = (fc.inputSigIndices.empty() || fc.inputSigIndices[0] < 0) ? (fc.in0Ptr ? *fc.in0Ptr : 0.0) : flatControlSignals[fc.inputSigIndices[0]];
                if (commitState) {
                    if (inVal >= fc.onThresh) fc.relayState = 1;
                    else if (inVal <= fc.offThresh) fc.relayState = 0;
                }
                val = (fc.relayState == 1) ? fc.outValOn : fc.outValOff;
            }
            else if (fc.type == ComponentType::Comparator) {
                double inA = fc.in0Ptr ? *fc.in0Ptr : 0.0;
                double inB = fc.in1Ptr ? *fc.in1Ptr : 0.0;
                val = (inA > inB) ? 1.0 : 0.0;
            }
            else if (fc.type == ComponentType::NOT_Gate) {
                double inVal = fc.in0Ptr ? *fc.in0Ptr : 0.0;
                val = (inVal <= 0.5) ? 1.0 : 0.0;
            }
            else if (fc.type == ComponentType::AND_Gate) {
                double in1 = fc.in0Ptr ? *fc.in0Ptr : 0.0;
                double in2 = fc.in1Ptr ? *fc.in1Ptr : 0.0;
                val = (in1 > 0.5 && in2 > 0.5) ? 1.0 : 0.0;
            }
            else if (fc.type == ComponentType::OR_Gate) {
                double in1 = fc.in0Ptr ? *fc.in0Ptr : 0.0;
                double in2 = fc.in1Ptr ? *fc.in1Ptr : 0.0;
                val = (in1 > 0.5 || in2 > 0.5) ? 1.0 : 0.0;
            }
            else if (fc.type == ComponentType::LogicOp) {
                double in1 = (fc.inputSigIndices.size() > 0 && fc.inputSigIndices[0] >= 0 && fc.inputSigIndices[0] < (int)flatControlSignals.size()) ? flatControlSignals[fc.inputSigIndices[0]] : (fc.in0Ptr ? *fc.in0Ptr : 0.0);
                double in2 = (fc.inputSigIndices.size() > 1 && fc.inputSigIndices[1] >= 0 && fc.inputSigIndices[1] < (int)flatControlSignals.size()) ? flatControlSignals[fc.inputSigIndices[1]] : (fc.in1Ptr ? *fc.in1Ptr : 0.0);
                const std::string& op = fc.polarity;
                bool a = (in1 > 0.5), b = (in2 > 0.5);
                if (op == "AND") val = (a && b) ? 1.0 : 0.0;
                else if (op == "OR") val = (a || b) ? 1.0 : 0.0;
                else if (op == "XOR") val = (a != b) ? 1.0 : 0.0;
                else if (op == "NAND") val = !(a && b) ? 1.0 : 0.0;
                else if (op == "NOR") val = !(a || b) ? 1.0 : 0.0;
                else if (op == "NXOR" || op == "XNOR") val = (a == b) ? 1.0 : 0.0;
                else if (op == "NOT") val = (!a) ? 1.0 : 0.0;
                else val = 0.0;

                std::vector<std::pair<std::string, double>> outputs = {
                    {"Out", val}, {"out", val}, {"Y", val}, {"y", val}, {"Out1", val}
                };
                for (const auto& p : outputs) {
                    auto it = signalKeyToIdx.find(fc.id + "." + p.first);
                    if (it != signalKeyToIdx.end() && it->second >= 0 && it->second < (int)flatControlSignals.size()) {
                        flatControlSignals[it->second] = p.second;
                    }
                }
            }
            else if (fc.type == ComponentType::BitwiseOp) {
                int in1 = (int)(fc.in0Ptr ? *fc.in0Ptr : 0.0);
                int in2 = (int)(fc.in1Ptr ? *fc.in1Ptr : 0.0);
                in1 &= (int)fc.bitMask;
                in2 &= (int)fc.bitMask;
                const std::string& op = fc.polarity;
                if (op == "AND") val = (double)(in1 & in2);
                else if (op == "OR") val = (double)(in1 | in2);
                else if (op == "XOR") val = (double)(in1 ^ in2);
                else if (op == "NOT") val = (double)(~in1);
                else if (op == "NAND") val = (double)(~(in1 & in2));
                else if (op == "NOR") val = (double)(~(in1 | in2));
                else if (op == "SHL") val = (double)(in1 << in2);
                else if (op == "SHR") val = (double)(in1 >> in2);
                else val = 0.0;
            }
            else if (fc.type == ComponentType::RelationalOp) {
                double in1 = fc.in0Ptr ? *fc.in0Ptr : 0.0;
                double in2 = fc.in1Ptr ? *fc.in1Ptr : 0.0;
                const std::string& op = fc.polarity;
                if (op == "==" || op == "=") val = (std::abs(in1 - in2) < 1e-12) ? 1.0 : 0.0;
                else if (op == "!=" || op == "~=") val = (std::abs(in1 - in2) >= 1e-12) ? 1.0 : 0.0;
                else if (op == "<") val = (in1 < in2) ? 1.0 : 0.0;
                else if (op == "<=") val = (in1 <= in2) ? 1.0 : 0.0;
                else if (op == ">") val = (in1 > in2) ? 1.0 : 0.0;
                else if (op == ">=") val = (in1 >= in2) ? 1.0 : 0.0;
                else val = 0.0;
            }
            else if (fc.type == ComponentType::CompareToConstant) {
                double inVal = (fc.inputSigIndices.size() > 0 && fc.inputSigIndices[0] >= 0 && fc.inputSigIndices[0] < (int)flatControlSignals.size()) ? flatControlSignals[fc.inputSigIndices[0]] : (fc.in0Ptr ? *fc.in0Ptr : 0.0);
                double cVal = fc.thresholdVal;
                const std::string& op = fc.polarity;
                if (op == "==" || op == "=") val = (std::abs(inVal - cVal) < 1e-12) ? 1.0 : 0.0;
                else if (op == "!=" || op == "~=") val = (std::abs(inVal - cVal) >= 1e-12) ? 1.0 : 0.0;
                else if (op == "<") val = (inVal < cVal) ? 1.0 : 0.0;
                else if (op == "<=") val = (inVal <= cVal) ? 1.0 : 0.0;
                else if (op == ">") val = (inVal > cVal) ? 1.0 : 0.0;
                else if (op == ">=") val = (inVal >= cVal) ? 1.0 : 0.0;
                else val = 0.0;

                std::vector<std::pair<std::string, double>> outputs = {
                    {"Out", val}, {"out", val}, {"Y", val}, {"y", val}, {"Out1", val}
                };
                for (const auto& p : outputs) {
                    auto it = signalKeyToIdx.find(fc.id + "." + p.first);
                    if (it != signalKeyToIdx.end() && it->second >= 0 && it->second < (int)flatControlSignals.size()) {
                        flatControlSignals[it->second] = p.second;
                    }
                }
            }
            else if (fc.type == ComponentType::EdgeDetect) {
                double inVal = (fc.inputSigIndices.size() > 0 && fc.inputSigIndices[0] >= 0 && fc.inputSigIndices[0] < (int)flatControlSignals.size()) ? flatControlSignals[fc.inputSigIndices[0]] : (fc.in0Ptr ? *fc.in0Ptr : 0.0);
                double pulseW = (fc.pulseDuration > 0.0) ? fc.pulseDuration : 1e-3;

                if (currentTime == 0.0) {
                    fc.prevVal = inVal;
                    fc.edgeActive = false;
                    fc.triggerTime = -1.0;
                }

                bool detected = false;
                if (fc.edgeMode == "falling") {
                    detected = (fc.prevVal > 0.5 && inVal <= 0.5);
                } else if (fc.edgeMode == "either") {
                    detected = ((fc.prevVal <= 0.5 && inVal > 0.5) || (fc.prevVal > 0.5 && inVal <= 0.5));
                } else {
                    detected = (fc.prevVal <= 0.5 && inVal > 0.5);
                }

                if (commitState && detected && !fc.edgeActive) {
                    fc.edgeActive = true;
                    fc.triggerTime = currentTime;
                }
                if (commitState && fc.edgeActive && fc.triggerTime >= 0.0 &&
                    (currentTime - fc.triggerTime) >= pulseW - 1e-12) {
                    fc.edgeActive = false;
                }
                if (commitState && currentTime > fc.lastTime) {
                    fc.prevVal = inVal;
                    fc.lastTime = currentTime;
                }
                val = fc.edgeActive ? 1.0 : 0.0;

                std::vector<std::pair<std::string, double>> outputs = {
                    {"Out", val}, {"out", val}, {"Y", val}, {"y", val}, {"Out1", val}
                };
                for (const auto& p : outputs) {
                    auto it = signalKeyToIdx.find(fc.id + "." + p.first);
                    if (it != signalKeyToIdx.end() && it->second >= 0 && it->second < (int)flatControlSignals.size()) {
                        flatControlSignals[it->second] = p.second;
                    }
                }
            }
            else if (fc.type == ComponentType::Monostable || fc.type == ComponentType::Monoflop) {
                double inVal = fc.in0Ptr ? *fc.in0Ptr : 0.0;
                double dur = (fc.pulseDuration > 0.0) ? fc.pulseDuration : 0.1;
                bool detected = false;
                if (fc.edgeMode == "rising") detected = (fc.prevVal <= 0.5 && inVal > 0.5);
                else if (fc.edgeMode == "falling") detected = (fc.prevVal > 0.5 && inVal <= 0.5);
                else detected = ((fc.prevVal <= 0.5 && inVal > 0.5) || (fc.prevVal > 0.5 && inVal <= 0.5));
                if (commitState && detected) {
                    if (!fc.edgeActive || fc.retriggerable) {
                        fc.edgeActive = true;
                        fc.triggerTime = currentTime;
                    }
                }
                if (commitState && fc.edgeActive && fc.triggerTime >= 0.0 &&
                    (currentTime - fc.triggerTime) >= dur - 1e-11) {
                    fc.edgeActive = false;
                }
                if (commitState && currentTime > fc.lastTime) {
                    fc.prevVal = inVal;
                    fc.lastTime = currentTime;
                }
                val = fc.edgeActive ? 1.0 : 0.0;
            }
            else if (fc.type == ComponentType::DFlipFlop) {
                double clkVal = fc.ctrlSigPtr ? *fc.ctrlSigPtr : 0.0;
                double dVal = fc.in0Ptr ? *fc.in0Ptr : 0.0;
                bool edgeDetected = false;
                if (fc.edgeMode == "rising") edgeDetected = (fc.prev_clk <= 0.5 && clkVal > 0.5);
                else edgeDetected = (fc.prev_clk > 0.5 && clkVal <= 0.5);
                if (commitState && edgeDetected) {
                    fc.q_state = (dVal > 0.5) ? 1.0 : 0.0;
                }
                if (commitState && currentTime > fc.lastTime) {
                    fc.prev_clk = clkVal;
                    fc.lastTime = currentTime;
                }
                // Write Q and Q_bar
                val = fc.q_state;
                int outIdx = fc.outSignalIdx;
                if (outIdx >= 0 && outIdx < (int)flatControlSignals.size()) {
                    flatControlSignals[outIdx] = fc.q_state;
                }
                // Q_bar output via outputSigIndices[1] if present
                if (fc.outputSigIndices.size() > 1) {
                    int qBarIdx = fc.outputSigIndices[1];
                    if (qBarIdx >= 0 && qBarIdx < (int)flatControlSignals.size()) {
                        flatControlSignals[qBarIdx] = (fc.q_state > 0.5) ? 0.0 : 1.0;
                    }
                }
            }
            else if (fc.type == ComponentType::JKFlipFlop) {
                double clkVal = fc.ctrlSigPtr ? *fc.ctrlSigPtr : 0.0;
                double jVal = fc.in0Ptr ? *fc.in0Ptr : 0.0;
                double kVal = fc.in1Ptr ? *fc.in1Ptr : 0.0;
                bool edgeDetected = false;
                if (fc.edgeMode == "rising") edgeDetected = (fc.prev_clk <= 0.5 && clkVal > 0.5);
                else edgeDetected = (fc.prev_clk > 0.5 && clkVal <= 0.5);
                if (commitState && edgeDetected) {
                    bool J = (jVal > 0.5), K = (kVal > 0.5);
                    if (J && K) fc.q_state = (fc.q_state > 0.5) ? 0.0 : 1.0; // Toggle
                    else if (J) fc.q_state = 1.0;
                    else if (K) fc.q_state = 0.0;
                    // else hold
                }
                if (commitState && currentTime > fc.lastTime) {
                    fc.prev_clk = clkVal;
                    fc.lastTime = currentTime;
                }
                val = fc.q_state;
                int outIdx = fc.outSignalIdx;
                if (outIdx >= 0 && outIdx < (int)flatControlSignals.size()) {
                    flatControlSignals[outIdx] = fc.q_state;
                }
                if (fc.outputSigIndices.size() > 1) {
                    int qBarIdx = fc.outputSigIndices[1];
                    if (qBarIdx >= 0 && qBarIdx < (int)flatControlSignals.size()) {
                        flatControlSignals[qBarIdx] = (fc.q_state > 0.5) ? 0.0 : 1.0;
                    }
                }
            }
            else if (fc.type == ComponentType::ShiftReg) {
                double clkVal = fc.ctrlSigPtr ? *fc.ctrlSigPtr : 0.0;
                double inVal = fc.in0Ptr ? *fc.in0Ptr : 0.0;
                bool edgeDetected = (fc.prev_clk <= 0.5 && clkVal > 0.5);
                if (edgeDetected && commitState && currentTime > fc.lastTime) {
                    // Shift right, push new input at front
                    for (int i = (int)fc.shiftBuffer.size() - 1; i > 0; --i) {
                        fc.shiftBuffer[i] = fc.shiftBuffer[i - 1];
                    }
                    fc.shiftBuffer[0] = inVal;
                    fc.prev_clk = clkVal;
                    fc.lastTime = currentTime;
                } else if (commitState && currentTime > fc.lastTime) {
                    fc.prev_clk = clkVal;
                    fc.lastTime = currentTime;
                }
                val = fc.shiftBuffer.empty() ? 0.0 : fc.shiftBuffer.back();
            }
            else if (fc.type == ComponentType::CombLogic) {
                // Combinational truth table: outputs 1 if input pattern matches any row in truth_table param
                double in1 = fc.in0Ptr ? *fc.in0Ptr : 0.0;
                double in2 = fc.in1Ptr ? *fc.in1Ptr : 0.0;
                int a = (in1 > 0.5) ? 1 : 0, b = (in2 > 0.5) ? 1 : 0;
                val = 0.0;
                // truth_table param format: "00:0,01:1,10:1,11:0" or just "AND"
                // Fall back to AND for unspecified
                if (fc.polarity.empty() || fc.polarity == "AND") {
                    val = (a && b) ? 1.0 : 0.0;
                } else {
                    // Parse "AB:Out" pairs
                    const std::string& tt = fc.polarity;
                    std::istringstream ss(tt);
                    std::string tok;
                    std::string inPat = std::to_string(a) + std::to_string(b);
                    while (std::getline(ss, tok, ',')) {
                        auto colon = tok.find(':');
                        if (colon != std::string::npos) {
                            std::string pat = tok.substr(0, colon);
                            std::string outS = tok.substr(colon + 1);
                            if (pat == inPat) { val = std::stod(outS); break; }
                        }
                    }
                }
            }
            else if (fc.type == ComponentType::Clarke) {
                double va = (fc.inputSigIndices.size() > 0 && fc.inputSigIndices[0] >= 0 && fc.inputSigIndices[0] < (int)flatControlSignals.size()) ? flatControlSignals[fc.inputSigIndices[0]] : (fc.in0Ptr ? *fc.in0Ptr : 0.0);
                double vb = (fc.inputSigIndices.size() > 1 && fc.inputSigIndices[1] >= 0 && fc.inputSigIndices[1] < (int)flatControlSignals.size()) ? flatControlSignals[fc.inputSigIndices[1]] : (fc.in1Ptr ? *fc.in1Ptr : 0.0);
                double vc = (fc.inputSigIndices.size() > 2 && fc.inputSigIndices[2] >= 0 && fc.inputSigIndices[2] < (int)flatControlSignals.size()) ? flatControlSignals[fc.inputSigIndices[2]] : 0.0;

                double alpha = (2.0 * va - vb - vc) / 3.0;
                double beta = (vb - vc) / 1.7320508075688772;

                std::vector<std::pair<std::string, double>> outputs = {
                    {"Alpha", alpha}, {"Valpha", alpha}, {"alpha", alpha}, {"Out1", alpha}, {"OutA", alpha},
                    {"Beta", beta}, {"Vbeta", beta}, {"beta", beta}, {"Out2", beta}, {"OutB", beta}
                };

                for (const auto& p : outputs) {
                    auto it = signalKeyToIdx.find(fc.id + "." + p.first);
                    if (it != signalKeyToIdx.end() && it->second >= 0 && it->second < (int)flatControlSignals.size()) {
                        flatControlSignals[it->second] = p.second;
                    }
                }
                val = alpha;
            }
            else if (fc.type == ComponentType::Clarke) {
                double va = (fc.inputSigIndices.size() > 0 && fc.inputSigIndices[0] >= 0 && fc.inputSigIndices[0] < (int)flatControlSignals.size()) ? flatControlSignals[fc.inputSigIndices[0]] : (fc.in0Ptr ? *fc.in0Ptr : 0.0);
                double vb = (fc.inputSigIndices.size() > 1 && fc.inputSigIndices[1] >= 0 && fc.inputSigIndices[1] < (int)flatControlSignals.size()) ? flatControlSignals[fc.inputSigIndices[1]] : (fc.in1Ptr ? *fc.in1Ptr : 0.0);
                double vc = (fc.inputSigIndices.size() > 2 && fc.inputSigIndices[2] >= 0 && fc.inputSigIndices[2] < (int)flatControlSignals.size()) ? flatControlSignals[fc.inputSigIndices[2]] : 0.0;

                double alpha = (2.0 * va - vb - vc) / 3.0;
                double beta = (vb - vc) / 1.7320508075688772;

                std::vector<std::pair<std::string, double>> outputs = {
                    {"Alpha", alpha}, {"Valpha", alpha}, {"alpha", alpha}, {"Out1", alpha}, {"OutA", alpha},
                    {"Beta", beta}, {"Vbeta", beta}, {"beta", beta}, {"Out2", beta}, {"OutB", beta}
                };

                for (const auto& p : outputs) {
                    auto it = signalKeyToIdx.find(fc.id + "." + p.first);
                    if (it != signalKeyToIdx.end() && it->second >= 0 && it->second < (int)flatControlSignals.size()) {
                        flatControlSignals[it->second] = p.second;
                    }
                }
                val = alpha;
            }
            else if (fc.type == ComponentType::InvClarke) {
                double alpha = (fc.inputSigIndices.size() > 0 && fc.inputSigIndices[0] >= 0 && fc.inputSigIndices[0] < (int)flatControlSignals.size()) ? flatControlSignals[fc.inputSigIndices[0]] : (fc.in0Ptr ? *fc.in0Ptr : 0.0);
                double beta = (fc.inputSigIndices.size() > 1 && fc.inputSigIndices[1] >= 0 && fc.inputSigIndices[1] < (int)flatControlSignals.size()) ? flatControlSignals[fc.inputSigIndices[1]] : (fc.in1Ptr ? *fc.in1Ptr : 0.0);

                double va = alpha;
                double vb = -0.5 * alpha + (1.7320508075688772 / 2.0) * beta;
                double vc = -0.5 * alpha - (1.7320508075688772 / 2.0) * beta;

                std::vector<std::pair<std::string, double>> outputs = {
                    {"A", va}, {"Va", va}, {"a", va}, {"OutA", va}, {"Out1", va},
                    {"B", vb}, {"Vb", vb}, {"b", vb}, {"OutB", vb}, {"Out2", vb},
                    {"C", vc}, {"Vc", vc}, {"c", vc}, {"OutC", vc}, {"Out3", vc}
                };
                for (const auto& p : outputs) {
                    auto it = signalKeyToIdx.find(fc.id + "." + p.first);
                    if (it != signalKeyToIdx.end() && it->second >= 0 && it->second < (int)flatControlSignals.size()) {
                        flatControlSignals[it->second] = p.second;
                    }
                }
                val = va;
            }
            else if (fc.type == ComponentType::Park) {
                double alpha = (fc.inputSigIndices.size() > 0 && fc.inputSigIndices[0] >= 0 && fc.inputSigIndices[0] < (int)flatControlSignals.size()) ? flatControlSignals[fc.inputSigIndices[0]] : (fc.in0Ptr ? *fc.in0Ptr : 0.0);
                double beta = (fc.inputSigIndices.size() > 1 && fc.inputSigIndices[1] >= 0 && fc.inputSigIndices[1] < (int)flatControlSignals.size()) ? flatControlSignals[fc.inputSigIndices[1]] : (fc.in1Ptr ? *fc.in1Ptr : 0.0);
                double theta = (fc.inputSigIndices.size() > 2 && fc.inputSigIndices[2] >= 0 && fc.inputSigIndices[2] < (int)flatControlSignals.size()) ? flatControlSignals[fc.inputSigIndices[2]] : 0.0;

                double va = (fc.inputSigIndices.size() > 3 && fc.inputSigIndices[3] >= 0 && fc.inputSigIndices[3] < (int)flatControlSignals.size()) ? flatControlSignals[fc.inputSigIndices[3]] : 0.0;
                double vb = (fc.inputSigIndices.size() > 4 && fc.inputSigIndices[4] >= 0 && fc.inputSigIndices[4] < (int)flatControlSignals.size()) ? flatControlSignals[fc.inputSigIndices[4]] : 0.0;
                double vc = (fc.inputSigIndices.size() > 5 && fc.inputSigIndices[5] >= 0 && fc.inputSigIndices[5] < (int)flatControlSignals.size()) ? flatControlSignals[fc.inputSigIndices[5]] : 0.0;

                if (alpha == 0.0 && beta == 0.0 && (va != 0.0 || vb != 0.0 || vc != 0.0)) {
                    alpha = (2.0 * va - vb - vc) / 3.0;
                    beta = (vb - vc) / 1.7320508075688772;
                }

                double cosT = std::cos(theta);
                double sinT = std::sin(theta);

                double vd = alpha * cosT + beta * sinT;
                double vq = -alpha * sinT + beta * cosT;

                std::vector<std::pair<std::string, double>> outputs = {
                    {"d", vd}, {"Vd", vd}, {"Direct", vd}, {"OutD", vd}, {"Out1", vd}, {"D", vd},
                    {"q", vq}, {"Vq", vq}, {"Quadrature", vq}, {"OutQ", vq}, {"Out2", vq}, {"Q", vq}
                };
                for (const auto& p : outputs) {
                    auto it = signalKeyToIdx.find(fc.id + "." + p.first);
                    if (it != signalKeyToIdx.end() && it->second >= 0 && it->second < (int)flatControlSignals.size()) {
                        flatControlSignals[it->second] = p.second;
                    }
                }
                val = vd;
            }
            else if (fc.type == ComponentType::InvPark) {
                double vd = (fc.inputSigIndices.size() > 0 && fc.inputSigIndices[0] >= 0 && fc.inputSigIndices[0] < (int)flatControlSignals.size()) ? flatControlSignals[fc.inputSigIndices[0]] : (fc.in0Ptr ? *fc.in0Ptr : 0.0);
                double vq = (fc.inputSigIndices.size() > 1 && fc.inputSigIndices[1] >= 0 && fc.inputSigIndices[1] < (int)flatControlSignals.size()) ? flatControlSignals[fc.inputSigIndices[1]] : (fc.in1Ptr ? *fc.in1Ptr : 0.0);
                double theta = (fc.inputSigIndices.size() > 2 && fc.inputSigIndices[2] >= 0 && fc.inputSigIndices[2] < (int)flatControlSignals.size()) ? flatControlSignals[fc.inputSigIndices[2]] : 0.0;

                double cosT = std::cos(theta);
                double sinT = std::sin(theta);

                double alpha = vd * cosT - vq * sinT;
                double beta = vd * sinT + vq * cosT;

                double va = alpha;
                double vb = -0.5 * alpha + (1.7320508075688772 / 2.0) * beta;
                double vc = -0.5 * alpha - (1.7320508075688772 / 2.0) * beta;

                std::vector<std::pair<std::string, double>> outputs = {
                    {"Alpha", alpha}, {"Valpha", alpha}, {"alpha", alpha}, {"Out1", alpha},
                    {"Beta", beta}, {"Vbeta", beta}, {"beta", beta}, {"Out2", beta},
                    {"A", va}, {"Va", va}, {"a", va},
                    {"B", vb}, {"Vb", vb}, {"b", vb},
                    {"C", vc}, {"Vc", vc}, {"c", vc}
                };
                for (const auto& p : outputs) {
                    auto it = signalKeyToIdx.find(fc.id + "." + p.first);
                    if (it != signalKeyToIdx.end() && it->second >= 0 && it->second < (int)flatControlSignals.size()) {
                        flatControlSignals[it->second] = p.second;
                    }
                }
                val = alpha;
            }
            else if (fc.type == ComponentType::DqToAbc) {
                double vd = (fc.inputSigIndices.size() > 0 && fc.inputSigIndices[0] >= 0 && fc.inputSigIndices[0] < (int)flatControlSignals.size()) ? flatControlSignals[fc.inputSigIndices[0]] : (fc.in0Ptr ? *fc.in0Ptr : 0.0);
                double vq = (fc.inputSigIndices.size() > 1 && fc.inputSigIndices[1] >= 0 && fc.inputSigIndices[1] < (int)flatControlSignals.size()) ? flatControlSignals[fc.inputSigIndices[1]] : (fc.in1Ptr ? *fc.in1Ptr : 0.0);
                double theta = (fc.inputSigIndices.size() > 2 && fc.inputSigIndices[2] >= 0 && fc.inputSigIndices[2] < (int)flatControlSignals.size()) ? flatControlSignals[fc.inputSigIndices[2]] : 0.0;

                double cosT = std::cos(theta);
                double sinT = std::sin(theta);

                double alpha = vd * cosT - vq * sinT;
                double beta = vd * sinT + vq * cosT;

                double va = alpha;
                double vb = -0.5 * alpha + (1.7320508075688772 / 2.0) * beta;
                double vc = -0.5 * alpha - (1.7320508075688772 / 2.0) * beta;

                std::vector<std::pair<std::string, double>> outputs = {
                    {"A", va}, {"Va", va}, {"a", va}, {"OutA", va}, {"Out1", va},
                    {"B", vb}, {"Vb", vb}, {"b", vb}, {"OutB", vb}, {"Out2", vb},
                    {"C", vc}, {"Vc", vc}, {"c", vc}, {"OutC", vc}, {"Out3", vc}
                };
                for (const auto& p : outputs) {
                    auto it = signalKeyToIdx.find(fc.id + "." + p.first);
                    if (it != signalKeyToIdx.end() && it->second >= 0 && it->second < (int)flatControlSignals.size()) {
                        flatControlSignals[it->second] = p.second;
                    }
                }
                val = va;
            }
            else if (fc.type == ComponentType::AbcToDq) {
                double va = (fc.inputSigIndices.size() > 0 && fc.inputSigIndices[0] >= 0 && fc.inputSigIndices[0] < (int)flatControlSignals.size()) ? flatControlSignals[fc.inputSigIndices[0]] : (fc.in0Ptr ? *fc.in0Ptr : 0.0);
                double vb = (fc.inputSigIndices.size() > 1 && fc.inputSigIndices[1] >= 0 && fc.inputSigIndices[1] < (int)flatControlSignals.size()) ? flatControlSignals[fc.inputSigIndices[1]] : (fc.in1Ptr ? *fc.in1Ptr : 0.0);
                double vc = (fc.inputSigIndices.size() > 2 && fc.inputSigIndices[2] >= 0 && fc.inputSigIndices[2] < (int)flatControlSignals.size()) ? flatControlSignals[fc.inputSigIndices[2]] : 0.0;
                double theta = (fc.inputSigIndices.size() > 3 && fc.inputSigIndices[3] >= 0 && fc.inputSigIndices[3] < (int)flatControlSignals.size()) ? flatControlSignals[fc.inputSigIndices[3]] : 0.0;

                double alpha = (2.0 * va - vb - vc) / 3.0;
                double beta = (vb - vc) / 1.7320508075688772;

                double cosT = std::cos(theta);
                double sinT = std::sin(theta);

                double vd = alpha * cosT + beta * sinT;
                double vq = -alpha * sinT + beta * cosT;

                std::vector<std::pair<std::string, double>> outputs = {
                    {"d", vd}, {"Vd", vd}, {"Direct", vd}, {"OutD", vd}, {"Out1", vd}, {"D", vd},
                    {"q", vq}, {"Vq", vq}, {"Quadrature", vq}, {"OutQ", vq}, {"Out2", vq}, {"Q", vq}
                };
                for (const auto& p : outputs) {
                    auto it = signalKeyToIdx.find(fc.id + "." + p.first);
                    if (it != signalKeyToIdx.end() && it->second >= 0 && it->second < (int)flatControlSignals.size()) {
                        flatControlSignals[it->second] = p.second;
                    }
                }
                val = vd;
            }
            else if (fc.type == ComponentType::PWM_3PH) {
                double freq = (fc.freq > 0.0) ? fc.freq : 10000.0;
                double period = 1.0 / freq;
                double phaseIn = std::fmod(currentTime, period) / period;
                if (phaseIn < 0.0) phaseIn += 1.0;
                double v_car = (phaseIn < 0.5) ? (4.0 * phaseIn - 1.0) : (3.0 - 4.0 * phaseIn); // -1 to +1 triangle wave

                double va = (fc.inputSigIndices.size() > 0 && fc.inputSigIndices[0] >= 0 && fc.inputSigIndices[0] < (int)flatControlSignals.size()) ? flatControlSignals[fc.inputSigIndices[0]] : (fc.in0Ptr ? *fc.in0Ptr : 0.0);
                double vb = (fc.inputSigIndices.size() > 1 && fc.inputSigIndices[1] >= 0 && fc.inputSigIndices[1] < (int)flatControlSignals.size()) ? flatControlSignals[fc.inputSigIndices[1]] : (fc.in1Ptr ? *fc.in1Ptr : 0.0);
                double vc = (fc.inputSigIndices.size() > 2 && fc.inputSigIndices[2] >= 0 && fc.inputSigIndices[2] < (int)flatControlSignals.size()) ? flatControlSignals[fc.inputSigIndices[2]] : 0.0;

                double outA = (va > v_car) ? 1.0 : 0.0;
                double outB = (vb > v_car) ? 1.0 : 0.0;
                double outC = (vc > v_car) ? 1.0 : 0.0;

                // Ports are registered as G1..G6, gA1..gC2, OutA..OutC, Out1..Out3 (in
                // that order). Writing outputSigIndices[0..2] put phase A/B/C on G1/G2/G3
                // and left OutA/OutB/OutC permanently at 0. Write each port explicitly;
                // the even gates are the complementary lower switches.
                {
                    const double raw[6] = {outA, 1.0 - outA, outB, 1.0 - outB, outC, 1.0 - outC};
                    double g[6];
                    pwm3PhGates(fc, currentTime, fc.delayDuration, commitState && pass == 0, raw, g);
                    writePwm3PhGates(fc, flatControlSignals, g);
                    outA = g[0]; outB = g[2]; outC = g[4];
                }
                val = outA;
            }
            else if (fc.type == ComponentType::SVPWM) {
                double fcHz = (fc.freq > 0.0) ? fc.freq : 10000.0;
                double deadTime = fc.delayDuration;
                double minVal = (fc.minVal != 0.0) ? fc.minVal : -1.0;
                double maxVal = (fc.maxVal != 0.0) ? fc.maxVal : 1.0;
                double Tc = 1.0 / fcHz;

                double in1 = (fc.inputSigIndices.size() > 0 && fc.inputSigIndices[0] >= 0 && fc.inputSigIndices[0] < (int)flatControlSignals.size()) ? flatControlSignals[fc.inputSigIndices[0]] : 0.0;
                double in2 = (fc.inputSigIndices.size() > 1 && fc.inputSigIndices[1] >= 0 && fc.inputSigIndices[1] < (int)flatControlSignals.size()) ? flatControlSignals[fc.inputSigIndices[1]] : 0.0;
                double in3 = (fc.inputSigIndices.size() > 2 && fc.inputSigIndices[2] >= 0 && fc.inputSigIndices[2] < (int)flatControlSignals.size()) ? flatControlSignals[fc.inputSigIndices[2]] : 0.0;

                double vA = 0.0, vB = 0.0, vC = 0.0;
                if (fc.svpwmAlphaBeta) {
                    double valpha = in1;
                    double vbeta = in2;
                    vA = valpha;
                    vB = -0.5 * valpha + (std::sqrt(3.0) / 2.0) * vbeta;
                    vC = -0.5 * valpha - (std::sqrt(3.0) / 2.0) * vbeta;
                } else {
                    vA = in1;
                    vB = in2;
                    vC = in3;
                }

                double v_max = std::max(vA, std::max(vB, vC));
                double v_min = std::min(vA, std::min(vB, vC));
                double v_offset = -0.5 * (v_max + v_min);

                double v_refA = vA + v_offset;
                double v_refB = vB + v_offset;
                double v_refC = vC + v_offset;

                double tLocal = std::fmod(currentTime, Tc);
                if (tLocal < 0.0) tLocal += Tc;
                double triVal = (tLocal < Tc / 2.0) 
                                ? minVal + (maxVal - minVal) * (tLocal / (Tc / 2.0))
                                : maxVal - (maxVal - minVal) * ((tLocal - Tc / 2.0) / (Tc / 2.0));

                double raw_gA1 = (v_refA > triVal) ? 1.0 : 0.0;
                double raw_gA2 = (v_refA <= triVal) ? 1.0 : 0.0;
                double raw_gB1 = (v_refB > triVal) ? 1.0 : 0.0;
                double raw_gB2 = (v_refB <= triVal) ? 1.0 : 0.0;
                double raw_gC1 = (v_refC > triVal) ? 1.0 : 0.0;
                double raw_gC2 = (v_refC <= triVal) ? 1.0 : 0.0;

                if (pass == 0 && commitState && currentTime > fc.lastTime) {
                    if (fc.pwmMasterLastTransDirect.size() < 6) {
                        fc.pwmMasterLastTransDirect.assign(6, -1.0);
                        fc.pwmMasterLastTargetDirect.assign(6, 0);
                    }
                    if (fc.pwmMasterLastTargetDirect[0] > 0 && raw_gA1 <= 0.5) fc.pwmMasterLastTransDirect[0] = currentTime;
                    if (fc.pwmMasterLastTargetDirect[1] > 0 && raw_gA2 <= 0.5) fc.pwmMasterLastTransDirect[1] = currentTime;
                    if (fc.pwmMasterLastTargetDirect[2] > 0 && raw_gB1 <= 0.5) fc.pwmMasterLastTransDirect[2] = currentTime;
                    if (fc.pwmMasterLastTargetDirect[3] > 0 && raw_gB2 <= 0.5) fc.pwmMasterLastTransDirect[3] = currentTime;
                    if (fc.pwmMasterLastTargetDirect[4] > 0 && raw_gC1 <= 0.5) fc.pwmMasterLastTransDirect[4] = currentTime;
                    if (fc.pwmMasterLastTargetDirect[5] > 0 && raw_gC2 <= 0.5) fc.pwmMasterLastTransDirect[5] = currentTime;

                    fc.pwmMasterLastTargetDirect[0] = (raw_gA1 > 0.5) ? 1 : 0;
                    fc.pwmMasterLastTargetDirect[1] = (raw_gA2 > 0.5) ? 1 : 0;
                    fc.pwmMasterLastTargetDirect[2] = (raw_gB1 > 0.5) ? 1 : 0;
                    fc.pwmMasterLastTargetDirect[3] = (raw_gB2 > 0.5) ? 1 : 0;
                    fc.pwmMasterLastTargetDirect[4] = (raw_gC1 > 0.5) ? 1 : 0;
                    fc.pwmMasterLastTargetDirect[5] = (raw_gC2 > 0.5) ? 1 : 0;
                    fc.lastTime = currentTime;
                }

                double gA1 = raw_gA1;
                if (gA1 > 0.5 && deadTime > 0 && fc.pwmMasterLastTransDirect.size() >= 6 && fc.pwmMasterLastTransDirect[1] >= 0 && (currentTime - fc.pwmMasterLastTransDirect[1]) < deadTime) gA1 = 0.0;
                double gA2 = raw_gA2;
                if (gA2 > 0.5 && deadTime > 0 && fc.pwmMasterLastTransDirect.size() >= 6 && fc.pwmMasterLastTransDirect[0] >= 0 && (currentTime - fc.pwmMasterLastTransDirect[0]) < deadTime) gA2 = 0.0;

                double gB1 = raw_gB1;
                if (gB1 > 0.5 && deadTime > 0 && fc.pwmMasterLastTransDirect.size() >= 6 && fc.pwmMasterLastTransDirect[3] >= 0 && (currentTime - fc.pwmMasterLastTransDirect[3]) < deadTime) gB1 = 0.0;
                double gB2 = raw_gB2;
                if (gB2 > 0.5 && deadTime > 0 && fc.pwmMasterLastTransDirect.size() >= 6 && fc.pwmMasterLastTransDirect[2] >= 0 && (currentTime - fc.pwmMasterLastTransDirect[2]) < deadTime) gB2 = 0.0;

                double gC1 = raw_gC1;
                if (gC1 > 0.5 && deadTime > 0 && fc.pwmMasterLastTransDirect.size() >= 6 && fc.pwmMasterLastTransDirect[5] >= 0 && (currentTime - fc.pwmMasterLastTransDirect[5]) < deadTime) gC1 = 0.0;
                double gC2 = raw_gC2;
                if (gC2 > 0.5 && deadTime > 0 && fc.pwmMasterLastTransDirect.size() >= 6 && fc.pwmMasterLastTransDirect[4] >= 0 && (currentTime - fc.pwmMasterLastTransDirect[4]) < deadTime) gC2 = 0.0;

                std::vector<std::pair<std::string, double>> outputs = {
                    {"G1", gA1}, {"G2", gA2}, {"G3", gB1}, {"G4", gB2}, {"G5", gC1}, {"G6", gC2},
                    {"OutA", gA1}, {"OutB", gB1}, {"OutC", gC1},
                    {"gA1", gA1}, {"gA2", gA2}, {"gB1", gB1}, {"gB2", gB2}, {"gC1", gC1}, {"gC2", gC2},
                    {"Out1", gA1}, {"Out2", gB1}, {"Out3", gC1}
                };

                for (const auto& p : outputs) {
                    auto it = signalKeyToIdx.find(fc.id + "." + p.first);
                    if (it != signalKeyToIdx.end() && it->second >= 0 && it->second < (int)flatControlSignals.size()) {
                        flatControlSignals[it->second] = p.second;
                    }
                }
                val = gA1;
            }
            else if (fc.type == ComponentType::PerAvg) {
                double inVal = fc.in0Ptr ? *fc.in0Ptr : 0.0;
                double period = (fc.delayDuration > 0.0) ? fc.delayDuration : 0.02;
                // Window LENGTH in samples, not an integration step: this block averages
                // over a fixed number of past samples, so it is sized from the nominal
                // step deliberately. Under variable-step operation the averaging window
                // is therefore only approximate; making it time-based is separate work.
                double dtNominal = (config.stepSize > 0.0) ? config.stepSize : 1e-5;
                int maxSamples = (int)std::round(period / dtNominal);
                if (maxSamples < 1) maxSamples = 1;

                if (currentTime <= 0.0) {
                    fc.shiftBuffer.clear();
                    fc.shiftBuffer.push_back(inVal);
                    fc.lastTime = 0.0;
                    val = inVal;
                } else {
                    if (pass == 0 && commitState && currentTime > fc.lastTime) {
                        fc.shiftBuffer.push_back(inVal);
                        while ((int)fc.shiftBuffer.size() > maxSamples) {
                            fc.shiftBuffer.erase(fc.shiftBuffer.begin());
                        }
                        fc.lastTime = currentTime;
                    }
                    double sum = 0.0;
                    for (double v : fc.shiftBuffer) sum += v;
                    val = fc.shiftBuffer.empty() ? inVal : (sum / fc.shiftBuffer.size());
                }
            }
            else if (fc.type == ComponentType::PeriodicImpAvg) {
                double inVal = fc.in0Ptr ? *fc.in0Ptr : 0.0;
                double trigVal = fc.ctrlSigPtr ? *fc.ctrlSigPtr : 0.0;
                bool isRising = (fc.prev_clk <= 0.5 && trigVal > 0.5);
                double dt = dtNow;
                if (currentTime == 0.0) {
                    fc.stateVal = 0.0; // accumulated integral
                    fc.filterState = 0.0; // accumulated time
                    fc.prevOut = fc.val; // held output
                }
                if (isRising && fc.filterState > 0.0) {
                    fc.prevOut = fc.stateVal / fc.filterState;
                    fc.stateVal = 0.0;
                    fc.filterState = 0.0;
                } else {
                    fc.stateVal += inVal * dt;
                    fc.filterState += dt;
                }
                if (commitState && currentTime > fc.lastTime) {
                    fc.prev_clk = trigVal;
                    fc.lastTime = currentTime;
                }
                val = fc.prevOut;
            }
            else if (fc.type == ComponentType::FourierTrans || fc.type == ComponentType::FourierAnalysis) {
                double inVal = fc.in0Ptr ? *fc.in0Ptr : 0.0;
                double fn = (fc.freq > 0.0) ? fc.freq : 50.0;
                double harmonic = (fc.shiftLength > 0) ? (double)fc.shiftLength : 1.0;
                // Sampling interval used to size the sliding DFT buffer, so this is a
                // window length rather than an integration step and stays on the nominal
                // step on purpose. Under variable-step operation the buffer then spans a
                // varying amount of real time; reformulating it as a time window is
                // separate work.
                double ts = (fc.delay > 0.0) ? fc.delay : (config.stepSize > 0.0 ? config.stepSize : 1e-4);
                int N = (int)std::round(1.0 / (fn * ts));
                if (N < 2) N = 2;

                // The window is N = 1/(f ts) samples, so one sample must enter per ts. The
                // buffer used to take a sample every simulation step, which made the window
                // cover 1/(f * step) of time instead of one fundamental cycle.
                if (commitState && currentTime > fc.lastTime && currentTime + 1e-12 >= fc.nextSampleTime) {
                    fc.shiftBuffer.push_back(inVal);
                    if ((int)fc.shiftBuffer.size() > N) fc.shiftBuffer.erase(fc.shiftBuffer.begin());
                    fc.lastTime = currentTime;
                    fc.nextSampleTime = currentTime + ts;
                }

                double Re = 0.0, Im = 0.0;
                int bufSize = (int)fc.shiftBuffer.size();
                double omega = 2.0 * 3.141592653589793 * harmonic / N;
                for (int i = 0; i < bufSize; ++i) {
                    Re += fc.shiftBuffer[i] * std::cos(omega * i);
                    Im += fc.shiftBuffer[i] * std::sin(omega * i);
                }
                Re = (2.0 / (bufSize > 0 ? bufSize : 1)) * Re;
                Im = (2.0 / (bufSize > 0 ? bufSize : 1)) * Im;

                double mag = std::sqrt(Re * Re + Im * Im);
                double phase = std::atan2(-Im, Re) * (180.0 / 3.141592653589793);

                if (fc.outputSigIndices.size() > 0 && fc.outputSigIndices[0] >= 0 && fc.outputSigIndices[0] < (int)flatControlSignals.size()) flatControlSignals[fc.outputSigIndices[0]] = mag;
                if (fc.outputSigIndices.size() > 1 && fc.outputSigIndices[1] >= 0 && fc.outputSigIndices[1] < (int)flatControlSignals.size()) flatControlSignals[fc.outputSigIndices[1]] = phase;
                val = mag;
            }
            else if (fc.type == ComponentType::UnitDelay || fc.type == ComponentType::ZoH) {
                // Sampled blocks: a sample is taken every ts and the output is held between samples.
                // ZOH outputs the sample just taken. UNIT_DELAY outputs the previous sample, so it
                // lags by one sample period. stateVal holds the last sample, prevOut the output.
                const double inVal = fc.in0Ptr ? *fc.in0Ptr : 0.0;
                const double ts = (fc.delay > 0.0) ? fc.delay : (config.stepSize > 0.0 ? config.stepSize : 1e-4);
                const bool sampleNow = currentTime + 1e-12 >= fc.nextSampleTime;
                const bool doCommit = (pass == 0 && commitState && sampleNow);
                if (fc.type == ComponentType::ZoH) {
                    if (doCommit) {
                        fc.prevOut = inVal;
                        fc.nextSampleTime = currentTime + ts;
                    }
                    val = sampleNow ? inVal : fc.prevOut;
                } else {
                    const double outNow = fc.stateVal;
                    if (doCommit) {
                        fc.prevOut = outNow;
                        fc.stateVal = inVal;
                        fc.nextSampleTime = currentTime + ts;
                    }
                    val = sampleNow ? outNow : fc.prevOut;
                }
            }
            else if (fc.type == ComponentType::DiscreteInt) {
                // Forward Euler on samples taken every ts. The output is held between samples.
                const double inVal = fc.in0Ptr ? *fc.in0Ptr : 0.0;
                const double ts = (fc.delay > 0.0) ? fc.delay : 0.01;
                const bool sampleNow = currentTime + 1e-12 >= fc.nextSampleTime;
                const bool doCommit = (pass == 0 && commitState && sampleNow);
                const double yk = fc.prevOut + ts * fc.stateVal;   // y[k] = y[k-1] + ts * u[k-1]
                if (doCommit) {
                    fc.prevOut = yk;
                    fc.stateVal = inVal;
                    fc.nextSampleTime = currentTime + ts;
                }
                val = sampleNow ? yk : fc.prevOut;
            }
            else if (fc.type == ComponentType::DiscreteMean) {
                // Window of the last shiftLength samples, updated on each sample instant.
                const double inVal = fc.in0Ptr ? *fc.in0Ptr : 0.0;
                const double ts = (fc.delay > 0.0) ? fc.delay : 0.001;
                const bool sampleNow = currentTime + 1e-12 >= fc.nextSampleTime;
                const bool doCommit = (pass == 0 && commitState && sampleNow);
                if (doCommit) {
                    fc.shiftBuffer.push_back(inVal);
                    if ((int)fc.shiftBuffer.size() > fc.shiftLength) fc.shiftBuffer.erase(fc.shiftBuffer.begin());
                    fc.nextSampleTime = currentTime + ts;
                }
                if (fc.shiftBuffer.empty()) {
                    val = fc.prevOut;   // initial_value until the first sample
                } else {
                    double sum = 0.0;
                    for (double v : fc.shiftBuffer) sum += v;
                    val = sum / (double)fc.shiftBuffer.size();
                }
            }
            else if (fc.type == ComponentType::DcMotor) {
                // Explicit Euler on the mechanical equation, once per committed step:
                //   J domega/dt = Kt i - TL - B omega
                // The speed is the state. The back-EMF Ke * omega is written for the armature.
                const double tl = fc.in0Ptr ? *fc.in0Ptr : 0.0;
                const double armCurrent = (!fc.inputSigIndices.empty() && fc.inputSigIndices[0] >= 0 &&
                                           fc.inputSigIndices[0] < (int)flatControlSignals.size())
                                              ? flatControlSignals[fc.inputSigIndices[0]] : 0.0;
                if (pass == 0 && commitState && currentTime > fc.lastTime) {
                    fc.stateVal += dtNow * (fc.dcKt * armCurrent - tl - fc.dcB * fc.stateVal) / fc.dcJ;
                    fc.lastTime = currentTime;
                }
                if (!fc.outputSigIndices.empty() && fc.outputSigIndices[0] >= 0 &&
                    fc.outputSigIndices[0] < (int)flatControlSignals.size()) {
                    flatControlSignals[fc.outputSigIndices[0]] = fc.dcKe * fc.stateVal;
                }
                val = fc.stateVal;
            }
            else if (fc.type == ComponentType::BldcMotor || fc.type == ComponentType::PmsmMotor ||
                     fc.type == ComponentType::SynchronousMotor) {
                // Phase EMF e_x = k * omega * shape(theta - 2*pi*x/3). k is Ke for BLDC and PMSM,
                // and Kf * field current for SYNC. shape is a trapezoid for BLDC and a sine otherwise.
                // Torque = k * sum_x shape_x i_x. Speed and electrical angle advance explicitly once
                // per committed step, after the torque is taken from the currents at that step.
                const double kTwoPi = 6.283185307179586;
                const double kTwoPiThird = kTwoPi / 3.0;
                const double tl = fc.in0Ptr ? *fc.in0Ptr : 0.0;
                auto sigAt = [&](size_t k) -> double {
                    return (k < fc.inputSigIndices.size() && fc.inputSigIndices[k] >= 0 &&
                            fc.inputSigIndices[k] < (int)flatControlSignals.size())
                               ? flatControlSignals[fc.inputSigIndices[k]] : 0.0;
                };
                const bool trapezoidal = (fc.type == ComponentType::BldcMotor);
                auto shape = [trapezoidal](double th) -> double {
                    return trapezoidal ? trapezoidShape(th) : std::sin(th);
                };
                const double k = (fc.type == ComponentType::SynchronousMotor) ? fc.mKf * sigAt(3) : fc.mKe;
                if (pass == 0 && commitState && currentTime > fc.lastTime) {
                    const double torque = k * (shape(fc.mTheta) * sigAt(0) +
                                               shape(fc.mTheta - kTwoPiThird) * sigAt(1) +
                                               shape(fc.mTheta - 2.0 * kTwoPiThird) * sigAt(2));
                    fc.stateVal += dtNow * (torque - tl - fc.mB * fc.stateVal) / fc.mJ;
                    fc.mTheta = std::fmod(fc.mTheta + dtNow * fc.mPoles * fc.stateVal, kTwoPi);
                    fc.lastTime = currentTime;
                }
                const double w = fc.stateVal;
                for (size_t x = 0; x < 3 && x < fc.outputSigIndices.size(); ++x) {
                    const int oi = fc.outputSigIndices[x];
                    if (oi >= 0 && oi < (int)flatControlSignals.size())
                        flatControlSignals[oi] = k * w * shape(fc.mTheta - (double)x * kTwoPiThird);
                }
                val = fc.stateVal;
            }
            else if (fc.type == ComponentType::SrMotor) {
                // Switched reluctance. Phase inductance L_x(theta) = Lavg + Ldiff cos(theta_x) with
                // theta_x = Nr * theta_m - 2*pi*x/3. The phase voltage is v = R i + L di/dt + i dL/dt,
                // so the controlled EMF is e_x = i_x dL_x/dt = i_x * omega * dL_x/dtheta_m.
                // Torque = sum_x 0.5 i_x^2 dL_x/dtheta_m. Inductances are written as signals for VAR_L.
                const double kTwoPi = 6.283185307179586;
                const double kTwoPiThird = kTwoPi / 3.0;
                const double tl = fc.in0Ptr ? *fc.in0Ptr : 0.0;
                auto sigAt = [&](size_t k) -> double {
                    return (k < fc.inputSigIndices.size() && fc.inputSigIndices[k] >= 0 &&
                            fc.inputSigIndices[k] < (int)flatControlSignals.size())
                               ? flatControlSignals[fc.inputSigIndices[k]] : 0.0;
                };
                auto thetaOf = [&](int x) -> double { return fc.mTheta - (double)x * kTwoPiThird; };
                auto dLdThetaM = [&](double th) -> double { return -fc.mLdiff * fc.mPoles * std::sin(th); };
                if (pass == 0 && commitState && currentTime > fc.lastTime) {
                    double torque = 0.0;
                    for (int x = 0; x < 3; ++x) {
                        const double ix = sigAt((size_t)x);
                        torque += 0.5 * ix * ix * dLdThetaM(thetaOf(x));
                    }
                    fc.stateVal += dtNow * (torque - tl - fc.mB * fc.stateVal) / fc.mJ;
                    fc.mTheta = std::fmod(fc.mTheta + dtNow * fc.mPoles * fc.stateVal, kTwoPi);
                    fc.lastTime = currentTime;
                }
                const double w = fc.stateVal;
                for (int x = 0; x < 3; ++x) {
                    const double th = thetaOf(x);
                    if (!(fc.outputSigIndices.size() > (size_t)(x + 3))) break;
                    const int oe = fc.outputSigIndices[(size_t)x];
                    const int ol = fc.outputSigIndices[(size_t)(x + 3)];
                    if (oe >= 0 && oe < (int)flatControlSignals.size())
                        flatControlSignals[oe] = sigAt((size_t)x) * w * dLdThetaM(th);
                    if (ol >= 0 && ol < (int)flatControlSignals.size())
                        flatControlSignals[ol] = fc.mLavg + fc.mLdiff * std::cos(th);
                }
                val = fc.stateVal;
            }
            else if (fc.type == ComponentType::InductionMotor) {
                // Induction machine, stationary alpha-beta frame, rotor flux linkage psi_r as state:
                //   d psi/dt = -(Rr/Lr)(psi - Lm i_s) + j P omega psi
                //   stator EMF e = (Lm / Lr) d psi / dt
                //   torque T = 1.5 P (Lm / Lr) (psi_a i_b - psi_b i_a)
                // Clarke transform with i_a + i_b + i_c = 0 (star without a neutral wire).
                const double tl = fc.in0Ptr ? *fc.in0Ptr : 0.0;
                auto sigAt = [&](size_t k) -> double {
                    return (k < fc.inputSigIndices.size() && fc.inputSigIndices[k] >= 0 &&
                            fc.inputSigIndices[k] < (int)flatControlSignals.size())
                               ? flatControlSignals[fc.inputSigIndices[k]] : 0.0;
                };
                const double sqrt3 = 1.7320508075688772;
                const double ia = sigAt(0), ib = sigAt(1), ic = sigAt(2);
                const double iAlpha = ia;
                const double iBeta = (ib - ic) / sqrt3;
                const double M = fc.mLm;
                const double k = fc.mRr / fc.mLr;
                if (pass == 0 && commitState && currentTime > fc.lastTime) {
                    const double torque = 1.5 * fc.mPoles * (M / fc.mLr) * (fc.mPsiA * iBeta - fc.mPsiB * iAlpha);
                    const double wr = fc.mPoles * fc.stateVal;
                    const double dPsiA = -k * (fc.mPsiA - M * iAlpha) - wr * fc.mPsiB;
                    const double dPsiB = -k * (fc.mPsiB - M * iBeta) + wr * fc.mPsiA;
                    fc.mPsiA += dtNow * dPsiA;
                    fc.mPsiB += dtNow * dPsiB;
                    fc.stateVal += dtNow * (torque - tl - fc.mB * fc.stateVal) / fc.mJ;
                    fc.lastTime = currentTime;
                }
                const double wr = fc.mPoles * fc.stateVal;
                const double dPsiA = -k * (fc.mPsiA - M * iAlpha) - wr * fc.mPsiB;
                const double dPsiB = -k * (fc.mPsiB - M * iBeta) + wr * fc.mPsiA;
                const double eAlpha = (M / fc.mLr) * dPsiA;
                const double eBeta = (M / fc.mLr) * dPsiB;
                const double eA = eAlpha;
                const double eB = -0.5 * eAlpha + 0.5 * sqrt3 * eBeta;
                const double eC = -0.5 * eAlpha - 0.5 * sqrt3 * eBeta;
                const double outE[3] = {eA, eB, eC};
                for (size_t x = 0; x < 3 && x < fc.outputSigIndices.size(); ++x) {
                    const int oi = fc.outputSigIndices[x];
                    if (oi >= 0 && oi < (int)flatControlSignals.size()) flatControlSignals[oi] = outE[x];
                }
                val = fc.stateVal;
            }
            else if (fc.type == ComponentType::DiscreteTF) {
                // Difference equation on samples taken every ts; the output is held between samples.
                const double uk = fc.in0Ptr ? *fc.in0Ptr : 0.0;
                const double ts = (fc.delay > 0.0) ? fc.delay : 0.01;
                const bool sampleNow = currentTime + 1e-12 >= fc.nextSampleTime;
                const bool doCommit = (pass == 0 && commitState && sampleNow);
                double acc = fc.dtfNum.empty() ? 0.0 : fc.dtfNum[0] * uk;
                for (size_t i = 1; i < fc.dtfNum.size(); ++i)
                    acc += fc.dtfNum[i] * (i - 1 < fc.dtfU.size() ? fc.dtfU[i - 1] : 0.0);
                for (size_t i = 1; i < fc.dtfDen.size(); ++i)
                    acc -= fc.dtfDen[i] * (i - 1 < fc.dtfY.size() ? fc.dtfY[i - 1] : 0.0);
                const double a0 = (!fc.dtfDen.empty() && fc.dtfDen[0] != 0.0) ? fc.dtfDen[0] : 1.0;
                const double yk = acc / a0;
                if (doCommit) {
                    if (!fc.dtfU.empty()) { fc.dtfU.insert(fc.dtfU.begin(), uk); fc.dtfU.pop_back(); }
                    if (!fc.dtfY.empty()) { fc.dtfY.insert(fc.dtfY.begin(), yk); fc.dtfY.pop_back(); }
                    fc.prevOut = yk;
                    fc.nextSampleTime = currentTime + ts;
                }
                val = sampleNow ? yk : fc.prevOut;
            }
            else if (fc.type == ComponentType::DiscreteSS) {
                // State update on samples taken every ts; the output is held between samples.
                const double uk = fc.in0Ptr ? *fc.in0Ptr : 0.0;
                const double ts = (fc.delay > 0.0) ? fc.delay : 0.01;
                const bool sampleNow = currentTime + 1e-12 >= fc.nextSampleTime;
                const bool doCommit = (pass == 0 && commitState && sampleNow);
                const size_t n = fc.dssX.size();
                double yk = fc.dssD * uk;
                for (size_t i = 0; i < n; ++i) yk += fc.dssC[i] * fc.dssX[i];
                if (doCommit) {
                    std::vector<double> xn(n, 0.0);
                    for (size_t i = 0; i < n; ++i) {
                        double s = fc.dssB[i] * uk;
                        for (size_t j = 0; j < n; ++j) s += fc.dssA[i * n + j] * fc.dssX[j];
                        xn[i] = s;
                    }
                    fc.dssX = xn;
                    fc.prevOut = yk;
                    fc.nextSampleTime = currentTime + ts;
                }
                val = sampleNow ? yk : fc.prevOut;
            }
            else if (fc.type == ComponentType::FourierSeries) {
                // Band-limited reconstruction: the DC term plus the first `harmonics` terms of
                // the Fourier series fitted to the last fundamental cycle, evaluated at the
                // newest sample. A pure sine at f passes through unchanged. Extra harmonics
                // are dropped, which low-pass filters the input.
                double inVal = fc.in0Ptr ? *fc.in0Ptr : 0.0;
                double fn = (fc.freq > 0.0) ? fc.freq : 50.0;
                int H = (fc.shiftLength > 0) ? fc.shiftLength : 10;
                double ts = (fc.delay > 0.0) ? fc.delay : (config.stepSize > 0.0 ? config.stepSize : 1e-4);
                int N = (int)std::round(1.0 / (fn * ts));
                if (N < 2) N = 2;
                // One sample enters per ts, as in FOURIER_TRANS, so the window is one cycle.
                if (commitState && currentTime > fc.lastTime && currentTime + 1e-12 >= fc.nextSampleTime) {
                    fc.shiftBuffer.push_back(inVal);
                    if ((int)fc.shiftBuffer.size() > N) fc.shiftBuffer.erase(fc.shiftBuffer.begin());
                    fc.lastTime = currentTime;
                    fc.nextSampleTime = currentTime + ts;
                }
                const int M = (int)fc.shiftBuffer.size();
                if (M == 0) {
                    val = 0.0;
                } else {
                    const double twoPi = 2.0 * 3.141592653589793;
                    double a0 = 0.0;
                    for (double v : fc.shiftBuffer) a0 += v;
                    a0 /= M;
                    // Evaluate at the current time, not at the last sample: advance from the
                    // newest sample by the elapsed fraction of ts. Otherwise the output lags
                    // the input by up to one sample interval.
                    const double tSample = fc.nextSampleTime - ts;
                    const double frac = std::clamp((currentTime - tSample) / ts, 0.0, 1.0);
                    const double xNow = (double)(M - 1) + frac;
                    double rec = a0;
                    for (int n = 1; n <= H; ++n) {
                        const double w = twoPi * n / N;
                        double an = 0.0, bn = 0.0;
                        for (int i = 0; i < M; ++i) {
                            an += fc.shiftBuffer[i] * std::cos(w * i);
                            bn += fc.shiftBuffer[i] * std::sin(w * i);
                        }
                        an *= 2.0 / M;
                        bn *= 2.0 / M;
                        rec += an * std::cos(w * xNow) + bn * std::sin(w * xNow);
                    }
                    val = rec;
                }
            }
            else if (fc.type == ComponentType::MovAvg) {
                double inVal = fc.in0Ptr ? *fc.in0Ptr : 0.0;
                int win = (fc.shiftLength > 0) ? fc.shiftLength : 10;
                if (commitState && currentTime > fc.lastTime) {
                    fc.shiftBuffer.push_back(inVal);
                    if ((int)fc.shiftBuffer.size() > win) fc.shiftBuffer.erase(fc.shiftBuffer.begin());
                    fc.lastTime = currentTime;
                }
                double sum = 0.0;
                for (double v : fc.shiftBuffer) sum += v;
                val = fc.shiftBuffer.empty() ? inVal : (sum / fc.shiftBuffer.size());
            }
            else if (fc.type == ComponentType::Filter1st) {
                double inVal = (fc.inputSigIndices.empty() || fc.inputSigIndices[0] < 0) ? (fc.in0Ptr ? *fc.in0Ptr : 0.0) : flatControlSignals[fc.inputSigIndices[0]];
                double fcHz = (fc.freq > 0.0) ? fc.freq : 100.0;
                double tau = 1.0 / (2.0 * 3.141592653589793 * fcHz);
                double dt = dtNow;
                if (currentTime <= 0.0) {
                    val = inVal;
                    fc.stateVal = inVal;
                    fc.nextStateVal = inVal;
                    fc.lastTime = 0.0;
                } else {
                    if (commitState && currentTime > fc.lastTime) {
                        fc.stateVal = fc.nextStateVal;
                        fc.lastTime = currentTime;
                    }
                    double prev = fc.stateVal;
                    double outVal = (tau / (tau + dt)) * prev + (dt / (tau + dt)) * inVal;
                    if (pass == 0 && commitState) {
                        fc.nextStateVal = outVal;
                    }
                    val = outVal;
                }
            }
            else if (fc.type == ComponentType::Filter2nd) {
                double inVal = (fc.inputSigIndices.empty() || fc.inputSigIndices[0] < 0) ? (fc.in0Ptr ? *fc.in0Ptr : 0.0) : flatControlSignals[fc.inputSigIndices[0]];
                double fcHz = (fc.freq > 0.0) ? fc.freq : 100.0;
                double zeta = (fc.gain >= 0.0) ? fc.gain : 0.707;
                double w0 = 2.0 * 3.141592653589793 * fcHz;
                double dt = dtNow;
                if (currentTime <= 0.0) {
                    val = inVal;
                    fc.stateVal = inVal;       // y(t_{n-1})
                    fc.nextStateVal = inVal;   // y(t_n)
                    fc.filterState = 0.0;      // dy/dt(t_{n-1})
                    fc.highStartTime = 0.0;    // dy/dt(t_n)
                    fc.lastTime = 0.0;
                } else {
                    if (commitState && currentTime > fc.lastTime) {
                        fc.stateVal = fc.nextStateVal;
                        fc.filterState = fc.highStartTime;
                        fc.lastTime = currentTime;
                    }
                    double y = fc.stateVal;
                    double dy = fc.filterState;
                    double d2y = w0 * w0 * (inVal - y) - 2.0 * zeta * w0 * dy;
                    double dy_next = dy + d2y * dt;
                    double y_next = y + dy_next * dt;
                    if (pass == 0 && commitState) {
                        fc.nextStateVal = y_next;
                        fc.highStartTime = dy_next;
                    }
                    val = y_next;
                }
            }
            else if (fc.type == ComponentType::StateSpace) {
                double inVal = fc.in0Ptr ? *fc.in0Ptr : 0.0;
                double dt = dtNow;

                auto parseMat = [](std::string s) -> std::vector<std::vector<double>> {
                    std::vector<std::vector<double>> mat;
                    if (s.empty()) return mat;
                    for (char& c : s) {
                        if (c == '[' || c == ']' || c == ',') c = ' ';
                    }
                    std::stringstream ss(s);
                    std::string line;
                    while (std::getline(ss, line, ';')) {
                        std::stringstream lineSS(line);
                        std::vector<double> row;
                        double v;
                        while (lineSS >> v) row.push_back(v);
                        if (!row.empty()) mat.push_back(row);
                    }
                    return mat;
                };

                std::vector<std::vector<double>> A = parseMat(fc.polarity);
                std::vector<std::vector<double>> B = parseMat(fc.vPlotKey);
                std::vector<std::vector<double>> C = parseMat(fc.vAlphaKey);
                std::vector<std::vector<double>> D = parseMat(fc.vBetaKey);

                if (A.empty()) A = {{-1.0}};
                if (B.empty()) B = {{1.0}};
                if (C.empty()) C = {{1.0}};
                if (D.empty()) D = {{0.0}};

                size_t n = A.size();

                if (fc.stateVector.size() != n) {
                    fc.stateVector.assign(n, 0.0);
                }

                auto getXDotSS = [&](const std::vector<double>& xCurr, double uVal) -> std::vector<double> {
                    std::vector<double> xD(n, 0.0);
                    for (size_t i = 0; i < n; ++i) {
                        double ax = 0.0;
                        for (size_t j = 0; j < n && j < A[i].size(); ++j) ax += A[i][j] * xCurr[j];
                        double bu = B[i].empty() ? 0.0 : B[i][0] * uVal;
                        xD[i] = ax + bu;
                    }
                    return xD;
                };

                std::vector<double> k1 = getXDotSS(fc.stateVector, inVal);
                std::vector<double> x2(n); for (size_t k = 0; k < n; ++k) x2[k] = fc.stateVector[k] + 0.5 * dt * k1[k];
                std::vector<double> k2 = getXDotSS(x2, inVal);
                std::vector<double> x3(n); for (size_t k = 0; k < n; ++k) x3[k] = fc.stateVector[k] + 0.5 * dt * k2[k];
                std::vector<double> k3 = getXDotSS(x3, inVal);
                std::vector<double> x4(n); for (size_t k = 0; k < n; ++k) x4[k] = fc.stateVector[k] + dt * k3[k];
                std::vector<double> k4 = getXDotSS(x4, inVal);

                if (pass == 0 && commitState) {
                    for (size_t k = 0; k < n; ++k) {
                        fc.stateVector[k] += (dt / 6.0) * (k1[k] + 2.0 * k2[k] + 2.0 * k3[k] + k4[k]);
                    }
                }

                double yVal = 0.0;
                for (size_t j = 0; j < n && j < C[0].size(); ++j) yVal += C[0][j] * fc.stateVector[j];
                double du = D[0].empty() ? 0.0 : D[0][0] * inVal;
                val = yVal + du;
            }
            else if (fc.type == ComponentType::MathFunction) {
                double u1 = fc.in0Ptr ? *fc.in0Ptr : 0.0;
                double u2 = fc.in1Ptr ? *fc.in1Ptr : 2.0;
                const std::string& fcn = fc.polarity;   // lower-cased at setup
                if (fcn == "exp" || fcn == "exponential") val = std::exp(u1);
                else if (fcn == "log" || fcn == "ln" || fcn == "logarithm") val = std::log(std::abs(u1) + 1e-15);
                else if (fcn == "10^u" || fcn == "pow10") val = std::pow(10.0, u1);
                else if (fcn == "log10") val = std::log10(std::abs(u1) + 1e-15);
                else if (fcn == "square") val = u1 * u1;
                else if (fcn == "sqrt" || fcn == "square root") val = (u1 >= 0.0) ? std::sqrt(u1) : 0.0;
                else if (fcn == "reciprocal" || fcn == "1/u") val = (u1 != 0.0) ? (1.0 / u1) : 0.0;
                else if (fcn == "abs") val = std::abs(u1);
                else if (fcn == "power" || fcn == "pow") val = std::pow(u1, u2);
                else if (fcn == "mod") { double m = (u2 == 0.0 ? 1.0 : u2); val = std::fmod(std::fmod(u1, m) + m, m); }
                else if (fcn == "rem") { double m = (u2 == 0.0 ? 1.0 : u2); val = std::fmod(u1, m); }
                // Trig and hyperbolic functions are listed as choices of the block but had
                // no branch, so each silently evaluated to exp(u).
                else if (fcn == "sin") val = std::sin(u1);
                else if (fcn == "cos") val = std::cos(u1);
                else if (fcn == "tan") val = std::tan(u1);
                else if (fcn == "asin") val = std::asin(std::max(-1.0, std::min(1.0, u1)));
                else if (fcn == "acos") val = std::acos(std::max(-1.0, std::min(1.0, u1)));
                else if (fcn == "atan") val = std::atan(u1);
                else if (fcn == "atan2") val = std::atan2(u1, u2);
                else if (fcn == "sinh") val = std::sinh(u1);
                else if (fcn == "cosh") val = std::cosh(u1);
                else if (fcn == "tanh") val = std::tanh(u1);
                else val = std::exp(u1);
            }
            else if (fc.type == ComponentType::Round) {
                double u = fc.in0Ptr ? *fc.in0Ptr : 0.0;
                // This branch is unreachable: an earlier `else if` in the same chain
                // already handles ComponentType::Round. Left as-is.
                std::string mode = toLowerAscii(fc.polarity);
                if (mode == "floor") val = std::floor(u);
                else if (mode == "ceil") val = std::ceil(u);
                else val = std::round(u);
            }
            else if (fc.type == ComponentType::PWM_MASTER) {
                int N = fc.numInputs;
                double fcHz = (fc.freq > 0.0) ? fc.freq : 10000.0;
                double deadTime = fc.delayDuration;
                double Tc = 1.0 / fcHz;

                for (int i = 0; i < N; ++i) {
                    double vMod = (fc.pwmMasterInIndices[i] >= 0 && fc.pwmMasterInIndices[i] < (int)flatControlSignals.size()) 
                                  ? flatControlSignals[fc.pwmMasterInIndices[i]] : 0.0;

                    double phaseDeg = fc.pwmMasterPhaseDeg[i];
                    if (fc.pwmMasterPhaseExt[i] && fc.pwmMasterExtPhaseIndices[i] >= 0 && fc.pwmMasterExtPhaseIndices[i] < (int)flatControlSignals.size()) {
                        phaseDeg = flatControlSignals[fc.pwmMasterExtPhaseIndices[i]];
                    }

                    double lOffset = fc.pwmMasterLevelOffset[i];
                    double tOffset = (phaseDeg / 360.0) * Tc;
                    double tLocal = std::fmod(currentTime - tOffset, Tc);
                    if (tLocal < 0.0) tLocal += Tc;

                    double triVal = (tLocal < Tc / 2.0) 
                                    ? (tLocal / (Tc / 2.0)) 
                                    : (1.0 - (tLocal - Tc / 2.0) / (Tc / 2.0));
                    double vCarrier = fc.minVal + (fc.maxVal - fc.minVal) * triVal + lOffset;

                    int targetDirect = (vMod >= vCarrier) ? 1 : 0;
                    int targetCompl = (targetDirect == 0) ? 1 : 0;

                    if (pass == 0 && commitState && currentTime > fc.lastTime) {
                        if (targetDirect == 1 && fc.pwmMasterLastTargetDirect[i] == 0) {
                            fc.pwmMasterLastTransDirect[i] = currentTime;
                        }
                        if (targetCompl == 1 && fc.pwmMasterLastTargetCompl[i] == 0) {
                            fc.pwmMasterLastTransCompl[i] = currentTime;
                        }
                        fc.pwmMasterLastTargetDirect[i] = targetDirect;
                        fc.pwmMasterLastTargetCompl[i] = targetCompl;
                    }

                    double outD = (targetDirect == 1 && (currentTime - fc.pwmMasterLastTransDirect[i] >= deadTime - 1e-12)) ? 1.0 : 0.0;
                    double outC = (targetCompl == 1 && (currentTime - fc.pwmMasterLastTransCompl[i] >= deadTime - 1e-12)) ? 1.0 : 0.0;

                    int dIdx = fc.pwmMasterOutDirectIndices[i];
                    int cIdx = fc.pwmMasterOutComplIndices[i];
                    if (dIdx >= 0 && dIdx < (int)flatControlSignals.size()) flatControlSignals[dIdx] = outD;
                    if (cIdx >= 0 && cIdx < (int)flatControlSignals.size()) flatControlSignals[cIdx] = outC;

                    int chIdx = i + 1;
                    auto itD2 = signalKeyToIdx.find(fc.id + ".OutDirect" + std::to_string(chIdx));
                    if (itD2 != signalKeyToIdx.end() && itD2->second < (int)flatControlSignals.size()) flatControlSignals[itD2->second] = outD;
                    auto itC2 = signalKeyToIdx.find(fc.id + ".OutCompl" + std::to_string(chIdx));
                    if (itC2 != signalKeyToIdx.end() && itC2->second < (int)flatControlSignals.size()) flatControlSignals[itC2->second] = outC;
                    auto itD3 = signalKeyToIdx.find(fc.id + ".Out" + std::to_string(chIdx));
                    if (itD3 != signalKeyToIdx.end() && itD3->second < (int)flatControlSignals.size()) flatControlSignals[itD3->second] = outD;
                }
                if (pass == 0 && commitState && currentTime > fc.lastTime) {
                    fc.lastTime = currentTime;
                }
                val = (N > 0 && fc.pwmMasterOutDirectIndices[0] >= 0 && fc.pwmMasterOutDirectIndices[0] < (int)flatControlSignals.size()) 
                      ? flatControlSignals[fc.pwmMasterOutDirectIndices[0]] : 0.0;
            }
            else if (fc.type == ComponentType::PWM_Generator) {
                double vMod = fc.in0Ptr ? *fc.in0Ptr : 0.0;
                double fcHz = (fc.freq > 0.0) ? fc.freq : 10000.0;
                double minVal = fc.minVal;
                double maxVal = fc.maxVal;
                if (maxVal <= minVal) maxVal = minVal + 1.0;
                double deadTime = fc.delayDuration;
                double Tc = 1.0 / fcHz;

                double tLocal = std::fmod(currentTime, Tc);
                if (tLocal < 0.0) tLocal += Tc;

                double triVal = (tLocal < Tc / 2.0) 
                                ? minVal + (maxVal - minVal) * (tLocal / (Tc / 2.0))
                                : maxVal - (maxVal - minVal) * ((tLocal - Tc / 2.0) / (Tc / 2.0));

                int targetDirect = (vMod >= triVal) ? 1 : 0;

                if (deadTime > 0.0) {
                    int targetCompl = (targetDirect == 0) ? 1 : 0;
                    if (pass == 0 && commitState && currentTime > fc.lastTime) {
                        if (fc.pwmMasterLastTargetDirect.empty()) {
                            fc.pwmMasterLastTargetDirect.assign(1, 0);
                            fc.pwmMasterLastTargetCompl.assign(1, 0);
                            fc.pwmMasterLastTransDirect.assign(1, 0.0);
                            fc.pwmMasterLastTransCompl.assign(1, 0.0);
                        }
                        if (targetDirect == 1 && fc.pwmMasterLastTargetDirect[0] == 0) {
                            fc.pwmMasterLastTransDirect[0] = currentTime;
                        }
                        if (targetCompl == 1 && fc.pwmMasterLastTargetCompl[0] == 0) {
                            fc.pwmMasterLastTransCompl[0] = currentTime;
                        }
                        fc.pwmMasterLastTargetDirect[0] = targetDirect;
                        fc.pwmMasterLastTargetCompl[0] = targetCompl;
                        fc.lastTime = currentTime;
                    }
                    double transT = (!fc.pwmMasterLastTransDirect.empty()) ? fc.pwmMasterLastTransDirect[0] : 0.0;
                    val = (targetDirect == 1 && (currentTime - transT >= deadTime - 1e-12)) ? 1.0 : 0.0;
                } else {
                    val = (targetDirect == 1) ? 1.0 : 0.0;
                }
            }
            else if (fc.type == ComponentType::LUT_3D) {
                // Trilinear interpolation of table[i][j][k] over axes x, y and z. Inputs are
                // clamped to the table range. A table whose shape does not match its axes
                // gives 0 instead of reading out of bounds.
                auto sigAt = [&](size_t i) -> double {
                    if (i < fc.inputSigIndices.size()) {
                        const int idx = fc.inputSigIndices[i];
                        if (idx >= 0 && idx < (int)flatControlSignals.size()) return flatControlSignals[idx];
                    }
                    return 0.0;
                };
                const double u1 = sigAt(0), u2 = sigAt(1), u3 = sigAt(2);

                auto parseV = [](std::string s) -> std::vector<double> {
                    std::vector<double> vec;
                    for (char& c : s) if (c == '[' || c == ']' || c == ',') c = ' ';
                    std::stringstream ss(s); double v;
                    while (ss >> v) vec.push_back(v);
                    return vec;
                };

                // Reads a nested "[[[..], [..]], [[..]]]" table into cube[i][j][k].
                auto parseCube = [](const std::string& src) -> std::vector<std::vector<std::vector<double>>> {
                    std::vector<std::vector<std::vector<double>>> cube;
                    int depth = 0;
                    const char* p = src.c_str();
                    while (*p) {
                        if (*p == '[') {
                            ++depth;
                            if (depth == 2) cube.emplace_back();
                            else if (depth == 3 && !cube.empty()) cube.back().emplace_back();
                            ++p;
                        } else if (*p == ']') {
                            --depth;
                            ++p;
                        } else if (std::isdigit((unsigned char)*p) || *p == '-' || *p == '+' || *p == '.') {
                            char* end = nullptr;
                            const double v = std::strtod(p, &end);
                            if (end == p) { ++p; continue; }
                            if (depth == 3 && !cube.empty() && !cube.back().empty()) cube.back().back().push_back(v);
                            p = end;
                        } else {
                            ++p;
                        }
                    }
                    return cube;
                };

                std::vector<double> ax = parseV(fc.polarity);
                std::vector<double> ay = parseV(fc.vPlotKey);
                std::vector<double> az = parseV(fc.vBetaKey);
                auto cube = parseCube(fc.vAlphaKey);
                if (ax.empty()) ax = {0.0, 1.0};
                if (ay.empty()) ay = {0.0, 1.0};
                if (az.empty()) az = {0.0, 1.0};
                if (cube.empty()) cube = {{{0.0, 1.0}, {1.0, 2.0}}, {{1.0, 2.0}, {2.0, 3.0}}};

                const size_t nx = ax.size(), ny = ay.size(), nz = az.size();
                bool shapeOk = nx >= 2 && ny >= 2 && nz >= 2 && cube.size() == nx;
                for (size_t i = 0; shapeOk && i < nx; ++i) {
                    if (cube[i].size() != ny) shapeOk = false;
                    for (size_t j = 0; shapeOk && j < ny; ++j)
                        if (cube[i][j].size() != nz) shapeOk = false;
                }
                val = 0.0;
                if (shapeOk) {
                    // Cell index and fractional position along one axis.
                    auto cell = [](const std::vector<double>& axis, double u, double& frac) -> size_t {
                        size_t i = 0;
                        for (size_t k = 0; k + 1 < axis.size(); ++k) if (u >= axis[k]) i = k;
                        const double d = axis[i + 1] - axis[i];
                        frac = (d > 0.0) ? std::clamp((u - axis[i]) / d, 0.0, 1.0) : 0.0;
                        return i;
                    };
                    double tx = 0.0, ty = 0.0, tz = 0.0;
                    const size_t i0 = cell(ax, u1, tx), j0 = cell(ay, u2, ty), k0 = cell(az, u3, tz);
                    for (size_t a = 0; a < 2; ++a) {
                        for (size_t b = 0; b < 2; ++b) {
                            for (size_t c = 0; c < 2; ++c) {
                                const double w = (a ? tx : 1.0 - tx) * (b ? ty : 1.0 - ty) * (c ? tz : 1.0 - tz);
                                val += w * cube[i0 + a][j0 + b][k0 + c];
                            }
                        }
                    }
                }
            }
            else if (fc.type == ComponentType::LUT_2D) {
                double u1 = fc.in0Ptr ? *fc.in0Ptr : 0.0;
                double u2 = fc.in1Ptr ? *fc.in1Ptr : 0.0;

                auto parseV = [](std::string s) -> std::vector<double> {
                    std::vector<double> vec;
                    if (s.empty()) return vec;
                    for (char& c : s) if (c == '[' || c == ']' || c == ',') c = ' ';
                    std::stringstream ss(s); double v;
                    while (ss >> v) vec.push_back(v);
                    return vec;
                };

                auto parseM = [](const std::string& src) -> std::vector<std::vector<double>> {
                    std::vector<std::vector<double>> mat;
                    if (src.empty()) return mat;
                    // Accepts "a, b; c, d" and the nested "[[a, b], [c, d]]" form used by the
                    // defaults. Every inner '[' that opens a new row becomes a ';' separator.
                    // Before this, the nested form was flattened into a single row.
                    std::string s;
                    s.reserve(src.size());
                    int depth = 0;
                    for (char c : src) {
                        if (c == '[') {
                            ++depth;
                            if (depth == 2 && !s.empty()) s += ';';
                        } else if (c == ']') {
                            --depth;
                        } else if (c == ',') {
                            s += ' ';
                        } else {
                            s += c;
                        }
                    }
                    std::stringstream ss(s); std::string line;
                    while (std::getline(ss, line, ';')) {
                        std::stringstream lineSS(line); std::vector<double> row; double v;
                        while (lineSS >> v) row.push_back(v);
                        if (!row.empty()) mat.push_back(row);
                    }
                    return mat;
                };

                std::vector<double> rowX = parseV(fc.polarity);
                std::vector<double> colY = parseV(fc.vPlotKey);
                std::vector<std::vector<double>> tableZ = parseM(fc.vAlphaKey);

                if (rowX.empty()) rowX = {0.0, 1.0};
                if (colY.empty()) colY = {0.0, 1.0};
                if (tableZ.empty()) tableZ = {{0.0, 1.0}, {1.0, 2.0}};

                size_t nr = rowX.size();
                size_t nc = colY.size();

                size_t rIdx = 0;
                for (size_t i = 0; i < nr - 1; ++i) if (u1 >= rowX[i]) rIdx = i;
                if (rIdx >= nr - 1 && nr >= 2) rIdx = nr - 2;

                size_t cIdx = 0;
                for (size_t j = 0; j < nc - 1; ++j) if (u2 >= colY[j]) cIdx = j;
                if (cIdx >= nc - 1 && nc >= 2) cIdx = nc - 2;

                double x0 = rowX[rIdx], x1 = (rIdx + 1 < nr) ? rowX[rIdx + 1] : x0 + 1.0;
                double y0 = colY[cIdx], y1 = (cIdx + 1 < nc) ? colY[cIdx + 1] : y0 + 1.0;

                double tx = (x1 > x0) ? (u1 - x0) / (x1 - x0) : 0.0;
                double ty = (y1 > y0) ? (u2 - y0) / (y1 - y0) : 0.0;
                tx = std::clamp(tx, 0.0, 1.0);
                ty = std::clamp(ty, 0.0, 1.0);

                double z00 = (rIdx < tableZ.size() && cIdx < tableZ[rIdx].size()) ? tableZ[rIdx][cIdx] : 0.0;
                double z01 = (rIdx < tableZ.size() && cIdx + 1 < tableZ[rIdx].size()) ? tableZ[rIdx][cIdx + 1] : z00;
                double z10 = (rIdx + 1 < tableZ.size() && cIdx < tableZ[rIdx + 1].size()) ? tableZ[rIdx + 1][cIdx] : z00;
                double z11 = (rIdx + 1 < tableZ.size() && cIdx + 1 < tableZ[rIdx + 1].size()) ? tableZ[rIdx + 1][cIdx + 1] : z01;

                val = (1.0 - tx) * (1.0 - ty) * z00 + tx * (1.0 - ty) * z10 + (1.0 - tx) * ty * z01 + tx * ty * z11;
            }
            else if (fc.type == ComponentType::RmsVal) {
                double inVal = fc.in0Ptr ? *fc.in0Ptr : 0.0;
                double fn = (fc.freq > 0.0) ? fc.freq : 50.0;
                // Buffer LENGTH in samples for the one-cycle window; a window length, not
                // an integration step, so it stays on the nominal step deliberately. See
                // the note on PerAvg above.
                int N = (int)std::round(1.0 / (fn * (config.stepSize > 0 ? config.stepSize : 1e-4)));
                if (N < 2) N = 2;
                if (commitState && currentTime > fc.lastTime) {
                    fc.shiftBuffer.push_back(inVal * inVal);
                    if ((int)fc.shiftBuffer.size() > N) fc.shiftBuffer.erase(fc.shiftBuffer.begin());
                    fc.lastTime = currentTime;
                }
                double sumSq = 0.0;
                for (double v2 : fc.shiftBuffer) sumSq += v2;
                val = std::sqrt(fc.shiftBuffer.empty() ? 0.0 : (sumSq / fc.shiftBuffer.size()));
            }
            else if (fc.type == ComponentType::ThdVal) {
                double inVal = fc.in0Ptr ? *fc.in0Ptr : 0.0;
                double fn = (fc.freq > 0.0) ? fc.freq : 50.0;
                // Buffer LENGTH in samples for the one-cycle window; a window length, not
                // an integration step, so it stays on the nominal step deliberately. See
                // the note on PerAvg above.
                int N = (int)std::round(1.0 / (fn * (config.stepSize > 0 ? config.stepSize : 1e-4)));
                if (N < 2) N = 2;
                // Sliding DFT over the last N samples. Running sums are kept over absolute
                // sample indices n; the fundamental magnitude does not depend on the
                // window's starting phase, so this equals the windowed DFT magnitude.
                const double omega = 2.0 * 3.141592653589793 / N;
                if (commitState && currentTime > fc.lastTime) {
                    if ((int)fc.thdRing.size() != N) {
                        fc.thdRing.assign(N, 0.0);
                        fc.thdHead = 0; fc.thdFill = 0; fc.thdCount = 0;
                        fc.thdRe = fc.thdIm = fc.thdSumSq = 0.0;
                    }
                    if (fc.thdFill == N) {   // remove the oldest sample from the sums
                        const double old = fc.thdRing[fc.thdHead];
                        const long long nOld = fc.thdCount - N;
                        fc.thdRe -= old * std::cos(omega * (double)nOld);
                        fc.thdIm -= old * std::sin(omega * (double)nOld);
                        fc.thdSumSq -= old * old;
                    } else {
                        fc.thdFill++;
                    }
                    fc.thdRing[fc.thdHead] = inVal;
                    fc.thdHead = (fc.thdHead + 1) % N;
                    fc.thdRe += inVal * std::cos(omega * (double)fc.thdCount);
                    fc.thdIm += inVal * std::sin(omega * (double)fc.thdCount);
                    fc.thdSumSq += inVal * inVal;
                    fc.thdCount++;
                    fc.lastTime = currentTime;
                }
                const int bufSize = fc.thdFill;
                const double sumSq = std::max(0.0, fc.thdSumSq);
                double totalRms = std::sqrt(bufSize > 0 ? sumSq / bufSize : 0.0);

                // Fundamental component amplitude (H1)
                const double norm = 2.0 / (bufSize > 0 ? bufSize : 1);
                double Re = norm * fc.thdRe;
                double Im = norm * fc.thdIm;
                double fundRms = std::sqrt(Re * Re + Im * Im) / 1.4142135623730951;

                if (fundRms < 1e-6) val = 0.0;
                else {
                    double harmonicSquare = totalRms * totalRms - fundRms * fundRms;
                    val = (harmonicSquare > 0.0) ? std::sqrt(harmonicSquare) / fundRms : 0.0;
                }
            }
            else if (fc.type == ComponentType::PllLoop) {
                double inVal = fc.in0Ptr ? *fc.in0Ptr : 0.0;
                double dt = dtNow;
                double fn = (fc.freq > 0.0) ? fc.freq : 50.0;
                double w0 = 2.0 * 3.141592653589793 * fn;
                if (currentTime == 0.0 && pass == 0 && commitState) {
                    fc.stateVal = 0.0;     // theta
                    fc.filterState = w0;  // omega
                    fc.pllHasOmega = false;
                }
                // Phase detector error (inVal * cos(theta))
                double err = inVal * std::cos(fc.stateVal);
                double Kp = (fc.gain > 0.0) ? fc.gain : 20.0;
                double Ki = (fc.maxVal > 0.0) ? fc.maxVal : 1000.0;
                double omega_total = fc.filterState + Kp * err;
                // Advance once per step. The block is evaluated in two passes, and the
                // state used to move on both, running the loop at twice its rate.
                if (pass == 0 && commitState && currentTime > 0.0) {
                    fc.filterState += Ki * err * dt;
                    fc.stateVal = std::fmod(fc.stateVal + omega_total * dt, 2.0 * 3.141592653589793);
                    if (fc.stateVal < 0.0) fc.stateVal += 2.0 * 3.141592653589793;
                    fc.pllOmega = omega_total;
                    fc.pllHasOmega = true;
                }

                double theta = fc.stateVal;
                // Report the omega that advanced theta (see PLL_3PH above).
                double freqEstimated = (fc.pllHasOmega ? fc.pllOmega : omega_total) / (2.0 * 3.141592653589793);
                double cosVal = std::cos(theta);
                double sinVal = std::sin(theta);

                if (fc.outputSigIndices.size() > 0 && fc.outputSigIndices[0] >= 0 && fc.outputSigIndices[0] < (int)flatControlSignals.size()) flatControlSignals[fc.outputSigIndices[0]] = theta;
                if (fc.outputSigIndices.size() > 1 && fc.outputSigIndices[1] >= 0 && fc.outputSigIndices[1] < (int)flatControlSignals.size()) flatControlSignals[fc.outputSigIndices[1]] = freqEstimated;
                if (fc.outputSigIndices.size() > 2 && fc.outputSigIndices[2] >= 0 && fc.outputSigIndices[2] < (int)flatControlSignals.size()) flatControlSignals[fc.outputSigIndices[2]] = cosVal;
                if (fc.outputSigIndices.size() > 3 && fc.outputSigIndices[3] >= 0 && fc.outputSigIndices[3] < (int)flatControlSignals.size()) flatControlSignals[fc.outputSigIndices[3]] = sinVal;
                val = theta;
            }
            else if (fc.type == ComponentType::Offset) {
                double inVal = fc.in0Ptr ? *fc.in0Ptr : 0.0;
                val = inVal + fc.thresholdVal;
            }
            else if (fc.type == ComponentType::Signum) {
                double inVal = fc.in0Ptr ? *fc.in0Ptr : 0.0;
                val = (inVal > 0.0) ? 1.0 : ((inVal < 0.0) ? -1.0 : 0.0);
            }
            else if (fc.type == ComponentType::Divide) {
                double num = fc.in0Ptr ? *fc.in0Ptr : 0.0;
                double den = fc.in1Ptr ? *fc.in1Ptr : 1.0;
                val = (std::abs(den) < 1e-15) ? (num / (den >= 0.0 ? 1e-15 : -1e-15)) : (num / den);
            }
            else if (fc.type == ComponentType::DataTypeConv) {
                double inVal = fc.in0Ptr ? *fc.in0Ptr : 0.0;
                const std::string& dt = fc.polarity;
                if (dt == "boolean" || dt == "bool") val = (inVal > 0.5) ? 1.0 : 0.0;
                else if (dt == "integer" || dt == "int") val = std::round(inVal);
                else val = inVal;
            }
            else if (fc.type == ComponentType::StateMachine) {
                double inVal = fc.in0Ptr ? *fc.in0Ptr : 0.0;
                val = inVal;
            }
            else if (fc.type == ComponentType::SummingJunction) {
                double sum = 0.0;
                size_t nIn = std::max((size_t)1, fc.inputSigIndices.size());
                for (size_t i = 0; i < nIn; ++i) {
                    double vIn = 0.0;
                    if (i < fc.inputSigIndices.size() && fc.inputSigIndices[i] >= 0 && fc.inputSigIndices[i] < (int)flatControlSignals.size()) {
                        vIn = flatControlSignals[fc.inputSigIndices[i]];
                    } else if (i == 0 && fc.in0Ptr) {
                        vIn = *fc.in0Ptr;
                    } else if (i == 1 && fc.in1Ptr) {
                        vIn = *fc.in1Ptr;
                    }
                    char sChar = (i < fc.polarity.size()) ? fc.polarity[i] : '+';
                    if (sChar == '-') sum -= vIn;
                    else sum += vIn;
                }
                val = sum;
            }
            else if (fc.type == ComponentType::Product) {
                double prod = 1.0;
                size_t nIn = std::max((size_t)1, fc.inputSigIndices.size());
                for (size_t i = 0; i < nIn; ++i) {
                    double vIn = 1.0;
                    if (i < fc.inputSigIndices.size() && fc.inputSigIndices[i] >= 0 && fc.inputSigIndices[i] < (int)flatControlSignals.size()) {
                        vIn = flatControlSignals[fc.inputSigIndices[i]];
                    } else if (i == 0 && fc.in0Ptr) {
                        vIn = *fc.in0Ptr;
                    } else if (i == 1 && fc.in1Ptr) {
                        vIn = *fc.in1Ptr;
                    }
                    char sChar = (i < fc.polarity.size()) ? fc.polarity[i] : '*';
                    if (sChar == '/') {
                        prod /= (std::abs(vIn) < 1e-15 ? 1e-15 : vIn);
                    } else {
                        prod *= vIn;
                    }
                }
                val = prod;
            }
            else if (fc.type == ComponentType::PulseGenerator) {
                double p = (fc.period > 0.0) ? fc.period : 0.0001;
                double w = (fc.width > 0.0 && fc.width <= 1.0) ? fc.width : 0.5;
                double d = fc.delay;
                double amp = (fc.amplitude != 0.0) ? fc.amplitude : 1.0;

                double tRel = currentTime - d;
                if (tRel < 0.0) {
                    val = 0.0;
                } else {
                    double phase = std::fmod(tRel, p);
                    if (phase < 0.0) phase += p;
                    val = (phase < p * w) ? amp : 0.0;
                }
            }
            else if (fc.type == ComponentType::Triangle_Carrier) {
                bool extPhase = (fc.polarity == "external");
                bool extFreq = (fc.vPlotKey == "external");

                double phase_deg = extPhase ? (fc.in1Ptr ? *fc.in1Ptr : 0.0) : fc.delay;
                double freq = extFreq ? (fc.in0Ptr ? *fc.in0Ptr : (fc.freq > 0 ? fc.freq : 10000.0)) : (fc.freq > 0 ? fc.freq : 10000.0);

                double min = fc.minVal;
                double max = fc.maxVal;

                double t_norm = std::fmod(currentTime * freq + phase_deg / 360.0, 1.0);
                if (t_norm < 0.0) t_norm += 1.0;

                val = (t_norm < 0.5) ? min + (max - min) * (t_norm / 0.5) : max - (max - min) * ((t_norm - 0.5) / 0.5);
            }
            else if (fc.type == ComponentType::Gain) {
                double inVal = fc.in0Ptr ? *fc.in0Ptr : 0.0;
                val = fc.gain * inVal;
            }
            else if (fc.type == ComponentType::Comparator) {
                double v0 = fc.in0Ptr ? *fc.in0Ptr : 0.0;
                double v1 = fc.in1Ptr ? *fc.in1Ptr : 0.0;
                val = (v0 >= v1) ? 1.0 : 0.0;
            }
            else if (fc.type == ComponentType::PI_Controller) {
                double err = fc.in0Ptr ? *fc.in0Ptr : 0.0;
                if (pass == 0 && commitState && fc.stateIdx >= 0 && fc.stateIdx < (int)flatPiIntegratorState.size()) {
                    flatPiIntegratorState[fc.stateIdx] += err * dtNow;
                }
                double piVal = (fc.stateIdx >= 0 && fc.stateIdx < (int)flatPiIntegratorState.size()) ? flatPiIntegratorState[fc.stateIdx] : 0.0;
                val = fc.Kp * err + fc.Ki * piVal;
            }
            else if (fc.type == ComponentType::CustomScript) {
                // The buffer is shared by all scripts: clear it so a script never sees
                // the previous script's inputs, and grow it rather than write past 20.
                if (scriptInValsBuf.size() < fc.inputSigIndices.size()) scriptInValsBuf.resize(fc.inputSigIndices.size(), 0.0);
                std::fill(scriptInValsBuf.begin(), scriptInValsBuf.end(), 0.0);
                for (size_t i = 0; i < fc.inputSigIndices.size(); ++i) {
                    int inIdx = fc.inputSigIndices[i];
                    double inVal = (inIdx >= 0 && inIdx < (int)flatControlSignals.size()) ? flatControlSignals[inIdx] : 0.0;
                    scriptInValsBuf[i] = inVal;
                }

                auto cIt = cscriptEngines.find(fc.id);
                if (cIt != cscriptEngines.end()) {
                    if (pass == 0 && commitState) {
                        cIt->second.step(currentTime, scriptInValsBuf, dtNow);
                    }

                    for (size_t i = 0; i < fc.outputSigIndices.size(); ++i) {
                        int outSigIdx = fc.outputSigIndices[i];
                        if (outSigIdx >= 0 && outSigIdx < (int)flatControlSignals.size()) {
                            flatControlSignals[outSigIdx] = cIt->second.getOutput(i);
                        }
                    }

                    for (size_t i = 0; i < fc.customPlotVarIndices.size(); ++i) {
                        int cpIdx = fc.customPlotVarIndices[i];
                        if (cpIdx >= 0 && cpIdx < (int)flatControlSignals.size()) {
                            flatControlSignals[cpIdx] = cIt->second.getVar(fc.customScriptVarNames[i]);
                        }
                    }
                }
            }
            else if (fc.type == ComponentType::EdgeDetector) {
                double inVal = fc.in0Ptr ? *fc.in0Ptr : 0.0;
                double pulseW = (fc.width > 0.0) ? fc.width : 1e-3;
                bool rising = (fc.polarity == "rising" || fc.polarity.empty());
                bool falling = (fc.polarity == "falling");
                bool either = (fc.polarity == "either" || fc.polarity == "both");

                double prevIn = fc.esr; // store prev input in esr field for fast state
                double trigTime = fc.delay; // store trig time in delay field
                bool isActive = (fc.minVal > 0.5); // store active state in minVal

                bool detected = false;
                if (rising && prevIn <= 0.5 && inVal > 0.5) detected = true;
                else if (falling && prevIn > 0.5 && inVal <= 0.5) detected = true;
                else if (either && ((prevIn <= 0.5 && inVal > 0.5) || (prevIn > 0.5 && inVal <= 0.5))) detected = true;

                if (pass == 0 && commitState) fc.esr = inVal; // update prev input

                if (detected && !isActive) {
                    isActive = true;
                    trigTime = currentTime;
                    if (pass == 0 && commitState) { fc.minVal = 1.0; fc.delay = trigTime; }
                }
                if (isActive && trigTime >= 0.0 && (currentTime - trigTime) >= pulseW - 1e-12) {
                    isActive = false;
                    if (pass == 0 && commitState) fc.minVal = 0.0;
                }
                val = isActive ? 1.0 : 0.0;
            }
            else if (fc.type == ComponentType::Polynomial) {
                double u = fc.in0Ptr ? *fc.in0Ptr : 0.0;
                double polyVal = 0.0;
                if (!fc.polyCoeffs.empty()) {
                    for (size_t i = 0; i < fc.polyCoeffs.size(); ++i) {
                        polyVal = polyVal * u + fc.polyCoeffs[i];
                    }
                } else {
                    polyVal = u;
                }
                val = polyVal;
            }
            else if (fc.type == ComponentType::AlgebraicConstraint) {
                double f_z = fc.in0Ptr ? *fc.in0Ptr : 0.0;
                double z_prev = fc.stateVal;
                double z_new = z_prev - fc.gain * f_z;
                fc.stateVal = z_new;
                val = z_new;
            }
            else if (fc.type == ComponentType::Inport || fc.type == ComponentType::Outport || fc.type == ComponentType::PhysicalInport || fc.type == ComponentType::PhysicalOutport || fc.type == ComponentType::EnablePort || fc.type == ComponentType::TriggerPort || fc.type == ComponentType::BusCreator || fc.type == ComponentType::BusSelector || fc.type == ComponentType::Terminator) {
                double inVal = fc.in0Ptr ? *fc.in0Ptr : 0.0;
                val = inVal;
            }
            else if (fc.type == ComponentType::KeyTrigger) {
                val = (fc.val != 0.0) ? fc.val : 1.0;
            }
            else if (fc.type == ComponentType::UnifiedProbe) {
                double pVal = 0.0;
                if (fc.ctrlSigPtr && *fc.ctrlSigPtr != 0.0) pVal = *fc.ctrlSigPtr;
                else if (fc.targetPtr && *fc.targetPtr != 0.0) pVal = *fc.targetPtr;

                if (pVal == 0.0 && fc.ctrlSigSignalIdx >= 0 && fc.ctrlSigSignalIdx < (int)flatControlSignals.size()) {
                    pVal = flatControlSignals[fc.ctrlSigSignalIdx];
                }
                val = pVal;

                for (int sIdx : fc.outputSigIndices) {
                    if (sIdx >= 0 && sIdx < (int)flatControlSignals.size()) {
                        flatControlSignals[sIdx] = val;
                    }
                }
            }

            if (fc.outPtr) {
                *fc.outPtr = val;
            }
            if (fc.outSignalIdx >= 0 && fc.outSignalIdx < (int)flatControlSignals.size()) {
                flatControlSignals[fc.outSignalIdx] = val;
            }
        }
    }
}

bool CircuitSimulator::updateDeviceStates() {
    bool changed = false;
    for (const auto& fc : fastPhysComps) {
        if (fc.type == ComponentType::Diode) {
            double v1 = (fc.n1 >= 0 && fc.n1 < totalDim) ? X[fc.n1] : 0.0;
            double v2 = (fc.n2 >= 0 && fc.n2 < totalDim) ? X[fc.n2] : 0.0;
            double vDiff = v1 - v2;

            double currentState = (fc.stateIdx >= 0 && fc.stateIdx < (int)flatDiodeStates.size()) ? flatDiodeStates[fc.stateIdx] : 0.0;
            double newState = currentState;

            if (currentState > 0.5) {
                double R = fc.Ron;
                if (R < 1e-6) R = 1e-6;
                double iForward = (vDiff - fc.Vvd) / R;
                if (iForward < -1e-5) newState = 0.0;
            } else {
                if (vDiff >= fc.Vvd + 1e-4) newState = 1.0;
            }

            if (fc.stateIdx >= 0 && fc.stateIdx < (int)flatDiodeStates.size()) {
                if (std::abs(newState - flatDiodeStates[fc.stateIdx]) > 0.1) {
                    flatDiodeStates[fc.stateIdx] = newState;
                    changed = true;
                }
            }
        } else if (fc.type == ComponentType::Thyristor) {
            double v1 = (fc.n1 >= 0 && fc.n1 < totalDim) ? X[fc.n1] : 0.0;
            double v2 = (fc.n2 >= 0 && fc.n2 < totalDim) ? X[fc.n2] : 0.0;
            double vDiff = v1 - v2;
            double vGate = fc.ctrlSigPtr ? *fc.ctrlSigPtr : 0.0;


            double currentState = (fc.stateIdx >= 0 && fc.stateIdx < (int)flatDiodeStates.size()) ? flatDiodeStates[fc.stateIdx] : 0.0;
            double newState = currentState;

            if (currentState > 0.5) {
                double R = fc.Ron;
                if (R < 1e-6) R = 1e-6;
                double iForward = (vDiff - fc.Vvd) / R;
                double ih = (fc.Iholding > 0.0) ? fc.Iholding : 0.01;
                if (iForward < ih) newState = 0.0;
            } else {
                double vgt = (fc.Vgt > 0.0) ? fc.Vgt : 0.5;
                if (vDiff >= fc.Vvd && vGate >= vgt) newState = 1.0;
            }

            if (fc.stateIdx >= 0 && fc.stateIdx < (int)flatDiodeStates.size()) {
                if (std::abs(newState - flatDiodeStates[fc.stateIdx]) > 0.1) {
                    flatDiodeStates[fc.stateIdx] = newState;
                    changed = true;
                }
            }
        } else if (fc.type == ComponentType::MOSFET) {
            double v1 = (fc.n1 >= 0 && fc.n1 < totalDim) ? X[fc.n1] : 0.0;
            double v2 = (fc.n2 >= 0 && fc.n2 < totalDim) ? X[fc.n2] : 0.0;
            double vGate = fc.ctrlSigPtr ? *fc.ctrlSigPtr : 0.0;

            bool isGateOn = (vGate > 0.5);
            // Body Diode is anti-parallel from Source (n2) to Drain (n1)
            bool isBodyDiodeOn = (v2 - v1 >= fc.Vvd - 1e-4);

            double newState = (isGateOn || isBodyDiodeOn) ? 1.0 : 0.0;

            if (fc.stateIdx >= 0 && fc.stateIdx < (int)flatDiodeStates.size()) {
                if (std::abs(newState - flatDiodeStates[fc.stateIdx]) > 0.1) {
                    flatDiodeStates[fc.stateIdx] = newState;
                    changed = true;
                }
            }
        } else if (fc.type == ComponentType::IGBT || fc.type == ComponentType::GTO ||
                   fc.type == ComponentType::IGCT || fc.type == ComponentType::BJT) {
            // Unidirectional gate/base turn-on AND turn-off capable devices. They
            // conduct only from n1 -> n2 while driven, and block reverse voltage.
            double v1 = (fc.n1 >= 0 && fc.n1 < totalDim) ? X[fc.n1] : 0.0;
            double v2 = (fc.n2 >= 0 && fc.n2 < totalDim) ? X[fc.n2] : 0.0;
            double vGate = fc.ctrlSigPtr ? *fc.ctrlSigPtr : 0.0;

            bool isDriven = (vGate > 0.5);
            bool isForward = ((v1 - v2) >= fc.Vvd - 1e-4);
            double newState = (isDriven && isForward) ? 1.0 : 0.0;

            if (fc.stateIdx >= 0 && fc.stateIdx < (int)flatDiodeStates.size()) {
                if (std::abs(newState - flatDiodeStates[fc.stateIdx]) > 0.1) {
                    flatDiodeStates[fc.stateIdx] = newState;
                    changed = true;
                }
            }
        } else if (fc.type == ComponentType::IGBTDiode) {
            // IGBT with anti-parallel freewheeling diode: forward conduction is gated,
            // reverse conduction happens whenever the diode is forward biased.
            double v1 = (fc.n1 >= 0 && fc.n1 < totalDim) ? X[fc.n1] : 0.0;
            double v2 = (fc.n2 >= 0 && fc.n2 < totalDim) ? X[fc.n2] : 0.0;
            double vGate = fc.ctrlSigPtr ? *fc.ctrlSigPtr : 0.0;

            bool isDriven = (vGate > 0.5);
            bool isForward = ((v1 - v2) >= fc.Vvd - 1e-4);
            bool isFreewheeling = ((v2 - v1) >= fc.Vvd - 1e-4);
            double newState = ((isDriven && isForward) || isFreewheeling) ? 1.0 : 0.0;

            if (fc.stateIdx >= 0 && fc.stateIdx < (int)flatDiodeStates.size()) {
                if (std::abs(newState - flatDiodeStates[fc.stateIdx]) > 0.1) {
                    flatDiodeStates[fc.stateIdx] = newState;
                    changed = true;
                }
            }
        } else if (fc.type == ComponentType::JFET) {
            // Gate-controlled bidirectional channel (no forward offset, no body diode).
            double vGate = fc.ctrlSigPtr ? *fc.ctrlSigPtr : 0.0;
            double newState = (vGate > 0.5) ? 1.0 : 0.0;

            if (fc.stateIdx >= 0 && fc.stateIdx < (int)flatDiodeStates.size()) {
                if (std::abs(newState - flatDiodeStates[fc.stateIdx]) > 0.1) {
                    flatDiodeStates[fc.stateIdx] = newState;
                    changed = true;
                }
            }
        } else if (fc.type == ComponentType::Switch) {
            double ctrlVal = fc.ctrlSigPtr ? *fc.ctrlSigPtr : 0.0;

            double newState = (ctrlVal > 0.5) ? 1.0 : 0.0;
            if (fc.stateIdx >= 0 && fc.stateIdx < (int)flatSwitchStates.size()) {
                if (std::abs(newState - flatSwitchStates[fc.stateIdx]) > 0.1) {
                    flatSwitchStates[fc.stateIdx] = newState;
                    changed = true;
                }
            }
        }
    }
    if (changed) {
        forceBackwardEulerSteps = 2;
    }
    return changed;
}

void CircuitSimulator::assembleMNA(double currentTime, double dtStep) {
    // Reset only what the dynamic stamps touched. The first assembly has to lay
    // down the whole static matrix; after that, positions absent from
    // dynStampIdx have never been written and still hold their K_static value.
    if (!dynStampBaseCopied) {
        std::copy(K_static.begin(), K_static.end(), K.begin());
        dynStampBaseCopied = true;
    } else {
        const double* src = K_static.data();
        double* dst = K.data();
        for (int idx : dynStampIdx) dst[idx] = src[idx];
    }
    std::fill(B.begin(), B.end(), 0.0);

    // The step actually being taken. This used to read config.stepSize, which meant the
    // companion models were stamped with the nominal step even when the loop advanced
    // time by something else - so the old "variable" path was not merely unused, it was
    // inconsistent, and the capacitor's stamp disagreed with its own state update.
    double dt = (dtStep > 0.0) ? dtStep : config.stepSize;
    if (dt <= 0) dt = 1e-6;

    // Integration mode affects the inductor stamp, so a trapezoidal <-> backward Euler
    // transition invalidates any existing factorization.
    const bool useTrapNow = (config.solver == "trapezoidal" || config.solver == "trap" || config.solver == "rk4")
                            && (forceBackwardEulerSteps <= 0);
    if (!trapModeStampValid || useTrapNow != trapModeStamped) {
        trapModeStamped = useTrapNow;
        trapModeStampValid = true;
        matrixKChanged = true;
    }

    for (auto& fc : fastPhysComps) {
        int n1 = fc.n1;
        int n2 = fc.n2;

        if (fc.type == ComponentType::Resistor && fc.isVariable) {
            // Signal-controlled resistor: conductance taken from the control signal.
            double Rtotal = liveElementValue(fc, 1e-6) + fc.esr;
            if (Rtotal < 1e-6) Rtotal = 1e-6;
            double g = 1.0 / Rtotal;
            if (g != fc.gStamped) { fc.gStamped = g; matrixKChanged = true; }

            stampConductance(n1, n2, g);
        }
        else if (fc.type == ComponentType::Capacitor) {
            double C = liveElementValue(fc, 1e-15);
            if (C < 1e-15) C = 1e-15;

            // Companion model of C in series with its ESR:
            //   BE  : vC = vC0 + (h/C)*i               -> i = (v - vC0) / (h/C + esr)
            //   trap: vC = vC0 + (h/2C)*(i + i0)       -> i = (v - vC0 - (h/2C)*i0) / (h/2C + esr)
            // The capacitor used to be stamped with BE even under the trapezoidal
            // solver, so "trapezoidal" was first order (and numerically damped) in
            // every capacitor.
            const bool trapC = useTrapNow && fc.stateIdx >= 0 && fc.stateIdx < (int)flatCapCurrents.size();
            const double hc = trapC ? dt / (2.0 * C) : dt / C;
            double rEq = hc + fc.esr;
            double gEq = 1.0 / rEq;
            if (gEq != fc.gStamped) { fc.gStamped = gEq; matrixKChanged = true; }
            double vCapPrev = (fc.stateIdx >= 0 && fc.stateIdx < (int)flatCapVoltages.size()) ? flatCapVoltages[fc.stateIdx] : 0.0;
            double iEq = gEq * (trapC ? vCapPrev + hc * flatCapCurrents[fc.stateIdx] : vCapPrev);

            stampConductance(n1, n2, gEq);

            if (n1 >= 0) B[n1] += iEq;
            if (n2 >= 0) B[n2] -= iEq;
        }
        else if (fc.type == ComponentType::Inductor) {
            double L = liveElementValue(fc, 1e-12);
            if (L < 1e-12) L = 1e-12;
            int lIdx = fc.lIdx;
            const bool useTrap = useTrapNow;

            // Track the stamped resistance so a changed L (variable inductor) marks the
            // factorization stale; the trap/BE transition is handled above.
            {
                double rEqNow = useTrap ? ((2.0 * L / dt) + fc.esr) : ((L / dt) + fc.esr);
                if (rEqNow != fc.gStamped) { fc.gStamped = rEqNow; matrixKChanged = true; }
            }

            if (useTrap) {
                double rEq = (2.0 * L / dt) + fc.esr;
                double iPrev = (fc.stateIdx >= 0 && fc.stateIdx < (int)flatIndCurrents.size()) ? flatIndCurrents[fc.stateIdx] : 0.0;
                double vPrev = (fc.stateIdx >= 0 && fc.stateIdx < (int)flatIndVoltages.size()) ? flatIndVoltages[fc.stateIdx] : 0.0;

                stampBranchDiagonal(lIdx, rEq);
                B[lIdx] += (2.0 * L / dt) * iPrev + vPrev;
            } else {
                double rEq = (L / dt) + fc.esr;
                double iPrev = (fc.stateIdx >= 0 && fc.stateIdx < (int)flatIndCurrents.size()) ? flatIndCurrents[fc.stateIdx] : 0.0;

                stampBranchDiagonal(lIdx, rEq);
                B[lIdx] += (L / dt) * iPrev;
            }
        }
        else if (fc.type == ComponentType::VoltageSource) {
            int vIdx = fc.vIdx;
            B[vIdx] = fc.val;
        }
        else if (fc.type == ComponentType::ACVoltageSource) {
            double phaseRad = fc.delay * 3.141592653589793 / 180.0;
            double val = fc.val * std::sin(2.0 * 3.141592653589793 * fc.freq * currentTime + phaseRad);
            int vIdx = fc.vIdx;
            B[vIdx] = val;
        }
        else if (fc.type == ComponentType::ControlledVoltageSource) {
            double ctrlVal = (fc.ctrlSigPtr && *fc.ctrlSigPtr != 0.0) ? *fc.ctrlSigPtr : (fc.in0Ptr ? *fc.in0Ptr : (fc.ctrlSigPtr ? *fc.ctrlSigPtr : 0.0));
            int vIdx = fc.vIdx;
            B[vIdx] = fc.gain * ctrlVal;
        }
        else if (fc.type == ComponentType::CurrentSource) {
            if (n1 >= 0) B[n1] -= fc.val;
            if (n2 >= 0) B[n2] += fc.val;
        }
        else if (fc.type == ComponentType::ACCurrentSource) {
            double phaseRad = fc.delay * 3.141592653589793 / 180.0;
            double iVal = fc.val * std::sin(2.0 * 3.141592653589793 * fc.freq * currentTime + phaseRad);
            if (n1 >= 0) B[n1] -= iVal;
            if (n2 >= 0) B[n2] += iVal;
        }
        else if (fc.type == ComponentType::ControlledCurrentSource) {
            double ctrlVal = (fc.ctrlSigPtr && *fc.ctrlSigPtr != 0.0) ? *fc.ctrlSigPtr : (fc.in0Ptr ? *fc.in0Ptr : (fc.ctrlSigPtr ? *fc.ctrlSigPtr : 0.0));
            double iVal = fc.gain * ctrlVal;
            if (n1 >= 0) B[n1] -= iVal;
            if (n2 >= 0) B[n2] += iVal;
        }
        else if (fc.type == ComponentType::Ammeter) {
            int vIdx = fc.vIdx;
            B[vIdx] = 0.0;
        }
        else if (fc.type == ComponentType::Diode || fc.type == ComponentType::Thyristor) {
            double state = (fc.stateIdx >= 0 && fc.stateIdx < (int)flatDiodeStates.size()) ? flatDiodeStates[fc.stateIdx] : 0.0;
            double R = (state > 0.5) ? fc.Ron : fc.Roff;
            if (R < 1e-6) R = 1e-6;
            double g = 1.0 / R;

            stampConductance(n1, n2, g);

            if (state > 0.5) {
                double iEq = g * fc.Vvd;
                if (n1 >= 0) B[n1] += iEq;
                if (n2 >= 0) B[n2] -= iEq;
            }
        }
        else if (fc.type == ComponentType::MOSFET) {
            double state = (fc.stateIdx >= 0 && fc.stateIdx < (int)flatDiodeStates.size()) ? flatDiodeStates[fc.stateIdx] : 0.0;
            double vGate = fc.ctrlSigPtr ? *fc.ctrlSigPtr : 0.0;
            bool isGateOn = (vGate > 0.5);

            double R = (state > 0.5) ? fc.Ron : fc.Roff;
            if (R < 1e-6) R = 1e-6;
            double g = 1.0 / R;

            stampConductance(n1, n2, g);

            if (!isGateOn && state > 0.5) {
                // Body diode: conducts from source (n2) to drain (n1), so the branch is
                // i(n1->n2) = g*(v1 - v2 + Vd). The offset used to carry the forward
                // diode's sign, which put +Vd across the device instead of -Vd: the
                // solved V_DS came out positive, updateDeviceStates() switched the
                // diode off again, and the body diode never conducted.
                double iEq = g * fc.Vvd;
                if (n1 >= 0) B[n1] -= iEq;
                if (n2 >= 0) B[n2] += iEq;
            }
        }
        else if (fc.type == ComponentType::IGBT || fc.type == ComponentType::GTO ||
                 fc.type == ComponentType::IGCT || fc.type == ComponentType::BJT ||
                 fc.type == ComponentType::IGBTDiode) {
            // Conducting state is resolved by updateDeviceStates(); stamp Ron/Roff plus
            // the on-state forward offset (Vce(sat) / Vt / Vf) as a Norton equivalent.
            double state = (fc.stateIdx >= 0 && fc.stateIdx < (int)flatDiodeStates.size()) ? flatDiodeStates[fc.stateIdx] : 0.0;
            double R = (state > 0.5) ? fc.Ron : fc.Roff;
            if (R < 1e-6) R = 1e-6;
            double g = 1.0 / R;

            stampConductance(n1, n2, g);

            if (state > 0.5) {
                double v1 = (n1 >= 0 && n1 < totalDim) ? X[n1] : 0.0;
                double v2 = (n2 >= 0 && n2 < totalDim) ? X[n2] : 0.0;
                // Offset opposes the direction of conduction.
                double sign = ((v2 - v1) > (v1 - v2)) ? -1.0 : 1.0;
                double iEq = g * fc.Vvd * sign;
                if (n1 >= 0) B[n1] += iEq;
                if (n2 >= 0) B[n2] -= iEq;
            }
        }
        else if (fc.type == ComponentType::JFET) {
            // Bidirectional gate-controlled channel, no forward voltage offset.
            double state = (fc.stateIdx >= 0 && fc.stateIdx < (int)flatDiodeStates.size()) ? flatDiodeStates[fc.stateIdx] : 0.0;
            double R = (state > 0.5) ? fc.Ron : fc.Roff;
            if (R < 1e-6) R = 1e-6;
            double g = 1.0 / R;

            stampConductance(n1, n2, g);
        }
        else if (fc.type == ComponentType::Switch) {
            double ctrlVal = fc.ctrlSigPtr ? *fc.ctrlSigPtr : 0.0;
            double R = (ctrlVal > 0.5) ? fc.Ron : fc.Roff;
            if (R < 1e-6) R = 1e-6;
            double g = 1.0 / R;

            stampConductance(n1, n2, g);
        }
    }
}

void CircuitSimulator::saveSolverState(SolverSnapshot& s) const {
    // Assigning into existing vectors reuses their capacity, so this does not
    // allocate after the first step.
    s.X = X;
    s.capV = flatCapVoltages;
    s.capI = flatCapCurrents;
    s.indI = flatIndCurrents;
    s.indV = flatIndVoltages;
    s.diodeStates = flatDiodeStates;
    s.switchStates = flatSwitchStates;

    // The per-element stamped-conductance memo is state too: if it were not
    // restored, the "has this element's contribution changed" test would compare
    // against a value from the abandoned attempt and could wrongly conclude the
    // factorization is still current.
    s.gStamped.resize(fastPhysComps.size());
    for (size_t i = 0; i < fastPhysComps.size(); ++i) s.gStamped[i] = fastPhysComps[i].gStamped;

    s.trapModeStamped = trapModeStamped;
    s.trapModeStampValid = trapModeStampValid;
    s.forceBackwardEulerSteps = forceBackwardEulerSteps;
    s.valid = true;
}

void CircuitSimulator::restoreSolverState(const SolverSnapshot& s) {
    if (!s.valid) return;
    X = s.X;
    flatCapVoltages = s.capV;
    flatCapCurrents = s.capI;
    flatIndCurrents = s.indI;
    flatIndVoltages = s.indV;
    flatDiodeStates = s.diodeStates;
    flatSwitchStates = s.switchStates;
    for (size_t i = 0; i < fastPhysComps.size() && i < s.gStamped.size(); ++i) {
        fastPhysComps[i].gStamped = s.gStamped[i];
    }
    trapModeStamped = s.trapModeStamped;
    trapModeStampValid = s.trapModeStampValid;
    forceBackwardEulerSteps = s.forceBackwardEulerSteps;

    // K still holds the abandoned attempt's stamps. The next assembly restores
    // every dynamic position from K_static and re-stamps, so K itself does not need
    // saving - but force a factorization decision rather than relying on the
    // restored memo to agree with whatever is currently in K. Retries are rare, and
    // a cache hit makes this cheap.
    matrixKChanged = true;
}

bool CircuitSimulator::solveNetworkStep(double t, double hStep) {
    bool statesChanged = true;
    int pwlIter = 0;
    while (statesChanged && pwlIter < 10) {
        pwlIter++;
        assembleMNA(t, hStep);

        if (totalDim > 0) {
            // assembleMNA() flags matrixKChanged whenever it stamps a different
            // dynamic value, so the old O(n^2) `K != K_prev` comparison (plus the
            // O(n^2) K_prev copy) is no longer needed.
            if (matrixKChanged) {
                prepareFactorization(totalDim);
                matrixKChanged = false;
            }
            solveLUSubstitution(totalDim);
        }

        statesChanged = updateDeviceStates();
        if (statesChanged) matrixKChanged = true;
    }

    // Final consistency solve if the last update changed device states.
    if (matrixKChanged) {
        assembleMNA(t, hStep);
        if (totalDim > 0) {
            prepareFactorization(totalDim);
            matrixKChanged = false;
            solveLUSubstitution(totalDim);
        }
    }

    return statesChanged;
}

SimulationOutput CircuitSimulator::runTransient() {
    double nextOutputTime = 0.0;
    cancelRequested_.store(false, std::memory_order_relaxed);
    cancelled_ = false;
    rngRandomNumbers_.seed(12345);
    rngWhiteNoise_.seed(54321);
    auto simClockStart = std::chrono::high_resolution_clock::now();
    setComputeTimeSeconds(0.0);
    SimulationOutput out;
    
    double tStop = config.stopTime > 0 ? config.stopTime : 0.01;
    double dtBase = config.stepSize > 0 ? config.stepSize : 1e-6;

    // Exact step count, computed in 64-bit: tStop/dtBase overflows int for very
    // small step sizes, and the result drives both the iteration budget and the
    // reserve hints below.
    const double estStepsExact = std::ceil(tStop / dtBase);
    const long long estStepsLL =
        (estStepsExact > 0.0 && estStepsExact < 9.0e15) ? (long long)estStepsExact : 0LL;

    // Reserve hint only - clamped so an extreme step size cannot ask the allocator
    // for an absurd block up front. The vectors still grow on demand if needed.
    constexpr long long kMaxReserve = 40000000LL;   // 40 M samples (~320 MB per signal)
    const int estSteps = (int)((estStepsLL > kMaxReserve) ? kMaxReserve : estStepsLL);

    out.time.reserve(estSteps + 1);

    nodeOutputBindings.clear();
    for (const auto& pair : nodeToIdx) {
        auto& vec = out.voltages[pair.first];
        vec.reserve(estSteps + 1);
        nodeOutputBindings.push_back({ pair.second - 1, &vec });
    }

    for (auto& fc : fastPhysComps) {
        auto& vVec = out.custom_plots[fc.vPlotKey];
        auto& iVec = out.custom_plots[fc.iPlotKey];
        vVec.reserve(estSteps + 1);
        iVec.reserve(estSteps + 1);
        fc.vPlotVecPtr = &vVec;
        fc.iPlotVecPtr = &iVec;

        if (fc.type == ComponentType::Voltmeter || fc.type == ComponentType::Ammeter) {
            auto& vmVec = out.voltmeters[fc.id];
            auto& sigVec = out.signals[fc.id];
            auto& sigOutVec = out.signals[fc.id + ".Out"];
            vmVec.reserve(estSteps + 1);
            sigVec.reserve(estSteps + 1);
            sigOutVec.reserve(estSteps + 1);
            fc.vmVecPtr = &vmVec;
            fc.sigVecPtr = &sigVec;
            fc.sigOutVecPtr = &sigOutVec;
        }
    }

    for (auto& fc : fastCtrlComps) {
        fc.customScriptOutputVecPtrs.clear();
        fc.customScriptPlotVecPtrs.clear();

        if (hasPerPortOutputs(fc)) {
            for (size_t i = 0; i < fc.outputSigKeys.size(); ++i) {
                const auto& outK = fc.outputSigKeys[i];
                auto& sigVec = out.signals[outK];
                auto& cpVec = out.custom_plots[outK];
                sigVec.reserve(estSteps + 1);
                cpVec.reserve(estSteps + 1);
                fc.customScriptOutputVecPtrs.push_back(&sigVec);
                fc.customScriptPlotVecPtrs.push_back(&cpVec);
            }
        }
        
        auto& sigVec = out.signals[fc.id];
        auto& sigOutVec = out.signals[fc.outKey];
        auto& cpVec = out.custom_plots[fc.id];
        auto& cpOutVec = out.custom_plots[fc.outKey];
        sigVec.reserve(estSteps + 1);
        sigOutVec.reserve(estSteps + 1);
        cpVec.reserve(estSteps + 1);
        cpOutVec.reserve(estSteps + 1);
        fc.sigVecPtr = &sigVec;
        fc.sigOutVecPtr = &sigOutVec;
        fc.vPlotVecPtr = &cpVec;
        fc.iPlotVecPtr = &cpOutVec;
    }

    double currentTime = 0.0;

    // Adaptive stepping is opt-in and must be requested explicitly.
    //
    // The previous predicate was
    //     isFixed = (step_type == "fixed") || (solver == "euler" && step_type != "variable")
    // which made a netlist with an empty or unrecognised step_type take the variable
    // path whenever the solver was not "euler". That path was not merely unused, it was
    // inconsistent: it advanced time by up to 5x the nominal step while assembleMNA and
    // every control block still used config.stepSize, so L and C behaved as though the
    // step had never changed. Anything not explicitly asking for adaptive stepping now
    // gets fixed stepping.
    const bool wantAdaptive = (config.step_type == "variable" || config.step_type == "adaptive");
    const bool isFixed = !wantAdaptive;
    double h = dtBase;

    // â”€â”€ Adaptive stepping setup â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
    // STATUS: working and opt-in. Fixed stepping remains the default.
    //
    // Measured on a 100 kHz buck over 2 ms, max |dV_C1| against a 10 ns fixed
    // reference: fixed 10 ns = 200000 samples / 107 ms; fixed 100 ns = 20000 samples /
    // 0.0712 V / 12.8 ms; adaptive relTol 1e-5 = 11109 samples / 0.0649 V / 12.5 ms.
    // So adaptive matches the 100 ns run's accuracy for fewer steps and slightly less
    // time, and is ~8.6x faster than the 10 ns run it replaces.
    //
    // Three bugs were found here by measurement rather than by reading, and the
    // comments below record them so they are not reintroduced:
    //   - the local-error estimate must use the states IMPLIED by the new solution,
    //     not the flat state vectors, which the recording block only advances after a
    //     step is accepted;
    //   - a step containing a switching discontinuity must be excluded from the error
    //     test, and the error history restarted afterwards;
    //   - a gate edge cannot be detected by probing the step endpoint alone, and the
    //     largest step must be held below a quarter of the shortest carrier period.
    //
    // Still open: diode commutation is bracketed by halving down to eventFloor, which
    // costs ~8 extra solves per commutation (evtRetry ~2200 of ~14000 total solves).
    // Interpolating the current zero-crossing instead would remove most of that.
    // Admissible steps form a ladder hCeil / 2^k. Restricting h to a small discrete
    // set is what keeps the factorization cache useful: the companion conductances
    // contain h, so an unrestricted h would make every single step a brand-new
    // matrix and put the O(n^3) factorization back on the critical path. Measured
    // on a dim-98 circuit, a continuously changing matrix costs 1933 ms against
    // 97 ms for one the cache can reuse.
    //
    // Halving is also how events are localised below, so the bisection stays on the
    // ladder for free.
    // Shortest carrier period in the circuit. The netlist states these explicitly, so
    // there is no need to guess: capping the largest step at a quarter period keeps at
    // most one gate edge inside any step for realistic duty cycles, which stops the
    // step from skipping an even number of edges and landing back on the same gate
    // pattern.
    double minCarrierPeriod = 0.0;
    auto noteCarrier = [&](double periodSec) {
        if (periodSec > 0.0 && (minCarrierPeriod == 0.0 || periodSec < minCarrierPeriod)) {
            minCarrierPeriod = periodSec;
        }
    };
    for (const auto& fc : fastCtrlComps) {
        switch (fc.type) {
            case ComponentType::PulseGenerator:
                noteCarrier(fc.period);
                break;
            case ComponentType::Triangle_Carrier:
            case ComponentType::PWM_3PH:
            case ComponentType::SVPWM:
                if (fc.freq > 0.0) noteCarrier(1.0 / fc.freq);
                break;
            default:
                break;
        }
    }

    double hCeil = config.hMax;
    if (hCeil <= 0.0) {
        // `stepSize` is the reference step, not a ceiling: capping hMax at stepSize
        // would leave the controller able only to shrink, which is the opposite of
        // what adaptive stepping is for. Allow a generous growth range but keep at
        // least ~100 steps over the run.
        hCeil = std::min(tStop / 100.0, dtBase * 1024.0);
    }
    if (minCarrierPeriod > 0.0 && config.hMax <= 0.0) {
        const double carrierCap = minCarrierPeriod * 0.25;
        if (carrierCap < hCeil) hCeil = carrierCap;
    }
    if (hCeil < dtBase) hCeil = dtBase;
    double hFloor = config.hMin;
    if (hFloor <= 0.0) hFloor = dtBase / 1024.0;
    if (hFloor > hCeil) hFloor = hCeil;

    int ladderDepth = 0;
    while (ladderDepth < 40 && hCeil / std::pow(2.0, (double)ladderDepth) > hFloor) ++ladderDepth;

    // How finely a switching instant is bracketed. Bisecting all the way to hFloor
    // would cost ~20 extra solves per event for no useful accuracy: what matters is
    // the timing error as a fraction of the switching period, and 1/256 of the
    // largest step is well under a percent of a carrier cycle for any realistic
    // converter. Local-error rejections may still go below this, down to hFloor.
    double eventFloor = hCeil / 256.0;
    if (eventFloor < hFloor) eventFloor = hFloor;

    // Snaps a requested step down onto the ladder, so the result is never larger
    // than asked for.
    auto quantiseStep = [&](double hWant) -> double {
        if (!(hWant > 0.0)) return hCeil / std::pow(2.0, (double)ladderDepth);
        if (hWant >= hCeil) return hCeil;
        double k = std::ceil(std::log2(hCeil / hWant));
        if (k < 0.0) k = 0.0;
        if (k > (double)ladderDepth) k = (double)ladderDepth;
        return hCeil / std::pow(2.0, k);
    };

    // Error-control state: the reactive states at the last two accepted steps, used
    // to estimate the second derivative by divided differences. Backward Euler is
    // first order, so its local error is (h^2/2)*y'' and no extra solve is needed to
    // estimate it.
    const size_t nCapStates = flatCapVoltages.size();
    std::vector<double> stNow, stPrev1, stPrev2;
    double tPrev1 = 0.0, tPrev2 = 0.0;
    int lteHistory = 0;

    auto packReactiveStates = [&](std::vector<double>& dst) {
        dst.clear();
        dst.insert(dst.end(), flatCapVoltages.begin(), flatCapVoltages.end());
        dst.insert(dst.end(), flatIndCurrents.begin(), flatIndCurrents.end());
    };

    // Reactive states implied by the solution X that solveNetworkStep() just produced.
    //
    // This is needed because flatCapVoltages / flatIndCurrents are not advanced by the
    // solve - the recording block does that, and it only runs once a step has been
    // ACCEPTED. Reading the flat vectors during an attempt therefore yields the state
    // from the PREVIOUS step, which made the divided difference collapse: with
    // stNow == stPrev1, d1 came out ~0 and (d1 - d0) degenerated into minus the first
    // derivative instead of a second difference. That inflated the error estimate by
    // roughly (slope * span) / (h * y''), predicting err = 1 at h ~ 1.5e-8 on a 100 kHz
    // buck against a true limit near 4 us - which is exactly the ~200x over-refinement
    // that was measured.
    //
    // The formulas below mirror the recording block's state advance exactly.
    auto packCandidateStates = [&](std::vector<double>& dst) {
        const size_t nCap = flatCapVoltages.size();
        dst.resize(nCap + flatIndCurrents.size());
        for (size_t i = 0; i < nCap; ++i) dst[i] = flatCapVoltages[i];
        for (size_t i = 0; i < flatIndCurrents.size(); ++i) dst[nCap + i] = flatIndCurrents[i];

        for (const auto& fc : fastPhysComps) {
            if (fc.type == ComponentType::Capacitor) {
                if (fc.stateIdx < 0 || fc.stateIdx >= (int)nCap) continue;
                const double v1 = (fc.n1 >= 0 && fc.n1 < totalDim) ? X[fc.n1] : 0.0;
                const double v2 = (fc.n2 >= 0 && fc.n2 < totalDim) ? X[fc.n2] : 0.0;
                const double vDiff = v1 - v2;
                double C = liveElementValue(fc, 1e-15);
                if (C < 1e-15) C = 1e-15;
                const bool trapC = trapModeStampValid && trapModeStamped && fc.stateIdx < (int)flatCapCurrents.size();
                const double hc = trapC ? h / (2.0 * C) : h / C;
                const double gEq = 1.0 / (hc + fc.esr);
                const double hist = trapC ? hc * flatCapCurrents[fc.stateIdx] : 0.0;
                const double iC = gEq * (vDiff - flatCapVoltages[fc.stateIdx] - hist);
                dst[fc.stateIdx] = vDiff - fc.esr * iC;
            } else if (fc.type == ComponentType::Inductor) {
                if (fc.stateIdx < 0 || fc.stateIdx >= (int)flatIndCurrents.size()) continue;
                if (fc.lIdx >= 0 && fc.lIdx < totalDim) {
                    dst[nCap + (size_t)fc.stateIdx] = X[fc.lIdx];
                }
            }
        }
    };

    // Scaled infinity norm of the estimated local error. <= 1 means acceptable.
    auto lteErrorNorm = [&](const std::vector<double>& yNow, double tNow, double hUsed) -> double {
        if (lteHistory < 2) return 0.0;             // not enough history yet
        const double dt1 = tNow - tPrev1;
        const double dt0 = tPrev1 - tPrev2;
        const double span = tNow - tPrev2;
        if (dt1 <= 0.0 || dt0 <= 0.0 || span <= 0.0) return 0.0;
        if (yNow.size() != stPrev1.size() || yNow.size() != stPrev2.size()) return 0.0;

        double worst = 0.0;
        for (size_t i = 0; i < yNow.size(); ++i) {
            const double d1 = (yNow[i] - stPrev1[i]) / dt1;
            const double d0 = (stPrev1[i] - stPrev2[i]) / dt0;
            // (h^2/2) * y'', with y'' approximated as 2*(d1-d0)/span.
            const double lte = hUsed * hUsed * (d1 - d0) / span;
            const double floorTol = (i < nCapStates) ? config.absTolV : config.absTolI;
            const double scale = config.relTol * std::fabs(yNow[i]) + floorTol;
            const double e = std::fabs(lte) / ((scale > 0.0) ? scale : 1e-300);
            if (e > worst) worst = e;
        }
        return worst;
    };

    // True when the converged device states differ from the ones the step started
    // from, i.e. a diode or switch changed topology somewhere inside the step.
    //
    // Locating that instant by HALVING, rather than by interpolating the switching
    // margin, is a deliberate choice and was verified by measurement.
    //
    // Interpolation looks obviously better on paper - one secant estimate instead of
    // about eight halvings - but a halved step stays on the power-of-two ladder, so its
    // matrix is already in the factorization cache and each retry is a cheap solve.
    // An interpolated step lands on an arbitrary h, which is a brand-new matrix and
    // costs a full factorization. Since a factorization costs roughly n/3 times a
    // solve, trading eight cached solves for one factorization only pays below about
    // n = 24, and even at n = 9 it measured worse once the cost of inserting a cache
    // entry is counted:
    //
    //   halving       : 3537 samples, 0.1875 V, 167 factorizations, 13956 cache hits
    //   interpolating : 2443 samples, 0.2178 V, 10103 factorizations, 3221 cache hits
    //
    // So bisection on the ladder is the cache-optimal search here, not a lazy choice.
    // The margin-interpolation code was removed rather than left dormant, because it
    // had to duplicate every switching threshold from updateDeviceStates() and would
    // have drifted out of step with it silently.
    auto deviceStatesChangedVs = [&](const SolverSnapshot& s) -> bool {
        const size_t nd = std::min(flatDiodeStates.size(), s.diodeStates.size());
        for (size_t i = 0; i < nd; ++i) if (flatDiodeStates[i] != s.diodeStates[i]) return true;
        const size_t ns = std::min(flatSwitchStates.size(), s.switchStates.size());
        for (size_t i = 0; i < ns; ++i) if (flatSwitchStates[i] != s.switchStates[i]) return true;
        return false;
    };

    if (wantAdaptive) {
        // Start from the ladder rung nearest the user's nominal step so the opening
        // steps behave like the fixed-step solver, then let the controller grow.
        h = quantiseStep(dtBase);

        // Seed the error-control history with the initial state at t = 0. No output
        // sample is emitted here: the recording block fills every signal vector in
        // lockstep, and emitting a partial sample would leave the vectors at
        // different lengths. The first sample therefore appears at t = h, labelled
        // at the END of its step - unlike the fixed path, which labels each solution
        // with the time at the start of its step and so is shifted by one step.
        packReactiveStates(stPrev1);
        tPrev1 = 0.0;
        lteHistory = 1;

        // Prime the control blocks for t = 0. Their `currentTime == 0` branches
        // initialise rather than integrate, so this sets state up without advancing
        // it, matching what the fixed path does on its first iteration.
        evaluateControls(0.0, h);
    }

    adaptiveAccepted = 0;
    adaptiveRejected = 0;
    adaptiveEventRetries = 0;

    adaptiveGateCuts = 0;
    adaptiveErrChecked = 0;
    adaptiveHMinUsed = 0.0;
    adaptiveHMaxUsed = 0.0;

    // â”€â”€ Gate-transition localisation â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
    // A switch's conducting state is driven by its control signal, which the solver
    // only refreshes at step boundaries. Left alone, that quantises every PWM edge to
    // the step size, so growing h corrupts the duty cycle - and the local-error
    // controller cannot see that at all, because it only measures integration error.
    // Measured on a 100 kHz buck, adaptive stepping without this was strictly worse
    // than fixed stepping at every tolerance.
    //
    // The fix is to find the first gate transition inside the step and cut the step to
    // land on it. The carrier chain is a closed-form function of time given the
    // modulating signal held over the step, so the edge can be bracketed by bisecting
    // on time using non-committing control evaluations - no network solves involved.
    std::vector<const double*> gatePtrs;
    if (wantAdaptive) {
        for (const auto& fc : fastPhysComps) {
            if (!fc.ctrlSigPtr) continue;
            switch (fc.type) {
                case ComponentType::MOSFET:
                case ComponentType::Switch:
                case ComponentType::IGBT:
                case ComponentType::IGBTDiode:
                case ComponentType::GTO:
                case ComponentType::IGCT:
                case ComponentType::BJT:
                case ComponentType::JFET:
                case ComponentType::Thyristor:
                    gatePtrs.push_back(fc.ctrlSigPtr);
                    break;
                default:
                    break;
            }
        }
    }

    std::vector<char> gateAtStart, gateProbe;
    std::vector<double> ctrlSignalsAtStart;

    // Time of the next known gate transition, or negative when unknown. Once an edge
    // has been located there is by construction no earlier one, so intervening steps
    // only need to be clipped to it rather than rescanning. Without this the coarse
    // scan ran on every step and became the dominant cost - ~97k control evaluations
    // for 11k steps, which made the adaptive run slower in wall-clock terms than the
    // fixed one despite using far fewer steps.
    double nextGateEdge = -1.0;

    auto sampleGates = [&](std::vector<char>& dst) {
        dst.resize(gatePtrs.size());
        for (size_t i = 0; i < gatePtrs.size(); ++i) dst[i] = (*gatePtrs[i] > 0.5) ? (char)1 : (char)0;
    };

    // Evaluates the control chain at `tProbe` without advancing any block state, and
    // reports whether any monitored gate has flipped relative to the step start.
    auto gatesDifferAt = [&](double tProbe, double hRef) -> bool {
        evaluateGateControls(tProbe, hRef);
        sampleGates(gateProbe);
        for (size_t i = 0; i < gateProbe.size(); ++i) {
            if (gateProbe[i] != gateAtStart[i]) return true;
        }
        return false;
    };

    // Safety cap on loop iterations. This has to scale with the requested step
    // count: a fixed cap silently truncated the run whenever tStop/stepSize
    // exceeded it. The old value of 300000 stopped a 10 ms run at a 10 ns step
    // after only 3 ms, and because the loop just exits normally there was no
    // error - the waveform simply ended early and looked like a wrong answer.
    // Variable-step runs can need more iterations than tStop/dtBase, so they get
    // generous headroom while still being guaranteed to terminate.
    const long long stepBudget = estStepsLL + 64;
    // For adaptive stepping the bound is set by the smallest admissible step, not by
    // the nominal one: h can legitimately fall below config.stepSize around switching
    // instants, and a budget based on the nominal step would silently truncate the run
    // exactly as the old fixed cap of 300000 did.
    long long adaptiveBudget = stepBudget;
    if (!isFixed && hFloor > 0.0) {
        const double worst = std::ceil(tStop / hFloor) + 64.0;
        adaptiveBudget = (worst < 9.0e15) ? (long long)worst : stepBudget * 1024;
    }
    const long long max_iterations = isFixed ? stepBudget : adaptiveBudget;
    long long iterCount = 0;
    matrixKChanged = true;

    // Wall-clock pacing for the live telemetry publish (see end of loop).
    constexpr double kTelemetryPublishIntervalMs = 33.0;   // ~30 Hz
    auto lastPublishClock = simClockStart;

    while (currentTime < tStop - 1e-12 && iterCount < max_iterations) {
        if (cancelRequested_.load(std::memory_order_relaxed)) { cancelled_ = true; break; }
        iterCount++;

        // Time this sample is labelled with: the end of the step, which is where the
        // solution X of the step lives. The fixed path used to label the start of the
        // step, so every fixed-step waveform was shifted one step early (the first
        // sample, at "t = 0", already held the state after one step).
        double sampleTime = currentTime;
        bool statesChanged = false;
        double lastErrAccepted = 0.0;   // scaled local-error norm of the accepted step

        if (isFixed) {
            if (currentTime + h > tStop) h = tStop - currentTime;

            // Step 1: Evaluate Control Loop blocks
            evaluateControls(currentTime, h);

            // Step 2: Iterative PWL solution loop for diode/switch convergence
            statesChanged = solveNetworkStep(currentTime, h);
            sampleTime = currentTime + h;
        } else {
            // â”€â”€ Error-controlled step with event localisation â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
            // Control signals are already valid for `currentTime` and are held
            // constant across the step, so an abandoned attempt never has to undo
            // any control-block state - only the electrical state, which
            // restoreSolverState() covers.
            double remaining = tStop - currentTime;
            if (h > remaining) h = remaining;
            // Absorb a short tail rather than leaving a degenerate final step: a step
            // of ~1e-19 would stamp gEq = h/C as ~1e15 and produce a garbage sample.
            if (remaining < h * 1.5) h = remaining;

            // Cut the step at the first gate transition inside it, so the step is
            // solved with one constant gate pattern and the edge sits on the boundary
            // where the post-acceptance committed evaluation will pick it up. This is
            // what makes duty-cycle accuracy independent of h.
            if (!gatePtrs.empty()) {
                ctrlSignalsAtStart = flatControlSignals;
                sampleGates(gateAtStart);

                const double edgeTol = std::max(hFloor * 1e-3, h * 1e-6);

                // Clip to an already-located edge first; that costs nothing.
                if (nextGateEdge > currentTime + edgeTol) {
                    const double hToEdge = nextGateEdge - currentTime;
                    if (hToEdge < h) { h = hToEdge; ++adaptiveGateCuts; }
                }

                // One endpoint probe verifies the prediction. If a gate has moved
                // inside the step after all - the modulating signal can shift an edge
                // earlier between evaluations - fall through to a full scan.
                if (gatesDifferAt(currentTime + h, h)) {
                    // Coarse scan, then bisect inside the bracket holding the change.
                    // Probing only the endpoint is not sufficient in general: an even
                    // number of edges inside one step returns the gate to its starting
                    // value, so the step looks clean. That limited gate detection to
                    // 14% of steps once h grew to span a carrier period.
                    constexpr int kGateScan = 8;
                    double lo = currentTime, hi = currentTime + h;
                    for (int s = 1; s <= kGateScan; ++s) {
                        const double tp = currentTime + h * ((double)s / (double)kGateScan);
                        if (gatesDifferAt(tp, h)) { hi = tp; break; }
                        lo = tp;
                    }

                    for (int it = 0; it < 60 && (hi - lo) > edgeTol; ++it) {
                        const double mid = 0.5 * (lo + hi);
                        if (gatesDifferAt(mid, h)) hi = mid; else lo = mid;
                    }
                    // `hi` is the earliest bracketed time at which a gate differs, so
                    // land there: the interval [t, hi) carries the old gate pattern and
                    // the edge lands within edgeTol of the boundary.
                    nextGateEdge = hi;
                    double hCut = hi - currentTime;
                    if (hCut < hFloor) hCut = hFloor;
                    if (hCut < h) { h = hCut; ++adaptiveGateCuts; }
                }

                // Test hook: extra probes must not change anything.
                for (int p = 0; p < config.debugExtraControlProbes; ++p) {
                    const double frac = (double)(p + 1) / (double)(config.debugExtraControlProbes + 1);
                    (void)gatesDifferAt(currentTime + h * frac, h);
                }

                // Undo the probes' writes to the signal outputs, so the solve below
                // sees the control values that belong to the start of the step.
                flatControlSignals = ctrlSignalsAtStart;
            }

            saveSolverState(stepSnapshot);

            int attempt = 0;
            // Counted separately from `attempt` so the debug rejection hook cannot
            // consume the real budget: if it did, a step needing many event halvings
            // would be accepted one halving early and the run would legitimately
            // differ, which would defeat the purpose of the hook.
            int realAttempts = 0;
            constexpr int kMaxAttempts = 16;
            bool eventInside = false;
            while (true) {
                ++attempt;
                statesChanged = solveNetworkStep(currentTime, h);
                packCandidateStates(stNow);

                // A topology change inside the step means it straddles a commutation.
                eventInside = deviceStatesChangedVs(stepSnapshot);

                // The local-error estimate is a divided-difference approximation to
                // y'', which is only meaningful across a smooth interval. Applied to a
                // step containing a switching discontinuity it returns a huge value
                // and drives h to the floor - measured as 200k steps where a few
                // thousand suffice. Event localisation already bounds such a step, so
                // the error test is skipped for it.
                const double err = eventInside ? 0.0
                                               : lteErrorNorm(stNow, currentTime + h, h);

                // Test hook: exercise the rollback path. Output must be unchanged.
                bool forcedReject = false;
                if (config.debugRejectEveryNthStep > 0 && attempt == 1 &&
                    (iterCount % config.debugRejectEveryNthStep) == 0) {
                    forcedReject = true;
                }

                if (!forcedReject) ++realAttempts;
                const bool atFloor = (h <= hFloor * 1.0000001);
                const bool atEventFloor = (h <= eventFloor * 1.0000001);
                const bool lastAttempt = (realAttempts >= kMaxAttempts);

                bool reject = false;
                if (forcedReject) {
                    reject = true;
                } else if (eventInside && !atEventFloor && !lastAttempt) {
                    // Halving walks the step boundary down towards the switching
                    // instant, and because halving stays on the ladder the retries
                    // reuse cached factorizations.
                    reject = true;
                    ++adaptiveEventRetries;
                } else if (err > 1.0 && !atFloor && !lastAttempt) {
                    reject = true;
                    ++adaptiveRejected;
                }

                if (!reject) { lastErrAccepted = err; break; }

                restoreSolverState(stepSnapshot);

                double hRetry;
                if (forcedReject) {
                    // The test hook must retry at the SAME step, otherwise the run
                    // legitimately differs and the comparison proves nothing.
                    hRetry = h;
                } else if (err > 1.0) {
                    // Order-1 method: local error ~ h^2, so h scales as err^(-1/2).
                    const double factor = 0.9 / std::sqrt(err);
                    hRetry = h * ((factor < 0.5) ? 0.5 : ((factor > 0.9) ? 0.9 : factor));
                } else {
                    // Commutation: aim straight at the interpolated crossing rather
                    // than halving towards it. The clamp guarantees the step actually
                    // shrinks, so the loop cannot stall on a near-1 estimate.
                    // Halve. This keeps the retry ON the step ladder, so its matrix is
                    // already in the factorization cache and the retry costs only a
                    // solve. See the note on deviceStatesChangedVs for why this beats
                    // aiming straight at the interpolated crossing.
                    hRetry = h * 0.5;
                }
                if (hRetry < hFloor) hRetry = hFloor;
                h = forcedReject ? hRetry : quantiseStep(hRetry);
                if (h < hFloor) h = hFloor;
            }

            sampleTime = currentTime + h;
            ++adaptiveAccepted;

            // Once the step has reached the located edge the gate pattern changes, so
            // the cached prediction no longer applies and the next step must rescan.
            if (nextGateEdge > 0.0 && sampleTime >= nextGateEdge - std::max(hFloor * 1e-3, h * 1e-6)) {
                nextGateEdge = -1.0;
            }
            if (!eventInside && lteHistory >= 2) ++adaptiveErrChecked;
            if (adaptiveHMaxUsed == 0.0 || h > adaptiveHMaxUsed) adaptiveHMaxUsed = h;
            if (adaptiveHMinUsed == 0.0 || h < adaptiveHMinUsed) adaptiveHMinUsed = h;

            // Advance the error-control history only for an accepted step. A step that
            // changed topology invalidates the history: the divided differences would
            // then straddle the discontinuity and misreport the error for the next two
            // steps, so start the history over from the post-event state instead.
            if (eventInside) {
                stPrev1 = stNow;
                tPrev1 = sampleTime;
                lteHistory = 1;
            } else {
                stPrev2.swap(stPrev1);
                tPrev2 = tPrev1;
                stPrev1 = stNow;
                tPrev1 = sampleTime;
                if (lteHistory < 2) ++lteHistory;
            }
        }

        if (forceBackwardEulerSteps > 0) forceBackwardEulerSteps--;

        // Store time step
        if (config.outputDecimation <= 0.0 || sampleTime >= nextOutputTime - 1e-12) {
        out.time.push_back(sampleTime);

        // Store node voltages (Zero map lookups)
        for (const auto& binding : nodeOutputBindings) {
            double v = (binding.nodeIdx >= 0 && binding.nodeIdx < totalDim) ? X[binding.nodeIdx] : 0.0;
            binding.vecPtr->push_back(v);
        }

        // Update & store component values, custom_plots (V_<comp>, I_<comp>)
        for (auto& fc : fastPhysComps) {
            int n1 = fc.n1;
            int n2 = fc.n2;

            double v1 = (n1 >= 0 && n1 < totalDim) ? X[n1] : 0.0;
            double v2 = (n2 >= 0 && n2 < totalDim) ? X[n2] : 0.0;
            double vDiff = v1 - v2;

            double iComp = 0.0;

            if (fc.type == ComponentType::Resistor) {
                double Rtotal = liveElementValue(fc, 1e-6) + fc.esr;
                if (Rtotal < 1e-6) Rtotal = 1e-6;
                iComp = vDiff / Rtotal;
            }
            else if (fc.type == ComponentType::Capacitor) {
                double C = liveElementValue(fc, 1e-15);
                if (C < 1e-15) C = 1e-15;
                // Same integration rule as the stamp this step was solved with.
                const bool trapC = trapModeStampValid && trapModeStamped &&
                                   fc.stateIdx >= 0 && fc.stateIdx < (int)flatCapCurrents.size();
                const double hc = trapC ? h / (2.0 * C) : h / C;
                double rEq = hc + fc.esr;
                double gEq = 1.0 / rEq;
                double vCapPrev = (fc.stateIdx >= 0 && fc.stateIdx < (int)flatCapVoltages.size()) ? flatCapVoltages[fc.stateIdx] : 0.0;
                const double hist = trapC ? hc * flatCapCurrents[fc.stateIdx] : 0.0;
                iComp = gEq * (vDiff - vCapPrev - hist);
                if (fc.stateIdx >= 0 && fc.stateIdx < (int)flatCapCurrents.size()) flatCapCurrents[fc.stateIdx] = iComp;
                if (fc.stateIdx >= 0 && fc.stateIdx < (int)flatCapVoltages.size()) {
                    // Store the voltage across the IDEAL capacitor, not across the
                    // whole branch. The companion model in assembleMNA() is
                    //     i = (v_branch - vC_prev) / (dt/C + esr)
                    // which is only correct if vC_prev excludes the ESR drop.
                    // Storing v_branch here instead made the branch behave as
                    //     C_eff = C / (1 + esr*C/dt)
                    // so the capacitance silently shrank as the step size was
                    // reduced (100 uF with 10 mOhm ESR became ~1 uF at dt = 10 ns),
                    // which showed up as huge non-physical output ripple and as
                    // results that diverged instead of converging under step
                    // refinement. v_branch = vC + esr*i, hence vC = v_branch - esr*i.
                    // This is an exact structural relation, so it stays valid
                    // regardless of the step size used to obtain iComp.
                    flatCapVoltages[fc.stateIdx] = vDiff - fc.esr * iComp;
                }
            }
            else if (fc.type == ComponentType::Inductor) {
                int lIdx = fc.lIdx;
                if (lIdx >= 0 && lIdx < totalDim) {
                    iComp = X[lIdx];
                    if (fc.stateIdx >= 0 && fc.stateIdx < (int)flatIndCurrents.size()) {
                        flatIndCurrents[fc.stateIdx] = iComp;
                        if (fc.stateIdx < (int)flatIndVoltages.size()) {
                            // Same reasoning as the capacitor above: the trapezoidal
                            // companion source uses this as the voltage across the
                            // IDEAL inductor, so the series ESR drop must come out.
                            flatIndVoltages[fc.stateIdx] = vDiff - fc.esr * iComp;
                        }
                    }
                }
                out.inductors[fc.id].push_back(iComp);
            }
            else if (fc.type == ComponentType::VoltageSource || fc.type == ComponentType::ACVoltageSource || fc.type == ComponentType::Ammeter) {
                int vIdx = fc.vIdx;
                if (vIdx >= 0 && vIdx < totalDim) {
                    iComp = X[vIdx];
                }
            }
            else if (fc.type == ComponentType::Diode) {
                double state = (fc.stateIdx >= 0 && fc.stateIdx < (int)flatDiodeStates.size()) ? flatDiodeStates[fc.stateIdx] : 0.0;
                double R = (state > 0.5) ? fc.Ron : fc.Roff;
                iComp = (state > 0.5) ? ((vDiff - fc.Vvd) / R) : (vDiff / R);
            }
            else if (fc.type == ComponentType::Winding) {
                if (fc.wIdx0 >= 0 && fc.wIdx0 < totalDim) iComp = X[fc.wIdx0];
            }
            else if (fc.type == ComponentType::IGBT || fc.type == ComponentType::GTO ||
                     fc.type == ComponentType::IGCT || fc.type == ComponentType::BJT ||
                     fc.type == ComponentType::IGBTDiode) {
                double state = (fc.stateIdx >= 0 && fc.stateIdx < (int)flatDiodeStates.size()) ? flatDiodeStates[fc.stateIdx] : 0.0;
                double R = (state > 0.5) ? fc.Ron : fc.Roff;
                if (R < 1e-6) R = 1e-6;
                if (state > 0.5) {
                    double offset = (vDiff >= 0.0) ? fc.Vvd : -fc.Vvd;
                    iComp = (vDiff - offset) / R;
                } else {
                    iComp = vDiff / R;
                }
            }
            else if (fc.type == ComponentType::JFET) {
                double state = (fc.stateIdx >= 0 && fc.stateIdx < (int)flatDiodeStates.size()) ? flatDiodeStates[fc.stateIdx] : 0.0;
                double R = (state > 0.5) ? fc.Ron : fc.Roff;
                if (R < 1e-6) R = 1e-6;
                iComp = vDiff / R;
            }
            else if (fc.type == ComponentType::MOSFET) {
                double state = (fc.stateIdx >= 0 && fc.stateIdx < (int)flatDiodeStates.size()) ? flatDiodeStates[fc.stateIdx] : 0.0;
                double R = (state > 0.5) ? fc.Ron : fc.Roff;
                if (R < 1e-6) R = 1e-6;
                double ctrlVal = fc.ctrlSigPtr ? *fc.ctrlSigPtr : 0.0;
                bool isGateOn = (ctrlVal > 0.5);
                
                if (!isGateOn && state > 0.5 && vDiff < 0) {
                    // Body diode conduction
                    iComp = (vDiff + fc.Vvd) / R;
                } else {
                    iComp = vDiff / R;
                }
            }
            // Current sources had no branch here, so I_<id> was recorded as 0 for
            // I, AC_I and CTRL_I. Same values as the MNA stamp at this time.
            else if (fc.type == ComponentType::CurrentSource) {
                iComp = fc.val;
            }
            else if (fc.type == ComponentType::ACCurrentSource) {
                double phaseRad = fc.delay * 3.141592653589793 / 180.0;
                iComp = fc.val * std::sin(2.0 * 3.141592653589793 * fc.freq * currentTime + phaseRad);
            }
            else if (fc.type == ComponentType::ControlledCurrentSource) {
                double ctrlVal = (fc.ctrlSigPtr && *fc.ctrlSigPtr != 0.0) ? *fc.ctrlSigPtr : (fc.in0Ptr ? *fc.in0Ptr : (fc.ctrlSigPtr ? *fc.ctrlSigPtr : 0.0));
                iComp = fc.gain * ctrlVal;
            }
            else if (fc.type == ComponentType::Switch) {
                double ctrlVal = fc.ctrlSigPtr ? *fc.ctrlSigPtr : 0.0;
                iComp = vDiff / ((ctrlVal > 0.5) ? fc.Ron : fc.Roff);
            }
            else if (isTransformerType(fc.type)) {
                int w0 = fc.wIdx0;
                if (w0 >= 0 && w0 < totalDim) {
                    iComp = X[w0];
                }
            }

            if (fc.vPlotVecPtr) fc.vPlotVecPtr->push_back(vDiff);
            if (fc.iPlotVecPtr) fc.iPlotVecPtr->push_back(iComp);
            if (fc.vPlotSignalIdx >= 0 && fc.vPlotSignalIdx < (int)flatControlSignals.size()) flatControlSignals[fc.vPlotSignalIdx] = vDiff;
            if (fc.iPlotSignalIdx >= 0 && fc.iPlotSignalIdx < (int)flatControlSignals.size()) flatControlSignals[fc.iPlotSignalIdx] = iComp;

            if (fc.type == ComponentType::Voltmeter) {
                if (fc.vmVecPtr) fc.vmVecPtr->push_back(vDiff);
                if (fc.sigVecPtr) fc.sigVecPtr->push_back(vDiff);
                if (fc.sigOutVecPtr) fc.sigOutVecPtr->push_back(vDiff);
                if (fc.outSignalIdx >= 0 && fc.outSignalIdx < (int)flatControlSignals.size()) flatControlSignals[fc.outSignalIdx] = vDiff;
                if (fc.compSelfSignalIdx >= 0 && fc.compSelfSignalIdx < (int)flatControlSignals.size()) flatControlSignals[fc.compSelfSignalIdx] = vDiff;
            } else if (fc.type == ComponentType::Ammeter) {
                if (fc.vmVecPtr) fc.vmVecPtr->push_back(iComp);
                if (fc.sigVecPtr) fc.sigVecPtr->push_back(iComp);
                if (fc.sigOutVecPtr) fc.sigOutVecPtr->push_back(iComp);
                if (fc.outSignalIdx >= 0 && fc.outSignalIdx < (int)flatControlSignals.size()) flatControlSignals[fc.outSignalIdx] = iComp;
                if (fc.compSelfSignalIdx >= 0 && fc.compSelfSignalIdx < (int)flatControlSignals.size()) flatControlSignals[fc.compSelfSignalIdx] = iComp;
            }
        }

        // Store control loop signals into output (Zero map lookups)
        for (auto& fc : fastCtrlComps) {
            if (hasPerPortOutputs(fc)) {
                double firstVal = 0.0;
                for (size_t i = 0; i < fc.outputSigIndices.size(); ++i) {
                    int sigIdx = fc.outputSigIndices[i];
                    double ctrlVal = (sigIdx >= 0 && sigIdx < (int)flatControlSignals.size()) ? flatControlSignals[sigIdx] : 0.0;
                    if (i == 0) firstVal = ctrlVal;
                    if (i < fc.customScriptOutputVecPtrs.size() && fc.customScriptOutputVecPtrs[i]) {
                        fc.customScriptOutputVecPtrs[i]->push_back(ctrlVal);
                    }
                    if (i < fc.customScriptPlotVecPtrs.size() && fc.customScriptPlotVecPtrs[i]) {
                        fc.customScriptPlotVecPtrs[i]->push_back(ctrlVal);
                    }
                }
                // "<id>" and "<id>.Out" were registered for every block but never filled
                // for per-port blocks, so they came back empty. Mirror the first port
                // into them, unless the key is itself one of the ports (e.g. ABC_DQ's
                // outKey is ".d") - that vector was already written above.
                auto isPortVec = [&](const std::vector<double>* p) {
                    return std::find(fc.customScriptOutputVecPtrs.begin(), fc.customScriptOutputVecPtrs.end(), p) != fc.customScriptOutputVecPtrs.end() ||
                           std::find(fc.customScriptPlotVecPtrs.begin(), fc.customScriptPlotVecPtrs.end(), p) != fc.customScriptPlotVecPtrs.end();
                };
                for (std::vector<double>* p : {fc.sigVecPtr, fc.sigOutVecPtr, fc.vPlotVecPtr, fc.iPlotVecPtr}) {
                    if (p && !isPortVec(p)) p->push_back(firstVal);
                }
            } else {
                double ctrlVal = (fc.outSignalIdx >= 0 && fc.outSignalIdx < (int)flatControlSignals.size()) ? flatControlSignals[fc.outSignalIdx] : 0.0;
                if (fc.sigVecPtr) fc.sigVecPtr->push_back(ctrlVal);
                if (fc.sigOutVecPtr) fc.sigOutVecPtr->push_back(ctrlVal);
                if (fc.vPlotVecPtr) fc.vPlotVecPtr->push_back(ctrlVal);
                if (fc.iPlotVecPtr) fc.iPlotVecPtr->push_back(ctrlVal);
            }
        }

        // The old "grow h by 1.2x up to 5x whenever no device switched" heuristic used to
        // live here. It had no error estimate behind it and, because the matrix and the
        // control blocks kept using the nominal step, growing h changed only the time
        // axis. It is removed rather than ported; adaptive stepping is handled by the
        // error-controlled attempt loop above.
            nextOutputTime = sampleTime + config.outputDecimation;
        }

        (void)statesChanged;

        if (isFixed) {
            h = dtBase;
            currentTime += h;
        } else {
            currentTime = sampleTime;

            // Control-block state advances exactly once per accepted step, for the
            // interval that was actually taken. This is what keeps rejected and
            // event-truncated attempts from corrupting integrators, latches,
            // flip-flops, delay histories and script engines.
            evaluateControls(currentTime, h);

            // Pick the next step from the error just measured. Quantisation snaps the
            // request down onto the ladder, so growth happens in doublings.
            double factor = 4.0;
            if (lastErrAccepted > 1e-10) factor = 0.9 / std::sqrt(lastErrAccepted);
            if (factor < 0.5) factor = 0.5;
            if (factor > 4.0) factor = 4.0;
            double hNext = quantiseStep(h * factor);
            if (hNext < hFloor) hNext = hFloor;
            if (hNext > hCeil) hNext = hCeil;
            h = hNext;
        }

        // â”€â”€ Live telemetry publish for real-time plotting â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
        // Paced by wall clock (~30 Hz) rather than by step count, so the cost is
        // independent of step size, and appends only the new tail rather than
        // deep-copying the whole history.
        if ((iterCount & 63) == 0) {
            auto simClockCur = std::chrono::high_resolution_clock::now();
            double sinceLastMs = std::chrono::duration<double, std::milli>(simClockCur - lastPublishClock).count();
            if (sinceLastMs >= kTelemetryPublishIntervalMs) {
                setComputeTimeSeconds(std::chrono::duration<double>(simClockCur - simClockStart).count());
                appendTelemetryFrom(out);
                lastPublishClock = simClockCur;
                if (progressCb_) progressCb_(currentTime, tStop);
            }
        }
    }

    auto simClockEnd = std::chrono::high_resolution_clock::now();
    double finalElSec = std::chrono::duration<double>(simClockEnd - simClockStart).count();
    setComputeTimeSeconds(finalElSec);
    if (progressCb_) progressCb_(currentTime, tStop);

    return out;
}

} // namespace CircuitSimEngine
