#include "CircuitSimulator.hpp"
#include <cmath>
#include <iostream>
#include <algorithm>
#include <sstream>
#include <random>

namespace CircuitSimEngine {

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
        } else if (comp.type == ComponentType::Diode) {
            diodeStatePrev[comp.id] = 0.0; // Initially OFF
        } else if (comp.type == ComponentType::Switch) {
            switchStatePrev[comp.id] = 0.0;
        } else if (comp.type == ComponentType::Transformer) {
            int wCount = (int)comp.nodes.size() / 2;
            if (wCount < 2) wCount = 2;
            numXfmrWindings += wCount;
        }
    }

    numNodes = nIdx - 1;
    totalDim = numNodes + (int)vSourceToIdx.size() + (int)inductorToIdx.size() + numXfmrWindings;

    K.assign(totalDim * totalDim, 0.0);
    K_static.assign(totalDim * totalDim, 0.0);
    K_prev.assign(totalDim * totalDim, 0.0);
    B.assign(totalDim, 0.0);
    X.assign(totalDim, 0.0);

    LU_buf.assign(totalDim * totalDim, 0.0);
    LU_cached.assign(totalDim * totalDim, 0.0);
    x_buf.assign(totalDim, 0.0);
    p_buf.assign(totalDim, 0);
    p_cached.assign(totalDim, 0);

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

        if (comp.type == ComponentType::Transformer) {
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

        fc.val = evaluateParam(comp, "value", 1000.0);
        if (comp.type == ComponentType::Capacitor) {
            fc.val = evaluateParam(comp, "C", 1e-6);
            fc.stateIdx = (int)flatCapVoltages.size();
            flatCapVoltages.push_back(capVoltagesPrev[comp.id]);
        }
        else if (comp.type == ComponentType::Inductor) {
            fc.val = evaluateParam(comp, "L", 1e-3);
            fc.stateIdx = (int)flatIndCurrents.size();
            flatIndCurrents.push_back(indCurrentsPrev[comp.id]);
            flatIndVoltages.push_back(0.0);
        }
        else if (comp.type == ComponentType::Diode) {
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
        fc.Vvd = evaluateParam(comp, "Vd", 0.7);
        fc.freq = evaluateParam(comp, "freq", 50.0);
        if (comp.type == ComponentType::ACVoltageSource) {
            // Web-tool netlist uses "amplitude" and "frequency" keys
            if (comp.parameters.count("amplitude")) fc.val = evaluateParam(comp, "amplitude", 1.0);
            if (comp.parameters.count("frequency")) fc.freq = evaluateParam(comp, "frequency", 50.0);
            if (comp.parameters.count("phase")) fc.delay = evaluateParam(comp, "phase", 0.0);
            // Also support "value" as amplitude alias (Windows tool schematic)
            if (!comp.parameters.count("amplitude") && comp.parameters.count("value")) fc.val = evaluateParam(comp, "value", 100.0);
        }

        fc.vPlotKey = "V_" + comp.id;
        fc.iPlotKey = "I_" + comp.id;
        fc.ctrlSigKey = getParamString(comp, "control_signal", "");

        fc.vPlotSignalIdx = getOrCreateSignalIdx(fc.vPlotKey);
        fc.iPlotSignalIdx = getOrCreateSignalIdx(fc.iPlotKey);
        fc.ctrlSigSignalIdx = getOrCreateSignalIdx(fc.ctrlSigKey);
        fc.outSignalIdx = getOrCreateSignalIdx(comp.id + ".Out");
        getOrCreateSignalIdx(comp.id);

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
            fc.delay = evaluateParam(ctrlComp, "phase", 0.0); // phase degrees
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
        } else if (ctrlComp.type == ComponentType::RandomNumbers) {
            fc.val = evaluateParam(ctrlComp, "mean", 0.0);
            fc.gain = evaluateParam(ctrlComp, "std", 1.0);
        } else if (ctrlComp.type == ComponentType::WhiteNoise) {
            fc.val = evaluateParam(ctrlComp, "psd", 0.1);
        } else if (ctrlComp.type == ComponentType::InitialCondition) {
            fc.val = evaluateParam(ctrlComp, "initial_value", 0.0);
            if (ctrlComp.parameters.count("x0")) fc.val = evaluateParam(ctrlComp, "x0", 0.0);
        } else if (ctrlComp.type == ComponentType::TrigFunction) {
            fc.polarity = getParamString(ctrlComp, "function", "sin");
        } else if (ctrlComp.type == ComponentType::Round) {
            fc.polarity = getParamString(ctrlComp, "mode", "nearest");
        } else if (ctrlComp.type == ComponentType::MinMax) {
            fc.polarity = getParamString(ctrlComp, "function", "min");
        } else if (ctrlComp.type == ComponentType::LUT_1D) {
            fc.polarity = getParamString(ctrlComp, "x", "[0, 1]");
            if (ctrlComp.parameters.count("x_data")) fc.polarity = getParamString(ctrlComp, "x_data", "[0, 1]");
            fc.vPlotKey = getParamString(ctrlComp, "y", "[0, 1]");
            if (ctrlComp.parameters.count("y_data")) fc.vPlotKey = getParamString(ctrlComp, "y_data", "[0, 1]");
        }

        if (ctrlComp.type == ComponentType::PI_Controller) {
            fc.stateIdx = (int)flatPiIntegratorState.size();
            flatPiIntegratorState.push_back(0.0);
            piIntegratorState[ctrlComp.id] = 0.0;
        }

        fc.in0Key = getParamString(ctrlComp, "In", "");
        if (fc.in0Key.empty()) fc.in0Key = getParamString(ctrlComp, "In1", "");
        if (fc.in0Key.empty()) fc.in0Key = getParamString(ctrlComp, "Plus", "");
        if (fc.in0Key.empty()) fc.in0Key = getParamString(ctrlComp, "input_0", "");
        if (fc.in0Key.empty()) fc.in0Key = getParamString(ctrlComp, "input", "");   // e.g. GAIN "input": "TRI1.Out"
        if (fc.in0Key.empty()) fc.in0Key = getParamString(ctrlComp, "input1", "");

        fc.in1Key = getParamString(ctrlComp, "In2", "");
        if (fc.in1Key.empty()) fc.in1Key = getParamString(ctrlComp, "Minus", "");
        if (fc.in1Key.empty()) fc.in1Key = getParamString(ctrlComp, "input_1", "");
        if (fc.in1Key.empty()) fc.in1Key = getParamString(ctrlComp, "input2", "");  // secondary input alias
        fc.outKey = getParamString(ctrlComp, "output", "");
        fc.targetKey = getParamString(ctrlComp, "target", "");
        fc.ctrlSigKey = getParamString(ctrlComp, "selected_signals", "");

        if (fc.outKey.empty()) {
            if (ctrlComp.type == ComponentType::UnifiedProbe && !fc.ctrlSigKey.empty()) {
                fc.outKey = ctrlComp.id + "." + fc.ctrlSigKey;
            } else {
                fc.outKey = ctrlComp.id + ".Out";
            }
        }

        fc.in0SignalIdx = getOrCreateSignalIdx(fc.in0Key);
        fc.in1SignalIdx = getOrCreateSignalIdx(fc.in1Key);
        fc.outSignalIdx = getOrCreateSignalIdx(fc.outKey);
        fc.targetSignalIdx = getOrCreateSignalIdx(fc.targetKey);
        fc.ctrlSigSignalIdx = getOrCreateSignalIdx(fc.ctrlSigKey);
        getOrCreateSignalIdx(fc.id);

        if (ctrlComp.type == ComponentType::CustomScript) {
            std::string code = getParamString(ctrlComp, "code", "");
            cscriptEngines[ctrlComp.id].setup(code, ctrlComp.parameters);

            for (int i = 0; i < 20; ++i) {
                std::string inK = getParamString(ctrlComp, "In" + std::to_string(i + 1), "");
                if (inK.empty()) inK = getParamString(ctrlComp, "input_" + std::to_string(i), "");
                if (inK.empty()) inK = ctrlComp.id + ".In" + std::to_string(i + 1);

                std::string outK = getParamString(ctrlComp, "Out" + std::to_string(i + 1), "");
                if (outK.empty()) outK = getParamString(ctrlComp, "output_" + std::to_string(i), "");
                if (outK.empty()) outK = ctrlComp.id + ".Out" + std::to_string(i + 1);

                fc.inputSigKeys.push_back(inK);
                fc.outputSigKeys.push_back(outK);
                fc.inputSigIndices.push_back(getOrCreateSignalIdx(inK));
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

    for (auto& fc : fastCtrlComps) {
        if (fc.in0SignalIdx >= 0 && fc.in0SignalIdx < (int)flatControlSignals.size()) fc.in0Ptr = &flatControlSignals[fc.in0SignalIdx];
        if (fc.in1SignalIdx >= 0 && fc.in1SignalIdx < (int)flatControlSignals.size()) fc.in1Ptr = &flatControlSignals[fc.in1SignalIdx];
        if (fc.outSignalIdx >= 0 && fc.outSignalIdx < (int)flatControlSignals.size()) fc.outPtr = &flatControlSignals[fc.outSignalIdx];
        if (fc.ctrlSigSignalIdx >= 0 && fc.ctrlSigSignalIdx < (int)flatControlSignals.size()) fc.ctrlSigPtr = &flatControlSignals[fc.ctrlSigSignalIdx];
        if (fc.targetSignalIdx >= 0 && fc.targetSignalIdx < (int)flatControlSignals.size()) fc.targetPtr = &flatControlSignals[fc.targetSignalIdx];
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
        else if (fc.type == ComponentType::Transformer) {
            if (fc.windings.empty()) continue;

            // 1. KCL contributions of all windings (rows n1..n2, column wIdx)
            for (const auto& w : fc.windings) {
                if (w.n1 >= 0) K_static[w.n1 * totalDim + w.wIdx] += 1.0;
                if (w.n2 >= 0) K_static[w.n2 * totalDim + w.wIdx] -= 1.0;
            }

            const auto& w0 = fc.windings[0];
            std::string polStr = fc.polarity;

            // 2. Ampere's Law (MMF balance) in row w0.wIdx: Sum(N_k * I_wk) = 0
            for (size_t k = 0; k < fc.windings.size(); ++k) {
                const auto& w = fc.windings[k];
                double polSign = (polStr == "inverted" && k == 1) ? -1.0 : 1.0;
                K_static[w0.wIdx * totalDim + w.wIdx] += polSign * w.turns;
            }

            // 3. Faraday's Law (Voltage ratio) in row w_k.wIdx: N_k * (V0_1 - V0_2) - N0 * (Vk_1 - Vk_2) = 0
            for (size_t k = 1; k < fc.windings.size(); ++k) {
                const auto& wk = fc.windings[k];
                double polSign = (polStr == "inverted" && k == 1) ? -1.0 : 1.0;
                if (w0.n1 >= 0) K_static[wk.wIdx * totalDim + w0.n1] += polSign * wk.turns;
                if (w0.n2 >= 0) K_static[wk.wIdx * totalDim + w0.n2] -= polSign * wk.turns;
                if (wk.n1 >= 0) K_static[wk.wIdx * totalDim + wk.n1] -= w0.turns;
                if (wk.n2 >= 0) K_static[wk.wIdx * totalDim + wk.n2] += w0.turns;
            }
        }
    }
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

    for (int i = 0; i < n; i++) {
        x_buf[i] = B[p_cached[i]];
    }

    // Forward substitution L*y = b
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < i; j++) {
            x_buf[i] -= LU_cached[i * n + j] * x_buf[j];
        }
    }

    // Backward substitution U*x = y
    for (int i = n - 1; i >= 0; i--) {
        for (int j = i + 1; j < n; j++) {
            x_buf[i] -= LU_cached[i * n + j] * x_buf[j];
        }
        x_buf[i] /= LU_cached[i * n + i];
    }

    std::copy(x_buf.begin(), x_buf.end(), X.begin());
    return true;
}

bool CircuitSimulator::solveLUFast(int n) {
    factorizeLU(n);
    return solveLUSubstitution(n);
}

void CircuitSimulator::evaluateControls(double currentTime) {
    for (auto& fc : fastPhysComps) {
        if (fc.type == ComponentType::Voltmeter) {
            int n1 = fc.n1, n2 = fc.n2;
            double v1 = (n1 >= 0 && n1 < totalDim) ? X[n1] : 0.0;
            double v2 = (n2 >= 0 && n2 < totalDim) ? X[n2] : 0.0;
            double vDiff = v1 - v2;
            int sigIdx = fc.outSignalIdx;
            if (sigIdx >= 0 && sigIdx < (int)flatControlSignals.size()) {
                flatControlSignals[sigIdx] = vDiff;
            }
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
                val = fc.amplitude * std::sin(2.0 * 3.141592653589793 * fc.freq * currentTime + fc.delay * 3.141592653589793 / 180.0);
            }
            else if (fc.type == ComponentType::Step) {
                val = (currentTime >= fc.delay) ? fc.maxVal : fc.minVal;
            }
            else if (fc.type == ComponentType::RandomNumbers) {
                thread_local std::mt19937 gen(12345);
                std::normal_distribution<double> dist(fc.val, (fc.gain > 0.0 ? fc.gain : 1.0));
                val = dist(gen);
            }
            else if (fc.type == ComponentType::WhiteNoise) {
                thread_local std::mt19937 gen(54321);
                std::normal_distribution<double> dist(0.0, 1.0);
                double dt = (config.stepSize > 0.0) ? config.stepSize : 1e-5;
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
                std::string f = fc.polarity;
                std::transform(f.begin(), f.end(), f.begin(), ::tolower);
                if (f == "cos") val = std::cos(inVal);
                else if (f == "tan") val = std::tan(inVal);
                else if (f == "asin") val = std::asin(std::max(-1.0, std::min(1.0, inVal)));
                else if (f == "acos") val = std::acos(std::max(-1.0, std::min(1.0, inVal)));
                else if (f == "atan") val = std::atan(inVal);
                else if (f == "atan2") val = std::atan2(inVal, inVal2);
                else if (f == "sinh") val = std::sinh(inVal);
                else if (f == "cosh") val = std::cosh(inVal);
                else if (f == "tanh") val = std::tanh(inVal);
                else val = std::sin(inVal);
            }
            else if (fc.type == ComponentType::Round) {
                double inVal = fc.in0Ptr ? *fc.in0Ptr : 0.0;
                if (fc.polarity == "floor") val = std::floor(inVal);
                else if (fc.polarity == "ceil") val = std::ceil(inVal);
                else val = std::round(inVal);
            }
            else if (fc.type == ComponentType::MinMax) {
                double in0 = fc.in0Ptr ? *fc.in0Ptr : 0.0;
                double in1 = fc.in1Ptr ? *fc.in1Ptr : 0.0;
                if (fc.polarity == "max") val = std::max(in0, in1);
                else val = std::min(in0, in1);
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
                if (pass == 0 && fc.stateIdx >= 0 && fc.stateIdx < (int)flatPiIntegratorState.size()) {
                    flatPiIntegratorState[fc.stateIdx] += err * config.stepSize;
                }
                double piVal = (fc.stateIdx >= 0 && fc.stateIdx < (int)flatPiIntegratorState.size()) ? flatPiIntegratorState[fc.stateIdx] : 0.0;
                val = fc.Kp * err + fc.Ki * piVal;
            }
            else if (fc.type == ComponentType::CustomScript) {
                for (size_t i = 0; i < fc.inputSigIndices.size(); ++i) {
                    int inIdx = fc.inputSigIndices[i];
                    double inVal = (inIdx >= 0 && inIdx < (int)flatControlSignals.size()) ? flatControlSignals[inIdx] : 0.0;
                    scriptInValsBuf[i] = inVal;
                }

                auto cIt = cscriptEngines.find(fc.id);
                if (cIt != cscriptEngines.end()) {
                    if (pass == 1) {
                        cIt->second.step(currentTime, scriptInValsBuf, config.stepSize);
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

                if (pass == 0) fc.esr = inVal; // update prev input

                if (detected && !isActive) {
                    isActive = true;
                    trigTime = currentTime;
                    if (pass == 0) { fc.minVal = 1.0; fc.delay = trigTime; }
                }
                if (isActive && trigTime >= 0.0 && (currentTime - trigTime) >= pulseW - 1e-12) {
                    isActive = false;
                    if (pass == 0) fc.minVal = 0.0;
                }
                val = isActive ? 1.0 : 0.0;
            }
            else if (fc.type == ComponentType::MathFunction) {
                double v1 = fc.in0Ptr ? *fc.in0Ptr : 0.0;
                double v2 = fc.in1Ptr ? *fc.in1Ptr : 2.0;
                std::string f = fc.polarity; // store function name in polarity string
                if (f.empty()) f = "square";

                if (f == "square") val = v1 * v1;
                else if (f == "sqrt" || f == "square root") val = std::sqrt(std::abs(v1));
                else if (f == "exp" || f == "exponential") val = std::exp(v1);
                else if (f == "log" || f == "ln" || f == "logarithm") val = std::log(std::abs(v1) + 1e-15);
                else if (f == "log10") val = std::log10(std::abs(v1) + 1e-15);
                else if (f == "reciprocal") val = 1.0 / (v1 == 0.0 ? 1e-15 : v1);
                else if (f == "abs") val = std::abs(v1);
                else if (f == "power" || f == "pow") val = std::pow(v1, v2);
                else if (f == "mod") { double m = (v2 == 0.0 ? 1.0 : v2); val = std::fmod(std::fmod(v1, m) + m, m); }
                else if (f == "rem") { double m = (v2 == 0.0 ? 1.0 : v2); val = std::fmod(v1, m); }
                else val = v1 * v1;
            }
            else if (fc.type == ComponentType::KeyTrigger) {
                val = (fc.val != 0.0) ? fc.val : 1.0;
            }
            else if (fc.type == ComponentType::UnifiedProbe) {
                double pVal = 0.0;
                if (fc.ctrlSigPtr) pVal = *fc.ctrlSigPtr;
                else if (fc.targetPtr) pVal = *fc.targetPtr;

                val = pVal;
            }

            if (fc.outPtr) {
                *fc.outPtr = val;
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
                if (iForward < 0.0) newState = 0.0;
            } else {
                if (vDiff >= fc.Vvd) newState = 1.0;
            }

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

void CircuitSimulator::assembleMNA(double currentTime) {
    std::copy(K_static.begin(), K_static.end(), K.begin());
    std::fill(B.begin(), B.end(), 0.0);

    double dt = config.stepSize;
    if (dt <= 0) dt = 1e-6;

    for (const auto& fc : fastPhysComps) {
        int n1 = fc.n1;
        int n2 = fc.n2;

        if (fc.type == ComponentType::Capacitor) {
            double C = fc.val;
            if (C < 1e-15) C = 1e-15;

            double rEq = (dt / C) + fc.esr;
            double gEq = 1.0 / rEq;
            double vCapPrev = (fc.stateIdx >= 0 && fc.stateIdx < (int)flatCapVoltages.size()) ? flatCapVoltages[fc.stateIdx] : 0.0;
            double iEq = gEq * vCapPrev;

            if (n1 >= 0) K[n1 * totalDim + n1] += gEq;
            if (n2 >= 0) K[n2 * totalDim + n2] += gEq;
            if (n1 >= 0 && n2 >= 0) {
                K[n1 * totalDim + n2] -= gEq;
                K[n2 * totalDim + n1] -= gEq;
            }

            if (n1 >= 0) B[n1] += iEq;
            if (n2 >= 0) B[n2] -= iEq;
        }
        else if (fc.type == ComponentType::Inductor) {
            double L = fc.val;
            if (L < 1e-12) L = 1e-12;
            int lIdx = fc.lIdx;
            bool useTrap = (config.solver == "trapezoidal" || config.solver == "trap" || config.solver == "rk4");
            if (forceBackwardEulerSteps > 0) {
                useTrap = false;
            }

            if (useTrap) {
                double rEq = (2.0 * L / dt) + fc.esr;
                double iPrev = (fc.stateIdx >= 0 && fc.stateIdx < (int)flatIndCurrents.size()) ? flatIndCurrents[fc.stateIdx] : 0.0;
                double vPrev = (fc.stateIdx >= 0 && fc.stateIdx < (int)flatIndVoltages.size()) ? flatIndVoltages[fc.stateIdx] : 0.0;

                K[lIdx * totalDim + lIdx] += rEq;
                B[lIdx] += (2.0 * L / dt) * iPrev + vPrev;
            } else {
                double rEq = (L / dt) + fc.esr;
                double iPrev = (fc.stateIdx >= 0 && fc.stateIdx < (int)flatIndCurrents.size()) ? flatIndCurrents[fc.stateIdx] : 0.0;

                K[lIdx * totalDim + lIdx] += rEq;
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
        else if (fc.type == ComponentType::Ammeter) {
            int vIdx = fc.vIdx;
            B[vIdx] = 0.0;
        }
        else if (fc.type == ComponentType::Diode) {
            double state = (fc.stateIdx >= 0 && fc.stateIdx < (int)flatDiodeStates.size()) ? flatDiodeStates[fc.stateIdx] : 0.0;
            double R = (state > 0.5) ? fc.Ron : fc.Roff;
            if (R < 1e-6) R = 1e-6;
            double g = 1.0 / R;

            if (n1 >= 0) K[n1 * totalDim + n1] += g;
            if (n2 >= 0) K[n2 * totalDim + n2] += g;
            if (n1 >= 0 && n2 >= 0) {
                K[n1 * totalDim + n2] -= g;
                K[n2 * totalDim + n1] -= g;
            }

            if (state > 0.5) {
                double iEq = g * fc.Vvd;
                if (n1 >= 0) B[n1] -= iEq;
                if (n2 >= 0) B[n2] += iEq;
            }
        }
        else if (fc.type == ComponentType::Switch) {
            double ctrlVal = fc.ctrlSigPtr ? *fc.ctrlSigPtr : 0.0;
            double R = (ctrlVal > 0.5) ? fc.Ron : fc.Roff;
            if (R < 1e-6) R = 1e-6;
            double g = 1.0 / R;

            if (n1 >= 0) K[n1 * totalDim + n1] += g;
            if (n2 >= 0) K[n2 * totalDim + n2] += g;
            if (n1 >= 0 && n2 >= 0) {
                K[n1 * totalDim + n2] -= g;
                K[n2 * totalDim + n1] -= g;
            }
        }
    }
}

SimulationOutput CircuitSimulator::runTransient() {
    SimulationOutput out;
    
    double tStop = config.stopTime > 0 ? config.stopTime : 0.01;
    double dtBase = config.stepSize > 0 ? config.stepSize : 1e-6;
    int estSteps = static_cast<int>(std::ceil(tStop / dtBase));

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

        if (fc.type == ComponentType::CustomScript) {
            for (size_t i = 0; i < fc.outputSigKeys.size(); ++i) {
                const auto& outK = fc.outputSigKeys[i];
                auto& sigVec = out.signals[outK];
                auto& cpVec = out.custom_plots[outK];
                sigVec.reserve(estSteps + 1);
                cpVec.reserve(estSteps + 1);
                fc.customScriptOutputVecPtrs.push_back(&sigVec);
                fc.customScriptPlotVecPtrs.push_back(&cpVec);
            }
        } else {
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
    }

    double currentTime = 0.0;
    bool isFixed = (config.step_type == "fixed") || (config.solver == "euler" && config.step_type != "variable");
    double h = dtBase;
    double h_max = dtBase * 5.0;

    int max_iterations = 300000;
    int iterCount = 0;
    matrixKChanged = true;

    while (currentTime < tStop - 1e-12 && iterCount < max_iterations) {
        iterCount++;
        if (currentTime + h > tStop) h = tStop - currentTime;

        // Step 1: Evaluate Control Loop blocks
        evaluateControls(currentTime);

        // Step 2: Iterative PWL solution loop for diode/switch convergence
        bool statesChanged = true;
        int pwlIter = 0;
        while (statesChanged && pwlIter < 20) {
            pwlIter++;
            assembleMNA(currentTime);

            if (totalDim > 0) {
                if (matrixKChanged || K != K_prev) {
                    factorizeLU(totalDim);
                    K_prev = K;
                    matrixKChanged = false;
                }
                solveLUSubstitution(totalDim);
            }

            statesChanged = updateDeviceStates();
            if (statesChanged) matrixKChanged = true;
        }

        if (forceBackwardEulerSteps > 0) forceBackwardEulerSteps--;

        // Store time step
        out.time.push_back(currentTime);

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
                double Rtotal = fc.val + fc.esr;
                if (Rtotal < 1e-6) Rtotal = 1e-6;
                iComp = vDiff / Rtotal;
            }
            else if (fc.type == ComponentType::Capacitor) {
                double C = fc.val;
                if (C < 1e-15) C = 1e-15;
                double rEq = (h / C) + fc.esr;
                double gEq = 1.0 / rEq;
                double vCapPrev = (fc.stateIdx >= 0 && fc.stateIdx < (int)flatCapVoltages.size()) ? flatCapVoltages[fc.stateIdx] : 0.0;
                iComp = gEq * (vDiff - vCapPrev);
                if (fc.stateIdx >= 0 && fc.stateIdx < (int)flatCapVoltages.size()) {
                    flatCapVoltages[fc.stateIdx] = vDiff;
                }
            }
            else if (fc.type == ComponentType::Inductor) {
                int lIdx = fc.lIdx;
                if (lIdx >= 0 && lIdx < totalDim) {
                    iComp = X[lIdx];
                    if (fc.stateIdx >= 0 && fc.stateIdx < (int)flatIndCurrents.size()) {
                        flatIndCurrents[fc.stateIdx] = iComp;
                        if (fc.stateIdx < (int)flatIndVoltages.size()) {
                            flatIndVoltages[fc.stateIdx] = vDiff;
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
            else if (fc.type == ComponentType::Switch) {
                double ctrlVal = fc.ctrlSigPtr ? *fc.ctrlSigPtr : 0.0;
                iComp = vDiff / ((ctrlVal > 0.5) ? fc.Ron : fc.Roff);
            }
            else if (fc.type == ComponentType::Transformer) {
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
            } else if (fc.type == ComponentType::Ammeter) {
                if (fc.vmVecPtr) fc.vmVecPtr->push_back(iComp);
                if (fc.sigVecPtr) fc.sigVecPtr->push_back(iComp);
                if (fc.sigOutVecPtr) fc.sigOutVecPtr->push_back(iComp);
            }
        }

        // Store control loop signals into output (Zero map lookups)
        for (auto& fc : fastCtrlComps) {
            if (fc.type == ComponentType::CustomScript) {
                for (size_t i = 0; i < fc.outputSigIndices.size(); ++i) {
                    int sigIdx = fc.outputSigIndices[i];
                    double ctrlVal = (sigIdx >= 0 && sigIdx < (int)flatControlSignals.size()) ? flatControlSignals[sigIdx] : 0.0;
                    if (i < fc.customScriptOutputVecPtrs.size() && fc.customScriptOutputVecPtrs[i]) fc.customScriptOutputVecPtrs[i]->push_back(ctrlVal);
                    if (i < fc.customScriptPlotVecPtrs.size() && fc.customScriptPlotVecPtrs[i]) fc.customScriptPlotVecPtrs[i]->push_back(ctrlVal);
                }
            } else {
                double ctrlVal = (fc.outSignalIdx >= 0 && fc.outSignalIdx < (int)flatControlSignals.size()) ? flatControlSignals[fc.outSignalIdx] : 0.0;
                if (fc.sigVecPtr) fc.sigVecPtr->push_back(ctrlVal);
                if (fc.sigOutVecPtr) fc.sigOutVecPtr->push_back(ctrlVal);
                if (fc.vPlotVecPtr) fc.vPlotVecPtr->push_back(ctrlVal);
                if (fc.iPlotVecPtr) fc.iPlotVecPtr->push_back(ctrlVal);
            }
        }

        if (!isFixed) {
            if (statesChanged) {
                h = dtBase;
                matrixKChanged = true;
            } else {
                h = std::min(h_max, h * 1.2);
            }
        } else {
            h = dtBase;
        }

        currentTime += h;
    }

    return out;
}

} // namespace CircuitSimEngine
