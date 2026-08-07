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
        } else if (ctrlComp.type == ComponentType::Integrator) {
            fc.val = evaluateParam(ctrlComp, "initial_condition", 0.0);
            if (ctrlComp.parameters.count("initial_value")) fc.val = evaluateParam(ctrlComp, "initial_value", 0.0);
            if (ctrlComp.parameters.count("x0")) fc.val = evaluateParam(ctrlComp, "x0", 0.0);
        } else if (ctrlComp.type == ComponentType::TransferFunction) {
            fc.polarity = getParamString(ctrlComp, "num", "[1]");
            fc.vPlotKey = getParamString(ctrlComp, "den", "[1, 1]");
        } else if (ctrlComp.type == ComponentType::ContinuousPID) {
            fc.gain = evaluateParam(ctrlComp, "Kp", 1.0);
            fc.vAlphaKey = std::to_string(evaluateParam(ctrlComp, "Ki", 0.0));
            fc.vBetaKey = std::to_string(evaluateParam(ctrlComp, "Kd", 0.0));
            fc.minVal = evaluateParam(ctrlComp, "Tf", 0.01);
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
            fc.minVal = evaluateParam(ctrlComp, "step_size", 0.5);
            fc.polarity = getParamString(ctrlComp, "mode", "round");
        } else if (ctrlComp.type == ComponentType::SignalSwitch) {
            fc.thresholdVal = evaluateParam(ctrlComp, "threshold", 0.5);
            fc.polarity = getParamString(ctrlComp, "criteria", "u2 >= threshold");
        } else if (ctrlComp.type == ComponentType::ManualSwitch) {
            fc.polarity = getParamString(ctrlComp, "state", "Input 1");
        } else if (ctrlComp.type == ComponentType::MultiportSwitch) {
            fc.polarity = getParamString(ctrlComp, "indexing", "1-based");
            fc.val = evaluateParam(ctrlComp, "inputs", 3.0);
        } else if (ctrlComp.type == ComponentType::HitCrossing) {
            fc.thresholdVal = evaluateParam(ctrlComp, "offset", 0.0);
            fc.polarity = getParamString(ctrlComp, "direction", "either");
        } else if (ctrlComp.type == ComponentType::Saturation) {
            fc.minVal = evaluateParam(ctrlComp, "min", -10.0);
            fc.maxVal = evaluateParam(ctrlComp, "max", 10.0);
        } else if (ctrlComp.type == ComponentType::DeadZone) {
            fc.minVal = evaluateParam(ctrlComp, "start", -0.5);
            fc.maxVal = evaluateParam(ctrlComp, "end", 0.5);
        } else if (ctrlComp.type == ComponentType::RateLimiter) {
            fc.rateUp = evaluateParam(ctrlComp, "up", 10.0);
            fc.rateDown = evaluateParam(ctrlComp, "down", -10.0);
        } else if (ctrlComp.type == ComponentType::Relay) {
            fc.onThresh = evaluateParam(ctrlComp, "on_threshold", 1.0);
            fc.offThresh = evaluateParam(ctrlComp, "off_threshold", -1.0);
        } else if (ctrlComp.type == ComponentType::LogicOp || ctrlComp.type == ComponentType::BitwiseOp) {
            fc.polarity = getParamString(ctrlComp, "operator", "AND");
        } else if (ctrlComp.type == ComponentType::CombLogic) {
            fc.polarity = getParamString(ctrlComp, "truth_table", "");
        } else if (ctrlComp.type == ComponentType::EdgeDetect) {
            fc.edgeMode = getParamString(ctrlComp, "edge", "rising");
            fc.pulseDuration = evaluateParam(ctrlComp, "pulse_width", 1e-3);
        } else if (ctrlComp.type == ComponentType::Monostable) {
            fc.pulseDuration = evaluateParam(ctrlComp, "duration", 0.1);
            fc.edgeMode = getParamString(ctrlComp, "edge", "rising");
        } else if (ctrlComp.type == ComponentType::Monoflop) {
            fc.pulseDuration = evaluateParam(ctrlComp, "duration", 0.1);
            fc.edgeMode = getParamString(ctrlComp, "trigger_edge", "rising");
            fc.retriggerable = (getParamString(ctrlComp, "retriggerable", "false") == "true");
        } else if (ctrlComp.type == ComponentType::RelationalOp) {
            fc.polarity = getParamString(ctrlComp, "operator", "==");
        } else if (ctrlComp.type == ComponentType::CompareToConstant) {
            fc.polarity = getParamString(ctrlComp, "operator", "==");
            fc.thresholdVal = evaluateParam(ctrlComp, "constant", 0.0);
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
        } else if (ctrlComp.type == ComponentType::SummingJunction) {
            fc.polarity = getParamString(ctrlComp, "signs", "++");
        } else if (ctrlComp.type == ComponentType::Product) {
            fc.polarity = getParamString(ctrlComp, "operators", "**");
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
        if (fc.in0Key.empty()) fc.in0Key = getParamString(ctrlComp, "input", "");   // e.g. GAIN "input": "TRI1.Out"
        if (fc.in0Key.empty()) fc.in0Key = getParamString(ctrlComp, "input1", "");

        fc.in1Key = getParamString(ctrlComp, "In2", "");
        if (fc.in1Key.empty()) fc.in1Key = getParamString(ctrlComp, "Den", "");
        if (fc.in1Key.empty()) fc.in1Key = getParamString(ctrlComp, "Minus", "");
        if (fc.in1Key.empty()) fc.in1Key = getParamString(ctrlComp, "input_1", "");
        if (fc.in1Key.empty()) fc.in1Key = getParamString(ctrlComp, "input2", "");  // secondary input alias
        fc.outKey = getParamString(ctrlComp, "output", "");
        fc.targetKey = getParamString(ctrlComp, "target", "");
        fc.ctrlSigKey = getParamString(ctrlComp, "selected_signals", "");
        // For flip-flops and shift registers, clock comes from "Ctrl" parameter
        if (fc.ctrlSigKey.empty()) {
            if (ctrlComp.type == ComponentType::DFlipFlop ||
                ctrlComp.type == ComponentType::JKFlipFlop ||
                ctrlComp.type == ComponentType::ShiftReg) {
                fc.ctrlSigKey = getParamString(ctrlComp, "Ctrl", "");
            }
        }

        if (ctrlComp.type == ComponentType::Clarke) {
            fc.outKey = ctrlComp.id + ".Alpha";
            std::string inA = getParamString(ctrlComp, "A", ""); if (inA.empty()) inA = getParamString(ctrlComp, "In1", "");
            std::string inB = getParamString(ctrlComp, "B", ""); if (inB.empty()) inB = getParamString(ctrlComp, "In2", "");
            std::string inC = getParamString(ctrlComp, "C", ""); if (inC.empty()) inC = getParamString(ctrlComp, "In3", "");
            fc.inputSigIndices.push_back(getOrCreateSignalIdx(inA));
            fc.inputSigIndices.push_back(getOrCreateSignalIdx(inB));
            fc.inputSigIndices.push_back(getOrCreateSignalIdx(inC));
            fc.outputSigKeys.push_back(ctrlComp.id + ".Alpha");
            fc.outputSigKeys.push_back(ctrlComp.id + ".Beta");
            fc.outputSigIndices.push_back(getOrCreateSignalIdx(ctrlComp.id + ".Alpha"));
            fc.outputSigIndices.push_back(getOrCreateSignalIdx(ctrlComp.id + ".Beta"));
        } else if (ctrlComp.type == ComponentType::InvClarke) {
            fc.outKey = ctrlComp.id + ".A";
            std::string inAlpha = getParamString(ctrlComp, "Alpha", ""); if (inAlpha.empty()) inAlpha = getParamString(ctrlComp, "In1", "");
            std::string inBeta = getParamString(ctrlComp, "Beta", ""); if (inBeta.empty()) inBeta = getParamString(ctrlComp, "In2", "");
            fc.inputSigIndices.push_back(getOrCreateSignalIdx(inAlpha));
            fc.inputSigIndices.push_back(getOrCreateSignalIdx(inBeta));
            fc.outputSigKeys.push_back(ctrlComp.id + ".A");
            fc.outputSigKeys.push_back(ctrlComp.id + ".B");
            fc.outputSigKeys.push_back(ctrlComp.id + ".C");
            fc.outputSigIndices.push_back(getOrCreateSignalIdx(ctrlComp.id + ".A"));
            fc.outputSigIndices.push_back(getOrCreateSignalIdx(ctrlComp.id + ".B"));
            fc.outputSigIndices.push_back(getOrCreateSignalIdx(ctrlComp.id + ".C"));
        } else if (ctrlComp.type == ComponentType::Park) {
            fc.outKey = ctrlComp.id + ".d";
            std::string inAlpha = getParamString(ctrlComp, "Alpha", ""); if (inAlpha.empty()) inAlpha = getParamString(ctrlComp, "In1", "");
            std::string inBeta = getParamString(ctrlComp, "Beta", ""); if (inBeta.empty()) inBeta = getParamString(ctrlComp, "In2", "");
            std::string inTheta = getParamString(ctrlComp, "Theta", ""); if (inTheta.empty()) inTheta = getParamString(ctrlComp, "In3", "");
            fc.inputSigIndices.push_back(getOrCreateSignalIdx(inAlpha));
            fc.inputSigIndices.push_back(getOrCreateSignalIdx(inBeta));
            fc.inputSigIndices.push_back(getOrCreateSignalIdx(inTheta));
            fc.outputSigKeys.push_back(ctrlComp.id + ".d");
            fc.outputSigKeys.push_back(ctrlComp.id + ".q");
            fc.outputSigIndices.push_back(getOrCreateSignalIdx(ctrlComp.id + ".d"));
            fc.outputSigIndices.push_back(getOrCreateSignalIdx(ctrlComp.id + ".q"));
        } else if (ctrlComp.type == ComponentType::InvPark) {
            fc.outKey = ctrlComp.id + ".Alpha";
            std::string inD = getParamString(ctrlComp, "d", ""); if (inD.empty()) inD = getParamString(ctrlComp, "In1", "");
            std::string inQ = getParamString(ctrlComp, "q", ""); if (inQ.empty()) inQ = getParamString(ctrlComp, "In2", "");
            std::string inTheta = getParamString(ctrlComp, "Theta", ""); if (inTheta.empty()) inTheta = getParamString(ctrlComp, "In3", "");
            fc.inputSigIndices.push_back(getOrCreateSignalIdx(inD));
            fc.inputSigIndices.push_back(getOrCreateSignalIdx(inQ));
            fc.inputSigIndices.push_back(getOrCreateSignalIdx(inTheta));
            fc.outputSigKeys.push_back(ctrlComp.id + ".Alpha");
            fc.outputSigKeys.push_back(ctrlComp.id + ".Beta");
            fc.outputSigIndices.push_back(getOrCreateSignalIdx(ctrlComp.id + ".Alpha"));
            fc.outputSigIndices.push_back(getOrCreateSignalIdx(ctrlComp.id + ".Beta"));
        } else if (ctrlComp.type == ComponentType::PWM_3PH || ctrlComp.type == ComponentType::SVPWM) {
            fc.outKey = ctrlComp.id + ".OutA";
            std::string inA = getParamString(ctrlComp, "A", ""); if (inA.empty()) inA = getParamString(ctrlComp, "In1", "");
            std::string inB = getParamString(ctrlComp, "B", ""); if (inB.empty()) inB = getParamString(ctrlComp, "In2", "");
            std::string inC = getParamString(ctrlComp, "C", ""); if (inC.empty()) inC = getParamString(ctrlComp, "In3", "");
            fc.inputSigIndices.push_back(getOrCreateSignalIdx(inA));
            fc.inputSigIndices.push_back(getOrCreateSignalIdx(inB));
            fc.inputSigIndices.push_back(getOrCreateSignalIdx(inC));
            fc.outputSigKeys.push_back(ctrlComp.id + ".OutA");
            fc.outputSigKeys.push_back(ctrlComp.id + ".OutB");
            fc.outputSigKeys.push_back(ctrlComp.id + ".OutC");
            fc.outputSigIndices.push_back(getOrCreateSignalIdx(ctrlComp.id + ".OutA"));
            fc.outputSigIndices.push_back(getOrCreateSignalIdx(ctrlComp.id + ".OutB"));
            fc.outputSigIndices.push_back(getOrCreateSignalIdx(ctrlComp.id + ".OutC"));
        } else if (ctrlComp.type == ComponentType::FourierTrans || ctrlComp.type == ComponentType::FourierAnalysis) {
            fc.outKey = ctrlComp.id + ".Mag";
            fc.outputSigKeys.push_back(ctrlComp.id + ".Mag");
            fc.outputSigKeys.push_back(ctrlComp.id + ".Phase");
            fc.outputSigIndices.push_back(getOrCreateSignalIdx(ctrlComp.id + ".Mag"));
            fc.outputSigIndices.push_back(getOrCreateSignalIdx(ctrlComp.id + ".Phase"));
        } else if (ctrlComp.type == ComponentType::PllLoop) {
            fc.outKey = ctrlComp.id + ".Theta";
            fc.outputSigKeys.push_back(ctrlComp.id + ".Theta");
            fc.outputSigKeys.push_back(ctrlComp.id + ".Freq");
            fc.outputSigKeys.push_back(ctrlComp.id + ".Cos");
            fc.outputSigKeys.push_back(ctrlComp.id + ".Sin");
            fc.outputSigIndices.push_back(getOrCreateSignalIdx(ctrlComp.id + ".Theta"));
            fc.outputSigIndices.push_back(getOrCreateSignalIdx(ctrlComp.id + ".Freq"));
            fc.outputSigIndices.push_back(getOrCreateSignalIdx(ctrlComp.id + ".Cos"));
            fc.outputSigIndices.push_back(getOrCreateSignalIdx(ctrlComp.id + ".Sin"));
        } else if (ctrlComp.type == ComponentType::PeriodicImpAvg) {
            fc.ctrlSigKey = getParamString(ctrlComp, "Trig", "");
        }

        if (fc.outKey.empty()) {
            if (ctrlComp.type == ComponentType::UnifiedProbe && !fc.ctrlSigKey.empty()) {
                fc.outKey = ctrlComp.id + "." + fc.ctrlSigKey;
            } else if (ctrlComp.type == ComponentType::DFlipFlop || ctrlComp.type == ComponentType::JKFlipFlop) {
                fc.outKey = ctrlComp.id + ".Q";
                // Register both Q and Q_bar in outputSigIndices
                fc.outputSigIndices.push_back(getOrCreateSignalIdx(ctrlComp.id + ".Q"));
                fc.outputSigIndices.push_back(getOrCreateSignalIdx(ctrlComp.id + ".Q_bar"));
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
            else if (fc.type == ComponentType::Integrator) {
                double inVal = fc.in0Ptr ? *fc.in0Ptr : 0.0;
                double dt = (config.stepSize > 0.0) ? config.stepSize : 1e-5;
                if (currentTime == 0.0) {
                    fc.stateVal = fc.val;
                } else {
                    fc.stateVal += inVal * dt;
                }
                val = fc.stateVal;
            }
            else if (fc.type == ComponentType::Derivative) {
                double inVal = fc.in0Ptr ? *fc.in0Ptr : 0.0;
                double dt = (config.stepSize > 0.0) ? config.stepSize : 1e-5;
                if (currentTime == 0.0) {
                    fc.stateVal = inVal;
                    val = 0.0;
                } else {
                    val = (inVal - fc.stateVal) / dt;
                    fc.stateVal = inVal;
                }
            }
            else if (fc.type == ComponentType::TransferFunction) {
                double inVal = fc.in0Ptr ? *fc.in0Ptr : 0.0;
                double dt = (config.stepSize > 0.0) ? config.stepSize : 1e-5;
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
                std::vector<double> num = parseVec(fc.polarity);
                std::vector<double> den = parseVec(fc.vPlotKey);
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

                    std::vector<double> xDot(n, 0.0);
                    for (size_t k = 0; k < n - 1; ++k) xDot[k] = fc.stateVector[k + 1];
                    double sumA = 0.0;
                    for (size_t k = 0; k < n; ++k) sumA += denNorm[n - k] * fc.stateVector[k];
                    xDot[n - 1] = -sumA + inVal;

                    if (currentTime > 0.0) {
                        for (size_t k = 0; k < n; ++k) fc.stateVector[k] += xDot[k] * dt;
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
                double dt = (config.stepSize > 0.0) ? config.stepSize : 1e-5;
                double Kp = fc.gain;
                double Ki = std::atof(fc.vAlphaKey.c_str());
                double Kd = std::atof(fc.vBetaKey.c_str());
                double Tf = (fc.minVal > 0.0) ? fc.minVal : 0.01;

                if (currentTime == 0.0) {
                    fc.stateVal = 0.0;
                    fc.filterState = 0.0;
                }
                fc.stateVal += Ki * inVal * dt;
                double dTerm = (Kd / (Tf + dt)) * (inVal - fc.filterState);
                fc.filterState = (Tf / (Tf + dt)) * fc.filterState + (dt / (Tf + dt)) * inVal;
                val = Kp * inVal + fc.stateVal + dTerm;
            }
            else if (fc.type == ComponentType::PLL_1PH || fc.type == ComponentType::PLL_3PH) {
                double inVal = fc.in0Ptr ? *fc.in0Ptr : 0.0;
                double dt = (config.stepSize > 0.0) ? config.stepSize : 1e-5;
                double fn = (fc.freq > 0.0) ? fc.freq : 50.0;
                double w0 = 2.0 * 3.141592653589793 * fn;
                if (currentTime == 0.0) fc.stateVal = 0.0;
                fc.stateVal = std::fmod(fc.stateVal + w0 * dt, 2.0 * 3.141592653589793);
                val = fc.stateVal;
            }
            else if (fc.type == ComponentType::Delay || fc.type == ComponentType::TransportDelay) {
                double inVal = fc.in0Ptr ? *fc.in0Ptr : 0.0;
                double delayDuration = (fc.delayDuration > 0.0) ? fc.delayDuration : 0.1;
                
                if (fc.delayHistory.empty() || currentTime > fc.delayHistory.back().t) {
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
                if (isHigh) {
                    if (!fc.prevInputHigh) {
                        fc.highStartTime = currentTime;
                    }
                } else {
                    fc.highStartTime = -1.0;
                }
                fc.prevInputHigh = isHigh;
                
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
                } else if (currentTime > fc.lastTime) {
                    fc.prevVal = fc.currentVal;
                    fc.currentVal = inVal;
                    fc.lastTime = currentTime;
                }
                val = fc.prevVal;
            }
            else if (fc.type == ComponentType::Quantizer) {
                double inVal = fc.in0Ptr ? *fc.in0Ptr : 0.0;
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
                double prev = fc.prevVal;
                if (fc.polarity == "rising" && prev < offset && inVal >= offset) hit = 1.0;
                else if (fc.polarity == "falling" && prev > offset && inVal <= offset) hit = 1.0;
                else if ((prev < offset && inVal >= offset) || (prev > offset && inVal <= offset)) hit = 1.0;
                if (currentTime > fc.lastTime) {
                    fc.prevVal = inVal;
                    fc.lastTime = currentTime;
                }
                val = hit;
            }
            else if (fc.type == ComponentType::Saturation) {
                double inVal = fc.in0Ptr ? *fc.in0Ptr : 0.0;
                val = std::max(fc.minVal, std::min(fc.maxVal, inVal));
            }
            else if (fc.type == ComponentType::DeadZone) {
                double inVal = fc.in0Ptr ? *fc.in0Ptr : 0.0;
                if (inVal > fc.maxVal) val = inVal - fc.maxVal;
                else if (inVal < fc.minVal) val = inVal - fc.minVal;
                else val = 0.0;
            }
            else if (fc.type == ComponentType::RateLimiter) {
                double inVal = fc.in0Ptr ? *fc.in0Ptr : 0.0;
                double dt = (config.stepSize > 0.0) ? config.stepSize : 1e-5;
                if (currentTime == 0.0) {
                    fc.prevOut = inVal;
                    val = inVal;
                } else {
                    double rate = (inVal - fc.prevOut) / dt;
                    double clampedRate = std::max(fc.rateDown, std::min(fc.rateUp, rate));
                    val = fc.prevOut + clampedRate * dt;
                    if (currentTime > fc.lastTime) {
                        fc.prevOut = val;
                        fc.lastTime = currentTime;
                    }
                }
            }
            else if (fc.type == ComponentType::Relay) {
                double inVal = fc.in0Ptr ? *fc.in0Ptr : 0.0;
                if (inVal >= fc.onThresh) fc.relayState = 1;
                else if (inVal <= fc.offThresh) fc.relayState = 0;
                val = (double)fc.relayState;
            }
            else if (fc.type == ComponentType::Comparator) {
                double inA = fc.in0Ptr ? *fc.in0Ptr : 0.0;
                double inB = fc.in1Ptr ? *fc.in1Ptr : 0.0;
                val = (inA > inB) ? 1.0 : 0.0;
            }
            else if (fc.type == ComponentType::LogicOp) {
                double in1 = fc.in0Ptr ? *fc.in0Ptr : 0.0;
                double in2 = fc.in1Ptr ? *fc.in1Ptr : 0.0;
                std::string op = fc.polarity;
                bool a = (in1 > 0.5), b = (in2 > 0.5);
                if (op == "AND") val = (a && b) ? 1.0 : 0.0;
                else if (op == "OR") val = (a || b) ? 1.0 : 0.0;
                else if (op == "XOR") val = (a != b) ? 1.0 : 0.0;
                else if (op == "NAND") val = !(a && b) ? 1.0 : 0.0;
                else if (op == "NOR") val = !(a || b) ? 1.0 : 0.0;
                else if (op == "NXOR" || op == "XNOR") val = (a == b) ? 1.0 : 0.0;
                else if (op == "NOT") val = (!a) ? 1.0 : 0.0;
                else val = 0.0;
            }
            else if (fc.type == ComponentType::BitwiseOp) {
                int in1 = (int)(fc.in0Ptr ? *fc.in0Ptr : 0.0);
                int in2 = (int)(fc.in1Ptr ? *fc.in1Ptr : 0.0);
                std::string op = fc.polarity;
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
                std::string op = fc.polarity;
                if (op == "==" || op == "=") val = (std::abs(in1 - in2) < 1e-12) ? 1.0 : 0.0;
                else if (op == "!=" || op == "~=") val = (std::abs(in1 - in2) >= 1e-12) ? 1.0 : 0.0;
                else if (op == "<") val = (in1 < in2) ? 1.0 : 0.0;
                else if (op == "<=") val = (in1 <= in2) ? 1.0 : 0.0;
                else if (op == ">") val = (in1 > in2) ? 1.0 : 0.0;
                else if (op == ">=") val = (in1 >= in2) ? 1.0 : 0.0;
                else val = 0.0;
            }
            else if (fc.type == ComponentType::CompareToConstant) {
                double inVal = fc.in0Ptr ? *fc.in0Ptr : 0.0;
                double cVal = fc.thresholdVal;
                std::string op = fc.polarity;
                if (op == "==" || op == "=") val = (std::abs(inVal - cVal) < 1e-12) ? 1.0 : 0.0;
                else if (op == "!=" || op == "~=") val = (std::abs(inVal - cVal) >= 1e-12) ? 1.0 : 0.0;
                else if (op == "<") val = (inVal < cVal) ? 1.0 : 0.0;
                else if (op == "<=") val = (inVal <= cVal) ? 1.0 : 0.0;
                else if (op == ">") val = (inVal > cVal) ? 1.0 : 0.0;
                else if (op == ">=") val = (inVal >= cVal) ? 1.0 : 0.0;
                else val = 0.0;
            }
            else if (fc.type == ComponentType::EdgeDetect) {
                double inVal = fc.in0Ptr ? *fc.in0Ptr : 0.0;
                double pulseW = (fc.pulseDuration > 0.0) ? fc.pulseDuration : 1e-3;
                bool detected = false;
                if (fc.edgeMode == "rising") detected = (fc.prevVal <= 0.5 && inVal > 0.5);
                else if (fc.edgeMode == "falling") detected = (fc.prevVal > 0.5 && inVal <= 0.5);
                else detected = ((fc.prevVal <= 0.5 && inVal > 0.5) || (fc.prevVal > 0.5 && inVal <= 0.5));
                if (detected && !fc.edgeActive) {
                    fc.edgeActive = true;
                    fc.triggerTime = currentTime;
                }
                if (fc.edgeActive && fc.triggerTime >= 0.0 && (currentTime - fc.triggerTime) >= pulseW - 1e-12) {
                    fc.edgeActive = false;
                }
                if (currentTime > fc.lastTime) {
                    fc.prevVal = inVal;
                    fc.lastTime = currentTime;
                }
                val = fc.edgeActive ? 1.0 : 0.0;
            }
            else if (fc.type == ComponentType::Monostable || fc.type == ComponentType::Monoflop) {
                double inVal = fc.in0Ptr ? *fc.in0Ptr : 0.0;
                double dur = (fc.pulseDuration > 0.0) ? fc.pulseDuration : 0.1;
                bool detected = false;
                if (fc.edgeMode == "rising") detected = (fc.prevVal <= 0.5 && inVal > 0.5);
                else if (fc.edgeMode == "falling") detected = (fc.prevVal > 0.5 && inVal <= 0.5);
                else detected = ((fc.prevVal <= 0.5 && inVal > 0.5) || (fc.prevVal > 0.5 && inVal <= 0.5));
                if (detected) {
                    if (!fc.edgeActive || fc.retriggerable) {
                        fc.edgeActive = true;
                        fc.triggerTime = currentTime;
                    }
                }
                if (fc.edgeActive && fc.triggerTime >= 0.0 && (currentTime - fc.triggerTime) >= dur - 1e-11) {
                    fc.edgeActive = false;
                }
                if (currentTime > fc.lastTime) {
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
                if (edgeDetected) {
                    fc.q_state = (dVal > 0.5) ? 1.0 : 0.0;
                }
                if (currentTime > fc.lastTime) {
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
                if (edgeDetected) {
                    bool J = (jVal > 0.5), K = (kVal > 0.5);
                    if (J && K) fc.q_state = (fc.q_state > 0.5) ? 0.0 : 1.0; // Toggle
                    else if (J) fc.q_state = 1.0;
                    else if (K) fc.q_state = 0.0;
                    // else hold
                }
                if (currentTime > fc.lastTime) {
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
                if (edgeDetected && currentTime > fc.lastTime) {
                    // Shift right, push new input at front
                    for (int i = (int)fc.shiftBuffer.size() - 1; i > 0; --i) {
                        fc.shiftBuffer[i] = fc.shiftBuffer[i - 1];
                    }
                    fc.shiftBuffer[0] = inVal;
                    fc.prev_clk = clkVal;
                    fc.lastTime = currentTime;
                } else if (currentTime > fc.lastTime) {
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
                    std::string tt = fc.polarity;
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

                if (fc.outputSigIndices.size() > 0 && fc.outputSigIndices[0] >= 0 && fc.outputSigIndices[0] < (int)flatControlSignals.size()) flatControlSignals[fc.outputSigIndices[0]] = alpha;
                if (fc.outputSigIndices.size() > 1 && fc.outputSigIndices[1] >= 0 && fc.outputSigIndices[1] < (int)flatControlSignals.size()) flatControlSignals[fc.outputSigIndices[1]] = beta;
                val = alpha;
            }
            else if (fc.type == ComponentType::InvClarke) {
                double alpha = (fc.inputSigIndices.size() > 0 && fc.inputSigIndices[0] >= 0 && fc.inputSigIndices[0] < (int)flatControlSignals.size()) ? flatControlSignals[fc.inputSigIndices[0]] : (fc.in0Ptr ? *fc.in0Ptr : 0.0);
                double beta = (fc.inputSigIndices.size() > 1 && fc.inputSigIndices[1] >= 0 && fc.inputSigIndices[1] < (int)flatControlSignals.size()) ? flatControlSignals[fc.inputSigIndices[1]] : (fc.in1Ptr ? *fc.in1Ptr : 0.0);

                double va = alpha;
                double vb = -0.5 * alpha + (1.7320508075688772 / 2.0) * beta;
                double vc = -0.5 * alpha - (1.7320508075688772 / 2.0) * beta;

                if (fc.outputSigIndices.size() > 0 && fc.outputSigIndices[0] >= 0 && fc.outputSigIndices[0] < (int)flatControlSignals.size()) flatControlSignals[fc.outputSigIndices[0]] = va;
                if (fc.outputSigIndices.size() > 1 && fc.outputSigIndices[1] >= 0 && fc.outputSigIndices[1] < (int)flatControlSignals.size()) flatControlSignals[fc.outputSigIndices[1]] = vb;
                if (fc.outputSigIndices.size() > 2 && fc.outputSigIndices[2] >= 0 && fc.outputSigIndices[2] < (int)flatControlSignals.size()) flatControlSignals[fc.outputSigIndices[2]] = vc;
                val = va;
            }
            else if (fc.type == ComponentType::Park) {
                double alpha = (fc.inputSigIndices.size() > 0 && fc.inputSigIndices[0] >= 0 && fc.inputSigIndices[0] < (int)flatControlSignals.size()) ? flatControlSignals[fc.inputSigIndices[0]] : (fc.in0Ptr ? *fc.in0Ptr : 0.0);
                double beta = (fc.inputSigIndices.size() > 1 && fc.inputSigIndices[1] >= 0 && fc.inputSigIndices[1] < (int)flatControlSignals.size()) ? flatControlSignals[fc.inputSigIndices[1]] : (fc.in1Ptr ? *fc.in1Ptr : 0.0);
                double theta = (fc.inputSigIndices.size() > 2 && fc.inputSigIndices[2] >= 0 && fc.inputSigIndices[2] < (int)flatControlSignals.size()) ? flatControlSignals[fc.inputSigIndices[2]] : 0.0;

                double cosT = std::cos(theta);
                double sinT = std::sin(theta);

                double vd = alpha * cosT + beta * sinT;
                double vq = -alpha * sinT + beta * cosT;

                if (fc.outputSigIndices.size() > 0 && fc.outputSigIndices[0] >= 0 && fc.outputSigIndices[0] < (int)flatControlSignals.size()) flatControlSignals[fc.outputSigIndices[0]] = vd;
                if (fc.outputSigIndices.size() > 1 && fc.outputSigIndices[1] >= 0 && fc.outputSigIndices[1] < (int)flatControlSignals.size()) flatControlSignals[fc.outputSigIndices[1]] = vq;
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

                if (fc.outputSigIndices.size() > 0 && fc.outputSigIndices[0] >= 0 && fc.outputSigIndices[0] < (int)flatControlSignals.size()) flatControlSignals[fc.outputSigIndices[0]] = alpha;
                if (fc.outputSigIndices.size() > 1 && fc.outputSigIndices[1] >= 0 && fc.outputSigIndices[1] < (int)flatControlSignals.size()) flatControlSignals[fc.outputSigIndices[1]] = beta;
                val = alpha;
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

                if (fc.outputSigIndices.size() > 0 && fc.outputSigIndices[0] >= 0 && fc.outputSigIndices[0] < (int)flatControlSignals.size()) flatControlSignals[fc.outputSigIndices[0]] = outA;
                if (fc.outputSigIndices.size() > 1 && fc.outputSigIndices[1] >= 0 && fc.outputSigIndices[1] < (int)flatControlSignals.size()) flatControlSignals[fc.outputSigIndices[1]] = outB;
                if (fc.outputSigIndices.size() > 2 && fc.outputSigIndices[2] >= 0 && fc.outputSigIndices[2] < (int)flatControlSignals.size()) flatControlSignals[fc.outputSigIndices[2]] = outC;
                val = outA;
            }
            else if (fc.type == ComponentType::SVPWM) {
                double freq = (fc.freq > 0.0) ? fc.freq : 10000.0;
                double period = 1.0 / freq;
                double phaseIn = std::fmod(currentTime, period) / period;
                if (phaseIn < 0.0) phaseIn += 1.0;
                double v_car = (phaseIn < 0.5) ? (4.0 * phaseIn - 1.0) : (3.0 - 4.0 * phaseIn); // -1 to +1 triangle wave

                double va = (fc.inputSigIndices.size() > 0 && fc.inputSigIndices[0] >= 0 && fc.inputSigIndices[0] < (int)flatControlSignals.size()) ? flatControlSignals[fc.inputSigIndices[0]] : (fc.in0Ptr ? *fc.in0Ptr : 0.0);
                double vb = (fc.inputSigIndices.size() > 1 && fc.inputSigIndices[1] >= 0 && fc.inputSigIndices[1] < (int)flatControlSignals.size()) ? flatControlSignals[fc.inputSigIndices[1]] : (fc.in1Ptr ? *fc.in1Ptr : 0.0);
                double vc = (fc.inputSigIndices.size() > 2 && fc.inputSigIndices[2] >= 0 && fc.inputSigIndices[2] < (int)flatControlSignals.size()) ? flatControlSignals[fc.inputSigIndices[2]] : 0.0;

                // Zero-sequence min-max offset injection (SVPWM equivalence)
                double v_max = std::max(va, std::max(vb, vc));
                double v_min = std::min(va, std::min(vb, vc));
                double v_offset = -0.5 * (v_max + v_min);

                double v_refA = va + v_offset;
                double v_refB = vb + v_offset;
                double v_refC = vc + v_offset;

                double outA = (v_refA > v_car) ? 1.0 : 0.0;
                double outB = (v_refB > v_car) ? 1.0 : 0.0;
                double outC = (v_refC > v_car) ? 1.0 : 0.0;

                if (fc.outputSigIndices.size() > 0 && fc.outputSigIndices[0] >= 0 && fc.outputSigIndices[0] < (int)flatControlSignals.size()) flatControlSignals[fc.outputSigIndices[0]] = outA;
                if (fc.outputSigIndices.size() > 1 && fc.outputSigIndices[1] >= 0 && fc.outputSigIndices[1] < (int)flatControlSignals.size()) flatControlSignals[fc.outputSigIndices[1]] = outB;
                if (fc.outputSigIndices.size() > 2 && fc.outputSigIndices[2] >= 0 && fc.outputSigIndices[2] < (int)flatControlSignals.size()) flatControlSignals[fc.outputSigIndices[2]] = outC;
                val = outA;
            }
            else if (fc.type == ComponentType::PerAvg) {
                double inVal = fc.in0Ptr ? *fc.in0Ptr : 0.0;
                double period = (fc.delayDuration > 0.0) ? fc.delayDuration : 0.02;
                if (currentTime > fc.lastTime) {
                    fc.shiftBuffer.push_back(inVal);
                    if ((int)fc.shiftBuffer.size() > 1000) fc.shiftBuffer.erase(fc.shiftBuffer.begin());
                    fc.lastTime = currentTime;
                }
                double sum = 0.0;
                for (double v : fc.shiftBuffer) sum += v;
                val = fc.shiftBuffer.empty() ? inVal : (sum / fc.shiftBuffer.size());
            }
            else if (fc.type == ComponentType::PeriodicImpAvg) {
                double inVal = fc.in0Ptr ? *fc.in0Ptr : 0.0;
                double trigVal = fc.ctrlSigPtr ? *fc.ctrlSigPtr : 0.0;
                bool isRising = (fc.prev_clk <= 0.5 && trigVal > 0.5);
                double dt = (config.stepSize > 0.0) ? config.stepSize : 1e-5;
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
                if (currentTime > fc.lastTime) {
                    fc.prev_clk = trigVal;
                    fc.lastTime = currentTime;
                }
                val = fc.prevOut;
            }
            else if (fc.type == ComponentType::FourierTrans || fc.type == ComponentType::FourierAnalysis) {
                double inVal = fc.in0Ptr ? *fc.in0Ptr : 0.0;
                double fn = (fc.freq > 0.0) ? fc.freq : 50.0;
                double harmonic = (fc.shiftLength > 0) ? (double)fc.shiftLength : 1.0;
                double ts = (fc.delay > 0.0) ? fc.delay : (config.stepSize > 0.0 ? config.stepSize : 1e-4);
                int N = (int)std::round(1.0 / (fn * ts));
                if (N < 2) N = 2;

                if (currentTime > fc.lastTime) {
                    fc.shiftBuffer.push_back(inVal);
                    if ((int)fc.shiftBuffer.size() > N) fc.shiftBuffer.erase(fc.shiftBuffer.begin());
                    fc.lastTime = currentTime;
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
            else if (fc.type == ComponentType::MovAvg) {
                double inVal = fc.in0Ptr ? *fc.in0Ptr : 0.0;
                int win = (fc.shiftLength > 0) ? fc.shiftLength : 10;
                if (currentTime > fc.lastTime) {
                    fc.shiftBuffer.push_back(inVal);
                    if ((int)fc.shiftBuffer.size() > win) fc.shiftBuffer.erase(fc.shiftBuffer.begin());
                    fc.lastTime = currentTime;
                }
                double sum = 0.0;
                for (double v : fc.shiftBuffer) sum += v;
                val = fc.shiftBuffer.empty() ? inVal : (sum / fc.shiftBuffer.size());
            }
            else if (fc.type == ComponentType::Filter1st) {
                double inVal = fc.in0Ptr ? *fc.in0Ptr : 0.0;
                double fcHz = (fc.freq > 0.0) ? fc.freq : 1000.0;
                double tau = 1.0 / (2.0 * 3.141592653589793 * fcHz);
                double dt = (config.stepSize > 0.0) ? config.stepSize : 1e-5;
                if (currentTime == 0.0) fc.filterState = inVal;
                else fc.filterState = (tau / (tau + dt)) * fc.filterState + (dt / (tau + dt)) * inVal;
                val = fc.filterState;
            }
            else if (fc.type == ComponentType::Filter2nd) {
                double inVal = fc.in0Ptr ? *fc.in0Ptr : 0.0;
                double fcHz = (fc.freq > 0.0) ? fc.freq : 1000.0;
                double Q = (fc.gain > 0.0) ? fc.gain : 0.707;
                double w0 = 2.0 * 3.141592653589793 * fcHz;
                double dt = (config.stepSize > 0.0) ? config.stepSize : 1e-5;
                if (currentTime == 0.0) {
                    fc.stateVal = inVal; // y
                    fc.filterState = 0.0; // dy/dt
                } else {
                    double d2y = w0 * w0 * (inVal - fc.stateVal) - (w0 / Q) * fc.filterState;
                    fc.filterState += d2y * dt;
                    fc.stateVal += fc.filterState * dt;
                }
                val = fc.stateVal;
            }
            else if (fc.type == ComponentType::RmsVal) {
                double inVal = fc.in0Ptr ? *fc.in0Ptr : 0.0;
                double fn = (fc.freq > 0.0) ? fc.freq : 50.0;
                int N = (int)std::round(1.0 / (fn * (config.stepSize > 0 ? config.stepSize : 1e-4)));
                if (N < 2) N = 2;
                if (currentTime > fc.lastTime) {
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
                int N = (int)std::round(1.0 / (fn * (config.stepSize > 0 ? config.stepSize : 1e-4)));
                if (N < 2) N = 2;
                if (currentTime > fc.lastTime) {
                    fc.shiftBuffer.push_back(inVal);
                    if ((int)fc.shiftBuffer.size() > N) fc.shiftBuffer.erase(fc.shiftBuffer.begin());
                    fc.lastTime = currentTime;
                }
                double sumSq = 0.0;
                for (double v : fc.shiftBuffer) sumSq += v * v;
                double totalRms = std::sqrt(fc.shiftBuffer.empty() ? 0.0 : (sumSq / fc.shiftBuffer.size()));

                // Fundamental component amplitude (H1)
                double Re = 0.0, Im = 0.0;
                int bufSize = (int)fc.shiftBuffer.size();
                double omega = 2.0 * 3.141592653589793 / N;
                for (int i = 0; i < bufSize; ++i) {
                    Re += fc.shiftBuffer[i] * std::cos(omega * i);
                    Im += fc.shiftBuffer[i] * std::sin(omega * i);
                }
                Re = (2.0 / (bufSize > 0 ? bufSize : 1)) * Re;
                Im = (2.0 / (bufSize > 0 ? bufSize : 1)) * Im;
                double fundRms = std::sqrt(Re * Re + Im * Im) / 1.4142135623730951;

                if (fundRms < 1e-6) val = 0.0;
                else {
                    double harmonicSquare = totalRms * totalRms - fundRms * fundRms;
                    val = (harmonicSquare > 0.0) ? std::sqrt(harmonicSquare) / fundRms : 0.0;
                }
            }
            else if (fc.type == ComponentType::PllLoop) {
                double inVal = fc.in0Ptr ? *fc.in0Ptr : 0.0;
                double dt = (config.stepSize > 0.0) ? config.stepSize : 1e-5;
                double fn = (fc.freq > 0.0) ? fc.freq : 50.0;
                double w0 = 2.0 * 3.141592653589793 * fn;
                if (currentTime == 0.0) {
                    fc.stateVal = 0.0;     // theta
                    fc.filterState = w0;  // omega
                }
                // Phase detector error (inVal * cos(theta))
                double err = inVal * std::cos(fc.stateVal);
                double Kp = (fc.gain > 0.0) ? fc.gain : 20.0;
                double Ki = (fc.maxVal > 0.0) ? fc.maxVal : 1000.0;
                fc.filterState += Ki * err * dt;
                double omega_total = fc.filterState + Kp * err;
                fc.stateVal = std::fmod(fc.stateVal + omega_total * dt, 2.0 * 3.141592653589793);
                if (fc.stateVal < 0.0) fc.stateVal += 2.0 * 3.141592653589793;

                double theta = fc.stateVal;
                double freqEstimated = omega_total / (2.0 * 3.141592653589793);
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
                std::string dt = fc.polarity;
                if (dt == "boolean" || dt == "bool") val = (inVal > 0.5) ? 1.0 : 0.0;
                else if (dt == "integer" || dt == "int") val = std::round(inVal);
                else val = inVal;
            }
            else if (fc.type == ComponentType::StateMachine) {
                double inVal = fc.in0Ptr ? *fc.in0Ptr : 0.0;
                val = inVal;
            }
            else if (fc.type == ComponentType::SummingJunction) {
                double in0 = fc.in0Ptr ? *fc.in0Ptr : 0.0;
                double in1 = fc.in1Ptr ? *fc.in1Ptr : 0.0;
                std::string signs = fc.polarity.empty() ? "++" : fc.polarity;
                double s0 = (signs.size() > 0 && signs[0] == '-') ? -1.0 : 1.0;
                double s1 = (signs.size() > 1 && signs[1] == '-') ? -1.0 : 1.0;
                val = s0 * in0 + s1 * in1;
            }
            else if (fc.type == ComponentType::Product) {
                double in0 = fc.in0Ptr ? *fc.in0Ptr : 0.0;
                double in1 = fc.in1Ptr ? *fc.in1Ptr : 0.0;
                std::string ops = fc.polarity;
                if (ops == "*/") val = (std::abs(in1) < 1e-15) ? (in0 / 1e-15) : (in0 / in1);
                else val = in0 * in1;
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
                if (n1 >= 0) B[n1] += iEq;
                if (n2 >= 0) B[n2] -= iEq;
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

        if (fc.outputSigKeys.size() > 1) {
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
        while (statesChanged && pwlIter < 10) {
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

        // Final consistency solve if last update changed device states
        if (matrixKChanged) {
            assembleMNA(currentTime);
            if (totalDim > 0) {
                factorizeLU(totalDim);
                K_prev = K;
                matrixKChanged = false;
                solveLUSubstitution(totalDim);
            }
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
            if (!fc.outputSigKeys.empty() && fc.outputSigKeys.size() > 1) {
                for (size_t i = 0; i < fc.outputSigIndices.size(); ++i) {
                    int sigIdx = fc.outputSigIndices[i];
                    double ctrlVal = (sigIdx >= 0 && sigIdx < (int)flatControlSignals.size()) ? flatControlSignals[sigIdx] : 0.0;
                    if (i < fc.customScriptOutputVecPtrs.size() && fc.customScriptOutputVecPtrs[i]) {
                        fc.customScriptOutputVecPtrs[i]->push_back(ctrlVal);
                    }
                    if (i < fc.customScriptPlotVecPtrs.size() && fc.customScriptPlotVecPtrs[i]) {
                        fc.customScriptPlotVecPtrs[i]->push_back(ctrlVal);
                    }
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
