#include "CircuitSimulator.hpp"
#include <iostream>
#include <cmath>
#include <algorithm>

namespace CircuitSim {

void CircuitSimulator::loadCircuit(const CircuitDesign& circuit) {
    stop();
    design = circuit;
    settings = circuit.settings;
    reset();
}

void CircuitSimulator::reset() {
    std::lock_guard<std::mutex> lock(telemetryMutex);
    currentTime = 0.0;
    
    nodeToIdx.clear();
    vSourceToIdx.clear();
    inductorToIdx.clear();
    
    int nIdx = 1; // Node 0 is Ground
    for (const auto& comp : design.components) {
        for (const auto& n : comp.nodes) {
            if (n != "0" && n != "node_0" && !n.empty() && nodeToIdx.find(n) == nodeToIdx.end()) {
                nodeToIdx[n] = nIdx++;
            }
        }
        if (comp.type == ComponentType::VoltageSource || comp.type == ComponentType::ACVoltageSource || comp.type == ComponentType::ControlledVoltageSource) {
            if (vSourceToIdx.find(comp.id) == vSourceToIdx.end()) {
                vSourceToIdx[comp.id] = (int)vSourceToIdx.size();
            }
        } else if (comp.type == ComponentType::Inductor) {
            if (inductorToIdx.find(comp.id) == inductorToIdx.end()) {
                inductorToIdx[comp.id] = (int)inductorToIdx.size();
            }
        } else if (comp.type == ComponentType::CustomScript) {
            auto it = comp.parameters.find("code");
            std::string code = (it != comp.parameters.end()) ? it->second : "";
            scriptEngines[comp.id].setScript(code);
        }
    }
    
    numNodes = nIdx - 1;
    numVoltageSources = (int)vSourceToIdx.size();
    numInductors = (int)inductorToIdx.size();
    totalDim = numNodes + numVoltageSources + numInductors;
    
    M.assign(totalDim * totalDim, 0.0);
    K.assign(totalDim * totalDim, 0.0);
    B.assign(totalDim, 0.0);
    X.assign(totalDim, 0.0);
    
    telemetry.timeHistory.clear();
    telemetry.voltages.clear();
    telemetry.currents.clear();
    telemetry.channelSignals.clear();
}

bool CircuitSimulator::solveLU(int n, const std::vector<double>& A, const std::vector<double>& b, std::vector<double>& x) {
    if (n <= 0) return true;
    std::vector<double> LU = A;
    x = b;
    std::vector<int> p(n);
    for (int i = 0; i < n; i++) p[i] = i;

    for (int i = 0; i < n; i++) {
        double maxA = 0.0;
        int maxRow = i;
        for (int k = i; k < n; k++) {
            double absA = std::fabs(LU[k * n + i]);
            if (absA > maxA) {
                maxA = absA;
                maxRow = k;
            }
        }
        if (maxA < 1e-14) return false; // Singular matrix

        if (maxRow != i) {
            std::swap(p[i], p[maxRow]);
            for (int k = 0; k < n; k++) {
                std::swap(LU[i * n + k], LU[maxRow * n + k]);
            }
            std::swap(x[i], x[maxRow]);
        }

        for (int j = i + 1; j < n; j++) {
            LU[j * n + i] /= LU[i * n + i];
            for (int k = i + 1; k < n; k++) {
                LU[j * n + k] -= LU[j * n + i] * LU[i * n + k];
            }
        }
    }

    // Forward substitution L*y = b
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < i; j++) {
            x[i] -= LU[i * n + j] * x[j];
        }
    }

    // Backward substitution U*x = y
    for (int i = n - 1; i >= 0; i--) {
        for (int j = i + 1; j < n; j++) {
            x[i] -= LU[i * n + j] * x[j];
        }
        x[i] /= LU[i * n + i];
    }

    return true;
}

void CircuitSimulator::buildMNAMatrix() {
    std::fill(K.begin(), K.end(), 0.0);
    std::fill(B.begin(), B.end(), 0.0);
    
    double h = settings.stepSize;
    if (h <= 0) h = 1e-5;

    for (const auto& comp : design.components) {
        int n1 = (comp.nodes.size() > 0 && nodeToIdx.count(comp.nodes[0])) ? nodeToIdx[comp.nodes[0]] - 1 : -1;
        int n2 = (comp.nodes.size() > 1 && nodeToIdx.count(comp.nodes[1])) ? nodeToIdx[comp.nodes[1]] - 1 : -1;

        if (comp.type == ComponentType::Resistor) {
            auto it = comp.parameters.find("value");
            double rVal = (it != comp.parameters.end()) ? ExpressionEvaluator::parseScientific(it->second) : 1000.0;
            if (rVal < 1e-6) rVal = 1e-6;
            double g = 1.0 / rVal;
            
            if (n1 >= 0) K[n1 * totalDim + n1] += g;
            if (n2 >= 0) K[n2 * totalDim + n2] += g;
            if (n1 >= 0 && n2 >= 0) {
                K[n1 * totalDim + n2] -= g;
                K[n2 * totalDim + n1] -= g;
            }
        } else if (comp.type == ComponentType::Capacitor) {
            auto it = comp.parameters.find("C");
            double cVal = (it != comp.parameters.end()) ? ExpressionEvaluator::parseScientific(it->second) : 1e-6;
            double gEq = cVal / h; // Companion model resistor
            
            if (n1 >= 0) K[n1 * totalDim + n1] += gEq;
            if (n2 >= 0) K[n2 * totalDim + n2] += gEq;
            if (n1 >= 0 && n2 >= 0) {
                K[n1 * totalDim + n2] -= gEq;
                K[n2 * totalDim + n1] -= gEq;
            }
        } else if (comp.type == ComponentType::VoltageSource) {
            auto it = comp.parameters.find("value");
            double vVal = (it != comp.parameters.end()) ? ExpressionEvaluator::parseScientific(it->second) : 12.0;
            int vIdx = numNodes + vSourceToIdx[comp.id];
            
            if (n1 >= 0) {
                K[n1 * totalDim + vIdx] += 1.0;
                K[vIdx * totalDim + n1] += 1.0;
            }
            if (n2 >= 0) {
                K[n2 * totalDim + vIdx] -= 1.0;
                K[vIdx * totalDim + n2] -= 1.0;
            }
            B[vIdx] = vVal;
        } else if (comp.type == ComponentType::ACVoltageSource) {
            auto itV = comp.parameters.find("value");
            auto itF = comp.parameters.find("freq");
            double amplitude = (itV != comp.parameters.end()) ? ExpressionEvaluator::parseScientific(itV->second) : 24.0;
            double freq = (itF != comp.parameters.end()) ? ExpressionEvaluator::parseScientific(itF->second) : 50.0;
            double vVal = amplitude * std::sin(2.0 * 3.141592653589793 * freq * currentTime);
            int vIdx = numNodes + vSourceToIdx[comp.id];
            
            if (n1 >= 0) {
                K[n1 * totalDim + vIdx] += 1.0;
                K[vIdx * totalDim + n1] += 1.0;
            }
            if (n2 >= 0) {
                K[n2 * totalDim + vIdx] -= 1.0;
                K[vIdx * totalDim + n2] -= 1.0;
            }
            B[vIdx] = vVal;
        }
    }
}

void CircuitSimulator::step() {
    buildMNAMatrix();
    
    // Step script engines
    for (auto& pair : scriptEngines) {
        std::vector<double> dummyIn = {0.0, 0.0};
        pair.second.executeStep(currentTime, settings.stepSize, dummyIn);
    }

    if (totalDim > 0) {
        solveLU(totalDim, K, B, X);
    }
    
    double t = currentTime;
    currentTime = t + settings.stepSize;

    // Log telemetry
    std::lock_guard<std::mutex> lock(telemetryMutex);
    telemetry.timeHistory.push_back(t);
    for (const auto& pair : nodeToIdx) {
        int idx = pair.second - 1;
        double v = (idx >= 0 && idx < totalDim) ? X[idx] : 0.0;
        telemetry.voltages[pair.first].push_back(v);
    }

    // Evaluate Control & Signal Generator Components (PULSE_GEN, PWM, CONST, etc.)
    for (const auto& comp : design.components) {
        std::string tName = comp.rawTypeStr;
        std::transform(tName.begin(), tName.end(), tName.begin(), ::toupper);

        if (tName == "PULSE" || tName == "PULSE_GEN" || comp.type == ComponentType::PulseGenerator) {
            auto itAmp = comp.parameters.find("amplitude");
            auto itPer = comp.parameters.find("period");
            auto itWid = comp.parameters.find("width");
            auto itDel = comp.parameters.find("delay");

            double amp = (itAmp != comp.parameters.end()) ? ExpressionEvaluator::parseScientific(itAmp->second) : 1.0;
            double period = (itPer != comp.parameters.end()) ? ExpressionEvaluator::parseScientific(itPer->second) : 0.001;
            double width = (itWid != comp.parameters.end()) ? ExpressionEvaluator::parseScientific(itWid->second) : 0.5;
            double delay = (itDel != comp.parameters.end()) ? ExpressionEvaluator::parseScientific(itDel->second) : 0.0;

            if (period <= 1e-12) period = 0.001;

            double tRel = t - delay;
            double outVal = 0.0;
            if (tRel >= 0.0) {
                double phase = std::fmod(tRel, period);
                if (phase < period * width) {
                    outVal = amp;
                }
            }

            telemetry.voltages[comp.id + ".Out"].push_back(outVal);
            telemetry.voltages["V_" + comp.id].push_back(outVal);
            telemetry.voltages[comp.id].push_back(outVal);
        } else if (tName == "PWM" || comp.type == ComponentType::PWM_Generator) {
            auto itFreq = comp.parameters.find("freq");
            double freq = (itFreq != comp.parameters.end()) ? ExpressionEvaluator::parseScientific(itFreq->second) : 10000.0;
            double period = (freq > 0.0) ? (1.0 / freq) : 0.0001;
            double phase = std::fmod(t, period);
            double triWave = (phase < period * 0.5) ? (4.0 * phase / period - 1.0) : (3.0 - 4.0 * phase / period);
            double duty = 0.5;
            double outVal = (duty > triWave) ? 1.0 : 0.0;

            telemetry.voltages[comp.id + ".Out"].push_back(outVal);
            telemetry.voltages["V_" + comp.id].push_back(outVal);
            telemetry.voltages[comp.id].push_back(outVal);
        } else if (tName == "CONST" || comp.type == ComponentType::Constant) {
            auto itV = comp.parameters.find("k");
            if (itV == comp.parameters.end()) itV = comp.parameters.find("value");
            double val = (itV != comp.parameters.end()) ? ExpressionEvaluator::parseScientific(itV->second) : 1.0;

            telemetry.voltages[comp.id + ".Out"].push_back(val);
            telemetry.voltages["V_" + comp.id].push_back(val);
            telemetry.voltages[comp.id].push_back(val);
        }
    }
    
    // Limit buffer length for smooth UI rendering
    if (telemetry.timeHistory.size() > 50000) {
        telemetry.timeHistory.erase(telemetry.timeHistory.begin(), telemetry.timeHistory.begin() + 10000);
        for (auto& pair : telemetry.voltages) {
            pair.second.erase(pair.second.begin(), pair.second.begin() + 10000);
        }
    }
}

void CircuitSimulator::runAsync() {
    if (isRunning) return;
    isRunning = true;
    isPaused = false;

    std::thread([this]() {
        while (isRunning) {
            if (!isPaused && currentTime < settings.stopTime) {
                step();
            } else {
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
            }
        }
    }).detach();
}

void CircuitSimulator::pause() { isPaused = true; }
void CircuitSimulator::resume() { isPaused = false; }
void CircuitSimulator::stop() {
    isRunning = false;
    isPaused = false;
}

TelemetryData CircuitSimulator::getTelemetryCopy() {
    std::lock_guard<std::mutex> lock(telemetryMutex);
    return telemetry;
}

} // namespace CircuitSim
