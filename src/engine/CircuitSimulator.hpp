#pragma once

#include <vector>
#include <unordered_map>
#include <string>
#include <memory>
#include <mutex>
#include "ExpressionEvaluator.hpp"
#include "CScriptEngine.hpp"

namespace CircuitSimEngine {

enum class ComponentType {
    Resistor,
    Capacitor,
    Inductor,
    VoltageSource,
    ACVoltageSource,
    CurrentSource,
    Diode,
    Switch,
    Voltmeter,
    Ammeter,
    UnifiedProbe,
    Oscilloscope,
    Constant,
    Gain,
    SummingJunction,
    Product,
    PWM_Generator,
    Triangle_Carrier,
    PI_Controller,
    PulseGenerator,
    Comparator,
    AND_Gate,
    OR_Gate,
    NOT_Gate,
    CustomScript,
    Transformer,
    Unknown
};

struct ComponentModel {
    std::string id;
    ComponentType type;
    std::string label;
    std::vector<std::string> nodes;
    std::unordered_map<std::string, std::string> parameters;
};

struct FastCompiledComponent {
    std::string id;
    ComponentType type = ComponentType::Unknown;
    int n1 = -1;
    int n2 = -1;
    int n3 = -1;
    int n4 = -1;
    int vIdx = -1;
    int lIdx = -1;
    int wIdx0 = -1;
    int wIdx1 = -1;

    struct WindingInfo {
        int n1 = -1;
        int n2 = -1;
        int wIdx = -1;
        double turns = 100.0;
    };
    std::vector<WindingInfo> windings;

    double val = 0.0;
    double esr = 0.0;
    double Ron = 0.01;
    double Roff = 1e6;
    double Vvd = 0.7;
    double freq = 50.0;
    double turns1 = 100.0;
    double turns2 = 100.0;
    double minVal = 0.0;
    double maxVal = 1.0;
    double gain = 1.0;
    double Kp = 1.0;
    double Ki = 0.0;
    double period = 0.0001;
    double width = 0.5;
    double delay = 0.0;
    double amplitude = 1.0;

    std::string vPlotKey;
    std::string iPlotKey;
    std::string ctrlSigKey;
    std::string in0Key;
    std::string in1Key;
    std::string outKey;
    std::string targetKey;
    std::string polarity;

    int stateIdx = -1;
    int in0SignalIdx = -1;
    int in1SignalIdx = -1;
    int outSignalIdx = -1;
    int ctrlSigSignalIdx = -1;
    int targetSignalIdx = -1;
    int vPlotSignalIdx = -1;
    int iPlotSignalIdx = -1;

    std::vector<std::string> inputSigKeys;
    std::vector<std::string> outputSigKeys;
    std::vector<std::string> customPlotVarKeys;
    std::vector<std::string> customScriptVarNames;

    std::vector<int> inputSigIndices;
    std::vector<int> outputSigIndices;
    std::vector<int> customPlotVarIndices;

    std::vector<double>* vPlotVecPtr = nullptr;
    std::vector<double>* iPlotVecPtr = nullptr;
    std::vector<double>* vmVecPtr = nullptr;
    std::vector<double>* amVecPtr = nullptr;
    std::vector<double>* sigVecPtr = nullptr;
    std::vector<double>* sigOutVecPtr = nullptr;

    std::vector<std::vector<double>*> customScriptOutputVecPtrs;
    std::vector<std::vector<double>*> customScriptPlotVecPtrs;

    const double* in0Ptr = nullptr;
    const double* in1Ptr = nullptr;
    double* outPtr = nullptr;
    const double* ctrlSigPtr = nullptr;
    const double* targetPtr = nullptr;
};

struct SimulationConfig {
    double stopTime = 0.01;   // Default 10ms
    double stepSize = 1e-6;   // Default 1us
    std::string solver = "euler";
    std::string solverMethod = "non-ideal";
    std::string step_type = "fixed";
};

struct SimulationOutput {
    std::vector<double> time;
    std::unordered_map<std::string, std::vector<double>> voltages;
    std::unordered_map<std::string, std::vector<double>> inductors;
    std::unordered_map<std::string, std::vector<double>> voltmeters;
    std::unordered_map<std::string, std::vector<double>> ammeters;
    std::unordered_map<std::string, std::vector<double>> signals;
    std::unordered_map<std::string, std::vector<double>> custom_plots;
};

struct TelemetryData {
    std::vector<double> timeHistory;
    std::unordered_map<std::string, std::vector<double>> voltages;
};

class CircuitSimulator {
private:
    std::vector<ComponentModel> components;
    std::vector<ComponentModel> controlBlocks;
    SimulationConfig config;

    std::vector<FastCompiledComponent> fastPhysComps;
    std::vector<FastCompiledComponent> fastCtrlComps;

    int numNodes = 0;
    int totalDim = 0;
    std::unordered_map<std::string, int> nodeToIdx;
    std::unordered_map<std::string, int> vSourceToIdx;
    std::unordered_map<std::string, int> inductorToIdx;

    std::vector<double> K;
    std::vector<double> K_static;
    std::vector<double> B;
    std::vector<double> X;

    bool matrixKChanged = true;
    int forceBackwardEulerSteps = 0;
    std::vector<double> K_prev;

    std::vector<double> LU_buf;
    std::vector<double> LU_cached;
    std::vector<double> x_buf;
    std::vector<int> p_buf;
    std::vector<int> p_cached;

    std::vector<double> scriptInValsBuf;

    std::vector<double> flatCapVoltages;
    std::vector<double> flatIndCurrents;
    std::vector<double> flatIndVoltages;
    std::vector<double> flatDiodeStates;
    std::vector<double> flatSwitchStates;
    std::vector<double> flatPiIntegratorState;
    std::vector<double> flatControlSignals;
    std::unordered_map<std::string, int> signalKeyToIdx;

    std::unordered_map<std::string, double> capVoltagesPrev;
    std::unordered_map<std::string, double> indCurrentsPrev;
    std::unordered_map<std::string, double> diodeStatePrev;
    std::unordered_map<std::string, double> switchStatePrev;
    std::unordered_map<std::string, double> piIntegratorState;
    std::unordered_map<std::string, double> controlSignalsCurrent;
    std::unordered_map<std::string, CScriptEngine> cscriptEngines;

    struct NodeOutputBinding {
        int nodeIdx = -1;
        std::vector<double>* vecPtr = nullptr;
    };
    std::vector<NodeOutputBinding> nodeOutputBindings;

    TelemetryData telemetry;
    std::mutex telemetryMutex;

    void buildIndexMaps();
    void evaluateControls(double currentTime);
    void assembleMNA(double currentTime);
    bool updateDeviceStates();
    bool factorizeLU(int n);
    bool solveLUSubstitution(int n);
    bool solveLUFast(int n);
    double evaluateParam(const ComponentModel& comp, const std::string& key, double defaultVal);

public:
    CircuitSimulator() = default;
    
    void setup(const std::vector<ComponentModel>& physComps, 
               const std::vector<ComponentModel>& ctrlComps, 
               const SimulationConfig& simCfg);
               
    SimulationOutput runTransient();

    template <typename T>
    void loadCircuit(const T& cd) {
        std::lock_guard<std::mutex> lock(telemetryMutex);
        telemetry.timeHistory.clear();
        telemetry.voltages.clear();
    }

    void setTelemetryOutput(const SimulationOutput& out) {
        std::lock_guard<std::mutex> lock(telemetryMutex);
        telemetry.timeHistory = out.time;
        telemetry.voltages = out.voltages;
        for (const auto& pair : out.signals) telemetry.voltages[pair.first] = pair.second;
        for (const auto& pair : out.inductors) telemetry.voltages[pair.first] = pair.second;
        for (const auto& pair : out.voltmeters) telemetry.voltages[pair.first] = pair.second;
        for (const auto& pair : out.ammeters) telemetry.voltages[pair.first] = pair.second;
        for (const auto& pair : out.custom_plots) telemetry.voltages[pair.first] = pair.second;
    }

    TelemetryData getTelemetryCopy() {
        std::lock_guard<std::mutex> lock(telemetryMutex);
        return telemetry;
    }

    double getCurrentTime() {
        std::lock_guard<std::mutex> lock(telemetryMutex);
        return telemetry.timeHistory.empty() ? 0.0 : telemetry.timeHistory.back();
    }

    void pause() {}
    void reset() {
        std::lock_guard<std::mutex> lock(telemetryMutex);
        telemetry.timeHistory.clear();
        telemetry.voltages.clear();
    }
};

} // namespace CircuitSimEngine
