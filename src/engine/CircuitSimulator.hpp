#pragma once

#include <vector>
#include <unordered_map>
#include <string>
#include <memory>
#include <mutex>
#include <atomic>
#include "ExpressionEvaluator.hpp"
#include "CScriptEngine.hpp"

namespace CircuitSimEngine {

enum class ComponentType {
    Resistor,
    VariableResistor,
    Capacitor,
    Inductor,
    VoltageSource,
    ACVoltageSource,
    ThreePhaseSource,
    ControlledVoltageSource,
    CurrentSource,
    ACCurrentSource,
    ControlledCurrentSource,
    ThreePhaseCurrentSource,
    ElectricalPort,
    ElectricalLabel,
    Voltmeter3Ph,
    Ammeter3Ph,
    VariableInductor,
    VariableCapacitor,
    SaturableInductor,
    SaturableCapacitor,
    PiSectionLine,
    TransmissionLine3Ph,
    PWLResistor,
    ElectricalAlgebraic,
    Thyristor,
    IGBT,
    GTO,
    IGBTDiode,
    IGCT,
    BJT,
    JFET,
    // Electrical <-> magnetic gyrator interface. All other magnetic blocks reduce to
    // existing primitives (permeance -> capacitor, magnetic resistance -> resistor,
    // MMF source -> voltage source), so this is the only new solver element.
    Winding,
    Breaker,
    DoubleSwitch,
    ElectricalManualSwitch,
    ManualDoubleSwitch,
    ManualTripleSwitch,
    SRSwitch,
    TripleSwitch,
    IdealTransformer,
    Transformer2W,
    Transformer3W,
    MutualInductor2W,
    MutualInductor3W,
    SaturableTransformer,
    Transformer3Ph2W,
    Transformer3Ph3W,
    InductionMotor,
    OpAmp,
    EComp,
    GenEBlock,
    GotoSignal,
    FromSignal,
    VGFET,
    MOSFET,
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
    MasterPWM,
    Triangle_Carrier,
    PI_Controller,
    PulseGenerator,
    EdgeDetector,
    MathFunction,
    KeyTrigger,
    Comparator,
    AND_Gate,
    OR_Gate,
    NOT_Gate,
    CustomScript,
    Transformer,
    
    // Control Sources Detailed Library
    Clock,
    InitialCondition,
    Ramp,
    RandomNumbers,
    SineWave,
    Step,
    WhiteNoise,

    // Control Functions & Tables Detailed Library
    TrigFunction,
    Abs,
    Sign,
    Round,
    MinMax,
    Polynomial,
    AlgebraicConstraint,
    LUT_1D,
    LUT_2D,
    LUT_3D,
    DLL,
    FMU,
    FourierSeries,

    // Ports and Subsystems
    Subsystem,
    Inport,
    Outport,
    PhysicalInport,
    PhysicalOutport,
    EnablePort,
    TriggerPort,
    BusCreator,
    BusSelector,
    Terminator,

    // Control Continuous Detailed Library
    Integrator,
    Derivative,
    TransferFunction,
    StateSpace,
    ContinuousPID,
    PLL_1PH,
    PLL_3PH,

    // Control Delays Detailed Library
    Delay,
    TransportDelay,
    TurnOnDelay,
    MemoryBlock,

    // Control Discontinuous Detailed Library
    Quantizer,
    SignalSwitch,
    ManualSwitch,
    MultiportSwitch,
    HitCrossing,
    Saturation,
    DeadZone,
    RateLimiter,
    Relay,

    // Control Logical & Bitwise Detailed Library
    LogicOp,
    BitwiseOp,
    CombLogic,
    EdgeDetect,
    Monostable,
    Monoflop,
    RelationalOp,
    CompareToConstant,
    DFlipFlop,
    JKFlipFlop,
    ShiftReg,

    // Control Modulators & Signal Transforms
    PWM_MASTER,
    PWM_3PH,
    SVPWM,
    Clarke,
    Park,
    InvClarke,
    InvPark,
    DqToAbc,
    AbcToDq,

    // Control Filters & Measurements
    PerAvg,
    PeriodicImpAvg,
    FourierTrans,
    MovAvg,
    Filter1st,
    Filter2nd,
    FourierAnalysis,
    RmsVal,
    ThdVal,
    PllLoop,

    // Control State Machines & Math
    Offset,
    Signum,
    Divide,
    DataTypeConv,
    StateMachine,

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

    // Signal-controlled passive elements (VAR_R / VAR_L / VAR_C). When isVariable is
    // set and a control signal is bound, the element value is taken from that signal
    // each timestep instead of the fixed nominal parameter.
    bool isVariable = false;
    double nominalVal = 0.0;
    double Vvd = 0.7;
    double Iholding = 0.01;
    double Vgt = 0.5;
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
    std::string vAlphaKey;
    std::string vBetaKey;

    struct TimePoint { double t; double val; };
    std::vector<TimePoint> delayHistory;
    double delayDuration = 0.1;
    double highStartTime = -1.0;
    bool prevInputHigh = false;
    double prevVal = 0.0;
    double currentVal = 0.0;
    double lastTime = -1.0;

    double prevOut = 0.0;
    double lastHit = 0.0;
    double thresholdVal = 0.5;
    double onThresh = 1.0;
    double offThresh = -1.0;
    double outValOn = 1.0;
    double outValOff = 0.0;
    double rateUp = 10.0;
    double rateDown = -10.0;
    int relayState = 0;

    // Logical & Bitwise state
    double q_state = 0.0;        // Flip-flop Q output state
    double prev_clk = 0.0;       // Previous clock for edge detection
    bool edgeActive = false;     // Active pulse output for EDGE_DETECT / MONOSTABLE / MONOFLOP
    double triggerTime = -1.0;   // Time pulse was triggered
    double pulseDuration = 0.1;  // Duration of pulse output
    std::vector<double> polyCoeffs;
    bool retriggerable = false;  // Whether monoflop is retriggerable
    std::string edgeMode;        // "rising", "falling", "either"
    std::vector<double> shiftBuffer;  // Buffer for SHIFT_REG
    int numInputs = 2;           // Number of inputs for multi-input logic blocks
    int shiftLength = 4;         // Shift register length

    // PWM_MASTER state fields
    std::vector<int> pwmMasterInIndices;
    std::vector<int> pwmMasterExtPhaseIndices;
    std::vector<int> pwmMasterOutDirectIndices;
    std::vector<int> pwmMasterOutComplIndices;
    std::vector<double> pwmMasterPhaseDeg;
    std::vector<double> pwmMasterLevelOffset;
    std::vector<bool> pwmMasterPhaseExt;
    std::vector<int> pwmMasterLastTargetDirect;
    std::vector<int> pwmMasterLastTargetCompl;
    std::vector<double> pwmMasterLastTransDirect;
    std::vector<double> pwmMasterLastTransCompl;
    std::vector<double> pwmMasterDirectOut;
    std::vector<double> pwmMasterComplOut;

    double stateVal = 0.0;
    double nextStateVal = 0.0;
    double filterState = 0.0;
    std::vector<double> stateVector;

    int stateIdx = -1;
    int in0SignalIdx = -1;
    int in1SignalIdx = -1;
    int outSignalIdx = -1;
    int compSelfSignalIdx = -1;
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

    std::atomic<uint64_t> telemetryVersion{0};
    std::atomic<double> computeTimeSeconds{0.0};

    uint64_t getTelemetryVersion() const {
        return telemetryVersion.load(std::memory_order_relaxed);
    }

    double getStopTime() const {
        return (config.stopTime > 0.0) ? config.stopTime : 0.01;
    }

    double getProgressPercent() {
        double curTime = getCurrentTime();
        double stopT = getStopTime();
        if (stopT <= 0.0) return 100.0;
        double pct = (curTime / stopT) * 100.0;
        return (pct > 100.0) ? 100.0 : ((pct < 0.0) ? 0.0 : pct);
    }

    double getComputeTimeSeconds() const {
        return computeTimeSeconds.load(std::memory_order_relaxed);
    }

    void setComputeTimeSeconds(double s) {
        computeTimeSeconds.store(s, std::memory_order_relaxed);
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
        telemetryVersion.fetch_add(1, std::memory_order_relaxed);
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
