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

    // Last conductance/resistance this element contributed to K. Comparing against it
    // during assembly detects a changed matrix in O(#dynamic elements) instead of the
    // O(n^2) full-matrix comparison the solver used to perform every iteration.
    double gStamped = -1e300;
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
    // Reuse LU factorizations for MNA matrices that have been seen before. A
    // switching converter cycles through a small set of topologies, so this turns
    // most O(n^3) factorizations into an O(n^2) lookup. Results are unaffected: a
    // hit is only taken after the cached matrix is confirmed bitwise identical.
    bool enableLUCache = true;

    // ── Adaptive stepping (step_type == "variable" or "adaptive") ────────────
    // Local error is measured on the reactive states - capacitor voltages and
    // inductor currents - against `relTol * |state| + absTol`. Separate absolute
    // floors for voltage and current because a single one cannot be dimensionally
    // meaningful for both, and a floor that is too tight forces needlessly small
    // steps whenever a state passes through zero.
    double relTol = 1e-3;
    double absTolV = 1e-3;    // 1 mV
    double absTolI = 1e-6;    // 1 uA
    // Step bounds. Zero means "derive from stepSize": hMax = stepSize and
    // hMin = stepSize / 1e6, so `stepSize` keeps its meaning as the largest step
    // the user is willing to accept and an existing netlist cannot become coarser
    // than it is today just by switching step_type.
    double hMin = 0.0;
    double hMax = 0.0;

    // Decimation interval for output storage. If > 0, outputs are only stored
    // at this time interval, drastically reducing memory overhead on adaptive runs.
    double outputDecimation = 0.0;

    // Test hooks, not for production use. Each must leave results unchanged.
    //   debugRejectEveryNthStep - forces every Nth attempted step to be rejected once
    //     and retried at the same h, exercising the rollback path.
    //   debugExtraControlProbes - performs N extra non-committing control evaluations
    //     per step at scattered times, proving that probing really is side-effect free.
    // Zero disables either.
    int debugRejectEveryNthStep = 0;
    int debugExtraControlProbes = 0;
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
    std::vector<FastCompiledComponent*> fastGateCtrlComps;

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

    // ── Step rollback ────────────────────────────────────────────────────────
    // The adaptive controller has to be able to abandon a step, either because the
    // local error came out too large or because a switching event turned out to lie
    // inside it. Everything an electrical step advances therefore has to be
    // restorable.
    //
    // Control-block state is deliberately NOT part of this. evaluateControls() is
    // called exactly once per ACCEPTED step, so a rejected step never advances an
    // integrator, latch, flip-flop, delay history or script engine - which is what
    // avoids having to copy the ~40 mutable per-block fields and the script engines
    // on every attempt.
    //
    // The LU cache needs no rollback either: entries are keyed on the matrix
    // contents, so a factorization produced during an abandoned step is still a
    // valid factorization of that matrix. dynStampIdx/dynStampSeen only ever grow,
    // and every assembly restores those positions from K_static, so they are safe
    // to leave alone as well.
    struct SolverSnapshot {
        std::vector<double> X;
        std::vector<double> capV, indI, indV;
        std::vector<double> diodeStates, switchStates;
        std::vector<double> gStamped;
        bool trapModeStamped = false;
        bool trapModeStampValid = false;
        int forceBackwardEulerSteps = 0;
        bool valid = false;
    };
    SolverSnapshot stepSnapshot;
    void saveSolverState(SolverSnapshot& s) const;
    void restoreSolverState(const SolverSnapshot& s);

    // ── Dynamic stamp positions ──────────────────────────────────────────────
    // Every assembly used to restore the static matrix with a full n^2 copy, which
    // was the single largest solver cost once factorizations were being reused
    // (30% at n=98: a 2000-step run moved ~215 MB just to reset the matrix). Only
    // the entries that dynamic stamps actually write need resetting.
    //
    // The positions are recorded by the stamping helpers themselves the first time
    // each one is written, so no separate list of "which components stamp where"
    // has to be maintained in parallel with the stamping code. A position that is
    // not in the list has never been written, and therefore still holds its
    // K_static value from the initial full copy - which is what makes the partial
    // restore exact.
    std::vector<int> dynStampIdx;
    std::vector<unsigned char> dynStampSeen;
    bool dynStampBaseCopied = false;

    inline void markDynStamp(int idx) {
        if (!dynStampSeen[(size_t)idx]) {
            dynStampSeen[(size_t)idx] = 1;
            dynStampIdx.push_back(idx);
        }
    }

    // Conductance across a node pair. The write order is kept exactly as it was
    // when these four lines were repeated inline, so results stay bit-identical.
    inline void stampConductance(int n1, int n2, double g) {
        if (n1 >= 0) { const int d = n1 * totalDim + n1; markDynStamp(d); K[d] += g; }
        if (n2 >= 0) { const int d = n2 * totalDim + n2; markDynStamp(d); K[d] += g; }
        if (n1 >= 0 && n2 >= 0) {
            const int a = n1 * totalDim + n2;
            const int b = n2 * totalDim + n1;
            markDynStamp(a); markDynStamp(b);
            K[a] -= g;
            K[b] -= g;
        }
    }

    inline void stampBranchDiagonal(int i, double v) {
        const int d = i * totalDim + i;
        markDynStamp(d);
        K[d] += v;
    }
    // Integration mode the current factorization was built for. Switching between
    // trapezoidal and backward Euler changes the inductor stamp, so a transition has
    // to invalidate the factorization.
    bool trapModeStamped = false;
    bool trapModeStampValid = false;

    std::vector<double> LU_cached;
    std::vector<double> x_buf;
    std::vector<int> p_cached;

    // ── Factorization cache ──────────────────────────────────────────────────
    // A switching converter revisits the same few topologies every carrier cycle,
    // so the solver keeps re-factorizing matrices it has already seen. Caching the
    // factors turns the O(n^3) factorization into an O(n^2) lookup.
    //
    // The key is a hash of K, but a hit is only accepted after confirming the
    // stored matrix is bitwise identical to the current one. A hash collision can
    // therefore cost a little time but can never produce a wrong answer, which is
    // why the entry keeps its own copy of K.
    // Row-compressed form of the triangular factors, used by the per-step solve.
    // An MNA matrix is sparse, so most of the n^2 multiply-subtracts in a dense
    // forward/backward substitution are against a structural zero. Skipping those
    // terms is exact - adding 0*x contributes nothing for finite x - so the sparse
    // solve is bitwise identical to the dense one while touching far fewer values.
    //
    // This is built once per factorization and then reused by every step that hits
    // the same cache entry, which is where it pays for itself.
    struct SparseTriangular {
        std::vector<int> Lptr, Lidx;      // strictly lower, unit diagonal implied
        std::vector<double> Lval;
        std::vector<int> Uptr, Uidx;      // strictly upper
        std::vector<double> Uval;
        std::vector<double> Udiag;
        bool valid = false;
    };

    struct LUCacheEntry {
        uint64_t key = 0;
        uint64_t lastUsed = 0;
        std::vector<double> K;
        std::vector<double> LU;
        std::vector<int> perm;
        SparseTriangular sparse;
    };
    std::vector<LUCacheEntry> luCache;
    size_t luCacheMaxEntries = 0;
    uint64_t luCacheClock = 0;

    // Adaptive bail-out. If the matrix changes continuously - a signal-controlled
    // passive, for instance - every lookup misses and the hash is pure overhead
    // (measured ~11% on a 4-stage converter driving a VAR_R load). Hit rate is
    // tracked over a rolling window and caching is suspended when it stops paying,
    // then retried periodically so a circuit that settles down benefits again.
    long long luWindowLookups = 0;
    long long luWindowHits = 0;
    bool luCacheSuspended = false;
    long long luSuspendCountdown = 0;

    // The triangular solve reads through these, so a cache hit does not have to
    // copy n^2 doubles back into LU_cached. They point either at LU_cached (fresh
    // factorization) or straight into a cache entry.
    const double* LU_active = nullptr;
    const int* p_active = nullptr;
    // Set when the active factorization has a compressed form worth using; null
    // means fall back to the dense substitution.
    const SparseTriangular* sparse_active = nullptr;

    long long luFactorizeCount = 0;
    long long luCacheHitCount = 0;

    // Adaptive-stepping diagnostics, for benchmarking and for understanding why the
    // controller chose the steps it did.
    long long adaptiveAccepted = 0;
    long long adaptiveRejected = 0;
    long long adaptiveEventRetries = 0;
    long long adaptiveGateCuts = 0;
    long long adaptiveErrChecked = 0;   // steps where the error estimate was usable
    double adaptiveHMinUsed = 0.0;
    double adaptiveHMaxUsed = 0.0;

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
    // Number of samples already mirrored into `telemetry`, so live publishes can
    // append just the new tail instead of re-copying the whole history.
    size_t telemetryPublishedCount = 0;

    void buildIndexMaps();
    // `dtStep` is the step actually being taken, which under variable-step control is
    // not config.stepSize. Every integration, differentiation, phase-accumulation and
    // noise-scaling term must use it; see evaluateControls() for the four measurement
    // blocks that still size their windows from the nominal step.
    //
    // `commit` selects whether block state is allowed to advance. With commit = false
    // the outputs in flatControlSignals are produced for the requested time but no
    // integrator, latch, flip-flop, delay history or script engine is stepped, which
    // is what lets the adaptive controller probe a trial time - to find the instant a
    // gate changes - without corrupting anything. Stateful blocks then report outputs
    // consistent with the last committed step, which matches the Stage 1 model of
    // holding control signals constant across a step.
    void evaluateControls(double currentTime, double dtStep, bool commit = true);
    void evaluateGateControls(double currentTime, double dtStep);
    void assembleMNA(double currentTime, double dtStep);
    // Solves the network at `t` for a step of `hStep`, iterating the piecewise-linear
    // switch/diode states to convergence. Returns whether a device state was still
    // changing when the iteration stopped. Factored out of the step loop so the
    // adaptive controller can attempt the same step more than once.
    bool solveNetworkStep(double t, double hStep);
    bool updateDeviceStates();


    bool factorizeLU(int n);
    // Makes LU_active/p_active describe the current K, reusing a cached
    // factorization when this exact matrix has been factorized before.
    void prepareFactorization(int n);
    // Compresses LU_cached into row-compressed triangular factors. Leaves
    // `out.valid` false when the factors are too dense for this to be worthwhile.
    void buildSparseTriangular(int n, SparseTriangular& out) const;
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
        telemetryPublishedCount = 0;
        // A cleared history is a new generation: consumers must drop their caches
        // instead of appending the next run's samples onto the previous run's curves.
        telemetryGeneration.fetch_add(1, std::memory_order_relaxed);
        telemetryVersion.fetch_add(1, std::memory_order_relaxed);
    }

    std::atomic<uint64_t> telemetryVersion{0};
    // Incremented every time the telemetry history is discarded (reset / new run).
    // `telemetryVersion` alone cannot express this: it is monotonic, so a consumer
    // that had already cached run N's samples could not tell that the buffer it is
    // now reading belongs to run N+1.
    std::atomic<uint64_t> telemetryGeneration{0};
    std::atomic<double> computeTimeSeconds{0.0};

    uint64_t getTelemetryVersion() const {
        return telemetryVersion.load(std::memory_order_relaxed);
    }

    uint64_t getTelemetryGeneration() const {
        return telemetryGeneration.load(std::memory_order_relaxed);
    }

    // Factorization statistics for benchmarking and diagnostics.
    long long getFactorizeCount() const { return luFactorizeCount; }
    long long getLUCacheHitCount() const { return luCacheHitCount; }
    size_t getLUCacheEntryCount() const { return luCache.size(); }

    long long getAdaptiveAccepted() const { return adaptiveAccepted; }
    long long getAdaptiveRejected() const { return adaptiveRejected; }
    long long getAdaptiveEventRetries() const { return adaptiveEventRetries; }

    long long getAdaptiveGateCuts() const { return adaptiveGateCuts; }
    long long getAdaptiveErrChecked() const { return adaptiveErrChecked; }
    double getAdaptiveHMin() const { return adaptiveHMinUsed; }
    double getAdaptiveHMax() const { return adaptiveHMaxUsed; }

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
        telemetryPublishedCount = out.time.size();
        telemetryVersion.fetch_add(1, std::memory_order_relaxed);
    }

    // ── Incremental live telemetry publish ───────────────────────────────────
    // Appends only the samples produced since the previous publish. The earlier
    // implementation deep-copied the entire history on every update, which made
    // the total cost O(N^2) in the step count and accounted for 78-92% of solver
    // runtime on long runs. Appending makes it O(N) overall.
    void appendTelemetryFrom(const SimulationOutput& out) {
        const size_t total = out.time.size();
        std::lock_guard<std::mutex> lock(telemetryMutex);
        size_t from = telemetryPublishedCount;
        if (from > total) from = 0;              // history was reset underneath us
        if (from == total) return;               // nothing new

        // Reserve once so repeated appends do not reallocate.
        if (telemetry.timeHistory.capacity() < total) {
            telemetry.timeHistory.reserve(total + total / 2 + 64);
        }
        telemetry.timeHistory.insert(telemetry.timeHistory.end(),
                                     out.time.begin() + (ptrdiff_t)from,
                                     out.time.begin() + (ptrdiff_t)total);

        auto appendMap = [&](const std::unordered_map<std::string, std::vector<double>>& src) {
            for (const auto& kv : src) {
                const std::vector<double>& s = kv.second;
                if (s.size() <= from) continue;                  // this signal has no new data
                size_t to = (s.size() < total) ? s.size() : total;
                if (to <= from) continue;
                std::vector<double>& dst = telemetry.voltages[kv.first];
                if (dst.capacity() < total) dst.reserve(total + total / 2 + 64);
                dst.insert(dst.end(),
                           s.begin() + (ptrdiff_t)from,
                           s.begin() + (ptrdiff_t)to);
            }
        };
        appendMap(out.voltages);
        appendMap(out.signals);
        appendMap(out.inductors);
        appendMap(out.voltmeters);
        appendMap(out.ammeters);
        appendMap(out.custom_plots);

        telemetryPublishedCount = total;
        telemetryVersion.fetch_add(1, std::memory_order_relaxed);
    }

    TelemetryData getTelemetryCopy() {
        std::lock_guard<std::mutex> lock(telemetryMutex);
        return telemetry;
    }

    // Incremental read for UI consumers. Appends whatever samples exist beyond
    // `haveCount` into `dst` and returns the new total, so a view that redraws every
    // frame copies only the newly produced tail instead of the entire history.
    // Full resolution is preserved — nothing is decimated.
    //
    // `consumerGen` is the generation the caller last synchronised with; it is
    // updated in place. When it does not match the engine's current generation the
    // cache is discarded wholesale, which is what makes a re-run replace the plotted
    // curves instead of extending them.
    size_t syncTelemetryInto(TelemetryData& dst, size_t haveCount, uint64_t& consumerGen) {
        std::lock_guard<std::mutex> lock(telemetryMutex);

        // Read the generation under the telemetry lock so it can never be observed
        // out of step with the buffer it describes.
        const uint64_t gen = telemetryGeneration.load(std::memory_order_relaxed);
        if (consumerGen != gen) {
            dst.timeHistory.clear();
            dst.voltages.clear();       // also drops signals that no longer exist
            haveCount = 0;
            consumerGen = gen;
        }

        const size_t total = telemetry.timeHistory.size();

        // History shrank without a generation bump — start the consumer over anyway.
        if (haveCount > total) {
            dst.timeHistory.clear();
            dst.voltages.clear();
            haveCount = 0;
        }
        if (haveCount == total) return total;

        if (dst.timeHistory.capacity() < total) dst.timeHistory.reserve(total + total / 2 + 64);
        dst.timeHistory.insert(dst.timeHistory.end(),
                               telemetry.timeHistory.begin() + (ptrdiff_t)haveCount,
                               telemetry.timeHistory.begin() + (ptrdiff_t)total);

        for (const auto& kv : telemetry.voltages) {
            const std::vector<double>& s = kv.second;
            if (s.size() <= haveCount) continue;
            const size_t to = (s.size() < total) ? s.size() : total;
            std::vector<double>& d = dst.voltages[kv.first];
            // A signal the consumer has not seen before (or fell behind on) is copied whole.
            size_t from = haveCount;
            if (d.size() != haveCount) {
                d.clear();
                from = 0;
            }
            if (to <= from) continue;
            if (d.capacity() < total) d.reserve(total + total / 2 + 64);
            d.insert(d.end(), s.begin() + (ptrdiff_t)from, s.begin() + (ptrdiff_t)to);
        }
        return total;
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
        telemetryPublishedCount = 0;
        // Start a new telemetry generation and publish a version change so every
        // view notices immediately and hard-clears its cache (see syncTelemetryInto).
        telemetryGeneration.fetch_add(1, std::memory_order_relaxed);
        telemetryVersion.fetch_add(1, std::memory_order_relaxed);
    }
};

} // namespace CircuitSimEngine
