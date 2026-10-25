#pragma once

#include <string>
#include <vector>
#include <map>
#include <unordered_map>
#include <memory>
#include <cmath>

namespace CircuitSim {

enum class ComponentType {
    // Electrical Components
    Resistor,
    VariableResistor,
    Capacitor,
    Inductor,
    VoltageSource,
    ACVoltageSource,
    ThreePhaseSource, // V_3PH
    ControlledVoltageSource,
    CurrentSource,
    ACCurrentSource,
    ControlledCurrentSource,
    Switch,
    Diode,
    MOSFET,
    Transformer,
    OpAmp,
    EComp,
    GenEBlock,
    Voltmeter,
    Ammeter,
    Oscilloscope,
    UnifiedProbe,
    
    // Control & Math Blocks
    PI_Controller,
    PWM_Generator,
    MasterPWM,        // PWM_MASTER
    Triangle_Carrier,
    PulseGenerator,   // PULSE
    EdgeDetector,     // EDGE_DETECT
    Constant,
    Gain,
    SummingJunction,  // SUM_RECT / SUM_ROUND
    Product,          // PRODUCT_RECT
    MathFunction,     // MATH_FCN
    Comparator,
    AND_Gate,
    OR_Gate,
    NOT_Gate,
    CustomFunction,
    Mux,
    Demux,
    CustomScript,     // CSCRIPT
    KeyTrigger,       // KEY_TRIGGER
    Goto,
    From,

    // Ports & Subsystem Library
    Subsystem,
    Inport,           // INPORT, IN
    Outport,          // OUTPORT, OUT
    PhysicalInport,   // PHYSICAL_INPORT, PIN
    PhysicalOutport,  // PHYSICAL_OUTPORT, POUT
    EnablePort,       // ENABLE_PORT
    TriggerPort,      // TRIGGER_PORT
    BusCreator,       // BUS_CREATOR
    BusSelector,      // BUS_SELECTOR
    Terminator,       // TERMINATOR

    // Additional Functions & Tables
    Polynomial,       // POLYNOMIAL
    AlgebraicConstraint, // ALGEBRAIC_CONSTRAINT
    
    // Control Sources Detailed Library
    Clock,            // CLOCK
    InitialCondition, // INIT_COND
    Ramp,             // RAMP
    RandomNumbers,    // RANDOM_NUM
    SineWave,         // SINE_WAVE
    Step,             // STEP
    WhiteNoise,       // WHITE_NOISE

    // Control Functions & Tables Detailed Library
    TrigFunction,     // TRIG_FCN
    Abs,              // ABS
    Sign,             // SIGN
    Round,            // ROUND
    MinMax,           // MIN_MAX
    LUT_1D,           // LUT_1D
    LUT_2D,           // LUT_2D
    LUT_3D,           // LUT_3D
    DLL,              // DLL
    FMU,              // FMU
    FourierSeries,    // FOURIER_SERIES

    // Control Continuous Detailed Library
    Integrator,       // INTEGRATOR
    Derivative,       // DERIVATIVE
    TransferFunction, // TRANSFER_FCN
    StateSpace,       // STATE_SPACE
    ContinuousPID,    // CONT_PID
    PLL_1PH,          // PLL_1PH
    PLL_3PH,          // PLL_3PH

    // Control Delays Detailed Library
    Delay,            // DELAY
    TransportDelay,   // TRANSPORT_DELAY
    TurnOnDelay,      // TURN_ON_DELAY
    MemoryBlock,      // MEMORY_BLOCK

    // Control Discontinuous Detailed Library
    Quantizer,        // QUANTIZER
    SignalSwitch,     // SIGNAL_SWITCH
    ManualSwitch,     // MANUAL_SWITCH
    MultiportSwitch,  // MULTIPORT_SWITCH
    HitCrossing,      // HIT_CROSSING
    Saturation,       // SATURATION
    DeadZone,         // DEAD_ZONE
    RateLimiter,      // RATE_LIMITER
    Relay,            // RELAY

    // Control Logical & Bitwise Detailed Library
    LogicOp,              // LOGIC_OP
    BitwiseOp,            // BITWISE_OP
    CombLogic,            // COMB_LOGIC
    EdgeDetect,           // EDGE_DETECT
    Monostable,           // MONOSTABLE
    Monoflop,             // MONOFLOP
    RelationalOp,         // RELATIONAL_OPERATOR
    CompareToConstant,    // COMPARE_TO_CONSTANT
    DFlipFlop,            // D_FLIP_FLOP
    JKFlipFlop,           // JK_FLIP_FLOP
    ShiftReg,             // SHIFT_REG

    // Control Modulators & Signal Transforms
    PWM_3PH,              // PWM_3PH
    SVPWM,                // SVPWM
    Clarke,               // CLARKE
    Park,                 // PARK
    InvClarke,            // INV_CLARKE
    InvPark,              // INV_PARK
    DqToAbc,              // DQ_TO_ABC, DQ_ABC, INV_PARK_3PH
    AbcToDq,              // ABC_TO_DQ, ABC_DQ, PARK_3PH

    // Control Filters & Measurements
    PerAvg,               // PER_AVG
    PeriodicImpAvg,       // PERIODIC_IMP_AVG
    FourierTrans,         // FOURIER_TRANS
    MovAvg,               // MOV_AVG
    Filter1st,            // FILTER_1ST
    Filter2nd,            // FILTER_2ND
    FourierAnalysis,      // FOURIER_ANALYSIS
    RmsVal,               // RMS_VAL
    ThdVal,               // THD_VAL
    PllLoop,              // PLL_LOOP

    // Control State Machines & Math
    Offset,               // OFFSET
    Signum,               // SIGNUM
    Divide,               // DIVIDE
    DataTypeConv,         // DATATYPE_CONV
    StateMachine,         // STATE_MACHINE
    GotoSignal,           // GOTO_SIG, GOTO
    FromSignal,           // FROM_SIG, FROM

    // Power Semiconductors
    VGFET,                // vg-FET

    // Electrical Connectivity & Sources
    ElectricalPort,       // E_PORT
    ElectricalLabel,      // E_LABEL
    ThreePhaseCurrentSource, // I_3PH

    // Electrical Meters & Passives
    Voltmeter3Ph,         // VM_3PH
    Ammeter3Ph,           // AM_3PH
    VariableInductor,     // VAR_L
    VariableCapacitor,    // VAR_C
    SaturableInductor,    // SAT_L
    SaturableCapacitor,   // SAT_C
    PiSectionLine,        // PI_SECTION
    TransmissionLine3Ph,  // LINE_3PH
    PWLResistor,          // PWL_R
    ElectricalAlgebraic,  // E_ALGEBRAIC

    // Power Semiconductors
    Thyristor,            // THYRISTOR
    IGBT,                 // IGBT
    GTO,                  // GTO
    IGBTDiode,            // IGBT_DIODE
    IGCT,                 // IGCT
    BJT,                  // BJT
    JFET,                 // JFET

    // Switches
    Breaker,              // BREAKER
    DoubleSwitch,         // DBL_SWITCH
    ElectricalManualSwitch, // MAN_SWITCH
    ManualDoubleSwitch,   // MAN_DBL_SWITCH
    ManualTripleSwitch,   // MAN_TRPL_SWITCH
    SRSwitch,             // SR_SWITCH
    TripleSwitch,         // TRPL_SWITCH

    // Transformers
    IdealTransformer,     // IDEAL_XFMR
    Transformer2W,        // XFMR_2W
    Transformer3W,        // XFMR_3W
    MutualInductor2W,     // MUTUAL_2W
    MutualInductor3W,     // MUTUAL_3W
    SaturableTransformer, // SAT_XFMR
    Transformer3Ph2W,     // XFMR_3PH_2W
    Transformer3Ph3W,     // XFMR_3PH_3W

    // Electrical Machines & Custom
    InductionMotor,       // INDUCTION_MOTOR, IND_MOTOR

    // Magnetic domain (permeance-capacitance analogy).
    // MMF <-> node potential, flux rate <-> branch current, permeance <-> capacitance.
    Winding,              // WINDING            electrical <-> magnetic gyrator
    MagneticPermeance,    // MAG_PERMEANCE      P [Wb/A]
    LinearCore,           // LINEAR_CORE        P = mu0*mur*A/l
    AirGap,               // AIR_GAP            P = mu0*A/l
    LeakageFluxPath,      // LEAKAGE_PATH       shunt leakage permeance
    MagneticResistance,   // MAG_RESISTANCE     PhiDot = Gfe*F (core loss)
    MMFSource,            // MMF_SRC            constant MMF
    MMFSourceControlled,  // MMF_SRC_CTRL       signal-driven MMF

    // Analysis Tools
    LossAnalyzer,         // LOSS_ANALYZER      Semantic analysis block for loss engine

    Unknown
};

// Block type string -> ComponentType. Data-driven: the block registry
// (data/blocks/*.json, field "ui_type", plus "aliases") is the only table.
// Implemented in BlockRegistry.cpp. Unknown strings give ComponentType::Unknown.
ComponentType stringToComponentType(const std::string& typeStr);

inline std::string componentTypeToString(ComponentType type) {
    switch (type) {
        case ComponentType::LossAnalyzer: return "LOSS_ANALYZER";
        case ComponentType::Winding: return "WINDING";
        case ComponentType::MagneticPermeance: return "MAG_PERMEANCE";
        case ComponentType::LinearCore: return "LINEAR_CORE";
        case ComponentType::AirGap: return "AIR_GAP";
        case ComponentType::LeakageFluxPath: return "LEAKAGE_PATH";
        case ComponentType::MagneticResistance: return "MAG_RESISTANCE";
        case ComponentType::MMFSource: return "MMF_SRC";
        case ComponentType::MMFSourceControlled: return "MMF_SRC_CTRL";
        case ComponentType::Resistor: return "Resistor";
        case ComponentType::VariableResistor: return "VariableResistor";
        case ComponentType::Capacitor: return "Capacitor";
        case ComponentType::Inductor: return "Inductor";
        case ComponentType::VoltageSource: return "VoltageSource";
        case ComponentType::ACVoltageSource: return "ACVoltageSource";
        case ComponentType::ThreePhaseSource: return "ThreePhaseSource";
        case ComponentType::ControlledVoltageSource: return "ControlledVoltageSource";
        case ComponentType::CurrentSource: return "CurrentSource";
        case ComponentType::ACCurrentSource: return "ACCurrentSource";
        case ComponentType::ControlledCurrentSource: return "ControlledCurrentSource";
        case ComponentType::Switch: return "Switch";
        case ComponentType::Diode: return "Diode";
        case ComponentType::MOSFET: return "MOSFET";
        case ComponentType::Transformer: return "Transformer";
        case ComponentType::OpAmp: return "OpAmp";
        case ComponentType::EComp: return "EComp";
        case ComponentType::GenEBlock: return "GenEBlock";
        case ComponentType::Voltmeter: return "Voltmeter";
        case ComponentType::Ammeter: return "Ammeter";
        case ComponentType::Oscilloscope: return "Oscilloscope";
        case ComponentType::UnifiedProbe: return "UnifiedProbe";
        case ComponentType::PI_Controller: return "PI_Controller";
        case ComponentType::PWM_Generator: return "PWM_Generator";
        case ComponentType::MasterPWM: return "MasterPWM";
        case ComponentType::Triangle_Carrier: return "Triangle_Carrier";
        case ComponentType::PulseGenerator: return "PulseGenerator";
        case ComponentType::EdgeDetector: return "EdgeDetector";
        case ComponentType::Constant: return "Constant";
        case ComponentType::Gain: return "Gain";
        case ComponentType::SummingJunction: return "SummingJunction";
        case ComponentType::Product: return "Product";
        case ComponentType::MathFunction: return "MathFunction";
        case ComponentType::Comparator: return "Comparator";
        case ComponentType::AND_Gate: return "AND_Gate";
        case ComponentType::OR_Gate: return "OR_Gate";
        case ComponentType::NOT_Gate: return "NOT_Gate";
        case ComponentType::CustomFunction: return "CustomFunction";
        case ComponentType::Mux: return "Mux";
        case ComponentType::Demux: return "Demux";
        case ComponentType::CustomScript: return "CustomScript";
        case ComponentType::KeyTrigger: return "KeyTrigger";
        case ComponentType::Goto: return "Goto";
        case ComponentType::From: return "From";
        case ComponentType::Subsystem: return "Subsystem";
        case ComponentType::Inport: return "Inport";
        case ComponentType::Outport: return "Outport";
        case ComponentType::PhysicalInport: return "PhysicalInport";
        case ComponentType::PhysicalOutport: return "PhysicalOutport";
        case ComponentType::EnablePort: return "EnablePort";
        case ComponentType::TriggerPort: return "TriggerPort";
        case ComponentType::BusCreator: return "BusCreator";
        case ComponentType::BusSelector: return "BusSelector";
        case ComponentType::Terminator: return "Terminator";
        case ComponentType::Polynomial: return "Polynomial";
        case ComponentType::AlgebraicConstraint: return "AlgebraicConstraint";
        case ComponentType::DqToAbc: return "DQ_TO_ABC";
        case ComponentType::AbcToDq: return "ABC_TO_DQ";
        default: return "Unknown";
    }
}

struct Pin {
    std::string name;
    float relativeX = 0.0f;
    float relativeY = 0.0f;
    bool isInput = false;
    bool isOutput = false;
    bool isCtrl = false;
    std::string opSign = "+"; // +, -, *, /
};

struct ComponentInstance {
    std::string id;
    std::string rawTypeStr;
    ComponentType type = ComponentType::Unknown;
    std::string label;
    float x = 0.0f;
    float y = 0.0f;
    float width = 80.0f;
    float height = 60.0f;
    int rotation = 0; // 0, 90, 180, 270 degrees
    
    std::vector<std::string> nodes; // Electrical terminal node connections
    std::unordered_map<std::string, std::string> parameters;
    
    // Dynamic configurator state for SUM_RECT, SUM_ROUND, PRODUCT_RECT
    int numInputPins = 2;
    std::vector<std::string> pinSigns = {"+", "+"};
    bool hasCtrlPin = true;
    
    std::vector<Pin> pins;
};

inline void setupComponentPins(ComponentInstance& comp) {
    comp.pins.clear();
    std::string t = comp.rawTypeStr;
    if (t.empty()) t = componentTypeToString(comp.type);
    
    if (t == "R" || t == "Resistor" || t == "L" || t == "Inductor" || t == "C" || t == "Capacitor" ||
        t == "V" || t == "VoltageSource" || t == "DC_V" || t == "DC_V1" || t == "ac" || t == "ACVoltageSource" || t == "AC_V" ||
        t == "I" || t == "CurrentSource" || t == "DC_I" || t == "AC_I" || t == "ACCurrentSource" ||
        t == "D" || t == "Diode" ||
        t == "SAT_L" || t == "SaturableInductor" || t == "SAT_C" || t == "SaturableCapacitor" || t == "PWL_R" || t == "PWLResistor") {
        
        Pin pinA; pinA.name = "A"; pinA.relativeX = 0; pinA.relativeY = -40;
        Pin pinB; pinB.name = "B"; pinB.relativeX = 0; pinB.relativeY = 40;
        comp.pins = {pinA, pinB};
    } else if (t == "VAR_R" || t == "VariableResistor" ||
               t == "VAR_L" || t == "VariableInductor" ||
               t == "VAR_C" || t == "VariableCapacitor") {
        // Signal-controlled passives need a control input so the element value can be
        // driven by a waveform. Order matches getTerminals(): A/B first, Ctrl last.
        Pin pinA; pinA.name = "A"; pinA.relativeX = 0; pinA.relativeY = -40;
        Pin pinB; pinB.name = "B"; pinB.relativeX = 0; pinB.relativeY = 40;
        Pin pinCtrl; pinCtrl.name = "Ctrl"; pinCtrl.relativeX = -30; pinCtrl.relativeY = 0; pinCtrl.isInput = true; pinCtrl.isCtrl = true;
        comp.pins = {pinA, pinB, pinCtrl};
    } else if (t == "VM" || t == "Voltmeter" || t == "AM" || t == "Ammeter") {
        Pin pinA; pinA.name = "A"; pinA.relativeX = 0; pinA.relativeY = -40;
        Pin pinB; pinB.name = "B"; pinB.relativeX = 0; pinB.relativeY = 40;
        Pin pinOut; pinOut.name = "Out"; pinOut.relativeX = 20; pinOut.relativeY = 0; pinOut.isOutput = true;
        comp.pins = {pinA, pinB, pinOut};
    } else if (t == "CTRL_V" || t == "ControlledVoltageSource" || t == "CTRL_I" || t == "ControlledCurrentSource") {
        // The control pin must be named "Ctrl": the netlist generator resolves the
        // driving signal via getIncomingSignal(id, "Ctrl"). It used to be named "In",
        // so the lookup always failed and the source saw a constant 0.
        Pin pinA; pinA.name = "A"; pinA.relativeX = 0; pinA.relativeY = -40;
        Pin pinB; pinB.name = "B"; pinB.relativeX = 0; pinB.relativeY = 40;
        Pin pinCtrl; pinCtrl.name = "Ctrl"; pinCtrl.relativeX = -30; pinCtrl.relativeY = 0; pinCtrl.isInput = true; pinCtrl.isCtrl = true;
        // Order must match getTerminals(): electrical A/B first, control pin last, so
        // nodes[0]/nodes[1] remain the two electrical terminals.
        comp.pins = {pinA, pinB, pinCtrl};
    } else if (t == "MOSFET" || t == "vg-FET" || t == "VGFET" ||
               t == "IGBT" || t == "IGBT_DIODE" || t == "IGBTDiode" ||
               t == "JFET") {
        Pin pinD; pinD.name = "D"; pinD.relativeX = 0; pinD.relativeY = -40;
        Pin pinS; pinS.name = "S"; pinS.relativeX = 0; pinS.relativeY = 40;
        Pin pinG; pinG.name = "G"; pinG.relativeX = -20; pinG.relativeY = 0; pinG.isCtrl = true;
        comp.pins = {pinD, pinS, pinG};
    } else if (t == "WINDING") {
        // Electrical pair on the left, magnetic pair on the right.
        Pin eP; eP.name = "E+"; eP.relativeX = -25; eP.relativeY = -25;
        Pin eN; eN.name = "E-"; eN.relativeX = -25; eN.relativeY =  25;
        Pin mP; mP.name = "M+"; mP.relativeX =  25; mP.relativeY = -25;
        Pin mN; mN.name = "M-"; mN.relativeX =  25; mN.relativeY =  25;
        comp.pins = {eP, eN, mP, mN};
    } else if (t == "MAG_PERMEANCE" || t == "LINEAR_CORE" || t == "AIR_GAP" ||
               t == "LEAKAGE_PATH" || t == "MAG_RESISTANCE" || t == "MMF_SRC") {
        Pin pinA; pinA.name = "A"; pinA.relativeX = 0; pinA.relativeY = -40;
        Pin pinB; pinB.name = "B"; pinB.relativeX = 0; pinB.relativeY = 40;
        comp.pins = {pinA, pinB};
    } else if (t == "MMF_SRC_CTRL") {
        Pin pinA; pinA.name = "A"; pinA.relativeX = 0; pinA.relativeY = -40;
        Pin pinB; pinB.name = "B"; pinB.relativeX = 0; pinB.relativeY = 40;
        Pin pinC; pinC.name = "Ctrl"; pinC.relativeX = -30; pinC.relativeY = 0; pinC.isInput = true; pinC.isCtrl = true;
        comp.pins = {pinA, pinB, pinC};
    } else if (t == "THYRISTOR" || t == "Thyristor" || t == "SCR" || t == "GTO" || t == "IGCT") {
        // Anode / Cathode / Gate
        Pin pinA; pinA.name = "A"; pinA.relativeX = 0; pinA.relativeY = -40;
        Pin pinK; pinK.name = "K"; pinK.relativeX = 0; pinK.relativeY = 40;
        Pin pinG; pinG.name = "G"; pinG.relativeX = -20; pinG.relativeY = 10; pinG.isCtrl = true;
        comp.pins = {pinA, pinK, pinG};
    } else if (t == "BJT") {
        // Collector / Emitter / Base
        Pin pinC; pinC.name = "C"; pinC.relativeX = 0; pinC.relativeY = -40;
        Pin pinE; pinE.name = "E"; pinE.relativeX = 0; pinE.relativeY = 40;
        Pin pinB; pinB.name = "B"; pinB.relativeX = -20; pinB.relativeY = 0; pinB.isCtrl = true;
        comp.pins = {pinC, pinE, pinB};
    } else if (t == "S" || t == "Switch") {
        Pin pinA; pinA.name = "A"; pinA.relativeX = 0; pinA.relativeY = -40;
        Pin pinB; pinB.name = "B"; pinB.relativeX = 0; pinB.relativeY = 40;
        Pin pinCtrl; pinCtrl.name = "Ctrl"; pinCtrl.relativeX = -20; pinCtrl.relativeY = 0; pinCtrl.isCtrl = true;
        comp.pins = {pinA, pinB, pinCtrl};
    } else if (t == "CSCRIPT" || t == "CustomScript") {
        Pin in1; in1.name = "In1"; in1.relativeX = -80; in1.relativeY = 0; in1.isInput = true;
        Pin out1; out1.name = "Out1"; out1.relativeX = 80; out1.relativeY = -30; out1.isOutput = true;
        Pin out2; out2.name = "Out2"; out2.relativeX = 80; out2.relativeY = -10; out2.isOutput = true;
        Pin out3; out3.name = "Out3"; out3.relativeX = 80; out3.relativeY = 10; out3.isOutput = true;
        Pin out4; out4.name = "Out4"; out4.relativeX = 80; out4.relativeY = 30; out4.isOutput = true;
        comp.pins = {in1, out1, out2, out3, out4};
    } else if (t == "PROBE" || t == "UnifiedProbe") {
        Pin p1; p1.name = "I_L1"; p1.relativeX = 40; p1.relativeY = -15; p1.isOutput = true;
        Pin p2; p2.name = "V_MOSFET2"; p2.relativeX = 40; p2.relativeY = 15; p2.isOutput = true;
        comp.pins = {p1, p2};
    } else if (t == "SCOPE" || t == "Oscilloscope") {
        int nCh = 2;
        auto it = comp.parameters.find("channels");
        if (it != comp.parameters.end()) {
            try { nCh = std::max(1, std::stoi(it->second)); } catch (...) {}
        }
        float spacing = 20.0f;
        float startY = -((nCh - 1) * spacing) * 0.5f;
        for (int i = 0; i < nCh; ++i) {
            Pin p;
            p.name = "In" + std::to_string(i + 1);
            p.relativeX = -30.0f;
            p.relativeY = startY + i * spacing;
            p.isInput = true;
            comp.pins.push_back(p);
        }
    } else if (t == "V_3PH" || t == "ThreePhaseSource" || t == "I_3PH" || t == "ThreePhaseCurrentSource") {
        Pin pinA; pinA.name = "A"; pinA.relativeX = -30; pinA.relativeY = -25;
        Pin pinB; pinB.name = "B"; pinB.relativeX = -30; pinB.relativeY = 0;
        Pin pinC; pinC.name = "C"; pinC.relativeX = -30; pinC.relativeY = 25;
        Pin pinN; pinN.name = "N"; pinN.relativeX = 30; pinN.relativeY = 0;
        comp.pins = {pinA, pinB, pinC, pinN};
    } else if (t == "VM_3PH" || t == "Voltmeter3Ph" || t == "AM_3PH" || t == "Ammeter3Ph") {
        Pin pinA; pinA.name = "A"; pinA.relativeX = -30; pinA.relativeY = -25;
        Pin pinB; pinB.name = "B"; pinB.relativeX = -30; pinB.relativeY = 0;
        Pin pinC; pinC.name = "C"; pinC.relativeX = -30; pinC.relativeY = 25;
        Pin pinVa; pinVa.name = "Va"; pinVa.relativeX = 30; pinVa.relativeY = -25; pinVa.isOutput = true;
        Pin pinVb; pinVb.name = "Vb"; pinVb.relativeX = 30; pinVb.relativeY = 0; pinVb.isOutput = true;
        Pin pinVc; pinVc.name = "Vc"; pinVc.relativeX = 30; pinVc.relativeY = 25; pinVc.isOutput = true;
        Pin pinIa; pinIa.name = "Ia"; pinIa.relativeX = 30; pinIa.relativeY = -25; pinIa.isOutput = true;
        Pin pinIb; pinIb.name = "Ib"; pinIb.relativeX = 30; pinIb.relativeY = 0; pinIb.isOutput = true;
        Pin pinIc; pinIc.name = "Ic"; pinIc.relativeX = 30; pinIc.relativeY = 25; pinIc.isOutput = true;
        comp.pins = {pinA, pinB, pinC, pinVa, pinVb, pinVc, pinIa, pinIb, pinIc};
    } else if (t == "PWM_MASTER" || t == "MasterPWM") {
        Pin in1; in1.name = "In1"; in1.relativeX = -40; in1.relativeY = -20; in1.isInput = true;
        Pin in2; in2.name = "In2"; in2.relativeX = -40; in2.relativeY = 0; in2.isInput = true;
        Pin in3; in3.name = "In3"; in3.relativeX = -40; in3.relativeY = 20; in3.isInput = true;
        Pin p1; p1.name = "P1"; p1.relativeX = 40; p1.relativeY = -25; p1.isOutput = true;
        Pin p1n; p1n.name = "P1_n"; p1n.relativeX = 40; p1n.relativeY = -15; p1n.isOutput = true;
        Pin p2; p2.name = "P2"; p2.relativeX = 40; p2.relativeY = -5; p2.isOutput = true;
        Pin p2n; p2n.name = "P2_n"; p2n.relativeX = 40; p2n.relativeY = 5; p2n.isOutput = true;
        Pin p3; p3.name = "P3"; p3.relativeX = 40; p3.relativeY = 15; p3.isOutput = true;
        Pin p3n; p3n.name = "P3_n"; p3n.relativeX = 40; p3n.relativeY = 25; p3n.isOutput = true;
        comp.pins = {in1, in2, in3, p1, p1n, p2, p2n, p3, p3n};
    } else if (t == "EDGE_DETECT" || t == "EdgeDetector") {
        Pin in; in.name = "In"; in.relativeX = -25; in.relativeY = 0; in.isInput = true;
        Pin out; out.name = "Out"; out.relativeX = 25; out.relativeY = 0; out.isOutput = true;
        comp.pins = {in, out};
    } else if (t == "MATH_FCN" || t == "FCN" || t == "MathFunction") {
        Pin in1; in1.name = "In1"; in1.relativeX = -25; in1.relativeY = -10; in1.isInput = true;
        Pin in2; in2.name = "In2"; in2.relativeX = -25; in2.relativeY = 10; in2.isInput = true;
        Pin out; out.name = "Out"; out.relativeX = 25; out.relativeY = 0; out.isOutput = true;
        comp.pins = {in1, in2, out};
    } else if (t == "KEY_TRIGGER" || t == "KeyTrigger") {
        Pin out; out.name = "Out"; out.relativeX = 25; out.relativeY = 0; out.isOutput = true;
        comp.pins = {out};
    } else if (t == "DQ_TO_ABC" || t == "DQ_ABC" || t == "INV_PARK_3PH" || t == "DqToAbc") {
        Pin d; d.name = "d"; d.relativeX = -25; d.relativeY = -15; d.isInput = true;
        Pin q; q.name = "q"; q.relativeX = -25; q.relativeY = 0; q.isInput = true;
        Pin theta; theta.name = "Theta"; theta.relativeX = -25; theta.relativeY = 15; theta.isInput = true;
        Pin a; a.name = "A"; a.relativeX = 25; a.relativeY = -15; a.isOutput = true;
        Pin b; b.name = "B"; b.relativeX = 25; b.relativeY = 0; b.isOutput = true;
        Pin c; c.name = "C"; c.relativeX = 25; c.relativeY = 15; c.isOutput = true;
        comp.pins = {d, q, theta, a, b, c};
    } else if (t == "ABC_TO_DQ" || t == "ABC_DQ" || t == "PARK_3PH" || t == "AbcToDq") {
        Pin a; a.name = "A"; a.relativeX = -25; a.relativeY = -15; a.isInput = true;
        Pin b; b.name = "B"; b.relativeX = -25; b.relativeY = 0; b.isInput = true;
        Pin c; c.name = "C"; c.relativeX = -25; c.relativeY = 15; c.isInput = true;
        Pin theta; theta.name = "Theta"; theta.relativeX = -25; theta.relativeY = 30; theta.isInput = true;
        Pin d; d.name = "d"; d.relativeX = 25; d.relativeY = -10; d.isOutput = true;
        Pin q; q.name = "q"; q.relativeX = 25; q.relativeY = 10; q.isOutput = true;
        comp.pins = {a, b, c, theta, d, q};
    }
}

struct Point2D {
    float x = 0.0f;
    float y = 0.0f;
};

struct WireEndpoint {
    std::string compId;
    std::string terminal;
    bool isWireJunction = false;
    std::string targetWireId;
    float junctionX = 0.0f;
    float junctionY = 0.0f;
};

struct WireInstance {
    std::string id;
    std::string fromNode;
    std::string toNode;
    WireEndpoint from;
    WireEndpoint to;
    std::vector<Point2D> manualPath;
};

struct SolverSettings {
    double stopTime = 0.05;
    double stepSize = 1e-5;
    std::string solverType = "euler"; // euler, trapezoidal
    std::string stepType = "fixed";   // fixed, variable
    std::string diodeModel = "non-ideal"; // non-ideal, ideal-pwl
    bool enableLUCache = true;

    // Adaptive stepping (stepType == "variable"). With fixed stepping these are
    // ignored. `stepSize` then acts as the reference/initial step rather than a
    // ceiling; hMax/hMin of 0 mean "derive automatically", which also caps the largest
    // step at a quarter of the shortest carrier period found in the circuit.
    double relTol = 1e-3;
    double absTolV = 1e-3;   // volts
    double absTolI = 1e-6;   // amps
    double hMin = 0.0;       // 0 = auto
    double hMax = 0.0;       // 0 = auto
};

struct PlotChannelConfig {
    std::string title = "Waveform Analysis";
    std::vector<std::string> variables;
};

struct PlotConfig {
    std::vector<PlotChannelConfig> plots;
};

struct CircuitDesign {
    std::vector<ComponentInstance> components;
    std::vector<WireInstance> wires;
    SolverSettings settings;
    PlotConfig plotConfig;
};

struct TerminalDef {
    std::string name;
    float relX;
    float relY;
    float dirX;
    float dirY;
    bool isControl;

    float x;
    float y;
    float dx;
    float dy;

    TerminalDef() = default;
    TerminalDef(std::string n, float rx, float ry, float dx_, float dy_, bool ctrl)
        : name(n), relX(rx), relY(ry), dirX(dx_), dirY(dy_), isControl(ctrl),
          x(rx), y(ry), dx(dx_), dy(dy_) {}
};

std::vector<TerminalDef> getTerminals(const ComponentInstance& comp);

} // namespace CircuitSim
