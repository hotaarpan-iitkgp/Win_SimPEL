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

    Unknown
};

inline ComponentType stringToComponentType(const std::string& typeStr) {
    if (typeStr == "Resistor" || typeStr == "R") return ComponentType::Resistor;
    if (typeStr == "VariableResistor") return ComponentType::VariableResistor;
    if (typeStr == "Capacitor" || typeStr == "C") return ComponentType::Capacitor;
    if (typeStr == "Inductor" || typeStr == "L") return ComponentType::Inductor;
    if (typeStr == "VoltageSource" || typeStr == "V" || typeStr == "dc") return ComponentType::VoltageSource;
    if (typeStr == "ACVoltageSource" || typeStr == "ac" || typeStr == "AC_V") return ComponentType::ACVoltageSource;
    if (typeStr == "ThreePhaseSource" || typeStr == "V_3PH") return ComponentType::ThreePhaseSource;
    if (typeStr == "ControlledVoltageSource") return ComponentType::ControlledVoltageSource;
    if (typeStr == "CurrentSource" || typeStr == "I") return ComponentType::CurrentSource;
    if (typeStr == "ACCurrentSource") return ComponentType::ACCurrentSource;
    if (typeStr == "ControlledCurrentSource") return ComponentType::ControlledCurrentSource;
    if (typeStr == "Switch" || typeStr == "S") return ComponentType::Switch;
    if (typeStr == "Diode" || typeStr == "D") return ComponentType::Diode;
    if (typeStr == "MOSFET" || typeStr == "vg-FET") return ComponentType::MOSFET;
    if (typeStr == "Transformer" || typeStr == "XFMR") return ComponentType::Transformer;
    if (typeStr == "OpAmp" || typeStr == "OPAMP") return ComponentType::OpAmp;
    if (typeStr == "EComp" || typeStr == "E_COMP") return ComponentType::EComp;
    if (typeStr == "GenEBlock" || typeStr == "GEN_EBLOCK") return ComponentType::GenEBlock;
    if (typeStr == "Voltmeter" || typeStr == "VM") return ComponentType::Voltmeter;
    if (typeStr == "Ammeter" || typeStr == "AM") return ComponentType::Ammeter;
    if (typeStr == "Oscilloscope" || typeStr == "SCOPE") return ComponentType::Oscilloscope;
    if (typeStr == "UnifiedProbe" || typeStr == "PROBE") return ComponentType::UnifiedProbe;
    if (typeStr == "PI_Controller" || typeStr == "PID") return ComponentType::PI_Controller;
    if (typeStr == "PWM_Generator" || typeStr == "PWM") return ComponentType::PWM_Generator;
    if (typeStr == "MasterPWM" || typeStr == "PWM_MASTER") return ComponentType::MasterPWM;
    if (typeStr == "Triangle_Carrier" || typeStr == "TRI" || typeStr == "TRI_GEN") return ComponentType::Triangle_Carrier;
    if (typeStr == "PulseGenerator" || typeStr == "PULSE" || typeStr == "PULSE_GEN") return ComponentType::PulseGenerator;
    if (typeStr == "EdgeDetector" || typeStr == "EDGE_DETECT") return ComponentType::EdgeDetector;
    if (typeStr == "Constant" || typeStr == "CONST") return ComponentType::Constant;
    if (typeStr == "Clock" || typeStr == "CLOCK") return ComponentType::Clock;
    if (typeStr == "InitialCondition" || typeStr == "INIT_COND") return ComponentType::InitialCondition;
    if (typeStr == "Ramp" || typeStr == "RAMP") return ComponentType::Ramp;
    if (typeStr == "RandomNumbers" || typeStr == "RANDOM_NUM") return ComponentType::RandomNumbers;
    if (typeStr == "SineWave" || typeStr == "SINE_WAVE") return ComponentType::SineWave;
    if (typeStr == "Step" || typeStr == "STEP") return ComponentType::Step;
    if (typeStr == "WhiteNoise" || typeStr == "WHITE_NOISE") return ComponentType::WhiteNoise;
    if (typeStr == "TrigFunction" || typeStr == "TRIG_FCN") return ComponentType::TrigFunction;
    if (typeStr == "Abs" || typeStr == "ABS") return ComponentType::Abs;
    if (typeStr == "Sign" || typeStr == "SIGN") return ComponentType::Sign;
    if (typeStr == "Integrator" || typeStr == "INTEGRATOR") return ComponentType::Integrator;
    if (typeStr == "Derivative" || typeStr == "DERIVATIVE") return ComponentType::Derivative;
    if (typeStr == "TransferFunction" || typeStr == "TRANSFER_FCN") return ComponentType::TransferFunction;
    if (typeStr == "StateSpace" || typeStr == "STATE_SPACE") return ComponentType::StateSpace;
    if (typeStr == "ContinuousPID" || typeStr == "CONT_PID") return ComponentType::ContinuousPID;
    if (typeStr == "PLL_1PH") return ComponentType::PLL_1PH;
    if (typeStr == "PLL_3PH") return ComponentType::PLL_3PH;
    if (typeStr == "Delay" || typeStr == "DELAY") return ComponentType::Delay;
    if (typeStr == "TransportDelay" || typeStr == "TRANSPORT_DELAY") return ComponentType::TransportDelay;
    if (typeStr == "TurnOnDelay" || typeStr == "TURN_ON_DELAY") return ComponentType::TurnOnDelay;
    if (typeStr == "MemoryBlock" || typeStr == "MEMORY" || typeStr == "MEMORY_BLOCK") return ComponentType::MemoryBlock;
    if (typeStr == "Quantizer" || typeStr == "QUANTIZER") return ComponentType::Quantizer;
    if (typeStr == "SignalSwitch" || typeStr == "SIGNAL_SWITCH") return ComponentType::SignalSwitch;
    if (typeStr == "ManualSwitch" || typeStr == "MANUAL_SWITCH") return ComponentType::ManualSwitch;
    if (typeStr == "MultiportSwitch" || typeStr == "MULTIPORT_SWITCH") return ComponentType::MultiportSwitch;
    if (typeStr == "HitCrossing" || typeStr == "HIT_CROSSING") return ComponentType::HitCrossing;
    if (typeStr == "Saturation" || typeStr == "SATURATION") return ComponentType::Saturation;
    if (typeStr == "DeadZone" || typeStr == "DEAD_ZONE") return ComponentType::DeadZone;
    if (typeStr == "RateLimiter" || typeStr == "RATE_LIMITER") return ComponentType::RateLimiter;
    if (typeStr == "Relay" || typeStr == "RELAY") return ComponentType::Relay;
    if (typeStr == "Comparator" || typeStr == "COMP") return ComponentType::Comparator;
    if (typeStr == "Round" || typeStr == "ROUND") return ComponentType::Round;
    if (typeStr == "MinMax" || typeStr == "MIN_MAX") return ComponentType::MinMax;
    if (typeStr == "LUT_1D") return ComponentType::LUT_1D;
    if (typeStr == "LUT_2D") return ComponentType::LUT_2D;
    if (typeStr == "LUT_3D") return ComponentType::LUT_3D;
    if (typeStr == "DLL") return ComponentType::DLL;
    if (typeStr == "FMU") return ComponentType::FMU;
    if (typeStr == "FourierSeries" || typeStr == "FOURIER_SERIES") return ComponentType::FourierSeries;
    if (typeStr == "Gain" || typeStr == "GAIN") return ComponentType::Gain;
    if (typeStr == "SummingJunction" || typeStr == "SUM" || typeStr == "SUM_RECT" || typeStr == "SUM_ROUND") return ComponentType::SummingJunction;
    if (typeStr == "Product" || typeStr == "PROD" || typeStr == "PRODUCT_RECT") return ComponentType::Product;
    if (typeStr == "MathFunction" || typeStr == "MATH_FCN") return ComponentType::MathFunction;
    if (typeStr == "Comparator" || typeStr == "COMP") return ComponentType::Comparator;
    if (typeStr == "AND_Gate" || typeStr == "AND") return ComponentType::AND_Gate;
    if (typeStr == "OR_Gate" || typeStr == "OR") return ComponentType::OR_Gate;
    if (typeStr == "NOT_Gate" || typeStr == "NOT") return ComponentType::NOT_Gate;
    if (typeStr == "CustomFunction" || typeStr == "FCN") return ComponentType::CustomFunction;
    if (typeStr == "Mux") return ComponentType::Mux;
    if (typeStr == "Demux") return ComponentType::Demux;
    if (typeStr == "CustomScript" || typeStr == "CSCRIPT") return ComponentType::CustomScript;
    if (typeStr == "KeyTrigger" || typeStr == "KEY_TRIGGER") return ComponentType::KeyTrigger;
    if (typeStr == "Goto" || typeStr == "GOTO") return ComponentType::Goto;
    if (typeStr == "From" || typeStr == "FROM") return ComponentType::From;
    return ComponentType::Unknown;
}

inline std::string componentTypeToString(ComponentType type) {
    switch (type) {
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
        t == "V" || t == "VoltageSource" || t == "ac" || t == "ACVoltageSource" || t == "AC_V" || t == "D" || t == "Diode" ||
        t == "VM" || t == "Voltmeter" || t == "AM" || t == "Ammeter") {
        
        Pin pinA; pinA.name = "A"; pinA.relativeX = 0; pinA.relativeY = -40;
        Pin pinB; pinB.name = "B"; pinB.relativeX = 0; pinB.relativeY = 40;
        comp.pins = {pinA, pinB};
    } else if (t == "MOSFET" || t == "vg-FET") {
        Pin pinD; pinD.name = "D"; pinD.relativeX = 0; pinD.relativeY = -40;
        Pin pinS; pinS.name = "S"; pinS.relativeX = 0; pinS.relativeY = 40;
        Pin pinG; pinG.name = "G"; pinG.relativeX = -20; pinG.relativeY = 0; pinG.isCtrl = true;
        comp.pins = {pinD, pinS, pinG};
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
        Pin p1; p1.name = "In1"; p1.relativeX = -30; p1.relativeY = -10; p1.isInput = true;
        Pin p2; p2.name = "In2"; p2.relativeX = -30; p2.relativeY = 10; p2.isInput = true;
        comp.pins = {p1, p2};
    } else if (t == "V_3PH" || t == "ThreePhaseSource") {
        Pin pinA; pinA.name = "A"; pinA.relativeX = -30; pinA.relativeY = -25;
        Pin pinB; pinB.name = "B"; pinB.relativeX = -30; pinB.relativeY = 0;
        Pin pinC; pinC.name = "C"; pinC.relativeX = -30; pinC.relativeY = 25;
        Pin pinN; pinN.name = "N"; pinN.relativeX = 30; pinN.relativeY = 0;
        comp.pins = {pinA, pinB, pinC, pinN};
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
    std::string solverType = "euler"; // euler, rk4, adaptive
    std::string stepType = "fixed";
    std::string diodeModel = "non-ideal"; // non-ideal, ideal-pwl
    bool enableLUCache = true;
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
