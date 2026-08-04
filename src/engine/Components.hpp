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
    Triangle_Carrier,
    PulseGenerator,  // PULSE
    Constant,
    Gain,
    SummingJunction, // SUM_RECT / SUM_ROUND
    Product,         // PRODUCT_RECT
    Comparator,
    AND_Gate,
    OR_Gate,
    NOT_Gate,
    CustomFunction,
    Mux,
    Demux,
    CustomScript,   // CSCRIPT
    Goto,
    From,
    
    Unknown
};

inline ComponentType stringToComponentType(const std::string& typeStr) {
    if (typeStr == "Resistor" || typeStr == "R") return ComponentType::Resistor;
    if (typeStr == "VariableResistor") return ComponentType::VariableResistor;
    if (typeStr == "Capacitor" || typeStr == "C") return ComponentType::Capacitor;
    if (typeStr == "Inductor" || typeStr == "L") return ComponentType::Inductor;
    if (typeStr == "VoltageSource" || typeStr == "V" || typeStr == "dc") return ComponentType::VoltageSource;
    if (typeStr == "ACVoltageSource" || typeStr == "ac" || typeStr == "AC_V") return ComponentType::ACVoltageSource;
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
    if (typeStr == "Triangle_Carrier" || typeStr == "TRI") return ComponentType::Triangle_Carrier;
    if (typeStr == "PulseGenerator" || typeStr == "PULSE" || typeStr == "PULSE_GEN") return ComponentType::PulseGenerator;
    if (typeStr == "Constant" || typeStr == "CONST") return ComponentType::Constant;
    if (typeStr == "Gain" || typeStr == "GAIN") return ComponentType::Gain;
    if (typeStr == "SummingJunction" || typeStr == "SUM" || typeStr == "SUM_RECT" || typeStr == "SUM_ROUND") return ComponentType::SummingJunction;
    if (typeStr == "Product" || typeStr == "PROD" || typeStr == "PRODUCT_RECT") return ComponentType::Product;
    if (typeStr == "Comparator" || typeStr == "COMP") return ComponentType::Comparator;
    if (typeStr == "AND_Gate" || typeStr == "AND") return ComponentType::AND_Gate;
    if (typeStr == "OR_Gate" || typeStr == "OR") return ComponentType::OR_Gate;
    if (typeStr == "NOT_Gate" || typeStr == "NOT") return ComponentType::NOT_Gate;
    if (typeStr == "CustomFunction" || typeStr == "FCN") return ComponentType::CustomFunction;
    if (typeStr == "Mux") return ComponentType::Mux;
    if (typeStr == "Demux") return ComponentType::Demux;
    if (typeStr == "CustomScript" || typeStr == "CSCRIPT") return ComponentType::CustomScript;
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
        case ComponentType::Triangle_Carrier: return "Triangle_Carrier";
        case ComponentType::Constant: return "Constant";
        case ComponentType::Gain: return "Gain";
        case ComponentType::SummingJunction: return "SummingJunction";
        case ComponentType::Product: return "Product";
        case ComponentType::Comparator: return "Comparator";
        case ComponentType::AND_Gate: return "AND_Gate";
        case ComponentType::OR_Gate: return "OR_Gate";
        case ComponentType::NOT_Gate: return "NOT_Gate";
        case ComponentType::CustomFunction: return "CustomFunction";
        case ComponentType::Mux: return "Mux";
        case ComponentType::Demux: return "Demux";
        case ComponentType::CustomScript: return "CustomScript";
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
    } else if (t == "SUM" || t == "SUM_ROUND" || t == "SUM_RECT") {
        Pin in1; in1.name = "In1"; in1.relativeX = -25; in1.relativeY = -15; in1.isInput = true; in1.opSign = "+";
        Pin in2; in2.name = "In2"; in2.relativeX = -25; in2.relativeY = 15; in2.isInput = true; in2.opSign = "-";
        Pin out; out.name = "Out"; out.relativeX = 25; out.relativeY = 0; out.isOutput = true;
        comp.pins = {in1, in2, out};
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
