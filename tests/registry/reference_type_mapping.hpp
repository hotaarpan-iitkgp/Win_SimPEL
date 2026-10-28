// FROZEN ORACLE: CircuitSim::stringToComponentType exactly as it was in
// engine/Components.hpp before the block registry took over the mapping.
// test_registry checks the registry gives the same answer for every string. Do not edit.
#pragma once
#include "engine/Components.hpp"
#include <string>
namespace golden_ref_types {
using namespace CircuitSim;
inline ComponentType legacyStringToComponentType(const std::string& typeStr) {
    if (typeStr == "Resistor" || typeStr == "R") return ComponentType::Resistor;
    if (typeStr == "VariableResistor") return ComponentType::VariableResistor;
    if (typeStr == "Capacitor" || typeStr == "C") return ComponentType::Capacitor;
    if (typeStr == "Inductor" || typeStr == "L") return ComponentType::Inductor;
    if (typeStr == "VoltageSource" || typeStr == "V" || typeStr == "dc" || typeStr == "DC_V") return ComponentType::VoltageSource;
    if (typeStr == "ACVoltageSource" || typeStr == "ac" || typeStr == "AC_V") return ComponentType::ACVoltageSource;
    if (typeStr == "ThreePhaseSource" || typeStr == "V_3PH") return ComponentType::ThreePhaseSource;
    if (typeStr == "ControlledVoltageSource" || typeStr == "CTRL_V") return ComponentType::ControlledVoltageSource;
    if (typeStr == "CurrentSource" || typeStr == "I" || typeStr == "DC_I") return ComponentType::CurrentSource;
    if (typeStr == "ACCurrentSource" || typeStr == "AC_I") return ComponentType::ACCurrentSource;
    if (typeStr == "ControlledCurrentSource" || typeStr == "CTRL_I") return ComponentType::ControlledCurrentSource;
    if (typeStr == "ThreePhaseCurrentSource" || typeStr == "I_3PH") return ComponentType::ThreePhaseCurrentSource;
    if (typeStr == "ElectricalPort" || typeStr == "E_PORT") return ComponentType::ElectricalPort;
    if (typeStr == "ElectricalLabel" || typeStr == "E_LABEL") return ComponentType::ElectricalLabel;
    if (typeStr == "Switch" || typeStr == "S") return ComponentType::Switch;
    if (typeStr == "Breaker" || typeStr == "BREAKER") return ComponentType::Breaker;
    if (typeStr == "DoubleSwitch" || typeStr == "DBL_SWITCH") return ComponentType::DoubleSwitch;
    if (typeStr == "ManualSwitch" || typeStr == "MAN_SWITCH") return ComponentType::ElectricalManualSwitch;
    if (typeStr == "ManualDoubleSwitch" || typeStr == "MAN_DBL_SWITCH") return ComponentType::ManualDoubleSwitch;
    if (typeStr == "ManualTripleSwitch" || typeStr == "MAN_TRPL_SWITCH") return ComponentType::ManualTripleSwitch;
    if (typeStr == "SRSwitch" || typeStr == "SR_SWITCH") return ComponentType::SRSwitch;
    if (typeStr == "TripleSwitch" || typeStr == "TRPL_SWITCH") return ComponentType::TripleSwitch;
    if (typeStr == "Diode" || typeStr == "D") return ComponentType::Diode;
    if (typeStr == "MOSFET" || typeStr == "MOSFET_DIODE") return ComponentType::MOSFET;
    if (typeStr == "vg-FET" || typeStr == "VGFET") return ComponentType::VGFET;
    if (typeStr == "GOTO_SIG" || typeStr == "GOTO" || typeStr == "GotoSignal") return ComponentType::GotoSignal;
    if (typeStr == "FROM_SIG" || typeStr == "FROM" || typeStr == "FromSignal") return ComponentType::FromSignal;
    if (typeStr == "IGBT") return ComponentType::IGBT;
    if (typeStr == "IGBT_DIODE" || typeStr == "IGBTDiode") return ComponentType::IGBTDiode;
    if (typeStr == "IGCT") return ComponentType::IGCT;
    if (typeStr == "GTO") return ComponentType::GTO;
    if (typeStr == "Thyristor" || typeStr == "THYRISTOR" || typeStr == "SCR") return ComponentType::Thyristor;
    if (typeStr == "BJT") return ComponentType::BJT;
    if (typeStr == "JFET") return ComponentType::JFET;
    // ── Magnetic domain blocks ──
    if (typeStr == "WINDING" || typeStr == "Winding") return ComponentType::Winding;
    if (typeStr == "MAG_PERMEANCE" || typeStr == "MagneticPermeance") return ComponentType::MagneticPermeance;
    if (typeStr == "LINEAR_CORE" || typeStr == "LinearCore") return ComponentType::LinearCore;
    if (typeStr == "AIR_GAP" || typeStr == "AirGap") return ComponentType::AirGap;
    if (typeStr == "LEAKAGE_PATH" || typeStr == "LeakageFluxPath") return ComponentType::LeakageFluxPath;
    if (typeStr == "MAG_RESISTANCE" || typeStr == "MagneticResistance") return ComponentType::MagneticResistance;
    if (typeStr == "MMF_SRC" || typeStr == "MMFSource") return ComponentType::MMFSource;
    if (typeStr == "MMF_SRC_CTRL" || typeStr == "MMFSourceControlled") return ComponentType::MMFSourceControlled;

    if (typeStr == "Transformer" || typeStr == "XFMR") return ComponentType::Transformer;
    if (typeStr == "IDEAL_XFMR" || typeStr == "IdealTransformer") return ComponentType::IdealTransformer;
    if (typeStr == "XFMR_2W" || typeStr == "Transformer2W") return ComponentType::Transformer2W;
    if (typeStr == "XFMR_3W" || typeStr == "Transformer3W") return ComponentType::Transformer3W;
    if (typeStr == "MUTUAL_2W" || typeStr == "MutualInductor2W") return ComponentType::MutualInductor2W;
    if (typeStr == "MUTUAL_3W" || typeStr == "MutualInductor3W") return ComponentType::MutualInductor3W;
    if (typeStr == "SAT_XFMR" || typeStr == "SaturableTransformer") return ComponentType::SaturableTransformer;
    if (typeStr == "XFMR_3PH_2W" || typeStr == "Transformer3Ph2W") return ComponentType::Transformer3Ph2W;
    if (typeStr == "XFMR_3PH_3W" || typeStr == "Transformer3Ph3W") return ComponentType::Transformer3Ph3W;
    if (typeStr == "OpAmp" || typeStr == "OPAMP") return ComponentType::OpAmp;
    if (typeStr == "InductionMotor" || typeStr == "INDUCTION_MOTOR" || typeStr == "IND_MOTOR") return ComponentType::InductionMotor;
    if (typeStr == "EComp" || typeStr == "E_COMP") return ComponentType::EComp;
    if (typeStr == "GenEBlock" || typeStr == "GEN_EBLOCK") return ComponentType::GenEBlock;
    if (typeStr == "Voltmeter" || typeStr == "VM") return ComponentType::Voltmeter;
    if (typeStr == "Ammeter" || typeStr == "AM") return ComponentType::Ammeter;
    if (typeStr == "VM_3PH" || typeStr == "Voltmeter3Ph") return ComponentType::Voltmeter3Ph;
    if (typeStr == "AM_3PH" || typeStr == "Ammeter3Ph") return ComponentType::Ammeter3Ph;
    if (typeStr == "VAR_R" || typeStr == "VariableResistor") return ComponentType::VariableResistor;
    if (typeStr == "VAR_L" || typeStr == "VariableInductor") return ComponentType::VariableInductor;
    if (typeStr == "VAR_C" || typeStr == "VariableCapacitor") return ComponentType::VariableCapacitor;
    if (typeStr == "SAT_L" || typeStr == "SaturableInductor") return ComponentType::SaturableInductor;
    if (typeStr == "SAT_C" || typeStr == "SaturableCapacitor") return ComponentType::SaturableCapacitor;
    if (typeStr == "PI_SECTION" || typeStr == "PiSectionLine") return ComponentType::PiSectionLine;
    if (typeStr == "LINE_3PH" || typeStr == "TransmissionLine3Ph") return ComponentType::TransmissionLine3Ph;
    if (typeStr == "PWL_R" || typeStr == "PWLResistor") return ComponentType::PWLResistor;
    if (typeStr == "E_ALGEBRAIC" || typeStr == "ElectricalAlgebraic") return ComponentType::ElectricalAlgebraic;
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
    if (typeStr == "ContinuousPID" || typeStr == "CONT_PID" || typeStr == "PID" || typeStr == "DISCRETE_PID") return ComponentType::ContinuousPID;
    if (typeStr == "SUBTRACT" || typeStr == "SUB") return ComponentType::SummingJunction;
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
    if (typeStr == "LogicOp" || typeStr == "LOGIC_OP") return ComponentType::LogicOp;
    if (typeStr == "BitwiseOp" || typeStr == "BITWISE_OP") return ComponentType::BitwiseOp;
    if (typeStr == "CombLogic" || typeStr == "COMB_LOGIC") return ComponentType::CombLogic;
    if (typeStr == "EdgeDetect" || typeStr == "EDGE_DETECT") return ComponentType::EdgeDetect;
    if (typeStr == "Monostable" || typeStr == "MONOSTABLE") return ComponentType::Monostable;
    if (typeStr == "Monoflop" || typeStr == "MONOFLOP") return ComponentType::Monoflop;
    if (typeStr == "RelationalOp" || typeStr == "RELATIONAL_OPERATOR") return ComponentType::RelationalOp;
    if (typeStr == "CompareToConstant" || typeStr == "COMPARE_TO_CONSTANT" || typeStr == "COMP_CONST") return ComponentType::CompareToConstant;
    if (typeStr == "DFlipFlop" || typeStr == "D_FLIP_FLOP") return ComponentType::DFlipFlop;
    if (typeStr == "JKFlipFlop" || typeStr == "JK_FLIP_FLOP") return ComponentType::JKFlipFlop;
    if (typeStr == "ShiftReg" || typeStr == "SHIFT_REG") return ComponentType::ShiftReg;
    if (typeStr == "PWM_3PH") return ComponentType::PWM_3PH;
    if (typeStr == "SVPWM") return ComponentType::SVPWM;
    if (typeStr == "CLARKE" || typeStr == "Clarke" || typeStr == "CLARKE_TRANSFORM" || typeStr == "CLARKE_TRANS") return ComponentType::Clarke;
    if (typeStr == "PARK" || typeStr == "Park" || typeStr == "PARK_TRANSFORM" || typeStr == "PARK_TRANS") return ComponentType::Park;
    if (typeStr == "INV_CLARKE" || typeStr == "InvClarke" || typeStr == "INV_CLARKE_TRANSFORM" || typeStr == "INV_CLARKE_TRANS") return ComponentType::InvClarke;
    if (typeStr == "INV_PARK" || typeStr == "InvPark" || typeStr == "INV_PARK_TRANSFORM" || typeStr == "INV_PARK_TRANS") return ComponentType::InvPark;
    if (typeStr == "DQ_TO_ABC" || typeStr == "DQ_ABC" || typeStr == "INV_PARK_3PH" || typeStr == "PARK_INV_3PH" || typeStr == "DqToAbc") return ComponentType::DqToAbc;
    if (typeStr == "ABC_TO_DQ" || typeStr == "ABC_DQ" || typeStr == "PARK_3PH" || typeStr == "PARK_3PHASE" || typeStr == "AbcToDq") return ComponentType::AbcToDq;
    if (typeStr == "PER_AVG" || typeStr == "PerAvg") return ComponentType::PerAvg;
    if (typeStr == "PERIODIC_IMP_AVG" || typeStr == "PeriodicImpAvg") return ComponentType::PeriodicImpAvg;
    if (typeStr == "FOURIER_TRANS" || typeStr == "FourierTrans") return ComponentType::FourierTrans;
    if (typeStr == "MOV_AVG" || typeStr == "MovAvg") return ComponentType::MovAvg;
    if (typeStr == "FILTER_1ST" || typeStr == "Filter1st") return ComponentType::Filter1st;
    if (typeStr == "FILTER_2ND" || typeStr == "Filter2nd") return ComponentType::Filter2nd;
    if (typeStr == "FOURIER_ANALYSIS" || typeStr == "FourierAnalysis") return ComponentType::FourierAnalysis;
    if (typeStr == "RMS_VAL" || typeStr == "RmsVal") return ComponentType::RmsVal;
    if (typeStr == "THD_VAL" || typeStr == "ThdVal") return ComponentType::ThdVal;
    if (typeStr == "PLL_LOOP" || typeStr == "PllLoop") return ComponentType::PllLoop;
    if (typeStr == "OFFSET" || typeStr == "Offset") return ComponentType::Offset;
    if (typeStr == "SIGNUM" || typeStr == "Signum" || typeStr == "SIGN" || typeStr == "Sign" || typeStr == "SGN") return ComponentType::Signum;
    if (typeStr == "DIVIDE" || typeStr == "Divide") return ComponentType::Divide;
    if (typeStr == "DATATYPE_CONV" || typeStr == "DataTypeConv") return ComponentType::DataTypeConv;
    if (typeStr == "STATE_MACHINE" || typeStr == "StateMachine") return ComponentType::StateMachine;
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

    if (typeStr == "Subsystem" || typeStr == "SUBSYSTEM") return ComponentType::Subsystem;
    if (typeStr == "Inport" || typeStr == "INPORT" || typeStr == "IN") return ComponentType::Inport;
    if (typeStr == "Outport" || typeStr == "OUTPORT" || typeStr == "OUT") return ComponentType::Outport;
    if (typeStr == "PhysicalInport" || typeStr == "PHYSICAL_INPORT" || typeStr == "PIN") return ComponentType::PhysicalInport;
    if (typeStr == "PhysicalOutport" || typeStr == "PHYSICAL_OUTPORT" || typeStr == "POUT") return ComponentType::PhysicalOutport;
    if (typeStr == "EnablePort" || typeStr == "ENABLE_PORT") return ComponentType::EnablePort;
    if (typeStr == "TriggerPort" || typeStr == "TRIGGER_PORT") return ComponentType::TriggerPort;
    if (typeStr == "BusCreator" || typeStr == "BUS_CREATOR") return ComponentType::BusCreator;
    if (typeStr == "BusSelector" || typeStr == "BUS_SELECTOR") return ComponentType::BusSelector;
    if (typeStr == "Terminator" || typeStr == "TERMINATOR") return ComponentType::Terminator;

    if (typeStr == "Polynomial" || typeStr == "POLYNOMIAL") return ComponentType::Polynomial;
    if (typeStr == "AlgebraicConstraint" || typeStr == "ALGEBRAIC_CONSTRAINT") return ComponentType::AlgebraicConstraint;

    if (typeStr == "LOSS_ANALYZER" || typeStr == "LossAnalyzer") return ComponentType::LossAnalyzer;

    if (typeStr == "DC_MOTOR" || typeStr == "DCMotor") return ComponentType::DCMotor;
    if (typeStr == "BLDC_MOTOR" || typeStr == "BLDCMotor") return ComponentType::BLDCMotor;
    if (typeStr == "PMSM_MOTOR" || typeStr == "PMSMMotor") return ComponentType::PMSMMotor;
    if (typeStr == "SYNC_MOTOR" || typeStr == "SynchronousMotor") return ComponentType::SynchronousMotor;
    if (typeStr == "SRM_MOTOR" || typeStr == "SRMotor") return ComponentType::SRMotor;
    if (typeStr == "UNIT_DELAY" || typeStr == "UnitDelay") return ComponentType::UnitDelay;
    if (typeStr == "ZOH" || typeStr == "ZoH") return ComponentType::ZoH;
    if (typeStr == "DISCRETE_INT" || typeStr == "DiscreteInt") return ComponentType::DiscreteInt;
    if (typeStr == "DISCRETE_MEAN" || typeStr == "DiscreteMean") return ComponentType::DiscreteMean;
    if (typeStr == "DISCRETE_TF" || typeStr == "DiscreteTF") return ComponentType::DiscreteTF;
    if (typeStr == "DISCRETE_SS" || typeStr == "DiscreteSS") return ComponentType::DiscreteSS;
    return ComponentType::Unknown;
}
inline std::vector<std::string> legacyLiterals() {
    return {
        "Resistor",
        "R",
        "VariableResistor",
        "Capacitor",
        "C",
        "Inductor",
        "L",
        "VoltageSource",
        "V",
        "dc",
        "DC_V",
        "ACVoltageSource",
        "ac",
        "AC_V",
        "ThreePhaseSource",
        "V_3PH",
        "ControlledVoltageSource",
        "CTRL_V",
        "CurrentSource",
        "I",
        "DC_I",
        "ACCurrentSource",
        "AC_I",
        "ControlledCurrentSource",
        "CTRL_I",
        "ThreePhaseCurrentSource",
        "I_3PH",
        "ElectricalPort",
        "E_PORT",
        "ElectricalLabel",
        "E_LABEL",
        "Switch",
        "S",
        "Breaker",
        "BREAKER",
        "DoubleSwitch",
        "DBL_SWITCH",
        "ManualSwitch",
        "MAN_SWITCH",
        "ManualDoubleSwitch",
        "MAN_DBL_SWITCH",
        "ManualTripleSwitch",
        "MAN_TRPL_SWITCH",
        "SRSwitch",
        "SR_SWITCH",
        "TripleSwitch",
        "TRPL_SWITCH",
        "Diode",
        "D",
        "MOSFET",
        "MOSFET_DIODE",
        "vg-FET",
        "VGFET",
        "GOTO_SIG",
        "GOTO",
        "GotoSignal",
        "FROM_SIG",
        "FROM",
        "FromSignal",
        "IGBT",
        "IGBT_DIODE",
        "IGBTDiode",
        "IGCT",
        "GTO",
        "Thyristor",
        "THYRISTOR",
        "SCR",
        "BJT",
        "JFET",
        "WINDING",
        "Winding",
        "MAG_PERMEANCE",
        "MagneticPermeance",
        "LINEAR_CORE",
        "LinearCore",
        "AIR_GAP",
        "AirGap",
        "LEAKAGE_PATH",
        "LeakageFluxPath",
        "MAG_RESISTANCE",
        "MagneticResistance",
        "MMF_SRC",
        "MMFSource",
        "MMF_SRC_CTRL",
        "MMFSourceControlled",
        "Transformer",
        "XFMR",
        "IDEAL_XFMR",
        "IdealTransformer",
        "XFMR_2W",
        "Transformer2W",
        "XFMR_3W",
        "Transformer3W",
        "MUTUAL_2W",
        "MutualInductor2W",
        "MUTUAL_3W",
        "MutualInductor3W",
        "SAT_XFMR",
        "SaturableTransformer",
        "XFMR_3PH_2W",
        "Transformer3Ph2W",
        "XFMR_3PH_3W",
        "Transformer3Ph3W",
        "OpAmp",
        "OPAMP",
        "InductionMotor",
        "INDUCTION_MOTOR",
        "IND_MOTOR",
        "DC_MOTOR",
        "BLDC_MOTOR",
        "PMSM_MOTOR",
        "SYNC_MOTOR",
        "SRM_MOTOR",
        "UNIT_DELAY",
        "ZOH",
        "DISCRETE_INT",
        "DISCRETE_MEAN",
        "DISCRETE_TF",
        "DISCRETE_SS",
        "EComp",
        "E_COMP",
        "GenEBlock",
        "GEN_EBLOCK",
        "Voltmeter",
        "VM",
        "Ammeter",
        "AM",
        "VM_3PH",
        "Voltmeter3Ph",
        "AM_3PH",
        "Ammeter3Ph",
        "VAR_R",
        "VAR_L",
        "VariableInductor",
        "VAR_C",
        "VariableCapacitor",
        "SAT_L",
        "SaturableInductor",
        "SAT_C",
        "SaturableCapacitor",
        "PI_SECTION",
        "PiSectionLine",
        "LINE_3PH",
        "TransmissionLine3Ph",
        "PWL_R",
        "PWLResistor",
        "E_ALGEBRAIC",
        "ElectricalAlgebraic",
        "Oscilloscope",
        "SCOPE",
        "UnifiedProbe",
        "PROBE",
        "PI_Controller",
        "PID",
        "PWM_Generator",
        "PWM",
        "MasterPWM",
        "PWM_MASTER",
        "Triangle_Carrier",
        "TRI",
        "TRI_GEN",
        "PulseGenerator",
        "PULSE",
        "PULSE_GEN",
        "EdgeDetector",
        "EDGE_DETECT",
        "Constant",
        "CONST",
        "Clock",
        "CLOCK",
        "InitialCondition",
        "INIT_COND",
        "Ramp",
        "RAMP",
        "RandomNumbers",
        "RANDOM_NUM",
        "SineWave",
        "SINE_WAVE",
        "Step",
        "STEP",
        "WhiteNoise",
        "WHITE_NOISE",
        "TrigFunction",
        "TRIG_FCN",
        "Abs",
        "ABS",
        "Sign",
        "SIGN",
        "Integrator",
        "INTEGRATOR",
        "Derivative",
        "DERIVATIVE",
        "TransferFunction",
        "TRANSFER_FCN",
        "StateSpace",
        "STATE_SPACE",
        "ContinuousPID",
        "CONT_PID",
        "DISCRETE_PID",
        "SUBTRACT",
        "SUB",
        "PLL_1PH",
        "PLL_3PH",
        "Delay",
        "DELAY",
        "TransportDelay",
        "TRANSPORT_DELAY",
        "TurnOnDelay",
        "TURN_ON_DELAY",
        "MemoryBlock",
        "MEMORY",
        "MEMORY_BLOCK",
        "Quantizer",
        "QUANTIZER",
        "SignalSwitch",
        "SIGNAL_SWITCH",
        "MANUAL_SWITCH",
        "MultiportSwitch",
        "MULTIPORT_SWITCH",
        "HitCrossing",
        "HIT_CROSSING",
        "Saturation",
        "SATURATION",
        "DeadZone",
        "DEAD_ZONE",
        "RateLimiter",
        "RATE_LIMITER",
        "Relay",
        "RELAY",
        "Comparator",
        "COMP",
        "LogicOp",
        "LOGIC_OP",
        "BitwiseOp",
        "BITWISE_OP",
        "CombLogic",
        "COMB_LOGIC",
        "EdgeDetect",
        "Monostable",
        "MONOSTABLE",
        "Monoflop",
        "MONOFLOP",
        "RelationalOp",
        "RELATIONAL_OPERATOR",
        "CompareToConstant",
        "COMPARE_TO_CONSTANT",
        "COMP_CONST",
        "DFlipFlop",
        "D_FLIP_FLOP",
        "JKFlipFlop",
        "JK_FLIP_FLOP",
        "ShiftReg",
        "SHIFT_REG",
        "PWM_3PH",
        "SVPWM",
        "CLARKE",
        "Clarke",
        "CLARKE_TRANSFORM",
        "CLARKE_TRANS",
        "PARK",
        "Park",
        "PARK_TRANSFORM",
        "PARK_TRANS",
        "INV_CLARKE",
        "InvClarke",
        "INV_CLARKE_TRANSFORM",
        "INV_CLARKE_TRANS",
        "INV_PARK",
        "InvPark",
        "INV_PARK_TRANSFORM",
        "INV_PARK_TRANS",
        "DQ_TO_ABC",
        "DQ_ABC",
        "INV_PARK_3PH",
        "PARK_INV_3PH",
        "DqToAbc",
        "ABC_TO_DQ",
        "ABC_DQ",
        "PARK_3PH",
        "PARK_3PHASE",
        "AbcToDq",
        "PER_AVG",
        "PerAvg",
        "PERIODIC_IMP_AVG",
        "PeriodicImpAvg",
        "FOURIER_TRANS",
        "FourierTrans",
        "MOV_AVG",
        "MovAvg",
        "FILTER_1ST",
        "Filter1st",
        "FILTER_2ND",
        "Filter2nd",
        "FOURIER_ANALYSIS",
        "FourierAnalysis",
        "RMS_VAL",
        "RmsVal",
        "THD_VAL",
        "ThdVal",
        "PLL_LOOP",
        "PllLoop",
        "OFFSET",
        "Offset",
        "SIGNUM",
        "Signum",
        "SGN",
        "DIVIDE",
        "Divide",
        "DATATYPE_CONV",
        "DataTypeConv",
        "STATE_MACHINE",
        "StateMachine",
        "Round",
        "ROUND",
        "MinMax",
        "MIN_MAX",
        "LUT_1D",
        "LUT_2D",
        "LUT_3D",
        "DLL",
        "FMU",
        "FourierSeries",
        "FOURIER_SERIES",
        "Gain",
        "GAIN",
        "SummingJunction",
        "SUM",
        "SUM_RECT",
        "SUM_ROUND",
        "Product",
        "PROD",
        "PRODUCT_RECT",
        "MathFunction",
        "MATH_FCN",
        "AND_Gate",
        "AND",
        "OR_Gate",
        "OR",
        "NOT_Gate",
        "NOT",
        "CustomFunction",
        "FCN",
        "Mux",
        "Demux",
        "CustomScript",
        "CSCRIPT",
        "KeyTrigger",
        "KEY_TRIGGER",
        "Goto",
        "From",
        "Subsystem",
        "SUBSYSTEM",
        "Inport",
        "INPORT",
        "IN",
        "Outport",
        "OUTPORT",
        "OUT",
        "PhysicalInport",
        "PHYSICAL_INPORT",
        "PIN",
        "PhysicalOutport",
        "PHYSICAL_OUTPORT",
        "POUT",
        "EnablePort",
        "ENABLE_PORT",
        "TriggerPort",
        "TRIGGER_PORT",
        "BusCreator",
        "BUS_CREATOR",
        "BusSelector",
        "BUS_SELECTOR",
        "Terminator",
        "TERMINATOR",
        "Polynomial",
        "POLYNOMIAL",
        "AlgebraicConstraint",
        "ALGEBRAIC_CONSTRAINT",
        "LOSS_ANALYZER",
        "LossAnalyzer",
    };
}
} // namespace golden_ref_types
