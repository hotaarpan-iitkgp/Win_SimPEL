// FROZEN ORACLE: the component palette table exactly as it was in
// ui/MainWindow.cpp renderComponentPalette() before the block registry replaced it.
// test_registry checks the registry reproduces every entry, in order. Do not edit.
#pragma once
#include "engine/Components.hpp"
#include <string>
#include <utility>
#include <vector>
namespace golden_ref_palette {
using namespace CircuitSim;
inline const auto& table() {
        struct ComponentMeta {
            const char* buttonText;
            const char* label;
            const char* prefix;
            ComponentType type;
            const char* rawTypeStr;
            const char* category;       // "general", "control", "electrical"
            const char* subcategory;    // Subheading matching webtool
            std::vector<std::pair<std::string, std::string>> defaultParams;
            bool isBasic = false;       // Only original basic library blocks are true
        };

        static const std::vector<ComponentMeta> allComponents = {
            // General Domain
            // General Domain - Ports and Subsystems
            { "Subsystem Block", "Subsystem", "SUBSYSTEM", ComponentType::Subsystem, "SUBSYSTEM", "general", "Ports and Subsystems", {}, true },
            { "Inport (INPORT)", "Inport", "INPORT", ComponentType::Inport, "INPORT", "general", "Ports and Subsystems", {{"port_number", "1"}}, true },
            { "Outport (OUTPORT)", "Outport", "OUTPORT", ComponentType::Outport, "OUTPORT", "general", "Ports and Subsystems", {{"port_number", "1"}}, true },
            { "Physical Inport (PIN)", "Phys Inport", "PIN", ComponentType::PhysicalInport, "PIN", "general", "Ports and Subsystems", {{"port_number", "1"}}, true },
            { "Physical Outport (POUT)", "Phys Outport", "POUT", ComponentType::PhysicalOutport, "POUT", "general", "Ports and Subsystems", {{"port_number", "1"}}, true },
            { "Enable Port (ENABLE_PORT)", "Enable", "ENABLE_PORT", ComponentType::EnablePort, "ENABLE_PORT", "general", "Ports and Subsystems", {{"states_when_disabled", "held"}}, true },
            { "Trigger Port (TRIGGER_PORT)", "Trigger", "TRIGGER_PORT", ComponentType::TriggerPort, "TRIGGER_PORT", "general", "Ports and Subsystems", {{"trigger_type", "rising"}}, true },
            { "Bus Creator (BUS_CREATOR)", "Bus Creator", "BUS_CREATOR", ComponentType::BusCreator, "BUS_CREATOR", "general", "Ports and Subsystems", {{"inputs", "2"}}, true },
            { "Bus Selector (BUS_SELECTOR)", "Bus Selector", "BUS_SELECTOR", ComponentType::BusSelector, "BUS_SELECTOR", "general", "Ports and Subsystems", {{"signals", "signal1,signal2"}}, true },
            { "Terminator (TERMINATOR)", "Terminator", "TERMINATOR", ComponentType::Terminator, "TERMINATOR", "general", "Ports and Subsystems", {}, true },

            { "Active Probe (PROBE)", "Active Probe", "PROBE", ComponentType::Unknown, "PROBE", "general", "Signal Routing", {{"target", ""}, {"selected_signals", ""}}, true },
            { "Oscilloscope (SCOPE)", "Oscilloscope", "SCOPE", ComponentType::Unknown, "SCOPE", "general", "Visualization & Logging", {{"channels", "2"}}, true },
            { "Loss Analyzer (LOSS_ANALYZER)", "Loss Analyzer", "LOSS_ANALYZER", ComponentType::LossAnalyzer, "LOSS_ANALYZER", "general", "Visualization & Logging", {{"start_time", "0.0"}, {"duration", "0.02"}, {"tj", "125.0"}}, true },

            // Control Domain (Sources)
            { "Constant (CONST)", "Constant", "CONST", ComponentType::Constant, "CONST", "control", "Sources", {{"value", "1.0"}}, true },
            { "Clock (CLOCK)", "Clock", "CLOCK", ComponentType::Clock, "CLOCK", "control", "Sources", {}, false },
            { "Initial Condition (INIT_COND)", "Initial Condition", "INIT_COND", ComponentType::InitialCondition, "INIT_COND", "control", "Sources", {{"initial_value", "0"}}, false },
            { "Pulse Generator (PULSE_GEN)", "Pulse Gen", "PULSE_GEN", ComponentType::PulseGenerator, "PULSE_GEN", "control", "Sources", {{"amplitude", "1"}, {"period", "1"}, {"width", "0.5"}, {"delay", "0"}}, true },
            { "Ramp (RAMP)", "Ramp", "RAMP", ComponentType::Ramp, "RAMP", "control", "Sources", {{"slope", "1"}, {"start_time", "0"}, {"initial_output", "0"}}, false },
            { "Random Numbers (RANDOM_NUM)", "Random Numbers", "RANDOM_NUM", ComponentType::RandomNumbers, "RANDOM_NUM", "control", "Sources", {{"mean", "0"}, {"std", "1"}}, false },
            { "Sine Wave (SINE_WAVE)", "Sine Wave", "SINE_WAVE", ComponentType::SineWave, "SINE_WAVE", "control", "Sources", {{"amplitude", "1"}, {"frequency", "50"}, {"phase", "0"}}, false },
            { "Step (STEP)", "Step", "STEP", ComponentType::Step, "STEP", "control", "Sources", {{"step_time", "1"}, {"initial_value", "0"}, {"final_value", "1"}}, false },
            { "Triangular Wave Generator (TRI_GEN)", "Triangle", "TRI", ComponentType::Triangle_Carrier, "TRI_GEN", "control", "Sources", {{"frequency", "10k"}, {"min", "0"}, {"max", "1"}}, true },
            { "White Noise (WHITE_NOISE)", "White Noise", "WHITE_NOISE", ComponentType::WhiteNoise, "WHITE_NOISE", "control", "Sources", {{"psd", "0.1"}}, false },

            // Functions & Tables
            { "Gain Scalar (GAIN)", "Gain", "GAIN", ComponentType::Gain, "GAIN", "control", "Functions & Tables", {{"K", "2.5"}}, true },
            { "Sum (SUM_RECT)", "Sum", "SUM_RECT", ComponentType::SummingJunction, "SUM_RECT", "control", "Functions & Tables", {{"inputs", "++"}}, true },
            { "Product (PROD)", "Product", "PROD", ComponentType::Product, "PRODUCT_RECT", "control", "Functions & Tables", {{"operators", "*/"}}, true },
            { "Trigonometric Function (TRIG_FCN)", "Trig", "TRIG_FCN", ComponentType::TrigFunction, "TRIG_FCN", "control", "Functions & Tables", {{"function", "sin"}}, false },
            { "Math Function (MATH_FCN)", "Math Function", "MATH_FCN", ComponentType::MathFunction, "MATH_FCN", "control", "Functions & Tables", {{"function", "exp"}}, true },
            { "Abs (ABS)", "Abs", "ABS", ComponentType::Abs, "ABS", "control", "Functions & Tables", {}, false },
            { "Sign (SIGN)", "Sign", "SIGN", ComponentType::Sign, "SIGN", "control", "Functions & Tables", {}, false },
            { "Round (ROUND)", "Round", "ROUND", ComponentType::Round, "ROUND", "control", "Functions & Tables", {{"mode", "nearest"}}, false },
            { "Min/Max (MIN_MAX)", "MinMax", "MIN_MAX", ComponentType::MinMax, "MIN_MAX", "control", "Functions & Tables", {{"function", "min"}, {"num_inputs", "2"}}, false },
            { "Polynomial (POLYNOMIAL)", "Polynomial", "POLYNOMIAL", ComponentType::Polynomial, "POLYNOMIAL", "control", "Functions & Tables", {{"coefficients", "[1, 0]"}}, true },
            { "1D Look-Up Table (LUT_1D)", "LUT 1D", "LUT_1D", ComponentType::LUT_1D, "LUT_1D", "control", "Functions & Tables", {{"x_data", "[0, 1, 2]"}, {"y_data", "[0, 2, 4]"}}, false },
            { "2D Look-Up Table (LUT_2D)", "LUT 2D", "LUT_2D", ComponentType::LUT_2D, "LUT_2D", "control", "Functions & Tables", {{"x_data", "[0, 1]"}, {"y_data", "[0, 1]"}, {"z_data", "[[0, 1], [1, 2]]"}}, false },
            { "3D Look-Up Table (LUT_3D)", "LUT 3D", "LUT_3D", ComponentType::LUT_3D, "LUT_3D", "control", "Functions & Tables", {}, false },
            { "C-Script (CSCRIPT)", "C-Script", "CSCRIPT", ComponentType::CustomScript, "CSCRIPT", "control", "Functions & Tables", {{"code", "// Step code\noutputs[0] = inputs[0] * 2.0;\n"}}, true },
            { "Algebraic Constraint (ALGEBRAIC_CONSTRAINT)", "Alg Constraint", "ALGEBRAIC_CONSTRAINT", ComponentType::AlgebraicConstraint, "ALGEBRAIC_CONSTRAINT", "control", "Functions & Tables", {{"initial_guess", "0.0"}}, true },
            { "DLL (DLL)", "DLL", "DLL", ComponentType::DLL, "DLL", "control", "Functions & Tables", {}, false },
            { "FMU (FMU)", "FMU", "FMU", ComponentType::FMU, "FMU", "control", "Functions & Tables", {}, false },
            { "Fourier Series (FOURIER_SERIES)", "Fourier", "FOURIER_SERIES", ComponentType::FourierSeries, "FOURIER_SERIES", "control", "Functions & Tables", {}, false },

            // Continuous
            { "Integrator (INTEGRATOR)", "1/s", "INTEGRATOR", ComponentType::Integrator, "INTEGRATOR", "control", "Continuous", {{"initial_condition", "0"}}, false },
            { "Derivative (DERIVATIVE)", "s", "DERIVATIVE", ComponentType::Derivative, "DERIVATIVE", "control", "Continuous", {}, false },
            { "Transfer Function (TRANSFER_FCN)", "G(s)", "TRANSFER_FCN", ComponentType::TransferFunction, "TRANSFER_FCN", "control", "Continuous", {{"num", "[1]"}, {"den", "[1, 1]"}}, false },
            { "State Space (STATE_SPACE)", "State Space", "STATE_SPACE", ComponentType::StateSpace, "STATE_SPACE", "control", "Continuous", {{"A", "[-1]"}, {"B", "[1]"}, {"C", "[1]"}, {"D", "[0]"}, {"x0", "0"}}, false },
            { "PI Controller (PID)", "PID", "PID", ComponentType::PI_Controller, "PID", "control", "Continuous", {{"Kp", "2.5"}, {"Ki", "50.0"}, {"Kd", "0"}}, true },
            { "Continuous PID Controller (CONT_PID)", "PID(s)", "CONT_PID", ComponentType::ContinuousPID, "CONT_PID", "control", "Continuous", {{"Kp", "1.0"}, {"Ki", "0.0"}, {"Kd", "0.0"}, {"Tf", "0.01"}}, false },
            { "Single-Phase PLL (PLL_1PH)", "PLL 1Ph", "PLL_1PH", ComponentType::PLL_1PH, "PLL_1PH", "control", "Continuous", {{"fn", "50.0"}, {"Kp", "20.0"}, {"Ki", "1000.0"}}, false },
            // Delays
            { "Delay (DELAY)", "Delay", "DELAY", ComponentType::Delay, "DELAY", "control", "Delays", {{"delay", "0.1"}}, false },
            { "Transport Delay (TRANSPORT_DELAY)", "e^-sT", "TRANSPORT_DELAY", ComponentType::TransportDelay, "TRANSPORT_DELAY", "control", "Delays", {{"delay", "0.1"}}, false },
            { "Turn-on Delay (TURN_ON_DELAY)", "Ton", "TURN_ON_DELAY", ComponentType::TurnOnDelay, "TURN_ON_DELAY", "control", "Delays", {{"delay", "0.05"}}, false },
            { "Memory (MEMORY_BLOCK)", "Mem", "MEMORY_BLOCK", ComponentType::MemoryBlock, "MEMORY_BLOCK", "control", "Delays", {{"initial_value", "0.0"}}, false },

            // Discontinuous
            { "Quantizer (QUANTIZER)", "Quantize", "QUANTIZER", ComponentType::Quantizer, "QUANTIZER", "control", "Discontinuous", {{"step_size", "0.5"}, {"mode", "round"}}, false },
            { "Signal Switch (SIGNAL_SWITCH)", "Switch", "SIGNAL_SWITCH", ComponentType::SignalSwitch, "SIGNAL_SWITCH", "control", "Discontinuous", {{"threshold", "0.5"}, {"criteria", "u2 >= threshold"}}, false },
            { "Manual Signal Switch (MANUAL_SWITCH)", "Manual Sw", "MANUAL_SWITCH", ComponentType::ManualSwitch, "MANUAL_SWITCH", "control", "Discontinuous", {{"state", "Input 1"}}, false },
            { "Multiport Signal Switch (MULTIPORT_SWITCH)", "Multiport Sw", "MULTIPORT_SWITCH", ComponentType::MultiportSwitch, "MULTIPORT_SWITCH", "control", "Discontinuous", {{"inputs", "3"}, {"indexing", "1-based"}}, false },
            { "Hit Crossing (HIT_CROSSING)", "Hit Cross", "HIT_CROSSING", ComponentType::HitCrossing, "HIT_CROSSING", "control", "Discontinuous", {{"offset", "0.0"}, {"direction", "either"}}, false },
            { "Saturation (SATURATION)", "Sat", "SATURATION", ComponentType::Saturation, "SATURATION", "control", "Discontinuous", {{"min", "-10"}, {"max", "10"}}, false },
            { "Dead Zone (DEAD_ZONE)", "Dead", "DEAD_ZONE", ComponentType::DeadZone, "DEAD_ZONE", "control", "Discontinuous", {{"start", "-0.5"}, {"end", "0.5"}}, false },
            { "Rate Limiter (RATE_LIMITER)", "Rate", "RATE_LIMITER", ComponentType::RateLimiter, "RATE_LIMITER", "control", "Discontinuous", {{"up", "10"}, {"down", "-10"}}, false },
            { "Relay (RELAY)", "Relay", "RELAY", ComponentType::Relay, "RELAY", "control", "Discontinuous", {{"on_threshold", "1"}, {"off_threshold", "-1"}}, false },
            { "Comparator (COMP)", "COMP", "COMP", ComponentType::Comparator, "COMP", "control", "Discontinuous", {}, true },

            // Logical & Bitwise sub-library
            { "AND Gate (AND)", "AND", "AND", ComponentType::AND_Gate, "AND", "control", "Logical & Bitwise", {{"inputs", "2"}}, true },
            { "OR Gate (OR)", "OR", "OR", ComponentType::OR_Gate, "OR", "control", "Logical & Bitwise", {{"inputs", "2"}}, true },
            { "NOT Gate (NOT)", "NOT", "NOT", ComponentType::NOT_Gate, "NOT", "control", "Logical & Bitwise", {}, true },
            { "NAND Gate (NAND)", "NAND", "NAND", ComponentType::AND_Gate, "NAND", "control", "Logical & Bitwise", {{"inputs", "2"}}, false },
            { "NOR Gate (NOR)", "NOR", "NOR", ComponentType::OR_Gate, "NOR", "control", "Logical & Bitwise", {{"inputs", "2"}}, false },
            { "XOR Gate (XOR)", "XOR", "XOR", ComponentType::LogicOp, "XOR", "control", "Logical & Bitwise", {{"inputs", "2"}}, false },
            { "XNOR Gate (XNOR)", "XNOR", "XNOR", ComponentType::LogicOp, "XNOR", "control", "Logical & Bitwise", {{"inputs", "2"}}, false },
            { "Compare To Constant (COMP_CONST)", "Compare", "COMP_CONST", ComponentType::CompareToConstant, "COMP_CONST", "control", "Logical & Bitwise", {{"operator", "=="}, {"const", "0"}}, false },
            // Compares two live signals (In1 vs In2) rather than a signal against a
            // fixed constant. Solver reads the comparison from the "operator" param.
            { "Relational Operator (RELATIONAL_OPERATOR)", "RelOp", "RELOP", ComponentType::RelationalOp, "RELATIONAL_OPERATOR", "control", "Logical & Bitwise", {{"operator", "<"}}, false },

            // Modulators sub-library
            { "PWM Generator (PWM)", "PWM", "PWM", ComponentType::PWM_Generator, "PWM", "control", "Modulators", {{"frequency", "10k"}, {"min", "0"}, {"max", "1"}}, true },
            { "Master PWM (PWM_MASTER)", "Master PWM", "PWM_MASTER", ComponentType::MasterPWM, "PWM_MASTER", "control", "Modulators", {{"frequency", "10k"}, {"min", "0"}, {"max", "1"}}, true },
            { "Edge Detector (EDGE_DETECT)", "Edge", "EDGE_DETECT", ComponentType::EdgeDetect, "EDGE_DETECT", "control", "Modulators", {{"edge_type", "rising"}}, true },

            // Signal Transforms sub-library
            { "ABC to dq Transform (PARK_TRANSFORM)", "αβ-dq", "PARK_TRANSFORM", ComponentType::Park, "PARK_TRANSFORM", "control", "Signal Transforms", {}, false },
            { "dq to ABC Transform (INV_PARK_TRANSFORM)", "dq-αβ", "INV_PARK_TRANSFORM", ComponentType::InvPark, "INV_PARK_TRANSFORM", "control", "Signal Transforms", {}, false },
            { "Clarke Transform (CLARKE_TRANSFORM)", "abc-αβ", "CLARKE_TRANSFORM", ComponentType::Clarke, "CLARKE_TRANSFORM", "control", "Signal Transforms", {}, false },
            { "Inverse Clarke Transform (INV_CLARKE)", "αβ-abc", "INV_CLARKE", ComponentType::InvClarke, "INV_CLARKE", "control", "Signal Transforms", {}, false },
            { "Park Transform (PARK)", "abc-dq", "PARK", ComponentType::Park, "PARK", "control", "Signal Transforms", {}, false },
            { "Inverse Park Transform (INV_PARK)", "dq-αβ", "INV_PARK", ComponentType::InvPark, "INV_PARK", "control", "Signal Transforms", {}, false },

            // Filters & Measurements sub-library
            { "Periodic Average (PER_AVG)", "Avg T", "PER_AVG", ComponentType::PerAvg, "PER_AVG", "control", "Filters & Measurements", {{"period", "0.02"}}, false },
            { "Periodic Impulse Average (PERIODIC_IMP_AVG)", "Imp Avg", "PERIODIC_IMP_AVG", ComponentType::PeriodicImpAvg, "PERIODIC_IMP_AVG", "control", "Filters & Measurements", {{"initial_value", "0.0"}}, false },
            { "Fourier Transform (FOURIER_TRANS)", "Fourier", "FOURIER_TRANS", ComponentType::FourierTrans, "FOURIER_TRANS", "control", "Filters & Measurements", {{"f", "50.0"}, {"harmonic", "1"}, {"ts", "100u"}}, false },
            { "Moving Average (MOV_AVG)", "MovAvg", "MOV_AVG", ComponentType::MovAvg, "MOV_AVG", "control", "Filters & Measurements", {{"window", "10"}}, false },
            { "First-Order Filter (FILTER_1ST)", "Filt1st", "FILTER_1ST", ComponentType::Filter1st, "FILTER_1ST", "control", "Filters & Measurements", {{"type", "Lowpass"}, {"fc", "1k"}}, false },
            { "Second-Order Filter (FILTER_2ND)", "Filt2nd", "FILTER_2ND", ComponentType::Filter2nd, "FILTER_2ND", "control", "Filters & Measurements", {{"type", "Lowpass"}, {"fc", "1k"}, {"Q", "0.707"}}, false },
            { "Fourier Analysis (FOURIER_ANALYSIS)", "Fourier", "FOURIER_ANALYSIS", ComponentType::FourierAnalysis, "FOURIER_ANALYSIS", "control", "Filters & Measurements", {{"f", "50.0"}}, false },
            { "RMS (RMS_VAL)", "RMS", "RMS_VAL", ComponentType::RmsVal, "RMS_VAL", "control", "Filters & Measurements", {{"frequency", "50"}}, false },
            { "THD (THD_VAL)", "THD", "THD_VAL", ComponentType::ThdVal, "THD_VAL", "control", "Filters & Measurements", {{"frequency", "50"}}, false },
            { "PLL (Phase-Locked Loop) (PLL_LOOP)", "PLL", "PLL_LOOP", ComponentType::PllLoop, "PLL_LOOP", "control", "Filters & Measurements", {{"fn", "50.0"}, {"Kp", "20.0"}, {"Ki", "1000.0"}}, false },

            // State Machines sub-library
            { "State Machine (STATE_MACHINE)", "State", "STATE_MACHINE", ComponentType::StateMachine, "STATE_MACHINE", "control", "State Machines", {}, false },

            // Math sub-library
            { "Offset (OFFSET)", "OFFSET", "OFFSET", ComponentType::Offset, "OFFSET", "control", "Math", {{"offset", "0.0"}}, false },
            { "Summing Block (SUM)", "SUM", "SUM_RECT", ComponentType::SummingJunction, "SUM_RECT", "control", "Math", {{"signs", "++"}}, false },
            { "Sum (round) (SUM_ROUND)", "SUM", "SUM_ROUND", ComponentType::SummingJunction, "SUM_ROUND", "control", "Math", {{"inputs", "2"}, {"signs", "++"}}, true },
            { "Sum (rectangular) (SUM_RECT)", "SUM", "SUM_RECT", ComponentType::SummingJunction, "SUM_RECT", "control", "Math", {{"inputs", "2"}, {"signs", "++"}}, true },
            { "Subtract (SUBTRACT)", "SUB", "SUBTRACT", ComponentType::SummingJunction, "SUBTRACT", "control", "Math", {{"signs", "+-"}}, false },
            { "Product (rectangular) (PRODUCT_RECT)", "PROD", "PRODUCT_RECT", ComponentType::Product, "PRODUCT_RECT", "control", "Math", {{"inputs", "2"}, {"operators", "**"}}, true },
            { "Signum (SIGNUM)", "sgn", "SIGNUM", ComponentType::Signum, "SIGNUM", "control", "Math", {}, false },
            { "Divide (DIVIDE)", "DIV", "DIVIDE", ComponentType::Divide, "DIVIDE", "control", "Math", {{"operators", "*/"}}, false },
            { "Data Type Conversion (DATATYPE_CONV)", "Cast", "DATATYPE_CONV", ComponentType::DataTypeConv, "DATATYPE_CONV", "control", "Math", {{"datatype", "boolean"}}, false },

            // Electrical Domain Connectivity sub-library
            { "Electrical Port (E_PORT)", "E-Port", "E_PORT", ComponentType::ElectricalPort, "E_PORT", "electrical", "Connectivity", {}, false },
            { "Electrical Label (E_LABEL)", "E-Tag", "E_LABEL", ComponentType::ElectricalLabel, "E_LABEL", "electrical", "Connectivity", {{"label", "A"}}, false },
            { "Ground (GND)", "GND", "GND", ComponentType::Unknown, "GND", "electrical", "Connectivity", {}, true },

            // Electrical Domain Sources sub-library
            { "DC Voltage Source (V)", "DC_V", "V", ComponentType::VoltageSource, "V", "electrical", "Sources", {{"value", "24"}}, true },
            { "DC Current Source (I)", "DC_I", "I", ComponentType::CurrentSource, "I", "electrical", "Sources", {{"value", "1"}}, true },
            { "AC Voltage Source (AC_V)", "AC_V", "AC_V", ComponentType::ACVoltageSource, "AC_V", "electrical", "Sources", {{"amplitude", "12"}, {"frequency", "50"}, {"phase", "0"}}, true },
            { "AC Current Source (AC_I)", "AC_I", "AC_I", ComponentType::ACCurrentSource, "AC_I", "electrical", "Sources", {{"amplitude", "1"}, {"frequency", "50"}, {"phase", "0"}}, false },
            { "Controlled Voltage Source (CTRL_V)", "Src V", "CTRL_V", ComponentType::ControlledVoltageSource, "CTRL_V", "electrical", "Sources", {}, false },
            { "Controlled Current Source (CTRL_I)", "Src I", "CTRL_I", ComponentType::ControlledCurrentSource, "CTRL_I", "electrical", "Sources", {}, false },
            { "3-Phase Voltage Source (V_3PH)", "3Ph V", "V3PH", ComponentType::ThreePhaseSource, "V_3PH", "electrical", "Sources", {{"magnitude", "230"}, {"frequency", "50"}, {"phase", "0"}, {"connection", "Y"}}, true },
            { "3-Phase Current Source (I_3PH)", "3Ph I", "I3PH", ComponentType::ThreePhaseCurrentSource, "I_3PH", "electrical", "Sources", {}, false },

            // Electrical Domain Meters (Sensors) sub-library
            { "Voltmeter (VM)", "VM", "VM", ComponentType::Voltmeter, "VM", "electrical", "Meters (Sensors)", {}, true },
            { "Ammeter (AM)", "AM", "AM", ComponentType::Ammeter, "AM", "electrical", "Meters (Sensors)", {}, true },
            { "Voltage Meter (3-Phase) (VM_3PH)", "3VM", "VM_3PH", ComponentType::Voltmeter3Ph, "VM_3PH", "electrical", "Meters (Sensors)", {}, false },
            { "Current Meter (3-Phase) (AM_3PH)", "3AM", "AM_3PH", ComponentType::Ammeter3Ph, "AM_3PH", "electrical", "Meters (Sensors)", {}, false },

            // Electrical Domain Passive Components sub-library
            { "Resistor (R)", "R", "R", ComponentType::Resistor, "R", "electrical", "Passive Components", {{"value", "10"}}, true },
            { "Inductor (L)", "L", "L", ComponentType::Inductor, "L", "electrical", "Passive Components", {{"L", "10m"}, {"esr", "0"}, {"iL0", "0"}}, true },
            { "Capacitor (C)", "C", "C", ComponentType::Capacitor, "C", "electrical", "Passive Components", {{"C", "100u"}, {"esr", "0"}, {"vC0", "0"}}, true },
            { "Variable Resistor (VAR_R)", "var R", "VAR_R", ComponentType::VariableResistor, "VAR_R", "electrical", "Passive Components", {{"value", "10"}}, false },
            { "Variable Inductor (VAR_L)", "var L", "VAR_L", ComponentType::VariableInductor, "VAR_L", "electrical", "Passive Components", {{"L", "10m"}, {"iL0", "0"}}, false },
            { "Variable Capacitor (VAR_C)", "var C", "VAR_C", ComponentType::VariableCapacitor, "VAR_C", "electrical", "Passive Components", {{"C", "100u"}, {"vC0", "0"}}, false },
            { "Saturable Inductor (SAT_L)", "sat L", "SAT_L", ComponentType::SaturableInductor, "SAT_L", "electrical", "Passive Components", {{"L", "10m"}, {"iL0", "0"}}, false },
            { "Saturable Capacitor (SAT_C)", "sat C", "SAT_C", ComponentType::SaturableCapacitor, "SAT_C", "electrical", "Passive Components", {{"C", "100u"}, {"vC0", "0"}}, false },
            { "Pi-Section Line (PI_SECTION)", "Pi Line", "PI_SECTION", ComponentType::PiSectionLine, "PI_SECTION", "electrical", "Passive Components", {}, false },
            { "Transmission Line (3ph) (LINE_3PH)", "3Ph Line", "LINE_3PH", ComponentType::TransmissionLine3Ph, "LINE_3PH", "electrical", "Passive Components", {}, false },
            { "Piece-wise Linear Resistor (PWL_R)", "pwl R", "PWL_R", ComponentType::PWLResistor, "PWL_R", "electrical", "Passive Components", {{"value", "10"}}, false },
            { "Electrical Algebraic Component (E_ALGEBRAIC)", "E-Alg", "E_ALGEBRAIC", ComponentType::ElectricalAlgebraic, "E_ALGEBRAIC", "electrical", "Passive Components", {}, false },

            // Signals Routing sub-library
            { "Signal Goto (GOTO_SIG)", "Goto", "GOTO_SIG", ComponentType::GotoSignal, "GOTO_SIG", "control", "Signals Routing", {{"tag", "S1"}}, false },
            { "Signal From (FROM_SIG)", "From", "FROM_SIG", ComponentType::FromSignal, "FROM_SIG", "control", "Signals Routing", {{"tag", "S1"}}, false },

            // Electrical Domain Power Semiconductors (Ideal Behavioral Switches) sub-library
            { "Diode (D)", "D", "D", ComponentType::Diode, "D", "electrical", "Power Semiconductors (Ideal Behavioral Switches)", {{"Vd", "0.7"}, {"Ron", "1m"}, {"Roff", "1M"}}, true },
            { "Thyristor (THYRISTOR)", "SCR", "THYRISTOR", ComponentType::Thyristor, "THYRISTOR", "electrical", "Power Semiconductors (Ideal Behavioral Switches)", {{"Vd", "0.8"}, {"Ron", "1m"}, {"Roff", "1M"}, {"Iholding", "10m"}, {"Vgt", "0.5"}}, true },
            { "GTO (Gate Turn-Off) (GTO)", "GTO", "GTO", ComponentType::GTO, "GTO", "electrical", "Power Semiconductors (Ideal Behavioral Switches)", {{"Vd", "0.8"}, {"Ron", "1m"}, {"Roff", "1M"}}, false },
            { "IGBT (IGBT)", "IGBT", "IGBT", ComponentType::Switch, "IGBT", "electrical", "Power Semiconductors (Ideal Behavioral Switches)", {{"Ron", "10m"}, {"Roff", "1M"}}, true },
            { "IGBT with Diode (IGBT_DIODE)", "IGBT+D", "IGBT_DIODE", ComponentType::IGBTDiode, "IGBT_DIODE", "electrical", "Power Semiconductors (Ideal Behavioral Switches)", {{"Ron", "10m"}, {"Roff", "1M"}}, false },
            { "IGCT (IGCT)", "IGCT", "IGCT", ComponentType::IGCT, "IGCT", "electrical", "Power Semiconductors (Ideal Behavioral Switches)", {{"Vd", "0.8"}, {"Ron", "1m"}, {"Roff", "1M"}}, false },
            { "MOSFET (MOSFET)", "MOSFET", "MOSFET", ComponentType::MOSFET, "MOSFET", "electrical", "Power Semiconductors (Ideal Behavioral Switches)", {{"Ron", "10m"}, {"Roff", "1M"}}, true },
            { "Voltage-Gated FET (vg-FET)", "vg-FET", "vg-FET", ComponentType::VGFET, "vg-FET", "electrical", "Power Semiconductors (Ideal Behavioral Switches)", {{"Gate_Signal_Label", "S1"}, {"Ron", "10m"}, {"Roff", "1M"}}, true },
            { "BJT (BJT)", "BJT", "BJT", ComponentType::BJT, "BJT", "electrical", "Power Semiconductors (Ideal Behavioral Switches)", {{"Vd", "0.7"}, {"Ron", "1m"}, {"Roff", "1M"}}, false },
            { "JFET (JFET)", "JFET", "JFET", ComponentType::JFET, "JFET", "electrical", "Power Semiconductors (Ideal Behavioral Switches)", {{"Ron", "10m"}, {"Roff", "1M"}}, false },

            // Electrical Domain Switches sub-library
            { "Switch (S)", "S", "S", ComponentType::Switch, "S", "electrical", "Switches", {{"Ron", "10m"}, {"Roff", "1M"}}, true },
            { "Breaker (BREAKER)", "Breaker", "BREAKER", ComponentType::Breaker, "BREAKER", "electrical", "Switches", {{"Ron", "10m"}, {"Roff", "1M"}}, false },
            { "Double Switch (DBL_SWITCH)", "2-Way", "DBL_SWITCH", ComponentType::DoubleSwitch, "DBL_SWITCH", "electrical", "Switches", {{"Ron", "10m"}, {"Roff", "1M"}}, false },
            { "Manual Switch (MAN_SWITCH)", "Man Sw", "MAN_SWITCH", ComponentType::ElectricalManualSwitch, "MAN_SWITCH", "electrical", "Switches", {{"Ron", "10m"}, {"Roff", "1M"}}, false },
            { "Manual Double Switch (MAN_DBL_SWITCH)", "Man 2Sw", "MAN_DBL_SWITCH", ComponentType::ManualDoubleSwitch, "MAN_DBL_SWITCH", "electrical", "Switches", {{"Ron", "10m"}, {"Roff", "1M"}}, false },
            { "Manual Triple Switch (MAN_TRPL_SWITCH)", "Man 3Sw", "MAN_TRPL_SWITCH", ComponentType::ManualTripleSwitch, "MAN_TRPL_SWITCH", "electrical", "Switches", {{"Ron", "10m"}, {"Roff", "1M"}}, false },
            { "Set/Reset Switch (SR_SWITCH)", "SR Sw", "SR_SWITCH", ComponentType::SRSwitch, "SR_SWITCH", "electrical", "Switches", {{"Ron", "10m"}, {"Roff", "1M"}}, false },
            // Electrical Domain Transformers sub-library
            { "Ideal Transformer (IDEAL_XFMR)", "Ideal XFMR", "IDEAL_XFMR", ComponentType::IdealTransformer, "IDEAL_XFMR", "electrical", "Transformers", {{"primary_turns", "100"}, {"secondary_turns", "100"}}, false },
            { "Linear Transformer (2-Winding) (XFMR_2W)", "XFMR 2W", "XFMR_2W", ComponentType::Transformer2W, "XFMR_2W", "electrical", "Transformers", {{"primary_turns", "100"}, {"secondary_turns", "100"}}, true },
            { "Linear Transformer (3-Winding) (XFMR_3W)", "XFMR 3W", "XFMR_3W", ComponentType::Transformer3W, "XFMR_3W", "electrical", "Transformers", {{"primary_turns", "100"}, {"secondary_turns", "100, 100"}}, false },
            { "Mutual Inductor (2-Winding) (MUTUAL_2W)", "Mut 2W", "MUTUAL_2W", ComponentType::MutualInductor2W, "MUTUAL_2W", "electrical", "Transformers", {{"primary_turns", "100"}, {"secondary_turns", "100"}}, false },
            { "Mutual Inductor (3-Winding) (MUTUAL_3W)", "Mut 3W", "MUTUAL_3W", ComponentType::MutualInductor3W, "MUTUAL_3W", "electrical", "Transformers", {{"primary_turns", "100"}, {"secondary_turns", "100, 100"}}, false },
            { "Saturable Transformer (SAT_XFMR)", "Sat XFMR", "SAT_XFMR", ComponentType::SaturableTransformer, "SAT_XFMR", "electrical", "Transformers", {{"primary_turns", "100"}, {"secondary_turns", "100"}}, false },
            { "Transformers (3-Phase 2-Winding) (XFMR_3PH_2W)", "3Ph 2W XFMR", "XFMR_3PH_2W", ComponentType::Transformer3Ph2W, "XFMR_3PH_2W", "electrical", "Transformers", {}, false },
            // Electrical Domain Electrical Machines sub-library
            { "Induction Motor 3-Phase (INDUCTION_MOTOR)", "Ind Motor 3Ph", "INDUCTION_MOTOR", ComponentType::InductionMotor, "INDUCTION_MOTOR", "electrical", "Electrical Machines", {{"Rs", "1.115"}, {"Lls", "0.005974"}, {"Rr", "1.083"}, {"Llr", "0.005974"}, {"Lm", "0.1"}, {"P", "2"}, {"J", "0.02"}, {"B", "1e-3"}, {"omega0", "0"}}, false },
            { "Induction Motor (IND_MOTOR)", "Ind Motor", "IND_MOTOR", ComponentType::InductionMotor, "IND_MOTOR", "electrical", "Electrical Machines", {{"Rs", "1.115"}, {"Lls", "0.005974"}, {"Rr", "1.083"}, {"Llr", "0.005974"}, {"Lm", "0.1"}, {"P", "2"}, {"J", "0.02"}, {"B", "1e-3"}, {"omega0", "0"}}, false },

            // Electrical Domain Electronics sub-library
            { "Operational Amplifier (OPAMP)", "OpAmp", "OPAMP", ComponentType::OpAmp, "OPAMP", "electrical", "Electronics", {{"gain", "1e5"}}, false },
            { "Comparator (Electrical) (E_COMP)", "E-Comp", "E_COMP", ComponentType::EComp, "E_COMP", "electrical", "Electronics", {{"gain", "1e5"}}, false },

            // Electrical Domain Custom Machine/Load Models sub-library
            { "Generalized Electrical Block (GEN_EBLOCK)", "Gen E-Block", "GEN_EBLOCK", ComponentType::GenEBlock, "GEN_EBLOCK", "electrical", "Custom Machine/Load Models", {{"terminals", "3"}}, true },

            // ═══════════════ MAGNETIC DOMAIN LIBRARY (permeance-capacitance analogy) ═══════════════
            // MMF <-> node potential, flux rate <-> branch current, permeance <-> capacitance.
            // Kept as its own top-level library because the magnetic domain carries
            // different physical quantities to the electrical one.

            // Magnetic Sources
            { "MMF Source (MMF_SRC)", "MMF", "FS", ComponentType::MMFSource, "MMF_SRC", "magnetic", "Sources", {{"F", "100"}}, false },
            { "MMF Source Controlled (MMF_SRC_CTRL)", "MMF Ctrl", "FSC", ComponentType::MMFSourceControlled, "MMF_SRC_CTRL", "magnetic", "Sources", {{"gain", "1.0"}}, false },

            // Magnetic Components
            { "Winding (WINDING)", "Winding", "W", ComponentType::Winding, "WINDING", "magnetic", "Components", {{"N", "100"}}, false },
            { "Magnetic Permeance (MAG_PERMEANCE)", "Permeance", "PM", ComponentType::MagneticPermeance, "MAG_PERMEANCE", "magnetic", "Components", {{"P", "1u"}, {"F0", "0"}}, false },
            { "Linear Core (LINEAR_CORE)", "Core", "CORE", ComponentType::LinearCore, "LINEAR_CORE", "magnetic", "Components", {{"A", "1e-4"}, {"l", "0.1"}, {"mu_r", "1000"}, {"F0", "0"}}, false },
            { "Air Gap (AIR_GAP)", "Air Gap", "GAP", ComponentType::AirGap, "AIR_GAP", "magnetic", "Components", {{"A", "1e-4"}, {"l", "1m"}, {"F0", "0"}}, false },
            { "Leakage Flux Path (LEAKAGE_PATH)", "Leakage", "PLK", ComponentType::LeakageFluxPath, "LEAKAGE_PATH", "magnetic", "Components", {{"P", "10n"}, {"F0", "0"}}, false },
            { "Magnetic Resistance (MAG_RESISTANCE)", "Rm", "RM", ComponentType::MagneticResistance, "MAG_RESISTANCE", "magnetic", "Components", {{"Rm", "1M"}}, false },
            { "BLDC Motor (BLDC_MOTOR)", "BLDC", "BLDC", ComponentType::BLDCMotor, "BLDC_MOTOR", "electrical", "Electrical Machines", {{"Rs", "1"}, {"Ls", "1e-3"}, {"Ke", "0.05"}, {"P", "2"}, {"J", "1e-4"}, {"B", "1e-4"}, {"omega0", "0"}, {"theta0", "0"}}, false },
            { "PMSM Motor (PMSM_MOTOR)", "PMSM", "PMSM", ComponentType::PMSMMotor, "PMSM_MOTOR", "electrical", "Electrical Machines", {{"Rs", "1"}, {"Ls", "1e-3"}, {"Ke", "0.05"}, {"P", "2"}, {"J", "1e-4"}, {"B", "1e-4"}, {"omega0", "0"}, {"theta0", "0"}}, false },
            { "Synchronous Motor (SYNC_MOTOR)", "Sync Motor", "SYNC", ComponentType::SynchronousMotor, "SYNC_MOTOR", "electrical", "Electrical Machines", {{"Rs", "1"}, {"Ls", "1e-3"}, {"Rf", "10"}, {"Lf", "0.5"}, {"Kf", "0.05"}, {"P", "2"}, {"J", "1e-4"}, {"B", "1e-4"}, {"omega0", "0"}, {"theta0", "0"}}, false },
            { "Switched Reluctance Motor (SRM_MOTOR)", "SRM", "SRM", ComponentType::SRMotor, "SRM_MOTOR", "electrical", "Electrical Machines", {{"Rs", "0.5"}, {"Lmin", "2e-3"}, {"Lmax", "10e-3"}, {"Nr", "4"}, {"J", "1e-4"}, {"B", "1e-4"}, {"omega0", "0"}, {"theta0", "0"}}, false },
            { "DC Motor (DC_MOTOR)", "DC Motor", "DCM", ComponentType::DCMotor, "DC_MOTOR", "electrical", "Electrical Machines", {{"Ra", "1"}, {"La", "1e-3"}, {"Ke", "0.05"}, {"Kt", "0.05"}, {"J", "1e-4"}, {"B", "1e-4"}, {"omega0", "0"}}, false },
        };
        return allComponents;
}
} // namespace golden_ref_palette
