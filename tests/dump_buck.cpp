// Dumps every sample of the buck-converter run to stdout with full precision, so the
// pre-change and post-change solvers can be diffed bitwise. Compiled twice: once
// against tests/baseline/engine (git HEAD) and once against src/engine.

#include "engine/CircuitSimulator.hpp"
#include <cstdio>
#include <map>
#include <string>
#include <vector>

using namespace CircuitSimEngine;

static ComponentModel mk(const std::string& id, ComponentType t,
                         const std::vector<std::string>& nodes,
                         const std::vector<std::pair<std::string, std::string>>& params) {
    ComponentModel c; c.id = id; c.type = t; c.nodes = nodes;
    for (const auto& p : params) c.parameters[p.first] = p.second;
    return c;
}

// Second circuit: exercises the control blocks whose selector strings were moved to
// setup-time normalisation (TrigFunction, MathFunction, Round) plus the logical /
// relational blocks whose operator strings became const references.
static void buildMathMix(std::vector<ComponentModel>& phys, std::vector<ComponentModel>& ctrl) {
    phys = {
        mk("V1", ComponentType::VoltageSource, {"node_1", "0"}, {{"value", "10"}}),
        mk("R1", ComponentType::Resistor,      {"node_1", "0"}, {{"value", "5"}}),
        mk("VM1", ComponentType::Voltmeter,    {"node_1", "0"}, {}),
    };
    ctrl = {
        mk("SIN1",  ComponentType::SineWave,     {}, {{"amplitude", "2"}, {"frequency", "1k"}, {"phase", "30"}}),
        mk("TRG1",  ComponentType::TrigFunction, {}, {{"function", "COS"}, {"In", "SIN1.Out"}}),
        mk("MTH1",  ComponentType::MathFunction, {}, {{"function", "SQRT"}, {"In", "TRG1.Out"}}),
        mk("RND1",  ComponentType::Round,        {}, {{"mode", "FLOOR"}, {"In", "MTH1.Out"}}),
        mk("CST1",  ComponentType::Constant,     {}, {{"value", "0.5"}}),
        mk("REL1",  ComponentType::RelationalOp, {}, {{"operator", ">"}, {"In1", "SIN1.Out"}, {"In2", "CST1.Out"}}),
        mk("CMPC1", ComponentType::CompareToConstant, {}, {{"operator", "<="}, {"constant", "0.25"}, {"In", "MTH1.Out"}}),
        mk("LOG1",  ComponentType::LogicOp,      {}, {{"operator", "AND"}, {"In1", "REL1.Out"}, {"In2", "CMPC1.Out"}}),
        mk("DTC1",  ComponentType::DataTypeConv, {}, {{"datatype", "boolean"}, {"In", "LOG1.Out"}}),
    };
}

int main(int argc, char** argv) {
    const char* solver = (argc > 1) ? argv[1] : "euler";
    double tStop = (argc > 2) ? atof(argv[2]) : 5e-3;
    const char* which = (argc > 3) ? argv[3] : "buck";

    std::vector<ComponentModel> phys = {
        mk("V1",      ComponentType::VoltageSource, {"node_1", "0"},      {{"value", "24"}}),
        mk("MOSFET1", ComponentType::MOSFET,        {"node_1", "node_2"},
           {{"Ron", "10m"}, {"Roff", "1M"}, {"control_signal", "COMP1.Out"}}),
        mk("D1",      ComponentType::Diode,         {"0", "node_2"},
           {{"Vd", "0.7"}, {"Ron", "1m"}, {"Roff", "1M"}}),
        mk("L1",      ComponentType::Inductor,      {"node_2", "node_3"}, {{"L", "100u"}, {"esr", "50m"}}),
        mk("C1",      ComponentType::Capacitor,     {"node_3", "0"},      {{"C", "100u"}, {"esr", "10m"}}),
        mk("R1",      ComponentType::Resistor,      {"node_3", "0"},      {{"value", "10"}}),
        mk("VM1",     ComponentType::Voltmeter,     {"node_3", "0"},      {}),
    };
    std::vector<ComponentModel> ctrl = {
        mk("CONST1",    ComponentType::Constant,        {}, {{"value", "5"}}),
        mk("SUBTRACT1", ComponentType::SummingJunction, {},
           {{"signs", "+-"}, {"In1", "CONST1.Out"}, {"In2", "VM1.Out"}}),
        mk("PID1",      ComponentType::PI_Controller,   {},
           {{"Kp", "0.1"}, {"Ki", "60"}, {"Kd", "0"}, {"In", "SUBTRACT1.Out"},
            {"limit_output", "true"}, {"lower_limit", "0.01"}, {"upper_limit", "0.9"},
            {"anti_windup", "true"}}),
        mk("TRI1",      ComponentType::Triangle_Carrier, {},
           {{"frequency", "100k"}, {"min", "0"}, {"max", "1"}}),
        mk("COMP1",     ComponentType::Comparator,      {},
           {{"In1", "PID1.Out"}, {"In2", "TRI1.Out"}, {"hysteresis", "0"}}),
    };

    if (std::string(which) == "mathmix") buildMathMix(phys, ctrl);

    SimulationConfig cfg;
    cfg.stopTime = tStop; cfg.stepSize = 1e-6; cfg.solver = solver; cfg.step_type = "fixed";

    CircuitSimulator sim;
    sim.setup(phys, ctrl, cfg);
    SimulationOutput out = sim.runTransient();

    // Deterministic ordering: gather every series into a sorted map.
    std::map<std::string, const std::vector<double>*> series;
    auto add = [&](const std::unordered_map<std::string, std::vector<double>>& m) {
        for (const auto& kv : m) series[kv.first] = &kv.second;
    };
    add(out.voltages); add(out.signals); add(out.inductors);
    add(out.voltmeters); add(out.ammeters); add(out.custom_plots);

    printf("circuit=%s solver=%s tStop=%.9g records=%zu series=%zu\n",
           which, solver, tStop, out.time.size(), series.size());
    for (size_t i = 0; i < out.time.size(); ++i) printf("t %zu %.17g\n", i, out.time[i]);
    for (const auto& kv : series) {
        const std::vector<double>& v = *kv.second;
        for (size_t i = 0; i < v.size(); ++i) printf("s %s %zu %.17g\n", kv.first.c_str(), i, v[i]);
    }
    return 0;
}
