// Functional test for the Power Semiconductor library behavioural models.
// Circuit under test:  V1(10V) --> [DUT] --> R1(10 ohm) --> gnd
// The DUT gate/base is driven by a Constant control block (1 = on, 0 = off).
// Expected: ~1 A when the device is conducting, ~0 A when it is blocking.

#include "engine/CircuitSimulator.hpp"
#include <cstdio>
#include <string>
#include <vector>
#include <cmath>

using namespace CircuitSimEngine;

struct Result { double iOn; double iOff; double iReverse; };

static double lastOf(const SimulationOutput& out, const std::string& key) {
    auto it = out.custom_plots.find(key);
    if (it != out.custom_plots.end() && !it->second.empty()) return it->second.back();
    auto it2 = out.signals.find(key);
    if (it2 != out.signals.end() && !it2->second.empty()) return it2->second.back();
    return 0.0;
}

// gateLevel: control value. reverse: flip source polarity to test blocking.
static double runOne(ComponentType dutType, double gateLevel, bool reverse, double vd = 0.0) {
    ComponentModel v1;
    v1.id = "V1";
    v1.type = ComponentType::VoltageSource;
    v1.nodes = reverse ? std::vector<std::string>{"0", "node_1"}
                       : std::vector<std::string>{"node_1", "0"};
    v1.parameters["value"] = "10";

    ComponentModel dut;
    dut.id = "DUT";
    dut.type = dutType;
    dut.nodes = {"node_1", "node_2"};
    dut.parameters["Ron"] = "0.001";
    dut.parameters["Roff"] = "1e6";
    dut.parameters["Vd"] = std::to_string(vd);
    dut.parameters["control_signal"] = "G1.Out";

    ComponentModel r1;
    r1.id = "R1";
    r1.type = ComponentType::Resistor;
    r1.nodes = {"node_2", "0"};
    r1.parameters["value"] = "10";

    ComponentModel g1;
    g1.id = "G1";
    g1.type = ComponentType::Constant;
    g1.parameters["value"] = gateLevel > 0.5 ? "1.0" : "0.0";

    SimulationConfig cfg;
    cfg.stopTime = 2e-3;
    cfg.stepSize = 1e-5;
    cfg.solver = "euler";
    cfg.step_type = "fixed";

    CircuitSimulator sim;
    sim.setup({v1, dut, r1}, {g1}, cfg);
    SimulationOutput out = sim.runTransient();

    return lastOf(out, "I_R1");
}

int main() {
    struct Case { const char* name; ComponentType type; bool gated; bool blocksReverse; };
    const Case cases[] = {
        { "Diode",      ComponentType::Diode,      false, true  },
        { "Thyristor",  ComponentType::Thyristor,  true,  true  },
        { "IGBT",       ComponentType::IGBT,       true,  true  },
        { "IGBT_DIODE", ComponentType::IGBTDiode,  true,  false },
        { "GTO",        ComponentType::GTO,        true,  true  },
        { "IGCT",       ComponentType::IGCT,       true,  true  },
        { "BJT",        ComponentType::BJT,        true,  true  },
        { "JFET",       ComponentType::JFET,       true,  false },
        { "MOSFET",     ComponentType::MOSFET,     true,  false },
        { "Switch",     ComponentType::Switch,     true,  false },
    };

    printf("%-12s %10s %10s %10s   %s\n", "Device", "I(on)", "I(off)", "I(rev)", "verdict");
    printf("------------------------------------------------------------\n");

    int failures = 0;
    for (const auto& c : cases) {
        double iOn  = runOne(c.type, 1.0, false);
        double iOff = runOne(c.type, 0.0, false);
        double iRev = runOne(c.type, 1.0, true);

        bool onOk = std::fabs(iOn) > 0.5;                       // should conduct ~1 A
        bool offOk = c.gated ? (std::fabs(iOff) < 1e-3) : true;  // gated parts must block
        bool revOk = c.blocksReverse ? (std::fabs(iRev) < 1e-3)
                                     : (std::fabs(iRev) > 0.5);

        bool ok = onOk && offOk && revOk;
        if (!ok) failures++;

        printf("%-12s %10.5f %10.2e %10.5f   %s%s%s%s\n",
               c.name, iOn, iOff, iRev,
               ok ? "PASS" : "FAIL",
               onOk ? "" : " [no conduction]",
               offOk ? "" : " [leaks when off]",
               revOk ? "" : (c.blocksReverse ? " [fails to block reverse]" : " [no reverse path]"));
    }

    printf("------------------------------------------------------------\n");
    printf("%d/%d devices behaved correctly\n", (int)(sizeof(cases)/sizeof(cases[0])) - failures,
           (int)(sizeof(cases)/sizeof(cases[0])));

    // Forward voltage drop check. With Vd = 0.8, Ron = 1 mohm, R = 10 ohm, V = 10 V
    // the on-state current should be (10 - 0.8) / 10.001 ~= 0.9199 A.
    printf("\nForward-drop (Vd = 0.8 V) check, expected ~0.9199 A:\n");
    const ComponentType vdParts[] = { ComponentType::IGBT, ComponentType::GTO,
                                      ComponentType::IGCT, ComponentType::BJT,
                                      ComponentType::IGBTDiode };
    const char* vdNames[] = { "IGBT", "GTO", "IGCT", "BJT", "IGBT_DIODE" };
    int vdFailures = 0;
    for (int i = 0; i < 5; ++i) {
        double iVd = runOne(vdParts[i], 1.0, false, 0.8);
        bool ok = std::fabs(iVd - 0.9199) < 0.01;
        if (!ok) vdFailures++;
        printf("  %-12s %8.5f A   %s\n", vdNames[i], iVd, ok ? "PASS" : "FAIL");
    }

    return (failures == 0 && vdFailures == 0) ? 0 : 1;
}
