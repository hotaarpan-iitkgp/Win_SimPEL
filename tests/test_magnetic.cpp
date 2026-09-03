// Verification for the magnetic-domain blocks (PLECS permeance-capacitance analogy).
//
// The only new solver element is the Winding gyrator:
//     v_elec = N * PhiDot        i_elec = F / N
// Everything else reuses existing primitives: a permeance is a capacitance in the
// magnetic domain, a magnetic resistance is a conductance, an MMF source is a
// voltage source.
//
// Checks performed (Section 6, Phase 1 of the specification):
//   A) A single winding closed on a permeance P must look like L = N^2 * P.
//   B) A two-winding transformer must reflect impedance by (N1/N2)^2.
//   C) A winding closed on a magnetic resistance must look like R = N^2 * G_fe.

#include "engine/CircuitSimulator.hpp"
#include <cstdio>
#include <string>
#include <vector>
#include <cmath>

using namespace CircuitSimEngine;

static ComponentModel mk(const std::string& id, ComponentType t,
                         const std::vector<std::string>& nodes,
                         const std::vector<std::pair<std::string, std::string>>& params) {
    ComponentModel c;
    c.id = id;
    c.type = t;
    c.nodes = nodes;
    for (const auto& p : params) c.parameters[p.first] = p.second;
    return c;
}

// std::to_string() only emits 6 decimals, which silently destroys values like
// 1.2566e-6. Format parameters with full round-trip precision instead.
static std::string num(double v) {
    char buf[64];
    snprintf(buf, sizeof(buf), "%.17g", v);
    return std::string(buf);
}

static double lastOf(const SimulationOutput& out, const std::string& key) {
    auto it = out.custom_plots.find(key);
    if (it != out.custom_plots.end() && !it->second.empty()) return it->second.back();
    auto it2 = out.signals.find(key);
    if (it2 != out.signals.end() && !it2->second.empty()) return it2->second.back();
    return 0.0;
}

static SimulationOutput run(const std::vector<ComponentModel>& phys, double tStop, double dt) {
    SimulationConfig cfg;
    cfg.stopTime = tStop;
    cfg.stepSize = dt;
    cfg.solver = "euler";
    cfg.step_type = "fixed";
    CircuitSimulator sim;
    sim.setup(phys, {}, cfg);
    return sim.runTransient();
}

static int checks = 0, failures = 0;
static void expectNear(const char* what, double got, double want, double tol) {
    checks++;
    bool ok = std::fabs(got - want) <= tol;
    if (!ok) failures++;
    printf("  %-44s got %14.6g  expected %14.6g   %s\n",
           what, got, want, ok ? "PASS" : "FAIL");
}

int main() {
    const double mu0 = 4.0 * 3.14159265358979323846 * 1e-7;

    // ---------------------------------------------------------------- A) L = N^2 * P
    // V1(10V) -- Rs(1mOhm) -- W1.E+ ; W1.E- = gnd
    // W1 magnetic port closed on a permeance P to gnd.
    // With L = N^2*P and a step input, i(t) = V*t/L while t << L/Rs.
    printf("A) Single winding on a permeance -> L = N^2 * P\n");
    {
        const double N = 10.0, P = 1e-3;      // => L = 100 * 1e-3 = 0.1 H
        const double V = 10.0, T = 1e-3;
        const double L = N * N * P;

        std::vector<ComponentModel> ckt = {
            mk("V1", ComponentType::VoltageSource, {"node_1", "0"}, {{"value", "10"}}),
            mk("Rs", ComponentType::Resistor,      {"node_1", "node_2"}, {{"value", "0.001"}}),
            mk("W1", ComponentType::Winding,       {"node_2", "0", "node_3", "0"}, {{"N", "10"}}),
            // Magnetic permeance == capacitance of value P in the magnetic domain
            mk("P1", ComponentType::Capacitor,     {"node_3", "0"}, {{"C", "1e-3"}, {"vC0", "0"}}),
        };

        SimulationOutput out = run(ckt, T, 1e-7);
        double i = lastOf(out, "I_W1");
        expectNear("winding current i = V*T/L", i, V * T / L, 0.002);

        // Same circuit, twice the turns -> four times the inductance -> quarter current
        ckt[2] = mk("W1", ComponentType::Winding, {"node_2", "0", "node_3", "0"}, {{"N", "20"}});
        out = run(ckt, T, 1e-7);
        double i2 = lastOf(out, "I_W1");
        expectNear("doubling N quarters the current", i2, V * T / (4.0 * L), 0.002);
    }

    // ------------------------------------------------- B) impedance ratio (N1/N2)^2
    // Primary N1=10, secondary N2=20 loaded with 40 ohm.
    // Reflected input impedance = (N1/N2)^2 * R2 = 0.25 * 40 = 10 ohm  => Iin = 1 A.
    // Secondary voltage = V * N2/N1 = 20 V.
    // Magnetic ports are wired in SERIES so the MMFs add (Ampere's law), which is how
    // PLECS builds multi-winding transformers.
    printf("\nB) Two-winding transformer -> impedance reflected by (N1/N2)^2\n");
    {
        std::vector<ComponentModel> ckt = {
            mk("V1", ComponentType::VoltageSource, {"node_1", "0"}, {{"value", "10"}}),
            // Primary
            mk("W1", ComponentType::Winding, {"node_1", "0", "node_10", "node_11"}, {{"N", "10"}}),
            // Secondary, magnetic port in series with the primary's
            mk("W2", ComponentType::Winding, {"node_2", "0", "node_11", "0"}, {{"N", "20"}}),
            // Large permeance closes the magnetic loop => tight coupling
            mk("Pc", ComponentType::Capacitor, {"node_10", "0"}, {{"C", "1.0"}, {"vC0", "0"}}),
            // Secondary load
            mk("R2", ComponentType::Resistor, {"node_2", "0"}, {{"value", "40"}}),
        };

        SimulationOutput out = run(ckt, 1e-3, 1e-7);
        double iIn  = lastOf(out, "I_W1");
        double vSec = lastOf(out, "V_R2");
        double iSec = lastOf(out, "I_R2");

        expectNear("primary current (Zin = 10 ohm)", iIn, 1.0, 0.01);
        expectNear("secondary voltage = V*N2/N1", std::fabs(vSec), 20.0, 0.2);
        expectNear("ampere-turns balance N1*I1 = N2*I2",
                   20.0 * std::fabs(iSec), 10.0 * std::fabs(iIn), 0.05);
    }

    // ------------------------------------------- C) magnetic resistance -> core loss
    // A winding closed on magnetic resistance R_mag appears electrically as N^2/R_mag.
    // N = 10, R_mag = 100  =>  R_elec = 1 ohm. With a 1 ohm series resistor the
    // divider gives i = 10 / (1 + 1) = 5 A.
    printf("\nC) Winding on a magnetic resistance -> R_elec = N^2 / R_mag\n");
    {
        std::vector<ComponentModel> ckt = {
            mk("V1", ComponentType::VoltageSource, {"node_1", "0"}, {{"value", "10"}}),
            mk("Rs", ComponentType::Resistor,      {"node_1", "node_2"}, {{"value", "1"}}),
            mk("W1", ComponentType::Winding,       {"node_2", "0", "node_3", "0"}, {{"N", "10"}}),
            // Magnetic resistance == resistance in the magnetic domain
            mk("Rm", ComponentType::Resistor,      {"node_3", "0"}, {{"value", "100"}}),
        };

        SimulationOutput out = run(ckt, 1e-3, 1e-6);
        double i = lastOf(out, "I_W1");
        expectNear("current through 1 ohm + reflected 1 ohm", std::fabs(i), 5.0, 0.05);
    }

    // ------------------------------------ D) geometry-derived permeance sanity check
    // Linear Core:  P = mu0 * mur * A / l      Air Gap: P = mu0 * A / l
    printf("\nD) Geometry-derived permeance formulas\n");
    {
        double A = 1e-4, l = 0.1, mur = 1000.0;
        double pCore = mu0 * mur * A / l;
        double pGap  = mu0 * A / l;
        expectNear("Linear Core P = mu0*mur*A/l", pCore, 1.2566370614e-6, 1e-12);
        expectNear("Air Gap    P = mu0*A/l",      pGap,  1.2566370614e-9, 1e-15);

        // A 100-turn winding on that core must give L = N^2 * P
        std::vector<ComponentModel> ckt = {
            mk("V1", ComponentType::VoltageSource, {"node_1", "0"}, {{"value", "10"}}),
            mk("Rs", ComponentType::Resistor,      {"node_1", "node_2"}, {{"value", "1e-4"}}),
            mk("W1", ComponentType::Winding,       {"node_2", "0", "node_3", "0"}, {{"N", "100"}}),
            mk("Pc", ComponentType::Capacitor,     {"node_3", "0"},
               {{"C", num(pCore)}, {"vC0", "0"}}),
        };
        double L = 100.0 * 100.0 * pCore;   // 1e4 * 1.2566e-6 = 12.566 mH
        SimulationOutput out = run(ckt, 1e-4, 1e-8);
        double i = lastOf(out, "I_W1");
        expectNear("core-wound inductor i = V*T/L", i, 10.0 * 1e-4 / L, 0.02);
    }

    // ------------------------------- E) MMF source and leakage path behaviour
    // An MMF source F drives a permeance P through a magnetic resistance Rm.
    // In the analogy this is a DC source charging a capacitor through a resistor,
    // so the steady-state MMF across the permeance settles at F.
    printf("\nE) MMF source driving a permeance through a magnetic resistance\n");
    {
        std::vector<ComponentModel> ckt = {
            // MMF source == voltage source of 100 A-t
            mk("F1", ComponentType::VoltageSource, {"node_1", "0"}, {{"value", "100"}}),
            // Magnetic resistance == resistor
            mk("Rm", ComponentType::Resistor,      {"node_1", "node_2"}, {{"value", "1000"}}),
            // Permeance == capacitor
            mk("P1", ComponentType::Capacitor,     {"node_2", "0"}, {{"C", "1e-6"}, {"vC0", "0"}}),
        };
        // tau = Rm*P = 1000 * 1e-6 = 1 ms; after 10 tau the MMF is essentially F.
        SimulationOutput out = run(ckt, 10e-3, 1e-6);
        double mmf = lastOf(out, "V_P1");
        expectNear("steady-state MMF across permeance = F", mmf, 100.0, 0.5);
    }

    // ------------------------------------- F) initial MMF on a permeance (F0)
    // A permeance pre-charged to F0 with nothing driving it must hold that MMF.
    printf("\nF) Initial MMF (F0) is honoured\n");
    {
        std::vector<ComponentModel> ckt = {
            mk("P1", ComponentType::Capacitor, {"node_1", "0"}, {{"C", "1e-6"}, {"vC0", "42"}}),
            // Very weak leak so the node stays referenced but barely discharges
            mk("Rm", ComponentType::Resistor,  {"node_1", "0"}, {{"value", "1e9"}}),
        };
        SimulationOutput out = run(ckt, 1e-4, 1e-7);
        double mmf = lastOf(out, "V_P1");
        expectNear("permeance holds its initial MMF", mmf, 42.0, 0.5);
    }

    printf("\n------------------------------------------------------------\n");
    printf("%d/%d checks passed\n", checks - failures, checks);
    return failures == 0 ? 0 : 1;
}
