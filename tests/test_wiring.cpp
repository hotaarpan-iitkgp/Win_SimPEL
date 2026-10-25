// Verifies that the adaptive-stepping settings survive the netlist round trip.
//
// The UI writes simulation_parameters into the netlist JSON and NetlistParser reads it
// back. This is the seam where `enable_lu_cache` sat unread for a long time - the UI
// emitted it and the engine ignored it - so the round trip is worth asserting rather
// than assuming.

#include "engine/NetlistParser.hpp"
#include <cstdio>
#include <string>

using namespace CircuitSimEngine;

static std::string netlist(const char* stepType, const char* extra) {
    std::string s =
        "{\"physical_stage\":{\"voltage_sources\":[{\"id\":\"V1\",\"nodes\":[\"node_1\",\"node_0\"],"
        "\"src_type\":\"dc\",\"value\":10}],\"resistors\":[{\"id\":\"R1\","
        "\"nodes\":[\"node_1\",\"node_0\"],\"value\":10,\"esr\":0}]},"
        "\"control_loops\":{},"
        "\"simulation_parameters\":{\"stop_time\":0.01,\"step_size\":1e-06,"
        "\"solver\":\"euler\",\"step_type\":\"";
    s += stepType;
    s += "\"";
    if (extra && *extra) { s += ","; s += extra; }
    s += "}}";
    return s;
}

static SimulationConfig parse(const std::string& js) {
    std::vector<ComponentModel> phys, ctrl;
    SimulationConfig cfg;
    NetlistParser::parseJsonString(js, phys, ctrl, cfg);
    return cfg;
}

int main() {
    int pass = 0, fail = 0;
    auto check = [&](const char* what, bool ok) {
        printf("  %-58s %s\n", what, ok ? "PASS" : "FAIL");
        if (ok) ++pass; else ++fail;
    };

    printf("Netlist round trip for solver settings\n");
    printf("======================================\n\n");

    // Defaults when nothing is specified.
    {
        SimulationConfig c = parse(netlist("fixed", ""));
        check("step_type=fixed parses", c.step_type == "fixed");
        check("relTol keeps its default", c.relTol == 1e-3);
        check("enableLUCache defaults to on", c.enableLUCache);
    }

    // Variable stepping with explicit tolerances, given as numbers.
    {
        SimulationConfig c = parse(netlist("variable",
            "\"rel_tol\":1e-05,\"abs_tol_v\":0.002,\"abs_tol_i\":2e-06,"
            "\"h_min\":1e-11,\"h_max\":3e-06,\"enable_lu_cache\":false"));
        check("step_type=variable parses", c.step_type == "variable");
        check("rel_tol parses", c.relTol == 1e-5);
        check("abs_tol_v parses", c.absTolV == 0.002);
        check("abs_tol_i parses", c.absTolI == 2e-6);
        check("h_min parses", c.hMin == 1e-11);
        check("h_max parses", c.hMax == 3e-6);
        check("enable_lu_cache=false is honoured", !c.enableLUCache);
    }

    // The UI writes these as strings via formatJSStyleDouble, so strings must work too.
    {
        SimulationConfig c = parse(netlist("variable",
            "\"rel_tol\":\"1e-4\",\"abs_tol_v\":\"5m\",\"h_max\":\"2.5u\""));
        check("rel_tol parses from a string", c.relTol == 1e-4);
        check("abs_tol_v parses engineering suffix (5m -> 0.005)",
              c.absTolV > 0.00499 && c.absTolV < 0.00501);
        check("h_max parses engineering suffix (2.5u -> 2.5e-6)",
              c.hMax > 2.49e-6 && c.hMax < 2.51e-6);
    }

    // "adaptive" must be accepted as a synonym, since the engine checks for both.
    {
        SimulationConfig c = parse(netlist("adaptive", ""));
        check("step_type=adaptive parses", c.step_type == "adaptive");
    }

    printf("\n------------------------------------------------------------\n");
    printf("%d/%d checks passed\n", pass, pass + fail);
    return fail == 0 ? 0 : 1;
}
