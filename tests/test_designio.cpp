// DesignIO: settings precedence across the three historical keys, and save -> load
// round trip of non-default settings, plot config and parameters.
#include "engine/DesignIO.hpp"

#include <cmath>
#include <cstdio>
#include <string>

using namespace CircuitSim;
using json = nlohmann::json;

static int g_pass = 0, g_fail = 0;
static void check(bool ok, const char* what) {
    if (ok) ++g_pass; else { ++g_fail; std::printf("FAIL: %s\n", what); }
}

int main() {
    // 1. Precedence: settings < simulationSettings < simulation_parameters.
    {
        json j = json::parse(R"({
            "components": [], "wires": [],
            "settings": {"stopTime": 1.0, "stepSize": 1e-3, "solver": "rk4", "relTol": 0.5, "h_max": 7e-6},
            "simulationSettings": {"stopTime": "20m", "stepType": "variable"},
            "simulation_parameters": {"stop_time": 0.03, "solver": "trapezoidal", "enable_lu_cache": false}
        })");
        CircuitDesign cd = DesignIO::loadDesign(j);
        check(cd.settings.stopTime == 0.03, "simulation_parameters wins stop time");
        check(cd.settings.stepSize == 1e-3, "settings supplies step size");
        check(cd.settings.solverType == "trapezoidal", "simulation_parameters wins solver");
        check(cd.settings.stepType == "variable", "simulationSettings supplies step type");
        check(cd.settings.relTol == 0.5, "settings supplies relTol (camel)");
        check(cd.settings.hMax == 7e-6, "settings supplies h_max (snake)");
        check(cd.settings.enableLUCache == false, "enable_lu_cache false");
    }
    // 2. Engineering-notation strings.
    {
        json j = json::parse(R"({"simulationSettings": {"stopTime": "10m", "stepSize": "1u"}})");
        CircuitDesign cd = DesignIO::loadDesign(j);
        check(std::fabs(cd.settings.stopTime - 0.01) < 1e-15, "10m parses");
        check(std::fabs(cd.settings.stepSize - 1e-6) < 1e-21, "1u parses");
    }
    // 3. Round trip.
    {
        json j = json::parse(R"({
            "components": [
                {"id": "R1", "type": "R", "x": 10, "y": 20, "rotation": 90, "parameters": {"value": "4.7k"}},
                {"id": "C1", "type": "C", "x": 50, "y": 20, "params": {"C": "1u"}}
            ],
            "wires": [{"id": "w1", "from": {"compId": "R1", "terminal": "B"}, "to": {"compId": "C1", "terminal": "A"}}],
            "simulation_parameters": {"stop_time": 0.002, "step_size": 2e-7, "solver": "trapezoidal",
                                      "step_type": "variable", "rel_tol": 1e-4, "abs_tol_v": 1e-5,
                                      "abs_tol_i": 1e-8, "h_min": 1e-12, "h_max": 1e-6},
            "plotConfig": {"plots": [{"title": "P", "variables": ["V_C1", "I_R1"]}]}
        })");
        CircuitDesign a = DesignIO::loadDesign(j);
        json saved = DesignIO::saveDesign(a);
        check(saved.contains("simulationSettings"), "writer emits simulationSettings");
        CircuitDesign b = DesignIO::loadDesign(json::parse(DesignIO::saveDesignString(a)));
        check(b.settings.stopTime == a.settings.stopTime && b.settings.stepSize == a.settings.stepSize, "times survive");
        check(b.settings.solverType == "trapezoidal" && b.settings.stepType == "variable", "solver/step survive");
        check(b.settings.relTol == 1e-4 && b.settings.absTolV == 1e-5 && b.settings.absTolI == 1e-8, "tolerances survive");
        check(b.settings.hMin == 1e-12 && b.settings.hMax == 1e-6, "h bounds survive");
        check(b.plotConfig.plots.size() == 1 && b.plotConfig.plots[0].variables.size() == 2, "plot config survives");
        check(b.components.size() == 2 && b.components[0].parameters.at("value") == "4.7k", "parameters survive");
        check(b.components[1].parameters.count("C") && b.components[1].parameters.at("vC0") == "0", "params key + vC0 backfill");
        check(b.components[0].rotation == 90 && b.components[0].x == 10.0f, "placement survives");
        check(b.wires.size() == 1 && b.wires[0].from.compId == "R1" && b.wires[0].to.terminal == "A", "wire survives");
    }
    // 3b. Components whose id starts with W (windings) keep their pin wires.
    {
        json j = json::parse(R"({
            "components": [
                {"id": "W1", "type": "WINDING", "parameters": {"N": "10"}},
                {"id": "R1", "type": "R", "parameters": {"value": "1"}}
            ],
            "wires": [{"id": "w1", "from": {"type": "pin", "compId": "W1", "terminal": "A"},
                                   "to": {"type": "pin", "compId": "R1", "terminal": "A"}}]
        })");
        CircuitDesign a = DesignIO::loadDesign(j);
        check(a.wires.size() == 1 && !a.wires[0].from.isWireJunction && a.wires[0].from.compId == "W1",
              "W1 pin endpoint is not a junction on load");
        CircuitDesign b = DesignIO::loadDesign(json::parse(DesignIO::saveDesignString(a)));
        check(b.wires.size() == 1 && !b.wires[0].from.isWireJunction && b.wires[0].from.compId == "W1",
              "W1 pin endpoint survives save/load");
        json legacy = json::parse(R"({"components": [{"id": "R1", "type": "R"}, {"id": "R2", "type": "R"}],
            "wires": [{"id": "w1", "from": {"compId": "R1", "terminal": "B"}, "to": {"compId": "R2", "terminal": "A"}},
                      {"id": "w2", "from": {"compId": "w1"}, "to": {"compId": "R2", "terminal": "B"}}]})");
        CircuitDesign c = DesignIO::loadDesign(legacy);
        check(c.wires.size() == 2 && c.wires[1].from.isWireJunction && c.wires[1].from.targetWireId == "w1",
              "legacy untyped wire-id endpoint is still a junction");
    }
    // 4. Errors.
    {
        bool threw = false;
        try { DesignIO::loadDesignFile("this/does/not/exist.json"); } catch (const std::exception&) { threw = true; }
        check(threw, "missing file throws");
        threw = false;
        try { DesignIO::loadDesign(json::array()); } catch (const std::exception&) { threw = true; }
        check(threw, "non-object throws");
    }
    std::printf("%d/%d checks passed\n", g_pass, g_pass + g_fail);
    return g_fail == 0 ? 0 : 1;
}
