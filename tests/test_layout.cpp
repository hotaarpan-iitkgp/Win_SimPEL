// Auto-layout: no overlapping blocks, positions on the grid, deterministic,
// signal and power flow left to right, and the netlist unchanged for every design
// in the corpus after re-placing it.
// Usage: test_layout <tests/golden/designs>
#include "engine/Circuit.hpp"
#include "engine/DesignIO.hpp"
#include "engine/NetlistBuilder.hpp"
#include "engine/NetlistGenerator.hpp"
#include "engine/layout/Placer.hpp"

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <map>
#include <string>

using namespace CircuitSim;
namespace fs = std::filesystem;

static int g_pass = 0, g_fail = 0;
static void check(bool ok, const std::string& what) {
    if (ok) ++g_pass; else { ++g_fail; std::printf("FAIL: %s\n", what.c_str()); }
}

static std::string overlaps(const CircuitDesign& d) {
    for (size_t i = 0; i < d.components.size(); ++i) {
        float wi, hi;
        placementExtent(d.components[i], wi, hi);
        for (size_t j = i + 1; j < d.components.size(); ++j) {
            float wj, hj;
            placementExtent(d.components[j], wj, hj);
            const auto& a = d.components[i];
            const auto& b = d.components[j];
            if (std::fabs(a.x - b.x) < wi + wj && std::fabs(a.y - b.y) < hi + hj) return a.id + " / " + b.id;
        }
    }
    return "";
}
static bool onGrid(const CircuitDesign& d) {
    for (const auto& c : d.components)
        if (std::fmod(std::fabs(c.x), 20.0f) != 0.0f || std::fmod(std::fabs(c.y), 20.0f) != 0.0f) return false;
    return true;
}
static std::string netlist(CircuitDesign d) {
    NetlistBuilder::buildNodesForCircuit(d);
    return generateNetlistJson(d);
}

int main(int argc, char** argv) {
    const fs::path dir = argc > 1 ? argv[1] : "tests/golden/designs";

    // 1. A buck built by connectivity.
    {
        Circuit c("buck");
        c.add("V", "V1", {{"value", "48"}});
        c.add("MOSFET", "Q1");
        c.add("D", "D1");
        c.add("L", "L1");
        c.add("C", "C1");
        c.add("R", "R1");
        c.add("TRI_GEN", "TRI1");
        c.add("CONST", "K1");
        c.add("COMP", "CMP1");
        c.add("PID", "PI1");
        c.add("VM", "VM1");
        c.connect({{"V1", "A"}, {"Q1", "D"}});
        c.connect({{"Q1", "S"}, {"D1", "B"}, {"L1", "A"}});
        c.connect({{"L1", "B"}, {"C1", "A"}, {"R1", "A"}, {"VM1", "A"}});
        c.ground({{"V1", "B"}, {"D1", "A"}, {"C1", "B"}, {"R1", "B"}, {"VM1", "B"}});
        c.connect({{"VM1", "Out"}, {"PI1", "In"}});
        c.connect({{"TRI1", "Out"}, {"CMP1", "In1"}});
        c.connect({{"K1", "Out"}, {"CMP1", "In2"}});
        c.connect({{"CMP1", "Out"}, {"Q1", "G"}});
        CircuitDesign d = c.design();
        const std::string before = netlist(d);
        CircuitDesign a = d, b = d;
        autoPlace(a);
        autoPlace(b);
        std::map<std::string, const ComponentInstance*> at;
        for (const auto& comp : a.components) at[comp.id] = &comp;
        check(overlaps(a).empty(), "buck: no overlaps (" + overlaps(a) + ")");
        check(onGrid(a), "buck: on the 20 px grid");
        bool same = true;
        for (size_t i = 0; i < a.components.size(); ++i)
            same &= a.components[i].x == b.components[i].x && a.components[i].y == b.components[i].y;
        check(same, "buck: deterministic");
        check(at["V1"]->x < at["Q1"]->x && at["Q1"]->x < at["L1"]->x && at["L1"]->x < at["C1"]->x,
              "buck: power flows source -> switch -> inductor -> capacitor");
        check(at["TRI1"]->x < at["CMP1"]->x && at["K1"]->x < at["CMP1"]->x, "buck: signals flow left to right");
        float powerBottom = 0;
        for (const char* id : {"V1", "Q1", "D1", "L1", "C1", "R1", "VM1"}) powerBottom = std::max(powerBottom, at[id]->y);
        check(at["CMP1"]->y > powerBottom && at["TRI1"]->y > powerBottom, "buck: control section below the power stage");
        check(at["GND"]->y > at["V1"]->y, "buck: ground below the stage");
        check(netlist(a) == before, "buck: netlist unchanged by placement");
    }

    // 2. Feedback loop: placement terminates and puts the loop's entry first.
    {
        Circuit c("loop");
        c.add("STEP", "S1");
        c.add("SUM_RECT", "E1", {{"inputs", "+-"}});
        c.add("INTEGRATOR", "I1");
        c.add("GAIN", "G1");
        c.connect({{"S1", "Out"}, {"E1", "In1"}});
        c.connect({{"E1", "Out"}, {"I1", "In"}});
        c.connect({{"I1", "Out"}, {"G1", "In"}});
        c.connect({{"G1", "Out"}, {"E1", "In2"}});
        CircuitDesign d = c.design();
        autoPlace(d);
        std::map<std::string, float> x;
        for (const auto& comp : d.components) x[comp.id] = comp.x;
        check(x["S1"] < x["E1"] && x["E1"] < x["I1"] && x["I1"] < x["G1"], "loop: S1 < E1 < I1 < G1");
        check(overlaps(d).empty(), "loop: no overlaps");
    }

    // 3. Corpus: every design keeps its netlist and gets an overlap-free layout.
    int designs = 0, badNetlist = 0, badOverlap = 0;
    for (const auto& e : fs::directory_iterator(dir)) {
        if (e.path().extension() != ".json") continue;
        ++designs;
        CircuitDesign d = DesignIO::loadDesignFile(e.path().string());
        const std::string before = netlist(d);
        CircuitDesign p = d;
        autoPlace(p);
        if (netlist(p) != before) { ++badNetlist; std::printf("  netlist changed: %s\n", e.path().filename().string().c_str()); }
        const std::string ov = overlaps(p);
        if (!ov.empty()) { ++badOverlap; std::printf("  overlap in %s: %s\n", e.path().filename().string().c_str(), ov.c_str()); }
        // Saved and reloaded, the placed design still has the same netlist.
        CircuitDesign r = DesignIO::loadDesign(nlohmann::json::parse(DesignIO::saveDesignString(p)));
        if (netlist(r) != before) { ++badNetlist; std::printf("  netlist changed after save: %s\n", e.path().filename().string().c_str()); }
    }
    check(designs > 50, "corpus scanned (" + std::to_string(designs) + ")");
    check(badNetlist == 0, "corpus: netlists unchanged by placement");
    check(badOverlap == 0, "corpus: no overlaps");

    std::printf("%d/%d checks passed\n", g_pass, g_pass + g_fail);
    return g_fail == 0 ? 0 : 1;
}
