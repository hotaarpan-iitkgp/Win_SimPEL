// Wire router: A* port behaviour, route validity (orthogonal, ends on the
// endpoints, never through a block other than at its own pin), few overlaps
// between different nets, and unchanged netlists over the whole corpus after
// placement + routing (also after save / reload).
// Usage: test_route <tests/golden/designs>
#include "engine/Circuit.hpp"
#include "engine/DesignIO.hpp"
#include "engine/NetlistBuilder.hpp"
#include "engine/NetlistGenerator.hpp"
#include "engine/layout/Placer.hpp"
#include "engine/layout/Router.hpp"

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <functional>
#include <map>
#include <set>
#include <string>

using namespace CircuitSim;
namespace fs = std::filesystem;

static int g_pass = 0, g_fail = 0;
static void check(bool ok, const std::string& what) {
    if (ok) ++g_pass; else { ++g_fail; std::printf("FAIL: %s\n", what.c_str()); }
}
static Point2D P(float x, float y) { Point2D p; p.x = x; p.y = y; return p; }
static std::string netlist(CircuitDesign d) {
    NetlistBuilder::buildNodesForCircuit(d);
    return generateNetlistJson(d);
}

struct Audit { int wires = 0, notOrthogonal = 0, badEnds = 0, throughBlock = 0, overlapCells = 0; };

// Checks every routed wire of a placed + routed design.
static Audit audit(const CircuitDesign& d) {
    Audit a;
    std::map<std::string, const ComponentInstance*> comps;
    for (const auto& c : d.components) comps[c.id] = &c;
    std::vector<RouteRect> boxes;
    for (const auto& c : d.components) {
        float hw, hh;
        placementExtent(c, hw, hh);
        boxes.push_back({c.x - hw, c.y - hh, c.x + hw, c.y + hh});
    }
    // Net of each wire, for overlap counting.
    std::map<std::string, std::string> parent;
    std::function<std::string(const std::string&)> find = [&](const std::string& x) -> std::string {
        auto it = parent.find(x);
        if (it == parent.end() || it->second == x) { parent[x] = x; return x; }
        return parent[x] = find(it->second);
    };
    for (const auto& w : d.wires) {
        find(w.id);
        if (w.from.isWireJunction) parent[find(w.id)] = find(w.from.targetWireId);
    }
    std::map<std::pair<long long, long long>, std::set<std::string>> cellsH, cellsV;
    for (const auto& w : d.wires) {
        if (w.manualPath.size() < 2) continue;
        ++a.wires;
        const auto& p = w.manualPath;
        for (size_t i = 0; i + 1 < p.size(); ++i)
            if (p[i].x != p[i + 1].x && p[i].y != p[i + 1].y) { ++a.notOrthogonal; break; }
        // Ends: the "to" pin, and the from pin or junction point.
        auto pinPos = [&](const WireEndpoint& ep, Point2D& out) {
            auto it = comps.find(ep.compId);
            if (it == comps.end()) return false;
            for (const auto& pin : instancePins(*it->second))
                if (pin.name == ep.terminal) { out = P(it->second->x + pin.relX, it->second->y + pin.relY); return true; }
            return false;
        };
        Point2D e1{}, e2{};
        bool ok1 = w.from.isWireJunction ? (e1 = P(w.from.junctionX, w.from.junctionY), true) : pinPos(w.from, e1);
        bool ok2 = pinPos(w.to, e2);
        if (!ok1 || !ok2 || p.front().x != e1.x || p.front().y != e1.y || p.back().x != e2.x || p.back().y != e2.y) ++a.badEnds;
        // Interior of the route (without the first / last segment, which leave a pin)
        // must not enter any block.
        for (size_t i = 1; i + 2 < p.size(); ++i) {
            const Point2D s = p[i], t = p[i + 1];
            const int steps = (int)(std::fabs(t.x - s.x) + std::fabs(t.y - s.y)) / 10;
            for (int k = 0; k <= steps; ++k) {
                const float f = steps ? (float)k / steps : 0.0f;
                const float x = s.x + (t.x - s.x) * f, y = s.y + (t.y - s.y) * f;
                for (size_t bi = 0; bi < boxes.size(); ++bi) {
                    const auto& b = boxes[bi];
                    if (x > b.xMin + 2 && x < b.xMax - 2 && y > b.yMin + 2 && y < b.yMax - 2) {
                        if (++a.throughBlock <= 3)
                            std::printf("    wire %s (%s.%s -> %s.%s) segment %zu (%g,%g)-(%g,%g) crosses %s\n", w.id.c_str(),
                                        w.from.compId.c_str(), w.from.terminal.c_str(), w.to.compId.c_str(), w.to.terminal.c_str(),
                                        i, s.x, s.y, t.x, t.y, d.components[bi].id.c_str());
                        goto nextWire;
                    }
                }
            }
        }
        {
            const std::string net = find(w.id);
            for (size_t i = 0; i + 1 < p.size(); ++i) {
                const Point2D s = p[i], t = p[i + 1];
                const bool hz = s.y == t.y;
                if (std::fmod(std::fabs(hz ? s.y : s.x), 20.0f) != 0.0f) continue;
                const float lo = hz ? std::min(s.x, t.x) : std::min(s.y, t.y), hi = hz ? std::max(s.x, t.x) : std::max(s.y, t.y);
                for (float v = std::ceil(lo / 20.0f) * 20.0f; v < hi; v += 20.0f) {
                    auto key = hz ? std::make_pair((long long)v, (long long)s.y) : std::make_pair((long long)s.x, (long long)v);
                    (hz ? cellsH : cellsV)[key].insert(net);
                }
            }
        }
    nextWire:;
    }
    for (const auto* m : {&cellsH, &cellsV})
        for (const auto& [k, nets] : *m) if (nets.size() > 1) ++a.overlapCells;
    return a;
}

int main(int argc, char** argv) {
    const fs::path dir = argc > 1 ? argv[1] : "tests/golden/designs";

    // 1. findAStarPath: straight line when free; detour around a block.
    {
        auto p = findAStarPath(P(0, 0), P(1, 0), P(200, 0), P(1, 0), {});
        check(p.size() == 11 && p.front().x == 0 && p.back().x == 200, "free path is the straight grid line");
        std::vector<RouteRect> wall{{80, -100, 120, 100}};
        auto q = simplifyPath(findAStarPath(P(0, 0), P(1, 0), P(200, 0), P(1, 0), wall));
        bool clear = !q.empty();
        for (const auto& pt : q) clear &= !(pt.x > 82 && pt.x < 118 && pt.y > -98 && pt.y < 98);
        check(clear && q.size() >= 4, "path detours around a wall");
        auto r = orthogonalPath(P(0, 0), P(1, 0), P(200, 40), P(-1, 0), {});
        bool ortho = true;
        for (size_t i = 0; i + 1 < r.size(); ++i) ortho &= r[i].x == r[i + 1].x || r[i].y == r[i + 1].y;
        check(ortho && r.front().x == 0 && r.back().x == 200 && r.back().y == 40, "orthogonal pin-to-pin route");
    }

    // 2. Buck from connectivity: placed + routed.
    {
        Circuit c("buck");
        c.add("V", "V1"); c.add("MOSFET", "Q1"); c.add("D", "D1"); c.add("L", "L1"); c.add("C", "C1"); c.add("R", "R1");
        c.add("TRI_GEN", "TRI1"); c.add("CONST", "K1"); c.add("COMP", "CMP1"); c.add("VM", "VM1"); c.add("PID", "PI1");
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
        autoPlace(d);
        RouteStats s = routeDesign(d);
        Audit a = audit(d);
        std::printf("  buck: %d nets, %d wires, overlaps %d cells, fallbacks %d\n", s.nets, s.wires, a.overlapCells, s.searchFailures);
        check(s.netsKept == 0 && s.nets == 8, "buck: every net routed");
        check(a.wires == s.wires && a.notOrthogonal == 0, "buck: routes orthogonal");
        check(a.badEnds == 0, "buck: routes end on their endpoints");
        check(a.throughBlock == 0, "buck: routes avoid blocks");
        check(a.overlapCells == 0, "buck: no wire runs along another net's wire");
        check(netlist(d) == before, "buck: netlist unchanged by routing");
        std::map<std::string, int> direct;   // pin -> direct wires (GUI merges pins with several)
        for (const auto& w : d.wires)
            for (const WireEndpoint* ep : {&w.from, &w.to})
                if (!ep->isWireJunction) ++direct[ep->compId + "." + ep->terminal];
        int multi = 0;
        for (const auto& [k, n] : direct) multi += n > 1;
        check(multi == 0, "buck: every pin has exactly one direct wire");
        CircuitDesign r = DesignIO::loadDesign(nlohmann::json::parse(DesignIO::saveDesignString(d)));
        bool kept = r.wires.size() == d.wires.size();
        for (size_t i = 0; kept && i < r.wires.size(); ++i) kept = r.wires[i].manualPath.size() == d.wires[i].manualPath.size();
        check(kept, "buck: routes survive save / load");
        check(netlist(r) == before, "buck: netlist unchanged after save / load");
    }

    // 3. Corpus.
    int designs = 0, badNetlist = 0;
    Audit total;
    RouteStats sum;
    for (const auto& e : fs::directory_iterator(dir)) {
        if (e.path().extension() != ".json") continue;
        ++designs;
        CircuitDesign d = DesignIO::loadDesignFile(e.path().string());
        const std::string before = netlist(d);
        autoPlace(d);
        RouteStats s = routeDesign(d);
        sum.nets += s.nets; sum.netsKept += s.netsKept; sum.wires += s.wires; sum.searchFailures += s.searchFailures;
        Audit a = audit(d);
        total.wires += a.wires; total.notOrthogonal += a.notOrthogonal; total.badEnds += a.badEnds;
        total.throughBlock += a.throughBlock; total.overlapCells += a.overlapCells;
        const std::string name = e.path().filename().string();
        if (netlist(d) != before) { ++badNetlist; std::printf("  netlist changed: %s\n", name.c_str()); }
        CircuitDesign r = DesignIO::loadDesign(nlohmann::json::parse(DesignIO::saveDesignString(d)));
        if (netlist(r) != before) { ++badNetlist; std::printf("  netlist changed after save: %s\n", name.c_str()); }
    }
    std::printf("  corpus: %d designs, %d nets routed, %d kept, %d wires, %d fallbacks; "
                "not orthogonal %d, bad ends %d, through blocks %d, overlap cells %d\n",
                designs, sum.nets, sum.netsKept, sum.wires, sum.searchFailures,
                total.notOrthogonal, total.badEnds, total.throughBlock, total.overlapCells);
    check(badNetlist == 0, "corpus: netlists unchanged by placement + routing");
    check(total.notOrthogonal == 0 && total.badEnds == 0, "corpus: routes orthogonal and end on their endpoints");
    check(total.throughBlock <= sum.searchFailures, "corpus: only heuristic fallbacks may cross blocks");
    check(total.overlapCells * 10 <= total.wires, "corpus: overlaps with other nets under 0.1 cell per wire");

    std::printf("%d/%d checks passed\n", g_pass, g_pass + g_fail);
    return g_fail == 0 ? 0 : 1;
}
