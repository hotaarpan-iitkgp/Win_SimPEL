#include "Placer.hpp"

#include "../BlockRegistry.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <functional>
#include <map>
#include <queue>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

namespace CircuitSim {

namespace {

std::string upper(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return (char)std::toupper(c); });
    return s;
}

float snap(float v, float g) { return std::round(v / g) * g; }

struct DSU {
    std::unordered_map<std::string, std::string> p;
    std::string find(const std::string& x) {
        auto it = p.find(x);
        if (it == p.end()) { p[x] = x; return x; }
        if (it->second == x) return x;
        std::string r = find(it->second);
        p[x] = r;
        return r;
    }
    void unite(const std::string& a, const std::string& b) {
        std::string ra = find(a), rb = find(b);
        if (ra != rb) p[ra] = rb;
    }
};

bool isSourceType(const std::string& type) {
    static const std::set<std::string> kSources = {"V", "I", "AC_V", "AC_I", "V_3PH", "I_3PH", "CTRL_V", "CTRL_I",
                                                   "DC_V", "DC_I", "MMF_SRC", "MMF_SRC_CTRL"};
    const BlockDef* d = BlockRegistry::instance().resolve(type);
    return kSources.count(upper(d ? d->type : type)) > 0;
}

struct Node {
    size_t comp;          // index into design.components
    bool power = false;   // has an electrical / magnetic pin
    bool ground = false;
    std::vector<BlockPin> pins;
    float hw = 40, hh = 40;
    int column = -1;
    double order = 0;     // position within the column
};

} // namespace

void placementExtent(const ComponentInstance& comp, float& halfW, float& halfH) {
    halfW = 40.0f;
    halfH = 30.0f;
    for (const auto& p : instancePins(comp)) {
        halfW = std::max(halfW, std::fabs(p.relX) + 10.0f);
        halfH = std::max(halfH, std::fabs(p.relY) + 10.0f);
    }
}

void autoPlace(CircuitDesign& design, const PlaceOptions& opts) {
    const size_t n = design.components.size();
    if (n == 0) return;
    const float g = opts.grid > 0 ? opts.grid : 20.0f;

    std::vector<Node> nodes(n);
    std::unordered_map<std::string, size_t> byId;
    for (size_t i = 0; i < n; ++i) {
        auto& c = design.components[i];
        c.rotation = 0;
        Node& nd = nodes[i];
        nd.comp = i;
        nd.pins = instancePins(c);
        for (const auto& p : nd.pins) nd.power |= !p.isControl;
        const BlockDef* def = BlockRegistry::instance().resolve(c.rawTypeStr);
        nd.ground = upper(def ? def->type : c.rawTypeStr) == "GND";
        placementExtent(c, nd.hw, nd.hh);
        byId[c.id] = i;
    }

    // ── nets ────────────────────────────────────────────────────────────────
    auto pinKey = [&](const std::string& compId, const std::string& term) {
        auto it = byId.find(compId);
        if (it != byId.end())
            for (const auto& p : nodes[it->second].pins)
                if (upper(p.name) == upper(term)) return compId + ":" + p.name;
        return compId + ":" + term;
    };
    DSU dsu;
    for (const auto& w : design.wires) {
        const std::string wn = "wire:" + w.id;
        for (const WireEndpoint* ep : {&w.from, &w.to}) {
            if (ep->isWireJunction) { if (!ep->targetWireId.empty()) dsu.unite(wn, "wire:" + ep->targetWireId); }
            else if (!ep->compId.empty()) dsu.unite(wn, pinKey(ep->compId, ep->terminal));
        }
    }
    struct Member { size_t node; const BlockPin* pin; };
    std::map<std::string, std::vector<Member>> nets;   // ordered for determinism
    for (size_t i = 0; i < n; ++i) {
        const std::string& id = design.components[i].id;
        for (const auto& p : nodes[i].pins) {
            const std::string k = id + ":" + p.name;
            if (!dsu.p.count(k)) continue;   // unconnected pin
            nets[dsu.find(k)].push_back({i, &p});
        }
    }

    // ── adjacency ───────────────────────────────────────────────────────────
    std::vector<std::set<size_t>> powerAdj(n);
    std::vector<std::set<size_t>> sigOut(n), sigIn(n);
    for (const auto& [root, members] : nets) {
        bool hasGround = false, anyPower = false;
        for (const auto& m : members) { hasGround |= nodes[m.node].ground; anyPower |= !m.pin->isControl; }
        if (anyPower) {
            if (hasGround) continue;   // the ground net touches everything
            for (const auto& a : members)
                for (const auto& b : members)
                    if (a.node != b.node && nodes[a.node].power && nodes[b.node].power) powerAdj[a.node].insert(b.node);
            continue;
        }
        std::vector<size_t> drivers, loads;
        for (const auto& m : members) (m.pin->dirX > 0.0f ? drivers : loads).push_back(m.node);
        for (size_t d : drivers)
            for (size_t l : loads)
                if (d != l) { sigOut[d].insert(l); sigIn[l].insert(d); }
    }

    // ── power columns: BFS from the sources ─────────────────────────────────
    std::vector<size_t> powerNodes, controlNodes, isolated;
    for (size_t i = 0; i < n; ++i) {
        if (nodes[i].ground) continue;
        if (nodes[i].power) powerNodes.push_back(i);
        else if (nodes[i].pins.empty()) isolated.push_back(i);
        else controlNodes.push_back(i);
    }
    {
        std::vector<size_t> roots;
        for (size_t i : powerNodes) if (isSourceType(design.components[i].rawTypeStr)) roots.push_back(i);
        std::queue<size_t> q;
        auto bfsFrom = [&](size_t r) {
            nodes[r].column = 0;
            q.push(r);
            while (!q.empty()) {
                size_t u = q.front(); q.pop();
                for (size_t v : powerAdj[u])
                    if (nodes[v].column < 0) { nodes[v].column = nodes[u].column + 1; q.push(v); }
            }
        };
        for (size_t r : roots) if (nodes[r].column < 0) bfsFrom(r);
        for (size_t i : powerNodes) if (nodes[i].column < 0) bfsFrom(i);
    }

    // ── control columns: longest path over the signal DAG ──────────────────
    {
        std::set<size_t> ctrl(controlNodes.begin(), controlNodes.end());
        // Break cycles: DFS from nodes in index order; edges to a node on the stack
        // are feedback and ignored.
        std::vector<int> state(n, 0);   // 0 new, 1 on stack, 2 done
        std::vector<std::vector<size_t>> dag(n);
        std::vector<size_t> topo;
        std::function<void(size_t)> dfs = [&](size_t u) {
            state[u] = 1;
            for (size_t v : sigOut[u]) {
                if (!ctrl.count(v)) continue;
                if (state[v] == 1) continue;           // feedback edge
                dag[u].push_back(v);
                if (state[v] == 0) dfs(v);
            }
            state[u] = 2;
            topo.push_back(u);
        };
        // Start from signal sources first so that feedback is broken at the loop's
        // far end, not at its entry.
        std::vector<size_t> starts;
        for (size_t i : controlNodes) {
            bool driven = false;
            for (size_t d : sigIn[i]) driven |= ctrl.count(d) > 0;
            if (!driven) starts.push_back(i);
        }
        for (size_t i : controlNodes) starts.push_back(i);
        for (size_t s : starts) if (state[s] == 0) dfs(s);
        std::reverse(topo.begin(), topo.end());
        for (size_t u : topo) if (nodes[u].column < 0) nodes[u].column = 0;
        for (size_t u : topo)
            for (size_t v : dag[u]) nodes[v].column = std::max(nodes[v].column, nodes[u].column + 1);
    }

    // ── ordering within columns (barycentre, two sweeps) ────────────────────
    auto arrange = [&](const std::vector<size_t>& group, const std::function<std::vector<size_t>(size_t)>& neighbours) {
        int maxCol = 0;
        for (size_t i : group) maxCol = std::max(maxCol, nodes[i].column);
        std::vector<std::vector<size_t>> cols((size_t)maxCol + 1);
        for (size_t i : group) cols[(size_t)nodes[i].column].push_back(i);   // index order = initial order
        for (auto& col : cols)
            for (size_t r = 0; r < col.size(); ++r) nodes[col[r]].order = (double)r;
        for (int pass = 0; pass < 2; ++pass) {
            for (size_t c = 1; c < cols.size(); ++c) {
                for (size_t i : cols[c]) {
                    double sum = 0; int cnt = 0;
                    for (size_t j : neighbours(i))
                        if (nodes[j].column == (int)c - 1) { sum += nodes[j].order; ++cnt; }
                    if (cnt) nodes[i].order = sum / cnt;
                }
                std::stable_sort(cols[c].begin(), cols[c].end(),
                                 [&](size_t a, size_t b) { return nodes[a].order < nodes[b].order; });
                for (size_t r = 0; r < cols[c].size(); ++r) nodes[cols[c][r]].order = (double)r;
            }
        }
        return cols;
    };
    auto powerCols = arrange(powerNodes, [&](size_t i) {
        return std::vector<size_t>(powerAdj[i].begin(), powerAdj[i].end());
    });
    auto controlCols = arrange(controlNodes, [&](size_t i) {
        return std::vector<size_t>(sigIn[i].begin(), sigIn[i].end());
    });

    // ── coordinates ─────────────────────────────────────────────────────────
    auto placeSection = [&](const std::vector<std::vector<size_t>>& cols, float top) {
        float x = 0.0f, bottom = top;
        for (const auto& col : cols) {
            if (col.empty()) continue;
            float colHalfW = 0.0f;
            for (size_t i : col) colHalfW = std::max(colHalfW, nodes[i].hw);
            const float cx = snap(x + colHalfW, g);
            float y = top;
            for (size_t i : col) {
                auto& c = design.components[nodes[i].comp];
                const float cy = snap(y + nodes[i].hh, g);
                c.x = cx;
                c.y = cy;
                y = cy + nodes[i].hh + opts.rowGap;
                bottom = std::max(bottom, cy + nodes[i].hh);
            }
            x = cx + colHalfW + opts.columnGap;
        }
        return bottom;
    };
    float y0 = 0.0f;
    float powerBottom = powerNodes.empty() ? y0 : placeSection(powerCols, y0);
    // Ground: below the power stage, under its first column.
    float next = powerNodes.empty() ? y0 : powerBottom + opts.rowGap;
    float gx = 0.0f;
    for (size_t i = 0; i < n; ++i) {
        if (!nodes[i].ground) continue;
        auto& c = design.components[i];
        c.x = snap(gx + nodes[i].hw, g);
        c.y = snap(next + nodes[i].hh, g);
        gx = c.x + nodes[i].hw + opts.columnGap;
        powerBottom = std::max(powerBottom, c.y + nodes[i].hh);
    }
    float controlTop = (powerNodes.empty() && gx == 0.0f) ? y0 : powerBottom + opts.sectionGap;
    float controlBottom = controlNodes.empty() ? controlTop : placeSection(controlCols, controlTop);
    // Blocks without pins (e.g. the loss analyser): a row at the bottom.
    float ix = 0.0f;
    const float isoTop = controlBottom + opts.sectionGap;
    for (size_t i : isolated) {
        auto& c = design.components[i];
        c.x = snap(ix + nodes[i].hw, g);
        c.y = snap(isoTop + nodes[i].hh, g);
        ix = c.x + nodes[i].hw + opts.columnGap;
    }

    // ── wires: junction points onto their target wire; drop stale paths ─────
    std::unordered_map<std::string, const WireInstance*> wireById;
    for (const auto& w : design.wires) wireById[w.id] = &w;
    auto pinPos = [&](const WireEndpoint& ep, float& x, float& y) {
        auto it = byId.find(ep.compId);
        if (ep.isWireJunction || it == byId.end()) return false;
        const auto& c = design.components[it->second];
        x = c.x; y = c.y;
        for (const auto& p : nodes[it->second].pins)
            if (upper(p.name) == upper(ep.terminal)) { x = c.x + p.relX; y = c.y + p.relY; break; }
        return true;
    };
    // Only junctions whose target wire ends on a pin are moved: the netlist builder
    // resolves those through the pin, so their coordinates never name a node.
    for (auto& w : design.wires) {
        w.manualPath.clear();
        for (WireEndpoint* ep : {&w.from, &w.to}) {
            if (!ep->isWireJunction) continue;
            auto it = wireById.find(ep->targetWireId);
            if (it == wireById.end()) continue;   // unresolved junctions keep their coordinates
            float ax, ay, bx, by;
            const bool fa = pinPos(it->second->from, ax, ay), fb = pinPos(it->second->to, bx, by);
            if (fa && fb) { ep->junctionX = snap((ax + bx) * 0.5f, g); ep->junctionY = snap((ay + by) * 0.5f, g); }
            else if (fa) { ep->junctionX = ax; ep->junctionY = ay; }
            else if (fb) { ep->junctionX = bx; ep->junctionY = by; }
        }
    }
}

} // namespace CircuitSim
