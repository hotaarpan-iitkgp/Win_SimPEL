#include "Router.hpp"

#include "Placer.hpp"
#include "../BlockRegistry.hpp"
#include "../NetlistBuilder.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <limits>
#include <map>
#include <queue>
#include <set>
#include <string>
#include <unordered_map>
#include <unordered_set>

namespace CircuitSim {

namespace {

std::string upper(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return (char)std::toupper(c); });
    return s;
}

Point2D pt(float x, float y) { Point2D p; p.x = x; p.y = y; return p; }
bool samePt(const Point2D& a, const Point2D& b) { return a.x == b.x && a.y == b.y; }
float snapTo(float v, float g) { return std::round(v / g) * g; }
long long cellKey(float x, float y, float g) {
    const long long ix = (long long)std::llround(x / g), iy = (long long)std::llround(y / g);
    return (ix << 32) ^ (iy & 0xffffffffLL);
}

bool blocked(float x, float y, float endX, float endY, const std::vector<RouteRect>& obs) {
    if (x == endX && y == endY) return false;
    for (const auto& b : obs)
        if (x > b.xMin + 2 && x < b.xMax - 2 && y > b.yMin + 2 && y < b.yMax - 2) return true;
    return false;
}

// Mirrors getOrthogonalTransition in routing.ts.
std::vector<Point2D> transition(const Point2D& p, const Point2D& dir, float gx, float gy) {
    if (p.x == gx && p.y == gy) return {};
    if (dir.x != 0) return {pt(gx, p.y)};
    if (dir.y != 0) return {pt(p.x, gy)};
    return {pt(gx, p.y)};
}

} // namespace

// ── occupancy ───────────────────────────────────────────────────────────────

void RouteOccupancy::addPath(const std::vector<Point2D>& path, int net, float g) {
    grid_ = g;
    for (size_t i = 0; i + 1 < path.size(); ++i) {
        const Point2D a = path[i], b = path[i + 1];
        const bool horizontal = a.y == b.y;
        if (!horizontal && a.x != b.x) continue;   // not axis-aligned (should not happen)
        const float fixed = horizontal ? a.y : a.x;
        if (snapTo(fixed, g) != fixed) continue;    // off-grid stub segment
        float lo = horizontal ? std::min(a.x, b.x) : std::min(a.y, b.y);
        float hi = horizontal ? std::max(a.x, b.x) : std::max(a.y, b.y);
        for (float v = std::ceil(lo / g) * g; v <= hi; v += g) {
            Cell& c = cells_[horizontal ? cellKey(v, fixed, g) : cellKey(fixed, v, g)];
            int& owner = horizontal ? c.netH : c.netV;
            bool& multi = horizontal ? c.multiH : c.multiV;
            if (owner == -1) owner = net;
            else if (owner != net) multi = true;
        }
    }
}

float RouteOccupancy::penalty(float x, float y, bool horizontal, int net, float perCell) const {
    auto it = cells_.find(cellKey(x, y, grid_));
    if (it == cells_.end()) return 0.0f;
    const Cell& c = it->second;
    const int owner = horizontal ? c.netH : c.netV;
    const bool multi = horizontal ? c.multiH : c.multiV;
    if (owner == -1) return 0.0f;
    return (owner != net || multi) ? perCell : 0.0f;
}

// ── paths ───────────────────────────────────────────────────────────────────

std::vector<Point2D> simplifyPath(const std::vector<Point2D>& pts) {
    if (pts.size() < 3) return pts;
    std::vector<Point2D> r{pts[0]};
    for (size_t i = 1; i + 1 < pts.size(); ++i) {
        const Point2D& prev = r.back();
        const Point2D& cur = pts[i];
        const Point2D& next = pts[i + 1];
        if (samePt(cur, prev) || samePt(cur, next)) continue;
        const bool collinear = (prev.x == cur.x && cur.x == next.x) || (prev.y == cur.y && cur.y == next.y);
        if (!collinear) r.push_back(cur);
    }
    r.push_back(pts.back());
    return r;
}

std::vector<Point2D> findAStarPath(Point2D s, Point2D ds, Point2D e, Point2D de,
                                   const std::vector<RouteRect>& obs, const RouteOptions& o,
                                   const RouteOccupancy* occ, int net) {
    (void)de;   // routing.ts takes it but does not use it either
    const float G = o.grid;
    const float sx = snapTo(s.x, G), sy = snapTo(s.y, G), ex = snapTo(e.x, G), ey = snapTo(e.y, G);
    if (sx == ex && sy == ey) return {pt(sx, sy)};
    const float xMin = std::min(sx, ex) - o.margin, xMax = std::max(sx, ex) + o.margin;
    const float yMin = std::min(sy, ey) - o.margin, yMax = std::max(sy, ey) + o.margin;

    struct Node { float x, y; int dx, dy; float g; int parent; long long seq; bool closed; };
    std::vector<Node> nodes;
    std::unordered_map<std::string, int> index;   // "x,y,dx,dy" like routing.ts
    auto key = [](float x, float y, int dx, int dy) {
        return std::to_string((long long)std::llround(x)) + "," + std::to_string((long long)std::llround(y)) + "," +
               std::to_string(dx) + "," + std::to_string(dy);
    };
    struct Entry { float f; long long seq; int node; };
    struct Cmp { bool operator()(const Entry& a, const Entry& b) const { return a.f != b.f ? a.f > b.f : a.seq > b.seq; } };
    std::priority_queue<Entry, std::vector<Entry>, Cmp> open;
    long long seq = 0;
    auto h = [&](float x, float y) { return std::fabs(x - ex) + std::fabs(y - ey); };

    const int sdx = ds.x > 0 ? 1 : (ds.x < 0 ? -1 : 0), sdy = ds.y > 0 ? 1 : (ds.y < 0 ? -1 : 0);
    nodes.push_back({sx, sy, sdx, sdy, 0.0f, -1, seq, false});
    index[key(sx, sy, sdx, sdy)] = 0;
    open.push({h(sx, sy), seq++, 0});

    static const int dirs[4][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};
    int iterations = 0;
    while (!open.empty()) {
        if (++iterations > o.maxIterations) break;
        const Entry top = open.top();
        open.pop();
        Node cur = nodes[(size_t)top.node];
        if (nodes[(size_t)top.node].closed || top.f != cur.g + h(cur.x, cur.y)) { --iterations; continue; }
        if (cur.x == ex && cur.y == ey) {
            std::vector<Point2D> path;
            for (int i = top.node; i >= 0; i = nodes[(size_t)i].parent) path.push_back(pt(nodes[(size_t)i].x, nodes[(size_t)i].y));
            std::reverse(path.begin(), path.end());
            return path;
        }
        nodes[(size_t)top.node].closed = true;
        for (const auto& d : dirs) {
            const float nx = cur.x + d[0] * G, ny = cur.y + d[1] * G;
            if (nx < xMin || nx > xMax || ny < yMin || ny > yMax) continue;
            if (blocked(nx, ny, ex, ey, obs)) continue;
            const std::string nk = key(nx, ny, d[0], d[1]);
            auto it = index.find(nk);
            if (it != index.end() && nodes[(size_t)it->second].closed) continue;
            float step = G;
            if ((cur.dx != 0 || cur.dy != 0) && (cur.dx != d[0] || cur.dy != d[1])) step += o.turnPenalty;
            if (occ) step += occ->penalty(nx, ny, d[1] == 0, net, o.otherNetPenalty);
            const float ng = cur.g + step;
            if (it != index.end()) {
                Node& ex2 = nodes[(size_t)it->second];
                if (ng < ex2.g) {
                    ex2.g = ng;
                    ex2.parent = top.node;
                    open.push({ng + h(nx, ny), ex2.seq, it->second});
                }
            } else {
                const int id = (int)nodes.size();
                nodes.push_back({nx, ny, d[0], d[1], ng, top.node, seq, false});
                index.emplace(nk, id);
                open.push({ng + h(nx, ny), seq++, id});
            }
        }
    }
    return {};
}

std::vector<Point2D> orthogonalPath(Point2D p1, Point2D dir1, Point2D p2, Point2D dir2,
                                    const std::vector<RouteRect>& obs, const RouteOptions& o,
                                    const RouteOccupancy* occ, int net, bool* searchOk) {
    const float d = o.stub;
    const bool z1 = dir1.x == 0 && dir1.y == 0, z2 = dir2.x == 0 && dir2.y == 0;
    const Point2D startExit = z1 ? p1 : pt(p1.x + dir1.x * d, p1.y + dir1.y * d);
    const Point2D endExit = z2 ? p2 : pt(p2.x + dir2.x * d, p2.y + dir2.y * d);

    auto astar = findAStarPath(startExit, dir1, endExit, pt(-dir2.x, -dir2.y), obs, o, occ, net);
    if (searchOk) *searchOk = !astar.empty();
    if (!astar.empty()) {
        const float G = o.grid;
        const float sx = snapTo(startExit.x, G), sy = snapTo(startExit.y, G);
        const float ex = snapTo(endExit.x, G), ey = snapTo(endExit.y, G);
        std::vector<Point2D> full{p1, startExit};
        for (const auto& q : transition(startExit, dir1, sx, sy)) full.push_back(q);
        full.insert(full.end(), astar.begin(), astar.end());
        for (const auto& q : transition(endExit, dir2, ex, ey)) full.push_back(q);
        full.push_back(endExit);
        full.push_back(p2);
        return simplifyPath(full);
    }

    // Heuristic fallback, as in routing.ts.
    std::vector<Point2D> points{p1};
    if (!z1) {
        points.push_back(startExit);
        if (dir1.x != 0) {
            if ((endExit.x - startExit.x) * dir1.x < 0) points.push_back(pt(startExit.x, p1.y + (endExit.y >= p1.y ? 60.0f : -60.0f)));
        } else if ((endExit.y - startExit.y) * dir1.y < 0) {
            points.push_back(pt(p1.x + (endExit.x >= p1.x ? 40.0f : -40.0f), startExit.y));
        }
    }
    std::vector<Point2D> pointsEnd{p2};
    if (!z2) {
        pointsEnd.push_back(endExit);
        if (dir2.x != 0) {
            if ((startExit.x - endExit.x) * dir2.x < 0) pointsEnd.push_back(pt(endExit.x, p2.y + (startExit.y >= p2.y ? 60.0f : -60.0f)));
        } else if ((startExit.y - endExit.y) * dir2.y < 0) {
            pointsEnd.push_back(pt(p2.x + (startExit.x >= p2.x ? 40.0f : -40.0f), endExit.y));
        }
    }
    std::reverse(pointsEnd.begin(), pointsEnd.end());
    const Point2D pA = points.back(), pB = pointsEnd.front();
    if (pA.x != pB.x && pA.y != pB.y) points.push_back(pt(pB.x, pA.y));
    points.insert(points.end(), pointsEnd.begin(), pointsEnd.end());
    return simplifyPath(points);
}

// ── design routing ──────────────────────────────────────────────────────────

namespace {

struct PinGeo { Point2D pos, dir; };

bool pinGeometry(const ComponentInstance& c, const std::string& term, PinGeo& out) {
    const auto pins = instancePins(c);
    const BlockPin* hit = nullptr;
    for (const auto& p : pins) if (p.name == term) { hit = &p; break; }
    if (!hit) for (const auto& p : pins) if (upper(p.name) == upper(term)) { hit = &p; break; }
    if (!hit) return false;
    const float rad = (float)c.rotation * 3.1415926535f / 180.0f;
    const float cs = std::cos(rad), sn = std::sin(rad);
    auto rot = [&](float x, float y) { return pt(x * cs - y * sn, x * sn + y * cs); };
    if (c.rotation == 0) {
        out.pos = pt(c.x + hit->relX, c.y + hit->relY);
        out.dir = pt(hit->dirX, hit->dirY);
    } else {
        const Point2D r = rot(hit->relX, hit->relY);
        out.pos = pt(c.x + r.x, c.y + r.y);
        out.dir = rot(hit->dirX, hit->dirY);
    }
    // Axis-aligned unit direction.
    if (std::fabs(out.dir.x) >= std::fabs(out.dir.y) && out.dir.x != 0) out.dir = pt(out.dir.x > 0 ? 1.0f : -1.0f, 0.0f);
    else if (out.dir.y != 0) out.dir = pt(0.0f, out.dir.y > 0 ? 1.0f : -1.0f);
    else out.dir = pt(0.0f, 0.0f);
    return true;
}

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

bool insideAny(const Point2D& q, const std::vector<RouteRect>& obs) {
    for (const auto& b : obs)
        if (q.x > b.xMin + 2 && q.x < b.xMax - 2 && q.y > b.yMin + 2 && q.y < b.yMax - 2) return true;
    return false;
}

} // namespace

RouteStats routeDesign(CircuitDesign& design, const RouteOptions& o) {
    RouteStats st;
    // See NetlistBuilder::firstGroundPinIsSetRoot: the rebuilt ground net is wired
    // so that this stays as it was (GND attached last when it was the
    // representative, GND as the tree root otherwise).
    std::string groundId;
    const bool groundWasRoot = NetlistBuilder::firstGroundPinIsSetRoot(design, &groundId);
    const float G = o.grid;
    std::unordered_map<std::string, const ComponentInstance*> comps;
    for (const auto& c : design.components) comps[c.id] = &c;

    RouteOccupancy occ;
    std::vector<RouteRect> obs;
    for (const auto& c : design.components) {
        float hw, hh;
        placementExtent(c, hw, hh);
        if (c.rotation % 180 != 0) std::swap(hw, hh);
        obs.push_back({c.x - hw, c.y - hh, c.x + hw, c.y + hh});
    }

    RouteOptions retry = o;
    retry.maxIterations = o.retryMaxIterations;
    retry.margin = o.retryMargin;
    // Pin-to-pin route: the routing.ts budget first, then the larger one.
    auto pinToPin = [&](const PinGeo& a, const PinGeo& b, int net) {
        bool ok = false;
        auto path = orthogonalPath(a.pos, a.dir, b.pos, b.dir, obs, o, &occ, net, &ok);
        if (!ok) {
            ++st.searchRetries;
            path = orthogonalPath(a.pos, a.dir, b.pos, b.dir, obs, retry, &occ, net, &ok);
            if (!ok) ++st.searchFailures;
        }
        return path;
    };

    // Nets over the existing wires.
    std::unordered_set<std::string> wireIds;
    for (const auto& w : design.wires) wireIds.insert(w.id);
    DSU dsu;
    std::vector<std::string> wireNet(design.wires.size());
    std::vector<bool> wireOk(design.wires.size(), true);
    for (size_t i = 0; i < design.wires.size(); ++i) {
        const auto& w = design.wires[i];
        const std::string wn = "wire:" + w.id;
        dsu.find(wn);
        for (const WireEndpoint* ep : {&w.from, &w.to}) {
            if (ep->isWireJunction) {
                if (ep->targetWireId.empty() || !wireIds.count(ep->targetWireId)) wireOk[i] = false;
                else dsu.unite(wn, "wire:" + ep->targetWireId);
            } else if (comps.count(ep->compId)) {
                dsu.unite(wn, "pin:" + ep->compId + ":" + upper(ep->terminal));
            } else {
                wireOk[i] = false;
            }
        }
    }
    // firstWire: index of the earliest original wire touching the pin. New wires
    // are put at that position, because the netlist generator resolves a control
    // input from the first wire in list order whose terminal matches (with
    // aliases: a query for "Ctrl" also matches "In1"); reordering would change
    // which signal such an input reads.
    struct NetPin { std::string comp, term; PinGeo geo; bool control = false, output = false; size_t firstWire = 0; };
    struct Net { std::vector<size_t> wires; std::vector<NetPin> pins; std::set<std::string> seen; bool ok = true; };
    std::map<std::string, Net> nets;   // ordered: deterministic
    std::vector<std::string> netOrder;
    for (size_t i = 0; i < design.wires.size(); ++i) {
        const auto& w = design.wires[i];
        const std::string r = dsu.find("wire:" + w.id);
        if (!nets.count(r)) netOrder.push_back(r);
        Net& n = nets[r];
        n.wires.push_back(i);
        if (!wireOk[i]) n.ok = false;
        for (const WireEndpoint* ep : {&w.from, &w.to}) {
            if (ep->isWireJunction || !comps.count(ep->compId)) continue;
            const std::string k = ep->compId + ":" + upper(ep->terminal);
            if (!n.seen.insert(k).second) continue;   // wires come in index order: first one wins
            NetPin np;
            np.firstWire = i;
            np.comp = ep->compId;
            np.term = ep->terminal;   // keep the spelling the design uses
            if (!pinGeometry(*comps[ep->compId], ep->terminal, np.geo)) { n.ok = false; continue; }
            for (const auto& p : instancePins(*comps[ep->compId]))
                if (upper(p.name) == upper(ep->terminal)) { np.control = p.isControl; np.output = p.isControl && p.dirX > 0.0f; break; }
            n.pins.push_back(np);
        }
    }

    std::vector<bool> drop(design.wires.size(), false);
    std::vector<std::pair<size_t, WireInstance>> added;   // (position key, wire)
    int netIndex = 0;
    for (const auto& r : netOrder) {
        Net& n = nets[r];
        ++netIndex;
        if (!n.ok || n.pins.size() < 2) { ++st.netsKept; continue; }
        // Root: the single driver of a pure signal net; power nets keep their first pin.
        bool power = false;
        std::vector<size_t> drivers;
        for (size_t i = 0; i < n.pins.size(); ++i) {
            if (!n.pins[i].control) power = true;
            else if (n.pins[i].output) drivers.push_back(i);
        }
        if (!power && drivers.size() != 1) { ++st.netsKept; continue; }
        size_t root = (!power) ? drivers[0] : 0;
        size_t groundPin = n.pins.size();
        for (size_t i = 0; i < n.pins.size(); ++i)
            if (!groundId.empty() && n.pins[i].comp == groundId && upper(n.pins[i].term) == "GND") groundPin = i;
        if (groundPin < n.pins.size() && !groundWasRoot) root = groundPin;
        std::vector<size_t> rest;
        for (size_t i = 0; i < n.pins.size(); ++i) if (i != root) rest.push_back(i);
        // Nearest pins first, so the tree grows outwards from the root.
        const Point2D rp = n.pins[root].geo.pos;
        std::stable_sort(rest.begin(), rest.end(), [&](size_t a, size_t b) {
            auto dist = [&](size_t i) { return std::fabs(n.pins[i].geo.pos.x - rp.x) + std::fabs(n.pins[i].geo.pos.y - rp.y); };
            return dist(a) < dist(b);
        });
        if (groundPin < n.pins.size() && groundWasRoot) {   // attach GND last
            rest.erase(std::remove(rest.begin(), rest.end(), groundPin), rest.end());
            rest.push_back(groundPin);
        }

        struct Routed { std::string id; std::vector<Point2D> path; };
        std::vector<Routed> tree;
        int k = 0;
        auto newId = [&]() {
            std::string id;
            do { id = "rw" + std::to_string(netIndex) + "_" + std::to_string(++k); } while (wireIds.count(id));
            wireIds.insert(id);
            return id;
        };
        for (size_t idx = 0; idx < rest.size(); ++idx) {
            const NetPin& target = n.pins[rest[idx]];
            WireInstance w;
            w.id = newId();
            w.to.compId = target.comp;
            w.to.terminal = target.term;
            std::vector<Point2D> path;
            if (idx == 0) {
                const NetPin& src = n.pins[root];
                path = pinToPin(src.geo, target.geo, netIndex);
                w.from.compId = src.comp;
                w.from.terminal = src.term;
            } else {
                // Tap: the grid point on the tree nearest to the target's stub.
                const Point2D stub = pt(target.geo.pos.x + target.geo.dir.x * o.stub, target.geo.pos.y + target.geo.dir.y * o.stub);
                float best = std::numeric_limits<float>::max();
                Point2D tap{};
                std::string tapWire;
                for (const auto& tw : tree) {
                    for (size_t s = 0; s + 1 < tw.path.size(); ++s) {
                        const Point2D a = tw.path[s], b = tw.path[s + 1];
                        const bool hz = a.y == b.y;
                        const float fixed = hz ? a.y : a.x;
                        if (snapTo(fixed, G) != fixed) continue;
                        const float lo = hz ? std::min(a.x, b.x) : std::min(a.y, b.y);
                        const float hi = hz ? std::max(a.x, b.x) : std::max(a.y, b.y);
                        for (float v = std::ceil(lo / G) * G; v <= hi; v += G) {
                            const Point2D q = hz ? pt(v, fixed) : pt(fixed, v);
                            if ((q.x == 0.0f && q.y == 0.0f) || insideAny(q, obs)) continue;
                            const float dd = std::fabs(q.x - stub.x) + std::fabs(q.y - stub.y);
                            if (dd < best) { best = dd; tap = q; tapWire = tw.id; }
                        }
                    }
                }
                if (tapWire.empty()) {
                    // No usable tap (all-off-grid tree): connect pin to pin from the root.
                    const NetPin& src = n.pins[root];
                    path = pinToPin(src.geo, target.geo, netIndex);
                    w.from.compId = src.comp;
                    w.from.terminal = src.term;
                } else {
                    const Point2D endExit = stub;
                    auto astar = findAStarPath(tap, pt(0, 0), endExit, pt(-target.geo.dir.x, -target.geo.dir.y), obs, o, &occ, netIndex);
                    if (astar.empty()) {
                        ++st.searchRetries;
                        astar = findAStarPath(tap, pt(0, 0), endExit, pt(-target.geo.dir.x, -target.geo.dir.y), obs, retry, &occ, netIndex);
                    }
                    std::vector<Point2D> full{tap};
                    if (!astar.empty()) {
                        full.insert(full.end(), astar.begin(), astar.end());
                        for (const auto& q : transition(endExit, target.geo.dir, snapTo(endExit.x, G), snapTo(endExit.y, G))) full.push_back(q);
                    } else {
                        ++st.searchFailures;
                        full.push_back(pt(endExit.x, tap.y));
                    }
                    full.push_back(endExit);
                    full.push_back(target.geo.pos);
                    path = simplifyPath(full);
                    w.from.isWireJunction = true;
                    w.from.targetWireId = tapWire;
                    w.from.junctionX = tap.x;
                    w.from.junctionY = tap.y;
                }
            }
            if (path.size() < 2) path = {path.empty() ? target.geo.pos : path.front(), target.geo.pos};
            w.manualPath = path;
            occ.addPath(path, netIndex, G);
            tree.push_back({w.id, path});
            // The GND wire of a net whose GND must stay representative goes after
            // every other wire, so it is the last union on that set.
            const bool lastUnion = groundWasRoot && rest[idx] == groundPin;
            added.emplace_back(lastUnion ? std::numeric_limits<size_t>::max() : target.firstWire, w);
            ++st.wires;
        }
        for (size_t wi : n.wires) drop[wi] = true;
        ++st.nets;
    }

    // Merge: kept wires at their own index, new wires at their load pin's first
    // original wire (stable for equal keys, kept wires first).
    std::vector<std::pair<std::pair<size_t, int>, WireInstance>> order;
    for (size_t i = 0; i < design.wires.size(); ++i)
        if (!drop[i]) order.push_back({{i, 0}, design.wires[i]});
    int seqNo = 1;
    for (auto& [key, w] : added) order.push_back({{key, seqNo++}, std::move(w)});
    std::stable_sort(order.begin(), order.end(), [](const auto& a, const auto& b) { return a.first < b.first; });
    std::vector<WireInstance> out;
    out.reserve(order.size());
    for (auto& e : order) out.push_back(std::move(e.second));
    design.wires = std::move(out);
    return st;
}

} // namespace CircuitSim
