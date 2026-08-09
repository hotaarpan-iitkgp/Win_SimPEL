#include "NetlistBuilder.hpp"
#include <unordered_map>
#include <unordered_set>
#include <algorithm>
#include <iostream>
#include <cmath>

namespace CircuitSim {

struct DSU {
    std::unordered_map<std::string, std::string> parent;

    std::string find(const std::string& i) {
        if (parent.find(i) == parent.end()) {
            parent[i] = i;
            return i;
        }
        if (parent[i] == i)
            return i;
        return parent[i] = find(parent[i]);
    }

    void unite(const std::string& i, const std::string& j) {
        std::string root_i = find(i);
        std::string root_j = find(j);
        if (root_i != root_j) {
            parent[root_i] = root_j;
        }
    }
};

static bool isTerminalMatch(const std::string& compTypeStr, const std::string& termA, const std::string& termB) {
    if (termA == termB) return true;
    std::string a = termA, b = termB;
    std::transform(a.begin(), a.end(), a.begin(), ::toupper);
    std::transform(b.begin(), b.end(), b.begin(), ::toupper);
    if (a == b) return true;

    bool isP1_A = (a == "P1" || a == "P1A" || a == "P1_1" || a == "PA");
    bool isP1_B = (b == "P1" || b == "P1A" || b == "P1_1" || b == "PA");
    if (isP1_A && isP1_B) return true;

    bool isP2_A = (a == "P2" || a == "P1B" || a == "P1_2" || a == "PB");
    bool isP2_B = (b == "P2" || b == "P1B" || b == "P1_2" || b == "PB");
    if (isP2_A && isP2_B) return true;

    bool isS1_A = (a == "S1" || a == "S1A" || a == "S1_1" || a == "SA");
    bool isS1_B = (b == "S1" || b == "S1A" || b == "S1_1" || b == "SA");
    if (isS1_A && isS1_B) return true;

    bool isS2_A = (a == "S2" || a == "S1B" || a == "S1_2" || a == "SB" || a == "S2A");
    bool isS2_B = (b == "S2" || b == "S1B" || b == "S1_2" || b == "SB" || b == "S2A");
    if (isS2_A && isS2_B) return true;

    return false;
}

static std::string getResolvedPin(const WireEndpoint& ep, const CircuitDesign& design, const std::unordered_map<std::string, WireInstance>& wireMap, std::unordered_set<std::string>& visited) {
    if (!ep.compId.empty()) {
        for (const auto& comp : design.components) {
            if (comp.id == ep.compId) {
                auto terms = getTerminals(comp);
                for (const auto& term : terms) {
                    if (isTerminalMatch(comp.rawTypeStr, term.name, ep.terminal)) {
                        return comp.id + ":" + term.name;
                    }
                }
                break;
            }
        }
        return ep.compId + ":" + ep.terminal;
    }
    if (ep.isWireJunction) {
        if (!ep.targetWireId.empty() && wireMap.count(ep.targetWireId) && visited.find(ep.targetWireId) == visited.end()) {
            visited.insert(ep.targetWireId);
            const auto& targetW = wireMap.at(ep.targetWireId);
            std::string p = getResolvedPin(targetW.from, design, wireMap, visited);
            if (!p.empty()) return p;
            p = getResolvedPin(targetW.to, design, wireMap, visited);
            if (!p.empty()) return p;
        }
        // Fallback: Spatial search for closest component terminal at (junctionX, junctionY)
        float jx = ep.junctionX;
        float jy = ep.junctionY;
        float minD = 1e9f;
        std::string bestPin = "";
        
        for (const auto& comp : design.components) {
            auto terms = getTerminals(comp);
            for (const auto& term : terms) {
                float rad = comp.rotation * 3.14159265f / 180.0f;
                float rx = term.relX * std::cos(rad) - term.relY * std::sin(rad);
                float ry = term.relX * std::sin(rad) + term.relY * std::cos(rad);
                float px = comp.x + rx;
                float py = comp.y + ry;
                float dist = std::sqrt((px - jx)*(px - jx) + (py - jy)*(py - jy));
                if (dist < minD) {
                    minD = dist;
                    bestPin = comp.id + ":" + term.name;
                }
            }
        }
        if (minD <= 25.0f) return bestPin;
    }
    return "";
}

void NetlistBuilder::buildNodesForCircuit(CircuitDesign& design) {
    DSU dsu;

    // Register all pins for all components
    for (auto& comp : design.components) {
        auto terms = getTerminals(comp);
        for (const auto& term : terms) {
            std::string pinId = comp.id + ":" + term.name;
            dsu.find(pinId);
        }
    }

    std::unordered_map<std::string, WireInstance> wireMap;
    for (const auto& w : design.wires) {
        wireMap[w.id] = w;
    }

    // Unite pins connected by wires and wire junctions
    for (const auto& wire : design.wires) {
        std::unordered_set<std::string> visited1, visited2;
        std::string pin1 = getResolvedPin(wire.from, design, wireMap, visited1);
        std::string pin2 = getResolvedPin(wire.to, design, wireMap, visited2);

        if (!pin1.empty() && !pin2.empty() && pin1 != pin2) {
            dsu.unite(pin1, pin2);
        }
    }

    // Identify Ground nodes (components of type GND / Ground or fallback to V1 negative terminal)
    std::string groundRoot = "";
    for (const auto& comp : design.components) {
        std::string t = comp.rawTypeStr;
        std::transform(t.begin(), t.end(), t.begin(), ::toupper);
        if (t == "GND" || t == "GROUND") {
            groundRoot = dsu.find(comp.id + ":Gnd");
            if (groundRoot.empty() || groundRoot == comp.id + ":Gnd") {
                groundRoot = dsu.find(comp.id + ":p");
            }
            break;
        }
    }

    if (groundRoot.empty()) {
        // Fallback: Default Ground to negative pin ("B" / "n") of first Voltage Source
        for (const auto& comp : design.components) {
            std::string t = comp.rawTypeStr;
            std::transform(t.begin(), t.end(), t.begin(), ::toupper);
            if (t == "V" || t == "AC_V" || comp.type == ComponentType::VoltageSource) {
                std::string pinB = dsu.find(comp.id + ":B");
                if (!pinB.empty()) {
                    groundRoot = pinB;
                    break;
                }
            }
        }
    }

    // Map roots to node names ("0" for Ground, "node_1", "node_2", etc.)
    std::unordered_map<std::string, std::string> rootToNode;
    if (!groundRoot.empty()) {
        rootToNode[groundRoot] = "0";
    }

    int nodeCounter = 1;
    for (auto& comp : design.components) {
        auto terms = getTerminals(comp);
        comp.nodes.clear();
        for (const auto& term : terms) {
            std::string pinId = comp.id + ":" + term.name;
            std::string root = dsu.find(pinId);

            if (rootToNode.find(root) == rootToNode.end()) {
                rootToNode[root] = "node_" + std::to_string(nodeCounter++);
            }
            comp.nodes.push_back(rootToNode[root]);
        }
    }
}

} // namespace CircuitSim
