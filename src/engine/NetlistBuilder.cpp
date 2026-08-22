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

    // Primary 1 Top (P1, P1A, P1_1, PA)
    bool isP1A_A = (a == "P1" || a == "P1A" || a == "P1_1" || a == "PA");
    bool isP1A_B = (b == "P1" || b == "P1A" || b == "P1_1" || b == "PA");
    if (isP1A_A && isP1A_B) return true;

    // Primary 1 Bottom (P2, P1B, P1_2, PB)
    bool isP1B_A = (a == "P2" || a == "P1B" || a == "P1_2" || a == "PB");
    bool isP1B_B = (b == "P2" || b == "P1B" || b == "P1_2" || b == "PB");
    if (isP1B_A && isP1B_B) return true;

    // Secondary 1 Top (S1, S1A, S1_1, SA)
    bool isS1A_A = (a == "S1" || a == "S1A" || a == "S1_1" || a == "SA");
    bool isS1A_B = (b == "S1" || b == "S1A" || b == "S1_1" || b == "SA");
    if (isS1A_A && isS1A_B) return true;

    // Secondary 1 Bottom (S2, S1B, S1_2, SB)
    bool isS1B_A = (a == "S2" || a == "S1B" || a == "S1_2" || a == "SB");
    bool isS1B_B = (b == "S2" || b == "S1B" || b == "S1_2" || b == "SB");
    if (isS1B_A && isS1B_B) return true;

    // Secondary 2 Top (S2A, S3, S2_1)
    bool isS2A_A = (a == "S2A" || a == "S3" || a == "S2_1");
    bool isS2A_B = (b == "S2A" || b == "S3" || b == "S2_1");
    if (isS2A_A && isS2A_B) return true;

    // Secondary 2 Bottom (S2B, S4, S2_2)
    bool isS2B_A = (a == "S2B" || a == "S4" || a == "S2_2");
    bool isS2B_B = (b == "S2B" || b == "S4" || b == "S2_2");
    if (isS2B_A && isS2B_B) return true;

    // Control / Signal Input pins (CTRL, IN, IN1, GATE, G)
    bool isCtrlA = (a == "CTRL" || a == "IN" || a == "IN1" || a == "GATE" || a == "G");
    bool isCtrlB = (b == "CTRL" || b == "IN" || b == "IN1" || b == "GATE" || b == "G");
    if (isCtrlA && isCtrlB) return true;

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
        // Junction without valid targetWireId: return unique coordinate-based junction ID
        char jbuf[64];
        snprintf(jbuf, sizeof(jbuf), "Junc_%.1f_%.1f", ep.junctionX, ep.junctionY);
        return std::string(jbuf);
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
                groundRoot = dsu.find(comp.id + ":GND");
            }
            if (groundRoot.empty() || groundRoot == comp.id + ":GND") {
                groundRoot = dsu.find(comp.id + ":p");
            }
            if (groundRoot.empty() || groundRoot == comp.id + ":p") {
                groundRoot = dsu.find(comp.id + ":A");
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
