#include "NetlistBuilder.hpp"
#include <unordered_map>
#include <algorithm>
#include <iostream>

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

    // Unite pins connected by wires
    for (const auto& wire : design.wires) {
        std::string pin1 = wire.from.compId + ":" + wire.from.terminal;
        if (!wire.to.isWireJunction) {
            std::string pin2 = wire.to.compId + ":" + wire.to.terminal;
            dsu.unite(pin1, pin2);
        }
    }

    // Identify Ground nodes (components of type GND / Ground)
    std::string groundRoot = "";
    for (const auto& comp : design.components) {
        std::string t = comp.rawTypeStr;
        std::transform(t.begin(), t.end(), t.begin(), ::toupper);
        if (t == "GND" || t == "GROUND") {
            groundRoot = dsu.find(comp.id + ":p");
            break;
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
