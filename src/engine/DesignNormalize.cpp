// Wire normalisation the schematic canvas applies whenever a design is loaded into
// it (SchematicCanvas::setCircuit). Moved verbatim from ui/SchematicCanvas.cpp so the
// headless paths can apply exactly the same rules; ImVec2 replaced by a POD point.
#include "DesignNormalize.hpp"
#include <algorithm>
#include <cmath>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace CircuitSim {

namespace {
struct Pt { float x, y; Pt(float x_, float y_) : x(x_), y(y_) {} };
} // namespace

static bool isTerminalMatch(const std::string& compTypeStr, const std::string& termA, const std::string& termB) {
    if (termA == termB) return true;
    std::string a = termA, b = termB;
    std::transform(a.begin(), a.end(), a.begin(), ::toupper);
    std::transform(b.begin(), b.end(), b.begin(), ::toupper);
    if (a == b) return true;

    // Top pin aliases (Plus / In1 / A / In / G)
    bool isTopA = (a == "A" || a == "PLUS" || a == "IN1" || a == "IN" || a == "INPUT_0" || a == "IN_0");
    bool isTopB = (b == "A" || b == "PLUS" || b == "IN1" || b == "IN" || b == "INPUT_0" || b == "IN_0");
    if (isTopA && isTopB) return true;

    // Bottom pin aliases (Minus / In2 / B)
    bool isBotA = (a == "B" || a == "MINUS" || a == "IN2" || a == "INPUT_1" || a == "IN_1");
    bool isBotB = (b == "B" || b == "MINUS" || b == "IN2" || b == "INPUT_1" || b == "IN_1");
    if (isBotA && isBotB) return true;

    // Output pin aliases (Out / Out1 / Output / Direct)
    bool isOutA = (a == "OUT" || a == "OUT1" || a == "OUTPUT" || a == "OUTPUT_0" || a == "OUTDIRECT1");
    bool isOutB = (b == "OUT" || b == "OUT1" || b == "OUTPUT" || b == "OUTPUT_0" || b == "OUTDIRECT1");
    if (isOutA && isOutB) return true;

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

    return false;
}

void cleanupOrphanedJunctions(CircuitDesign& design) {
    std::unordered_set<std::string> validWireIds;
    for (const auto& w : design.wires) {
        validWireIds.insert(w.id);
    }

    // Clear junction flag if targetWireId no longer exists in design.wires
    for (auto& w : design.wires) {
        if (w.from.isWireJunction && !w.from.targetWireId.empty()) {
            if (validWireIds.count(w.from.targetWireId) == 0) {
                w.from.isWireJunction = false;
                w.from.targetWireId.clear();
            }
        }
        if (w.to.isWireJunction && !w.to.targetWireId.empty()) {
            if (validWireIds.count(w.to.targetWireId) == 0) {
                w.to.isWireJunction = false;
                w.to.targetWireId.clear();
            }
        }
    }
}

void consolidateOverlappingWires(CircuitDesign& design) {
    if (design.wires.empty() || design.components.empty()) return;

    // Component map for fast coordinate lookups
    std::unordered_map<std::string, const ComponentInstance*> compMap;
    compMap.reserve(design.components.size());
    for (const auto& comp : design.components) compMap[comp.id] = &comp;

    // Group direct wire connections by component terminal "compId.terminal"
    struct PinConn {
        size_t wireIdx;
        bool isFrom;
    };
    std::unordered_map<std::string, std::vector<PinConn>> termConnMap;

    for (size_t wi = 0; wi < design.wires.size(); ++wi) {
        const auto& w = design.wires[wi];
        if (!w.from.isWireJunction && !w.from.compId.empty() && !w.from.terminal.empty()) {
            std::string key = w.from.compId + "." + w.from.terminal;
            termConnMap[key].push_back({wi, true});
        }
        if (!w.to.isWireJunction && !w.to.compId.empty() && !w.to.terminal.empty()) {
            std::string key = w.to.compId + "." + w.to.terminal;
            termConnMap[key].push_back({wi, false});
        }
    }

    // For any terminal with > 1 incoming/outgoing direct wires, convert subsequent wires to junctions
    for (const auto& [termKey, conns] : termConnMap) {
        if (conns.size() <= 1) continue;

        size_t dotPos = termKey.find('.');
        if (dotPos == std::string::npos) continue;
        std::string compId = termKey.substr(0, dotPos);
        std::string termName = termKey.substr(dotPos + 1);

        auto itC = compMap.find(compId);
        if (itC == compMap.end()) continue;
        const ComponentInstance& comp = *(itC->second);

        // Find terminal coordinate and port stub point in world space
        Pt pinPos(comp.x, comp.y);
        Pt stubPos(comp.x, comp.y);
        auto terms = getTerminals(comp);
        for (const auto& t : terms) {
            if (isTerminalMatch(comp.rawTypeStr, t.name, termName)) {
                float rad = comp.rotation * 3.14159265f / 180.0f;
                float rx = t.relX * std::cos(rad) - t.relY * std::sin(rad);
                float ry = t.relX * std::sin(rad) + t.relY * std::cos(rad);
                pinPos = Pt(comp.x + rx, comp.y + ry);

                float dx = t.dx * std::cos(rad) - t.dy * std::sin(rad);
                float dy = t.dx * std::sin(rad) + t.dy * std::cos(rad);
                stubPos = Pt(pinPos.x + dx * 20.0f, pinPos.y + dy * 20.0f);
                break;
            }
        }

        // Primary wire stays directly connected
        std::string mainWireId = design.wires[conns[0].wireIdx].id;

        // Subsequent wires connect as junction onto mainWire at stubPos
        for (size_t i = 1; i < conns.size(); ++i) {
            size_t subIdx = conns[i].wireIdx;
            if (subIdx >= design.wires.size()) continue;
            WireInstance& subW = design.wires[subIdx];

            if (conns[i].isFrom) {
                subW.from.isWireJunction = true;
                subW.from.targetWireId = mainWireId;
                subW.from.junctionX = stubPos.x;
                subW.from.junctionY = stubPos.y;
                subW.from.compId.clear();
                subW.from.terminal.clear();
                subW.fromNode = mainWireId + ".junction";
            } else {
                subW.to.isWireJunction = true;
                subW.to.targetWireId = mainWireId;
                subW.to.junctionX = stubPos.x;
                subW.to.junctionY = stubPos.y;
                subW.to.compId.clear();
                subW.to.terminal.clear();
                subW.toNode = mainWireId + ".junction";
            }
        }
    }
}

void normaliseDesignWires(CircuitDesign& design) {
    cleanupOrphanedJunctions(design);
    consolidateOverlappingWires(design);
}

} // namespace CircuitSim
