#include "SchematicCanvas.hpp"
#include "imgui_internal.h"
#include <nlohmann/json.hpp>
#include <windows.h>
#include <commdlg.h>
#include <cmath>
#include <algorithm>
#include <iostream>
#include <sstream>
#include <fstream>
#include <unordered_set>

namespace CircuitSim {

static std::string g_internalClipboard;

enum class DomainType { Power, Control };

static ImVec2 rotatePt(float px, float py, float cx, float cy, float angleDeg) {
    if (angleDeg == 0.0f) return ImVec2(cx + px, cy + py);
    float rad = angleDeg * 3.1415926535f / 180.0f;
    float cosA = std::cos(rad);
    float sinA = std::sin(rad);
    float rx = px * cosA - py * sinA;
    float ry = px * sinA + py * cosA;
    return ImVec2(cx + rx, cy + ry);
}

static ImVec2 getClosestPointOnSegment(ImVec2 p, ImVec2 a, ImVec2 b, float& outDist) {
    float l2 = (b.x - a.x)*(b.x - a.x) + (b.y - a.y)*(b.y - a.y);
    if (l2 == 0) {
        outDist = std::sqrt((p.x - a.x)*(p.x - a.x) + (p.y - a.y)*(p.y - a.y));
        return a;
    }
    float t = std::max(0.0f, std::min(1.0f, ((p.x - a.x)*(b.x - a.x) + (p.y - a.y)*(b.y - a.y)) / l2));
    ImVec2 proj(a.x + t * (b.x - a.x), a.y + t * (b.y - a.y));
    outDist = std::sqrt((p.x - proj.x)*(p.x - proj.x) + (p.y - proj.y)*(p.y - proj.y));
    return proj;
}

std::vector<TerminalDef> getTerminals(const ComponentInstance& comp) {
    const std::string& t = comp.rawTypeStr;
    
    if (t == "R" || t == "L" || t == "C" || t == "V" || t == "I" || t == "D" || t == "AC_V") {
        return {{"A", 0, -40, 0, -1, false}, {"B", 0, 40, 0, 1, false}};
    }
    if (t == "VM" || t == "AM") {
        return {{"A", 0, -40, 0, -1, false}, {"B", 0, 40, 0, 1, false}, {"Out", 20, 0, 1, 0, true}};
    }
    if (t == "MOSFET" || t == "vg-FET") {
        return {{"D", 0, -40, 0, -1, false}, {"S", 0, 40, 0, 1, false}, {"G", -20, 0, -1, 0, true}};
    }
    if (t == "S") {
        return {{"A", 0, -40, 0, -1, false}, {"B", 0, 40, 0, 1, false}, {"Ctrl", -20, 0, -1, 0, true}};
    }
    if (t == "GND") {
        return {{"Gnd", 0, -20, 0, -1, false}};
    }
    if (t == "CONST" || t == "TRI" || t == "TRI_GEN" || t == "PULSE" || t == "PULSE_GEN" || t == "STEP" || t == "RAMP" || t == "SINE_WAVE" || t == "CLOCK") {
        return {{"Out", 20, 0, 1, 0, true}};
    }
    if (t == "GAIN" || t == "PID" || t == "PWM" || t == "FCN" || t == "NOT") {
        return {{"In", -20, 0, -1, 0, true}, {"Out", 20, 0, 1, 0, true}};
    }
    if (t == "SUM" || t == "SUM_ROUND" || t == "SUM_RECT" || t == "PROD" || t == "PRODUCT_RECT" || t == "COMP" || t == "AND" || t == "OR") {
        return {{"A", -20, -20, -1, 0, true}, {"B", -20, 20, -1, 0, true}, {"Out", 20, 0, 1, 0, true}};
    }
    if (t == "SCOPE") {
        int numCh = 2;
        if (comp.parameters.count("channels")) {
            try { numCh = std::stoi(comp.parameters.at("channels")); } catch (...) {}
        }
        if (numCh < 1) numCh = 1;
        if (numCh > 8) numCh = 8;

        std::vector<TerminalDef> terms;
        for (int i = 0; i < numCh; ++i) {
            float yOff = (numCh > 1) ? (-10.0f * (numCh - 1) + 20.0f * i) : 0.0f;
            std::string termName = "In" + std::to_string(i + 1);
            terms.push_back({termName, -16.0f, yOff, -1.0f, 0.0f, true});
        }
        return terms;
    }
    if (t == "PROBE") {
        std::string sigStr = comp.parameters.count("selected_signals") ? comp.parameters.at("selected_signals") : "";
        std::vector<std::string> sigs;
        std::stringstream ss(sigStr);
        std::string item;
        while (std::getline(ss, item, ',')) { if (!item.empty()) sigs.push_back(item); }

        std::vector<TerminalDef> terms;
        if (sigs.empty()) {
            terms.push_back({"Out", 30.0f, 0.0f, 1.0f, 0.0f, true});
        } else {
            int n = (int)sigs.size();
            for (int i = 0; i < n; ++i) {
                float yOff = (n > 1) ? (-15.0f * (n - 1) + 30.0f * i) : 0.0f;
                terms.push_back({sigs[i], 30.0f, yOff, 1.0f, 0.0f, true});
            }
        }
        return terms;
    }
    if (t == "MUX") {
        return {{"In1", -20, -12, -1, 0, true}, {"In2", -20, 12, -1, 0, true}, {"Out", 20, 0, 1, 0, true}};
    }
    if (t == "CSCRIPT") {
        return {{"In1", -80, 0, -1, 0, true}, {"Out1", 80, -30, 1, 0, true}, {"Out2", 80, -10, 1, 0, true}, {"Out3", 80, 10, 1, 0, true}, {"Out4", 80, 30, 1, 0, true}};
    }
    return {};
}

static DomainType getPinDomain(const ComponentInstance& comp, const std::string& pinName) {
    std::string t = comp.rawTypeStr;
    std::transform(t.begin(), t.end(), t.begin(), ::toupper);
    std::string p = pinName;
    std::transform(p.begin(), p.end(), p.begin(), ::toupper);

    if ((t == "MOSFET" || t == "VG-FET") && (p == "G" || p == "GATE")) return DomainType::Control;
    if (t == "S" && (p == "CTRL" || p == "GATE")) return DomainType::Control;
    if ((t == "VM" || t == "AM") && (p == "OUT" || p == "V" || p == "I")) return DomainType::Control;

    if (t == "GAIN" || t == "PID" || t == "PWM" || t == "TRI" || t == "TRI_GEN" || t == "PULSE" || t == "PULSE_GEN" || t == "CONST" || t == "STEP" || t == "RAMP" || t == "SINE_WAVE" || t == "CLOCK" ||
        t == "SUM" || t == "SUM_ROUND" || t == "SUM_RECT" ||
        t == "PROD" || t == "PRODUCT_RECT" || t == "COMP" ||
        t == "AND" || t == "OR" || t == "NOT" || t == "FCN" ||
        t == "CSCRIPT" || t == "SCOPE" || t == "PROBE" || t == "MUX" || t == "DEMUX" || t == "KEY_TRIGGER") {
        return DomainType::Control;
    }

    return DomainType::Power;
}

static DomainType getWireDomain(const WireInstance& wire, const CircuitDesign& design) {
    for (const auto& comp : design.components) {
        if (comp.id == wire.from.compId) {
            return getPinDomain(comp, wire.from.terminal);
        }
    }
    return DomainType::Power;
}

static bool isControlOutputPin(const ComponentInstance& comp, const std::string& pinName) {
    if (getPinDomain(comp, pinName) != DomainType::Control) return false;
    std::string p = pinName;
    std::transform(p.begin(), p.end(), p.begin(), ::toupper);
    if (p.rfind("OUT", 0) == 0 || p == "OUT" || p == "OUT1" || p == "OUT2" || p == "OUT3" || p == "OUT4" || p == "D1" || p == "C1") {
        return true;
    }
    return false;
}

bool SchematicCanvas::validateSingleOutportConstraint(const std::string& startCompId, const std::string& startPin, const std::string& targetCompId, const std::string& targetPin) const {
    const ComponentInstance* startComp = nullptr;
    const ComponentInstance* targetComp = nullptr;
    for (const auto& c : design.components) {
        if (c.id == startCompId) startComp = &c;
        if (c.id == targetCompId) targetComp = &c;
    }
    if (!startComp || !targetComp) return true;

    if (getPinDomain(*startComp, startPin) == DomainType::Control) {
        bool startIsOut = isControlOutputPin(*startComp, startPin);
        bool targetIsOut = isControlOutputPin(*targetComp, targetPin);

        if (startIsOut && targetIsOut) {
            return false;
        }
    }
    return true;
}

void SchematicCanvas::normalizeControlWires() {
    for (auto& wire : design.wires) {
        if (wire.to.isWireJunction) continue;

        const ComponentInstance* fromComp = nullptr;
        const ComponentInstance* toComp = nullptr;
        for (const auto& c : design.components) {
            if (c.id == wire.from.compId) fromComp = &c;
            if (c.id == wire.to.compId) toComp = &c;
        }

        if (fromComp && toComp) {
            if (getPinDomain(*fromComp, wire.from.terminal) == DomainType::Control) {
                bool fromIsOut = isControlOutputPin(*fromComp, wire.from.terminal);
                bool toIsOut = isControlOutputPin(*toComp, wire.to.terminal);

                if (!fromIsOut && toIsOut) {
                    auto tempEndpoint = wire.from;
                    wire.from = wire.to;
                    wire.to = tempEndpoint;

                    if (!wire.manualPath.empty()) {
                        std::reverse(wire.manualPath.begin(), wire.manualPath.end());
                    }
                }
            }
        }
    }
    rebuildNetlist();
}

void SchematicCanvas::rebuildNetlist() {
    std::unordered_map<std::string, std::string> parent;
    
    auto findRoot = [&](const std::string& i) {
        std::string root = i;
        while (parent.count(root) && parent[root] != root) {
            root = parent[root];
        }
        std::string curr = i;
        while (parent.count(curr) && parent[curr] != root) {
            std::string nxt = parent[curr];
            parent[curr] = root;
            curr = nxt;
        }
        return root;
    };
    
    auto unionNodes = [&](const std::string& a, const std::string& b) {
        if (a.empty() || b.empty()) return;
        if (!parent.count(a)) parent[a] = a;
        if (!parent.count(b)) parent[b] = b;
        std::string rootA = findRoot(a);
        std::string rootB = findRoot(b);
        if (rootA != rootB) {
            parent[rootA] = rootB;
        }
    };

    // 1. Register all component pins
    for (const auto& comp : design.components) {
        auto terms = getTerminals(comp);
        for (const auto& t : terms) {
            std::string key = comp.id + "." + t.name;
            parent[key] = key;
        }
    }

    // 2. Map wires and T-junctions
    std::unordered_map<std::string, std::string> wireNetMap;
    for (const auto& wire : design.wires) {
        std::string fromKey = wire.from.compId + "." + wire.from.terminal;
        if (!wire.to.isWireJunction) {
            std::string toKey = wire.to.compId + "." + wire.to.terminal;
            unionNodes(fromKey, toKey);
            wireNetMap[wire.id] = fromKey;
        } else {
            auto it = wireNetMap.find(wire.to.targetWireId);
            if (it != wireNetMap.end()) {
                unionNodes(fromKey, it->second);
            }
            wireNetMap[wire.id] = fromKey;
        }
    }

    // 3. Assign merged net names to component nodes for simulation
    for (auto& comp : design.components) {
        auto terms = getTerminals(comp);
        comp.nodes.clear();
        for (const auto& t : terms) {
            std::string key = comp.id + "." + t.name;
            std::string rootNet = findRoot(key);
            if (comp.rawTypeStr == "GND" || rootNet.find("GND") != std::string::npos) {
                comp.nodes.push_back("0");
            } else {
                comp.nodes.push_back("net_" + rootNet);
            }
        }
    }
}

std::vector<ImVec2> SchematicCanvas::simplifyPath(const std::vector<ImVec2>& points) const {
    std::vector<ImVec2> result;
    if (points.size() <= 2) return points;
    result.push_back(points[0]);
    for (size_t i = 1; i < points.size() - 1; ++i) {
        ImVec2 prev = result.back();
        ImVec2 curr = points[i];
        ImVec2 next = points[i + 1];
        bool collinearX = (std::abs(prev.x - curr.x) < 1.0f && std::abs(curr.x - next.x) < 1.0f);
        bool collinearY = (std::abs(prev.y - curr.y) < 1.0f && std::abs(curr.y - next.y) < 1.0f);
        if (!collinearX && !collinearY) {
            result.push_back(curr);
        }
    }
    result.push_back(points.back());
    return result;
}

void SchematicCanvas::drawCurrentFlowAnimation(ImDrawList* drawList, ImVec2 p1, ImVec2 mid, ImVec2 mid2, ImVec2 p2, bool isControlNet, float timeSec) {
    ImVec2 pts[4] = {p1, mid, mid2, p2};
    float totalLen = 0.0f;
    float segLens[3];
    for (int i = 0; i < 3; ++i) {
        float dx = pts[i+1].x - pts[i].x;
        float dy = pts[i+1].y - pts[i].y;
        segLens[i] = std::sqrt(dx*dx + dy*dy);
        totalLen += segLens[i];
    }

    if (totalLen < 5.0f) return;

    float spacing = 28.0f * zoomLevel;
    float offset = std::fmod(timeSec * 45.0f * zoomLevel, spacing);
    ImU32 particleCol = isControlNet ? IM_COL32(56, 189, 248, 255) : IM_COL32(0, 255, 180, 240);

    for (float d = offset; d < totalLen; d += spacing) {
        float rem = d;
        ImVec2 particlePos = pts[0];
        for (int i = 0; i < 3; ++i) {
            if (rem <= segLens[i]) {
                if (segLens[i] > 0.0f) {
                    float t = rem / segLens[i];
                    particlePos = ImVec2(pts[i].x + t * (pts[i+1].x - pts[i].x), pts[i].y + t * (pts[i+1].y - pts[i].y));
                }
                break;
            }
            rem -= segLens[i];
        }
        drawList->AddCircleFilled(particlePos, 2.5f * zoomLevel, particleCol);
    }
}

bool SchematicCanvas::isPinConnected(const std::string& compId, const std::string& pinName) const {
    for (const auto& w : design.wires) {
        if ((w.from.compId == compId && w.from.terminal == pinName) ||
            (w.to.compId == compId && w.to.terminal == pinName)) {
            return true;
        }
    }
    return false;
}

void SchematicCanvas::pushUndoState() {
    undoStack.push_back(design);
    redoStack.clear();
    if (undoStack.size() > 50) undoStack.erase(undoStack.begin());
}

void SchematicCanvas::undo() {
    if (undoStack.size() > 1) {
        redoStack.push_back(undoStack.back());
        undoStack.pop_back();
        design = undoStack.back();
    }
}

void SchematicCanvas::redo() {
    if (!redoStack.empty()) {
        design = redoStack.back();
        undoStack.push_back(redoStack.back());
        redoStack.pop_back();
    }
}

void SchematicCanvas::fitToScreen(ImVec2 canvasSize) {
    if (canvasSize.x <= 0.0f || canvasSize.y <= 0.0f) {
        canvasSize = lastRenderedCanvasSize;
    }
    if (canvasSize.x <= 0.0f || canvasSize.y <= 0.0f) {
        canvasSize = ImVec2(800.0f, 600.0f);
    }
    if (design.components.empty()) {
        zoomLevel = 1.0f;
        panOffset = ImVec2(canvasSize.x * 0.5f - 400.0f, canvasSize.y * 0.5f - 400.0f);
        return;
    }
    float minX = 1e9f, maxX = -1e9f, minY = 1e9f, maxY = -1e9f;
    for (const auto& c : design.components) {
        minX = std::min(minX, c.x - 40.0f);
        maxX = std::max(maxX, c.x + 40.0f);
        minY = std::min(minY, c.y - 40.0f);
        maxY = std::max(maxY, c.y + 40.0f);
    }
    for (const auto& w : design.wires) {
        for (const auto& pt : w.manualPath) {
            minX = std::min(minX, pt.x);
            maxX = std::max(maxX, pt.x);
            minY = std::min(minY, pt.y);
            maxY = std::max(maxY, pt.y);
        }
        if (w.to.isWireJunction) {
            minX = std::min(minX, w.to.junctionX);
            maxX = std::max(maxX, w.to.junctionX);
            minY = std::min(minY, w.to.junctionY);
            maxY = std::max(maxY, w.to.junctionY);
        }
    }
    float width = maxX - minX + 80.0f;
    float height = maxY - minY + 80.0f;
    if (width < 100.0f) width = 100.0f;
    if (height < 100.0f) height = 100.0f;

    float centerX = (minX + maxX) * 0.5f;
    float centerY = (minY + maxY) * 0.5f;

    float zoomX = (canvasSize.x * 0.85f) / width;
    float zoomY = (canvasSize.y * 0.85f) / height;
    float newZoom = std::min(zoomX, zoomY);

    if (newZoom < 0.15f) newZoom = 0.15f;
    if (newZoom > 2.0f) newZoom = 2.0f;

    zoomLevel = newZoom;
    panOffset.x = (canvasSize.x * 0.5f) / zoomLevel - centerX;
    panOffset.y = (canvasSize.y * 0.5f) / zoomLevel - centerY;
}

void SchematicCanvas::addComponent(const ComponentInstance& comp) {
    pushUndoState();
    ComponentInstance newComp = comp;
    
    if (hasLastClickPos) {
        newComp.x = lastCanvasClickWorldPos.x;
        newComp.y = lastCanvasClickWorldPos.y;
    }
    
    if (newComp.type == ComponentType::SummingJunction || newComp.type == ComponentType::Product ||
        newComp.rawTypeStr == "SUM_RECT" || newComp.rawTypeStr == "SUM_ROUND" || newComp.rawTypeStr == "PRODUCT_RECT") {
        
        showConfigurator = true;
        pendingConfigComp = newComp;
        pendingConfigCompIdx = (int)design.components.size();
    }
    
    design.components.push_back(newComp);
}

ComponentInstance* SchematicCanvas::getSelectedComponent() {
    if (selectedComponentIds.empty()) return nullptr;
    std::string targetId = *selectedComponentIds.begin();
    for (auto& c : design.components) {
        if (c.id == targetId) return &c;
    }
    return nullptr;
}

void SchematicCanvas::drawGrid(ImDrawList* drawList, ImVec2 canvasSize, ImVec2 canvasPos) {
    float gridSize = 20.0f * zoomLevel;
    ImU32 gridColor = isDarkMode ? IM_COL32(35, 42, 56, 180) : IM_COL32(232, 227, 185, 220);
    ImU32 gridMajor = isDarkMode ? IM_COL32(50, 60, 80, 220) : IM_COL32(212, 205, 158, 255);
    
    float startX = std::fmod(panOffset.x * zoomLevel, gridSize);
    float startY = std::fmod(panOffset.y * zoomLevel, gridSize);
    if (startX < 0) startX += gridSize;
    if (startY < 0) startY += gridSize;

    int cellIdx = 0;
    for (float x = startX; x < canvasSize.x; x += gridSize) {
        ImU32 col = (cellIdx % 5 == 0) ? gridMajor : gridColor;
        drawList->AddLine(ImVec2(canvasPos.x + x, canvasPos.y), ImVec2(canvasPos.x + x, canvasPos.y + canvasSize.y), col, 1.0f);
        cellIdx++;
    }
    cellIdx = 0;
    for (float y = startY; y < canvasSize.y; y += gridSize) {
        ImU32 col = (cellIdx % 5 == 0) ? gridMajor : gridColor;
        drawList->AddLine(ImVec2(canvasPos.x, canvasPos.y + y), ImVec2(canvasPos.x + canvasSize.x, canvasPos.y + y), col, 1.0f);
        cellIdx++;
    }
}

void SchematicCanvas::drawBreadcrumbs(ImDrawList* drawList, ImVec2 canvasPos) {
    if (subsystemStack.empty()) return;

    std::string pathText = "Main Workspace";
    for (const auto& sub : subsystemStack) {
        pathText += " > " + sub.name;
    }

    drawList->AddRectFilled(canvasPos, ImVec2(canvasPos.x + 320, canvasPos.y + 32), IM_COL32(24, 28, 38, 230), 4.0f);
    drawList->AddText(ImVec2(canvasPos.x + 10, canvasPos.y + 8), IM_COL32(56, 189, 248, 255), pathText.c_str());

    ImGui::SetCursorScreenPos(ImVec2(canvasPos.x + 330, canvasPos.y + 2));
    if (ImGui::Button("< Back to Main Workspace")) {
        design = subsystemStack.front().design;
        subsystemStack.clear();
    }
}

void SchematicCanvas::drawComponentShape(ImDrawList* drawList, const ComponentInstance& comp, ImVec2 c, float s, ImU32 color) {
    const std::string& t = comp.rawTypeStr;
    float rot = (float)comp.rotation;
    ImU32 blockBg = isDarkMode ? IM_COL32(30, 41, 59, 230) : IM_COL32(252, 250, 225, 240);
    
    if (t == "R") {
        ImVec2 rawPts[] = {
            {0, -40*s}, {0, -20*s},
            {-10*s, -15*s}, {10*s, -9*s},
            {-10*s, -3*s},  {10*s, 3*s},
            {-10*s, 9*s},   {10*s, 15*s},
            {0, 20*s}, {0, 40*s}
        };
        ImVec2 pts[10];
        for (int i = 0; i < 10; ++i) pts[i] = rotatePt(rawPts[i].x, rawPts[i].y, c.x, c.y, rot);
        drawList->AddPolyline(pts, 10, color, 0, 2.0f * s);
    } else if (t == "L") {
        ImVec2 p1 = rotatePt(0, -40*s, c.x, c.y, rot);
        ImVec2 p2 = rotatePt(0, -20*s, c.x, c.y, rot);
        drawList->AddLine(p1, p2, color, 2.0f*s);
        for (int i = 0; i < 3; ++i) {
            float cy = -13.3f*s + i * 13.3f*s;
            ImVec2 c0 = rotatePt(0, cy - 6.7f*s, c.x, c.y, rot);
            ImVec2 c1 = rotatePt(-14*s, cy - 6.7f*s, c.x, c.y, rot);
            ImVec2 c2 = rotatePt(-14*s, cy + 6.7f*s, c.x, c.y, rot);
            ImVec2 c3 = rotatePt(0, cy + 6.7f*s, c.x, c.y, rot);
            drawList->AddBezierCubic(c0, c1, c2, c3, color, 2.0f*s, 12);
        }
        ImVec2 p3 = rotatePt(0, 20*s, c.x, c.y, rot);
        ImVec2 p4 = rotatePt(0, 40*s, c.x, c.y, rot);
        drawList->AddLine(p3, p4, color, 2.0f*s);
    } else if (t == "C") {
        drawList->AddLine(rotatePt(0, -40*s, c.x, c.y, rot), rotatePt(0, -5*s, c.x, c.y, rot), color, 2.0f*s);
        drawList->AddLine(rotatePt(-15*s, -5*s, c.x, c.y, rot), rotatePt(15*s, -5*s, c.x, c.y, rot), color, 2.0f*s);
        drawList->AddLine(rotatePt(-15*s, 5*s, c.x, c.y, rot), rotatePt(15*s, 5*s, c.x, c.y, rot), color, 2.0f*s);
        drawList->AddLine(rotatePt(0, 5*s, c.x, c.y, rot), rotatePt(0, 40*s, c.x, c.y, rot), color, 2.0f*s);
    } else if (t == "S") {
        drawList->AddLine(rotatePt(0, -40*s, c.x, c.y, rot), rotatePt(0, -20*s, c.x, c.y, rot), color, 2.0f*s);
        drawList->AddCircleFilled(rotatePt(0, -20*s, c.x, c.y, rot), 3.0f*s, color);
        drawList->AddCircleFilled(rotatePt(0, 20*s, c.x, c.y, rot), 3.0f*s, color);
        drawList->AddLine(rotatePt(0, -20*s, c.x, c.y, rot), rotatePt(13*s, 16*s, c.x, c.y, rot), color, 2.0f*s);
        drawList->AddLine(rotatePt(0, 20*s, c.x, c.y, rot), rotatePt(0, 40*s, c.x, c.y, rot), color, 2.0f*s);
        drawList->AddLine(rotatePt(-20*s, 0, c.x, c.y, rot), rotatePt(-6*s, 0, c.x, c.y, rot), color, 2.0f*s);
    } else if (t == "D") {
        drawList->AddLine(rotatePt(0, -40*s, c.x, c.y, rot), rotatePt(0, -10*s, c.x, c.y, rot), color, 2.0f*s);
        ImVec2 tri[] = {rotatePt(-15*s, -10*s, c.x, c.y, rot), rotatePt(15*s, -10*s, c.x, c.y, rot), rotatePt(0, 12*s, c.x, c.y, rot)};
        drawList->AddTriangleFilled(tri[0], tri[1], tri[2], IM_COL32(0, 230, 120, 30));
        drawList->AddTriangle(tri[0], tri[1], tri[2], color, 2.0f*s);
        drawList->AddLine(rotatePt(-15*s, 12*s, c.x, c.y, rot), rotatePt(15*s, 12*s, c.x, c.y, rot), color, 2.0f*s);
        drawList->AddLine(rotatePt(0, 12*s, c.x, c.y, rot), rotatePt(0, 40*s, c.x, c.y, rot), color, 2.0f*s);
    } else if (t == "MOSFET" || t == "vg-FET") {
        drawList->AddLine(rotatePt(0, -40*s, c.x, c.y, rot), rotatePt(0, -15*s, c.x, c.y, rot), color, 2.0f*s);
        drawList->AddLine(rotatePt(0, 15*s, c.x, c.y, rot), rotatePt(0, 40*s, c.x, c.y, rot), color, 2.0f*s);
        drawList->AddLine(rotatePt(-5*s, -15*s, c.x, c.y, rot), rotatePt(-5*s, 15*s, c.x, c.y, rot), color, 2.0f*s);
        drawList->AddLine(rotatePt(-5*s, 0, c.x, c.y, rot), rotatePt(0, 0, c.x, c.y, rot), color, 2.0f*s);
        drawList->AddLine(rotatePt(-10*s, -15*s, c.x, c.y, rot), rotatePt(-10*s, 15*s, c.x, c.y, rot), color, 2.0f*s);
        drawList->AddLine(rotatePt(-20*s, 0, c.x, c.y, rot), rotatePt(-10*s, 0, c.x, c.y, rot), color, 2.0f*s);
        
        drawList->AddLine(rotatePt(0, 15*s, c.x, c.y, rot), rotatePt(12*s, 15*s, c.x, c.y, rot), color, 1.5f*s);
        drawList->AddLine(rotatePt(12*s, 15*s, c.x, c.y, rot), rotatePt(12*s, 6*s, c.x, c.y, rot), color, 1.5f*s);
        
        ImVec2 dTri[] = {rotatePt(7*s, 6*s, c.x, c.y, rot), rotatePt(17*s, 6*s, c.x, c.y, rot), rotatePt(12*s, -6*s, c.x, c.y, rot)};
        drawList->AddTriangleFilled(dTri[0], dTri[1], dTri[2], IM_COL32(0, 230, 120, 30));
        drawList->AddTriangle(dTri[0], dTri[1], dTri[2], color, 1.5f*s);
        
        drawList->AddLine(rotatePt(7*s, -6*s, c.x, c.y, rot), rotatePt(17*s, -6*s, c.x, c.y, rot), color, 1.5f*s);
        
        drawList->AddLine(rotatePt(12*s, -6*s, c.x, c.y, rot), rotatePt(12*s, -15*s, c.x, c.y, rot), color, 1.5f*s);
        drawList->AddLine(rotatePt(12*s, -15*s, c.x, c.y, rot), rotatePt(0, -15*s, c.x, c.y, rot), color, 1.5f*s);
    } else if (t == "V") {
        drawList->AddLine(rotatePt(0, -40*s, c.x, c.y, rot), rotatePt(0, -16*s, c.x, c.y, rot), color, 2.0f*s);
        drawList->AddLine(rotatePt(0, 16*s, c.x, c.y, rot), rotatePt(0, 40*s, c.x, c.y, rot), color, 2.0f*s);
        drawList->AddCircle(c, 16*s, color, 0, 2.0f*s);
        drawList->AddLine(rotatePt(-3*s, -7*s, c.x, c.y, rot), rotatePt(3*s, -7*s, c.x, c.y, rot), color, 1.5f*s);
        drawList->AddLine(rotatePt(0, -10*s, c.x, c.y, rot), rotatePt(0, -4*s, c.x, c.y, rot), color, 1.5f*s);
        drawList->AddLine(rotatePt(-3*s, 7*s, c.x, c.y, rot), rotatePt(3*s, 7*s, c.x, c.y, rot), color, 1.5f*s);
    } else if (t == "I") {
        drawList->AddLine(rotatePt(0, -40*s, c.x, c.y, rot), rotatePt(0, -16*s, c.x, c.y, rot), color, 2.0f*s);
        drawList->AddLine(rotatePt(0, 16*s, c.x, c.y, rot), rotatePt(0, 40*s, c.x, c.y, rot), color, 2.0f*s);
        drawList->AddCircle(c, 16*s, color, 0, 2.0f*s);
        drawList->AddLine(rotatePt(0, -9*s, c.x, c.y, rot), rotatePt(0, 9*s, c.x, c.y, rot), color, 2.0f*s);
        ImVec2 arr[] = {rotatePt(-4*s, 3*s, c.x, c.y, rot), rotatePt(0, 9*s, c.x, c.y, rot), rotatePt(4*s, 3*s, c.x, c.y, rot)};
        drawList->AddPolyline(arr, 3, color, 0, 2.0f*s);
    } else if (t == "AC_V") {
        drawList->AddLine(rotatePt(0, -40*s, c.x, c.y, rot), rotatePt(0, -16*s, c.x, c.y, rot), color, 2.0f*s);
        drawList->AddLine(rotatePt(0, 16*s, c.x, c.y, rot), rotatePt(0, 40*s, c.x, c.y, rot), color, 2.0f*s);
        drawList->AddCircle(c, 16*s, color, 0, 2.0f*s);
        ImVec2 c0 = rotatePt(-8*s, 0, c.x, c.y, rot);
        ImVec2 c1 = rotatePt(-4*s, -8*s, c.x, c.y, rot);
        ImVec2 c2 = rotatePt(0, -8*s, c.x, c.y, rot);
        ImVec2 c3 = rotatePt(0, 0, c.x, c.y, rot);
        drawList->AddBezierCubic(c0, c1, c2, c3, color, 2.0f*s, 10);
        ImVec2 c4 = rotatePt(0, 0, c.x, c.y, rot);
        ImVec2 c5 = rotatePt(0, 8*s, c.x, c.y, rot);
        ImVec2 c6 = rotatePt(4*s, 8*s, c.x, c.y, rot);
        ImVec2 c7 = rotatePt(8*s, 0, c.x, c.y, rot);
        drawList->AddBezierCubic(c4, c5, c6, c7, color, 2.0f*s, 10);
    } else if (t == "VM" || t == "AM") {
        drawList->AddLine(rotatePt(0, -40*s, c.x, c.y, rot), rotatePt(0, -16*s, c.x, c.y, rot), color, 2.0f*s);
        drawList->AddLine(rotatePt(0, 16*s, c.x, c.y, rot), rotatePt(0, 40*s, c.x, c.y, rot), color, 2.0f*s);
        drawList->AddLine(rotatePt(16*s, 0, c.x, c.y, rot), rotatePt(20*s, 0, c.x, c.y, rot), color, 2.0f*s);
        drawList->AddCircle(c, 16*s, color, 0, 2.0f*s);
        drawList->AddText(rotatePt(-4*s, -7*s, c.x, c.y, rot), color, (t == "VM") ? "V" : "A");
    } else if (t == "GND") {
        drawList->AddLine(rotatePt(0, -20*s, c.x, c.y, rot), rotatePt(0, 0, c.x, c.y, rot), color, 2.0f*s);
        drawList->AddLine(rotatePt(-12*s, 0, c.x, c.y, rot), rotatePt(12*s, 0, c.x, c.y, rot), color, 2.0f*s);
        drawList->AddLine(rotatePt(-8*s, 6*s, c.x, c.y, rot), rotatePt(8*s, 6*s, c.x, c.y, rot), color, 2.0f*s);
        drawList->AddLine(rotatePt(-4*s, 12*s, c.x, c.y, rot), rotatePt(4*s, 12*s, c.x, c.y, rot), color, 2.0f*s);
    } else if (t == "GAIN") {
        ImVec2 t1 = rotatePt(-16*s, -18*s, c.x, c.y, rot);
        ImVec2 t2 = rotatePt(16*s, 0, c.x, c.y, rot);
        ImVec2 t3 = rotatePt(-16*s, 18*s, c.x, c.y, rot);
        drawList->AddTriangleFilled(t1, t2, t3, IM_COL32(38, 50, 70, 200));
        drawList->AddTriangle(t1, t2, t3, color, 2.0f*s);
        drawList->AddLine(rotatePt(-20*s, 0, c.x, c.y, rot), rotatePt(-16*s, 0, c.x, c.y, rot), color, 2.0f*s);
        drawList->AddLine(rotatePt(16*s, 0, c.x, c.y, rot), rotatePt(20*s, 0, c.x, c.y, rot), color, 2.0f*s);
        drawList->AddText(rotatePt(-4*s, -7*s, c.x, c.y, rot), color, "K");
    } else if (t == "SUM_ROUND" || t == "SUM") {
        drawList->AddCircleFilled(c, 14*s, IM_COL32(38, 50, 70, 200));
        drawList->AddCircle(c, 14*s, color, 0, 2.0f*s);
        drawList->AddLine(rotatePt(-20*s, -20*s, c.x, c.y, rot), rotatePt(-10*s, -10*s, c.x, c.y, rot), color, 2.0f*s);
        drawList->AddLine(rotatePt(-20*s, 20*s, c.x, c.y, rot), rotatePt(-10*s, 10*s, c.x, c.y, rot), color, 2.0f*s);
        drawList->AddLine(rotatePt(14*s, 0, c.x, c.y, rot), rotatePt(20*s, 0, c.x, c.y, rot), color, 2.0f*s);
        drawList->AddText(rotatePt(-6*s, -10*s, c.x, c.y, rot), color, "+");
        drawList->AddText(rotatePt(-6*s, 2*s, c.x, c.y, rot), color, "-");
    } else if (t == "SUM_RECT") {
        float hw = 25*s, hh = (comp.numInputPins * 15 + 10)*s;
        drawList->AddRectFilled({c.x - hw, c.y - hh}, {c.x + hw, c.y + hh}, IM_COL32(38, 50, 70, 200), 4*s);
        drawList->AddRect({c.x - hw, c.y - hh}, {c.x + hw, c.y + hh}, color, 4*s, 0, 2.0f*s);
        for (int i = 0; i < comp.numInputPins; ++i) {
            float py = -hh + (i + 1)*(2*hh / (comp.numInputPins + 1));
            std::string signStr = (i < (int)comp.pinSigns.size()) ? comp.pinSigns[i] : "+";
            drawList->AddText(rotatePt(-hw + 6*s, py - 6*s, c.x, c.y, rot), color, signStr.c_str());
        }
    } else if (t == "PRODUCT_RECT" || t == "PROD") {
        drawList->AddCircleFilled(c, 14*s, IM_COL32(38, 50, 70, 200));
        drawList->AddCircle(c, 14*s, color, 0, 2.0f*s);
        drawList->AddLine(rotatePt(-20*s, -20*s, c.x, c.y, rot), rotatePt(-10*s, -10*s, c.x, c.y, rot), color, 2.0f*s);
        drawList->AddLine(rotatePt(-20*s, 20*s, c.x, c.y, rot), rotatePt(-10*s, 10*s, c.x, c.y, rot), color, 2.0f*s);
        drawList->AddLine(rotatePt(14*s, 0, c.x, c.y, rot), rotatePt(20*s, 0, c.x, c.y, rot), color, 2.0f*s);
        drawList->AddText(rotatePt(-4*s, -7*s, c.x, c.y, rot), color, "X");
    } else if (t == "COMP") {
        ImVec2 t1 = rotatePt(-16*s, -20*s, c.x, c.y, rot);
        ImVec2 t2 = rotatePt(16*s, 0, c.x, c.y, rot);
        ImVec2 t3 = rotatePt(-16*s, 20*s, c.x, c.y, rot);
        drawList->AddTriangleFilled(t1, t2, t3, IM_COL32(38, 50, 70, 200));
        drawList->AddTriangle(t1, t2, t3, color, 2.0f*s);
        drawList->AddText(rotatePt(-10*s, -14*s, c.x, c.y, rot), color, "+");
        drawList->AddText(rotatePt(-10*s, 4*s, c.x, c.y, rot), color, "-");
    } else if (t == "PID") {
        float hw = 20*s, hh = 16*s;
        drawList->AddRectFilled({c.x - hw, c.y - hh}, {c.x + hw, c.y + hh}, IM_COL32(38, 50, 70, 200), 4*s);
        drawList->AddRect({c.x - hw, c.y - hh}, {c.x + hw, c.y + hh}, color, 4*s, 0, 2.0f*s);
        drawList->AddText(rotatePt(-12*s, -6*s, c.x, c.y, rot), color, "PID");
    } else if (t == "PWM") {
        float hw = 20*s, hh = 16*s;
        drawList->AddRectFilled({c.x - hw, c.y - hh}, {c.x + hw, c.y + hh}, IM_COL32(38, 50, 70, 200), 4*s);
        drawList->AddRect({c.x - hw, c.y - hh}, {c.x + hw, c.y + hh}, color, 4*s, 0, 2.0f*s);
        ImVec2 wave[] = {rotatePt(-10*s, 6*s, c.x, c.y, rot), rotatePt(-10*s, -6*s, c.x, c.y, rot), rotatePt(0, -6*s, c.x, c.y, rot), rotatePt(0, 6*s, c.x, c.y, rot), rotatePt(10*s, 6*s, c.x, c.y, rot)};
        drawList->AddPolyline(wave, 5, color, 0, 1.8f*s);
    } else if (t == "TRI") {
        float hw = 20*s, hh = 16*s;
        drawList->AddRectFilled({c.x - hw, c.y - hh}, {c.x + hw, c.y + hh}, blockBg, 4*s);
        drawList->AddRect({c.x - hw, c.y - hh}, {c.x + hw, c.y + hh}, color, 4*s, 0, 2.0f*s);
        ImVec2 triWave[] = {rotatePt(-10*s, 6*s, c.x, c.y, rot), rotatePt(0, -6*s, c.x, c.y, rot), rotatePt(10*s, 6*s, c.x, c.y, rot)};
        drawList->AddPolyline(triWave, 3, color, 0, 1.8f*s);
    } else if (t == "V_3PH" || t == "ThreePhaseSource") {
        float hw = 24*s, hh = 28*s;
        drawList->AddRectFilled({c.x - hw, c.y - hh}, {c.x + hw, c.y + hh}, blockBg, 4*s);
        drawList->AddRect({c.x - hw, c.y - hh}, {c.x + hw, c.y + hh}, color, 4*s, 0, 2.0f*s);
        drawList->AddText(rotatePt(-16*s, -12*s, c.x, c.y, rot), color, "3-Phase");
        drawList->AddText(rotatePt(-12*s, 2*s, c.x, c.y, rot), IM_COL32(56, 189, 248, 255), "AC");
    } else if (t == "PWM_MASTER" || t == "MasterPWM") {
        float hw = 30*s, hh = 28*s;
        drawList->AddRectFilled({c.x - hw, c.y - hh}, {c.x + hw, c.y + hh}, blockBg, 4*s);
        drawList->AddRect({c.x - hw, c.y - hh}, {c.x + hw, c.y + hh}, color, 4*s, 0, 2.0f*s);
        drawList->AddText(rotatePt(-22*s, -10*s, c.x, c.y, rot), color, "PWM");
        drawList->AddText(rotatePt(-24*s, 2*s, c.x, c.y, rot), IM_COL32(245, 158, 11, 255), "MASTER");
    } else if (t == "EDGE_DETECT" || t == "EdgeDetector") {
        float hw = 22*s, hh = 16*s;
        drawList->AddRectFilled({c.x - hw, c.y - hh}, {c.x + hw, c.y + hh}, blockBg, 4*s);
        drawList->AddRect({c.x - hw, c.y - hh}, {c.x + hw, c.y + hh}, color, 4*s, 0, 2.0f*s);
        drawList->AddText(rotatePt(-14*s, -6*s, c.x, c.y, rot), color, "EDGE");
    } else if (t == "MATH_FCN" || t == "FCN" || t == "MathFunction") {
        float hw = 22*s, hh = 16*s;
        drawList->AddRectFilled({c.x - hw, c.y - hh}, {c.x + hw, c.y + hh}, blockBg, 4*s);
        drawList->AddRect({c.x - hw, c.y - hh}, {c.x + hw, c.y + hh}, color, 4*s, 0, 2.0f*s);
        std::string fnLabel = comp.parameters.count("function") ? comp.parameters.at("function") : "FCN";
        if (fnLabel.length() > 5) fnLabel = fnLabel.substr(0, 5);
        drawList->AddText(rotatePt(-14*s, -6*s, c.x, c.y, rot), color, fnLabel.c_str());
    } else if (t == "KEY_TRIGGER" || t == "KeyTrigger") {
        float hw = 22*s, hh = 16*s;
        drawList->AddRectFilled({c.x - hw, c.y - hh}, {c.x + hw, c.y + hh}, blockBg, 4*s);
        drawList->AddRect({c.x - hw, c.y - hh}, {c.x + hw, c.y + hh}, color, 4*s, 0, 2.0f*s);
        std::string keyLabel = comp.parameters.count("key") ? comp.parameters.at("key") : "KEY";
        drawList->AddText(rotatePt(-14*s, -6*s, c.x, c.y, rot), IM_COL32(236, 72, 153, 255), keyLabel.c_str());
    } else if (t == "PULSE" || t == "PULSE_GEN") {
        float hw = 22*s, hh = 16*s;
        drawList->AddRectFilled({c.x - hw, c.y - hh}, {c.x + hw, c.y + hh}, blockBg, 4*s);
        drawList->AddRect({c.x - hw, c.y - hh}, {c.x + hw, c.y + hh}, color, 4*s, 0, 2.0f*s);
        ImVec2 pulseW[] = {
            rotatePt(-12*s, 6*s, c.x, c.y, rot),
            rotatePt(-12*s, -6*s, c.x, c.y, rot),
            rotatePt(0, -6*s, c.x, c.y, rot),
            rotatePt(0, 6*s, c.x, c.y, rot),
            rotatePt(12*s, 6*s, c.x, c.y, rot)
        };
        drawList->AddPolyline(pulseW, 5, color, 0, 1.8f*s);
        drawList->AddLine(rotatePt(hw, 0, c.x, c.y, rot), rotatePt(hw + 4*s, 0, c.x, c.y, rot), color, 2.0f*s);
    } else if (t == "SCOPE") {
        int numCh = 2;
        if (comp.parameters.count("channels")) {
            try { numCh = std::stoi(comp.parameters.at("channels")); } catch (...) {}
        }
        if (numCh < 1) numCh = 1;
        float hw = 16.0f * s;
        float hh = std::max(16.0f, numCh * 10.0f) * s;

        drawList->AddRectFilled({c.x - hw, c.y - hh}, {c.x + hw, c.y + hh}, blockBg, 4.0f * s);
        drawList->AddRect({c.x - hw, c.y - hh}, {c.x + hw, c.y + hh}, color, 4.0f * s, 0, 2.0f * s);

        ImVec2 scW[] = {
            rotatePt(-8*s, -6*s, c.x, c.y, rot),
            rotatePt(-4*s, -6*s, c.x, c.y, rot),
            rotatePt(-4*s, 6*s, c.x, c.y, rot),
            rotatePt(4*s, 6*s, c.x, c.y, rot),
            rotatePt(4*s, -6*s, c.x, c.y, rot),
            rotatePt(8*s, -6*s, c.x, c.y, rot)
        };
        drawList->AddPolyline(scW, 6, IM_COL32(56, 189, 248, 255), 0, 1.5f * s);

        for (int i = 0; i < numCh; ++i) {
            float yOff = (numCh > 1) ? (-10.0f * (numCh - 1) + 20.0f * i) : 0.0f;
            ImVec2 arr[] = {
                rotatePt(-hw, (yOff - 3.0f)*s, c.x, c.y, rot),
                rotatePt(-hw + 4.0f*s, yOff*s, c.x, c.y, rot),
                rotatePt(-hw, (yOff + 3.0f)*s, c.x, c.y, rot)
            };
            drawList->AddTriangleFilled(arr[0], arr[1], arr[2], color);
        }
    } else if (t == "PROBE") {
        std::string sigStr = comp.parameters.count("selected_signals") ? comp.parameters.at("selected_signals") : "";
        std::vector<std::string> sigs;
        std::stringstream ss(sigStr);
        std::string item;
        while (std::getline(ss, item, ',')) { if (!item.empty()) sigs.push_back(item); }

        int numPins = std::max(1, (int)sigs.size());
        float hw = 30.0f * s;
        float hh = std::max(20.0f, numPins * 15.0f) * s;

        drawList->AddRectFilled({c.x - hw, c.y - hh}, {c.x + hw, c.y + hh}, blockBg, 4.0f * s);
        drawList->AddRect({c.x - hw, c.y - hh}, {c.x + hw, c.y + hh}, IM_COL32(14, 165, 233, 230), 4.0f * s, 0, 2.0f * s);
        drawList->AddText(rotatePt(-18*s, -6*s, c.x, c.y, rot), IM_COL32(14, 165, 233, 255), "PROBE");
    } else if (t == "MUX") {
        ImVec2 mPts[] = {rotatePt(-20*s, -25*s, c.x, c.y, rot), rotatePt(20*s, -15*s, c.x, c.y, rot), rotatePt(20*s, 15*s, c.x, c.y, rot), rotatePt(-20*s, 25*s, c.x, c.y, rot)};
        drawList->AddConvexPolyFilled(mPts, 4, blockBg);
        drawList->AddPolyline(mPts, 4, color, ImDrawFlags_Closed, 2.0f*s);
        drawList->AddText(rotatePt(-12*s, -6*s, c.x, c.y, rot), color, "MUX");
    } else if (t == "AND") {
        float hw = 20*s, hh = 18*s;
        drawList->AddRectFilled({c.x - hw, c.y - hh}, {c.x + hw, c.y + hh}, blockBg, 4*s);
        drawList->AddRect({c.x - hw, c.y - hh}, {c.x + hw, c.y + hh}, color, 4*s, 0, 2.0f*s);
        drawList->AddText(rotatePt(-4*s, -7*s, c.x, c.y, rot), color, "&");
    } else if (t == "OR") {
        float hw = 20*s, hh = 18*s;
        drawList->AddRectFilled({c.x - hw, c.y - hh}, {c.x + hw, c.y + hh}, blockBg, 4*s);
        drawList->AddRect({c.x - hw, c.y - hh}, {c.x + hw, c.y + hh}, color, 4*s, 0, 2.0f*s);
        drawList->AddText(rotatePt(-10*s, -7*s, c.x, c.y, rot), color, ">=1");
    } else if (t == "NOT") {
        ImVec2 t1 = rotatePt(-14*s, -15*s, c.x, c.y, rot);
        ImVec2 t2 = rotatePt(6*s, 0, c.x, c.y, rot);
        ImVec2 t3 = rotatePt(-14*s, 15*s, c.x, c.y, rot);
        drawList->AddTriangleFilled(t1, t2, t3, blockBg);
        drawList->AddTriangle(t1, t2, t3, color, 2.0f*s);
        drawList->AddCircle(rotatePt(10*s, 0, c.x, c.y, rot), 3.0f*s, color, 0, 2.0f*s);
    } else if (t == "SUBSYSTEM") {
        float hw = 50*s, hh = 40*s;
        drawList->AddRectFilled({c.x - hw, c.y - hh}, {c.x + hw, c.y + hh}, blockBg, 6*s);
        drawList->AddRect({c.x - hw, c.y - hh}, {c.x + hw, c.y + hh}, color, 6*s, 0, 2.0f*s);
        drawList->AddText({c.x - 28*s, c.y - 6*s}, color, "Subsystem");
    } else if (t == "CSCRIPT") {
        float hw = 80*s, hh = 40*s;
        drawList->AddRectFilled({c.x - hw, c.y - hh}, {c.x + hw, c.y + hh}, blockBg, 4*s);
        drawList->AddRect({c.x - hw, c.y - hh}, {c.x + hw, c.y + hh}, color, 4*s, 0, 2.0f*s);
        drawList->AddText({c.x - 30*s, c.y - 8*s}, color, "[ C++ Script ]");
    } else {
        drawList->AddRect({c.x - 20*s, c.y - 20*s}, {c.x + 20*s, c.y + 20*s}, color, 4*s, 0, 2.0f*s);
        drawList->AddText({c.x - 12*s, c.y - 5*s}, color, t.c_str());
    }
}

void SchematicCanvas::drawTerminals(ImDrawList* drawList, const ComponentInstance& comp, ImVec2 center, float s, ImVec2 mousePos, float& minPinDist) {
    auto terminals = getTerminals(comp);
    if (terminals.empty() && !comp.pins.empty()) {
        for (const auto& pin : comp.pins) {
            TerminalDef td;
            td.name = pin.name.c_str();
            td.x = pin.relativeX;
            td.y = pin.relativeY;
            td.dx = pin.isOutput ? 1.0f : -1.0f;
            td.dy = 0;
            td.isControl = pin.isCtrl || pin.isInput || pin.isOutput;
            terminals.push_back(td);
        }
    }

    const ComponentInstance* startComp = nullptr;
    if (isWiring) {
        for (const auto& c : design.components) {
            if (c.id == wireStartCompId) { startComp = &c; break; }
        }
    }
    
    for (const auto& term : terminals) {
        ImVec2 tPos = rotatePt(term.x * s, term.y * s, center.x, center.y, (float)comp.rotation);
        float dist = std::sqrt((mousePos.x - tPos.x)*(mousePos.x - tPos.x) + (mousePos.y - tPos.y)*(mousePos.y - tPos.y));
        bool isHovered = (dist < 14.0f * s);
        bool isConnected = isPinConnected(comp.id, term.name);

        bool isControl = getPinDomain(comp, term.name) == DomainType::Control;
        
        if (isHovered && dist < minPinDist) {
            minPinDist = dist;
            hoveredPinCompId = comp.id;
            hoveredPinName = term.name;

            if (isWiring && startComp) {
                DomainType startDom = getPinDomain(*startComp, wireStartPin);
                DomainType targetDom = getPinDomain(comp, term.name);
                if (startDom != targetDom) {
                    ImGui::SetTooltip("🚫 CANNOT CONNECT: %s Pin cannot connect to %s Pin!",
                        startDom == DomainType::Control ? "Control Signal" : "Electrical Power",
                        targetDom == DomainType::Control ? "Control Signal" : "Electrical Power");
                } else {
                    ImGui::SetTooltip("%s.%s (%s)", comp.id.c_str(), term.name, isControl ? "Control Domain" : "Power Domain");
                }
            } else {
                ImGui::SetTooltip("%s.%s (%s)", comp.id.c_str(), term.name, isControl ? "Control Domain" : "Power Domain");
            }
        }

        bool isClosestHovered = (!hoveredPinCompId.empty() && hoveredPinCompId == comp.id && hoveredPinName == term.name);
        float radius = isClosestHovered ? 6.0f * s : (isConnected ? 2.0f * s : (isControl ? 4.0f * s : 4.5f * s));
        ImU32 termColor = isClosestHovered ? IM_COL32(255, 200, 50, 255) : (isControl ? IM_COL32(56, 189, 248, 230) : IM_COL32(0, 230, 120, 230));

        if (isClosestHovered && isWiring && startComp) {
            DomainType startDom = getPinDomain(*startComp, wireStartPin);
            DomainType targetDom = getPinDomain(comp, term.name);
            if (startDom != targetDom) {
                termColor = IM_COL32(255, 50, 50, 255);
            }
        }

        drawList->AddCircleFilled(tPos, radius, termColor);
    }
}

bool SchematicCanvas::getTerminalPortStub(const ComponentInstance& comp, const std::string& terminalName, ImVec2 canvasPos, float zoomLevel, ImVec2& outPinPos, ImVec2& outStubPos, bool& outIsVertical) const {
    auto terminals = getTerminals(comp);
    if (terminals.empty() && !comp.pins.empty()) {
        for (const auto& pin : comp.pins) {
            TerminalDef td;
            td.name = pin.name.c_str();
            td.x = pin.relativeX;
            td.y = pin.relativeY;
            td.dx = pin.isOutput ? 1.0f : -1.0f;
            td.dy = 0;
            td.isControl = pin.isCtrl || pin.isInput || pin.isOutput;
            terminals.push_back(td);
        }
    }

    ImVec2 compCenter = worldToScreen(comp.x, comp.y, canvasPos);
    for (const auto& t : terminals) {
        if (t.name == terminalName) {
            outPinPos = rotatePt(t.x * zoomLevel, t.y * zoomLevel, compCenter.x, compCenter.y, (float)comp.rotation);
            ImVec2 dir = rotatePt(t.dx * 20.0f * zoomLevel, t.dy * 20.0f * zoomLevel, 0, 0, (float)comp.rotation);
            outStubPos = ImVec2(outPinPos.x + dir.x, outPinPos.y + dir.y);
            ImVec2 rotatedDir = rotatePt(t.dx, t.dy, 0, 0, (float)comp.rotation);
            outIsVertical = (std::abs(rotatedDir.y) > std::abs(rotatedDir.x));
            return true;
        }
    }
    outIsVertical = false;
    return false;
}

void SchematicCanvas::drawWires(ImDrawList* drawList, ImVec2 canvasPos) {
    ImGuiIO& io = ImGui::GetIO();
    ImVec2 mousePos = io.MousePos;

    hoveredWireId.clear();
    float minWireDist = 1e9f;

    std::map<std::string, std::tuple<ImVec2, ImVec2, ImVec2, ImVec2>> wirePointsMap;

    for (auto& wire : design.wires) {
        ImVec2 p1(0, 0), p1_stub(0, 0), p2(0, 0), p2_stub(0, 0);
        bool foundFrom = false, foundTo = false;

        DomainType wDom = getWireDomain(wire, design);
        bool isControlNet = (wDom == DomainType::Control);

        bool fromIsVertical = false;
        for (const auto& comp : design.components) {
            if (comp.id == wire.from.compId) {
                foundFrom = getTerminalPortStub(comp, wire.from.terminal, canvasPos, zoomLevel, p1, p1_stub, fromIsVertical);
            }
            if (comp.id == wire.to.compId) {
                bool dummyVert;
                foundTo = getTerminalPortStub(comp, wire.to.terminal, canvasPos, zoomLevel, p2, p2_stub, dummyVert);
            }
        }

        if (!foundFrom) p1_stub = p1;
        if (!foundTo) p2_stub = p2;

        if (wire.to.isWireJunction) {
            std::string targetLower = wire.to.targetWireId;
            std::transform(targetLower.begin(), targetLower.end(), targetLower.begin(), ::tolower);

            bool foundTarget = false;
            for (const auto& [k, v] : wirePointsMap) {
                std::string kLower = k;
                std::transform(kLower.begin(), kLower.end(), kLower.begin(), ::tolower);
                if (kLower == targetLower) {
                    ImVec2 tp1 = std::get<0>(v);
                    ImVec2 tmid = std::get<1>(v);
                    ImVec2 tmid2 = std::get<2>(v);
                    ImVec2 tp2 = std::get<3>(v);

                    ImVec2 jScreen = worldToScreen(wire.to.junctionX, wire.to.junctionY, canvasPos);
                    float d1=0, d2=0, d3=0;
                    ImVec2 q1 = getClosestPointOnSegment(jScreen, tp1, tmid, d1);
                    ImVec2 q2 = getClosestPointOnSegment(jScreen, tmid, tmid2, d2);
                    ImVec2 q3 = getClosestPointOnSegment(jScreen, tmid2, tp2, d3);

                    float minD = std::min({d1, d2, d3});
                    p2 = (minD == d1) ? q1 : ((minD == d2) ? q2 : q3);
                    p2_stub = p2;
                    foundTo = true;
                    foundTarget = true;
                    break;
                }
            }
            if (!foundTarget) {
                p2 = worldToScreen(wire.to.junctionX, wire.to.junctionY, canvasPos);
                p2_stub = p2;
                foundTo = true;
            }
        }

        if (foundFrom && foundTo) {
            if (fromIsVertical) {
                float distP1P2 = std::abs(p2.y - p1.y);
                float distP1Stub = std::abs(p1_stub.y - p1.y);
                if (distP1P2 <= distP1Stub) {
                    p1_stub = p1;
                }
            } else {
                float distP1P2 = std::abs(p2.x - p1.x);
                float distP1Stub = std::abs(p1_stub.x - p1.x);
                if (distP1P2 <= distP1Stub) {
                    p1_stub = p1;
                }
            }

            ImVec2 c1(0, 0), c2(0, 0);
            bool hasCustomOffset = !wire.manualPath.empty();

            if (hasCustomOffset) {
                if (isDraggingSegmentHorizontal) {
                    float snapY = worldToScreen(0, wire.manualPath[0].y, canvasPos).y;
                    c1 = ImVec2(p1_stub.x, snapY);
                    c2 = ImVec2(p2.x, snapY);
                } else {
                    float snapX = worldToScreen(wire.manualPath[0].x, 0, canvasPos).x;
                    c1 = ImVec2(snapX, p1_stub.y);
                    c2 = ImVec2(snapX, p2.y);
                }
            } else {
                ImVec2 mid = fromIsVertical ? ImVec2(p1_stub.x, p2.y) : ImVec2(p2.x, p1_stub.y);
                c1 = mid;
                c2 = mid;
            }

            wirePointsMap[wire.id] = {p1_stub, c1, c2, p2};

            float d0 = 0, d1 = 0, d2 = 0, d3 = 0;
            ImVec2 q0 = getClosestPointOnSegment(mousePos, p1, p1_stub, d0);
            ImVec2 q1 = getClosestPointOnSegment(mousePos, p1_stub, c1, d1);
            ImVec2 q2 = getClosestPointOnSegment(mousePos, c1, c2, d2);
            ImVec2 q3 = getClosestPointOnSegment(mousePos, c2, p2, d3);

            float bestD = std::min({d0, d1, d2, d3});
            ImVec2 bestQ = (bestD == d0) ? q0 : ((bestD == d1) ? q1 : ((bestD == d2) ? q2 : q3));

            bool isWireHovered = (bestD < 12.0f * zoomLevel);

            if (isWireHovered && bestD < minWireDist) {
                minWireDist = bestD;
                hoveredWireId = wire.id;
                ImVec2 snapQ = bestQ;
                if (isWiring) {
                    ImVec2 startP(0, 0), startStub(0, 0);
                    bool startIsVert = false;
                    for (const auto& comp : design.components) {
                        if (comp.id == wireStartCompId) {
                            getTerminalPortStub(comp, wireStartPin, canvasPos, zoomLevel, startP, startStub, startIsVert);
                            break;
                        }
                    }
                    if (startIsVert) {
                        snapQ.x = startStub.x;
                    } else {
                        snapQ.y = startStub.y;
                    }
                }
                hoveredWireJunctionPos = screenToWorld(snapQ, canvasPos);
            }

            bool isSelected = selectedWireIds.count(wire.id) > 0;
            ImU32 powerWireColor = isDarkMode ? IM_COL32(0, 230, 120, 255) : IM_COL32(4, 120, 87, 255);
            ImU32 ctrlWireColor = isDarkMode ? IM_COL32(56, 189, 248, 255) : IM_COL32(2, 132, 199, 255);
            ImU32 selColor = isDarkMode ? IM_COL32(255, 180, 0, 255) : IM_COL32(217, 119, 6, 255);
            ImU32 hovColor = isDarkMode ? IM_COL32(255, 220, 100, 255) : IM_COL32(245, 158, 11, 255);

            ImU32 wireColor = isSelected ? selColor : (isWireHovered ? hovColor : (isControlNet ? ctrlWireColor : powerWireColor));
            float thickness = (isSelected || isWireHovered) ? 3.5f * zoomLevel : 2.5f * zoomLevel;

            if (isSelected) {
                drawList->AddLine(p1, p1_stub, IM_COL32(245, 158, 11, 60), thickness + 4.0f*zoomLevel);
                drawList->AddLine(p1_stub, c1, IM_COL32(245, 158, 11, 60), thickness + 4.0f*zoomLevel);
                drawList->AddLine(c1, c2, IM_COL32(245, 158, 11, 60), thickness + 4.0f*zoomLevel);
                drawList->AddLine(c2, p2, IM_COL32(245, 158, 11, 60), thickness + 4.0f*zoomLevel);
            }
            
            // Render 100% Strictly Orthogonal Wire Path matching Image 2
            drawList->AddLine(p1, p1_stub, wireColor, thickness);
            drawList->AddLine(p1_stub, c1, wireColor, thickness);
            drawList->AddLine(c1, c2, wireColor, thickness);
            drawList->AddLine(c2, p2, wireColor, thickness);

            // Real-time Current & Signal Flow Particle Animation Overlay
            drawCurrentFlowAnimation(drawList, p1_stub, c1, c2, p2, isControlNet, (float)ImGui::GetTime());

            if (wire.to.isWireJunction) {
                drawList->AddCircleFilled(p2, 4.0f * zoomLevel, isControlNet ? IM_COL32(56, 189, 248, 255) : IM_COL32(0, 230, 120, 255));
            }

            // Interactive Segment Drag Handles (Horizontal / Vertical 4-Way Dragging)
            if (isSelected && selectedComponentIds.empty()) {
                ImVec2 h1((p1_stub.x + c1.x)*0.5f, (p1_stub.y + c1.y)*0.5f);
                ImVec2 h2((c1.x + c2.x)*0.5f, (c1.y + c2.y)*0.5f);
                ImVec2 h3((c2.x + p2_stub.x)*0.5f, (c2.y + p2_stub.y)*0.5f);
                float hs = 6.0f * zoomLevel;

                drawList->AddRectFilled({h1.x - hs, h1.y - hs}, {h1.x + hs, h1.y + hs}, IM_COL32(255, 180, 0, 255));
                drawList->AddRectFilled({h2.x - hs, h2.y - hs}, {h2.x + hs, h2.y + hs}, IM_COL32(255, 180, 0, 255));
                drawList->AddRectFilled({h3.x - hs, h3.y - hs}, {h3.x + hs, h3.y + hs}, IM_COL32(255, 180, 0, 255));

                if (ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
                    if (isWireHovered) {
                        pushUndoState();
                        isDraggingWireSegment = true;
                        draggingWireId = wire.id;
                        
                        ImVec2 segP1 = (bestD == d1 || bestD == d0) ? p1_stub : ((bestD == d2) ? c1 : c2);
                        ImVec2 segP2 = (bestD == d1 || bestD == d0) ? c1 : ((bestD == d2) ? c2 : p2_stub);
                        
                        isDraggingSegmentHorizontal = (std::abs(segP1.y - segP2.y) <= std::abs(segP1.x - segP2.x));
                    }
                }
            }

            if (isControlNet) {
                float arrLen = 8.0f * zoomLevel;
                ImVec2 arr1(p2.x - arrLen, p2.y - arrLen * 0.5f);
                ImVec2 arr2(p2.x - arrLen, p2.y + arrLen * 0.5f);
                drawList->AddTriangleFilled(p2, arr1, arr2, wireColor);
            }
        }
    }

    if (!hoveredPinCompId.empty()) {
        hoveredWireId.clear();
    }

    if (isWiring && !hoveredWireId.empty()) {
        const ComponentInstance* startComp = nullptr;
        for (const auto& c : design.components) {
            if (c.id == wireStartCompId) { startComp = &c; break; }
        }
        const WireInstance* targetWire = nullptr;
        for (auto& w : design.wires) {
            if (w.id == hoveredWireId) { targetWire = &w; break; }
        }

        DomainType startDom = startComp ? getPinDomain(*startComp, wireStartPin) : DomainType::Power;
        DomainType targetWireDom = targetWire ? getWireDomain(*targetWire, design) : DomainType::Power;

        ImVec2 jScreen = worldToScreen(hoveredWireJunctionPos.x, hoveredWireJunctionPos.y, canvasPos);

        if (startDom != targetWireDom) {
            drawList->AddCircleFilled(jScreen, 6.0f * zoomLevel, IM_COL32(255, 50, 50, 255));
            drawList->AddCircle(jScreen, 9.0f * zoomLevel, IM_COL32(255, 50, 50, 255), 0, 2.0f * zoomLevel);
            ImGui::SetTooltip("🚫 CANNOT CONNECT: %s Line cannot attach to %s Wire!",
                startDom == DomainType::Control ? "Control Signal" : "Electrical Power",
                targetWireDom == DomainType::Control ? "Control Signal" : "Electrical Power");
        } else {
            ImU32 dotCol = (startDom == DomainType::Control) ? IM_COL32(56, 189, 248, 255) : IM_COL32(0, 230, 120, 255);
            drawList->AddCircleFilled(jScreen, 6.0f * zoomLevel, dotCol);
            drawList->AddCircle(jScreen, 9.0f * zoomLevel, IM_COL32(255, 200, 50, 255), 0, 2.0f * zoomLevel);
        }
    }

    // Active 4-Way Orthogonal Segment Dragging Engine
    if (isDraggingWireSegment && ImGui::IsMouseDragging(ImGuiMouseButton_Left)) {
        for (auto& w : design.wires) {
            if (w.id == draggingWireId) {
                ImVec2 wPos = screenToWorld(mousePos, canvasPos);
                float snapX = std::round(wPos.x / 20.0f) * 20.0f;
                float snapY = std::round(wPos.y / 20.0f) * 20.0f;

                Point2D pt;
                pt.x = snapX;
                pt.y = snapY;
                w.manualPath = {pt};
                break;
            }
        }
    } else if (ImGui::IsMouseReleased(ImGuiMouseButton_Left)) {
        isDraggingWireSegment = false;
    }

    // Active Wire Drawing Preview with Single Right-Angle L-Shape Corner (Matching User Diagram)
    if (isWiring) {
        ImVec2 p1(0, 0), p1_stub(0, 0);
        bool fromIsVertical = false;
        for (const auto& comp : design.components) {
            if (comp.id == wireStartCompId) {
                getTerminalPortStub(comp, wireStartPin, canvasPos, zoomLevel, p1, p1_stub, fromIsVertical);
                break;
            }
        }

        ImU32 previewColor = IM_COL32(255, 200, 0, 200);
        drawList->AddLine(p1, p1_stub, previewColor, 2.0f * zoomLevel);

        ImVec2 curr = p1_stub;
        for (const auto& corner : activeWireCorners) {
            ImVec2 sc = worldToScreen(corner.x, corner.y, canvasPos);
            drawList->AddLine(curr, sc, previewColor, 2.0f * zoomLevel);
            curr = sc;
        }

        ImVec2 targetP = (!hoveredWireId.empty()) ? worldToScreen(hoveredWireJunctionPos.x, hoveredWireJunctionPos.y, canvasPos) : wireCurrentPos;
        // SINGLE RIGHT-ANGLE L-SHAPE PREVIEW (matching user screenshot 100%)
        ImVec2 mid = fromIsVertical ? ImVec2(curr.x, targetP.y) : ImVec2(targetP.x, curr.y);
        drawList->AddLine(curr, mid, previewColor, 2.0f * zoomLevel);
        drawList->AddLine(mid, targetP, previewColor, 2.0f * zoomLevel);
    }
}

void SchematicCanvas::getComponentBounds(const ComponentInstance& comp, float& outHalfW, float& outHalfH) {
    float minX = 0.0f, maxX = 0.0f, minY = 0.0f, maxY = 0.0f;
    bool hasPins = !comp.pins.empty();

    if (hasPins) {
        minX = comp.pins[0].relativeX;
        maxX = comp.pins[0].relativeX;
        minY = comp.pins[0].relativeY;
        maxY = comp.pins[0].relativeY;
        for (const auto& pin : comp.pins) {
            minX = std::min(minX, (float)pin.relativeX);
            maxX = std::max(maxX, (float)pin.relativeX);
            minY = std::min(minY, (float)pin.relativeY);
            maxY = std::max(maxY, (float)pin.relativeY);
        }
        outHalfW = std::max(std::abs(minX), std::abs(maxX)) + 4.0f;
        outHalfH = std::max(std::abs(minY), std::abs(maxY)) + 4.0f;
    } else {
        outHalfW = 25.0f;
        outHalfH = 25.0f;
    }

    const std::string& t = comp.rawTypeStr;

    if (t == "CSCRIPT") {
        outHalfW = std::max(outHalfW, 82.0f);
        outHalfH = std::max(outHalfH, 42.0f);
    } else if (t == "SUBSYSTEM") {
        outHalfW = std::max(outHalfW, 52.0f);
        outHalfH = std::max(outHalfH, 42.0f);
    } else if (t == "PULSE" || t == "PULSE_GEN") {
        outHalfW = std::max(outHalfW, 24.0f);
        outHalfH = std::max(outHalfH, 18.0f);
    } else if (t == "SCOPE") {
        int numCh = 2;
        if (comp.parameters.count("channels")) {
            try { numCh = std::stoi(comp.parameters.at("channels")); } catch (...) {}
        }
        if (numCh < 1) numCh = 1;
        outHalfW = std::max(outHalfW, 18.0f);
        outHalfH = std::max(outHalfH, std::max(18.0f, numCh * 10.0f + 2.0f));
    } else if (t == "PROBE") {
        std::string sigStr = comp.parameters.count("selected_signals") ? comp.parameters.at("selected_signals") : "";
        int count = 0;
        std::stringstream ss(sigStr);
        std::string item;
        while (std::getline(ss, item, ',')) { if (!item.empty()) count++; }
        int numPins = std::max(1, count);
        outHalfW = std::max(outHalfW, 32.0f);
        outHalfH = std::max(outHalfH, std::max(22.0f, numPins * 15.0f + 2.0f));
    } else if (t == "R" || t == "C" || t == "L" || t == "D" || t == "V" || t == "I" ||
               t == "AC_V" || t == "S" || t == "MOSFET" || t == "VM" || t == "AM") {
        outHalfW = std::max(outHalfW, 14.0f);
        outHalfH = std::max(outHalfH, 14.0f);
    } else {
        outHalfW = std::max(outHalfW, 22.0f);
        outHalfH = std::max(outHalfH, 22.0f);
    }

    int rot = ((comp.rotation % 360) + 360) % 360;
    if (rot == 90 || rot == 270) {
        std::swap(outHalfW, outHalfH);
    }
}

void SchematicCanvas::drawComponents(ImDrawList* drawList, ImVec2 canvasPos) {
    ImGuiIO& io = ImGui::GetIO();
    ImVec2 mousePos = io.MousePos;
    hoveredPinCompId.clear();
    hoveredPinName.clear();
    float minPinDist = 1e9f;
    
    for (auto& comp : design.components) {
        ImVec2 center = worldToScreen(comp.x, comp.y, canvasPos);
        float s = zoomLevel;
        
        bool isSelected = selectedComponentIds.count(comp.id) > 0;
        ImU32 defaultColor = isDarkMode ? IM_COL32(200, 210, 230, 255) : IM_COL32(30, 41, 59, 255);
        ImU32 componentColor = isSelected ? IM_COL32(255, 180, 0, 255) : defaultColor;

        float hw = 25.0f, hh = 25.0f;
        getComponentBounds(comp, hw, hh);

        if (isSelected) {
            drawList->AddRectFilled(
                {center.x - hw*s, center.y - hh*s},
                {center.x + hw*s, center.y + hh*s},
                IM_COL32(255, 180, 0, 40), 6*s);
            drawList->AddRect(
                {center.x - hw*s, center.y - hh*s},
                {center.x + hw*s, center.y + hh*s},
                IM_COL32(255, 180, 0, 180), 6*s, 0, 1.5f*s);
        }
        
        drawComponentShape(drawList, comp, center, s, componentColor);
        ImU32 labelColor = isDarkMode ? IM_COL32(180, 190, 210, 255) : IM_COL32(15, 23, 42, 255);
        drawList->AddText({center.x - (hw - 4.0f)*s, center.y + (hh + 4.0f)*s}, labelColor, comp.label.c_str());
        drawTerminals(drawList, comp, center, s, mousePos, minPinDist);
    }
}

void SchematicCanvas::autoSelectWiresForSelectedComponents() {
    if (selectedComponentIds.empty()) return;

    auto isCompSel = [this](const std::string& cid) -> bool {
        if (cid.empty()) return false;
        return selectedComponentIds.count(cid) > 0;
    };

    auto isNodeSel = [this](const std::string& nodeStr) -> bool {
        if (nodeStr.empty()) return false;
        size_t dot = nodeStr.find('.');
        if (dot != std::string::npos) {
            std::string cid = nodeStr.substr(0, dot);
            return selectedComponentIds.count(cid) > 0;
        }
        return false;
    };

    bool addedAny = true;
    while (addedAny) {
        addedAny = false;
        for (const auto& w : design.wires) {
            if (selectedWireIds.count(w.id) > 0) continue;

            bool fromIsSel = isCompSel(w.from.compId) || isNodeSel(w.fromNode);
            bool toIsSel = isCompSel(w.to.compId) || isNodeSel(w.toNode);

            if (w.to.isWireJunction) {
                if (fromIsSel || selectedWireIds.count(w.to.targetWireId) > 0) {
                    selectedWireIds.insert(w.id);
                    addedAny = true;
                }
            } else if (fromIsSel || toIsSel) {
                selectedWireIds.insert(w.id);
                addedAny = true;
            }
        }
    }
}

void SchematicCanvas::copySelected() {
    if (selectedComponentIds.empty() && selectedWireIds.empty()) return;

    autoSelectWiresForSelectedComponents();

    nlohmann::json jData;
    jData["components"] = nlohmann::json::array();
    jData["wires"] = nlohmann::json::array();

    std::unordered_set<std::string> selComps(selectedComponentIds.begin(), selectedComponentIds.end());
    std::unordered_set<std::string> selWires(selectedWireIds.begin(), selectedWireIds.end());

    for (const auto& comp : design.components) {
        if (selComps.count(comp.id)) {
            nlohmann::json jComp;
            jComp["id"] = comp.id;
            jComp["type"] = comp.rawTypeStr;
            jComp["label"] = comp.label;
            jComp["x"] = comp.x;
            jComp["y"] = comp.y;
            jComp["rotation"] = comp.rotation;
            jComp["params"] = comp.parameters;
            
            nlohmann::json jPins = nlohmann::json::array();
            for (const auto& p : comp.pins) {
                nlohmann::json jp;
                jp["name"] = p.name;
                jp["relX"] = p.relativeX;
                jp["relY"] = p.relativeY;
                jp["isInput"] = p.isInput;
                jp["isOutput"] = p.isOutput;
                jp["opSign"] = p.opSign;
                jPins.push_back(jp);
            }
            jComp["pins"] = jPins;

            jData["components"].push_back(jComp);
        }
    }

    std::unordered_map<std::string, std::string> oldToNewWireId;
    int wireCounter = 1;
    for (const auto& w : design.wires) {
        if (selWires.count(w.id) > 0) {
            std::string tempWireId = "w_copy_" + std::to_string(wireCounter++);
            oldToNewWireId[w.id] = tempWireId;

            nlohmann::json jw;
            jw["id"] = tempWireId;
            jw["oldId"] = w.id;
            jw["fromComp"] = w.from.compId;
            jw["fromTerm"] = w.from.terminal;
            jw["toComp"] = w.to.compId;
            jw["toTerm"] = w.to.terminal;
            jw["fromNode"] = w.fromNode;
            jw["toNode"] = w.toNode;
            jw["isJunction"] = w.to.isWireJunction;
            jw["targetWireId"] = w.to.targetWireId;
            jw["jX"] = w.to.junctionX;
            jw["jY"] = w.to.junctionY;
            
            nlohmann::json jPath = nlohmann::json::array();
            for (const auto& pt : w.manualPath) {
                jPath.push_back({{"x", pt.x}, {"y", pt.y}});
            }
            jw["path"] = jPath;

            jData["wires"].push_back(jw);
        }
    }

    std::string jsonStr = jData.dump(2);
    g_internalClipboard = jsonStr;

    if (OpenClipboard(NULL)) {
        EmptyClipboard();
        HGLOBAL hMem = GlobalAlloc(GMEM_MOVEABLE, jsonStr.size() + 1);
        if (hMem) {
            memcpy(GlobalLock(hMem), jsonStr.c_str(), jsonStr.size() + 1);
            GlobalUnlock(hMem);
            SetClipboardData(CF_TEXT, hMem);
        }
        CloseClipboard();
    }
}

void SchematicCanvas::pasteSelected() {
    pushUndoState();
    std::string jsonStr;

    if (OpenClipboard(NULL)) {
        HANDLE hData = GetClipboardData(CF_TEXT);
        if (hData) {
            char* pszText = static_cast<char*>(GlobalLock(hData));
            if (pszText) {
                jsonStr = std::string(pszText);
                GlobalUnlock(hData);
            }
        }
        CloseClipboard();
    }

    if (jsonStr.empty() || jsonStr.find("components") == std::string::npos) {
        jsonStr = g_internalClipboard;
    }

    if (jsonStr.empty()) return;

    try {
        nlohmann::json jData = nlohmann::json::parse(jsonStr);
        if (!jData.contains("components") || !jData["components"].is_array()) return;

        selectedComponentIds.clear();
        selectedWireIds.clear();

        std::unordered_map<std::string, std::string> oldToNewId;
        float offsetX = 30.0f;
        float offsetY = 30.0f;

        for (const auto& jComp : jData["components"]) {
            ComponentInstance comp;
            std::string oldId = jComp.value("id", "comp");
            std::string typeStr = jComp.value("type", "R");
            
            std::string prefix = typeStr;
            int idx = 1;
            std::string newId;
            while (true) {
                newId = prefix + std::to_string(idx);
                bool exists = false;
                for (const auto& existing : design.components) {
                    if (existing.id == newId) { exists = true; break; }
                }
                if (!exists && oldToNewId.find(newId) == oldToNewId.end()) break;
                idx++;
            }

            oldToNewId[oldId] = newId;
            comp.id = newId;
            comp.rawTypeStr = typeStr;
            comp.label = jComp.value("label", newId);
            comp.x = jComp.value("x", 0.0f) + offsetX;
            comp.y = jComp.value("y", 0.0f) + offsetY;
            comp.rotation = jComp.value("rotation", 0);

            if (jComp.contains("params")) {
                for (auto& [k, v] : jComp["params"].items()) {
                    if (v.is_string()) comp.parameters[k] = v.get<std::string>();
                    else comp.parameters[k] = v.dump();
                }
            }

            if (jComp.contains("pins") && jComp["pins"].is_array()) {
                for (const auto& jp : jComp["pins"]) {
                    Pin p;
                    p.name = jp.value("name", "");
                    p.relativeX = jp.value("relX", 0.0f);
                    p.relativeY = jp.value("relY", 0.0f);
                    p.isInput = jp.value("isInput", false);
                    p.isOutput = jp.value("isOutput", false);
                    p.opSign = jp.value("opSign", "+");
                    comp.pins.push_back(p);
                }
            }

            if (typeStr == "R") comp.type = ComponentType::Resistor;
            else if (typeStr == "C") comp.type = ComponentType::Capacitor;
            else if (typeStr == "L") comp.type = ComponentType::Inductor;
            else if (typeStr == "V") comp.type = ComponentType::VoltageSource;
            else if (typeStr == "AC_V") comp.type = ComponentType::ACVoltageSource;
            else if (typeStr == "I") comp.type = ComponentType::CurrentSource;
            else if (typeStr == "D") comp.type = ComponentType::Diode;
            else if (typeStr == "MOSFET") comp.type = ComponentType::MOSFET;
            else if (typeStr == "S") comp.type = ComponentType::Switch;
            else if (typeStr == "VM") comp.type = ComponentType::Voltmeter;
            else if (typeStr == "AM") comp.type = ComponentType::Ammeter;
            else if (typeStr == "GAIN") comp.type = ComponentType::Gain;
            else if (typeStr == "PID") comp.type = ComponentType::PI_Controller;
            else if (typeStr == "COMP") comp.type = ComponentType::Comparator;
            else if (typeStr == "PWM") comp.type = ComponentType::PWM_Generator;
            else if (typeStr == "PULSE" || typeStr == "PULSE_GEN") comp.type = ComponentType::PulseGenerator;
            else if (typeStr == "TRI") comp.type = ComponentType::Triangle_Carrier;
            else if (typeStr == "SUM_RECT" || typeStr == "SUM_ROUND") comp.type = ComponentType::SummingJunction;
            else if (typeStr == "PRODUCT_RECT") comp.type = ComponentType::Product;
            else if (typeStr == "AND") comp.type = ComponentType::AND_Gate;
            else if (typeStr == "OR") comp.type = ComponentType::OR_Gate;
            else if (typeStr == "NOT") comp.type = ComponentType::NOT_Gate;
            else if (typeStr == "CSCRIPT") comp.type = ComponentType::CustomScript;
            else comp.type = ComponentType::Unknown;

            design.components.push_back(comp);
            selectedComponentIds.insert(newId);
        }

        if (jData.contains("wires") && jData["wires"].is_array()) {
            std::unordered_map<std::string, std::string> oldWireIdToNewWireId;

            for (const auto& jw : jData["wires"]) {
                WireInstance wire;
                std::string oldWId = jw.value("id", "");
                std::string origWId = jw.value("oldId", "");
                std::string newWId = "wire_" + std::to_string(rand() % 100000);
                
                if (!oldWId.empty()) oldWireIdToNewWireId[oldWId] = newWId;
                if (!origWId.empty()) oldWireIdToNewWireId[origWId] = newWId;

                wire.id = newWId;
                std::string oldFromComp = jw.value("fromComp", "");
                std::string oldToComp = jw.value("toComp", "");

                wire.from.compId = oldToNewId.count(oldFromComp) ? oldToNewId[oldFromComp] : oldFromComp;
                wire.from.terminal = jw.value("fromTerm", "");
                wire.to.compId = oldToNewId.count(oldToComp) ? oldToNewId[oldToComp] : oldToComp;
                wire.to.terminal = jw.value("toTerm", "");

                wire.to.isWireJunction = jw.value("isJunction", false);
                std::string origTargetW = jw.value("targetWireId", "");
                wire.to.targetWireId = oldWireIdToNewWireId.count(origTargetW) ? oldWireIdToNewWireId[origTargetW] : origTargetW;
                
                wire.to.junctionX = jw.value("jX", 0.0f) + offsetX;
                wire.to.junctionY = jw.value("jY", 0.0f) + offsetY;

                wire.fromNode = wire.from.compId + "." + wire.from.terminal;
                wire.toNode = wire.to.isWireJunction ? (wire.to.targetWireId + ".junction") : (wire.to.compId + "." + wire.to.terminal);

                if (jw.contains("path") && jw["path"].is_array()) {
                    for (const auto& jpt : jw["path"]) {
                        Point2D pt;
                        pt.x = jpt.value("x", 0.0f) + offsetX;
                        pt.y = jpt.value("y", 0.0f) + offsetY;
                        wire.manualPath.push_back(pt);
                    }
                }

                design.wires.push_back(wire);
                selectedWireIds.insert(newWId);
            }
        }
    } catch (...) {}
}

void SchematicCanvas::duplicateSelected() {
    copySelected();
    pasteSelected();
}

void SchematicCanvas::flipHorizontal() {
    pushUndoState();
    for (auto& comp : design.components) {
        if (selectedComponentIds.count(comp.id)) {
            comp.rotation = (comp.rotation + 180) % 360;
        }
    }
}

void SchematicCanvas::flipVertical() {
    flipHorizontal();
}

void SchematicCanvas::renderModals() {
    if (showConfigurator && pendingConfigCompIdx >= 0 && pendingConfigCompIdx < (int)design.components.size()) {
        if (ConfiguratorDialog::showConfiguratorModal(design.components[pendingConfigCompIdx], &showConfigurator)) {
            pendingConfigCompIdx = -1;
        }
    }

    if (showCScriptModal) {
        ImGui::OpenPopup("Edit C-Script Logic Modal");
        if (ImGui::BeginPopupModal("Edit C-Script Logic Modal", &showCScriptModal, ImGuiWindowFlags_AlwaysAutoResize)) {
            ImGui::Text("C-Script Execution Logic (C++ Syntax):");
            ImGui::InputTextMultiline("##code", cscriptCodeBuf, sizeof(cscriptCodeBuf), ImVec2(500, 300));
            if (ImGui::Button("Save Code", ImVec2(120, 30))) {
                pushUndoState();
                if (cscriptCompIdx >= 0 && cscriptCompIdx < (int)design.components.size()) {
                    design.components[cscriptCompIdx].parameters["code"] = cscriptCodeBuf;
                }
                showCScriptModal = false;
                cscriptCompIdx = -1;
                ImGui::CloseCurrentPopup();
            }
            ImGui::SameLine();
            if (ImGui::Button("Cancel", ImVec2(120, 30))) {
                showCScriptModal = false;
                cscriptCompIdx = -1;
                ImGui::CloseCurrentPopup();
            }
            ImGui::EndPopup();
        }
    }

    if (showScopeModal) {
        ImGui::OpenPopup("Oscilloscope Configuration Modal");
        if (ImGui::BeginPopupModal("Oscilloscope Configuration Modal", &showScopeModal, ImGuiWindowFlags_AlwaysAutoResize)) {
            ImGui::Text("Edit Oscilloscope Configuration:");
            ImGui::Separator();
            ImGui::InputText("Number of Channels (1..8)", scopeChannelsBuf, sizeof(scopeChannelsBuf));
            ImGui::Spacing();
            if (ImGui::Button("Save Parameters", ImVec2(140, 30))) {
                pushUndoState();
                if (scopeCompIdx >= 0 && scopeCompIdx < (int)design.components.size()) {
                    int numCh = 2;
                    try { numCh = std::stoi(scopeChannelsBuf); } catch (...) {}
                    if (numCh < 1) numCh = 1;
                    if (numCh > 8) numCh = 8;
                    design.components[scopeCompIdx].parameters["channels"] = std::to_string(numCh);
                    design.components[scopeCompIdx].label = "SCOPE (" + std::to_string(numCh) + " Ch)";
                }
                showScopeModal = false;
                scopeCompIdx = -1;
                ImGui::CloseCurrentPopup();
            }
            ImGui::SameLine();
            if (ImGui::Button("Cancel", ImVec2(100, 30))) {
                showScopeModal = false;
                scopeCompIdx = -1;
                ImGui::CloseCurrentPopup();
            }
            ImGui::EndPopup();
        }
    }

    if (showPulseModal) {
        ImGui::OpenPopup("Pulse Generator Parameters Modal");
        if (ImGui::BeginPopupModal("Pulse Generator Parameters Modal", &showPulseModal, ImGuiWindowFlags_AlwaysAutoResize)) {
            ImGui::Text("Edit Pulse Generator Configuration:");
            ImGui::Separator();
            ImGui::InputText("Amplitude", pulseAmpBuf, sizeof(pulseAmpBuf));
            ImGui::InputText("Period (s)", pulsePeriodBuf, sizeof(pulsePeriodBuf));
            ImGui::InputText("Pulse Width (0..1)", pulseWidthBuf, sizeof(pulseWidthBuf));
            ImGui::InputText("Delay (s)", pulseDelayBuf, sizeof(pulseDelayBuf));
            ImGui::Spacing();
            if (ImGui::Button("Save Parameters", ImVec2(140, 30))) {
                pushUndoState();
                if (pulseCompIdx >= 0 && pulseCompIdx < (int)design.components.size()) {
                    auto& p = design.components[pulseCompIdx].parameters;
                    p.clear();
                    p["amplitude"] = pulseAmpBuf;
                    p["period"] = pulsePeriodBuf;
                    p["width"] = pulseWidthBuf;
                    p["delay"] = pulseDelayBuf;
                    design.components[pulseCompIdx].label = "Pulse Gen";
                }
                showPulseModal = false;
                pulseCompIdx = -1;
                ImGui::CloseCurrentPopup();
            }
            ImGui::SameLine();
            if (ImGui::Button("Cancel", ImVec2(100, 30))) {
                showPulseModal = false;
                pulseCompIdx = -1;
                ImGui::CloseCurrentPopup();
            }
            ImGui::EndPopup();
        }
    }
}

void SchematicCanvas::render(const char* title, ImVec2 size) {
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0, 0));
    ImGui::Begin(title, nullptr, ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
    
    ImVec2 canvasPos = ImGui::GetCursorScreenPos();
    ImVec2 canvasSize = ImGui::GetContentRegionAvail();
    lastRenderedCanvasSize = canvasSize;
    ImDrawList* drawList = ImGui::GetWindowDrawList();

    ImU32 canvasBgColor = isDarkMode ? IM_COL32(15, 23, 42, 255) : IM_COL32(246, 243, 206, 255);
    drawList->AddRectFilled(canvasPos, ImVec2(canvasPos.x + canvasSize.x, canvasPos.y + canvasSize.y), canvasBgColor);
    drawList->PushClipRect(canvasPos, ImVec2(canvasPos.x + canvasSize.x, canvasPos.y + canvasSize.y), true);
    
    drawGrid(drawList, canvasSize, canvasPos);
    drawComponents(drawList, canvasPos);
    drawWires(drawList, canvasPos);
    drawBreadcrumbs(drawList, canvasPos);

    ImGuiIO& io = ImGui::GetIO();
    ImVec2 mousePos = io.MousePos;

    if (isBoxSelecting) {
        boxSelectEnd = mousePos;
        drawList->AddRectFilled(boxSelectStart, boxSelectEnd, IM_COL32(255, 180, 0, 35));
        drawList->AddRect(boxSelectStart, boxSelectEnd, IM_COL32(255, 180, 0, 255), 0, 0, 1.5f);
    }

    drawList->PopClipRect();

    // Adaptive Box Zoom rubber-band — drawn OUTSIDE clip rect so it renders on top
    if (isBoxZooming) {
        boxZoomEnd = mousePos;
        float bx0 = std::min(boxZoomStart.x, boxZoomEnd.x);
        float by0 = std::min(boxZoomStart.y, boxZoomEnd.y);
        float bx1 = std::max(boxZoomStart.x, boxZoomEnd.x);
        float by1 = std::max(boxZoomStart.y, boxZoomEnd.y);
        float bw = bx1 - bx0, bh = by1 - by0;
        float aspect = (bh > 1.0f) ? (bw / bh) : 99.0f;

        if (aspect > 3.0f) {
            // X-zoom: draw full-height cyan band spanning the canvas height
            float top    = canvasPos.y;
            float bottom = canvasPos.y + canvasSize.y;
            drawList->AddRectFilled(ImVec2(bx0, top), ImVec2(bx1, bottom), IM_COL32(0, 220, 255, 22));
            drawList->AddLine(ImVec2(bx0, top), ImVec2(bx0, bottom), IM_COL32(0, 220, 255, 255), 2.0f);
            drawList->AddLine(ImVec2(bx1, top), ImVec2(bx1, bottom), IM_COL32(0, 220, 255, 255), 2.0f);
        } else if (aspect < 0.33f) {
            // Y-zoom: draw full-width magenta band spanning the canvas width
            float left  = canvasPos.x;
            float right = canvasPos.x + canvasSize.x;
            drawList->AddRectFilled(ImVec2(left, by0), ImVec2(right, by1), IM_COL32(220, 0, 255, 22));
            drawList->AddLine(ImVec2(left, by0), ImVec2(right, by0), IM_COL32(220, 0, 255, 255), 2.0f);
            drawList->AddLine(ImVec2(left, by1), ImVec2(right, by1), IM_COL32(220, 0, 255, 255), 2.0f);
        } else {
            // Box zoom: green rectangle
            drawList->AddRectFilled(ImVec2(bx0, by0), ImVec2(bx1, by1), IM_COL32(60, 255, 120, 30));
            drawList->AddRect(ImVec2(bx0, by0), ImVec2(bx1, by1), IM_COL32(60, 255, 120, 255), 0, 0, 2.0f);
        }
    }

    // Set focus on click
    if (ImGui::IsWindowHovered() && ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
        ImGui::SetWindowFocus();
    }

    // Escape or Right-Click cancels wiring / box-zoom drag
    if (isWiring && (ImGui::IsKeyPressed(ImGuiKey_Escape) || ImGui::IsMouseClicked(ImGuiMouseButton_Right))) {
        isWiring = false;
        activeWireCorners.clear();
    }
    if (isBoxZooming && ImGui::IsKeyPressed(ImGuiKey_Escape)) {
        isBoxZooming = false;
    }

    // Mouse Pan with Right Button or Spacebar + Left Mouse
    // When adaptive zoom is active, left-drag is NOT a pan — it's a zoom drag
    bool lmbPanActive = ImGui::IsKeyDown(ImGuiKey_Space) && ImGui::IsMouseDragging(ImGuiMouseButton_Left);
    bool rmbPanActive = ImGui::IsMouseDragging(ImGuiMouseButton_Right);
    bool suppressPan = adaptiveZoomMode && !lmbPanActive; // in zoom mode, only space+LMB still pans
    if (ImGui::IsWindowHovered() && !isDraggingWireSegment && (rmbPanActive || lmbPanActive)) {
        if (!suppressPan || lmbPanActive) {
            panOffset.x += io.MouseDelta.x / zoomLevel;
            panOffset.y += io.MouseDelta.y / zoomLevel;
        }
    }

    // Zoom with scroll wheel centered on mouse pointer position
    if (ImGui::IsWindowHovered() && io.MouseWheel != 0.0f) {
        float oldZoom = zoomLevel;
        float zoomFactor = 1.15f;
        float newZoom = (io.MouseWheel > 0.0f) ? (oldZoom * zoomFactor) : (oldZoom / zoomFactor);
        
        if (newZoom < 0.15f) newZoom = 0.15f;
        if (newZoom > 6.0f) newZoom = 6.0f;
        
        if (newZoom != oldZoom) {
            ImVec2 worldMouse = screenToWorld(mousePos, canvasPos);
            zoomLevel = newZoom;
            panOffset.x = (mousePos.x - canvasPos.x) / newZoom - worldMouse.x;
            panOffset.y = (mousePos.y - canvasPos.y) / newZoom - worldMouse.y;
        }
    }

    // Left click handling
    if (ImGui::IsWindowHovered() && ImGui::IsMouseClicked(ImGuiMouseButton_Left) && !isDraggingWireSegment) {
        ImVec2 clickW = screenToWorld(mousePos, canvasPos);
        lastCanvasClickWorldPos.x = std::round(clickW.x / 20.0f) * 20.0f;
        lastCanvasClickWorldPos.y = std::round(clickW.y / 20.0f) * 20.0f;
        hasLastClickPos = true;

        if (adaptiveZoomMode && !isWiring) {
            // Adaptive box zoom mode: always start rubber-band regardless of what is under cursor
            isBoxZooming = true;
            boxZoomStart = mousePos;
            boxZoomEnd   = mousePos;
        } else if (!hoveredPinCompId.empty()) {
            if (!isWiring) {
                isWiring = true;
                wireStartCompId = hoveredPinCompId;
                wireStartPin = hoveredPinName;
                wireCurrentPos = mousePos;
                activeWireCorners.clear();
            } else {
                // STRICT DOMAIN VALIDATION: Pin-to-Pin Connection
                const ComponentInstance* startComp = nullptr;
                const ComponentInstance* targetComp = nullptr;
                for (const auto& c : design.components) {
                    if (c.id == wireStartCompId) startComp = &c;
                    if (c.id == hoveredPinCompId) targetComp = &c;
                }

                if (startComp && targetComp) {
                    DomainType startDom = getPinDomain(*startComp, wireStartPin);
                    DomainType targetDom = getPinDomain(*targetComp, hoveredPinName);

                    if (startDom != targetDom) {
                        // REJECT CROSS-DOMAIN PIN CONNECTION!
                        isWiring = false;
                        activeWireCorners.clear();
                    } else if (!validateSingleOutportConstraint(wireStartCompId, wireStartPin, hoveredPinCompId, hoveredPinName)) {
                        // REJECT MULTIPLE CONTROL OUTPUT DRIVERS CONFLICT!
                        isWiring = false;
                        activeWireCorners.clear();
                    } else {
                        pushUndoState();
                        WireInstance wire;
                        wire.id = "w" + std::to_string(design.wires.size() + 1);
                        wire.from.compId = wireStartCompId;
                        wire.from.terminal = wireStartPin;
                        wire.to.compId = hoveredPinCompId;
                        wire.to.terminal = hoveredPinName;
                        design.wires.push_back(wire);
                        normalizeControlWires();
                        isWiring = false;
                        activeWireCorners.clear();
                    }
                }
            }
        } else if (isWiring) {
            // STRICT DOMAIN VALIDATION: T-Junction Connection
            if (!hoveredWireId.empty()) {
                const ComponentInstance* startComp = nullptr;
                for (const auto& c : design.components) {
                    if (c.id == wireStartCompId) { startComp = &c; break; }
                }
                const WireInstance* targetWire = nullptr;
                for (const auto& w : design.wires) {
                    if (w.id == hoveredWireId) { targetWire = &w; break; }
                }

                if (startComp && targetWire) {
                    DomainType startDom = getPinDomain(*startComp, wireStartPin);
                    DomainType targetWireDom = getWireDomain(*targetWire, design);

                    if (startDom != targetWireDom) {
                        // REJECT CROSS-DOMAIN T-JUNCTION CONNECTION!
                        isWiring = false;
                        activeWireCorners.clear();
                    } else {
                        pushUndoState();
                        WireInstance wire;
                        wire.id = "w" + std::to_string(design.wires.size() + 1);
                        wire.from.compId = wireStartCompId;
                        wire.from.terminal = wireStartPin;
                        wire.to.isWireJunction = true;
                        wire.to.targetWireId = hoveredWireId;
                        wire.to.junctionX = hoveredWireJunctionPos.x;
                        wire.to.junctionY = hoveredWireJunctionPos.y;
                        design.wires.push_back(wire);
                        normalizeControlWires();
                        isWiring = false;
                        activeWireCorners.clear();
                    }
                }
            } else {
                ImVec2 worldCorner = screenToWorld(mousePos, canvasPos);
                activeWireCorners.push_back(worldCorner);
            }
        } else {
            bool hitComp = false;
            for (auto& comp : design.components) {
                ImVec2 center = worldToScreen(comp.x, comp.y, canvasPos);
                float hw = 25.0f, hh = 25.0f;
                getComponentBounds(comp, hw, hh);

                if (mousePos.x >= center.x - hw*zoomLevel && mousePos.x <= center.x + hw*zoomLevel &&
                    mousePos.y >= center.y - hh*zoomLevel && mousePos.y <= center.y + hh*zoomLevel) {
                    
                    if (!io.KeyShift && selectedComponentIds.count(comp.id) == 0) {
                        selectedComponentIds.clear();
                        selectedWireIds.clear();
                    }
                    selectedComponentIds.insert(comp.id);
                    hitComp = true;
                    break;
                }
            }
            if (!hitComp) {
                if (!hoveredWireId.empty()) {
                    if (!io.KeyShift) {
                        selectedComponentIds.clear();
                        selectedWireIds.clear();
                    }
                    selectedWireIds.insert(hoveredWireId);
                } else if (adaptiveZoomMode) {
                    // In zoom mode: start box-zoom rubber-band, don't touch selection
                    isBoxZooming = true;
                    boxZoomStart = mousePos;
                    boxZoomEnd  = mousePos;
                } else if (!io.KeyShift) {
                    selectedComponentIds.clear();
                    selectedWireIds.clear();
                    isBoxSelecting = true;
                    boxSelectStart = mousePos;
                }
            }
        }
    }

    if (isBoxSelecting && ImGui::IsMouseReleased(ImGuiMouseButton_Left)) {
        isBoxSelecting = false;
        ImVec2 minP(std::min(boxSelectStart.x, boxSelectEnd.x), std::min(boxSelectStart.y, boxSelectEnd.y));
        ImVec2 maxP(std::max(boxSelectStart.x, boxSelectEnd.x), std::max(boxSelectStart.y, boxSelectEnd.y));

        for (const auto& comp : design.components) {
            ImVec2 center = worldToScreen(comp.x, comp.y, canvasPos);
            float hw = 25.0f, hh = 25.0f;
            getComponentBounds(comp, hw, hh);

            if (center.x + hw*zoomLevel >= minP.x && center.x - hw*zoomLevel <= maxP.x &&
                center.y + hh*zoomLevel >= minP.y && center.y - hh*zoomLevel <= maxP.y) {
                selectedComponentIds.insert(comp.id);
            }
        }

        autoSelectWiresForSelectedComponents();
    }

    // Commit adaptive box zoom on mouse release
    if (isBoxZooming && ImGui::IsMouseReleased(ImGuiMouseButton_Left)) {
        isBoxZooming = false;
        float bx0 = std::min(boxZoomStart.x, boxZoomEnd.x);
        float by0 = std::min(boxZoomStart.y, boxZoomEnd.y);
        float bx1 = std::max(boxZoomStart.x, boxZoomEnd.x);
        float by1 = std::max(boxZoomStart.y, boxZoomEnd.y);
        float bw = bx1 - bx0, bh = by1 - by0;

        // Require a minimum drag of 8 pixels to avoid accidental trigger
        if (bw > 8.0f || bh > 8.0f) {
            float aspect = (bh > 1.0f) ? (bw / bh) : 99.0f;
            float viewW = canvasSize.x;
            float viewH = canvasSize.y;

            if (aspect > 3.0f) {
                // --- X-axis zoom only ---
                // Center Y stays, zoom so that bw maps to viewW
                ImVec2 wldLeft  = screenToWorld(ImVec2(bx0, by0 + bh * 0.5f), canvasPos);
                ImVec2 wldRight = screenToWorld(ImVec2(bx1, by0 + bh * 0.5f), canvasPos);
                float worldW = wldRight.x - wldLeft.x;
                if (worldW > 0.001f) {
                    float newZoom = viewW / worldW;
                    newZoom = std::max(0.15f, std::min(newZoom, 6.0f));
                    float worldMidX = (wldLeft.x + wldRight.x) * 0.5f;
                    panOffset.x = viewW / (2.0f * newZoom) - worldMidX;
                    zoomLevel = newZoom;
                }
            } else if (aspect < 0.33f) {
                // --- Y-axis zoom only ---
                ImVec2 wldTop    = screenToWorld(ImVec2(bx0 + bw * 0.5f, by0), canvasPos);
                ImVec2 wldBottom = screenToWorld(ImVec2(bx0 + bw * 0.5f, by1), canvasPos);
                float worldH = wldBottom.y - wldTop.y;
                if (worldH > 0.001f) {
                    float newZoom = viewH / worldH;
                    newZoom = std::max(0.15f, std::min(newZoom, 6.0f));
                    float worldMidY = (wldTop.y + wldBottom.y) * 0.5f;
                    panOffset.y = viewH / (2.0f * newZoom) - worldMidY;
                    zoomLevel = newZoom;
                }
            } else {
                // --- Full box zoom ---
                ImVec2 wldTL = screenToWorld(ImVec2(bx0, by0), canvasPos);
                ImVec2 wldBR = screenToWorld(ImVec2(bx1, by1), canvasPos);
                float worldW = wldBR.x - wldTL.x;
                float worldH = wldBR.y - wldTL.y;
                if (worldW > 0.001f && worldH > 0.001f) {
                    float zx = viewW / worldW;
                    float zy = viewH / worldH;
                    float newZoom = std::min(zx, zy);
                    newZoom = std::max(0.15f, std::min(newZoom, 6.0f));
                    float worldMidX = (wldTL.x + wldBR.x) * 0.5f;
                    float worldMidY = (wldTL.y + wldBR.y) * 0.5f;
                    panOffset.x = viewW / (2.0f * newZoom) - worldMidX;
                    panOffset.y = viewH / (2.0f * newZoom) - worldMidY;
                    zoomLevel = newZoom;
                }
            }
        }
    }

    if (isWiring) wireCurrentPos = mousePos;

    // Drag selected group (with Ctrl+Drag cloning support)
    static bool hasClonedForCtrlDrag = false;
    if (ImGui::IsWindowHovered() && !selectedComponentIds.empty() && !isWiring && !isBoxSelecting && !isBoxZooming && !isDraggingWireSegment) {
        if (ImGui::IsMouseDragging(ImGuiMouseButton_Left)) {
            if (io.KeyCtrl && !hasClonedForCtrlDrag) {
                duplicateSelected();
                hasClonedForCtrlDrag = true;
            }
            for (auto& comp : design.components) {
                if (selectedComponentIds.count(comp.id)) {
                    comp.x += io.MouseDelta.x / zoomLevel;
                    comp.y += io.MouseDelta.y / zoomLevel;
                }
            }
        } else {
            hasClonedForCtrlDrag = false;
        }
    }

    // Right-Click Context Menu for Selection
    if (ImGui::IsWindowHovered() && ImGui::IsMouseClicked(ImGuiMouseButton_Right) && (!selectedComponentIds.empty() || !selectedWireIds.empty())) {
        ImGui::OpenPopup("ComponentContextMenu");
    }
    if (ImGui::BeginPopup("ComponentContextMenu")) {
        if (!selectedComponentIds.empty()) {
            if (ImGui::MenuItem("Rotate 90° (R)")) {
                pushUndoState();
                for (auto& comp : design.components) {
                    if (selectedComponentIds.count(comp.id)) comp.rotation = (comp.rotation + 90) % 360;
                }
            }
            if (ImGui::MenuItem("Flip Horizontal (H)")) flipHorizontal();
            if (ImGui::MenuItem("Flip Vertical (V)")) flipVertical();
            ImGui::Separator();
            if (ImGui::MenuItem("Duplicate (Ctrl+D)")) duplicateSelected();
        }
        if (ImGui::MenuItem("Delete (Del)")) {
            pushUndoState();
            auto shouldRemoveWire = [this](const WireInstance& w) {
                if (selectedWireIds.count(w.id) > 0) return true;
                if (!w.from.compId.empty() && selectedComponentIds.count(w.from.compId) > 0) return true;
                if (!w.to.compId.empty() && selectedComponentIds.count(w.to.compId) > 0) return true;
                for (const std::string& compId : selectedComponentIds) {
                    std::string prefix = compId + ".";
                    if (w.fromNode.rfind(prefix, 0) == 0 || w.toNode.rfind(prefix, 0) == 0) return true;
                }
                return false;
            };

            design.wires.erase(
                std::remove_if(design.wires.begin(), design.wires.end(), shouldRemoveWire),
                design.wires.end()
            );
            design.components.erase(
                std::remove_if(design.components.begin(), design.components.end(),
                               [this](const ComponentInstance& c) { return selectedComponentIds.count(c.id) > 0; }),
                design.components.end()
            );
            selectedComponentIds.clear();
            selectedWireIds.clear();
        }
        ImGui::EndPopup();
    }

    // Double-click handler to open dynamic modals or enter Subsystem
    if (ImGui::IsWindowHovered() && ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) {
        for (size_t i = 0; i < design.components.size(); ++i) {
            auto& comp = design.components[i];
            ImVec2 center = worldToScreen(comp.x, comp.y, canvasPos);
            float hw = 25.0f, hh = 25.0f;
            getComponentBounds(comp, hw, hh);

            if (mousePos.x >= center.x - hw*zoomLevel && mousePos.x <= center.x + hw*zoomLevel &&
                mousePos.y >= center.y - hh*zoomLevel && mousePos.y <= center.y + hh*zoomLevel) {
                
                if (comp.rawTypeStr == "SUBSYSTEM") {
                    subsystemStack.push_back({comp.label.empty() ? comp.id : comp.label, design});
                    design = CircuitDesign();
                } else if (comp.rawTypeStr == "CSCRIPT") {
                    showCScriptModal = true;
                    cscriptCompIdx = (int)i;
                    strncpy(cscriptCodeBuf, comp.parameters["code"].c_str(), sizeof(cscriptCodeBuf));
                } else if (comp.rawTypeStr == "PULSE" || comp.rawTypeStr == "PULSE_GEN") {
                    showPulseModal = true;
                    pulseCompIdx = (int)i;
                    std::string amp = comp.parameters.count("amplitude") ? comp.parameters["amplitude"] : "1";
                    std::string period = comp.parameters.count("period") ? comp.parameters["period"] : "1";
                    std::string width = comp.parameters.count("width") ? comp.parameters["width"] : "0.5";
                    std::string delay = comp.parameters.count("delay") ? comp.parameters["delay"] : "0";

                    strncpy(pulseAmpBuf, amp.c_str(), sizeof(pulseAmpBuf));
                    strncpy(pulsePeriodBuf, period.c_str(), sizeof(pulsePeriodBuf));
                    strncpy(pulseWidthBuf, width.c_str(), sizeof(pulseWidthBuf));
                    strncpy(pulseDelayBuf, delay.c_str(), sizeof(pulseDelayBuf));
                } else if (comp.rawTypeStr == "SCOPE") {
                    showScopeModal = true;
                    scopeCompIdx = (int)i;
                    std::string ch = comp.parameters.count("channels") ? comp.parameters["channels"] : "2";
                    strncpy(scopeChannelsBuf, ch.c_str(), sizeof(scopeChannelsBuf));
                } else if (comp.rawTypeStr == "SUM_RECT" || comp.rawTypeStr == "SUM_ROUND" || comp.rawTypeStr == "PRODUCT_RECT") {
                    showConfigurator = true;
                    pendingConfigCompIdx = (int)i;
                }
                break;
            }
        }
    }

    // Keyboard Shortcuts (works when window is hovered or focused)
    if (ImGui::IsWindowFocused() || ImGui::IsWindowHovered()) {
        if (io.KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_Z)) undo();
        if (io.KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_Y)) redo();
        if (io.KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_C)) copySelected();
        if (io.KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_V)) pasteSelected();
        if (io.KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_D)) duplicateSelected();
        if (ImGui::IsKeyPressed(ImGuiKey_F)) fitToScreen(canvasSize);
        if (ImGui::IsKeyPressed(ImGuiKey_H)) flipHorizontal();
        if (ImGui::IsKeyPressed(ImGuiKey_V)) flipVertical();
        if (ImGui::IsKeyPressed(ImGuiKey_R)) {
            pushUndoState();
            for (auto& comp : design.components) {
                if (selectedComponentIds.count(comp.id)) comp.rotation = (comp.rotation + 90) % 360;
            }
        }
        if (ImGui::IsKeyPressed(ImGuiKey_Delete)) {
            pushUndoState();
            auto shouldRemoveWire = [this](const WireInstance& w) {
                if (selectedWireIds.count(w.id) > 0) return true;
                if (!w.from.compId.empty() && selectedComponentIds.count(w.from.compId) > 0) return true;
                if (!w.to.compId.empty() && selectedComponentIds.count(w.to.compId) > 0) return true;
                for (const std::string& compId : selectedComponentIds) {
                    std::string prefix = compId + ".";
                    if (w.fromNode.rfind(prefix, 0) == 0 || w.toNode.rfind(prefix, 0) == 0) return true;
                }
                return false;
            };

            design.wires.erase(
                std::remove_if(design.wires.begin(), design.wires.end(), shouldRemoveWire),
                design.wires.end()
            );
            design.components.erase(
                std::remove_if(design.components.begin(), design.components.end(),
                               [this](const ComponentInstance& c) { return selectedComponentIds.count(c.id) > 0; }),
                design.components.end()
            );
            selectedComponentIds.clear();
            selectedWireIds.clear();
        }
    }

    renderModals();

    ImGui::End();
    ImGui::PopStyleVar();
}

void SchematicCanvas::syncProbeSignals() {
    std::vector<std::string> probedSignals;
    std::vector<std::string> probedTargets;

    for (const auto& comp : design.components) {
        bool isProbed = (comp.parameters.count("probe_signal") && comp.parameters.at("probe_signal") == "1") ||
                        (comp.parameters.count("plotI") && comp.parameters.at("plotI") == "1") ||
                        (comp.parameters.count("plotV") && comp.parameters.at("plotV") == "1");
        if (isProbed) {
            probedTargets.push_back(comp.id);
            if (comp.type == ComponentType::Inductor) {
                probedSignals.push_back("I_" + comp.id);
            } else {
                probedSignals.push_back("V_" + comp.id);
            }
        }
    }

    std::string sigStr = "";
    for (size_t i = 0; i < probedSignals.size(); ++i) {
        if (i > 0) sigStr += ",";
        sigStr += probedSignals[i];
    }
    std::string targetStr = "";
    for (size_t i = 0; i < probedTargets.size(); ++i) {
        if (i > 0) targetStr += ",";
        targetStr += probedTargets[i];
    }

    for (auto& comp : design.components) {
        if (comp.rawTypeStr == "PROBE") {
            comp.parameters["selected_signals"] = sigStr;
            comp.parameters["target"] = targetStr;
        }
    }
}

} // namespace CircuitSim
