#include "SVGExporter.hpp"
#include "SchematicCanvas.hpp"
#include <windows.h>
#include <commdlg.h>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <cmath>
#include <cfloat>

#include <unordered_map>
#include <unordered_set>
#include <functional>

namespace CircuitSim {

std::vector<std::string> SVGExporter::traceScopeInputSignals(const CircuitDesign& design, const std::string& scopeId, int numChannels) {
    std::vector<std::string> signalKeys(numChannels, "");

    // Build a wire lookup map for junction traversal
    std::unordered_map<std::string, const WireInstance*> wireMap;
    for (const auto& w : design.wires) {
        wireMap[w.id] = &w;
    }

    // Recursive helper: given a wire endpoint, follow junctions until we find a component pin.
    struct PinResult { std::string compId; std::string terminal; };
    
    std::function<PinResult(const WireEndpoint&, std::unordered_set<std::string>&)> resolveEndpoint;
    resolveEndpoint = [&](const WireEndpoint& ep, std::unordered_set<std::string>& visited) -> PinResult {
        if (!ep.compId.empty()) {
            return {ep.compId, ep.terminal};
        }
        if (ep.isWireJunction && !ep.targetWireId.empty()) {
            if (visited.count(ep.targetWireId)) return {"", ""};
            visited.insert(ep.targetWireId);
            auto it = wireMap.find(ep.targetWireId);
            if (it != wireMap.end()) {
                const WireInstance* targetWire = it->second;
                PinResult r = resolveEndpoint(targetWire->from, visited);
                if (!r.compId.empty()) return r;
                r = resolveEndpoint(targetWire->to, visited);
                if (!r.compId.empty()) return r;
            }
        }
        return {"", ""};
    };

    auto findSourceOnNet = [&](const std::string& startWireId, const std::string& excludeCompId) -> PinResult {
        std::unordered_set<std::string> visitedWires;
        std::vector<std::string> wireQueue;
        wireQueue.push_back(startWireId);
        visitedWires.insert(startWireId);

        std::vector<PinResult> foundPins;

        while (!wireQueue.empty()) {
            std::string wId = wireQueue.back();
            wireQueue.pop_back();

            auto it = wireMap.find(wId);
            if (it == wireMap.end()) continue;
            const WireInstance* w = it->second;

            for (const WireEndpoint* ep : {&w->from, &w->to}) {
                if (!ep->compId.empty()) {
                    if (ep->compId != excludeCompId) {
                        foundPins.push_back({ep->compId, ep->terminal});
                    }
                } else if (ep->isWireJunction && !ep->targetWireId.empty()) {
                    if (!visitedWires.count(ep->targetWireId)) {
                        visitedWires.insert(ep->targetWireId);
                        wireQueue.push_back(ep->targetWireId);
                    }
                }
            }

            for (const auto& pair : wireMap) {
                if (visitedWires.count(pair.first)) continue;
                const WireInstance* other = pair.second;
                if ((other->from.isWireJunction && other->from.targetWireId == wId) ||
                    (other->to.isWireJunction && other->to.targetWireId == wId)) {
                    visitedWires.insert(pair.first);
                    wireQueue.push_back(pair.first);
                }
            }
        }

        for (const auto& pin : foundPins) {
            if (pin.terminal.find("Out") != std::string::npos || 
                pin.terminal == "A" || pin.terminal == "B") {
                return pin;
            }
        }
        if (!foundPins.empty()) return foundPins[0];
        return {"", ""};
    };

    for (int ch = 0; ch < numChannels; ++ch) {
        std::string targetPin = "In" + std::to_string(ch + 1);

        for (const auto& wire : design.wires) {
            bool scopeOnTo = (wire.to.compId == scopeId && wire.to.terminal == targetPin);
            bool scopeOnFrom = (wire.from.compId == scopeId && wire.from.terminal == targetPin);

            if (!scopeOnTo && !scopeOnFrom) continue;

            const WireEndpoint& sourceEp = scopeOnTo ? wire.from : wire.to;

            PinResult source;
            if (!sourceEp.compId.empty()) {
                source = {sourceEp.compId, sourceEp.terminal};
            } else if (sourceEp.isWireJunction) {
                source = findSourceOnNet(wire.id, scopeId);
            }

            if (!source.compId.empty()) {
                std::string sigKey;
                for (const auto& comp : design.components) {
                    if (comp.id == source.compId) {
                        if (comp.rawTypeStr == "PROBE") {
                            if (comp.parameters.count("selected_signals") && !comp.parameters.at("selected_signals").empty()) {
                                sigKey = comp.parameters.at("selected_signals");
                            } else if (comp.parameters.count("target") && !comp.parameters.at("target").empty()) {
                                std::string pType = comp.parameters.count("probe_type") ? comp.parameters.at("probe_type") : "Voltage";
                                if (pType == "Current" || pType == "I") sigKey = "I_" + comp.parameters.at("target");
                                else sigKey = "V_" + comp.parameters.at("target");
                            } else {
                                sigKey = comp.id + ".Out";
                            }
                        } else if (comp.type == ComponentType::Voltmeter) {
                            sigKey = comp.id;
                        } else if (comp.type == ComponentType::Ammeter) {
                            sigKey = comp.id;
                        } else if (comp.type == ComponentType::Resistor || 
                                   comp.type == ComponentType::Capacitor ||
                                   comp.type == ComponentType::Inductor ||
                                   comp.type == ComponentType::VoltageSource ||
                                   comp.type == ComponentType::ACVoltageSource ||
                                   comp.type == ComponentType::Diode ||
                                   comp.type == ComponentType::Switch ||
                                   comp.type == ComponentType::MOSFET) {
                            if (source.terminal.find("I") != std::string::npos || source.terminal == "AM") {
                                sigKey = "I_" + comp.id;
                            } else {
                                sigKey = "V_" + comp.id;
                            }
                        } else {
                            if (source.terminal.find("Out") != std::string::npos) {
                                sigKey = comp.id + "." + source.terminal;
                            } else {
                                sigKey = comp.id + ".Out";
                            }
                        }
                        break;
                    }
                }

                if (!sigKey.empty()) {
                    signalKeys[ch] = sigKey;
                }
                break;
            }
        }
    }

    return signalKeys;
}

static ImVec2 svgRotatePt(float px, float py, float cx, float cy, float angleDeg) {
    if (angleDeg == 0.0f) return ImVec2(cx + px, cy + py);
    float rad = angleDeg * 3.1415926535f / 180.0f;
    float cosA = std::cos(rad);
    float sinA = std::sin(rad);
    float rx = px * cosA - py * sinA;
    float ry = px * sinA + py * cosA;
    return ImVec2(cx + rx, cy + ry);
}

static std::string xmlEscape(const std::string& str) {
    std::string out;
    out.reserve(str.size());
    for (char c : str) {
        switch (c) {
            case '&':  out += "&amp;"; break;
            case '<':  out += "&lt;"; break;
            case '>':  out += "&gt;"; break;
            case '"':  out += "&quot;"; break;
            case '\'': out += "&apos;"; break;
            default:   out += c; break;
        }
    }
    return out;
}

std::string SVGExporter::saveSVGFileDialog(const std::string& title, const std::string& defaultName) {
    char szFile[260] = {0};
    strncpy_s(szFile, defaultName.c_str(), sizeof(szFile) - 1);
    OPENFILENAMEA ofn = {0};
    ofn.lStructSize = sizeof(ofn);
    ofn.lpstrFilter = "SVG Vector Graphic (*.svg)\0*.svg\0All Files (*.*)\0*.*\0";
    ofn.lpstrFile = szFile;
    ofn.nMaxFile = sizeof(szFile);
    ofn.lpstrTitle = title.c_str();
    ofn.Flags = OFN_PATHMUSTEXIST | OFN_OVERWRITEPROMPT;
    if (GetSaveFileNameA(&ofn)) {
        std::string res(szFile);
        if (res.find(".svg") == std::string::npos && res.find(".SVG") == std::string::npos) {
            res += ".svg";
        }
        return res;
    }
    return "";
}

std::string SVGExporter::saveHTMLFileDialog(const std::string& title, const std::string& defaultName) {
    char szFile[260] = {0};
    strncpy_s(szFile, defaultName.c_str(), sizeof(szFile) - 1);
    OPENFILENAMEA ofn = {0};
    ofn.lStructSize = sizeof(ofn);
    ofn.lpstrFilter = "HTML Report (*.html)\0*.html\0All Files (*.*)\0*.*\0";
    ofn.lpstrFile = szFile;
    ofn.nMaxFile = sizeof(szFile);
    ofn.lpstrTitle = title.c_str();
    ofn.Flags = OFN_PATHMUSTEXIST | OFN_OVERWRITEPROMPT;
    if (GetSaveFileNameA(&ofn)) {
        std::string res(szFile);
        if (res.find(".html") == std::string::npos && res.find(".HTML") == std::string::npos) {
            res += ".html";
        }
        return res;
    }
    return "";
}

static bool getTerminalPortStubSVG(const ComponentInstance& comp, const std::string& terminalName, ImVec2& outPinPos, ImVec2& outStubPos, bool& outIsVertical) {
    auto terminals = getTerminals(comp);
    if (terminals.empty() && !comp.pins.empty()) {
        for (const auto& pin : comp.pins) {
            TerminalDef td;
            td.name = pin.name;
            td.x = pin.relativeX;
            td.y = pin.relativeY;
            td.dx = pin.isOutput ? 1.0f : -1.0f;
            td.dy = 0;
            td.isControl = pin.isCtrl || pin.isInput || pin.isOutput;
            terminals.push_back(td);
        }
    }

    ImVec2 compCenter(comp.x, comp.y);
    for (const auto& t : terminals) {
        if (t.name == terminalName) {
            outPinPos = svgRotatePt(t.x, t.y, compCenter.x, compCenter.y, (float)comp.rotation);
            ImVec2 dir = svgRotatePt(t.dx * 20.0f, t.dy * 20.0f, 0, 0, (float)comp.rotation);
            outStubPos = ImVec2(outPinPos.x + dir.x, outPinPos.y + dir.y);
            ImVec2 rotatedDir = svgRotatePt(t.dx, t.dy, 0, 0, (float)comp.rotation);
            outIsVertical = (std::abs(rotatedDir.y) > std::abs(rotatedDir.x));
            return true;
        }
    }
    if (!terminals.empty()) {
        const auto& t = terminals[0];
        outPinPos = svgRotatePt(t.x, t.y, compCenter.x, compCenter.y, (float)comp.rotation);
        ImVec2 dir = svgRotatePt(t.dx * 20.0f, t.dy * 20.0f, 0, 0, (float)comp.rotation);
        outStubPos = ImVec2(outPinPos.x + dir.x, outPinPos.y + dir.y);
        ImVec2 rotatedDir = svgRotatePt(t.dx, t.dy, 0, 0, (float)comp.rotation);
        outIsVertical = (std::abs(rotatedDir.y) > std::abs(rotatedDir.x));
        return true;
    }
    outPinPos = compCenter;
    outStubPos = compCenter;
    outIsVertical = false;
    return true;
}

// ============================================================================
// PART 1: SCHEMATIC CANVAS LIGHT-MODE SVG EXPORT (100% MATCHING WORKSPACE)
// ============================================================================

bool SVGExporter::exportSchematicToSVGString(const CircuitDesign& design, std::string& outSVG, bool /*isDarkMode*/) {
    std::ostringstream out;

    // 1. Calculate World Bounding Box
    float minX = 1e9f, maxX = -1e9f, minY = 1e9f, maxY = -1e9f;
    for (const auto& c : design.components) {
        minX = std::min(minX, c.x - 60.0f);
        maxX = std::max(maxX, c.x + 60.0f);
        minY = std::min(minY, c.y - 60.0f);
        maxY = std::max(maxY, c.y + 60.0f);
    }
    for (const auto& w : design.wires) {
        for (const auto& pt : w.manualPath) {
            minX = std::min(minX, pt.x - 20.0f);
            maxX = std::max(maxX, pt.x + 20.0f);
            minY = std::min(minY, pt.y - 20.0f);
            maxY = std::max(maxY, pt.y + 20.0f);
        }
        if (w.to.isWireJunction) {
            minX = std::min(minX, w.to.junctionX - 20.0f);
            maxX = std::max(maxX, w.to.junctionX + 20.0f);
            minY = std::min(minY, w.to.junctionY - 20.0f);
            maxY = std::max(maxY, w.to.junctionY + 20.0f);
        }
    }

    if (design.components.empty()) {
        minX = -300.0f; maxX = 300.0f;
        minY = -200.0f; maxY = 200.0f;
    }

    float margin = 50.0f;
    minX -= margin; maxX += margin;
    minY -= margin; maxY += margin;

    float viewW = maxX - minX;
    float viewH = maxY - minY;

    // Pure Light Mode palette for publication & paper insertion
    std::string bgCol = "#ffffff";
    std::string gridCol = "#f1f5f9";
    std::string wireCol = "#0284c7";      // Dark blue power wire
    std::string ctrlWireCol = "#0284c7";  // Control wire color matching signal wires
    std::string bodyStroke = "#0f172a";   // Dark slate/black component stroke
    std::string bodyFill = "#ffffff";     // Clean white component background
    std::string compLabelCol = "#0f172a"; // Component ID text
    std::string paramLabelCol = "#0284c7";// Parameter value text
    std::string pinDotCol = "#059669";    // Green terminal dot fill
    std::string ctrlPinDotCol = "#0284c7";// Cyan control dot fill

    out << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
    out << "<svg xmlns=\"http://www.w3.org/2000/svg\" ";
    out << "viewBox=\"" << minX << " " << minY << " " << viewW << " " << viewH << "\" ";
    out << "width=\"" << std::max(800.0f, viewW) << "\" height=\"" << std::max(600.0f, viewH) << "\">\n";
    out << "<style>\n";
    out << "  .grid { stroke: " << gridCol << "; stroke-width: 0.5; stroke-dasharray: 2,4; }\n";
    out << "  .wire { stroke: " << wireCol << "; stroke-width: 2.2; fill: none; stroke-linecap: round; stroke-linejoin: round; }\n";
    out << "  .control-wire { stroke: " << ctrlWireCol << "; stroke-width: 2.2; fill: none; stroke-linecap: round; stroke-linejoin: round; }\n";
    out << "  .comp-body { stroke: " << bodyStroke << "; stroke-width: 2.0; fill: " << bodyFill << "; stroke-linecap: round; stroke-linejoin: round; }\n";
    out << "  .comp-label { font-family: system-ui, -apple-system, sans-serif; font-size: 11px; font-weight: bold; fill: " << compLabelCol << "; text-anchor: middle; }\n";
    out << "  .param-label { font-family: system-ui, -apple-system, sans-serif; font-size: 10px; font-weight: 500; fill: " << paramLabelCol << "; text-anchor: middle; }\n";
    out << "  .pin-dot { fill: " << pinDotCol << "; }\n";
    out << "  .ctrl-dot { fill: " << ctrlPinDotCol << "; }\n";
    out << "</style>\n";

    // Pure White Background
    out << "<rect x=\"" << minX << "\" y=\"" << minY << "\" width=\"" << viewW << "\" height=\"" << viewH << "\" fill=\"" << bgCol << "\"/>\n";

    // Light Mode Grid
    out << "<g class=\"grid\">\n";
    float startX = std::floor(minX / 20.0f) * 20.0f;
    float startY = std::floor(minY / 20.0f) * 20.0f;
    for (float x = startX; x <= maxX; x += 40.0f) {
        out << "  <line x1=\"" << x << "\" y1=\"" << minY << "\" x2=\"" << x << "\" y2=\"" << maxY << "\"/>\n";
    }
    for (float y = startY; y <= maxY; y += 40.0f) {
        out << "  <line x1=\"" << minX << "\" y1=\"" << y << "\" x2=\"" << maxX << "\" y2=\"" << y << "\"/>\n";
    }
    out << "</g>\n";

    // Map of components for fast lookup
    std::map<std::string, const ComponentInstance*> compMap;
    for (const auto& c : design.components) compMap[c.id] = &c;

    // Helper functions for SVG primitive rendering
    auto drawLine = [&](const ImVec2& a, const ImVec2& b, float width = 2.0f) {
        out << "    <line x1=\"" << a.x << "\" y1=\"" << a.y << "\" x2=\"" << b.x << "\" y2=\"" << b.y << "\" class=\"comp-body\" stroke-width=\"" << width << "\"/>\n";
    };

    auto drawPolyline = [&](const ImVec2* pts, int count, float width = 2.0f, bool fill = false) {
        out << "    <polyline points=\"";
        for (int i = 0; i < count; ++i) {
            if (i > 0) out << " ";
            out << pts[i].x << "," << pts[i].y;
        }
        out << "\" class=\"comp-body\" stroke-width=\"" << width << "\" fill=\"" << (fill ? bodyFill : "none") << "\"/>\n";
    };

    auto drawPolygon = [&](const ImVec2* pts, int count, float width = 2.0f, bool fill = true) {
        out << "    <polygon points=\"";
        for (int i = 0; i < count; ++i) {
            if (i > 0) out << " ";
            out << pts[i].x << "," << pts[i].y;
        }
        out << "\" class=\"comp-body\" stroke-width=\"" << width << "\" fill=\"" << (fill ? bodyFill : "none") << "\"/>\n";
    };

    auto drawCircle = [&](const ImVec2& center, float r, float width = 2.0f, bool fill = false) {
        out << "    <circle cx=\"" << center.x << "\" cy=\"" << center.y << "\" r=\"" << r << "\" class=\"comp-body\" stroke-width=\"" << width << "\" fill=\"" << (fill ? bodyFill : "none") << "\"/>\n";
    };

    auto drawBezierCubic = [&](const ImVec2& p0, const ImVec2& p1, const ImVec2& p2, const ImVec2& p3, float width = 2.0f) {
        out << "    <path d=\"M " << p0.x << "," << p0.y << " C " << p1.x << "," << p1.y << " " << p2.x << "," << p2.y << " " << p3.x << "," << p3.y << "\" class=\"comp-body\" stroke-width=\"" << width << "\" fill=\"none\"/>\n";
    };

    // 2. Draw Wires with EXACT STUB & OBSTACLE AVOIDANCE ROUTING MATCHING CANVAS
    out << "<g id=\"wires\">\n";
    for (const auto& w : design.wires) {
        ImVec2 p1(0, 0), p1_stub(0, 0), p2(0, 0), p2_stub(0, 0);
        bool foundFrom = false, foundTo = false;
        bool fromIsVertical = false;

        if (w.from.isWireJunction || (!w.from.compId.empty() && compMap.find(w.from.compId) == compMap.end())) {
            p1 = ImVec2(w.from.junctionX, w.from.junctionY);
            p1_stub = p1;
            foundFrom = true;
        } else {
            auto itC = compMap.find(w.from.compId);
            if (itC != compMap.end()) {
                foundFrom = getTerminalPortStubSVG(*(itC->second), w.from.terminal, p1, p1_stub, fromIsVertical);
            }
        }

        if (w.to.isWireJunction || (!w.to.compId.empty() && compMap.find(w.to.compId) == compMap.end())) {
            p2 = ImVec2(w.to.junctionX, w.to.junctionY);
            p2_stub = p2;
            foundTo = true;
        } else {
            auto itC = compMap.find(w.to.compId);
            if (itC != compMap.end()) {
                bool dummyVert;
                foundTo = getTerminalPortStubSVG(*(itC->second), w.to.terminal, p2, p2_stub, dummyVert);
            }
        }

        if (!foundFrom) p1_stub = p1;
        if (!foundTo) p2_stub = p2;

        ImVec2 c1(0, 0), c2(0, 0);
        if (!w.manualPath.empty()) {
            c1 = ImVec2(w.manualPath.front().x, w.manualPath.front().y);
            c2 = ImVec2(w.manualPath.back().x, w.manualPath.back().y);
        } else {
            const ComponentInstance* fromComp = compMap.count(w.from.compId) ? compMap[w.from.compId] : nullptr;
            if (fromIsVertical) {
                c1 = ImVec2(p1_stub.x, p2_stub.y);
                c2 = c1;
                if (fromComp) {
                    float hw = 25.0f, hh = 25.0f;
                    bool cutsThroughY = ((p1_stub.y - fromComp->y) * (p2_stub.y - fromComp->y) < 0) || (std::abs(p2_stub.y - fromComp->y) <= hh);
                    bool alignsX = (std::abs(p1_stub.x - fromComp->x) <= hw + 10.0f);
                    if (cutsThroughY && alignsX) {
                        float detourX = (p2_stub.x >= fromComp->x) ? (fromComp->x + hw + 25.0f) : (fromComp->x - hw - 25.0f);
                        c1 = ImVec2(detourX, p1_stub.y);
                        c2 = ImVec2(detourX, p2_stub.y);
                    }
                }
            } else {
                c1 = ImVec2(p2_stub.x, p1_stub.y);
                c2 = c1;
                if (fromComp) {
                    float hw = 25.0f, hh = 25.0f;
                    bool cutsThroughX = ((p1_stub.x - fromComp->x) * (p2_stub.x - fromComp->x) < 0) || (std::abs(p2_stub.x - fromComp->x) <= hw);
                    bool alignsY = (std::abs(p1_stub.y - fromComp->y) <= hh + 10.0f);
                    if (cutsThroughX && alignsY) {
                        float detourY = (p2_stub.y >= fromComp->y) ? (fromComp->y + hh + 25.0f) : (fromComp->y - hh - 25.0f);
                        c1 = ImVec2(p1_stub.x, detourY);
                        c2 = ImVec2(p2_stub.x, detourY);
                    }
                }
            }
        }

        bool isControl = (!w.from.terminal.empty() && (w.from.terminal == "G" || w.from.terminal == "Out" || w.from.terminal == "In"));
        std::string cls = isControl ? "control-wire" : "wire";

        if (!w.manualPath.empty()) {
            out << "  <polyline class=\"" << cls << "\" points=\"" << p1.x << "," << p1.y << " " << p1_stub.x << "," << p1_stub.y;
            for (const auto& pt : w.manualPath) {
                out << " " << pt.x << "," << pt.y;
            }
            out << " " << p2_stub.x << "," << p2_stub.y << " " << p2.x << "," << p2.y << "\"/>\n";
        } else {
            out << "  <polyline class=\"" << cls << "\" points=\""
                << p1.x << "," << p1.y << " "
                << p1_stub.x << "," << p1_stub.y << " "
                << c1.x << "," << c1.y << " "
                << c2.x << "," << c2.y << " "
                << p2_stub.x << "," << p2_stub.y << " "
                << p2.x << "," << p2.y << "\"/>\n";
        }

        // Control wire filled arrowhead pointing into target port
        if (isControl && !w.to.isWireJunction) {
            float arrLen = 7.0f;
            float arrWidth = 4.0f;
            ImVec2 dir(p2.x - p2_stub.x, p2.y - p2_stub.y);
            float len = std::sqrt(dir.x * dir.x + dir.y * dir.y);
            if (len > 1e-4f) { dir.x /= len; dir.y /= len; } else { dir = ImVec2(1.0f, 0.0f); }
            ImVec2 basePt(p2.x - dir.x * arrLen, p2.y - dir.y * arrLen);
            ImVec2 arr1(basePt.x + dir.y * arrWidth, basePt.y - dir.x * arrWidth);
            ImVec2 arr2(basePt.x - dir.y * arrWidth, basePt.y + dir.x * arrWidth);
            out << "  <polygon points=\"" << p2.x << "," << p2.y << " " << arr1.x << "," << arr1.y << " " << arr2.x << "," << arr2.y << "\" fill=\"" << ctrlWireCol << "\"/>\n";
        }

        // Junction connection dots
        if (w.to.isWireJunction) {
            out << "  <circle cx=\"" << w.to.junctionX << "\" cy=\"" << w.to.junctionY << "\" r=\"4.0\" class=\"" << (isControl ? "ctrl-dot" : "pin-dot") << "\"/>\n";
        }
        if (w.from.isWireJunction) {
            out << "  <circle cx=\"" << w.from.junctionX << "\" cy=\"" << w.from.junctionY << "\" r=\"4.0\" class=\"" << (isControl ? "ctrl-dot" : "pin-dot") << "\"/>\n";
        }
    }
    out << "</g>\n";

    // 3. Draw Components with 100% EXACT GEOMETRIC ACCURACY
    out << "<g id=\"components\">\n";
    for (const auto& comp : design.components) {
        ImVec2 c(comp.x, comp.y);
        float rot = (float)comp.rotation;
        const std::string& t = comp.rawTypeStr;

        out << "  <g id=\"" << xmlEscape(comp.id) << "\">\n";

        if (t == "R" || t == "VAR_R" || t == "PWL_R") {
            ImVec2 rawPts[] = {
                {0, -40}, {0, -20},
                {-10, -15}, {10, -9},
                {-10, -3},  {10, 3},
                {-10, 9},   {10, 15},
                {0, 20}, {0, 40}
            };
            ImVec2 pts[10];
            for (int i = 0; i < 10; ++i) pts[i] = svgRotatePt(rawPts[i].x, rawPts[i].y, c.x, c.y, rot);
            drawPolyline(pts, 10);
        } else if (t == "L" || t == "VAR_L" || t == "SAT_L") {
            ImVec2 p1 = svgRotatePt(0, -40, c.x, c.y, rot);
            ImVec2 p2 = svgRotatePt(0, -20, c.x, c.y, rot);
            drawLine(p1, p2);
            for (int i = 0; i < 3; ++i) {
                float cy = -13.3f + i * 13.3f;
                ImVec2 c0 = svgRotatePt(0, cy - 6.7f, c.x, c.y, rot);
                ImVec2 c1 = svgRotatePt(-14, cy - 6.7f, c.x, c.y, rot);
                ImVec2 c2 = svgRotatePt(-14, cy + 6.7f, c.x, c.y, rot);
                ImVec2 c3 = svgRotatePt(0, cy + 6.7f, c.x, c.y, rot);
                drawBezierCubic(c0, c1, c2, c3);
            }
            ImVec2 p3 = svgRotatePt(0, 20, c.x, c.y, rot);
            ImVec2 p4 = svgRotatePt(0, 40, c.x, c.y, rot);
            drawLine(p3, p4);
        } else if (t == "C" || t == "VAR_C" || t == "SAT_C") {
            drawLine(svgRotatePt(0, -40, c.x, c.y, rot), svgRotatePt(0, -5, c.x, c.y, rot));
            drawLine(svgRotatePt(-15, -5, c.x, c.y, rot), svgRotatePt(15, -5, c.x, c.y, rot));
            drawLine(svgRotatePt(-15, 5, c.x, c.y, rot), svgRotatePt(15, 5, c.x, c.y, rot));
            drawLine(svgRotatePt(0, 5, c.x, c.y, rot), svgRotatePt(0, 40, c.x, c.y, rot));
        } else if (t == "S") {
            drawLine(svgRotatePt(0, -40, c.x, c.y, rot), svgRotatePt(0, -20, c.x, c.y, rot));
            drawCircle(svgRotatePt(0, -20, c.x, c.y, rot), 3.0f, 1.5f, true);
            drawCircle(svgRotatePt(0, 20, c.x, c.y, rot), 3.0f, 1.5f, true);
            drawLine(svgRotatePt(0, -20, c.x, c.y, rot), svgRotatePt(13, 16, c.x, c.y, rot));
            drawLine(svgRotatePt(0, 20, c.x, c.y, rot), svgRotatePt(0, 40, c.x, c.y, rot));
            drawLine(svgRotatePt(-20, 0, c.x, c.y, rot), svgRotatePt(-6, 0, c.x, c.y, rot));
        } else if (t == "D" || t == "DIODE") {
            drawLine(svgRotatePt(0, -40, c.x, c.y, rot), svgRotatePt(0, -8, c.x, c.y, rot));
            ImVec2 tri[] = {svgRotatePt(-12, -8, c.x, c.y, rot), svgRotatePt(12, -8, c.x, c.y, rot), svgRotatePt(0, 8, c.x, c.y, rot)};
            drawPolygon(tri, 3, 2.0f, true);
            drawLine(svgRotatePt(-12, 8, c.x, c.y, rot), svgRotatePt(12, 8, c.x, c.y, rot));
            drawLine(svgRotatePt(0, 8, c.x, c.y, rot), svgRotatePt(0, 40, c.x, c.y, rot));
        } else if (t == "THYRISTOR" || t == "SCR" || t == "GTO" || t == "IGCT") {
            drawLine(svgRotatePt(0, -20, c.x, c.y, rot), svgRotatePt(0, -8, c.x, c.y, rot));
            ImVec2 tri[] = {svgRotatePt(-12, -8, c.x, c.y, rot), svgRotatePt(12, -8, c.x, c.y, rot), svgRotatePt(0, 8, c.x, c.y, rot)};
            drawPolygon(tri, 3, 2.0f, true);
            drawLine(svgRotatePt(-12, 8, c.x, c.y, rot), svgRotatePt(12, 8, c.x, c.y, rot));
            drawLine(svgRotatePt(0, 8, c.x, c.y, rot), svgRotatePt(0, 20, c.x, c.y, rot));
            drawLine(svgRotatePt(-20, 10, c.x, c.y, rot), svgRotatePt(-10, 10, c.x, c.y, rot));
            drawLine(svgRotatePt(-10, 10, c.x, c.y, rot), svgRotatePt(-5, 8, c.x, c.y, rot));
        } else if (t == "MOSFET" || t == "vg-FET" || t == "VGFET") {
            drawLine(svgRotatePt(0, -40, c.x, c.y, rot), svgRotatePt(0, -15, c.x, c.y, rot));
            drawLine(svgRotatePt(0, 15, c.x, c.y, rot), svgRotatePt(0, 40, c.x, c.y, rot));
            drawLine(svgRotatePt(-5, -15, c.x, c.y, rot), svgRotatePt(-5, 15, c.x, c.y, rot));
            drawLine(svgRotatePt(-5, 0, c.x, c.y, rot), svgRotatePt(0, 0, c.x, c.y, rot));
            drawLine(svgRotatePt(-10, -15, c.x, c.y, rot), svgRotatePt(-10, 15, c.x, c.y, rot));
            drawLine(svgRotatePt(-20, 0, c.x, c.y, rot), svgRotatePt(-10, 0, c.x, c.y, rot));
            drawLine(svgRotatePt(0, 15, c.x, c.y, rot), svgRotatePt(12, 15, c.x, c.y, rot));
            drawLine(svgRotatePt(12, 15, c.x, c.y, rot), svgRotatePt(12, 6, c.x, c.y, rot));
            ImVec2 dTri[] = {svgRotatePt(7, 6, c.x, c.y, rot), svgRotatePt(17, 6, c.x, c.y, rot), svgRotatePt(12, -6, c.x, c.y, rot)};
            drawPolygon(dTri, 3, 1.5f, true);
            drawLine(svgRotatePt(7, -6, c.x, c.y, rot), svgRotatePt(17, -6, c.x, c.y, rot));
            drawLine(svgRotatePt(12, -6, c.x, c.y, rot), svgRotatePt(12, -15, c.x, c.y, rot));
            drawLine(svgRotatePt(12, -15, c.x, c.y, rot), svgRotatePt(0, -15, c.x, c.y, rot));
        } else if (t == "V" || t == "DC_V" || t == "DC_V1" || t == "VoltageSource") {
            drawLine(svgRotatePt(0, -40, c.x, c.y, rot), svgRotatePt(0, -16, c.x, c.y, rot));
            drawLine(svgRotatePt(0, 16, c.x, c.y, rot), svgRotatePt(0, 40, c.x, c.y, rot));
            drawCircle(c, 16.0f, 2.0f, true);
            drawLine(svgRotatePt(-3, -7, c.x, c.y, rot), svgRotatePt(3, -7, c.x, c.y, rot));
            drawLine(svgRotatePt(0, -10, c.x, c.y, rot), svgRotatePt(0, -4, c.x, c.y, rot));
            drawLine(svgRotatePt(-3, 7, c.x, c.y, rot), svgRotatePt(3, 7, c.x, c.y, rot));
        } else if (t == "I" || t == "DC_I" || t == "DC_I1" || t == "CurrentSource") {
            drawLine(svgRotatePt(0, -40, c.x, c.y, rot), svgRotatePt(0, -16, c.x, c.y, rot));
            drawLine(svgRotatePt(0, 16, c.x, c.y, rot), svgRotatePt(0, 40, c.x, c.y, rot));
            drawCircle(c, 16.0f, 2.0f, true);
            drawLine(svgRotatePt(0, -9, c.x, c.y, rot), svgRotatePt(0, 9, c.x, c.y, rot));
            ImVec2 arr[] = {svgRotatePt(-4, 3, c.x, c.y, rot), svgRotatePt(0, 9, c.x, c.y, rot), svgRotatePt(4, 3, c.x, c.y, rot)};
            drawPolyline(arr, 3);
        } else if (t == "AC_V") {
            drawLine(svgRotatePt(0, -40, c.x, c.y, rot), svgRotatePt(0, -16, c.x, c.y, rot));
            drawLine(svgRotatePt(0, 16, c.x, c.y, rot), svgRotatePt(0, 40, c.x, c.y, rot));
            drawCircle(c, 16.0f, 2.0f, true);
            drawBezierCubic(svgRotatePt(-8, 0, c.x, c.y, rot), svgRotatePt(-4, -8, c.x, c.y, rot), svgRotatePt(0, -8, c.x, c.y, rot), svgRotatePt(0, 0, c.x, c.y, rot));
            drawBezierCubic(svgRotatePt(0, 0, c.x, c.y, rot), svgRotatePt(0, 8, c.x, c.y, rot), svgRotatePt(4, 8, c.x, c.y, rot), svgRotatePt(8, 0, c.x, c.y, rot));
        } else if (t == "VM" || t == "AM") {
            drawLine(svgRotatePt(0, -40, c.x, c.y, rot), svgRotatePt(0, -16, c.x, c.y, rot));
            drawLine(svgRotatePt(0, 16, c.x, c.y, rot), svgRotatePt(0, 40, c.x, c.y, rot));
            drawLine(svgRotatePt(16, 0, c.x, c.y, rot), svgRotatePt(20, 0, c.x, c.y, rot));
            drawCircle(c, 16.0f, 2.0f, true);
            ImVec2 txtPos = svgRotatePt(-4, -7, c.x, c.y, rot);
            out << "    <text x=\"" << txtPos.x << "\" y=\"" << txtPos.y + 11 << "\" class=\"comp-label\">" << (t == "VM" ? "V" : "A") << "</text>\n";
        } else if (t == "GND") {
            drawLine(svgRotatePt(0, -20, c.x, c.y, rot), svgRotatePt(0, 0, c.x, c.y, rot));
            drawLine(svgRotatePt(-12, 0, c.x, c.y, rot), svgRotatePt(12, 0, c.x, c.y, rot));
            drawLine(svgRotatePt(-8, 6, c.x, c.y, rot), svgRotatePt(8, 6, c.x, c.y, rot));
            drawLine(svgRotatePt(-4, 12, c.x, c.y, rot), svgRotatePt(4, 12, c.x, c.y, rot));
        } else if (t == "PULSE" || t == "PULSE_GEN") {
            float hw = 22.0f, hh = 16.0f;
            ImVec2 rectPts[] = {
                svgRotatePt(-hw, -hh, c.x, c.y, rot),
                svgRotatePt(hw, -hh, c.x, c.y, rot),
                svgRotatePt(hw, hh, c.x, c.y, rot),
                svgRotatePt(-hw, hh, c.x, c.y, rot)
            };
            drawPolygon(rectPts, 4, 2.0f, true);
            ImVec2 pulseW[] = {
                svgRotatePt(-12, 6, c.x, c.y, rot),
                svgRotatePt(-12, -6, c.x, c.y, rot),
                svgRotatePt(0, -6, c.x, c.y, rot),
                svgRotatePt(0, 6, c.x, c.y, rot),
                svgRotatePt(12, 6, c.x, c.y, rot)
            };
            drawPolyline(pulseW, 5, 1.8f);
            drawLine(svgRotatePt(hw, 0, c.x, c.y, rot), svgRotatePt(hw + 4, 0, c.x, c.y, rot));
        } else if (t == "SUM_ROUND" || t == "SUM" || t == "SUM_RECT" || t == "PRODUCT_RECT" || t == "PROD") {
            float hw = 25.0f, hh = 20.0f;
            ImVec2 rectPts[] = {
                svgRotatePt(-hw, -hh, c.x, c.y, rot),
                svgRotatePt(hw, -hh, c.x, c.y, rot),
                svgRotatePt(hw, hh, c.x, c.y, rot),
                svgRotatePt(-hw, hh, c.x, c.y, rot)
            };
            drawPolygon(rectPts, 4, 2.0f, true);
            ImVec2 symPos = svgRotatePt(0, 4, c.x, c.y, rot);
            std::string sym = (t == "PROD" || t == "PRODUCT_RECT") ? "Π" : "Σ";
            out << "    <text x=\"" << symPos.x << "\" y=\"" << symPos.y << "\" class=\"comp-label\">" << sym << "</text>\n";
        } else {
            // Generic Block Box with rotated geometry
            float hw = 24.0f, hh = 18.0f;
            ImVec2 rectPts[] = {
                svgRotatePt(-hw, -hh, c.x, c.y, rot),
                svgRotatePt(hw, -hh, c.x, c.y, rot),
                svgRotatePt(hw, hh, c.x, c.y, rot),
                svgRotatePt(-hw, hh, c.x, c.y, rot)
            };
            drawPolygon(rectPts, 4, 2.0f, true);
            ImVec2 lblPos = svgRotatePt(0, 4, c.x, c.y, rot);
            out << "    <text x=\"" << lblPos.x << "\" y=\"" << lblPos.y << "\" class=\"comp-label\">" << xmlEscape(t) << "</text>\n";
        }

        // Draw Component ID (above/beside) & Parameter Label (below/beside)
        float textYOff = 32.0f;
        if (rot == 90.0f || rot == 270.0f) textYOff = 22.0f; // Prevent label overlap on horizontal components

        ImVec2 idPos = svgRotatePt(0, -textYOff, c.x, c.y, 0.0f); // Keep text unrotated so it's readable
        out << "    <text x=\"" << idPos.x << "\" y=\"" << idPos.y << "\" class=\"comp-label\">" << xmlEscape(comp.id) << "</text>\n";

        std::string valStr;
        if (comp.parameters.count("value")) valStr = comp.parameters.at("value");
        else if (comp.parameters.count("C")) valStr = comp.parameters.at("C");
        else if (comp.parameters.count("L")) valStr = comp.parameters.at("L");
        else if (comp.parameters.count("R")) valStr = comp.parameters.at("R");

        if (!valStr.empty()) {
            ImVec2 valPos = svgRotatePt(0, textYOff + 6.0f, c.x, c.y, 0.0f);
            out << "    <text x=\"" << valPos.x << "\" y=\"" << valPos.y << "\" class=\"param-label\">" << xmlEscape(valStr) << "</text>\n";
        }

        // Draw pin terminal dots only at terminal connection endpoints
        auto terms = getTerminals(comp);
        for (const auto& term : terms) {
            ImVec2 pPos = svgRotatePt(term.x, term.y, c.x, c.y, rot);
            bool isControl = (term.name == "G" || term.name == "Out" || term.name == "In" || term.name == "Ctrl");
            out << "    <circle cx=\"" << pPos.x << "\" cy=\"" << pPos.y << "\" r=\"2.5\" class=\"" << (isControl ? "ctrl-dot" : "pin-dot") << "\"/>\n";
        }

        out << "  </g>\n";
    }
    out << "</g>\n";

    out << "</svg>\n";
    outSVG = out.str();
    return true;
}

bool SVGExporter::exportSchematicToSVG(const CircuitDesign& design, const std::string& filename, bool isDarkMode) {
    std::string svgStr;
    if (!exportSchematicToSVGString(design, svgStr, isDarkMode)) return false;
    std::ofstream out(filename);
    if (!out.is_open()) return false;
    out << svgStr;
    return true;
}

// ============================================================================
// PART 2: IEEE / ACADEMIC PUBLICATION GRADE WAVEFORM EXPORT
// ============================================================================

bool SVGExporter::exportScopeToSVGString(
    const CircuitSimEngine::TelemetryData& telemetry,
    const std::vector<std::string>& signalKeys,
    const std::vector<std::string>& labels,
    const std::string& scopeTitle,
    std::string& outSVG,
    int numPanes,
    bool /*isDarkMode*/,
    double timeMin, double timeMax)
{
    std::ostringstream out;

    const auto& timeData = telemetry.timeHistory;
    if (timeData.empty()) return false;

    double tStart = (timeMin >= 0.0) ? timeMin : timeData.front();
    double tEnd = (timeMax > tStart) ? timeMax : timeData.back();
    if (tEnd <= tStart) tEnd = tStart + 1.0;

    int totalChannels = (int)signalKeys.size();
    if (totalChannels == 0) return false;

    int renderPanes = std::clamp(numPanes, 1, totalChannels);

    // High-Contrast IEEE / Publication Grade Palette
    static const char* IEEE_PALETTE[] = {
        "#004488", // IEEE Deep Blue
        "#990000", // IEEE Crimson Red
        "#117733", // IEEE Forest Green
        "#cc6600", // IEEE Amber Gold
        "#662288", // IEEE Deep Purple
        "#008899", // IEEE Dark Teal
        "#44aa99", // IEEE Seafoam
        "#888888"  // IEEE Neutral Slate
    };

    float svgWidth = 1000.0f;
    float paneHeight = 280.0f;
    float headerHeight = 60.0f;
    float footerHeight = 45.0f;
    float totalHeight = headerHeight + renderPanes * paneHeight + footerHeight;

    std::string bgCol = "#ffffff";
    std::string cardBg = "#ffffff";
    std::string textCol = "#000000";
    std::string gridCol = "#e2e8f0";
    std::string borderCol = "#0f172a";

    out << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
    out << "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"" << svgWidth << "\" height=\"" << totalHeight << "\" viewBox=\"0 0 " << svgWidth << " " << totalHeight << "\">\n";
    out << "<style>\n";
    out << "  .bg { fill: " << bgCol << "; }\n";
    out << "  .title { font-family: system-ui, -apple-system, 'Segoe UI', Helvetica, Arial, sans-serif; font-size: 18px; font-weight: bold; fill: " << textCol << "; text-anchor: middle; }\n";
    out << "  .subtitle { font-family: system-ui, -apple-system, sans-serif; font-size: 11px; fill: #475569; text-anchor: middle; }\n";
    out << "  .pane-bg { fill: " << cardBg << "; stroke: " << borderCol << "; stroke-width: 1.2; rx: 2px; }\n";
    out << "  .grid-line { stroke: " << gridCol << "; stroke-width: 0.8; stroke-dasharray: 3,3; }\n";
    out << "  .axis-text { font-family: 'Helvetica Neue', Helvetica, Arial, monospace, sans-serif; font-size: 11px; font-weight: 500; fill: #000000; }\n";
    out << "  .axis-title { font-family: system-ui, -apple-system, sans-serif; font-size: 12px; font-weight: bold; fill: #000000; text-anchor: middle; }\n";
    out << "  .legend-bg { fill: #ffffff; stroke: #cbd5e1; stroke-width: 1.0; rx: 3px; }\n";
    out << "  .legend-text { font-family: system-ui, -apple-system, sans-serif; font-size: 11px; font-weight: 600; fill: #0f172a; }\n";
    out << "</style>\n";

    // Pure White Publication Background
    out << "<rect class=\"bg\" width=\"" << svgWidth << "\" height=\"" << totalHeight << "\"/>\n";

    // Title
    out << "<text class=\"title\" x=\"" << svgWidth * 0.5f << "\" y=\"30\">" << xmlEscape(scopeTitle) << "</text>\n";
    out << "<text class=\"subtitle\" x=\"" << svgWidth * 0.5f << "\" y=\"48\">IEEE Academic Publication Output | Span: ["
        << std::setprecision(5) << tStart << "s - " << tEnd << "s]</text>\n";

    float plotX = 90.0f;
    float plotW = svgWidth - 130.0f;

    for (int p = 0; p < renderPanes; ++p) {
        float paneY = headerHeight + p * paneHeight + 10.0f;
        float plotY = paneY + 25.0f;
        float plotH = paneHeight - 65.0f;

        out << "<g id=\"subplot_" << p << "\">\n";

        // Publication Subplot Border Box
        out << "  <rect class=\"pane-bg\" x=\"" << plotX << "\" y=\"" << plotY << "\" width=\"" << plotW << "\" height=\"" << plotH << "\"/>\n";

        std::vector<int> paneChans;
        for (int ch = 0; ch < totalChannels; ++ch) {
            if (numPanes == 1 || (ch % numPanes) == p) {
                paneChans.push_back(ch);
            }
        }

        // Calculate exact Y range across signals in this pane
        double yMin = 1e30, yMax = -1e30;
        for (int ch : paneChans) {
            const std::string& key = signalKeys[ch];
            auto it = telemetry.voltages.find(key);
            if (it == telemetry.voltages.end()) {
                std::string alt = key;
                if (alt.size() > 4 && alt.substr(alt.size()-4) == ".Out") {
                    alt = alt.substr(0, alt.size()-4);
                    it = telemetry.voltages.find(alt);
                }
                if (it == telemetry.voltages.end()) it = telemetry.voltages.find("V_" + key);
                if (it == telemetry.voltages.end()) it = telemetry.voltages.find("I_" + key);
            }

            if (it != telemetry.voltages.end()) {
                const auto& vec = it->second;
                for (size_t i = 0; i < timeData.size() && i < vec.size(); ++i) {
                    if (timeData[i] >= tStart && timeData[i] <= tEnd) {
                        if (vec[i] < yMin) yMin = vec[i];
                        if (vec[i] > yMax) yMax = vec[i];
                    }
                }
            }
        }

        if (yMin > yMax) { yMin = -1.0; yMax = 1.0; }
        double ySpan = yMax - yMin;
        double yPad = (ySpan > 1e-12) ? (0.12 * ySpan) : ((std::abs(yMin) > 1e-6) ? (0.12 * std::abs(yMin)) : 1.0);
        yMin -= yPad;
        yMax += yPad;

        // Draw Vertical Grid Lines & Inward Axis Ticks
        int numXTicks = 6;
        for (int i = 0; i < numXTicks; ++i) {
            float frac = (float)i / (numXTicks - 1);
            float gx = plotX + frac * plotW;
            double tVal = tStart + frac * (tEnd - tStart);

            out << "  <line class=\"grid-line\" x1=\"" << gx << "\" y1=\"" << plotY << "\" x2=\"" << gx << "\" y2=\"" << plotY + plotH << "\"/>\n";
            // Inward Tick
            out << "  <line x1=\"" << gx << "\" y1=\"" << plotY + plotH - 4 << "\" x2=\"" << gx << "\" y2=\"" << plotY + plotH << "\" stroke=\"#000000\" stroke-width=\"1.0\"/>\n";

            if (p == renderPanes - 1) {
                std::ostringstream ss;
                ss << std::fixed << std::setprecision(4) << tVal;
                out << "  <text class=\"axis-text\" x=\"" << gx << "\" y=\"" << plotY + plotH + 18 << "\" text-anchor=\"middle\">" << ss.str() << "</text>\n";
            }
        }

        // Draw Horizontal Grid Lines & Inward Axis Ticks
        int numYTicks = 5;
        for (int j = 0; j < numYTicks; ++j) {
            float frac = (float)j / (numYTicks - 1);
            float gy = plotY + plotH - frac * plotH;
            double yVal = yMin + frac * (yMax - yMin);

            out << "  <line class=\"grid-line\" x1=\"" << plotX << "\" y1=\"" << gy << "\" x2=\"" << plotX + plotW << "\" y2=\"" << gy << "\"/>\n";
            out << "  <line x1=\"" << plotX << "\" y1=\"" << gy << "\" x2=\"" << plotX + 4 << "\" y2=\"" << gy << "\" stroke=\"#000000\" stroke-width=\"1.0\"/>\n";

            std::ostringstream ss;
            ss << std::setprecision(3) << yVal;
            out << "  <text class=\"axis-text\" x=\"" << plotX - 8 << "\" y=\"" << gy + 4 << "\" text-anchor=\"end\">" << ss.str() << "</text>\n";
        }

        // Rotated Subplot Y-Axis Label
        std::string yTitle = (!paneChans.empty() && paneChans[0] < (int)labels.size()) ? labels[paneChans[0]] : "Amplitude";
        out << "  <text class=\"axis-title\" transform=\"rotate(-90, 30, " << (plotY + plotH * 0.5f) << ")\" x=\"30\" y=\"" << (plotY + plotH * 0.5f) << "\">" << xmlEscape(yTitle) << "</text>\n";

        // Subplot Clip Path
        out << "  <clipPath id=\"clip_pane_" << p << "\">\n";
        out << "    <rect x=\"" << plotX << "\" y=\"" << plotY << "\" width=\"" << plotW << "\" height=\"" << plotH << "\"/>\n";
        out << "  </clipPath>\n";

        // Plot Traces
        out << "  <g clip-path=\"url(#clip_pane_" << p << ")\">\n";

        for (int ch : paneChans) {
            const std::string& key = signalKeys[ch];
            const char* traceColor = IEEE_PALETTE[ch % 8];

            auto it = telemetry.voltages.find(key);
            if (it == telemetry.voltages.end()) {
                std::string alt = key;
                if (alt.size() > 4 && alt.substr(alt.size()-4) == ".Out") {
                    alt = alt.substr(0, alt.size()-4);
                    it = telemetry.voltages.find(alt);
                }
                if (it == telemetry.voltages.end()) it = telemetry.voltages.find("V_" + key);
                if (it == telemetry.voltages.end()) it = telemetry.voltages.find("I_" + key);
            }

            if (it != telemetry.voltages.end() && !it->second.empty()) {
                const auto& vec = it->second;
                out << "    <path d=\"M";
                bool first = true;

                for (size_t i = 0; i < timeData.size() && i < vec.size(); ++i) {
                    double t = timeData[i];
                    if (t < tStart || t > tEnd) continue;

                    double v = vec[i];
                    float px = plotX + (float)((t - tStart) / (tEnd - tStart)) * plotW;
                    float py = plotY + plotH - (float)((v - yMin) / (yMax - yMin)) * plotH;

                    if (first) {
                        out << " " << px << "," << py;
                        first = false;
                    } else {
                        out << " L " << px << "," << py;
                    }
                }
                out << "\" fill=\"none\" stroke=\"" << traceColor << "\" stroke-width=\"2.2\" stroke-linejoin=\"round\" stroke-linecap=\"round\"/>\n";
            }
        }
        out << "  </g>\n";

        // Publication Legend Box (Top Right Corner inside plot)
        float legWidth = 220.0f;
        float legHeight = 10.0f + paneChans.size() * 18.0f;
        float legX = plotX + plotW - legWidth - 12.0f;
        float legY = plotY + 10.0f;

        out << "  <rect class=\"legend-bg\" x=\"" << legX << "\" y=\"" << legY << "\" width=\"" << legWidth << "\" height=\"" << legHeight << "\"/>\n";

        float ly = legY + 16.0f;
        for (int ch : paneChans) {
            const char* traceColor = IEEE_PALETTE[ch % 8];
            std::string labelStr = (ch < (int)labels.size()) ? labels[ch] : signalKeys[ch];

            out << "  <rect x=\"" << legX + 8.0f << "\" y=\"" << ly - 9.0f << "\" width=\"14\" height=\"10\" rx=\"2\" fill=\"" << traceColor << "\"/>\n";
            out << "  <text class=\"legend-text\" x=\"" << legX + 28.0f << "\" y=\"" << ly << "\">" << xmlEscape(labelStr) << "</text>\n";
            ly += 18.0f;
        }

        out << "</g>\n";
    }

    // Main X-Axis Label at the bottom
    out << "</svg>\n";
    outSVG = out.str();
    return true;
}

bool SVGExporter::exportScopeToSVG(
    const CircuitSimEngine::TelemetryData& telemetry,
    const std::vector<std::string>& signalKeys,
    const std::vector<std::string>& labels,
    const std::string& scopeTitle,
    const std::string& filename,
    int numPanes,
    bool isDarkMode,
    double timeMin, double timeMax)
{
    std::string svgStr;
    if (!exportScopeToSVGString(telemetry, signalKeys, labels, scopeTitle, svgStr, numPanes, isDarkMode, timeMin, timeMax)) return false;
    std::ofstream out(filename);
    if (!out.is_open()) return false;
    out << svgStr;
    return true;
}

bool SVGExporter::exportFullReportToHTML(
    const CircuitDesign& design,
    const CircuitSimEngine::TelemetryData& telemetry,
    const std::vector<ScopeReportData>& scopesData,
    const std::string& schematicJson,
    const std::string& netlistJson,
    const std::string& filename,
    bool /*isDarkMode*/)
{
    std::string schematicSvg;
    exportSchematicToSVGString(design, schematicSvg, false /* Light Mode Only */);

    std::ofstream out(filename);
    if (!out.is_open()) return false;

    out << "<!DOCTYPE html>\n<html lang=\"en\">\n<head>\n";
    out << "<meta charset=\"UTF-8\">\n";
    out << "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">\n";
    out << "<title>Circuit Simulation & Verification Report</title>\n";
    out << "<style>\n";
    out << "  :root {\n";
    out << "    --bg-color: #f8fafc;\n";
    out << "    --card-bg: #ffffff;\n";
    out << "    --card-border: #e2e8f0;\n";
    out << "    --text-main: #0f172a;\n";
    out << "    --text-sub: #64748b;\n";
    out << "    --accent: #0284c7;\n";
    out << "    --code-bg: #f1f5f9;\n";
    out << "    --code-text: #0f172a;\n";
    out << "  }\n";
    out << "  body {\n";
    out << "    font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, Helvetica, Arial, sans-serif;\n";
    out << "    background-color: var(--bg-color);\n";
    out << "    color: var(--text-main);\n";
    out << "    margin: 0;\n";
    out << "    padding: 24px;\n";
    out << "    line-height: 1.5;\n";
    out << "  }\n";
    out << "  .header {\n";
    out << "    margin-bottom: 24px;\n";
    out << "    border-bottom: 2px solid var(--card-border);\n";
    out << "    padding-bottom: 16px;\n";
    out << "  }\n";
    out << "  .header h1 {\n";
    out << "    margin: 0 0 8px 0;\n";
    out << "    color: var(--accent);\n";
    out << "    font-size: 28px;\n";
    out << "  }\n";
    out << "  .meta-bar {\n";
    out << "    display: flex;\n";
    out << "    gap: 16px;\n";
    out << "    font-size: 14px;\n";
    out << "    color: var(--text-sub);\n";
    out << "  }\n";
    out << "  .meta-item {\n";
    out << "    background: #e0f2fe;\n";
    out << "    color: #0369a1;\n";
    out << "    padding: 4px 10px;\n";
    out << "    border-radius: 6px;\n";
    out << "    border: 1px solid #0284c733;\n";
    out << "    font-weight: 500;\n";
    out << "  }\n";
    out << "  .card {\n";
    out << "    background-color: var(--card-bg);\n";
    out << "    border: 1px solid var(--card-border);\n";
    out << "    border-radius: 12px;\n";
    out << "    padding: 20px;\n";
    out << "    margin-bottom: 24px;\n";
    out << "    box-shadow: 0 1px 3px 0 rgba(0, 0, 0, 0.05);\n";
    out << "  }\n";
    out << "  .card-title {\n";
    out << "    font-size: 20px;\n";
    out << "    font-weight: 600;\n";
    out << "    margin-top: 0;\n";
    out << "    margin-bottom: 16px;\n";
    out << "    color: var(--text-main);\n";
    out << "    display: flex;\n";
    out << "    justify-content: space-between;\n";
    out << "    align-items: center;\n";
    out << "  }\n";
    out << "  .svg-wrapper {\n";
    out << "    background: #ffffff;\n";
    out << "    border: 1px solid var(--card-border);\n";
    out << "    border-radius: 8px;\n";
    out << "    padding: 12px;\n";
    out << "    overflow: hidden;\n";
    out << "    text-align: center;\n";
    out << "    max-width: 100%;\n";
    out << "  }\n";
    out << "  .svg-wrapper svg {\n";
    out << "    max-width: 100%;\n";
    out << "    height: auto;\n";
    out << "  }\n";
    out << "  details summary {\n";
    out << "    cursor: pointer;\n";
    out << "    font-weight: 600;\n";
    out << "    font-size: 18px;\n";
    out << "    color: var(--accent);\n";
    out << "    padding: 8px 0;\n";
    out << "    user-select: none;\n";
    out << "  }\n";
    out << "  details[open] summary {\n";
    out << "    border-bottom: 1px solid var(--card-border);\n";
    out << "    margin-bottom: 12px;\n";
    out << "  }\n";
    out << "  pre {\n";
    out << "    background-color: var(--code-bg);\n";
    out << "    border: 1px solid #cbd5e1;\n";
    out << "    border-radius: 8px;\n";
    out << "    padding: 16px;\n";
    out << "    overflow-x: auto;\n";
    out << "    font-family: 'Fira Code', Consolas, Monaco, monospace;\n";
    out << "    font-size: 13px;\n";
    out << "    color: var(--code-text);\n";
    out << "    max-height: 400px;\n";
    out << "  }\n";
    out << "  .copy-btn {\n";
    out << "    background: #0284c7;\n";
    out << "    color: #fff;\n";
    out << "    border: none;\n";
    out << "    padding: 6px 12px;\n";
    out << "    border-radius: 6px;\n";
    out << "    font-size: 12px;\n";
    out << "    cursor: pointer;\n";
    out << "    transition: background 0.2s;\n";
    out << "    float: right;\n";
    out << "  }\n";
    out << "  .copy-btn:hover { background: #0369a1; }\n";
    out << "</style>\n</head>\n<body>\n";

    out << "  <div class=\"header\">\n";
    out << "    <h1>Circuit Simulation & Verification Report</h1>\n";
    out << "    <div class=\"meta-bar\">\n";
    out << "      <div class=\"meta-item\">Generated by SimPEL / CircuitSim Pro</div>\n";
    out << "      <div class=\"meta-item\">Format: Light Mode Document HTML</div>\n";
    out << "    </div>\n";
    out << "  </div>\n\n";

    out << "  <div class=\"card\">\n";
    out << "    <div class=\"card-title\">1. Circuit Schematic</div>\n";
    out << "    <div class=\"svg-wrapper\" id=\"schematicContainer\">\n";
    out << schematicSvg << "\n";
    out << "    </div>\n";
    out << "  </div>\n\n";

    int sectionIdx = 2;

    for (const auto& srd : scopesData) {
        std::string scopeSvg;
        exportScopeToSVGString(telemetry, srd.signalKeys, srd.signalLabels, srd.scopeTitle, scopeSvg, srd.numPanes, false /* Light Mode Scope */);

        if (!scopeSvg.empty()) {
            out << "  <div class=\"card\">\n";
            out << "    <div class=\"card-title\">" << sectionIdx << ". Oscilloscope Waveforms: " << xmlEscape(srd.scopeTitle) << "</div>\n";
            out << "    <div class=\"svg-wrapper\">\n";
            out << scopeSvg << "\n";
            out << "    </div>\n";
            out << "  </div>\n\n";
            sectionIdx++;
        }
    }

    out << "  <div class=\"card\">\n";
    out << "    <details open>\n";
    out << "      <summary>" << sectionIdx++ << ". Schematic JSON Structure</summary>\n";
    out << "      <button class=\"copy-btn\" onclick=\"copyToClipboard('schematic-json-text')\">Copy JSON</button>\n";
    out << "      <pre id=\"schematic-json-text\">" << xmlEscape(schematicJson) << "</pre>\n";
    out << "    </details>\n";
    out << "  </div>\n\n";

    out << "  <div class=\"card\">\n";
    out << "    <details open>\n";
    out << "      <summary>" << sectionIdx++ << ". Netlist JSON Specification</summary>\n";
    out << "      <button class=\"copy-btn\" onclick=\"copyToClipboard('netlist-json-text')\">Copy Netlist</button>\n";
    out << "      <pre id=\"netlist-json-text\">" << xmlEscape(netlistJson) << "</pre>\n";
    out << "    </details>\n";
    out << "  </div>\n\n";

    out << "  <script>\n";
    out << "    function copyToClipboard(elementId) {\n";
    out << "      const text = document.getElementById(elementId).innerText;\n";
    out << "      navigator.clipboard.writeText(text).then(() => {\n";
    out << "        alert('Copied to clipboard!');\n";
    out << "      });\n";
    out << "    }\n";
    out << "  </script>\n";
    out << "</body>\n</html>\n";

    out.close();
    return true;
}

} // namespace CircuitSim
