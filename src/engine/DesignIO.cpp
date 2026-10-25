// See DesignIO.hpp. The parsing rules below were consolidated from the private
// loaders in main.cpp (headless CLI) and ui/MainWindow.cpp (File -> Open, demo pane,
// batch export); tests/golden checks that every design in the corpus still produces
// the netlist the GUI produced before the consolidation.
#include "DesignIO.hpp"
#include "DesignNormalize.hpp"
#include "ExpressionEvaluator.hpp"

#include <algorithm>
#include <cctype>
#include <fstream>
#include <functional>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace CircuitSim {
namespace DesignIO {

using json = nlohmann::json;

namespace {

void parseWireEndpoint(const json& epObj, WireEndpoint& ep,
                       const std::function<std::string(const std::string&, const std::string&)>& resolveTerm) {
    if (!epObj.is_object()) return;

    std::string compId = epObj.value("compId", epObj.value("componentId", ""));
    std::string wireId = epObj.value("wireId", epObj.value("targetWireId", ""));
    std::string epType = epObj.value("type", "");
    std::string terminal = epObj.value("terminal", epObj.value("terminalName", ""));

    std::string targetW = !wireId.empty() ? wireId : compId;

    bool isJunction = (epType == "wire" || epType == "junction" || epObj.value("isWireJunction", false));
    // Legacy files without an endpoint "type" name junctions only by a wire id
    // ("w12"). The guess must not apply to an explicit "pin": component ids such as
    // W1 (windings) start with W too.
    if (!isJunction && epType.empty() && !targetW.empty()) {
        if ((targetW[0] == 'w' || targetW[0] == 'W') && targetW.find('.') == std::string::npos) {
            isJunction = true;
        }
    }

    if (isJunction) {
        ep.isWireJunction = true;
        ep.targetWireId = targetW;
        ep.junctionX = epObj.value("x", epObj.value("junctionX", 0.0f));
        ep.junctionY = epObj.value("y", epObj.value("junctionY", 0.0f));
        ep.compId = "";
        ep.terminal = "";
    } else {
        ep.isWireJunction = false;
        ep.compId = compId;
        ep.terminal = resolveTerm ? resolveTerm(compId, terminal) : terminal;
        ep.targetWireId = "";
        ep.junctionX = 0.0f;
        ep.junctionY = 0.0f;
    }
}

json serializeWireEndpoint(const WireEndpoint& ep) {
    json obj;
    // The loader sets isWireJunction for every junction it reads, so the endpoint's
    // own flag is authoritative; guessing from a leading 'w' would turn pins of
    // components such as W1 into junctions.
    bool isJunc = ep.isWireJunction || (!ep.targetWireId.empty() && ep.compId.empty());

    if (isJunc) {
        std::string targetId = !ep.targetWireId.empty() ? ep.targetWireId : ep.compId;
        obj["type"] = "junction";
        obj["compId"] = targetId;
        obj["terminal"] = "";
        obj["x"] = ep.junctionX;
        obj["y"] = ep.junctionY;
    } else {
        obj["type"] = "pin";
        obj["compId"] = ep.compId;
        obj["terminal"] = ep.terminal;
    }
    return obj;
}

void sanitizeWires(CircuitDesign& cd) {
    // 1. Remove invalid, dangling, or incomplete wire entries
    std::vector<WireInstance> cleanWires;
    std::unordered_map<std::string, const ComponentInstance*> compMap;
    for (const auto& c : cd.components) compMap[c.id] = &c;

    for (const auto& w : cd.wires) {
        bool fromValid = w.from.isWireJunction ? !w.from.targetWireId.empty() : !w.from.compId.empty();
        bool toValid   = w.to.isWireJunction   ? !w.to.targetWireId.empty()   : !w.to.compId.empty();

        if (!fromValid || !toValid) continue;

        if (!w.from.isWireJunction && !compMap.count(w.from.compId)) continue;
        if (!w.to.isWireJunction && !compMap.count(w.to.compId)) continue;

        if (!w.from.isWireJunction && !w.to.isWireJunction && w.from.compId == w.to.compId && w.from.terminal == w.to.terminal) {
            continue;
        }

        cleanWires.push_back(w);
    }
    cd.wires = cleanWires;

    // 2. Remove duplicate or reverse-parallel wire segments connecting the exact same endpoints
    std::vector<WireInstance> dedupedWires;
    std::unordered_set<std::string> seenEndpoints;

    for (const auto& w : cd.wires) {
        std::string ep1 = w.from.isWireJunction ? ("j:" + w.from.targetWireId) : ("p:" + w.from.compId + "." + w.from.terminal);
        std::string ep2 = w.to.isWireJunction   ? ("j:" + w.to.targetWireId)   : ("p:" + w.to.compId + "." + w.to.terminal);

        std::string forwardKey = ep1 + "<->" + ep2;
        std::string reverseKey = ep2 + "<->" + ep1;

        if (seenEndpoints.count(forwardKey) || seenEndpoints.count(reverseKey)) {
            continue;
        }
        seenEndpoints.insert(forwardKey);
        seenEndpoints.insert(reverseKey);
        dedupedWires.push_back(w);
    }
    cd.wires = dedupedWires;

    // 3. Fix uninitialized junction coordinates (0, 0)
    std::unordered_map<std::string, const WireInstance*> wireLookup;
    for (const auto& w : cd.wires) wireLookup[w.id] = &w;

    auto resolvePinPos = [&](const std::string& compId, const std::string& termName, float& outX, float& outY) -> bool {
        auto it = compMap.find(compId);
        if (it == compMap.end()) return false;
        const auto* comp = it->second;
        for (const auto& pin : comp->pins) {
            if (pin.name == termName) {
                outX = comp->x + pin.relativeX;
                outY = comp->y + pin.relativeY;
                return true;
            }
        }
        outX = comp->x; outY = comp->y;
        return true;
    };

    auto fixJunction = [&](WireEndpoint& ep) {
        if (!(ep.isWireJunction && ep.junctionX == 0.0f && ep.junctionY == 0.0f)) return;
        auto it = wireLookup.find(ep.targetWireId);
        if (it == wireLookup.end()) return;
        float fx = 0, fy = 0, tx = 0, ty = 0;
        bool fOk = resolvePinPos(it->second->from.compId, it->second->from.terminal, fx, fy);
        bool tOk = resolvePinPos(it->second->to.compId, it->second->to.terminal, tx, ty);
        if (fOk && tOk) {
            ep.junctionX = (fx + tx) * 0.5f;
            ep.junctionY = (fy + ty) * 0.5f;
        } else if (fOk) {
            ep.junctionX = fx; ep.junctionY = fy;
        } else if (tOk) {
            ep.junctionX = tx; ep.junctionY = ty;
        }
    };
    for (auto& w : cd.wires) {
        fixJunction(w.from);
        fixJunction(w.to);
    }
}

// Reads a number that may be stored as a JSON number or as an engineering-notation
// string ("10u", "1e-5"). Leaves `dst` untouched if the key is absent or unparsable.
void readNum(const json& obj, const char* key, double& dst) {
    if (!obj.contains(key)) return;
    const auto& v = obj[key];
    if (v.is_number()) dst = v.get<double>();
    else if (v.is_string()) {
        try { dst = CircuitSimEngine::ExpressionEvaluator::parseScientific(v.get<std::string>()); } catch (...) {}
    }
}

void readBool(const json& obj, const char* key, bool& dst) {
    if (!obj.contains(key)) return;
    const auto& v = obj[key];
    if (v.is_boolean()) dst = v.get<bool>();
    else if (v.is_number()) dst = (v.get<double>() != 0.0);
    else if (v.is_string()) {
        std::string s = v.get<std::string>();
        std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return (char)std::tolower(c); });
        if (s == "true" || s == "1") dst = true;
        else if (s == "false" || s == "0") dst = false;
    }
}

void readStr(const json& obj, const char* key, std::string& dst) {
    if (obj.contains(key) && obj[key].is_string()) dst = obj[key].get<std::string>();
}

void readCamel(const json& ss, SolverSettings& s) {
    readNum(ss, "stopTime", s.stopTime);
    readNum(ss, "stepSize", s.stepSize);
    readStr(ss, "solver", s.solverType);
    readStr(ss, "solverType", s.solverType);
    readStr(ss, "stepType", s.stepType);
    readStr(ss, "diodeModel", s.diodeModel);
    readBool(ss, "enableLUCache", s.enableLUCache);
    readNum(ss, "relTol", s.relTol);
    readNum(ss, "absTolV", s.absTolV);
    readNum(ss, "absTolI", s.absTolI);
    readNum(ss, "hMin", s.hMin);
    readNum(ss, "hMax", s.hMax);
}

void readSnake(const json& sp, SolverSettings& s) {
    readNum(sp, "stop_time", s.stopTime);
    readNum(sp, "step_size", s.stepSize);
    readStr(sp, "solver", s.solverType);
    readStr(sp, "step_type", s.stepType);
    readStr(sp, "diode_model", s.diodeModel);
    readBool(sp, "enable_lu_cache", s.enableLUCache);
    readNum(sp, "rel_tol", s.relTol);
    readNum(sp, "abs_tol_v", s.absTolV);
    readNum(sp, "abs_tol_i", s.absTolI);
    readNum(sp, "h_min", s.hMin);
    readNum(sp, "h_max", s.hMax);
}

} // namespace

void readSettings(const json& j, SolverSettings& s, PlotConfig* plot) {
    if (!j.is_object()) return;
    if (j.contains("settings") && j["settings"].is_object()) {
        readCamel(j["settings"], s);
        readSnake(j["settings"], s);
    }
    if (j.contains("simulationSettings") && j["simulationSettings"].is_object()) readCamel(j["simulationSettings"], s);
    if (j.contains("simulation_parameters") && j["simulation_parameters"].is_object()) readSnake(j["simulation_parameters"], s);

    if (plot && j.contains("plotConfig") && j["plotConfig"].is_object()) {
        const auto& pc = j["plotConfig"];
        if (pc.contains("plots") && pc["plots"].is_array()) {
            plot->plots.clear();
            for (const auto& p : pc["plots"]) {
                if (!p.is_object()) continue;
                PlotChannelConfig ch;
                ch.title = p.value("title", ch.title);
                if (p.contains("variables") && p["variables"].is_array())
                    for (const auto& v : p["variables"]) if (v.is_string()) ch.variables.push_back(v.get<std::string>());
                plot->plots.push_back(ch);
            }
        }
    }
}

CircuitDesign loadDesign(const json& j, const LoadOptions& opts) {
    CircuitDesign cd;
    if (!j.is_object()) throw std::runtime_error("design JSON must be an object");

    if (j.contains("components") && j["components"].is_array()) {
        for (const auto& cItem : j["components"]) {
            ComponentInstance comp;
            comp.id = cItem.value("id", "");
            comp.rawTypeStr = cItem.value("type", "R");
            comp.type = stringToComponentType(comp.rawTypeStr);
            comp.label = cItem.value("label", comp.id);
            comp.x = cItem.value("x", 0.0f);
            comp.y = cItem.value("y", 0.0f);
            comp.rotation = cItem.value("rotation", 0);
            auto parseParamsObj = [&](const json& pObj) {
                for (auto& [k, v] : pObj.items()) {
                    if (v.is_string()) comp.parameters[k] = v.get<std::string>();
                    else if (v.is_number()) comp.parameters[k] = std::to_string(v.get<double>());
                    else if (v.is_boolean()) comp.parameters[k] = v.get<bool>() ? "true" : "false";
                }
            };
            if (cItem.contains("parameters") && cItem["parameters"].is_object()) parseParamsObj(cItem["parameters"]);
            if (cItem.contains("params") && cItem["params"].is_object()) parseParamsObj(cItem["params"]);

            // Normalise web-tool parameter aliases onto the native keys, otherwise the
            // value is ignored by the solver and silently falls back to a default.
            if (comp.type == ComponentType::Resistor ||
                comp.type == ComponentType::VariableResistor ||
                comp.type == ComponentType::PWLResistor) {
                if (!comp.parameters.count("value")) {
                    if (comp.parameters.count("resistance")) comp.parameters["value"] = comp.parameters["resistance"];
                    else if (comp.parameters.count("R")) comp.parameters["value"] = comp.parameters["R"];
                }
                comp.parameters.erase("resistance");
            }

            // Backfill energy-storage initial-condition params so they are editable
            // in the Property Inspector even for circuits saved without them.
            if (comp.type == ComponentType::Inductor ||
                comp.type == ComponentType::VariableInductor ||
                comp.type == ComponentType::SaturableInductor) {
                if (!comp.parameters.count("iL0")) comp.parameters["iL0"] = "0";
            }
            if (comp.type == ComponentType::Capacitor ||
                comp.type == ComponentType::VariableCapacitor ||
                comp.type == ComponentType::SaturableCapacitor) {
                if (!comp.parameters.count("vC0")) comp.parameters["vC0"] = "0";
            }

            setupComponentPins(comp);
            cd.components.push_back(comp);
        }
    }

    std::unordered_map<std::string, std::string> compTypeMap;
    for (const auto& c : cd.components) compTypeMap[c.id] = c.rawTypeStr;

    // Scope inputs have been saved as ch1/CH1/in1; the canvas names them In1.
    auto resolveTerminalName = [&](const std::string& compId, const std::string& term) -> std::string {
        auto it = compTypeMap.find(compId);
        if (it != compTypeMap.end()) {
            const std::string& t = it->second;
            if (t == "SCOPE" || t == "Oscilloscope") {
                std::string lowerTerm = term;
                std::transform(lowerTerm.begin(), lowerTerm.end(), lowerTerm.begin(), ::tolower);
                if (lowerTerm.rfind("ch", 0) == 0 && lowerTerm.length() > 2) return "In" + lowerTerm.substr(2);
                if (lowerTerm.rfind("in", 0) == 0 && lowerTerm.length() > 2) return "In" + lowerTerm.substr(2);
            }
        }
        return term;
    };

    if (j.contains("wires") && j["wires"].is_array()) {
        // Pass 1: build a remap table from legacy/original wire ids ("oldId") to the
        // current wire ids. Copy-pasted or re-exported schematics keep pointing their
        // junction "targetWireId" at the pre-copy ids, which would otherwise dangle.
        std::unordered_map<std::string, std::string> wireIdRemap;
        std::unordered_set<std::string> knownWireIds;
        for (const auto& wItem : j["wires"]) {
            std::string wid = wItem.value("id", "");
            if (wid.empty()) continue;
            knownWireIds.insert(wid);
            std::string oldId = wItem.value("oldId", "");
            if (!oldId.empty() && oldId != wid) wireIdRemap[oldId] = wid;
        }

        for (const auto& wItem : j["wires"]) {
            WireInstance wire;
            wire.id = wItem.value("id", "");
            if (wItem.contains("from")) {
                parseWireEndpoint(wItem["from"], wire.from, resolveTerminalName);
            } else if (wItem.contains("fromComp")) {
                wire.from.compId = wItem.value("fromComp", "");
                wire.from.terminal = resolveTerminalName(wire.from.compId, wItem.value("fromTerm", wItem.value("fromTerminal", "")));
                wire.from.isWireJunction = false;
            }

            if (wItem.contains("to")) {
                parseWireEndpoint(wItem["to"], wire.to, resolveTerminalName);
            } else if (wItem.contains("toComp")) {
                wire.to.compId = wItem.value("toComp", "");
                wire.to.terminal = resolveTerminalName(wire.to.compId, wItem.value("toTerm", wItem.value("toTerminal", "")));
                wire.to.isWireJunction = false;
            }

            bool isJunc = wItem.value("isJunction", false);
            std::string targetWId = wItem.value("targetWireId", "");
            // Only convert the "to" endpoint into a wire junction when it does NOT
            // already reference a real component pin.
            if ((isJunc || !targetWId.empty()) && wire.to.compId.empty()) {
                wire.to.isWireJunction = true;
                wire.to.targetWireId = targetWId;
                wire.to.junctionX = wItem.value("jX", 0.0f);
                wire.to.junctionY = wItem.value("jY", 0.0f);
            }

            // Routed wire geometry (written by this file's saveDesign; the web tool's
            // "manualPath" uses a different geometry and is not read).
            if (wItem.contains("route") && wItem["route"].is_array()) {
                for (const auto& p : wItem["route"]) {
                    if (!p.is_object() || !p.contains("x") || !p.contains("y") ||
                        !p["x"].is_number() || !p["y"].is_number()) { wire.manualPath.clear(); break; }
                    Point2D q;
                    q.x = p["x"].get<float>();
                    q.y = p["y"].get<float>();
                    wire.manualPath.push_back(q);
                }
            }

            cd.wires.push_back(wire);
        }

        // Pass 2: resolve junction targets through the oldId remap table.
        for (auto& w : cd.wires) {
            auto remapEndpoint = [&](WireEndpoint& ep) {
                if (!ep.isWireJunction || ep.targetWireId.empty()) return;
                if (knownWireIds.count(ep.targetWireId)) return;
                auto it = wireIdRemap.find(ep.targetWireId);
                if (it != wireIdRemap.end()) ep.targetWireId = it->second;
            };
            remapEndpoint(w.from);
            remapEndpoint(w.to);
        }

        // Pass 3: drop junctions that still cannot be resolved; they contribute no
        // electrical connection and would create phantom isolated nets.
        for (auto& w : cd.wires) {
            auto dropDangling = [&](WireEndpoint& ep) {
                if (ep.isWireJunction && !ep.targetWireId.empty() && !knownWireIds.count(ep.targetWireId)) {
                    ep.targetWireId.clear();
                }
            };
            dropDangling(w.from);
            dropDangling(w.to);
        }

        sanitizeWires(cd);
    }

    readSettings(j, cd.settings, &cd.plotConfig);

    if (opts.normaliseWires) normaliseDesignWires(cd);
    return cd;
}

CircuitDesign loadDesignFile(const std::string& path, const LoadOptions& opts) {
    std::ifstream f(path, std::ios::binary);
    if (!f.is_open()) throw std::runtime_error("cannot open design file: " + path);
    json j = json::parse(f);
    return loadDesign(j, opts);
}

json saveDesign(const CircuitDesign& cd) {
    json j;
    json compArray = json::array();
    for (const auto& comp : cd.components) {
        json cObj;
        cObj["id"] = comp.id;
        cObj["type"] = comp.rawTypeStr;
        cObj["label"] = comp.label;
        cObj["x"] = comp.x;
        cObj["y"] = comp.y;
        cObj["rotation"] = comp.rotation;
        json pObj = json::object();
        for (const auto& [k, v] : comp.parameters) pObj[k] = v;
        cObj["parameters"] = pObj;
        compArray.push_back(cObj);
    }
    j["components"] = compArray;
    json wireArray = json::array();
    for (const auto& wire : cd.wires) {
        json wObj;
        wObj["id"] = wire.id;
        wObj["from"] = serializeWireEndpoint(wire.from);
        wObj["to"] = serializeWireEndpoint(wire.to);
        if (!wire.manualPath.empty()) {
            json route = json::array();
            for (const auto& p : wire.manualPath) route.push_back({{"x", p.x}, {"y", p.y}});
            wObj["route"] = route;
        }
        wireArray.push_back(wObj);
    }
    j["wires"] = wireArray;

    const SolverSettings& s = cd.settings;
    json ss;
    ss["stopTime"] = s.stopTime;
    ss["stepSize"] = s.stepSize;
    ss["solver"] = s.solverType;
    ss["stepType"] = s.stepType;
    ss["diodeModel"] = s.diodeModel;
    ss["enableLUCache"] = s.enableLUCache;
    ss["relTol"] = s.relTol;
    ss["absTolV"] = s.absTolV;
    ss["absTolI"] = s.absTolI;
    ss["hMin"] = s.hMin;
    ss["hMax"] = s.hMax;
    j["simulationSettings"] = ss;

    if (!cd.plotConfig.plots.empty()) {
        json plots = json::array();
        for (const auto& p : cd.plotConfig.plots) {
            json pj;
            pj["title"] = p.title;
            pj["variables"] = p.variables;
            plots.push_back(pj);
        }
        j["plotConfig"]["plots"] = plots;
    }
    return j;
}

std::string saveDesignString(const CircuitDesign& design) {
    return saveDesign(design).dump(2);
}

void saveDesignFile(const CircuitDesign& design, const std::string& path) {
    std::ofstream out(path, std::ios::binary);
    if (!out.is_open()) throw std::runtime_error("cannot write design file: " + path);
    out << saveDesignString(design);
    if (!out) throw std::runtime_error("write failed: " + path);
}

} // namespace DesignIO
} // namespace CircuitSim
