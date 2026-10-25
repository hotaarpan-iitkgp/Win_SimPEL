// FROZEN ORACLE. Verbatim copies of the loaders as they existed before the
// SimPEL API refactor (MainWindow.cpp File->Open and main.cpp headless CLI).
// The golden runner compares the refactored code paths against these. Do not edit.
#pragma once
#include "engine/Components.hpp"
#include "engine/ExpressionEvaluator.hpp"
#include <nlohmann/json.hpp>
#include <algorithm>
#include <functional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace golden_ref_gui {
using namespace CircuitSim;
using json = nlohmann::json;

static void parseWireEndpointJSON(
    const json& epObj,
    WireEndpoint& ep,
    const std::function<std::string(const std::string&, const std::string&)>& resolveTerm = nullptr
) {
    if (!epObj.is_object()) return;

    std::string compId = epObj.value("compId", epObj.value("componentId", ""));
    std::string wireId = epObj.value("wireId", epObj.value("targetWireId", ""));
    std::string epType = epObj.value("type", "");
    std::string terminal = epObj.value("terminal", epObj.value("terminalName", ""));

    std::string targetW = !wireId.empty() ? wireId : compId;

    bool isJunction = (epType == "wire" || epType == "junction" || epObj.value("isWireJunction", false));
    if (!isJunction && !targetW.empty()) {
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

static void sanitizeCircuitWires(CircuitDesign& cd) {
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

    for (auto& w : cd.wires) {
        if (w.from.isWireJunction && w.from.junctionX == 0.0f && w.from.junctionY == 0.0f) {
            auto it = wireLookup.find(w.from.targetWireId);
            if (it != wireLookup.end()) {
                float fx = 0, fy = 0, tx = 0, ty = 0;
                bool fOk = resolvePinPos(it->second->from.compId, it->second->from.terminal, fx, fy);
                bool tOk = resolvePinPos(it->second->to.compId, it->second->to.terminal, tx, ty);
                if (fOk && tOk) {
                    w.from.junctionX = (fx + tx) * 0.5f;
                    w.from.junctionY = (fy + ty) * 0.5f;
                } else if (fOk) {
                    w.from.junctionX = fx; w.from.junctionY = fy;
                } else if (tOk) {
                    w.from.junctionX = tx; w.from.junctionY = ty;
                }
            }
        }
        if (w.to.isWireJunction && w.to.junctionX == 0.0f && w.to.junctionY == 0.0f) {
            auto it = wireLookup.find(w.to.targetWireId);
            if (it != wireLookup.end()) {
                float fx = 0, fy = 0, tx = 0, ty = 0;
                bool fOk = resolvePinPos(it->second->from.compId, it->second->from.terminal, fx, fy);
                bool tOk = resolvePinPos(it->second->to.compId, it->second->to.terminal, tx, ty);
                if (fOk && tOk) {
                    w.to.junctionX = (fx + tx) * 0.5f;
                    w.to.junctionY = (fy + ty) * 0.5f;
                } else if (fOk) {
                    w.to.junctionX = fx; w.to.junctionY = fy;
                } else if (tOk) {
                    w.to.junctionX = tx; w.to.junctionY = ty;
                }
            }
        }
    }
}

// Body of MainWindow::renderMenuBar() "Open Schematic (.json)" between parse and canvas.setCircuit.
static CircuitDesign loadGuiOpen(const json& j) {
                        CircuitDesign cd;
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
                                if (cItem.contains("parameters") && cItem["parameters"].is_object()) {
                                    for (auto& [k, v] : cItem["parameters"].items()) {
                                        if (v.is_string()) comp.parameters[k] = v.get<std::string>();
                                        else if (v.is_number()) comp.parameters[k] = std::to_string(v.get<double>());
                                        else if (v.is_boolean()) comp.parameters[k] = v.get<bool>() ? "true" : "false";
                                    }
                                }
                                setupComponentPins(comp);
                                cd.components.push_back(comp);
                            }
                        }
                        if (j.contains("wires") && j["wires"].is_array()) {
                            for (const auto& wItem : j["wires"]) {
                                WireInstance wire;
                                wire.id = wItem.value("id", "");
                                if (wItem.contains("from")) parseWireEndpointJSON(wItem["from"], wire.from);
                                if (wItem.contains("to")) parseWireEndpointJSON(wItem["to"], wire.to);
                                cd.wires.push_back(wire);
                            }

                            sanitizeCircuitWires(cd);
                        }
                        if (j.contains("simulationSettings") && j["simulationSettings"].is_object()) {
                            const auto& ss = j["simulationSettings"];
                            if (ss.contains("stopTime")) {
                                if (ss["stopTime"].is_number()) cd.settings.stopTime = ss["stopTime"].get<double>();
                                else if (ss["stopTime"].is_string()) cd.settings.stopTime = CircuitSimEngine::ExpressionEvaluator::parseScientific(ss["stopTime"].get<std::string>());
                            }
                            if (ss.contains("stepSize")) {
                                if (ss["stepSize"].is_number()) cd.settings.stepSize = ss["stepSize"].get<double>();
                                else if (ss["stepSize"].is_string()) cd.settings.stepSize = CircuitSimEngine::ExpressionEvaluator::parseScientific(ss["stepSize"].get<std::string>());
                            }
                        }
                        if (j.contains("simulation_parameters") && j["simulation_parameters"].is_object()) {
                            const auto& sp = j["simulation_parameters"];
                            if (sp.contains("stop_time")) {
                                if (sp["stop_time"].is_number()) cd.settings.stopTime = sp["stop_time"].get<double>();
                                else if (sp["stop_time"].is_string()) cd.settings.stopTime = CircuitSimEngine::ExpressionEvaluator::parseScientific(sp["stop_time"].get<std::string>());
                            }
                            if (sp.contains("step_size")) {
                                if (sp["step_size"].is_number()) cd.settings.stepSize = sp["step_size"].get<double>();
                                else if (sp["step_size"].is_string()) cd.settings.stepSize = CircuitSimEngine::ExpressionEvaluator::parseScientific(sp["step_size"].get<std::string>());
                            }
                            // Solver selection and adaptive-stepping controls. These were
                            // previously not restored at all, so the solver and step type
                            // silently reverted to the defaults every time a design was
                            // reopened.
                            if (sp.contains("solver") && sp["solver"].is_string()) {
                                cd.settings.solverType = sp["solver"].get<std::string>();
                            }
                            if (sp.contains("step_type") && sp["step_type"].is_string()) {
                                cd.settings.stepType = sp["step_type"].get<std::string>();
                            }
                            if (sp.contains("enable_lu_cache")) {
                                const auto& v = sp["enable_lu_cache"];
                                if (v.is_boolean()) cd.settings.enableLUCache = v.get<bool>();
                                else if (v.is_number()) cd.settings.enableLUCache = (v.get<double>() != 0.0);
                            }
                            {
                                auto loadNum = [&](const char* key, double& dst) {
                                    if (!sp.contains(key)) return;
                                    const auto& v = sp[key];
                                    if (v.is_number()) dst = v.get<double>();
                                    else if (v.is_string()) dst = CircuitSimEngine::ExpressionEvaluator::parseScientific(v.get<std::string>());
                                };
                                loadNum("rel_tol",   cd.settings.relTol);
                                loadNum("abs_tol_v", cd.settings.absTolV);
                                loadNum("abs_tol_i", cd.settings.absTolI);
                                loadNum("h_min",     cd.settings.hMin);
                                loadNum("h_max",     cd.settings.hMax);
                            }
                        }
    return cd;
}
} // namespace golden_ref_gui

namespace golden_ref_cli {

static void parseWireEndpointJSON(
    const nlohmann::json& epObj,
    CircuitSim::WireEndpoint& ep,
    const std::function<std::string(const std::string&, const std::string&)>& resolveTerm = nullptr
) {
    if (!epObj.is_object()) return;

    std::string compId = epObj.value("compId", epObj.value("componentId", ""));
    std::string wireId = epObj.value("wireId", epObj.value("targetWireId", ""));
    std::string epType = epObj.value("type", "");
    std::string terminal = epObj.value("terminal", epObj.value("terminalName", ""));

    std::string targetW = !wireId.empty() ? wireId : compId;

    bool isJunction = (epType == "wire" || epType == "junction" || epObj.value("isWireJunction", false));
    if (!isJunction && !targetW.empty()) {
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

static void sanitizeCircuitWires(CircuitSim::CircuitDesign& cd) {
    std::vector<CircuitSim::WireInstance> cleanWires;
    std::unordered_map<std::string, const CircuitSim::ComponentInstance*> compMap;
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

    std::vector<CircuitSim::WireInstance> dedupedWires;
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
}

static CircuitSim::CircuitDesign loadSchematicFromJson(const nlohmann::json& j) {
    CircuitSim::CircuitDesign cd;
    if (j.contains("components") && j["components"].is_array()) {
        for (const auto& cItem : j["components"]) {
            CircuitSim::ComponentInstance comp;
            comp.id = cItem.value("id", "");
            comp.rawTypeStr = cItem.value("type", "R");
            comp.type = CircuitSim::stringToComponentType(comp.rawTypeStr);
            comp.label = cItem.value("label", comp.id);
            comp.x = cItem.value("x", 0.0f);
            comp.y = cItem.value("y", 0.0f);
            comp.rotation = cItem.value("rotation", 0);
            auto parseParamsObj = [&](const nlohmann::json& pObj) {
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
            if (comp.type == CircuitSim::ComponentType::Resistor ||
                comp.type == CircuitSim::ComponentType::VariableResistor ||
                comp.type == CircuitSim::ComponentType::PWLResistor) {
                if (!comp.parameters.count("value")) {
                    if (comp.parameters.count("resistance")) comp.parameters["value"] = comp.parameters["resistance"];
                    else if (comp.parameters.count("R")) comp.parameters["value"] = comp.parameters["R"];
                }
                comp.parameters.erase("resistance");
            }

            // Backfill energy-storage initial-condition params so they are editable
            // in the Property Inspector even for circuits saved without them.
            if (comp.type == CircuitSim::ComponentType::Inductor ||
                comp.type == CircuitSim::ComponentType::VariableInductor ||
                comp.type == CircuitSim::ComponentType::SaturableInductor) {
                if (!comp.parameters.count("iL0")) comp.parameters["iL0"] = "0";
            }
            if (comp.type == CircuitSim::ComponentType::Capacitor ||
                comp.type == CircuitSim::ComponentType::VariableCapacitor ||
                comp.type == CircuitSim::ComponentType::SaturableCapacitor) {
                if (!comp.parameters.count("vC0")) comp.parameters["vC0"] = "0";
            }

            CircuitSim::setupComponentPins(comp);
            cd.components.push_back(comp);
        }
    }

    std::unordered_map<std::string, std::string> compTypeMap;
    for (const auto& c : cd.components) {
        compTypeMap[c.id] = c.rawTypeStr;
    }

    auto resolveTerminalName = [&](const std::string& compId, const std::string& term) -> std::string {
        auto it = compTypeMap.find(compId);
        if (it != compTypeMap.end()) {
            std::string t = it->second;
            if (t == "SCOPE" || t == "Oscilloscope") {
                std::string lowerTerm = term;
                std::transform(lowerTerm.begin(), lowerTerm.end(), lowerTerm.begin(), ::tolower);
                if (lowerTerm.rfind("ch", 0) == 0 && lowerTerm.length() > 2) {
                    return "In" + lowerTerm.substr(2);
                }
                if (lowerTerm.rfind("in", 0) == 0 && lowerTerm.length() > 2) {
                    return "In" + lowerTerm.substr(2);
                }
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
            CircuitSim::WireInstance wire;
            wire.id = wItem.value("id", "");
            if (wItem.contains("from")) {
                parseWireEndpointJSON(wItem["from"], wire.from, resolveTerminalName);
            } else if (wItem.contains("fromComp")) {
                wire.from.compId = wItem.value("fromComp", "");
                wire.from.terminal = resolveTerminalName(wire.from.compId, wItem.value("fromTerm", wItem.value("fromTerminal", "")));
                wire.from.isWireJunction = false;
            }

            if (wItem.contains("to")) {
                parseWireEndpointJSON(wItem["to"], wire.to, resolveTerminalName);
            } else if (wItem.contains("toComp")) {
                wire.to.compId = wItem.value("toComp", "");
                wire.to.terminal = resolveTerminalName(wire.to.compId, wItem.value("toTerm", wItem.value("toTerminal", "")));
                wire.to.isWireJunction = false;
            }

            bool isJunc = wItem.value("isJunction", false);
            std::string targetWId = wItem.value("targetWireId", "");
            // Only convert the "to" endpoint into a wire junction when it does NOT
            // already reference a real component pin. Previously this overwrote valid
            // pin connections, silently disconnecting components (e.g. series meters).
            if ((isJunc || !targetWId.empty()) && wire.to.compId.empty()) {
                wire.to.isWireJunction = true;
                wire.to.targetWireId = targetWId;
                wire.to.junctionX = wItem.value("jX", 0.0f);
                wire.to.junctionY = wItem.value("jY", 0.0f);
            }

            cd.wires.push_back(wire);
        }

        // Pass 2: resolve junction targets through the oldId remap table.
        for (auto& w : cd.wires) {
            auto remapEndpoint = [&](CircuitSim::WireEndpoint& ep) {
                if (!ep.isWireJunction || ep.targetWireId.empty()) return;
                if (knownWireIds.count(ep.targetWireId)) return; // already valid
                auto it = wireIdRemap.find(ep.targetWireId);
                if (it != wireIdRemap.end()) ep.targetWireId = it->second;
            };
            remapEndpoint(w.from);
            remapEndpoint(w.to);
        }

        // Pass 3: drop junctions that still cannot be resolved — they contribute no
        // electrical connection and would create phantom isolated nets.
        for (auto& w : cd.wires) {
            auto dropDangling = [&](CircuitSim::WireEndpoint& ep) {
                if (ep.isWireJunction && !ep.targetWireId.empty() &&
                    !knownWireIds.count(ep.targetWireId)) {
                    ep.targetWireId.clear();
                }
            };
            dropDangling(w.from);
            dropDangling(w.to);
        }

        sanitizeCircuitWires(cd);
    }
    return cd;
}

} // namespace golden_ref_cli
