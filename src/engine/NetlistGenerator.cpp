// Schematic (CircuitDesign) -> solver netlist JSON. Moved verbatim from
// ui/NetlistSourceView.cpp so the engine can emit netlists without the UI.
#include "NetlistGenerator.hpp"
#include "NetlistBuilder.hpp"
#include "BlockRegistry.hpp"
#include "ExpressionEvaluator.hpp"
#include "CScriptEngine.hpp"
#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>
#include <algorithm>
#include <unordered_set>
#include <set>
#include <sstream>
#include <cmath>

using json = nlohmann::json;

namespace CircuitSim {

static std::vector<double> parseTurnsArrayStr(const std::string& str) {
    std::vector<double> res;
    if (str.empty()) { res.push_back(100.0); return res; }
    std::string clean = str;
    clean.erase(std::remove(clean.begin(), clean.end(), '['), clean.end());
    clean.erase(std::remove(clean.begin(), clean.end(), ']'), clean.end());
    std::replace(clean.begin(), clean.end(), ',', ' ');
    std::stringstream ss(clean);
    double val;
    while (ss >> val) {
        res.push_back(val);
    }
    if (res.empty()) res.push_back(100.0);
    return res;
}

static double roundToDigits(double val, int digits = 9) {
    if (val == 0.0) return 0.0;
    double scale = std::pow(10.0, digits);
    return std::round(val * scale) / scale;
}

static json formatJSStyleDouble(double val) {
    if (val == 0.0) return 0;
    if (val == std::floor(val) && std::abs(val) < 1e15) {
        return (long long)val;
    }
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%.17g", val);
    std::string s(buf);
    try {
        return json::parse(s);
    } catch (...) {
        return val;
    }
}

// Node name of the i-th electrical pin, or node_0 when the pin is missing.
static std::string machineNodeAt(const json& nodes, size_t i) {
    return (i < nodes.size()) ? nodes[i].get<std::string>() : std::string("node_0");
}

// Stator of a three-phase machine. Each phase x (A, B, C) is a resistor R, an inductor L and a
// controlled EMF source in series from terminal term[x] to the star node "node_<id>_N". The EMF
// source obeys V(EMF+) - V(EMF-) = signal eSignals[x]. Inductor current "I_<id>_L<x>" flows from
// the terminal into the machine. With lSignals set, the inductors are signal driven (VAR_L).
static void emitMachineStator(json& phys, const std::string& id, const std::vector<std::string>& term,
                              double R, double L, const std::vector<std::string>* lSignals,
                              const std::vector<std::string>& eSignals) {
    static const char* const phaseNames[3] = {"A", "B", "C"};
    const std::string star = "node_" + id + "_N";
    for (int x = 0; x < 3; ++x) {
        const std::string sfx = phaseNames[x];
        const std::string mid1 = "node_" + id + "_R" + sfx;
        const std::string mid2 = "node_" + id + "_L" + sfx;

        json rObj;
        rObj["id"] = id + "_R" + sfx;
        rObj["nodes"] = json::array({term[x], mid1});
        rObj["value"] = formatJSStyleDouble(R);
        rObj["esr"] = formatJSStyleDouble(0.0);
        rObj["src_type"] = "static";
        phys["resistors"].push_back(rObj);

        json lObj;
        lObj["id"] = id + "_L" + sfx;
        lObj["nodes"] = json::array({mid1, mid2});
        lObj["L"] = formatJSStyleDouble(L);
        lObj["esr"] = formatJSStyleDouble(0.0);
        lObj["iL0"] = formatJSStyleDouble(0.0);
        if (lSignals) {
            lObj["control_signal"] = (*lSignals)[x];
            lObj["src_type"] = "variable";
        } else {
            lObj["src_type"] = "static";
        }
        phys["inductors"].push_back(lObj);

        json eObj;
        eObj["id"] = id + "_E" + sfx;
        eObj["nodes"] = json::array({mid2, star});
        eObj["value"] = 1.0;
        eObj["control_signal"] = eSignals[x];
        eObj["src_type"] = "controlled";
        phys["voltage_sources"].push_back(eObj);
    }
}

std::string generateNetlistJson(const CircuitDesign& design) {
    CircuitDesign tempDesign = design;
    NetlistBuilder::buildNodesForCircuit(tempDesign);

    json root;

    json physStageObj;
    physStageObj["resistors"] = json::array();
    physStageObj["inductors"] = json::array();
    physStageObj["capacitors"] = json::array();
    physStageObj["voltage_sources"] = json::array();
    physStageObj["current_sources"] = json::array();
    physStageObj["switches"] = json::array();
    physStageObj["diodes"] = json::array();
    physStageObj["analog_switches"] = json::array();
    physStageObj["transformers"] = json::array();
    physStageObj["voltmeters"] = json::array();
    physStageObj["ammeters"] = json::array();
    physStageObj["custom_eblocks"] = json::array();
    // Magnetic domain: only the Winding gyrator needs its own category. Permeances,
    // cores, air gaps, leakage paths, magnetic resistances and MMF sources are emitted
    // into the existing capacitor / resistor / voltage-source categories, because under
    // the permeance-capacitance analogy they *are* those elements.
    physStageObj["windings"] = json::array();

    json ctrlLoopsObj;
    ctrlLoopsObj["constants"] = json::array();
    ctrlLoopsObj["gains"] = json::array();
    ctrlLoopsObj["pi_controllers"] = json::array();
    ctrlLoopsObj["pid_controllers"] = json::array();
    ctrlLoopsObj["summing_junctions"] = json::array();
    ctrlLoopsObj["pwm_generators"] = json::array();
    ctrlLoopsObj["triangle_carriers"] = json::array();
    ctrlLoopsObj["comparators"] = json::array();
    ctrlLoopsObj["logic_gates"] = json::array();
    ctrlLoopsObj["product_blocks"] = json::array();
    ctrlLoopsObj["custom_functions"] = json::array();
    ctrlLoopsObj["custom_scripts"] = json::array();
    ctrlLoopsObj["signals_routing"] = json::array();
    ctrlLoopsObj["plls"] = json::array();
    ctrlLoopsObj["probes"] = json::array();
    ctrlLoopsObj["pwm_masters"] = json::array();

    std::unordered_set<std::string> validCompIds;
    for (const auto& comp : tempDesign.components) {
        validCompIds.insert(comp.id);
    }

    std::unordered_set<std::string> seenWIds;
    int maxWNum = 0;
    for (const auto& w : tempDesign.wires) {
        if (w.id.size() > 1 && (w.id[0] == 'w' || w.id[0] == 'W')) {
            try {
                int num = std::stoi(w.id.substr(1));
                if (num > maxWNum) maxWNum = num;
            } catch (...) {}
        }
    }
    for (auto& w : tempDesign.wires) {
        if (w.id.empty() || seenWIds.count(w.id)) {
            maxWNum++;
            std::string newId = "w" + std::to_string(maxWNum);
            std::string oldId = w.id;
            w.id = newId;
            for (auto& tw : tempDesign.wires) {
                if (tw.to.isWireJunction && tw.to.targetWireId == oldId) {
                    tw.to.targetWireId = newId;
                }
            }
        }
        seenWIds.insert(w.id);
    }

    std::function<std::string(const std::string&, const std::string&)> getIncomingSignal;
    std::function<std::string(const std::string&)> getSignalFromWire;

    // ── Control-signal resolution ──────────────────────────────────────────────
    // A control input reads the output pin that drives its signal net. The net is
    // the set of pins joined by wires and by wire-to-wire junctions; its driver is
    // the (first, in wire order) output pin on it.
    //
    // This used to return whatever pin sat at the far end of the first wire whose
    // terminal matched the query, with alias groups that overlapped real pins: a
    // query for "Ctrl" also accepted an "In1" wire, so a SIGNAL_SWITCH or
    // MULTIPORT_SWITCH could read its data input as its control signal depending
    // on wire order, and a junction was followed only along its target wire's
    // "from" end, which is not necessarily the driver.
    std::unordered_map<std::string, const ComponentInstance*> compById;
    std::unordered_map<std::string, std::vector<TerminalDef>> termsById;
    for (const auto& c : tempDesign.components) {
        compById[c.id] = &c;
        termsById[c.id] = getTerminals(c);
    }
    auto upperStr = [](std::string s) {
        std::transform(s.begin(), s.end(), s.begin(), ::toupper);
        return s;
    };
    auto isRealTerminal = [&](const std::string& compId, const std::string& term) {
        auto it = termsById.find(compId);
        if (it == termsById.end()) return false;
        for (const auto& td : it->second) if (td.name == term) return true;
        return false;
    };
    auto isOutputPin = [&](const std::string& compId, const std::string& term) {
        auto it = termsById.find(compId);
        if (it == termsById.end()) return false;
        const std::string u = upperStr(term);
        for (const auto& td : it->second)
            if (upperStr(td.name) == u) return td.isControl && td.dirX > 0.0f;
        return false;
    };
    // Legacy spellings accepted for a queried pin name (exact matches are tried first).
    auto isAliasSpelling = [](const std::string& pinName, const std::string& term) {
        return (pinName == "In" && (term == "In1" || term == "Ctrl")) ||
               (pinName == "Ctrl" && (term == "In" || term == "In1")) ||
               ((pinName == "A" || pinName == "Plus") && (term == "A" || term == "Plus" || term == "In1")) ||
               ((pinName == "B" || pinName == "Minus") && (term == "B" || term == "Minus" || term == "In2"));
    };

    // Signal nets over the wires.
    std::unordered_map<std::string, std::string> sigParent;
    std::function<std::string(const std::string&)> sigFind = [&](const std::string& k) -> std::string {
        auto it = sigParent.find(k);
        if (it == sigParent.end()) { sigParent[k] = k; return k; }
        if (it->second == k) return k;
        std::string r = sigFind(it->second);
        sigParent[k] = r;
        return r;
    };
    auto sigUnite = [&](const std::string& a, const std::string& b) {
        std::string ra = sigFind(a), rb = sigFind(b);
        if (ra != rb) sigParent[ra] = rb;
    };
    for (const auto& w : tempDesign.wires) {
        const std::string wk = "w:" + w.id;
        sigFind(wk);
        for (const WireEndpoint* ep : {&w.from, &w.to}) {
            if (ep->isWireJunction) {
                if (!ep->targetWireId.empty()) sigUnite(wk, "w:" + ep->targetWireId);
            } else if (!ep->compId.empty()) {
                sigUnite(wk, "p:" + ep->compId + ":" + upperStr(ep->terminal));
            }
        }
    }
    // Driver of each net: the first output pin in wire order (with its spelling).
    std::unordered_map<std::string, std::pair<std::string, std::string>> netDriver;
    for (const auto& w : tempDesign.wires) {
        for (const WireEndpoint* ep : {&w.from, &w.to}) {
            if (ep->isWireJunction || ep->compId.empty()) continue;
            if (!isOutputPin(ep->compId, ep->terminal)) continue;
            const std::string r = sigFind("w:" + w.id);
            if (!netDriver.count(r)) netDriver[r] = {ep->compId, ep->terminal};
        }
    }

    std::unordered_set<std::string> resolving;   // guards GOTO/FROM cycles
    // Signal name for an output pin, following FROM -> GOTO.
    auto signalOfPin = [&](const std::string& compId, const std::string& term) -> std::string {
        auto it = compById.find(compId);
        if (it != compById.end()) {
            const std::string ct = upperStr(it->second->rawTypeStr);
            if (ct == "FROM_SIG" || ct == "FROM") {
                const auto& fp = it->second->parameters;
                std::string fromTag = fp.count("tag") ? fp.at("tag") : "A";
                for (const auto& g : tempDesign.components) {
                    const std::string gt = upperStr(g.rawTypeStr);
                    if (gt != "GOTO_SIG" && gt != "GOTO") continue;
                    std::string gotoTag = g.parameters.count("tag") ? g.parameters.at("tag") : "A";
                    if (gotoTag != fromTag) continue;
                    if (resolving.count(g.id)) return "0.0";
                    resolving.insert(g.id);
                    std::string s = getIncomingSignal(g.id, "In");
                    resolving.erase(g.id);
                    return s;
                }
            }
        }
        return compId + "." + term;
    };

    getSignalFromWire = [&](const std::string& wireId) -> std::string {
        auto d = netDriver.find(sigFind("w:" + wireId));
        if (d != netDriver.end()) return signalOfPin(d->second.first, d->second.second);
        // No output pin on the net: keep the old answer (the tapped wire's start).
        std::unordered_set<std::string> seen;
        std::string id = wireId;
        while (seen.insert(id).second) {
            const WireInstance* tw = nullptr;
            for (const auto& w : tempDesign.wires) if (w.id == id) { tw = &w; break; }
            if (!tw) break;
            if (tw->from.isWireJunction) { id = tw->from.targetWireId; continue; }
            if (!tw->from.compId.empty()) return signalOfPin(tw->from.compId, tw->from.terminal);
            break;
        }
        return "0.0";
    };

    getIncomingSignal = [&](const std::string& compId, const std::string& pinName) -> std::string {
        // Pass 0: the pin's own name. Pass 1: a legacy spelling, unless that spelling
        // is another real terminal of the same block (then the wire belongs to it).
        for (int pass = 0; pass < 2; ++pass) {
            for (const auto& w : tempDesign.wires) {
                auto matches = [&](const WireEndpoint& ep) {
                    if (ep.isWireJunction || ep.compId != compId) return false;
                    if (pass == 0) return ep.terminal == pinName;
                    return ep.terminal != pinName && isAliasSpelling(pinName, ep.terminal) &&
                           !isRealTerminal(compId, ep.terminal);
                };
                const bool atTo = matches(w.to);
                const bool atFrom = !atTo && matches(w.from);
                if (!atTo && !atFrom) continue;
                auto d = netDriver.find(sigFind("w:" + w.id));
                if (d != netDriver.end() && !(d->second.first == compId && upperStr(d->second.second) == upperStr(pinName)))
                    return signalOfPin(d->second.first, d->second.second);
                // No driver on the net: the far end, as before.
                const WireEndpoint& other = atTo ? w.from : w.to;
                if (other.isWireJunction) return getSignalFromWire(other.targetWireId);
                if (!other.compId.empty()) return signalOfPin(other.compId, other.terminal);
            }
        }
        return "0.0";
    };

    for (const auto& comp : tempDesign.components) {
        std::string t = comp.rawTypeStr;
        std::transform(t.begin(), t.end(), t.begin(), ::toupper);

        json cObj;
        cObj["id"] = comp.id;

        // Format nodes into "node_0", "node_1", etc.
        json formattedNodes = json::array();
        for (const auto& n : comp.nodes) {
            if (n == "0" || n == "node_0") formattedNodes.push_back("node_0");
            else if (n.rfind("node_", 0) == 0) formattedNodes.push_back(n);
            else formattedNodes.push_back("node_" + n);
        }

        // Parse all component parameters using ExpressionEvaluator::parseScientific and roundToDigits
        std::unordered_map<std::string, double> parsedParams;
        for (const auto& pair : comp.parameters) {
            double rawVal = CircuitSimEngine::ExpressionEvaluator::parseScientific(pair.second);
            parsedParams[pair.first] = roundToDigits(rawVal, 9);
        }

        if (t == "R" || t == "RESISTOR") {
            cObj["nodes"] = formattedNodes;
            // Accept "value" (native) plus "resistance"/"R" aliases used by the web tool,
            // otherwise an imported resistor silently falls back to the 10 ohm default.
            double rVal = 10.0;
            if (parsedParams.count("value")) rVal = parsedParams["value"];
            else if (parsedParams.count("resistance")) rVal = parsedParams["resistance"];
            else if (parsedParams.count("R")) rVal = parsedParams["R"];
            cObj["value"] = formatJSStyleDouble(rVal);
            cObj["esr"] = formatJSStyleDouble(parsedParams.count("esr") ? parsedParams["esr"] : 0.0);
            cObj["src_type"] = "static";
            physStageObj["resistors"].push_back(cObj);
        } else if (t == "AC_V" || t == "ACVOLTAGESOURCE") {
            cObj["nodes"] = formattedNodes;
            double amp = parsedParams.count("amplitude") ? parsedParams["amplitude"] : 12.0;
            double freq = parsedParams.count("frequency") ? parsedParams["frequency"] : 50.0;
            double phase = parsedParams.count("phase") ? parsedParams["phase"] : 0.0;
            cObj["amplitude"] = formatJSStyleDouble(roundToDigits(amp, 9));
            cObj["frequency"] = formatJSStyleDouble(roundToDigits(freq, 9));
            cObj["phase"] = formatJSStyleDouble(roundToDigits(phase, 9));
            cObj["type"] = "ac";
            cObj["src_type"] = "ac";
            physStageObj["voltage_sources"].push_back(cObj);
        } else if (t == "AC_I" || t == "ACCURRENTSOURCE") {
            cObj["nodes"] = formattedNodes;
            double amp = parsedParams.count("amplitude") ? parsedParams["amplitude"] : 1.0;
            double freq = parsedParams.count("frequency") ? parsedParams["frequency"] : 50.0;
            double phase = parsedParams.count("phase") ? parsedParams["phase"] : 0.0;
            cObj["amplitude"] = formatJSStyleDouble(roundToDigits(amp, 9));
            cObj["frequency"] = formatJSStyleDouble(roundToDigits(freq, 9));
            cObj["phase"] = formatJSStyleDouble(roundToDigits(phase, 9));
            cObj["src_type"] = "ac";
            physStageObj["current_sources"].push_back(cObj);
        } else if (t == "CTRL_V" || t == "CONTROLLEDVOLTAGESOURCE") {
            cObj["nodes"] = formattedNodes;
            cObj["value"] = formatJSStyleDouble(parsedParams.count("value") ? parsedParams["value"] : 1.0);
            cObj["control_signal"] = getIncomingSignal(comp.id, "Ctrl");
            cObj["src_type"] = "controlled";
            physStageObj["voltage_sources"].push_back(cObj);
        } else if (t == "CTRL_I" || t == "CONTROLLEDCURRENTSOURCE") {
            cObj["nodes"] = formattedNodes;
            cObj["value"] = formatJSStyleDouble(parsedParams.count("value") ? parsedParams["value"] : 1.0);
            cObj["control_signal"] = getIncomingSignal(comp.id, "Ctrl");
            cObj["src_type"] = "controlled";
            physStageObj["current_sources"].push_back(cObj);
        } else if (t == "V_3PH" || t == "THREEPHASESOURCE") {
            std::string conn = comp.parameters.count("connection") ? comp.parameters.at("connection") : "Y";
            double amp = parsedParams.count("magnitude") ? parsedParams["magnitude"] : (parsedParams.count("amplitude") ? parsedParams["amplitude"] : 230.0);
            double freq = parsedParams.count("frequency") ? parsedParams["frequency"] : 50.0;
            double phase = parsedParams.count("phase") ? parsedParams["phase"] : 0.0;
            
            std::string nodeA = (formattedNodes.size() > 0) ? formattedNodes[0].get<std::string>() : "node_0";
            std::string nodeB = (formattedNodes.size() > 1) ? formattedNodes[1].get<std::string>() : "node_0";
            std::string nodeC = (formattedNodes.size() > 2) ? formattedNodes[2].get<std::string>() : "node_0";
            std::string nodeN = (formattedNodes.size() > 3) ? formattedNodes[3].get<std::string>() : (comp.id + "_N");

            if (conn == "Delta") {
                // Only two independent sources are emitted. Three ideal sources wired in a
                // closed delta form a KVL loop of voltage sources, which is rank-deficient
                // and makes the MNA matrix singular. With V_AB and V_BC defined, V_CA is
                // implied as -(V_AB + V_BC), which is exact for a balanced set.
                json vsAB; vsAB["id"] = comp.id + "_AB"; vsAB["nodes"] = json::array({nodeA, nodeB}); vsAB["amplitude"] = formatJSStyleDouble(amp); vsAB["frequency"] = formatJSStyleDouble(freq); vsAB["phase"] = formatJSStyleDouble(phase); vsAB["type"] = "ac"; physStageObj["voltage_sources"].push_back(vsAB);
                json vsBC; vsBC["id"] = comp.id + "_BC"; vsBC["nodes"] = json::array({nodeB, nodeC}); vsBC["amplitude"] = formatJSStyleDouble(amp); vsBC["frequency"] = formatJSStyleDouble(freq); vsBC["phase"] = formatJSStyleDouble(phase - 120.0); vsBC["type"] = "ac"; physStageObj["voltage_sources"].push_back(vsBC);
            } else {
                json vsA; vsA["id"] = comp.id + "_A"; vsA["nodes"] = json::array({nodeA, nodeN}); vsA["amplitude"] = formatJSStyleDouble(amp); vsA["frequency"] = formatJSStyleDouble(freq); vsA["phase"] = formatJSStyleDouble(phase); vsA["type"] = "ac"; physStageObj["voltage_sources"].push_back(vsA);
                json vsB; vsB["id"] = comp.id + "_B"; vsB["nodes"] = json::array({nodeB, nodeN}); vsB["amplitude"] = formatJSStyleDouble(amp); vsB["frequency"] = formatJSStyleDouble(freq); vsB["phase"] = formatJSStyleDouble(phase - 120.0); vsB["type"] = "ac"; physStageObj["voltage_sources"].push_back(vsB);
                json vsC; vsC["id"] = comp.id + "_C"; vsC["nodes"] = json::array({nodeC, nodeN}); vsC["amplitude"] = formatJSStyleDouble(amp); vsC["frequency"] = formatJSStyleDouble(freq); vsC["phase"] = formatJSStyleDouble(phase + 120.0); vsC["type"] = "ac"; physStageObj["voltage_sources"].push_back(vsC);
            }
        } else if (t == "I_3PH" || t == "THREEPHASECURRENTSOURCE") {
            double amp = parsedParams.count("amplitude") ? parsedParams["amplitude"] : 1.0;
            std::string nodeA = (formattedNodes.size() > 0) ? formattedNodes[0].get<std::string>() : "node_0";
            std::string nodeB = (formattedNodes.size() > 1) ? formattedNodes[1].get<std::string>() : "node_0";
            std::string nodeC = (formattedNodes.size() > 2) ? formattedNodes[2].get<std::string>() : "node_0";
            json csA; csA["id"] = comp.id + "_A"; csA["nodes"] = json::array({nodeA, "node_0"}); csA["value"] = formatJSStyleDouble(amp); csA["src_type"] = "dc"; physStageObj["current_sources"].push_back(csA);
            json csB; csB["id"] = comp.id + "_B"; csB["nodes"] = json::array({nodeB, "node_0"}); csB["value"] = formatJSStyleDouble(amp); csB["src_type"] = "dc"; physStageObj["current_sources"].push_back(csB);
            json csC; csC["id"] = comp.id + "_C"; csC["nodes"] = json::array({nodeC, "node_0"}); csC["value"] = formatJSStyleDouble(amp); csC["src_type"] = "dc"; physStageObj["current_sources"].push_back(csC);
        } else if (t == "VM_3PH" || t == "VOLTMETER3PH") {
            cObj["nodes"] = formattedNodes;
            cObj["signal"] = comp.id + ".Out";
            physStageObj["voltmeters"].push_back(cObj);
        } else if (t == "AM_3PH" || t == "AMMETER3PH") {
            cObj["nodes"] = formattedNodes;
            cObj["signal"] = comp.id + ".Out";
            physStageObj["ammeters"].push_back(cObj);
        } else if (t == "VAR_R" || t == "PWL_R" || t == "E_ALGEBRAIC") {
            cObj["nodes"] = formattedNodes;
            cObj["value"] = formatJSStyleDouble(parsedParams.count("value") ? parsedParams["value"] : 10.0);
            cObj["esr"] = formatJSStyleDouble(parsedParams.count("esr") ? parsedParams["esr"] : 0.0);
            cObj["control_signal"] = getIncomingSignal(comp.id, "Ctrl");
            cObj["src_type"] = (t == "VAR_R") ? "variable" : "static";
            physStageObj["resistors"].push_back(cObj);
        } else if (t == "VAR_L" || t == "SAT_L") {
            cObj["nodes"] = formattedNodes;
            cObj["L"] = formatJSStyleDouble(parsedParams.count("L") ? parsedParams["L"] : 0.01);
            cObj["esr"] = formatJSStyleDouble(parsedParams.count("esr") ? parsedParams["esr"] : 0.0);
            cObj["iL0"] = formatJSStyleDouble(parsedParams.count("iL0") ? parsedParams["iL0"] : 0.0);
            cObj["control_signal"] = getIncomingSignal(comp.id, "Ctrl");
            // "variable" marks a signal-driven value; saturable elements are not.
            cObj["src_type"] = (t == "VAR_L") ? "variable" : "static";
            physStageObj["inductors"].push_back(cObj);
        } else if (t == "VAR_C" || t == "SAT_C") {
            cObj["nodes"] = formattedNodes;
            cObj["C"] = formatJSStyleDouble(parsedParams.count("C") ? parsedParams["C"] : 1e-4);
            cObj["esr"] = formatJSStyleDouble(parsedParams.count("esr") ? parsedParams["esr"] : 0.0);
            cObj["vC0"] = formatJSStyleDouble(parsedParams.count("vC0") ? parsedParams["vC0"] : 0.0);
            cObj["control_signal"] = getIncomingSignal(comp.id, "Ctrl");
            cObj["src_type"] = (t == "VAR_C") ? "variable" : "static";
            physStageObj["capacitors"].push_back(cObj);

        // ─────────────────────── MAGNETIC DOMAIN BLOCKS ───────────────────────
        // Permeance-capacitance analogy: MMF <-> node potential, flux rate <-> branch
        // current, permeance <-> capacitance, magnetic conductance <-> conductance.
        // This lets the existing MNA solver integrate magnetic states directly, with no
        // separate magnetic solver and no algebraic loops.
        } else if (t == "WINDING") {
            // Electrical <-> magnetic gyrator. nodes = [E+, E-, M+, M-]
            cObj["type"] = "WINDING";
            cObj["nodes"] = formattedNodes;
            double turns = 1.0;
            if (parsedParams.count("N")) turns = parsedParams["N"];
            else if (parsedParams.count("turns")) turns = parsedParams["turns"];
            if (std::abs(turns) < 1e-12) turns = 1.0;
            cObj["N"] = formatJSStyleDouble(turns);
            physStageObj["windings"].push_back(cObj);

        } else if (t == "MAG_PERMEANCE" || t == "LINEAR_CORE" || t == "AIR_GAP" || t == "LEAKAGE_PATH") {
            // All four are a permeance P, which stamps as a capacitance of value P.
            // Initial MMF maps onto the capacitor's initial voltage.
            const double MU0 = 4.0 * 3.14159265358979323846e-7;
            double P = 1e-6;
            if (t == "LINEAR_CORE") {
                double A   = parsedParams.count("A") ? parsedParams["A"] : 1e-4;
                double len = parsedParams.count("l") ? parsedParams["l"] : 0.1;
                double mur = parsedParams.count("mu_r") ? parsedParams["mu_r"] : 1000.0;
                if (len <= 0.0) len = 1e-6;
                P = MU0 * mur * A / len;
            } else if (t == "AIR_GAP") {
                double A   = parsedParams.count("A") ? parsedParams["A"] : 1e-4;
                double len = parsedParams.count("l") ? parsedParams["l"] : 1e-3;
                if (len <= 0.0) len = 1e-6;
                P = MU0 * A / len;
            } else {
                // MAG_PERMEANCE / LEAKAGE_PATH take the permeance directly.
                if (parsedParams.count("P")) P = parsedParams["P"];
                else if (parsedParams.count("permeance")) P = parsedParams["permeance"];
            }
            if (P <= 0.0) P = 1e-12;

            // Permeance is a static conductance: flux = P * (MMF - F0). Modelled as a resistor of
            // 1 / P. It used to be a capacitor, which passes no current at DC, so these
            // elements carried no steady flux. F0 is a permanent-magnet MMF in series with the
            // reluctance, so it gets its own DC source and an internal node.
            const double F0 = parsedParams.count("F0") ? parsedParams["F0"] : 0.0;
            if (F0 == 0.0) {
                cObj["type"] = "Resistor";
                cObj["nodes"] = formattedNodes;
                cObj["value"] = formatJSStyleDouble(1.0 / P);
                cObj["esr"] = formatJSStyleDouble(0.0);
                cObj["src_type"] = "static";
                physStageObj["resistors"].push_back(cObj);
            } else {
                const std::string midNode = "node_" + comp.id + "_F0";
                json rObj;
                rObj["id"] = comp.id;
                rObj["type"] = "Resistor";
                rObj["nodes"] = json::array({formattedNodes[0], midNode});
                rObj["value"] = formatJSStyleDouble(1.0 / P);
                rObj["esr"] = formatJSStyleDouble(0.0);
                rObj["src_type"] = "static";
                physStageObj["resistors"].push_back(rObj);
                json vObj;
                vObj["id"] = comp.id + "_F0";
                vObj["type"] = "VoltageSource";
                vObj["nodes"] = json::array({midNode, formattedNodes[1]});
                vObj["value"] = formatJSStyleDouble(F0);
                vObj["src_type"] = "dc";
                physStageObj["voltage_sources"].push_back(vObj);
            }

        } else if (t == "MAG_RESISTANCE") {
            // PhiDot = G_fe * F, i.e. a resistor of value R_m = 1 / G_fe.
            double Rm = 1e6;
            if (parsedParams.count("Rm")) Rm = parsedParams["Rm"];
            else if (parsedParams.count("R")) Rm = parsedParams["R"];
            else if (parsedParams.count("Gfe") && parsedParams["Gfe"] > 0.0) Rm = 1.0 / parsedParams["Gfe"];
            if (Rm <= 0.0) Rm = 1e-6;

            cObj["type"] = "Resistor";
            cObj["nodes"] = formattedNodes;
            cObj["value"] = formatJSStyleDouble(Rm);
            cObj["esr"] = formatJSStyleDouble(0.0);
            cObj["src_type"] = "static";
            physStageObj["resistors"].push_back(cObj);

        } else if (t == "MMF_SRC") {
            // Constant MMF across the terminals == a DC voltage source in the analogy.
            cObj["type"] = "VoltageSource";
            cObj["nodes"] = formattedNodes;
            double F = 0.0;
            if (parsedParams.count("F")) F = parsedParams["F"];
            else if (parsedParams.count("mmf")) F = parsedParams["mmf"];
            else if (parsedParams.count("value")) F = parsedParams["value"];
            cObj["value"] = formatJSStyleDouble(F);
            cObj["src_type"] = "dc";
            physStageObj["voltage_sources"].push_back(cObj);

        } else if (t == "MMF_SRC_CTRL") {
            // Signal-driven MMF == controlled voltage source in the analogy.
            cObj["type"] = "CTRL_V";
            cObj["nodes"] = formattedNodes;
            cObj["value"] = formatJSStyleDouble(parsedParams.count("gain") ? parsedParams["gain"] : 1.0);
            cObj["control_signal"] = getIncomingSignal(comp.id, "Ctrl");
            cObj["src_type"] = "controlled";
            physStageObj["voltage_sources"].push_back(cObj);
        } else if (t == "D" || t == "DIODE") {
            cObj["type"] = "Diode";
            cObj["nodes"] = formattedNodes;
            cObj["Vd"] = formatJSStyleDouble(parsedParams.count("Vd") ? parsedParams["Vd"] : 0.7);
            double rOn = parsedParams.count("Ron") ? parsedParams["Ron"] : 0.001;
            double rOff = parsedParams.count("Roff") ? parsedParams["Roff"] : 1000000.0;
            if (rOff < rOn * 1e4 || rOff <= 1.0) rOff = 1000000.0;
            cObj["Ron"] = formatJSStyleDouble(rOn);
            cObj["Roff"] = formatJSStyleDouble(rOff);
            physStageObj["diodes"].push_back(cObj);
        } else if (t == "MAN_SWITCH" || t == "DBL_SWITCH" || t == "MAN_DBL_SWITCH" || t == "MAN_TRPL_SWITCH" || t == "TRPL_SWITCH") {
            // Multi-pole switches. Each pole is a two-terminal switch between two pins, looked
            // up by name. The old code took the first two nodes by position, which left the B
            // throw and the other poles unconnected.
            auto nodeOf = [&](const std::string& pin) -> json {
                auto it = termsById.find(comp.id);
                if (it != termsById.end()) {
                    for (size_t i = 0; i < it->second.size() && i < formattedNodes.size(); ++i)
                        if (it->second[i].name == pin) return formattedNodes[i];
                }
                return "node_0";
            };
            const double rOnM = parsedParams.count("Ron") ? parsedParams["Ron"] : 0.001;
            double rOffM = parsedParams.count("Roff") ? parsedParams["Roff"] : 1000000.0;
            if (rOffM < rOnM * 1e4 || rOffM <= 1.0) rOffM = 1000000.0;
            const std::string stateStr = comp.parameters.count("state") ? comp.parameters.at("state") : "";

            auto addPole = [&](const std::string& suffix, const std::string& a, const std::string& b, const std::string& controlKey) {
                json sw;
                sw["id"] = comp.id + "_" + suffix;
                sw["type"] = "Switch";
                sw["nodes"] = json::array({nodeOf(a), nodeOf(b)});
                sw["Ron"] = formatJSStyleDouble(rOnM);
                sw["Roff"] = formatJSStyleDouble(rOffM);
                sw["initial_state"] = false;
                sw["control_signal"] = controlKey;
                physStageObj["switches"].push_back(sw);
            };
            // A constant control signal holds a manual switch in its chosen state.
            auto addConstant = [&](const std::string& key, bool closed) {
                json k;
                k["output"] = key;
                k["original_type"] = "CONST";
                k["value"] = closed ? 1.0 : 0.0;
                ctrlLoopsObj["constants"].push_back(k);
            };

            if (t == "MAN_SWITCH" || t == "DBL_SWITCH") {
                // Single-pole double-throw: Common goes to A when state is "A", to B when "B".
                const bool toA = (stateStr != "B");
                addConstant(comp.id + ".ctlA", toA);
                addConstant(comp.id + ".ctlB", !toA);
                addPole("A", "Common", "A", comp.id + ".ctlA");
                addPole("B", "Common", "B", comp.id + ".ctlB");
            } else if (t == "MAN_DBL_SWITCH") {
                // Two ganged single-pole switches (A1-B1 and A2-B2), open or closed together.
                const bool closed = (stateStr != "open");
                addConstant(comp.id + ".ctl", closed);
                addPole("1", "A1", "B1", comp.id + ".ctl");
                addPole("2", "A2", "B2", comp.id + ".ctl");
            } else {
                // MAN_TRPL_SWITCH and TRPL_SWITCH: three ganged poles driven by the Ctrl pin.
                const std::string ctrl = getIncomingSignal(comp.id, "Ctrl");
                addPole("1", "A1", "B1", ctrl);
                addPole("2", "A2", "B2", ctrl);
                addPole("3", "A3", "B3", ctrl);
            }
        } else if (t == "S" || t == "BREAKER" || t == "SR_SWITCH") {
            cObj["type"] = "Switch";
            json pNodes = json::array();
            if (formattedNodes.size() >= 2) {
                pNodes.push_back(formattedNodes[0]);
                pNodes.push_back(formattedNodes[1]);
            } else pNodes = formattedNodes;
            cObj["nodes"] = pNodes;
            double rOn = parsedParams.count("Ron") ? parsedParams["Ron"] : 0.001;
            double rOff = parsedParams.count("Roff") ? parsedParams["Roff"] : 1000000.0;
            if (rOff < rOn * 1e4 || rOff <= 1.0) rOff = 1000000.0;
            cObj["Ron"] = formatJSStyleDouble(rOn);
            cObj["Roff"] = formatJSStyleDouble(rOff);
            cObj["initial_state"] = false;
            cObj["control_signal"] = getIncomingSignal(comp.id, "Ctrl");
            physStageObj["switches"].push_back(cObj);
        } else if (t == "MOSFET" || t == "IGBT" || t == "VG-FET" || t == "VGFET" || t == "IGBT_DIODE" || t == "IGCT" || t == "GTO" || t == "THYRISTOR" || t == "JFET" || t == "BJT") {
            bool isBJT = (t == "BJT");
            std::string termG = isBJT ? "B" : "G";

            cObj["type"] = (t == "VG-FET" || t == "VGFET") ? "MOSFET" : comp.rawTypeStr;
            json pNodes = json::array();
            if (formattedNodes.size() >= 2) {
                pNodes.push_back(formattedNodes[0]);
                pNodes.push_back(formattedNodes[1]);
            } else pNodes = formattedNodes;
            cObj["nodes"] = pNodes;
            cObj["control_node"] = comp.id + "." + termG;
            if (t == "VG-FET" || t == "VGFET") {
                std::string gateTag = comp.parameters.count("Gate_Signal_Label") ? comp.parameters.at("Gate_Signal_Label") : (comp.parameters.count("tag") ? comp.parameters.at("tag") : "S1");
                std::string sig = "0.0";
                for (const auto& g : tempDesign.components) {
                    std::string gt = g.rawTypeStr;
                    std::transform(gt.begin(), gt.end(), gt.begin(), ::toupper);
                    if (gt == "GOTO_SIG" || gt == "GOTO") {
                        std::string gotoTag = g.parameters.count("tag") ? g.parameters.at("tag") : "A";
                        if (gotoTag == gateTag) {
                            sig = getIncomingSignal(g.id, "In");
                            break;
                        }
                    }
                }
                // A GOTO with the gate label takes priority. With no such GOTO, a wire into G
                // drives the gate, as it does for the other FET types.
                if (sig == "0.0") sig = getIncomingSignal(comp.id, termG);
                cObj["control_signal"] = sig;
            } else {
                cObj["control_signal"] = getIncomingSignal(comp.id, termG);
            }

            double rOn = parsedParams.count("Ron") ? parsedParams["Ron"] : 0.001;
            double rOff = parsedParams.count("Roff") ? parsedParams["Roff"] : 1000000.0;
            if (rOff < rOn * 1e4 || rOff <= 1.0) rOff = 1000000.0;
            cObj["Ron"] = formatJSStyleDouble(rOn);
            cObj["Roff"] = formatJSStyleDouble(rOff);
            cObj["Vd"] = formatJSStyleDouble(parsedParams.count("Vd") ? parsedParams["Vd"] : (parsedParams.count("Vf") ? parsedParams["Vf"] : 0.8));
            cObj["Iholding"] = formatJSStyleDouble(parsedParams.count("Iholding") ? parsedParams["Iholding"] : (parsedParams.count("Ih") ? parsedParams["Ih"] : 0.01));
            cObj["Vgt"] = formatJSStyleDouble(parsedParams.count("Vgt") ? parsedParams["Vgt"] : 0.5);
            physStageObj["analog_switches"].push_back(cObj);
        } else if (t == "XFMR" || t == "IDEAL_XFMR" || t == "XFMR_2W" || t == "SAT_XFMR" || t == "MUTUAL_2W" || t == "Transformer" || t == "IdealTransformer" || t == "IDEAL_TRANSFORMER") {
            json xObj;
            xObj["id"] = comp.id;

            std::string pStr = comp.parameters.count("primary_turns") ? comp.parameters.at("primary_turns") : "[100]";
            std::string sStr = comp.parameters.count("secondary_turns") ? comp.parameters.at("secondary_turns") : "[100]";
            auto pTurns = parseTurnsArrayStr(pStr);
            auto sTurns = parseTurnsArrayStr(sStr);

            json pArr = json::array();
            for (size_t i = 0; i < pTurns.size(); ++i) {
                int idxA = (int)i * 2;
                int idxB = (int)i * 2 + 1;
                std::string nA = (idxA < (int)formattedNodes.size()) ? formattedNodes[idxA].get<std::string>() : "node_0";
                std::string nB = (idxB < (int)formattedNodes.size()) ? formattedNodes[idxB].get<std::string>() : "node_0";
                pArr.push_back({{"nodes", json::array({nA, nB})}, {"turns", pTurns[i]}});
            }

            int pNodeOffset = (int)pTurns.size() * 2;
            json sArr = json::array();
            for (size_t j = 0; j < sTurns.size(); ++j) {
                int idxA = pNodeOffset + (int)j * 2;
                int idxB = pNodeOffset + (int)j * 2 + 1;
                std::string nA = (idxA < (int)formattedNodes.size()) ? formattedNodes[idxA].get<std::string>() : "node_0";
                std::string nB = (idxB < (int)formattedNodes.size()) ? formattedNodes[idxB].get<std::string>() : "node_0";
                sArr.push_back({{"nodes", json::array({nA, nB})}, {"turns", sTurns[j]}});
            }

            xObj["primary_windings"] = pArr;
            xObj["secondary_windings"] = sArr;
            xObj["core_permeability"] = formatJSStyleDouble(parsedParams.count("permeability") ? parsedParams["permeability"] : 2000.0);
            if (comp.parameters.count("polarity")) xObj["polarity"] = comp.parameters.at("polarity");

            physStageObj["transformers"].push_back(xObj);

            // The winding stamp is an ideal transformer, so the magnetising inductance
            // the multi-winding XFMR declares is a separate inductor across the first
            // primary winding (its current is the magnetising current referred to
            // that winding). Lm <= 0 or absent keeps the transformer ideal.
            if ((t == "XFMR" || t == "Transformer") && parsedParams.count("Lm") && parsedParams["Lm"] > 0.0 &&
                std::isfinite(parsedParams["Lm"]) && !pArr.empty()) {
                json lObj;
                lObj["id"] = comp.id + "_Lm";
                lObj["nodes"] = pArr[0]["nodes"];
                lObj["L"] = formatJSStyleDouble(parsedParams["Lm"]);
                lObj["esr"] = formatJSStyleDouble(0.0);
                lObj["iL0"] = formatJSStyleDouble(0.0);
                physStageObj["inductors"].push_back(lObj);
            }
        } else if (t == "XFMR_3W" || t == "MUTUAL_3W" || t == "XFMR_3PH_2W" || t == "XFMR_3PH_3W") {
            json xObj;
            xObj["id"] = comp.id;
            std::string nP1 = (formattedNodes.size() > 0) ? formattedNodes[0].get<std::string>() : "node_0";
            std::string nP2 = (formattedNodes.size() > 1) ? formattedNodes[1].get<std::string>() : "node_0";
            std::string nS1 = (formattedNodes.size() > 2) ? formattedNodes[2].get<std::string>() : "node_0";
            std::string nS2 = (formattedNodes.size() > 3) ? formattedNodes[3].get<std::string>() : "node_0";
            xObj["primary_windings"] = json::array({{{"nodes", json::array({nP1, nP2})}, {"turns", 100}}});
            xObj["secondary_windings"] = json::array({{{"nodes", json::array({nS1, nS2})}, {"turns", 100}}});
            xObj["core_permeability"] = formatJSStyleDouble(parsedParams.count("permeability") ? parsedParams["permeability"] : 2000.0);
            physStageObj["transformers"].push_back(xObj);
        } else if (t == "OPAMP" || t == "E_COMP") {
            // Finite-gain amplifier with rails, built from parts the engine already has:
            //   sense:  voltmeter <id>_sense across Plus and Minus, signal <id>_sense.Out
            //   gain:   K = gain, signal <id>.Ag
            //   clamp:  saturation to +/- rail, signal <id>.Sat
            //   drive:  controlled source from Out to ground, control <id>.Sat
            // This replaces a fixed 12 V source that also had its pins in the wrong order.
            auto nodeOf = [&](const std::string& pin) -> json {
                auto it = termsById.find(comp.id);
                if (it != termsById.end()) {
                    for (size_t i = 0; i < it->second.size() && i < formattedNodes.size(); ++i)
                        if (it->second[i].name == pin) return formattedNodes[i];
                }
                return "node_0";
            };
            const double opGain = parsedParams.count("gain") ? parsedParams["gain"] : 1e5;
            const double opRail = parsedParams.count("rail") ? parsedParams["rail"] : 12.0;
            // A voltmeter names its output after its own id, so the sense signal is <id>_sense.Out.
            const std::string opSense = comp.id + "_sense.Out";
            const std::string opAmp = comp.id + ".Ag";
            const std::string opSat = comp.id + ".Sat";

            json vm;
            vm["id"] = comp.id + "_sense";
            vm["nodes"] = json::array({nodeOf("Plus"), nodeOf("Minus")});
            physStageObj["voltmeters"].push_back(vm);

            json gainObj;
            gainObj["output"] = opAmp;
            gainObj["original_type"] = "GAIN";
            gainObj["input"] = opSense;
            gainObj["input1"] = opSense;
            gainObj["input2"] = "0.0";
            gainObj["K"] = formatJSStyleDouble(opGain);
            ctrlLoopsObj["gains"].push_back(gainObj);

            json satObj;
            satObj["output"] = opSat;
            satObj["original_type"] = "SATURATION";
            satObj["min"] = formatJSStyleDouble(-opRail);
            satObj["max"] = formatJSStyleDouble(opRail);
            satObj["lower_limit"] = formatJSStyleDouble(-opRail);
            satObj["upper_limit"] = formatJSStyleDouble(opRail);
            satObj["input"] = opAmp;
            satObj["input1"] = opAmp;
            satObj["input2"] = "0.0";
            satObj["K"] = 1.0;
            ctrlLoopsObj["gains"].push_back(satObj);

            json drive;
            drive["id"] = comp.id;
            drive["nodes"] = json::array({nodeOf("Out"), "node_0"});
            drive["value"] = 1.0;
            drive["control_signal"] = opSat;
            drive["src_type"] = "controlled";
            physStageObj["voltage_sources"].push_back(drive);
        } else if (t == "GEN_EBLOCK" || t == "GENERALIZEDEBLOCK") {
            json eObj;
            eObj["id"] = comp.id;
            eObj["nodes"] = formattedNodes;
            eObj["code"] = comp.parameters.count("code") ? comp.parameters.at("code") : "";
            eObj["timestep"] = comp.parameters.count("timestep") ? comp.parameters.at("timestep") : "0";
            eObj["terminals"] = std::to_string(formattedNodes.size());
            physStageObj["custom_eblocks"].push_back(eObj);
        } else if (t == "DC_MOTOR") {
            // DC machine. The electrical side is the armature R and L in series with a back-EMF
            // source that tracks Ke * speed: V(A) - V(B) = Ra i + La di/dt + Ke omega. The
            // mechanical side is one control block that integrates J domega/dt = Kt i - TL - B omega.
            // TL is the load torque signal. Speed is the block output.
            const std::string dmA = (formattedNodes.size() > 0) ? formattedNodes[0].get<std::string>() : "node_0";
            const std::string dmB = (formattedNodes.size() > 1) ? formattedNodes[1].get<std::string>() : "node_0";
            const std::string dmMid1 = "node_" + comp.id + "_Ra";
            const std::string dmMid2 = "node_" + comp.id + "_La";
            const double dmRa = parsedParams.count("Ra") ? parsedParams["Ra"] : 1.0;
            const double dmLa = parsedParams.count("La") ? parsedParams["La"] : 1e-3;

            json rObj;
            rObj["id"] = comp.id + "_Ra";
            rObj["nodes"] = json::array({dmA, dmMid1});
            rObj["value"] = formatJSStyleDouble(dmRa);
            rObj["esr"] = formatJSStyleDouble(0.0);
            rObj["src_type"] = "static";
            physStageObj["resistors"].push_back(rObj);

            json lObj;
            lObj["id"] = comp.id + "_La";
            lObj["nodes"] = json::array({dmMid1, dmMid2});
            lObj["L"] = formatJSStyleDouble(dmLa);
            lObj["esr"] = formatJSStyleDouble(0.0);
            lObj["iL0"] = formatJSStyleDouble(0.0);
            lObj["src_type"] = "static";
            physStageObj["inductors"].push_back(lObj);

            json eObj;
            eObj["id"] = comp.id + "_Emf";
            eObj["nodes"] = json::array({dmMid2, dmB});
            eObj["value"] = 1.0;
            eObj["control_signal"] = comp.id + ".Emf";
            eObj["src_type"] = "controlled";
            physStageObj["voltage_sources"].push_back(eObj);

            cObj["output"] = comp.id + ".Speed";
            cObj["original_type"] = "DC_MOTOR";
            cObj["input"] = getIncomingSignal(comp.id, "TL");
            cObj["current"] = "I_" + comp.id + "_La";
            cObj["Kt"] = formatJSStyleDouble(parsedParams.count("Kt") ? parsedParams["Kt"] : 0.05);
            cObj["Ke"] = formatJSStyleDouble(parsedParams.count("Ke") ? parsedParams["Ke"] : 0.05);
            cObj["J"] = formatJSStyleDouble(parsedParams.count("J") ? parsedParams["J"] : 1e-4);
            cObj["B"] = formatJSStyleDouble(parsedParams.count("B") ? parsedParams["B"] : 1e-4);
            cObj["omega0"] = formatJSStyleDouble(parsedParams.count("omega0") ? parsedParams["omega0"] : 0.0);
            ctrlLoopsObj["gains"].push_back(cObj);
        } else if (t == "BLDC_MOTOR" || t == "PMSM_MOTOR") {
            // Permanent-magnet three-phase machine. Each phase is R, L and a controlled back-EMF
            // Ke * omega * shape(theta). BLDC uses a trapezoidal shape and PMSM a sine. The
            // mechanical equation and the rotor angle live in the control block.
            const std::vector<std::string> term = {machineNodeAt(formattedNodes, 0),
                                                   machineNodeAt(formattedNodes, 1),
                                                   machineNodeAt(formattedNodes, 2)};
            const double mR = parsedParams.count("Rs") ? parsedParams["Rs"] : 1.0;
            const double mL = parsedParams.count("Ls") ? parsedParams["Ls"] : 1e-3;
            const std::vector<std::string> eSig = {comp.id + ".Ea", comp.id + ".Eb", comp.id + ".Ec"};
            emitMachineStator(physStageObj, comp.id, term, mR, mL, nullptr, eSig);

            cObj["output"] = comp.id + ".Speed";
            cObj["original_type"] = t;
            cObj["input"] = getIncomingSignal(comp.id, "TL");
            cObj["current_a"] = "I_" + comp.id + "_LA";
            cObj["current_b"] = "I_" + comp.id + "_LB";
            cObj["current_c"] = "I_" + comp.id + "_LC";
            cObj["P"] = formatJSStyleDouble(parsedParams.count("P") ? parsedParams["P"] : 2.0);
            cObj["Ke"] = formatJSStyleDouble(parsedParams.count("Ke") ? parsedParams["Ke"] : 0.05);
            cObj["J"] = formatJSStyleDouble(parsedParams.count("J") ? parsedParams["J"] : 1e-4);
            cObj["B"] = formatJSStyleDouble(parsedParams.count("B") ? parsedParams["B"] : 1e-4);
            cObj["omega0"] = formatJSStyleDouble(parsedParams.count("omega0") ? parsedParams["omega0"] : 0.0);
            cObj["theta0"] = formatJSStyleDouble(parsedParams.count("theta0") ? parsedParams["theta0"] : 0.0);
            ctrlLoopsObj["gains"].push_back(cObj);
        } else if (t == "SYNC_MOTOR") {
            // Wound-field synchronous machine. The stator is as for the PM machines, with EMF
            // Kf * i_field * omega * sin(theta). The field winding FA-FB is R_f and L_f in series,
            // and its current I_<id>_Lf drives the EMF amplitude.
            const std::vector<std::string> term = {machineNodeAt(formattedNodes, 0),
                                                   machineNodeAt(formattedNodes, 1),
                                                   machineNodeAt(formattedNodes, 2)};
            const std::string fA = machineNodeAt(formattedNodes, 3);
            const std::string fB = machineNodeAt(formattedNodes, 4);
            const double mR = parsedParams.count("Rs") ? parsedParams["Rs"] : 1.0;
            const double mL = parsedParams.count("Ls") ? parsedParams["Ls"] : 1e-3;
            const double rf = parsedParams.count("Rf") ? parsedParams["Rf"] : 10.0;
            const double lf = parsedParams.count("Lf") ? parsedParams["Lf"] : 0.5;
            const std::vector<std::string> eSig = {comp.id + ".Ea", comp.id + ".Eb", comp.id + ".Ec"};
            emitMachineStator(physStageObj, comp.id, term, mR, mL, nullptr, eSig);

            const std::string fMid = "node_" + comp.id + "_Rf";
            json rfObj;
            rfObj["id"] = comp.id + "_Rf";
            rfObj["nodes"] = json::array({fA, fMid});
            rfObj["value"] = formatJSStyleDouble(rf);
            rfObj["esr"] = formatJSStyleDouble(0.0);
            rfObj["src_type"] = "static";
            physStageObj["resistors"].push_back(rfObj);

            json lfObj;
            lfObj["id"] = comp.id + "_Lf";
            lfObj["nodes"] = json::array({fMid, fB});
            lfObj["L"] = formatJSStyleDouble(lf);
            lfObj["esr"] = formatJSStyleDouble(0.0);
            lfObj["iL0"] = formatJSStyleDouble(0.0);
            lfObj["src_type"] = "static";
            physStageObj["inductors"].push_back(lfObj);

            cObj["output"] = comp.id + ".Speed";
            cObj["original_type"] = t;
            cObj["input"] = getIncomingSignal(comp.id, "TL");
            cObj["current_a"] = "I_" + comp.id + "_LA";
            cObj["current_b"] = "I_" + comp.id + "_LB";
            cObj["current_c"] = "I_" + comp.id + "_LC";
            cObj["current_f"] = "I_" + comp.id + "_Lf";
            cObj["P"] = formatJSStyleDouble(parsedParams.count("P") ? parsedParams["P"] : 2.0);
            cObj["Kf"] = formatJSStyleDouble(parsedParams.count("Kf") ? parsedParams["Kf"] : 0.05);
            cObj["J"] = formatJSStyleDouble(parsedParams.count("J") ? parsedParams["J"] : 1e-4);
            cObj["B"] = formatJSStyleDouble(parsedParams.count("B") ? parsedParams["B"] : 1e-4);
            cObj["omega0"] = formatJSStyleDouble(parsedParams.count("omega0") ? parsedParams["omega0"] : 0.0);
            cObj["theta0"] = formatJSStyleDouble(parsedParams.count("theta0") ? parsedParams["theta0"] : 0.0);
            ctrlLoopsObj["gains"].push_back(cObj);
        } else if (t == "SRM_MOTOR") {
            // Switched reluctance machine. Each phase inductance is signal driven (VAR_L) from the
            // control block, L_x(theta) = Lavg + Ldiff cos(theta_x). The EMF i dL/dt is a
            // controlled source fed by the same block. The converter that switches each phase is
            // external to this block.
            const std::vector<std::string> term = {machineNodeAt(formattedNodes, 0),
                                                   machineNodeAt(formattedNodes, 1),
                                                   machineNodeAt(formattedNodes, 2)};
            const double mR = parsedParams.count("Rs") ? parsedParams["Rs"] : 0.5;
            const double lmin = parsedParams.count("Lmin") ? parsedParams["Lmin"] : 2e-3;
            const double lmax = parsedParams.count("Lmax") ? parsedParams["Lmax"] : 10e-3;
            const std::vector<std::string> lSig = {comp.id + ".La", comp.id + ".Lb", comp.id + ".Lc"};
            const std::vector<std::string> eSig = {comp.id + ".Ea", comp.id + ".Eb", comp.id + ".Ec"};
            emitMachineStator(physStageObj, comp.id, term, mR, 0.5 * (lmin + lmax), &lSig, eSig);

            cObj["output"] = comp.id + ".Speed";
            cObj["original_type"] = t;
            cObj["input"] = getIncomingSignal(comp.id, "TL");
            cObj["current_a"] = "I_" + comp.id + "_LA";
            cObj["current_b"] = "I_" + comp.id + "_LB";
            cObj["current_c"] = "I_" + comp.id + "_LC";
            cObj["Lmin"] = formatJSStyleDouble(lmin);
            cObj["Lmax"] = formatJSStyleDouble(lmax);
            cObj["Nr"] = formatJSStyleDouble(parsedParams.count("Nr") ? parsedParams["Nr"] : 4.0);
            cObj["J"] = formatJSStyleDouble(parsedParams.count("J") ? parsedParams["J"] : 1e-4);
            cObj["B"] = formatJSStyleDouble(parsedParams.count("B") ? parsedParams["B"] : 1e-4);
            cObj["omega0"] = formatJSStyleDouble(parsedParams.count("omega0") ? parsedParams["omega0"] : 0.0);
            cObj["theta0"] = formatJSStyleDouble(parsedParams.count("theta0") ? parsedParams["theta0"] : 0.0);
            ctrlLoopsObj["gains"].push_back(cObj);
        } else if (t == "INDUCTION_MOTOR" || t == "IND_MOTOR") {
            // Squirrel-cage induction machine. Stator per phase: Rs and the stator leakage
            // sigma*Ls = Lls + Lm Llr / (Lm + Llr). The back-EMF (Lm / Lr) d psi_r / dt comes from the
            // rotor flux state in the control block, which also computes the torque.
            const std::vector<std::string> term = {machineNodeAt(formattedNodes, 0),
                                                   machineNodeAt(formattedNodes, 1),
                                                   machineNodeAt(formattedNodes, 2)};
            const double mRs = parsedParams.count("Rs") ? parsedParams["Rs"] : 1.115;
            const double mLls = parsedParams.count("Lls") ? parsedParams["Lls"] : 0.005974;
            const double mRr = parsedParams.count("Rr") ? parsedParams["Rr"] : 1.083;
            const double mLlr = parsedParams.count("Llr") ? parsedParams["Llr"] : 0.005974;
            const double mLm = parsedParams.count("Lm") ? parsedParams["Lm"] : 0.1;
            const double mLr = mLm + mLlr;
            const double mSigmaL = mLls + mLm * mLlr / mLr;
            const std::vector<std::string> eSig = {comp.id + ".Ea", comp.id + ".Eb", comp.id + ".Ec"};
            emitMachineStator(physStageObj, comp.id, term, mRs, mSigmaL, nullptr, eSig);

            cObj["output"] = comp.id + ".Speed";
            cObj["original_type"] = "IND_MOTOR";
            cObj["input"] = getIncomingSignal(comp.id, "TL");
            cObj["current_a"] = "I_" + comp.id + "_LA";
            cObj["current_b"] = "I_" + comp.id + "_LB";
            cObj["current_c"] = "I_" + comp.id + "_LC";
            cObj["Rr"] = formatJSStyleDouble(mRr);
            cObj["Lr"] = formatJSStyleDouble(mLr);
            cObj["Lm"] = formatJSStyleDouble(mLm);
            cObj["P"] = formatJSStyleDouble(parsedParams.count("P") ? parsedParams["P"] : 2.0);
            cObj["J"] = formatJSStyleDouble(parsedParams.count("J") ? parsedParams["J"] : 0.02);
            cObj["B"] = formatJSStyleDouble(parsedParams.count("B") ? parsedParams["B"] : 1e-3);
            cObj["omega0"] = formatJSStyleDouble(parsedParams.count("omega0") ? parsedParams["omega0"] : 0.0);
            ctrlLoopsObj["gains"].push_back(cObj);
        } else if (t == "GOTO_SIG" || t == "GOTO") {
            json gObj;
            gObj["id"] = comp.id;
            gObj["type"] = "GOTO_SIG";
            gObj["tag"] = comp.parameters.count("tag") ? comp.parameters.at("tag") : "A";
            gObj["input"] = getIncomingSignal(comp.id, "In");
            ctrlLoopsObj["signals_routing"].push_back(gObj);
        } else if (t == "FROM_SIG" || t == "FROM") {
            json fObj;
            fObj["id"] = comp.id;
            fObj["type"] = "FROM_SIG";
            fObj["tag"] = comp.parameters.count("tag") ? comp.parameters.at("tag") : "A";
            fObj["output"] = comp.id + ".Out";
            ctrlLoopsObj["signals_routing"].push_back(fObj);
        } else if (t == "CONST" || t == "CONSTANT") {
            cObj["output"] = comp.id + ".Out";
            cObj["original_type"] = "CONST";
            double val = 1.0;
            if (comp.parameters.count("value")) val = CircuitSimEngine::ExpressionEvaluator::parseScientific(comp.parameters.at("value"));
            else if (comp.parameters.count("constant")) val = CircuitSimEngine::ExpressionEvaluator::parseScientific(comp.parameters.at("constant"));
            else if (comp.parameters.count("const")) val = CircuitSimEngine::ExpressionEvaluator::parseScientific(comp.parameters.at("const"));
            else if (comp.parameters.count("val")) val = CircuitSimEngine::ExpressionEvaluator::parseScientific(comp.parameters.at("val"));
            cObj["value"] = formatJSStyleDouble(roundToDigits(val, 9));
            ctrlLoopsObj["constants"].push_back(cObj);
        } else if (t == "PULSE" || t == "PULSE_GEN") {
            cObj["output"] = comp.id + ".Out";
            cObj["original_type"] = comp.rawTypeStr;
            cObj["amplitude"] = formatJSStyleDouble(parsedParams.count("amplitude") ? parsedParams["amplitude"] : 1.0);
            cObj["period"] = formatJSStyleDouble(parsedParams.count("period") ? parsedParams["period"] : 0.0001);
            cObj["width"] = formatJSStyleDouble(parsedParams.count("width") ? parsedParams["width"] : 0.5);
            cObj["delay"] = formatJSStyleDouble(parsedParams.count("delay") ? parsedParams["delay"] : 0.0);
            cObj["value"] = 1.0;
            ctrlLoopsObj["constants"].push_back(cObj);
        } else if (t == "CLOCK") {
            cObj["output"] = comp.id + ".Out";
            cObj["original_type"] = "CLOCK";
            cObj["value"] = 1.0;
            ctrlLoopsObj["constants"].push_back(cObj);
        } else if (t == "INIT_COND") {
            cObj["output"] = comp.id + ".Out";
            cObj["original_type"] = "INIT_COND";
            cObj["initial_value"] = formatJSStyleDouble(parsedParams.count("initial_value") ? parsedParams["initial_value"] : (parsedParams.count("x0") ? parsedParams["x0"] : 0.0));
            std::string inSig = getIncomingSignal(comp.id, "In");
            cObj["input"] = inSig;
            cObj["input1"] = inSig;
            cObj["input2"] = "0.0";
            cObj["K"] = 1.0;
            ctrlLoopsObj["gains"].push_back(cObj);
        } else if (t == "RAMP") {
            cObj["output"] = comp.id + ".Out";
            cObj["original_type"] = "RAMP";
            cObj["slope"] = formatJSStyleDouble(parsedParams.count("slope") ? parsedParams["slope"] : 1.0);
            cObj["start_time"] = formatJSStyleDouble(parsedParams.count("start_time") ? parsedParams["start_time"] : 0.0);
            cObj["initial_output"] = formatJSStyleDouble(parsedParams.count("initial_output") ? parsedParams["initial_output"] : 0.0);
            cObj["value"] = 1.0;
            ctrlLoopsObj["constants"].push_back(cObj);
        } else if (t == "RANDOM_NUM") {
            cObj["output"] = comp.id + ".Out";
            cObj["original_type"] = "RANDOM_NUM";
            cObj["mean"] = formatJSStyleDouble(parsedParams.count("mean") ? parsedParams["mean"] : 0.0);
            cObj["std"] = formatJSStyleDouble(parsedParams.count("std") ? parsedParams["std"] : 1.0);
            cObj["value"] = 1.0;
            ctrlLoopsObj["constants"].push_back(cObj);
        } else if (t == "SINE_WAVE") {
            cObj["output"] = comp.id + ".Out";
            cObj["original_type"] = "SINE_WAVE";

            auto getParamVal = [&](const std::vector<std::string>& keys, double defaultVal) -> double {
                for (const auto& k : keys) {
                    if (parsedParams.count(k)) return parsedParams[k];
                    if (comp.parameters.count(k)) {
                        try { return std::stod(comp.parameters.at(k)); } catch(...) {}
                    }
                }
                return defaultVal;
            };

            cObj["amplitude"] = formatJSStyleDouble(getParamVal({"amplitude", "amp", "a"}, 1.0));
            cObj["frequency"] = formatJSStyleDouble(getParamVal({"frequency", "freq", "f"}, 50.0));
            cObj["phase"] = formatJSStyleDouble(getParamVal({"phase", "ph"}, 0.0));
            double biasVal = getParamVal({"bias", "offset", "dc_offset", "dc"}, 0.0);
            cObj["bias"] = formatJSStyleDouble(biasVal);
            cObj["offset"] = formatJSStyleDouble(biasVal);
            cObj["value"] = 1.0;
            ctrlLoopsObj["constants"].push_back(cObj);
        } else if (t == "STEP") {
            cObj["output"] = comp.id + ".Out";
            cObj["original_type"] = "STEP";
            cObj["step_time"] = formatJSStyleDouble(parsedParams.count("step_time") ? parsedParams["step_time"] : 1.0);
            cObj["initial_value"] = formatJSStyleDouble(parsedParams.count("initial_value") ? parsedParams["initial_value"] : 0.0);
            cObj["final_value"] = formatJSStyleDouble(parsedParams.count("final_value") ? parsedParams["final_value"] : 1.0);
            cObj["value"] = 1.0;
            ctrlLoopsObj["constants"].push_back(cObj);
        } else if (t == "WHITE_NOISE") {
            cObj["output"] = comp.id + ".Out";
            cObj["original_type"] = "WHITE_NOISE";
            cObj["psd"] = formatJSStyleDouble(parsedParams.count("psd") ? parsedParams["psd"] : 0.1);
            cObj["value"] = 1.0;
            ctrlLoopsObj["constants"].push_back(cObj);
        } else if (t == "MIN_MAX" || t == "MIN" || t == "MAX") {
            cObj["output"] = comp.id + ".Out";
            cObj["original_type"] = "MIN_MAX";
            cObj["function"] = comp.parameters.count("function") ? comp.parameters.at("function") : "min";

            std::string nStr = comp.parameters.count("num_inputs") ? comp.parameters.at("num_inputs") : "";
            if (nStr.empty() && comp.parameters.count("inputs")) nStr = comp.parameters.at("inputs");
            if (nStr.empty() && comp.parameters.count("number_of_inputs")) nStr = comp.parameters.at("number_of_inputs");

            int nPins = 2;
            if (!nStr.empty()) {
                try { nPins = std::clamp(std::stoi(nStr), 1, 32); } catch (...) { nPins = 2; }
            }

            cObj["num_inputs"] = std::to_string(nPins);
            json inputsArr = json::array();
            for (int i = 1; i <= nPins; ++i) {
                std::string sigKey = getIncomingSignal(comp.id, "In" + std::to_string(i));
                if (sigKey == "0.0" && i == 1) {
                    std::string sigAlt = getIncomingSignal(comp.id, "A");
                    if (sigAlt != "0.0") sigKey = sigAlt;
                }
                if (sigKey == "0.0" && i == 2) {
                    std::string sigAlt = getIncomingSignal(comp.id, "B");
                    if (sigAlt != "0.0") sigKey = sigAlt;
                }
                inputsArr.push_back(sigKey);
                cObj["In" + std::to_string(i)] = sigKey;
                cObj["input_" + std::to_string(i - 1)] = sigKey;
            }
            cObj["inputs"] = inputsArr;
            cObj["K"] = 1.0;
            ctrlLoopsObj["gains"].push_back(cObj);
        } else if (t == "QUANTIZER") {
            cObj["output"] = comp.id + ".Out";
            cObj["original_type"] = "QUANTIZER";
            double stepVal = 0.5;
            if (parsedParams.count("interval")) stepVal = parsedParams["interval"];
            else if (parsedParams.count("step_size")) stepVal = parsedParams["step_size"];
            else if (parsedParams.count("step")) stepVal = parsedParams["step"];
            else if (parsedParams.count("quantization_interval")) stepVal = parsedParams["quantization_interval"];
            else if (parsedParams.count("q")) stepVal = parsedParams["q"];
            else if (comp.parameters.count("interval")) { try { stepVal = std::stod(comp.parameters.at("interval")); } catch(...) {} }
            else if (comp.parameters.count("step_size")) { try { stepVal = std::stod(comp.parameters.at("step_size")); } catch(...) {} }
            else if (comp.parameters.count("step")) { try { stepVal = std::stod(comp.parameters.at("step")); } catch(...) {} }
            else if (comp.parameters.count("quantization_interval")) { try { stepVal = std::stod(comp.parameters.at("quantization_interval")); } catch(...) {} }
            else if (comp.parameters.count("q")) { try { stepVal = std::stod(comp.parameters.at("q")); } catch(...) {} }

            cObj["step_size"] = formatJSStyleDouble(stepVal);
            cObj["interval"] = formatJSStyleDouble(stepVal);
            cObj["mode"] = comp.parameters.count("mode") ? comp.parameters.at("mode") : "round";
            std::string inSig = getIncomingSignal(comp.id, "In");
            cObj["input"] = inSig;
            cObj["input1"] = inSig;
            cObj["input2"] = "0.0";
            cObj["K"] = 1.0;
            ctrlLoopsObj["gains"].push_back(cObj);
        } else if (t == "SIGNAL_SWITCH") {
            cObj["output"] = comp.id + ".Out";
            cObj["original_type"] = "SIGNAL_SWITCH";
            cObj["threshold"] = formatJSStyleDouble(parsedParams.count("threshold") ? parsedParams["threshold"] : 0.5);
            cObj["criteria"] = comp.parameters.count("criteria") ? comp.parameters.at("criteria") : "u2 >= threshold";
            cObj["input1"] = getIncomingSignal(comp.id, "In1");
            cObj["control_signal"] = getIncomingSignal(comp.id, "Ctrl");
            cObj["input2"] = getIncomingSignal(comp.id, "In2");
            ctrlLoopsObj["gains"].push_back(cObj);
        } else if (t == "MANUAL_SWITCH") {
            cObj["output"] = comp.id + ".Out";
            cObj["original_type"] = "MANUAL_SWITCH";
            cObj["state"] = comp.parameters.count("state") ? comp.parameters.at("state") : "Input 1";
            cObj["input1"] = getIncomingSignal(comp.id, "In1");
            cObj["input2"] = getIncomingSignal(comp.id, "In2");
            ctrlLoopsObj["gains"].push_back(cObj);
        } else if (t == "MULTIPORT_SWITCH") {
            cObj["output"] = comp.id + ".Out";
            cObj["original_type"] = "MULTIPORT_SWITCH";
            cObj["control_signal"] = getIncomingSignal(comp.id, "Ctrl");
            cObj["inputs_count"] = comp.parameters.count("inputs") ? comp.parameters.at("inputs") : "3";
            int numInp = 3;
            if (parsedParams.count("inputs")) numInp = (int)parsedParams["inputs"];
            json inpsArr = json::array();
            for (int i = 1; i <= numInp; ++i) {
                inpsArr.push_back(getIncomingSignal(comp.id, "In" + std::to_string(i)));
            }
            cObj["inputs"] = inpsArr;
            ctrlLoopsObj["gains"].push_back(cObj);
        } else if (t == "HIT_CROSSING") {
            cObj["output"] = comp.id + ".Out";
            cObj["original_type"] = "HIT_CROSSING";
            cObj["offset"] = formatJSStyleDouble(parsedParams.count("offset") ? parsedParams["offset"] : 0.0);
            cObj["direction"] = comp.parameters.count("direction") ? comp.parameters.at("direction") : "either";
            std::string inSig = getIncomingSignal(comp.id, "In");
            cObj["input"] = inSig;
            cObj["input1"] = inSig;
            cObj["input2"] = "0.0";
            cObj["K"] = 1.0;
            ctrlLoopsObj["gains"].push_back(cObj);
        } else if (t == "SATURATION") {
            cObj["output"] = comp.id + ".Out";
            cObj["original_type"] = "SATURATION";

            auto getParamVal = [&](const std::vector<std::string>& keys, double defaultVal) -> double {
                for (const auto& k : keys) {
                    if (parsedParams.count(k)) return parsedParams[k];
                    if (comp.parameters.count(k)) {
                        try { return std::stod(comp.parameters.at(k)); } catch(...) {}
                    }
                }
                return defaultVal;
            };

            double minVal = getParamVal({"lower_limit", "min_limit", "min", "minVal", "v_min", "min_val", "lower", "low"}, -10.0);
            double maxVal = getParamVal({"upper_limit", "max_limit", "max", "maxVal", "v_max", "max_val", "upper", "high"}, 10.0);

            cObj["min"] = formatJSStyleDouble(minVal);
            cObj["max"] = formatJSStyleDouble(maxVal);
            cObj["lower_limit"] = formatJSStyleDouble(minVal);
            cObj["upper_limit"] = formatJSStyleDouble(maxVal);

            std::string inSig = getIncomingSignal(comp.id, "In");
            cObj["input"] = inSig;
            cObj["input1"] = inSig;
            cObj["input2"] = "0.0";
            cObj["K"] = 1.0;
            ctrlLoopsObj["gains"].push_back(cObj);
        } else if (t == "DEAD_ZONE") {
            cObj["output"] = comp.id + ".Out";
            cObj["original_type"] = "DEAD_ZONE";

            auto getParamVal = [&](const std::vector<std::string>& keys, double defaultVal) -> double {
                for (const auto& k : keys) {
                    if (parsedParams.count(k)) return parsedParams[k];
                    if (comp.parameters.count(k)) {
                        try { return std::stod(comp.parameters.at(k)); } catch(...) {}
                    }
                }
                return defaultVal;
            };

            double startVal = getParamVal({"start_of_dead_zone", "dead_zone_start", "start_dead_zone", "start", "min", "lower_limit", "low", "lower"}, -0.5);
            double endVal = getParamVal({"end_of_dead_zone", "dead_zone_end", "end_dead_zone", "end", "max", "upper_limit", "high", "upper"}, 0.5);

            cObj["start"] = formatJSStyleDouble(startVal);
            cObj["end"] = formatJSStyleDouble(endVal);
            cObj["start_of_dead_zone"] = formatJSStyleDouble(startVal);
            cObj["end_of_dead_zone"] = formatJSStyleDouble(endVal);

            std::string inSig = getIncomingSignal(comp.id, "In");
            cObj["input"] = inSig;
            cObj["input1"] = inSig;
            cObj["input2"] = "0.0";
            cObj["K"] = 1.0;
            ctrlLoopsObj["gains"].push_back(cObj);
        } else if (t == "RATE_LIMITER") {
            cObj["output"] = comp.id + ".Out";
            cObj["original_type"] = "RATE_LIMITER";

            auto getParamVal = [&](const std::vector<std::string>& keys, double defaultVal) -> double {
                for (const auto& k : keys) {
                    if (parsedParams.count(k)) return parsedParams[k];
                    if (comp.parameters.count(k)) {
                        try { return std::stod(comp.parameters.at(k)); } catch(...) {}
                    }
                }
                return defaultVal;
            };

            double upRate = getParamVal({"rising_slew_rate", "slew_rate_rising", "rising_rate", "rate_up", "slew_rate_up", "rate_rising", "up", "rising"}, 10.0);
            double downRate = getParamVal({"falling_slew_rate", "slew_rate_falling", "falling_rate", "rate_down", "slew_rate_down", "rate_falling", "down", "falling"}, -10.0);

            cObj["up"] = formatJSStyleDouble(upRate);
            cObj["down"] = formatJSStyleDouble(downRate);
            cObj["rising_slew_rate"] = formatJSStyleDouble(upRate);
            cObj["falling_slew_rate"] = formatJSStyleDouble(downRate);

            std::string inSig = getIncomingSignal(comp.id, "In");
            cObj["input"] = inSig;
            cObj["input1"] = inSig;
            cObj["input2"] = "0.0";
            cObj["K"] = 1.0;
            ctrlLoopsObj["gains"].push_back(cObj);
        } else if (t == "RELAY") {
            cObj["output"] = comp.id + ".Out";
            cObj["original_type"] = "RELAY";

            auto getParamVal = [&](const std::vector<std::string>& keys, double defaultVal) -> double {
                for (const auto& k : keys) {
                    if (parsedParams.count(k)) return parsedParams[k];
                    if (comp.parameters.count(k)) {
                        try { return std::stod(comp.parameters.at(k)); } catch(...) {}
                    }
                }
                return defaultVal;
            };

            double onThresh = getParamVal({"switch_on_point", "on_point", "switch_on", "on_threshold", "high_threshold", "upper_threshold", "on"}, 1.0);
            double offThresh = getParamVal({"switch_off_point", "off_point", "switch_off", "off_threshold", "low_threshold", "lower_threshold", "off"}, -1.0);
            double outOn = getParamVal({"output_on", "on_output", "on_val", "output_high", "on_value"}, 1.0);
            double outOff = getParamVal({"output_off", "off_output", "off_val", "output_low", "off_value"}, 0.0);

            cObj["on_threshold"] = formatJSStyleDouble(onThresh);
            cObj["off_threshold"] = formatJSStyleDouble(offThresh);
            cObj["switch_on_point"] = formatJSStyleDouble(onThresh);
            cObj["switch_off_point"] = formatJSStyleDouble(offThresh);
            cObj["output_on"] = formatJSStyleDouble(outOn);
            cObj["output_off"] = formatJSStyleDouble(outOff);

            std::string inSig = getIncomingSignal(comp.id, "In");
            cObj["input"] = inSig;
            cObj["input1"] = inSig;
            cObj["input2"] = "0.0";
            cObj["K"] = 1.0;
            ctrlLoopsObj["gains"].push_back(cObj);
        } else if (t == "COMP" || t == "Comparator") {
            cObj["output"] = comp.id + ".Out";
            cObj["original_type"] = "COMP";
            std::string inA = getIncomingSignal(comp.id, "A");
            if (inA == "0.0") inA = getIncomingSignal(comp.id, "Plus");
            if (inA == "0.0") inA = getIncomingSignal(comp.id, "In1");
            std::string inB = getIncomingSignal(comp.id, "B");
            if (inB == "0.0") inB = getIncomingSignal(comp.id, "Minus");
            if (inB == "0.0") inB = getIncomingSignal(comp.id, "In2");
            cObj["input_a"] = inA;
            cObj["input_b"] = inB;
            cObj["input_0"] = inA;
            cObj["input_1"] = inB;
            ctrlLoopsObj["comparators"].push_back(cObj);
        } else if (t == "LOGIC_OP" || t == "NAND" || t == "NOR" || t == "XOR" || t == "XNOR" || t == "NXOR" || t == "AND" || t == "OR" || t == "NOT") {
            cObj["output"] = comp.id + ".Out";
            cObj["original_type"] = "LOGIC_OP";
            std::string op = comp.parameters.count("operator") ? comp.parameters.at("operator") : t;
            if (op.empty()) op = t;
            cObj["operator"] = op;
            std::string in1 = getIncomingSignal(comp.id, "In1");
            if (in1 == "0.0") in1 = getIncomingSignal(comp.id, "In");
            if (in1 == "0.0") in1 = getIncomingSignal(comp.id, "A");
            std::string in2 = getIncomingSignal(comp.id, "In2");
            if (in2 == "0.0") in2 = getIncomingSignal(comp.id, "B");
            cObj["input1"] = in1;
            cObj["input2"] = in2;
            cObj["input_a"] = in1;
            cObj["input_b"] = in2;
            ctrlLoopsObj["gains"].push_back(cObj);
        } else if (t == "MATH_FCN" || t == "MATH_FUNC" || t == "MathFunction" || t == "MATH") {
            cObj["output"] = comp.id + ".Out";
            cObj["original_type"] = "MATH_FCN";
            cObj["function"] = comp.parameters.count("function") ? comp.parameters.at("function") : (comp.parameters.count("fcn") ? comp.parameters.at("fcn") : "exp");
            std::string in1 = getIncomingSignal(comp.id, "In");
            if (in1 == "0.0") in1 = getIncomingSignal(comp.id, "In1");
            if (in1 == "0.0") in1 = getIncomingSignal(comp.id, "A");
            std::string in2 = getIncomingSignal(comp.id, "In2");
            if (in2 == "0.0") in2 = getIncomingSignal(comp.id, "B");
            cObj["input"] = in1;
            cObj["input1"] = in1;
            cObj["input2"] = in2;
            cObj["K"] = 1.0;
            ctrlLoopsObj["gains"].push_back(cObj);
        } else if (t == "BITWISE_OP") {
            cObj["output"] = comp.id + ".Out";
            cObj["original_type"] = "BITWISE_OP";
            cObj["operator"] = comp.parameters.count("operator") ? comp.parameters.at("operator") : "XOR";
            // Bit mask applied to both operands before the operation. -1 keeps every bit.
            cObj["bit_mask"] = comp.parameters.count("bit_mask") ? comp.parameters.at("bit_mask") : "-1";
            cObj["input1"] = getIncomingSignal(comp.id, "In1");
            cObj["input2"] = getIncomingSignal(comp.id, "In2");
            ctrlLoopsObj["gains"].push_back(cObj);
        } else if (t == "COMB_LOGIC") {
            cObj["output"] = comp.id + ".Out";
            cObj["original_type"] = "COMB_LOGIC";
            cObj["truth_table"] = comp.parameters.count("truth_table") ? comp.parameters.at("truth_table") : "";
            cObj["input1"] = getIncomingSignal(comp.id, "In1");
            cObj["input2"] = getIncomingSignal(comp.id, "In2");
            ctrlLoopsObj["gains"].push_back(cObj);
        } else if (t == "EDGE_DETECT") {
            cObj["output"] = comp.id + ".Out";
            cObj["original_type"] = "EDGE_DETECT";
            // Registry parameter is edge_type (rising / falling / either).
            cObj["edge"] = comp.parameters.count("edge_type") ? comp.parameters.at("edge_type")
                         : comp.parameters.count("edge") ? comp.parameters.at("edge") : "rising";
            cObj["pulse_width"] = comp.parameters.count("pulse_width") ? comp.parameters.at("pulse_width") : "1e-3";
            std::string inSig = getIncomingSignal(comp.id, "In");
            cObj["input"] = inSig;
            cObj["input1"] = inSig;
            cObj["input2"] = "0.0";
            cObj["K"] = 1.0;
            ctrlLoopsObj["gains"].push_back(cObj);
        } else if (t == "MONOSTABLE") {
            cObj["output"] = comp.id + ".Out";
            cObj["original_type"] = "MONOSTABLE";
            // Registry parameter names are pulse_width / edge_type. The old code read
            // "duration" and "edge" (absent), so every monostable ran a 0.1 s pulse.
            cObj["duration"] = comp.parameters.count("pulse_width") ? comp.parameters.at("pulse_width")
                             : comp.parameters.count("duration") ? comp.parameters.at("duration") : "0.002";
            cObj["edge"] = comp.parameters.count("edge_type") ? comp.parameters.at("edge_type")
                         : comp.parameters.count("edge") ? comp.parameters.at("edge") : "rising";
            std::string inSig = getIncomingSignal(comp.id, "In");
            cObj["input"] = inSig;
            cObj["input1"] = inSig;
            cObj["input2"] = "0.0";
            cObj["K"] = 1.0;
            ctrlLoopsObj["gains"].push_back(cObj);
        } else if (t == "MONOFLOP") {
            cObj["output"] = comp.id + ".Out";
            cObj["original_type"] = "MONOFLOP";
            cObj["duration"] = comp.parameters.count("duration") ? comp.parameters.at("duration") : "0.1";
            cObj["trigger_edge"] = comp.parameters.count("trigger_edge") ? comp.parameters.at("trigger_edge") : "rising";
            cObj["retriggerable"] = comp.parameters.count("retriggerable") ? comp.parameters.at("retriggerable") : "false";
            std::string inSig = getIncomingSignal(comp.id, "In");
            cObj["input"] = inSig;
            cObj["input1"] = inSig;
            cObj["input2"] = "0.0";
            cObj["K"] = 1.0;
            ctrlLoopsObj["gains"].push_back(cObj);
        } else if (t == "RELATIONAL_OPERATOR") {
            cObj["output"] = comp.id + ".Out";
            cObj["original_type"] = "RELATIONAL_OPERATOR";
            cObj["operator"] = comp.parameters.count("operator") ? comp.parameters.at("operator") : "<";
            cObj["input1"] = getIncomingSignal(comp.id, "In1");
            cObj["input2"] = getIncomingSignal(comp.id, "In2");
            ctrlLoopsObj["gains"].push_back(cObj);
        } else if (t == "COMPARE_TO_CONSTANT" || t == "COMP_CONST") {
            cObj["output"] = comp.id + ".Out";
            cObj["original_type"] = "COMPARE_TO_CONSTANT";
            std::string op = comp.parameters.count("operator") ? comp.parameters.at("operator") : (comp.parameters.count("op") ? comp.parameters.at("op") : ">=");
            cObj["operator"] = op;

            double cVal = 0.0;
            if (parsedParams.count("threshold")) cVal = parsedParams["threshold"];
            else if (parsedParams.count("constant")) cVal = parsedParams["constant"];
            else if (parsedParams.count("const")) cVal = parsedParams["const"];
            else if (parsedParams.count("value")) cVal = parsedParams["value"];
            else if (parsedParams.count("val")) cVal = parsedParams["val"];

            cObj["constant"] = formatJSStyleDouble(cVal);
            cObj["threshold"] = formatJSStyleDouble(cVal);

            std::string inSig = getIncomingSignal(comp.id, "In");
            if (inSig == "0.0") inSig = getIncomingSignal(comp.id, "In1");
            if (inSig == "0.0") inSig = getIncomingSignal(comp.id, "A");
            cObj["input"] = inSig;
            cObj["input1"] = inSig;
            cObj["input2"] = "0.0";
            cObj["K"] = 1.0;
            ctrlLoopsObj["gains"].push_back(cObj);
        } else if (t == "D_FLIP_FLOP") {
            cObj["output"] = comp.id + ".Q";
            cObj["output_bar"] = comp.id + ".Q_bar";
            cObj["original_type"] = "D_FLIP_FLOP";
            cObj["initial_state"] = formatJSStyleDouble(parsedParams.count("initial_state") ? parsedParams["initial_state"] : 0.0);
            cObj["trigger_edge"] = comp.parameters.count("trigger_edge") ? comp.parameters.at("trigger_edge") : "rising";
            // The engine binds data to "input" and the clock to "control_signal" (the
            // keys every other block uses). "D" and "Clk" were never read, so the clock
            // was always 0 and Q never changed.
            cObj["input"] = getIncomingSignal(comp.id, "D");
            cObj["control_signal"] = getIncomingSignal(comp.id, "Ctrl");
            cObj["D"] = getIncomingSignal(comp.id, "D");
            cObj["Clk"] = getIncomingSignal(comp.id, "Ctrl");
            ctrlLoopsObj["gains"].push_back(cObj);
        } else if (t == "JK_FLIP_FLOP") {
            cObj["output"] = comp.id + ".Q";
            cObj["output_bar"] = comp.id + ".Q_bar";
            cObj["original_type"] = "JK_FLIP_FLOP";
            cObj["initial_state"] = formatJSStyleDouble(parsedParams.count("initial_state") ? parsedParams["initial_state"] : 0.0);
            cObj["trigger_edge"] = comp.parameters.count("trigger_edge") ? comp.parameters.at("trigger_edge") : "rising";
            cObj["J"] = getIncomingSignal(comp.id, "J");
            cObj["K"] = getIncomingSignal(comp.id, "K");
            cObj["Clk"] = getIncomingSignal(comp.id, "Ctrl");
            ctrlLoopsObj["gains"].push_back(cObj);
        } else if (t == "SHIFT_REG") {
            cObj["output"] = comp.id + ".Out";
            cObj["original_type"] = "SHIFT_REG";
            int len = parsedParams.count("length") ? (int)parsedParams["length"] : 4;
            cObj["length"] = len;
            cObj["D"] = getIncomingSignal(comp.id, "D");
            cObj["Clk"] = getIncomingSignal(comp.id, "Ctrl");
            ctrlLoopsObj["gains"].push_back(cObj);
        } else if (t == "CLARKE" || t == "CLARKE_TRANSFORM" || t == "Clarke") {
            cObj["output_alpha"] = comp.id + ".Alpha";
            cObj["output_beta"] = comp.id + ".Beta";
            cObj["original_type"] = "CLARKE";

            std::string inA = getIncomingSignal(comp.id, "A");
            if (inA == "0.0") inA = getIncomingSignal(comp.id, "Va");
            if (inA == "0.0") inA = getIncomingSignal(comp.id, "a");
            if (inA == "0.0") inA = getIncomingSignal(comp.id, "In1");

            std::string inB = getIncomingSignal(comp.id, "B");
            if (inB == "0.0") inB = getIncomingSignal(comp.id, "Vb");
            if (inB == "0.0") inB = getIncomingSignal(comp.id, "b");
            if (inB == "0.0") inB = getIncomingSignal(comp.id, "In2");

            std::string inC = getIncomingSignal(comp.id, "C");
            if (inC == "0.0") inC = getIncomingSignal(comp.id, "Vc");
            if (inC == "0.0") inC = getIncomingSignal(comp.id, "c");
            if (inC == "0.0") inC = getIncomingSignal(comp.id, "In3");

            cObj["input_a"] = inA;
            cObj["input_b"] = inB;
            cObj["input_c"] = inC;
            cObj["input_A"] = inA;
            cObj["input_B"] = inB;
            cObj["input_C"] = inC;
            cObj["Va"] = inA;
            cObj["Vb"] = inB;
            cObj["Vc"] = inC;
            cObj["A"] = inA;
            cObj["B"] = inB;
            cObj["C"] = inC;
            ctrlLoopsObj["gains"].push_back(cObj);
        } else if (t == "INV_CLARKE" || t == "INV_CLARKE_TRANSFORM" || t == "InvClarke") {
            cObj["output_a"] = comp.id + ".A";
            cObj["output_b"] = comp.id + ".B";
            cObj["output_c"] = comp.id + ".C";
            cObj["original_type"] = "INV_CLARKE";

            std::string inAlpha = getIncomingSignal(comp.id, "Alpha");
            if (inAlpha == "0.0") inAlpha = getIncomingSignal(comp.id, "Valpha");
            if (inAlpha == "0.0") inAlpha = getIncomingSignal(comp.id, "alpha");
            if (inAlpha == "0.0") inAlpha = getIncomingSignal(comp.id, "In1");

            std::string inBeta = getIncomingSignal(comp.id, "Beta");
            if (inBeta == "0.0") inBeta = getIncomingSignal(comp.id, "Vbeta");
            if (inBeta == "0.0") inBeta = getIncomingSignal(comp.id, "beta");
            if (inBeta == "0.0") inBeta = getIncomingSignal(comp.id, "In2");

            cObj["input_alpha"] = inAlpha;
            cObj["input_beta"] = inBeta;
            cObj["Valpha"] = inAlpha;
            cObj["Vbeta"] = inBeta;
            cObj["Alpha"] = inAlpha;
            cObj["Beta"] = inBeta;
            ctrlLoopsObj["gains"].push_back(cObj);
        } else if (t == "PARK" || t == "PARK_TRANSFORM" || t == "Park") {
            cObj["output_d"] = comp.id + ".d";
            cObj["output_q"] = comp.id + ".q";
            cObj["original_type"] = "PARK";

            std::string inAlpha = getIncomingSignal(comp.id, "Alpha");
            if (inAlpha == "0.0") inAlpha = getIncomingSignal(comp.id, "Valpha");
            if (inAlpha == "0.0") inAlpha = getIncomingSignal(comp.id, "alpha");
            if (inAlpha == "0.0") inAlpha = getIncomingSignal(comp.id, "In1");

            std::string inBeta = getIncomingSignal(comp.id, "Beta");
            if (inBeta == "0.0") inBeta = getIncomingSignal(comp.id, "Vbeta");
            if (inBeta == "0.0") inBeta = getIncomingSignal(comp.id, "beta");
            if (inBeta == "0.0") inBeta = getIncomingSignal(comp.id, "In2");

            std::string inTheta = getIncomingSignal(comp.id, "Theta");
            if (inTheta == "0.0") inTheta = getIncomingSignal(comp.id, "theta");
            if (inTheta == "0.0") inTheta = getIncomingSignal(comp.id, "wt");
            if (inTheta == "0.0") inTheta = getIncomingSignal(comp.id, "In3");

            std::string inA = getIncomingSignal(comp.id, "A");
            if (inA == "0.0") inA = getIncomingSignal(comp.id, "Va");
            std::string inB = getIncomingSignal(comp.id, "B");
            if (inB == "0.0") inB = getIncomingSignal(comp.id, "Vb");
            std::string inC = getIncomingSignal(comp.id, "C");
            if (inC == "0.0") inC = getIncomingSignal(comp.id, "Vc");

            cObj["input_alpha"] = inAlpha;
            cObj["input_beta"] = inBeta;
            cObj["input_theta"] = inTheta;
            cObj["input_a"] = inA;
            cObj["input_b"] = inB;
            cObj["input_c"] = inC;
            cObj["Valpha"] = inAlpha;
            cObj["Vbeta"] = inBeta;
            cObj["Theta"] = inTheta;
            cObj["theta"] = inTheta;
            cObj["Alpha"] = inAlpha;
            cObj["Beta"] = inBeta;
            cObj["A"] = inA;
            cObj["B"] = inB;
            cObj["C"] = inC;
            ctrlLoopsObj["gains"].push_back(cObj);
        } else if (t == "INV_PARK" || t == "INV_PARK_TRANSFORM" || t == "InvPark") {
            cObj["output_alpha"] = comp.id + ".Alpha";
            cObj["output_beta"] = comp.id + ".Beta";
            cObj["original_type"] = "INV_PARK";

            std::string inD = getIncomingSignal(comp.id, "d");
            if (inD == "0.0") inD = getIncomingSignal(comp.id, "Vd");
            if (inD == "0.0") inD = getIncomingSignal(comp.id, "D");
            if (inD == "0.0") inD = getIncomingSignal(comp.id, "d_in");
            if (inD == "0.0") inD = getIncomingSignal(comp.id, "In1");

            std::string inQ = getIncomingSignal(comp.id, "q");
            if (inQ == "0.0") inQ = getIncomingSignal(comp.id, "Vq");
            if (inQ == "0.0") inQ = getIncomingSignal(comp.id, "Q");
            if (inQ == "0.0") inQ = getIncomingSignal(comp.id, "q_in");
            if (inQ == "0.0") inQ = getIncomingSignal(comp.id, "In2");

            std::string inTheta = getIncomingSignal(comp.id, "Theta");
            if (inTheta == "0.0") inTheta = getIncomingSignal(comp.id, "theta");
            if (inTheta == "0.0") inTheta = getIncomingSignal(comp.id, "wt");
            if (inTheta == "0.0") inTheta = getIncomingSignal(comp.id, "In3");

            cObj["input_d"] = inD;
            cObj["input_q"] = inQ;
            cObj["input_theta"] = inTheta;
            cObj["Vd"] = inD;
            cObj["Vq"] = inQ;
            cObj["d"] = inD;
            cObj["q"] = inQ;
            cObj["D"] = inD;
            cObj["Q"] = inQ;
            cObj["Theta"] = inTheta;
            cObj["theta"] = inTheta;
            ctrlLoopsObj["gains"].push_back(cObj);
        } else if (t == "DQ_TO_ABC" || t == "DQ_ABC" || t == "INV_PARK_3PH" || t == "DqToAbc") {
            cObj["output_a"] = comp.id + ".A";
            cObj["output_b"] = comp.id + ".B";
            cObj["output_c"] = comp.id + ".C";
            cObj["original_type"] = "DQ_TO_ABC";

            std::string inD = getIncomingSignal(comp.id, "d");
            if (inD == "0.0") inD = getIncomingSignal(comp.id, "Vd");
            if (inD == "0.0") inD = getIncomingSignal(comp.id, "D");
            if (inD == "0.0") inD = getIncomingSignal(comp.id, "In1");

            std::string inQ = getIncomingSignal(comp.id, "q");
            if (inQ == "0.0") inQ = getIncomingSignal(comp.id, "Vq");
            if (inQ == "0.0") inQ = getIncomingSignal(comp.id, "Q");
            if (inQ == "0.0") inQ = getIncomingSignal(comp.id, "In2");

            std::string inTheta = getIncomingSignal(comp.id, "Theta");
            if (inTheta == "0.0") inTheta = getIncomingSignal(comp.id, "theta");
            if (inTheta == "0.0") inTheta = getIncomingSignal(comp.id, "wt");
            if (inTheta == "0.0") inTheta = getIncomingSignal(comp.id, "In3");

            cObj["input_d"] = inD;
            cObj["input_q"] = inQ;
            cObj["input_theta"] = inTheta;
            cObj["Vd"] = inD;
            cObj["Vq"] = inQ;
            cObj["d"] = inD;
            cObj["q"] = inQ;
            cObj["Theta"] = inTheta;
            cObj["theta"] = inTheta;
            ctrlLoopsObj["gains"].push_back(cObj);
        } else if (t == "ABC_TO_DQ" || t == "ABC_DQ" || t == "PARK_3PH" || t == "AbcToDq") {
            cObj["output_d"] = comp.id + ".d";
            cObj["output_q"] = comp.id + ".q";
            cObj["original_type"] = "ABC_TO_DQ";

            std::string inA = getIncomingSignal(comp.id, "A");
            if (inA == "0.0") inA = getIncomingSignal(comp.id, "Va");
            if (inA == "0.0") inA = getIncomingSignal(comp.id, "In1");

            std::string inB = getIncomingSignal(comp.id, "B");
            if (inB == "0.0") inB = getIncomingSignal(comp.id, "Vb");
            if (inB == "0.0") inB = getIncomingSignal(comp.id, "In2");

            std::string inC = getIncomingSignal(comp.id, "C");
            if (inC == "0.0") inC = getIncomingSignal(comp.id, "Vc");
            if (inC == "0.0") inC = getIncomingSignal(comp.id, "In3");

            std::string inTheta = getIncomingSignal(comp.id, "Theta");
            if (inTheta == "0.0") inTheta = getIncomingSignal(comp.id, "theta");
            if (inTheta == "0.0") inTheta = getIncomingSignal(comp.id, "wt");
            if (inTheta == "0.0") inTheta = getIncomingSignal(comp.id, "In4");

            cObj["input_a"] = inA;
            cObj["input_b"] = inB;
            cObj["input_c"] = inC;
            cObj["input_theta"] = inTheta;
            cObj["Va"] = inA;
            cObj["Vb"] = inB;
            cObj["Vc"] = inC;
            cObj["Theta"] = inTheta;
            cObj["theta"] = inTheta;
            ctrlLoopsObj["gains"].push_back(cObj);
        } else if (t == "PWM_3PH" || t == "SVPWM") {
            cObj["output_a"] = comp.id + ".OutA";
            cObj["output_b"] = comp.id + ".OutB";
            cObj["output_c"] = comp.id + ".OutC";
            cObj["original_type"] = t;
            // The block's carrier parameter is "fc" (and "dead_time"); only
            // "frequency" was forwarded, so fc was ignored (always 10 kHz) and the
            // dead time fell back to the simulator default of 1 us.
            double carrier = 10000.0;
            if (parsedParams.count("fc")) carrier = parsedParams["fc"];
            else if (parsedParams.count("frequency")) carrier = parsedParams["frequency"];
            else if (parsedParams.count("carrier_freq")) carrier = parsedParams["carrier_freq"];
            cObj["frequency"] = formatJSStyleDouble(carrier);
            cObj["dead_time"] = formatJSStyleDouble(parsedParams.count("dead_time") ? parsedParams["dead_time"] : 0.0);
            cObj["input_a"] = getIncomingSignal(comp.id, "A");
            cObj["input_b"] = getIncomingSignal(comp.id, "B");
            cObj["input_c"] = getIncomingSignal(comp.id, "C");
            ctrlLoopsObj["gains"].push_back(cObj);
        } else if (t == "FILTER_1ST") {
            cObj["output"] = comp.id + ".Out";
            cObj["original_type"] = "FILTER_1ST";

            auto getParamVal = [&](const std::vector<std::string>& keys, double defaultVal) -> double {
                for (const auto& k : keys) {
                    if (parsedParams.count(k)) return parsedParams[k];
                    if (comp.parameters.count(k)) {
                        try { return std::stod(comp.parameters.at(k)); } catch(...) {}
                    }
                }
                return defaultVal;
            };

            double fcVal = getParamVal({"cutoff_freq", "cutoff", "fc", "freq", "frequency", "f"}, 100.0);
            cObj["cutoff_freq"] = formatJSStyleDouble(fcVal);
            cObj["fc"] = formatJSStyleDouble(fcVal);
            cObj["type"] = comp.parameters.count("type") ? comp.parameters.at("type") : "Lowpass";

            std::string inSig = getIncomingSignal(comp.id, "In");
            cObj["input"] = inSig; cObj["input1"] = inSig; cObj["input2"] = "0.0"; cObj["K"] = 1.0;
            ctrlLoopsObj["gains"].push_back(cObj);
        } else if (t == "FILTER_2ND") {
            cObj["output"] = comp.id + ".Out";
            cObj["original_type"] = "FILTER_2ND";

            auto getParamVal = [&](const std::vector<std::string>& keys, double defaultVal) -> double {
                for (const auto& k : keys) {
                    if (parsedParams.count(k)) return parsedParams[k];
                    if (comp.parameters.count(k)) {
                        try { return std::stod(comp.parameters.at(k)); } catch(...) {}
                    }
                }
                return defaultVal;
            };

            double fcVal = getParamVal({"cutoff_freq", "cutoff", "fc", "freq", "frequency", "f"}, 100.0);
            double dampingVal = getParamVal({"damping", "damping_ratio", "zeta", "Q", "q"}, 0.707);

            cObj["cutoff_freq"] = formatJSStyleDouble(fcVal);
            cObj["fc"] = formatJSStyleDouble(fcVal);
            cObj["damping"] = formatJSStyleDouble(dampingVal);
            cObj["zeta"] = formatJSStyleDouble(dampingVal);
            cObj["Q"] = formatJSStyleDouble(dampingVal);
            cObj["type"] = comp.parameters.count("type") ? comp.parameters.at("type") : "Lowpass";

            std::string inSig = getIncomingSignal(comp.id, "In");
            cObj["input"] = inSig; cObj["input1"] = inSig; cObj["input2"] = "0.0"; cObj["K"] = 1.0;
            ctrlLoopsObj["gains"].push_back(cObj);
        } else if (t == "CONT_PID" || t == "DISCRETE_PID") {
            cObj["output"] = comp.id + ".Out";
            cObj["original_type"] = comp.rawTypeStr;
            cObj["Kp"] = formatJSStyleDouble(parsedParams.count("Kp") ? parsedParams["Kp"] : 1.0);
            cObj["Ki"] = formatJSStyleDouble(parsedParams.count("Ki") ? parsedParams["Ki"] : 0.0);
            cObj["Kd"] = formatJSStyleDouble(parsedParams.count("Kd") ? parsedParams["Kd"] : 0.0);
            cObj["Tf"] = formatJSStyleDouble(parsedParams.count("Tf") ? parsedParams["Tf"] : 0.01);
            if (comp.parameters.count("ts")) cObj["ts"] = comp.parameters.at("ts");
            else if (t == "DISCRETE_PID") cObj["ts"] = "0.01";   // JSON default: a discrete controller always samples
            if (comp.parameters.count("method")) cObj["method"] = comp.parameters.at("method");
            cObj["limit_output"] = comp.parameters.count("limit_output") ? comp.parameters.at("limit_output") : "false";
            cObj["upper_limit"] = comp.parameters.count("upper_limit") ? comp.parameters.at("upper_limit") : "1";
            cObj["lower_limit"] = comp.parameters.count("lower_limit") ? comp.parameters.at("lower_limit") : "-1";
            cObj["anti_windup"] = comp.parameters.count("anti_windup") ? comp.parameters.at("anti_windup") : "false";
            cObj["input"] = getIncomingSignal(comp.id, "In");
            ctrlLoopsObj["pid_controllers"].push_back(cObj);
        } else if (t == "PLL_1PH") {
            cObj["output_theta"] = comp.id + ".Theta";
            cObj["output_freq"] = comp.id + ".Freq";
            cObj["output_cos"] = comp.id + ".Cos";
            cObj["output_sin"] = comp.id + ".Sin";
            cObj["original_type"] = "PLL_1PH";
            cObj["fn"] = formatJSStyleDouble(parsedParams.count("fn") ? parsedParams["fn"] : 50.0);
            cObj["Kp"] = formatJSStyleDouble(parsedParams.count("Kp") ? parsedParams["Kp"] : 20.0);
            cObj["Ki"] = formatJSStyleDouble(parsedParams.count("Ki") ? parsedParams["Ki"] : 1000.0);
            cObj["input"] = getIncomingSignal(comp.id, "In");
            ctrlLoopsObj["plls"].push_back(cObj);
        } else if (t == "PLL_3PH") {
            cObj["output_theta"] = comp.id + ".Theta";
            cObj["output_freq"] = comp.id + ".Freq";
            cObj["output_cos"] = comp.id + ".Cos";
            cObj["output_sin"] = comp.id + ".Sin";
            cObj["original_type"] = "PLL_3PH";
            cObj["fn"] = formatJSStyleDouble(parsedParams.count("fn") ? parsedParams["fn"] : 50.0);
            cObj["Kp"] = formatJSStyleDouble(parsedParams.count("Kp") ? parsedParams["Kp"] : 20.0);
            cObj["Ki"] = formatJSStyleDouble(parsedParams.count("Ki") ? parsedParams["Ki"] : 1000.0);
            json inputsArr = json::array();
            inputsArr.push_back(getIncomingSignal(comp.id, "A"));
            inputsArr.push_back(getIncomingSignal(comp.id, "B"));
            inputsArr.push_back(getIncomingSignal(comp.id, "C"));
            cObj["inputs"] = inputsArr;
            ctrlLoopsObj["plls"].push_back(cObj);
        } else if (t == "GAIN") {
            cObj["output"] = comp.id + ".Out";
            cObj["original_type"] = "GAIN";
            double kVal = 1.0;
            if (comp.parameters.count("K")) kVal = CircuitSimEngine::ExpressionEvaluator::parseScientific(comp.parameters.at("K"));
            else if (comp.parameters.count("gain")) kVal = CircuitSimEngine::ExpressionEvaluator::parseScientific(comp.parameters.at("gain"));
            else if (comp.parameters.count("k")) kVal = CircuitSimEngine::ExpressionEvaluator::parseScientific(comp.parameters.at("k"));
            cObj["K"] = formatJSStyleDouble(roundToDigits(kVal, 9));

            std::string inSig = getIncomingSignal(comp.id, "In");
            if (inSig == "0.0") inSig = getIncomingSignal(comp.id, "In1");
            cObj["input"] = inSig;
            cObj["input1"] = "0.0";
            cObj["input2"] = "0.0";
            ctrlLoopsObj["gains"].push_back(cObj);
        } else if (t == "TRI" || t == "TRI_GEN" || t == "TRIANGLE" || t == "TRIANGLE_CARRIER" || comp.type == ComponentType::Triangle_Carrier) {
            cObj["output"] = comp.id + ".Out";
            double freq = 10000.0;
            if (comp.parameters.count("frequency")) freq = CircuitSimEngine::ExpressionEvaluator::parseScientific(comp.parameters.at("frequency"));
            else if (comp.parameters.count("freq")) freq = CircuitSimEngine::ExpressionEvaluator::parseScientific(comp.parameters.at("freq"));

            // min, max and amplitude declare "" so the registry leaves them unset unless the user
            // sets them. amplitude A gives -A..+A. An explicit min or max overrides that side.
            // With nothing set the range is 0..1.
            auto isSet = [&](const char* k) {
                auto it = comp.parameters.find(k);
                return it != comp.parameters.end() && !it->second.empty();
            };
            double minV = 0.0;
            double maxV = 1.0;
            if (isSet("amplitude")) {
                const double amp = CircuitSimEngine::ExpressionEvaluator::parseScientific(comp.parameters.at("amplitude"));
                minV = -amp;
                maxV = amp;
            }
            if (isSet("min")) minV = CircuitSimEngine::ExpressionEvaluator::parseScientific(comp.parameters.at("min"));
            if (isSet("max")) maxV = CircuitSimEngine::ExpressionEvaluator::parseScientific(comp.parameters.at("max"));

            cObj["frequency"] = formatJSStyleDouble(roundToDigits(freq, 9));
            cObj["min"] = formatJSStyleDouble(roundToDigits(minV, 9));
            cObj["max"] = formatJSStyleDouble(roundToDigits(maxV, 9));
            cObj["phase_source"] = comp.parameters.count("phase_source") ? comp.parameters.at("phase_source") : "internal";
            cObj["phase"] = comp.parameters.count("phase") ? comp.parameters.at("phase") : "0";
            cObj["freq_source"] = comp.parameters.count("freq_source") ? comp.parameters.at("freq_source") : "internal";
            cObj["input_phase"] = nullptr;
            cObj["input_freq"] = nullptr;
            ctrlLoopsObj["triangle_carriers"].push_back(cObj);
        } else if (t == "PID" || t == "PI" || t == "PI_CONTROLLER") {
            cObj["output"] = comp.id + ".Out";
            cObj["original_type"] = "PID";
            cObj["Kp"] = formatJSStyleDouble(parsedParams.count("Kp") ? parsedParams["Kp"] : 1.0);
            cObj["Ki"] = formatJSStyleDouble(parsedParams.count("Ki") ? parsedParams["Ki"] : 0.0);
            cObj["Kd"] = formatJSStyleDouble(parsedParams.count("Kd") ? parsedParams["Kd"] : 0.0);
            cObj["input"] = getIncomingSignal(comp.id, "In");
            ctrlLoopsObj["pi_controllers"].push_back(cObj);
        } else if (t == "SUM" || t == "SUM_RECT" || t == "SUM_ROUND" || t == "SUBTRACT") {
            cObj["output"] = comp.id + ".Out";
            cObj["original_type"] = comp.rawTypeStr;

            std::string inputParam = comp.parameters.count("inputs") ? comp.parameters.at("inputs") : "";
            if (inputParam.empty() && comp.parameters.count("signs")) inputParam = comp.parameters.at("signs");
            if (inputParam.empty() && comp.parameters.count("num_inputs")) inputParam = comp.parameters.at("num_inputs");
            if (inputParam.empty()) inputParam = (t == "SUBTRACT" ? "+-" : "++");

            int nPins = 2;
            std::string signsStr = "";
            bool isNumeric = !inputParam.empty();
            for (char c : inputParam) { if (!std::isdigit((unsigned char)c)) { isNumeric = false; break; } }

            if (isNumeric) {
                try { nPins = std::clamp(std::stoi(inputParam), 1, 32); } catch (...) { nPins = 2; }
                signsStr = std::string(nPins, '+');
            } else {
                nPins = (int)inputParam.length();
                signsStr = inputParam;
            }

            cObj["signs"] = signsStr;
            cObj["operators"] = signsStr;

            json inputsArr = json::array();
            for (int i = 1; i <= nPins; ++i) {
                std::string sigKey = getIncomingSignal(comp.id, "In" + std::to_string(i));
                if (sigKey == "0.0" && i == 1) {
                    std::string sigAlt = getIncomingSignal(comp.id, "A");
                    if (sigAlt != "0.0") sigKey = sigAlt;
                }
                if (sigKey == "0.0" && i == 2) {
                    std::string sigAlt = getIncomingSignal(comp.id, "B");
                    if (sigAlt != "0.0") sigKey = sigAlt;
                }
                inputsArr.push_back(sigKey);
                cObj["In" + std::to_string(i)] = sigKey;
                cObj["input_" + std::to_string(i - 1)] = sigKey;
            }
            cObj["inputs"] = inputsArr;
            ctrlLoopsObj["summing_junctions"].push_back(cObj);
        } else if (t == "PROD" || t == "PRODUCT_RECT" || t == "PRODUCT") {
            cObj["output"] = comp.id + ".Out";
            cObj["original_type"] = comp.rawTypeStr;

            std::string inputParam = comp.parameters.count("inputs") ? comp.parameters.at("inputs") : "";
            if (inputParam.empty() && comp.parameters.count("operators")) inputParam = comp.parameters.at("operators");
            if (inputParam.empty() && comp.parameters.count("num_inputs")) inputParam = comp.parameters.at("num_inputs");
            if (inputParam.empty()) inputParam = "**";

            int nPins = 2;
            std::string opsStr = "";
            bool isNumeric = !inputParam.empty();
            for (char c : inputParam) { if (!std::isdigit((unsigned char)c)) { isNumeric = false; break; } }

            if (isNumeric) {
                try { nPins = std::clamp(std::stoi(inputParam), 1, 32); } catch (...) { nPins = 2; }
                opsStr = std::string(nPins, '*');
            } else {
                nPins = (int)inputParam.length();
                opsStr = inputParam;
            }

            cObj["operators"] = opsStr;
            cObj["signs"] = opsStr;

            json inputsArr = json::array();
            for (int i = 1; i <= nPins; ++i) {
                std::string sigKey = getIncomingSignal(comp.id, "In" + std::to_string(i));
                inputsArr.push_back(sigKey);
                cObj["In" + std::to_string(i)] = sigKey;
                cObj["input_" + std::to_string(i - 1)] = sigKey;
            }
            cObj["inputs"] = inputsArr;
            ctrlLoopsObj["product_blocks"].push_back(cObj);
        } else if (t == "PWM" || t == "PWM_GENERATOR") {
            cObj["output"] = comp.id + ".Out";
            cObj["original_type"] = "PWM";
            // Registry defaults: frequency 10 kHz, min 0, max 1. The generator used to
            // default to 20 kHz and never forwarded min/max, so those were ignored.
            cObj["frequency"] = formatJSStyleDouble(parsedParams.count("frequency") ? parsedParams["frequency"] : 10000.0);
            cObj["min"] = formatJSStyleDouble(parsedParams.count("min") ? parsedParams["min"] : 0.0);
            cObj["max"] = formatJSStyleDouble(parsedParams.count("max") ? parsedParams["max"] : 1.0);
            cObj["input"] = getIncomingSignal(comp.id, "In");
            ctrlLoopsObj["pwm_generators"].push_back(cObj);
        } else if (t == "PWM_MASTER" || t == "MASTER_PWM") {
            cObj["original_type"] = "PWM_MASTER";
            // Carrier frequency. The JSON parameter is "frequency"; fc, carrier_freq and freq are
            // accepted too. Only "fc" used to be read, so "frequency" was silently ignored.
            std::string carrier = "10k";
            for (const char* key : {"fc", "carrier_freq", "frequency", "freq"}) {
                if (comp.parameters.count(key) && !comp.parameters.at(key).empty()) {
                    carrier = comp.parameters.at(key);
                    break;
                }
            }
            cObj["fc"] = carrier;
            cObj["dead_time"] = comp.parameters.count("dead_time") ? comp.parameters.at("dead_time") : "1u";
            int nCarriers = 3;
            if (comp.parameters.count("num_carriers")) {
                try { nCarriers = std::stoi(comp.parameters.at("num_carriers")); } catch (...) { nCarriers = 3; }
            }
            if (nCarriers < 1) nCarriers = 1;
            if (nCarriers > 32) nCarriers = 32;
            cObj["num_carriers"] = std::to_string(nCarriers);
            // Carrier range. The engine scales the triangle to [min, max].
            cObj["min"] = comp.parameters.count("min") ? comp.parameters.at("min") : "0";
            cObj["max"] = comp.parameters.count("max") ? comp.parameters.at("max") : "1";
            if (comp.parameters.count("config")) cObj["config"] = comp.parameters.at("config");
            // Modulating inputs In1..InN, one per carrier. They were never emitted, so every
            // carrier compared against 0 and the outputs stayed low.
            for (int k = 1; k <= nCarriers; ++k) {
                const std::string pin = "In" + std::to_string(k);
                cObj[pin] = getIncomingSignal(comp.id, pin);
            }
            ctrlLoopsObj["pwm_masters"].push_back(cObj);
        } else if (t == "COMP" || t == "COMPARATOR") {
            cObj["output"] = comp.id + ".Out";
            std::string inPlus = getIncomingSignal(comp.id, "Plus");
            if (inPlus == "0.0") inPlus = getIncomingSignal(comp.id, "In1");
            if (inPlus == "0.0") inPlus = getIncomingSignal(comp.id, "In");

            std::string inMinus = getIncomingSignal(comp.id, "Minus");
            if (inMinus == "0.0") inMinus = getIncomingSignal(comp.id, "In2");

            json inputsArr = json::array();
            inputsArr.push_back(inPlus);
            inputsArr.push_back(inMinus);
            cObj["inputs"] = inputsArr;

            double hyst = 0.0;
            if (comp.parameters.count("hysteresis")) hyst = CircuitSimEngine::ExpressionEvaluator::parseScientific(comp.parameters.at("hysteresis"));
            cObj["hysteresis"] = formatJSStyleDouble(roundToDigits(hyst, 9));

            ctrlLoopsObj["comparators"].push_back(cObj);
        } else if (t == "AND" || t == "OR" || t == "NOT") {
            cObj["output"] = comp.id + ".Out";
            cObj["type"] = (t == "AND") ? "and" : ((t == "OR") ? "or" : "not");
            json inputsArr = json::array();
            if (t == "NOT") {
                std::string inSig = getIncomingSignal(comp.id, "In");
                if (inSig == "0.0") inSig = getIncomingSignal(comp.id, "In1");
                inputsArr.push_back(inSig);
            } else {
                std::string inA = getIncomingSignal(comp.id, "A");
                if (inA == "0.0") inA = getIncomingSignal(comp.id, "In1");
                std::string inB = getIncomingSignal(comp.id, "B");
                if (inB == "0.0") inB = getIncomingSignal(comp.id, "In2");
                inputsArr.push_back(inA);
                inputsArr.push_back(inB);
            }
            cObj["inputs"] = inputsArr;
            ctrlLoopsObj["logic_gates"].push_back(cObj);
        } else if (t == "CSCRIPT" || t == "CUSTOMSCRIPT") {
            std::string scriptCode = comp.parameters.count("code") ? comp.parameters.at("code") : "";

            auto discParams = CircuitSimEngine::CScriptEngine::discoverParamsFromCode(scriptCode);
            for (const auto& dp : discParams) {
                if (comp.parameters.count(dp.name)) {
                    std::string pValStr = comp.parameters.at(dp.name);
                    try {
                        double nVal = CircuitSimEngine::ExpressionEvaluator::parseScientific(pValStr);
                        scriptCode = CircuitSimEngine::CScriptEngine::updateParamInCode(scriptCode, dp.name, nVal);
                    } catch (...) {}
                }
            }
            cObj["code"] = scriptCode;

            std::vector<CircuitSimEngine::CScriptPort> discIn, discOut;
            CircuitSimEngine::CScriptEngine::discoverPorts(scriptCode, discIn, discOut);

            // Inputs are positional (inputs[k] <-> array slot k), so order them by
            // script index and keep a "0.0" placeholder for an unconnected port.
            // Skipping it shifted every later input down one slot: with In1 open,
            // inputs[0] silently read the signal wired to In2.
            int maxInIdx = -1;
            for (const auto& p : discIn) maxInIdx = std::max(maxInIdx, p.index);
            std::vector<std::string> inByIdx(maxInIdx + 1);
            for (const auto& p : discIn) {
                if (p.index >= 0 && p.index <= maxInIdx && inByIdx[p.index].empty()) inByIdx[p.index] = p.name;
            }
            json inputsArr = json::array();
            for (int k = 0; k <= maxInIdx; ++k) {
                const std::string portName = inByIdx[k].empty() ? ("In" + std::to_string(k + 1)) : inByIdx[k];
                std::string inSig = getIncomingSignal(comp.id, portName);
                if (inSig == "0.0") inSig = getIncomingSignal(comp.id, "In" + std::to_string(k + 1));
                if (inSig.empty()) inSig = "0.0";
                inputsArr.push_back(inSig);
            }
            // Drop trailing placeholders so a fully unconnected script stays as before.
            while (!inputsArr.empty() && inputsArr.back() == "0.0") inputsArr.erase(inputsArr.size() - 1);
            cObj["inputs"] = inputsArr;

            // Outputs are positional too: order by script index (gaps get OutN names).
            json outputsArr = json::array();
            if (!discOut.empty()) {
                int maxOutIdx = -1;
                for (const auto& p : discOut) maxOutIdx = std::max(maxOutIdx, p.index);
                std::vector<std::string> outByIdx(maxOutIdx + 1);
                for (const auto& p : discOut) {
                    if (p.index >= 0 && p.index <= maxOutIdx && outByIdx[p.index].empty()) outByIdx[p.index] = p.name;
                }
                for (int k = 0; k <= maxOutIdx; ++k) {
                    outputsArr.push_back(comp.id + "." + (outByIdx[k].empty() ? ("Out" + std::to_string(k + 1)) : outByIdx[k]));
                }
            } else {
                outputsArr.push_back(comp.id + ".Out1");
            }
            cObj["outputs"] = outputsArr;

            std::string tsStr = comp.parameters.count("timestep") ? comp.parameters.at("timestep") : "0";
            cObj["timestep"] = tsStr;

            for (const auto& [pk, pv] : comp.parameters) {
                if (pk != "code" && pk != "timestep" && pk != "id" && pk != "type" && pk != "num_inputs" && pk != "num_outputs" && pk != "original_type") {
                    cObj[pk] = pv;
                }
            }

            ctrlLoopsObj["custom_scripts"].push_back(cObj);
        } else if (t == "INPORT" || t == "IN") {
            cObj["original_type"] = "INPORT";
            cObj["output"] = comp.id + ".Out";
            std::string pNum = comp.parameters.count("port_number") ? comp.parameters.at("port_number") : "1";
            cObj["port_number"] = pNum;
            ctrlLoopsObj["ports"].push_back(cObj);
        } else if (t == "OUTPORT" || t == "OUT") {
            cObj["original_type"] = "OUTPORT";
            std::string inSig = getIncomingSignal(comp.id, "In");
            cObj["input"] = inSig;
            std::string pNum = comp.parameters.count("port_number") ? comp.parameters.at("port_number") : "1";
            cObj["port_number"] = pNum;
            ctrlLoopsObj["ports"].push_back(cObj);
        } else if (t == "BUS_CREATOR") {
            cObj["original_type"] = "BUS_CREATOR";
            cObj["output"] = comp.id + ".Bus";
            int n = 2;
            if (comp.parameters.count("inputs")) {
                try { n = std::stoi(comp.parameters.at("inputs")); } catch (...) {}
            }
            json inputsArr = json::array();
            for (int i = 1; i <= n; ++i) {
                inputsArr.push_back(getIncomingSignal(comp.id, "In" + std::to_string(i)));
            }
            cObj["inputs"] = inputsArr;
            ctrlLoopsObj["buses"].push_back(cObj);
        } else if (t == "BUS_SELECTOR") {
            cObj["original_type"] = "BUS_SELECTOR";
            cObj["input"] = getIncomingSignal(comp.id, "Bus");
            cObj["signals"] = comp.parameters.count("signals") ? comp.parameters.at("signals") : "";
            ctrlLoopsObj["buses"].push_back(cObj);
            // A bus carries one signal per creator input. The engine passes only one value
            // through a bus, so each selector output is wired directly to the creator input
            // it reads: a unity gain from that input's signal. Output k is <id>.Outk.
            const std::string busKey = getIncomingSignal(comp.id, "Bus");
            size_t nSignals = 1;
            if (comp.parameters.count("signals")) {
                nSignals = 1;
                for (char ch : comp.parameters.at("signals")) if (ch == ',') ++nSignals;
            }
            for (const auto& creator : tempDesign.components) {
                if (upperStr(creator.rawTypeStr) != "BUS_CREATOR") continue;
                if (busKey != creator.id + ".Bus") continue;
                for (size_t k = 1; k <= nSignals; ++k) {
                    json gObj;
                    gObj["output"] = comp.id + ".Out" + std::to_string(k);
                    gObj["original_type"] = "GAIN";
                    const std::string src = getIncomingSignal(creator.id, "In" + std::to_string(k));
                    gObj["input"] = src;
                    gObj["input1"] = src;
                    gObj["input2"] = "0.0";
                    gObj["K"] = 1.0;
                    ctrlLoopsObj["gains"].push_back(gObj);
                }
            }
        } else if (const BlockDef* nb = BlockRegistry::instance().netlistBlockFor(comp.rawTypeStr)) {
            // No hand-written emitter: the block registry describes the entry
            // (data/blocks/*.json, "netlist"). See docs/adding-a-block.md.
            auto expand = [&](const std::string& tpl) {
                std::string out;
                for (size_t i = 0; i < tpl.size();) {
                    if (tpl.compare(i, 4, "{id}") == 0) { out += comp.id; i += 4; }
                    else if (tpl.compare(i, 6, "{type}") == 0) { out += comp.rawTypeStr; i += 6; }
                    else out += tpl[i++];
                }
                return out;
            };
            for (const auto& f : nb->netlist.fields) {
                using K = NetlistField::Kind;
                switch (f.kind) {
                    case K::Text: cObj[f.key] = expand(f.text); break;
                    case K::Number:
                        if (f.isInt) cObj[f.key] = (int)f.number; else cObj[f.key] = f.number;
                        break;
                    case K::Signal: cObj[f.key] = getIncomingSignal(comp.id, f.signals[0]); break;
                    case K::SignalArray: {
                        json arr = json::array();
                        for (const auto& pin : f.signals) arr.push_back(getIncomingSignal(comp.id, pin));
                        cObj[f.key] = arr;
                        break;
                    }
                    case K::Nodes: cObj[f.key] = formattedNodes; break;
                    case K::Param:
                        if (f.asString) {
                            cObj[f.key] = comp.parameters.count(f.text) ? comp.parameters.at(f.text) : f.defaultText;
                        } else if (f.isInt) {
                            cObj[f.key] = parsedParams.count(f.text) ? (int)parsedParams[f.text] : (int)f.number;
                        } else {
                            cObj[f.key] = formatJSStyleDouble(parsedParams.count(f.text) ? parsedParams[f.text] : f.number);
                        }
                        break;
                }
            }
            (nb->netlist.stage == "control_loops" ? ctrlLoopsObj : physStageObj)[nb->netlist.section].push_back(cObj);
        }
    }

    // ─── Ground Detection & Contiguous Node Re-indexing Pass ───
    std::string groundNodeStr = "";

    auto checkGround = [&](const json& compList) {
        for (const auto& item : compList) {
            if (item.contains("nodes") && item["nodes"].is_array()) {
                for (const auto& nVal : item["nodes"]) {
                    std::string nStr = nVal.get<std::string>();
                    if (nStr == "0" || nStr == "node_0") {
                        groundNodeStr = nStr;
                        return;
                    }
                }
            }
        }
    };

    checkGround(physStageObj["voltage_sources"]);
    if (groundNodeStr.empty()) checkGround(physStageObj["resistors"]);
    if (groundNodeStr.empty()) checkGround(physStageObj["capacitors"]);
    if (groundNodeStr.empty()) checkGround(physStageObj["diodes"]);
    if (groundNodeStr.empty()) checkGround(physStageObj["inductors"]);
    if (groundNodeStr.empty()) checkGround(physStageObj["analog_switches"]);

    // If no explicit "0" or "node_0" was found, default Ground to negative pin (nodes[1]) of primary Voltage Source V1
    if (groundNodeStr.empty()) {
        for (const auto& item : physStageObj["voltage_sources"]) {
            if (item.contains("nodes") && item["nodes"].is_array() && item["nodes"].size() >= 2) {
                groundNodeStr = item["nodes"][1].get<std::string>();
                break;
            }
        }
    }

    std::unordered_map<std::string, std::string> nodeRemap;
    if (!groundNodeStr.empty()) {
        nodeRemap[groundNodeStr] = "node_0";
    }
    nodeRemap["0"] = "node_0";
    nodeRemap["node_0"] = "node_0";

    int physNodeCounter = 1;

    auto compactNodes = [&](json& compList) {
        for (auto& item : compList) {
            if (item.contains("nodes") && item["nodes"].is_array()) {
                json newNodes = json::array();
                for (const auto& nVal : item["nodes"]) {
                    std::string nStr = nVal.get<std::string>();
                    if (nodeRemap.find(nStr) == nodeRemap.end()) {
                        nodeRemap[nStr] = "node_" + std::to_string(physNodeCounter++);
                    }
                    newNodes.push_back(nodeRemap[nStr]);
                }
                item["nodes"] = newNodes;
            }
        }
    };

    compactNodes(physStageObj["voltage_sources"]);
    compactNodes(physStageObj["inductors"]);
    compactNodes(physStageObj["analog_switches"]);

    for (auto& xfmr : physStageObj["transformers"]) {
        if (xfmr.contains("primary_windings") && xfmr["primary_windings"].is_array()) {
            for (auto& pw : xfmr["primary_windings"]) {
                if (pw.contains("nodes") && pw["nodes"].is_array()) {
                    json newNodes = json::array();
                    for (const auto& nVal : pw["nodes"]) {
                        std::string nStr = nVal.get<std::string>();
                        if (nodeRemap.find(nStr) == nodeRemap.end()) {
                            nodeRemap[nStr] = "node_" + std::to_string(physNodeCounter++);
                        }
                        newNodes.push_back(nodeRemap[nStr]);
                    }
                    pw["nodes"] = newNodes;
                }
            }
        }
        if (xfmr.contains("secondary_windings") && xfmr["secondary_windings"].is_array()) {
            for (auto& sw : xfmr["secondary_windings"]) {
                if (sw.contains("nodes") && sw["nodes"].is_array()) {
                    json newNodes = json::array();
                    for (const auto& nVal : sw["nodes"]) {
                        std::string nStr = nVal.get<std::string>();
                        if (nodeRemap.find(nStr) == nodeRemap.end()) {
                            nodeRemap[nStr] = "node_" + std::to_string(physNodeCounter++);
                        }
                        newNodes.push_back(nodeRemap[nStr]);
                    }
                    sw["nodes"] = newNodes;
                }
            }
        }
    }

    compactNodes(physStageObj["diodes"]);
    compactNodes(physStageObj["capacitors"]);
    compactNodes(physStageObj["resistors"]);
    compactNodes(physStageObj["current_sources"]);
    compactNodes(physStageObj["switches"]);
    compactNodes(physStageObj["voltmeters"]);
    compactNodes(physStageObj["ammeters"]);

    root["physical_stage"] = physStageObj;
    root["control_loops"] = ctrlLoopsObj;

    json simParamsObj;
    double rawStopTime = (tempDesign.settings.stopTime > 0.0) ? tempDesign.settings.stopTime : 0.01;
    double rawStepSize = (tempDesign.settings.stepSize > 0.0) ? tempDesign.settings.stepSize : 0.000001;

    simParamsObj["stop_time"] = formatJSStyleDouble(rawStopTime);
    simParamsObj["step_size"] = formatJSStyleDouble(rawStepSize);

    simParamsObj["solver"] = tempDesign.settings.solverType.empty() ? "euler" : tempDesign.settings.solverType;
    const std::string stepTypeOut =
        tempDesign.settings.stepType.empty() ? "fixed" : tempDesign.settings.stepType;
    simParamsObj["step_type"] = stepTypeOut;
    simParamsObj["solverMethod"] = "non-ideal";
    simParamsObj["engine"] = "auto";
    simParamsObj["enable_lu_cache"] = tempDesign.settings.enableLUCache;

    // Adaptive-stepping controls. Emitted only for variable stepping so a fixed-step
    // netlist stays exactly as it was and keeps diffing cleanly against older files.
    if (stepTypeOut == "variable" || stepTypeOut == "adaptive") {
        simParamsObj["rel_tol"] = formatJSStyleDouble(
            (tempDesign.settings.relTol > 0.0) ? tempDesign.settings.relTol : 1e-3);
        simParamsObj["abs_tol_v"] = formatJSStyleDouble(
            (tempDesign.settings.absTolV > 0.0) ? tempDesign.settings.absTolV : 1e-3);
        simParamsObj["abs_tol_i"] = formatJSStyleDouble(
            (tempDesign.settings.absTolI > 0.0) ? tempDesign.settings.absTolI : 1e-6);
        // Zero means "derive automatically" and is passed through as such.
        simParamsObj["h_min"] = formatJSStyleDouble(tempDesign.settings.hMin);
        simParamsObj["h_max"] = formatJSStyleDouble(tempDesign.settings.hMax);
    }

    // ─── Component-Aware Wanted Variables Generation (with Series Current & CScript Output Pruning) ───
    struct CompNodes { std::string id; std::string t; std::string n1; std::string n2; };
    std::vector<CompNodes> physComps;
    std::unordered_map<std::string, std::vector<std::string>> nodeToComps;
    std::unordered_map<std::string, int> compPriority;

    for (const auto& comp : tempDesign.components) {
        std::string t = comp.rawTypeStr;
        std::transform(t.begin(), t.end(), t.begin(), ::toupper);

        int prio = 0;
        if (t == "L" || t == "INDUCTOR") prio = 10;
        else if (t == "C" || t == "CAPACITOR") prio = 8;
        else if (t == "D" || t == "DIODE") prio = 6;
        else if (t == "MOSFET" || t == "S" || t == "SWITCH") prio = 5;
        else if (t == "R" || t == "RESISTOR") prio = 4;
        else if (t == "V" || t == "AC_V" || t == "VOLTAGESOURCE") prio = 2;

        compPriority[comp.id] = prio;

        if (comp.nodes.size() >= 2) {
            std::string n1 = comp.nodes[0];
            std::string n2 = comp.nodes[1];
            physComps.push_back({comp.id, t, n1, n2});
            nodeToComps[n1].push_back(comp.id);
            nodeToComps[n2].push_back(comp.id);
        }
    }

    std::unordered_map<std::string, std::string> parent;
    for (const auto& pc : physComps) parent[pc.id] = pc.id;

    std::function<std::string(const std::string&)> findRoot = [&](const std::string& i) -> std::string {
        if (parent[i] == i) return i;
        return parent[i] = findRoot(parent[i]);
    };

    auto unionSet = [&](const std::string& a, const std::string& b) {
        std::string rA = findRoot(a);
        std::string rB = findRoot(b);
        if (rA != rB) {
            if (compPriority[rA] >= compPriority[rB]) parent[rB] = rA;
            else parent[rA] = rB;
        }
    };

    for (const auto& [nodeName, compList] : nodeToComps) {
        if (compList.size() == 2) {
            unionSet(compList[0], compList[1]);
        }
    }

    std::unordered_set<std::string> chosenSeriesRep;
    for (const auto& pc : physComps) {
        std::string rootComp = findRoot(pc.id);
        chosenSeriesRep.insert(rootComp);
    }

    json wantedVars = json::array();
    std::unordered_set<std::string> addedVars;

    for (const auto& comp : tempDesign.components) {
        std::string t = comp.rawTypeStr;
        std::transform(t.begin(), t.end(), t.begin(), ::toupper);

        if (t == "C" || t == "CAPACITOR") {
            std::string varName = "V_" + comp.id;
            if (!addedVars.count(varName)) { wantedVars.push_back(varName); addedVars.insert(varName); }
        } else if (t == "L" || t == "INDUCTOR" || t == "D" || t == "DIODE" || t == "MOSFET" || t == "S") {
            if (chosenSeriesRep.count(comp.id)) {
                std::string varName = "I_" + comp.id;
                if (!addedVars.count(varName)) { wantedVars.push_back(varName); addedVars.insert(varName); }
            }
        } else if (t == "PULSE" || t == "PULSE_GEN" || t == "PWM" || t == "CONST" || t == "CONSTANT" || t == "GAIN" || t == "TRI" || t == "TRI_GEN" || t == "TRIANGLE" || comp.type == ComponentType::Triangle_Carrier) {
            std::string varName = comp.id + ".Out";
            if (!addedVars.count(varName)) { wantedVars.push_back(varName); addedVars.insert(varName); }
        } else if (t == "CSCRIPT") {
            std::string scriptCode = comp.parameters.count("code") ? comp.parameters.at("code") : "";
            std::vector<CircuitSimEngine::CScriptPort> discIn, discOut;
            CircuitSimEngine::CScriptEngine::discoverPorts(scriptCode, discIn, discOut);
            for (const auto& op : discOut) {
                std::string varName = comp.id + "." + op.name;
                if (!addedVars.count(varName)) { wantedVars.push_back(varName); addedVars.insert(varName); }
            }
        } else if (t == "SCOPE" || t == "OSCILLOSCOPE") {
            int numChannels = 2;
            if (comp.parameters.count("channels")) {
                try { numChannels = std::stoi(comp.parameters.at("channels")); } catch (...) {}
            }
            for (int ch = 1; ch <= std::max(1, numChannels); ++ch) {
                std::string inSig = getIncomingSignal(comp.id, "In" + std::to_string(ch));
                if (inSig != "0.0") {
                    if (!addedVars.count(inSig)) { wantedVars.push_back(inSig); addedVars.insert(inSig); }
                } else {
                    std::string varName = comp.id + ".In" + std::to_string(ch);
                    if (!addedVars.count(varName)) { wantedVars.push_back(varName); addedVars.insert(varName); }
                }
            }
        }
    }

    simParamsObj["wanted_variables"] = wantedVars;
    root["simulation_parameters"] = simParamsObj;

    root["probes"] = json::array();
    for (const auto& comp : tempDesign.components) {
        std::string t = comp.rawTypeStr;
        std::transform(t.begin(), t.end(), t.begin(), ::toupper);
        if (t == "SCOPE" || t == "OSCILLOSCOPE") {
            json pObj;
            pObj["id"] = comp.id;
            json inTraces = json::array();
            int numChannels = 2;
            if (comp.parameters.count("channels")) {
                try { numChannels = std::stoi(comp.parameters.at("channels")); } catch (...) {}
            }
            for (int ch = 1; ch <= std::max(1, numChannels); ++ch) {
                std::string inSig = getIncomingSignal(comp.id, "In" + std::to_string(ch));
                if (inSig != "0.0") {
                    inTraces.push_back(inSig);
                }
            }
            pObj["channels_inputs"] = inTraces;
            root["probes"].push_back(pObj);
        } else if (t == "PROBE" || t == "UNIFIEDPROBE") {
            json pObj;
            pObj["id"] = comp.id;
            pObj["target"] = comp.parameters.count("target") ? comp.parameters.at("target") : "";
            pObj["selected_signals"] = comp.parameters.count("selected_signals") ? comp.parameters.at("selected_signals") : "";
            root["probes"].push_back(pObj);
        }
    }

    return root.dump(2);
}

} // namespace CircuitSim
