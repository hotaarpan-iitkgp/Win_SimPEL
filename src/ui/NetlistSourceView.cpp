#include "NetlistSourceView.hpp"
#include "engine/NetlistBuilder.hpp"
#include "implot.h"
#include <nlohmann/json.hpp>
#include <windows.h>
#include <commdlg.h>
#include <fstream>
#include <iostream>
#include <algorithm>

#include "engine/ExpressionEvaluator.hpp"
#include "../engine/CScriptEngine.hpp"
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

std::string NetlistSourceView::generateNetlistJson(const CircuitDesign& design) {
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

    getSignalFromWire = [&](const std::string& wireId) -> std::string {
        for (const auto& w : tempDesign.wires) {
            if (w.id == wireId) {
                if (w.from.isWireJunction) {
                    return getSignalFromWire(w.from.targetWireId);
                } else if (!w.from.compId.empty()) {
                    std::string cId = w.from.compId;
                    std::string pName = w.from.terminal;
                    for (const auto& c : tempDesign.components) {
                        if (c.id == cId) {
                            std::string ct = c.rawTypeStr;
                            std::transform(ct.begin(), ct.end(), ct.begin(), ::toupper);
                            if (ct == "FROM_SIG" || ct == "FROM") {
                                std::string fromTag = c.parameters.count("tag") ? c.parameters.at("tag") : "A";
                                for (const auto& g : tempDesign.components) {
                                    std::string gt = g.rawTypeStr;
                                    std::transform(gt.begin(), gt.end(), gt.begin(), ::toupper);
                                    if (gt == "GOTO_SIG" || gt == "GOTO") {
                                        std::string gotoTag = g.parameters.count("tag") ? g.parameters.at("tag") : "A";
                                        if (gotoTag == fromTag) {
                                            return getIncomingSignal(g.id, "In");
                                        }
                                    }
                                }
                            }
                        }
                    }
                    return cId + "." + pName;
                }
            }
        }
        return "0.0";
    };

    getIncomingSignal = [&](const std::string& compId, const std::string& pinName) -> std::string {
        for (const auto& w : tempDesign.wires) {
            bool matchesTo = (w.to.compId == compId && (w.to.terminal == pinName ||
                (pinName == "In" && (w.to.terminal == "In" || w.to.terminal == "In1")) ||
                ((pinName == "A" || pinName == "Plus") && (w.to.terminal == "A" || w.to.terminal == "Plus" || w.to.terminal == "In1")) ||
                ((pinName == "B" || pinName == "Minus") && (w.to.terminal == "B" || w.to.terminal == "Minus" || w.to.terminal == "In2"))));
            bool matchesFrom = (w.from.compId == compId && (w.from.terminal == pinName ||
                (pinName == "In" && (w.from.terminal == "In" || w.from.terminal == "In1")) ||
                ((pinName == "A" || pinName == "Plus") && (w.from.terminal == "A" || w.from.terminal == "Plus" || w.from.terminal == "In1")) ||
                ((pinName == "B" || pinName == "Minus") && (w.from.terminal == "B" || w.from.terminal == "Minus" || w.from.terminal == "In2"))));

            if (matchesTo) {
                if (w.from.isWireJunction) {
                    return getSignalFromWire(w.from.targetWireId);
                }
                std::string otherCompId = w.from.compId;
                std::string otherPinName = w.from.terminal;
                if (!otherCompId.empty()) {
                    for (const auto& c : tempDesign.components) {
                        if (c.id == otherCompId) {
                            std::string ct = c.rawTypeStr;
                            std::transform(ct.begin(), ct.end(), ct.begin(), ::toupper);
                            if (ct == "FROM_SIG" || ct == "FROM") {
                                std::string fromTag = c.parameters.count("tag") ? c.parameters.at("tag") : "A";
                                for (const auto& g : tempDesign.components) {
                                    std::string gt = g.rawTypeStr;
                                    std::transform(gt.begin(), gt.end(), gt.begin(), ::toupper);
                                    if (gt == "GOTO_SIG" || gt == "GOTO") {
                                        std::string gotoTag = g.parameters.count("tag") ? g.parameters.at("tag") : "A";
                                        if (gotoTag == fromTag) {
                                            return getIncomingSignal(g.id, "In");
                                        }
                                    }
                                }
                            }
                        }
                    }
                    return otherCompId + "." + otherPinName;
                }
            } else if (matchesFrom) {
                if (w.to.isWireJunction) {
                    return getSignalFromWire(w.to.targetWireId);
                }
                std::string otherCompId = w.to.compId;
                std::string otherPinName = w.to.terminal;
                if (!otherCompId.empty()) {
                    for (const auto& c : tempDesign.components) {
                        if (c.id == otherCompId) {
                            std::string ct = c.rawTypeStr;
                            std::transform(ct.begin(), ct.end(), ct.begin(), ::toupper);
                            if (ct == "FROM_SIG" || ct == "FROM") {
                                std::string fromTag = c.parameters.count("tag") ? c.parameters.at("tag") : "A";
                                for (const auto& g : tempDesign.components) {
                                    std::string gt = g.rawTypeStr;
                                    std::transform(gt.begin(), gt.end(), gt.begin(), ::toupper);
                                    if (gt == "GOTO_SIG" || gt == "GOTO") {
                                        std::string gotoTag = g.parameters.count("tag") ? g.parameters.at("tag") : "A";
                                        if (gotoTag == fromTag) {
                                            return getIncomingSignal(g.id, "In");
                                        }
                                    }
                                }
                            }
                        }
                    }
                    return otherCompId + "." + otherPinName;
                }
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
            cObj["value"] = formatJSStyleDouble(parsedParams.count("value") ? parsedParams["value"] : 10.0);
            cObj["esr"] = formatJSStyleDouble(parsedParams.count("esr") ? parsedParams["esr"] : 0.0);
            cObj["src_type"] = "static";
            physStageObj["resistors"].push_back(cObj);
        } else if (t == "L" || t == "INDUCTOR") {
            cObj["nodes"] = formattedNodes;
            cObj["L"] = formatJSStyleDouble(parsedParams.count("L") ? parsedParams["L"] : 0.0001);
            cObj["esr"] = formatJSStyleDouble(parsedParams.count("esr") ? parsedParams["esr"] : 0.05);
            cObj["iL0"] = formatJSStyleDouble(parsedParams.count("iL0") ? parsedParams["iL0"] : 0.0);
            physStageObj["inductors"].push_back(cObj);
        } else if (t == "C" || t == "CAPACITOR") {
            cObj["nodes"] = formattedNodes;
            cObj["C"] = formatJSStyleDouble(parsedParams.count("C") ? parsedParams["C"] : 0.0001);
            cObj["esr"] = formatJSStyleDouble(parsedParams.count("esr") ? parsedParams["esr"] : 0.01);
            cObj["vC0"] = formatJSStyleDouble(parsedParams.count("vC0") ? parsedParams["vC0"] : 0.0);
            physStageObj["capacitors"].push_back(cObj);
        } else if (t == "V" || t == "VOLTAGESOURCE" || t == "DC_V") {
            cObj["nodes"] = formattedNodes;
            cObj["value"] = formatJSStyleDouble(parsedParams.count("value") ? parsedParams["value"] : 24.0);
            cObj["src_type"] = "dc";
            physStageObj["voltage_sources"].push_back(cObj);
        } else if (t == "I" || t == "CURRENTSOURCE" || t == "DC_I") {
            cObj["nodes"] = formattedNodes;
            cObj["value"] = formatJSStyleDouble(parsedParams.count("value") ? parsedParams["value"] : 1.0);
            cObj["src_type"] = "dc";
            physStageObj["current_sources"].push_back(cObj);
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
                json vsAB; vsAB["id"] = comp.id + "_AB"; vsAB["nodes"] = json::array({nodeA, nodeB}); vsAB["amplitude"] = formatJSStyleDouble(amp); vsAB["frequency"] = formatJSStyleDouble(freq); vsAB["phase"] = formatJSStyleDouble(phase); vsAB["type"] = "ac"; physStageObj["voltage_sources"].push_back(vsAB);
                json vsBC; vsBC["id"] = comp.id + "_BC"; vsBC["nodes"] = json::array({nodeB, nodeC}); vsBC["amplitude"] = formatJSStyleDouble(amp); vsBC["frequency"] = formatJSStyleDouble(freq); vsBC["phase"] = formatJSStyleDouble(phase - 120.0); vsBC["type"] = "ac"; physStageObj["voltage_sources"].push_back(vsBC);
                json vsCA; vsCA["id"] = comp.id + "_CA"; vsCA["nodes"] = json::array({nodeC, nodeA}); vsCA["amplitude"] = formatJSStyleDouble(amp); vsCA["frequency"] = formatJSStyleDouble(freq); vsCA["phase"] = formatJSStyleDouble(phase + 120.0); vsCA["type"] = "ac"; physStageObj["voltage_sources"].push_back(vsCA);
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
        } else if (t == "VM" || t == "VOLTMETER") {
            cObj["nodes"] = formattedNodes;
            cObj["signal"] = comp.id + ".Out";
            physStageObj["voltmeters"].push_back(cObj);
        } else if (t == "AM" || t == "AMMETER") {
            cObj["nodes"] = formattedNodes;
            cObj["signal"] = comp.id + ".Out";
            physStageObj["ammeters"].push_back(cObj);
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
            physStageObj["inductors"].push_back(cObj);
        } else if (t == "VAR_C" || t == "SAT_C") {
            cObj["nodes"] = formattedNodes;
            cObj["C"] = formatJSStyleDouble(parsedParams.count("C") ? parsedParams["C"] : 1e-4);
            cObj["esr"] = formatJSStyleDouble(parsedParams.count("esr") ? parsedParams["esr"] : 0.0);
            cObj["vC0"] = formatJSStyleDouble(parsedParams.count("vC0") ? parsedParams["vC0"] : 0.0);
            cObj["control_signal"] = getIncomingSignal(comp.id, "Ctrl");
            physStageObj["capacitors"].push_back(cObj);
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
        } else if (t == "S" || t == "BREAKER" || t == "SR_SWITCH" || t == "DBL_SWITCH" || t == "MAN_SWITCH" || t == "MAN_DBL_SWITCH" || t == "MAN_TRPL_SWITCH" || t == "TRPL_SWITCH") {
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
            std::string nOut = (formattedNodes.size() > 0) ? formattedNodes[0].get<std::string>() : "node_0";
            std::string nPlus = (formattedNodes.size() > 1) ? formattedNodes[1].get<std::string>() : "node_0";
            std::string nMinus = (formattedNodes.size() > 2) ? formattedNodes[2].get<std::string>() : "node_0";
            json vObj;
            vObj["id"] = comp.id;
            vObj["nodes"] = json::array({nOut, "node_0"});
            vObj["plus_node"] = nPlus;
            vObj["minus_node"] = nMinus;
            vObj["gain"] = formatJSStyleDouble(parsedParams.count("gain") ? parsedParams["gain"] : 1e5);
            vObj["value"] = formatJSStyleDouble(parsedParams.count("value") ? parsedParams["value"] : 12.0);
            vObj["src_type"] = (t == "OPAMP") ? "opamp" : "e_comp";
            physStageObj["voltage_sources"].push_back(vObj);
        } else if (t == "GEN_EBLOCK" || t == "GENERALIZEDEBLOCK") {
            json eObj;
            eObj["id"] = comp.id;
            eObj["nodes"] = formattedNodes;
            eObj["code"] = comp.parameters.count("code") ? comp.parameters.at("code") : "";
            eObj["timestep"] = comp.parameters.count("timestep") ? comp.parameters.at("timestep") : "0";
            eObj["terminals"] = std::to_string(formattedNodes.size());
            physStageObj["custom_eblocks"].push_back(eObj);
        } else if (t == "INDUCTION_MOTOR" || t == "IND_MOTOR") {
            std::string nA = (formattedNodes.size() > 0) ? formattedNodes[0].get<std::string>() : "node_0";
            std::string nB = (formattedNodes.size() > 1) ? formattedNodes[1].get<std::string>() : "node_0";
            std::string nC = (formattedNodes.size() > 2) ? formattedNodes[2].get<std::string>() : "node_0";
            std::string nN = comp.id + "_N";
            double rsVal = parsedParams.count("Rs") ? parsedParams["Rs"] : 1.115;

            json rA; rA["id"] = comp.id + "_RsA"; rA["nodes"] = json::array({nA, nN}); rA["value"] = formatJSStyleDouble(rsVal); rA["esr"] = 0.0; physStageObj["resistors"].push_back(rA);
            json rB; rB["id"] = comp.id + "_RsB"; rB["nodes"] = json::array({nB, nN}); rB["value"] = formatJSStyleDouble(rsVal); rB["esr"] = 0.0; physStageObj["resistors"].push_back(rB);
            json rC; rC["id"] = comp.id + "_RsC"; rC["nodes"] = json::array({nC, nN}); rC["value"] = formatJSStyleDouble(rsVal); rC["esr"] = 0.0; physStageObj["resistors"].push_back(rC);
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
            cObj["amplitude"] = formatJSStyleDouble(parsedParams.count("amplitude") ? parsedParams["amplitude"] : 1.0);
            cObj["frequency"] = formatJSStyleDouble(parsedParams.count("frequency") ? parsedParams["frequency"] : 50.0);
            cObj["phase"] = formatJSStyleDouble(parsedParams.count("phase") ? parsedParams["phase"] : 0.0);
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
        } else if (t == "TRIG_FCN") {
            cObj["output"] = comp.id + ".Out";
            cObj["original_type"] = "TRIG_FCN";
            cObj["function"] = comp.parameters.count("function") ? comp.parameters.at("function") : "sin";
            std::string inSig = getIncomingSignal(comp.id, "In");
            cObj["input"] = inSig;
            cObj["input1"] = inSig;
            cObj["input2"] = getIncomingSignal(comp.id, "In2");
            cObj["K"] = 1.0;
            ctrlLoopsObj["gains"].push_back(cObj);
        } else if (t == "ABS") {
            cObj["output"] = comp.id + ".Out";
            cObj["original_type"] = "ABS";
            std::string inSig = getIncomingSignal(comp.id, "In");
            cObj["input"] = inSig;
            cObj["input1"] = inSig;
            cObj["input2"] = "0.0";
            cObj["K"] = 1.0;
            ctrlLoopsObj["gains"].push_back(cObj);
        } else if (t == "SIGN") {
            cObj["output"] = comp.id + ".Out";
            cObj["original_type"] = "SIGN";
            std::string inSig = getIncomingSignal(comp.id, "In");
            cObj["input"] = inSig;
            cObj["input1"] = inSig;
            cObj["input2"] = "0.0";
            cObj["K"] = 1.0;
            ctrlLoopsObj["gains"].push_back(cObj);
        } else if (t == "ROUND") {
            cObj["output"] = comp.id + ".Out";
            cObj["original_type"] = "ROUND";
            cObj["mode"] = comp.parameters.count("mode") ? comp.parameters.at("mode") : "nearest";
            std::string inSig = getIncomingSignal(comp.id, "In");
            cObj["input"] = inSig;
            cObj["input1"] = inSig;
            cObj["input2"] = "0.0";
            cObj["K"] = 1.0;
            ctrlLoopsObj["gains"].push_back(cObj);
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
        } else if (t == "LUT_1D") {
            cObj["output"] = comp.id + ".Out";
            cObj["original_type"] = "LUT_1D";
            cObj["x"] = comp.parameters.count("x_data") ? comp.parameters.at("x_data") : "[0, 1]";
            cObj["y"] = comp.parameters.count("y_data") ? comp.parameters.at("y_data") : "[0, 1]";
            std::string inSig = getIncomingSignal(comp.id, "In");
            cObj["input"] = inSig;
            cObj["input1"] = inSig;
            cObj["input2"] = "0.0";
            cObj["K"] = 1.0;
            ctrlLoopsObj["gains"].push_back(cObj);
        } else if (t == "LUT_2D") {
            cObj["output"] = comp.id + ".Out";
            cObj["original_type"] = "LUT_2D";
            cObj["x"] = comp.parameters.count("x_data") ? comp.parameters.at("x_data") : "[0, 1]";
            cObj["y"] = comp.parameters.count("y_data") ? comp.parameters.at("y_data") : "[0, 1]";
            cObj["z"] = comp.parameters.count("z_data") ? comp.parameters.at("z_data") : "[[0, 1], [1, 2]]";
            std::string inSig = getIncomingSignal(comp.id, "In");
            cObj["input"] = inSig;
            cObj["input1"] = inSig;
            cObj["input2"] = "0.0";
            cObj["K"] = 1.0;
            ctrlLoopsObj["gains"].push_back(cObj);
        } else if (t == "LUT_3D") {
            cObj["output"] = comp.id + ".Out";
            cObj["original_type"] = "LUT_3D";
            std::string inSig = getIncomingSignal(comp.id, "In");
            cObj["input"] = inSig;
            cObj["input1"] = inSig;
            cObj["input2"] = "0.0";
            cObj["K"] = 1.0;
            ctrlLoopsObj["gains"].push_back(cObj);
        } else if (t == "INTEGRATOR") {
            cObj["output"] = comp.id + ".Out";
            cObj["original_type"] = "INTEGRATOR";
            cObj["initial_condition"] = formatJSStyleDouble(parsedParams.count("initial_condition") ? parsedParams["initial_condition"] : 0.0);
            std::string inSig = getIncomingSignal(comp.id, "In");
            cObj["input"] = inSig;
            cObj["input1"] = inSig;
            cObj["input2"] = "0.0";
            cObj["K"] = 1.0;
            ctrlLoopsObj["gains"].push_back(cObj);
        } else if (t == "DERIVATIVE") {
            cObj["output"] = comp.id + ".Out";
            cObj["original_type"] = "DERIVATIVE";
            std::string inSig = getIncomingSignal(comp.id, "In");
            cObj["input"] = inSig;
            cObj["input1"] = inSig;
            cObj["input2"] = "0.0";
            cObj["K"] = 1.0;
            ctrlLoopsObj["gains"].push_back(cObj);
        } else if (t == "TRANSFER_FCN") {
            cObj["output"] = comp.id + ".Out";
            cObj["original_type"] = "TRANSFER_FCN";
            cObj["num"] = comp.parameters.count("num") ? comp.parameters.at("num") : "[1]";
            cObj["den"] = comp.parameters.count("den") ? comp.parameters.at("den") : "[1 1]";
            std::string inSig = getIncomingSignal(comp.id, "In");
            cObj["input"] = inSig;
            cObj["input1"] = inSig;
            cObj["input2"] = "0.0";
            cObj["K"] = 1.0;
            ctrlLoopsObj["gains"].push_back(cObj);
        } else if (t == "STATE_SPACE") {
            cObj["output"] = comp.id + ".Out";
            cObj["original_type"] = "STATE_SPACE";
            cObj["A"] = comp.parameters.count("A") ? comp.parameters.at("A") : "[-1]";
            cObj["B"] = comp.parameters.count("B") ? comp.parameters.at("B") : "[1]";
            cObj["C"] = comp.parameters.count("C") ? comp.parameters.at("C") : "[1]";
            cObj["D"] = comp.parameters.count("D") ? comp.parameters.at("D") : "[0]";
            cObj["x0"] = comp.parameters.count("x0") ? comp.parameters.at("x0") : "0";
            std::string inSig = getIncomingSignal(comp.id, "In");
            cObj["input"] = inSig;
            cObj["input1"] = inSig;
            cObj["input2"] = "0.0";
            cObj["K"] = 1.0;
            ctrlLoopsObj["gains"].push_back(cObj);
        } else if (t == "DELAY" || t == "TRANSPORT_DELAY") {
            cObj["output"] = comp.id + ".Out";
            cObj["original_type"] = comp.rawTypeStr;
            cObj["delay"] = formatJSStyleDouble(parsedParams.count("delay") ? parsedParams["delay"] : 0.1);
            std::string inSig = getIncomingSignal(comp.id, "In");
            cObj["input"] = inSig;
            cObj["input1"] = inSig;
            cObj["input2"] = "0.0";
            cObj["K"] = 1.0;
            ctrlLoopsObj["gains"].push_back(cObj);
        } else if (t == "TURN_ON_DELAY") {
            cObj["output"] = comp.id + ".Out";
            cObj["original_type"] = "TURN_ON_DELAY";
            cObj["delay"] = formatJSStyleDouble(parsedParams.count("delay") ? parsedParams["delay"] : 0.05);
            std::string inSig = getIncomingSignal(comp.id, "In");
            cObj["input"] = inSig;
            cObj["input1"] = inSig;
            cObj["input2"] = "0.0";
            cObj["K"] = 1.0;
            ctrlLoopsObj["gains"].push_back(cObj);
        } else if (t == "MEMORY_BLOCK" || t == "MEMORY") {
            cObj["output"] = comp.id + ".Out";
            cObj["original_type"] = "MEMORY_BLOCK";
            cObj["initial_value"] = formatJSStyleDouble(parsedParams.count("initial_value") ? parsedParams["initial_value"] : 0.0);
            std::string inSig = getIncomingSignal(comp.id, "In");
            cObj["input"] = inSig;
            cObj["input1"] = inSig;
            cObj["input2"] = "0.0";
            cObj["K"] = 1.0;
            ctrlLoopsObj["gains"].push_back(cObj);
        } else if (t == "QUANTIZER") {
            cObj["output"] = comp.id + ".Out";
            cObj["original_type"] = "QUANTIZER";
            cObj["step_size"] = formatJSStyleDouble(parsedParams.count("step_size") ? parsedParams["step_size"] : 0.5);
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
            cObj["min"] = formatJSStyleDouble(parsedParams.count("min") ? parsedParams["min"] : -10.0);
            cObj["max"] = formatJSStyleDouble(parsedParams.count("max") ? parsedParams["max"] : 10.0);
            std::string inSig = getIncomingSignal(comp.id, "In");
            cObj["input"] = inSig;
            cObj["input1"] = inSig;
            cObj["input2"] = "0.0";
            cObj["K"] = 1.0;
            ctrlLoopsObj["gains"].push_back(cObj);
        } else if (t == "DEAD_ZONE") {
            cObj["output"] = comp.id + ".Out";
            cObj["original_type"] = "DEAD_ZONE";
            cObj["start"] = formatJSStyleDouble(parsedParams.count("start") ? parsedParams["start"] : -0.5);
            cObj["end"] = formatJSStyleDouble(parsedParams.count("end") ? parsedParams["end"] : 0.5);
            std::string inSig = getIncomingSignal(comp.id, "In");
            cObj["input"] = inSig;
            cObj["input1"] = inSig;
            cObj["input2"] = "0.0";
            cObj["K"] = 1.0;
            ctrlLoopsObj["gains"].push_back(cObj);
        } else if (t == "RATE_LIMITER") {
            cObj["output"] = comp.id + ".Out";
            cObj["original_type"] = "RATE_LIMITER";
            cObj["up"] = formatJSStyleDouble(parsedParams.count("up") ? parsedParams["up"] : 10.0);
            cObj["down"] = formatJSStyleDouble(parsedParams.count("down") ? parsedParams["down"] : -10.0);
            std::string inSig = getIncomingSignal(comp.id, "In");
            cObj["input"] = inSig;
            cObj["input1"] = inSig;
            cObj["input2"] = "0.0";
            cObj["K"] = 1.0;
            ctrlLoopsObj["gains"].push_back(cObj);
        } else if (t == "RELAY") {
            cObj["output"] = comp.id + ".Out";
            cObj["original_type"] = "RELAY";
            cObj["on_threshold"] = formatJSStyleDouble(parsedParams.count("on_threshold") ? parsedParams["on_threshold"] : 1.0);
            cObj["off_threshold"] = formatJSStyleDouble(parsedParams.count("off_threshold") ? parsedParams["off_threshold"] : -1.0);
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
            cObj["operator"] = comp.parameters.count("operator") ? comp.parameters.at("operator") : "AND";
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
            cObj["edge"] = comp.parameters.count("edge") ? comp.parameters.at("edge") : "rising";
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
            cObj["duration"] = comp.parameters.count("duration") ? comp.parameters.at("duration") : "0.1";
            cObj["edge"] = comp.parameters.count("edge") ? comp.parameters.at("edge") : "rising";
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
            cObj["operator"] = comp.parameters.count("operator") ? comp.parameters.at("operator") : "==";
            cObj["input1"] = getIncomingSignal(comp.id, "In1");
            cObj["input2"] = getIncomingSignal(comp.id, "In2");
            ctrlLoopsObj["gains"].push_back(cObj);
        } else if (t == "COMPARE_TO_CONSTANT" || t == "COMP_CONST") {
            cObj["output"] = comp.id + ".Out";
            cObj["original_type"] = "COMPARE_TO_CONSTANT";
            std::string op = comp.parameters.count("operator") ? comp.parameters.at("operator") : (comp.parameters.count("op") ? comp.parameters.at("op") : "==");
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
        } else if (t == "CLARKE") {
            cObj["output_alpha"] = comp.id + ".Alpha";
            cObj["output_beta"] = comp.id + ".Beta";
            cObj["original_type"] = "CLARKE";
            cObj["input_a"] = getIncomingSignal(comp.id, "A");
            cObj["input_b"] = getIncomingSignal(comp.id, "B");
            cObj["input_c"] = getIncomingSignal(comp.id, "C");
            ctrlLoopsObj["gains"].push_back(cObj);
        } else if (t == "INV_CLARKE") {
            cObj["output_a"] = comp.id + ".A";
            cObj["output_b"] = comp.id + ".B";
            cObj["output_c"] = comp.id + ".C";
            cObj["original_type"] = "INV_CLARKE";
            cObj["input_alpha"] = getIncomingSignal(comp.id, "Alpha");
            cObj["input_beta"] = getIncomingSignal(comp.id, "Beta");
            ctrlLoopsObj["gains"].push_back(cObj);
        } else if (t == "PARK") {
            cObj["output_d"] = comp.id + ".d";
            cObj["output_q"] = comp.id + ".q";
            cObj["original_type"] = "PARK";
            cObj["input_alpha"] = getIncomingSignal(comp.id, "Alpha");
            cObj["input_beta"] = getIncomingSignal(comp.id, "Beta");
            cObj["input_theta"] = getIncomingSignal(comp.id, "Theta");
            ctrlLoopsObj["gains"].push_back(cObj);
        } else if (t == "INV_PARK") {
            cObj["output_alpha"] = comp.id + ".Alpha";
            cObj["output_beta"] = comp.id + ".Beta";
            cObj["original_type"] = "INV_PARK";
            cObj["input_d"] = getIncomingSignal(comp.id, "d");
            cObj["input_q"] = getIncomingSignal(comp.id, "q");
            cObj["input_theta"] = getIncomingSignal(comp.id, "Theta");
            ctrlLoopsObj["gains"].push_back(cObj);
        } else if (t == "PWM_3PH") {
            cObj["output_a"] = comp.id + ".OutA";
            cObj["output_b"] = comp.id + ".OutB";
            cObj["output_c"] = comp.id + ".OutC";
            cObj["original_type"] = "PWM_3PH";
            cObj["frequency"] = formatJSStyleDouble(parsedParams.count("frequency") ? parsedParams["frequency"] : 10000.0);
            cObj["input_a"] = getIncomingSignal(comp.id, "A");
            cObj["input_b"] = getIncomingSignal(comp.id, "B");
            cObj["input_c"] = getIncomingSignal(comp.id, "C");
            ctrlLoopsObj["gains"].push_back(cObj);
        } else if (t == "SVPWM") {
            cObj["output_a"] = comp.id + ".OutA";
            cObj["output_b"] = comp.id + ".OutB";
            cObj["output_c"] = comp.id + ".OutC";
            cObj["original_type"] = "SVPWM";
            cObj["frequency"] = formatJSStyleDouble(parsedParams.count("frequency") ? parsedParams["frequency"] : 10000.0);
            cObj["input_a"] = getIncomingSignal(comp.id, "A");
            cObj["input_b"] = getIncomingSignal(comp.id, "B");
            cObj["input_c"] = getIncomingSignal(comp.id, "C");
            ctrlLoopsObj["gains"].push_back(cObj);
        } else if (t == "PER_AVG") {
            cObj["output"] = comp.id + ".Out";
            cObj["original_type"] = "PER_AVG";
            cObj["period"] = formatJSStyleDouble(parsedParams.count("period") ? parsedParams["period"] : 0.02);
            std::string inSig = getIncomingSignal(comp.id, "In");
            cObj["input"] = inSig; cObj["input1"] = inSig; cObj["input2"] = "0.0"; cObj["K"] = 1.0;
            ctrlLoopsObj["gains"].push_back(cObj);
        } else if (t == "PERIODIC_IMP_AVG") {
            cObj["output"] = comp.id + ".Out";
            cObj["original_type"] = "PERIODIC_IMP_AVG";
            cObj["initial_value"] = formatJSStyleDouble(parsedParams.count("initial_value") ? parsedParams["initial_value"] : 0.0);
            cObj["input"] = getIncomingSignal(comp.id, "In");
            cObj["control_signal"] = getIncomingSignal(comp.id, "Trig");
            ctrlLoopsObj["gains"].push_back(cObj);
        } else if (t == "FOURIER_TRANS") {
            cObj["output_mag"] = comp.id + ".Mag";
            cObj["output_phase"] = comp.id + ".Phase";
            cObj["original_type"] = "FOURIER_TRANS";
            cObj["f"] = formatJSStyleDouble(parsedParams.count("f") ? parsedParams["f"] : 50.0);
            cObj["harmonic"] = parsedParams.count("harmonic") ? (int)parsedParams["harmonic"] : 1;
            cObj["ts"] = comp.parameters.count("ts") ? comp.parameters.at("ts") : "100u";
            cObj["input"] = getIncomingSignal(comp.id, "In");
            ctrlLoopsObj["gains"].push_back(cObj);
        } else if (t == "MOV_AVG") {
            cObj["output"] = comp.id + ".Out";
            cObj["original_type"] = "MOV_AVG";
            cObj["window"] = parsedParams.count("window") ? (int)parsedParams["window"] : 10;
            std::string inSig = getIncomingSignal(comp.id, "In");
            cObj["input"] = inSig; cObj["input1"] = inSig; cObj["input2"] = "0.0"; cObj["K"] = 1.0;
            ctrlLoopsObj["gains"].push_back(cObj);
        } else if (t == "FILTER_1ST") {
            cObj["output"] = comp.id + ".Out";
            cObj["original_type"] = "FILTER_1ST";
            cObj["type"] = comp.parameters.count("type") ? comp.parameters.at("type") : "Lowpass";
            cObj["fc"] = comp.parameters.count("fc") ? comp.parameters.at("fc") : "1k";
            std::string inSig = getIncomingSignal(comp.id, "In");
            cObj["input"] = inSig; cObj["input1"] = inSig; cObj["input2"] = "0.0"; cObj["K"] = 1.0;
            ctrlLoopsObj["gains"].push_back(cObj);
        } else if (t == "FILTER_2ND") {
            cObj["output"] = comp.id + ".Out";
            cObj["original_type"] = "FILTER_2ND";
            cObj["type"] = comp.parameters.count("type") ? comp.parameters.at("type") : "Lowpass";
            cObj["fc"] = comp.parameters.count("fc") ? comp.parameters.at("fc") : "1k";
            cObj["Q"] = formatJSStyleDouble(parsedParams.count("Q") ? parsedParams["Q"] : 0.707);
            std::string inSig = getIncomingSignal(comp.id, "In");
            cObj["input"] = inSig; cObj["input1"] = inSig; cObj["input2"] = "0.0"; cObj["K"] = 1.0;
            ctrlLoopsObj["gains"].push_back(cObj);
        } else if (t == "FOURIER_ANALYSIS") {
            cObj["output_mag"] = comp.id + ".Mag";
            cObj["output_phase"] = comp.id + ".Phase";
            cObj["original_type"] = "FOURIER_ANALYSIS";
            cObj["f"] = formatJSStyleDouble(parsedParams.count("f") ? parsedParams["f"] : 50.0);
            cObj["input"] = getIncomingSignal(comp.id, "In");
            ctrlLoopsObj["gains"].push_back(cObj);
        } else if (t == "RMS_VAL") {
            cObj["output"] = comp.id + ".Out";
            cObj["original_type"] = "RMS_VAL";
            cObj["frequency"] = formatJSStyleDouble(parsedParams.count("frequency") ? parsedParams["frequency"] : 50.0);
            std::string inSig = getIncomingSignal(comp.id, "In");
            cObj["input"] = inSig; cObj["input1"] = inSig; cObj["input2"] = "0.0"; cObj["K"] = 1.0;
            ctrlLoopsObj["gains"].push_back(cObj);
        } else if (t == "THD_VAL") {
            cObj["output"] = comp.id + ".Out";
            cObj["original_type"] = "THD_VAL";
            cObj["frequency"] = formatJSStyleDouble(parsedParams.count("frequency") ? parsedParams["frequency"] : 50.0);
            std::string inSig = getIncomingSignal(comp.id, "In");
            cObj["input"] = inSig; cObj["input1"] = inSig; cObj["input2"] = "0.0"; cObj["K"] = 1.0;
            ctrlLoopsObj["gains"].push_back(cObj);
        } else if (t == "PLL_LOOP") {
            cObj["output_theta"] = comp.id + ".Theta";
            cObj["output_freq"] = comp.id + ".Freq";
            cObj["output_cos"] = comp.id + ".Cos";
            cObj["output_sin"] = comp.id + ".Sin";
            cObj["original_type"] = "PLL_LOOP";
            cObj["fn"] = formatJSStyleDouble(parsedParams.count("fn") ? parsedParams["fn"] : 50.0);
            cObj["Kp"] = formatJSStyleDouble(parsedParams.count("Kp") ? parsedParams["Kp"] : 20.0);
            cObj["Ki"] = formatJSStyleDouble(parsedParams.count("Ki") ? parsedParams["Ki"] : 1000.0);
            cObj["input"] = getIncomingSignal(comp.id, "In");
            ctrlLoopsObj["gains"].push_back(cObj);
        } else if (t == "OFFSET") {
            cObj["output"] = comp.id + ".Out";
            cObj["original_type"] = "OFFSET";
            cObj["offset"] = formatJSStyleDouble(parsedParams.count("offset") ? parsedParams["offset"] : 0.0);
            std::string inSig = getIncomingSignal(comp.id, "In");
            cObj["input"] = inSig; cObj["input1"] = inSig; cObj["input2"] = "0.0"; cObj["K"] = 1.0;
            ctrlLoopsObj["gains"].push_back(cObj);
        } else if (t == "SIGNUM" || t == "SIGN") {
            cObj["output"] = comp.id + ".Out";
            cObj["original_type"] = "SIGNUM";
            std::string inSig = getIncomingSignal(comp.id, "In");
            cObj["input"] = inSig; cObj["input1"] = inSig; cObj["input2"] = "0.0"; cObj["K"] = 1.0;
            ctrlLoopsObj["gains"].push_back(cObj);
        } else if (t == "DATATYPE_CONV") {
            cObj["output"] = comp.id + ".Out";
            cObj["original_type"] = "DATATYPE_CONV";
            cObj["datatype"] = comp.parameters.count("datatype") ? comp.parameters.at("datatype") : "boolean";
            std::string inSig = getIncomingSignal(comp.id, "In");
            cObj["input"] = inSig; cObj["input1"] = inSig; cObj["input2"] = "0.0"; cObj["K"] = 1.0;
            ctrlLoopsObj["gains"].push_back(cObj);
        } else if (t == "DIVIDE") {
            cObj["output"] = comp.id + ".Out";
            cObj["original_type"] = "DIVIDE";
            cObj["inputs"] = nlohmann::json::array({
                getIncomingSignal(comp.id, "Num"),
                getIncomingSignal(comp.id, "Den")
            });
            ctrlLoopsObj["product_blocks"].push_back(cObj);
        } else if (t == "STATE_MACHINE") {
            cObj["output"] = comp.id + ".Out";
            cObj["original_type"] = "STATE_MACHINE";
            std::string inSig = getIncomingSignal(comp.id, "In");
            cObj["input"] = inSig; cObj["input1"] = inSig; cObj["input2"] = "0.0"; cObj["K"] = 1.0;
            ctrlLoopsObj["gains"].push_back(cObj);
        } else if (t == "DLL" || t == "FMU" || t == "FOURIER_SERIES") {
            cObj["output"] = comp.id + ".Out";
            cObj["original_type"] = comp.rawTypeStr;
            std::string inSig = getIncomingSignal(comp.id, "In");
            cObj["input"] = inSig;
            cObj["input1"] = inSig;
            cObj["input2"] = "0.0";
            cObj["K"] = 1.0;
            ctrlLoopsObj["gains"].push_back(cObj);
        } else if (t == "CONT_PID" || t == "DISCRETE_PID") {
            cObj["output"] = comp.id + ".Out";
            cObj["original_type"] = comp.rawTypeStr;
            cObj["Kp"] = formatJSStyleDouble(parsedParams.count("Kp") ? parsedParams["Kp"] : 1.0);
            cObj["Ki"] = formatJSStyleDouble(parsedParams.count("Ki") ? parsedParams["Ki"] : 0.0);
            cObj["Kd"] = formatJSStyleDouble(parsedParams.count("Kd") ? parsedParams["Kd"] : 0.0);
            cObj["Tf"] = formatJSStyleDouble(parsedParams.count("Tf") ? parsedParams["Tf"] : 0.01);
            if (comp.parameters.count("ts")) cObj["ts"] = comp.parameters.at("ts");
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

            double minV = 0.0;
            if (comp.parameters.count("min")) minV = CircuitSimEngine::ExpressionEvaluator::parseScientific(comp.parameters.at("min"));

            double maxV = 1.0;
            if (comp.parameters.count("max")) maxV = CircuitSimEngine::ExpressionEvaluator::parseScientific(comp.parameters.at("max"));

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
            cObj["frequency"] = formatJSStyleDouble(parsedParams.count("frequency") ? parsedParams["frequency"] : 20000.0);
            cObj["input"] = getIncomingSignal(comp.id, "In");
            ctrlLoopsObj["pwm_generators"].push_back(cObj);
        } else if (t == "PWM_MASTER" || t == "MASTER_PWM") {
            cObj["original_type"] = "PWM_MASTER";
            cObj["fc"] = comp.parameters.count("fc") ? comp.parameters.at("fc") : "10k";
            cObj["dead_time"] = comp.parameters.count("dead_time") ? comp.parameters.at("dead_time") : "1u";
            cObj["num_carriers"] = comp.parameters.count("num_carriers") ? comp.parameters.at("num_carriers") : "3";
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

            json inputsArr = json::array();
            for (size_t i = 0; i < discIn.size(); ++i) {
                std::string inSig = getIncomingSignal(comp.id, discIn[i].name);
                if (inSig == "0.0") inSig = getIncomingSignal(comp.id, "In" + std::to_string(i + 1));
                if (!inSig.empty() && inSig != "0.0") {
                    inputsArr.push_back(inSig);
                }
            }
            cObj["inputs"] = inputsArr;

            json outputsArr = json::array();
            if (!discOut.empty()) {
                for (size_t j = 0; j < discOut.size(); ++j) {
                    outputsArr.push_back(comp.id + "." + discOut[j].name);
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
        } else if (t == "TERMINATOR") {
            cObj["original_type"] = "TERMINATOR";
            cObj["input"] = getIncomingSignal(comp.id, "In");
            ctrlLoopsObj["terminators"].push_back(cObj);
        } else if (t == "POLYNOMIAL") {
            cObj["original_type"] = "POLYNOMIAL";
            cObj["output"] = comp.id + ".Out";
            cObj["input"] = getIncomingSignal(comp.id, "In");
            cObj["coefficients"] = comp.parameters.count("coefficients") ? comp.parameters.at("coefficients") : "[1, 0]";
            ctrlLoopsObj["functions"].push_back(cObj);
        } else if (t == "ALGEBRAIC_CONSTRAINT") {
            cObj["original_type"] = "ALGEBRAIC_CONSTRAINT";
            cObj["output"] = comp.id + ".Out";
            cObj["input"] = getIncomingSignal(comp.id, "In");
            cObj["initial_guess"] = comp.parameters.count("initial_guess") ? comp.parameters.at("initial_guess") : "0.0";
            ctrlLoopsObj["functions"].push_back(cObj);
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
    simParamsObj["step_type"] = tempDesign.settings.stepType.empty() ? "fixed" : tempDesign.settings.stepType;
    simParamsObj["solverMethod"] = "non-ideal";
    simParamsObj["engine"] = "auto";
    simParamsObj["enable_lu_cache"] = true;

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
                std::string varName = comp.id + ".In" + std::to_string(ch);
                if (!addedVars.count(varName)) { wantedVars.push_back(varName); addedVars.insert(varName); }
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

void NetlistSourceView::updateFromCircuit(const CircuitDesign& design) {
    lastGeneratedJson = generateNetlistJson(design);
    strncpy(jsonBuffer, lastGeneratedJson.c_str(), sizeof(jsonBuffer) - 1);
    isNetlistValid = true;
    netlistStatusMsg = "Valid Netlist";
}

void NetlistSourceView::render(const char* title, CircuitDesign& design, CircuitSimEngine::CircuitSimulator& simulator) {
    if (jsonBuffer[0] == '\0') {
        updateFromCircuit(design);
    }

    ImGuiViewport* viewport = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(ImVec2(viewport->WorkPos.x, viewport->WorkPos.y + 26.0f), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(viewport->WorkSize.x, viewport->WorkSize.y - 26.0f), ImGuiCond_Always);
    ImGui::SetNextWindowViewport(viewport->ID);

    ImGui::Begin(title, nullptr, ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoDocking);

    float availWidth = ImGui::GetContentRegionAvail().x;
    float splitterWidth = 8.0f;
    float leftWidth = availWidth * splitRatio;
    if (leftWidth < 120.0f) leftWidth = 120.0f;
    if (availWidth - leftWidth - splitterWidth < 200.0f) {
        leftWidth = availWidth - splitterWidth - 200.0f;
    }

    // ─── LEFT PANEL: RAW NETLIST SOURCE CODE EDITOR ───
    ImGui::BeginChild("NetlistLeftPanel", ImVec2(leftWidth, 0), true);
    
    ImGui::TextColored(ImVec4(0.2f, 0.8f, 1.0f, 1.0f), "Raw Netlist Source Code (JSON Schema)");
    ImGui::SameLine();
    if (isNetlistValid) {
        ImGui::TextColored(ImVec4(0.2f, 0.9f, 0.4f, 1.0f), "[Valid Netlist]");
    } else {
        ImGui::TextColored(ImVec4(1.0f, 0.3f, 0.3f, 1.0f), "[JSON Error]");
    }

    ImGui::Spacing();

    if (ImGui::Button("Copy JSON")) {
        ImGui::SetClipboardText(jsonBuffer);
    }
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("Copy complete schematic design JSON to clipboard");
    }
    ImGui::SameLine();
    if (ImGui::Button("Download Netlist")) {
        OPENFILENAMEA ofn;
        char szFile[260] = "circuit_netlist.json";
        ZeroMemory(&ofn, sizeof(ofn));
        ofn.lStructSize = sizeof(ofn);
        ofn.hwndOwner = NULL;
        ofn.lpstrFile = szFile;
        ofn.nMaxFile = sizeof(szFile);
        ofn.lpstrFilter = "JSON Files (*.json)\0*.json\0All Files (*.*)\0*.*\0";
        ofn.nFilterIndex = 1;
        ofn.lpstrFileTitle = NULL;
        ofn.nMaxFileTitle = 0;
        ofn.lpstrInitialDir = NULL;
        ofn.Flags = OFN_PATHMUSTEXIST | OFN_OVERWRITEPROMPT;

        if (GetSaveFileNameA(&ofn) == TRUE) {
            std::ofstream out(ofn.lpstrFile);
            if (out.is_open()) {
                out << jsonBuffer;
            }
        }
    }

    if (!isNetlistValid) {
        ImGui::TextColored(ImVec4(1.0f, 0.3f, 0.3f, 1.0f), "%s", netlistStatusMsg.c_str());
    }

    ImGui::Spacing();
    ImGui::InputTextMultiline("##NetlistJsonEditor", jsonBuffer, sizeof(jsonBuffer), ImVec2(-1, -1), ImGuiInputTextFlags_AllowTabInput);

    ImGui::EndChild();

    ImGui::SameLine(0, 0);

    // ─── INTERACTIVE DRAGGABLE SPLITTER BAR ───
    ImGui::PushStyleColor(ImGuiCol_Button, isDarkMode ? ImVec4(0.20f, 0.27f, 0.38f, 0.50f) : ImVec4(0.75f, 0.80f, 0.88f, 0.80f));
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.05f, 0.65f, 0.91f, 0.85f));
    ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(0.02f, 0.75f, 1.00f, 1.00f));

    ImGui::Button("##SplitterBar", ImVec2(splitterWidth, -1));
    if (ImGui::IsItemActive()) {
        float mouseDeltaX = ImGui::GetIO().MouseDelta.x;
        splitRatio += mouseDeltaX / availWidth;
        if (splitRatio < 0.05f) splitRatio = 0.05f;
        if (splitRatio > 0.85f) splitRatio = 0.85f;
    }
    if (ImGui::IsItemHovered() || ImGui::IsItemActive()) {
        ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeEW);
    }

    ImGui::PopStyleColor(3);

    ImGui::SameLine(0, 0);

    // ─── RIGHT PANEL: WAVEFORM SOLVER PLOTTER ───
    ImGui::BeginChild("WaveformRightPanel", ImVec2(0, 0), true);

    ImGui::TextColored(ImVec4(0.2f, 0.8f, 1.0f, 1.0f), "Interactive Waveform Solver Plotter");
    double pct = simulator.getProgressPercent();
    double cpuSec = simulator.getComputeTimeSeconds();
    ImGui::TextColored(ImVec4(0.7f, 0.7f, 0.7f, 1.0f), "| Progress: %.1f%% | Compute Time: %.3f s", pct, cpuSec);

    ImGui::Spacing();

    CircuitSimEngine::TelemetryData data = simulator.getTelemetryCopy();
    if (data.timeHistory.empty()) {
        ImGui::TextColored(ImVec4(0.7f, 0.7f, 0.7f, 1.0f), "No simulation telemetry data. Click PLAY or Start Simulation (Ctrl+T) to plot waveforms.");
    } else {
        struct SignalCat {
            std::string title;
            std::string yLabel;
            std::vector<std::pair<std::string, std::vector<double>>> variables;
        };

        SignalCat voltageCat{"Voltage Waveforms (V)", "Voltage (V)", {}};
        SignalCat currentCat{"Current Waveforms (I)", "Current (A)", {}};
        SignalCat controlCat{"Control & Pulse Signals", "Signal (V / State)", {}};
        SignalCat otherCat{"Other Signals", "Magnitude", {}};

        std::set<std::string> probedSet;

        // 1. Add all signals explicitly checked in Property Inspector
        for (const auto& plot : design.plotConfig.plots) {
            for (const auto& var : plot.variables) {
                if (!var.empty()) {
                    probedSet.insert(var);
                    if (var.rfind("V_", 0) == 0 || var.rfind("I_", 0) == 0) {
                        probedSet.insert(var.substr(2));
                    }
                }
            }
        }

        // 2. Add signals from components with explicit probe parameters
        for (const auto& comp : design.components) {
            if (comp.parameters.count("plotV") && comp.parameters.at("plotV") == "1") {
                probedSet.insert("V_" + comp.id);
            }
            if (comp.parameters.count("plotI") && comp.parameters.at("plotI") == "1") {
                probedSet.insert("I_" + comp.id);
            }
            if (comp.parameters.count("probe_signal") && comp.parameters.at("probe_signal") == "1") {
                probedSet.insert("V_" + comp.id);
                probedSet.insert("I_" + comp.id);
                probedSet.insert(comp.id);
            }
            if (comp.parameters.count("selected_signals") && !comp.parameters.at("selected_signals").empty()) {
                std::stringstream ss(comp.parameters.at("selected_signals"));
                std::string item;
                while (std::getline(ss, item, ',')) {
                    if (!item.empty()) {
                        probedSet.insert(item);
                        if (item.rfind("V_", 0) != 0 && item.rfind("I_", 0) != 0) {
                            probedSet.insert("V_" + item);
                            probedSet.insert("I_" + item);
                        }
                    }
                }
            }
        }

        // 3. Add signals connected specifically to SCOPE or PROBE block terminals
        std::set<std::string> scopeProbeCompIds;
        for (const auto& comp : design.components) {
            std::string t = comp.rawTypeStr;
            std::transform(t.begin(), t.end(), t.begin(), ::toupper);
            if (t == "SCOPE" || t == "PROBE" || t == "UNIFIEDPROBE") {
                scopeProbeCompIds.insert(comp.id);
            }
        }

        for (const auto& wire : design.wires) {
            if (!wire.from.isWireJunction && scopeProbeCompIds.count(wire.from.compId)) {
                if (!wire.to.isWireJunction && !wire.to.compId.empty()) {
                    probedSet.insert(wire.to.compId);
                    probedSet.insert("V_" + wire.to.compId);
                    probedSet.insert("I_" + wire.to.compId);
                }
            }
            if (!wire.to.isWireJunction && scopeProbeCompIds.count(wire.to.compId)) {
                if (!wire.from.isWireJunction && !wire.from.compId.empty()) {
                    probedSet.insert(wire.from.compId);
                    probedSet.insert("V_" + wire.from.compId);
                    probedSet.insert("I_" + wire.from.compId);
                }
            }
        }

        auto isSigProbed = [&](const std::string& name) -> bool {
            if (probedSet.empty()) return false;
            if (probedSet.count(name) > 0) return true;
            std::string base = name;
            if (base.rfind("V_", 0) == 0 || base.rfind("I_", 0) == 0) {
                base = base.substr(2);
                if (probedSet.count(base) > 0) return true;
            }
            for (const auto& p : probedSet) {
                if (p.empty()) continue;
                if (name == p || name == ("V_" + p) || name == ("I_" + p)) return true;
            }
            return false;
        };

        // Synchronize enabledSignals map
        for (const auto& pair : data.voltages) {
            const std::string& name = pair.first;
            if (name.rfind("node_", 0) == 0 || name == "0" || name == "node_0") continue;
            enabledSignals[name] = isSigProbed(name);
        }

        for (const auto& pair : data.voltages) {
            const std::string& name = pair.first;
            const std::vector<double>& vals = pair.second;
            if (vals.empty()) continue;

            // Skip internal raw MNA matrix node voltages (node_1, node_2, 0, etc.)
            if (name.rfind("node_", 0) == 0 || name == "0" || name == "node_0") continue;

            // Filter: only plot signals selected by user / probed
            if (!enabledSignals[name]) continue;

            if (name.rfind("I_", 0) == 0) {
                currentCat.variables.push_back({name, vals});
            } else if (name.rfind("V_", 0) == 0) {
                voltageCat.variables.push_back({name, vals});
            } else if (name.find(".Out") != std::string::npos || name.rfind("Ctrl_", 0) == 0 || name.rfind("PULSE", 0) != std::string::npos || name.rfind("PWM", 0) != std::string::npos) {
                controlCat.variables.push_back({name, vals});
            } else {
                otherCat.variables.push_back({name, vals});
            }
        }

        std::vector<SignalCat> categories;
        if (!voltageCat.variables.empty()) categories.push_back(voltageCat);
        if (!currentCat.variables.empty()) categories.push_back(currentCat);
        if (!controlCat.variables.empty()) categories.push_back(controlCat);
        if (!otherCat.variables.empty()) categories.push_back(otherCat);

        if (!categories.empty()) {
            bool doFitThisFrame = autoFitNext;
            if (autoFitNext) autoFitNext = false;
            if (ImGui::Button("Fit Waveforms / Reset Zoom")) doFitThisFrame = true;
            ImGui::SameLine();

            ImGui::TextDisabled("|");
            ImGui::SameLine();

            // Interactive Signal Selection Dropdown
            int selCount = 0;
            int totCount = 0;
            for (const auto& pair : data.voltages) {
                const std::string& name = pair.first;
                if (name.rfind("node_", 0) == 0 || name == "0" || name == "node_0") continue;
                totCount++;
                if (enabledSignals[name]) selCount++;
            }

            char filterBuf[64];
            snprintf(filterBuf, sizeof(filterBuf), "📊 Signals (%d/%d)##sigFilterNet", selCount, totCount);
            ImGui::SetNextItemWidth(140.0f);
            if (ImGui::BeginCombo("##SignalComboNet", filterBuf, ImGuiComboFlags_HeightLarge)) {
                if (ImGui::SmallButton("Select All")) {
                    for (const auto& pair : data.voltages) enabledSignals[pair.first] = true;
                }
                ImGui::SameLine();
                if (ImGui::SmallButton("Deselect All")) {
                    for (const auto& pair : data.voltages) enabledSignals[pair.first] = false;
                }
                ImGui::Separator();

                for (const auto& pair : data.voltages) {
                    const std::string& name = pair.first;
                    if (name.rfind("node_", 0) == 0 || name == "0" || name == "node_0") continue;
                    bool enabled = enabledSignals[name];
                    if (ImGui::Checkbox(name.c_str(), &enabled)) {
                        enabledSignals[name] = enabled;
                    }
                }
                ImGui::EndCombo();
            }
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip("Filter which variables are plotted on screen.\nCheck or uncheck variables to customize waveforms.");
            }
            ImGui::SameLine();
            ImGui::TextDisabled("|");
            ImGui::SameLine();

            // Dedicated Zoom Mode Buttons
            if (activeZoomMode == ActiveZoomMode::Adaptive) {
                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.10f, 0.45f, 0.85f, 1.0f));
                if (ImGui::Button("🔍 Adaptive")) activeZoomMode = ActiveZoomMode::Disabled;
                ImGui::PopStyleColor();
            } else {
                if (ImGui::Button("🔍 Adaptive")) activeZoomMode = ActiveZoomMode::Adaptive;
            }
            ImGui::SameLine();

            if (activeZoomMode == ActiveZoomMode::X_Only) {
                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.00f, 0.65f, 0.85f, 1.0f));
                if (ImGui::Button("↔ X-Axis Zoom")) activeZoomMode = ActiveZoomMode::Disabled;
                ImGui::PopStyleColor();
            } else {
                if (ImGui::Button("↔ X-Axis Zoom")) activeZoomMode = ActiveZoomMode::X_Only;
            }
            ImGui::SameLine();

            if (activeZoomMode == ActiveZoomMode::Y_Only) {
                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.70f, 0.20f, 0.80f, 1.0f));
                if (ImGui::Button("↕ Y-Axis Zoom")) activeZoomMode = ActiveZoomMode::Disabled;
                ImGui::PopStyleColor();
            } else {
                if (ImGui::Button("↕ Y-Axis Zoom")) activeZoomMode = ActiveZoomMode::Y_Only;
            }
            ImGui::SameLine();

            if (activeZoomMode == ActiveZoomMode::Box_2D) {
                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.15f, 0.65f, 0.35f, 1.0f));
                if (ImGui::Button("⤢ Box Zoom")) activeZoomMode = ActiveZoomMode::Disabled;
                ImGui::PopStyleColor();
            } else {
                if (ImGui::Button("⤢ Box Zoom")) activeZoomMode = ActiveZoomMode::Box_2D;
            }
            ImGui::SameLine();

            if (ImGui::Button("➕ Add Plot Pane")) {
                numPanes = std::min(numPanes + 1, 4);
            }
            ImGui::SameLine();
            if (numPanes > 1) {
                if (ImGui::Button("➖ Remove Plot Pane")) {
                    numPanes = std::max(numPanes - 1, 1);
                }
                ImGui::SameLine();
            }

            ImGui::TextDisabled("|");
            ImGui::SameLine();

            // Plot Interpolation Mode selector
            const char* modeNamesNet[] = { "Hybrid (Auto)", "Linear", "Stairs" };
            int currentModeIdxNet = (int)globalPlotMode;
            ImGui::SetNextItemWidth(110.0f);
            if (ImGui::Combo("##PlotModeComboNet", &currentModeIdxNet, modeNamesNet, 3)) {
                globalPlotMode = (InterpolationMode)currentModeIdxNet;
            }
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip("Interpolation Mode:\n- Hybrid: Step at switching events (e.g. V_ds, V_L), Linear elsewhere\n- Linear: Continuous linear interpolation\n- Stairs: Step plot (e.g. Gate Pulses)");
            }

            ImGui::SameLine();

            ImGui::SetNextItemWidth(100.0f);
            ImGui::SliderFloat("Line Width", &traceLineWidth, 1.0f, 6.0f, "%.1f px");

            int renderPanes = std::min(numPanes, (int)categories.size());
            if (renderPanes < 1) renderPanes = 1;

            bool isZoomActive = (activeZoomMode != ActiveZoomMode::Disabled);

            static const ImVec4 DARK_MODE_COLORS[] = {
                ImVec4(0.00f, 0.95f, 1.00f, 1.00f), // Neon Cyan
                ImVec4(0.10f, 1.00f, 0.45f, 1.00f), // Bright Emerald Green
                ImVec4(1.00f, 0.88f, 0.00f, 1.00f), // Vivid Gold Yellow
                ImVec4(1.00f, 0.25f, 0.60f, 1.00f), // Bright Neon Pink
                ImVec4(1.00f, 0.50f, 0.10f, 1.00f), // Bright Coral Orange
                ImVec4(0.70f, 0.40f, 1.00f, 1.00f), // Bright Electric Violet
                ImVec4(0.40f, 0.90f, 1.00f, 1.00f), // Bright Sky Blue
                ImVec4(0.75f, 1.00f, 0.20f, 1.00f)  // Electric Lime
            };

            static const ImVec4 LIGHT_MODE_COLORS[] = {
                ImVec4(0.05f, 0.35f, 0.75f, 1.00f), // Deep Royal Navy
                ImVec4(0.02f, 0.50f, 0.25f, 1.00f), // Dark Forest Green
                ImVec4(0.80f, 0.12f, 0.12f, 1.00f), // Deep Crimson Red
                ImVec4(0.50f, 0.15f, 0.75f, 1.00f), // Deep Dark Violet
                ImVec4(0.85f, 0.30f, 0.05f, 1.00f), // Rich Dark Orange
                ImVec4(0.05f, 0.50f, 0.55f, 1.00f), // Dark Teal
                ImVec4(0.45f, 0.25f, 0.08f, 1.00f), // Dark Warm Brown
                ImVec4(0.12f, 0.18f, 0.28f, 1.00f)  // Deep Charcoal Slate
            };

            const auto& palette = isDarkMode ? DARK_MODE_COLORS : LIGHT_MODE_COLORS;
            size_t numColors = sizeof(DARK_MODE_COLORS) / sizeof(DARK_MODE_COLORS[0]);

            if (ImPlot::BeginSubplots("Waveform Subplots", renderPanes, 1, ImVec2(-1, -1), ImPlotSubplotFlags_LinkCols)) {
                for (int i = 0; i < renderPanes; ++i) {
                    const auto& cat = categories[i % categories.size()];

                    // Pending zoom from previous frame (before BeginPlot)
                    if (pendingZoom[i].hasPending) {
                        if (pendingZoom[i].type == ZOOM_X_ONLY) {
                            ImPlot::SetNextAxisLimits(ImAxis_X1, pendingZoom[i].xMin, pendingZoom[i].xMax, ImGuiCond_Always);
                        } else if (pendingZoom[i].type == ZOOM_Y_ONLY) {
                            ImPlot::SetNextAxisLimits(ImAxis_Y1, pendingZoom[i].yMin, pendingZoom[i].yMax, ImGuiCond_Always);
                        } else if (pendingZoom[i].type == ZOOM_BOX_2D) {
                            ImPlot::SetNextAxisLimits(ImAxis_X1, pendingZoom[i].xMin, pendingZoom[i].xMax, ImGuiCond_Always);
                            ImPlot::SetNextAxisLimits(ImAxis_Y1, pendingZoom[i].yMin, pendingZoom[i].yMax, ImGuiCond_Always);
                        }
                        pendingZoom[i].hasPending = false;
                    } else if (doFitThisFrame) {
                        // Manual fit with 8% Y padding for breathing room
                        double xMin = data.timeHistory.empty() ? 0.0 : data.timeHistory.front();
                        double xMax = data.timeHistory.empty() ? 1.0 : data.timeHistory.back();
                        double yMin =  1e30, yMax = -1e30;
                        for (const auto& vp : cat.variables) {
                            for (double v : vp.second) {
                                if (v < yMin) yMin = v;
                                if (v > yMax) yMax = v;
                            }
                        }
                        if (yMin > yMax) { yMin = -1.0; yMax = 1.0; }
                        double yRange = yMax - yMin;
                        double yPad = (yRange > 1e-9) ? yRange * 0.08 : 0.5;
                        ImPlot::SetNextAxesLimits(xMin, xMax, yMin - yPad, yMax + yPad, ImGuiCond_Always);
                    }

                    if (isZoomActive) {
                        ImPlot::PushStyleColor(ImPlotCol_Selection, ImVec4(0.0f, 0.0f, 0.0f, 0.0f));
                    }

                    if (ImPlot::BeginPlot(cat.title.c_str(), ImVec2(-1, -1),
                                           isZoomActive ? ImPlotFlags_NoMenus : ImPlotFlags_None)) {

                        // Assign Select to Middle Mouse Button (valid index 2) to avoid ImGui IM_ASSERT(button >= 0 && button < 5) crash
                        if (isZoomActive) {
                            ImPlot::GetInputMap().Select       = ImGuiMouseButton_Middle;
                            ImPlot::GetInputMap().SelectCancel = ImGuiMouseButton_Right;
                            ImPlot::GetInputMap().Pan          = ImGuiMouseButton_Right; // RMB = Pan
                        } else {
                            ImPlot::GetInputMap().Select       = ImGuiMouseButton_Right;
                            ImPlot::GetInputMap().SelectCancel = ImGuiMouseButton_Left;
                            ImPlot::GetInputMap().Pan          = ImGuiMouseButton_Left;
                        }

                        ImPlot::SetupAxes("Time (s)", cat.yLabel.c_str());

                        // --- DEDICATED SEPARATE ZOOM MODULE (Bypasses ImPlot 2D Box engine) ---
                        if (isZoomActive) {
                            bool isHovered = ImPlot::IsPlotHovered();
                            bool isMouseDown = ImGui::IsMouseDown(ImGuiMouseButton_Left);
                            bool isMouseReleased = ImGui::IsMouseReleased(ImGuiMouseButton_Left);
                            ImVec2 mousePx = ImGui::GetMousePos();

                            // 1. Start custom drag gesture
                            if (isHovered && ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
                                customDragState[i].isDragging = true;
                                customDragState[i].startPt = ImPlot::GetPlotMousePos();
                                customDragState[i].startPx = mousePx;
                            }

                            // 2. Active custom drag gesture: render visual selection area
                            if (customDragState[i].isDragging && isMouseDown) {
                                customDragState[i].currentPt = ImPlot::GetPlotMousePos();
                                customDragState[i].currentPx = mousePx;

                                ImVec2 pStart = customDragState[i].startPx;
                                ImVec2 pCurr = customDragState[i].currentPx;

                                ImVec2 plotPos = ImPlot::GetPlotPos();
                                ImVec2 plotSize = ImPlot::GetPlotSize();
                                float pLeft = plotPos.x;
                                float pRight = plotPos.x + plotSize.x;
                                float pTop = plotPos.y;
                                float pBottom = plotPos.y + plotSize.y;

                                float x1 = std::min(pStart.x, pCurr.x);
                                float x2 = std::max(pStart.x, pCurr.x);
                                float y1 = std::min(pStart.y, pCurr.y);
                                float y2 = std::max(pStart.y, pCurr.y);

                                float dxPx = x2 - x1;
                                float dyPx = y2 - y1;

                                WaveformZoomType currentDragType = ZOOM_BOX_2D;
                                if (activeZoomMode == ActiveZoomMode::X_Only) {
                                    currentDragType = ZOOM_X_ONLY;
                                } else if (activeZoomMode == ActiveZoomMode::Y_Only) {
                                    currentDragType = ZOOM_Y_ONLY;
                                } else if (activeZoomMode == ActiveZoomMode::Box_2D) {
                                    currentDragType = ZOOM_BOX_2D;
                                } else if (activeZoomMode == ActiveZoomMode::Adaptive) {
                                    if (dyPx <= 0.10f * dxPx || dyPx <= 12.0f) {
                                        currentDragType = ZOOM_X_ONLY;
                                    } else if (dxPx <= 0.10f * dyPx || dxPx <= 12.0f) {
                                        currentDragType = ZOOM_Y_ONLY;
                                    } else {
                                        currentDragType = ZOOM_BOX_2D;
                                    }
                                }

                                if (dxPx > 3.0f || dyPx > 3.0f) {
                                    ImDrawList* drawList = ImPlot::GetPlotDrawList();

                                    if (currentDragType == ZOOM_X_ONLY) {
                                        // --- PURE X-AXIS SELECTION AREA (FULL HEIGHT CYAN BAND) ---
                                        drawList->AddRectFilled(ImVec2(x1, pTop), ImVec2(x2, pBottom), IM_COL32(0, 220, 255, 40));
                                        drawList->AddLine(ImVec2(x1, pTop), ImVec2(x1, pBottom), IM_COL32(0, 220, 255, 255), 2.0f);
                                        drawList->AddLine(ImVec2(x2, pTop), ImVec2(x2, pBottom), IM_COL32(0, 220, 255, 255), 2.0f);

                                        const char* tag = " [ ↔ X-Axis Zoom (Time Only) ] ";
                                        ImVec2 txtSz = ImGui::CalcTextSize(tag);
                                        float midX = (x1 + x2) * 0.5f;
                                        drawList->AddRectFilled(ImVec2(midX - txtSz.x * 0.5f - 4, pTop + 6), ImVec2(midX + txtSz.x * 0.5f + 4, pTop + 6 + txtSz.y + 2), IM_COL32(0, 150, 200, 230), 4.0f);
                                        drawList->AddText(ImVec2(midX - txtSz.x * 0.5f, pTop + 7), IM_COL32(255, 255, 255, 255), tag);
                                    } else if (currentDragType == ZOOM_Y_ONLY) {
                                        // --- PURE Y-AXIS SELECTION AREA (FULL WIDTH MAGENTA BAND) ---
                                        drawList->AddRectFilled(ImVec2(pLeft, y1), ImVec2(pRight, y2), IM_COL32(220, 0, 255, 40));
                                        drawList->AddLine(ImVec2(pLeft, y1), ImVec2(pRight, y1), IM_COL32(220, 0, 255, 255), 2.0f);
                                        drawList->AddLine(ImVec2(pLeft, y2), ImVec2(pRight, y2), IM_COL32(220, 0, 255, 255), 2.0f);

                                        const char* tag = " [ ↕ Y-Axis Zoom (Amp Only) ] ";
                                        ImVec2 txtSz = ImGui::CalcTextSize(tag);
                                        float midY = (y1 + y2) * 0.5f;
                                        drawList->AddRectFilled(ImVec2(pLeft + 6, midY - txtSz.y * 0.5f - 2), ImVec2(pLeft + 6 + txtSz.x + 8, midY + txtSz.y * 0.5f + 2), IM_COL32(160, 0, 180, 230), 4.0f);
                                        drawList->AddText(ImVec2(pLeft + 10, midY - txtSz.y * 0.5f), IM_COL32(255, 255, 255, 255), tag);
                                    } else {
                                        // --- 2D BOX SELECTION AREA ---
                                        drawList->AddRectFilled(ImVec2(x1, y1), ImVec2(x2, y2), IM_COL32(60, 255, 120, 40));
                                        drawList->AddRect(ImVec2(x1, y1), ImVec2(x2, y2), IM_COL32(60, 255, 120, 255), 0, 0, 2.0f);

                                        const char* tag = " [ ⤢ 2D Box Zoom ] ";
                                        ImVec2 txtSz = ImGui::CalcTextSize(tag);
                                        drawList->AddRectFilled(ImVec2(x1 + 4, y1 + 4), ImVec2(x1 + 12 + txtSz.x, y1 + 6 + txtSz.y), IM_COL32(30, 160, 80, 230), 4.0f);
                                        drawList->AddText(ImVec2(x1 + 8, y1 + 5), IM_COL32(255, 255, 255, 255), tag);
                                    }
                                }
                            }

                            // 3. Mouse release: commit zoom limits from custom module
                            if (customDragState[i].isDragging && isMouseReleased) {
                                customDragState[i].isDragging = false;
                                customDragState[i].currentPt = ImPlot::GetPlotMousePos();
                                customDragState[i].currentPx = mousePx;

                                float dxPx = std::abs(customDragState[i].currentPx.x - customDragState[i].startPx.x);
                                float dyPx = std::abs(customDragState[i].currentPx.y - customDragState[i].startPx.y);

                                if (dxPx > 5.0f || dyPx > 5.0f) {
                                    WaveformZoomType finalZoomType = ZOOM_BOX_2D;
                                    if (activeZoomMode == ActiveZoomMode::X_Only) {
                                        finalZoomType = ZOOM_X_ONLY;
                                    } else if (activeZoomMode == ActiveZoomMode::Y_Only) {
                                        finalZoomType = ZOOM_Y_ONLY;
                                    } else if (activeZoomMode == ActiveZoomMode::Box_2D) {
                                        finalZoomType = ZOOM_BOX_2D;
                                    } else if (activeZoomMode == ActiveZoomMode::Adaptive) {
                                        if (dyPx <= 0.10f * dxPx || dyPx <= 12.0f) {
                                            finalZoomType = ZOOM_X_ONLY;
                                        } else if (dxPx <= 0.10f * dyPx || dxPx <= 12.0f) {
                                            finalZoomType = ZOOM_Y_ONLY;
                                        } else {
                                            finalZoomType = ZOOM_BOX_2D;
                                        }
                                    }

                                    pendingZoom[i].type = finalZoomType;
                                    pendingZoom[i].xMin = std::min(customDragState[i].startPt.x, customDragState[i].currentPt.x);
                                    pendingZoom[i].xMax = std::max(customDragState[i].startPt.x, customDragState[i].currentPt.x);
                                    pendingZoom[i].yMin = std::min(customDragState[i].startPt.y, customDragState[i].currentPt.y);
                                    pendingZoom[i].yMax = std::max(customDragState[i].startPt.y, customDragState[i].currentPt.y);
                                    pendingZoom[i].hasPending = true;
                                }
                            }
                        }

                        // Right-Click Context Menu (only when zoom mode is off)
                        if (!isZoomActive && ImGui::BeginPopupContextItem("PlotContextMenu")) {
                            if (ImGui::MenuItem("➕ Add Subplot Pane Below")) {
                                numPanes = std::min(numPanes + 1, 4);
                            }
                            if (numPanes > 1) {
                                if (ImGui::MenuItem("➖ Remove Subplot Pane")) {
                                    numPanes = std::max(numPanes - 1, 1);
                                }
                            }
                            if (ImGui::MenuItem("Fit Waveforms / Reset Zoom")) {
                                ImPlot::SetNextAxesToFit();
                            }
                            ImGui::EndPopup();
                        }

                        int varIdx = 0;
                        for (const auto& varPair : cat.variables) {
                            const std::string& varName = varPair.first;
                            const std::vector<double>& vals = varPair.second;
                            int count = (int)std::min(data.timeHistory.size(), vals.size());
                            if (count > 0) {
                                ImPlotSpec spec;
                                spec.LineColor = palette[varIdx % numColors];
                                spec.LineWeight = traceLineWidth;

                                InterpolationMode mode = globalPlotMode;
                                if (globalPlotMode == InterpolationMode::AutoHybrid) {
                                    mode = detectDefaultInterpolationMode(varName);
                                }

                                if (mode == InterpolationMode::AlwaysStairs) {
                                    ImPlot::PlotStairs(varName.c_str(), data.timeHistory.data(), vals.data(), count, spec);
                                } else if (mode == InterpolationMode::AutoHybrid) {
                                    std::vector<double> rawT(data.timeHistory.begin(), data.timeHistory.begin() + count);
                                    std::vector<double> rawY(vals.begin(), vals.begin() + count);
                                    std::vector<double> hT, hY;
                                    buildHybridVertices(rawT, rawY, hT, hY);
                                    ImPlot::PlotLine(varName.c_str(), hT.data(), hY.data(), (int)hT.size(), spec);
                                } else {
                                    ImPlot::PlotLine(varName.c_str(), data.timeHistory.data(), vals.data(), count, spec);
                                }
                            }
                            varIdx++;
                        }
                        ImPlot::EndPlot();
                    }
                    if (isZoomActive) {
                        ImPlot::PopStyleColor();
                    }
                }
                ImPlot::EndSubplots();
            }
        }
    }

    ImGui::EndChild();

    ImGui::End();
}

} // namespace CircuitSim
