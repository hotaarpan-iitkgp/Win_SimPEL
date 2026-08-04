#include "NetlistParser.hpp"
#include <nlohmann/json.hpp>
#include <iostream>
#include <unordered_map>
#include <set>
#include <algorithm>

using json = nlohmann::json;

namespace CircuitSimEngine {

static ComponentType stringToComponentType(const std::string& typeStr, const std::string& fallbackCategory = "") {
    std::string t = typeStr;
    if (t.empty()) t = fallbackCategory;

    if (t == "Resistor" || t == "R" || t == "resistors" || t == "VariableResistor") return ComponentType::Resistor;
    if (t == "Capacitor" || t == "C" || t == "capacitors") return ComponentType::Capacitor;
    if (t == "Inductor" || t == "L" || t == "inductors") return ComponentType::Inductor;
    if (t == "VoltageSource" || t == "V" || t == "DCVoltageSource" || t == "voltage_sources" || t == "dc_sources" || t == "ControlledVoltageSource") return ComponentType::VoltageSource;
    if (t == "ACVoltageSource" || t == "ac_sources") return ComponentType::ACVoltageSource;
    if (t == "CurrentSource" || t == "I" || t == "current_sources") return ComponentType::CurrentSource;
    if (t == "Diode" || t == "D" || t == "diodes") return ComponentType::Diode;
    if (t == "Switch" || t == "S" || t == "MOSFET" || t == "vg-FET" || t == "IGBT" || t == "IGBT_DIODE" || t == "IGCT" || t == "GTO" || t == "THYRISTOR" || t == "JFET" || t == "BJT" || t == "switches" || t == "analog_switches" || t == "mosfets") return ComponentType::Switch;
    if (t == "Voltmeter" || t == "VM" || t == "voltmeters") return ComponentType::Voltmeter;
    if (t == "Ammeter" || t == "AM" || t == "ammeters") return ComponentType::Ammeter;
    if (t == "UnifiedProbe" || t == "PROBE" || t == "probes") return ComponentType::UnifiedProbe;
    if (t == "Oscilloscope" || t == "SCOPE") return ComponentType::Oscilloscope;
    if (t == "Constant" || t == "CONST" || t == "constants") return ComponentType::Constant;
    if (t == "Gain" || t == "GAIN" || t == "gains") return ComponentType::Gain;
    if (t == "SummingJunction" || t == "SUM" || t == "summing_junctions") return ComponentType::SummingJunction;
    if (t == "Product" || t == "PRODUCT" || t == "product_blocks") return ComponentType::Product;
    if (t == "PWM_Generator" || t == "PWM" || t == "PWM_MASTER" || t == "pwm_generators") return ComponentType::PWM_Generator;
    if (t == "Triangle_Carrier" || t == "TRIANGLE" || t == "triangle_carriers") return ComponentType::Triangle_Carrier;
    if (t == "PI_Controller" || t == "PI" || t == "pi_controllers") return ComponentType::PI_Controller;
    if (t == "Comparator" || t == "COMP" || t == "E_COMP" || t == "comparators") return ComponentType::Comparator;
    if (t == "CustomScript" || t == "CSCRIPT" || t == "custom_scripts") return ComponentType::CustomScript;
    if (t == "XFMR" || t == "Transformer" || t == "transformers") return ComponentType::Transformer;
    return ComponentType::Unknown;
}

static void parseComponentItem(const json& item, const std::string& defaultCategoryType, std::vector<ComponentModel>& outList) {
    if (!item.is_object()) return;
    ComponentModel comp;
    if (item.contains("id")) comp.id = item["id"].get<std::string>();
    
    std::string itemType = item.contains("type") ? item["type"].get<std::string>() : defaultCategoryType;
    comp.type = stringToComponentType(itemType, defaultCategoryType);
    if (item.contains("label")) comp.label = item["label"].get<std::string>();

    if (item.contains("nodes") && item["nodes"].is_array()) {
        for (const auto& n : item["nodes"]) {
            if (n.is_string()) comp.nodes.push_back(n.get<std::string>());
            else if (n.is_number()) comp.nodes.push_back(std::to_string(n.get<int>()));
        }
    }

    if (item.contains("primary_windings") && item["primary_windings"].is_array()) {
        std::string pTurnsStr = "[";
        for (const auto& w : item["primary_windings"]) {
            if (w.contains("nodes") && w["nodes"].is_array()) {
                for (const auto& n : w["nodes"]) {
                    if (n.is_string()) comp.nodes.push_back(n.get<std::string>());
                }
            }
            if (w.contains("turns")) {
                if (w["turns"].is_number()) pTurnsStr += std::to_string(w["turns"].get<double>()) + " ";
                else if (w["turns"].is_string()) pTurnsStr += w["turns"].get<std::string>() + " ";
            }
        }
        pTurnsStr += "]";
        comp.parameters["primary_turns"] = pTurnsStr;
    }

    if (item.contains("secondary_windings") && item["secondary_windings"].is_array()) {
        std::string sTurnsStr = "[";
        for (const auto& w : item["secondary_windings"]) {
            if (w.contains("nodes") && w["nodes"].is_array()) {
                for (const auto& n : w["nodes"]) {
                    if (n.is_string()) comp.nodes.push_back(n.get<std::string>());
                }
            }
            if (w.contains("turns")) {
                if (w["turns"].is_number()) sTurnsStr += std::to_string(w["turns"].get<double>()) + " ";
                else if (w["turns"].is_string()) sTurnsStr += w["turns"].get<std::string>() + " ";
            }
        }
        sTurnsStr += "]";
        comp.parameters["secondary_turns"] = sTurnsStr;
    }

    if (item.contains("channels") && item["channels"].is_object()) {
        int inCount = 0;
        int outCount = 0;
        for (auto& [chKey, chVal] : item["channels"].items()) {
            if (chVal.is_string()) {
                std::string sVal = chVal.get<std::string>();
                if (chKey == "In" || chKey.rfind("In", 0) == 0 || chKey == "Plus" || chKey == "Minus" || chKey == "Ctrl") {
                    comp.parameters["input_" + std::to_string(inCount++)] = sVal;
                    comp.parameters[chKey] = sVal;
                } else if (chKey == "Out" || chKey.rfind("Out", 0) == 0) {
                    comp.parameters["output_" + std::to_string(outCount++)] = sVal;
                    comp.parameters[chKey] = sVal;
                }
            }
        }
    }

    if (item.contains("inputs") && item["inputs"].is_array()) {
        int idx = 0;
        for (const auto& inp : item["inputs"]) {
            if (inp.is_string()) {
                comp.parameters["input_" + std::to_string(idx)] = inp.get<std::string>();
                idx++;
            }
        }
    }

    if (item.contains("outputs") && item["outputs"].is_array()) {
        int idx = 0;
        for (const auto& out : item["outputs"]) {
            if (out.is_string()) {
                comp.parameters["output_" + std::to_string(idx)] = out.get<std::string>();
                idx++;
            }
        }
    }

    if (item.contains("output") && item["output"].is_string()) {
        comp.parameters["output"] = item["output"].get<std::string>();
    }

    if (item.contains("control_signal") && item["control_signal"].is_string()) {
        comp.parameters["control_signal"] = item["control_signal"].get<std::string>();
    }

    if (item.contains("parameters") && item["parameters"].is_object()) {
        for (auto& [k, v] : item["parameters"].items()) {
            if (v.is_string()) comp.parameters[k] = v.get<std::string>();
            else if (v.is_number()) comp.parameters[k] = std::to_string(v.get<double>());
        }
    }

    for (auto& [k, v] : item.items()) {
        if (k == "id" || k == "nodes" || k == "type" || k == "parameters" || k == "label" || k == "inputs" || k == "outputs") continue;
        if (v.is_string()) comp.parameters[k] = v.get<std::string>();
        else if (v.is_number()) comp.parameters[k] = std::to_string(v.get<double>());
    }

    outList.push_back(comp);
}

// Disjoint set helper for raw schematic wire graph discovery
struct DisjointSet {
    std::unordered_map<std::string, std::string> parent;

    std::string find(const std::string& i) {
        if (parent.find(i) == parent.end()) parent[i] = i;
        if (parent[i] == i) return i;
        return parent[i] = find(parent[i]);
    }

    void unite(const std::string& i, const std::string& j) {
        std::string rootI = find(i);
        std::string rootJ = find(j);
        if (rootI != rootJ) parent[rootI] = rootJ;
    }
};

static std::string endpointToPinKey(const json& ep) {
    if (!ep.is_object()) return "";
    if (ep.contains("type") && ep["type"] == "pin") {
        std::string cId = ep.contains("compId") ? ep["compId"].get<std::string>() : "";
        std::string term = ep.contains("terminal") ? ep["terminal"].get<std::string>() : "";
        return cId + "." + term;
    } else if (ep.contains("type") && ep["type"] == "wire") {
        std::string wId = ep.contains("wireId") ? ep["wireId"].get<std::string>() : "";
        return "WIRE:" + wId;
    }
    return "";
}

bool NetlistParser::parseJsonString(const std::string& jsonContent, 
                                std::vector<ComponentModel>& outPhysical, 
                                std::vector<ComponentModel>& outControl, 
                                SimulationConfig& outConfig) {
    try {
        auto root = json::parse(jsonContent);

        // Parse simulation parameters
        if (root.contains("simulation_parameters")) {
            auto simParams = root["simulation_parameters"];
            if (simParams.contains("t_stop")) {
                if (simParams["t_stop"].is_number()) outConfig.stopTime = simParams["t_stop"].get<double>();
                else if (simParams["t_stop"].is_string()) outConfig.stopTime = ExpressionEvaluator::parseScientific(simParams["t_stop"].get<std::string>());
            }
            if (simParams.contains("stop_time")) {
                if (simParams["stop_time"].is_number()) outConfig.stopTime = simParams["stop_time"].get<double>();
                else if (simParams["stop_time"].is_string()) outConfig.stopTime = ExpressionEvaluator::parseScientific(simParams["stop_time"].get<std::string>());
            }
            if (simParams.contains("stopTime")) {
                if (simParams["stopTime"].is_number()) outConfig.stopTime = simParams["stopTime"].get<double>();
                else if (simParams["stopTime"].is_string()) outConfig.stopTime = ExpressionEvaluator::parseScientific(simParams["stopTime"].get<std::string>());
            }
            if (simParams.contains("dt")) {
                if (simParams["dt"].is_number()) outConfig.stepSize = simParams["dt"].get<double>();
                else if (simParams["dt"].is_string()) outConfig.stepSize = ExpressionEvaluator::parseScientific(simParams["dt"].get<std::string>());
            }
            if (simParams.contains("step_size")) {
                if (simParams["step_size"].is_number()) outConfig.stepSize = simParams["step_size"].get<double>();
                else if (simParams["step_size"].is_string()) outConfig.stepSize = ExpressionEvaluator::parseScientific(simParams["step_size"].get<std::string>());
            }
            if (simParams.contains("stepSize")) {
                if (simParams["stepSize"].is_number()) outConfig.stepSize = simParams["stepSize"].get<double>();
                else if (simParams["stepSize"].is_string()) outConfig.stepSize = ExpressionEvaluator::parseScientific(simParams["stepSize"].get<std::string>());
            }
            if (simParams.contains("solver") && simParams["solver"].is_string()) {
                outConfig.solver = simParams["solver"].get<std::string>();
            }
            if (simParams.contains("solverMethod") && simParams["solverMethod"].is_string()) {
                outConfig.solverMethod = simParams["solverMethod"].get<std::string>();
            }
            if (simParams.contains("step_type") && simParams["step_type"].is_string()) {
                outConfig.step_type = simParams["step_type"].get<std::string>();
            }
        }

        bool hasPreExtractedStage = false;
        if (root.contains("physical_stage") && !root["physical_stage"].empty()) hasPreExtractedStage = true;
        if (root.contains("control_loops") && !root["control_loops"].empty()) hasPreExtractedStage = true;

        if (hasPreExtractedStage) {
            if (root.contains("physical_stage")) {
                const auto& stage = root["physical_stage"];
                if (stage.is_array()) {
                    for (const auto& item : stage) parseComponentItem(item, "", outPhysical);
                } else if (stage.is_object()) {
                    for (auto& [catKey, catVal] : stage.items()) {
                        if (catVal.is_array()) {
                            for (const auto& item : catVal) parseComponentItem(item, catKey, outPhysical);
                        }
                    }
                }
            }

            if (root.contains("control_loops")) {
                const auto& ctrl = root["control_loops"];
                if (ctrl.is_array()) {
                    for (const auto& item : ctrl) parseComponentItem(item, "", outControl);
                } else if (ctrl.is_object()) {
                    for (auto& [catKey, catVal] : ctrl.items()) {
                        if (catVal.is_array()) {
                            for (const auto& item : catVal) parseComponentItem(item, catKey, outControl);
                        }
                    }
                }
            }

            if (root.contains("probes") && root["probes"].is_array()) {
                for (const auto& item : root["probes"]) parseComponentItem(item, "probes", outControl);
            }
        } else if (root.contains("components") && root["components"].is_array() && root.contains("wires") && root["wires"].is_array()) {
            // Raw Schematic Export (components + wires) - Run Disjoint-Set Wire Graph Discovery
            std::vector<ComponentModel> rawComps;
            for (const auto& item : root["components"]) {
                parseComponentItem(item, "", rawComps);
            }

            DisjointSet dset;
            for (const auto& w : root["wires"]) {
                std::string kFrom = endpointToPinKey(w["from"]);
                std::string kTo = endpointToPinKey(w["to"]);
                if (!kFrom.empty() && !kTo.empty()) {
                    dset.unite(kFrom, kTo);
                }
            }

            // Map pin keys to root representative
            std::unordered_map<std::string, std::string> rootToNodeName;
            int nodeCounter = 1;

            // Identify ground reference pins (terminal B of V1, or GND)
            for (const auto& c : rawComps) {
                if (c.type == ComponentType::VoltageSource) {
                    std::string gndRoot = dset.find(c.id + ".B");
                    if (!gndRoot.empty() && rootToNodeName.find(gndRoot) == rootToNodeName.end()) {
                        rootToNodeName[gndRoot] = "0";
                    }
                }
            }

            auto getNodeNameForPin = [&](const std::string& pinKey) -> std::string {
                std::string r = dset.find(pinKey);
                if (rootToNodeName.find(r) != rootToNodeName.end()) return rootToNodeName[r];
                std::string nName = "node_" + std::to_string(nodeCounter++);
                rootToNodeName[r] = nName;
                return nName;
            };

            // Map GOTO, FROM, and direct wire connections to switch gates & control inputs
            std::unordered_map<std::string, std::string> gotoTagToSignalKey;
            for (const auto& w : root["wires"]) {
                std::string kFrom = endpointToPinKey(w["from"]);
                std::string kTo = endpointToPinKey(w["to"]);
                
                for (const auto& c : rawComps) {
                    if (c.parameters.count("tag")) {
                        std::string tag = c.parameters.at("tag");
                        if (kTo == c.id + ".In") {
                            gotoTagToSignalKey[tag] = kFrom;
                        }
                    }
                }

                size_t dotPos = kTo.find('.');
                if (dotPos != std::string::npos) {
                    std::string compId = kTo.substr(0, dotPos);
                    std::string pinName = kTo.substr(dotPos + 1);
                    if (pinName == "G" || pinName == "Ctrl") {
                        for (auto& c : rawComps) {
                            if (c.id == compId && c.type == ComponentType::Switch) {
                                c.parameters["control_signal"] = kFrom;
                            }
                        }
                    } else if (pinName == "In" || pinName.rfind("In", 0) == 0) {
                        for (auto& c : rawComps) {
                            if (c.id == compId) {
                                c.parameters[pinName] = kFrom;
                            }
                        }
                    }
                }
            }

            // Build netlist for each component
            for (auto& c : rawComps) {
                if (c.type == ComponentType::Resistor || c.type == ComponentType::Capacitor || 
                    c.type == ComponentType::Inductor || c.type == ComponentType::VoltageSource || 
                    c.type == ComponentType::ACVoltageSource || c.type == ComponentType::CurrentSource || 
                    c.type == ComponentType::Diode || c.type == ComponentType::Switch || 
                    c.type == ComponentType::Voltmeter || c.type == ComponentType::Ammeter) {
                    
                    std::string pA = c.id + ".A";
                    std::string pB = c.id + ".B";
                    if (c.type == ComponentType::Switch) { pA = c.id + ".D"; pB = c.id + ".S"; }
                    
                    c.nodes.push_back(getNodeNameForPin(pA));
                    c.nodes.push_back(getNodeNameForPin(pB));

                    if (c.type == ComponentType::Switch && c.parameters.count("Gate_Signal_Label")) {
                        std::string gateTag = c.parameters["Gate_Signal_Label"];
                        if (gotoTagToSignalKey.count(gateTag)) {
                            c.parameters["control_signal"] = gotoTagToSignalKey[gateTag];
                        }
                    }

                    outPhysical.push_back(c);
                }
                else if (c.type == ComponentType::Transformer) {
                    // Multi-winding pin discovery: P1A/P1B..P4A/P4B, S1A/S1B..S4A/S4B
                    if (c.nodes.empty()) {
                        for (int w = 1; w <= 4; ++w) {
                            std::string pA = c.id + ".P" + std::to_string(w) + "A";
                            std::string pB = c.id + ".P" + std::to_string(w) + "B";
                            std::string rA = dset.find(pA);
                            std::string rB = dset.find(pB);
                            if (!rA.empty() || !rB.empty() || w == 1) {
                                c.nodes.push_back(getNodeNameForPin(pA));
                                c.nodes.push_back(getNodeNameForPin(pB));
                            }
                        }

                        for (int w = 1; w <= 4; ++w) {
                            std::string sA = c.id + ".S" + std::to_string(w) + "A";
                            std::string sB = c.id + ".S" + std::to_string(w) + "B";
                            std::string rA = dset.find(sA);
                            std::string rB = dset.find(sB);
                            if (!rA.empty() || !rB.empty() || w == 1) {
                                c.nodes.push_back(getNodeNameForPin(sA));
                                c.nodes.push_back(getNodeNameForPin(sB));
                            }
                        }
                    }

                    outPhysical.push_back(c);
                }
                else {
                    // Control blocks
                    outControl.push_back(c);
                }
            }
        } else if (root.contains("components") && root["components"].is_array()) {
            for (const auto& item : root["components"]) parseComponentItem(item, "", outPhysical);
        }

        return true;
    } catch (const std::exception& e) {
        std::cerr << "Netlist Json Parse Error: " << e.what() << std::endl;
        return false;
    }
}

} // namespace CircuitSimEngine
