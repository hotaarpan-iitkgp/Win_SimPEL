// Block registry: loads, matches the frozen palette table entry for entry, covers
// every block type used by the golden design corpus, and rejects malformed data.
#include "reference_palette.hpp"
#include "reference_type_mapping.hpp"
#include "reference_inspector.hpp"

#include <algorithm>

#include "engine/BlockRegistry.hpp"
#include "engine/NetlistBuilder.hpp"
#include "engine/NetlistGenerator.hpp"

#include <nlohmann/json.hpp>

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <set>
#include <string>

using namespace CircuitSim;

static int g_pass = 0, g_fail = 0;
static void check(bool ok, const std::string& what) {
    if (ok) ++g_pass;
    else { ++g_fail; std::printf("FAIL: %s\n", what.c_str()); }
}

static bool throwsWith(const std::vector<std::pair<std::string, std::string>>& files, const std::string& needle) {
    try {
        BlockRegistry::fromJsonTexts(files);
    } catch (const std::exception& e) {
        if (std::string(e.what()).find(needle) != std::string::npos) return true;
        std::printf("  (message was: %s)\n", e.what());
        return false;
    }
    return false;
}

int main(int argc, char** argv) {
    const std::string designDir = argc > 1 ? argv[1] : "tests/golden/designs";
    const BlockRegistry& reg = BlockRegistry::instance();

    // 1. Palette reproduces the original table exactly, in order.
    const auto& ref = golden_ref_palette::table();
    const auto& pal = reg.palette();
    check(pal.size() == ref.size(), "palette size " + std::to_string(pal.size()) + " == " + std::to_string(ref.size()));
    for (size_t i = 0; i < std::min(pal.size(), ref.size()); ++i) {
        const auto& a = pal[i];
        const auto& b = ref[i];
        bool same = a.text == b.buttonText && a.label == b.label && a.prefix == b.prefix && a.uiType == b.type &&
                    a.type == b.rawTypeStr && a.category == b.category && a.subcategory == b.subcategory &&
                    a.basic == b.isBasic && a.params == b.defaultParams && a.order == (int)i;
        check(same, "palette entry " + std::to_string(i) + " (" + b.buttonText + ")");
    }

    // 2. Enum name table round-trips every enumerator.
    for (int i = 0; i < componentTypeCount(); ++i) {
        ComponentType t;
        bool ok = componentTypeFromName(componentTypeName((ComponentType)i), t) && (int)t == i;
        if (!ok) check(false, std::string("enum round trip ") + componentTypeName((ComponentType)i));
    }
    check(std::string(componentTypeName(ComponentType::Unknown)) == "Unknown", "Unknown name");

    // 3. Every block type in the golden corpus is in the registry.
    std::set<std::string> corpus;
    for (const auto& e : std::filesystem::directory_iterator(designDir)) {
        if (e.path().extension() != ".json") continue;
        std::ifstream f(e.path(), std::ios::binary);
        auto j = nlohmann::json::parse(f);
        for (const auto& c : j.value("components", nlohmann::json::array())) corpus.insert(c.value("type", ""));
    }
    for (const auto& t : corpus) check(reg.find(t) != nullptr, "corpus type '" + t + "' registered");
    check(corpus.size() > 100, "corpus scanned (" + std::to_string(corpus.size()) + " types)");

    // 4. Instances and pins.
    {
        ComponentInstance r = reg.makeInstance("R", "R7", {{"value", "4.7k"}});
        check(r.rawTypeStr == "R" && r.parameters.at("value") == "4.7k" && r.type == ComponentType::Resistor, "makeInstance R");
        auto mp = reg.pins("MOSFET");
        std::set<std::string> names;
        for (const auto& p : mp) names.insert(p.name);
        check(mp.size() == 3, "MOSFET has 3 pins (" + std::to_string(mp.size()) + ")");
        bool hasCtrl = false;
        for (const auto& p : mp) hasCtrl |= p.isControl;
        check(hasCtrl, "MOSFET gate is a control pin");
        int empty = 0;
        std::string emptyList;
        for (const auto& b : reg.blocks())
            if (reg.pins(b.type).empty()) { ++empty; emptyList += " " + b.type; }
        if (empty) std::printf("blocks without pins:%s\n", emptyList.c_str());
        std::printf("blocks: %zu, palette entries: %zu, blocks without pins: %d\n", reg.blocks().size(), pal.size(), empty);
        bool threw = false;
        try { reg.makeInstance("MOSFT", "Q1"); } catch (const std::exception& e) {
            threw = std::string(e.what()).find("MOSFET") != std::string::npos;
        }
        check(threw, "unknown type error suggests MOSFET");
        auto s = reg.suggest("resistor");
        check(!s.empty(), "suggest('resistor') non-empty");
    }

    // 4b. Type mapping is data-driven and identical to the original hard-coded table
    //     for every string it knew, every registry name and every corpus type.
    {
        std::set<std::string> probe(corpus.begin(), corpus.end());
        for (const auto& s : golden_ref_types::legacyLiterals()) probe.insert(s);
        for (const auto& b : reg.blocks()) {
            probe.insert(b.type);
            for (const auto& a : b.aliases) probe.insert(a);
        }
        for (const char* s : {"", "nonsense", "r", "mosfet", "Mosfet", "SUM ", "V\t"}) probe.insert(s);
        int bad = 0;
        for (const auto& s : probe) {
            if (stringToComponentType(s) != golden_ref_types::legacyStringToComponentType(s)) {
                if (bad++ < 10) std::printf("  type mapping differs for '%s'\n", s.c_str());
            }
        }
        check(bad == 0, "type mapping identical for " + std::to_string(probe.size()) + " strings");
        size_t legacyKnown = 0;
        for (const auto& s : golden_ref_types::legacyLiterals()) legacyKnown += reg.resolve(s) != nullptr;
        check(legacyKnown == golden_ref_types::legacyLiterals().size(), "every legacy type string resolves to a block");
    }

    // 4c. Roles match what the netlist generator actually does: exactly the
    //     "model" blocks produce a netlist entry for a lone instance.
    {
        int bad = 0;
        for (const auto& b : reg.blocks()) {
            CircuitDesign cd;
            cd.components.push_back(reg.makeInstance(b.type, "XQ1"));
            NetlistBuilder::buildNodesForCircuit(cd);
            auto j = nlohmann::json::parse(generateNetlistJson(cd));
            int hits = 0;
            for (const char* sec : {"control_loops", "physical_stage"})
                for (auto& [k, v] : j[sec].items())
                    if (v.is_array())
                        for (auto& it : v)
                            if (it.is_object() && it.value("id", "").rfind("XQ1", 0) == 0) ++hits;
            const bool emitted = hits > 0;
            if (emitted != (b.role == BlockRole::Model)) {
                if (bad++ < 10)
                    std::printf("  %s: role %s but netlist entry %s\n", b.type.c_str(), blockRoleName(b.role),
                                emitted ? "present" : "absent");
            }
        }
        check(bad == 0, "block roles agree with netlist emission");
    }

    // 4d. Registry-supplied pin layouts.
    {
        auto same = [&](const std::string& a, const std::string& b) {
            auto pa = reg.pins(a), pb = reg.pins(b);
            if (pa.size() != pb.size() || pa.empty()) return false;
            for (size_t i = 0; i < pa.size(); ++i)
                if (pa[i].name != pb[i].name || pa[i].relX != pb[i].relX || pa[i].relY != pb[i].relY) return false;
            return true;
        };
        check(same("CONSTANT", "CONST"), "CONSTANT uses CONST pins");
        check(same("PARK_TRANSFORM", "PARK"), "PARK_TRANSFORM uses PARK pins");
        check(same("INV_PARK_TRANSFORM", "INV_PARK"), "INV_PARK_TRANSFORM uses INV_PARK pins");
        check(same("CLARKE_TRANSFORM", "CLARKE"), "CLARKE_TRANSFORM uses CLARKE pins");
        for (const auto& b : reg.blocks())
            if (b.role == BlockRole::Model && reg.pins(b.type).empty())
                check(false, "simulated block " + b.type + " has pins");

        auto r2 = BlockRegistry::fromJsonTexts({{"n.json", R"({"schema":1,"category":"control","blocks":[
            {"type":"NEWBLK","name":"New","subcategory":"S","ui_type":"Unknown",
             "pins":[{"name":"In","x":-30,"y":0,"dx":-1,"dy":0,"kind":"control"},
                     {"name":"Out","x":30,"y":0,"dx":1,"dy":0,"kind":"control"}]}]})"}});
        const BlockDef* nb = r2.find("NEWBLK");
        check(nb && nb->declaredPins.size() == 2 && nb->declaredPins[1].relX == 30.0f, "explicit pins parsed");
    }

    // 4e. Inspector dropdowns ("choices") reproduce the old hard-coded rules for
    //     every block and parameter key. One deliberate difference: the old code
    //     offered the filter-type list for a "filter_type" parameter on ANY block;
    //     the registry only declares it for the filter blocks.
    {
        int bad = 0, withChoices = 0;
        for (const auto& b : reg.blocks()) {
            std::string up = b.type;
            std::transform(up.begin(), up.end(), up.begin(), ::toupper);
            for (const auto& k : golden_ref_inspector::legacyKeys()) {
                std::vector<std::string> want = golden_ref_inspector::legacyChoices(up, k);
                if (k == "filter_type" && b.type != "FILTER_1ST" && b.type != "FILTER_2ND") want.clear();
                const auto* got = b.choicesFor(k);
                std::vector<std::string> g = got ? *got : std::vector<std::string>{};
                if (!g.empty()) ++withChoices;
                if (g != want && bad++ < 10) std::printf("  choices differ: %s.%s\n", b.type.c_str(), k.c_str());
            }
            if (b.choicesFor("FUNCTION") != b.choicesFor("function")) check(false, "choicesFor is case-insensitive");
        }
        check(bad == 0, "inspector choices identical (" + std::to_string(withChoices) + " block/parameter pairs)");
    }

    // 5. Schema validation.
    const std::string good = R"({"schema":1,"category":"control","blocks":[{"type":"A","name":"A","subcategory":"S","ui_type":"Gain"}]})";
    check(BlockRegistry::fromJsonTexts({{"g.json", good}}).blocks().size() == 1, "minimal file loads");
    check(throwsWith({{"x.json", "{"}}, "x.json"), "invalid JSON names the file");
    check(throwsWith({{"x.json", R"({"schema":2,"category":"control","blocks":[]})"}}, "schema"), "wrong schema rejected");
    check(throwsWith({{"x.json", R"({"schema":1,"category":"ctrl","blocks":[]})"}}, "unknown category"), "unknown category rejected");
    check(throwsWith({{"a.json", good}, {"b.json", good}}, "duplicate type"), "duplicate type across files rejected");
    check(throwsWith({{"x.json", R"({"schema":1,"category":"control","blocks":[{"type":"A","name":"A","subcategory":"S","ui_type":"Gain",
        "palette":[{"order":0,"text":"t","label":"l","prefix":"A","ui_type":"NoSuchEnum"}]}]})"}}, "unknown ui_type"),
          "unknown ui_type rejected");
    check(throwsWith({{"x.json", R"({"schema":1,"category":"control","blocks":[{"type":"A","name":"A","subcategory":"S","ui_type":"Gain",
        "params":[{"name":"k","default":"1"},{"name":"k","default":"2"}]}]})"}}, "duplicate parameter"),
          "duplicate parameter rejected");
    check(throwsWith({{"x.json", R"({"schema":1,"category":"control","blocks":[{"type":"A","name":"A","subcategory":"S","ui_type":"Gain",
        "palette":[{"order":3,"text":"t","label":"l","prefix":"A","ui_type":"Gain"},
                   {"order":3,"text":"u","label":"l","prefix":"A","ui_type":"Gain"}]}]})"}}, "order 3"),
          "duplicate palette order rejected");


    check(throwsWith({{"x.json", R"({"schema":1,"category":"control","blocks":[{"type":"A","name":"A","subcategory":"S"}]})"}},
                     "ui_type"), "missing ui_type rejected");
    check(throwsWith({{"x.json", R"({"schema":1,"category":"control","blocks":[{"type":"A","name":"A","subcategory":"S","ui_type":"Gain","role":"weird"}]})"}},
                     "unknown role"), "unknown role rejected");
    check(throwsWith({{"x.json", R"({"schema":1,"category":"control","blocks":[
        {"type":"A","name":"A","subcategory":"S","ui_type":"Gain","aliases":["Z"]},
        {"type":"B","name":"B","subcategory":"S","ui_type":"Gain","aliases":["Z"]}]})"}}, "alias 'Z'"),
          "duplicate alias rejected");
    check(throwsWith({{"x.json", R"({"schema":1,"category":"control","blocks":[
        {"type":"A","name":"A","subcategory":"S","ui_type":"Gain","aliases":["B"]},
        {"type":"B","name":"B","subcategory":"S","ui_type":"Gain"}]})"}}, "also a block type"),
          "alias equal to a type rejected");
    check(throwsWith({{"x.json", R"({"schema":1,"category":"control","blocks":[
        {"type":"A","name":"A","subcategory":"S","ui_type":"Gain","pins_from":"NOPE"}]})"}}, "pins_from"),
          "pins_from to unknown type rejected");
    check(throwsWith({{"x.json", R"({"schema":1,"category":"control","blocks":[
        {"type":"A","name":"A","subcategory":"S","ui_type":"Gain","choices":{"Mode":["a"]}}]})"}}, "lower-case"),
          "upper-case choices key rejected");
    check(throwsWith({{"x.json", R"({"schema":1,"category":"control","blocks":[
        {"type":"A","name":"A","subcategory":"S","ui_type":"Gain","choices":{"mode":[]}}]})"}}, "non-empty"),
          "empty choices rejected");
    check(throwsWith({{"x.json", R"({"schema":1,"category":"control","blocks":[
        {"type":"A","name":"A","subcategory":"S","ui_type":"Gain","netlist":{"section":"gains","fields":{}}}]})"}},
                     "control_loops.<name>"), "netlist section without stage rejected");
    check(throwsWith({{"x.json", R"({"schema":1,"category":"control","blocks":[
        {"type":"A","name":"A","subcategory":"S","ui_type":"Gain","netlist":{"section":"control_loops.gains","fields":{"id":"x"}}}]})"}},
                     "'id'"), "netlist id override rejected");
    check(throwsWith({{"x.json", R"({"schema":1,"category":"control","blocks":[
        {"type":"A","name":"A","subcategory":"S","ui_type":"Gain","netlist":{"section":"control_loops.gains","fields":{"k":{"param":"p"}}}}]})"}},
                     "default"), "netlist param without default rejected");
    check(throwsWith({{"x.json", R"({"schema":1,"category":"control","blocks":[
        {"type":"Ab","name":"A","subcategory":"S","ui_type":"Gain","netlist":{"section":"control_loops.gains","fields":{}}},
        {"type":"AB","name":"B","subcategory":"S","ui_type":"Gain","netlist":{"section":"control_loops.gains","fields":{}}}]})"}},
                     "case-insensitively"), "case-insensitive clash between emitted blocks rejected");
    {
        int spec = 0;
        for (const auto& b : reg.blocks()) spec += b.netlist.present;
        check(spec == 46, "46 blocks emitted from registry specs (" + std::to_string(spec) + ")");
        check(reg.netlistBlockFor("inductor") == reg.find("L"), "netlist lookup is case-insensitive");
        check(reg.netlistBlockFor("MOSFET") == nullptr, "hand-written blocks have no spec");
    }

    std::printf("%d/%d checks passed\n", g_pass, g_pass + g_fail);
    return g_fail == 0 ? 0 : 1;
}
