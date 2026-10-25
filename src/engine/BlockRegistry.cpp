#include "BlockRegistry.hpp"
#include "EmbeddedFiles.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <set>
#include <sstream>
#include <stdexcept>

namespace CircuitSim {

namespace {

struct TypeName { const char* name; ComponentType value; };
const TypeName kTypeNames[] = {
#include "ComponentTypeNames.inc"
};
constexpr int kTypeNameCount = (int)(sizeof(kTypeNames) / sizeof(kTypeNames[0]));
// Every enumerator must be in the table, in declaration order.
static_assert(kTypeNameCount == (int)ComponentType::Unknown + 1,
              "ComponentTypeNames.inc is out of date with Components.hpp");

const std::set<std::string> kKnownCategories = {"general", "control", "electrical", "magnetic"};

[[noreturn]] void fail(const std::string& file, const std::string& where, const std::string& what) {
    throw std::runtime_error("block registry: " + file + (where.empty() ? "" : " (" + where + ")") + ": " + what);
}

std::string reqString(const nlohmann::json& o, const char* key, const std::string& file, const std::string& where,
                      bool allowEmpty = false) {
    if (!o.contains(key) || !o[key].is_string()) fail(file, where, std::string("missing string field '") + key + "'");
    std::string s = o[key].get<std::string>();
    if (s.empty() && !allowEmpty) fail(file, where, std::string("field '") + key + "' is empty");
    return s;
}

std::string optString(const nlohmann::json& o, const char* key, const std::string& file, const std::string& where) {
    if (!o.contains(key)) return "";
    if (!o[key].is_string()) fail(file, where, std::string("field '") + key + "' must be a string");
    return o[key].get<std::string>();
}

std::string lower(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return (char)std::tolower(c); });
    return s;
}

size_t editDistance(const std::string& a, const std::string& b) {
    std::vector<size_t> prev(b.size() + 1), cur(b.size() + 1);
    for (size_t j = 0; j <= b.size(); ++j) prev[j] = j;
    for (size_t i = 1; i <= a.size(); ++i) {
        cur[0] = i;
        for (size_t j = 1; j <= b.size(); ++j)
            cur[j] = std::min({prev[j] + 1, cur[j - 1] + 1, prev[j - 1] + (a[i - 1] == b[j - 1] ? 0u : 1u)});
        std::swap(prev, cur);
    }
    return prev[b.size()];
}

} // namespace

bool componentTypeFromName(const std::string& name, ComponentType& out) {
    for (const auto& t : kTypeNames)
        if (name == t.name) { out = t.value; return true; }
    return false;
}

const char* componentTypeName(ComponentType t) {
    int i = (int)t;
    return (i >= 0 && i < kTypeNameCount) ? kTypeNames[i].name : "Unknown";
}

int componentTypeCount() { return kTypeNameCount; }

const BlockParam* BlockDef::findParam(const std::string& n) const {
    for (const auto& p : params) if (p.name == n) return &p;
    return nullptr;
}

const std::vector<std::string>* BlockDef::choicesFor(const std::string& name) const {
    auto it = choices.find(lower(name));
    return it == choices.end() ? nullptr : &it->second;
}

BlockRegistry BlockRegistry::fromJsonTexts(const std::vector<std::pair<std::string, std::string>>& files) {
    BlockRegistry r;
    std::map<int, std::string> orders;   // order -> owner, for duplicate detection
    for (const auto& [file, text] : files) {
        nlohmann::json doc;
        try {
            doc = nlohmann::json::parse(text);
        } catch (const std::exception& e) {
            fail(file, "", std::string("invalid JSON: ") + e.what());
        }
        if (!doc.is_object()) fail(file, "", "top level must be an object");
        if (!doc.contains("schema") || !doc["schema"].is_number_integer() || doc["schema"].get<int>() != 1)
            fail(file, "", "unsupported or missing 'schema' (expected 1)");
        const std::string category = reqString(doc, "category", file, "");
        if (!kKnownCategories.count(category))
            fail(file, "", "unknown category '" + category + "' (expected general, control, electrical or magnetic)");
        if (!doc.contains("blocks") || !doc["blocks"].is_array()) fail(file, "", "missing 'blocks' array");

        for (const auto& b : doc["blocks"]) {
            if (!b.is_object()) fail(file, "", "every entry in 'blocks' must be an object");
            BlockDef def;
            def.type = reqString(b, "type", file, "block");
            const std::string where = "block " + def.type;
            if (r.index_.count(def.type))
                fail(file, where, "duplicate type, already defined in " + r.blocks_[r.index_[def.type]].sourceFile);
            def.name = reqString(b, "name", file, where);
            def.category = category;
            def.subcategory = reqString(b, "subcategory", file, where);
            def.description = optString(b, "description", file, where);
            def.sourceFile = file;
            def.uiTypeName = reqString(b, "ui_type", file, where);
            if (!componentTypeFromName(def.uiTypeName, def.uiType))
                fail(file, where, "unknown ui_type '" + def.uiTypeName + "'");
            if (b.contains("aliases")) {
                if (!b["aliases"].is_array()) fail(file, where, "'aliases' must be an array of strings");
                for (const auto& a : b["aliases"]) {
                    if (!a.is_string() || a.get<std::string>().empty()) fail(file, where, "aliases must be non-empty strings");
                    def.aliases.push_back(a.get<std::string>());
                }
            }
            {
                const std::string role = optString(b, "role", file, where);
                if (role.empty() || role == "model") def.role = BlockRole::Model;
                else if (role == "structural") def.role = BlockRole::Structural;
                else if (role == "observer") def.role = BlockRole::Observer;
                else if (role == "unsupported") def.role = BlockRole::Unsupported;
                else fail(file, where, "unknown role '" + role + "' (expected model, structural, observer or unsupported)");
            }
            if (b.contains("legacy")) {
                if (!b["legacy"].is_boolean()) fail(file, where, "'legacy' must be true or false");
                def.legacy = b["legacy"].get<bool>();
            }
            def.pinsFrom = optString(b, "pins_from", file, where);
            if (b.contains("pins")) {
                if (!b["pins"].is_array()) fail(file, where, "'pins' must be an array");
                for (const auto& p : b["pins"]) {
                    if (!p.is_object()) fail(file, where, "every pin must be an object");
                    BlockPin bp;
                    bp.name = reqString(p, "name", file, where);
                    auto num = [&](const char* k) -> float {
                        if (!p.contains(k) || !p[k].is_number()) fail(file, where + "." + bp.name, std::string("pin needs numeric '") + k + "'");
                        return p[k].get<float>();
                    };
                    bp.relX = num("x"); bp.relY = num("y");
                    bp.dirX = num("dx"); bp.dirY = num("dy");
                    const std::string kind = reqString(p, "kind", file, where + "." + bp.name);
                    if (kind != "control" && kind != "power") fail(file, where + "." + bp.name, "pin kind must be control or power");
                    bp.isControl = (kind == "control");
                    def.declaredPins.push_back(bp);
                }
                if (!def.pinsFrom.empty()) fail(file, where, "use either 'pins' or 'pins_from', not both");
            }

            if (b.contains("params")) {
                if (!b["params"].is_array()) fail(file, where, "'params' must be an array");
                for (const auto& p : b["params"]) {
                    if (!p.is_object()) fail(file, where, "every parameter must be an object");
                    BlockParam bp;
                    bp.name = reqString(p, "name", file, where);
                    bp.defaultValue = reqString(p, "default", file, where + "." + bp.name, true);
                    bp.unit = optString(p, "unit", file, where + "." + bp.name);
                    bp.description = optString(p, "description", file, where + "." + bp.name);
                    if (def.findParam(bp.name)) fail(file, where, "duplicate parameter '" + bp.name + "'");
                    def.params.push_back(bp);
                }
            }

            if (b.contains("choices")) {
                if (!b["choices"].is_object()) fail(file, where, "'choices' must be an object of name -> [values]");
                for (const auto& [k, v] : b["choices"].items()) {
                    if (k.empty() || lower(k) != k) fail(file, where, "choices key '" + k + "' must be lower-case");
                    if (!v.is_array() || v.empty()) fail(file, where, "choices for '" + k + "' must be a non-empty array");
                    std::vector<std::string> vals;
                    for (const auto& x : v) {
                        if (!x.is_string()) fail(file, where, "choices for '" + k + "' must be strings");
                        vals.push_back(x.get<std::string>());
                    }
                    def.choices[k] = std::move(vals);
                }
            }

            if (b.contains("netlist")) {
                const auto& n = b["netlist"];
                const std::string nw = where + ".netlist";
                if (!n.is_object()) fail(file, nw, "must be an object");
                const std::string sec = reqString(n, "section", file, nw);
                const size_t dot = sec.find('.');
                def.netlist.stage = dot == std::string::npos ? "" : sec.substr(0, dot);
                def.netlist.section = dot == std::string::npos ? "" : sec.substr(dot + 1);
                if ((def.netlist.stage != "control_loops" && def.netlist.stage != "physical_stage") ||
                    def.netlist.section.empty())
                    fail(file, nw, "section must be control_loops.<name> or physical_stage.<name>");
                if (!n.contains("fields") || !n["fields"].is_object()) fail(file, nw, "missing 'fields' object");
                for (const auto& [k, v] : n["fields"].items()) {
                    const std::string fw = nw + "." + k;
                    if (k == "id") fail(file, fw, "'id' is always emitted and cannot be overridden");
                    NetlistField f;
                    f.key = k;
                    if (v.is_string()) {
                        f.kind = NetlistField::Kind::Text;
                        f.text = v.get<std::string>();
                    } else if (v.is_number()) {
                        f.kind = NetlistField::Kind::Number;
                        f.isInt = v.is_number_integer();
                        f.number = v.get<double>();
                    } else if (v.is_array()) {
                        f.kind = NetlistField::Kind::SignalArray;
                        for (const auto& s : v) {
                            if (!s.is_object() || !s.contains("signal") || !s["signal"].is_string())
                                fail(file, fw, "array entries must be {\"signal\": \"<pin>\"}");
                            f.signals.push_back(s["signal"].get<std::string>());
                        }
                    } else if (v.is_object() && v.contains("signal")) {
                        f.kind = NetlistField::Kind::Signal;
                        f.signals.push_back(reqString(v, "signal", file, fw));
                    } else if (v.is_object() && v.contains("nodes")) {
                        f.kind = NetlistField::Kind::Nodes;
                    } else if (v.is_object() && v.contains("param")) {
                        f.kind = NetlistField::Kind::Param;
                        f.text = reqString(v, "param", file, fw);
                        const std::string as = optString(v, "as", file, fw);
                        if (!v.contains("default")) fail(file, fw, "param field needs a 'default'");
                        const auto& d = v["default"];
                        if (as == "string") {
                            if (!d.is_string()) fail(file, fw, "string param default must be a string");
                            f.asString = true;
                            f.defaultText = d.get<std::string>();
                        } else if (as.empty() || as == "number" || as == "int") {
                            if (!d.is_number()) fail(file, fw, "numeric param default must be a number");
                            f.isInt = (as == "int");
                            f.number = d.get<double>();
                        } else {
                            fail(file, fw, "'as' must be number, int or string");
                        }
                    } else {
                        fail(file, fw, "unrecognised field form");
                    }
                    def.netlist.fields.push_back(std::move(f));
                }
                def.netlist.present = true;
            }

            if (b.contains("palette")) {
                if (!b["palette"].is_array()) fail(file, where, "'palette' must be an array");
                for (const auto& pe : b["palette"]) {
                    if (!pe.is_object()) fail(file, where, "every palette entry must be an object");
                    PaletteEntry e;
                    if (!pe.contains("order") || !pe["order"].is_number_integer())
                        fail(file, where, "palette entry needs an integer 'order'");
                    e.order = pe["order"].get<int>();
                    if (orders.count(e.order))
                        fail(file, where, "palette order " + std::to_string(e.order) + " already used by " + orders[e.order]);
                    orders[e.order] = file + ":" + def.type;
                    e.text = reqString(pe, "text", file, where);
                    e.label = reqString(pe, "label", file, where, true);
                    e.prefix = reqString(pe, "prefix", file, where);
                    e.type = def.type;
                    e.uiTypeName = reqString(pe, "ui_type", file, where);
                    if (!componentTypeFromName(e.uiTypeName, e.uiType))
                        fail(file, where, "unknown ui_type '" + e.uiTypeName + "'");
                    e.category = category;
                    e.subcategory = pe.contains("subcategory") ? reqString(pe, "subcategory", file, where) : def.subcategory;
                    e.basic = pe.value("basic", false);
                    if (pe.contains("params")) {
                        if (!pe["params"].is_array()) fail(file, where, "palette 'params' must be an array of [name, value]");
                        for (const auto& kv : pe["params"]) {
                            if (!kv.is_array() || kv.size() != 2 || !kv[0].is_string() || !kv[1].is_string())
                                fail(file, where, "palette 'params' entries must be [\"name\", \"value\"]");
                            e.params.emplace_back(kv[0].get<std::string>(), kv[1].get<std::string>());
                        }
                    }
                    r.palette_.push_back(e);
                }
            }
            r.index_[def.type] = r.blocks_.size();
            r.blocks_.push_back(std::move(def));
        }
    }

    // Cross-file checks: aliases unique and distinct from types; pins_from targets.
    for (size_t i = 0; i < r.blocks_.size(); ++i) {
        const BlockDef& d = r.blocks_[i];
        for (const auto& a : d.aliases) {
            if (r.index_.count(a)) fail(d.sourceFile, "block " + d.type, "alias '" + a + "' is also a block type");
            auto [it, inserted] = r.aliasIndex_.emplace(a, i);
            if (!inserted)
                fail(d.sourceFile, "block " + d.type, "alias '" + a + "' already used by " + r.blocks_[it->second].type);
        }
    }
    // Case-insensitive lookup for netlist emission; must be unambiguous.
    for (size_t i = 0; i < r.blocks_.size(); ++i) {
        const BlockDef& d = r.blocks_[i];
        if (!d.netlist.present) continue;
        std::vector<std::string> names = d.aliases;
        names.push_back(d.type);
        for (const auto& n : names) {
            std::string up = n;
            std::transform(up.begin(), up.end(), up.begin(), [](unsigned char c) { return (char)std::toupper(c); });
            auto [it, inserted] = r.netlistUpperIndex_.emplace(up, i);
            if (!inserted && it->second != i)
                fail(d.sourceFile, "block " + d.type, "name '" + n + "' matches block " + r.blocks_[it->second].type +
                                                      " case-insensitively; both have a netlist spec");
        }
    }
    for (const auto& d : r.blocks_) {
        if (d.pinsFrom.empty()) continue;
        auto it = r.index_.find(d.pinsFrom);
        if (it == r.index_.end()) fail(d.sourceFile, "block " + d.type, "pins_from '" + d.pinsFrom + "' is not a block type");
        const BlockDef& t = r.blocks_[it->second];
        if (!t.pinsFrom.empty() || !t.declaredPins.empty())
            fail(d.sourceFile, "block " + d.type, "pins_from '" + d.pinsFrom + "' must have a built-in pin layout");
    }

    std::stable_sort(r.palette_.begin(), r.palette_.end(),
                     [](const PaletteEntry& a, const PaletteEntry& b) { return a.order < b.order; });
    for (size_t i = 0; i < r.palette_.size(); ++i)
        r.blocks_[r.index_.at(r.palette_[i].type)].paletteEntries.push_back((int)i);
    return r;
}

BlockRegistry BlockRegistry::fromDirectory(const std::string& dir) {
    namespace fs = std::filesystem;
    std::vector<std::pair<std::string, std::string>> files;
    std::error_code ec;
    if (!fs::is_directory(dir, ec)) throw std::runtime_error("block registry: not a directory: " + dir);
    std::vector<fs::path> paths;
    for (const auto& e : fs::directory_iterator(dir))
        if (e.is_regular_file() && e.path().extension() == ".json") paths.push_back(e.path());
    std::sort(paths.begin(), paths.end());
    for (const auto& p : paths) {
        std::ifstream f(p, std::ios::binary);
        std::stringstream ss;
        ss << f.rdbuf();
        files.emplace_back(p.filename().string(), ss.str());
    }
    if (files.empty()) throw std::runtime_error("block registry: no .json files in " + dir);
    return fromJsonTexts(files);
}

BlockRegistry BlockRegistry::fromEmbedded() {
    std::vector<std::pair<std::string, std::string>> files;
    for (unsigned long i = 0; i < simpel_embedded::kBlockFiles_count; ++i) {
        const auto& f = simpel_embedded::kBlockFiles[i];
        files.emplace_back(f.name, std::string(reinterpret_cast<const char*>(f.data), f.size));
    }
    return fromJsonTexts(files);
}

const BlockRegistry& BlockRegistry::instance() {
    static const BlockRegistry reg = []() {
        const char* dir = std::getenv("SIMPEL_BLOCKS_DIR");
        if (dir && *dir) return fromDirectory(dir);
        return fromEmbedded();
    }();
    return reg;
}

const BlockDef* BlockRegistry::find(const std::string& type) const {
    auto it = index_.find(type);
    return it == index_.end() ? nullptr : &blocks_[it->second];
}

const BlockDef* BlockRegistry::resolve(const std::string& name) const {
    if (const BlockDef* d = find(name)) return d;
    auto it = aliasIndex_.find(name);
    return it == aliasIndex_.end() ? nullptr : &blocks_[it->second];
}

const BlockDef* BlockRegistry::netlistBlockFor(const std::string& name) const {
    if (const BlockDef* d = resolve(name)) return d->netlist.present ? d : nullptr;
    std::string up = name;
    std::transform(up.begin(), up.end(), up.begin(), [](unsigned char c) { return (char)std::toupper(c); });
    auto it = netlistUpperIndex_.find(up);
    return it == netlistUpperIndex_.end() ? nullptr : &blocks_[it->second];
}

const char* blockRoleName(BlockRole r) {
    switch (r) {
        case BlockRole::Model: return "model";
        case BlockRole::Structural: return "structural";
        case BlockRole::Observer: return "observer";
        case BlockRole::Unsupported: return "unsupported";
    }
    return "model";
}

ComponentType stringToComponentType(const std::string& typeStr) {
    const BlockDef* d = BlockRegistry::instance().resolve(typeStr);
    return d ? d->uiType : ComponentType::Unknown;
}

std::vector<TerminalDef> registryTerminals(const ComponentInstance& comp) {
    const BlockDef* d = BlockRegistry::instance().resolve(comp.rawTypeStr);
    if (!d) return {};
    if (!d->pinsFrom.empty()) {
        ComponentInstance proxy = comp;
        proxy.rawTypeStr = d->pinsFrom;   // validated to have a built-in layout: no recursion
        return getTerminals(proxy);
    }
    std::vector<TerminalDef> out;
    for (const auto& p : d->declaredPins)
        out.emplace_back(p.name, p.relX, p.relY, p.dirX, p.dirY, p.isControl);
    return out;
}

std::vector<std::string> BlockRegistry::categories() const {
    std::vector<std::string> out;
    for (const auto& b : blocks_)
        if (std::find(out.begin(), out.end(), b.category) == out.end()) out.push_back(b.category);
    return out;
}

std::vector<std::string> BlockRegistry::suggest(const std::string& type, size_t maxResults) const {
    const std::string q = lower(type);
    std::vector<std::pair<size_t, std::string>> scored;
    for (const auto& b : blocks_) {
        const std::string t = lower(b.type), n = lower(b.name);
        size_t d = editDistance(q, t);
        if (!q.empty() && (t.find(q) != std::string::npos || n.find(q) != std::string::npos)) d = std::min<size_t>(d, 1);
        scored.emplace_back(d, b.type);
    }
    std::stable_sort(scored.begin(), scored.end(), [](const auto& a, const auto& b) { return a.first < b.first; });
    std::vector<std::string> out;
    const size_t limit = std::max<size_t>(2, q.size() / 2);
    for (const auto& [d, t] : scored) {
        if (out.size() >= maxResults || d > limit) break;
        out.push_back(t);
    }
    return out;
}

ComponentInstance BlockRegistry::makeInstance(const std::string& type, const std::string& id,
                                              const std::map<std::string, std::string>& overrides) const {
    const BlockDef* def = find(type);
    if (!def) {
        std::string msg = "unknown block type '" + type + "'";
        auto s = suggest(type);
        if (!s.empty()) {
            msg += "; did you mean";
            for (size_t i = 0; i < s.size(); ++i) msg += (i ? ", " : " ") + s[i];
            msg += "?";
        }
        throw std::runtime_error(msg);
    }
    ComponentInstance c;
    c.id = id;
    c.label = id;
    c.rawTypeStr = def->type;
    c.type = stringToComponentType(def->type);
    for (const auto& p : def->params) c.parameters[p.name] = p.defaultValue;
    for (const auto& [k, v] : overrides) c.parameters[k] = v;
    setupComponentPins(c);
    return c;
}

std::vector<BlockPin> BlockRegistry::pins(const std::string& type, const std::map<std::string, std::string>* params) const {
    return instancePins(makeInstance(type, "X1", params ? *params : std::map<std::string, std::string>{}));
}

std::vector<BlockPin> instancePins(const ComponentInstance& c) {
    std::vector<BlockPin> out;
    for (const auto& t : getTerminals(c)) {
        BlockPin p;
        p.name = t.name;
        p.relX = t.relX; p.relY = t.relY;
        p.dirX = t.dirX; p.dirY = t.dirY;
        p.isControl = t.isControl;
        out.push_back(p);
    }
    // Same fallback as SchematicCanvas::cachedTerminals: blocks without a terminal
    // layout use the generic pins from setupComponentPins.
    if (out.empty()) {
        for (const auto& pin : c.pins) {
            BlockPin p;
            p.name = pin.name;
            p.relX = pin.relativeX; p.relY = pin.relativeY;
            p.dirX = pin.isOutput ? 1.0f : -1.0f; p.dirY = 0.0f;
            p.isControl = pin.isCtrl || pin.isInput || pin.isOutput;
            out.push_back(p);
        }
    }
    return out;
}

} // namespace CircuitSim
