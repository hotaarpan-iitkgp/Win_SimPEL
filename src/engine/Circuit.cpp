#include "Circuit.hpp"

#include "NetlistGenerator.hpp"

#include <algorithm>
#include <cctype>
#include <functional>
#include <set>
#include <stdexcept>
#include <unordered_map>

namespace CircuitSim {

namespace {

std::string upper(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return (char)std::toupper(c); });
    return s;
}

bool validId(const std::string& id) {
    if (id.empty() || id.size() > 64) return false;
    if (!(std::isalpha((unsigned char)id[0]) || id[0] == '_')) return false;
    for (char ch : id)
        if (!(std::isalnum((unsigned char)ch) || ch == '_' || ch == '-')) return false;
    return true;
}

// Parameters used outside the solver netlist (GUI state, loss analysis).
const std::set<std::string> kNonNetlistParams = {
    "thermal_model", "plotV", "plotI", "probe_signal", "selected_signals", "target", "probe_type",
};

struct DSU {
    std::unordered_map<std::string, std::string> parent;
    std::string find(const std::string& x) {
        auto it = parent.find(x);
        if (it == parent.end()) { parent[x] = x; return x; }
        if (it->second == x) return x;
        std::string r = find(it->second);
        parent[x] = r;
        return r;
    }
    void unite(const std::string& a, const std::string& b) {
        std::string ra = find(a), rb = find(b);
        if (ra != rb) parent[ra] = rb;
    }
};

std::string splitComp(const std::string& key) { return key.substr(0, key.find(':')); }
std::string splitPin(const std::string& key) { return key.substr(key.find(':') + 1); }

} // namespace

Circuit::Circuit(std::string name) : name_(std::move(name)) {}

Circuit Circuit::fromDesign(const CircuitDesign& design, std::string name) {
    Circuit c(std::move(name));
    c.comps_ = design.components;
    c.baseWires_ = design.wires;
    c.settings_ = design.settings;
    c.plotConfig_ = design.plotConfig;
    for (const auto& comp : c.comps_)
        if (upper(comp.rawTypeStr) == "GND" && c.groundId_.empty()) c.groundId_ = comp.id;
    return c;
}

ComponentInstance* Circuit::find(const std::string& id) {
    for (auto& c : comps_) if (c.id == id) return &c;
    return nullptr;
}
const ComponentInstance* Circuit::find(const std::string& id) const {
    for (const auto& c : comps_) if (c.id == id) return &c;
    return nullptr;
}

bool Circuit::has(const std::string& id) const { return find(id) != nullptr; }

const ComponentInstance& Circuit::component(const std::string& id) const {
    const ComponentInstance* c = find(id);
    if (!c) throw std::out_of_range("no component '" + id + "' in circuit '" + name_ + "'");
    return *c;
}

std::vector<std::string> Circuit::componentIds() const {
    std::vector<std::string> out;
    for (const auto& c : comps_) out.push_back(c.id);
    return out;
}

void Circuit::placeNew(ComponentInstance& c) const {
    // Placeholder grid so a saved design opens with every block visible; real
    // placement is done by the layout step.
    const size_t i = comps_.size();
    c.x = 160.0f * (float)(i % 6);
    c.y = 160.0f * (float)(i / 6);
    c.rotation = 0;
}

std::string Circuit::add(const std::string& type, const std::string& idIn,
                         const std::map<std::string, std::string>& params) {
    const BlockRegistry& reg = BlockRegistry::instance();
    const BlockDef* def = reg.resolve(type);
    if (!def) {
        std::string msg = "unknown block type '" + type + "'";
        auto s = reg.suggest(type);
        if (!s.empty()) {
            msg += "; did you mean";
            for (size_t i = 0; i < s.size(); ++i) msg += (i ? ", " : " ") + s[i];
            msg += "?";
        }
        throw std::invalid_argument(msg);
    }
    std::string id = idIn;
    if (id.empty()) {
        std::string prefix;
        if (!def->paletteEntries.empty()) prefix = reg.palette()[def->paletteEntries[0]].prefix;
        if (prefix.empty()) prefix = def->type;
        std::string clean;
        for (char ch : prefix) if (std::isalnum((unsigned char)ch) || ch == '_') clean += ch;
        if (clean.empty() || !std::isalpha((unsigned char)clean[0])) clean = "X" + clean;
        for (int n = 1;; ++n) {
            id = clean + std::to_string(n);
            if (!has(id)) break;
        }
    }
    if (!validId(id))
        throw std::invalid_argument("invalid component id '" + id +
                                    "': use letters, digits, '_' or '-', starting with a letter");
    if (has(id)) throw std::invalid_argument("component id '" + id + "' is already used");

    ComponentInstance c = reg.makeInstance(def->type, id, params);
    placeNew(c);
    comps_.push_back(std::move(c));
    return id;
}

void Circuit::remove(const std::string& id) {
    auto it = std::find_if(comps_.begin(), comps_.end(), [&](const ComponentInstance& c) { return c.id == id; });
    if (it == comps_.end()) throw std::out_of_range("no component '" + id + "'");
    comps_.erase(it);
    links_.erase(std::remove_if(links_.begin(), links_.end(), [&](const auto& l) {
        return splitComp(l.first) == id || splitComp(l.second) == id;
    }), links_.end());
    baseWires_.erase(std::remove_if(baseWires_.begin(), baseWires_.end(), [&](const WireInstance& w) {
        return (!w.from.isWireJunction && w.from.compId == id) || (!w.to.isWireJunction && w.to.compId == id);
    }), baseWires_.end());
    if (groundId_ == id) groundId_.clear();
}

void Circuit::setParam(const std::string& id, const std::string& name, const std::string& value) {
    ComponentInstance* c = find(id);
    if (!c) throw std::out_of_range("no component '" + id + "'");
    if (name.empty()) throw std::invalid_argument("parameter name must not be empty");
    c->parameters[name] = value;
    // Pins of some blocks depend on parameters (C-Script ports, sum signs, windings).
    setupComponentPins(*c);
}

std::vector<BlockPin> Circuit::pins(const std::string& id) const {
    return instancePins(component(id));
}

std::string Circuit::resolvePin(const std::string& id, const std::string& pin) const {
    const auto ps = pins(id);
    for (const auto& p : ps) if (p.name == pin) return p.name;
    const std::string up = upper(pin);
    std::string found;
    for (const auto& p : ps) {
        if (upper(p.name) == up) {
            if (!found.empty() && found != p.name)
                throw std::invalid_argument("pin '" + pin + "' of " + id + " is ambiguous");
            found = p.name;
        }
    }
    if (!found.empty()) return found;
    std::string list;
    for (const auto& p : ps) list += (list.empty() ? "" : ", ") + p.name;
    throw std::invalid_argument(id + " (" + component(id).rawTypeStr + ") has no pin '" + pin + "'; pins: " +
                                (list.empty() ? "(none)" : list));
}

std::string Circuit::pinKey(const PinRef& p) const {
    if (!has(p.component)) throw std::invalid_argument("no component '" + p.component + "' in circuit '" + name_ + "'");
    return p.component + ":" + resolvePin(p.component, p.pin);
}

void Circuit::connect(const std::vector<PinRef>& pins) {
    if (pins.size() < 2) throw std::invalid_argument("connect needs at least two pins");
    std::vector<std::string> keys;
    for (const auto& p : pins) keys.push_back(pinKey(p));   // validate all before changing anything
    for (size_t i = 1; i < keys.size(); ++i)
        if (keys[i] != keys[0]) links_.emplace_back(keys[0], keys[i]);
}

void Circuit::ground(const std::vector<PinRef>& pins) {
    if (pins.empty()) throw std::invalid_argument("ground needs at least one pin");
    for (const auto& p : pins) pinKey(p);
    if (groundId_.empty() || !has(groundId_)) groundId_ = add("GND", has("GND") ? "" : "GND");
    std::vector<PinRef> all{{groundId_, "Gnd"}};
    all.insert(all.end(), pins.begin(), pins.end());
    connect(all);
}

bool Circuit::paramAffectsNetlist(const std::string& id, const std::string& name) const {
    const ComponentInstance& comp = component(id);
    const BlockDef* def = BlockRegistry::instance().resolve(comp.rawTypeStr);
    if (!def || def->role != BlockRole::Model) return true;   // nothing to compare against
    if (def->findParam(name) || kNonNetlistParams.count(name)) return true;
    auto netlistWith = [&](const std::string& value) {
        CircuitDesign d;
        ComponentInstance c = comp;
        c.parameters[name] = value;
        setupComponentPins(c);
        d.components.push_back(c);
        return generateNetlistJson(d);
    };
    return netlistWith("4321.5") != netlistWith("8642.25");
}

std::vector<std::string> Circuit::ineffectiveParams(const std::string& id) const {
    const ComponentInstance& comp = component(id);
    std::vector<std::string> out;
    for (const auto& kv : comp.parameters)
        if (!paramAffectsNetlist(id, kv.first)) out.push_back(kv.first);
    std::sort(out.begin(), out.end());
    return out;
}

std::vector<ValidationIssue> Circuit::validate() const {
    using S = ValidationIssue::Severity;
    std::vector<ValidationIssue> issues;
    auto add = [&](S s, const char* code, const std::string& comp, const std::string& pin, const std::string& msg) {
        issues.push_back({s, code, comp, pin, msg});
    };
    const BlockRegistry& reg = BlockRegistry::instance();
    if (comps_.empty()) {
        add(S::Error, "empty_circuit", "", "", "the circuit has no components");
        return issues;
    }

    // Connectivity: generated links plus wires of a loaded design (junction
    // endpoints join the net of the wire they land on).
    DSU dsu;
    std::unordered_map<std::string, int> degree;   // pin key -> number of connections
    for (const auto& [a, b] : links_) { dsu.unite(a, b); ++degree[a]; ++degree[b]; }
    auto canonical = [&](const std::string& compId, const std::string& term) {
        const ComponentInstance* c = find(compId);
        if (!c) return compId + ":" + term;
        for (const auto& p : instancePins(*c)) if (upper(p.name) == upper(term)) return compId + ":" + p.name;
        return compId + ":" + term;
    };
    for (const auto& w : baseWires_) {
        const std::string wn = "wire:" + w.id;
        for (const WireEndpoint* ep : {&w.from, &w.to}) {
            if (ep->isWireJunction) {
                if (!ep->targetWireId.empty()) dsu.unite(wn, "wire:" + ep->targetWireId);
            } else if (!ep->compId.empty()) {
                const std::string k = canonical(ep->compId, ep->terminal);
                dsu.unite(wn, k);
                ++degree[k];
            }
        }
    }

    bool anyPower = false, hasGround = false;
    std::unordered_map<std::string, std::vector<std::string>> nets;   // root -> pin keys
    struct PinInfo { bool control; bool output; };
    std::unordered_map<std::string, PinInfo> pinInfo;
    for (const auto& c : comps_) {
        const BlockDef* def = reg.resolve(c.rawTypeStr);
        if (!def) {
            add(S::Warning, "unknown_type", c.id, "", c.id + ": block type '" + c.rawTypeStr + "' is not in the registry");
            continue;
        }
        if (upper(def->type) == "GND") hasGround = true;
        if (def->role == BlockRole::Unsupported)
            add(S::Warning, "unsupported_block", c.id, "",
                c.id + " (" + def->type + ") has no solver model yet and is left out of the simulation");
        for (const auto& p : ineffectiveParams(c.id)) {
            std::string msg = c.id + ": parameter '" + p + "' has no effect on the simulation";
            std::string best;
            size_t bestScore = 0;
            for (const auto& bp : def->params) {   // cheap suggestion: shared case-insensitive prefix
                size_t n = 0;
                const std::string a = upper(p), b = upper(bp.name);
                while (n < a.size() && n < b.size() && a[n] == b[n]) ++n;
                if (a == b) n = 1000;
                if (n > bestScore) { bestScore = n; best = bp.name; }
            }
            if (bestScore > 0) msg += "; did you mean '" + best + "'?";
            add(S::Warning, "ineffective_param", c.id, p, msg);
        }
        const bool observer = def->role == BlockRole::Observer;
        for (const auto& p : instancePins(c)) {
            const std::string k = c.id + ":" + p.name;
            pinInfo[k] = {p.isControl, p.isControl && p.dirX > 0.0f};
            if (!p.isControl) anyPower = true;
            if (degree[k] == 0) {
                if (observer || upper(def->type) == "GND") continue;
                if (!p.isControl)
                    add(S::Warning, "unconnected_pin", c.id, p.name, c.id + "." + p.name + " is not connected");
                else if (p.dirX <= 0.0f)
                    add(S::Warning, "unconnected_input", c.id, p.name,
                        c.id + "." + p.name + " is not driven; it reads 0");
                continue;
            }
            nets[dsu.find(k)].push_back(k);
        }
    }
    for (const auto& [root, members] : nets) {
        bool power = false;
        std::vector<std::string> drivers;
        for (const auto& k : members) {
            const PinInfo& pi = pinInfo[k];
            if (!pi.control) power = true;
            else if (pi.output) drivers.push_back(k);
        }
        if (power || members.size() < 2) continue;
        if (drivers.size() > 1) {
            std::string list;
            for (const auto& d : drivers) list += (list.empty() ? "" : ", ") + splitComp(d) + "." + splitPin(d);
            add(S::Error, "multiple_drivers", splitComp(drivers[0]), splitPin(drivers[0]),
                "control outputs " + list + " are connected together");
        } else if (drivers.empty()) {
            std::string list;
            for (const auto& m : members) list += (list.empty() ? "" : ", ") + splitComp(m) + "." + splitPin(m);
            add(S::Warning, "no_driver", splitComp(members[0]), splitPin(members[0]),
                "signal net " + list + " has no output driving it");
        }
    }
    if (anyPower && !hasGround)
        add(S::Warning, "no_ground", "", "",
            "no ground: the negative terminal of the first voltage source is used as the reference");
    return issues;
}

CircuitDesign Circuit::design() const {
    CircuitDesign d;
    d.components = comps_;
    d.wires = baseWires_;
    d.settings = settings_;
    d.plotConfig = plotConfig_;

    // Nets of the programmatic connections, in order of first appearance.
    DSU dsu;
    std::vector<std::string> order;
    std::set<std::string> seen;
    for (const auto& [a, b] : links_) {
        dsu.unite(a, b);
        for (const auto* k : {&a, &b}) if (seen.insert(*k).second) order.push_back(*k);
    }
    std::vector<std::string> roots;
    std::unordered_map<std::string, std::vector<std::string>> nets;
    for (const auto& k : order) {
        const std::string r = dsu.find(k);
        if (!nets.count(r)) roots.push_back(r);
        nets[r].push_back(k);
    }

    std::set<std::string> usedWireIds;
    for (const auto& w : d.wires) usedWireIds.insert(w.id);
    int wn = 0;
    auto nextWireId = [&]() {
        std::string id;
        do { id = "w_api_" + std::to_string(++wn); } while (usedWireIds.count(id));
        usedWireIds.insert(id);
        return id;
    };
    for (const auto& r : roots) {
        const auto& members = nets[r];
        // Hub: the single control driver of a pure signal net, else the first pin.
        size_t hub = 0;
        bool power = false;
        std::vector<size_t> drivers;
        for (size_t i = 0; i < members.size(); ++i) {
            const ComponentInstance* c = find(splitComp(members[i]));
            if (!c) continue;
            for (const auto& p : instancePins(*c)) {
                if (p.name != splitPin(members[i])) continue;
                if (!p.isControl) power = true;
                else if (p.dirX > 0.0f) drivers.push_back(i);
            }
        }
        if (!power && drivers.size() == 1) hub = drivers[0];
        for (size_t i = 0; i < members.size(); ++i) {
            if (i == hub) continue;
            WireInstance w;
            w.id = nextWireId();
            w.from.compId = splitComp(members[hub]);
            w.from.terminal = splitPin(members[hub]);
            w.to.compId = splitComp(members[i]);
            w.to.terminal = splitPin(members[i]);
            d.wires.push_back(w);
        }
    }
    return d;
}

} // namespace CircuitSim
