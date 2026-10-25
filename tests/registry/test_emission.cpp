// Per-block netlist emission snapshot.
//
// For every registered block the netlist of three small designs is hashed:
//   default  a lone instance with the registry defaults
//   params   every parameter (except C-Script code) set to a distinct sentinel
//   wired    every control pin driven by its own Constant block
// and, for simulated ("model") blocks, every parameter whose sentinel value does
// not appear anywhere in the netlist is listed as "unmapped": the generator never
// passes it to the solver (or transforms it, e.g. frequency -> period).
//
// The snapshot was recorded from the hand-written generator before registry-driven
// emission existed, so a match proves the generic path emits byte-identical netlists
// for every block it took over, with default, non-default and wired inputs.
//
// Usage: test_emission <snapshot file> [--update]
#include "engine/BlockRegistry.hpp"
#include "engine/NetlistGenerator.hpp"

#include <cstdint>
#include <cstdio>
#include <fstream>
#include <map>
#include <set>
#include <string>
#include <vector>

using namespace CircuitSim;

static uint64_t fnv1a(const std::string& s) {
    uint64_t h = 1469598103934665603ULL;
    for (unsigned char c : s) { h ^= c; h *= 1099511628211ULL; }
    return h;
}

static std::string line(const std::string& type, const char* variant, const std::string& netlist) {
    char b[64];
    std::snprintf(b, sizeof b, " %016llx %zu", (unsigned long long)fnv1a(netlist), netlist.size());
    return type + " " + variant + b;
}

static std::string sentinel(size_t i) {
    char b[32];
    std::snprintf(b, sizeof b, "%.1f", 4321.5 + (double)i);
    return b;
}

static std::string emit(const CircuitDesign& cd) {
    try {
        return generateNetlistJson(cd);
    } catch (const std::exception& e) {
        return std::string("error: ") + e.what();
    }
}

int main(int argc, char** argv) {
    if (argc < 2) { std::fprintf(stderr, "usage: test_emission <snapshot> [--update]\n"); return 2; }
    const std::string snapPath = argv[1];
    const bool update = argc > 2 && std::string(argv[2]) == "--update";
    const BlockRegistry& reg = BlockRegistry::instance();

    std::vector<std::string> got;
    for (const auto& b : reg.blocks()) {
        // default
        {
            CircuitDesign cd;
            cd.components.push_back(reg.makeInstance(b.type, "XQ1"));
            got.push_back(line(b.type, "default", emit(cd)));
        }
        // params
        {
            std::map<std::string, std::string> ov;
            std::vector<std::pair<std::string, std::string>> sent;
            size_t i = 0;
            for (const auto& p : b.params) {
                if (p.name == "code") continue;
                std::string s = sentinel(i++);
                ov[p.name] = s;
                sent.emplace_back(p.name, s);
            }
            CircuitDesign cd;
            cd.components.push_back(reg.makeInstance(b.type, "XQ1", ov));
            const std::string nl = emit(cd);
            got.push_back(line(b.type, "params", nl));
            if (b.role == BlockRole::Model)
                for (const auto& [name, s] : sent)
                    if (nl.find(s) == std::string::npos) got.push_back("unmapped " + b.type + " " + name);
        }
        // wired
        {
            CircuitDesign cd;
            cd.components.push_back(reg.makeInstance(b.type, "XQ1"));
            int k = 0;
            for (const auto& pin : reg.pins(b.type)) {
                if (!pin.isControl) continue;
                const std::string src = "KSRC" + std::to_string(++k);
                cd.components.push_back(reg.makeInstance("CONST", src, {{"value", std::to_string(k) + ".25"}}));
                WireInstance w;
                w.id = "w" + std::to_string(k);
                w.from.compId = src;
                w.from.terminal = "Out";
                w.to.compId = "XQ1";
                w.to.terminal = pin.name;
                cd.wires.push_back(w);
            }
            got.push_back(line(b.type, "wired", emit(cd)));
        }
    }

    if (update) {
        std::ofstream o(snapPath, std::ios::binary);
        o << "# Per-block netlist emission snapshot. Regenerate with test_emission <file> --update\n";
        o << "# only for an intended change to netlist emission.\n";
        for (const auto& l : got) o << l << "\n";
        std::printf("wrote %zu lines to %s\n", got.size(), snapPath.c_str());
        return 0;
    }

    std::vector<std::string> want;
    {
        std::ifstream f(snapPath);
        std::string l;
        while (std::getline(f, l)) {
            if (!l.empty() && l.back() == '\r') l.pop_back();
            if (!l.empty() && l[0] != '#') want.push_back(l);
        }
    }
    if (want.empty()) { std::printf("snapshot %s missing or empty\n", snapPath.c_str()); return 1; }
    std::set<std::string> ws(want.begin(), want.end()), gs(got.begin(), got.end());
    int diffs = 0;
    for (const auto& l : want) if (!gs.count(l) && diffs++ < 20) std::printf("  missing: %s\n", l.c_str());
    for (const auto& l : got) if (!ws.count(l) && diffs++ < 40) std::printf("  new:     %s\n", l.c_str());
    size_t unmapped = 0;
    for (const auto& l : got) unmapped += l.rfind("unmapped ", 0) == 0;
    std::printf("emission: %zu lines, %zu unmapped parameters, %d differences\n", got.size(), unmapped, diffs);
    return diffs == 0 ? 0 : 1;
}
