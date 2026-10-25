// A block added purely in data (no C++ change) is placed, wired, emitted into the
// netlist and simulated. The block reuses an existing solver model (OFFSET) through
// "original_type", which is the case registry-driven emission covers; a block with
// new behaviour additionally needs solver code (see docs/adding-a-block.md).
//
// Usage: test_custom_block <data/blocks dir>
#include "engine/BlockRegistry.hpp"
#include "engine/Runner.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <string>

namespace fs = std::filesystem;
using namespace CircuitSim;

static int g_pass = 0, g_fail = 0;
static void check(bool ok, const std::string& what) {
    if (ok) ++g_pass; else { ++g_fail; std::printf("FAIL: %s\n", what.c_str()); }
}

int main(int argc, char** argv) {
    if (argc < 2) { std::fprintf(stderr, "usage: test_custom_block <data/blocks dir>\n"); return 2; }
    const fs::path tmp = fs::temp_directory_path() / "simpel_custom_block_test";
    fs::remove_all(tmp);
    fs::create_directories(tmp);
    for (const auto& e : fs::directory_iterator(argv[1]))
        if (e.path().extension() == ".json") fs::copy_file(e.path(), tmp / e.path().filename());
    {
        std::ofstream o(tmp / "zz_custom.json", std::ios::binary);
        o << R"({"schema": 1, "category": "control", "description": "test-only block",
  "blocks": [{
    "type": "BIAS_ADD", "name": "Bias Adder", "subcategory": "Math", "ui_type": "Unknown",
    "params": [{"name": "bias", "default": "2.5"}],
    "pins": [{"name": "In",  "x": -30, "y": 0, "dx": -1, "dy": 0, "kind": "control"},
             {"name": "Out", "x":  30, "y": 0, "dx":  1, "dy": 0, "kind": "control"}],
    "netlist": {"section": "control_loops.gains",
                "fields": {"output": "{id}.Out", "original_type": "OFFSET",
                           "offset": {"param": "bias", "default": 0.0},
                           "input": {"signal": "In"}, "input1": {"signal": "In"},
                           "input2": "0.0", "K": 1.0}}
  }]})";
    }
#ifdef _WIN32
    _putenv_s("SIMPEL_BLOCKS_DIR", tmp.string().c_str());
#else
    setenv("SIMPEL_BLOCKS_DIR", tmp.string().c_str(), 1);
#endif
    const BlockRegistry& reg = BlockRegistry::instance();
    check(reg.find("BIAS_ADD") != nullptr, "custom block registered from SIMPEL_BLOCKS_DIR");
    check(reg.pins("BIAS_ADD").size() == 2, "custom block has its declared pins");

    CircuitDesign cd;
    cd.settings.stopTime = 1e-3;
    cd.settings.stepSize = 1e-5;
    cd.components.push_back(reg.makeInstance("CONST", "K1", {{"value", "4"}}));
    cd.components.push_back(reg.makeInstance("BIAS_ADD", "B1", {{"bias", "1.5"}}));
    WireInstance w;
    w.id = "w1";
    w.from.compId = "K1"; w.from.terminal = "Out";
    w.to.compId = "B1";   w.to.terminal = "In";
    cd.wires.push_back(w);

    RunResult r = runDesign(cd);
    check(r.netlistJson.find("\"B1\"") != std::string::npos, "custom block emitted into the netlist");
    check(r.netlistJson.find("\"K1.Out\"") != std::string::npos, "custom block input wired to K1.Out");
    auto it = r.signals.find("B1.Out");
    check(it != r.signals.end() && !it->second.empty(), "B1.Out simulated");
    if (it != r.signals.end() && !it->second.empty())
        check(std::fabs(it->second.back() - 5.5) < 1e-12, "B1.Out = 4 + 1.5 (got " + std::to_string(it->second.back()) + ")");

    fs::remove_all(tmp);
    std::printf("%d/%d checks passed\n", g_pass, g_pass + g_fail);
    return g_fail == 0 ? 0 : 1;
}
