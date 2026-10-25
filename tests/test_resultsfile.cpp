// .simres: bitwise round trip of a real run, validation of damaged files.
// Usage: test_resultsfile <tests/golden/designs>
#include "engine/DesignIO.hpp"
#include "engine/ResultsFile.hpp"
#include "engine/Runner.hpp"

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <map>
#include <string>

using namespace CircuitSim;
namespace fs = std::filesystem;

static int g_pass = 0, g_fail = 0;
static void check(bool ok, const std::string& what) {
    if (ok) ++g_pass; else { ++g_fail; std::printf("FAIL: %s\n", what.c_str()); }
}
static bool throwsWith(const std::string& path, const std::string& needle) {
    try { readResultsFile(path); } catch (const std::exception& e) {
        if (std::string(e.what()).find(needle) != std::string::npos) return true;
        std::printf("  (message: %s)\n", e.what());
    }
    return false;
}

int main(int argc, char** argv) {
    const fs::path designs = argc > 1 ? argv[1] : "tests/golden/designs";
    const fs::path tmp = fs::temp_directory_path() / "simpel_simres_test";
    fs::create_directories(tmp);
    const std::string file = (tmp / "r.simres").string();

    RunOverrides ov;
    ov.tStop = 2e-3;
    RunResult r = runDesignFile((designs / "Buck_converter.json").string(), ov);
    std::vector<ResultsSignalView> views;
    for (const auto& n : r.signalNames()) views.push_back({n, r.signals.at(n).data(), r.signals.at(n).size()});
    const nlohmann::json settings = {{"t_stop", r.stopTime}, {"dt", r.stepSize}};
    const nlohmann::json meta = {{"wall_seconds", r.wallSeconds}, {"cancelled", r.cancelled}};
    writeResultsFile(file, DesignIO::saveDesign(r.design), r.netlistJson, settings, meta, r.time.data(), r.time.size(), views);

    check(looksLikeResultsFile(file), "magic recognised");
    ResultsFileData d = readResultsFile(file);
    check(d.time == r.time, "time bitwise identical");
    bool all = d.signals.size() == r.signals.size();
    for (const auto& [n, v] : d.signals) all &= r.signals.count(n) && r.signals.at(n) == v;
    check(all, "every signal bitwise identical");
    check(d.netlist == r.netlistJson, "netlist text identical");
    check(d.settings == settings && d.meta == meta, "settings and meta survive");
    CircuitDesign back = DesignIO::loadDesign(d.design);
    check(back.components.size() == r.design.components.size(), "design survives");
    check(!fs::exists(file + ".partial"), "no partial file left behind");

    // Damage.
    const std::string bytes = [&] { std::ifstream f(file, std::ios::binary); return std::string(std::istreambuf_iterator<char>(f), {}); }();
    auto writeBytes = [&](const std::string& name, const std::string& b) {
        const std::string p = (tmp / name).string();
        std::ofstream(p, std::ios::binary) << b;
        return p;
    };
    check(throwsWith(writeBytes("trunc.simres", bytes.substr(0, bytes.size() - 8)), "data size"), "truncated file rejected");
    check(throwsWith(writeBytes("long.simres", bytes + "x"), "data size"), "trailing bytes rejected");
    std::string wrongMagic = bytes; wrongMagic[0] = 'X';
    check(throwsWith(writeBytes("magic.simres", wrongMagic), "wrong magic"), "wrong magic rejected");
    std::string future = bytes; future[8] = 9;
    check(throwsWith(writeBytes("v9.simres", future), "version 9"), "future version rejected with its number");
    check(throwsWith(writeBytes("tiny.simres", "SIMPEL"), "too short"), "tiny file rejected");
    check(throwsWith((tmp / "missing.simres").string(), "cannot open"), "missing file");
    check(!looksLikeResultsFile(writeBytes("x.json", "{}")), "json is not a results file");
    bool dup = false;
    try {
        std::vector<ResultsSignalView> v2 = {{"a", r.time.data(), 1}, {"a", r.time.data(), 1}};
        writeResultsFile((tmp / "dup.simres").string(), nlohmann::json::object(), "", {}, {}, r.time.data(), 1, v2);
    } catch (const std::exception&) { dup = true; }
    check(dup, "duplicate signal names rejected on write");

    fs::remove_all(tmp);
    std::printf("%d/%d checks passed\n", g_pass, g_pass + g_fail);
    return g_fail == 0 ? 0 : 1;
}
