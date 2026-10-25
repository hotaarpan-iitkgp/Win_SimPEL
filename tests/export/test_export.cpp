// Export snapshot: schematic SVG, scope SVG and full HTML report bytes for a set
// of corpus designs, hashed. Recorded from the exporter while it still lived in
// the UI library, so a match proves the move into the core (ImVec2 -> POD point,
// dialogs split off) changed no output byte. Also checks the CSV exporter.
//
// Usage: test_export <designs dir> <snapshot> [--update]
#ifdef SIMPEL_EXPORT_PRE_MOVE
#include "ui/SVGExporter.hpp"
#else
#include "engine/SVGExporter.hpp"
#include "engine/Exporters.hpp"
#endif
#include "engine/DesignIO.hpp"
#include "engine/Runner.hpp"

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <set>
#include <sstream>
#include <string>
#include <vector>

using namespace CircuitSim;
namespace fs = std::filesystem;

static uint64_t fnv1a(const std::string& s) {
    uint64_t h = 1469598103934665603ULL;
    for (unsigned char c : s) { h ^= c; h *= 1099511628211ULL; }
    return h;
}
static std::string hashLine(const std::string& name, const char* what, const std::string& bytes) {
    char b[64];
    std::snprintf(b, sizeof b, " %016llx %zu", (unsigned long long)fnv1a(bytes), bytes.size());
    return name + " " + what + b;
}
static std::string slurp(const fs::path& p) {
    std::ifstream f(p, std::ios::binary);
    std::stringstream ss;
    ss << f.rdbuf();
    return ss.str();
}

static const char* kDesigns[] = {
    "Buck_converter.json", "boost closed loop.json", "fixed_thyristor_bridge.json",
    "Electrical_Semiconductors_Test.json", "Electrical_Complex_Transformers_Machines_Test.json",
    "Math_Blocks_Test.json", "three_phase_inverter_basic_spwm_fixed.json", "test01_buck_pid.json",
    "test04_isolated_flyback.json", "functionstables.json",
};

int main(int argc, char** argv) {
    if (argc < 3) { std::fprintf(stderr, "usage: test_export <designs> <snapshot> [--update]\n"); return 2; }
    const fs::path dir = argv[1];
    const std::string snap = argv[2];
    const bool update = argc > 3 && std::string(argv[3]) == "--update";
    const fs::path tmp = fs::temp_directory_path() / "simpel_export_test";
    fs::create_directories(tmp);

    std::vector<std::string> got;
    int fails = 0;
    for (const char* name : kDesigns) {
        CircuitDesign design = DesignIO::loadDesignFile((dir / name).string());
        RunOverrides ov;
        ov.tStop = 2e-3;
        RunResult r = runDesign(design, ov);
        CircuitSimEngine::TelemetryData tel;
        tel.timeHistory = r.time;
        tel.voltages = r.signals;

        std::string svg;
        SVGExporter::exportSchematicToSVGString(r.design, svg, false);
        got.push_back(hashLine(name, "schematic", svg));

        std::vector<std::string> keys = r.signalNames();
        if (keys.size() > 4) keys.resize(4);
        std::string scope;
        SVGExporter::exportScopeToSVGString(tel, keys, keys, "Scope", scope, (int)keys.size(), false);
        got.push_back(hashLine(name, "scope", scope));

        std::vector<ScopeReportData> scopes;
        for (const auto& c : r.design.components) {
            if (c.rawTypeStr != "SCOPE") continue;
            int n = 2;
            if (c.parameters.count("channels")) try { n = std::stoi(c.parameters.at("channels")); } catch (...) {}
            ScopeReportData s;
            s.scopeId = c.id;
            s.scopeTitle = c.id;
            for (const auto& k : SVGExporter::traceScopeInputSignals(r.design, c.id, n))
                if (!k.empty()) { s.signalKeys.push_back(k); s.signalLabels.push_back(k); }
            s.numPanes = (int)s.signalKeys.size();
            if (!s.signalKeys.empty()) scopes.push_back(s);
        }
        if (scopes.empty()) {
            ScopeReportData s;
            s.scopeId = "Signals"; s.scopeTitle = "Signals";
            s.signalKeys = keys; s.signalLabels = keys; s.numPanes = (int)keys.size();
            scopes.push_back(s);
        }
        got.push_back(hashLine(name, "scopes", [&] {
            std::string all;
            for (const auto& s : scopes) for (const auto& k : s.signalKeys) all += k + ";";
            return all;
        }()));
        const fs::path html = tmp / "report.html";
        SVGExporter::exportFullReportToHTML(r.design, tel, scopes, DesignIO::saveDesignString(r.design),
                                            r.netlistJson, html.string(), false);
        got.push_back(hashLine(name, "html", slurp(html)));
    }

#ifndef SIMPEL_EXPORT_PRE_MOVE
    // CSV: header, column order, precision, explicit selection, and errors.
    {
        std::vector<double> t = {0.0, 1e-6, 2.5e-6};
        std::unordered_map<std::string, std::vector<double>> sig = {
            {"V_C1", {1.0, 2.123456789, -3.0}}, {"I_L1", {0.5, 0.25, 1e-9}}, {"node_3", {9, 9, 9}}};
        const fs::path csv = tmp / "out.csv";
        std::string err;
        bool ok = exportCSV(t, sig, {}, csv.string(), 8, &err);
        const std::string all = slurp(csv);
        const std::string want = "time,I_L1,V_C1,node_3\n0,0.5,1,9\n1e-06,0.25,2.1234568,9\n2.5e-06,1e-09,-3,9\n";
        if (!ok || all != want) { ++fails; std::printf("FAIL csv default:\n%s(err %s)\n", all.c_str(), err.c_str()); }
        ok = exportCSV(t, sig, {"V_C1", "node_3"}, csv.string(), 3, &err);
        if (!ok || slurp(csv) != "time,V_C1,node_3\n0,1,9\n1e-06,2.12,9\n2.5e-06,-3,9\n") { ++fails; std::printf("FAIL csv selection\n"); }
        if (exportCSV(t, sig, {"NOPE"}, csv.string(), 8, &err) || err.find("NOPE") == std::string::npos) {
            ++fails; std::printf("FAIL csv unknown signal\n");
        }
        std::vector<double> dt, dy;
        std::vector<double> rt(10000), ry(10000);
        for (size_t i = 0; i < rt.size(); ++i) { rt[i] = (double)i; ry[i] = (i == 4321) ? 100.0 : 0.0; }
        decimateMinMax(rt.data(), ry.data(), rt.size(), 200, dt, dy);
        if (dt.size() > 200 || std::find(dy.begin(), dy.end(), 100.0) == dy.end()) {
            ++fails; std::printf("FAIL decimation keeps the spike within 200 points\n");
        }
    }
#endif
    fs::remove_all(tmp);

    if (update) {
        std::ofstream o(snap, std::ios::binary);
        o << "# Export snapshot (schematic SVG, scope SVG, scope selection, HTML report).\n";
        for (const auto& l : got) o << l << "\n";
        std::printf("wrote %zu lines\n", got.size());
        return 0;
    }
    std::vector<std::string> want;
    {
        std::ifstream f(snap);
        std::string l;
        while (std::getline(f, l)) {
            if (!l.empty() && l.back() == '\r') l.pop_back();
            if (!l.empty() && l[0] != '#') want.push_back(l);
        }
    }
    std::set<std::string> ws(want.begin(), want.end());
    for (const auto& l : got) if (!ws.count(l)) { ++fails; std::printf("  differs: %s\n", l.c_str()); }
    if (want.size() != got.size()) { ++fails; std::printf("  line count %zu vs %zu\n", got.size(), want.size()); }
    std::printf("export: %zu outputs, %d failures\n", got.size(), fails);
    return fails == 0 ? 0 : 1;
}
