// simpel_cli: headless command-line front end linked against simpel_core only.
//
//   simpel_cli version
//   simpel_cli run <design.json> [options]      simulate and print a signal summary
//   simpel_cli netlist <design.json> [-o file]  print / write the solver netlist
//   simpel_cli signals <design.json>            simulate and list signal names
//
// run options:
//   -t, --tstop <s>        override stop time
//   -dt, --step <s>        override step size
//   --step-type <fixed|variable>
//   --solver <euler|trapezoidal|...>
//   -p, --param Comp.param=value   (repeatable)
//   --netlist <file>       also write the emitted netlist
//   --csv <file>           write the waveforms as CSV (all signals, or --signals)
//   --signals a,b,...      signals for --csv / --svg
//   --svg <file>           write the selected signals (default: first 4) as an SVG plot
//   --html <file>          write the HTML report (schematic, scopes, netlist)
//   --schematic <file>     write the schematic as SVG
//   --simres <file>        write the results for the GUI (File > Open Results)
//
//   simpel_cli info <results.simres>   summary of a results file
//
// Exit codes: 0 success, 1 simulation/design error, 2 usage error.
#include "engine/DesignIO.hpp"
#include "engine/Exporters.hpp"
#include "engine/ResultsFile.hpp"
#include "engine/SVGExporter.hpp"
#include "engine/NetlistBuilder.hpp"
#include "engine/NetlistGenerator.hpp"
#include "engine/Runner.hpp"
#include "engine/Version.hpp"

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

using namespace CircuitSim;

static void usage() {
    std::fprintf(stderr,
        "SimPEL %s headless CLI\n"
        "usage: simpel_cli version\n"
        "       simpel_cli run <design.json> [-t s] [-dt s] [--step-type fixed|variable]\n"
        "                                    [--solver name] [-p Comp.param=value]... [--netlist file]\n"
        "                                    [--csv file] [--signals a,b] [--svg file] [--html file]\n"
        "                                    [--schematic file] [--simres file]\n"
        "       simpel_cli info <results.simres>\n"
        "       simpel_cli netlist <design.json> [-o file]\n"
        "       simpel_cli signals <design.json>\n",
        simpelVersion());
}

static double parseSeconds(const std::string& s, const char* what) {
    try {
        size_t n = 0;
        double v = std::stod(s, &n);
        if (n == s.size() && v > 0.0) return v;
    } catch (...) {}
    std::fprintf(stderr, "invalid %s: '%s'\n", what, s.c_str());
    std::exit(2);
}

static bool writeFile(const std::string& path, const std::string& text) {
    std::ofstream o(path, std::ios::binary);
    if (!o) return false;
    o << text;
    return (bool)o;
}

int main(int argc, char** argv) {
    if (argc < 2) { usage(); return 2; }
    const std::string cmd = argv[1];
    if (cmd == "version" || cmd == "--version") { std::printf("%s\n", simpelVersion()); return 0; }
    if (cmd == "-h" || cmd == "--help" || cmd == "help") { usage(); return 0; }
    if (argc < 3) { usage(); return 2; }
    const std::string design = argv[2];

    RunOverrides ov;
    std::string netlistOut, outFile, csvOut, svgOut, htmlOut, schematicOut, signalList, simresOut;
    for (int i = 3; i < argc; ++i) {
        std::string a = argv[i];
        auto next = [&](const char* what) -> std::string {
            if (i + 1 >= argc) { std::fprintf(stderr, "missing value for %s\n", what); std::exit(2); }
            return argv[++i];
        };
        if (a == "-t" || a == "--tstop") ov.tStop = parseSeconds(next("--tstop"), "stop time");
        else if (a == "-dt" || a == "--step") ov.dt = parseSeconds(next("--step"), "step size");
        else if (a == "--step-type") ov.stepType = next("--step-type");
        else if (a == "--solver") ov.solver = next("--solver");
        else if (a == "-p" || a == "--param") {
            std::string spec = next("--param");
            size_t eq = spec.find('=');
            if (eq == std::string::npos) { std::fprintf(stderr, "--param expects Comp.param=value\n"); return 2; }
            ov.params.push_back({spec.substr(0, eq), spec.substr(eq + 1)});
        }
        else if (a == "--netlist") netlistOut = next("--netlist");
        else if (a == "--csv") csvOut = next("--csv");
        else if (a == "--svg") svgOut = next("--svg");
        else if (a == "--html") htmlOut = next("--html");
        else if (a == "--schematic") schematicOut = next("--schematic");
        else if (a == "--signals") signalList = next("--signals");
        else if (a == "--simres") simresOut = next("--simres");
        else if (a == "-o" || a == "--output") outFile = next("--output");
        else { std::fprintf(stderr, "unknown option: %s\n", a.c_str()); usage(); return 2; }
    }
    if (!ov.stepType.empty() && ov.stepType != "fixed" && ov.stepType != "variable") {
        std::fprintf(stderr, "--step-type must be fixed or variable\n");
        return 2;
    }

    try {
        if (cmd == "info") {
            ResultsFileData d = readResultsFile(design);
            std::printf("results  %s (core %s)\n", design.c_str(), d.coreVersion.c_str());
            std::printf("design   %zu components, %zu wires\n",
                        d.design.value("components", nlohmann::json::array()).size(),
                        d.design.value("wires", nlohmann::json::array()).size());
            std::printf("settings %s\n", d.settings.dump().c_str());
            std::printf("meta     %s\n", d.meta.dump().c_str());
            std::printf("samples  %zu, t = %g .. %g s, signals %zu\n", d.time.size(),
                        d.time.empty() ? 0.0 : d.time.front(), d.time.empty() ? 0.0 : d.time.back(), d.signals.size());
            for (const auto& [n, v] : d.signals) std::printf("  %s (%zu)\n", n.c_str(), v.size());
            return 0;
        }
        if (cmd == "netlist") {
            CircuitDesign cd = DesignIO::loadDesignFile(design);
            NetlistBuilder::buildNodesForCircuit(cd);
            std::string nl = generateNetlistJson(cd);
            if (outFile.empty()) std::cout << nl << "\n";
            else if (!writeFile(outFile, nl)) { std::fprintf(stderr, "cannot write %s\n", outFile.c_str()); return 1; }
            return 0;
        }
        if (cmd != "run" && cmd != "signals") { usage(); return 2; }

        RunResult r = runDesignFile(design, ov);
        if (!netlistOut.empty() && !writeFile(netlistOut, r.netlistJson)) {
            std::fprintf(stderr, "cannot write %s\n", netlistOut.c_str());
            return 1;
        }
        const auto names = r.signalNames();
        std::vector<std::string> chosen;
        for (size_t s = 0; s < signalList.size();) {
            size_t e = signalList.find(',', s);
            std::string n = signalList.substr(s, e == std::string::npos ? std::string::npos : e - s);
            if (!n.empty()) {
                if (!r.signals.count(n)) { std::fprintf(stderr, "no signal '%s' (see: simpel_cli signals)\n", n.c_str()); return 1; }
                chosen.push_back(n);
            }
            if (e == std::string::npos) break;
            s = e + 1;
        }
        if (!csvOut.empty()) {
            std::string err;
            if (!exportCSV(r.time, r.signals, chosen, csvOut, 12, &err)) { std::fprintf(stderr, "csv: %s\n", err.c_str()); return 1; }
        }
        CircuitSimEngine::TelemetryData tel;
        if (!svgOut.empty() || !htmlOut.empty()) { tel.timeHistory = r.time; tel.voltages = r.signals; }
        if (!svgOut.empty()) {
            std::vector<std::string> keys = chosen;
            if (keys.empty()) keys.assign(names.begin(), names.begin() + std::min<size_t>(4, names.size()));
            if (!SVGExporter::exportScopeToSVG(tel, keys, keys, design, svgOut, (int)keys.size(), false)) {
                std::fprintf(stderr, "cannot write %s\n", svgOut.c_str());
                return 1;
            }
        }
        if (!simresOut.empty()) {
            std::vector<ResultsSignalView> views;
            for (const auto& n : names) views.push_back({n, r.signals.at(n).data(), r.signals.at(n).size()});
            nlohmann::json settings = {{"t_stop", r.stopTime}, {"dt", r.stepSize}, {"solver", r.solver},
                                       {"step", r.stepType == "variable" ? std::string("adaptive") : r.stepType}};
            nlohmann::json meta = {{"wall_seconds", r.wallSeconds}, {"cancelled", r.cancelled}, {"source", "simpel_cli"}};
            writeResultsFile(simresOut, DesignIO::saveDesign(r.design), r.netlistJson, settings, meta,
                             r.time.data(), r.time.size(), views);
        }
        if (!schematicOut.empty() && !SVGExporter::exportSchematicToSVG(r.design, schematicOut, false)) {
            std::fprintf(stderr, "cannot write %s\n", schematicOut.c_str());
            return 1;
        }
        if (!htmlOut.empty()) {
            auto scopes = SVGExporter::scopesForReport(r.design, tel);
            if (!SVGExporter::exportFullReportToHTML(r.design, tel, scopes, DesignIO::saveDesignString(r.design),
                                                     r.netlistJson, htmlOut, false)) {
                std::fprintf(stderr, "cannot write %s\n", htmlOut.c_str());
                return 1;
            }
        }
        if (cmd == "signals") {
            for (const auto& n : names) std::printf("%s\n", n.c_str());
            return 0;
        }
        std::printf("design   %s\n", design.c_str());
        std::printf("solver   %s / %s, t_stop=%g s, dt=%g s\n", r.solver.c_str(), r.stepType.c_str(), r.stopTime, r.stepSize);
        std::printf("samples  %zu, signals %zu, wall %.3f s\n", r.time.size(), names.size(), r.wallSeconds);
        std::printf("%-40s %16s %16s %16s\n", "signal", "min", "max", "final");
        for (const auto& n : names) {
            const auto& v = r.signals.at(n);
            if (v.empty()) { std::printf("%-40s %16s\n", n.c_str(), "(empty)"); continue; }
            double lo = v[0], hi = v[0];
            for (double x : v) { if (x < lo) lo = x; if (x > hi) hi = x; }
            std::printf("%-40s %16.6g %16.6g %16.6g\n", n.c_str(), lo, hi, v.back());
        }
        return 0;
    } catch (const std::exception& e) {
        std::fprintf(stderr, "error: %s\n", e.what());
        return 1;
    }
}
