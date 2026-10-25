// Golden-output regression harness.
//
// For every design listed in manifest.txt this runs the same pipeline the GUI runs
// when a schematic is opened and simulated (File -> Open, canvas normalisation,
// node building, netlist emission, parse, transient run), and records:
//   * a hash of the emitted netlist JSON bytes (GUI path and headless-CLI path),
//   * a bitwise hash of the time vector and of every telemetry signal.
// The result is compared line-for-line against expected/<name>.golden.
//
// Usage: golden_runner [--root <tests/golden>] [--update] [--filter <substr>]
//                      [--determinism] [--verbose]
// Exit code 0 = all designs match, 1 = mismatch or error.
//
// A netlist hash match is the strong gate for loader / emitter refactors (the netlist
// fully determines the simulation); the signal hashes gate solver changes.
#include "golden_pipeline.hpp"

#include <chrono>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace fs = std::filesystem;

static std::vector<std::string> readLines(const fs::path& p) {
    std::vector<std::string> out;
    std::ifstream f(p);
    std::string line;
    while (std::getline(f, line)) {
        if (!line.empty() && line.back() == '\r') line.pop_back();
        out.push_back(line);
    }
    return out;
}

int main(int argc, char** argv) {
    std::string root = "tests/golden";
    std::string filter;
    bool update = false, determinism = false, verbose = false;
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--root" && i + 1 < argc) root = argv[++i];
        else if (a == "--filter" && i + 1 < argc) filter = argv[++i];
        else if (a == "--update") update = true;
        else if (a == "--determinism") determinism = true;
        else if (a == "--verbose") verbose = true;
        else { std::cerr << "unknown argument: " << a << "\n"; return 2; }
    }

    const fs::path rootP(root);
    const auto manifest = golden::readManifest(rootP / "manifest.txt");
    if (manifest.empty()) {
        std::cerr << "no entries in " << (rootP / "manifest.txt").string() << "\n";
        return 2;
    }
    fs::create_directories(rootP / "expected");

    int pass = 0, fail = 0, updated = 0;
    const auto tAll = std::chrono::steady_clock::now();
    for (const auto& entry : manifest) {
        if (!filter.empty() && entry.name.find(filter) == std::string::npos) continue;
        const auto t0 = std::chrono::steady_clock::now();
        std::vector<std::string> got = golden::runEntry(rootP, entry);
        const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();

        if (determinism) {
            std::vector<std::string> again = golden::runEntry(rootP, entry);
            if (again != got) {
                std::printf("[NONDET] %s: two runs in one process differ\n", entry.name.c_str());
                ++fail;
                continue;
            }
        }

        const fs::path expP = rootP / "expected" / (entry.name + ".golden");
        if (update) {
            std::ofstream o(expP, std::ios::binary);
            for (const auto& l : got) o << l << "\n";
            ++updated;
            std::printf("[UPDATE] %-60s %8.1f ms\n", entry.name.c_str(), ms);
            continue;
        }
        if (!fs::exists(expP)) {
            std::printf("[MISSING] %s (run with --update to create)\n", entry.name.c_str());
            ++fail;
            continue;
        }
        const auto exp = readLines(expP);
        if (exp == got) {
            ++pass;
            if (verbose) std::printf("[PASS] %-60s %8.1f ms\n", entry.name.c_str(), ms);
            continue;
        }
        ++fail;
        std::printf("[FAIL] %s\n", entry.name.c_str());
        // Report differing lines, keyed by their first two tokens so a changed
        // signal shows expected and actual side by side.
        auto key = [](const std::string& l) {
            std::istringstream ss(l); std::string a, b; ss >> a >> b; return a + " " + b;
        };
        int shown = 0;
        for (const auto& e : exp) {
            bool found = false;
            for (const auto& g : got) if (g == e) { found = true; break; }
            if (found) continue;
            std::string actual = "(absent)";
            for (const auto& g : got) if (key(g) == key(e)) { actual = g; break; }
            if (shown++ < 12) std::printf("    expected: %s\n    actual:   %s\n", e.c_str(), actual.c_str());
        }
        for (const auto& g : got) {
            bool found = false;
            for (const auto& e : exp) if (key(e) == key(g)) { found = true; break; }
            if (!found && shown++ < 12) std::printf("    new:      %s\n", g.c_str());
        }
    }
    const double sAll = std::chrono::duration<double>(std::chrono::steady_clock::now() - tAll).count();
    if (update) std::printf("golden: updated %d expectation file(s) in %.1f s\n", updated, sAll);
    else std::printf("golden: %d passed, %d failed (%.1f s)\n", pass, fail, sAll);
    return fail == 0 ? 0 : 1;
}
