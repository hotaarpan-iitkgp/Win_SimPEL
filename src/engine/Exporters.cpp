// CSV export. Replaces CLIProcessor::exportCSVData (src/cli, never built), which
// wrote columns in hash-map order, padded precision inconsistently (8 digits for
// time, 6 for values) and silently skipped "node_*" signals.
#include "Exporters.hpp"

#include <algorithm>
#include <cstdio>
#include <fstream>

namespace CircuitSim {

bool exportCSV(const std::vector<double>& time,
               const std::unordered_map<std::string, std::vector<double>>& signals,
               const std::vector<std::string>& namesIn,
               const std::string& path,
               int precision,
               std::string* error) {
    auto fail = [&](const std::string& msg) {
        if (error) *error = msg;
        return false;
    };
    if (precision < 1 || precision > 17) return fail("precision must be between 1 and 17");
    std::vector<std::string> names = namesIn;
    if (names.empty()) {
        for (const auto& kv : signals) names.push_back(kv.first);
        std::sort(names.begin(), names.end());
    }
    std::vector<const std::vector<double>*> cols;
    for (const auto& n : names) {
        auto it = signals.find(n);
        if (it == signals.end()) return fail("no signal '" + n + "'");
        cols.push_back(&it->second);
    }
    std::ofstream out(path, std::ios::binary);
    if (!out) return fail("cannot write " + path);

    auto csvField = [](const std::string& s) {
        if (s.find_first_of(",\"\n\r") == std::string::npos) return s;
        std::string q = "\"";
        for (char c : s) { if (c == '"') q += '"'; q += c; }
        return q + "\"";
    };
    std::string line = "time";
    for (const auto& n : names) line += "," + csvField(n);
    out << line << "\n";

    char buf[40];
    char fmt[8];
    std::snprintf(fmt, sizeof fmt, "%%.%dg", precision);
    std::string row;
    for (size_t i = 0; i < time.size(); ++i) {
        row.clear();
        std::snprintf(buf, sizeof buf, fmt, time[i]);
        row += buf;
        for (const auto* c : cols) {
            row += ',';
            if (i < c->size()) {
                std::snprintf(buf, sizeof buf, fmt, (*c)[i]);
                row += buf;
            }
        }
        row += '\n';
        out.write(row.data(), (std::streamsize)row.size());
    }
    out.flush();
    if (!out) return fail("write failed: " + path);
    return true;
}

} // namespace CircuitSim
