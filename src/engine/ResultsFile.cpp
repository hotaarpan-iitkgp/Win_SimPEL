#include "ResultsFile.hpp"

#include "Version.hpp"

#include <cstdint>
#include <cstring>
#include <fstream>
#include <limits>
#include <set>
#include <stdexcept>

namespace CircuitSim {

namespace {

const char kMagic[8] = {'S', 'I', 'M', 'P', 'E', 'L', 'R', 'S'};

bool hostIsLittleEndian() {
    const uint16_t v = 1;
    unsigned char b;
    std::memcpy(&b, &v, 1);
    return b == 1;
}

void requireLittleEndian() {
    if (!hostIsLittleEndian()) throw std::runtime_error(".simres files are little-endian; this host is not supported");
}

template <class T> void putLE(std::ofstream& o, T v) {
    unsigned char b[sizeof(T)];
    for (size_t i = 0; i < sizeof(T); ++i) b[i] = (unsigned char)((uint64_t)v >> (8 * i));
    o.write(reinterpret_cast<const char*>(b), sizeof(T));
}

template <class T> T getLE(const unsigned char* p) {
    uint64_t v = 0;
    for (size_t i = 0; i < sizeof(T); ++i) v |= (uint64_t)p[i] << (8 * i);
    return (T)v;
}

} // namespace

void writeResultsFile(const std::string& path, const nlohmann::json& design, const std::string& netlist,
                      const nlohmann::json& settings, const nlohmann::json& meta,
                      const double* time, size_t samples, const std::vector<ResultsSignalView>& signals) {
    requireLittleEndian();
    nlohmann::json h;
    h["format"] = "simpel-results";
    h["version"] = kResultsFileVersion;
    h["core_version"] = simpelVersion();
    h["design"] = design;
    h["netlist"] = netlist;
    h["settings"] = settings;
    h["meta"] = meta;
    h["samples"] = samples;
    nlohmann::json list = nlohmann::json::array();
    std::set<std::string> seen;
    for (const auto& s : signals) {
        if (s.name.empty()) throw std::runtime_error("signal names must not be empty");
        if (!seen.insert(s.name).second) throw std::runtime_error("duplicate signal name '" + s.name + "'");
        if (s.size && !s.data) throw std::runtime_error("signal '" + s.name + "' has no data");
        list.push_back({{"name", s.name}, {"length", s.size}});
    }
    h["signals"] = list;
    const std::string header = h.dump();

    const std::string tmp = path + ".partial";
    {
        std::ofstream o(tmp, std::ios::binary | std::ios::trunc);
        if (!o) throw std::runtime_error("cannot write " + path);
        o.write(kMagic, sizeof kMagic);
        putLE<uint32_t>(o, kResultsFileVersion);
        putLE<uint64_t>(o, (uint64_t)header.size());
        o.write(header.data(), (std::streamsize)header.size());
        if (samples) o.write(reinterpret_cast<const char*>(time), (std::streamsize)(samples * sizeof(double)));
        for (const auto& s : signals)
            if (s.size) o.write(reinterpret_cast<const char*>(s.data), (std::streamsize)(s.size * sizeof(double)));
        o.flush();
        if (!o) { o.close(); std::remove(tmp.c_str()); throw std::runtime_error("write failed: " + path); }
    }
    // Replace the target only once the file is complete.
    std::remove(path.c_str());
    if (std::rename(tmp.c_str(), path.c_str()) != 0) {
        std::remove(tmp.c_str());
        throw std::runtime_error("cannot write " + path);
    }
}

bool looksLikeResultsFile(const std::string& path) {
    std::ifstream f(path, std::ios::binary);
    char m[8] = {};
    f.read(m, sizeof m);
    return f.gcount() == (std::streamsize)sizeof m && std::memcmp(m, kMagic, sizeof m) == 0;
}

ResultsFileData readResultsFile(const std::string& path) {
    requireLittleEndian();
    std::ifstream f(path, std::ios::binary | std::ios::ate);
    if (!f) throw std::runtime_error("cannot open " + path);
    const std::streamoff fileSize = f.tellg();
    f.seekg(0);
    auto bad = [&](const std::string& why) { return std::runtime_error(path + ": not a valid .simres file (" + why + ")"); };
    if (fileSize < 20) throw bad("too short");
    unsigned char fixed[20];
    f.read(reinterpret_cast<char*>(fixed), sizeof fixed);
    if (std::memcmp(fixed, kMagic, 8) != 0) throw bad("wrong magic");
    const uint32_t version = getLE<uint32_t>(fixed + 8);
    if (version != kResultsFileVersion)
        throw std::runtime_error(path + ": .simres version " + std::to_string(version) + " is not supported (this build reads " +
                                 std::to_string(kResultsFileVersion) + ")");
    const uint64_t hlen = getLE<uint64_t>(fixed + 12);
    if (hlen > (uint64_t)(fileSize - 20)) throw bad("header length past end of file");
    std::string header((size_t)hlen, '\0');
    f.read(&header[0], (std::streamsize)hlen);

    nlohmann::json h;
    try { h = nlohmann::json::parse(header); } catch (const std::exception& e) { throw bad(std::string("header: ") + e.what()); }
    if (!h.is_object() || h.value("format", "") != "simpel-results") throw bad("header format");
    if (!h.contains("samples") || !h["samples"].is_number_unsigned()) throw bad("samples");
    if (!h.contains("signals") || !h["signals"].is_array()) throw bad("signals");

    const uint64_t samples = h["samples"].get<uint64_t>();
    std::vector<std::pair<std::string, uint64_t>> layout;
    uint64_t totalValues = samples;
    std::set<std::string> seen;
    for (const auto& s : h["signals"]) {
        if (!s.is_object() || !s.contains("name") || !s["name"].is_string() || !s.contains("length") ||
            !s["length"].is_number_unsigned())
            throw bad("signal entry");
        const std::string name = s["name"].get<std::string>();
        if (name.empty() || !seen.insert(name).second) throw bad("signal name '" + name + "'");
        const uint64_t len = s["length"].get<uint64_t>();
        if (len > std::numeric_limits<uint64_t>::max() / 8 - totalValues) throw bad("sizes");
        totalValues += len;
        layout.emplace_back(name, len);
    }
    const uint64_t payload = (uint64_t)fileSize - 20 - hlen;
    if (payload != totalValues * 8)
        throw bad("data size " + std::to_string(payload) + " bytes, header describes " + std::to_string(totalValues * 8));

    ResultsFileData d;
    d.design = h.value("design", nlohmann::json::object());
    d.netlist = h.value("netlist", "");
    d.settings = h.value("settings", nlohmann::json::object());
    d.meta = h.value("meta", nlohmann::json::object());
    d.coreVersion = h.value("core_version", "");
    d.time.resize((size_t)samples);
    if (samples) f.read(reinterpret_cast<char*>(d.time.data()), (std::streamsize)(samples * 8));
    d.signals.reserve(layout.size());
    for (const auto& [name, len] : layout) {
        std::vector<double> v((size_t)len);
        if (len) f.read(reinterpret_cast<char*>(v.data()), (std::streamsize)(len * 8));
        d.signals.emplace_back(name, std::move(v));
    }
    if (!f) throw bad("read failed");
    return d;
}

} // namespace CircuitSim
