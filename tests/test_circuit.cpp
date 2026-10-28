// Circuit facade: build a buck converter by connectivity alone, validate it,
// simulate it; reproduce a design file bitwise through fromDesign; error and
// validation behaviour.
// Usage: test_circuit <tests/golden/designs>
#include "engine/Circuit.hpp"
#include "engine/DesignIO.hpp"
#include "engine/Runner.hpp"
#include "engine/SignalAnalysis.hpp"

#include <atomic>
#include <cmath>
#include <cstdio>
#include <string>

using namespace CircuitSim;

static int g_pass = 0, g_fail = 0;
static void check(bool ok, const std::string& what) {
    if (ok) ++g_pass; else { ++g_fail; std::printf("FAIL: %s\n", what.c_str()); }
}
template <class F> static bool throws(F f, const std::string& needle) {
    try { f(); } catch (const std::exception& e) {
        if (std::string(e.what()).find(needle) != std::string::npos) return true;
        std::printf("  (message: %s)\n", e.what());
    }
    return false;
}
static bool hasIssue(const std::vector<ValidationIssue>& v, const std::string& code, const std::string& comp = "") {
    for (const auto& i : v) if (i.code == code && (comp.empty() || i.component == comp)) return true;
    return false;
}

static Circuit buck() {
    Circuit c("buck");
    c.add("V", "V1", {{"value", "48"}});
    c.add("MOSFET", "Q1", {{"Ron", "10m"}});
    c.add("D", "D1");
    c.add("L", "L1", {{"L", "100u"}});
    c.add("C", "C1", {{"C", "100u"}});
    c.add("R", "R1", {{"value", "5"}});
    c.add("TRI_GEN", "TRI1", {{"frequency", "20k"}});
    c.add("CONST", "K1", {{"value", "0.5"}});
    c.add("COMP", "CMP1");
    c.connect({{"V1", "A"}, {"Q1", "D"}});
    c.connect({{"Q1", "S"}, {"D1", "B"}, {"L1", "A"}});
    c.connect({{"L1", "B"}, {"C1", "A"}, {"R1", "a"}});      // pin names are case-insensitive
    c.ground({{"V1", "B"}, {"D1", "A"}, {"C1", "B"}, {"R1", "B"}});
    c.connect({{"TRI1", "Out"}, {"CMP1", "In1"}});
    c.connect({{"CMP1", "In2"}, {"K1", "Out"}});             // driver given second on purpose
    c.connect({{"CMP1", "Out"}, {"Q1", "G"}});
    c.settings().stopTime = 20e-3;
    c.settings().stepSize = 1e-6;
    return c;
}

int main(int argc, char** argv) {
    const std::string designs = argc > 1 ? argv[1] : "tests/golden/designs";

    // 1. Build, validate, simulate.
    {
        Circuit c = buck();
        auto issues = c.validate();
        for (const auto& i : issues) std::printf("  issue: %s\n", i.message.c_str());
        check(issues.empty(), "buck validates cleanly");
        CircuitDesign d = c.design();
        check(d.components.size() == 10, "GND added once (10 components)");
        bool driverHub = false;
        for (const auto& w : d.wires)
            if (w.from.compId == "K1" && w.from.terminal == "Out" && w.to.compId == "CMP1") driverHub = true;
        check(driverHub, "signal wire runs from the driver K1.Out");
        RunResult r = runDesign(d);
        check(!r.cancelled && r.time.size() == 20000, "ran 20000 steps (" + std::to_string(r.time.size()) + ")");
        check(r.signals.count("V_C1") == 1, "V_C1 recorded");
        if (r.signals.count("V_C1")) {
            auto st = CircuitSimEngine::computeSignalStats(r.time, r.signals.at("V_C1"), 18e-3, r.time.back());
            std::printf("  Vout mean %.3f V, ripple %.3f V\n", st.mean, st.pk2pk);
            check(st.isValid && st.mean > 20.0 && st.mean < 26.0, "Vout settles near D*Vin = 24 V");
        }
        // Save -> load -> same netlist.
        CircuitDesign again = DesignIO::loadDesign(nlohmann::json::parse(DesignIO::saveDesignString(d)));
        RunOverrides ov; ov.tStop = 1e-3;
        check(runDesign(again, ov).netlistJson == runDesign(d, ov).netlistJson, "saved circuit reloads to the same netlist");
    }

    // 2. fromDesign reproduces a design file exactly.
    {
        const std::string path = designs + "/Buck_converter.json";
        CircuitDesign file = DesignIO::loadDesignFile(path);
        Circuit c = Circuit::fromDesign(file);
        RunResult a = runDesign(c.design());
        RunResult b = runDesignFile(path);
        bool same = a.netlistJson == b.netlistJson && a.time == b.time && a.signals == b.signals;
        check(same, "fromDesign(Buck_converter.json) is bitwise identical to running the file");
        auto issues = c.validate();
        int errors = 0;
        for (const auto& i : issues) errors += i.severity == ValidationIssue::Severity::Error;
        check(errors == 0, "loaded design has no validation errors");
    }

    // 3. Errors.
    {
        Circuit c;
        check(throws([&] { c.add("MOSFT", "Q1"); }, "did you mean"), "unknown type suggests");
        c.add("R", "R1");
        check(throws([&] { c.add("R", "R1"); }, "already used"), "duplicate id rejected");
        check(throws([&] { c.add("R", "1R"); }, "invalid component id"), "bad id rejected");
        check(throws([&] { c.add("R", "R.1"); }, "invalid component id"), "dotted id rejected");
        check(throws([&] { c.connect({{"R1", "A"}, {"R1", "Z"}}); }, "pins: A, B"), "unknown pin lists pins");
        check(throws([&] { c.connect({{"R1", "A"}, {"NOPE", "A"}}); }, "no component 'NOPE'"), "unknown component");
        check(throws([&] { c.connect({{"R1", "A"}}); }, "at least two"), "single-pin connect rejected");
        check(c.add("R") == "R2" && c.add("MOSFET") == "MOSFET1", "automatic ids from the palette prefix");
        c.remove("R2");
        check(!c.has("R2"), "remove");
    }

    // 4. Validation findings.
    {
        Circuit c;
        c.add("R", "R1");
        c.add("MOSFET", "Q1", {{"Rn", "5m"}});                 // misspelt Ron
        c.add("CONST", "K1"); c.add("CONST", "K2"); c.add("GAIN", "G1"); c.add("PI_SECTION", "Z1");   // PI_SECTION is an unsupported block
        c.connect({{"K1", "Out"}, {"K2", "Out"}, {"G1", "In"}});
        auto v = c.validate();
        check(hasIssue(v, "ineffective_param", "Q1"), "misspelt parameter reported");
        bool suggested = false;
        for (const auto& i : v) if (i.code == "ineffective_param" && i.message.find("'Ron'") != std::string::npos) suggested = true;
        check(suggested, "ineffective parameter suggests Ron");
        check(hasIssue(v, "multiple_drivers"), "two outputs on one net is an error");
        check(hasIssue(v, "unconnected_pin", "R1"), "unconnected power pin");
        check(hasIssue(v, "unconnected_input", "Q1"), "undriven gate");
        check(hasIssue(v, "unsupported_block", "Z1"), "unsupported block");
        check(hasIssue(v, "no_ground"), "missing ground");
        check(c.ineffectiveParams("R1").empty(), "registry parameters never reported");
        check(Circuit().validate().size() == 1 && hasIssue(Circuit().validate(), "empty_circuit"), "empty circuit");
    }

    // 5. Cancellation through RunControl returns a partial run.
    {
        Circuit c = buck();
        c.settings().stopTime = 2.0;   // long run; cancelled from the progress callback
        std::atomic<bool> cancel{false};
        int calls = 0;
        RunControl ctl;
        ctl.cancel = &cancel;
        ctl.progress = [&](double t, double) { ++calls; if (t > 1e-3) cancel.store(true); };
        RunResult r = runDesign(c.design(), RunOverrides(), &ctl);
        check(r.cancelled, "run reports cancelled");
        check(!r.time.empty() && r.time.back() < 1.0, "cancelled run stopped early (t=" +
              std::to_string(r.time.empty() ? 0.0 : r.time.back()) + ")");
        check(calls >= 2, "progress called repeatedly (" + std::to_string(calls) + ")");
        bool lengthsAgree = true;
        for (const auto& [k, s] : r.signals) lengthsAgree &= s.size() == r.time.size();
        check(lengthsAgree, "partial signals have the same length as time");
    }

    std::printf("%d/%d checks passed\n", g_pass, g_pass + g_fail);
    return g_fail == 0 ? 0 : 1;
}
