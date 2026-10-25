// simpel._core: the compiled half of the SimPEL Python package.
// Keep this layer thin: argument conversion, GIL handling and NumPy wrapping only.
// Everything else belongs in simpel_core (C++) or the pure-Python `simpel` package.
#include "engine/BlockRegistry.hpp"
#include "engine/Circuit.hpp"
#include "engine/DesignIO.hpp"
#include "engine/Exporters.hpp"
#include "engine/ResultsFile.hpp"
#include "engine/SVGExporter.hpp"
#include "engine/FourierAnalysis.hpp"
#include "engine/LossAnalysisEngine.hpp"
#include "engine/LossModelLibrary.hpp"
#include "engine/Runner.hpp"
#include "engine/SignalAnalysis.hpp"
#include "engine/Sweep.hpp"
#include "engine/Version.hpp"
#include "engine/layout/Placer.hpp"
#include "engine/layout/Router.hpp"

#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <atomic>
#include <memory>
#include <mutex>
#include <cmath>
#include <map>
#include <optional>
#include <string>
#include <vector>

namespace py = pybind11;
using namespace CircuitSim;

namespace {

// Hands a std::vector<double> to NumPy without copying: the vector is moved onto the
// heap and owned by a capsule that NumPy releases with the array.
py::array_t<double> toNumpy(std::vector<double>&& v) {
    auto* owned = new std::vector<double>(std::move(v));
    py::capsule owner(owned, [](void* p) { delete static_cast<std::vector<double>*>(p); });
    return py::array_t<double>({(py::ssize_t)owned->size()}, {(py::ssize_t)sizeof(double)}, owned->data(), owner);
}

py::dict resultToDict(RunResult&& r) {
    py::dict sig;
    for (const auto& name : r.signalNames()) sig[py::str(name)] = toNumpy(std::move(r.signals[name]));
    py::dict d;
    d["time"] = toNumpy(std::move(r.time));
    d["signals"] = sig;
    d["netlist"] = r.netlistJson;
    d["wall_seconds"] = r.wallSeconds;
    d["t_stop"] = r.stopTime;
    d["dt"] = r.stepSize;
    d["solver"] = r.solver;
    d["step"] = r.stepType;
    d["cancelled"] = r.cancelled;
    d["design_json"] = DesignIO::saveDesignString(r.design);
    return d;
}

std::vector<double> toVector(const py::array_t<double, py::array::c_style | py::array::forcecast>& a) {
    if (a.ndim() != 1) throw py::value_error("expected a 1-D array");
    return std::vector<double>(a.data(), a.data() + a.size());
}

// Shared cancellation flag a Python thread can set while another one simulates.
struct CancelToken {
    std::shared_ptr<std::atomic<bool>> flag = std::make_shared<std::atomic<bool>>(false);
};

void applySettings(SolverSettings& s, const py::dict& d) {
    for (auto item : d) {
        const std::string k = py::cast<std::string>(item.first);
        py::handle v = item.second;
        if (v.is_none()) continue;
        auto num = [&]() {
            double x = py::cast<double>(v);
            if (!(x >= 0.0) || !std::isfinite(x)) throw py::value_error(k + " must be a finite number >= 0");
            return x;
        };
        if (k == "t_stop") { s.stopTime = num(); if (s.stopTime <= 0) throw py::value_error("t_stop must be > 0"); }
        else if (k == "dt") { s.stepSize = num(); if (s.stepSize <= 0) throw py::value_error("dt must be > 0"); }
        else if (k == "solver") s.solverType = py::cast<std::string>(v);
        else if (k == "step") {
            std::string st = py::cast<std::string>(v);
            if (st == "adaptive") st = "variable";
            if (st != "fixed" && st != "variable") throw py::value_error("step must be 'fixed' or 'adaptive'");
            s.stepType = st;
        }
        else if (k == "rel_tol") s.relTol = num();
        else if (k == "abs_tol_v") s.absTolV = num();
        else if (k == "abs_tol_i") s.absTolI = num();
        else if (k == "h_min") s.hMin = num();
        else if (k == "h_max") s.hMax = num();
        else if (k == "lu_cache") s.enableLUCache = py::cast<bool>(v);
        else if (k == "diode_model") s.diodeModel = py::cast<std::string>(v);
        else throw py::value_error("unknown simulation setting '" + k + "'");
    }
}

py::dict settingsToDict(const SolverSettings& s) {
    py::dict d;
    d["t_stop"] = s.stopTime;
    d["dt"] = s.stepSize;
    d["solver"] = s.solverType;
    d["step"] = s.stepType == "variable" ? std::string("adaptive") : s.stepType;
    d["rel_tol"] = s.relTol;
    d["abs_tol_v"] = s.absTolV;
    d["abs_tol_i"] = s.absTolI;
    d["h_min"] = s.hMin;
    d["h_max"] = s.hMax;
    d["lu_cache"] = s.enableLUCache;
    d["diode_model"] = s.diodeModel;
    return d;
}

std::vector<PinRef> toPins(const std::vector<std::pair<std::string, std::string>>& v) {
    std::vector<PinRef> out;
    for (const auto& [c, p] : v) out.push_back({c, p});
    return out;
}

CircuitDesign laidOut(const Circuit& c, const std::string& layout) {
    CircuitDesign d = c.design();
    if (layout == "auto") { autoPlace(d); routeDesign(d); }
    else if (layout == "place") autoPlace(d);
    else if (layout != "keep") throw py::value_error("layout must be 'auto', 'place' or 'keep'");
    return d;
}

std::once_flag g_lossOnce;
void ensureLossModels() {
    std::call_once(g_lossOnce, [] {
        auto& lib = CircuitSimEngine::LossModelLibrary::getInstance();
        if (lib.getLoadedCount() == 0 && !lib.isLoading()) lib.loadAllModels("");
    });
}

RunOverrides makeOverrides(std::optional<double> tStop, std::optional<double> dt,
                           std::optional<std::string> step, std::optional<std::string> solver,
                           std::optional<std::vector<std::pair<std::string, std::string>>> params) {
    RunOverrides ov;
    if (tStop) {
        if (!(*tStop > 0.0)) throw py::value_error("t_stop must be > 0");
        ov.tStop = *tStop;
    }
    if (dt) {
        if (!(*dt > 0.0)) throw py::value_error("dt must be > 0");
        ov.dt = *dt;
    }
    if (step) {
        std::string s = *step;
        if (s == "adaptive") s = "variable";
        if (s != "fixed" && s != "variable") throw py::value_error("step must be 'fixed' or 'adaptive'");
        ov.stepType = s;
    }
    if (solver) ov.solver = *solver;
    if (params) ov.params = *params;
    return ov;
}

} // namespace

PYBIND11_MODULE(_core, m) {
    m.doc() = "SimPEL compiled core (internal; use the `simpel` package)";

    m.def("version", []() { return std::string(simpelVersion()); },
          "Version of the C++ core.");

    m.def("run_design_file",
          [](const std::string& path, std::optional<double> t_stop, std::optional<double> dt,
             std::optional<std::string> step, std::optional<std::string> solver,
             std::optional<std::vector<std::pair<std::string, std::string>>> params) {
              RunOverrides ov = makeOverrides(t_stop, dt, step, solver, params);
              RunResult r;
              {
                  // The simulation touches no Python state.
                  py::gil_scoped_release release;
                  r = runDesignFile(path, ov);
              }
              return resultToDict(std::move(r));
          },
          py::arg("path"), py::kw_only(), py::arg("t_stop") = py::none(), py::arg("dt") = py::none(),
          py::arg("step") = py::none(), py::arg("solver") = py::none(), py::arg("params") = py::none(),
          "Simulate a schematic .json file the way the GUI does. Returns a dict with\n"
          "'time' and 'signals' (NumPy arrays), 'netlist', 'wall_seconds' and the\n"
          "effective 't_stop', 'dt', 'solver', 'step'. `params` is a list of\n"
          "('CompId.param', 'value') overrides.");

    // ── Block registry ──────────────────────────────────────────────────────
    m.def("block_types", []() {
        py::list out;
        for (const auto& b : BlockRegistry::instance().blocks()) {
            py::dict d;
            d["type"] = b.type;
            d["name"] = b.name;
            d["category"] = b.category;
            d["subcategory"] = b.subcategory;
            d["role"] = blockRoleName(b.role);
            d["legacy"] = b.legacy;
            out.append(d);
        }
        return out;
    }, "Summary of every registered block type, in registry order.");

    m.def("block_info", [](const std::string& type) {
        const BlockRegistry& reg = BlockRegistry::instance();
        const BlockDef* b = reg.resolve(type);
        if (!b) {
            std::string msg = "unknown block type '" + type + "'";
            auto s = reg.suggest(type);
            if (!s.empty()) {
                msg += "; did you mean";
                for (size_t i = 0; i < s.size(); ++i) msg += (i ? ", " : " ") + s[i];
                msg += "?";
            }
            throw py::key_error(msg);
        }
        py::dict d;
        d["type"] = b->type;
        d["name"] = b->name;
        d["category"] = b->category;
        d["subcategory"] = b->subcategory;
        d["description"] = b->description;
        d["source_file"] = b->sourceFile;
        d["role"] = blockRoleName(b->role);
        d["legacy"] = b->legacy;
        d["aliases"] = b->aliases;
        py::list params;
        for (const auto& p : b->params) {
            py::dict pd;
            pd["name"] = p.name;
            pd["default"] = p.defaultValue;
            pd["unit"] = p.unit;
            pd["description"] = p.description;
            params.append(pd);
        }
        d["params"] = params;
        d["choices"] = b->choices;
        py::list pins;
        for (const auto& p : reg.pins(b->type)) {
            py::dict pd;
            pd["name"] = p.name;
            pd["kind"] = p.isControl ? "control" : "power";
            pd["x"] = p.relX;
            pd["y"] = p.relY;
            pins.append(pd);
        }
        d["pins"] = pins;
        py::list palette;
        for (int idx : b->paletteEntries) {
            const auto& e = reg.palette()[idx];
            py::dict pd;
            pd["text"] = e.text;
            pd["subcategory"] = e.subcategory;
            pd["basic"] = e.basic;
            py::list kv;
            for (const auto& [k, v] : e.params) kv.append(py::make_tuple(k, v));
            pd["params"] = kv;
            palette.append(pd);
        }
        d["palette"] = palette;
        return d;
    }, py::arg("type"), "Full description of one block type. Raises KeyError for unknown types.");

    // ── Circuit construction ────────────────────────────────────────────────
    py::register_exception<std::invalid_argument>(m, "CircuitError", PyExc_ValueError);

    py::class_<Circuit>(m, "Circuit")
        .def(py::init<std::string>(), py::arg("name") = "circuit")
        .def_static("from_design_file", [](const std::string& path, const std::string& name) {
            return Circuit::fromDesign(DesignIO::loadDesignFile(path), name);
        }, py::arg("path"), py::arg("name") = "circuit")
        .def_property_readonly("name", &Circuit::name)
        .def("add", &Circuit::add, py::arg("type"), py::arg("id") = "",
             py::arg("params") = std::map<std::string, std::string>{})
        .def("remove", &Circuit::remove)
        .def("has", &Circuit::has)
        .def("ids", &Circuit::componentIds)
        .def("component", [](const Circuit& c, const std::string& id) {
            const ComponentInstance& ci = c.component(id);
            py::dict d;
            d["id"] = ci.id;
            d["type"] = ci.rawTypeStr;
            d["label"] = ci.label;
            std::map<std::string, std::string> params(ci.parameters.begin(), ci.parameters.end());
            d["params"] = params;
            return d;
        })
        .def("set_param", &Circuit::setParam)
        .def("pins", [](const Circuit& c, const std::string& id) {
            py::list out;
            for (const auto& p : c.pins(id)) {
                py::dict d;
                d["name"] = p.name;
                d["kind"] = p.isControl ? "control" : "power";
                d["output"] = p.isControl && p.dirX > 0.0f;
                out.append(d);
            }
            return out;
        })
        .def("resolve_pin", &Circuit::resolvePin)
        .def("connect", [](Circuit& c, const std::vector<std::pair<std::string, std::string>>& pins) {
            c.connect(toPins(pins));
        })
        .def("ground", [](Circuit& c, const std::vector<std::pair<std::string, std::string>>& pins) {
            c.ground(toPins(pins));
        })
        .def("ineffective_params", &Circuit::ineffectiveParams)
        .def("param_affects_netlist", &Circuit::paramAffectsNetlist)
        .def("validate", [](const Circuit& c) {
            py::list out;
            for (const auto& i : c.validate()) {
                py::dict d;
                d["severity"] = i.severity == ValidationIssue::Severity::Error ? "error" : "warning";
                d["code"] = i.code;
                d["component"] = i.component;
                d["pin"] = i.pin;
                d["message"] = i.message;
                out.append(d);
            }
            return out;
        })
        .def("settings", [](const Circuit& c) { return settingsToDict(c.settings()); })
        .def("set_settings", [](Circuit& c, const py::dict& d) { applySettings(c.settings(), d); })
        .def("design_json", [](const Circuit& c, const std::string& layout) {
            return DesignIO::saveDesignString(laidOut(c, layout));
        }, py::arg("layout") = "keep")
        .def("save", [](const Circuit& c, const std::string& path, const std::string& layout) {
            DesignIO::saveDesignFile(laidOut(c, layout), path);
        }, py::arg("path"), py::arg("layout") = "keep");

    // ── Simulation ──────────────────────────────────────────────────────────
    py::class_<CancelToken>(m, "CancelToken")
        .def(py::init<>())
        .def("set", [](CancelToken& t) { t.flag->store(true); })
        .def("clear", [](CancelToken& t) { t.flag->store(false); })
        .def("is_set", [](const CancelToken& t) { return t.flag->load(); });

    m.def("simulate", [](const Circuit& c, const py::dict& settings, py::object progress, CancelToken token) {
        CircuitDesign d = c.design();
        applySettings(d.settings, settings);

        std::shared_ptr<std::atomic<bool>> flag = token.flag;
        std::unique_ptr<py::error_already_set> pyError;
        RunControl ctl;
        ctl.cancel = flag.get();
        ctl.progress = [&](double t, double tStop) {
            py::gil_scoped_acquire gil;
            if (pyError) return;
            if (PyErr_CheckSignals() != 0) {               // Ctrl+C
                pyError = std::make_unique<py::error_already_set>();
                flag->store(true);
                return;
            }
            if (progress.is_none()) return;
            try {
                py::object r = progress(t, tStop);
                if (!r.is_none() && !py::bool_(r)) flag->store(true);   // returning False cancels
            } catch (py::error_already_set& e) {
                pyError = std::make_unique<py::error_already_set>(std::move(e));
                flag->store(true);
            }
        };
        RunResult r;
        {
            py::gil_scoped_release release;
            r = runDesign(std::move(d), RunOverrides(), &ctl);
        }
        if (pyError) throw std::move(*pyError);
        return resultToDict(std::move(r));
    }, py::arg("circuit"), py::arg("settings"), py::arg("progress") = py::none(),
       py::arg("cancel") = CancelToken(),
       "Simulate a Circuit. `settings` keys: t_stop, dt, solver, step ('fixed' or\n"
       "'adaptive'), rel_tol, abs_tol_v, abs_tol_i, h_min, h_max, lu_cache.\n"
       "`progress(t, t_stop)` is called ~30 times a second; returning False cancels.");

    m.def("sweep", [](const Circuit& c, const py::dict& settings,
                      const std::vector<std::vector<std::pair<std::string, std::string>>>& points,
                      int workers, const std::vector<std::string>& keep, py::object progress, CancelToken token) {
        CircuitDesign d = c.design();
        applySettings(d.settings, settings);
        std::shared_ptr<std::atomic<bool>> flag = token.flag;
        std::unique_ptr<py::error_already_set> pyError;
        std::mutex errMutex;
        SweepOptions o;
        o.workers = workers;
        o.keepSignals = keep;
        o.cancel = flag.get();
        o.onPointDone = [&](size_t done, size_t total) {
            py::gil_scoped_acquire gil;
            std::lock_guard<std::mutex> lock(errMutex);
            if (pyError) return;
            if (PyErr_CheckSignals() != 0) {
                pyError = std::make_unique<py::error_already_set>();
                flag->store(true);
                return;
            }
            if (progress.is_none()) return;
            try {
                py::object r = progress(done, total);
                if (!r.is_none() && !py::bool_(r)) flag->store(true);
            } catch (py::error_already_set& e) {
                pyError = std::make_unique<py::error_already_set>(std::move(e));
                flag->store(true);
            }
        };
        std::vector<SweepPointResult> res;
        {
            py::gil_scoped_release release;
            res = runSweep(d, RunOverrides(), points, o);
        }
        if (pyError) throw std::move(*pyError);
        py::list out;
        for (auto& r : res) {
            if (r.ok) out.append(py::make_tuple(resultToDict(std::move(r.result)), py::none()));
            else out.append(py::make_tuple(py::none(), r.error));
        }
        return out;
    }, py::arg("circuit"), py::arg("settings"), py::arg("points"), py::arg("workers") = 0,
       py::arg("keep") = std::vector<std::string>{}, py::arg("progress") = py::none(),
       py::arg("cancel") = CancelToken());

    m.def("default_workers", &defaultWorkerCount);

    // ── Analyses ────────────────────────────────────────────────────────────
    m.def("signal_stats", [](const py::array_t<double, py::array::c_style | py::array::forcecast>& t,
                             const py::array_t<double, py::array::c_style | py::array::forcecast>& y,
                             double t1, double t2) {
        auto tv = toVector(t), yv = toVector(y);
        if (tv.size() != yv.size()) throw py::value_error("time and signal lengths differ");
        CircuitSimEngine::SignalStats s;
        { py::gil_scoped_release release; s = CircuitSimEngine::computeSignalStats(tv, yv, t1, t2); }
        py::dict d;
        d["mean"] = s.mean; d["rms"] = s.rms; d["min"] = s.minVal; d["max"] = s.maxVal;
        d["abs_max"] = s.absMaxVal; d["pk2pk"] = s.pk2pk; d["y_t1"] = s.yAtT1; d["y_t2"] = s.yAtT2;
        d["valid"] = s.isValid;
        return d;
    });

    m.def("fourier", [](const py::array_t<double, py::array::c_style | py::array::forcecast>& t,
                        const py::array_t<double, py::array::c_style | py::array::forcecast>& y,
                        double t1, double t2, int maxHarmonic) {
        auto tv = toVector(t), yv = toVector(y);
        if (tv.size() != yv.size()) throw py::value_error("time and signal lengths differ");
        if (maxHarmonic < 1) throw py::value_error("max_harmonic must be >= 1");
        CircuitSimEngine::FourierResult f;
        { py::gil_scoped_release release; f = CircuitSimEngine::computeFourierSpectrum(tv, yv, t1, t2, maxHarmonic); }
        py::dict d;
        d["f0"] = f.fundamentalFreq; d["fundamental"] = f.fundamentalMag; d["thd"] = f.thdRatio;
        d["dc"] = f.dcOffset; d["valid"] = f.isValid;
        d["orders"] = toNumpy(std::move(f.harmonicOrders));
        d["magnitudes"] = toNumpy(std::move(f.harmonicMags));
        d["phases"] = toNumpy(std::move(f.harmonicPhases));
        return d;
    });

    m.def("loss_models", []() {
        ensureLossModels();
        return CircuitSimEngine::LossModelLibrary::getInstance().getAvailableModels();
    }, "Names of the semiconductor loss models available for 'thermal_model'.");

    m.def("load_loss_models", [](const std::vector<std::string>& dirs) {
        std::call_once(g_lossOnce, [] {});   // an explicit choice replaces the default search
        auto& lib = CircuitSimEngine::LossModelLibrary::getInstance();
        { py::gil_scoped_release release; lib.loadFromDirectories(dirs); }
        return lib.getLoadedCount();
    }, py::arg("dirs"), "Replace the loss model library with the models found in `dirs`.");

    m.def("losses", [](const std::string& designJson,
                       const py::array_t<double, py::array::c_style | py::array::forcecast>& t,
                       const std::map<std::string, py::array_t<double, py::array::c_style | py::array::forcecast>>& signals,
                       double tStart, double duration, double tj) {
        ensureLossModels();
        DesignIO::LoadOptions o;
        o.normaliseWires = false;
        CircuitDesign design = DesignIO::loadDesign(nlohmann::json::parse(designJson), o);
        CircuitSimEngine::TelemetryData tel;
        tel.timeHistory = toVector(t);
        for (const auto& [k, v] : signals) tel.voltages[k] = toVector(v);
        std::vector<CircuitSimEngine::SwitchLossResult> res;
        {
            py::gil_scoped_release release;
            res = CircuitSimEngine::LossAnalysisEngine::runAnalysis(tel, design, tStart, duration, tj);
        }
        py::list out;
        for (const auto& r : res) {
            py::dict d;
            d["component"] = r.componentId; d["model"] = r.modelName;
            d["conduction"] = r.P_cond; d["turn_on"] = r.P_sw_on; d["turn_off"] = r.P_sw_off;
            d["total"] = r.P_total;
            d["hard_turn_ons"] = r.num_hard_turn_on; d["soft_turn_ons"] = r.num_soft_turn_on;
            d["turn_offs"] = r.num_turn_off;
            out.append(d);
        }
        return out;
    });

    // ── Exports and plotting support ────────────────────────────────────────
    using Arr = py::array_t<double, py::array::c_style | py::array::forcecast>;
    auto telemetryOf = [](const Arr& t, const std::map<std::string, Arr>& signals) {
        CircuitSimEngine::TelemetryData tel;
        tel.timeHistory = toVector(t);
        for (const auto& [k, v] : signals) tel.voltages[k] = toVector(v);
        return tel;
    };

    m.def("export_csv", [telemetryOf](const std::string& path, const Arr& t, const std::map<std::string, Arr>& signals,
                                      const std::vector<std::string>& names, int precision) {
        auto tel = telemetryOf(t, signals);
        std::string err;
        bool ok;
        { py::gil_scoped_release release; ok = exportCSV(tel.timeHistory, tel.voltages, names, path, precision, &err); }
        if (!ok) throw py::value_error(err);
    }, py::arg("path"), py::arg("time"), py::arg("signals"), py::arg("names"), py::arg("precision") = 12);

    m.def("scope_svg", [telemetryOf](const Arr& t, const std::map<std::string, Arr>& signals,
                                     const std::vector<std::string>& names, const std::vector<std::string>& labels,
                                     const std::string& title, int panes, bool dark, double t1, double t2) {
        auto tel = telemetryOf(t, signals);
        std::string svg;
        bool ok;
        { py::gil_scoped_release release;
          ok = SVGExporter::exportScopeToSVGString(tel, names, labels, title, svg, panes, dark, t1, t2); }
        if (!ok) throw py::value_error("could not render the plot (no samples in the window?)");
        return svg;
    });

    m.def("schematic_svg", [](const std::string& designJson, bool dark) {
        DesignIO::LoadOptions o;
        o.normaliseWires = false;
        CircuitDesign d = DesignIO::loadDesign(nlohmann::json::parse(designJson), o);
        std::string svg;
        if (!SVGExporter::exportSchematicToSVGString(d, svg, dark)) throw py::value_error("could not render the schematic");
        return svg;
    });

    m.def("export_html", [telemetryOf](const std::string& path, const std::string& designJson, const std::string& netlist,
                                       const Arr& t, const std::map<std::string, Arr>& signals,
                                       const std::vector<std::vector<std::string>>& panes, bool pdf) {
        DesignIO::LoadOptions o;
        o.normaliseWires = false;
        CircuitDesign d = DesignIO::loadDesign(nlohmann::json::parse(designJson), o);
        auto tel = telemetryOf(t, signals);
        std::vector<ScopeReportData> scopes;
        if (panes.empty()) {
            scopes = SVGExporter::scopesForReport(d, tel);
        } else {
            int i = 0;
            for (const auto& keys : panes) {
                for (const auto& k : keys)
                    if (!tel.voltages.count(k)) throw py::value_error("no signal '" + k + "'");
                ScopeReportData s;
                s.scopeId = "Plot" + std::to_string(++i);
                s.scopeTitle = s.scopeId;
                s.signalKeys = keys;
                s.signalLabels = keys;
                s.numPanes = (int)keys.size();
                scopes.push_back(s);
            }
        }
        ReportExportOptions opts;
        opts.format = pdf ? ReportExportFormat::BOTH_HTML_PDF : ReportExportFormat::HTML;
        bool ok;
        { py::gil_scoped_release release;
          ok = SVGExporter::exportFullReportToHTML(d, tel, scopes, DesignIO::saveDesignString(d), netlist, path, false, opts); }
        if (!ok) throw py::value_error("could not write the report to " + path);
    });

    m.def("write_results", [](const std::string& path, const std::string& designJson, const std::string& netlist,
                              const std::string& settingsJson, const std::string& metaJson, const Arr& t,
                              const std::vector<std::pair<std::string, Arr>>& signals) {
        if (t.ndim() != 1) throw py::value_error("time must be 1-D");
        std::vector<ResultsSignalView> views;
        for (const auto& [name, a] : signals) {
            if (a.ndim() != 1) throw py::value_error("signal '" + name + "' must be 1-D");
            views.push_back({name, a.data(), (size_t)a.size()});
        }
        auto design = nlohmann::json::parse(designJson);
        auto settings = nlohmann::json::parse(settingsJson);
        auto meta = nlohmann::json::parse(metaJson);
        py::gil_scoped_release release;
        writeResultsFile(path, design, netlist, settings, meta, t.data(), (size_t)t.size(), views);
    });

    m.def("read_results", [](const std::string& path) {
        ResultsFileData d;
        { py::gil_scoped_release release; d = readResultsFile(path); }
        py::dict sig;
        for (auto& [name, v] : d.signals) sig[py::str(name)] = toNumpy(std::move(v));
        py::dict out;
        out["time"] = toNumpy(std::move(d.time));
        out["signals"] = sig;
        out["netlist"] = d.netlist;
        out["design_json"] = d.design.dump(2);
        out["settings_json"] = d.settings.dump();
        out["meta_json"] = d.meta.dump();
        out["core_version"] = d.coreVersion;
        return out;
    });

    m.def("is_results_file", &looksLikeResultsFile);

    m.def("decimate", [](const Arr& t, const Arr& y, size_t maxPoints) {
        if (t.size() != y.size()) throw py::value_error("time and signal lengths differ");
        std::vector<double> ot, oy;
        { py::gil_scoped_release release; decimateMinMax(t.data(), y.data(), (size_t)t.size(), maxPoints, ot, oy); }
        return py::make_tuple(toNumpy(std::move(ot)), toNumpy(std::move(oy)));
    }, "Min-max decimation: at most max_points samples keeping every bucket's extremes.");

    // The library prints progress for the GUI console; keep Python quiet.
    CircuitSimEngine::LossModelLibrary::getInstance().setVerbose(false);

    // std::runtime_error from the core maps to RuntimeError by default; design-file
    // problems are more useful as ValueError / FileNotFoundError on the Python side,
    // which the pure-Python wrapper does.
}
