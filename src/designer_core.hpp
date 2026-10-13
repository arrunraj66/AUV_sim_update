// designer_core.hpp - the non-GUI half of the Designer integration (Step 10): vehicle library, building a Spec from an edited design (with the published
// overrides of a real vehicle), applying a design to a deployed vehicle, the scripted "test this design" run, and the saved window state.
// No OpenGL and no ImGui in here, so design_integration_test can run it.
#pragma once
#include "real_models.hpp"
#include "config.hpp"
#include "project_paths.hpp"
#include <string>
#include <vector>
#include <cmath>
#include <fstream>
#include <iterator>

namespace dcore {

// ------------------------------------------------------------------------------------------------ the vehicle library
struct LibEntry {
    enum Kind { Saved, Real, DeployOnly } kind = Saved;
    std::string name;            // file stem for a saved design, REAL_NAMES text for a real vehicle
    std::string label;           // what the combo shows
    int real_id = -1;
    int builtin = -1;            // >= 0: one of design::builtin_designs(); if its file is unreadable the built-in copy is used
};

struct Library {
    std::string dir;             // the designs folder (absolute; default <project root>/config/designs)
    std::vector<LibEntry> items;
    int n_saved = 0, n_real = 0, skipped = 0;
    std::vector<std::string> log;        // per file: accepted or why skipped (for debugging; nothing is shown to the user)
    Library() : dir(paths::designs_dir()) {}
    static std::string stem_of(const std::string& name) { std::string f; for (char c : name) f += (std::isalnum((unsigned char)c) ? c : '_'); return f; }
    // Creates the folder, regenerates any missing built-in design file (existing files, edited or not, are left alone), then lists:
    // the saved designs (files that are not designs are skipped without hiding the others), the four built-in designs even if their file is unusable,
    // then Tethys, LAUV, ECA A9 and REMUS as editable copies and BlueROV2 (deploy-only).
    void refresh() {
        items.clear(); log.clear(); skipped = 0; std::error_code ec; std::filesystem::create_directories(dir, ec); design::write_builtin_designs(dir);
        std::vector<bool> have(design::builtin_designs().size(), false);
        const std::vector<design::DesignIn> bi = design::builtin_designs();
        for (const std::string& n : design::list_designs(dir)) {
            std::string path = dir + "/" + n + ".json"; design::DesignIn probe;
            bool ok = paths::looks_like_design(path) && design::load_design(path, probe);
            log.push_back(n + ": " + (ok ? "accepted" : "skipped (not a design)")); if (!ok) { ++skipped; continue; }
            LibEntry e; e.kind = LibEntry::Saved; e.name = n; e.label = n;
            for (size_t i = 0; i < bi.size(); ++i) if (stem_of(bi[i].name) == n) { have[i] = true; e.builtin = (int)i; }
            items.push_back(e);
        }
        for (size_t i = 0; i < bi.size(); ++i) if (!have[i]) { LibEntry e; e.kind = LibEntry::Saved; e.name = stem_of(bi[i].name); e.label = e.name; e.builtin = (int)i; items.insert(items.begin() + std::min(items.size(), i), e); log.push_back(e.name + ": built-in copy (file unusable)"); }
        n_saved = (int)items.size();
        const int ids[4] = {RV_REMUS, RV_TETHYS, RV_LAUV, RV_ECA_A9};
        for (int id : ids) { LibEntry e; e.kind = LibEntry::Real; e.real_id = id; e.name = REAL_NAMES[id]; e.label = std::string("[real] ") + REAL_NAMES[id] + " (editable copy)"; items.push_back(e); }
        n_real = 4;
        LibEntry b; b.kind = LibEntry::DeployOnly; b.real_id = RV_BLUEROV2; b.name = REAL_NAMES[RV_BLUEROV2]; b.label = std::string("[real] ") + REAL_NAMES[RV_BLUEROV2] + " (deploy only)"; items.push_back(b);
    }
    int find(const std::string& name) const { for (size_t i = 0; i < items.size(); ++i) if (items[i].name == name) return (int)i; return -1; }
    std::string summary() const { return "Library: " + std::to_string(n_saved) + " designs, " + std::to_string(n_real) + " real vehicles"; }
};

// where the design in the editor came from
struct Source {
    int real_base = -1;          // >= 0: the design started as this real vehicle and its published numbers override the designer's
    bool overrides_on = true;    // false: use the designer's own numbers
    std::string loaded;          // library entry it was loaded from
};

// load an entry as a DesignIn. A real vehicle becomes a DesignIn of its hull, fins and propeller (the original data is never touched: Save writes a new file).
inline bool load_entry(const Library& lib, int idx, design::DesignIn& d, Source& src, std::string& err) {
    if (idx < 0 || idx >= (int)lib.items.size()) { err = "no such entry"; return false; }
    const LibEntry& e = lib.items[idx];
    if (e.kind == LibEntry::DeployOnly) { err = e.name + " is deploy-only: its vectored thrusters and frame are not a hull the Designer can describe."; return false; }
    if (e.kind == LibEntry::Saved) {
        if (!design::load_design(lib.dir + "/" + e.name + ".json", d)) {
            if (e.builtin < 0) { err = "cannot read " + e.name; return false; }
            d = design::builtin_designs()[e.builtin];                // the file is unusable: the built-in design is used
        }
        src = Source{}; src.loaded = e.name; return true;
    }
    d = real::design_in(e.real_id); d.name = e.name + " (copy)"; src.real_base = e.real_id; src.overrides_on = true; src.loaded = e.name; return true;
}

// the Spec of a design; with a real base the published numbers are written over the designer's, and `overridden` lists what changed
inline Spec build_spec(const design::DesignIn& d, const Source& src, design::DesignOut* out = nullptr, std::vector<std::string>* overridden = nullptr) {
    design::Designer des; design::DesignOut o = des.run(d); Spec s = design::to_spec(d, o);
    if (overridden) overridden->clear();
    if (src.real_base >= 0 && src.real_base < RV_BLUEROV2 && src.overrides_on) {
        Spec before = s; real::apply_published(src.real_base, s);
        if (overridden) *overridden = real::overridden_fields(before, s);
    }
    s.name = d.name; if (out) *out = o; return s;
}

// ------------------------------------------------------------------------------------------------ apply a design to a deployed vehicle
// Position, orientation, velocities, fin and pump positions and the tuning multipliers stay. The battery keeps its state of charge (the same percentage of the new capacity),
// because rebuild() would refill it. The thrust of thrusters that still exist is kept so there is no jerk.
inline void apply_to_vehicle(Vehicle& v, const Spec& ns) {
    double soc = v.s.battery_Wh > 0 ? std::max(0.0, std::min(1.0, v.energy_Wh / v.s.battery_Wh)) : 1.0;
    std::vector<double> th = v.thrust; size_t nold = v.s.thr.size();
    v.s = ns; v.rebuild();
    v.energy_Wh = soc * ns.battery_Wh; v.flat = v.energy_Wh <= 0.0;
    if (nold == v.s.thr.size()) for (size_t i = 0; i < th.size() && i < v.thrust.size(); ++i) { v.thrust[i] = th[i]; v.rot[i].thrust = th[i]; }
}

// ------------------------------------------------------------------------------------------------ "test this design": scripted runs in calm water, stepped a few seconds per frame
struct TestResult {
    bool done = false;
    double v_top = 0, P_main = 0, E_full_Wh = 0;             // phase 1: 30 s at full surge
    double v_cruise = 0, P_cruise = 0, P_drain = 0, endurance_h = 0;   // phase 2: speed held at the design's cruise speed
};
struct DesignTest {
    static constexpr double DT = 0.005, T1 = 30.0, T2 = 100.0, TAIL = 5.0, TAIL2 = 20.0;
    bool running = false; int phase = 0; double t = 0;
    Vehicle v; Env env; Spec spec; design::DesignIn in; TestResult r;
    double e0 = 0, vsum = 0, psum = 0, nsum = 0, isum = 0, integ = 0, rho = 1025;
    void start(const Spec& s, const design::DesignIn& d, double rho_water) {
        spec = s; in = d; rho = rho_water; r = TestResult{}; phase = 1; t = 0; running = true; begin_phase();
    }
    void begin_phase() {
        Spec sp = spec; if (phase == 2) sp.trim = 1025.0 / rho - 1.0;       // level flight: neutral buoyancy
        v = Vehicle{}; v.s = sp; v.rebuild(); v.place({0, 0, -20}, 0);
        env = Env{}; env.rho = rho; env.eta = [](double, double) { return 0.0; }; env.ground = [](double, double) { return -1000.0; };
        e0 = v.energy_Wh; vsum = psum = nsum = isum = 0; integ = 0; t = 0;
    }
    double progress() const { return !running ? (r.done ? 1.0 : 0.0) : (phase == 1 ? 0.0 + 0.3 * t / T1 : 0.3 + 0.7 * t / T2); }
    // advance by `secs` of simulated time (call a few times per frame)
    void advance(double secs) {
        if (!running) return;
        int n = (int)(secs / DT);
        for (int i = 0; i < n && running; ++i) {
            if (phase == 1) {
                v.cmd[0] = 1.0; v.step(DT, env); t += DT;
                if (t >= T1 - TAIL) { vsum += len(v.v); psum += v.rot.empty() ? 0.0 : v.rot[0].power; nsum += 1; }
                if (t >= T1) { r.v_top = vsum / std::max(nsum, 1.0); r.P_main = psum / std::max(nsum, 1.0); r.E_full_Wh = e0 - v.energy_Wh; phase = 2; begin_phase(); }
            } else {
                double e = in.v_cruise - v.v.x; integ = std::max(-1.0, std::min(1.0, integ + 0.25 * e * DT));
                v.cmd[0] = std::max(-1.0, std::min(1.0, 0.6 * e + integ)); v.step(DT, env); t += DT;
                if (t >= T2 - TAIL2) { if (nsum == 0) e0 = v.energy_Wh; vsum += v.v.x; psum += v.power_W - spec.hotel_W; isum += v.power_W; nsum += 1; }
                if (t >= T2) {
                    double used = e0 - v.energy_Wh, hours = nsum * DT / 3600.0;
                    r.v_cruise = vsum / std::max(nsum, 1.0); r.P_cruise = psum / std::max(nsum, 1.0); r.P_drain = hours > 0 ? used / hours : 0.0;
                    r.endurance_h = r.P_drain > 1e-6 ? 0.9 * spec.battery_Wh / r.P_drain : 0.0; r.done = true; running = false;
                }
            }
        }
    }
};

// ------------------------------------------------------------------------------------------------ what the Designer remembers between runs
struct DesignerState {
    bool open = false, result_open = false, pause_while_open = true;
    std::string design_json, lib_name, designs_dir; int real_base = -1; bool overrides_on = true; int tab = -1;
    Json to_json() const {
        Json j = Json::object(); j["open"] = Json::boolean(open); j["result_open"] = Json::boolean(result_open); j["design"] = Json::string(design_json);
        j["library_entry"] = Json::string(lib_name); j["real_base"] = Json::number(real_base); j["overrides_on"] = Json::boolean(overrides_on); j["tab"] = Json::number(tab);
        j["pause_while_open"] = Json::boolean(pause_while_open); j["designs_dir"] = Json::string(designs_dir); return j;
    }
    void from_json(const Json& j) {
        open = j.bool_or("open", false); result_open = j.bool_or("result_open", false); design_json = j.str_or("design", ""); lib_name = j.str_or("library_entry", "");
        pause_while_open = j.bool_or("pause_while_open", true); designs_dir = j.str_or("designs_dir", ""); real_base = (int)j.num_or("real_base", -1); overrides_on = j.bool_or("overrides_on", true); tab = (int)j.num_or("tab", -1);
    }
    bool save(const std::string& path, std::string& err) const { return save_json_file(path, to_json(), err); }
    bool load(const std::string& path, std::string& err) { Json j; if (!load_json_file(path, j, err)) return false; from_json(j); return true; }
};

}  // namespace dcore
