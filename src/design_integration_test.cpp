// design_integration_test.cpp - Step 10: the Designer integration logic (designer_core.hpp) without a GUI:
//   * the vehicle library (saved designs + real vehicles, deploy-only BlueROV2, files that are not designs are skipped)
//   * published overrides of a real vehicle on top of an edited design (and "off")
//   * Apply to deployed AUV keeps position, orientation, velocities, fin positions and the battery's percentage
//   * saved window state round trip (config/designer_state.json format)
//   * "Test this design" numbers for the built-in designs against the designer's own prediction
// Build: cmake target design_integration_test.   Run: ./design_integration_test
#include "design_test_common.hpp"
#include "designer_core.hpp"
#include <filesystem>
#include <fstream>
#include <unistd.h>

static std::string temp_dir() { std::string p = (std::filesystem::temp_directory_path() / ("auv_integration_" + std::to_string((long)getpid()))).string(); std::filesystem::create_directories(p); return p; }

int main() {
    namespace fs = std::filesystem; const std::string dir = temp_dir();
    // ---------------------------------------------------------------- library
    std::printf("== library\n");
    { std::ofstream f(dir + "/auv_preset.json"); f << "{ \"model\": 0, \"name\": \"AUV 1\", \"mass_scale\": 1.0 }\n"; }      // a 6A AUV page preset sharing the folder
    dcore::Library lib; lib.dir = dir; lib.refresh();
    int saved = 0, real = 0, deploy_only = 0; bool has_preset = false;
    for (const dcore::LibEntry& e : lib.items) { saved += e.kind == dcore::LibEntry::Saved; real += e.kind == dcore::LibEntry::Real; deploy_only += e.kind == dcore::LibEntry::DeployOnly; if (e.name == "auv_preset") has_preset = true; }
    std::printf("   %d entries: %d saved designs, %d real vehicles through the Designer, %d deploy-only\n", (int)lib.items.size(), saved, real, deploy_only);
    for (const dcore::LibEntry& e : lib.items) std::printf("     %s\n", e.label.c_str());
    CHECK(saved == (int)design::builtin_designs().size(), "expected %d saved designs, got %d", (int)design::builtin_designs().size(), saved);
    CHECK(real == 4 && deploy_only == 1, "expected 4 real vehicles + BlueROV2 deploy-only, got %d + %d", real, deploy_only);
    CHECK(!has_preset, "the 6A preset file in the same folder must not appear as a design");
    CHECK(lib.find(REAL_NAMES[RV_TETHYS]) >= 0 && lib.find(REAL_NAMES[RV_LAUV]) >= 0 && lib.find(REAL_NAMES[RV_ECA_A9]) >= 0 && lib.find(REAL_NAMES[RV_REMUS]) >= 0, "Tethys, LAUV, ECA A9 and REMUS must be listed");
    CHECK(lib.items.back().kind == dcore::LibEntry::DeployOnly && lib.items.back().label.find("deploy only") != std::string::npos, "BlueROV2 is last and says deploy only");
    { design::DesignIn d; dcore::Source s; std::string err; int ib = lib.find(REAL_NAMES[RV_BLUEROV2]);
      bool ok = dcore::load_entry(lib, ib, d, s, err); CHECK(!ok && err.find("deploy-only") != std::string::npos, "BlueROV2 must refuse to load into the Designer (%s)", err.c_str()); }
    // a saved design loads back unchanged; a real vehicle becomes an editable copy
    { design::DesignIn d; dcore::Source s; std::string err; int i0 = lib.find("REMUS_100_class");
      CHECK(i0 >= 0 && dcore::load_entry(lib, i0, d, s, err) && s.real_base < 0 && std::fabs(d.L - 1.6) < 1e-9, "saved REMUS design loads");
      int it = lib.find(REAL_NAMES[RV_TETHYS]); design::DesignIn t;
      CHECK(dcore::load_entry(lib, it, t, s, err) && s.real_base == RV_TETHYS && std::fabs(t.L - 2.3) < 1e-9 && t.name.find("(copy)") != std::string::npos, "Tethys loads as an editable copy (L %.2f, name %s)", t.L, t.name.c_str()); }

    // ---------------------------------------------------------------- the library does not depend on the working directory, and always has the built-ins
    std::printf("== library from different working directories\n");
    {
        const std::string root = dir + "/project"; fs::create_directories(root); setenv("AUV_PROJECT_ROOT", root.c_str(), 1);
        fs::path cwd0 = fs::current_path(); std::string expect = (fs::path(root) / "config" / "designs").lexically_normal().string();
        const char* builtin_names[4] = {"REMUS_100_class", "Mini_AUV_0_9_m", "Survey_AUV__sonar__contra_rotating_", "Hovering_inspection_AUV"};
        fs::create_directories(dir + "/elsewhere/deep"); std::vector<std::string> from = {root, dir + "/elsewhere", dir + "/elsewhere/deep"};
        for (const std::string& wd : from) {
            fs::current_path(wd); dcore::Library l; l.refresh();
            std::printf("   cwd %-34s -> folder %s : %d entries (%d designs + %d real + BlueROV2)\n", fs::relative(wd, dir).string().c_str(), l.dir.c_str(), (int)l.items.size(), l.n_saved, l.n_real);
            CHECK(l.dir == expect, "the designs folder must be the same from every directory: %s vs %s", l.dir.c_str(), expect.c_str());
            CHECK(l.items.size() >= 8, "at least 8 entries from %s, got %d", wd.c_str(), (int)l.items.size());
            for (const char* n : builtin_names) CHECK(l.find(n) >= 0, "built-in design %s missing when started from %s", n, wd.c_str());
            CHECK(l.find(REAL_NAMES[RV_TETHYS]) >= 0 && l.find(REAL_NAMES[RV_LAUV]) >= 0 && l.find(REAL_NAMES[RV_ECA_A9]) >= 0 && l.items.back().kind == dcore::LibEntry::DeployOnly, "real vehicles and BlueROV2 follow the designs");
            CHECK(wd == root || !fs::exists(fs::path(wd) / "config" / "designs"), "no stray config/designs may appear in the working directory %s", wd.c_str());
            CHECK(l.summary() == "Library: 4 designs, 4 real vehicles", "count line: %s", l.summary().c_str());
        }
        fs::current_path(cwd0);
        // deleted built-in: regenerated; edited built-in: untouched; junk and unusable files: skipped without hiding the others
        std::string d = expect;
        fs::remove(d + "/Mini_AUV_0_9_m.json");
        { design::DesignIn e = design::builtin_designs()[0]; e.L = 1.777; design::save_design(d + "/REMUS_100_class.json", e); }       // the user's edit
        { std::ofstream f(d + "/zz_not_json.json"); f << "this is not json {{{"; }
        { std::ofstream f(d + "/zz_preset.json"); f << "{ \"model\": 0, \"name\": \"AUV 1\" }"; }
        { std::ofstream f(d + "/Survey_AUV__sonar__contra_rotating_.json"); f << "garbage"; }                                         // a built-in name with an unusable file
        { std::ofstream f(d + "/zz_empty.json"); }
        dcore::Library l; l.refresh();
        for (const char* n : builtin_names) CHECK(l.find(n) >= 0, "built-in %s must be in the list after the damage", n);
        CHECK(fs::exists(d + "/Mini_AUV_0_9_m.json"), "the deleted built-in file is regenerated");
        design::DesignIn ed; CHECK(design::load_design(d + "/REMUS_100_class.json", ed) && std::fabs(ed.L - 1.777) < 1e-9, "the user's edit of a built-in file is left alone (L %.3f)", ed.L);
        CHECK(l.find("zz_not_json") < 0 && l.find("zz_preset") < 0 && l.find("zz_empty") < 0, "files that are not designs are skipped");
        design::DesignIn sv; dcore::Source sc; std::string err; int is = l.find("Survey_AUV__sonar__contra_rotating_");
        CHECK(is >= 0 && dcore::load_entry(l, is, sv, sc, err) && sv.props[0].contra, "an unusable file with a built-in name falls back to the built-in design (%s)", err.c_str());
        CHECK(l.skipped >= 3 && l.n_saved == 4, "%d skipped, %d designs", l.skipped, l.n_saved);
        std::printf("   after deleting, damaging and adding junk: %s, %d file(s) skipped\n", l.summary().c_str(), l.skipped);
        // migration from the old shared folder: designs move, 6A presets stay, a second run changes nothing
        fs::create_directories(root + "/config/auv"); design::save_design(root + "/config/auv/My_old_design.json", design::builtin_designs()[3]);
        { std::ofstream f(root + "/config/auv/My_6A_preset.json"); f << "{ \"model\": 0, \"name\": \"AUV 1\" }"; }
        int moved = paths::migrate_all(expect, root), again = paths::migrate_all(expect, root);
        CHECK(moved >= 1 && again == 0 && fs::exists(expect + "/My_old_design.json") && !fs::exists(root + "/config/auv/My_old_design.json") && fs::exists(root + "/config/auv/My_6A_preset.json"),
              "design moved to config/designs, the 6A preset stays in config/auv, second run moves nothing (%d, %d)", moved, again);
        std::printf("   migration: %d design file(s) moved to config/designs, 6A preset left in config/auv\n", moved);
        unsetenv("AUV_PROJECT_ROOT");
    }

    // ---------------------------------------------------------------- published overrides on a real base
    std::printf("== overrides\n");
    for (int id : {RV_REMUS, RV_TETHYS, RV_LAUV, RV_ECA_A9}) {
        design::DesignIn d; dcore::Source s; std::string err; dcore::load_entry(lib, lib.find(REAL_NAMES[id]), d, s, err);
        std::vector<std::string> over; Spec sp = dcore::build_spec(d, s, nullptr, &over); Spec pub = make_real_spec(id);
        std::printf("   %-34s mass %.2f kg (published %.2f), %d fields overridden, e.g. %s\n", REAL_NAMES[id], sp.mass, pub.mass, (int)over.size(), over.empty() ? "-" : over[0].c_str());
        CHECK(std::fabs(sp.mass - pub.mass) < 1e-6 * pub.mass, "%s: mass %.3f vs published %.3f", REAL_NAMES[id], sp.mass, pub.mass);
        CHECK(!over.empty(), "%s: no overridden fields listed", REAL_NAMES[id]);
        for (int k = 0; k < 6; ++k) CHECK(std::fabs(sp.A[k] - pub.A[k]) < 1e-9, "%s: added mass %d", REAL_NAMES[id], k);
        s.overrides_on = false; Spec own = dcore::build_spec(d, s, nullptr, &over);
        CHECK(over.empty() && std::fabs(own.mass - pub.mass) > 1e-3, "%s: with the overrides off the designer's own mass must differ (%.2f)", REAL_NAMES[id], own.mass);
        // editing the design: the hull follows the edit, the published mass and added mass stay
        s.overrides_on = true; design::DesignIn e = d; e.L *= 1.1; Spec ed = dcore::build_spec(e, s);
        CHECK(std::fabs(ed.length - 1.1 * sp.length) < 1e-9 && std::fabs(ed.mass - pub.mass) < 1e-6 * pub.mass, "%s: hull length follows the edit (%.3f m), published mass stays", REAL_NAMES[id], ed.length);
    }

    // ---------------------------------------------------------------- apply to a deployed vehicle
    std::printf("== apply to deployed AUV\n");
    {
        design::DesignIn a = design::builtin_designs()[0], b = design::builtin_designs()[3];     // REMUS-class -> hovering inspection (more thrusters, other battery)
        dcore::Source none; Spec sa = dcore::build_spec(a, none), sb = dcore::build_spec(b, none);
        Vehicle v; v.s = sa; v.rebuild(); v.place({12.5, -7.25, -33.0}, 0.9); Env env = Env{}; env.rho = 1025; env.eta = [](double, double) { return 0.0; }; env.ground = [](double, double) { return -500.0; };
        for (int i = 0; i < 2000; ++i) { v.cmd[0] = 0.7; v.cmd[4] = 0.3; v.step(0.005, env); }          // cruising and turning
        v.energy_Wh = 0.6 * sa.battery_Wh;                                                          // 60 % charge
        V3 p = v.p, vel = v.v, w = v.w; Q q = v.q; double d1 = v.defl[1], d2 = v.defl[2], thr0 = v.thrust[0]; double drag_scale = v.drag_scale = 1.3; v.am_scale = 0.9;
        dcore::apply_to_vehicle(v, sb);
        auto same = [](V3 x, V3 y) { return len(x - y) < 1e-12; };
        CHECK(same(v.p, p), "position kept"); CHECK(same(v.v, vel) && same(v.w, w), "velocities kept");
        CHECK(std::fabs(v.q.w - q.w) + std::fabs(v.q.x - q.x) + std::fabs(v.q.y - q.y) + std::fabs(v.q.z - q.z) < 1e-12, "orientation kept");
        CHECK(std::fabs(v.p.z - p.z) < 1e-12, "depth kept (%.2f m)", -v.p.z);
        CHECK(v.defl[1] == d1 && v.defl[2] == d2, "fin deflections kept");
        CHECK(v.drag_scale == drag_scale && v.am_scale == 0.9, "tuning multipliers kept");
        CHECK(std::fabs(v.energy_Wh / v.s.battery_Wh - 0.6) < 1e-12, "battery state of charge kept at 60 %% (%.3f)", v.energy_Wh / v.s.battery_Wh);
        CHECK(std::fabs(v.energy_Wh - 0.6 * sb.battery_Wh) < 1e-9 && v.s.battery_Wh == sb.battery_Wh, "new capacity %.0f Wh, charge %.1f Wh", v.s.battery_Wh, v.energy_Wh);
        CHECK(v.s.thr.size() == sb.thr.size() && v.thrust.size() == sb.thr.size() && v.s.mass == sb.mass, "new Spec in place: %d thrusters, mass %.2f", (int)v.s.thr.size(), v.s.mass);
        std::printf("   position %.2f %.2f %.2f, speed %.2f m/s, 60 %% charge kept (%.0f Wh -> %.0f Wh), thrusters %d -> %d\n", v.p.x, v.p.y, v.p.z, len(v.v), 0.6 * sa.battery_Wh, v.energy_Wh, (int)sa.thr.size(), (int)sb.thr.size());
        (void)thr0;
        // same thruster count: the thrust carries over without a jerk
        Vehicle v2; v2.s = sa; v2.rebuild(); v2.place({0, 0, -20}, 0); for (int i = 0; i < 1000; ++i) { v2.cmd[0] = 1.0; v2.step(0.005, env); }
        double t_before = v2.thrust[0]; design::DesignIn a2 = a; a2.battery_Wh = 2000; dcore::apply_to_vehicle(v2, dcore::build_spec(a2, none));
        CHECK(std::fabs(v2.thrust[0] - t_before) < 1e-12 && t_before > 1.0, "thrust carried over when the thruster count is unchanged");
        // an empty battery stays empty
        Vehicle v3; v3.s = sa; v3.rebuild(); v3.energy_Wh = 0; v3.flat = true; dcore::apply_to_vehicle(v3, sb); CHECK(v3.energy_Wh == 0 && v3.flat, "a flat battery stays flat");
        // and the vehicle still flies after the swap
        for (int i = 0; i < 4000; ++i) { v.cmd[0] = 1.0; v.step(0.005, env); } CHECK(len(v.v) > 0.5 && std::isfinite(v.p.x), "the vehicle keeps flying after the swap (%.2f m/s)", len(v.v));
    }

    // ---------------------------------------------------------------- saved state
    std::printf("== saved state\n");
    {
        design::DesignIn d = design::builtin_designs()[1]; d.L = 1.234; d.fins[0].count = 3;
        dcore::DesignerState st; st.open = true; st.result_open = true; st.design_json = design::to_json(d); st.lib_name = "Mini AUV 0.9 m"; st.real_base = RV_LAUV; st.overrides_on = false; st.tab = 2;
        std::string path = dir + "/config/designer_state.json", err; CHECK(st.save(path, err), "save: %s", err.c_str());
        dcore::DesignerState r; CHECK(r.load(path, err), "load: %s", err.c_str());
        design::DesignIn d2; CHECK(design::from_json(r.design_json, d2), "the design inside the state parses");
        CHECK(r.open && r.result_open && r.lib_name == "Mini AUV 0.9 m" && r.real_base == RV_LAUV && !r.overrides_on && r.tab == 2, "flags and names restored");
        CHECK(std::fabs(d2.L - 1.234) < 1e-9 && d2.fins[0].count == 3 && d2.name == d.name && std::fabs(d2.battery_Wh - d.battery_Wh) < 1e-9, "the design is restored exactly (L %.3f, %d fins)", d2.L, d2.fins[0].count);
        dcore::DesignerState miss; CHECK(!miss.load(dir + "/nothing.json", err) && !miss.open, "a missing file gives defaults");
        std::printf("   saved and restored %s (%d bytes)\n", path.c_str(), (int)fs::file_size(path));
    }

    // ---------------------------------------------------------------- "test this design" for the built-in designs
    std::printf("== test this design (scripted, stepped in slices like the GUI does)\n");
    std::printf("   %-38s %18s %18s %18s %18s %16s\n", "design", "top speed", "main power", "cruise power", "endurance", "cruise speed");
    for (const design::DesignIn& d : design::builtin_designs()) {
        dcore::Source none; design::DesignOut o; Spec sp = dcore::build_spec(d, none, &o);
        dcore::DesignTest t; t.start(sp, d, o.rho_water); int slices = 0; while (t.running && slices < 1000) { t.advance(5.0); ++slices; }
        const dcore::TestResult& r = t.r;
        CHECK(r.done && slices > 10 && slices < 100, "%s: finished in %d slices", d.name.c_str(), slices);
        auto pc = [](double a, double b) { return 100.0 * (a - b) / b; };
        std::printf("   %-38s %5.2f/%5.2f %+5.1f%% %5.1f/%5.1f %+5.1f%% %5.1f/%5.1f %+5.1f%% %5.1f/%5.1f %+5.1f%% %5.2f/%4.2f\n", d.name.c_str(), r.v_top, o.v_top, pc(r.v_top, o.v_top), r.P_main, o.P_elec_max, pc(r.P_main, o.P_elec_max),
                    r.P_cruise, o.P_elec_cruise, pc(r.P_cruise, o.P_elec_cruise), r.endurance_h, o.endurance_h, pc(r.endurance_h, o.endurance_h), r.v_cruise, d.v_cruise);
        CHECK(std::fabs(r.v_top - o.v_top) < 0.10 * o.v_top, "%s: top speed %.3f vs %.3f", d.name.c_str(), r.v_top, o.v_top);
        CHECK(std::fabs(r.P_main - o.P_elec_max) < 0.10 * o.P_elec_max, "%s: main power %.1f vs %.1f", d.name.c_str(), r.P_main, o.P_elec_max);
        CHECK(std::fabs(r.v_cruise - d.v_cruise) < 0.05 * d.v_cruise, "%s: cruise speed %.3f vs %.3f", d.name.c_str(), r.v_cruise, d.v_cruise);
        CHECK(std::fabs(r.P_cruise - o.P_elec_cruise) < 0.10 * o.P_elec_cruise, "%s: cruise power %.1f vs %.1f", d.name.c_str(), r.P_cruise, o.P_elec_cruise);
        CHECK(std::fabs(r.endurance_h - o.endurance_h) < 0.10 * o.endurance_h, "%s: endurance %.2f vs %.2f", d.name.c_str(), r.endurance_h, o.endurance_h);
        CHECK(r.E_full_Wh > 0, "%s: energy used at full surge", d.name.c_str());
    }
    fs::remove_all(dir);
    std::printf("\n%d checks, %d failed\n", g_checks, g_fails);
    return g_fails ? 1 : 0;
}
