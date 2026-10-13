// design_io_test.cpp - Step 8C-0: save / load, sensors, battery pack and motor, water types, drawing data.
// Build: g++ -std=c++17 -O1 -Isrc design_io_test.cpp -o design_io_test      Run: ./design_io_test
#include "design_io.hpp"
using namespace design;
static int fails = 0;
static void check(const char* what, double got, double lo, double hi, const char* unit) {
    bool ok = got >= lo - 1e-9 && got <= hi + 1e-9; if (!ok) ++fails;
    std::printf("  [%s] %-58s %10.3f %-6s (expected %.4g .. %.4g)\n", ok ? " OK " : "FAIL", what, got, unit, lo, hi);
}
static void yes(const char* what, bool c) { if (!c) ++fails; std::printf("  [%s] %s\n", c ? " OK " : "FAIL", what); }
static bool has_warning(const DesignOut& o, const char* key) { for (const std::string& w : o.warnings) if (w.find(key) != std::string::npos) return true; return false; }

int main() {
    Designer ds;
    std::printf("\n1. SAVE AND LOAD\n");
    std::vector<DesignIn> all = builtin_designs();
    yes("four built-in example designs", all.size() == 4);
    for (const DesignIn& d : all) {
        DesignIn r; bool ok = from_json(to_json(d), r);
        DesignOut a = ds.run(d), b = ds.run(r);
        yes((d.name + ": text -> design -> text is identical").c_str(), ok && to_json(r) == to_json(d));
        yes((d.name + ": the loaded design gives the same physics").c_str(), std::fabs(a.mass - b.mass) < 1e-9 && std::fabs(a.endurance_h - b.endurance_h) < 1e-9 && a.fin_rows.size() == b.fin_rows.size());
        yes((d.name + ": can be built (not too heavy)").c_str(), a.ok);
        yes((d.name + ": balanced fore and aft (CG under CB within 1 cm)").c_str(), std::fabs(a.rg.x) < 0.01);
    }
    { std::string dir = "/tmp/auv_io_test"; std::filesystem::remove_all(dir); int w = write_builtin_designs(dir); int w2 = write_builtin_designs(dir);
      yes("writes the examples once and never overwrites", w == 4 && w2 == 0 && list_designs(dir).size() == 4);
      DesignIn r; yes("loads one back from disk", load_design(dir + "/Mini_AUV_0_9_m.json", r) && r.L == 0.9); }
    { DesignIn r; yes("rejects broken JSON", !from_json("{\"hull\": {", r)); yes("accepts a file with missing fields (uses defaults)", from_json("{\"name\":\"x\"}", r) && r.L == 1.6); }

    std::printf("\n2. SENSORS ADD MASS, POWER AND DRAG\n");
    DesignIn base = default_design(); DesignOut b = ds.run(base);
    { DesignIn d = base; d.sensors = {SE_MBES, SE_SSS, SE_FLS, SE_DVL}; DesignOut o = ds.run(d);
      check("sensor mass of a sonar suite", o.sensors_kg, 7.0, 8.5, "kg"); check("sensor power", o.sensors_W, 45, 50, "W");
      yes("more power: shorter endurance", o.endurance_h < b.endurance_h); yes("sensors stick out: more drag", o.drag_cruise > b.drag_cruise);
      yes("total mass stays balanced (ballast shrinks)", std::fabs(o.net_up / o.W - base.trim) < 0.003 && o.ballast < b.ballast); }

    std::printf("\n3. BATTERY PACK AND MOTOR\n");
    check("pack voltage of the default (6 cells in series)", b.pack.V, 22.0, 22.4, "V");
    check("pack capacity 1000 Wh / 22.2 V", b.pack.Ah, 44.0, 46.0, "Ah");
    check("motor Kv chosen by the engine", b.pack.kv, 150, 400, "rpm/V");
    yes("a gearbox ratio is chosen", b.pack.gear >= 1.0 && b.pack.gear <= 40.0);
    { DesignIn d = base; d.battery_Wh = 60; d.cell_maxC = 0.5; DesignOut o = ds.run(d); yes("tiny battery for the load: peak-current warning", !o.pack.ok && has_warning(o, "peak current")); }
    { DesignIn d = base; d.bus_V = 48; DesignOut o = ds.run(d); yes("higher bus voltage: lower current", o.pack.I_max < b.pack.I_max); }

    std::printf("\n4. WATER TYPES\n");
    { DesignIn d = base; d.water = 2; DesignOut o = ds.run(d); check("same vehicle, fresh water: buoyancy lower by about 2.5 %", o.B / b.B, 0.970, 0.980, "");
      yes("ballast re-calculated: net up is still the wanted 2 %", std::fabs(o.net_up / o.W - base.trim) < 0.003); }
    yes("sea design warns that it sinks in a lake", has_warning(b, "fresh water"));
    check("net up in fresh water for the sea design", b.net_up_fresh, -6, 0, "N");

    std::printf("\n5. DRAWING DATA FOR THE GUI\n");
    yes("hull outline has points from tail to nose", b.outline.size() == 121 && b.outline.front().first < b.outline.back().first);
    yes("outline radius never exceeds the hull radius", [&] { for (auto& p : b.outline) if (p.second > 0.5 * base.D + 1e-9) return false; return true; }());
    yes("one drawing entry per fin and per propeller", b.fin_draw.size() == 4 && b.prop_draw.size() == 1);
    std::printf("\n%s (%d failed)\n", fails ? "SOME CHECKS FAILED" : "ALL CHECKS PASSED", fails);
    return fails;
}
