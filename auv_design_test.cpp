// auv_design_test.cpp - Step 8A: the design engine against the REMUS 100 data sheet, and the way the answers change when you change the design.
// Build: g++ -std=c++17 -O1 -Isrc auv_design_test.cpp -o auv_design_test      Run: ./auv_design_test
#include "auv_design.hpp"
using namespace design;
static int fails = 0;
static void check(const char* what, double got, double lo, double hi, const char* unit) {
    bool ok = got >= lo && got <= hi; if (!ok) ++fails;
    std::printf("  [%s] %-52s %10.3f %-6s (expected %.4g .. %.4g)\n", ok ? " OK " : "FAIL", what, got, unit, lo, hi);
}
static void yes(const char* what, bool c) { if (!c) ++fails; std::printf("  [%s] %s\n", c ? " OK " : "FAIL", what); }
static bool has_warning(const DesignOut& o, const char* key) { for (const std::string& w : o.warnings) if (w.find(key) != std::string::npos) return true; return false; }

int main() {
    Designer ds;
    DesignIn base = default_design(); DesignOut b = ds.run(base);
    std::printf("\n1. DEFAULT DESIGN vs REMUS 100 DATA SHEET (1.6 m x 190 mm, 37 kg, 22 h at 1.5 m/s, 2.6 m/s top, 100 m)\n");
    check("mass", b.mass, 33, 42, "kg");
    check("hull volume", b.V, 0.034, 0.042, "m3");
    check("wall thickness for 100 m, aluminium", b.wall_t * 1000, 3.0, 7.0, "mm");
    check("drag coefficient Xu|u| (Prestero bare hull 1.62)", b.Xuu, 1.6, 4.0, "kg/m");
    check("top speed", b.v_top, 2.3, 2.9, "m/s");
    check("endurance at 1.5 m/s", b.endurance_h, 15, 30, "h");
    check("electric power at 1.5 m/s", b.P_elec_cruise, 10, 40, "W");
    check("roll period (Prestero model 1.1 s)", b.T_roll, 0.5, 1.5, "s");
    yes("a design that floats: buoyancy about 2 % over weight", std::fabs(b.net_up / b.W - base.trim) < 0.003);
    check("centre of gravity fore/aft", b.rg.x, -0.01, 0.01, "m");

    std::printf("\n2. CHANGE THE DESIGN, THE PHYSICS FOLLOWS\n");
    { DesignIn d = base; d.D *= 1.25; DesignOut o = ds.run(d); yes("wider hull: more volume, more drag", o.V > b.V * 1.4 && o.drag_cruise > b.drag_cruise); }
    { DesignIn d = base; d.L *= 1.25; DesignOut o = ds.run(d); yes("longer hull: more volume, more room, slower roll", o.V > b.V * 1.2 && o.Lcyl > b.Lcyl); }
    { DesignIn d = base; d.battery_Wh *= 2; DesignOut o = ds.run(d); check("double the battery: endurance ratio", o.endurance_h / b.endurance_h, 1.8, 2.05, ""); yes("... and the ballast gets smaller", o.ballast < b.ballast); }
    { DesignIn d = base; d.fins[0].count = 6; DesignOut o = ds.run(d); check("6 fins instead of 4: fin area ratio", o.fin_area_total / b.fin_area_total, 1.45, 1.55, ""); yes("... more drag", o.drag_cruise > b.drag_cruise); }
    { DesignIn d = base; d.fins[0].span *= 1.6; DesignOut o = ds.run(d); yes("bigger fins: better directional stability", o.stability_ratio > 2 * b.stability_ratio); yes("... and tighter turns", o.turn_radius < b.turn_radius); }
    { DesignIn d = base; d.fins[0].on = false; DesignOut o = ds.run(d); yes("no fins: warning, no turn authority", has_warning(o, "No fins") && o.fin_rows.empty()); }
    { DesignIn d = base; d.fins[0].control = false; DesignOut o = ds.run(d); yes("fixed fins only: warning that nothing steers", has_warning(o, "No moving fins")); }
    { DesignIn d = base; PropSpec v; v.role = 1; v.D = 0.07; v.rpm = 3000; v.x = 0.3; d.props.push_back(v);
      DesignOut o = ds.run(d); yes("add a vertical thruster: heave thrust appears", b.cap_heave == 0 && o.cap_heave > 10); yes("... and its mass is counted", o.items.size() > b.items.size()); }
    { DesignIn d = base; d.material = 1; DesignOut o = ds.run(d); yes("carbon fibre hull: lighter shell (more ballast needed)", o.items[0].m < b.items[0].m * 0.8 && o.ballast > b.ballast); }
    { DesignIn d = base; d.rated_depth = 300; DesignOut o = ds.run(d); yes("300 m rating: thicker wall", o.wall_t > 1.3 * b.wall_t); }
    { DesignIn d = base; d.v_max = 3.5; DesignOut o = ds.run(d); yes("faster target: more rpm and more power", o.rpm_main > b.rpm_main && o.P_elec_max > b.P_elec_max); }
    { DesignIn d = base; d.props.clear(); DesignOut o = ds.run(d); yes("no propeller: warning", has_warning(o, "No main propeller")); }
    { DesignIn d = base; d.L = 1.0; d.payload_kg = 15; d.battery_Wh = 2000; DesignOut o = ds.run(d); yes("overloaded: too heavy, not ok, no room", !o.ok && has_warning(o, "TOO HEAVY") && has_warning(o, "Not enough room")); }
    { DesignIn d = base; d.L = 0.9; d.D = 0.12; d.battery_Wh = 200; d.payload_kg = 0.5; d.electronics_kg = 0.6; d.rated_depth = 50; d.props[0].D = 0.11; d.v_max = 1.5; d.v_cruise = 1.0; d.fins[0].span = 0.06; d.fins[0].cr = 0.08; d.fins[0].ct = 0.05; d.fins[0].x = -0.35;
      DesignOut o = ds.run(d); check("mini AUV mass", o.mass, 5, 12, "kg"); yes("mini AUV works", o.ok && o.v_top > 1.4); }
    std::printf("\n%s (%d failed)\n", fails ? "SOME CHECKS FAILED" : "ALL CHECKS PASSED", fails);
    return fails;
}
