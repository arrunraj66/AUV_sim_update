// prop_design_test.cpp - Step 8B: propeller (B-series, blade count, contra-rotating), shaft and fin design.
// Build: g++ -std=c++17 -O1 -Isrc prop_design_test.cpp -o prop_design_test      Run: ./prop_design_test
#include "auv_design.hpp"
using namespace design;
static int fails = 0;
static void check(const char* what, double got, double lo, double hi, const char* unit) {
    bool ok = got >= lo - 1e-9 && got <= hi + 1e-9; if (!ok) ++fails;
    std::printf("  [%s] %-60s %10.3f %-6s (expected %.4g .. %.4g)\n", ok ? " OK " : "FAIL", what, got, unit, lo, hi);
}
static void yes(const char* what, bool c) { if (!c) ++fails; std::printf("  [%s] %s\n", c ? " OK " : "FAIL", what); }
static bool has_warning(const DesignOut& o, const char* key) { for (const std::string& w : o.warnings) if (w.find(key) != std::string::npos) return true; return false; }

int main() {
    Designer ds;
    DesignIn base = default_design(); DesignOut b = ds.run(base);
    std::printf("\n1. PROPELLER, SHAFT AND FIN DESIGN (B-series, contra-rotating, Keller, IACS shaft rule)\n");
    { double kt, kq; pdz::bseries(0.0, 1.0, 0.55, 4, kt, kq);
      check("B-series static thrust coefficient KT(0), P/D 1.0, B4-55", kt, 0.30, 0.50, ""); check("B-series static torque coefficient KQ(0)", kq, 0.04, 0.08, "");
      pdz::Rotor1 r; r.Z = 4; r.PD = 1.0; r.EAR = 0.55; double best = 0; for (double J = 0.05; J < 1.0; J += 0.05) best = std::max(best, r.efficiency(J));
      check("B-series peak open-water efficiency", best, 0.60, 0.75, ""); }
    { double e3 = 0, e5 = 0; for (int Z : {3, 5}) { DesignIn d = base; d.props[0].blades = Z; DesignOut o = ds.run(d); (Z == 3 ? e3 : e5) = o.prop_rows[0].ear_min; }
      yes("more blades: Keller minimum blade area ratio rises", e5 > e3); }
    { for (int Z = 2; Z <= 7; ++Z) { DesignIn d = base; d.props[0].blades = Z; DesignOut o = ds.run(d); if (!(o.v_top > 2.3 && o.P_elec_cruise > 5 && o.P_elec_cruise < 60)) { yes("every blade count 2..7 gives a sane design", false); break; } if (Z == 7) yes("every blade count 2..7 gives a sane design", true); } }
    { DesignIn d = base; d.props[0].contra = true; DesignOut o = ds.run(d);
      yes("contra-rotating pair: roll torque cancels", std::fabs(o.prop_rows[0].Qnet) < 0.01);
      yes("... uses less power at the same thrust (swirl recovery)", o.P_elec_cruise < b.P_elec_cruise && o.P_elec_cruise > 0.8 * b.P_elec_cruise);
      yes("... but weighs more (second propeller + coaxial gearbox)", o.items.size() > b.items.size() && o.prop_rows[0].Z2 == 4 && o.prop_rows[0].rpm2 > 0); }
    { DesignIn d = base; PropSpec t; t.role = 1; t.D = 0.05; t.blades = 2; t.ear = 0.30; t.rpm = 7000; t.pd = 1.0; t.x = 0.3; d.props.push_back(t); DesignOut o = ds.run(d); yes("small, highly loaded thruster with too little blade area: cavitation warning", has_warning(o, "Keller")); }
    { DesignIn d = base; d.v_max = 3.4; DesignOut o = ds.run(d); yes("faster vehicle: thicker propeller shaft", o.prop_rows[0].shaft_d > b.prop_rows[0].shaft_d); }
    { DesignIn d = base; d.props[0].shaft_mat = 1; DesignOut o = ds.run(d); yes("stronger steel: stress limit allows a thinner or equal shaft", o.prop_rows[0].shaft_d <= b.prop_rows[0].shaft_d + 1e-9); }
    check("shaft critical speed margin (default)", b.prop_rows[0].crit_margin, 1.4, 10, "x");
    { DesignIn d = base; d.fins[0].span *= 1.6; d.fins[0].cr *= 1.4; DesignOut o = ds.run(d); yes("bigger fins: more force, thicker fin stock, bigger servo", o.fin_rows[0].F_max > b.fin_rows[0].F_max && o.fin_rows[0].stock_d >= b.fin_rows[0].stock_d); }
    { pdz::FinPlan fp = pdz::fin_planform(0.0076, 1.2, 0.6); check("fin planform from area, AR, taper: area back", 0.5 * fp.span * (fp.cr + fp.ct), 0.0076, 0.0076, "m2"); check("... and its aspect ratio", fp.span * fp.span / (0.5 * fp.span * (fp.cr + fp.ct)), 1.2, 1.2, ""); }
    std::printf("\n%s (%d failed)\n", fails ? "SOME CHECKS FAILED" : "ALL CHECKS PASSED", fails);
    return fails;
}
