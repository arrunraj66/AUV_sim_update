// real_models_test.cpp - the five real vehicles of real_models.hpp: static checks, thruster allocation, a 30 s full-surge run compared with the
// published top speed, and (part D) the Step 7A REMUS preset against Fossen's published REMUS 100 numbers.
// Differences above 25 % are flagged with the likely cause; nothing is tuned to hide them. Run: ./real_models_test
#include "design_test_common.hpp"
#include "real_models.hpp"
#include "models.hpp"
#include "remus100.hpp"

static const double DT = 0.005;
struct Wrench { double F[3], M[3]; };

// full command on one axis with the hull held still: the wrench the thrusters make about the body origin
static Wrench still_wrench(const Spec& s, int axis, double rho) {
    Vehicle v; v.s = s; v.rebuild(); v.place({0, 0, -20}, 0); v.assist = false; Env env = calm_water(rho);
    for (int i = 0; i < 1200; ++i) { v.cmd[axis] = 1.0; v.step(DT, env); v.v = {}; v.w = {}; }
    Wrench w{{0, 0, 0}, {0, 0, 0}};
    for (size_t i = 0; i < s.thr.size(); ++i) {
        V3 f = s.thr[i].dir * v.thrust[i], m = cross(s.thr[i].pos, f);
        w.F[0] += f.x; w.F[1] += f.y; w.F[2] += f.z; w.M[0] += m.x; w.M[1] += m.y; w.M[2] += m.z;
    }
    return w;
}
struct Top { double speed, pitch_deg, energy_Wh; };
static Top full_surge(const Spec& s, double rho, double secs = 30.0) {
    Vehicle v; v.s = s; v.rebuild(); v.place({0, 0, -20}, 0); Env env = calm_water(rho); double e0 = v.energy_Wh, sum = 0; int n = (int)(secs / DT), k = 0;
    for (int i = 0; i < n; ++i) { v.cmd[0] = 1.0; v.step(DT, env); if (i >= n - 1000) { sum += len(v.v); ++k; } }
    double yw, pt, rl; v.heading_pitch_roll(yw, pt, rl); return {sum / k, pt * 57.2958, e0 - v.energy_Wh};
}
// closed-loop yaw rate (PI on the rudder), speed held: the largest rate the rudder can hold before it reaches its stop
static bool hold_rate(const Spec& s, double rho, double u_target, double r_target, double& rate, double& defl) {
    Vehicle v; v.s = s; v.rebuild(); v.place({0, 0, -60}, 0); v.assist = false; Env env = calm_water(rho); SpeedHold sh; sh.target = u_target; double ir = 0, rs = 0, ds = 0; int k = 0, n = (int)(60 / DT);
    for (int i = 0; i < n; ++i) {
        double e = r_target - v.w.z; ir = std::max(-1.0, std::min(1.0, ir + 0.5 * e * DT));
        v.cmd[0] = sh.update(v.v.x, DT); v.cmd[4] = std::max(-1.0, std::min(1.0, 1.5 * e + ir)); v.step(DT, env);
        if (i >= n - (int)(8 / DT)) { rs += v.w.z; ds += std::fabs(v.defl[1]); ++k; }
    }
    rate = rs / k; defl = ds / k; return std::fabs(rate - r_target) < 0.03 * r_target + 0.005 && defl < 0.985;
}
static double max_yaw_rate(const Spec& s, double rho, double u) {
    double lo = 0.02, hi = 2.0, r, d, best = 0; if (!hold_rate(s, rho, u, lo, r, d)) return 0; best = r;
    for (int i = 0; i < 11; ++i) { double mid = 0.5 * (lo + hi); if (hold_rate(s, rho, u, mid, r, d)) { lo = mid; best = r; } else hi = mid; }
    return best;
}

int main() {
    const char* axn[6] = {"surge", "sway", "heave", "roll", "pitch", "yaw"};
    struct Row { std::string who; double sim, pub; };
    std::vector<Row> speeds; std::vector<std::string> causes;
    for (int id = 0; id < RV_COUNT; ++id) {
        const real::Info& I = real::info(id); Spec s = make_real_spec(id); Vehicle v; v.s = s; v.rebuild(); const double rho = 1025.0;
        std::printf("== %s  [%s]\n", s.name.c_str(), I.licence);
        // ---- mass, stability, trim
        double BG = -s.rg.z;
        std::printf("   mass %.2f kg (source %.2f), BG %.3f m (CG %s the CB), trim %+.2f %%, thrusters %d, fins %d\n", s.mass, I.mass_pub, BG, BG > 0 ? "below" : "NOT below", 100 * s.trim, (int)s.thr.size(), (int)s.fins.size());
        CHECK(std::fabs(s.mass - I.mass_pub) < 0.005 * I.mass_pub, "mass %.3f vs published %.3f", s.mass, I.mass_pub);
        CHECK(BG > 0, "CG must be below the CB (BG %.3f m)", BG);
        CHECK(std::fabs(s.trim) < 0.03, "trim %.3f is not near neutral", s.trim);
        CHECK(s.battery_Wh > 0 && s.rated_depth > 0 && s.hotel_W >= 0, "battery / depth / hotel load not filled");
        // ---- thruster allocation
        std::printf("   thruster capability (full command, hull held still): ");
        double cap[6]; for (int k = 0; k < 6; ++k) cap[k] = v.cap[k];
        for (int k : {0, 1, 2, 5, 3, 4}) std::printf("%s %.1f%s  ", axn[k], cap[k], k >= 3 ? " N m" : " N");
        std::printf("\n");
        CHECK(cap[0] > 0, "no surge thrust");
        if (!I.rov) {
            std::printf("   AUV: single thruster gives surge only; yaw and pitch come from the fins (%s rudders, %s elevators), no sway or heave thrust\n", v.has_rud ? "has" : "NO", v.has_elev ? "has" : "NO");
            CHECK(v.has_rud && v.has_elev, "fins should give yaw and pitch control");
            CHECK(cap[1] == 0 && cap[2] == 0, "an AUV with one propeller must not have sway or heave thrust");
        } else {
            for (int k : {0, 1, 2, 5}) CHECK(cap[k] > 0, "ROV has no %s authority", axn[k]);
            const char* name[3] = {"surge", "sway", "heave"}; int ax[3] = {0, 1, 2};
            for (int i = 0; i < 3; ++i) {
                Wrench w = still_wrench(s, ax[i], rho); double main = std::fabs(w.F[ax[i]]);
                double cross_f = 0, cross_m = 0; for (int j = 0; j < 3; ++j) { if (j != ax[i]) cross_f = std::max(cross_f, std::fabs(w.F[j])); cross_m = std::max(cross_m, std::fabs(w.M[j])); }
                std::printf("   full %s command: force %.1f N (capability %.1f N), largest cross force %.2f N, largest moment %.2f N m\n", name[i], main, cap[ax[i]], cross_f, cross_m);
                CHECK(main > 0.6 * cap[ax[i]], "%s command gives %.1f N, capability %.1f N", name[i], main, cap[ax[i]]);
                CHECK(cross_f < 0.05 * main + 0.5, "%s command leaks %.2f N into another axis", name[i], cross_f);
            }
            Wrench wy = still_wrench(s, 4, rho);
            std::printf("   full yaw command: moment %.2f N m (capability %.2f), net force %.2f N\n", std::fabs(wy.M[2]), cap[5], std::max(std::fabs(wy.F[0]), std::fabs(wy.F[1])));
            CHECK(std::fabs(wy.M[2]) > 0.6 * cap[5], "yaw command gives %.2f N m, capability %.2f", std::fabs(wy.M[2]), cap[5]);
            std::printf("   roll: %.1f N m of differential vertical thrust exists, but the commands (surge, sway, heave, pitch, yaw) have no roll input, so roll is held by the righting moment;\n"
                        "   pitch: only %.2f N m of thruster authority (the vertical thrusters are %.4f m from the CG): no usable pitch control. Both are as the published layout allows.\n", cap[3], cap[4], s.thr[4].pos.x - s.rg.x);
        }
        // ---- 30 s full surge vs the published speed
        Top t = full_surge(s, rho);
        double d = t.speed < I.v_lo ? rel_pct(t.speed, I.v_lo) : (t.speed > I.v_hi ? rel_pct(t.speed, I.v_hi) : 0.0);
        std::printf("   30 s full surge: %.2f m/s, pitch %+.1f deg, %.2f Wh used; published %.2f to %.2f m/s (%s) -> %+.1f %%%s\n", t.speed, t.pitch_deg, t.energy_Wh, I.v_lo, I.v_hi, I.v_src, d, std::fabs(d) > 25 ? "   <-- FLAG > 25 %" : "");
        speeds.push_back({s.name, t.speed, 0.5 * (I.v_lo + I.v_hi)});
        CHECK(t.speed > 0.2 && std::fabs(t.pitch_deg) < 30, "speed %.2f pitch %.1f", t.speed, t.pitch_deg);
        if (std::fabs(d) > 25) { ++g_flags; }
        if (id == RV_BLUEROV2) {   // the source's published damping is linear only
            Spec so = s; for (int k = 0; k < 6; ++k) so.Dq[k] = 0; Top t0 = full_surge(so, rho);
            double d0 = rel_pct(t0.speed, I.v_hi);
            std::printf("   with the published damping only (the source's quadratic terms are all zero): %.2f m/s -> %+.1f %%%s. The linear terms are low-speed values from a thesis; a real frame needs quadratic drag, which is why it was estimated.\n", t0.speed, d0, std::fabs(d0) > 25 ? "   <-- FLAG > 25 %" : "");
            if (std::fabs(d0) > 25) ++g_flags;
        }
        std::printf("   estimated: %s\n", I.estimated);
    }

    // ---- part D: the Step 7A REMUS preset against Fossen's reference numbers
    std::printf("\n== D. REMUS 100: Step 7A preset (models.hpp) vs Fossen reference data (remus100.py, MIT)\n");
    Spec a = make_spec_ex(M_TORPEDO), f = make_real_spec(RV_REMUS); const double rho = 1025.0;
    struct D { const char* what; double step7a, fossen; const char* unit; };
    std::vector<D> rows;
    rows.push_back({"mass", a.mass, f.mass, "kg"});
    rows.push_back({"added mass surge A11", a.A[0], f.A[0], "kg"});
    rows.push_back({"added mass sway A22", a.A[1], f.A[1], "kg"});
    rows.push_back({"added mass roll A44", a.A[3], f.A[3], "kg m2"});
    rows.push_back({"added mass pitch A55", a.A[4], f.A[4], "kg m2"});
    rows.push_back({"surge drag at 1.5 m/s", a.axial_drag(1.5), f.axial_drag(1.5), "N"});
    rows.push_back({"surge drag at 2.5 m/s", a.axial_drag(2.5), f.axial_drag(2.5), "N"});
    rows.push_back({"sway drag at 0.3 m/s", a.Dq[1] * 0.09 + a.Dl[1] * 0.3, f.Dq[1] * 0.09 + f.Dl[1] * 0.3, "N"});
    Top ta = full_surge(a, rho), tf = full_surge(f, rho);
    rows.push_back({"top speed, 30 s full surge", ta.speed, tf.speed, "m/s"});
    double ra = max_yaw_rate(a, rho, 1.5) * 57.2958, rf = max_yaw_rate(f, rho, 1.5) * 57.2958;
    rows.push_back({"largest held yaw rate at 1.5 m/s (rudder at the stop)", ra, rf, "deg/s"});
    std::printf("   (a yaw rate of 0.0 for Fossen means the controller could not hold even 1 deg/s: with CL 0.5 on 0.0133 m2 the rudder is far too weak for the hull's Munk moment)\n");
    std::printf("   %-52s %12s %12s %9s\n", "quantity", "Step 7A", "Fossen", "diff");
    for (const D& r : rows) { double dd = rel_pct(r.step7a, r.fossen); bool fl = r.fossen == 0 || std::fabs(dd) > 25; if (fl) ++g_flags; std::printf("   %-52s %7.2f %-5s %6.2f %-5s %+8.1f %%%s\n", r.what, r.step7a, r.unit, r.fossen, r.unit, r.fossen != 0 ? dd : 0.0, fl ? "  <-- FLAG > 25 %" : ""); }
    std::printf("   Fossen's own design limit for the yaw rate is r_max = 5 deg/s (Nomoto reference); published top speed 2.5 m/s; Step 7A mass is the real REMUS 31.9 kg class value (30.48 kg).\n");
    CHECK(std::fabs(a.mass - f.mass) < 0.1 * f.mass, "masses differ by more than 10 %%");

    // ---- part D2: Step 7A preset and the designer's REMUS preset against Prestero's thesis
    // T. Prestero, "Verification of a Six-Degree of Freedom Simulation Model for the REMUS Autonomous Underwater Vehicle", MIT/WHOI Joint Program, M.S. thesis,
    // Sept 2001, https://dspace.mit.edu/handle/1721.1/65068 (numbers only). Values: Table 2.2 (fins), 2.3 (W 299 N, B 306 N), 2.5 (CG 1.96 cm below the CB), 2.6 (inertia),
    // 4.2 / 4.3 and C.1 to C.10 (coefficients; Appendix C values include the adjustment factors of Table 8.2: sway and heave x10, pitch x12.5, yaw x10, roll resistance x100).
    // The thesis is inconsistent about the axial drag: Table C.1 gives Xuu = 1.62 kg/m (Cd 0.11, empirical estimate) while the text adopts the tow-tank value Cd = 0.267, Af = 0.0285 m2
    // (Xuu = 3.92 kg/m); both rows are shown. Likewise Table C.10 gives the rudder lift as 9.64 kg/(m rad) per fin pair, where rho CLa S_fin with CLa 3.12 would give 21.4.
    std::printf("\n== D2. REMUS 100 against Prestero (MIT/WHOI 2001): Step 7A preset, the designer's preset, Fossen\n");
    design::DesignIn din = design::builtin_designs()[0]; design::Designer des; design::DesignOut od = des.run(din); Spec dsp = design::to_spec(din, od);
    auto pairlift = [&](const Spec& sp, int group) { double c = 0; for (const Fin& fn : sp.fins) if (fn.group == group) c += 0.5 * 1025.0 * fn.area * fn.cl_alpha; return c; };   // kg/(m rad) per pair
    auto arm = [&](const Spec& sp) { return sp.fins.empty() ? 0.0 : sp.fins[0].pos.x; };
    auto maxd = [&](const Spec& sp) { double m = 0; for (const Fin& fn : sp.fins) m = std::max(m, fn.max_defl); return m * 57.2958; };
    auto effx = [&](const Spec& sp) { return sp.axial_drag(1.5) / 2.25; };      // surge drag at 1.5 m/s over u^2
    Spec old7 = make_spec(M_TORPEDO);       // the preset as it was before the Prestero values (models.hpp, unchanged)
    struct P { const char* what; double s7, des, fos, pre; const char* unit; double old = 0; };
    std::vector<P> pr = {
        {"mass (W 299 N)", a.mass, dsp.mass, f.mass, 299.0 / 9.81, "kg"},
        {"BG (CG below CB)", -a.rg.z, -dsp.rg.z, -f.rg.z, 0.0196, "m"},
        {"Iyy about the CG/CB", a.I[1], dsp.I[1], f.I[1], 3.45, "kg m2"},
        {"added mass A11 (axial)", a.A[0], dsp.A[0], f.A[0], 0.93, "kg"},
        {"added mass A22 (sway = heave)", a.A[1], dsp.A[1], f.A[1], 35.5, "kg"},
        {"added roll inertia A44", a.A[3], dsp.A[3], f.A[3], 0.0704, "kg m2"},
        {"added pitch/yaw inertia A55", a.A[4], dsp.A[4], f.A[4], 4.88, "kg m2"},
        {"axial drag Xuu, Table C.1", effx(a), effx(dsp), effx(f), 1.62, "kg/m"},
        {"axial drag Xuu, measured Cd 0.267", effx(a), effx(dsp), effx(f), 0.5 * 1030 * 0.267 * 0.0285, "kg/m"},
        {"sway cross-flow drag Yv|v| (adjusted)", a.Dq[1], dsp.Dq[1], f.Dq[1], 1310.0, "kg/m"},
        {"pitch rate drag Mq|q| (adjusted)", a.Dq[4], dsp.Dq[4], f.Dq[4], 188.0, "kg m2/rad2"},
        {"yaw rate drag Nr|r| (adjusted)", a.Dq[5], dsp.Dq[5], f.Dq[5], 94.0, "kg m2/rad2"},
        {"roll resistance Kp|p| (adjusted)", a.Dq[3], dsp.Dq[3], f.Dq[3], 0.13, "kg m2/rad2"},
        {"rudder lift per pair Yuudr", pairlift(a, 1), pairlift(dsp, 1), pairlift(f, 1), 9.64, "kg/(m rad)"},
        {"stern plane lift per pair Zuuds", pairlift(a, 2), pairlift(dsp, 2), pairlift(f, 2), 9.64, "kg/(m rad)"},
        {"fin planform area, each", a.fins[0].area, dsp.fins[0].area, f.fins[0].area, 0.00665, "m2"},
        {"fin lift slope per fin, cl_alpha", a.fins[0].cl_alpha, dsp.fins[0].cl_alpha, f.fins[0].cl_alpha, 9.64 / (1025.0 * 0.00665), "/rad"},
        {"fin moment arm from the CB", arm(a), arm(dsp), arm(f), -0.638, "m"},
        {"maximum fin angle", maxd(a), maxd(dsp), maxd(f), 13.6, "deg"},
    };
    // yaw response: Prestero reports about 10 deg/s for a 4 deg rudder step (Sections 7.5.1 and 8.3). Open loop, 1.5 m/s held, 4 deg rudder, steady rate over the last 8 s.
    auto yaw4 = [&](const Spec& sp) {
        Vehicle v; v.s = sp; v.rebuild(); v.place({0, 0, -60}, 0); v.assist = false; Env env = calm_water(1025.0); SpeedHold sh; sh.target = 1.5; double sum = 0; int k = 0, n = (int)(50 / DT);
        double md = 0; for (const Fin& fn : sp.fins) if (fn.group == 1) md = std::max(md, fn.max_defl); double cmd = 4.0 / 57.2958 / std::max(md, 1e-6);
        for (int i = 0; i < n; ++i) { v.cmd[0] = sh.update(v.v.x, DT); v.cmd[4] = cmd; v.step(DT, env); if (i >= n - (int)(8 / DT)) { sum += v.w.z; ++k; } }
        return std::fabs(sum / k) * 57.2958; };
    pr.push_back({"yaw rate, 4 deg rudder, 1.5 m/s", yaw4(a), yaw4(dsp), yaw4(f), 10.0, "deg/s"});
    {   // the old Step 7A column, computed with the same lambdas
        double o[] = {old7.mass, -old7.rg.z, old7.I[1], old7.A[0], old7.A[1], old7.A[3], old7.A[4], effx(old7), effx(old7), old7.Dq[1], old7.Dq[4], old7.Dq[5], old7.Dq[3], pairlift(old7, 1), pairlift(old7, 2), old7.fins[0].area, old7.fins[0].cl_alpha, arm(old7), maxd(old7), yaw4(old7)};
        for (size_t i = 0; i < pr.size() && i < sizeof o / sizeof o[0]; ++i) pr[i].old = o[i]; }
    std::printf("   %-40s %10s %10s %10s %10s %10s   %9s %9s %9s\n", "quantity", "Step 7A", "7A before", "designer", "Fossen", "Prestero", "7A vs P", "before", "dsgn vs P");
    for (const P& r : pr) {
        double d7 = rel_pct(r.s7, r.pre), dd = rel_pct(r.des, r.pre), dold = rel_pct(r.old, r.pre); bool f7 = std::fabs(d7) > 25, fd = std::fabs(dd) > 25, fo = std::fabs(dold) > 25; if (f7) ++g_flags; if (fd) ++g_flags;
        std::printf("   %-40s %10.4g %10.4g %10.4g %10.4g %10.4g   %+7.1f %%%s %+7.1f %%%s %+7.1f %%%s  %s\n", r.what, r.s7, r.old, r.des, r.fos, r.pre, d7, f7 ? "*" : " ", dold, fo ? "*" : " ", dd, fd ? "*" : " ", r.unit);
    }
    std::printf("   (* = more than 25 %% from Prestero. 'Step 7A' is the preset now in use (remus100.hpp): Prestero's published values, so its columns match by construction; its yaw response to a 4 deg rudder, which is NOT\n"
                "    an input, is 10.8 deg/s against the thesis' roughly 10 deg/s. '7A before' is the old preset in models.hpp: axial drag 0.9 (his added mass 0.93 looks to have been typed into that slot), fins 6.8 times his area\n"
                "    with 11 times the lift per pair, damping 120 / 80 / 0.3 and sway/heave 131 (his unadjusted value). Designer's preset: fin AREA now equals his (the preset had 0.00855 m2); the remaining fin lift excess is the\n"
                "    lift SLOPE (the designer's finite-aspect-ratio foil, 2.3 per rad, against his effective 1.41), and the designer's cross-flow, pitch/yaw/roll damping are its own strip estimates, far from his tuned values.)\n");

    std::printf("\n%d checks, %d failed, %d value(s) differ from the published number by more than 25 %%\n", g_checks, g_fails, g_flags);
    return g_fails ? 1 : 0;
}
