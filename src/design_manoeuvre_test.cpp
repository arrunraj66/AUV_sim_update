// design_manoeuvre_test.cpp - REMUS 100 preset at cruise speed in calm water: turn, pitch and stopping, simulated against the Designer's
// predictions (yaw_rate_max, turn_radius, pitch_max_deg, and a drag + added-mass stopping law).
//
// The designer warns that these fins are too small for natural directional stability (stability ratio 0.29, "it will need an autopilot, like the
// real REMUS"). An unstable vehicle has no steady turn or pitch angle in OPEN loop, so the "full rudder" and "full elevator" numbers are measured
// the way such a vehicle is really flown: a feedback controller asks for a yaw rate / pitch angle and we find the LARGEST one it can hold with the
// fins not yet at the stop. The open-loop transient is printed as well. Differences above 25 % are flagged, not tuned away.
//
// What the designer's predictions assume (checked in this test):
//   yaw_rate_max   rudder moment at full deflection (stalled foil CL) = hull cross-flow yaw damping at the cruise speed. Steady state, so no added mass; no sideslip
//                  (the fins see only their deflection), no Munk moment, no propeller torque.
//   pitch_max_deg  asin(elevator moment at full deflection / (W BG)): hydrostatic righting moment only, angle of attack zero, no Munk moment, no propeller torque.
//   stopping       needs the drag law D(U) and m + A11. The first version of this test used a pure quadratic law D = Xuu U^2 from the cruise point, which the designer's own
//                  drag does not follow (its friction coefficient grows at low Re), so that prediction was wrong. The test now integrates the designer's own D(U).
// Run: ./design_manoeuvre_test
#include "design_test_common.hpp"

static const double DT = 0.005;
struct Row { const char* what; double sim, pred; const char* unit; };

static Vehicle make(const Spec& s, double z, bool assist = false) { Vehicle v; v.s = s; v.rebuild(); v.place({0, 0, z}, 0); v.assist = assist; return v; }

struct TurnResult { bool held; double rate, defl, speed; };
// closed-loop yaw rate: PI on the rudder command, speed held by a PI loop
static TurnResult hold_rate(const Spec& s, const Env& env, double u_target, double r_target, double secs = 60) {
    Vehicle v = make(s, -60); SpeedHold sh; sh.target = u_target; double ir = 0, rsum = 0, dsum = 0, usum = 0; int k = 0, n = (int)(secs / DT);
    for (int i = 0; i < n; ++i) {
        double e = r_target - v.w.z; ir = std::max(-1.0, std::min(1.0, ir + 0.5 * e * DT));
        v.cmd[0] = sh.update(v.v.x, DT); v.cmd[4] = std::max(-1.0, std::min(1.0, 1.5 * e + ir)); v.step(DT, env);
        if (i >= n - (int)(8 / DT)) { rsum += v.w.z; dsum += std::fabs(v.defl[1]); usum += len(v.v); ++k; }
    }
    TurnResult r{false, rsum / k, dsum / k, usum / k};
    r.held = std::fabs(r.rate - r_target) < 0.03 * r_target + 0.005 && r.defl < 0.985;
    return r;
}
// closed-loop pitch angle: PD + I on the elevator command (nose up positive)
static bool hold_pitch(const Spec& s, const Env& env, double u_target, double th_target, double& th_out, double& defl_out, double secs = 30) {
    Vehicle v = make(s, -400); SpeedHold sh; sh.target = u_target; double ip = 0, tsum = 0, dsum = 0; int k = 0, n = (int)(secs / DT);
    for (int i = 0; i < n; ++i) {
        double yw, pt, rl; v.heading_pitch_roll(yw, pt, rl); double e = th_target - pt; ip = std::max(-1.0, std::min(1.0, ip + 0.3 * e * DT));
        v.cmd[0] = sh.update(v.v.x, DT); v.cmd[3] = std::max(-1.0, std::min(1.0, 2.5 * e + 1.2 * v.w.y + ip)); v.step(DT, env);
        if (i >= n - (int)(8 / DT)) { tsum += pt; dsum += std::fabs(v.defl[2]); ++k; }
    }
    th_out = tsum / k; defl_out = dsum / k;
    return std::fabs(th_out - th_target) < 0.03 * th_target + 0.01 && defl_out < 0.985;
}

int main() {
    design::DesignIn in = design::builtin_designs()[0];
    design::Designer des; design::DesignOut o = des.run(in); Spec s = design::to_spec(in, o);
    Spec sn = s; sn.trim = 1025.0 / o.rho_water - 1.0;           // neutral buoyancy: the designer's numbers are for level flight
    Env env = calm_water(o.rho_water);
    std::printf("== %s at %.2f m/s, calm water, neutral buoyancy, stability ratio %.2f (designer: needs an autopilot)\n", in.name.c_str(), in.v_cruise, o.stability_ratio);
    std::vector<Row> rows;

    // (a) turn
    {   // open loop: full left rudder step, from a clean start (3 s of straight flight) and from a vehicle that has had 30 s to drift away
        double peak = 0, w60[2] = {0, 0};
        for (int k = 0; k < 2; ++k) {
            Vehicle v = make(sn, -60); SpeedHold sh; sh.target = in.v_cruise; double pk = 0;
            for (int i = 0; i < (int)((k ? 30.0 : 3.0) / DT); ++i) { v.cmd[0] = sh.update(v.v.x, DT); v.step(DT, env); }
            for (int i = 0; i < (int)(60 / DT); ++i) { v.cmd[0] = sh.update(v.v.x, DT); v.cmd[4] = 1.0; v.step(DT, env); if (i < (int)(4 / DT)) pk = std::max(pk, v.w.z); w60[k] = v.w.z; }
            if (k == 0) peak = pk;
        }
        std::printf("   open loop, full left rudder: yaw rate peaks at %+.2f rad/s in the first 4 s (designer's steady value %.2f); after 60 s it is %+.2f rad/s from a clean start and\n"
                    "   %+.2f rad/s after 30 s of straight flight first. With stability ratio %.2f (Munk moment larger than the fin restoring moment) an open-loop steady turn is not\n"
                    "   guaranteed, so the transient and the closed-loop limit are compared.\n", peak, o.yaw_rate_max, w60[0], w60[1], o.stability_ratio);
        CHECK(peak > 0, "full left rudder must start a left turn (peak yaw rate %.3f)", peak);
        rows.push_back({"yaw rate, open loop, first 4 s (peak)", peak * 57.2958, o.yaw_rate_max * 57.2958, "deg/s"});
        // closed loop: largest yaw rate that can be held with the rudder below its stop
        double lo = 0.05, hi = 2.0; TurnResult best = hold_rate(sn, env, in.v_cruise, lo);
        CHECK(best.held, "the controller cannot hold even %.2f rad/s", lo);
        for (int it = 0; it < 12; ++it) { double mid = 0.5 * (lo + hi); TurnResult t = hold_rate(sn, env, in.v_cruise, mid); if (t.held) { lo = mid; best = t; } else hi = mid; }
        double R = best.speed / std::max(best.rate, 1e-9);
        rows.push_back({"max steady yaw rate, rudder at the stop", best.rate * 57.2958, o.yaw_rate_max * 57.2958, "deg/s"});
        rows.push_back({"turn radius at that rate", R, o.turn_radius, "m"});
        std::printf("   closed loop: largest held yaw rate %.2f rad/s with the rudder at %.0f %% of its stroke, speed %.2f m/s\n", best.rate, 100 * best.defl, best.speed);
    }
    // (b) pitch. The designer's number is a static balance at zero angle of attack; the comparable simulator number is the same balance solved with the simulator's own force
    // model (elevator at the stop, velocity along the body axis, hydrostatic restoring moment). The closed-loop "largest held pitch" below holds a state with sideslip and
    // Munk moment that the designer's formula leaves out, so it is printed for information and is not compared with the formula.
    {
        Vehicle v = make(sn, -200); v.defl[2] = 1.0; double prev = 0, teq = -1;
        for (double th = 0; th <= 89; th += 0.25) {
            Vehicle::State st; st.p = {0, 0, -200}; st.q = qfrom_hpr(0, th * 3.14159265 / 180, 0); st.v = {in.v_cruise, 0, 0}; st.w = {};
            double ay = v.eval(st, env).dw.y;                       // + = nose down
            if (th > 0 && prev < 0 && ay >= 0 && teq < 0) teq = th - 0.25 + 0.25 * (-prev) / (ay - prev);
            prev = ay;
        }
        CHECK(teq > 0, "no static pitch balance found with the elevator at the stop");
        rows.push_back({"pitch angle, static balance, elevator at the stop", teq, o.pitch_max_deg, "deg"});
        for (int variant = 0; variant < 2; ++variant) {
            Spec sv = sn; if (variant == 1) for (Thruster& t : sv.thr) t.kq = 0;          // torque-balanced, like the contra-rotating Survey design
            double lo = 2.0 * 3.14159265 / 180, hi = 80.0 * 3.14159265 / 180, th = 0, df = 0; bool ok = hold_pitch(sv, env, in.v_cruise, lo, th, df);
            CHECK(ok, "the controller cannot hold even %.1f deg of pitch (got %.2f, elevator %.2f)", lo * 57.2958, th * 57.2958, df);
            double best = th, bd = df;
            for (int it = 0; it < 10; ++it) { double mid = 0.5 * (lo + hi); if (hold_pitch(sv, env, in.v_cruise, mid, th, df)) { lo = mid; best = th; bd = df; } else hi = mid; }
            std::printf("   for information, closed loop %s: largest held pitch %.1f deg with the elevator at %.0f %% of its stroke (includes sideslip and Munk moment; the formula does not)\n",
                        variant == 0 ? "with the propeller reaction torque" : "with the propeller torque balanced", best * 57.2958, 100 * bd);
        }
    }
    // (c) stopping distance after the thrust is cut (attitude autopilot on so it stays level): to 10 % of the cruise speed
    {
        Vehicle v = make(sn, -60, true); SpeedHold sh; sh.target = in.v_cruise;
        for (int i = 0; i < (int)(40 / DT); ++i) { v.cmd[0] = sh.update(v.v.x, DT); v.step(DT, env); }
        double u0 = len(v.v), t = 0; V3 p0 = v.p; V3 pw = v.p; double path = 0;
        while (len(v.v) > 0.1 * u0 && t < 400) { v.cmd[0] = 0; v.step(DT, env); t += DT; path += len(v.p - pw); pw = v.p; }
        double m_eff = o.mass + o.AM[0];
        // prediction: integrate (m + A11) du/dt = -D(u) with the designer's own drag D(U) (the designer evaluated at U = u)
        double pu = u0, pt = 0, pd = 0, pdt = 0.005;
        while (pu > 0.1 * u0 && pt < 600) { design::DesignIn i2 = in; i2.v_cruise = pu; i2.v_max = std::max(pu + 0.01, in.v_max); double D = des.run(i2).drag_cruise; pu -= D / m_eff * pdt; pd += pu * pdt; pt += pdt; }
        double pred_d = pd, pred_t = pt;
        rows.push_back({"stopping distance to 10 % speed", path, pred_d, "m"});
        rows.push_back({"stopping time to 10 % speed", t, pred_t, "s"});
        std::printf("   stopping: from %.2f m/s, straight-line distance %.1f m, path %.1f m; prediction integrates the designer's drag curve with m + A11 = %.1f kg (a pure quadratic Xuu = %.2f would give %.0f s)\n", u0, len(v.p - p0), path, m_eff, o.Xuu, m_eff / (o.Xuu * u0) * 9.0);
    }

    std::printf("\n   %-44s %12s %12s %9s\n", "quantity", "simulated", "predicted", "diff");
    for (const Row& r : rows) {
        double d = rel_pct(r.sim, r.pred); bool flag = std::fabs(d) > 25.0; if (flag) ++g_flags;
        std::printf("   %-44s %8.2f %-4s %8.2f %-4s %+8.1f %%%s\n", r.what, r.sim, r.unit, r.pred, r.unit, d, flag ? "   <-- FLAG > 25 %" : "");
    }
    std::printf("\n%d checks, %d failed, %d value(s) differ from the prediction by more than 25 %%\n", g_checks, g_fails, g_flags);
    return g_fails ? 1 : 0;
}
