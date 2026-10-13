// design_spec_test.cpp - checks that a design from the AUV Designer becomes a simulator vehicle that behaves like its report:
// mass / CG / buoyancy, top speed, electrical power and endurance, contra-rotating propeller torque, fin servo speed, hovering thrusters.
// Build: cmake target design_spec_test.   Run: ./design_spec_test
#include "design_test_common.hpp"

static const double DT = 0.005;

struct Run { double speed = 0, depth0 = 0, depth1 = 0, pitch_deg = 0, roll_deg = 0, used_Wh = 0, mean_P = 0, prop_P = 0, main_P = 0; };
// fly with a constant surge command (or, when hold_speed > 0, with the speed held) and average over the last `tail` seconds
static Run fly(const Spec& spec, double rho, double surge, double seconds, double hold_speed = 0, double tail = 5.0) {
    Vehicle v; v.s = spec; v.rebuild(); v.place({0, 0, -20}, 0); Env env = calm_water(rho);
    Run r; r.depth0 = -v.p.z; double e0 = v.energy_Wh, vsum = 0, psum = 0, msum = 0; int nv = 0, n = (int)(seconds / DT), n_tail = (int)(tail / DT);
    SpeedHold sh; sh.target = hold_speed;
    for (int i = 0; i < n; ++i) {
        v.cmd[0] = hold_speed > 0 ? sh.update(v.v.x, DT) : surge; v.step(DT, env);
        if (i >= n - n_tail) { vsum += v.v.x; psum += v.power_W - spec.hotel_W; msum += v.rot[0].power; ++nv; }
    }
    double yw, pt, rl; v.heading_pitch_roll(yw, pt, rl);
    r.speed = vsum / std::max(nv, 1); r.prop_P = psum / std::max(nv, 1); r.main_P = msum / std::max(nv, 1); r.depth1 = -v.p.z; r.pitch_deg = pt * 57.2958; r.roll_deg = rl * 57.2958;
    r.used_Wh = e0 - v.energy_Wh; r.mean_P = r.used_Wh * 3600.0 / seconds;
    return r;
}

int main() {
    int n = 0;
    for (const design::DesignIn& in : design::builtin_designs()) {
        design::Designer des; design::DesignOut o = des.run(in); Spec s = design::to_spec(in, o);
        const bool remus = in.name.find("REMUS") == 0, contra = in.name.find("contra") != std::string::npos, hover = in.name.find("Hovering") == 0;
        std::printf("== %s\n", in.name.c_str());
        // ---- static properties match the report
        CHECK(std::fabs(s.mass - o.mass) < 1e-9, "mass %.3f vs %.3f", s.mass, o.mass);
        CHECK(std::fabs(s.rg.x - o.rg.x) < 1e-9 && std::fabs(s.rg.z - o.rg.z) < 1e-9, "CG (%.3f %.3f) vs (%.3f %.3f)", s.rg.x, s.rg.z, o.rg.x, o.rg.z);
        Vehicle v; v.s = s; v.rebuild();
        double B = o.rho_water * 9.80665 * v.volume();
        CHECK(std::fabs(B - o.B) < 1e-3 * o.B, "buoyancy %.2f N vs %.2f N", B, o.B);
        CHECK(std::fabs((B - o.mass * 9.80665) - o.net_up) < 0.05 + 1e-3 * o.B, "net up %.2f vs %.2f", B - o.mass * 9.80665, o.net_up);
        CHECK(v.s.thr.size() >= 1 && s.fins.size() == o.fin_rows.size(), "%d thrusters, %d fins (report %d fins)", (int)s.thr.size(), (int)s.fins.size(), (int)o.fin_rows.size());
        std::printf("   mass %.2f kg  net up %+.2f N  CG x %+.3f z %+.3f  thrusters %d  fins %d  v_top(report) %.2f m/s\n", o.mass, o.net_up, o.rg.x, o.rg.z, (int)s.thr.size(), (int)s.fins.size(), o.v_top);

        // ---- 30 s at full surge in calm water: top speed and propulsion power
        Run r = fly(s, o.rho_water, 1.0, 30.0);
        double pe_max = r.main_P;                                  // the report's P_elec_max is the main propeller's; the autopilot may also use tunnel thrusters
        std::printf("   full surge 30 s: speed %.2f m/s (report top speed %.2f), main propeller power %.1f W (report P_elec_max %.1f W, %+.1f %%; all thrusters %.1f W), pitch %+.1f deg, roll %+.2f deg\n", r.speed, o.v_top, pe_max, o.P_elec_max, rel_pct(pe_max, o.P_elec_max), r.prop_P, r.pitch_deg, r.roll_deg);
        CHECK(std::fabs(r.speed - o.v_top) < 0.10 * o.v_top, "top speed %.3f m/s is not within 10 %% of the report %.3f m/s", r.speed, o.v_top);
        CHECK(std::fabs(pe_max - o.P_elec_max) < 0.10 * o.P_elec_max, "full-surge electrical power %.1f W is not within 10 %% of P_elec_max %.1f W", pe_max, o.P_elec_max);
        CHECK(std::fabs(r.pitch_deg) < 20, "pitch %.1f deg at full surge", r.pitch_deg);

        // ---- cruise: power at v_cruise and the endurance it implies. The designer's drag is that of level flight, so the power check
        // uses a copy of the vehicle with neutral buoyancy; the design's own +2 % trim (nose-down autopilot offset, climb drag) is reported below it.
        Spec sn = s; sn.trim = 1025.0 / o.rho_water - 1.0;        // displaced volume = mass / water density
        Run c = fly(sn, o.rho_water, 0.0, 120.0, in.v_cruise, 20.0), ct = fly(s, o.rho_water, 0.0, 120.0, in.v_cruise, 20.0);
        double endur = 0.9 * in.battery_Wh / std::max(c.mean_P, 1e-6);
        std::printf("   cruise %.2f m/s, neutral buoyancy: speed %.2f, propulsion power %.1f W (report P_elec_cruise %.1f W, %+.1f %%); drain %.1f W -> endurance %.1f h (report %.1f h, %+.1f %%)\n",
                    in.v_cruise, c.speed, c.prop_P, o.P_elec_cruise, rel_pct(c.prop_P, o.P_elec_cruise), c.mean_P, endur, o.endurance_h, rel_pct(endur, o.endurance_h));
        std::printf("   cruise with the design's own trim (%+.1f %%): propulsion power %.1f W (%+.1f %% vs report), endurance %.1f h (%+.1f %%)\n", 100 * o.net_up / o.W, ct.prop_P, rel_pct(ct.prop_P, o.P_elec_cruise), 0.9 * in.battery_Wh / std::max(ct.mean_P, 1e-6), rel_pct(0.9 * in.battery_Wh / std::max(ct.mean_P, 1e-6), o.endurance_h));
        CHECK(std::fabs(c.speed - in.v_cruise) < 0.05 * in.v_cruise, "cruise speed %.3f vs %.3f", c.speed, in.v_cruise);
        CHECK(std::fabs(c.prop_P - o.P_elec_cruise) < 0.10 * o.P_elec_cruise, "cruise electrical power %.1f W is not within 10 %% of P_elec_cruise %.1f W", c.prop_P, o.P_elec_cruise);
        CHECK(std::fabs(endur - o.endurance_h) < 0.10 * o.endurance_h, "endurance implied by the energy drain %.2f h vs report %.2f h", endur, o.endurance_h);

        // ---- zero command: neutral buoyancy holds depth; the design's own trim rises at the speed the drag law predicts
        Run z0 = fly(sn, o.rho_water, 0.0, 30.0);
        std::printf("   zero command, neutral: depth %.2f -> %.2f m\n", z0.depth0, z0.depth1);
        CHECK(std::fabs(z0.depth1 - z0.depth0) < 0.5, "drifts %.2f m in 30 s with neutral buoyancy", z0.depth1 - z0.depth0);
        Run z2 = fly(s, o.rho_water, 0.0, 30.0);
        double vt = std::sqrt(std::max(o.net_up, 0.0) / std::max(s.Dq[2], 1e-6));
        std::printf("   zero command, design trim %+.1f %%: depth %.2f -> %.2f m (drag-limited rise speed %.2f m/s)\n", 100 * o.net_up / o.W, z2.depth0, z2.depth1, vt);
        CHECK(z2.depth1 < z2.depth0 && z2.depth0 - z2.depth1 < vt * 30.0 * 1.05 + 0.3 && z2.depth0 - z2.depth1 > 0.5 * vt * 30.0 * 0.5, "design trim: rose %.2f m, limit %.2f m", z2.depth0 - z2.depth1, vt * 30.0);

        // ---- low-speed drag: the vehicle's axial law against the designer's own drag at the same speed (hull part: the simulator adds the fins' drag itself)
        {
            design::Designer dd; double fin_cd0 = 0.5 * o.rho_water * 0.008 * o.fin_area_total, worst = 0; double worst_u = 0;
            for (double U : {0.1, 0.15, 0.2, 0.3, 0.45, 0.6, 1.0, 1.5, 2.0}) {
                design::DesignIn i2 = in; i2.v_cruise = U; i2.v_max = std::max(U + 0.01, in.v_max); double Dd = dd.run(i2).drag_cruise, Ds = s.axial_drag(U) + fin_cd0 * U * U;
                double e = std::fabs(Ds - Dd) / Dd; if (e > worst) { worst = e; worst_u = U; }
            }
            double fit_err = 0; for (double U : {0.15, 0.3}) { design::DesignIn i2 = in; i2.v_cruise = U; i2.v_max = std::max(U + 0.01, in.v_max); double Dd = dd.run(i2).drag_cruise, Ds = s.Dq[0] * U * U + s.Dl[0] * U + fin_cd0 * U * U; fit_err = std::max(fit_err, std::fabs(Ds - Dd) / Dd); }
            std::printf("   axial drag, simulator vs designer, 0.1 to 2 m/s: worst %.1f %% (at %.2f m/s); the old two-point fit was %.0f %% off at 0.15 or 0.3 m/s\n", 100 * worst, worst_u, 100 * fit_err);
            CHECK(worst < 0.15, "axial drag within 15 %% of the designer's law at all speeds (worst %.1f %% at %.2f m/s)", 100 * worst, worst_u);
        }

        // ---- fin servo: a step command ramps at the servo speed
        {
            Vehicle f; f.s = s; f.rebuild(); f.place({0, 0, -20}, 0); Env env = calm_water(o.rho_water);
            double maxd = 0; for (const Fin& fn : s.fins) if (fn.group) maxd = std::max(maxd, fn.max_defl);
            double t_full = -1, max_slope = 0, prev = 0; f.assist = false;
            for (int i = 0; i < 400; ++i) {
                f.cmd[4] = 1.0; f.step(DT, env);
                max_slope = std::max(max_slope, (f.defl[1] - prev) / DT); prev = f.defl[1];
                if (t_full < 0 && f.defl[1] >= 0.9999) t_full = (i + 1) * DT;
            }
            double rate = s.servo_rate * 3.14159265358979 / 180.0 / maxd, t_exp = 1.0 / rate;
            std::printf("   fin servo: %.0f deg/s, %.0f deg stroke: full deflection after %.3f s (expected %.3f s), peak rate %.1f deg/s\n", s.servo_rate, maxd * 57.2958, t_full, t_exp, max_slope * maxd * 57.2958);
            if (maxd > 0) CHECK(t_full > 0 && std::fabs(t_full - t_exp) <= 0.02 * t_exp + 2 * DT, "full deflection after %.3f s, expected %.3f s", t_full, t_exp);
            if (maxd > 0) CHECK(std::fabs(max_slope * maxd * 57.2958 - s.servo_rate) < 0.02 * s.servo_rate, "peak servo rate %.1f deg/s, limit %.1f", max_slope * maxd * 57.2958, s.servo_rate);
        }

        // ---- propeller reaction torque (single propeller heels the hull; a contra-rotating pair does not)
        if (contra) {
            const design::PropRow& pr = o.prop_rows[0];
            std::printf("   contra-rotating pair: front %.0f rpm, rear %.0f rpm, net torque %.3f N m (report); simulated as ONE thruster, bollard %.1f N (report %.1f N), reaction torque per N %.4f, steady roll at full surge %+.3f deg\n",
                        pr.rpm, pr.rpm2, pr.Qnet, s.thr[0].max_thrust, o.thrust_bollard, s.thr[0].kq, r.roll_deg);
            CHECK(pr.contra && pr.rpm2 > 0 && pr.rpm2 != pr.rpm, "the pair should turn at two different speeds (%.0f / %.0f rpm)", pr.rpm, pr.rpm2);
            CHECK(s.thr.size() == 1 && std::fabs(s.thr[0].max_thrust - o.thrust_bollard) < 1e-6 * o.thrust_bollard, "single thruster must give the pair's net bollard thrust");
            CHECK(std::fabs(s.thr[0].kq * s.thr[0].max_thrust) <= std::fabs(pr.Qnet) + 1e-9 + 0.0, "net reaction torque at full thrust must be the reported Qnet");
            CHECK(std::fabs(r.roll_deg) < 0.05, "a torque-balanced pair must not heel the hull: roll %.3f deg", r.roll_deg);
        } else if (remus) {
            std::printf("   single propeller: reaction torque %.4f N m per N (shaft %.1f W at %.0f rpm), steady roll at full surge %+.2f deg\n", s.thr[0].kq, o.P_shaft_cruise, o.rpm_cruise, r.roll_deg);
            CHECK(s.thr[0].kq > 0 && std::fabs(r.roll_deg) > 0.05 && std::fabs(r.roll_deg) < 10, "single propeller should heel the hull a little: %.2f deg", r.roll_deg);
        }

        // ---- the four tunnel thrusters of the hovering design
        if (hover) {
            Env env = calm_water(o.rho_water);
            auto still = [&](int cmd_axis) {            // full command with the hull held still, so the thrusters give their bollard thrust
                Vehicle q; q.s = s; q.rebuild(); q.place({0, 0, -20}, 0); q.assist = false;
                for (int i = 0; i < 800; ++i) { q.cmd[cmd_axis] = 1.0; q.step(DT, env); q.v = {}; q.w = {}; }
                double F = 0; for (size_t i = 0; i < s.thr.size(); ++i) F += q.thrust[i] * (cmd_axis == 1 ? s.thr[i].dir.y : s.thr[i].dir.z);
                return F; };
            double Fz = still(2), Fy = still(1);
            int nv = 0, nl = 0; double sv = 0, sl = 0, xv0 = 1e9, xv1 = -1e9, xl0 = 1e9, xl1 = -1e9;
            for (const Thruster& t : s.thr) {
                if (std::fabs(t.dir.z) > 0.9) { ++nv; sv += t.max_thrust; xv0 = std::min(xv0, t.pos.x - o.rg.x); xv1 = std::max(xv1, t.pos.x - o.rg.x); }
                if (std::fabs(t.dir.y) > 0.9) { ++nl; sl += t.max_thrust; xl0 = std::min(xl0, t.pos.x - o.rg.x); xl1 = std::max(xl1, t.pos.x - o.rg.x); }
            }
            // a pure heave (or sway) command must not pitch (or yaw) the hull, so the thruster with the longer arm is held back: F = Tmax (a + b) / max(a, b)
            double Tv = sv / std::max(nv, 1), Tl = sl / std::max(nl, 1);
            double Fz_pred = Tv * (xv1 - xv0) / std::max(std::fabs(xv0), std::fabs(xv1)), Fy_pred = Tl * (xl1 - xl0) / std::max(std::fabs(xl0), std::fabs(xl1));
            std::printf("   tunnel thrusters (%d vertical at x %+.2f/%+.2f m, %d lateral at x %+.2f/%+.2f m from the CG): sum of the maxima %.1f N heave, %.1f N sway (report cap_heave %.1f, cap_sway %.1f)\n", nv, xv0, xv1, nl, xl0, xl1, sv, sl, o.cap_heave, o.cap_sway);
            std::printf("   pure heave command (no pitch): %.1f N (%+.1f %% vs cap_heave; lever-arm prediction %.1f N), pure sway command (no yaw): %.1f N (%+.1f %% vs cap_sway; prediction %.1f N)\n", Fz, rel_pct(Fz, o.cap_heave), Fz_pred, Fy, rel_pct(Fy, o.cap_sway), Fy_pred);
            CHECK(nv == 2 && nl == 2, "expected 2 vertical + 2 lateral tunnel thrusters, got %d + %d", nv, nl);
            CHECK(std::fabs(sv - o.cap_heave) < 1e-6 * o.cap_heave && std::fabs(sl - o.cap_sway) < 1e-6 * o.cap_sway, "thruster maxima %.1f / %.1f N vs cap_heave %.1f / cap_sway %.1f N", sv, sl, o.cap_heave, o.cap_sway);
            CHECK(std::fabs(Fz - Fz_pred) < 0.03 * Fz_pred, "pure heave force %.1f N vs lever-arm prediction %.1f N", Fz, Fz_pred);
            CHECK(std::fabs(Fy - Fy_pred) < 0.03 * Fy_pred, "pure sway force %.1f N vs lever-arm prediction %.1f N", Fy, Fy_pred);
            CHECK(Fz > 0.85 * o.cap_heave && Fy > 0.85 * o.cap_sway, "balanced forces %.1f / %.1f N are far below the report's capacity", Fz, Fy);
            CHECK(o.cap_heave > std::fabs(o.net_up) * 3, "heave capacity %.1f N must comfortably beat the net buoyancy %.1f N", o.cap_heave, o.net_up);
            // hover: hold depth and position at zero speed with a depth controller on the heave command
            Vehicle k; k.s = s; k.rebuild(); k.place({0, 0, -20}, 0); k.assist = false; double zt = -20.0, emax = 0, vmax_h = 0, cmax = 0, yaw0 = 0, pt0 = 0, rl0 = 0; k.heading_pitch_roll(yaw0, pt0, rl0);
            for (int i = 0; i < (int)(90 / DT); ++i) {
                double ez = zt - k.p.z; k.cmd[2] = std::max(-1.0, std::min(1.0, 1.0 * ez - 2.0 * k.v.z)); k.cmd[0] = k.cmd[1] = 0; k.step(DT, env);
                if (i > (int)(30 / DT)) { emax = std::max(emax, std::fabs(ez)); vmax_h = std::max(vmax_h, len(k.v)); cmax = std::max(cmax, std::fabs(k.cmd[2])); }
            }
            double yw, pt, rl; k.heading_pitch_roll(yw, pt, rl);
            std::printf("   hover 90 s at zero speed (depth held by the heave thrusters): depth error <= %.3f m, speed <= %.3f m/s, heave command up to %.2f (needs %.2f), pitch %+.2f deg, roll %+.2f deg\n", emax, vmax_h, cmax, std::fabs(o.net_up) / o.cap_heave, pt * 57.2958, rl * 57.2958);
            CHECK(emax < 0.15 && vmax_h < 0.1, "hover: depth error %.3f m, speed %.3f m/s", emax, vmax_h);
            CHECK(std::fabs(pt) < 0.05 && std::fabs(rl) < 0.05, "hover attitude pitch %.3f roll %.3f rad", pt, rl);
        }
        ++n;
    }
    std::printf("\n%d designs, %d checks, %d failed\n", n, g_checks, g_fails);
    return g_fails ? 1 : 0;
}
