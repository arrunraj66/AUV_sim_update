// vehicle.hpp - 6-degree-of-freedom underwater vehicle dynamics (Fossen's marine craft model).
//
//   M * dv/dt + C_RB(v) v + C_A(vr) vr + D(vr) vr + g(q) = tau_thrusters + tau_fins + tau_contact
//
// v  = [u v w p q r]  velocity of the vehicle in its own (body) frame
// vr = v minus the water current (the water moves, so forces depend on the RELATIVE velocity)
// M  = rigid-body mass matrix (with the centre of gravity offset) + added mass of the water
// D  = linear + quadratic drag,   g = weight and buoyancy,   fins = lifting surfaces with angle of attack
// Integrated with RK4 at a fixed step, so the result does not depend on the frame rate.
#pragma once
#include "vmath.hpp"
#include <vector>
#include <array>
#include <string>
#include <functional>

struct Thruster { V3 pos, dir; double max_thrust, tau, u_free, rev; double pk = 1.4, kdrop = 0.7;   // pk: electrical power = pk * thrust^1.5 (W, N); kdrop: share of the thrust lost at forward speed u_free
                  double disc_ik = 0, zeta = 0, wake = 0;   // zeta > 0: electrical power from the actuator-disc law P = T (Va/2 + sqrt(Va^2/4 + T * disc_ik)) / zeta, Va = speed * (1 - wake), disc_ik = 1 / (2 rho A)
                  double kq = 0; };            // reaction torque about the thrust axis per newton of thrust (N m / N), acts on the hull   // dir = direction of the force on the vehicle
struct Fin { V3 pos, normal; double area, cl_alpha, max_defl; int group; double sign;   // group 0 fixed, 1 rudder, 2 elevator, 3 mixed (X tail)
             double kr = 0, ke = 0, stall = 0.30, f_lim = 0; };   // group 3: deflection = (kr * rudder + ke * elevator) * max_defl; stall angle (rad); servo force limit (N, 0 = none)

struct Rotor { double thrust = 0, w = 0, power = 0, wet = 1, adv = 0; };   // telemetry of one thruster: N, rad/s, electrical W, fraction in water

// one slice of the hull for per-strip drag. Cross-flow force = cy * v_y|v_y| (and cz for z) acts at the slice position (x, y, z); the axial drag
// is shared out by `ax` (fractions sum to 1) and acts there too, so a slice that sits above or below the body origin makes a pitching moment.
// axial drag law as a table of (speed, force) points, interpolated in log-log (D ~ U^n between points, so it is exact at the points and has the right slope between them).
// Below the first point the slope of the first segment is kept (clamped 1..2), above the last point the slope of the last one (clamped 1.5..2.5).
inline double curve_drag(const std::vector<std::array<double, 2>>& c, double U) {
    if (c.empty() || U <= 0) return 0.0;
    if (c.size() == 1) return c[0][1] * (U / c[0][0]) * (U / c[0][0]);
    auto seg = [&](size_t i, double u) { double n = std::log(c[i + 1][1] / std::max(c[i][1], 1e-12)) / std::log(c[i + 1][0] / c[i][0]); return std::make_pair(c[i][1] * std::pow(u / c[i][0], n), n); };
    if (U <= c.front()[0]) { double n = std::max(1.0, std::min(2.0, seg(0, c[0][0]).second)); return c[0][1] * std::pow(U / c[0][0], n); }
    if (U >= c.back()[0]) { size_t i = c.size() - 2; double n = std::max(1.5, std::min(2.5, seg(i, c[i][0]).second)); return c.back()[1] * std::pow(U / c.back()[0], n); }
    size_t i = 0; while (i + 2 < c.size() && U > c[i + 1][0]) ++i;
    return seg(i, U).first;
}

struct DragStrip { double x = 0, y = 0, z = 0, cy = 0, cz = 0, ax = 0; };

struct Spec {
    std::string name, note;
    int mesh = 0;                    // which 3D model draws it
    double mass = 30;                // kg
    double trim = 0.0;               // net buoyancy as a fraction of the weight (0.02 = 2 % positive)
    double length = 1.5, width = 0.3, height = 0.3;      // outer size, m
    V3 rg = {0, 0, -0.02}, rb = {0, 0, 0};              // centre of gravity / buoyancy, from the body origin
    double I[3] = {0.2, 3, 3};                          // moments of inertia about the CG
    double A[6] = {1, 10, 10, 0.1, 1, 1};               // added mass (kg) and added inertia (kg m2)
    double Dl[6] = {0, 0, 0, 0, 0, 0};                  // linear drag
    double Dq[6] = {1, 50, 50, 0.1, 5, 5};              // quadratic drag
    std::vector<Thruster> thr; std::vector<Fin> fins; std::vector<V3> contact;
    std::vector<std::array<double, 2>> axial_curve;     // optional: axial drag as (speed m/s, force N) points; when not empty it replaces Dq[0], Dl[0] in the per-strip drag
    std::vector<DragStrip> strips;                      // when not empty: surge, sway, heave, pitch and yaw drag come from these slices (Dq[0], Dl[0] set the axial law; Dq[1], Dq[2], Dq[4], Dq[5] are not used)
    bool glider = false; double pump_range = 0, pump_time = 20, mass_x = 0, mass_y = 0;   // buoyancy engine (m3) and moving mass (m)
    V3 lamp_pos = {0.5, 0, 0}, cam_pos = {0.5, 0, 0.05};
    double chase = 4.0, rated_depth = 100;
    double axial_drag(double U) const { return axial_curve.empty() ? Dq[0] * U * U + Dl[0] * U : curve_drag(axial_curve, U); }   // hull axial drag at speed U (N), without the fins' own drag
    double assist_gain = 1.0;                        // gain of the attitude assist: vehicles with weak fins need less, or the assist over-corrects and hunts
    double servo_rate = 60;                          // fin servo speed, deg/s
    double battery_Wh = 500, hotel_W = 15;           // battery capacity and constant electronics load
};

struct Env {                                // what the water and sea bed look like at the vehicle
    double rho = 1025, g = 9.80665;
    V3 cur;                                 // current, world frame
    std::function<double(double, double)> eta;       // wave surface height z at x,y
    std::function<double(double, double)> ground;    // sea bed height z at x,y
};

struct Vehicle {
    Spec s;
    V3 p = {0, 0, -5}; Q q; V3 v, w;                  // state
    std::vector<double> thrust;                       // actual thrust of every thruster, N
    double defl[3] = {0, 0, 0};                       // fin deflection (-1..1) of rudder and elevator groups
    double pump = 0, mx = 0, my = 0;                  // glider: buoyancy engine and moving-mass positions, -1..1
    double prop_angle = 0;
    double cmd[5] = {0, 0, 0, 0, 0};                  // surge, sway, heave, pitch(nose up), yaw(left)  each -1..1
    double drag_scale = 1, am_scale = 1, thrust_scale = 1, cg_lift = 0;   // GUI tuning
    bool assist = true, has_elev = false, has_rud = false;                 // attitude assist: stick = pitch angle / turn rate
    // results for the GUI
    double sub = 1, buoy = 0, weight = 0, rho_now = 1025, t = 0;
    bool on_bottom = false;
    std::vector<float> hist; double hist_t = 0;
    std::vector<std::array<double, 6>> alloc;   // thruster allocation (n x 6)
    std::vector<Rotor> rot; std::vector<double> fang;   // thruster telemetry, fin angles (rad)
    double energy_Wh = 500, power_W = 0; bool flat = false;   // battery state
    double srate[3] = {1e9, 1e9, 1e9};         // fin servo speed per command group (rudder, elevator), in full strokes per second
    double pitch_i = 0;                                          // integral of the attitude assist's pitch loop
    std::vector<double> axial_lut; double axial_lut_dU = 0;     // the axial drag table sampled evenly in speed (built in rebuild()): a lookup is much cheaper than the log-log interpolation
    double cap[6] = {0, 0, 0, 0, 0, 0};         // largest force/moment the thrusters can make per axis

    double axial_lookup(double U) const {          // axial drag from the lookup table (the exact log-log curve beyond its end)
        if (axial_lut.empty()) return curve_drag(s.axial_curve, U);
        double x = U / axial_lut_dU; if (x >= (double)axial_lut.size() - 1) return curve_drag(s.axial_curve, U);
        int i = (int)x; double t = x - i; return axial_lut[i] * (1 - t) + axial_lut[i + 1] * t;
    }
    double fin_delta(const Fin& f) const {          // deflection angle of a fin (rad)
        if (f.group == 3) return std::max(-1.0, std::min(1.0, f.kr * defl[1] + f.ke * defl[2])) * f.max_defl;
        return f.group ? f.sign * defl[f.group] * f.max_defl : 0.0;
    }
    double volume() const { return s.mass / 1025.0 * (1.0 + s.trim); }
    V3 cg() const { V3 r = s.rg + V3{0, 0, cg_lift}; if (s.glider) { r.x -= mx * s.mass_x; r.y += my * s.mass_y; } return r; }

    // thruster allocation: pseudo-inverse of the thruster matrix, so a command like "surge" is shared out correctly
    void rebuild() {
        int n = (int)s.thr.size();
        thrust.assign(n, 0.0); alloc.assign(n, {}); rot.assign(n, Rotor{}); fang.assign(s.fins.size(), 0.0);
        energy_Wh = s.battery_Wh; power_W = s.hotel_W; flat = false;
        for (int k = 0; k < 6; ++k) cap[k] = 0;
        axial_lut.clear();
        if (!s.axial_curve.empty()) { const int N = 512; double Umax = 1.6 * s.axial_curve.back()[0]; axial_lut_dU = Umax / N; axial_lut.resize(N + 1); for (int i = 0; i <= N; ++i) axial_lut[i] = curve_drag(s.axial_curve, i * axial_lut_dU); }
        has_elev = has_rud = false; srate[1] = srate[2] = 1e9;
        for (const Fin& f : s.fins) {
            if (f.group == 0 || f.max_defl <= 0) continue;
            double r = s.servo_rate * M_PI / 180.0 / f.max_defl;
            if (f.group == 1 || f.group == 3) srate[1] = std::min(srate[1], r);
            if (f.group == 2 || f.group == 3) srate[2] = std::min(srate[2], r);
        }
        for (const Fin& f : s.fins) { if (f.group == 2 || (f.group == 3 && std::fabs(f.ke) > 0.3)) has_elev = true; if (f.group == 1 || (f.group == 3 && std::fabs(f.kr) > 0.3)) has_rud = true; }
        if (!n) return;
        std::vector<std::array<double, 6>> B(n);           // columns: wrench of each thruster
        for (int i = 0; i < n; ++i) {
            V3 F = s.thr[i].dir * s.thr[i].max_thrust, Mo = cross(s.thr[i].pos - V3{0, 0, 0}, F);
            double c[6] = {F.x, F.y, F.z, Mo.x, Mo.y, Mo.z};
            for (int k = 0; k < 6; ++k) { cap[k] += std::fabs(c[k]); }
            V3 d = s.thr[i].dir, Mu = cross(s.thr[i].pos, d);
            B[i] = {d.x, d.y, d.z, Mu.x, Mu.y, Mu.z};            // per newton of thrust
        }
        double G[6][6];                                       // G = B B^T + lambda I  (6x6)
        for (int a = 0; a < 6; ++a) for (int b = 0; b < 6; ++b) {
            double sum = 0; for (int i = 0; i < n; ++i) sum += B[i][a] * B[i][b];
            G[a][b] = sum + (a == b ? 1e-6 : 0.0);
        }
        // we want alloc = B^T G^-1 : solve G x = e_k for each column k
        double inv[6][6];
        for (int k = 0; k < 6; ++k) { double e[6] = {0, 0, 0, 0, 0, 0}, x[6]; e[k] = 1; solve_n(6, G, e, x); for (int a = 0; a < 6; ++a) inv[a][k] = x[a]; }
        for (int i = 0; i < n; ++i) for (int k = 0; k < 6; ++k) {
            double sum = 0; for (int a = 0; a < 6; ++a) sum += B[i][a] * inv[a][k];
            alloc[i][k] = sum;
        }
    }

    // ---------------------------------------------------------------------------------------------
    struct State { V3 p; Q q; V3 v, w; };
    struct Deriv { V3 dp; Q dq; V3 dv, dw; };

    Deriv eval(const State& st, const Env& env, double* sub_out = nullptr, double* buoy_out = nullptr, bool* touch = nullptr) const {
        const double m = s.mass, g = env.g, rho = env.rho;
        V3 rg = cg(), rb = s.rb;
        // how much of the vehicle is under the waves
        V3 cbw = st.p + qrot(st.q, rb);
        double eta = env.eta ? env.eta(st.p.x, st.p.y) : 0.0;
        double h = std::max(s.height, 0.05);
        double sub = std::max(0.0, std::min(1.0, (eta - (cbw.z - 0.5 * h)) / h));
        double hyd = 0.003 + 0.997 * sub;                 // water forces fade out in air

        V3 vc = qrot_inv(st.q, env.cur);                  // current seen from the body
        V3 vr = st.v - vc, wr = st.w;

        // mass matrix: rigid body about the body origin + added mass
        double Io[3][3];
        double r2 = dot(rg, rg); double rgv[3] = {rg.x, rg.y, rg.z};
        for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j)
            Io[i][j] = (i == j ? s.I[i] + m * r2 : 0.0) - m * rgv[i] * rgv[j];
        double M[6][6] = {};
        for (int i = 0; i < 3; ++i) M[i][i] = m + s.A[i] * am_scale * hyd;
        double S[3][3] = {{0, -rg.z, rg.y}, {rg.z, 0, -rg.x}, {-rg.y, rg.x, 0}};   // S(rg) a = rg x a
        for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j) {
            M[i][3 + j] = -m * S[i][j]; M[3 + i][j] = m * S[i][j]; M[3 + i][3 + j] = Io[i][j];
        }
        for (int i = 0; i < 3; ++i) M[3 + i][3 + i] += s.A[3 + i] * am_scale * hyd;

        // forces on the right-hand side
        V3 F, Mo;
        F -= (cross(st.w, st.v) + cross(st.w, cross(st.w, rg))) * m;                                // Coriolis, rigid body
        Mo -= cross(st.w, V3{Io[0][0] * st.w.x + Io[0][1] * st.w.y + Io[0][2] * st.w.z,
                             Io[1][0] * st.w.x + Io[1][1] * st.w.y + Io[1][2] * st.w.z,
                             Io[2][0] * st.w.x + Io[2][1] * st.w.y + Io[2][2] * st.w.z});
        Mo -= cross(rg, cross(st.w, st.v)) * m;
        V3 a = hmul(V3{s.A[0], s.A[1], s.A[2]}, vr) * (am_scale * hyd), b = hmul(V3{s.A[3], s.A[4], s.A[5]}, wr) * (am_scale * hyd);
        F -= cross(wr, a); Mo -= cross(vr, a) + cross(wr, b);                                       // Coriolis, added mass (Munk moment)
        V3 dl = {(s.Dl[0] + s.Dq[0] * std::fabs(vr.x)) * vr.x, (s.Dl[1] + s.Dq[1] * std::fabs(vr.y)) * vr.y, (s.Dl[2] + s.Dq[2] * std::fabs(vr.z)) * vr.z};
        V3 dr = {(s.Dl[3] + s.Dq[3] * std::fabs(wr.x)) * wr.x, (s.Dl[4] + s.Dq[4] * std::fabs(wr.y)) * wr.y, (s.Dl[5] + s.Dq[5] * std::fabs(wr.z)) * wr.z};
        if (!s.strips.empty()) {                          // per-strip drag: every slice feels the water velocity at its own position
            dl = {0, s.Dl[1] * vr.y, s.Dl[2] * vr.z}; dr.y = s.Dl[4] * wr.y; dr.z = s.Dl[5] * wr.z;     // only the linear terms stay in the matrices
            for (const DragStrip& sp : s.strips) {
                V3 r = {sp.x, sp.y, sp.z}, vs = vr + cross(wr, r);
                V3 f = {(s.axial_curve.empty() ? -(s.Dl[0] + s.Dq[0] * std::fabs(vs.x)) * vs.x : -(vs.x < 0 ? -1.0 : 1.0) * axial_lookup(std::fabs(vs.x))) * sp.ax, -sp.cy * std::fabs(vs.y) * vs.y, -sp.cz * std::fabs(vs.z) * vs.z};
                F += f * (drag_scale * hyd); Mo += cross(r, f) * (drag_scale * hyd);
            }
        }
        F -= dl * (drag_scale * hyd); Mo -= dr * (drag_scale * hyd);                                // drag
        V3 down = qrot_inv(st.q, V3{0, 0, -1});
        V3 Fg = down * (m * g);
        double V = volume() + (s.glider ? pump * s.pump_range : 0.0);
        double B = rho * g * V * sub;
        V3 Fb = down * (-B);
        F += Fg + Fb; Mo += cross(rg, Fg) + cross(rb, Fb);                                          // weight and buoyancy
        if (sub_out) *sub_out = sub;
        if (buoy_out) *buoy_out = B;

        for (size_t i = 0; i < s.thr.size(); ++i) {                                                 // thrusters
            V3 f = s.thr[i].dir * thrust[i]; F += f; Mo += cross(s.thr[i].pos, f) - s.thr[i].dir * (s.thr[i].kq * thrust[i]);
        }
        for (const Fin& f : s.fins) {                                                               // fins
            double delta = fin_delta(f);
            V3 vf = vr + cross(wr, f.pos);
            double ux = vf.x, wn = dot(vf, f.normal), V2 = ux * ux + wn * wn;
            if (V2 < 1e-6) continue;
            double al = delta - std::atan2(wn, ux);
            if (al > M_PI) al -= 2 * M_PI;
            if (al < -M_PI) al += 2 * M_PI;
            double sa = std::sin(al), ca = std::cos(al), aa = std::fabs(al);
            double sstall = std::min(1.0, std::max(0.0, (aa - f.stall) / 0.25)); sstall = sstall * sstall * (3 - 2 * sstall);
            double cn = (1 - sstall) * f.cl_alpha * sa * std::max(ca, 0.0) + sstall * 1.2 * sa * std::fabs(sa);
            double cd = 0.008 + 0.9 * std::fabs(cn * sa);
            double q = 0.5 * rho * hyd * f.area * V2;
            V3 ff = f.normal * (q * cn) + V3{1, 0, 0} * (-q * cd * ux / std::sqrt(V2));
            if (f.f_lim > 0 && f.group) { double fl = len(ff); if (fl > f.f_lim) ff = ff * (f.f_lim / fl); }       // the servo can not hold more
            F += ff; Mo += cross(f.pos, ff);
        }
        bool tch = false;
        double k = 30.0 * m * g, c = 2 * 0.8 * std::sqrt(k * m / std::max<size_t>(1, s.contact.size()));   // sea bed contact (spring + damper + friction)
        for (const V3& r : s.contact) {
            V3 pw = st.p + qrot(st.q, r);
            double gz = env.ground ? env.ground(pw.x, pw.y) : -1e9, pen = gz - pw.z;
            if (pen <= 0) continue;
            tch = true;
            double e = 0.4, gx = (env.ground(pw.x + e, pw.y) - env.ground(pw.x - e, pw.y)) / (2 * e), gy = (env.ground(pw.x, pw.y + e) - env.ground(pw.x, pw.y - e)) / (2 * e);
            V3 n = unit(V3{-gx, -gy, 1});
            V3 vp = qrot(st.q, st.v + cross(st.w, r));
            double vn = dot(vp, n), Fn = std::max(0.0, k * pen - c * vn);
            V3 vt = vp - n * vn; double vtl = len(vt);
            V3 Fw = n * Fn + vt * (-0.6 * Fn / (vtl + 0.03));
            V3 Fbd = qrot_inv(st.q, Fw); F += Fbd; Mo += cross(r, Fbd);
        }
        if (touch) *touch = tch;

        double rhs[6] = {F.x, F.y, F.z, Mo.x, Mo.y, Mo.z}, acc[6] = {0, 0, 0, 0, 0, 0};
        solve_n(6, M, rhs, acc);
        Deriv d;
        d.dp = qrot(st.q, st.v);
        d.dq = Q{st.q.w, st.q.x, st.q.y, st.q.z} * Q{0, st.w.x, st.w.y, st.w.z};
        d.dq = {0.5 * d.dq.w, 0.5 * d.dq.x, 0.5 * d.dq.y, 0.5 * d.dq.z};
        d.dv = {acc[0], acc[1], acc[2]}; d.dw = {acc[3], acc[4], acc[5]};
        return d;
    }

    static State advance(const State& a, const Deriv& d, double h) {
        State r;
        r.p = a.p + d.dp * h; r.q = {a.q.w + d.dq.w * h, a.q.x + d.dq.x * h, a.q.y + d.dq.y * h, a.q.z + d.dq.z * h};
        r.v = a.v + d.dv * h; r.w = a.w + d.dw * h; return r;
    }

    // one fixed step. The actuators (thrusters, fins, glider pump) are first-order lags with limits.
    void step(double dt, const Env& env) {
        auto lag = [&](double cur, double target, double tau) { return cur + (target - cur) * (1.0 - std::exp(-dt / std::max(tau, 1e-3))); };
        double c[5]; for (int k = 0; k < 5; ++k) c[k] = std::max(-1.0, std::min(1.0, cmd[k]));
        if (assist) {          // the stick asks for a pitch ANGLE (elevators) or a turn RATE (rudders); a small controller does the rest
            double yw, pt, rl; q_to_hpr(q, yw, pt, rl);
            if (has_elev) {
                // proportional + rate term as before, plus an integral of the pitch error: a proportional-only loop leaves a steady offset (the hull's nose-down moment at speed,
                // the thrust and the buoyancy trim all need some elevator), which showed up as a vehicle that pitched 2 to 4 deg nose-down under thrust and slowly dived
                double e = 0.45 * c[3] - pt, speed = std::fabs(v.x);
                double pd = s.assist_gain * (2.5 * e + 1.2 * w.y);
                if (speed > 0.3 && std::fabs(pd + pitch_i) < 1.0) pitch_i = std::max(-0.6, std::min(0.6, pitch_i + s.assist_gain * 0.5 * e * dt));      // frozen when the elevator is at its stop (no wind-up) or the fins have no authority
                else if (speed <= 0.3) pitch_i *= std::exp(-dt / 2.0);
                c[3] = std::max(-1.0, std::min(1.0, pd + pitch_i));
            }
            if (has_rud) c[4] = std::max(-1.0, std::min(1.0, s.assist_gain * (2.0 * (0.40 * c[4] - w.z))));
        }
        // thrusters
        if (!s.thr.empty()) {
            double tau_des[6] = {c[0] * cap[0], c[1] * cap[1], c[2] * cap[2], 0, -c[3] * cap[4], c[4] * cap[5]};
            std::vector<double> f(s.thr.size()); double worst = 1.0;
            for (size_t i = 0; i < f.size(); ++i) {
                f[i] = 0; for (int k = 0; k < 6; ++k) f[i] += alloc[i][k] * tau_des[k];
                worst = std::max(worst, std::fabs(f[i]) / (s.thr[i].max_thrust * (f[i] < 0 ? std::max(s.thr[i].rev, 1e-3) : 1.0)));   // reverse thrust is weaker: scale by the limit of the direction it is used in
            }
            for (size_t i = 0; i < f.size(); ++i) {
                const Thruster& T = s.thr[i];
                double want = flat ? 0.0 : std::max(-T.rev * T.max_thrust, std::min(T.max_thrust, f[i] / worst)) * thrust_scale;
                V3 wp = st_p() + qrot(q, T.pos);
                double eta = env.eta ? env.eta(wp.x, wp.y) : 0.0;
                double wet = std::max(0.0, std::min(1.0, (eta - wp.z) / 0.1 + 0.5));            // a propeller in air does nothing
                double adv = dot(v - qrot_inv(q, env.cur), T.dir);
                want *= wet * (1.0 - T.kdrop * std::max(0.0, std::min(1.0, adv / T.u_free)));        // thrust drops with forward speed
                thrust[i] = lag(thrust[i], want, T.tau);
                rot[i].wet = wet; rot[i].thrust = thrust[i]; rot[i].adv = adv;
            }
            if (!s.thr.empty()) prop_angle += dt * 60.0 * thrust[0] / std::max(1.0, s.thr[0].max_thrust) * (thrust[0] >= 0 ? 1 : 1);
        }
        // electrical power: propeller power ~ thrust^1.5, plus the hotel load; the battery drains and the thrusters stop when it is flat
        power_W = s.hotel_W;
        for (size_t i = 0; i < rot.size(); ++i) {
            double fr = std::fabs(thrust[i]) / std::max(1e-6, s.thr[i].max_thrust);
            rot[i].w = (thrust[i] < 0 ? -1 : 1) * 314.0 * std::sqrt(fr);
            const Thruster& T = s.thr[i]; double Tn = std::fabs(thrust[i]);
            if (T.zeta > 0) { double Va = std::max(0.0, rot[i].adv) * (1.0 - T.wake); rot[i].power = Tn * (0.5 * Va + std::sqrt(0.25 * Va * Va + Tn * T.disc_ik)) / T.zeta; }
            else rot[i].power = T.pk * std::pow(Tn, 1.5);
            power_W += rot[i].power;
        }
        energy_Wh = std::max(0.0, energy_Wh - power_W * dt / 3600.0);
        flat = energy_Wh <= 0.0;
        auto servo = [&](double cur, double target, double rate) { double d = target - cur, m = rate * dt; return cur + std::max(-m, std::min(m, d)); };
        defl[1] = servo(defl[1], c[4], srate[1]); defl[2] = servo(defl[2], c[3], srate[2]);     // rudder follows yaw, elevator follows pitch, at the servo speed
        if (s.glider) {                                                          // buoyancy engine and moving mass, rate limited
            auto slew = [&](double cur, double target, double time) { double d = target - cur, mxs = dt / time; return cur + std::max(-mxs, std::min(mxs, d)); };
            pump = slew(pump, c[2], s.pump_time); mx = slew(mx, c[3], 6.0); my = slew(my, c[4], 6.0);
        }
        for (size_t i = 0; i < s.fins.size() && i < fang.size(); ++i) fang[i] = fin_delta(s.fins[i]);
        // RK4
        Env e = env;
        State s0 = {p, q, v, w};
        double sb = 1, bf = 0; bool tc = false;
        Deriv k1 = eval(s0, e, &sb, &bf, &tc);
        Deriv k2 = eval(advance(s0, k1, dt / 2), e);
        Deriv k3 = eval(advance(s0, k2, dt / 2), e);
        Deriv k4 = eval(advance(s0, k3, dt), e);
        Deriv d;
        d.dp = (k1.dp + k2.dp * 2 + k3.dp * 2 + k4.dp) * (1.0 / 6);
        d.dq = {(k1.dq.w + 2 * k2.dq.w + 2 * k3.dq.w + k4.dq.w) / 6, (k1.dq.x + 2 * k2.dq.x + 2 * k3.dq.x + k4.dq.x) / 6,
                (k1.dq.y + 2 * k2.dq.y + 2 * k3.dq.y + k4.dq.y) / 6, (k1.dq.z + 2 * k2.dq.z + 2 * k3.dq.z + k4.dq.z) / 6};
        d.dv = (k1.dv + k2.dv * 2 + k3.dv * 2 + k4.dv) * (1.0 / 6);
        d.dw = (k1.dw + k2.dw * 2 + k3.dw * 2 + k4.dw) * (1.0 / 6);
        State r = advance(s0, d, dt);
        p = r.p; q = qnorm(r.q); v = r.v; w = r.w;
        sub = sb; buoy = bf; weight = s.mass * env.g; rho_now = env.rho; on_bottom = tc; t += dt;
    }
    V3 st_p() const { return p; }

    void place(V3 pos, double yaw) { p = pos; q = qfrom_hpr(yaw, 0, 0); v = {}; w = {}; for (double& x : thrust) x = 0; defl[0] = defl[1] = defl[2] = 0; pump = mx = my = 0; pitch_i = 0; hist.clear(); energy_Wh = s.battery_Wh; flat = false; }
    void heading_pitch_roll(double& yaw, double& pitch, double& roll) const { q_to_hpr(q, yaw, pitch, roll); }
};