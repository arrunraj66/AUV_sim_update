// physics/surface_waves.hpp - what a moving body and a spinning propeller do to the SURFACE of the sea.
// All of these are ADDED to the sea waves you already have (waves.hpp).  Sea level is z = 0, z up.
//
//   KelvinWake    : the ship-wave pattern of a body travelling just under the surface (Havelock source line, linear theory)
//                     - transverse waves with wavelength 2 pi U^2 / g, diverging waves, the 19.5 degree wedge
//                     - amplitude falls off like exp(-k h) with depth h, so a deep vehicle makes nothing and a shallow one a lot
//                     - the bow source (+) and stern sink (-) interfere, so the pattern depends on hull length (humps and hollows)
//   near_field    : the local rise at the nose and dip over the body from Bernoulli, eta = U u_x / g (with the surface as a rigid lid, x2)
//   PropBoil      : a jet that is near the surface heaves it up into a mound and a boil
//   RippleSystem  : ring waves that spread out from a disturbance (dispersive: wavelength decides speed, c_group = c_phase / 2)
//   SurfaceDisturbance : collects all of that: elevation(x, y, t) for the renderer
#pragma once
#include "flowfield.hpp"
#include <random>

namespace phys {

struct KelvinWake {
    struct Src { double x, m, h; };               // x along body (m), strength (m3/s, + bow, - stern), depth under the surface (m)
    std::vector<Src> src;
    double U = 0, yaw = 0, kmax = 2 * M_PI / 0.25;       // kmax: waves shorter than 0.25 m are ignored (below hull size, damped by viscosity)
    V3 pos; double x_ref = 0; int ntheta = 160; double theta_max = 1.35;
    double g = 9.80665;

    // build the sources from the hull slices: strength -U dS/dx (bow +, stern -), depth = depth of the hull axis
    void set_hull(const Hull& h, double speed, V3 position, double yaw_rad, double axis_depth) {
        U = std::max(std::fabs(speed), 0.0); pos = position; yaw = yaw_rad; src.clear();
        size_t n = h.st.size(); if (n < 2) return;
        std::vector<double> A(n); for (size_t i = 0; i < n; ++i) A[i] = h.area(h.st[i]);
        for (size_t i = 0; i < n; ++i) {
            double Ap = i + 1 < n ? A[i + 1] : 0.0, Am = i > 0 ? A[i - 1] : 0.0, m = -U * (Ap - Am) * 0.5;
            double a = std::sqrt(h.st[i].hw * h.st[i].hh);
            // sources are merged in groups of 4 slices to save time (the pattern changes by less than 1 %)
            src.push_back({h.st[i].x, m, std::max(axis_depth, 1.2 * a)});
        }
        merge(4);
        x_ref = 0.5 * (h.xa + h.xb);
    }
    void merge(int k) {
        std::vector<Src> o;
        for (size_t i = 0; i < src.size(); i += k) {
            double m = 0, xm = 0, hm = 0; int c = 0;
            for (size_t j = i; j < std::min(src.size(), i + k); ++j) { m += src[j].m; xm += src[j].x * std::fabs(src[j].m) + 1e-12 * src[j].x; hm += src[j].h; ++c; }
            double aw = 0; for (size_t j = i; j < std::min(src.size(), i + k); ++j) aw += std::fabs(src[j].m) + 1e-12;
            o.push_back({xm / aw, m, hm / c});
        }
        src = o;
    }
    double wavelength() const { return U > 0.05 ? 2 * M_PI * U * U / g : 1e9; }
    double froude(double length) const { return U / std::sqrt(g * length); }

    // per-angle table (depends only on the sources and U, not on the point), rebuilt automatically when they change
    struct Ang { double ct, st, k, w, Cr, Ci; };
    mutable std::vector<Ang> tab; mutable double key = -1;
    double make_key() const { double k = U * 1000 + ntheta + theta_max; for (const Src& q : src) k += q.m * 31.0 + q.x * 7.0 + q.h * 3.0; return k; }
    void prepare() const {
        tab.clear(); double k0 = g / (U * U), dth = 2 * theta_max / ntheta;
        for (int it = 0; it < ntheta; ++it) {
            double th = -theta_max + (it + 0.5) * dth, ct = std::cos(th), k = k0 / (ct * ct), damp = std::exp(-sqr(k / kmax));
            if (damp < 1e-3) continue;
            double Cr = 0, Ci = 0;
            for (const Src& q : src) { double a = q.m * std::exp(-k * q.h), ph = k * q.x * ct; Cr += a * std::cos(ph); Ci += a * std::sin(ph); }
            tab.push_back({ct, std::sin(th), k, damp * k / ct * dth / (M_PI * U), Cr, Ci});     // w = damp * k0 sec^3 dtheta / (pi U)
        }
        key = make_key();
    }
    // wave elevation at world (X, Y)
    double elevation(double X, double Y) const {
        if (U < 0.15 || src.empty()) return 0;
        if (key != make_key()) prepare();
        double c = std::cos(yaw), s = std::sin(yaw), dx = X - pos.x, dy = Y - pos.y;
        double xb = c * dx + s * dy, yb = -s * dx + c * dy, sum = 0;       // position in the body-aligned frame (x forward)
        for (const Ang& a : tab) {
            double down = (x_ref - xb) * a.ct + yb * a.st;                  // how far downstream of the hull along this wave direction
            double win = 0.5 * (1 + std::tanh(down * a.k / 1.5)); if (win < 1e-3) continue;
            double ph = a.k * (-xb * a.ct + yb * a.st);
            sum += win * a.w * (a.Cr * std::cos(ph) - a.Ci * std::sin(ph));  // k0 sec^3(theta) e^(-k h) cos(phase)
        }
        return sum;
    }
};

// local rise and dip from the displacement flow, eta = 2 U u_x / g (x2: surface acts as a rigid lid at low Froude number)
inline double near_field_eta(const HullFlow& hf, V3 body_pos_world, Q q, V3 world_point, double U, double g = 9.80665) {
    V3 pb = qrot_inv(q, world_point - body_pos_world);
    V3 u = qrot(q, hf.velocity(pb));
    V3 fwd = qrot(q, V3{1, 0, 0}); double ux = dot(u, fwd) * (U >= 0 ? 1.0 : -1.0);
    double e = 2 * std::fabs(U) * ux / g;
    double cap = 0.5 * U * U / g;
    return std::max(-cap, std::min(cap, e));
}

// ------------------------------------------------------------------------------------------------ jet near the surface
struct PropBoil {
    // surface heave from a jet.  Evaluated along the jet; a jet deeper than ~3 widths does nothing to the surface.
    static double eta(const PropJet& j, double X, double Y, double g = 9.80665) {
        if (!j.on) return 0;
        double best = 0, L = 14 * 2 * j.R, step = std::max(0.02, L / 40);
        for (double x = 0; x <= L; x += step) {
            V3 c = j.pos + j.dir * x; double b = j.radius_at(x), depth = -c.z;
            if (depth > 3.2 * b) continue;
            double uc = j.centre_speed(x);
            double mound = uc * uc / (2 * g) * std::exp(-sqr(depth / (1.5 * b)));
            mound = std::min(mound, 0.3 * b);                                   // the surface breaks and sprays instead of rising further
            double rh = std::hypot(X - c.x, Y - c.y);
            best = std::max(best, mound * std::exp(-sqr(rh / (1.1 * b))));
        }
        return best;
    }
    // how hard the jet disturbs the surface (0..1), used to drive the ripple emitter and the spray/aeration:
    // the best place along the jet, where it is still fast but already wide enough to reach the surface
    static double intensity(const PropJet& j) {
        if (!j.on) return 0;
        double best = 0, L = 14 * 2 * j.R, step = std::max(0.02, L / 40);
        for (double x = 0; x <= L; x += step) {
            V3 c = j.pos + j.dir * x; double b = j.radius_at(x), depth = -c.z;
            double reach = 1 - depth / std::max(3.0 * b, 1e-3);
            if (reach <= 0) continue;
            best = std::max(best, reach * std::min(1.0, j.centre_speed(x) / 3.0));
        }
        return std::min(1.0, best);
    }
};

// ------------------------------------------------------------------------------------------------ ripples
struct RippleSystem {
    struct Ring { double x, y, t0, A, lam; };
    std::vector<Ring> rings; size_t max_rings = 120; double g = 9.80665, life = 40; std::mt19937 rng{12345};
    void emit(double x, double y, double t, double amp, double lam) { rings.push_back({x, y, t, amp, lam}); if (rings.size() > max_rings) rings.erase(rings.begin()); }
    // random wavelengths 0.15 .. 0.5 m, amplitude proportional to the boil intensity
    void emit_random(double x, double y, double t, double amp) {
        std::uniform_real_distribution<double> d(0.0, 1.0);
        emit(x + 0.05 * (d(rng) - 0.5), y + 0.05 * (d(rng) - 0.5), t, amp, 0.15 + 0.35 * d(rng));
    }
    static double phase_speed(double lam, double g = 9.80665) { double k = 2 * M_PI / lam; return std::sqrt(g / k + 0.074 / 1025.0 * k); }  // gravity + surface tension
    double elevation(double X, double Y, double t) const {
        double e = 0;
        for (const Ring& r : rings) {
            double age = t - r.t0; if (age < 0 || age > life) continue;
            double k = 2 * M_PI / r.lam, cp = phase_speed(r.lam), cg = 0.5 * cp;   // deep water group speed
            double R = std::hypot(X - r.x, Y - r.y), Rg = cg * age;
            double w = 2 * r.lam + 0.12 * Rg;
            double d = (R - Rg) / w; if (std::fabs(d) > 3.5) continue;
            double amp = r.A * std::sqrt(0.15 / std::max(R, 0.15)) * std::exp(-age / 14.0) * std::exp(-d * d);
            e += amp * std::cos(k * (R - cp * age));
        }
        return e;
    }
    void step(double t) { rings.erase(std::remove_if(rings.begin(), rings.end(), [&](const Ring& r) { return t - r.t0 > life; }), rings.end()); }
};

// ------------------------------------------------------------------------------------------------ everything together
struct SurfaceDisturbance {
    struct Vessel { KelvinWake kelvin; const VehicleFlow* flow = nullptr; double depth = 99; };
    std::vector<Vessel> vessels; RippleSystem ripples; double ring_clock = 0;

    // call once per frame per vehicle (the Kelvin sources are rebuilt only when needed)
    void update_vessel(size_t i, const VehicleFlow& vf, const Hull& hull, double yaw, double axis_depth) {
        if (vessels.size() <= i) vessels.resize(i + 1);
        Vessel& v = vessels[i]; v.flow = &vf; v.depth = axis_depth;
        double U = len(V3{vf.vel_world.x, vf.vel_world.y, 0});
        v.kelvin.set_hull(hull, U, vf.pos, yaw, std::max(axis_depth, 0.0));
    }
    // emit ripples where propellers are close to the surface
    void step(double t, double dt, const std::vector<PropJet>& jets) {
        ring_clock += dt;
        for (const PropJet& j : jets) {
            double I = PropBoil::intensity(j);
            if (I > 0.02 && ring_clock > 0.12) { ripples.emit_random(j.pos.x + j.dir.x * 4 * j.R, j.pos.y + j.dir.y * 4 * j.R, t, 0.02 * I + 0.004); }
        }
        if (ring_clock > 0.12) ring_clock = 0;
        ripples.step(t);
    }
    double elevation(double X, double Y, double t, const std::vector<PropJet>& jets) const {
        double e = ripples.elevation(X, Y, t);
        for (const Vessel& v : vessels) {
            if (v.depth > 6.0 * std::max(0.5, v.kelvin.U * v.kelvin.U / 9.80665)) continue;    // too deep to matter
            e += v.kelvin.elevation(X, Y);
            if (v.flow && v.flow->hull.hull) {
                e += near_field_eta(v.flow->hull, v.flow->pos, v.flow->q, V3{X, Y, 0.0}, v.kelvin.U);   // point on the surface
            }
        }
        for (const PropJet& j : jets) e += PropBoil::eta(j, X, Y);
        return e;
    }
};

}  // namespace phys