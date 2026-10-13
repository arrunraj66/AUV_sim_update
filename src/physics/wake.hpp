// physics/wake.hpp - the water that a propeller, a hull and a fin set in motion, and what it leaves behind.
//
//   PropJet     : steady slipstream of one propeller (actuator-disc momentum theory)
//                   - disc speed (Va+uj)/2, final jet speed uj = sqrt(Va^2 + 2T/(rho A)), contraction of the jet to R*sqrt(ud/uj)
//                   - turbulent round-jet spreading (Gaussian profile, spread rate 0.107, slower in a co-flow)
//                   - swirl from the propeller torque (angular momentum is conserved while the jet widens)
//                   - suction (inflow) in front of the disc
//                   Used for: fins/rudders sitting in the wash, other vehicles, particles.  Moves with the vehicle.
//   WakeCloud   : what is left in the water after the vehicle has passed (it stays where it was made).
//                   - momentum blobs  : hull drag wake and the end of the propeller jet.  Impulse is conserved, the blob widens
//                                       by turbulent diffusion, so its speed falls like 1/b^3.
//                   - vortices        : trailing tip vortices of fins and rudders (Lamb-Oseen, circulation = lift / (rho U s))
//                   - particles       : bubbles (rise) and stirred-up sediment (settle)
//   boundary_layer_thickness : turbulent flat plate boundary layer, sets how wide the hull wake starts.
#pragma once
#include "../vmath.hpp"
#include <vector>
#include <cmath>
#include <algorithm>

namespace phys {

inline double sqr(double x) { return x * x; }

inline double boundary_layer_thickness(double x, double U, double nu) {          // turbulent flat plate, 0.37 x Re^-0.2
    double Re = std::max(1e3, U * x / nu);
    return 0.37 * x * std::pow(Re, -0.2);
}

// ------------------------------------------------------------------------------------------------ steady propeller jet
struct PropJet {
    V3 pos, dir;                                // disc centre, direction the water is pushed (world or body, you choose, but be consistent)
    double R = 0.05, T = 0, Q = 0, v0 = 0, rho = 1025, spin = 1, M = 0;
    double u_disc = 0, u_far = 0, R_far = 0, kj = 0.107;
    bool on = false;

    // axis: shaft direction in which positive thrust pushes the vehicle; torque: propeller torque on the water (N m, magnitude)
    // Va: water speed arriving at the disc along the thrust direction; handed: +1 if the propeller turns right-handed about the jet direction
    void set(V3 p, V3 axis, double D, double thrust, double torque, double Va, double rho_, double handed) {
        pos = p; R = 0.5 * D; rho = rho_; T = thrust; Q = std::fabs(torque); spin = handed; v0 = std::max(Va, 0.0);
        dir = unit(axis) * (thrust >= 0 ? -1.0 : 1.0);              // thrust on the vehicle along +axis means water goes along -axis
        double A = M_PI * R * R, aT = std::fabs(thrust);
        on = aT > 1e-3; M = aT;
        u_far = std::sqrt(v0 * v0 + 2 * aT / (rho * A));
        u_disc = 0.5 * (v0 + u_far);
        R_far = R * std::sqrt(u_disc / std::max(u_far, 1e-6));
        double d = u_far - v0; kj = std::max(0.02, 0.107 * d / (d + 2 * v0 + 1e-9));
    }
    double radius_at(double x) const {                          // 1/e radius of the Gaussian jet at distance x behind the disc
        double Rc = R_far + (R - R_far) * std::exp(-x / (0.6 * R));
        return 1.4142 * Rc + kj * x;
    }
    double centre_speed(double x) const {                       // excess speed on the axis (momentum flux is conserved)
        double b = radius_at(x);
        return -v0 + std::sqrt(v0 * v0 + 2 * M / (rho * M_PI * b * b));
    }
    // extra water velocity (on top of the ambient water) at p
    V3 velocity(V3 p) const {
        if (!on) return {};
        V3 rel = p - pos; double x = dot(rel, dir); V3 rv = rel - dir * x; double r = len(rv);
        double ui = u_disc - v0;
        if (x < 0) {                                            // suction in front of the disc
            double f = 1 + x / std::sqrt(x * x + R * R);
            return dir * (ui * f * std::exp(-sqr(r / (1.5 * R))));
        }
        double b = radius_at(x), uc = centre_speed(x);
        double w0 = std::min(1.0, ui / std::max(centre_speed(0), 1e-6));
        uc *= w0 + (1 - w0) * (1 - std::exp(-x / (0.5 * R)));    // smooth start from the disc speed
        double g = std::exp(-sqr(r / b));
        V3 v = dir * (uc * g);
        if (Q > 1e-9 && r > 1e-9) {
            double s = Q / (rho * 2 * M_PI * b * b * b * (0.5 * v0 + uc / 8));
            v += cross(dir, rv) * (spin * s * g / b);           // v_theta = s (r/b) e^-(r/b)^2
        }
        return v;
    }
    // momentum flux through a plane at distance x, integrated numerically (for checks)
    double momentum_flux(double x) const {
        double b = radius_at(x), uc = centre_speed(x), sum = 0, dr = b * 0.02;
        double w0 = std::min(1.0, (u_disc - v0) / std::max(centre_speed(0), 1e-6));
        uc *= w0 + (1 - w0) * (1 - std::exp(-x / (0.5 * R)));
        for (double r = 0.5 * dr; r < 5 * b; r += dr) { double u = uc * std::exp(-sqr(r / b)); sum += rho * (v0 + u) * u * 2 * M_PI * r * dr; }
        return sum;
    }
};

// ------------------------------------------------------------------------------------------------ what stays behind
struct Blob { V3 pos, imp; double b0 = 0.1, nut = 1e-4, age = 0, life = 240; };
struct Vortex { V3 pos, axis; double gamma = 0, rc0 = 0.02, len = 0.1, nut = 1e-4, age = 0; };
struct Particles { V3 pos; double conc = 0, b0 = 0.1, vz = 0, age = 0, tau = 30; int kind = 0; };   // kind 0 bubbles, 1 sediment

struct WakeCloud {
    std::vector<Blob> blobs; std::vector<Vortex> vort; std::vector<Particles> part;
    size_t max_blobs = 1500, max_vort = 600, max_part = 800;
    double rho = 1025, nu = 1.19e-6, surface_z = 0.0, floor_z = -1e9, tau_vortex = 30.0;
    V3 current;                                    // ambient current that carries everything
    double acc = 0;

    // --- sources -------------------------------------------------------------------------------------------
    // impulse (N s) given to the water at pos; merges with a very recent blob from the same place so the cloud stays small
    void add_impulse(V3 pos, V3 imp, double b0) {
        double m = len(imp); if (m < 1e-9) return;
        for (int i = (int)blobs.size() - 1; i >= 0 && i >= (int)blobs.size() - 6; --i) {
            Blob& o = blobs[i];
            if (o.age < 0.25 && len(o.pos - pos) < 0.5 * b0 && dot(o.imp, imp) > 0) {
                double w = len(o.imp); o.pos = (o.pos * w + pos * m) * (1.0 / (w + m)); o.imp += imp; o.b0 = std::max(o.b0, b0); return;
            }
        }
        Blob b; b.pos = pos; b.imp = imp; b.b0 = b0;
        double u0 = m / (rho * std::pow(M_PI, 1.5) * b0 * b0 * b0);
        b.nut = 0.015 * u0 * b0 + 2e-5;
        blobs.push_back(b);
        if (blobs.size() > max_blobs) blobs.erase(blobs.begin(), blobs.begin() + (blobs.size() - max_blobs));
    }
    // the hull drags water along with it: impulse = drag * dt along the direction of travel
    void hull_wake(V3 stern, V3 travel_dir, double drag, double dt, double b0) { add_impulse(stern, unit(travel_dir) * (drag * dt), b0); }
    // the end of a propeller jet: impulse = thrust * dt along the jet direction
    void jet_end(V3 pos, V3 jet_dir, double thrust, double dt, double b0) { add_impulse(pos, unit(jet_dir) * (std::fabs(thrust) * dt), b0); }
    void tip_vortex(V3 pos, V3 axis, double gamma, double rc0, double len_) {
        Vortex v; v.pos = pos; v.axis = unit(axis); v.gamma = gamma; v.rc0 = rc0; v.len = len_;
        v.nut = 1e-5 + 0.01 * std::fabs(gamma); vort.push_back(v);
        if (vort.size() > max_vort) vort.erase(vort.begin(), vort.begin() + (vort.size() - max_vort));
    }
    void bubbles(V3 pos, double conc, double b0, double rise = 0.22) {
        Particles p; p.pos = pos; p.conc = conc; p.b0 = b0; p.vz = rise; p.kind = 0; p.tau = 25; part.push_back(p); trim_part();
    }
    // sediment: settling speed from Soulsby (1997), d50 in m
    static double settling_speed(double d50, double rho_s = 2650, double rho_w = 1025, double nu = 1.19e-6) {
        double Ds = d50 * std::cbrt(9.80665 * (rho_s / rho_w - 1) / (nu * nu));
        return nu / d50 * (std::sqrt(10.36 * 10.36 + 1.049 * Ds * Ds * Ds) - 10.36);
    }
    void sediment(V3 pos, double conc, double b0, double d50) {
        Particles p; p.pos = pos; p.conc = conc; p.b0 = b0; p.vz = -settling_speed(d50); p.kind = 1; p.tau = 90; part.push_back(p); trim_part();
    }
    void trim_part() { if (part.size() > max_part) part.erase(part.begin(), part.begin() + (part.size() - max_part)); }

    // --- the fields ----------------------------------------------------------------------------------------
    static double blob_b(const Blob& b) { return std::sqrt(b.b0 * b.b0 + 4 * b.nut * b.age); }
    V3 blob_velocity(const Blob& b, V3 p) const {
        double bb = blob_b(b); V3 d = p - b.pos; double d2 = dot(d, d);
        if (d2 > 16 * bb * bb) return {};
        return b.imp * (std::exp(-d2 / (bb * bb)) / (rho * std::pow(M_PI, 1.5) * bb * bb * bb));
    }
    static V3 vortex_velocity_of(const Vortex& v, V3 p) {
        double rc = std::sqrt(v.rc0 * v.rc0 + 4 * v.nut * v.age);
        V3 rel = p - v.pos; double z = dot(rel, v.axis); V3 rv = rel - v.axis * z; double r = len(rv);
        if (r > 5.0 || r < 1e-9) return {};                // far field is 1/r, cut off at 5 m
        double G = v.gamma * std::exp(-v.age / 30.0);
        double win = 0.5 * (std::erf((z + 0.5 * v.len) / rc) - std::erf((z - 0.5 * v.len) / rc));
        return cross(v.axis, rv) * (G / (2 * M_PI * r * r) * (1 - std::exp(-r * r / (rc * rc))) * win);
    }
    V3 velocity_at(V3 p) const {
        V3 u;
        for (const Blob& b : blobs) u += blob_velocity(b, p);
        for (const Vortex& v : vort) u += vortex_velocity_of(v, p);
        return u;
    }
    double concentration_at(V3 p, int kind) const {          // bubble void fraction or sediment mass concentration
        double c = 0;
        for (const Particles& q : part) if (q.kind == kind) {
            double bb = std::sqrt(q.b0 * q.b0 + 0.02 * q.age * q.b0 * 4 + 1e-6); V3 d = p - q.pos;
            double w = std::exp(-dot(d, d) / (bb * bb)); double v0 = std::pow(q.b0 / bb, 3.0);
            c += q.conc * v0 * std::exp(-q.age / q.tau) * w;
        }
        return c;
    }
    V3 net_impulse() const { V3 s; for (const Blob& b : blobs) s += b.imp; return s; }
    double max_speed() const { double m = 0; for (const Blob& b : blobs) m = std::max(m, len(b.imp) / (rho * std::pow(M_PI, 1.5) * std::pow(blob_b(b), 3))); return m; }

    // --- time stepping ---------------------------------------------------------------------------------------
    void step(double dt) {
        for (Blob& b : blobs) {
            b.age += dt; double u0 = len(b.imp) / (rho * std::pow(M_PI, 1.5) * std::pow(blob_b(b), 3));
            b.pos += (current + unit(b.imp) * u0) * dt;          // the stirred water drifts along with its own speed
        }
        blobs.erase(std::remove_if(blobs.begin(), blobs.end(), [&](const Blob& b) {
            return b.age > b.life || len(b.imp) / (rho * std::pow(M_PI, 1.5) * std::pow(blob_b(b), 3)) < 5e-4; }), blobs.end());
        // vortices push each other (a pair of tip vortices sinks and slows) - updated at 20 Hz to save time
        acc += dt;
        if (acc >= 0.05 && !vort.empty()) {
            std::vector<V3> vel(vort.size());
            for (size_t i = 0; i < vort.size(); ++i) {
                V3 u;
                for (size_t j = 0; j < vort.size(); ++j) if (j != i && sqr(len(vort[i].pos - vort[j].pos)) < 4.0) u += vortex_velocity_of(vort[j], vort[i].pos);
                vel[i] = u;
            }
            for (size_t i = 0; i < vort.size(); ++i) vort[i].pos += vel[i] * acc;
            acc = 0;
        }
        for (Vortex& v : vort) { v.age += dt; v.pos += current * dt; }
        vort.erase(std::remove_if(vort.begin(), vort.end(), [](const Vortex& v) { return v.age > 60 || std::fabs(v.gamma) * std::exp(-v.age / 30.0) < 1e-4; }), vort.end());
        for (Particles& p : part) {
            p.age += dt; p.pos += (current + V3{0, 0, p.vz}) * dt;
            if (p.kind == 1 && p.pos.z < floor_z) p.pos.z = floor_z;
        }
        part.erase(std::remove_if(part.begin(), part.end(), [&](const Particles& p) {
            return p.age > 4 * p.tau || (p.kind == 0 && p.pos.z > surface_z) || p.conc * std::exp(-p.age / p.tau) < 1e-5; }), part.end());
    }
};

}  // namespace phys