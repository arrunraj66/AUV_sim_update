// physics/tether.hpp - a cable between a vehicle and a fixed point (surface buoy, winch, anchor) or trailing free.
//   Lumped-mass model: n+1 nodes joined by springs that only pull (a slack cable does not push), with axial damping.
//   Per node: wet weight (weight - buoyancy of the cable), added mass, and Morison drag on the water velocity
//             normal : 1/2 rho d Cdn |v_n| v_n          tangential : 1/2 rho pi d Cdt |v_t| v_t
//   step(dt, end_pos, end_vel): end_pos is where the vehicle end is at the start of the step.
//   Node 0 is the fixed end (or free if fixed = false), node n follows the vehicle.  end_force() is what the cable pulls the vehicle with.
#pragma once
#include "wake.hpp"
#include <functional>

namespace phys {

struct Tether {
    int n = 20;                     // segments
    double L = 10, d = 0.01;        // length (m), diameter (m)
    double EA = 5.0e4;              // axial stiffness (N)
    double mass = 0.2;              // dry mass per length (kg/m)
    double wet_w = 2.0;             // submerged weight per length (N/m), + sinks
    double Cdn = 1.2, Cdt = 0.02, Ca = 1.0, c_ax = 0.02, extra_damp = 0.0;   // c_ax: axial damping fraction of critical
    bool fixed = true; double rho = 1025;
    std::vector<V3> p, v; V3 Fend;

    void init_line(V3 a, V3 b) {
        p.resize(n + 1); v.assign(n + 1, V3{});
        for (int i = 0; i <= n; ++i) p[i] = a + (b - a) * (double(i) / n);
    }
    // water velocity at a point can be a flow field (current + waves + wakes)
    void step(double dt, V3 end_pos, V3 end_vel, const std::function<V3(V3)>& water = nullptr) {
        double l0 = L / n, m_node = (mass + Ca * rho * M_PI * d * d / 4) * l0, kseg = EA / l0;
        double c_node = std::sqrt(kseg / m_node);                            // stiffest wave: fastest rate in the model
        int nsub = std::max(1, (int)std::ceil(dt * c_node / 0.4));
        double h = dt / nsub;
        for (int s = 0; s < nsub; ++s) {
            p[n] = end_pos + end_vel * (h * s); v[n] = end_vel;      // end_pos = vehicle end at the START of this step
            std::vector<V3> f(n + 1);
            for (int i = 0; i <= n; ++i) {
                V3 w = water ? water(p[i]) : V3{};
                V3 tdir = unit((i < n ? p[i + 1] : p[i]) - (i > 0 ? p[i - 1] : p[i]));
                V3 vr = v[i] - w, vt = tdir * dot(vr, tdir), vn = vr - vt;
                double wgt = (i == 0 || i == n) ? 0.5 : 1.0;
                f[i] += V3{0, 0, -wet_w * l0 * wgt};
                f[i] -= vn * (0.5 * rho * d * l0 * wgt * Cdn * len(vn));
                f[i] -= vt * (0.5 * rho * M_PI * d * l0 * wgt * Cdt * len(vt));
                f[i] -= v[i] * (extra_damp * l0);
            }
            for (int i = 0; i < n; ++i) {
                V3 dl = p[i + 1] - p[i]; double l = len(dl); if (l < 1e-9) continue;
                V3 e = dl * (1.0 / l);
                double T = kseg * (l - l0) + 2 * c_ax * std::sqrt(kseg * m_node) * dot(v[i + 1] - v[i], e);
                if (T < 0) T = 0;                                              // a cable only pulls
                f[i] += e * T; f[i + 1] -= e * T;
            }
            { V3 dl = p[n - 1] - p[n]; double l = len(dl); V3 e = dl * (1.0 / l);
              double T = kseg * (l - l0) + 2 * c_ax * std::sqrt(kseg * m_node) * dot(v[n - 1] - v[n], e); Fend = e * std::max(T, 0.0); }                 // pull of the last segment on the vehicle
            for (int i = (fixed ? 1 : 0); i < n; ++i) { v[i] += f[i] * (h / m_node); p[i] += v[i] * h; }
        }
    }
    double tension_at_end() const { return len(Fend); }
    double length_now() const { double s = 0; for (int i = 0; i < n; ++i) s += len(p[i + 1] - p[i]); return s; }
};

}  // namespace phys
