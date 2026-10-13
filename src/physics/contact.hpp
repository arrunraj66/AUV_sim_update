// physics/contact.hpp - the hull touching the sea floor, rocks or coral.
//   Points on every hull slice (8 around each section) are tested against
//     - a height-field sea floor z = floor(x, y)   (normal from the slope of the floor)
//     - spheres (rocks, coral heads, other bodies)
//   Normal force: Hunt-Crossley  F = k d^1.5 (1 - a v_n)   (Hertz-type stiffness, energy lost on impact, no sticking, never pulls)
//   Friction    : Coulomb with a smooth start,  F_t = mu F_n v_t / sqrt(|v_t|^2 + eps^2)
//   Result in the body frame: F, M, the deepest penetration and the hardest impact speed (for damage / sound)
#pragma once
#include "hull.hpp"
#include "wake.hpp"
#include <functional>

namespace phys {

struct Obstacle { V3 c; double r; };

struct Contact {
    std::function<double(double, double)> floor;      // sea floor height (world z), empty = no floor
    std::vector<Obstacle> spheres;
    double k = 2.0e5;            // N / m^1.5 per contact point
    double damp = 0.6;           // Hunt-Crossley dissipation, s/m
    double mu = 0.5;             // friction (sand ~0.4, rock ~0.7)
    double eps = 0.02;           // m/s, friction regularisation
    V3 F, M; double max_pen = 0, max_impact = 0; int points_in_contact = 0;

    V3 floor_normal(double x, double y) const {
        const double e = 0.05;
        double dzx = (floor(x + e, y) - floor(x - e, y)) / (2 * e), dzy = (floor(x, y + e) - floor(x, y - e)) / (2 * e);
        return unit(V3{-dzx, -dzy, 1.0});
    }
    void push(V3 pb, V3 pw, V3 vw, V3 n, double d, const Q& q) {
        double vn = dot(vw, n);
        double Fn = k * std::pow(d, 1.5) * std::max(0.0, 1.0 - damp * vn);          // vn < 0 = moving into the surface
        V3 vt = vw - n * vn; double vtm = std::sqrt(dot(vt, vt) + eps * eps);
        V3 fw = n * Fn - vt * (mu * Fn / vtm);
        V3 fb = qrot_inv(q, fw); F += fb; M += cross(pb, fb);
        ++points_in_contact; max_pen = std::max(max_pen, d); max_impact = std::max(max_impact, -vn);
    }
    // vb, wb: body velocity and angular velocity (body frame)
    void eval(const Hull& h, V3 pos, Q q, V3 vb, V3 wb) {
        F = {}; M = {}; max_pen = max_impact = 0; points_in_contact = 0;
        for (const Strip& s : h.st) for (int a = 0; a < 8; ++a) {
            double th = a * M_PI / 4;
            V3 pb{s.x, s.hw * std::cos(th), s.hh * std::sin(th)}, pw = pos + qrot(q, pb);
            V3 vw = qrot(q, vb + cross(wb, pb));
            if (floor) {
                double d = floor(pw.x, pw.y) - pw.z;
                if (d > 0) { V3 n = floor_normal(pw.x, pw.y); push(pb, pw, vw, n, d * n.z, q); }
            }
            for (const Obstacle& o : spheres) {
                V3 r = pw - o.c; double dist = len(r);
                if (dist < o.r && dist > 1e-9) push(pb, pw, vw, r * (1.0 / dist), o.r - dist, q);
            }
        }
    }
};

}  // namespace phys
