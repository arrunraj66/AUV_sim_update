// physics/hydrostatics.hpp - buoyancy and the restoring forces (Fossen's g(eta)).
//
//   * Fully under water: buoyancy = rho g V at the centre of buoyancy (CB).
//   * At the surface   : every hull slice is cut by the local wave surface, so the vehicle floats at the
//                        right draught, pitches and rolls on the waves, and the CB moves with the water line.
//   * Depth            : the water gets heavier (rho rises) and the hull is squeezed (hull compressibility),
//                        so a vehicle that is neutral at the surface is not neutral at 300 m.
//   * Variable buoyancy device (VBD, e.g. a glider's oil bladder) adds or removes volume at its own place.
//   * Stability numbers for the GUI: BG, GM (transverse and longitudinal), water-plane area.
#pragma once
#include "hull.hpp"
#include "seawater.hpp"
#include <functional>

namespace phys {

struct Hydrostatics {
    double V0 = 0.03;                  // displaced volume at the surface, m3 (whole vehicle, not just the hull slices)
    V3 cb;                             // centre of buoyancy when fully under water (body frame)
    double kappa = 2.5e-10;            // hull compressibility, 1/Pa (sea water is ~4.4e-10: aluminium hulls are stiffer)
    double vbd = 0, vbd_max = 0;       // variable buoyancy, m3 (+ = more volume, lighter)
    V3 vbd_pos;

    struct Out {
        double B = 0, frac = 1, V = 0;            // buoyancy (N), fraction under water, displaced volume
        V3 F, M;                                    // force and moment on the body (body frame), about the body origin
        V3 cb_now;                                  // centre of buoyancy now
        double awp = 0, gm_t = 0, gm_l = 0;         // water-plane area and metacentric heights (at the surface)
    };

    // submerged fraction and centroid offset of an ellipse/rectangle cut by a water line s = depth of centre / half height
    static void cut(double s, bool rect, double& f, double& zc) {
        s = std::max(-1.0, std::min(1.0, s));
        if (rect) { f = 0.5 * (1 + s); zc = -0.5 * (1 - s); return; }    // zc in units of half height, below centre is negative
        double A = std::acos(-s) + s * std::sqrt(1 - s * s); f = A / M_PI;
        zc = A > 1e-9 ? -2.0 * std::pow(1 - s * s, 1.5) / (3.0 * A) : 0.0;
    }

    // q: attitude (body->world), p: position of the body origin, eta(x,y): wave surface height,
    // rho: density at this depth, rg: centre of gravity (for GM)
    Out eval(const Hull& hull, V3 p, const Q& q, std::function<double(double, double)> eta, double rho, double g, V3 rg) const {
        Out o;
        V3 up_b = qrot_inv(q, V3{0, 0, 1});                    // world "up" seen in the body frame
        double ey = std::fabs(up_b.y), ez = std::fabs(up_b.z);
        double vol_all = 0, vol_sub = 0; V3 mom_sub, mom_all;
        double Iwt = 0, Iwl = 0, awp = 0;
        for (const Strip& s : hull.st) {
            V3 cw = p + qrot(q, V3{s.x, 0, 0});
            double h = eta ? eta(cw.x, cw.y) : 0.0, d = h - cw.z;            // depth of slice centre under the local surface
            double e = std::sqrt(s.hw * s.hw * up_b.y * up_b.y + s.hh * s.hh * up_b.z * up_b.z) + 1e-9;   // vertical half extent
            double f, zc; cut(d / e, hull.rect, f, zc);
            double A = hull.area(s) * s.dx;
            vol_all += A; mom_all += V3{s.x, 0, 0} * A;
            vol_sub += A * f; mom_sub += (V3{s.x, 0, 0} + up_b * (zc * e)) * (A * f);
            if (std::fabs(d) < e) {                                // slice crosses the surface: water-plane strip
                double half = (ey > ez ? s.hh : s.hw) * std::sqrt(std::max(0.0, 1 - (d / e) * (d / e)));
                awp += 2 * half * s.dx; Iwt += (2.0 / 3.0) * half * half * half * s.dx; Iwl += s.x * s.x * 2 * half * s.dx;
            }
        }
        double frac = vol_all > 0 ? vol_sub / vol_all : 1.0;
        V3 shift = vol_sub > 1e-12 ? mom_sub * (1.0 / vol_sub) - mom_all * (1.0 / std::max(vol_all, 1e-12)) : V3{};
        double depth = std::max(0.0, (eta ? eta(p.x, p.y) : 0.0) - p.z);
        double Vfull = V0 * (1.0 - kappa * (rho * g * depth)) ;
        o.V = Vfull * frac + vbd * (frac > 0.5 ? 1.0 : 0.0);
        o.B = rho * g * o.V;
        V3 cbv = cb + shift;
        o.cb_now = o.V > 0 ? (cbv * (Vfull * frac) + vbd_pos * (o.V - Vfull * frac)) * (1.0 / o.V) : cb;
        V3 Fb = up_b * o.B;
        o.F = Fb; o.M = cross(o.cb_now, Fb); o.frac = frac;
        o.awp = awp;
        double Vs = std::max(o.V, 1e-9), BG = dot(o.cb_now - rg, up_b);
        o.gm_t = BG + (frac < 0.999 ? Iwt / Vs : 0.0);
        o.gm_l = BG + (frac < 0.999 ? Iwl / Vs : 0.0);
        return o;
    }
};

}  // namespace phys