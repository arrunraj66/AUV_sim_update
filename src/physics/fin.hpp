// physics/fin.hpp - a fin, wing or stabiliser: planform geometry + foil physics + hinge moment.
//
// Geometry (the 3-D drawing uses the same numbers):
//   root  = quarter-chord point at the root (where it meets the hull), span_dir = root -> tip,
//   normal = direction of the lift force at zero deflection, chords cr / ct, span, sweep (tip moved aft), t/c.
//   From these: area, aspect ratio, taper, mean aerodynamic chord (MAC) and its spanwise place -> aerodynamic centre.
//   Fins on a hull see the hull as an end plate, so their effective aspect ratio is larger (ar_gain ~ 1.4-2).
// Deflection: positive delta turns the leading edge towards +normal -> force towards +normal.
// The fin turns about its hinge line (hinge_x = fraction of the MAC behind the leading edge).
// Output: force and moment on the vehicle, angle of attack, CL, CD and the HINGE MOMENT the servo has to fight.
#pragma once
#include "foil.hpp"

namespace phys {

struct FinGeom {
    V3 root, span_dir = {0, 1, 0}, normal = {0, 0, 1};
    double cr = 0.2, ct = 0.12, span = 0.15, sweep = 0.0, tc = 0.12;
    double ar_gain = 1.0, hinge_x = 0.22;     // hinge a little ahead of the quarter chord: the fin tends back to zero
    // derived
    double area = 0, ar = 1, taper = 1, mac = 0.1, y_mac = 0.05;
    V3 ac, hinge_pt; Foil foil;

    void setup() {
        span_dir = unit(span_dir); normal = unit(normal - span_dir * dot(normal, span_dir));
        area = 0.5 * span * (cr + ct); taper = ct / std::max(cr, 1e-6);
        ar = ar_gain * span * span / std::max(area, 1e-9);
        mac = (2.0 / 3.0) * cr * (1 + taper + taper * taper) / (1 + taper);
        y_mac = span / 6.0 * (1 + 2 * taper) / (1 + taper);
        double x_sweep = -sweep * y_mac / std::max(span, 1e-9);
        ac = root + span_dir * y_mac + V3{x_sweep, 0, 0};
        hinge_pt = ac + V3{(0.25 - hinge_x) * mac, 0, 0};
        foil = Foil::wing(ar, std::atan2(sweep, span), tc);
    }
    V3 chord_dir() const { V3 x = {1, 0, 0}; return unit(x - span_dir * dot(x, span_dir)); }
};

struct FinOut { V3 F, M; double alpha = 0, CL = 0, CD = 0, hinge = 0, Vp = 0; };

// vrel = velocity of the vehicle through the water (body frame), w = turn rate, wash = extra water speed at the fin
// (propeller slipstream, body frame), delta = deflection (rad), hyd = 0..1 how much of it is in the water
inline FinOut fin_force(const FinGeom& g, V3 vrel, V3 w, V3 wash, double delta, double rho, double nu, double hyd = 1.0) {
    FinOut o;
    V3 c0 = g.chord_dir(), n0 = g.normal;
    double cd = std::cos(delta), sd = std::sin(delta);
    V3 c = c0 * cd + n0 * sd, n = n0 * cd - c0 * sd;               // chord and normal after the deflection
    V3 V = vrel + cross(w, g.ac) - wash;
    FoilForce f = foil_force(g.foil, V, c, n, g.area, g.mac, rho, nu);
    o.F = f.F * hyd; o.M = cross(g.ac, o.F);
    o.alpha = f.alpha; o.CL = f.CL; o.CD = f.CD; o.Vp = f.Vp;
    // hinge moment about the hinge line (positive = tries to increase delta)
    double M_ac = f.q * g.area * g.mac * f.CM * hyd;                 // pitching moment about the quarter chord (nose-up +)
    double arm = (g.hinge_x - 0.25) * g.mac;                          // quarter chord is ahead of the hinge by arm
    double Fn = dot(o.F, n);
    o.hinge = M_ac + Fn * arm;
    return o;
}

}  // namespace phys