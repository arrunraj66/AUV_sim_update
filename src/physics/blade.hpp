// physics/blade.hpp - propeller blades by Blade Element Momentum Theory (BEMT).
//
// Each blade is cut into rings from the hub to the tip. On every ring:
//   * the blade section moves round at omega r and the water arrives at Va (ship speed x (1 - wake)),
//   * the propeller itself speeds the water up (axial induced velocity u_a) and swirls it (tangential u_t),
//   * the section has a geometric pitch angle  tan(theta) = P / (2 pi r)  and a foil section (foil.hpp),
//     so it sees an angle of attack and makes lift and drag -> thrust dT and torque dQ of the ring,
//   * momentum theory says the same thrust must speed the ring's water up:  dT = 4 pi r rho F |Va + u_a| u_a dr,
//     dQ = 4 pi r^2 rho F |Va + u_a| u_t dr  (F = Prandtl tip-loss factor; 1 inside a close-fitting duct).
// The two are solved together by iteration. Works for forward and reverse rotation and for water from behind.
#pragma once
#include "foil.hpp"
#include <vector>

namespace phys {

struct BladeGeom {
    int B = 3;                         // number of blades
    double D = 0.1;                    // diameter, m
    double hub = 0.2;                  // hub diameter / D
    double pd = 1.0;                   // pitch / diameter (constant pitch)
    double ear = 0.55;                 // expanded area ratio (total blade area / disc area) -> sets the chords
    double tc_root = 0.18, tc_tip = 0.06;
    bool ducted = false;               // close-fitting duct: no tip loss
    double chord(double x) const {     // x = r/R; Wageningen-like chord shape scaled to the area ratio
        double shape = 1.6 * std::sqrt(std::max(0.0, 1.0 - std::pow((x - 0.6) / 0.62, 2))) + 0.35;
        double R = 0.5 * D;
        return shape * ear * M_PI * R / (B * 1.55) * (ducted ? (0.75 + 0.25 * x) / 0.9 : 1.0);
    }
    double tc(double x) const { return tc_root + (tc_tip - tc_root) * (x - hub) / (1 - hub); }
};

struct BemOut { double T = 0, Q = 0, eta = 0, ua_mean = 0; };

// n = revolutions per second (signed), Va = speed of the water arriving along the thrust axis (+ = vehicle moving forward)
inline BemOut bem_solve(const BladeGeom& g, double n, double Va, double rho, double nu, int rings = 14) {
    BemOut o; if (std::fabs(n) < 1e-6) return o;
    const double R = 0.5 * g.D, rh = g.hub * R, dr = (R - rh) / rings, Om = 2 * M_PI * n;
    double sumA = 0;
    for (int i = 0; i < rings; ++i) {
        double r = rh + (i + 0.5) * dr, x = r / R, c = g.chord(x);
        double theta = std::atan(g.pd * g.D / (2 * M_PI * r));
        Foil foil = Foil::section(g.tc(x));
        V3 X = {1, 0, 0}, Tg = {0, 1, 0};                        // axial (thrust direction) and the direction of positive rotation
        V3 cdir = Tg * std::cos(theta) + X * std::sin(theta), ndir = X * std::cos(theta) - Tg * std::sin(theta);
        double ua = 0, ut = 0, dT = 0, dQ = 0;
        for (int it = 0; it < 60; ++it) {
            V3 V = X * (Va + ua) + Tg * (Om * r - ut);            // velocity of the blade section through the water
            FoilForce f = foil_force(foil, V, cdir, ndir, c * dr, c, rho, nu);
            dT = g.B * dot(f.F, X);
            dQ = -g.B * dot(f.F, Tg) * r;                         // torque the shaft must supply (+ along the rotation)
            double phi = std::atan2(std::fabs(Va + ua), std::fabs(Om * r - ut) + 1e-9);
            double F = 1.0;
            if (!g.ducted) { double ex = -g.B * (R - r) / (2 * r * std::max(std::sin(phi), 0.02)); F = std::max(0.05, 2 / M_PI * std::acos(std::exp(ex))); }
            double floorv = 0.02 * std::fabs(Om) * r + 1e-3;
            double m = 4 * M_PI * r * rho * F * dr;
            double ua_new = dT / (m * std::max(std::fabs(Va + ua), floorv));
            double ut_new = (Om >= 0 ? 1.0 : 1.0) * dQ / (m * r * std::max(std::fabs(Va + ua), floorv));
            if (Va + ua_new < 0 && Va > 0 && dT < 0) ua_new = std::max(ua_new, -0.5 * Va);   // turbine / brake state limit
            double k = 0.25;
            if (std::fabs(ua_new - ua) < 1e-5 && std::fabs(ut_new - ut) < 1e-5) { ua = ua_new; ut = ut_new; break; }
            ua += k * (ua_new - ua); ut += k * (ut_new - ut);
        }
        o.T += dT; o.Q += dQ; o.ua_mean += ua * 2 * M_PI * r * dr; sumA += 2 * M_PI * r * dr;
    }
    o.ua_mean /= std::max(sumA, 1e-12);
    double Pin = o.Q * Om; o.eta = (Pin > 1e-9 && o.T > 0 && Va > 0) ? o.T * Va / Pin : 0.0;
    return o;
}

}  // namespace phys