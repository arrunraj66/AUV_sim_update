// physics/added_mass.hpp - the water that has to be accelerated together with the vehicle.
//
//   * Surge          : Lamb's k1 for an ellipsoid of the same length and diameter (strip theory cannot do surge).
//   * Sway / heave   : strip theory - every slice adds its 2-D added mass (ellipse: rho pi b^2, the half axis across
//                      the motion; rectangle: Lewis-form value), corrected for the 3-D ends with Lamb's k2.
//   * Pitch / yaw    : the same slices times x^2, plus the cross terms (Y_r, Z_q) when the hull is not symmetric fore/aft.
//   * Roll           : ellipse sections (rho pi/8 (a^2-b^2)^2) plus every fin (a flat plate of chord c: rho pi c^2/4 per span).
//   * Fins and wings : flat plates moving across their own plane.
// Result: the full 6x6 added-mass matrix M_A and the Coriolis term C_A(nu_r) (Munk moment and friends).
#pragma once
#include "hull.hpp"

namespace phys {

struct PlateAM { V3 pos, normal; double chord, span; };      // a fin or wing, for its added mass

struct AddedMass {
    double MA[6][6] = {};

    static void lamb(double L, double D, double& k1, double& k2, double& kp) {
        double a = 0.5 * L, b = 0.5 * D, e2 = 1 - (b * b) / (a * a);
        if (e2 < 0.01) { k1 = k2 = 0.5; kp = 0; return; }
        double e = std::sqrt(e2), lg = std::log((1 + e) / (1 - e));
        double a0 = 2 * (1 - e2) / (e2 * e) * (0.5 * lg - e), b0 = 1 / e2 - (1 - e2) / (2 * e2 * e) * lg;
        k1 = a0 / (2 - a0); k2 = b0 / (2 - b0); kp = e2 * e2 * (b0 - a0) / ((2 - e2) * (2 * e2 - (2 - e2) * (b0 - a0)));
    }

    void compute(const Hull& h, const std::vector<PlateAM>& plates, double rho) {
        for (int i = 0; i < 6; ++i) for (int j = 0; j < 6; ++j) MA[i][j] = 0;
        double k1, k2, kp; lamb(h.length, h.mean_diameter(), k1, k2, kp);
        double c3 = std::min(1.0, k2 / 0.97);                  // 3-D end correction of the strip sums
        double m22 = 0, m33 = 0, m22x = 0, m33x = 0, m22xx = 0, m33xx = 0, m44 = 0;
        for (const Strip& s : h.st) {
            double a = s.hw, b = s.hh;
            double sway = h.rect ? rho * M_PI * b * b * 1.36 : rho * M_PI * b * b;       // across the height when moving sideways
            double heave = h.rect ? rho * M_PI * a * a * 1.36 : rho * M_PI * a * a;
            double roll = h.rect ? 0.0234 * rho * M_PI * std::pow(2 * std::max(a, b), 4) * 0.5 : rho * M_PI / 8.0 * (a * a - b * b) * (a * a - b * b);
            m22 += sway * s.dx; m33 += heave * s.dx; m22x += sway * s.x * s.dx; m33x += heave * s.x * s.dx;
            m22xx += sway * s.x * s.x * s.dx; m33xx += heave * s.x * s.x * s.dx; m44 += roll * s.dx;
        }
        MA[0][0] = k1 * rho * h.volume;
        MA[1][1] = c3 * m22; MA[2][2] = c3 * m33;
        MA[4][4] = c3 * m33xx; MA[5][5] = c3 * m22xx;
        MA[1][5] = MA[5][1] = c3 * m22x;                         // sway-yaw coupling
        MA[2][4] = MA[4][2] = -c3 * m33x;                        // heave-pitch coupling (pitch +q moves the nose down)
        MA[3][3] = m44;
        for (const PlateAM& p : plates) {                         // fins and wings
            double mp = rho * M_PI * 0.25 * p.chord * p.chord * p.span;
            V3 n = unit(p.normal); double nv[3] = {n.x, n.y, n.z};
            V3 rn = cross(p.pos, n); double rv[3] = {rn.x, rn.y, rn.z};
            double col[6] = {nv[0], nv[1], nv[2], rv[0], rv[1], rv[2]};          // plate moves along n (and turns about r x n)
            for (int i = 0; i < 6; ++i) for (int j = 0; j < 6; ++j) MA[i][j] += mp * col[i] * col[j];
        }
    }
    // Coriolis of the added mass, as forces ON the body:  -C_A(nu_r) nu_r   (Fossen eq. 6.43)
    void coriolis(V3 vr, V3 w, V3& F, V3& Mo) const {
        double nu[6] = {vr.x, vr.y, vr.z, w.x, w.y, w.z}, p[6];
        for (int i = 0; i < 6; ++i) { p[i] = 0; for (int j = 0; j < 6; ++j) p[i] += MA[i][j] * nu[j]; }
        V3 p1 = {p[0], p[1], p[2]}, p2 = {p[3], p[4], p[5]};
        F = cross(p1, w) * 1.0;            // -(w x p1) = p1 x w
        Mo = cross(p1, vr) + cross(p2, w);  // -(vr x p1) - (w x p2)
    }
    double diag(int i) const { return MA[i][i]; }
};

}  // namespace phys