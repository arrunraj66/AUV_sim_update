// physics/foil.hpp - lift, drag and pitching moment of a foil section or a finite wing, at any angle (0-360 deg).
// Used by fins, rudders, wings AND by every element of a propeller blade.
//
//   attached flow : CL = a sin(alpha)       a = lift slope (2-D: 2 pi * 0.9;  3-D: Helmbold's low aspect ratio formula)
//   stall         : smooth change to flat-plate flow  CN = CN_max sin(alpha) (2.0 for 2-D, 1.2-1.3 for small wings)
//   drag          : skin friction (Reynolds number, thickness) + induced drag CL^2/(pi e AR) + separated flow
//   moment        : centre of pressure at 1/4 chord while attached, moving to 1/2 chord when the flow separates
//   reverse flow  : water arriving at the trailing edge (backing up) - same law, weaker lift (blunt edge leading)
#pragma once
#include "../vmath.hpp"
#include <algorithm>
#include <cmath>

namespace phys {

struct FoilCoef { double CL = 0, CD = 0, CM = 0, alpha = 0; };      // CM about the quarter chord, nose-up positive

struct Foil {
    double cla = 2 * M_PI * 0.9;     // lift slope, 1/rad
    double ar = 1e9;                 // aspect ratio (1e9 = 2-D section)
    double e = 0.85;                 // span efficiency
    double tc = 0.12;                // thickness / chord
    double alpha_stall = 0.24;       // rad
    double cn_max = 2.0;             // normal-force coefficient of fully separated flow at 90 deg
    double rev_eff = 0.7;            // lift in reverse flow / lift in normal flow

    static Foil section(double tc = 0.12) {                     // 2-D section (propeller blades)
        Foil f; f.tc = tc; f.ar = 1e9; f.cla = 2 * M_PI * 0.9; f.alpha_stall = (10.0 + 30.0 * tc) * M_PI / 180.0; f.cn_max = 2.0; return f;
    }
    static Foil wing(double ar, double sweep_rad = 0, double tc = 0.12) {   // finite wing (fins, rudders)
        Foil f; f.tc = tc; f.ar = ar; double t = std::tan(sweep_rad);
        f.cla = 2 * M_PI * ar / (2 + std::sqrt(4 + ar * ar * (1 + t * t)));
        f.alpha_stall = (14.0 + 40.0 / (ar + 2.0)) * M_PI / 180.0;
        f.cn_max = 1.1 + 0.9 / (1.0 + 0.5 * ar);              // low aspect ratio plates ~1.2 ... 2-D ~2
        return f;
    }
    // alpha in (-pi, pi], Re on the chord
    FoilCoef coef(double alpha, double Re) const {
        FoilCoef c; c.alpha = alpha;
        double a = alpha; bool rev = false;
        if (a > M_PI / 2) { a = M_PI - a; rev = true; } else if (a < -M_PI / 2) { a = -M_PI - a; rev = true; }
        double aa = std::fabs(a), sg = a >= 0 ? 1.0 : -1.0, sa = std::sin(aa), ca = std::cos(aa);
        double t = std::max(0.0, std::min(1.0, (aa - alpha_stall) / 0.20)), s = t * t * (3 - 2 * t);
        double Rl = std::max(Re, 1.0e4), l = std::log10(Rl) - 2.0, cf = 0.075 / (l * l);
        double cd0 = 2.0 * cf * (1.0 + 2.0 * tc + 60.0 * std::pow(tc, 4));
        double cl_att = cla * sa, cdi = ar < 1e8 ? cl_att * cl_att / (M_PI * e * ar) : 0.0;
        double cn = cn_max * sa, cl_sep = cn * ca, cd_sep = cn * sa;
        double CL = (1 - s) * cl_att + s * cl_sep;
        double CD = cd0 + (1 - s) * cdi + s * cd_sep;
        double xcp = 0.25 + s * 0.25 * sa;                     // centre of pressure, fraction of chord from the leading edge
        double CN = CL * ca + (CD - cd0) * sa;
        if (rev) { CL *= rev_eff; CN *= rev_eff; xcp = 1.0 - xcp; }
        c.CL = sg * CL; c.CD = CD; c.CM = -sg * CN * (xcp - 0.25);
        return c;
    }
};

// the force of a foil element in 3-D. c = chord direction (towards the leading edge), n = direction of positive lift,
// V = velocity of the element THROUGH the water, area = planform area, chord for the Reynolds number.
struct FoilForce { V3 F; double alpha = 0, CL = 0, CD = 0, CM = 0, q = 0, Vp = 0; };
inline FoilForce foil_force(const Foil& foil, V3 V, V3 c, V3 n, double area, double chord, double rho, double nu) {
    FoilForce r;
    double uc = dot(V, c), un = dot(V, n), Vp = std::sqrt(uc * uc + un * un);
    r.Vp = Vp; if (Vp < 1e-4) return r;
    double alpha = std::atan2(-un, uc);
    FoilCoef k = foil.coef(alpha, Vp * chord / nu);
    V3 dh = (c * uc + n * un) * (-1.0 / Vp);                 // drag: against the motion through the water
    V3 l0 = n - dh * dot(n, dh); double ll = len(l0);
    V3 lh = ll > 1e-9 ? l0 * (1.0 / ll) : V3{};               // lift: across the flow, towards +n
    r.q = 0.5 * rho * Vp * Vp;
    r.F = lh * (r.q * area * k.CL) + dh * (r.q * area * k.CD);
    r.alpha = alpha; r.CL = k.CL; r.CD = k.CD; r.CM = k.CM;
    return r;
}

}  // namespace phys