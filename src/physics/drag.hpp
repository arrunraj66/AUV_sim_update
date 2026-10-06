// physics/drag.hpp - viscous drag of the hull and of everything bolted onto it (Fossen's D(nu_r) nu_r).
//
//   Axial      : skin friction from the ITTC-1957 line Cf = 0.075/(log10 Re - 2)^2 on the wetted area, times a form
//                factor (Hoerner: 1 + 1.5 (D/L)^1.5 + 7 (D/L)^3), plus base / blunt-body pressure drag.
//   Cross flow : strip theory. Each slice feels 0.5 rho Cd (height or width) v|v| dx from the sideways water.
//                Cd depends on the shape of the section (thin -> 2.0, circle 1.2, long oval less) and on the
//                Reynolds number (the drag crisis of a cylinder above Re ~ 3e5). Turning makes the slices move
//                differently, which gives the yaw and pitch damping by itself.
//   Roll       : friction of the spinning surface.
//   Wave drag  : a hull running just under the surface makes waves (Froude number, depth / diameter).
//   Appendages : masts, skids, sensors, nozzles - each with areas and a drag coefficient along x, y, z.
#pragma once
#include "hull.hpp"
#include <vector>

namespace phys {

struct Appendage { V3 pos; double ax = 0, ay = 0, az = 0, cd = 1.0; };

inline double ittc_cf(double Re) { Re = std::max(Re, 1.0e4); double l = std::log10(Re) - 2.0; return 0.075 / (l * l); }

// cross-flow drag coefficient of a section: t = length along the flow / height across it, Re on the height
inline double crossflow_cd(double t, double Re, bool rect) {
    if (rect) return 2.05 - 0.5 * std::min(1.0, std::max(0.0, t - 1.0));           // sharp-edged box section
    double cd = 0.4 + 1.6 / (1.0 + std::pow(std::max(t, 0.0), 1.3));                // 2.0 flat plate ... 1.2 circle ... ~0.85 long oval
    double crisis = 1.0;                                                             // drag crisis of round sections
    if (Re > 2.0e5) { double u = std::min(1.0, (Re - 2.0e5) / 3.0e5); crisis = 1.0 - 0.6 * u; }
    if (Re > 5.0e5) crisis = std::min(0.75, 0.4 + 0.35 * std::min(1.0, (Re - 5.0e5) / 3.5e6));
    return cd * crisis;
}

struct Drag {
    double form = 0;                    // form factor; 0 = take Hoerner's from the shape
    double cd_base = 0.0;               // extra pressure drag on the frontal area (blunt ends)
    double cf_roll = 0.01;
    double crossflow_scale = 1.0;       // to tune the cross-flow against measurements
    bool wave_drag = true;
    std::vector<Appendage> app;

    struct Out { V3 F, M; double Xu = 0, Re = 0, cf = 0, wave = 1; };

    Out eval(const Hull& h, V3 vr, V3 wr, double rho, double nu, double depth, double g) const {
        Out o;
        double U = std::sqrt(vr.x * vr.x + vr.y * vr.y + vr.z * vr.z), ux = std::fabs(vr.x);
        double D = h.mean_diameter(), L = std::max(h.length, 0.05), dl = D / L;
        double k = form > 0 ? form : 1.0 + 1.5 * std::pow(dl, 1.5) + 7.0 * dl * dl * dl;
        o.Re = std::max(U, 0.01) * L / nu; o.cf = ittc_cf(o.Re);
        if (wave_drag) {
            double Fn = ux / std::sqrt(g * L), hd = std::max(depth, 0.0) / std::max(D, 0.05), f = Fn / 0.4;
            o.wave = 1.0 + 4.0 * std::exp(-1.5 * hd) * f * f / (1.0 + f * f * f * f);
        }
        double Cx = (o.cf * k * h.wetted + cd_base * h.front) * o.wave;
        o.F.x = -0.5 * rho * Cx * vr.x * ux; o.Xu = 0.5 * rho * Cx;
        for (const Strip& s : h.st) {
            double vy = vr.y + wr.z * s.x, vz = vr.z - wr.y * s.x;
            double cdy = crossflow_cd(s.hw / std::max(s.hh, 1e-6), std::fabs(vy) * 2 * s.hh / nu, h.rect) * crossflow_scale;
            double cdz = crossflow_cd(s.hh / std::max(s.hw, 1e-6), std::fabs(vz) * 2 * s.hw / nu, h.rect) * crossflow_scale;
            double fy = -0.5 * rho * cdy * (2 * s.hh) * vy * std::fabs(vy) * s.dx;     // sideways water hits the height of the slice
            double fz = -0.5 * rho * cdz * (2 * s.hw) * vz * std::fabs(vz) * s.dx;     // vertical water hits the width
            o.F.y += fy; o.F.z += fz; o.M.z += s.x * fy; o.M.y -= s.x * fz;
            double r = std::sqrt(s.hw * s.hh), vt = wr.x * r;
            o.M.x -= 0.5 * rho * cf_roll * vt * std::fabs(vt) * h.perim(s) * s.dx * r;
        }
        for (const Appendage& a : app) {
            V3 va = vr + cross(wr, a.pos);
            V3 f = {-0.5 * rho * a.cd * a.ax * va.x * std::fabs(va.x), -0.5 * rho * a.cd * a.ay * va.y * std::fabs(va.y), -0.5 * rho * a.cd * a.az * va.z * std::fabs(va.z)};
            o.F += f; o.M += cross(a.pos, f);
        }
        return o;
    }
};

}  // namespace phys