// physics/proximity.hpp - a hull close to the seabed or the sea surface, and a propeller jet hitting the seabed.
//   Proximity : image-flow (method of images) solution for a slender body parallel to a wall:
//                 added mass of a slice:  m = m0 (1 + (a / 2h)^2)         (a = slice radius, h = axis height above the wall)
//                 Bernoulli suction:      F = pi rho a^4 U^2 / (4 h^3)   per metre, towards the wall  (-d/dh of the kinetic energy)
//               the sea surface acts as a second wall at low Froude number (rigid lid); at Fr > ~1 waves take over (factor fades out)
//   Scour     : jet on the seabed -> wall-jet speed -> bed shear -> Shields number (Soulsby 1997) -> pickup (van Rijn 1984)
//               -> sediment cloud (the muddy plume that blinds cameras and sonar)
#pragma once
#include "wake.hpp"
#include "hull.hpp"
#include "seawater.hpp"

namespace phys {

struct Proximity {
    V3 F, M;                  // body frame
    double ca_bed = 1, ca_surf = 1, f_lid = 1, gap_min = 99;
    // bed_z, surf_z : world heights of the sea bed and the (calm) surface
    void eval(const Hull& h, V3 pos, Q q, double speed, double rho, double bed_z, double surf_z, double length_for_fr) {
        F = {}; M = {}; double Fr = std::fabs(speed) / std::sqrt(G * std::max(surf_z - bed_z, 0.5));
        f_lid = std::exp(-sqr(Fr / 0.8));
        double ma = 0, mb = 0, ms = 0; gap_min = 99;
        for (const Strip& s : h.st) {
            V3 pb{s.x, 0, 0}, pw = pos + qrot(q, pb);
            double a = std::sqrt(s.hw * s.hh), hb = pw.z - bed_z, hs = surf_z - pw.z;
            double dz = 0;
            if (hb > 1.05 * a && hb < 12 * a) { double Fm = M_PI * rho * std::pow(a, 4) * speed * speed / (4 * std::pow(hb, 3)) * s.dx; dz -= Fm; mb += (1 + sqr(a / (2 * hb))) * s.dx; }
            else mb += s.dx;
            if (hs > 1.05 * a && hs < 12 * a) { double Fm = M_PI * rho * std::pow(a, 4) * speed * speed / (4 * std::pow(hs, 3)) * s.dx * f_lid; dz += Fm; ms += s.dx; }
            else ms += s.dx;
            gap_min = std::min(gap_min, std::min(hb - s.hh, hs - s.hh));
            V3 fb = qrot_inv(q, V3{0, 0, dz}); F += fb; M += cross(pb, fb); ma += s.dx;
        }
        ca_bed = mb / std::max(ma, 1e-9);
        // the free surface reduces added mass for fast (wave-making) heave motion: image has opposite sign, 1 - (a/2h)^2
        double sum = 0; ma = 0;
        for (const Strip& s : h.st) { V3 pw = pos + qrot(q, V3{s.x, 0, 0}); double a = std::sqrt(s.hw * s.hh), hs = surf_z - pw.z; double f = hs > 1.05 * a ? 1 - sqr(a / (2 * hs)) : 0.75; sum += f * s.dx; ma += s.dx; }
        ca_surf = sum / std::max(ma, 1e-9);
    }
};

struct Scour {
    double d50 = 0.0003, rho_s = 2650;               // medium sand
    struct Out { double u_bed = 0, tau = 0, theta = 0, theta_cr = 0, conc = 0, pickup_depth = 0; bool mobile = false; V3 where; };
    // largest bed shear under a jet; bed_z is the world height of the sea floor
    Out eval(const PropJet& j, double bed_z, double rho_w, double nu) const {
        Out o; if (!j.on) return o;
        double s = (rho_s / rho_w), Ds = d50 * std::cbrt(G * (s - 1) / (nu * nu));
        o.theta_cr = 0.30 / (1 + 1.2 * Ds) + 0.055 * (1 - std::exp(-0.020 * Ds));
        double best = 0, L = 40 * 2 * j.R;
        for (double x = 0; x < L; x += 0.5 * j.R) {
            V3 c = j.pos + j.dir * x; if (c.z - bed_z > 6 * j.radius_at(x)) continue;
            V3 pb{c.x, c.y, bed_z}; double u = len(j.velocity(pb));
            if (c.z < bed_z) break;
            // a jet flowing along the bed is a wall jet: the near-bed speed is ~0.75 of the free jet speed, at an impinging jet slightly less
            if (u * 0.75 > best) { best = u * 0.75; o.where = pb; }
        }
        o.u_bed = best; double Cf = 0.0128;                         // wall jet friction coefficient (Poreh et al. 1967)
        o.tau = 0.5 * rho_w * Cf * best * best;
        o.theta = o.tau / ((rho_s - rho_w) * G * d50);
        o.mobile = o.theta > o.theta_cr;
        if (o.mobile) {
            double T = (o.theta - o.theta_cr) / o.theta_cr, a = 0.01;           // reference height 1 cm
            o.conc = 0.015 * rho_s * (d50 / a) * std::pow(T, 1.5) / std::pow(Ds, 0.3);   // kg/m3 at the reference height (van Rijn)
            o.conc = std::min(o.conc, 20.0);
        }
        return o;
    }
};

}  // namespace phys