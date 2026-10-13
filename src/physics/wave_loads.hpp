// physics/wave_loads.hpp - what sea waves do to a vehicle that is under them.
//   AirySea : linear (Airy) waves, any number of components: surface elevation, water velocity and acceleration at any depth
//             - finite water depth dispersion  w^2 = g k tanh(k d)
//             - orbital motion falls off like exp(k z): a vehicle at depth z ~ L/2 feels almost nothing
//   WaveLoads : Morison loads on every hull slice (Froude-Krylov + added mass inertia, and drag on the relative cross-flow velocity)
//             The wave velocity is also what you subtract from the vehicle velocity to get the relative velocity Fossen's drag needs.
#pragma once
#include "hull.hpp"
#include "seawater.hpp"
#include "wake.hpp"

namespace phys {

struct AiryWave { double a, T, dir, phase, k, w; };

struct AirySea {
    std::vector<AiryWave> w; double depth = 1e6, g = G;
    static double wavenumber(double w, double d, double g = G) {
        double k = w * w / g;                                   // deep water first guess
        for (int i = 0; i < 40; ++i) { double t = std::tanh(k * d), f = g * k * t - w * w, df = g * t + g * k * d * (1 - t * t); k -= f / df; if (std::fabs(f) < 1e-12) break; }
        return k;
    }
    void add(double amp, double period, double dir_rad, double phase = 0) {
        double om = 2 * M_PI / period; w.push_back({amp, period, dir_rad, phase, wavenumber(om, depth, g), om});
    }
    double eta(double x, double y, double t) const {
        double e = 0; for (const AiryWave& c : w) e += c.a * std::cos(c.k * (x * std::cos(c.dir) + y * std::sin(c.dir)) - c.w * t + c.phase); return e;
    }
    // depth factors: horizontal cosh(k(z+d))/sinh(kd), vertical sinh(k(z+d))/sinh(kd), written so they never overflow
    void factors(const AiryWave& c, double z, double& ch, double& sh) const {
        double zz = std::min(z, 0.0), den = 1 - std::exp(-2 * c.k * depth);
        ch = (std::exp(c.k * zz) + std::exp(-c.k * (zz + 2 * depth))) / den;
        sh = (std::exp(c.k * zz) - std::exp(-c.k * (zz + 2 * depth))) / den;
    }
    V3 velocity(double x, double y, double z, double t) const {
        V3 v;
        for (const AiryWave& c : w) {
            double ch, sh; factors(c, z, ch, sh); double th = c.k * (x * std::cos(c.dir) + y * std::sin(c.dir)) - c.w * t + c.phase;
            double uh = c.a * c.w * ch * std::cos(th), uz = c.a * c.w * sh * std::sin(th);
            v += V3{uh * std::cos(c.dir), uh * std::sin(c.dir), uz};
        }
        return v;
    }
    V3 acceleration(double x, double y, double z, double t) const {
        V3 a;
        for (const AiryWave& c : w) {
            double ch, sh; factors(c, z, ch, sh); double th = c.k * (x * std::cos(c.dir) + y * std::sin(c.dir)) - c.w * t + c.phase;
            double ah = c.a * c.w * c.w * ch * std::sin(th), az = -c.a * c.w * c.w * sh * std::cos(th);
            a += V3{ah * std::cos(c.dir), ah * std::sin(c.dir), az};
        }
        return a;
    }
};

struct WaveLoads {
    V3 F, M;                                    // body frame, about the body origin
    double Cd = 1.0;
    // pos, q: vehicle pose (world); vb: vehicle velocity in the body frame; ab: vehicle acceleration (body frame, rough is fine)
    void eval(const Hull& h, const AirySea& sea, double t, V3 pos, Q q, V3 vb, V3 ab, double rho) {
        F = {}; M = {};
        for (const Strip& s : h.st) {
            V3 pb{s.x, 0, 0}, pw = pos + qrot(q, pb);
            if (pw.z > sea.eta(pw.x, pw.y, t)) continue;                   // slice is out of the water: no wave load
            V3 aw = qrot_inv(q, sea.acceleration(pw.x, pw.y, pw.z, t)), vw = qrot_inv(q, sea.velocity(pw.x, pw.y, pw.z, t));
            double A = h.area(s), Cay = s.hh / s.hw, Caz = s.hw / s.hh;     // ellipse added-mass coefficients for cross flow
            double Cax = 0.05;
            V3 vbs = vb + cross(V3{}, pb);                                  // body velocity at the slice (rotation is added by the caller if needed)
            V3 f;
            f.x = rho * A * s.dx * aw.x * (1 + Cax) - rho * A * s.dx * Cax * ab.x;
            f.y = rho * A * s.dx * aw.y * (1 + Cay) - rho * A * s.dx * Cay * ab.y;
            f.z = rho * A * s.dx * aw.z * (1 + Caz) - rho * A * s.dx * Caz * ab.z;
            V3 vr = vw - vbs;
            f.y += 0.5 * rho * Cd * (2 * s.hw) * s.dx * std::fabs(vr.y) * vr.y;
            f.z += 0.5 * rho * Cd * (2 * s.hh) * s.dx * std::fabs(vr.z) * vr.z;
            F += f; M += cross(pb, f);
        }
    }
};

}  // namespace phys