// physics/flowfield.hpp - how the water moves around a moving hull, and one place to ask "what is the water doing at this point?"
//
//   HullFlow  : slender-body potential flow.  A body moving through water pushes it out of the way:
//                 - a line of sources, strength -U dS/dx per length (S = cross-section area, body moving toward +x), makes the bow push water forward and
//                   outwards and the stern pull it back in (displacement flow, closed body so the sum of sources is zero)
//                 - a line of 3D doublets, strength 2 pi v a^2 per length, for sideways / up-down motion (cross flow); a turning body
//                   gets v(x) = v + r x along its length
//               gives the induced water velocity at any point outside the hull.
//   VehicleFlow : one vehicle = hull flow + its propeller jets + what it leaves in the cloud (hull wake, jet end, fin tip vortices)
//   FlowWorld   : all vehicles + the shared WakeCloud + ambient current -> velocity_at(p)
#pragma once
#include "hull.hpp"
#include "wake.hpp"
#include <memory>

namespace phys {

struct HullFlow {
    struct Line { double x, q, a, ay, az; };      // station x, source strength (m3/s per m), sizes
    const Hull* hull = nullptr;
    std::vector<double> A;                        // area of each strip
    double u = 0, v0 = 0, w0 = 0, wy = 0, wz = 0; // body velocity relative to the water (body frame) and turn rates

    void set_hull(const Hull& h) { hull = &h; A.resize(h.st.size()); for (size_t i = 0; i < A.size(); ++i) A[i] = h.area(h.st[i]); }
    void set_motion(V3 vr, V3 w) { u = vr.x; v0 = vr.y; w0 = vr.z; wy = w.y; wz = w.z; }

    bool inside(V3 p) const {
        if (!hull) return false;
        for (const Strip& s : hull->st) if (std::fabs(p.x - s.x) <= 0.5 * s.dx)
            return sqr(p.y / std::max(s.hw, 1e-6)) + sqr(p.z / std::max(s.hh, 1e-6)) < 1.0;
        return false;
    }
    // induced water velocity (body frame) at p (body frame, hull origin at x = 0)
    V3 velocity(V3 p) const {
        if (!hull || hull->st.empty() || inside(p)) return {};
        V3 out; const auto& st = hull->st; size_t n = st.size();
        for (size_t i = 0; i < n; ++i) {
            const Strip& s = st[i];
            double Ap = i + 1 < n ? A[i + 1] : 0.0, Am = i > 0 ? A[i - 1] : 0.0;
            double q = -u * (Ap - Am) * 0.5;                            // source strength of this slice (m3/s) = -U dS/dx dx (nose is a source)
            double a = std::sqrt(s.hw * s.hh), eps2 = sqr(0.7 * a);
            V3 R = p - V3{s.x, 0, 0}; double r2 = dot(R, R) + eps2, r3 = r2 * std::sqrt(r2), r5 = r3 * r2;
            out += R * (q / (4 * M_PI * r3));
            double vl = v0 + wz * s.x, wl = w0 - wy * s.x;              // cross-flow speed of this slice
            double lam_y = 2 * M_PI * vl * s.hh * s.hh * s.dx, lam_z = 2 * M_PI * wl * s.hw * s.hw * s.dx;   // 3D doublet moments
            V3 ny{0, 1, 0}, nz{0, 0, 1};
            out += (R * (3 * dot(ny, R) / r5) - ny * (1 / r3)) * (lam_y / (4 * M_PI));
            out += (R * (3 * dot(nz, R) / r5) - nz * (1 / r3)) * (lam_z / (4 * M_PI));
        }
        return out;
    }
    double net_source() const { double s = 0; for (size_t i = 0; i < A.size(); ++i) s -= u * ((i + 1 < A.size() ? A[i + 1] : 0.0) - (i > 0 ? A[i - 1] : 0.0)) * 0.5; return s; }
};

// ------------------------------------------------------------------------------------------------
struct VehicleFlow {
    HullFlow hull;
    std::vector<PropJet> jets;                    // world frame
    V3 pos; Q q; V3 vel_world;                    // pose and velocity through the water (world)
    double acc = 0, facc = 0;
    int id = 0;

    void set_pose(V3 p, Q qq, V3 vel_w) { pos = p; q = qq; vel_world = vel_w; }
    V3 to_body(V3 pw) const { return qrot_inv(q, pw - pos); }
    V3 to_world_vec(V3 vb) const { return qrot(q, vb); }
    V3 velocity(V3 pw) const {
        V3 u = to_world_vec(hull.velocity(to_body(pw)));
        for (const PropJet& j : jets) u += j.velocity(pw);
        return u;
    }
    // feed the cloud: call every physics step.  drag = total hull drag magnitude (N), thrusts come from the PropJets
    void emit(WakeCloud& cloud, double dt, double drag, double nu) {
        acc += dt; if (acc < 0.05) return;
        double U = len(vel_world);
        if (U > 0.08 && hull.hull) {
            V3 stern = pos + qrot(q, V3{hull.hull->xa, 0, 0});
            double b0 = std::max(0.5 * hull.hull->max_hh, boundary_layer_thickness(hull.hull->length, U, nu));
            cloud.hull_wake(stern, vel_world, drag, acc, b0);
        }
        for (const PropJet& j : jets) if (j.on) {
            double Lcut = 10 * 2 * j.R;
            double b0 = j.radius_at(Lcut);
            cloud.jet_end(j.pos + j.dir * Lcut, j.dir, j.M, acc, b0);
        }
        acc = 0;
    }
    // trailing vortices of a lifting surface: tips at world positions, lift_dir = direction the force pushes the vehicle
    void emit_foil(WakeCloud& cloud, V3 tipA, V3 tipB, V3 lift_dir, double lift, double rho, double dt) {
        facc += dt; if (facc < 0.05) return;                                   // one vortex segment every 50 ms
        double seg_t = facc; facc = 0;
        double U = len(vel_world); if (U < 0.1 || std::fabs(lift) < 0.05) return;
        double span = len(tipA - tipB), s = 0.785 * span;
        double G = lift / (rho * U * s);
        V3 vd = unit(vel_world), mid = (tipA + tipB) * 0.5, rA = tipA - mid;
        double sgn = dot(cross(vd, rA * -1.0), lift_dir * -1.0) > 0 ? 1.0 : -1.0;
        double seg = U * seg_t;
        cloud.tip_vortex(tipA, vd, sgn * G, 0.03 * span + 0.01, seg * 1.15);
        cloud.tip_vortex(tipB, vd, -sgn * G, 0.03 * span + 0.01, seg * 1.15);
    }
};

struct FlowWorld {
    WakeCloud cloud;
    std::vector<VehicleFlow*> vehicles;
    V3 current;
    // everything the water does at p, except the vehicle `skip` (a vehicle does not feel its own displacement flow)
    V3 velocity_at(V3 p, const VehicleFlow* skip = nullptr) const {
        V3 u = current + cloud.velocity_at(p);
        for (const VehicleFlow* v : vehicles) {
            if (v == skip) { for (const PropJet& j : v->jets) u += j.velocity(p); continue; }   // own jet is used for fins in the wash
            u += v->velocity(p);
        }
        return u;
    }
    void step(double dt) { cloud.current = current; cloud.step(dt); }
};

}  // namespace phys