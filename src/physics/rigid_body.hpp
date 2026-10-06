// physics/rigid_body.hpp - mass, centre of gravity and inertia built from the real parts of a vehicle,
// and the rigid-body terms of Fossen's equation:  M_RB nu_dot + C_RB(nu) nu = tau
//
// Add the parts (hull shell, battery, motors, payload ...) and the total mass, CG and full inertia tensor
// are worked out with the parallel-axis theorem. Moving a part (a glider's battery) moves the CG.
#pragma once
#include "../vmath.hpp"
#include <vector>
#include <string>

namespace phys {

struct Mat3 { double a[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}}; };
inline V3 mul(const Mat3& m, V3 v) { return {m.a[0][0] * v.x + m.a[0][1] * v.y + m.a[0][2] * v.z, m.a[1][0] * v.x + m.a[1][1] * v.y + m.a[1][2] * v.z, m.a[2][0] * v.x + m.a[2][1] * v.y + m.a[2][2] * v.z}; }

struct Part {
    std::string name; double m = 0; V3 c;      // mass (kg) and centre (body frame, m)
    Mat3 I;                                     // inertia about its own centre
};

struct MassProps {
    std::vector<Part> parts;
    double m = 0; V3 rg; Mat3 Ig, Io;           // total mass, CG, inertia about the CG and about the body origin

    void add_point(const std::string& n, double mass, V3 c) { Part p; p.name = n; p.m = mass; p.c = c; parts.push_back(p); }
    void add_box(const std::string& n, double mass, V3 c, V3 size) {          // solid box
        Part p; p.name = n; p.m = mass; p.c = c;
        p.I.a[0][0] = mass * (size.y * size.y + size.z * size.z) / 12; p.I.a[1][1] = mass * (size.x * size.x + size.z * size.z) / 12; p.I.a[2][2] = mass * (size.x * size.x + size.y * size.y) / 12;
        parts.push_back(p);
    }
    void add_cylinder_x(const std::string& n, double mass, V3 c, double r, double L, bool shell = false) {   // along the body x axis
        Part p; p.name = n; p.m = mass; p.c = c;
        double rr = shell ? r * r : 0.5 * r * r;
        p.I.a[0][0] = mass * rr; p.I.a[1][1] = p.I.a[2][2] = mass * (0.5 * rr + L * L / 12);
        parts.push_back(p);
    }
    Part* find(const std::string& n) { for (Part& p : parts) if (p.name == n) return &p; return nullptr; }

    void update() {                              // call after adding or moving parts
        m = 0; V3 s;
        for (const Part& p : parts) { m += p.m; s += p.c * p.m; }
        rg = m > 0 ? s * (1.0 / m) : V3{};
        Ig = Mat3{}; Io = Mat3{};
        for (const Part& p : parts) {
            for (int pass = 0; pass < 2; ++pass) {
                V3 d = p.c - (pass == 0 ? rg : V3{}); double dv[3] = {d.x, d.y, d.z}, d2 = dot(d, d);
                Mat3& T = pass == 0 ? Ig : Io;
                for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j)
                    T.a[i][j] += p.I.a[i][j] + p.m * ((i == j ? d2 : 0.0) - dv[i] * dv[j]);
            }
        }
    }
    // 6x6 rigid-body mass matrix about the body origin (Fossen eq. 3.44)
    void mass_matrix(double M[6][6]) const {
        double S[3][3] = {{0, -rg.z, rg.y}, {rg.z, 0, -rg.x}, {-rg.y, rg.x, 0}};
        for (int i = 0; i < 6; ++i) for (int j = 0; j < 6; ++j) M[i][j] = 0;
        for (int i = 0; i < 3; ++i) {
            M[i][i] = m;
            for (int j = 0; j < 3; ++j) { M[i][3 + j] = -m * S[i][j]; M[3 + i][j] = m * S[i][j]; M[3 + i][3 + j] = Io.a[i][j]; }
        }
    }
    // the rigid-body Coriolis and centripetal forces, returned as forces ON the body:  -C_RB(nu) nu
    void coriolis(V3 v, V3 w, V3& F, V3& Mo) const {
        F = (cross(w, v) + cross(w, cross(w, rg))) * (-m);
        Mo = cross(w, mul(Io, w)) * -1.0 - cross(rg, cross(w, v)) * m;
    }
    // weight in the body frame (q rotates body -> world), applied at the CG
    void gravity(const Q& q, double g, V3& F, V3& Mo) const {
        F = qrot_inv(q, V3{0, 0, -m * g}); Mo = cross(rg, F);
    }
};

}  // namespace phys