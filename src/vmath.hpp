// vmath.hpp - small double-precision maths for the vehicle physics (vectors, quaternions, 6x6 solver).
// Body frame: x forward, y left, z up.  World frame: x east, y north, z up (sea surface z = 0).
#pragma once
#include <cmath>
#include <algorithm>

struct V3 { double x = 0, y = 0, z = 0; };
inline V3 operator+(V3 a, V3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
inline V3 operator-(V3 a, V3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
inline V3 operator*(V3 a, double s) { return {a.x * s, a.y * s, a.z * s}; }
inline V3 operator*(double s, V3 a) { return {a.x * s, a.y * s, a.z * s}; }
inline V3& operator+=(V3& a, V3 b) { a = a + b; return a; }
inline V3& operator-=(V3& a, V3 b) { a = a - b; return a; }
inline double dot(V3 a, V3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline V3 cross(V3 a, V3 b) { return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x}; }
inline double len(V3 a) { return std::sqrt(dot(a, a)); }
inline V3 unit(V3 a) { double l = len(a); return l < 1e-12 ? V3{1, 0, 0} : a * (1.0 / l); }
inline V3 hmul(V3 a, V3 b) { return {a.x * b.x, a.y * b.y, a.z * b.z}; }   // component by component

struct Q { double w = 1, x = 0, y = 0, z = 0; };       // rotates body vectors into the world
inline Q operator*(Q a, Q b) {
    return {a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z, a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
            a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x, a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w};
}
inline V3 qrot(Q q, V3 v) {              // body -> world
    V3 u = {q.x, q.y, q.z};
    V3 t = cross(u, v) * 2.0;
    return v + t * q.w + cross(u, t);
}
inline V3 qrot_inv(Q q, V3 v) { return qrot({q.w, -q.x, -q.y, -q.z}, v); }   // world -> body
inline Q qnorm(Q q) { double l = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z); return {q.w / l, q.x / l, q.y / l, q.z / l}; }
inline Q qaxis(V3 a, double ang) { a = unit(a); double s = std::sin(ang / 2); return {std::cos(ang / 2), a.x * s, a.y * s, a.z * s}; }
// heading (yaw), nose-up pitch and roll in radians -> quaternion
inline Q qfrom_hpr(double yaw, double pitch_up, double roll) {
    return qaxis({0, 0, 1}, yaw) * qaxis({0, 1, 0}, -pitch_up) * qaxis({1, 0, 0}, roll);
}
inline void q_to_hpr(Q q, double& yaw, double& pitch_up, double& roll) {
    V3 f = qrot(q, {1, 0, 0}), l = qrot(q, {0, 1, 0}), u = qrot(q, {0, 0, 1});
    yaw = std::atan2(f.y, f.x); pitch_up = std::asin(std::max(-1.0, std::min(1.0, f.z)));
    roll = std::atan2(l.z, u.z);
}

// solve A x = b for n <= 6 (Gaussian elimination with pivoting). Returns false if singular.
inline bool solve_n(int n, double A[6][6], const double* b, double* x) {
    double M[6][7];
    for (int i = 0; i < n; ++i) { for (int j = 0; j < n; ++j) M[i][j] = A[i][j]; M[i][n] = b[i]; }
    for (int c = 0; c < n; ++c) {
        int p = c;
        for (int r = c + 1; r < n; ++r) if (std::fabs(M[r][c]) > std::fabs(M[p][c])) p = r;
        if (std::fabs(M[p][c]) < 1e-14) return false;
        if (p != c) for (int j = 0; j <= n; ++j) std::swap(M[p][j], M[c][j]);
        for (int r = c + 1; r < n; ++r) {
            double f = M[r][c] / M[c][c];
            for (int j = c; j <= n; ++j) M[r][j] -= f * M[c][j];
        }
    }
    for (int i = n - 1; i >= 0; --i) {
        double s = M[i][n];
        for (int j = i + 1; j < n; ++j) s -= M[i][j] * x[j];
        x[i] = s / M[i][i];
    }
    return true;
}