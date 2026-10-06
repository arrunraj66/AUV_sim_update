// physics/hull.hpp - the shape of the hull as a stack of slices (strips) along the body x axis.
// Buoyancy, drag and added mass all use the same slices, so they always agree with each other.
// A slice is an ellipse (half width hw, half height hh) or a rectangle (for box-shaped frames).
#pragma once
#include "../vmath.hpp"
#include <vector>
#include <functional>
#include <cmath>

namespace phys {

struct Strip { double x, hw, hh, dx; };

struct Hull {
    std::vector<Strip> st;
    bool rect = false;                         // rectangular sections instead of elliptic
    double xa = -0.5, xb = 0.5;
    double length = 1, volume = 0, wetted = 0, front = 0, max_hw = 0, max_hh = 0;
    V3 centroid;                               // centre of the hull volume

    double area(const Strip& s) const { return rect ? 4 * s.hw * s.hh : M_PI * s.hw * s.hh; }
    double perim(const Strip& s) const {
        if (rect) return 4 * (s.hw + s.hh);
        double a = s.hw, b = s.hh; return M_PI * (3 * (a + b) - std::sqrt((3 * a + b) * (a + 3 * b)));
    }
    // r(x) = radius, ys / zs scale it to an oval: hw = r ys, hh = r zs
    void from_profile(std::function<double(double)> r, double x0, double x1, double ys, double zs, int n = 40) {
        st.clear(); rect = false; xa = x0; xb = x1;
        double dx = (x1 - x0) / n;
        for (int i = 0; i < n; ++i) { double x = x0 + (i + 0.5) * dx, rr = r(x); st.push_back({x, rr * ys, rr * zs, dx}); }
        finish(r, ys, zs);
    }
    void from_box(double L, double W, double H, int n = 12) {
        st.clear(); rect = true; xa = -0.5 * L; xb = 0.5 * L; double dx = L / n;
        for (int i = 0; i < n; ++i) st.push_back({xa + (i + 0.5) * dx, 0.5 * W, 0.5 * H, dx});
        finish(nullptr, 1, 1);
    }
    void finish(std::function<double(double)> r, double ys, double zs) {
        length = xb - xa; volume = wetted = front = max_hw = max_hh = 0; double sx = 0;
        for (const Strip& s : st) {
            double A = area(s), slope = 0;
            if (r) slope = (r(s.x + 0.5 * s.dx) - r(s.x - 0.5 * s.dx)) / s.dx * std::sqrt(0.5 * (ys * ys + zs * zs));
            volume += A * s.dx; sx += A * s.dx * s.x; front = std::max(front, A);
            wetted += perim(s) * s.dx * std::sqrt(1 + slope * slope);
            max_hw = std::max(max_hw, s.hw); max_hh = std::max(max_hh, s.hh);
        }
        if (rect) wetted += 2 * front;           // flat ends of a box
        centroid = {volume > 0 ? sx / volume : 0.0, 0, 0};
    }
    double mean_diameter() const { return 2 * std::sqrt(front / M_PI); }
    void scale(double f) {
        for (Strip& s : st) { s.x *= f; s.hw *= f; s.hh *= f; s.dx *= f; }
        xa *= f; xb *= f; length *= f; volume *= f * f * f; wetted *= f * f; front *= f * f; max_hw *= f; max_hh *= f; centroid = centroid * f;
    }
};

}  // namespace phys