// vmesh.hpp - the 3D shapes of the vehicles, built from code, plus an STL file loader/exporter.
// A vehicle is a list of parts. Most parts are fixed; fins turn with the rudder/elevator and the propeller spins.
#pragma once
#include "gfx.hpp"
#include "reef.hpp"
#include "vehicle.hpp"
#include <functional>
#include <fstream>
#include <unordered_map>
#include <cstdint>
#include <cstring>

static inline Vec3f P3(double x, double y, double z) { return {(float)x, (float)y, (float)z}; }
static inline Vec3f F3(V3 a) { return {(float)a.x, (float)a.y, (float)a.z}; }

struct VPart {
    Mesh mesh; int kind = 0;           // 0 fixed, 1 fin (turns), 2 propeller (spins)
    int index = -1;                    // which fin
    V3 hinge, axis = {1, 0, 0};
    GLuint vao = 0, vbo = 0, ebo = 0; int count = 0;
};
struct VModel {
    std::vector<VPart> parts; bool ready = false;
    Vec3f bmin = {1e9f, 1e9f, 1e9f}, bmax = {-1e9f, -1e9f, -1e9f};
    void upload() {
        for (auto& p : parts) {
            for (auto& v : p.mesh.v) for (int k = 0; k < 3; ++k) { bmin = {std::min(bmin.x, v.p[0]), std::min(bmin.y, v.p[1]), std::min(bmin.z, v.p[2])}; bmax = {std::max(bmax.x, v.p[0]), std::max(bmax.y, v.p[1]), std::max(bmax.z, v.p[2])}; }
            glGenVertexArrays(1, &p.vao); glBindVertexArray(p.vao);
            glGenBuffers(1, &p.vbo); glBindBuffer(GL_ARRAY_BUFFER, p.vbo);
            glBufferData(GL_ARRAY_BUFFER, p.mesh.v.size() * sizeof(Vertex), p.mesh.v.data(), GL_STATIC_DRAW);
            glGenBuffers(1, &p.ebo); glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, p.ebo);
            glBufferData(GL_ELEMENT_ARRAY_BUFFER, p.mesh.idx.size() * sizeof(unsigned), p.mesh.idx.data(), GL_STATIC_DRAW);
            glEnableVertexAttribArray(0); glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)0);
            glEnableVertexAttribArray(1); glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)12);
            glEnableVertexAttribArray(2); glVertexAttribPointer(2, 1, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)24);
            p.count = (int)p.mesh.idx.size();
        }
        ready = true;
    }
    void release() { for (auto& p : parts) { glDeleteBuffers(1, &p.vbo); glDeleteBuffers(1, &p.ebo); glDeleteVertexArrays(1, &p.vao); } parts.clear(); ready = false; }
};

// ---------------------------------------------------------------------------------------------
// building blocks
static void quad(Mesh& m, Vec3f a, Vec3f b, Vec3f c, Vec3f d, float tone, Vec3f want) {       // flat 4-corner face
    int i0 = m.add(a, tone), i1 = m.add(b, tone), i2 = m.add(c, tone), i3 = m.add(d, tone);
    m.tri(i0, i1, i2, want); m.tri(i0, i2, i3, want);
}
static void add_box(Mesh& m, Vec3f c, Vec3f h, float tone) {
    for (int ax = 0; ax < 3; ++ax) for (int sg = -1; sg <= 1; sg += 2) {
        int u = (ax + 1) % 3, v = (ax + 2) % 3; float hv[3] = {h.x, h.y, h.z}, cv[3] = {c.x, c.y, c.z};
        auto corner = [&](int su, int sv) { float p[3]; p[ax] = cv[ax] + sg * hv[ax]; p[u] = cv[u] + su * hv[u]; p[v] = cv[v] + sv * hv[v]; return Vec3f{p[0], p[1], p[2]}; };
        float w[3] = {0, 0, 0}; w[ax] = (float)sg;
        quad(m, corner(-1, -1), corner(1, -1), corner(1, 1), corner(-1, 1), tone, {w[0], w[1], w[2]});
    }
}
// a thin lifting plate (fin, wing, blade). xd = chord direction, sd = span direction (root to tip), nd = normal
static void add_plate(Mesh& m, Vec3f c, Vec3f xd, Vec3f sd, Vec3f nd, float cr, float ct, float span, float thick, float tone, float sweep = 0) {
    auto P = [&](float xo, float s, float side) { return vadd(vadd(vadd(c, vmul(xd, xo)), vmul(sd, s)), vmul(nd, side * thick * 0.5f)); };
    Vec3f rLE[2], rTE[2], tTE[2], tLE[2];
    for (int k = 0; k < 2; ++k) {
        float sd2 = k ? 1.f : -1.f;
        rLE[k] = P(cr / 2, -span / 2, sd2); rTE[k] = P(-cr / 2, -span / 2, sd2);
        tTE[k] = P(-ct / 2 + sweep, span / 2, sd2); tLE[k] = P(ct / 2 + sweep, span / 2, sd2);
    }
    quad(m, rLE[1], rTE[1], tTE[1], tLE[1], tone, nd); quad(m, rLE[0], rTE[0], tTE[0], tLE[0], tone, vmul(nd, -1.f));
    quad(m, rLE[0], rLE[1], tLE[1], tLE[0], tone, xd);                      // leading edge
    quad(m, rTE[0], rTE[1], tTE[1], tTE[0], tone, vmul(xd, -1.f));          // trailing edge
    quad(m, tLE[0], tLE[1], tTE[1], tTE[0], tone, sd);                      // tip
    quad(m, rLE[0], rLE[1], rTE[1], rTE[0], tone, vmul(sd, -1.f));          // root
}
// body of revolution around the x axis (hull). radius(x) gives the profile; ys/zs make the section oval.
// Normals are exact, so different coloured pieces join without a visible seam.
static void add_revolve(Mesh& m, float xa, float xb, int rings, int sides, float a0, float a1,
                        std::function<float(float)> radius, float ys, float zs, float tone) {
    int base = (int)m.v.size();
    float eps = (xb - xa) * 1e-3f;
    for (int i = 0; i <= rings; ++i) {
        float t = (float)i / rings, x = xa + (xb - xa) * 0.5f * (1.f - std::cos(3.14159265f * t));
        float r = radius(x);
        float xm = std::max(xa, x - eps), xp = std::min(xb, x + eps);
        float dr = (radius(xp) - radius(xm)) / std::max(xp - xm, 1e-6f);
        for (int j = 0; j <= sides; ++j) {
            float a = a0 + (a1 - a0) * j / sides, ca = std::cos(a), sa = std::sin(a);
            Vertex vt; vt.p[0] = x; vt.p[1] = r * ys * ca; vt.p[2] = r * zs * sa; vt.tone = tone;
            Vec3f tx = {1, dr * ys * ca, dr * zs * sa}, ta = {0, -r * ys * sa, r * zs * ca};
            Vec3f n = cross(tx, ta); float l = std::sqrt(dot(n, n));
            if (l < 1e-7f) n = {(x - xa) > (xb - x) ? 1.f : -1.f, 0, 0}; else n = {n.x / l, n.y / l, n.z / l};
            Vec3f out = {0, ca, sa}; if (dot(n, out) < 0 && l >= 1e-7f) n = {-n.x, -n.y, -n.z};
            vt.n[0] = n.x; vt.n[1] = n.y; vt.n[2] = n.z; m.v.push_back(vt);
        }
    }
    for (int i = 0; i < rings; ++i) for (int j = 0; j < sides; ++j) {
        int a = base + i * (sides + 1) + j, b = a + 1, c = a + (sides + 1), d = c + 1;
        Vec3f want = {0, std::cos(a0 + (a1 - a0) * (j + 0.5f) / sides), std::sin(a0 + (a1 - a0) * (j + 0.5f) / sides)};
        m.tri(a, b, d, want); m.tri(a, d, c, want);
    }
}
static float smoothstepf(float a, float b, float x) { float t = std::max(0.f, std::min(1.f, (x - a) / (b - a))); return t * t * (3 - 2 * t); }
static void full_hull(Mesh& m, float xa, float xb, int rings, std::function<float(float)> r, float ys, float zs, float tone) {
    add_revolve(m, xa, xb, rings, 40, 0.f, 6.2831853f, r, ys, zs, tone);
}
static void finish(VPart& p, bool smooth = true) { if (smooth) p.mesh.smooth_normals(); }
static VPart& add_part(VModel& vm, int kind = 0, int index = -1) { vm.parts.emplace_back(); vm.parts.back().kind = kind; vm.parts.back().index = index; return vm.parts.back(); }

// a fin as a turning part
static void fin_part(VModel& vm, const Spec& s, int fi, float cr, float ct, float span, float thick, float tone, float sweep = 0) {
    const Fin& f = s.fins[fi];
    bool moving = f.group != 0;
    VPart& p = add_part(vm, moving ? 1 : 0, fi);
    p.hinge = f.pos; p.axis = unit(cross(f.normal, V3{1, 0, 0}));
    Vec3f nd = F3(f.normal), xd = {1, 0, 0};
    V3 rad = f.pos - V3{f.pos.x, 0, 0};       // outward from the body axis
    Vec3f sd = F3(len(rad) > 1e-6 ? unit(rad) : V3{0, 1, 0});
    if (std::fabs(dot(sd, nd)) > 0.5f) sd = F3(unit(cross(f.normal, V3{1, 0, 0})));
    add_plate(p.mesh, F3(f.pos), xd, sd, nd, cr, ct, span, thick, tone, sweep);
    finish(p);
}

// ---------------------------------------------------------------------------------------------
static VModel build_torpedo(const Spec& s) {
    VModel vm; const float R = 0.095f;
    auto prof = [&](float x) {
        if (x > 0.5f) { float t = (x - 0.5f) / 0.3f; return R * std::sqrt(std::max(0.f, 1 - t * t)); }
        if (x < -0.30f) { float t = std::min(1.f, (-0.30f - x) / 0.36f); return R * (1.f - 0.5f * std::pow(t, 1.5f)); }
        return R;
    };
    { VPart& p = add_part(vm); full_hull(p.mesh, 0.74f, 0.80f, 14, prof, 1, 1, 1); }       // nose cap (black)
    { VPart& p = add_part(vm); full_hull(p.mesh, -0.32f, 0.74f, 90, prof, 1, 1, 0); }       // yellow body
    { VPart& p = add_part(vm); full_hull(p.mesh, -0.66f, -0.32f, 30, prof, 1, 1, 1); }      // black tail cone
    { VPart& p = add_part(vm); add_tube(p.mesh, P3(0.797, 0, 0), P3(0.808, 0, 0), 0.022f, 0.022f, 16, 6, 6, true, 6); finish(p); }   // lamp / camera window
    { VPart& p = add_part(vm);                                                              // hub, shroud, struts
      add_tube(p.mesh, P3(-0.84, 0, 0), P3(-0.62, 0, 0), 0.034f, 0.034f, 16, 2, 2, true, 2);
      add_tube(p.mesh, P3(-0.85, 0, 0), P3(-0.67, 0, 0), 0.108f, 0.108f, 40, 2, 2, false);
      add_tube(p.mesh, P3(-0.85, 0, 0), P3(-0.84, 0, 0), 0.112f, 0.112f, 40, 7, 7, false);
      for (int k = 0; k < 4; ++k) { float a = 0.7854f + 1.5708f * k; add_tube(p.mesh, P3(-0.70, 0.03f * std::cos(a), 0.03f * std::sin(a)), P3(-0.70, 0.108f * std::cos(a), 0.108f * std::sin(a)), 0.006f, 0.006f, 8, 2, 2); }
      finish(p); }
    { VPart& p = add_part(vm, 2); p.hinge = {-0.78, 0, 0}; p.axis = {1, 0, 0};              // propeller
      for (int k = 0; k < 3; ++k) {
          float a = 6.2831853f * k / 3; Vec3f sd = {0, std::cos(a), std::sin(a)}, td = {0, -std::sin(a), std::cos(a)};
          float c30 = 0.866f, s30 = 0.5f;
          Vec3f nd = vadd(vmul(td, c30), vmul(Vec3f{1, 0, 0}, s30)), xd = vadd(vmul(Vec3f{1, 0, 0}, c30), vmul(td, -s30));
          add_plate(p.mesh, vadd(Vec3f{-0.78f, 0, 0}, vmul(sd, 0.066f)), xd, sd, nd, 0.05f, 0.03f, 0.064f, 0.004f, 1);
      } finish(p); }
    for (int i = 0; i < 4; ++i) fin_part(vm, s, i, 0.22f, 0.14f, 0.16f, 0.012f, 1, -0.02f);
    { VPart& p = add_part(vm);                                                              // mast, side-scan strips
      add_tube(p.mesh, P3(-0.28, 0, 0.085), P3(-0.28, 0, 0.22), 0.006f, 0.005f, 8, 1, 1);
      add_tube(p.mesh, P3(-0.28, 0, 0.22), P3(-0.28, 0, 0.245), 0.012f, 0.012f, 12, 7, 7, true, 7);
      add_box(p.mesh, P3(0.1, 0.0935, 0), P3(0.20, 0.004, 0.012), 1); add_box(p.mesh, P3(0.1, -0.0935, 0), P3(0.20, 0.004, 0.012), 1);
      finish(p); }
    return vm;
}

static VModel build_hover(const Spec& s) {
    VModel vm;
    { VPart& p = add_part(vm);
      add_box(p.mesh, P3(0, 0, 0.105), P3(0.19, 0.14, 0.04), 3);                            // foam float
      for (int sg = -1; sg <= 1; sg += 2) {                                                  // side frames (open, so you can see inside)
          float y = sg * 0.165f;
          add_box(p.mesh, P3(0, y, 0.062), P3(0.22, 0.005, 0.014), 5); add_box(p.mesh, P3(0, y, -0.115), P3(0.22, 0.005, 0.014), 5);
          add_box(p.mesh, P3(0.205, y, -0.03), P3(0.016, 0.005, 0.09), 5); add_box(p.mesh, P3(-0.205, y, -0.03), P3(0.016, 0.005, 0.09), 5);
          add_box(p.mesh, P3(0, y, -0.03), P3(0.2, 0.004, 0.010), 5);
      }
      add_box(p.mesh, P3(0, 0, -0.118), P3(0.2, 0.15, 0.006), 2);                           // bottom plate
      add_box(p.mesh, P3(0.20, 0, -0.01), P3(0.006, 0.165, 0.11), 2); add_box(p.mesh, P3(-0.20, 0, -0.01), P3(0.006, 0.165, 0.11), 2);   // end frames
      add_tube(p.mesh, P3(-0.17, 0, -0.005), P3(0.13, 0, -0.005), 0.08f, 0.08f, 32, 2, 2, true, 2);       // electronics tube
      add_tube(p.mesh, P3(-0.12, 0, -0.088), P3(0.12, 0, -0.088), 0.032f, 0.032f, 20, 2, 2, true, 2);     // battery tube
      for (int sg = -1; sg <= 1; sg += 2) add_tube(p.mesh, P3(0.233, sg * 0.075, 0.025), P3(0.246, sg * 0.075, 0.025), 0.022f, 0.022f, 14, 6, 6, true, 6);   // lights
      finish(p); }
    { VPart& p = add_part(vm); full_hull(p.mesh, 0.13f, 0.21f, 14, [](float x) { float t = (x - 0.13f) / 0.08f; return 0.08f * std::sqrt(std::max(0.f, 1 - t * t)); }, 1, 1, 6); }   // glass dome
    { VPart& p = add_part(vm);
      for (const Thruster& t : s.thr) {
          V3 a = t.pos - t.dir * 0.05, b = t.pos + t.dir * 0.05;
          add_tube(p.mesh, F3(a), F3(b), 0.042f, 0.042f, 24, 1, 1);
          add_tube(p.mesh, F3(b), F3(b + t.dir * 0.01), 0.046f, 0.046f, 24, 5, 5);
          add_tube(p.mesh, F3(a - t.dir * 0.01), F3(a), 0.046f, 0.046f, 24, 5, 5);
          add_tube(p.mesh, F3(t.pos - t.dir * 0.03), F3(t.pos + t.dir * 0.03), 0.012f, 0.012f, 10, 2, 2, true, 2);
      }
      finish(p); }
    return vm;
}

static VModel build_glider(const Spec& s) {
    VModel vm; const float R = 0.1065f;
    auto prof = [&](float x) {
        if (x > 0.55f) { float t = (x - 0.55f) / 0.2f; return R * std::sqrt(std::max(0.f, 1 - t * t * 0.98f)); }
        if (x < -0.35f) { float t = std::min(1.f, (-0.35f - x) / 0.40f); return R * (1.f - 0.82f * std::pow(t, 1.4f)); }
        return R;
    };
    { VPart& p = add_part(vm); full_hull(p.mesh, 0.64f, 0.75f, 16, prof, 1, 1, 3); }        // white nose
    { VPart& p = add_part(vm); full_hull(p.mesh, -0.50f, 0.64f, 90, prof, 1, 1, 0); }       // yellow hull
    { VPart& p = add_part(vm); full_hull(p.mesh, -0.75f, -0.50f, 24, prof, 1, 1, 1); }      // black tail
    for (int i = 0; i < (int)s.fins.size(); ++i) {
        if (i < 2) fin_part(vm, s, i, 0.16f, 0.08f, 0.45f, 0.012f, 1, -0.04f);
        else fin_part(vm, s, i, 0.18f, 0.10f, 0.20f, 0.010f, 1, -0.03f);
    }
    { VPart& p = add_part(vm);
      add_tube(p.mesh, P3(-0.74, 0, 0.06), P3(-0.74, 0, 0.42), 0.006f, 0.004f, 8, 1, 1);
      add_tube(p.mesh, P3(-0.74, 0, 0.42), P3(-0.74, 0, 0.45), 0.01f, 0.01f, 10, 7, 7, true, 7);
      add_tube(p.mesh, P3(0.70, 0, -0.095), P3(0.70, 0, -0.112), 0.014f, 0.014f, 12, 2, 2, true, 2);   // altimeter
      finish(p); }
    return vm;
}

static VModel build_scout(const Spec& s) {
    VModel vm;
    auto prof = [](float x) { float u = std::fabs(x) / 0.6f; return 0.2f * std::pow(std::max(0.f, 1.f - std::pow(u, 3.0f)), 1.f / 3.f); };
    { VPart& p = add_part(vm); add_revolve(p.mesh, -0.6f, 0.6f, 90, 24, 0.12f, 3.0216f, prof, 1.f, 0.65f, 4); }              // orange top
    { VPart& p = add_part(vm); add_revolve(p.mesh, -0.6f, 0.6f, 90, 24, 3.0216f, 6.1632f, prof, 1.f, 0.65f, 1); }           // dark underside
    { VPart& p = add_part(vm);
      add_tube(p.mesh, P3(0.585, 0, 0.0), P3(0.60, 0, 0.0), 0.05f, 0.05f, 20, 6, 6, true, 6);                                // front window
      add_tube(p.mesh, P3(0.55, 0.12, -0.03), P3(0.58, 0.12, -0.03), 0.025f, 0.025f, 14, 6, 6, true, 6); add_tube(p.mesh, P3(0.55, -0.12, -0.03), P3(0.58, -0.12, -0.03), 0.025f, 0.025f, 14, 6, 6, true, 6);
      for (int sg = -1; sg <= 1; sg += 2) add_tube(p.mesh, P3(-0.4, sg * 0.14, -0.15), P3(0.4, sg * 0.14, -0.15), 0.014f, 0.014f, 10, 2, 2);        // skids
      for (int sg = -1; sg <= 1; sg += 2) for (int e = -1; e <= 1; e += 2) add_tube(p.mesh, P3(e * 0.4, sg * 0.14, -0.15), P3(e * 0.4, sg * 0.14, -0.09), 0.012f, 0.012f, 8, 2, 2);
      add_tube(p.mesh, P3(0.1, 0, 0.12), P3(0.1, 0, 0.2), 0.008f, 0.006f, 8, 1, 1);                                             // mast
      add_tube(p.mesh, P3(0.1, 0, 0.2), P3(0.1, 0, 0.22), 0.012f, 0.012f, 10, 7, 7, true, 7);
      finish(p); }
    { VPart& p = add_part(vm);                                                              // thruster ducts
      for (const Thruster& t : s.thr) {
          V3 a = t.pos - t.dir * (t.dir.x > 0.5 ? 0.07 : 0.15), b = t.pos + t.dir * (t.dir.x > 0.5 ? 0.07 : 0.15);
          if (t.dir.x > 0.5) { a = t.pos - t.dir * 0.08; b = t.pos + t.dir * 0.08; }
          add_tube(p.mesh, F3(a), F3(b), 0.052f, 0.052f, 24, 1, 1);
          add_tube(p.mesh, F3(b), F3(b + unit(b - a) * 0.01), 0.056f, 0.056f, 24, 5, 5);
          add_tube(p.mesh, F3(a - unit(b - a) * 0.01), F3(a), 0.056f, 0.056f, 24, 5, 5);
      }
      finish(p); }
    for (int i = 0; i < (int)s.fins.size(); ++i) fin_part(vm, s, i, i < 2 ? 0.2f : 0.2f, 0.12f, i < 2 ? 0.16f : 0.14f, 0.012f, 1, -0.02f);
    return vm;
}

static VModel build_model(int id, const Spec& s) {
    switch (id) { case M_TORPEDO: return build_torpedo(s); case M_HOVER: return build_hover(s); case M_GLIDER: return build_glider(s); default: return build_scout(s); }
}

// ---------------------------------------------------------------------------------------------
// STL: read binary or ASCII files, make normals (smooth where the surface is smooth, sharp at creases), write binary files.
struct StlTri { Vec3f a, b, c; };
static bool read_stl(const std::string& path, std::vector<StlTri>& tris) {
    std::ifstream f(path, std::ios::binary); if (!f) return false;
    std::vector<char> data((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
    if (data.size() < 84) return false;
    uint32_t n; std::memcpy(&n, &data[80], 4);
    bool binary = (size_t)84 + (size_t)n * 50 == data.size();
    tris.clear();
    if (binary) {
        for (uint32_t i = 0; i < n; ++i) {
            float v[12]; std::memcpy(v, &data[84 + (size_t)i * 50], 48);
            tris.push_back({{v[3], v[4], v[5]}, {v[6], v[7], v[8]}, {v[9], v[10], v[11]}});
        }
    } else {                                    // ASCII: read every "vertex x y z"
        std::string txt(data.begin(), data.end()); size_t pos = 0; std::vector<Vec3f> vs;
        while ((pos = txt.find("vertex", pos)) != std::string::npos) {
            float x, y, z; if (std::sscanf(txt.c_str() + pos + 6, "%f %f %f", &x, &y, &z) == 3) vs.push_back({x, y, z}); pos += 6;
        }
        for (size_t i = 0; i + 2 < vs.size(); i += 3) tris.push_back({vs[i], vs[i + 1], vs[i + 2]});
    }
    return !tris.empty();
}
static bool write_stl(const std::string& path, const Mesh& m) {
    std::ofstream f(path, std::ios::binary); if (!f) return false;
    char hdr[80] = {}; std::snprintf(hdr, 80, "auvsim test model"); f.write(hdr, 80);
    uint32_t n = (uint32_t)(m.idx.size() / 3); f.write((char*)&n, 4);
    for (uint32_t t = 0; t < n; ++t) {
        Vec3f a = m.pos(m.idx[3 * t]), b = m.pos(m.idx[3 * t + 1]), c = m.pos(m.idx[3 * t + 2]);
        Vec3f nn = cross(sub(b, a), sub(c, a)); float l = std::sqrt(dot(nn, nn)); if (l > 0) nn = {nn.x / l, nn.y / l, nn.z / l};
        float v[12] = {nn.x, nn.y, nn.z, a.x, a.y, a.z, b.x, b.y, b.z, c.x, c.y, c.z}; f.write((char*)v, 48);
        uint16_t attr = 0; f.write((char*)&attr, 2);
    }
    return true;
}
struct StlInfo { bool ok = false; Vec3f size = {1, 1, 1}; double volume = 0; std::string error; };
// swap_yz: file has y up instead of z.  flip_x: file points the other way.  length: scale so the longest side is this many metres (0 = keep units)
static VModel build_stl(const std::string& path, bool swap_yz, bool flip_x, float length, StlInfo& info) {
    VModel vm; std::vector<StlTri> tris;
    if (!read_stl(path, tris)) { info.error = "could not read " + path; return vm; }
    auto xf = [&](Vec3f p) { if (swap_yz) p = {p.x, -p.z, p.y}; if (flip_x) p = {-p.x, -p.y, p.z}; return p; };
    Vec3f lo = {1e30f, 1e30f, 1e30f}, hi = {-1e30f, -1e30f, -1e30f};
    for (auto& t : tris) for (Vec3f* q : {&t.a, &t.b, &t.c}) { *q = xf(*q); lo = {std::min(lo.x, q->x), std::min(lo.y, q->y), std::min(lo.z, q->z)}; hi = {std::max(hi.x, q->x), std::max(hi.y, q->y), std::max(hi.z, q->z)}; }
    Vec3f ext = sub(hi, lo), cen = {(lo.x + hi.x) / 2, (lo.y + hi.y) / 2, (lo.z + hi.z) / 2};
    float longest = std::max(ext.x, std::max(ext.y, ext.z)), sc = (length > 0 && longest > 0) ? length / longest : 1.0f;
    VPart& p = add_part(vm); Mesh& m = p.mesh;
    // normals: average the face normals around a corner only if the faces are within ~45 degrees of each other
    std::vector<Vec3f> fn(tris.size());
    std::unordered_map<uint64_t, std::vector<int>> grid;
    float qstep = longest * sc * 1e-5f + 1e-9f;
    auto key = [&](Vec3f q) { auto h = [qstep](float v) { return (uint64_t)(int64_t)std::llround(v / qstep) & 0x1FFFFF; }; return h(q.x) * 4398046511104ull ^ h(q.y) * 2097152ull ^ h(q.z); };
    double vol = 0;
    for (size_t i = 0; i < tris.size(); ++i) {
        for (Vec3f* q : {&tris[i].a, &tris[i].b, &tris[i].c}) *q = {(q->x - cen.x) * sc, (q->y - cen.y) * sc, (q->z - cen.z) * sc};
        Vec3f n = cross(sub(tris[i].b, tris[i].a), sub(tris[i].c, tris[i].a)); float l = std::sqrt(dot(n, n));
        fn[i] = l > 0 ? Vec3f{n.x / l, n.y / l, n.z / l} : Vec3f{0, 0, 1};
        vol += dot(tris[i].a, cross(tris[i].b, tris[i].c)) / 6.0;
        for (Vec3f q : {tris[i].a, tris[i].b, tris[i].c}) grid[key(q)].push_back((int)i);
    }
    for (size_t i = 0; i < tris.size(); ++i) {
        Vec3f cs[3] = {tris[i].a, tris[i].b, tris[i].c};
        for (int k = 0; k < 3; ++k) {
            Vec3f n = {0, 0, 0};
            for (int j : grid[key(cs[k])]) if (dot(fn[j], fn[i]) > 0.7f) n = vadd(n, fn[j]);
            float l = std::sqrt(dot(n, n)); n = l > 0 ? Vec3f{n.x / l, n.y / l, n.z / l} : fn[i];
            Vertex v; v.p[0] = cs[k].x; v.p[1] = cs[k].y; v.p[2] = cs[k].z; v.n[0] = n.x; v.n[1] = n.y; v.n[2] = n.z; v.tone = 8;
            m.v.push_back(v); m.idx.push_back((unsigned)m.v.size() - 1);
        }
    }
    info.ok = true; info.size = {ext.x * sc, ext.y * sc, ext.z * sc}; info.volume = std::fabs(vol);
    return vm;
}

// a small research submarine, exported once as models/test_auv.stl so there is a ready-made file to load
static Mesh make_test_sub() {
    Mesh m; Mesh hull;
    full_hull(hull, -0.5f, 0.5f, 60, [](float x) { float u = std::fabs(x) / 0.5f; return 0.11f * std::pow(std::max(0.f, 1.f - std::pow(u, 2.4f)), 1.f / 2.4f); }, 1, 1, 0);
    auto append = [&](const Mesh& a) { int b = (int)m.v.size(); for (auto& v : a.v) m.v.push_back(v); for (unsigned i : a.idx) m.idx.push_back(b + i); };
    append(hull);
    Mesh sail; add_revolve(sail, -0.10f, 0.12f, 20, 24, 0, 6.2831853f, [](float x) { float u = (x - 0.01f) / 0.11f; return 0.055f * std::sqrt(std::max(0.f, 1 - u * u)); }, 0.5f, 1.0f, 0);
    for (auto& v : sail.v) { v.p[2] = v.p[2] * 1.5f + 0.13f; }
    append(sail);
    Mesh t;
    for (int k = 0; k < 4; ++k) {
        float a = 1.5708f * k; Vec3f sd = {0, std::cos(a), std::sin(a)}, nd = {0, -std::sin(a), std::cos(a)};
        add_plate(t, vadd(Vec3f{-0.43f, 0, 0}, vmul(sd, 0.12f)), {1, 0, 0}, sd, nd, 0.14f, 0.08f, 0.14f, 0.012f, 0, -0.02f);
    }
    add_tube(t, P3(-0.52, 0, 0), P3(-0.40, 0, 0), 0.075f, 0.075f, 32, 0, 0, false);
    for (int k = 0; k < 4; ++k) { float a = 0.7854f + 1.5708f * k; add_tube(t, P3(-0.47, 0.05f * std::cos(a), 0.05f * std::sin(a)), P3(-0.47, 0.075f * std::cos(a), 0.075f * std::sin(a)), 0.006f, 0.006f, 8, 0, 0); }
    t.smooth_normals(); append(t);
    return m;
}