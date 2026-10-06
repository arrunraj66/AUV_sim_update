
// reef.hpp - real 3D rocks, corals and sea grass.
//
// 1. We build a few small 3D models (meshes) once: 3 rocks, 2 branching corals, a brain coral,
//    a table coral, a tube sponge and a bunch of sea grass.
// 2. We scatter thousands of copies over the sea floor (random size, turn and colour seed).
//    Corals only grow where there is enough light, so they get rarer as the sea gets deeper.
// 3. The GPU draws every copy of a model with ONE call ("instancing").
#pragma once
#include "gfx.hpp"
#include "noise.hpp"
#include "terrain.hpp"
#include <vector>
#include <map>
#include <array>
#include <algorithm>
#include <cstdint>

struct Rng {
    uint32_t s;
    explicit Rng(uint32_t seed = 1) : s(seed * 2654435761u + 12345u) {}
    float next() { s = s * 1664525u + 1013904223u; return (s >> 8) / 16777216.0f; }
    float range(float a, float b) { return a + (b - a) * next(); }
};

inline Vec3f vadd(Vec3f a, Vec3f b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
inline Vec3f vmul(Vec3f a, float s) { return {a.x * s, a.y * s, a.z * s}; }

struct Vertex { float p[3]; float n[3]; float tone; };

struct Mesh {
    std::vector<Vertex> v;
    std::vector<unsigned> idx;
    int add(Vec3f p, float tone) {
        Vertex x = {{p.x, p.y, p.z}, {0, 0, 0}, tone};
        v.push_back(x); return (int)v.size() - 1;
    }
    Vec3f pos(int i) const { return {v[i].p[0], v[i].p[1], v[i].p[2]}; }
    // add a triangle, turning it round if needed so that it faces the direction "want"
    void tri(int a, int b, int c, Vec3f want) {
        Vec3f n = cross(sub(pos(b), pos(a)), sub(pos(c), pos(a)));
        if (dot(n, want) < 0) std::swap(b, c);
        idx.push_back(a); idx.push_back(b); idx.push_back(c);
    }
    void smooth_normals() {
        for (auto& x : v) x.n[0] = x.n[1] = x.n[2] = 0;
        for (size_t t = 0; t + 2 < idx.size(); t += 3) {
            Vec3f n = cross(sub(pos(idx[t + 1]), pos(idx[t])), sub(pos(idx[t + 2]), pos(idx[t])));
            for (int k = 0; k < 3; ++k) { v[idx[t + k]].n[0] += n.x; v[idx[t + k]].n[1] += n.y; v[idx[t + k]].n[2] += n.z; }
        }
        for (auto& x : v) {
            float l = std::sqrt(x.n[0] * x.n[0] + x.n[1] * x.n[1] + x.n[2] * x.n[2]);
            if (l < 1e-8f) { x.n[0] = 0; x.n[1] = 0; x.n[2] = 1; } else { x.n[0] /= l; x.n[1] /= l; x.n[2] /= l; }
        }
    }
};

// a tapered tube from a to b (a branch, a stem, a sponge pipe)
static void add_tube(Mesh& m, Vec3f a, Vec3f b, float ra, float rb, int sides, float ta, float tb,
                     bool cap = false, float cap_tone = 2.0f) {
    Vec3f ax = norm(sub(b, a));
    Vec3f h = std::fabs(ax.z) < 0.9f ? Vec3f{0, 0, 1} : Vec3f{1, 0, 0};
    Vec3f u = norm(cross(ax, h)), w = cross(ax, u);
    int base = (int)m.v.size();
    for (int k = 0; k < 2; ++k) for (int i = 0; i < sides; ++i) {
        float ang = 6.2831853f * i / sides;
        Vec3f d = vadd(vmul(u, std::cos(ang)), vmul(w, std::sin(ang)));
        m.add(vadd(k ? b : a, vmul(d, k ? rb : ra)), k ? tb : ta);
    }
    for (int i = 0; i < sides; ++i) {
        int i2 = (i + 1) % sides, a0 = base + i, a1 = base + i2, b0 = base + sides + i, b1 = base + sides + i2;
        float ang = 6.2831853f * i / sides;
        Vec3f out = vadd(vmul(u, std::cos(ang)), vmul(w, std::sin(ang)));
        m.tri(a0, a1, b1, out); m.tri(a0, b1, b0, out);
    }
    if (cap) {
        int c = m.add(b, cap_tone), r0 = (int)m.v.size();
        for (int i = 0; i < sides; ++i) {
            float ang = 6.2831853f * i / sides;
            m.add(vadd(b, vmul(vadd(vmul(u, std::cos(ang)), vmul(w, std::sin(ang))), rb)), cap_tone);
        }
        for (int i = 0; i < sides; ++i) m.tri(c, r0 + i, r0 + (i + 1) % sides, ax);
    }
}

static Mesh icosphere(int sub) {
    Mesh m; float t = (1.0f + std::sqrt(5.0f)) / 2.0f;
    Vec3f vs[12] = {{-1, t, 0}, {1, t, 0}, {-1, -t, 0}, {1, -t, 0}, {0, -1, t}, {0, 1, t},
                    {0, -1, -t}, {0, 1, -t}, {t, 0, -1}, {t, 0, 1}, {-t, 0, -1}, {-t, 0, 1}};
    for (auto& v : vs) m.add(norm(v), 0);
    int f[20][3] = {{0, 11, 5}, {0, 5, 1}, {0, 1, 7}, {0, 7, 10}, {0, 10, 11}, {1, 5, 9}, {5, 11, 4},
                    {11, 10, 2}, {10, 7, 6}, {7, 1, 8}, {3, 9, 4}, {3, 4, 2}, {3, 2, 6}, {3, 6, 8},
                    {3, 8, 9}, {4, 9, 5}, {2, 4, 11}, {6, 2, 10}, {8, 6, 7}, {9, 8, 1}};
    std::vector<std::array<int, 3>> faces;
    for (auto& q : f) faces.push_back({q[0], q[1], q[2]});
    for (int s = 0; s < sub; ++s) {
        std::map<std::pair<int, int>, int> cache;
        auto mid = [&](int a, int b) {
            auto key = std::make_pair(std::min(a, b), std::max(a, b));
            auto it = cache.find(key); if (it != cache.end()) return it->second;
            Vec3f p = norm(vmul(vadd(m.pos(a), m.pos(b)), 0.5f));
            int i = m.add(p, 0); cache[key] = i; return i;
        };
        std::vector<std::array<int, 3>> nf;
        for (auto& q : faces) {
            int a = mid(q[0], q[1]), b = mid(q[1], q[2]), c = mid(q[2], q[0]);
            nf.push_back({q[0], a, c}); nf.push_back({q[1], b, a}); nf.push_back({q[2], c, b}); nf.push_back({a, b, c});
        }
        faces = nf;
    }
    for (auto& q : faces) {
        Vec3f cen = vmul(vadd(vadd(m.pos(q[0]), m.pos(q[1])), m.pos(q[2])), 1.0f / 3);
        m.tri(q[0], q[1], q[2], cen);
    }
    return m;
}

// an angular boulder: a sphere pushed around by noise and cut by a few flat planes
static Mesh make_rock(int seed) {
    Mesh m = icosphere(3); Rng r(seed * 17 + 3);
    int np = 5 + (int)(r.next() * 3); Vec3f pn[8]; float pc[8];
    for (int i = 0; i < np; ++i) {
        pn[i] = norm({r.range(-1, 1), r.range(-1, 1), r.range(-1, 1)}); pc[i] = r.range(0.70f, 0.90f);
    }
    float sx = r.range(0.8f, 1.25f), sy = r.range(0.8f, 1.25f), sz = r.range(0.55f, 0.85f);
    for (auto& vt : m.v) {
        Vec3f d = norm({vt.p[0], vt.p[1], vt.p[2]});
        float rr = 1.0f + 0.30f * (fbm3(d.x * 1.6f + seed, d.y * 1.6f, d.z * 1.6f, seed) - 0.5f) * 2
                        + 0.07f * (fbm3(d.x * 5.0f, d.y * 5.0f, d.z * 5.0f + seed, seed + 9) - 0.5f) * 2;
        for (int i = 0; i < np; ++i) { float dp = dot(d, pn[i]); if (dp > 0.05f) rr = std::min(rr, pc[i] / dp); }
        vt.p[0] = d.x * rr * sx; vt.p[1] = d.y * rr * sy; vt.p[2] = std::max(d.z * rr * sz, -0.30f); vt.tone = 0.5f;
    }
    std::vector<unsigned> old = m.idx; m.idx.clear();
    for (size_t t = 0; t + 2 < old.size(); t += 3) {
        Vec3f cen = vmul(vadd(vadd(m.pos(old[t]), m.pos(old[t + 1])), m.pos(old[t + 2])), 1.0f / 3);
        m.tri(old[t], old[t + 1], old[t + 2], cen);
    }
    m.smooth_normals(); return m;
}

static void branch(Mesh& m, Rng& r, Vec3f p, Vec3f dir, float len, float rad, int depth, float total_h) {
    Vec3f q = vadd(p, vmul(dir, len));
    add_tube(m, p, q, rad, rad * 0.72f, 5, std::min(1.f, std::max(0.f, p.z / total_h)), std::min(1.f, std::max(0.f, q.z / total_h)));
    if (depth <= 0) return;
    int nc = 2 + (depth >= 2 && r.next() < 0.5f ? 1 : 0);
    for (int i = 0; i < nc; ++i) {
        Vec3f d = vadd(dir, vmul(norm({r.range(-1, 1), r.range(-1, 1), r.range(-1, 1)}), 0.85f));
        d.z += 0.25f; branch(m, r, q, norm(d), len * 0.74f, rad * 0.70f, depth - 1, total_h);
    }
}
static Mesh make_branching(int seed) {      // staghorn-like branching coral
    Mesh m; Rng r(seed * 31 + 7);
    for (int i = 0; i < 3; ++i) {
        float a = r.range(0, 6.28f);
        branch(m, r, {0, 0, 0}, norm({std::cos(a) * 0.45f, std::sin(a) * 0.45f, 1.0f}), 0.32f, 0.075f, 3, 1.4f);
    }
    m.smooth_normals(); return m;
}

static Mesh make_brain(int seed) {          // dome with winding grooves
    Mesh m; int R = 14, S = 22;
    for (int i = 0; i <= R; ++i) for (int j = 0; j < S; ++j) {
        float th = 1.75f * i / R, ph = 6.2831853f * j / S;
        Vec3f d = {std::sin(th) * std::cos(ph), std::sin(th) * std::sin(ph), std::cos(th)};
        float g = fbm3(d.x * 5.0f, d.y * 5.0f, d.z * 5.0f, seed);
        float groove = 1.0f - smooth01(0.0f, 0.10f, std::fabs(g - 0.5f));
        float rr = 1.0f - 0.055f * groove;
        m.add({d.x * rr, d.y * rr, d.z * rr * 0.62f}, groove);
    }
    for (int i = 0; i < R; ++i) for (int j = 0; j < S; ++j) {
        int a = i * S + j, b = i * S + (j + 1) % S, c = (i + 1) * S + j, d = (i + 1) * S + (j + 1) % S;
        Vec3f cen = vmul(vadd(vadd(m.pos(a), m.pos(b)), vadd(m.pos(c), m.pos(d))), 0.25f);
        m.tri(a, c, d, cen); m.tri(a, d, b, cen);
    }
    m.smooth_normals(); return m;
}

static Mesh make_table(int seed) {          // flat plate on a stem
    Mesh m; Rng r(seed);
    add_tube(m, {0, 0, 0}, {0, 0, 0.38f}, 0.13f, 0.10f, 8, 0.0f, 0.2f);
    int R = 5, S = 26, top = (int)m.v.size();
    auto wav = [&](float rho, float ph) { return 0.05f * std::sin(5 * ph + seed) * rho * rho + 0.03f * rho * (fbm3(std::cos(ph) * 2, std::sin(ph) * 2, seed, seed) - 0.5f); };
    for (int k = 0; k <= R; ++k) for (int j = 0; j < S; ++j) {
        float rho = (float)k / R, ph = 6.2831853f * j / S;
        m.add({std::cos(ph) * rho, std::sin(ph) * rho, 0.38f + 0.05f * rho + wav(rho, ph)}, rho);
    }
    int bot = (int)m.v.size();
    for (int k = 0; k <= R; ++k) for (int j = 0; j < S; ++j) {
        float rho = (float)k / R, ph = 6.2831853f * j / S;
        m.add({std::cos(ph) * rho, std::sin(ph) * rho, 0.38f + 0.05f * rho + wav(rho, ph) - 0.07f}, rho * 0.5f);
    }
    for (int k = 0; k < R; ++k) for (int j = 0; j < S; ++j) {
        int j2 = (j + 1) % S;
        int a = top + k * S + j, b = top + k * S + j2, c = top + (k + 1) * S + j, d = top + (k + 1) * S + j2;
        m.tri(a, c, d, {0, 0, 1}); m.tri(a, d, b, {0, 0, 1});
        a += bot - top; b += bot - top; c += bot - top; d += bot - top;
        m.tri(a, c, d, {0, 0, -1}); m.tri(a, d, b, {0, 0, -1});
    }
    for (int j = 0; j < S; ++j) {                   // thin rim between top and underside
        int j2 = (j + 1) % S; float ph = 6.2831853f * j / S;
        int a = top + R * S + j, b = top + R * S + j2, c = bot + R * S + j, d = bot + R * S + j2;
        Vec3f out = {std::cos(ph), std::sin(ph), 0};
        m.tri(a, b, d, out); m.tri(a, d, c, out);
    }
    m.smooth_normals(); return m;
}

static Mesh make_tube_sponge(int seed) {    // a cluster of open pipes
    Mesh m; Rng r(seed * 13 + 1);
    for (int i = 0; i < 6; ++i) {
        float a = r.range(0, 6.28f), d = r.range(0.0f, 0.34f), h = r.range(0.4f, 0.95f), rad = r.range(0.08f, 0.13f);
        Vec3f b = {std::cos(a) * d, std::sin(a) * d, -0.05f};
        add_tube(m, b, {b.x + r.range(-0.05f, 0.05f), b.y + r.range(-0.05f, 0.05f), h}, rad, rad * 0.85f, 10, 0.0f, 1.0f, true, 2.0f);
    }
    m.smooth_normals(); return m;
}

static Mesh make_grass(int seed) {          // a tuft of blades
    Mesh m; Rng r(seed * 7 + 5); int segs = 6;
    for (int b = 0; b < 9; ++b) {
        float ba = r.range(0, 6.28f), br = r.range(0, 0.25f), h = r.range(0.7f, 1.4f), w = r.range(0.035f, 0.06f);
        float ang = r.range(0, 6.28f), lean_l = r.range(0.1f, 0.4f);
        Vec3f base = {std::cos(ba) * br, std::sin(ba) * br, 0}, lean = {std::cos(ang) * lean_l, std::sin(ang) * lean_l, 0};
        Vec3f side = {-std::sin(ang), std::cos(ang), 0};
        int first = (int)m.v.size();
        for (int s = 0; s <= segs; ++s) {
            float t = (float)s / segs, ww = w * (1.0f - 0.8f * t);
            Vec3f c = vadd(vadd(base, vmul(lean, t * t)), {0, 0, h * t});
            m.add(vadd(c, vmul(side, -ww)), t); m.add(vadd(c, vmul(side, ww)), t);
        }
        Vec3f want = {-std::cos(ang), -std::sin(ang), 0.2f};
        for (int s = 0; s < segs; ++s) {
            int a = first + 2 * s, c = first + 2 * s + 1, d = first + 2 * s + 2, e = first + 2 * s + 3;
            m.tri(a, c, e, want); m.tri(a, e, d, want);
        }
    }
    m.smooth_normals(); return m;
}

// ---- one copy of a model on the sea floor -------------------------------------------------
struct Inst { float x, y, s, yaw, seed, zstretch, ground_z, pad; };

struct ReefParams {
    int   seed = 1;
    float half = 140.0f;     // reef area: -half..+half metres
    float rock = 1.0f, coral = 1.0f, grass = 1.0f;     // how much of each (0 = none)
};

class Reef {
public:
    static const int KINDS = 9;     // 0-2 rocks, 3-4 branching coral, 5 brain, 6 table, 7 tube sponge, 8 grass
    static constexpr int shade_kind[KINDS] = {0, 0, 0, 1, 1, 2, 3, 4, 5};
    Mesh mesh[KINDS];
    std::vector<Inst> inst[KINDS];
    GLuint vao[KINDS] = {}, vbo[KINDS] = {}, ebo[KINDS] = {}, ivbo[KINDS] = {};

    void init() {
        mesh[0] = make_rock(1); mesh[1] = make_rock(2); mesh[2] = make_rock(3);
        mesh[3] = make_branching(1); mesh[4] = make_branching(2); mesh[5] = make_brain(1);
        mesh[6] = make_table(1); mesh[7] = make_tube_sponge(1); mesh[8] = make_grass(1);
        for (int k = 0; k < KINDS; ++k) {
            glGenVertexArrays(1, &vao[k]); glBindVertexArray(vao[k]);
            glGenBuffers(1, &vbo[k]); glBindBuffer(GL_ARRAY_BUFFER, vbo[k]);
            glBufferData(GL_ARRAY_BUFFER, mesh[k].v.size() * sizeof(Vertex), mesh[k].v.data(), GL_STATIC_DRAW);
            glEnableVertexAttribArray(0); glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)0);
            glEnableVertexAttribArray(1); glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)12);
            glEnableVertexAttribArray(2); glVertexAttribPointer(2, 1, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)24);
            glGenBuffers(1, &ebo[k]); glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo[k]);
            glBufferData(GL_ELEMENT_ARRAY_BUFFER, mesh[k].idx.size() * sizeof(unsigned), mesh[k].idx.data(), GL_STATIC_DRAW);
            glGenBuffers(1, &ivbo[k]); glBindBuffer(GL_ARRAY_BUFFER, ivbo[k]);
            glEnableVertexAttribArray(3); glVertexAttribPointer(3, 4, GL_FLOAT, GL_FALSE, sizeof(Inst), (void*)0);
            glVertexAttribDivisor(3, 1);
            glEnableVertexAttribArray(4); glVertexAttribPointer(4, 4, GL_FLOAT, GL_FALSE, sizeof(Inst), (void*)16);
            glVertexAttribDivisor(4, 1);
        }
    }

    // scatter rocks, corals and grass over the sea floor
    void generate(const Terrain& T, const ReefParams& P) {
        for (auto& v : inst) v.clear();
        Rng r(P.seed * 977 + 5); float H = P.half;
        auto add = [&](int kind, float x, float y, float s, float zs) {
            inst[kind].push_back({x, y, s, r.range(0, 6.2831853f), r.next(), zs, T.ground_z(x, y), 0});
        };
        // rocks: boulder fields where the terrain says "rock", a few stones everywhere
        for (float gx = -H; gx <= H; gx += 2.0f) for (float gy = -H; gy <= H; gy += 2.0f) {
            float x = gx + r.range(0, 2), y = gy + r.range(0, 2), rm = T.rock_at(x, y);
            if (r.next() > P.rock * (0.03f + 0.55f * rm * rm + 0.2f * rm)) continue;
            float u = r.next(), s = 0.22f + std::pow(u, 3.0f) * (0.9f + 3.0f * rm);
            add((int)(r.next() * 3) % 3, x, y, s, r.range(0.55f, 1.0f));
        }
        // coral reef patches (need light, so they thin out with depth)
        for (float gx = -H; gx <= H; gx += 1.1f) for (float gy = -H; gy <= H; gy += 1.1f) {
            float x = gx + r.range(0, 1.1f), y = gy + r.range(0, 1.1f);
            float reefv = smooth01(0.46f, 0.62f, fbm2(x / 45 + 200, y / 45 + 200));
            float depth = T.floor_depth - T.height(x, y);
            float lf = std::min(1.0f, std::exp(-depth / 32.0f) * 1.8f);
            float patch = 0.25f + 0.75f * fbm2(x / 7 + 300, y / 7 + 300);
            if (r.next() > P.coral * (reefv * lf * patch * 0.55f + 0.004f * lf)) continue;
            float t = fbm2(x / 12 + 500, y / 12 + 500) + (r.next() - 0.5f) * 0.25f;
            if (t < 0.47f)      add(3 + (r.next() < 0.5f ? 0 : 1), x, y, r.range(0.35f, 1.1f), r.range(0.8f, 1.3f));
            else if (t < 0.53f) add(5, x, y, r.range(0.3f, 0.9f), 1.0f);
            else if (t < 0.58f) add(6, x, y, r.range(0.4f, 1.0f), 1.0f);
            else                add(7, x, y, r.range(0.3f, 0.7f), r.range(0.8f, 1.3f));
        }
        // sea grass in sandy shallow areas
        for (float gx = -H; gx <= H; gx += 1.0f) for (float gy = -H; gy <= H; gy += 1.0f) {
            float x = gx + r.range(0, 1.0f), y = gy + r.range(0, 1.0f);
            float gm = smooth01(0.50f, 0.68f, fbm2(x / 30 + 50, y / 30 + 50)) * (1.0f - T.rock_at(x, y));
            float depth = T.floor_depth - T.height(x, y);
            if (r.next() > P.grass * gm * std::min(1.0f, std::exp(-depth / 45.0f) * 2.0f) * 0.5f) continue;
            add(8, x, y, r.range(0.6f, 1.5f), 1.0f);
        }
        for (int k = 0; k < KINDS; ++k) {
            glBindBuffer(GL_ARRAY_BUFFER, ivbo[k]);
            glBufferData(GL_ARRAY_BUFFER, inst[k].size() * sizeof(Inst), inst[k].data(), GL_DYNAMIC_DRAW);
        }
    }

    int total() const { int n = 0; for (auto& v : inst) n += (int)v.size(); return n; }
    int coral_count() const { int n = 0; for (int k = 3; k < 8; ++k) n += (int)inst[k].size(); return n; }

    // draw all models; "set_kind" tells the shader which material to use
    template <class F> void draw(F set_kind) const {
        for (int k = 0; k < KINDS; ++k) {
            if (inst[k].empty()) continue;
            set_kind(shade_kind[k]);
            glBindVertexArray(vao[k]);
            glDrawElementsInstanced(GL_TRIANGLES, (GLsizei)mesh[k].idx.size(), GL_UNSIGNED_INT, nullptr, (GLsizei)inst[k].size());
        }
    }
};