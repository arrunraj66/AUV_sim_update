// propwash.hpp - the bubble wake behind running propellers and thrusters.
//
// Every thruster that is working (and in the water) sheds air bubbles into its slipstream:
//   * how many: grows with thrust (about thrust^1.3), and strongly near the surface, where the propeller sucks in air (a ventilated, frothy wake); a thruster at rest sheds none
//   * where they start: on the propeller disc, just behind it
//   * how they move: with the slipstream (momentum theory: the induced velocity v_i = sqrt(u^2 + 2T / (rho A)) - u, doubled in the fully developed jet) blown aft, a swirl
//     around the thrust axis, then the water takes them: the jet dies out in a second or so, the current carries them, turbulence jitters them and buoyancy lifts them
//     (a few mm of air rise at 0.2 to 0.3 m/s). A vehicle moving fast therefore leaves a trail behind it that stays where the vehicle was.
//   * they pop when they reach the surface, and dissolve slowly with age.
// The simulation (WashSim) has no OpenGL in it and is tested on its own; PropWash adds the drawing (lit point sprites with a bright rim and a highlight, hidden by the vehicle
// and the sea bed through the depth test).
#pragma once
#include "fleet.hpp"
#include <vector>
#include <cmath>
#include <cstdint>

struct Bubble { float x = 0, y = 0, z = 0, vx = 0, vy = 0, vz = 0, age = 0, life = 0, size = 0; bool alive = false; };

struct WashSim {
    std::vector<Bubble> pool; size_t next = 0; uint32_t rng = 12345u; double carry[64][8] = {};     // emission remainders per (unit slot, thruster)
    int emitted = 0;                                                                              // total emitted since the start (for tests)
    explicit WashSim(size_t n = 40000) : pool(n) {}
    float rnd() { rng = rng * 1664525u + 1013904223u; return (rng >> 8) / 16777216.0f; }
    float rnd2() { return rnd() * 2.f - 1.f; }
    int alive() const { int n = 0; for (const Bubble& b : pool) n += b.alive; return n; }

    // bubbles per second of one thruster: `frac` = |thrust| / max thrust, `depth` of the propeller in metres
    static double rate(double frac, double depth, bool main_prop) {
        if (frac < 0.03) return 0.0;
        double surface = 1.0 + 9.0 * std::exp(-std::max(depth, 0.0) / 1.0);                       // ventilation near the surface
        return (main_prop ? 1500.0 : 600.0) * std::pow(frac, 1.3) * surface;
    }

    // one step: emit from every working thruster of every unit, then move the bubbles. `current_at(depth, vx, vy)` gives the water velocity.
    template <class CurrentFn>
    void update(double dt, const Fleet& fleet, CurrentFn current_at) {
        dt = std::min(dt, 0.1); if (dt <= 0) return;
        const double rho = 1025.0;
        for (size_t ui = 0; ui < fleet.units.size() && ui < 64; ++ui) {
            const Vehicle& v = fleet.units[ui].veh;
            for (size_t i = 0; i < v.s.thr.size() && i < 8 && i < v.thrust.size(); ++i) {
                const Thruster& T = v.s.thr[i]; double F = v.thrust[i], frac = std::fabs(F) / std::max(T.max_thrust, 1e-6);
                V3 wp = v.p + qrot(v.q, T.pos); double depth = -wp.z;
                bool wet = i < v.rot.size() ? v.rot[i].wet > 0.5 : depth > 0.0;
                if (!wet || depth < -0.2) { carry[ui][i] = 0; continue; }
                double n = rate(frac, depth, i == 0) * dt + carry[ui][i]; int cnt = (int)n; carry[ui][i] = n - cnt;
                V3 fdir = qrot(v.q, T.dir); V3 jd = fdir * (F >= 0 ? -1.0 : 1.0);                  // the water is thrown against the force on the vehicle
                double A = T.disc_ik > 0 ? 1.0 / (2.0 * rho * T.disc_ik) : 3.14159 * std::pow(0.40 * v.s.width, 2); double R = std::sqrt(A / 3.14159265);
                V3 vw = qrot(v.q, v.v);                                                             // vehicle velocity in the world
                double ua = std::max(0.0, -dot(vw, jd));                                            // speed of the flow into the propeller along the thrust axis
                double vi = std::sqrt(ua * ua + 2.0 * std::fabs(F) / (rho * A)) - ua, jet = 2.0 * vi;
                V3 e1 = unit(std::fabs(jd.z) < 0.9 ? cross(jd, V3{0, 0, 1}) : cross(jd, V3{1, 0, 0})), e2 = cross(jd, e1);
                for (int k = 0; k < cnt; ++k) {
                    double a = rnd() * 6.2831853, rr = R * 0.9 * std::sqrt(rnd()); V3 radial = e1 * std::cos(a) + e2 * std::sin(a);
                    V3 pos = wp + jd * (0.05 + 0.12 * rnd()) + radial * rr;
                    V3 vel = jd * (jet * (0.5 + 0.5 * rnd())) + cross(jd, radial) * (0.30 * jet * (rr / std::max(R, 1e-6))) + radial * (0.15 * jet) + V3{rnd2() * 0.15, rnd2() * 0.15, rnd2() * 0.15};
                    // the slipstream is measured against still water, not against the vehicle: at speed the wash is nearly at rest in the water, so the trail stays where the vehicle was
                    Bubble& b = pool[next]; next = (next + 1) % pool.size();
                    b.alive = true; b.x = (float)pos.x; b.y = (float)pos.y; b.z = (float)pos.z; b.vx = (float)vel.x; b.vy = (float)vel.y; b.vz = (float)vel.z;
                    b.age = 0; b.life = 2.5f + 4.5f * rnd(); b.size = 0.004f + 0.022f * rnd() * rnd() * rnd(); ++emitted;       // mostly tiny bubbles, a few larger ones
                }
            }
        }
        for (Bubble& b : pool) {
            if (!b.alive) continue;
            b.age += (float)dt; if (b.age > b.life) { b.alive = false; continue; }
            double vx, vy; current_at(std::max(0.0, -(double)b.z), vx, vy);
            double tau = 0.8, k = 1.0 - std::exp(-dt / tau);                                          // the jet dies out into the surrounding water
            b.vx += (float)((vx - b.vx) * k); b.vy += (float)((vy - b.vy) * k); b.vz += (float)((std::min(0.35, 0.08 + 0.27 * (b.size - 0.004) / 0.022) - b.vz) * k);   // terminal rise 0.08 to 0.35 m/s, larger bubbles rise faster
            double jit = 0.9 * std::sqrt(dt); b.vx += (float)(rnd2() * jit * 0.4); b.vy += (float)(rnd2() * jit * 0.4); b.vz += (float)(rnd2() * jit * 0.25);
            b.x += b.vx * (float)dt; b.y += b.vy * (float)dt; b.z += b.vz * (float)dt;
            b.size *= 1.0f + 0.03f * (float)dt;                                                       // they swell slightly as the pressure falls
            if (b.z >= 0.0f) b.alive = false;                                                         // pop at the surface
        }
    }
};

static const std::string WASH_VS = R"GLSL(
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec2 aSA;          // size (m), alpha
uniform mat4 uVP; uniform float uPx;
out float vA; out float vL;
void main() {
    float d = length(aPos - uCam);
    gl_Position = uVP * vec4(aPos, 1.0);
    vec3 L = irradiance(max(-aPos.z, 0.0));
    vL = clamp(dot(L, vec3(0.33)) * 2.0 + 0.12, 0.12, 1.3);
    vA = aSA.y * exp(-uK.g * d * 0.5) * (aPos.z < 0.0 ? 1.0 : 0.0);
    gl_PointSize = clamp(aSA.x * uPx * 0.87 / max(d, 0.25), 2.0, 36.0);
}
)GLSL";
static const std::string WASH_FS = R"GLSL(
in float vA; in float vL;
out vec4 fragColor;
void main() {
    vec2 q = (gl_PointCoord - 0.5) * 2.0; float r = length(q);
    if (r > 1.0) discard;
    float rim = smoothstep(0.55, 0.92, r) * (1.0 - smoothstep(0.92, 1.0, r));      // the thin bright edge of a bubble
    vec2 h = q - vec2(-0.35, -0.40); float hl = exp(-dot(h, h) * 26.0);            // the reflection of the light
    vec2 h2 = q - vec2(0.38, 0.42); float hl2 = exp(-dot(h2, h2) * 40.0) * 0.4;      // a fainter one on the other side
    float fill = 0.10 * (1.0 - r * r);
    float a = (rim * 0.80 + fill + hl * 0.95 + hl2) * vA;
    vec3 col = vec3(0.80, 0.93, 1.0) * vL;
    fragColor = vec4(col * a, a * 0.85);                                         // premultiplied alpha
}
)GLSL";

struct PropWash {
    WashSim sim; GLuint prog = 0, vao = 0, vbo = 0; bool on = true; std::vector<float> buf;
    void init() {
        prog = make_program(GLSL_COMMON + WASH_VS, GLSL_COMMON + WASH_FS, "propwash");
        glGenVertexArrays(1, &vao); glBindVertexArray(vao); glGenBuffers(1, &vbo); glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glBufferData(GL_ARRAY_BUFFER, sim.pool.size() * 5 * sizeof(float), nullptr, GL_DYNAMIC_DRAW);
        glEnableVertexAttribArray(0); glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(1); glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)(3 * sizeof(float)));
    }
    // draw after the vehicles (the caller has set the common and view-projection uniforms of `prog`)
    void draw(float screen_h) {
        if (!on) return;
        buf.clear();
        for (const Bubble& b : sim.pool) {
            if (!b.alive) continue;
            float fade_in = std::min(1.f, b.age / 0.15f), fade_out = std::min(1.f, (b.life - b.age) / (0.35f * b.life));
            buf.push_back(b.x); buf.push_back(b.y); buf.push_back(b.z); buf.push_back(b.size); buf.push_back(0.9f * fade_in * fade_out);
        }
        if (buf.empty()) return;
        glEnable(GL_PROGRAM_POINT_SIZE); glEnable(GL_BLEND); glBlendFunc(GL_ONE, GL_ONE_MINUS_SRC_ALPHA); glDepthMask(GL_FALSE);
        u1f(prog, "uPx", screen_h);
        glBindVertexArray(vao); glBindBuffer(GL_ARRAY_BUFFER, vbo); glBufferSubData(GL_ARRAY_BUFFER, 0, buf.size() * sizeof(float), buf.data());
        glDrawArrays(GL_POINTS, 0, (GLsizei)(buf.size() / 5));
        glDepthMask(GL_TRUE); glDisable(GL_BLEND);
    }
};
