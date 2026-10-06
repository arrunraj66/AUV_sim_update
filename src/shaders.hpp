// shaders.hpp - the GPU programs (GLSL): sky, sea floor, rocks/corals, particles, sea surface.
#pragma once
#include <string>

// ---- shared by all programs: light, colours, how light fades in water -----------------------
static const std::string GLSL_COMMON = R"GLSL(
#version 330 core
uniform vec3  uCam;       // camera position (z up, sea surface is z = 0)
uniform vec3  uSunDir;    // unit vector pointing TOWARD the sun
uniform vec3  uSunCol;    // sunlight colour and strength (already includes day/night)
uniform float uDay;       // 0 = night, 1 = full day
uniform vec3  uK;         // light attenuation in water, per metre (R,G,B)
uniform vec3  uBb;        // backscatter (how much light the water itself scatters back)
uniform float uTime;

const float N_WATER = 1.333;

vec3 gammaOut(vec3 c) { return pow(clamp(c, 0.0, 1.0), vec3(1.0 / 2.2)); }
vec3 ambientSky() { return vec3(0.30, 0.46, 0.78) * (0.04 + 0.96 * uDay); }

// colour of the sky in direction d
vec3 horizonSea() {      // colour of the distant sea at the horizon
    return mix(vec3(0.52, 0.68, 0.88), vec3(0.16, 0.34, 0.52), 0.45) * (0.03 + 0.97 * uDay);
}
vec3 skyColour(vec3 d) {
    float t = clamp(d.z, 0.0, 1.0);
    vec3 hor = vec3(0.52, 0.68, 0.88) * (0.03 + 0.97 * uDay);
    vec3 zen = vec3(0.14, 0.32, 0.78) * (0.03 + 0.97 * uDay);
    vec3 c = mix(hor, zen, pow(t, 0.45));
    float s = max(dot(d, uSunDir), 0.0);
    c += uSunCol * (pow(s, 2000.0) * 80.0 + pow(s, 120.0) * 0.25 + pow(s, 8.0) * 0.06);
    return c;
}

// sunlight + skylight that reaches a depth z (metres below the surface)
vec3 irradiance(float z) {
    float cz = max(uSunDir.z, 0.0);
    float sinT = sqrt(max(1.0 - cz * cz, 0.0)) / N_WATER;       // light bends toward the vertical
    float cosT = max(sqrt(1.0 - sinT * sinT), 0.35);
    vec3 direct = uSunCol * cz * exp(-uK * z / cosT);
    vec3 diffuse = ambientSky() * 0.6 * exp(-uK * z * 1.2);
    return direct + diffuse;
}

// same, for a ray that starts at the sea surface and goes DOWN to the floor
// (what you see looking into the water from above). mu = how steeply it dives (0 flat .. 1 straight down)
vec3 veilAbove(float L, float mu) {
    float cz = max(uSunDir.z, 0.0);
    float sinT = sqrt(max(1.0 - cz * cz, 0.0)) / N_WATER;
    float cosT = max(sqrt(1.0 - sinT * sinT), 0.35);
    vec3 a1 = uK * (1.0 + mu / cosT), a2 = uK * (1.0 + 1.2 * mu);
    vec3 d0 = uSunCol * cz, m0 = ambientSky() * 0.6;
    return uBb * (d0 * (1.0 - exp(-a1 * L)) / a1 + m0 * (1.0 - exp(-a2 * L)) / a2);
}

// light the water itself adds along a ray of length L (the blue-green "haze" of the sea)
vec3 veil(float zmean, float L) {
    return irradiance(max(zmean, 0.0)) * uBb / uK * (1.0 - exp(-uK * L));
}
)GLSL";

// ---- sky (full screen triangle) --------------------------------------------------------------
static const std::string SKY_VS = R"GLSL(
#version 330 core
out vec2 vNdc;
void main() {
    vec2 p = vec2(float((gl_VertexID << 1) & 2), float(gl_VertexID & 2));
    vNdc = p * 2.0 - 1.0;
    gl_Position = vec4(vNdc, 0.9999, 1.0);
}
)GLSL";
static const std::string SKY_FS = R"GLSL(
in vec2 vNdc;
uniform vec3 uRight, uUp, uFwd; uniform vec2 uTan;
out vec4 fragColor;
void main() {
    vec3 d = normalize(uFwd + uRight * vNdc.x * uTan.x + uUp * vNdc.y * uTan.y);
    vec3 c;
    if (uCam.z >= 0.0) {
        if (d.z >= 0.0) c = mix(horizonSea(), skyColour(d), smoothstep(0.0, 0.015, d.z));
        else c = mix(horizonSea(), veilAbove(10000.0, -d.z), smoothstep(0.02, 0.06, -d.z));   // deep water below the horizon
    }
    else c = veil(-uCam.z, 10000.0) * (0.9 + 0.6 * max(d.z, 0.0));       // open water all around
    fragColor = vec4(gammaOut(c), 1.0);
}
)GLSL";

// ---- colour noise (only used for colours and fine detail, never for shapes) --------------------
static const std::string NOISE = R"GLSL(
float hash(vec2 p) { p = fract(p * vec2(123.34, 456.21)); p += dot(p, p + 45.32); return fract(p.x * p.y); }
float vnoise(vec2 p) {
    vec2 i = floor(p), f = fract(p); f = f * f * (3.0 - 2.0 * f);
    return mix(mix(hash(i), hash(i + vec2(1, 0)), f.x), mix(hash(i + vec2(0, 1)), hash(i + vec2(1, 1)), f.x), f.y);
}
float fbm(vec2 p) {
    float a = 0.5, s = 0.0;
    for (int i = 0; i < 5; ++i) { s += a * vnoise(p); p = p * 2.03 + vec2(17.1, 9.2); a *= 0.5; }
    return s;
}
)GLSL";

// ---- the height map of the sea floor (made on the CPU, shared with the GPU) --------------------
static const std::string TERRAIN = R"GLSL(
uniform sampler2D uTerrain;
uniform float uHalf;       // the map covers -uHalf..+uHalf metres
uniform float uFloor;      // mean depth of the sea bed (m)
uniform float uRelief;     // height of hills and rocks (m)
vec2 terrUV(vec2 p) { return (p / uHalf * 0.5 + 0.5) * (1023.0 / 1024.0) + 0.5 / 1024.0; }
float groundH(vec2 p) { return min(textureLod(uTerrain, terrUV(p), 0.0).r * uRelief, uFloor * 0.8); }
float rockAt(vec2 p) { return textureLod(uTerrain, terrUV(p), 0.0).g; }
)GLSL";

// ---- light under water: sun, sky, the vehicle lamp, caustics, and the water in between ---------
static const std::string LIGHTING = R"GLSL(
uniform float uLamp;       // vehicle lamp brightness (0 = off)
uniform vec3  uLampPos, uLampDir;

float caustic(vec2 p, float t) {            // moving bright network of lines made by the waves
    vec2 i = p; float c = 1.0; const float inten = 0.005;
    for (int n = 0; n < 4; ++n) {
        float tt = t * (1.0 - 3.5 / float(n + 1));
        i = p + vec2(cos(tt - i.x) + sin(tt + i.y), sin(tt - i.y) + cos(tt + i.x));
        c += 1.0 / max(length(vec2(p.x / (sin(i.x + tt) / inten), p.y / (cos(i.y + tt) / inten))), 0.001);
    }
    c /= 4.0; c = 1.17 - pow(c, 1.4);
    float r = pow(abs(c), 8.0);
    return isnan(r) ? 0.0 : min(r, 1.0);
}

// colour of a surface at point p with normal n, as seen from the camera
vec3 shadeUnderwater(vec3 alb, vec3 n, vec3 p, float causticAmt) {
    float dF = max(-p.z, 0.0);
    vec3 Lw = -refract(-uSunDir, vec3(0, 0, 1), 1.0 / N_WATER);          // sun direction under water
    float ndl = max(dot(n, Lw), 0.0) * step(0.0, uSunDir.z);
    float cosT = max(Lw.z, 0.35);
    vec3 direct = uSunCol * ndl * exp(-uK * dF / cosT);
    direct *= mix(1.0, 0.25 + 2.6 * caustic(p.xy * 0.35, uTime * 0.6), causticAmt * exp(-dF * 0.07));
    vec3 amb = ambientSky() * 0.6 * exp(-uK * dF * 1.2) * (0.5 + 0.5 * n.z);
    vec3 lamp = vec3(0.0);
    if (uLamp > 0.0) {
        vec3 Ld = uLampPos - p; float d = length(Ld); Ld /= d;
        float cone = smoothstep(0.70, 0.93, dot(-Ld, uLampDir));
        lamp = vec3(1.0, 0.96, 0.88) * uLamp * cone * max(dot(n, Ld), 0.0) / (1.0 + 0.04 * d * d) * exp(-uK * d);
    }
    vec3 col = alb * (direct + amb + lamp);

    // the water between the surface point and the camera absorbs light and adds blue haze
    vec3 toC = uCam - p;
    float Ltot = length(toC), Lw2, zmean;
    if (uCam.z < 0.0) {
        Lw2 = Ltot;
        float dep = -uCam.z;
        zmean = max(0.0, dep + 0.5 * (dF - dep) * min(1.0, (2.5 / uK.g) / Ltot));
        col = col * exp(-uK * Lw2) + veil(zmean, Lw2);
    } else {
        Lw2 = Ltot * dF / (uCam.z + dF);
        col = col * exp(-uK * Lw2) + veilAbove(Lw2, (uCam.z + dF) / Ltot);
    }
    return col;
}
)GLSL";

// ---- sea floor -------------------------------------------------------------------------------
static const std::string FLOOR_VS = R"GLSL(
layout(location = 0) in vec2 aPos;
uniform mat4 uVP; uniform vec2 uCenter; uniform float uSize;
out vec3 vPos;
void main() {
    vec2 xy = uCenter + aPos * uSize;
    vPos = vec3(xy, -uFloor + groundH(xy));
    gl_Position = uVP * vec4(vPos, 1.0);
}
)GLSL";

static const std::string FLOOR_FS = R"GLSL(
in vec3 vPos;
out vec4 fragColor;

float rippleH(vec2 p) {     // sand ripples: a few centimetres high
    return (1.0 - rockAt(p)) * 0.035 * sin(dot(p, vec2(0.8, 0.6)) * 8.5 + 4.0 * fbm(p / 9.0));
}

void main() {
    vec2 p = vPos.xy;
    float e = 0.6, e2 = 0.08;
    vec2 g = vec2(groundH(p + vec2(e, 0)) - groundH(p - vec2(e, 0)), groundH(p + vec2(0, e)) - groundH(p - vec2(0, e))) / (2.0 * e);
    g += vec2(rippleH(p + vec2(e2, 0)) - rippleH(p - vec2(e2, 0)), rippleH(p + vec2(0, e2)) - rippleH(p - vec2(0, e2))) / (2.0 * e2);
    vec3 n = normalize(vec3(-g, 1.0));

    float rock = rockAt(p), grav = smoothstep(0.45, 0.62, fbm(p / 12.0 + 10.0));
    vec3 sand = mix(vec3(0.78, 0.70, 0.52), vec3(0.62, 0.55, 0.40), fbm(p / 3.0));
    vec3 gravel = mix(vec3(0.50, 0.46, 0.40), vec3(0.35, 0.33, 0.30), vnoise(p * 4.0));
    vec3 rockc = mix(vec3(0.22, 0.21, 0.21), vec3(0.42, 0.38, 0.33), vnoise(p * 1.7));
    vec3 algae = vec3(0.16, 0.30, 0.10);
    vec3 alb = mix(sand, gravel, grav * 0.8);
    alb = mix(alb, rockc, rock);
    alb = mix(alb, algae, rock * smoothstep(0.55, 0.8, fbm(p / 15.0 + 80.0)) * 0.7);
    alb *= 0.9 + 0.2 * vnoise(p * 15.0);

    fragColor = vec4(gammaOut(shadeUnderwater(alb, n, vPos, 1.0)), 1.0);
}
)GLSL";

// ---- rocks, corals, grass (instanced 3D models) ----------------------------------------------
static const std::string OBJ_VS = R"GLSL(
layout(location = 0) in vec3 aP;
layout(location = 1) in vec3 aN;
layout(location = 2) in float aTone;
layout(location = 3) in vec4 iA;      // x, y, scale, turn
layout(location = 4) in vec4 iB;      // colour seed, height stretch, ground height
uniform mat4 uVP;
uniform int uSway;                    // 1 = sea grass waves in the current
out vec3 vPos; out vec3 vN; out vec3 vLocal; out float vTone; out float vSeed;
void main() {
    float s = iA.z, yaw = iA.w;
    vec3 lp = aP; lp.z *= iB.y;
    if (uSway == 1) {
        float h = aTone, ph = iB.x * 40.0;
        lp.x += 0.10 * h * h * sin(uTime * 1.2 + ph + h * 2.0);
        lp.y += 0.10 * h * h * cos(uTime * 0.9 + ph * 1.3 + h * 1.5);
    }
    float c = cos(yaw), sn = sin(yaw);
    mat2 R = mat2(c, sn, -sn, c);
    vPos = vec3(iA.xy, iB.z) + vec3(R * lp.xy, lp.z) * s - vec3(0.0, 0.0, 0.06 * s);
    vN = vec3(R * aN.xy, aN.z);
    vLocal = aP; vTone = aTone; vSeed = iB.x;
    gl_Position = uVP * vec4(vPos, 1.0);
}
)GLSL";

static const std::string OBJ_FS = R"GLSL(
in vec3 vPos; in vec3 vN; in vec3 vLocal; in float vTone; in float vSeed;
uniform int uKind;     // 0 rock, 1 branching coral, 2 brain coral, 3 table coral, 4 tube sponge, 5 grass
out vec4 fragColor;

vec3 coralHue(float seed, int kind) {
    int k = int(floor(fract(seed * 7.31) * 5.0));
    if (kind == 1) {            // branching: tan, pink, purple, yellow, green
        if (k == 0) return vec3(0.85, 0.55, 0.35); if (k == 1) return vec3(0.82, 0.35, 0.55);
        if (k == 2) return vec3(0.55, 0.40, 0.78); if (k == 3) return vec3(0.92, 0.78, 0.35);
        return vec3(0.45, 0.68, 0.40);
    }
    if (kind == 2) {            // brain: green, olive, brown, yellow, teal
        if (k == 0) return vec3(0.40, 0.62, 0.28); if (k == 1) return vec3(0.55, 0.55, 0.25);
        if (k == 2) return vec3(0.60, 0.42, 0.25); if (k == 3) return vec3(0.80, 0.70, 0.30);
        return vec3(0.30, 0.60, 0.50);
    }
    if (kind == 3) {            // table: pale blue, lilac, cream, rose, sea green
        if (k == 0) return vec3(0.60, 0.70, 0.85); if (k == 1) return vec3(0.70, 0.60, 0.85);
        if (k == 2) return vec3(0.85, 0.80, 0.62); if (k == 3) return vec3(0.88, 0.58, 0.62);
        return vec3(0.55, 0.78, 0.65);
    }
    if (k == 0) return vec3(0.90, 0.30, 0.15); if (k == 1) return vec3(0.95, 0.60, 0.15);   // tube sponge
    if (k == 2) return vec3(0.90, 0.80, 0.20); if (k == 3) return vec3(0.75, 0.25, 0.50);
    return vec3(0.85, 0.45, 0.30);
}

void main() {
    vec3 n = normalize(vN);
    if (!gl_FrontFacing) n = -n;
    vec3 alb = vec3(0.5);
    vec3 l = vLocal * 4.0 + vSeed * 50.0;
    float polyp = vnoise(vLocal.xy * 40.0 + vLocal.z * 30.0);

    if (uKind == 0) {                                   // rock
        vec3 bump = vec3(vnoise(l.yz * 5.0), vnoise(l.zx * 5.0), vnoise(l.xy * 5.0)) - 0.5;
        n = normalize(n + bump * 0.7);
        float st = vnoise(l.xy * 1.3 + l.z * 0.7);
        vec3 base = mix(vec3(0.24, 0.22, 0.20), vec3(0.46, 0.42, 0.36), st);
        base *= 0.75 + 0.5 * vnoise(l.yz * 6.0);
        float up = smoothstep(0.5, 0.9, n.z);
        float algae = up * smoothstep(0.35, 0.7, vnoise(l.xy * 2.0 + 9.0));
        base = mix(base, vec3(0.16, 0.28, 0.09), algae * 0.75);
        float barn = step(0.8, vnoise(l.xy * 22.0 + l.z * 9.0)) * (1.0 - up);
        alb = mix(base, vec3(0.80, 0.78, 0.72), barn * 0.7);
    } else if (uKind == 1) {                            // branching coral
        vec3 c = coralHue(vSeed, 1);
        alb = mix(c, vec3(1.0, 0.92, 0.82), smoothstep(0.75, 1.0, vTone) * 0.55) * (0.8 + 0.35 * polyp);
    } else if (uKind == 2) {                            // brain coral
        alb = coralHue(vSeed, 2) * (1.0 - 0.5 * vTone) * (0.85 + 0.3 * polyp);
    } else if (uKind == 3) {                            // table coral
        vec3 c = coralHue(vSeed, 3);
        alb = mix(c * 0.8, c, smoothstep(0.6, 1.0, vTone)) * (0.85 + 0.3 * polyp);
    } else if (uKind == 4) {                            // tube sponge
        alb = (vTone > 1.5) ? vec3(0.05, 0.02, 0.02) : coralHue(vSeed, 4) * (0.8 + 0.3 * polyp);
    } else {                                            // sea grass / kelp
        vec3 g = mix(vec3(0.10, 0.30, 0.08), vec3(0.45, 0.55, 0.15), vTone);
        vec3 k = mix(vec3(0.25, 0.20, 0.06), vec3(0.55, 0.45, 0.15), vTone);
        alb = (vSeed > 0.75) ? k : g;
    }
    fragColor = vec4(gammaOut(shadeUnderwater(alb, n, vPos, 1.0)), 1.0);
}
)GLSL";

// ---- floating particles ("marine snow") that make the water feel like a volume -----------------
static const std::string PART_VS = R"GLSL(
layout(location = 0) in vec3 aSeed;
uniform mat4 uVP; uniform float uBox; uniform vec3 uDrift; uniform float uPx;
out float vA;
void main() {
    vec3 p = mod(aSeed * uBox + uDrift * uTime - uCam, uBox) - 0.5 * uBox + uCam;
    float d = length(p - uCam);
    vec3 L = irradiance(max(-p.z, 0.0));
    vA = (p.z < -0.2 ? 1.0 : 0.0) * (0.15 + 0.85 * smoothstep(0.5 * uBox, 0.3 * uBox, d)) * exp(-uK.g * d);
    vA *= dot(L, vec3(0.33));
    gl_Position = uVP * vec4(p, 1.0);
    gl_PointSize = clamp(uPx * 0.012 / max(d, 0.3), 1.0, 5.0);
}
)GLSL";
static const std::string PART_FS = R"GLSL(
in float vA;
out vec4 fragColor;
void main() {
    float r = length(gl_PointCoord - 0.5) * 2.0;
    float a = smoothstep(1.0, 0.2, r) * vA * 3.0;
    fragColor = vec4(vec3(0.75, 0.9, 0.95) * a, a);
}
)GLSL";

// ---- sea surface -----------------------------------------------------------------------------
static const std::string SEA_VS = R"GLSL(
layout(location = 0) in vec2 aPos;
uniform mat4 uVP; uniform vec2 uCenter; uniform float uSize;
uniform int   uNW;
uniform vec4  uWave[32];     // kx, ky, omega, amplitude
uniform float uPhase[32];
uniform float uChop;         // 0 = smooth sine waves, 1 = sharp crests (Gerstner)
out vec3 vPos; out vec2 vSlope; out float vJac;
void main() {
    vec2 xy = uCenter + aPos * uSize;
    float h = 0.0, jac = 1.0; vec2 sl = vec2(0.0), disp = vec2(0.0);
    for (int i = 0; i < uNW; ++i) {
        vec2 k = uWave[i].xy; float kl = length(k);
        float th = dot(k, xy) - uWave[i].z * uTime + uPhase[i];
        float a = uWave[i].w, c = cos(th), s = sin(th);
        h += a * c;
        sl += -a * s * k;
        disp += -uChop * a * s * k / kl;
        jac -= uChop * a * kl * c;
    }
    vPos = vec3(xy + disp, h);
    vSlope = sl; vJac = jac;
    gl_Position = uVP * vec4(vPos, 1.0);
}
)GLSL";

static const std::string SEA_FS = R"GLSL(
in vec3 vPos; in vec2 vSlope; in float vJac;
uniform float uRough;        // how much small ripple detail (grows with wind)
uniform float uWaveDir;      // radians
uniform float uFoam;         // amount of white caps (grows with sea state)
out vec4 fragColor;

// tiny wind ripples that the mesh is too coarse to show: we only tilt the normal
vec2 ripples(vec2 p) {
    const float rk[6] = float[6](3.1, 4.6, 6.9, 9.7, 14.0, 21.0);
    const float ra[6] = float[6](-0.6, 0.4, -0.2, 0.7, -1.0, 0.1);
    vec2 s = vec2(0.0);
    for (int i = 0; i < 6; ++i) {
        float ang = uWaveDir + ra[i];
        vec2 dir = vec2(cos(ang), sin(ang));
        float k = rk[i], w = sqrt(9.81 * k);
        float th = k * dot(dir, p) - w * uTime + float(i) * 1.7;
        s += -uRough * 0.045 * pow(0.78, float(i)) * sin(th) * dir;
    }
    return s;
}

void main() {
    vec3 toC = uCam - vPos; float dist = length(toC); vec3 V = toC / dist;
    vec2 sl = (vSlope + ripples(vPos.xy)) * (1.0 - 0.6 * smoothstep(60.0, 240.0, dist));   // far waves look flatter
    vec3 n = normalize(vec3(-sl, 1.0));

    if (uCam.z >= 0.0) {
        // ----- seen from above: mirror of the sky; the sea floor shows through (blending) -----
        float cosv = clamp(dot(n, V), 0.0, 1.0);
        float F = 0.02 + 0.98 * pow(1.0 - cosv, 5.0);
        vec3 R = reflect(-V, n); R.z = abs(R.z);
        vec3 rgb = F * gammaOut(skyColour(R));
        float a = F;
        // white foam on breaking crests
        float foam = smoothstep(0.42, 0.10, vJac) * uFoam;
        foam *= 0.6 + 0.4 * sin(vPos.x * 3.1 + vPos.y * 2.3) * sin(vPos.y * 4.1 - vPos.x * 1.7);
        vec3 foamCol = gammaOut((uSunCol * max(uSunDir.z, 0.0) * 0.7 + ambientSky() * 0.8) * 0.9);
        rgb = mix(rgb, foamCol, foam); a = mix(a, 1.0, foam);
        // distant sea fades into the horizon
        float haze = smoothstep(35.0, 140.0, dist);
        vec3 hz = gammaOut(horizonSea());
        rgb = mix(rgb, hz, haze); a = mix(a, 1.0, haze);
        fragColor = vec4(rgb, a);
    } else {
        // ----- seen from below: a bright window straight up, a mirror at shallow angles -----
        vec3 nn = -n;
        float cosv = clamp(dot(nn, V), 0.0, 1.0);
        float dep = -uCam.z;
        vec3 Linf = veil(dep, 10000.0);
        vec3 col;
        if (N_WATER * sqrt(1.0 - cosv * cosv) >= 1.0) {
            col = Linf;                                               // total internal reflection
        } else {
            vec3 T = refract(-V, nn, N_WATER);
            float F = 0.02 + 0.98 * pow(1.0 - abs(T.z), 5.0);
            vec3 sky = skyColour(T) * exp(-uK * dist) + veil(0.5 * dep, dist);
            col = (1.0 - F) * sky + F * Linf;
        }
        fragColor = vec4(gammaOut(col), 1.0);
    }
}
)GLSL";