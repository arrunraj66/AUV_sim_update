// vshader.hpp - GPU program that draws the vehicles (paint, plastic, glass, metal) with the same underwater light as the rest of the scene.
#pragma once
#include <string>

static const std::string VEH_VS = R"GLSL(
layout(location = 0) in vec3 aP;
layout(location = 1) in vec3 aN;
layout(location = 2) in float aTone;
uniform mat4 uVP;
uniform mat4 uModel;
out vec3 vPos; out vec3 vN; out vec3 vLocal; out float vTone;
void main() {
    vec4 wp = uModel * vec4(aP, 1.0);
    vPos = wp.xyz; vN = mat3(uModel) * aN; vLocal = aP; vTone = aTone;
    gl_Position = uVP * wp;
}
)GLSL";

static const std::string VEH_FS = R"GLSL(
in vec3 vPos; in vec3 vN; in vec3 vLocal; in float vTone;
uniform int uMesh;          // 0 torpedo, 1 hover, 2 glider, 3 scout, 4 stl
uniform vec3 uStlCol;
out vec4 fragColor;

void main() {
    vec3 n = normalize(vN);
    if (!gl_FrontFacing) n = -n;
    int t = int(vTone + 0.5);
    vec3 alb = vec3(0.5); float gloss = 0.4;
    float grain = vnoise(vLocal.xy * 90.0 + vLocal.z * 70.0);
    if (t == 0)      { alb = vec3(0.95, 0.70, 0.05); gloss = 0.9; if (uMesh == 0 && (abs(vLocal.x - 0.12) < 0.010 || abs(vLocal.x + 0.18) < 0.010)) alb = vec3(0.03); }
    else if (t == 1) { alb = vec3(0.045); gloss = 0.35; }
    else if (t == 2) { alb = vec3(0.55, 0.57, 0.60) * (0.9 + 0.2 * grain); gloss = 1.0; }
    else if (t == 3) { alb = vec3(0.85, 0.86, 0.84) * (0.92 + 0.12 * grain); gloss = 0.5; }
    else if (t == 4) { alb = vec3(0.95, 0.34, 0.04); gloss = 0.9; }
    else if (t == 5) { alb = vec3(0.04, 0.22, 0.75); gloss = 0.7; }
    else if (t == 6) { alb = vec3(0.03, 0.05, 0.07); gloss = 1.0; }
    else if (t == 7) { alb = vec3(0.85, 0.05, 0.04); gloss = 0.6; }
    else             { alb = uStlCol * (0.92 + 0.12 * grain); gloss = 0.6; }

    float dF = max(-vPos.z, 0.0);
    vec3 V = normalize(uCam - vPos);
    vec3 col, spec;
    if (vPos.z > 0.0) {                       // the part of the vehicle above the waves: ordinary sunlight
        col = alb * (uSunCol * max(dot(n, uSunDir), 0.0) * 0.9 + ambientSky() * 0.7 * (0.5 + 0.5 * n.z));
        spec = uSunCol * pow(max(dot(reflect(-uSunDir, n), V), 0.0), 60.0) * gloss * 0.6;
        col += spec;
    } else {
        vec3 Lw = -refract(-uSunDir, vec3(0, 0, 1), 1.0 / N_WATER);
        col = shadeUnderwater(alb, n, vPos, 0.7);
        float d = length(uCam - vPos);
        float sp = pow(max(dot(reflect(-Lw, n), V), 0.0), 60.0) * gloss * step(0.0, uSunDir.z);
        col += uSunCol * sp * 0.6 * exp(-uK * (dF / max(Lw.z, 0.3) + d));
        if (t == 6 && uLamp > 0.0) col += vec3(1.0, 0.95, 0.8) * 0.6 * exp(-uK * d);     // lamp window glows
        float rim = pow(1.0 - max(dot(n, V), 0.0), 4.0);
        col += 0.03 * rim * exp(-uK * d) * ambientSky() * exp(-uK * dF);
    }
    fragColor = vec4(gammaOut(col), 1.0);
}
)GLSL";