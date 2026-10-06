// noise.hpp - smooth random numbers on the CPU (used to make the sea floor and the reef).
#pragma once
#include <cmath>
#include <cstdint>

inline float fractf(float x) { return x - std::floor(x); }
inline float smooth01(float a, float b, float x) {
    float t = (x - a) / (b - a); t = t < 0 ? 0 : (t > 1 ? 1 : t);
    return t * t * (3 - 2 * t);
}

// 2D value noise, same recipe as the GPU colour noise
inline float hash2(float px, float py) {
    px = fractf(px * 123.34f); py = fractf(py * 456.21f);
    float d = px * (px + 45.32f) + py * (py + 45.32f);
    px += d; py += d;
    return fractf(px * py);
}
inline float vnoise2(float x, float y) {
    float ix = std::floor(x), iy = std::floor(y), fx = x - ix, fy = y - iy;
    fx = fx * fx * (3 - 2 * fx); fy = fy * fy * (3 - 2 * fy);
    float a = hash2(ix, iy), b = hash2(ix + 1, iy), c = hash2(ix, iy + 1), d = hash2(ix + 1, iy + 1);
    return (a + (b - a) * fx) + ((c + (d - c) * fx) - (a + (b - a) * fx)) * fy;
}
inline float fbm2(float x, float y) {      // 5 layers of noise, each finer and weaker
    float a = 0.5f, s = 0;
    for (int i = 0; i < 5; ++i) { s += a * vnoise2(x, y); x = x * 2.03f + 17.1f; y = y * 2.03f + 9.2f; a *= 0.5f; }
    return s;
}

// 3D value noise (for shaping rocks and corals)
inline float hash3i(int x, int y, int z, int seed) {
    uint32_t h = (uint32_t)x * 374761393u + (uint32_t)y * 668265263u + (uint32_t)z * 2147483647u + (uint32_t)seed * 362437u;
    h = (h ^ (h >> 13)) * 1274126177u; h ^= h >> 16;
    return (h & 0xFFFFFF) / 16777215.0f;
}
inline float noise3(float x, float y, float z, int seed) {
    int ix = (int)std::floor(x), iy = (int)std::floor(y), iz = (int)std::floor(z);
    float fx = x - ix, fy = y - iy, fz = z - iz;
    fx = fx * fx * (3 - 2 * fx); fy = fy * fy * (3 - 2 * fy); fz = fz * fz * (3 - 2 * fz);
    auto L = [](float a, float b, float t) { return a + (b - a) * t; };
    float c00 = L(hash3i(ix, iy, iz, seed), hash3i(ix + 1, iy, iz, seed), fx);
    float c10 = L(hash3i(ix, iy + 1, iz, seed), hash3i(ix + 1, iy + 1, iz, seed), fx);
    float c01 = L(hash3i(ix, iy, iz + 1, seed), hash3i(ix + 1, iy, iz + 1, seed), fx);
    float c11 = L(hash3i(ix, iy + 1, iz + 1, seed), hash3i(ix + 1, iy + 1, iz + 1, seed), fx);
    return L(L(c00, c10, fy), L(c01, c11, fy), fz);
}
inline float fbm3(float x, float y, float z, int seed) {
    float a = 0.5f, s = 0;
    for (int i = 0; i < 4; ++i) { s += a * noise3(x, y, z, seed + i); x *= 2.1f; y *= 2.1f; z *= 2.1f; a *= 0.5f; }
    return s / 0.9375f;
}