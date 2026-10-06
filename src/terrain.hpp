#pragma once
#include "gfx.hpp"
#include "noise.hpp"
#include <algorithm>

struct Terrain {
    float floor_depth = 14.0f;
    float relief = 2.5f;
    float half = 350.0f;
    int seed = 1;
    GLuint tex = 0;

    void build(int s) {
        seed = s;
        half = 350.0f;
    }

    float height(float x, float y) const {
        float nx = x * 0.065f + seed * 0.13f;
        float ny = y * 0.065f - seed * 0.11f;
        float base = fbm2(nx, ny) * (0.7f + relief * 0.35f);
        float dunes = 0.55f * std::sin((x + seed) * 0.11f) * std::cos((y - seed) * 0.09f);
        float ridge = 0.8f * std::sin((x * 0.14f + y * 0.08f) + seed * 0.5f);
        float h = base + dunes + ridge;
        return std::clamp(h, 0.0f, floor_depth);
    }

    float ground_z(float x, float y) const {
        return -floor_depth + height(x, y);
    }

    float rock_at(float x, float y) const {
        float r = fbm2(x * 0.12f + 98.0f + seed, y * 0.12f - 71.0f + seed);
        return std::clamp(0.35f + (r - 0.5f) * 1.35f, 0.0f, 1.0f);
    }
};