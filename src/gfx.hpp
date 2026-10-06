// gfx.hpp - small OpenGL helpers: matrices, shader compiling, a flat grid mesh.
#pragma once
#define GL_GLEXT_PROTOTYPES
#include <SDL.h>
#include <SDL_opengl.h>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

struct Vec3f { float x, y, z; };
inline Vec3f sub(Vec3f a, Vec3f b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
inline Vec3f cross(Vec3f a, Vec3f b) { return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x}; }
inline float dot(Vec3f a, Vec3f b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline Vec3f norm(Vec3f a) { float l = std::sqrt(dot(a, a)); return {a.x / l, a.y / l, a.z / l}; }

struct Mat4 { float m[16]; };   // column-major, as OpenGL wants it
inline Mat4 mul(const Mat4& a, const Mat4& b) {
    Mat4 r;
    for (int c = 0; c < 4; ++c) for (int rr = 0; rr < 4; ++rr) {
        float s = 0; for (int k = 0; k < 4; ++k) s += a.m[k * 4 + rr] * b.m[c * 4 + k];
        r.m[c * 4 + rr] = s;
    }
    return r;
}
inline Mat4 perspective(float fovy, float aspect, float zn, float zf) {
    float f = 1.0f / std::tan(fovy / 2);
    Mat4 r = {}; r.m[0] = f / aspect; r.m[5] = f; r.m[10] = (zf + zn) / (zn - zf); r.m[11] = -1; r.m[14] = 2 * zf * zn / (zn - zf);
    return r;
}
inline Mat4 look_at(Vec3f eye, Vec3f target, Vec3f up) {
    Vec3f f = norm(sub(target, eye)), s = norm(cross(f, up)), u = cross(s, f);
    Mat4 r = {};
    r.m[0] = s.x; r.m[4] = s.y; r.m[8] = s.z;
    r.m[1] = u.x; r.m[5] = u.y; r.m[9] = u.z;
    r.m[2] = -f.x; r.m[6] = -f.y; r.m[10] = -f.z;
    r.m[12] = -dot(s, eye); r.m[13] = -dot(u, eye); r.m[14] = dot(f, eye); r.m[15] = 1;
    return r;
}

inline GLuint compile(GLenum type, const std::string& src, const char* name) {
    GLuint s = glCreateShader(type);
    const char* p = src.c_str();
    glShaderSource(s, 1, &p, nullptr);
    glCompileShader(s);
    GLint ok; glGetShaderiv(s, GL_COMPILE_STATUS, &ok);
    if (!ok) { char log[2048]; glGetShaderInfoLog(s, 2048, nullptr, log); std::fprintf(stderr, "Shader error in %s:\n%s\n", name, log); std::exit(1); }
    return s;
}
inline GLuint make_program(const std::string& vs, const std::string& fs, const char* name) {
    GLuint p = glCreateProgram();
    glAttachShader(p, compile(GL_VERTEX_SHADER, vs, name));
    glAttachShader(p, compile(GL_FRAGMENT_SHADER, fs, name));
    glLinkProgram(p);
    GLint ok; glGetProgramiv(p, GL_LINK_STATUS, &ok);
    if (!ok) { char log[2048]; glGetProgramInfoLog(p, 2048, nullptr, log); std::fprintf(stderr, "Link error in %s:\n%s\n", name, log); std::exit(1); }
    return p;
}

// uniform helpers
inline void u1f(GLuint p, const char* n, float v) { glUniform1f(glGetUniformLocation(p, n), v); }
inline void u1i(GLuint p, const char* n, int v) { glUniform1i(glGetUniformLocation(p, n), v); }
inline void u2f(GLuint p, const char* n, float a, float b) { glUniform2f(glGetUniformLocation(p, n), a, b); }
inline void u3f(GLuint p, const char* n, float a, float b, float c) { glUniform3f(glGetUniformLocation(p, n), a, b, c); }

// A flat square grid, n x n cells, with x,y from -0.5 to +0.5. The shaders scale and move it.
struct GridMesh {
    GLuint vao = 0, vbo = 0, ebo = 0; int index_count = 0;
    void build(int n) {
        std::vector<float> v; std::vector<unsigned> idx;
        for (int j = 0; j <= n; ++j) for (int i = 0; i <= n; ++i) { v.push_back((float)i / n - 0.5f); v.push_back((float)j / n - 0.5f); }
        for (int j = 0; j < n; ++j) for (int i = 0; i < n; ++i) {
            unsigned a = j * (n + 1) + i, b = a + 1, c = a + (n + 1), d = c + 1;
            idx.insert(idx.end(), {a, b, d, a, d, c});     // counter-clockwise seen from above
        }
        index_count = (int)idx.size();
        glGenVertexArrays(1, &vao); glBindVertexArray(vao);
        glGenBuffers(1, &vbo); glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glBufferData(GL_ARRAY_BUFFER, v.size() * sizeof(float), v.data(), GL_STATIC_DRAW);
        glGenBuffers(1, &ebo); glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, idx.size() * sizeof(unsigned), idx.data(), GL_STATIC_DRAW);
        glEnableVertexAttribArray(0); glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 0, nullptr);
    }
    void draw() const { glBindVertexArray(vao); glDrawElements(GL_TRIANGLES, index_count, GL_UNSIGNED_INT, nullptr); }
};