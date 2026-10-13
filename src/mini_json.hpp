// mini_json.hpp - a very small JSON reader/writer (no libraries needed), enough for config files.
#pragma once
#include <string>
#include <vector>
#include <map>
#include <cstdlib>
#include <cstdio>
#include <cmath>

struct Json {
    enum Type { Null, Bool, Num, Str, Arr, Obj } t = Null;
    bool b = false; double n = 0; std::string s; std::vector<Json> a; std::map<std::string, Json> o;

    static Json number(double v) { Json j; j.t = Num; j.n = v; return j; }
    static Json boolean(bool v) { Json j; j.t = Bool; j.b = v; return j; }
    static Json string(const std::string& v) { Json j; j.t = Str; j.s = v; return j; }
    static Json array() { Json j; j.t = Arr; return j; }
    static Json object() { Json j; j.t = Obj; return j; }
    Json& operator[](const std::string& k) { t = Obj; return o[k]; }
    void push(const Json& v) { t = Arr; a.push_back(v); }
    const Json* find(const std::string& k) const { if (t != Obj) return nullptr; auto it = o.find(k); return it == o.end() ? nullptr : &it->second; }
    double num_or(const std::string& k, double d) const { const Json* j = find(k); return j && j->t == Num ? j->n : (j && j->t == Bool ? (j->b ? 1.0 : 0.0) : d); }
    bool bool_or(const std::string& k, bool d) const { const Json* j = find(k); return j && j->t == Bool ? j->b : (j && j->t == Num ? j->n != 0 : d); }
    std::string str_or(const std::string& k, const std::string& d) const { const Json* j = find(k); return j && j->t == Str ? j->s : d; }

    static void esc(std::string& out, const std::string& s) {
        out += '"';
        for (char c : s) { if (c == '"') out += "\\\""; else if (c == '\\') out += "\\\\"; else if (c == '\n') out += "\\n"; else if (c == '\t') out += "\\t"; else if ((unsigned char)c < 32) out += ' '; else out += c; }
        out += '"';
    }
    void write(std::string& out, int ind) const {
        std::string pad(ind * 2, ' '), pad2((ind + 1) * 2, ' ');
        switch (t) {
            case Null: out += "null"; break;
            case Bool: out += b ? "true" : "false"; break;
            case Num: { char buf[40]; if (n == std::floor(n) && std::fabs(n) < 1e15) std::snprintf(buf, sizeof buf, "%.0f", n); else std::snprintf(buf, sizeof buf, "%.9g", n); out += buf; break; }
            case Str: esc(out, s); break;
            case Arr:
                if (a.empty()) { out += "[]"; break; }
                out += "[\n";
                for (size_t i = 0; i < a.size(); ++i) { out += pad2; a[i].write(out, ind + 1); out += i + 1 < a.size() ? ",\n" : "\n"; }
                out += pad + "]"; break;
            case Obj:
                if (o.empty()) { out += "{}"; break; }
                out += "{\n"; { size_t i = 0;
                for (auto& kv : o) { out += pad2; esc(out, kv.first); out += ": "; kv.second.write(out, ind + 1); out += ++i < o.size() ? ",\n" : "\n"; } }
                out += pad + "}"; break;
        }
    }
    std::string dump() const { std::string out; write(out, 0); out += "\n"; return out; }

    // ---- parser
    struct P {
        const std::string& s; size_t i = 0; std::string err;
        explicit P(const std::string& str) : s(str) {}
        void ws() { while (i < s.size() && (s[i] == ' ' || s[i] == '\n' || s[i] == '\t' || s[i] == '\r')) ++i; }
        bool fail(const char* m) { if (err.empty()) err = std::string(m) + " at character " + std::to_string(i); return false; }
        bool str(std::string& out) {
            if (i >= s.size() || s[i] != '"') return fail("expected a string");
            ++i; out.clear();
            while (i < s.size() && s[i] != '"') {
                if (s[i] == '\\' && i + 1 < s.size()) { ++i; char c = s[i]; out += c == 'n' ? '\n' : c == 't' ? '\t' : c; }
                else out += s[i];
                ++i;
            }
            if (i >= s.size()) return fail("unterminated string");
            ++i; return true;
        }
        bool value(Json& j) {
            ws(); if (i >= s.size()) return fail("unexpected end");
            char c = s[i];
            if (c == '{') {
                ++i; j = Json::object(); ws();
                if (i < s.size() && s[i] == '}') { ++i; return true; }
                while (true) {
                    ws(); std::string k; if (!str(k)) return false; ws();
                    if (i >= s.size() || s[i] != ':') return fail("expected ':'");
                    ++i; Json v; if (!value(v)) return false; j.o[k] = v; ws();
                    if (i < s.size() && s[i] == ',') { ++i; continue; }
                    if (i < s.size() && s[i] == '}') { ++i; return true; }
                    return fail("expected ',' or '}'");
                }
            }
            if (c == '[') {
                ++i; j = Json::array(); ws();
                if (i < s.size() && s[i] == ']') { ++i; return true; }
                while (true) {
                    Json v; if (!value(v)) return false; j.a.push_back(v); ws();
                    if (i < s.size() && s[i] == ',') { ++i; continue; }
                    if (i < s.size() && s[i] == ']') { ++i; return true; }
                    return fail("expected ',' or ']'");
                }
            }
            if (c == '"') { j.t = Str; return str(j.s); }
            if (s.compare(i, 4, "true") == 0) { i += 4; j = Json::boolean(true); return true; }
            if (s.compare(i, 5, "false") == 0) { i += 5; j = Json::boolean(false); return true; }
            if (s.compare(i, 4, "null") == 0) { i += 4; j = Json(); return true; }
            char* end = nullptr; double v = std::strtod(s.c_str() + i, &end);
            if (end == s.c_str() + i) return fail("unexpected character");
            i = end - s.c_str(); j = Json::number(v); return true;
        }
    };
    static bool parse(const std::string& text, Json& out, std::string& err) {
        P p(text); if (!p.value(out)) { err = p.err; return false; } p.ws();
        if (p.i != text.size()) { err = "extra text at character " + std::to_string(p.i); return false; }
        return true;
    }
};
