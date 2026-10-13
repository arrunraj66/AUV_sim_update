// design_io.hpp - save and load a design as a JSON file, and the built-in example designs.
// Self-contained (its own small JSON reader), so it does not depend on the Step 6A files.
// Files live in config/auv/<name>.json; the GUI lists that folder as "presets".
#pragma once
#include "auv_design.hpp"
#include <fstream>
#include <sstream>
#include <map>
#include <filesystem>
#include <memory>

namespace design {

// ---- a tiny JSON value -------------------------------------------------------------------------------------
struct JV {
    enum T { Null, Bool, Num, Str, Arr, Obj } t = Null;
    bool b = false; double n = 0; std::string s; std::vector<JV> a; std::map<std::string, JV> o;
    const JV* get(const std::string& k) const { if (t != Obj) return nullptr; auto i = o.find(k); return i == o.end() ? nullptr : &i->second; }
    double num(const std::string& k, double d) const { const JV* j = get(k); return j ? (j->t == Num ? j->n : (j->t == Bool ? (j->b ? 1.0 : 0.0) : d)) : d; }
    bool flag(const std::string& k, bool d) const { const JV* j = get(k); return j ? (j->t == Bool ? j->b : (j->t == Num ? j->n != 0 : d)) : d; }
    std::string str(const std::string& k, const std::string& d) const { const JV* j = get(k); return j && j->t == Str ? j->s : d; }
};

struct JParser {
    const std::string& x; size_t i = 0; bool ok = true;
    explicit JParser(const std::string& s) : x(s) {}
    void ws() { while (i < x.size() && (x[i] == ' ' || x[i] == '\n' || x[i] == '\t' || x[i] == '\r')) ++i; }
    bool lit(const char* w) { size_t k = std::char_traits<char>::length(w); if (x.compare(i, k, w) == 0) { i += k; return true; } return false; }
    JV val() {
        ws(); JV v; if (i >= x.size()) { ok = false; return v; }
        char c = x[i];
        if (c == '{') { ++i; v.t = JV::Obj; ws(); if (i < x.size() && x[i] == '}') { ++i; return v; }
            while (ok) { ws(); JV k = val(); if (k.t != JV::Str) { ok = false; break; } ws(); if (i >= x.size() || x[i] != ':') { ok = false; break; } ++i; v.o[k.s] = val(); ws();
                if (i < x.size() && x[i] == ',') { ++i; continue; } if (i < x.size() && x[i] == '}') { ++i; break; } ok = false; } return v; }
        if (c == '[') { ++i; v.t = JV::Arr; ws(); if (i < x.size() && x[i] == ']') { ++i; return v; }
            while (ok) { v.a.push_back(val()); ws(); if (i < x.size() && x[i] == ',') { ++i; continue; } if (i < x.size() && x[i] == ']') { ++i; break; } ok = false; } return v; }
        if (c == '"') { ++i; v.t = JV::Str; while (i < x.size() && x[i] != '"') { if (x[i] == '\\' && i + 1 < x.size()) { ++i; char e = x[i]; v.s += e == 'n' ? '\n' : (e == 't' ? '\t' : e); } else v.s += x[i]; ++i; } if (i >= x.size()) ok = false; ++i; return v; }
        if (lit("true")) { v.t = JV::Bool; v.b = true; return v; }
        if (lit("false")) { v.t = JV::Bool; v.b = false; return v; }
        if (lit("null")) return v;
        char* e = nullptr; v.n = std::strtod(x.c_str() + i, &e); if (e == x.c_str() + i) { ok = false; return v; } v.t = JV::Num; i = e - x.c_str(); return v;
    }
};

// ---- writer ------------------------------------------------------------------------------------------------
inline std::string jnum(double v) { char b[40]; std::snprintf(b, sizeof b, "%.10g", v); return b; }
inline std::string jstr(const std::string& s) { std::string o = "\""; for (char c : s) { if (c == '"' || c == '\\') { o += '\\'; o += c; } else if ((unsigned char)c >= 32) o += c; } return o + "\""; }

inline std::string to_json(const DesignIn& d) {
    std::ostringstream o;
    o << "{\n  \"name\": " << jstr(d.name) << ",\n";
    o << "  \"hull\": {\"L\": " << jnum(d.L) << ", \"D\": " << jnum(d.D) << ", \"nose_frac\": " << jnum(d.nose_frac) << ", \"tail_frac\": " << jnum(d.tail_frac)
      << ", \"nose_kind\": " << d.nose_kind << ", \"tail_end\": " << jnum(d.tail_end) << ", \"material\": " << d.material << ", \"rated_depth\": " << jnum(d.rated_depth) << ", \"trim\": " << jnum(d.trim) << "},\n";
    o << "  \"equipment\": {\"battery_Wh\": " << jnum(d.battery_Wh) << ", \"pack_Wh_per_kg\": " << jnum(d.pack_Wh_per_kg) << ", \"payload_kg\": " << jnum(d.payload_kg)
      << ", \"electronics_kg\": " << jnum(d.electronics_kg) << ", \"hotel_W\": " << jnum(d.hotel_W) << ", \"mast\": " << (d.mast ? "true" : "false")
      << ", \"bus_V\": " << jnum(d.bus_V) << ", \"cell_Ah\": " << jnum(d.cell_Ah) << ", \"cell_maxC\": " << jnum(d.cell_maxC) << "},\n";
    o << "  \"mission\": {\"v_cruise\": " << jnum(d.v_cruise) << ", \"v_max\": " << jnum(d.v_max) << ", \"water\": " << d.water << ", \"temp\": " << jnum(d.temp) << ", \"salinity\": " << jnum(d.salinity) << "},\n";
    o << "  \"sensors\": [";
    for (size_t i = 0; i < d.sensors.size(); ++i) o << (i ? ", " : "") << jstr(sensor_parts()[std::max(0, std::min(SE_COUNT - 1, d.sensors[i]))].id);
    o << "],\n  \"fins\": [\n";
    for (size_t i = 0; i < d.fins.size(); ++i) { const FinSet& f = d.fins[i];
        o << "    {\"on\": " << (f.on ? "true" : "false") << ", \"count\": " << f.count << ", \"angle0\": " << jnum(f.angle0) << ", \"x\": " << jnum(f.x) << ", \"cr\": " << jnum(f.cr) << ", \"ct\": " << jnum(f.ct)
          << ", \"span\": " << jnum(f.span) << ", \"sweep\": " << jnum(f.sweep) << ", \"tc\": " << jnum(f.tc) << ", \"control\": " << (f.control ? "true" : "false")
          << ", \"max_defl\": " << jnum(f.max_defl) << ", \"stock_mat\": " << f.stock_mat << "}" << (i + 1 < d.fins.size() ? "," : "") << "\n"; }
    o << "  ],\n  \"props\": [\n";
    for (size_t i = 0; i < d.props.size(); ++i) { const PropSpec& p = d.props[i];
        o << "    {\"role\": " << p.role << ", \"D\": " << jnum(p.D) << ", \"pd\": " << jnum(p.pd) << ", \"blades\": " << p.blades << ", \"ducted\": " << (p.ducted ? "true" : "false")
          << ", \"model\": " << p.model << ", \"ear\": " << jnum(p.ear) << ", \"contra\": " << (p.contra ? "true" : "false") << ", \"blades2\": " << p.blades2 << ", \"d2_ratio\": " << jnum(p.d2_ratio)
          << ", \"rpm\": " << jnum(p.rpm) << ", \"auto_rpm\": " << (p.auto_rpm ? "true" : "false") << ", \"x\": " << jnum(p.x) << ", \"y\": " << jnum(p.y) << ", \"z\": " << jnum(p.z)
          << ", \"shaft_mat\": " << p.shaft_mat << ", \"keyway\": " << (p.keyway ? "true" : "false") << ", \"bore\": " << jnum(p.bore) << "}" << (i + 1 < d.props.size() ? "," : "") << "\n"; }
    o << "  ]\n}\n";
    return o.str();
}

inline bool from_json(const std::string& text, DesignIn& d) {
    JParser ps(text); JV r = ps.val(); if (!ps.ok || r.t != JV::Obj) return false;
    DesignIn n; n.fins.clear(); n.props.clear();
    n.name = r.str("name", "My AUV");
    if (const JV* h = r.get("hull")) { n.L = h->num("L", n.L); n.D = h->num("D", n.D); n.nose_frac = h->num("nose_frac", n.nose_frac); n.tail_frac = h->num("tail_frac", n.tail_frac); n.nose_kind = (int)h->num("nose_kind", n.nose_kind);
        n.tail_end = h->num("tail_end", n.tail_end); n.material = (int)h->num("material", n.material); n.rated_depth = h->num("rated_depth", n.rated_depth); n.trim = h->num("trim", n.trim); }
    if (const JV* e = r.get("equipment")) { n.battery_Wh = e->num("battery_Wh", n.battery_Wh); n.pack_Wh_per_kg = e->num("pack_Wh_per_kg", n.pack_Wh_per_kg); n.payload_kg = e->num("payload_kg", n.payload_kg);
        n.electronics_kg = e->num("electronics_kg", n.electronics_kg); n.hotel_W = e->num("hotel_W", n.hotel_W); n.mast = e->flag("mast", n.mast);
        n.bus_V = e->num("bus_V", n.bus_V); n.cell_Ah = e->num("cell_Ah", n.cell_Ah); n.cell_maxC = e->num("cell_maxC", n.cell_maxC); }
    if (const JV* m = r.get("mission")) { n.v_cruise = m->num("v_cruise", n.v_cruise); n.v_max = m->num("v_max", n.v_max); n.water = (int)m->num("water", n.water); n.temp = m->num("temp", n.temp); n.salinity = m->num("salinity", n.salinity); }
    if (const JV* s = r.get("sensors")) for (const JV& e : s->a) for (int k = 0; k < SE_COUNT; ++k) if (e.t == JV::Str && e.s == sensor_parts()[k].id) n.sensors.push_back(k);
    if (const JV* f = r.get("fins")) for (const JV& e : f->a) { FinSet x; x.on = e.flag("on", true); x.count = (int)e.num("count", x.count); x.angle0 = e.num("angle0", x.angle0); x.x = e.num("x", x.x); x.cr = e.num("cr", x.cr); x.ct = e.num("ct", x.ct);
        x.span = e.num("span", x.span); x.sweep = e.num("sweep", x.sweep); x.tc = e.num("tc", x.tc); x.control = e.flag("control", true); x.max_defl = e.num("max_defl", x.max_defl); x.stock_mat = (int)e.num("stock_mat", 0); n.fins.push_back(x); }
    if (const JV* f = r.get("props")) for (const JV& e : f->a) { PropSpec p; p.role = (int)e.num("role", 0); p.D = e.num("D", p.D); p.pd = e.num("pd", p.pd); p.blades = (int)e.num("blades", p.blades); p.ducted = e.flag("ducted", true);
        p.model = (int)e.num("model", p.model); p.ear = e.num("ear", p.ear); p.contra = e.flag("contra", false); p.blades2 = (int)e.num("blades2", p.blades2); p.d2_ratio = e.num("d2_ratio", p.d2_ratio);
        p.rpm = e.num("rpm", p.rpm); p.auto_rpm = e.flag("auto_rpm", true); p.x = e.num("x", 0); p.y = e.num("y", 0); p.z = e.num("z", 0); p.shaft_mat = (int)e.num("shaft_mat", 0); p.keyway = e.flag("keyway", true); p.bore = e.num("bore", 0); n.props.push_back(p); }
    d = n; return true;
}

inline bool save_design(const std::string& path, const DesignIn& d) {
    std::error_code ec; std::filesystem::create_directories(std::filesystem::path(path).parent_path(), ec);
    std::ofstream f(path); if (!f) return false; f << to_json(d); return (bool)f;
}
inline bool load_design(const std::string& path, DesignIn& d) {
    std::ifstream f(path); if (!f) return false; std::stringstream ss; ss << f.rdbuf(); return from_json(ss.str(), d);
}
inline std::vector<std::string> list_designs(const std::string& dir) {          // file names without .json, sorted
    std::vector<std::string> v; std::error_code ec;
    for (const auto& e : std::filesystem::directory_iterator(dir, ec)) if (e.path().extension() == ".json") v.push_back(e.path().stem().string());
    std::sort(v.begin(), v.end()); return v;
}

// ---- built-in example designs ----------------------------------------------------------------------------
inline std::vector<DesignIn> builtin_designs() {
    std::vector<DesignIn> v;
    { DesignIn d = default_design(); d.name = "REMUS 100 class"; d.sensors = {SE_IMU, SE_COMPASS, SE_DEPTH, SE_DVL, SE_GPS, SE_CAMERA}; d.payload_kg = 0.5;
      d.fins[0].cr = 0.094; d.fins[0].ct = 0.061; d.fins[0].span = 0.0857;      // Prestero Table 2.2: S_fin 6.65e-3 m2, span 8.57e-2 m, taper 0.654 (the preset had 0.12 / 0.07 / 0.09 m = 8.55e-3 m2)
      v.push_back(d); }
    { DesignIn d = default_design(); d.name = "Mini AUV 0.9 m"; d.L = 0.9; d.D = 0.12; d.battery_Wh = 200; d.payload_kg = 0.2; d.electronics_kg = 0.6; d.rated_depth = 50; d.props[0].D = 0.11; d.v_max = 1.5; d.v_cruise = 1.0;
      d.fins[0].span = 0.06; d.fins[0].cr = 0.08; d.fins[0].ct = 0.05; d.fins[0].x = -0.35; d.bus_V = 14.8; d.sensors = {SE_IMU, SE_DEPTH, SE_CAMERA}; v.push_back(d); }
    { DesignIn d = default_design(); d.name = "Survey AUV (sonar, contra-rotating)"; d.L = 2.4; d.D = 0.32; d.battery_Wh = 3500; d.payload_kg = 2.0; d.electronics_kg = 4.0; d.rated_depth = 200; d.bus_V = 44.4; d.v_cruise = 1.8; d.v_max = 2.8;
      d.props[0].D = 0.30; d.props[0].contra = true; d.props[0].blades = 3; d.props[0].blades2 = 4;
      d.fins[0].x = -0.95; d.fins[0].cr = 0.20; d.fins[0].ct = 0.12; d.fins[0].span = 0.18; d.fins[0].sweep = 0.05;
      d.sensors = {SE_IMU, SE_COMPASS, SE_DEPTH, SE_DVL, SE_GPS, SE_MBES, SE_SSS, SE_FLS, SE_CAMERA}; v.push_back(d); }
    { DesignIn d = default_design(); d.name = "Hovering inspection AUV"; d.L = 1.25; d.D = 0.26; d.battery_Wh = 800; d.payload_kg = 1.0; d.rated_depth = 100; d.v_cruise = 0.8; d.v_max = 1.5; d.props[0].D = 0.14;
      d.bus_V = 44.4; d.cell_maxC = 3.0; d.fins[0].x = -0.50; d.fins[0].cr = 0.12; d.fins[0].ct = 0.08; d.fins[0].span = 0.10; d.fins[0].sweep = 0.02; d.tail_end = 0.5;
      PropSpec t; t.role = 1; t.D = 0.08; t.rpm = 3000; t.x = 0.30; d.props.push_back(t); t.x = -0.30; d.props.push_back(t);
      PropSpec l = t; l.role = 2; l.x = 0.25; d.props.push_back(l); l.x = -0.25; d.props.push_back(l);
      d.sensors = {SE_IMU, SE_COMPASS, SE_DEPTH, SE_CAMERA, SE_FLS}; v.push_back(d); }
    return v;
}
inline int write_builtin_designs(const std::string& dir) {          // writes the examples that are not there yet; returns how many were written
    int n = 0; for (const DesignIn& d : builtin_designs()) { std::string f; for (char c : d.name) f += (std::isalnum((unsigned char)c) ? c : '_');
        std::string p = dir + "/" + f + ".json"; if (!std::filesystem::exists(p)) { save_design(p, d); ++n; } }
    return n;
}

}  // namespace design
