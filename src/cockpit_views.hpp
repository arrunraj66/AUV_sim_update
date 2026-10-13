// cockpit_views.hpp - the data-heavy Cockpit widgets: live plots, map, thruster/fin panel, sensor readouts and sonar views.
// They only draw; all state they need is passed in.
#pragma once
#include "config.hpp"
#include "fleet.hpp"
#include <deque>
#include <map>

// ------------------------------------------------------------------------------------------------ colours
namespace ck {
inline ImU32 C(int r, int g, int b, int a = 255) { return IM_COL32(r, g, b, a); }
static const ImU32 TEXT = IM_COL32(226, 236, 244, 255), DIM = IM_COL32(128, 148, 166, 255), LINE = IM_COL32(120, 215, 235, 255), ACCENT = IM_COL32(40, 200, 225, 255);
static const ImU32 OK = IM_COL32(80, 220, 140, 255), WARN = IM_COL32(255, 190, 60, 255), BAD = IM_COL32(255, 85, 80, 255), GLASS = IM_COL32(10, 18, 26, 120);
inline ImU32 alpha(ImU32 c, float a) { return (c & 0x00FFFFFF) | ((ImU32)(255.f * std::max(0.f, std::min(1.f, a))) << 24); }
}

// ------------------------------------------------------------------------------------------------ history of each vehicle
struct Sample { float t, depth, alt, speed, power, batt; };
struct Hist {
    std::deque<Sample> s; std::vector<std::pair<float, float>> trail; double acc = 0, tacc = 0, t = 0;
    V3 prev_v; double prev_t = -1; V3 acc_b;             // for the IMU preview (acceleration from the velocity change)
};

// ------------------------------------------------------------------------------------------------ plots
struct PlotState { int win = 1; bool on[4] = {true, true, true, true}; };
inline void plot_one(ImDrawList* dl, ImVec2 a, ImVec2 b, const std::deque<Sample>& s, float Sample::*f, const char* name, const char* unit, ImU32 col, bool down, double win, float vmin_floor) {
    dl->AddRectFilled(a, b, IM_COL32(8, 14, 22, 150), 5.f); dl->AddRect(a, b, IM_COL32(255, 255, 255, 30), 5.f);
    float w = b.x - a.x, h = b.y - a.y; double now = s.empty() ? 0 : s.back().t;
    float lo = 1e30f, hi = -1e30f;
    for (auto& e : s) if (now - e.t <= win) { lo = std::min(lo, e.*f); hi = std::max(hi, e.*f); }
    if (lo > hi) { lo = 0; hi = 1; }
    bool floored = lo >= vmin_floor; if (floored) lo = vmin_floor;
    float span = std::max(hi - lo, 1e-3f), pad = span * 0.1f; hi += pad; if (!floored) lo -= pad; span = hi - lo;
    for (int g = 1; g < 4; ++g) { float y = a.y + h * g / 4.f; dl->AddLine(ImVec2(a.x, y), ImVec2(b.x, y), IM_COL32(255, 255, 255, 18)); }
    auto ymap = [&](float v) { float u = (v - lo) / span; return down ? a.y + u * h : b.y - u * h; };
    static std::vector<ImVec2> pts; pts.clear();
    for (auto& e : s) if (now - e.t <= win) pts.push_back(ImVec2(b.x - (float)((now - e.t) / win) * w, ymap(e.*f)));
    if (pts.size() > 1) { dl->PushClipRect(a, b, true); dl->AddPolyline(pts.data(), (int)pts.size(), col, 0, 2.0f); dl->PopClipRect(); }
    char t[64]; std::snprintf(t, sizeof t, "%s  %.*f %s", name, std::fabs(s.empty() ? 0 : s.back().*f) < 10 ? 2 : 1, s.empty() ? 0.f : s.back().*f, unit);
    dl->AddText(ImVec2(a.x + 8, a.y + 4), col, t);
    char r[32]; std::snprintf(r, sizeof r, "%.3g", down ? lo : hi); dl->AddText(ImVec2(b.x - ImGui::CalcTextSize(r).x - 6, a.y + 4), ck::DIM, r);
    std::snprintf(r, sizeof r, "%.3g", down ? hi : lo); dl->AddText(ImVec2(b.x - ImGui::CalcTextSize(r).x - 6, b.y - ImGui::GetFontSize() - 3), ck::DIM, r);
}
inline void view_plots(Hist& H, PlotState& ps) {
    ImGui::SetNextItemWidth(110); const char* wn[3] = {"30 s", "2 min", "10 min"}; ImGui::Combo("##win", &ps.win, wn, 3); ImGui::SameLine();
    const char* nm[4] = {"Depth", "Speed", "Power", "Battery"}; for (int i = 0; i < 4; ++i) { ImGui::Checkbox(nm[i], &ps.on[i]); if (i < 3) ImGui::SameLine(); }
    ImVec2 p = ImGui::GetCursorScreenPos(), sz = ImGui::GetContentRegionAvail(); ImDrawList* dl = ImGui::GetWindowDrawList();
    int n = 0; for (bool b : ps.on) n += b; if (!n || sz.y < 30) return;
    double win = ps.win == 0 ? 30 : ps.win == 1 ? 120 : 600; float rh = (sz.y - 4.f * (n - 1)) / n, y = p.y;
    for (int i = 0; i < 4; ++i) if (ps.on[i]) {
        ImVec2 a(p.x, y), b(p.x + sz.x, y + rh);
        if (i == 0) plot_one(dl, a, b, H.s, &Sample::depth, "Depth", "m", ck::LINE, true, win, 0.f);
        if (i == 1) plot_one(dl, a, b, H.s, &Sample::speed, "Speed", "m/s", ck::OK, false, win, 0.f);
        if (i == 2) plot_one(dl, a, b, H.s, &Sample::power, "Power", "W", ck::WARN, false, win, 0.f);
        if (i == 3) plot_one(dl, a, b, H.s, &Sample::batt, "Battery", "%", IM_COL32(190, 150, 255, 255), false, win, 0.f);
        y += rh + 4;
    }
    ImGui::Dummy(sz);
}

// ------------------------------------------------------------------------------------------------ map
struct MapState {
    float range = 60;                       // half width of the map, m
    bool follow = true; float cx = 0, cy = 0;
    static constexpr int N = 44; float grid[N * N]; float gcx = 1e9f, gcy = 1e9f, grange = 0; double gt = -1; float dmin = 0, dmax = 1;
};
inline void view_map(MapState& M, const Terrain& T, const Current& cur, Fleet& F, Unit& u, Hist& H, double now) {
    ImGuiIO& io = ImGui::GetIO(); Vehicle& v = u.veh;
    if (ImGui::SmallButton("-")) M.range = std::min(800.f, M.range * 1.4f); ImGui::SameLine(); if (ImGui::SmallButton("+")) M.range = std::max(10.f, M.range / 1.4f);
    ImGui::SameLine(); ImGui::Checkbox("Follow", &M.follow); ImGui::SameLine(); ImGui::TextDisabled("%.0f m across", M.range * 2);
    ImVec2 p = ImGui::GetCursorScreenPos(), sz = ImGui::GetContentRegionAvail(); float side = std::min(sz.x, sz.y); if (side < 40) return;
    ImVec2 c0(p.x + (sz.x - side) * 0.5f, p.y), c1(c0.x + side, c0.y + side), ctr((c0.x + c1.x) * 0.5f, (c0.y + c1.y) * 0.5f);
    ImGui::InvisibleButton("##map", ImVec2(sz.x, side));
    if (ImGui::IsItemHovered() && io.MouseWheel != 0) M.range = std::max(10.f, std::min(800.f, M.range * std::pow(0.85f, io.MouseWheel)));
    if (M.follow) { M.cx = (float)v.p.x; M.cy = (float)v.p.y; }
    else if (ImGui::IsItemActive() && ImGui::IsMouseDragging(0)) { float ppm = side / (2 * M.range); M.cx -= io.MouseDelta.x / ppm; M.cy += io.MouseDelta.y / ppm; }
    float ppm = side / (2 * M.range); ImDrawList* dl = ImGui::GetWindowDrawList();
    // sea-bed shading (cached, refreshed a few times per second)
    if (std::fabs(M.cx - M.gcx) > M.range * 0.04f || std::fabs(M.cy - M.gcy) > M.range * 0.04f || M.range != M.grange || now - M.gt > 1.0) {
        M.gcx = M.cx; M.gcy = M.cy; M.grange = M.range; M.gt = now; float lo = 1e9f, hi = -1e9f;
        for (int j = 0; j < M.N; ++j) for (int i = 0; i < M.N; ++i) {
            float x = M.cx + ((i + 0.5f) / M.N * 2 - 1) * M.range, y = M.cy + (1 - (j + 0.5f) / M.N * 2) * M.range, d = -T.ground_z(x, y);
            M.grid[j * M.N + i] = d; lo = std::min(lo, d); hi = std::max(hi, d);
        }
        M.dmin = lo; M.dmax = std::max(hi, lo + 3.f);
    }
    dl->AddRectFilled(c0, c1, IM_COL32(6, 14, 22, 200), 6.f); dl->PushClipRect(c0, c1, true);
    float cw = side / M.N;
    for (int j = 0; j < M.N; ++j) for (int i = 0; i < M.N; ++i) {
        float t = (M.grid[j * M.N + i] - M.dmin) / (M.dmax - M.dmin); t = std::max(0.f, std::min(1.f, t));
        int r = (int)(70 - 55 * t), g = (int)(150 - 105 * t), b = (int)(165 - 95 * t);
        dl->AddRectFilled(ImVec2(c0.x + i * cw, c0.y + j * cw), ImVec2(c0.x + (i + 1) * cw + 0.5f, c0.y + (j + 1) * cw + 0.5f), IM_COL32(r, g, b, 215));
    }
    auto S = [&](double x, double y) { return ImVec2(ctr.x + (float)(x - M.cx) * ppm, ctr.y - (float)(y - M.cy) * ppm); };
    // range rings and grid
    float step = 1;
    { float s10 = std::pow(10.f, std::floor(std::log10(M.range / 2))); step = s10; if (step * 2 * 1.0f < M.range / 2.5f) step *= 2; if (step * 2.5f < M.range / 3) step *= 2.5f; }
    for (float g = std::floor((M.cx - M.range) / step) * step; g < M.cx + M.range; g += step) dl->AddLine(S(g, M.cy - M.range), S(g, M.cy + M.range), IM_COL32(255, 255, 255, 22));
    for (float g = std::floor((M.cy - M.range) / step) * step; g < M.cy + M.range; g += step) dl->AddLine(S(M.cx - M.range, g), S(M.cx + M.range, g), IM_COL32(255, 255, 255, 22));
    // trail
    if (H.trail.size() > 1) {
        for (size_t i = 1; i < H.trail.size(); ++i) { float a = (float)i / H.trail.size(); dl->AddLine(S(H.trail[i - 1].first, H.trail[i - 1].second), S(H.trail[i].first, H.trail[i].second), ck::alpha(ck::WARN, 0.15f + 0.8f * a), 2.0f); }
    }
    // other vehicles
    for (Unit& o : F.units) if (&o != &u) { ImVec2 q = S(o.veh.p.x, o.veh.p.y); dl->AddCircleFilled(q, 4, ck::DIM); dl->AddText(ImVec2(q.x + 6, q.y - 6), ck::DIM, o.veh.s.name.c_str()); }
    // heading line and the vehicle
    double yw, pt, rl; v.heading_pitch_roll(yw, pt, rl); ImVec2 me = S(v.p.x, v.p.y); ImVec2 fw((float)std::cos(yw), -(float)std::sin(yw)), rt(-fw.y, fw.x);
    dl->AddLine(me, ImVec2(me.x + fw.x * side, me.y + fw.y * side), ck::alpha(ck::LINE, 0.35f), 1.0f);
    float L = 9; dl->AddTriangleFilled(ImVec2(me.x + fw.x * L * 1.4f, me.y + fw.y * L * 1.4f), ImVec2(me.x - fw.x * L + rt.x * L * 0.7f, me.y - fw.y * L + rt.y * L * 0.7f), ImVec2(me.x - fw.x * L - rt.x * L * 0.7f, me.y - fw.y * L - rt.y * L * 0.7f), ck::LINE);
    dl->AddTriangle(ImVec2(me.x + fw.x * L * 1.4f, me.y + fw.y * L * 1.4f), ImVec2(me.x - fw.x * L + rt.x * L * 0.7f, me.y - fw.y * L + rt.y * L * 0.7f), ImVec2(me.x - fw.x * L - rt.x * L * 0.7f, me.y - fw.y * L - rt.y * L * 0.7f), IM_COL32(0, 0, 0, 200), 1.5f);
    dl->PopClipRect();
    dl->AddRect(c0, c1, IM_COL32(255, 255, 255, 50), 6.f);
    // north arrow, current, scale bar
    dl->AddText(ImVec2(c1.x - 18, c0.y + 4), ck::TEXT, "N"); dl->AddTriangleFilled(ImVec2(c1.x - 14, c0.y + 22), ImVec2(c1.x - 19, c0.y + 32), ImVec2(c1.x - 9, c0.y + 32), ck::TEXT);
    double cvx, cvy; cur.at(std::max(0.0, -v.p.z), cvx, cvy); double cs = std::sqrt(cvx * cvx + cvy * cvy);
    if (cs > 0.01) { ImVec2 o(c0.x + 26, c0.y + 28), d((float)(cvx / cs), -(float)(cvy / cs)); dl->AddLine(ImVec2(o.x - d.x * 14, o.y - d.y * 14), ImVec2(o.x + d.x * 14, o.y + d.y * 14), ck::WARN, 2.f);
        ImVec2 tip(o.x + d.x * 14, o.y + d.y * 14), n(-d.y, d.x); dl->AddTriangleFilled(tip, ImVec2(tip.x - d.x * 7 + n.x * 4, tip.y - d.y * 7 + n.y * 4), ImVec2(tip.x - d.x * 7 - n.x * 4, tip.y - d.y * 7 - n.y * 4), ck::WARN);
        char t[40]; std::snprintf(t, sizeof t, "%.2f m/s", cs); dl->AddText(ImVec2(c0.x + 8, c0.y + 46), ck::WARN, t); }
    { char t[40]; std::snprintf(t, sizeof t, "%.0f m", step); ImVec2 a(c0.x + 10, c1.y - 14); dl->AddLine(a, ImVec2(a.x + step * ppm, a.y), ck::TEXT, 2.f); dl->AddText(ImVec2(a.x, a.y - 16), ck::TEXT, t);
      char d[64]; std::snprintf(d, sizeof d, "sea bed %.0f - %.0f m", M.dmin, M.dmax); dl->AddText(ImVec2(c1.x - ImGui::CalcTextSize(d).x - 8, c1.y - 18), ck::alpha(ck::TEXT, 0.8f), d); }
}

// ------------------------------------------------------------------------------------------------ thrusters and fins
inline void bar_row(ImDrawList* dl, ImVec2 a, float w, float h, float u, ImU32 pos, ImU32 neg) {
    dl->AddRectFilled(a, ImVec2(a.x + w, a.y + h), IM_COL32(255, 255, 255, 22), 3.f);
    float mid = a.x + w * 0.5f; u = std::max(-1.f, std::min(1.f, u));
    if (u >= 0) dl->AddRectFilled(ImVec2(mid, a.y), ImVec2(mid + u * w * 0.5f, a.y + h), pos, 3.f); else dl->AddRectFilled(ImVec2(mid + u * w * 0.5f, a.y), ImVec2(mid, a.y + h), neg, 3.f);
    dl->AddLine(ImVec2(mid, a.y - 2), ImVec2(mid, a.y + h + 2), IM_COL32(255, 255, 255, 120));
}
inline void view_thrusters(Unit& u) {
    Vehicle& v = u.veh; ImDrawList* dl = ImGui::GetWindowDrawList(); ImVec2 p = ImGui::GetCursorScreenPos(); float W = ImGui::GetContentRegionAvail().x, rowh = 24, lab = 34, txt = 190, bw = std::max(40.f, W - lab - txt);
    ImGui::TextColored(ImVec4(0.5f, 0.85f, 0.95f, 1), "THRUSTERS"); p.y += ImGui::GetTextLineHeightWithSpacing(); float y = p.y;
    for (size_t i = 0; i < v.rot.size(); ++i) {
        const Rotor& r = v.rot[i]; char t[96]; std::snprintf(t, sizeof t, "T%d", (int)i + 1); dl->AddText(ImVec2(p.x, y + 3), ck::TEXT, t);
        bar_row(dl, ImVec2(p.x + lab, y + 3), bw, rowh - 8, (float)(r.thrust / std::max(1.0, v.s.thr[i].max_thrust)), ck::ACCENT, IM_COL32(255, 140, 70, 255));
        std::snprintf(t, sizeof t, "%+6.1f N %+5.0f rpm %4.0f W", r.thrust, r.w * 9.5493, r.power); dl->AddText(ImVec2(p.x + lab + bw + 8, y + 3), r.wet < 0.9 ? ck::WARN : ck::DIM, t); y += rowh;
    }
    if (v.rot.empty()) { dl->AddText(ImVec2(p.x, y), ck::DIM, "(no thrusters)"); y += rowh; }
    y += 8; dl->AddText(ImVec2(p.x, y), IM_COL32(128, 218, 242, 255), "FINS"); y += ImGui::GetTextLineHeightWithSpacing();
    for (size_t i = 0; i < v.fang.size(); ++i) {
        const Fin& f = v.s.fins[i]; char t[96]; std::snprintf(t, sizeof t, "F%d", (int)i + 1); dl->AddText(ImVec2(p.x, y + 3), ck::TEXT, t);
        bar_row(dl, ImVec2(p.x + lab, y + 3), bw, rowh - 8, (float)(v.fang[i] / std::max(0.05, f.max_defl)), ck::ACCENT, IM_COL32(255, 140, 70, 255));
        std::snprintf(t, sizeof t, "%+5.1f deg  %s", v.fang[i] * 57.296, f.group == 1 ? "rudder" : (f.group == 2 ? "elevator" : "fixed")); dl->AddText(ImVec2(p.x + lab + bw + 8, y + 3), ck::DIM, t); y += rowh;
    }
    if (v.fang.empty()) { dl->AddText(ImVec2(p.x, y), ck::DIM, "(no fins)"); y += rowh; }
    y += 8; float tp = 0; for (auto& r : v.rot) tp += r.power;
    char t[128]; std::snprintf(t, sizeof t, "Thrusters %.0f W   hotel %.0f W   total %.0f W", tp, v.s.hotel_W, v.power_W); dl->AddText(ImVec2(p.x, y), ck::TEXT, t); y += 22;
    float bf = (float)(v.energy_Wh / std::max(1.0, v.s.battery_Wh)); dl->AddRectFilled(ImVec2(p.x, y), ImVec2(p.x + W, y + 10), IM_COL32(255, 255, 255, 22), 3.f);
    dl->AddRectFilled(ImVec2(p.x, y), ImVec2(p.x + W * bf, y + 10), bf > 0.3f ? ck::OK : (bf > 0.15f ? ck::WARN : ck::BAD), 3.f);
    ImGui::Dummy(ImVec2(W, y + 14 - ImGui::GetCursorScreenPos().y));
}

// ------------------------------------------------------------------------------------------------ sensors
struct SenseState {
    int sel = 0; double t_son = -1; std::vector<float> fan; std::vector<float> mb; int nfan_b = 64, nfan_r = 56; uint32_t seed = 1;
    float rnd() { seed = seed * 1664525u + 1013904223u; return (seed >> 8) / 16777216.0f; }
    float gauss() { float s = 0; for (int i = 0; i < 4; ++i) s += rnd(); return (s - 2.f) * 1.7320508f; }
};
// distance along a ray to the sea bed (or the surface); -1 if nothing within range. Also gives the surface normal there.
inline float ray_hit(const Terrain& T, V3 o, V3 d, float range, V3* n = nullptr, bool* surface = nullptr) {
    float step = std::max(0.15f, range / 140.f);
    for (float r = step; r <= range; r += step) {
        V3 q = o + d * r;
        if (q.z > 0) { if (surface) *surface = true; if (n) *n = {0, 0, -1}; return r; }
        float gz = T.ground_z((float)q.x, (float)q.y);
        if (q.z < gz) {
            if (surface) *surface = false;
            if (n) { float e = 0.5f; V3 g = {(T.ground_z((float)q.x - e, (float)q.y) - T.ground_z((float)q.x + e, (float)q.y)) / (2 * e), (T.ground_z((float)q.x, (float)q.y - e) - T.ground_z((float)q.x, (float)q.y + e)) / (2 * e), 1}; *n = unit(g); }
            return r;
        }
    }
    return -1;
}
inline V3 mount_dir(const Sensor& s) { double y = s.mount[3] * M_PI / 180, p = s.mount[4] * M_PI / 180; return {std::cos(p) * std::cos(y), std::cos(p) * std::sin(y), std::sin(p)}; }
inline V3 rot_z(V3 d, double a) { return {d.x * std::cos(a) - d.y * std::sin(a), d.x * std::sin(a) + d.y * std::cos(a), d.z}; }

inline void sonar_fan(SenseState& S, const Terrain& T, Vehicle& v, const Sensor& s, double now, bool fls) {
    double rng = fls ? s.get("range") : s.get("range"), hfov = (fls ? s.get("hfov") : s.get("swath")) * M_PI / 180; int nb = S.nfan_b, nr = S.nfan_r;
    if (now - S.t_son > 0.25 || (int)S.fan.size() != nb * nr) {
        S.t_son = now; S.fan.assign(nb * nr, 0.f); S.mb.assign(nb, -1.f);
        V3 o = v.p + qrot(v.q, {s.mount[0], s.mount[1], s.mount[2]}); V3 d0 = mount_dir(s); double vf = (fls ? s.get("vfov") : 4.0) * M_PI / 180;
        for (int b = 0; b < nb; ++b) {
            double az = (b / (double)(nb - 1) - 0.5) * hfov; V3 dbody;
            if (fls) dbody = rot_z(d0, -az); else { double th = az; dbody = {0, std::sin(th), -std::cos(th)}; }       // multibeam: fan across the track, looking down
            float best = -1; float inten = 0;
            for (int k = -1; k <= 1; ++k) {
                V3 dd = dbody; if (fls) { double e = k * vf * 0.4; double ce = std::cos(e), se = std::sin(e); V3 up = {0, 0, 1}; dd = unit(dbody * ce + up * se); }
                V3 dw = qrot(v.q, dd); V3 n; bool surf = false; float h = ray_hit(T, o, dw, (float)rng, &n, &surf);
                if (h > 0 && (best < 0 || h < best)) { best = h; inten = surf ? 0.45f : (float)std::max(0.05, std::fabs(dot(dw, n))); }
            }
            S.mb[b] = best;
            if (fls) {
                for (int r = 0; r < nr; ++r) { float rr = (r + 0.5f) / nr * (float)rng; float a = 0.04f * (0.5f + S.rnd());
                    if (best > 0) { if (std::fabs(rr - best) < rng / nr * 1.2f) a = 0.25f + 0.75f * inten * (0.7f + 0.3f * S.rnd()); else if (rr > best) a = 0.f; }
                    S.fan[b * nr + r] = a; }
            }
        }
    }
}
inline void view_sonar_fls(SenseState& S, const Sensor& s, float w, float h) {
    ImDrawList* dl = ImGui::GetWindowDrawList(); ImVec2 p = ImGui::GetCursorScreenPos(); double hfov = s.get("hfov") * M_PI / 180; int nb = S.nfan_b, nr = S.nfan_r;
    ImVec2 o(p.x + w * 0.5f, p.y + h - 4); float R = std::min(h - 10, (w * 0.5f - 4) / (float)std::max(0.2, std::sin(std::min(hfov * 0.5, M_PI * 0.5))));
    dl->AddRectFilled(p, ImVec2(p.x + w, p.y + h), IM_COL32(4, 8, 12, 230), 5.f);
    if ((int)S.fan.size() == nb * nr) for (int b = 0; b < nb; ++b) for (int r = 0; r < nr; ++r) {
        float a = S.fan[b * nr + r]; if (a < 0.02f) continue;
        double a0 = ((b - 0.5) / (nb - 1) - 0.5) * hfov, a1 = ((b + 0.5) / (nb - 1) - 0.5) * hfov; float r0 = (float)r / nr * R, r1 = (float)(r + 1) / nr * R;
        auto P = [&](double ang, float rr) { return ImVec2(o.x + (float)std::sin(ang) * rr, o.y - (float)std::cos(ang) * rr); };
        int g = (int)(40 + 215 * std::min(1.f, a)); dl->AddQuadFilled(P(a0, r0), P(a1, r0), P(a1, r1), P(a0, r1), IM_COL32(g, (int)(g * 0.8f), (int)(g * 0.25f), 255));
    }
    for (int k = 1; k <= 3; ++k) { dl->PathArcTo(o, R * k / 3.f, -(float)M_PI / 2 - (float)hfov / 2, -(float)M_PI / 2 + (float)hfov / 2, 32); dl->PathStroke(IM_COL32(255, 255, 255, 40), 0, 1.f); char t[24]; std::snprintf(t, sizeof t, "%.0f m", s.get("range") * k / 3); dl->AddText(ImVec2(o.x + 3, o.y - R * k / 3.f), ck::DIM, t); }
    ImGui::Dummy(ImVec2(w, h));
}
inline void view_multibeam(SenseState& S, const Sensor& s, float w, float h) {
    ImDrawList* dl = ImGui::GetWindowDrawList(); ImVec2 p = ImGui::GetCursorScreenPos(); int nb = S.nfan_b; double sw = s.get("swath") * M_PI / 180, rng = s.get("range");
    dl->AddRectFilled(p, ImVec2(p.x + w, p.y + h), IM_COL32(4, 8, 12, 230), 5.f);
    ImVec2 o(p.x + w * 0.5f, p.y + 6); float sc = (h - 14) / (float)rng; dl->AddCircleFilled(o, 4, ck::LINE);
    static std::vector<ImVec2> pts; pts.clear();
    for (int b = 0; b < nb && b < (int)S.mb.size(); ++b) { float r = S.mb[b]; if (r < 0) continue; double a = (b / (double)(nb - 1) - 0.5) * sw; pts.push_back(ImVec2(o.x + (float)std::sin(a) * r * sc, o.y + (float)std::cos(a) * r * sc)); }
    if (pts.size() > 1) dl->AddPolyline(pts.data(), (int)pts.size(), ck::WARN, 0, 2.f);
    for (auto& q : pts) dl->AddCircleFilled(q, 1.8f, IM_COL32(255, 220, 120, 255));
    dl->AddText(ImVec2(p.x + 6, p.y + h - 18), ck::DIM, "cross-track profile of the sea bed"); ImGui::Dummy(ImVec2(w, h));
}

inline void view_sensors(SenseState& S, Unit& u, Hist& H, const std::vector<Sensor>& list, const Terrain& T, const Current& cur, const Water& water, double now, Fleet& F) {
    Vehicle& v = u.veh;
    if (list.empty()) { ImGui::TextDisabled("No sensors were configured for this vehicle (Setup > Sensors)."); return; }
    double yw, pt, rl; v.heading_pitch_roll(yw, pt, rl); double hdg = std::fmod(90.0 - yw * 57.29578 + 720.0, 360.0);
    double depth = -v.p.z, alt = v.p.z - T.ground_z((float)v.p.x, (float)v.p.y); double rho, pres; water.state_at(std::max(0.0, depth), rho, pres);
    float W = ImGui::GetContentRegionAvail().x;
    for (int i = 0; i < (int)list.size(); ++i) {
        const Sensor& s = list[i]; if (!s.enabled) continue; const SensorType& t = sensor_types()[s.type];
        ImGui::PushID(i); ImGui::PushStyleColor(ImGuiCol_Header, ImVec4(0.12f, 0.30f, 0.38f, 0.8f));
        char head[160]; std::snprintf(head, sizeof head, "%s   (%s, %.0f Hz)###s%d", s.name.c_str(), t.name, s.rate, i);
        bool open = ImGui::CollapsingHeader(head, (s.type == S_FLS || s.type == S_MBES) ? ImGuiTreeNodeFlags_DefaultOpen : 0); ImGui::PopStyleColor();
        if (open) {
            double ns = std::sqrt(s.rate);
            switch (s.type) {
                case S_IMU: {
                    double ax = H.acc_b.x, ay = H.acc_b.y, az = H.acc_b.z; V3 g = qrot_inv(v.q, {0, 0, 9.80665});
                    ImGui::Text("accel  x %+6.2f  y %+6.2f  z %+6.2f  m/s2", ax + g.x + s.get("accel_bias") + S.gauss() * s.get("accel_nd") * ns, ay + g.y + S.gauss() * s.get("accel_nd") * ns, az + g.z + S.gauss() * s.get("accel_nd") * ns);
                    ImGui::Text("gyro   p %+6.1f  q %+6.1f  r %+6.1f  deg/s", (v.w.x + s.get("gyro_bias") + S.gauss() * s.get("gyro_nd") * ns) * 57.29578, (v.w.y + S.gauss() * s.get("gyro_nd") * ns) * 57.29578, (v.w.z + S.gauss() * s.get("gyro_nd") * ns) * 57.29578);
                    break; }
                case S_COMPASS: { double hd = std::fmod(hdg + s.get("decl") + S.gauss() * s.get("noise") * 0.3 + s.get("hard_iron") * 0.4 + 720.0, 360.0); ImGui::Text("heading %05.1f deg magnetic   field %.0f uT", hd, s.get("field")); break; }
                case S_DEPTH: {
                    double rd = s.get("max_depth"); double meas = std::round((depth + s.get("bias") + S.gauss() * s.get("noise")) / s.get("resolution")) * s.get("resolution");
                    if (depth > rd) ImGui::TextColored(ImVec4(1, 0.35f, 0.3f, 1), "over-range (rated %.0f m)", rd); else ImGui::Text("depth %.3f m   pressure %.3f bar", meas, (pres + 0.0) / 1e5);
                    break; }
                case S_DVL: {
                    double mn = s.get("min_alt"), mx = s.get("max_alt"); bool lock = alt > mn && alt < mx && depth > 0.2; double nz = s.get("noise") * 1e-3;
                    if (lock && S.rnd() > s.get("dropouts") / 100.0) ImGui::Text("BOTTOM LOCK   u %+5.2f  v %+5.2f  w %+5.2f m/s   alt %.1f m", v.v.x + S.gauss() * nz, v.v.y + S.gauss() * nz, v.v.z + S.gauss() * nz, alt);
                    else if (s.get("water_track") > 0.5 && depth > 0.2) { double cx, cy; cur.at(depth, cx, cy); V3 cb = qrot_inv(v.q, {cx, cy, 0}); ImGui::TextColored(ImVec4(1, 0.8f, 0.3f, 1), "WATER TRACK   u %+5.2f  v %+5.2f  w %+5.2f m/s", v.v.x - cb.x, v.v.y - cb.y, v.v.z - cb.z); }
                    else ImGui::TextColored(ImVec4(1, 0.4f, 0.35f, 1), "NO LOCK (altitude %.1f m, range %.1f - %.0f m)", alt, mn, mx);
                    break; }
                case S_GPS: { bool fix = (v.p.z + 0.5 * v.s.height + s.get("ant_h")) > 0.0 && depth < 0.3; if (fix) ImGui::TextColored(ImVec4(0.4f, 1, 0.6f, 1), "FIX   x %.1f  y %.1f  (+-%.1f m)", v.p.x + S.gauss() * s.get("noise_h"), v.p.y + S.gauss() * s.get("noise_h"), s.get("noise_h")); else ImGui::TextColored(ImVec4(1, 0.4f, 0.35f, 1), "NO FIX - the antenna is under water"); break; }
                case S_FLS: ImGui::TextDisabled("Preview from the sea-bed model (no fish, no clutter yet)."); sonar_fan(S, T, v, s, now, true); view_sonar_fls(S, s, W - 16, std::min(190.f, (W - 16) * 0.6f)); break;
                case S_MBES: sonar_fan(S, T, v, s, now, false); view_multibeam(S, s, W - 16, 120); break;
                case S_SSS: ImGui::TextDisabled("Side-scan waterfall comes with the sensor-data step. Range per side %.0f m, tilt %.0f deg.", s.get("range"), s.get("tilt")); break;
                case S_CAMERA: ImGui::TextDisabled("%.0fx%.0f @ %.0f Hz, hfov %.0f deg", s.get("width"), s.get("height"), s.rate, s.get("hfov")); if (ImGui::Button("View through the onboard camera")) F.cam_mode = 2; ImGui::SameLine(); if (ImGui::Button("Back to chase view")) F.cam_mode = 1; break;
                case S_LIDAR: ImGui::TextColored(ImVec4(1, 0.8f, 0.3f, 1), depth > 0.5 ? "No returns: light does not travel under water." : "Surfaced: scanning (data comes in a later step)."); break;
            }
        }
        ImGui::PopID();
    }
}
