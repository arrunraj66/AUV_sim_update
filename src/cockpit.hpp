// cockpit.hpp - the "Cockpit": a full-screen 3D view with HUD widgets you place yourself.
//   Toolbar on top (Setup / Edit layout / Layouts / Widgets), glass widgets on the 3D view.
//   Edit layout = every widget can be dragged and resized; the layout is saved as JSON (config/layout/*.json).
#pragma once
#include "cockpit_views.hpp"
#include <cstdlib>

struct CkCtx { Fleet* fleet; const Terrain* T; const Current* cur; const Water* water; std::map<int, std::vector<Sensor>>* usens; bool* lamp_on; bool* open_designer = nullptr; };

struct Cockpit {
    enum { W_HEADING, W_DEPTH, W_QUICK, W_ATT, W_STATUS, W_CONTROLS, W_MAP, W_PLOTS, W_THRUST, W_SENSORS, W_TUNING, W_COUNT };
    static const char* wname(int i) { static const char* n[W_COUNT] = {"Heading", "Depth", "Key numbers", "Attitude", "Status and alerts", "Controls", "Map", "Live plots", "Thrusters and fins", "Sensors", "Physics tuning"}; return n[i]; }
    static const char* wkey(int i) { static const char* n[W_COUNT] = {"heading", "depth", "quick", "attitude", "status", "controls", "map", "plots", "thrusters", "sensors", "tuning"}; return n[i]; }
    struct Slot { bool vis = false; float x = 0, y = 0, w = 0.2f, h = 0.2f; bool apply = true; };
    Slot L[W_COUNT];
    bool edit = false, want_setup = false, show_live = false; float opacity = 0.50f; std::string dir = "config/layout", msg, cur_name = "Pilot"; char name_buf[64] = "";
    std::vector<std::string> saved; bool refresh = true;
    ImFont *f_big = nullptr, *f_med = nullptr; ImVec2 last_disp{0, 0};
    std::map<int, Hist> hist; PlotState plots; MapState map; SenseState sense;
    std::deque<std::string> log; std::map<std::string, int> active_alerts; double clock = 0;

    // ---------------------------------------------------------------------------------------------- fonts and theme
    void load_fonts() {                                    // call once, right after ImGui::CreateContext()
        ImGuiIO& io = ImGui::GetIO();
        const char* reg[] = {"/usr/share/fonts/truetype/ubuntu/Ubuntu-R.ttf", "/usr/share/fonts/truetype/noto/NotoSans-Regular.ttf", "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", "/usr/share/fonts/truetype/liberation/LiberationSans-Regular.ttf", nullptr};
        const char* bold[] = {"/usr/share/fonts/truetype/ubuntu/Ubuntu-M.ttf", "/usr/share/fonts/truetype/ubuntu/Ubuntu-B.ttf", "/usr/share/fonts/truetype/noto/NotoSans-Bold.ttf", "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", "/usr/share/fonts/truetype/liberation/LiberationSans-Bold.ttf", nullptr};
        ImFont *r = nullptr, *b = nullptr;
        for (int i = 0; reg[i] && !r; ++i) if (std::filesystem::exists(reg[i])) r = io.Fonts->AddFontFromFileTTF(reg[i], 16.5f);
        for (int i = 0; bold[i] && !b; ++i) if (std::filesystem::exists(bold[i])) b = io.Fonts->AddFontFromFileTTF(bold[i], 34.f);
        if (!r) r = io.Fonts->AddFontDefault();
        f_med = r; f_big = b ? b : r; io.FontDefault = r;
    }
    static void apply_theme() {
        ImGuiStyle& s = ImGui::GetStyle(); ImVec4* c = s.Colors;
        s.WindowRounding = 8; s.ChildRounding = 6; s.FrameRounding = 5; s.PopupRounding = 6; s.GrabRounding = 5; s.TabRounding = 5; s.ScrollbarRounding = 8;
        s.WindowPadding = ImVec2(10, 8); s.FramePadding = ImVec2(8, 4); s.ItemSpacing = ImVec2(8, 6); s.WindowBorderSize = 1; s.FrameBorderSize = 0; s.GrabMinSize = 12; s.SeparatorTextBorderSize = 1;
        auto C = [](float r, float g, float b, float a = 1) { return ImVec4(r, g, b, a); };
        ImVec4 acc = C(0.16f, 0.78f, 0.88f), accD = C(0.10f, 0.42f, 0.52f), bg = C(0.045f, 0.075f, 0.105f, 0.94f), fr = C(0.10f, 0.15f, 0.20f, 1);
        c[ImGuiCol_Text] = C(0.89f, 0.93f, 0.96f); c[ImGuiCol_TextDisabled] = C(0.50f, 0.58f, 0.65f); c[ImGuiCol_WindowBg] = bg; c[ImGuiCol_ChildBg] = C(0.06f, 0.09f, 0.12f, 0.9f); c[ImGuiCol_PopupBg] = C(0.06f, 0.09f, 0.12f, 0.98f);
        c[ImGuiCol_Border] = C(0.30f, 0.50f, 0.60f, 0.35f); c[ImGuiCol_FrameBg] = fr; c[ImGuiCol_FrameBgHovered] = C(0.14f, 0.22f, 0.28f); c[ImGuiCol_FrameBgActive] = C(0.16f, 0.27f, 0.34f);
        c[ImGuiCol_TitleBg] = C(0.05f, 0.10f, 0.14f); c[ImGuiCol_TitleBgActive] = C(0.08f, 0.22f, 0.28f); c[ImGuiCol_TitleBgCollapsed] = C(0.05f, 0.10f, 0.14f, 0.7f);
        c[ImGuiCol_Button] = C(0.12f, 0.24f, 0.30f); c[ImGuiCol_ButtonHovered] = accD; c[ImGuiCol_ButtonActive] = acc;
        c[ImGuiCol_Header] = C(0.10f, 0.28f, 0.35f, 0.8f); c[ImGuiCol_HeaderHovered] = accD; c[ImGuiCol_HeaderActive] = acc;
        c[ImGuiCol_SliderGrab] = acc; c[ImGuiCol_SliderGrabActive] = C(0.5f, 0.95f, 1.0f); c[ImGuiCol_CheckMark] = acc;
        c[ImGuiCol_Tab] = C(0.10f, 0.20f, 0.26f); c[ImGuiCol_TabHovered] = accD; c[ImGuiCol_TabActive] = C(0.12f, 0.45f, 0.55f);
        c[ImGuiCol_Separator] = C(0.30f, 0.50f, 0.60f, 0.35f); c[ImGuiCol_ResizeGrip] = C(0.16f, 0.78f, 0.88f, 0.25f); c[ImGuiCol_ResizeGripHovered] = accD; c[ImGuiCol_ResizeGripActive] = acc;
        c[ImGuiCol_ScrollbarBg] = C(0, 0, 0, 0.2f); c[ImGuiCol_ScrollbarGrab] = C(0.2f, 0.35f, 0.42f); c[ImGuiCol_PlotLines] = acc; c[ImGuiCol_PlotHistogram] = acc;
    }

    // ---------------------------------------------------------------------------------------------- layouts
    struct Pre { bool vis; float x, y, w, h; };
    static const char* preset_name(int i) { static const char* n[4] = {"Pilot", "Minimal HUD", "Engineering", "Inspection"}; return n[i]; }
    void apply_preset(int p) {
        static const Pre T[4][W_COUNT] = {
            // heading                    depth                        quick                       attitude                    status                       controls                     map                          plots                        thrusters                    sensors                      tuning
            {{1, .33f, .055f, .34f, .10f}, {1, .915f, .15f, .075f, .50f}, {1, .012f, .07f, .17f, .38f}, {1, .405f, .72f, .19f, .27f}, {1, .012f, .46f, .17f, .20f}, {1, .012f, .66f, .24f, .33f}, {1, .745f, .675f, .245f, .315f}, {0, .25f, .5f, .4f, .4f}, {0, .6f, .1f, .3f, .5f}, {0, .6f, .1f, .3f, .5f}, {0, .3f, .1f, .25f, .4f}},
            {{1, .33f, .055f, .34f, .10f}, {1, .915f, .15f, .075f, .50f}, {1, .012f, .07f, .15f, .34f}, {1, .43f, .76f, .14f, .21f}, {0, 0, 0, .1f, .1f},         {0, 0, 0, .1f, .1f},         {0, 0, 0, .1f, .1f},         {0, .25f, .5f, .4f, .4f}, {0, .6f, .1f, .3f, .5f}, {0, .6f, .1f, .3f, .5f}, {0, .3f, .1f, .25f, .4f}},
            {{1, .26f, .055f, .30f, .10f}, {1, .915f, .07f, .075f, .40f}, {1, .012f, .07f, .15f, .33f}, {1, .012f, .42f, .14f, .21f}, {1, .012f, .64f, .15f, .34f}, {1, .17f, .64f, .22f, .34f}, {1, .745f, .5f, .245f, .33f}, {1, .17f, .17f, .30f, .45f}, {1, .485f, .17f, .24f, .34f}, {1, .40f, .64f, .34f, .34f}, {1, .745f, .07f, .165f, .4f}},
            {{1, .26f, .055f, .30f, .10f}, {1, .012f, .42f, .075f, .40f}, {1, .012f, .07f, .15f, .33f}, {1, .09f, .70f, .13f, .27f}, {1, .23f, .70f, .17f, .27f}, {0, .4f, .1f, .2f, .3f},   {1, .745f, .675f, .245f, .315f}, {0, .25f, .5f, .4f, .4f}, {0, .6f, .1f, .3f, .5f}, {1, .62f, .06f, .37f, .60f}, {0, .3f, .1f, .25f, .4f}}};
        for (int i = 0; i < W_COUNT; ++i) { Slot& s = L[i]; s.vis = T[p][i].vis; s.x = T[p][i].x; s.y = T[p][i].y; s.w = T[p][i].w; s.h = T[p][i].h; s.apply = true; }
        cur_name = preset_name(p);
    }
    Json layout_json() const {
        Json j = Json::object(); j["opacity"] = Json::number(opacity); Json w = Json::object();
        for (int i = 0; i < W_COUNT; ++i) { Json e = Json::object(); e["visible"] = Json::boolean(L[i].vis); e["x"] = Json::number(L[i].x); e["y"] = Json::number(L[i].y); e["w"] = Json::number(L[i].w); e["h"] = Json::number(L[i].h); w[wkey(i)] = e; }
        j["widgets"] = w; return j;
    }
    void layout_load(const Json& j) {
        opacity = (float)std::max(0.1, std::min(1.0, j.num_or("opacity", opacity)));
        if (const Json* w = j.find("widgets")) for (int i = 0; i < W_COUNT; ++i) if (const Json* e = w->find(wkey(i))) {
            Slot& s = L[i]; s.vis = e->bool_or("visible", s.vis); s.x = (float)std::max(-0.2, std::min(1.0, e->num_or("x", s.x))); s.y = (float)std::max(0.0, std::min(1.0, e->num_or("y", s.y)));
            s.w = (float)std::max(0.04, std::min(1.0, e->num_or("w", s.w))); s.h = (float)std::max(0.04, std::min(1.0, e->num_or("h", s.h))); s.apply = true;
        }
    }
    void init(const std::string& d) {
        dir = d + "/layout"; apply_preset(0); Json j; std::string err;
        if (load_json_file(dir + "/_last.json", j, err)) { layout_load(j); cur_name = j.str_or("name", "Last used"); }
    }
    void save_last() { Json j = layout_json(); j["name"] = Json::string(cur_name); std::string err; save_json_file(dir + "/_last.json", j, err); }

    // ---------------------------------------------------------------------------------------------- a widget window
    struct Rc { ImVec2 a, b; float w() const { return b.x - a.x; } float h() const { return b.y - a.y; } float cx() const { return (a.x + b.x) * 0.5f; } float cy() const { return (a.y + b.y) * 0.5f; } };
    Rc rc() const { ImVec2 p = ImGui::GetWindowPos(), s = ImGui::GetWindowSize(); float t = edit ? ImGui::GetFrameHeight() : 0; return {{p.x + 8, p.y + 8 + t}, {p.x + s.x - 8, p.y + s.y - 8}}; }
    bool open(int id, bool interactive = false) {
        Slot& s = L[id]; if (!s.vis) return false; ImVec2 D = ImGui::GetIO().DisplaySize;
        ImGuiWindowFlags f = ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoFocusOnAppearing | ImGuiWindowFlags_NoNav;
        if (!interactive) f |= ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse;
        if (!edit) { f |= ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoBringToFrontOnFocus; if (!interactive) f |= ImGuiWindowFlags_NoInputs; }
        if (!edit || s.apply) { ImGui::SetNextWindowPos(ImVec2(s.x * D.x, s.y * D.y), ImGuiCond_Always); ImGui::SetNextWindowSize(ImVec2(s.w * D.x, s.h * D.y), ImGuiCond_Always); if (edit) s.apply = false; }
        ImGui::SetNextWindowSizeConstraints(ImVec2(80, 50), ImVec2(D.x, D.y));
        ImGui::SetNextWindowBgAlpha(opacity);
        ImGui::PushStyleColor(ImGuiCol_Border, edit ? ImVec4(0.2f, 0.85f, 0.95f, 0.8f) : ImVec4(0.45f, 0.7f, 0.8f, 0.22f));
        char title[96]; std::snprintf(title, sizeof title, "%s###ck%d", wname(id), id);
        ImGui::Begin(title, nullptr, f); return true;
    }
    void close(int id) {
        if (edit) { ImVec2 D = ImGui::GetIO().DisplaySize, p = ImGui::GetWindowPos(), z = ImGui::GetWindowSize(); Slot& s = L[id]; if (!s.apply) { s.x = p.x / D.x; s.y = p.y / D.y; s.w = z.x / D.x; s.h = z.y / D.y; } }
        ImGui::End(); ImGui::PopStyleColor();
    }
    static void text_c(ImDrawList* dl, ImFont* f, float size, float cx, float y, ImU32 col, const char* t) { ImVec2 s = f->CalcTextSizeA(size, FLT_MAX, 0, t); dl->AddText(f, size, ImVec2(cx - s.x * 0.5f, y), col, t); }

    // ---------------------------------------------------------------------------------------------- alerts
    struct Alert { int lvl; std::string text; };
    std::vector<Alert> alerts(Unit& u, const Terrain& T, const std::vector<Sensor>* sens) {
        std::vector<Alert> a; Vehicle& v = u.veh; char b[128]; double depth = -v.p.z, alt = v.p.z - T.ground_z((float)v.p.x, (float)v.p.y), pct = 100.0 * v.energy_Wh / std::max(1.0, v.s.battery_Wh);
        if (v.flat) a.push_back({2, "Battery flat: thrusters off"}); else if (pct < 15) { std::snprintf(b, sizeof b, "Battery critical (%.0f %%)", pct); a.push_back({2, b}); } else if (pct < 30) { std::snprintf(b, sizeof b, "Battery low (%.0f %%)", pct); a.push_back({1, b}); }
        if (depth > v.s.rated_depth) a.push_back({2, "Deeper than the rated depth"}); else if (depth > 0.9 * v.s.rated_depth) a.push_back({1, "Close to the rated depth"});
        if (v.on_bottom) a.push_back({1, "On the sea bed"}); else if (alt < 1.0 && depth > 0.5) { std::snprintf(b, sizeof b, "Sea bed close: altitude %.1f m", alt); a.push_back({1, b}); }
        if (v.sub < 0.05) a.push_back({1, "Out of the water"}); else if (v.sub < 0.99) a.push_back({0, "At the surface"});
        for (auto& r : v.rot) if (r.wet < 0.9 && std::fabs(r.thrust) > 0.01) { a.push_back({1, "A thruster is out of the water"}); break; }
        if (sens) for (auto& s : *sens) if (s.enabled && s.type == S_DVL && depth > 0.3 && (alt > s.get("max_alt") || alt < s.get("min_alt"))) { a.push_back({1, "DVL: no bottom lock"}); break; }
        if (sens) for (auto& s : *sens) if (s.enabled && s.type == S_GPS && depth > 0.3) { a.push_back({0, "GPS: no fix (submerged)"}); break; }
        return a;
    }

    // ---------------------------------------------------------------------------------------------- update (history and alerts log)
    void update(double dt, CkCtx& cx) {
        clock += dt;
        for (Unit& u : cx.fleet->units) {
            Hist& H = hist[u.id]; Vehicle& v = u.veh; H.t += dt; H.acc += dt; H.tacc += dt;
            if (H.prev_t >= 0 && dt > 1e-4) { V3 dv = (v.v - H.prev_v) * (1.0 / dt); V3 a = dv + cross(v.w, v.v); double k = std::min(1.0, dt * 8); H.acc_b = H.acc_b + (a - H.acc_b) * k; }
            H.prev_v = v.v; H.prev_t = H.t;
            if (H.acc >= 0.2) { H.acc = 0; H.s.push_back({(float)H.t, (float)-v.p.z, (float)(v.p.z - cx.T->ground_z((float)v.p.x, (float)v.p.y)), (float)len(v.v), (float)v.power_W, (float)(100.0 * v.energy_Wh / std::max(1.0, v.s.battery_Wh))}); if (H.s.size() > 3100) H.s.pop_front(); }
            if (H.tacc >= 0.5) { H.tacc = 0; H.trail.push_back({(float)v.p.x, (float)v.p.y}); if (H.trail.size() > 1500) H.trail.erase(H.trail.begin()); }
        }
        Unit* u = cx.fleet->act(); if (!u) return;
        auto it = cx.usens->find(u->id); auto al = alerts(*u, *cx.T, it == cx.usens->end() ? nullptr : &it->second);
        std::map<std::string, int> now; for (auto& a : al) now[a.text] = a.lvl;
        char t[160];
        for (auto& kv : now) if (!active_alerts.count(kv.first)) { std::snprintf(t, sizeof t, "%02d:%02d  + %s", (int)clock / 60, (int)clock % 60, kv.first.c_str()); log.push_back(t); }
        for (auto& kv : active_alerts) if (!now.count(kv.first)) { std::snprintf(t, sizeof t, "%02d:%02d  - %s", (int)clock / 60, (int)clock % 60, kv.first.c_str()); log.push_back(t); }
        active_alerts = now; while (log.size() > 40) log.pop_front();
    }

    // ---------------------------------------------------------------------------------------------- the widgets
    void w_heading(double hdg) {
        Rc r = rc(); ImDrawList* dl = ImGui::GetWindowDrawList(); float span = 80, ppd = r.w() / span, base = r.a.y + r.h() * 0.50f;
        dl->PushClipRect(r.a, r.b, true); dl->AddLine(ImVec2(r.a.x, base), ImVec2(r.b.x, base), ck::alpha(ck::LINE, 0.8f), 1.5f);
        for (int d = (int)std::floor(hdg - span / 2); d <= (int)std::ceil(hdg + span / 2); ++d) {
            if (d % 5) continue; int dd = ((d % 360) + 360) % 360; bool maj = dd % 10 == 0; float x = r.cx() + (float)(d - hdg) * ppd, tl = r.h() * (maj ? 0.15f : 0.08f);
            dl->AddLine(ImVec2(x, base), ImVec2(x, base + tl), ck::LINE, maj ? 2.f : 1.f);
            if (dd % 30 == 0) { char t[8]; if (dd % 90 == 0) std::snprintf(t, sizeof t, "%s", dd == 0 ? "N" : dd == 90 ? "E" : dd == 180 ? "S" : "W"); else std::snprintf(t, sizeof t, "%03d", dd); text_c(dl, f_med, 16, x, base + tl + 1, dd % 90 == 0 ? ck::WARN : ck::TEXT, t); }
        }
        dl->PopClipRect(); float bw = 78, bh = r.h() * 0.5f; ImVec2 a(r.cx() - bw / 2, r.a.y), b(r.cx() + bw / 2, r.a.y + bh);
        dl->AddRectFilled(a, b, IM_COL32(8, 20, 28, 230), 5.f); dl->AddRect(a, b, ck::LINE, 5.f, 0, 1.5f); char t[16]; std::snprintf(t, sizeof t, "%03.0f", hdg); text_c(dl, f_big, bh * 0.82f, r.cx(), r.a.y + bh * 0.07f, ck::TEXT, t);
        dl->AddTriangleFilled(ImVec2(r.cx() - 6, b.y + 1), ImVec2(r.cx() + 6, b.y + 1), ImVec2(r.cx(), base), ck::LINE);
    }
    void w_depth(Unit& u, double depth, double alt) {
        Rc r = rc(); ImDrawList* dl = ImGui::GetWindowDrawList(); Vehicle& v = u.veh; double rated = v.s.rated_depth;
        double span = rated <= 150 ? 16 : rated <= 600 ? 60 : rated <= 2000 ? 200 : 800; float ppm = (float)(r.h() / span), cy = r.cy();
        dl->PushClipRect(r.a, r.b, true); float tx = r.b.x - 24;
        auto Y = [&](double z) { return cy + (float)(z - depth) * ppm; };
        dl->AddLine(ImVec2(tx, r.a.y), ImVec2(tx, r.b.y), ck::alpha(ck::LINE, 0.8f), 1.5f);
        double step = span / 8; double nice[] = {0.5, 1, 2, 5, 10, 20, 50, 100, 200, 500}; for (double n : nice) if (n >= step) { step = n; break; }
        for (double z = std::floor((depth - span / 2) / step) * step; z < depth + span / 2; z += step) {
            float y = Y(z); dl->AddLine(ImVec2(tx - 8, y), ImVec2(tx, y), ck::LINE, 1.5f); char t[16]; std::snprintf(t, sizeof t, "%.0f", z); dl->AddText(ImVec2(tx - 12 - ImGui::CalcTextSize(t).x, y - 8), ck::TEXT, t);
        }
        float ys = Y(0); if (ys > r.a.y && ys < r.b.y) { dl->AddRectFilled(ImVec2(r.a.x, r.a.y), ImVec2(tx, std::min(ys, r.b.y)), IM_COL32(120, 190, 235, 40)); dl->AddLine(ImVec2(r.a.x, ys), ImVec2(tx + 6, ys), IM_COL32(150, 210, 255, 255), 2.f); dl->AddText(ImVec2(r.a.x + 2, ys + 2), IM_COL32(150, 210, 255, 255), "SURFACE"); }
        float yb = Y(depth + alt); if (yb < r.b.y) { dl->AddRectFilled(ImVec2(r.a.x, std::max(yb, r.a.y)), ImVec2(tx, r.b.y), IM_COL32(160, 120, 70, 90)); if (yb > r.a.y) { dl->AddLine(ImVec2(r.a.x, yb), ImVec2(tx + 6, yb), IM_COL32(220, 170, 100, 255), 2.f); dl->AddText(ImVec2(r.a.x + 2, yb - 16), IM_COL32(220, 170, 100, 255), "SEA BED"); } }
        float yr = Y(rated); if (yr > r.a.y && yr < r.b.y) { dl->AddLine(ImVec2(r.a.x, yr), ImVec2(tx + 6, yr), ck::BAD, 2.f); dl->AddText(ImVec2(r.a.x + 2, yr - 16), ck::BAD, "RATED"); }
        dl->PopClipRect();
        ImVec2 a(r.a.x, cy - 15), b(tx + 2, cy + 15); dl->AddRectFilled(a, b, IM_COL32(8, 20, 28, 235), 4.f); dl->AddRect(a, b, ck::LINE, 4.f, 0, 1.5f); dl->AddTriangleFilled(ImVec2(b.x, cy - 8), ImVec2(b.x, cy + 8), ImVec2(b.x + 7, cy), ck::LINE);
        char t[24]; std::snprintf(t, sizeof t, "%.1f m", depth); text_c(dl, f_med, 20, (a.x + b.x) * 0.5f, cy - 11, depth > rated ? ck::BAD : ck::TEXT, t);
        std::snprintf(t, sizeof t, "ALT %.1f", alt); text_c(dl, f_med, 15, (r.a.x + tx) * 0.5f, r.b.y - 19, alt < 1.5 ? ck::WARN : ck::DIM, t);
    }
    void cell(ImDrawList* dl, Rc r, int col, int row, int ncol, int nrow, const char* label, const char* value, const char* unit, ImU32 vc) {
        float cw = r.w() / ncol, ch = r.h() / nrow; ImVec2 a(r.a.x + col * cw, r.a.y + row * ch); float fs = std::min(std::min(34.f, ch * 0.5f), cw * 0.30f);
        dl->AddText(ImVec2(a.x + 4, a.y + 1), ck::DIM, label); dl->AddText(f_big, fs, ImVec2(a.x + 4, a.y + ch * 0.28f), vc, value);
        if (unit[0]) dl->AddText(ImVec2(a.x + 8 + f_big->CalcTextSizeA(fs, FLT_MAX, 0, value).x, a.y + ch * 0.28f + fs * 0.45f), ck::DIM, unit);
    }
    void w_quick(Unit& u, double depth, double alt) {
        Rc r = rc(); ImDrawList* dl = ImGui::GetWindowDrawList(); Vehicle& v = u.veh; char b[32];
        double pct = 100.0 * v.energy_Wh / std::max(1.0, v.s.battery_Wh), hrs = v.power_W > 1 ? v.energy_Wh / v.power_W : 99;
        std::snprintf(b, sizeof b, "%.1f", depth); cell(dl, r, 0, 0, 2, 3, "DEPTH", b, "m", depth > v.s.rated_depth ? ck::BAD : ck::TEXT);
        std::snprintf(b, sizeof b, "%.1f", alt); cell(dl, r, 1, 0, 2, 3, "ALTITUDE", b, "m", alt < 1.5 ? ck::WARN : ck::TEXT);
        std::snprintf(b, sizeof b, "%.2f", len(v.v)); cell(dl, r, 0, 1, 2, 3, "SPEED", b, "m/s", ck::TEXT);
        std::snprintf(b, sizeof b, "%.1f", len(v.v) * 1.944); cell(dl, r, 1, 1, 2, 3, "SPEED", b, "kn", ck::TEXT);
        std::snprintf(b, sizeof b, "%.0f", pct); cell(dl, r, 0, 2, 2, 3, "BATTERY", b, "%", pct < 15 ? ck::BAD : pct < 30 ? ck::WARN : ck::OK);
        if (hrs < 90) std::snprintf(b, sizeof b, "%.1f", hrs); else std::snprintf(b, sizeof b, "--"); cell(dl, r, 1, 2, 2, 3, "ENDURANCE", b, "h", ck::TEXT);
    }
    void w_attitude(double pt, double rl) {
        Rc r = rc(); ImDrawList* dl = ImGui::GetWindowDrawList(); float R = std::min(r.w(), r.h()) * 0.5f - 14; if (R < 20) return; ImVec2 c(r.cx(), r.cy() + 4);
        float ppd = R / 32.f, off = (float)(pt * 57.29578) * ppd; double th = -rl, ct = std::cos(th), st = std::sin(th);
        ImVec2 n((float)-st, (float)ct); float cc = std::max(-R * 0.98f, std::min(R * 0.98f, off)); double ang0 = std::atan2(n.y, n.x), half = std::acos(cc / R);
        auto arc = [&](double a0, double a1, ImU32 col) { for (int i = 0; i <= 48; ++i) { double a = a0 + (a1 - a0) * i / 48; dl->PathLineTo(ImVec2(c.x + (float)std::cos(a) * R, c.y + (float)std::sin(a) * R)); } dl->PathFillConvex(col); };
        arc(ang0 - half, ang0 + half, IM_COL32(120, 80, 40, 150)); arc(ang0 + half, ang0 + 2 * M_PI - half, IM_COL32(60, 140, 200, 120));
        dl->AddCircle(c, R, ck::alpha(ck::LINE, 0.9f), 64, 2.f);
        ImVec2 h(-n.y, n.x); dl->AddLine(ImVec2(c.x + n.x * off - h.x * R, c.y + n.y * off - h.y * R), ImVec2(c.x + n.x * off + h.x * R, c.y + n.y * off + h.y * R), IM_COL32(255, 255, 255, 220), 2.f);
        for (int m = -30; m <= 30; m += 10) { if (!m) continue; float o = off - m * ppd; if (std::fabs(o) > R * 0.8f) continue; float hw = R * (m % 20 ? 0.14f : 0.22f); ImVec2 q(c.x + n.x * o, c.y + n.y * o);
            dl->AddLine(ImVec2(q.x - h.x * hw, q.y - h.y * hw), ImVec2(q.x + h.x * hw, q.y + h.y * hw), IM_COL32(255, 255, 255, 190), 1.5f); char t[8]; std::snprintf(t, sizeof t, "%d", std::abs(m)); ImVec2 re = h.x > 0 ? ImVec2(q.x + h.x * hw, q.y + h.y * hw) : ImVec2(q.x - h.x * hw, q.y - h.y * hw); dl->AddText(ImVec2(re.x + 4, re.y - 8), ck::TEXT, t); }
        for (int d : {-60, -45, -30, -20, -10, 0, 10, 20, 30, 45, 60}) { double a = -M_PI / 2 + d * M_PI / 180; float l = d % 30 == 0 ? 10 : 6; dl->AddLine(ImVec2(c.x + (float)std::cos(a) * R, c.y + (float)std::sin(a) * R), ImVec2(c.x + (float)std::cos(a) * (R - l), c.y + (float)std::sin(a) * (R - l)), ck::LINE, 1.5f); }
        { double a = -M_PI / 2 - rl; ImVec2 q(c.x + (float)std::cos(a) * (R + 1), c.y + (float)std::sin(a) * (R + 1)); ImVec2 dir((float)std::cos(a), (float)std::sin(a)), pp(-dir.y, dir.x); dl->AddTriangleFilled(q, ImVec2(q.x + dir.x * 10 + pp.x * 6, q.y + dir.y * 10 + pp.y * 6), ImVec2(q.x + dir.x * 10 - pp.x * 6, q.y + dir.y * 10 - pp.y * 6), ck::WARN); }
        dl->AddLine(ImVec2(c.x - R * 0.45f, c.y), ImVec2(c.x - R * 0.15f, c.y), ck::WARN, 3.f); dl->AddLine(ImVec2(c.x + R * 0.15f, c.y), ImVec2(c.x + R * 0.45f, c.y), ck::WARN, 3.f); dl->AddLine(ImVec2(c.x - R * 0.15f, c.y), ImVec2(c.x, c.y + 8), ck::WARN, 3.f); dl->AddLine(ImVec2(c.x + R * 0.15f, c.y), ImVec2(c.x, c.y + 8), ck::WARN, 3.f);
        char t[32]; std::snprintf(t, sizeof t, "P %+.1f   R %+.1f", pt * 57.29578, rl * 57.29578); text_c(dl, f_med, 15, c.x, r.b.y - 16, ck::TEXT, t);
    }
    void w_status(Unit& u, const Terrain& T, const std::vector<Sensor>* sens) {
        auto al = alerts(u, T, sens); bool any = false;
        for (auto& a : al) { if (a.lvl == 0) ImGui::TextColored(ImVec4(0.5f, 0.85f, 0.95f, 1), "i  %s", a.text.c_str()); else if (a.lvl == 1) ImGui::TextColored(ImVec4(1, 0.75f, 0.25f, 1), "!  %s", a.text.c_str()); else ImGui::TextColored(ImVec4(1, 0.35f, 0.32f, 1), "X  %s", a.text.c_str()); if (a.lvl > 0) any = true; }
        if (!any) ImGui::TextColored(ImVec4(0.35f, 0.9f, 0.55f, 1), "ALL SYSTEMS NORMAL");
        ImGui::Separator(); ImGui::PushStyleColor(ImGuiCol_ChildBg, ImVec4(0, 0, 0, 0)); ImGui::BeginChild("log", ImVec2(0, 0), false, ImGuiWindowFlags_NoScrollbar);
        for (int i = (int)log.size() - 1; i >= 0; --i) ImGui::TextDisabled("%s", log[i].c_str());
        ImGui::EndChild(); ImGui::PopStyleColor();
    }
    void w_controls(CkCtx& cx, Unit& u) {
        Fleet& F = *cx.fleet; Vehicle& v = u.veh; ImGui::PushItemWidth(-72);
        if (F.units.size() > 1) { char cur[96]; std::snprintf(cur, sizeof cur, "#%d %s", u.id, v.s.name.c_str()); if (ImGui::BeginCombo("Vehicle", cur)) { for (int i = 0; i < (int)F.units.size(); ++i) { char l[96]; std::snprintf(l, sizeof l, "#%d %s##%d", F.units[i].id, F.units[i].veh.s.name.c_str(), i); if (ImGui::Selectable(l, F.active == i)) F.active = i; } ImGui::EndCombo(); } }
        ImGui::TextDisabled("Camera"); ImGui::SameLine();
        const char* cn[3] = {"Free", "Follow", "Onboard"};
        for (int i = 0; i < 3; ++i) { bool on = F.cam_mode == i; if (on) ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.12f, 0.50f, 0.60f, 1)); if (ImGui::Button(cn[i])) F.cam_mode = i; if (on) ImGui::PopStyleColor(); if (i < 2) ImGui::SameLine(); }
        ImGui::Checkbox("Lamp", cx.lamp_on); if (v.has_elev || v.has_rud) { ImGui::SameLine(); ImGui::Checkbox("Attitude assist", &v.assist); }
        const char* nm[5] = {"Surge", "Sway", "Heave", "Pitch", "Yaw"}; if (v.s.glider) { nm[2] = "Pump"; nm[3] = "Mass"; nm[4] = "Roll"; }
        for (int k = 0; k < 5; ++k) { float f = (float)u.ui[k]; if (ImGui::SliderFloat(nm[k], &f, -1.f, 1.f, "%.2f")) u.ui[k] = f; }
        if (ImGui::Button("Zero")) for (double& c : u.ui) c = 0;
        ImGui::SameLine(); if (ImGui::Button("Stop and level")) { V3 p = v.p; double yw, pt, rl; v.heading_pitch_roll(yw, pt, rl); v.place(p, yw); for (double& c : u.ui) c = 0; }
        ImGui::PopItemWidth(); ImGui::TextDisabled("W/S surge  A/D turn  Q/E sideways  R/F up/down  T/G pitch (arrows work too)");
    }
    void w_tuning(Unit& u) {
        Vehicle& v = u.veh; ImGui::PushItemWidth(-150); float m = (float)v.s.mass, tr = (float)(v.s.trim * 100), ds = (float)v.drag_scale, am = (float)v.am_scale, ts = (float)v.thrust_scale, cg = (float)v.cg_lift;
        if (ImGui::SliderFloat("Mass (kg)", &m, 1.f, 400.f, "%.1f", ImGuiSliderFlags_Logarithmic)) v.s.mass = m;
        if (ImGui::SliderFloat("Net buoyancy (%)", &tr, -5.f, 5.f, "%+.2f")) v.s.trim = tr / 100.0;
        if (ImGui::SliderFloat("Drag x", &ds, 0.2f, 3.f)) v.drag_scale = ds; if (ImGui::SliderFloat("Added mass x", &am, 0.2f, 3.f)) v.am_scale = am;
        if (ImGui::SliderFloat("Thruster power x", &ts, 0.2f, 3.f)) v.thrust_scale = ts; if (ImGui::SliderFloat("CG up (m)", &cg, -0.1f, 0.1f)) v.cg_lift = cg; ImGui::PopItemWidth();
    }

    // ---------------------------------------------------------------------------------------------- toolbar
    void toolbar(CkCtx& cx) {
        ImGuiIO& io = ImGui::GetIO(); ImGui::SetNextWindowPos(ImVec2(0, 0)); ImGui::SetNextWindowSize(ImVec2(io.DisplaySize.x, 42)); ImGui::SetNextWindowBgAlpha(0.92f);
        ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0); ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(12, 7));
        ImGui::Begin("##ckbar", nullptr, ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoNav | ImGuiWindowFlags_NoBringToFrontOnFocus);
        ImGui::PushFont(f_big); ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.16f, 0.78f, 0.88f, 1)); ImGui::SetWindowFontScale(0.55f); ImGui::TextUnformatted("AUVSIM  COCKPIT"); ImGui::SetWindowFontScale(1.f); ImGui::PopStyleColor(); ImGui::PopFont(); ImGui::SameLine(0, 24);
        if (ImGui::Button("< Setup")) want_setup = true; ImGui::SameLine();
        bool was_edit = edit; if (was_edit) ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.85f, 0.55f, 0.1f, 1));
        if (ImGui::Button(edit ? "Done editing" : "Edit layout")) { edit = !edit; if (!edit) save_last(); }
        if (was_edit) ImGui::PopStyleColor(); ImGui::SameLine();
        if (ImGui::Button("Layouts")) { ImGui::OpenPopup("layouts"); refresh = true; } ImGui::SameLine();
        if (ImGui::Button("Widgets")) ImGui::OpenPopup("widgets"); ImGui::SameLine();
        bool was_live = show_live; if (was_live) ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.12f, 0.50f, 0.60f, 1)); if (ImGui::Button("Sea and camera")) show_live = !show_live; if (was_live) ImGui::PopStyleColor();
        if (cx.open_designer) { ImGui::SameLine(); if (ImGui::Button("Designer  F2")) *cx.open_designer = true; }
        if (ImGui::BeginPopup("layouts")) {
            ImGui::TextDisabled("Ready-made"); for (int i = 0; i < 4; ++i) { if (ImGui::Selectable(preset_name(i))) { apply_preset(i); save_last(); } }
            if (refresh) { saved.clear(); for (auto& s : list_presets(dir)) if (s[0] != '_') saved.push_back(s); refresh = false; }
            ImGui::Separator(); ImGui::TextDisabled("Your layouts");
            for (auto& s : saved) { if (ImGui::Selectable(s.c_str())) { Json j; std::string err; if (load_json_file(dir + "/" + s + ".json", j, err)) { layout_load(j); cur_name = s; save_last(); msg = "Loaded " + s; } } }
            ImGui::Separator(); ImGui::SetNextItemWidth(170); ImGui::InputTextWithHint("##nm", "name", name_buf, sizeof name_buf); ImGui::SameLine();
            if (ImGui::Button("Save current") && name_buf[0]) { std::string err, nm = safe_name(name_buf); if (save_json_file(dir + "/" + nm + ".json", layout_json(), err)) { cur_name = nm; msg = "Saved " + nm; refresh = true; } else msg = err; }
            if (!msg.empty()) ImGui::TextColored(ImVec4(0.6f, 0.9f, 1, 1), "%s", msg.c_str());
            ImGui::EndPopup();
        }
        if (ImGui::BeginPopup("widgets")) {
            for (int i = 0; i < W_COUNT; ++i) if (ImGui::Checkbox(wname(i), &L[i].vis)) L[i].apply = true;
            ImGui::Separator(); ImGui::SetNextItemWidth(150); ImGui::SliderFloat("Glass opacity", &opacity, 0.1f, 1.f, "%.2f"); ImGui::EndPopup();
        }
        Unit* u = cx.fleet->act(); ImGui::SameLine(0, 24); ImGui::TextDisabled("Layout: %s", cur_name.c_str());
        char right[160]; std::snprintf(right, sizeof right, "%s   %02d:%02d   %.0f FPS", u ? u->veh.s.name.c_str() : "no vehicle", (int)clock / 60, (int)clock % 60, io.Framerate);
        ImGui::SameLine(io.DisplaySize.x - ImGui::CalcTextSize(right).x - 16); ImGui::TextUnformatted(right);
        ImGui::End(); ImGui::PopStyleVar(2);
    }

    // ---------------------------------------------------------------------------------------------- the frame
    void draw(CkCtx& cx, double sim_time) {
        ImVec2 D = ImGui::GetIO().DisplaySize; if (D.x != last_disp.x || D.y != last_disp.y) { for (Slot& s : L) s.apply = true; last_disp = D; }
        toolbar(cx); Unit* u = cx.fleet->act();
        if (!u) { ImGui::SetNextWindowPos(ImVec2(D.x * 0.5f, D.y * 0.4f), 0, ImVec2(0.5f, 0.5f)); ImGui::Begin("##none", nullptr, ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoSavedSettings); ImGui::Text("No vehicle in the water. Press < Setup and START."); ImGui::End(); return; }
        Vehicle& v = u->veh; Hist& H = hist[u->id]; const Terrain& T = *cx.T; auto si = cx.usens->find(u->id); const std::vector<Sensor>* sens = si == cx.usens->end() ? nullptr : &si->second;
        double yw, pt, rl; v.heading_pitch_roll(yw, pt, rl); double hdg = std::fmod(90.0 - yw * 57.29578 + 720.0, 360.0), depth = -v.p.z, alt = v.p.z - T.ground_z((float)v.p.x, (float)v.p.y);
        ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 10.f);
        if (open(W_HEADING)) { w_heading(hdg); close(W_HEADING); }
        if (open(W_DEPTH)) { w_depth(*u, depth, alt); close(W_DEPTH); }
        if (open(W_QUICK)) { w_quick(*u, depth, alt); close(W_QUICK); }
        if (open(W_ATT)) { w_attitude(pt, rl); close(W_ATT); }
        if (open(W_STATUS, true)) { w_status(*u, T, sens); close(W_STATUS); }
        if (open(W_CONTROLS, true)) { w_controls(cx, *u); close(W_CONTROLS); }
        if (open(W_MAP, true)) { view_map(map, T, *cx.cur, *cx.fleet, *u, H, sim_time); close(W_MAP); }
        if (open(W_PLOTS, true)) { view_plots(H, plots); close(W_PLOTS); }
        if (open(W_THRUST, true)) { view_thrusters(*u); close(W_THRUST); }
        if (open(W_SENSORS, true)) { if (sens) view_sensors(sense, *u, H, *sens, T, *cx.cur, *cx.water, sim_time, *cx.fleet); else ImGui::TextDisabled("No sensor list for this vehicle."); close(W_SENSORS); }
        if (open(W_TUNING, true)) { w_tuning(*u); close(W_TUNING); }
        ImGui::PopStyleVar();
    }
};
