// design_ui.hpp - Step 8C: the "AUV Designer" page. Controls on the left, a live drawing in the middle, result tables at the bottom.
// Every change re-runs the design engine (auv_design.hpp) and everything updates at once.
#pragma once
#include "imgui.h"
#include "auv_design.hpp"
#include "design_io.hpp"
#include <cstring>
#include <filesystem>
#include <algorithm>
#include <cmath>
#include <string>
#include <vector>
#include <functional>

class DesignerUI {
public:
    design::DesignIn d;
    design::DesignOut o;
    std::string dir = "config/designs";
    explicit DesignerUI(const std::string& presets_dir = "config/designs") : dir(presets_dir) {
        std::error_code ec; std::filesystem::create_directories(dir, ec);
        design::write_builtin_designs(dir);
        refresh(); int k = 0; for (size_t i = 0; i < files_.size(); ++i) if (files_[i].find("REMUS") == 0) k = (int)i; load_index(k);
    }

    // draw the whole page inside the rectangle (x, y, w, h)
    void draw(float x, float y, float w, float h) {
        ImGui::SetNextWindowPos(ImVec2(x, y)); ImGui::SetNextWindowSize(ImVec2(w, h));
        ImGui::Begin("AUV Designer", nullptr, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoScrollbar);
        bool changed = false;
        float lw = std::min(400.f, w * 0.34f), bh = std::max(250.f, h * 0.36f);
        ImGui::BeginChild("left", ImVec2(lw, 0), true);
        changed |= controls();
        ImGui::EndChild();
        ImGui::SameLine();
        ImGui::BeginGroup();
        ImGui::BeginChild("top", ImVec2(0, h - bh - 24), true, ImGuiWindowFlags_NoScrollbar);
        drawing();
        ImGui::EndChild();
        ImGui::BeginChild("bottom", ImVec2(0, 0), true);
        tabs();
        ImGui::EndChild();
        ImGui::EndGroup();
        ImGui::End();
        if (changed || first_) { recompute(); first_ = false; }
    }
    void recompute() { design::Designer ds; o = ds.run(d); }
    void load_index_public(int i) { load_index(i); }

private:
    bool first_ = true;
public:
    int tab_req_ = -1;          // test aid: open this result tab
    std::function<bool()> lib_hook;                 // when set, it draws the preset / library row of the control panel instead (returns true if the design changed)
    std::vector<std::string> overridden;            // published numbers that replace the designer's results (shown above the result tabs)
    void relist() { refresh(); }
private:
    ImGuiTabItemFlags tf(int i) const { return tab_req_ == i ? ImGuiTabItemFlags_SetSelected : 0; }
    std::vector<std::string> files_; int sel_ = 0; char save_name_[64] = "";

    void refresh() { files_ = design::list_designs(dir); }
    void load_index(int i) { if (i >= 0 && i < (int)files_.size()) { design::DesignIn t; if (design::load_design(dir + "/" + files_[i] + ".json", t)) { d = t; sel_ = i; std::snprintf(save_name_, sizeof save_name_, "%s", d.name.c_str()); first_ = true; } } }

    // ---- small widgets ----
    static bool SD(const char* label, double& v, double mn, double mx, const char* fmt = "%.2f", const char* tip = nullptr) {
        ImGui::SetNextItemWidth(-150); bool c = ImGui::SliderScalar(label, ImGuiDataType_Double, &v, &mn, &mx, fmt, ImGuiSliderFlags_AlwaysClamp);
        if (tip && ImGui::IsItemHovered()) ImGui::SetTooltip("%s\n(Ctrl+click to type a value)", tip); return c;
    }
    static bool SI(const char* label, int& v, int mn, int mx, const char* tip = nullptr) {
        ImGui::SetNextItemWidth(-150); bool c = ImGui::SliderInt(label, &v, mn, mx);
        if (tip && ImGui::IsItemHovered()) ImGui::SetTooltip("%s", tip); return c;
    }
    static bool CB(const char* label, int& v, const char* const* items, int n) { ImGui::SetNextItemWidth(-150); return ImGui::Combo(label, &v, items, n); }
    static bool CK(const char* label, bool& v, const char* tip = nullptr) { bool c = ImGui::Checkbox(label, &v); if (tip && ImGui::IsItemHovered()) ImGui::SetTooltip("%s", tip); return c; }
    static bool has(const std::vector<int>& v, int k) { return std::find(v.begin(), v.end(), k) != v.end(); }

    // ---- the control panel ----
    bool controls() {
        bool ch = false;
        ImGui::TextColored(ImVec4(0.16f, 0.78f, 0.88f, 1), "AUV DESIGNER");
        ImGui::Separator();
        // presets: the Designer hub supplies the vehicle library; stand-alone it is the plain list of saved designs
        if (lib_hook) ch |= lib_hook();
        else
        {
            std::vector<const char*> names; for (auto& s : files_) names.push_back(s.c_str());
            ImGui::SetNextItemWidth(-60);
            if (ImGui::Combo("##preset", &sel_, names.data(), (int)names.size())) load_index(sel_);
            ImGui::SameLine(); if (ImGui::Button("Reload")) { refresh(); load_index(sel_); }
            ImGui::SetNextItemWidth(-60); ImGui::InputText("##sname", save_name_, sizeof save_name_);
            ImGui::SameLine();
            if (ImGui::Button("Save")) {
                d.name = save_name_; std::string f; for (char c : d.name) f += (std::isalnum((unsigned char)c) ? c : '_');
                if (!f.empty()) { design::save_design(dir + "/" + f + ".json", d); refresh(); for (size_t i = 0; i < files_.size(); ++i) if (files_[i] == f) sel_ = (int)i; }
            }
            if (ImGui::IsItemHovered()) ImGui::SetTooltip("Saves to %s/<name>.json", dir.c_str());
        }
        ImGui::Spacing();

        if (ImGui::CollapsingHeader("Hull", ImGuiTreeNodeFlags_DefaultOpen)) {
            ch |= SD("Length (m)", d.L, 0.5, 4.0, "%.2f"); ch |= SD("Diameter (m)", d.D, 0.08, 0.6, "%.3f");
            ch |= SD("Nose length / L", d.nose_frac, 0.05, 0.3, "%.2f"); ch |= SD("Tail cone / L", d.tail_frac, 0.1, 0.5, "%.2f");
            ch |= SD("Tail end dia / D", d.tail_end, 0.1, 0.9, "%.2f", "Diameter at the very tail, where the propeller shaft leaves");
            const char* nk[] = {"Ellipsoid (low drag)", "Bullet (more sensor room)"}; ch |= CB("Nose shape", d.nose_kind, nk, 2);
            const char* mt[4]; for (int i = 0; i < 4; ++i) mt[i] = design::MATERIALS[i].name; ch |= CB("Hull material", d.material, mt, 4);
            ch |= SD("Rated depth (m)", d.rated_depth, 10, 1000, "%.0f", "Sets the pressure hull wall thickness (buckling + yield, safety factor 1.5)");
            ch |= SD("Wanted buoyancy", d.trim, 0.0, 0.08, "%.3f", "Positive buoyancy as a fraction of the weight. Lead ballast is added to reach it.");
        }
        if (ImGui::CollapsingHeader("Fins", ImGuiTreeNodeFlags_DefaultOpen)) {
            int del = -1;
            for (size_t i = 0; i < d.fins.size(); ++i) {
                ImGui::PushID((int)(100 + i)); auto& f = d.fins[i];
                char hn[48]; std::snprintf(hn, sizeof hn, "Fin set %d  (%d fins)###fs", (int)i + 1, f.count);
                if (ImGui::TreeNodeEx(hn, ImGuiTreeNodeFlags_DefaultOpen)) {
                    ch |= CK("In use", f.on); ImGui::SameLine(); ch |= CK("Moving (rudder/elevator)", f.control, "Off = fixed stabiliser");
                    ch |= SI("Number of fins", f.count, 1, 8);
                    ch |= SD("Angle of first fin", f.angle0, 0, 90, "%.0f deg", "0 = '+' layout, 45 = 'X' layout (4 fins)");
                    ch |= SD("Position x (m)", f.x, -2.0, 2.0, "%.2f", "Quarter-chord point from the middle of the hull (+ forward)");
                    ch |= SD("Root chord (m)", f.cr, 0.03, 0.5, "%.3f"); ch |= SD("Tip chord (m)", f.ct, 0.02, 0.5, "%.3f");
                    ch |= SD("Span (m)", f.span, 0.02, 0.5, "%.3f"); ch |= SD("Sweep (m)", f.sweep, 0.0, 0.3, "%.3f");
                    ch |= SD("Thickness / chord", f.tc, 0.06, 0.25, "%.2f", "NACA 00xx: 0.12 = NACA 0012");
                    ch |= SD("Max deflection", f.max_defl, 5, 45, "%.0f deg");
                    const char* sm[4]; for (int k = 0; k < 4; ++k) sm[k] = pdz::SHAFT_MATS[k].name; ch |= CB("Fin stock", f.stock_mat, sm, 4);
                    if (d.fins.size() > 1 && ImGui::Button("Remove this fin set")) del = (int)i;
                    ImGui::TreePop();
                }
                ImGui::PopID();
            }
            if (del >= 0) { d.fins.erase(d.fins.begin() + del); ch = true; }
            if (d.fins.size() < 4 && ImGui::Button("+ Add fin set")) { design::FinSet f; f.x = d.fins.empty() ? -0.5 * d.L * 0.7 : 0.2 * d.L; f.control = false; f.count = 2; d.fins.push_back(f); ch = true; }
            ImGui::SameLine(); if (ImGui::Button("Planform helper")) ImGui::OpenPopup("pf");
            if (ImGui::BeginPopup("pf")) {
                ImGui::TextDisabled("Make set 1 from area, aspect ratio and taper");
                ImGui::SliderFloat("Area (cm2)", &pf_area_, 10, 400, "%.0f"); ImGui::SliderFloat("Aspect ratio", &pf_ar_, 0.5f, 3, "%.2f"); ImGui::SliderFloat("Taper", &pf_tp_, 0.3f, 1, "%.2f");
                if (ImGui::Button("Apply")) {
                    double S = pf_area_ * 1e-4, ar = pf_ar_, tp = pf_tp_; double b = std::sqrt(ar * S), cm = S / b, cr = 2 * cm / (1 + tp);
                    auto& f = d.fins[0]; f.span = b; f.cr = cr; f.ct = cr * tp; f.sweep = 0.25 * (f.cr - f.ct); ch = true; ImGui::CloseCurrentPopup();
                }
                ImGui::EndPopup();
            }
        }
        if (ImGui::CollapsingHeader("Propellers and thrusters", ImGuiTreeNodeFlags_DefaultOpen)) {
            int del = -1; const char* roles[] = {"Main (tail)", "Vertical thruster", "Lateral thruster"};
            for (size_t i = 0; i < d.props.size(); ++i) {
                ImGui::PushID((int)(200 + i)); auto& p = d.props[i];
                char hn[64]; std::snprintf(hn, sizeof hn, "%s %d  (%d blades)###pr", roles[p.role], (int)i + 1, p.blades);
                if (ImGui::TreeNodeEx(hn, ImGuiTreeNodeFlags_DefaultOpen)) {
                    ch |= SD("Diameter (m)", p.D, 0.03, 0.5, "%.3f"); ch |= SI("Blades", p.blades, 2, 7);
                    const char* mdl[] = {"Blade element (BEMT)", "Wageningen B-series"}; ch |= CB("Design method", p.model, mdl, 2);
                    if (p.model == 1) { ch |= SD("Blade area ratio", p.ear, 0.3, 1.05, "%.2f", "AE/A0. The Keller cavitation limit is checked for you."); ch |= SD("Pitch / diameter", p.pd, 0.5, 1.4, "%.2f"); }
                    else ch |= SD("Pitch / diameter", p.pd, 0.5, 1.6, "%.2f");
                    ch |= CK("Duct (Kort nozzle)", p.ducted);
                    if (p.role == 0) {
                        ch |= CK("Automatic rpm", p.auto_rpm, "Choose the rpm that reaches the top speed"); if (!p.auto_rpm) ch |= SD("Max rpm", p.rpm, 200, 6000, "%.0f");
                        ch |= CK("Contra-rotating pair", p.contra, "Two coaxial propellers turning opposite ways: the torques cancel and the swirl is recovered");
                        if (p.contra) { ch |= SI("Rear blades", p.blades2, 2, 7); ch |= SD("Rear dia / front", p.d2_ratio, 0.8, 1.0, "%.2f"); }
                    } else {
                        ch |= SD("Max rpm", p.rpm, 500, 8000, "%.0f"); ch |= SD("Position x (m)", p.x, -1.5, 1.5, "%.2f");
                    }
                    const char* sm[4]; for (int k = 0; k < 4; ++k) sm[k] = pdz::SHAFT_MATS[k].name; ch |= CB("Shaft material", p.shaft_mat, sm, 4);
                    ch |= CK("Key way", p.keyway); ImGui::SameLine(); ch |= SD("Hollow ratio", p.bore, 0.0, 0.7, "%.2f", "Inner / outer diameter of a hollow shaft (0 = solid)");
                    if (d.props.size() > 1 && ImGui::Button("Remove")) del = (int)i;
                    ImGui::TreePop();
                }
                ImGui::PopID();
            }
            if (del >= 0) { d.props.erase(d.props.begin() + del); ch = true; }
            if (ImGui::Button("+ Vertical thruster")) { design::PropSpec t; t.role = 1; t.D = 0.08; t.rpm = 3000; t.x = 0.3; d.props.push_back(t); ch = true; }
            ImGui::SameLine();
            if (ImGui::Button("+ Lateral thruster")) { design::PropSpec t; t.role = 2; t.D = 0.08; t.rpm = 3000; t.x = 0.25; d.props.push_back(t); ch = true; }
        }
        if (ImGui::CollapsingHeader("Battery, motor, payload")) {
            ch |= SD("Energy (Wh)", d.battery_Wh, 50, 10000, "%.0f"); ch |= SD("Pack Wh/kg", d.pack_Wh_per_kg, 80, 260, "%.0f");
            ch |= SD("Bus voltage (V)", d.bus_V, 7, 60, "%.1f", "Cells in series = voltage / 3.7");
            ch |= SD("Cell capacity (Ah)", d.cell_Ah, 1, 12, "%.1f"); ch |= SD("Cell max C-rate", d.cell_maxC, 0.5, 10, "%.1f");
            ch |= SD("Electronics (kg)", d.electronics_kg, 0.1, 10, "%.1f"); ch |= SD("Payload (kg)", d.payload_kg, 0, 20, "%.1f"); ch |= SD("Hotel load (W)", d.hotel_W, 1, 100, "%.0f", "Computer and instruments that run all the time");
            ch |= CK("Antenna mast", d.mast);
        }
        if (ImGui::CollapsingHeader("Sensors")) {
            const design::SensorPart* sp = design::sensor_parts();
            for (int i = 0; i < design::SE_COUNT; ++i) {
                bool on = has(d.sensors, i);
                char lb[96]; std::snprintf(lb, sizeof lb, "%s  (%.2f kg, %.1f W)", sp[i].name, sp[i].mass, sp[i].power);
                if (ImGui::Checkbox(lb, &on)) { if (on) d.sensors.push_back(i); else d.sensors.erase(std::remove(d.sensors.begin(), d.sensors.end(), i), d.sensors.end()); ch = true; }
                if (ImGui::IsItemHovered()) ImGui::SetTooltip("%s", sp[i].note);
            }
        }
        if (ImGui::CollapsingHeader("Mission and water")) {
            ch |= SD("Cruise speed (m/s)", d.v_cruise, 0.3, 4, "%.2f"); ch |= SD("Top speed (m/s)", d.v_max, 0.5, 5, "%.2f");
            const char* wn[] = {design::WATERS[0].name, design::WATERS[1].name, design::WATERS[2].name, design::WATERS[3].name}; ch |= CB("Water", d.water, wn, 4);
        }
        return ch;
    }
    float pf_area_ = 100, pf_ar_ = 1.2f, pf_tp_ = 0.6f;

    // ---- the drawing ----
    static ImU32 col(float r, float g, float b, float a = 1) { return IM_COL32((int)(r * 255), (int)(g * 255), (int)(b * 255), (int)(a * 255)); }
    double radius_at(double x) const { if (o.outline.empty()) return 0; double best = 0, bd = 1e9; for (auto& p : o.outline) { double dd = std::fabs(p.first - x); if (dd < bd) { bd = dd; best = p.second; } } return best; }

    void drawing() {
        ImDrawList* dl = ImGui::GetWindowDrawList(); ImVec2 p0 = ImGui::GetCursorScreenPos(), sz = ImGui::GetContentRegionAvail();
        if (sz.x < 100 || sz.y < 100 || o.outline.empty()) return;
        float ew = std::min(sz.y * 0.9f, sz.x * 0.22f);                 // end view width
        float sw = sz.x - ew - 30;
        double Lx = o.x_nose - o.x_tail; float k = (float)std::min((sw - 70) / Lx, (sz.y - 70) / (std::max(d.D, 0.1) * 2.6));
        float cxs = p0.x + 20 + sw * 0.5f - (float)((o.x_nose + o.x_tail) * 0.5) * k, cys = p0.y + sz.y * 0.5f;
        auto P = [&](double x, double z) { return ImVec2(cxs + (float)x * k, cys - (float)z * k); };
        dl->AddRectFilled(p0, ImVec2(p0.x + sz.x, p0.y + sz.y), col(0.03f, 0.05f, 0.08f), 6);
        // grid
        for (double gx = std::ceil(o.x_tail * 5) / 5; gx < o.x_nose; gx += 0.2) dl->AddLine(P(gx, -0.45), P(gx, 0.45), col(0.2f, 0.3f, 0.4f, 0.25f));
        dl->AddLine(P(o.x_tail - 0.2, 0), P(o.x_nose + 0.2, 0), col(0.4f, 0.55f, 0.65f, 0.35f));
        // fins behind the hull first
        for (auto& f : o.fin_draw) {
            double c = std::cos(f.ang * M_PI / 180), s = std::sin(f.ang * M_PI / 180); double xr = f.x_le, r0 = radius_at(xr - 0.5 * f.cr);
            ImU32 fc = f.control ? col(0.95f, 0.65f, 0.2f, 0.85f) : col(0.6f, 0.75f, 0.85f, 0.75f);
            if (std::fabs(c) < 0.08) { dl->AddLine(P(xr, 0), P(xr - f.cr, 0), fc, 3); continue; }
            double z0 = r0 * c, z1 = (r0 + f.span) * c; ImVec2 q[4] = {P(xr, z0), P(xr - f.cr, z0), P(xr - f.sweep - f.ct, z1), P(xr - f.sweep, z1)};
            dl->AddConvexPolyFilled(q, 4, s >= 0 ? fc : col(0.4f, 0.45f, 0.5f, 0.6f)); dl->AddPolyline(q, 4, col(1, 1, 1, 0.5f), ImDrawFlags_Closed, 1);
        }
        // hull
        std::vector<ImVec2> top, bot; for (auto& p : o.outline) { top.push_back(P(p.first, p.second)); bot.push_back(P(p.first, -p.second)); }
        std::vector<ImVec2> poly = top; for (int i = (int)bot.size() - 1; i >= 0; --i) poly.push_back(bot[i]);
        dl->AddConvexPolyFilled(poly.data(), (int)poly.size(), col(0.12f, 0.30f, 0.42f, 0.95f)); dl->AddPolyline(poly.data(), (int)poly.size(), col(0.45f, 0.85f, 0.95f), ImDrawFlags_Closed, 1.6f);
        // components
        for (auto& it : o.items) { ImVec2 c = P(it.x, it.z); float s = 3 + 3 * std::sqrt((float)(it.m / std::max(o.mass, 1.0))) * 4; ImU32 cc = col(0.9f, 0.9f, 0.35f, 0.65f);
            if (it.name.find("battery") != std::string::npos) cc = col(0.3f, 0.9f, 0.4f, 0.55f); else if (it.name.find("ballast") != std::string::npos) cc = col(0.8f, 0.4f, 0.4f, 0.65f);
            dl->AddRectFilled(ImVec2(c.x - s, c.y - s), ImVec2(c.x + s, c.y + s), cc, 2);
            ImVec2 a(c.x - s, c.y - s), b(c.x + s, c.y + s); if (ImGui::IsMouseHoveringRect(a, b)) ImGui::SetTooltip("%s\n%.2f kg", it.name.c_str(), it.m); }
        // propellers
        for (auto& p : o.prop_draw) {
            if (p.role == 0) { ImVec2 a = P(p.x - 0.02, p.D * 0.5), b = P(p.x + 0.02, -p.D * 0.5); dl->AddRectFilled(a, b, col(0.95f, 0.4f, 0.4f, 0.8f)); if (p.ducted) { dl->AddRect(P(p.x - 0.06, p.D * 0.5 + 0.01), P(p.x + 0.06, p.D * 0.5 + 0.015), col(0.7f, 0.7f, 0.7f)); dl->AddRect(P(p.x - 0.06, -p.D * 0.5 - 0.01), P(p.x + 0.06, -p.D * 0.5 - 0.015), col(0.7f, 0.7f, 0.7f)); } if (p.contra) dl->AddRectFilled(P(p.x - 0.07, p.D * 0.47), P(p.x - 0.04, -p.D * 0.47), col(1, 0.6f, 0.2f, 0.8f)); }
            else if (p.role == 1) dl->AddRect(P(p.x - 0.5 * p.D, d.D * 0.5 + 0.005), P(p.x + 0.5 * p.D, -d.D * 0.5 - 0.005), col(0.4f, 0.95f, 0.5f), 2, 0, 2);
            else dl->AddCircle(P(p.x, 0), (float)(0.5 * p.D) * k, col(0.4f, 0.8f, 1.0f), 24, 2);
        }
        // CG, CB
        { ImVec2 cg = P(o.rg.x, o.rg.z), cb = P(0, 0); dl->AddCircleFilled(cb, 6, col(0.3f, 0.6f, 1)); dl->AddCircle(cb, 8, col(1, 1, 1)); dl->AddText(ImVec2(cb.x + 10, cb.y - 22), col(0.5f, 0.75f, 1), "CB");
          dl->AddCircleFilled(cg, 6, col(1, 0.35f, 0.3f)); dl->AddCircle(cg, 8, col(1, 1, 1)); dl->AddText(ImVec2(cg.x + 10, cg.y + 4), col(1, 0.55f, 0.5f), "CG"); }
        // scale bar
        { ImVec2 a = P(o.x_tail, -0.42), b = P(o.x_tail + 0.5, -0.42); dl->AddLine(a, b, col(0.8f, 0.9f, 1), 2); dl->AddText(ImVec2(a.x, a.y + 4), col(0.8f, 0.9f, 1), "0.5 m"); }
        dl->AddText(ImVec2(p0.x + 10, p0.y + 8), col(0.7f, 0.85f, 0.95), "SIDE VIEW  (nose to the right)");
        dl->AddText(ImVec2(p0.x + 10, p0.y + 26), col(0.55f, 0.65f, 0.75), "orange = moving fin   grey = fixed fin   green = battery   red = ballast   yellow = other parts");
        // end view
        {
            ImVec2 c(p0.x + sz.x - ew * 0.5f - 10, cys); float ke = ew * 0.38f / (float)std::max(d.D * 0.5 + 0.18, 0.25);
            dl->AddText(ImVec2(c.x - 40, p0.y + 8), col(0.7f, 0.85f, 0.95), "END VIEW (from behind)");
            dl->AddCircle(c, (float)(0.5 * d.D) * ke, col(0.45f, 0.85f, 0.95), 48, 2); dl->AddCircleFilled(c, (float)(0.5 * d.D) * ke, col(0.12f, 0.30f, 0.42f, 0.9f), 48);
            for (auto& f : o.fin_draw) { double a = f.ang * M_PI / 180; ImVec2 u((float)std::sin(a), -(float)std::cos(a)); float r0 = (float)(0.5 * d.D) * ke, r1 = (float)(0.5 * d.D + f.span) * ke;
                dl->AddLine(ImVec2(c.x + u.x * r0, c.y + u.y * r0), ImVec2(c.x + u.x * r1, c.y + u.y * r1), f.control ? col(0.95f, 0.65f, 0.2f) : col(0.6f, 0.75f, 0.85f), 4); }
            for (auto& p : o.prop_draw) if (p.role == 0) { dl->AddCircle(c, (float)(0.5 * p.D) * ke, col(0.95f, 0.4f, 0.4f), 40, 1.5f); break; }
        }
        ImGui::Dummy(sz);
    }

    // ---- result tables ----
    static void kv(const char* a, const char* fmt, double v, const char* unit = "") { ImGui::TableNextRow(); ImGui::TableNextColumn(); ImGui::TextUnformatted(a); ImGui::TableNextColumn(); ImGui::Text(fmt, v); ImGui::SameLine(); ImGui::TextDisabled("%s", unit); }
    static bool is_alarm(const std::string& w) { int n = 0; while (n < (int)w.size() && std::isupper((unsigned char)w[n])) ++n; return n >= 4; }

    void tabs() {
        if (!overridden.empty()) {
            std::string t = "Published data replaces these designer results: "; for (size_t i = 0; i < overridden.size() && i < 6; ++i) t += (i ? "; " : "") + overridden[i]; if (overridden.size() > 6) t += "; ... (" + std::to_string(overridden.size()) + " in all)";
            ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.72f, 0.3f, 1)); ImGui::TextWrapped("%s", t.c_str()); ImGui::PopStyleColor();
            if (ImGui::IsItemHovered()) { std::string all; for (auto& x : overridden) all += x + "\n"; ImGui::SetTooltip("%s", all.c_str()); }
        }
        if (ImGui::BeginTabBar("rt")) {
            if (ImGui::BeginTabItem("Summary", nullptr, tf(0))) {
                ImGui::Columns(3, nullptr, false);
                ImGui::TextColored(ImVec4(0.16f, 0.78f, 0.88f, 1), "Hull and mass");
                if (ImGui::BeginTable("t1", 2)) { kv("Mass", "%.1f", o.mass, "kg"); kv("Volume", "%.1f", o.V * 1000, "litres"); kv("Wetted area", "%.2f", o.wetted, "m2"); kv("Fineness L/D", "%.1f", o.fineness); kv("Wall thickness", "%.1f", o.wall_t * 1000, "mm"); kv("Ballast", "%.2f", o.ballast, "kg"); kv("Net buoyancy", "%.2f", o.net_up, "N"); ImGui::EndTable(); }
                ImGui::NextColumn(); ImGui::TextColored(ImVec4(0.16f, 0.78f, 0.88f, 1), "Performance");
                if (ImGui::BeginTable("t2", 2)) { kv("Top speed", "%.2f", o.v_top, "m/s"); kv("Drag at cruise", "%.1f", o.drag_cruise, "N"); kv("Power at cruise", "%.0f", o.P_elec_cruise, "W"); kv("Endurance", "%.1f", o.endurance_h, "h"); kv("Range", "%.0f", o.range_km, "km"); kv("Thrust (bollard)", "%.0f", o.thrust_bollard, "N"); kv("Main rpm", "%.0f", o.rpm_main); ImGui::EndTable(); }
                ImGui::NextColumn(); ImGui::TextColored(ImVec4(0.16f, 0.78f, 0.88f, 1), "Stability and control");
                if (ImGui::BeginTable("t3", 2)) { kv("BG (CG below CB)", "%.1f", o.BG * 1000, "mm"); kv("Roll period", "%.1f", o.T_roll, "s"); kv("Pitch period", "%.1f", o.T_pitch, "s"); kv("Fin stability ratio", "%.2f", o.stability_ratio); kv("Max yaw rate", "%.1f", o.yaw_rate_max * 180 / M_PI, "deg/s"); kv("Turn radius", "%.1f", o.turn_radius, "m"); kv("Max pitch", "%.0f", o.pitch_max_deg, "deg"); ImGui::EndTable(); }
                ImGui::Columns(1); ImGui::EndTabItem();
            }
            if (ImGui::BeginTabItem("Mass budget", nullptr, tf(1))) {
                if (ImGui::BeginTable("mb", 5, ImGuiTableFlags_RowBg | ImGuiTableFlags_ScrollY | ImGuiTableFlags_Borders, ImVec2(0, 0))) {
                    ImGui::TableSetupScrollFreeze(0, 1); ImGui::TableSetupColumn("Part", ImGuiTableColumnFlags_WidthStretch); ImGui::TableSetupColumn("Mass kg"); ImGui::TableSetupColumn("%"); ImGui::TableSetupColumn("x m"); ImGui::TableSetupColumn("z m"); ImGui::TableHeadersRow();
                    for (auto& it : o.items) { ImGui::TableNextRow(); ImGui::TableNextColumn(); ImGui::TextUnformatted(it.name.c_str()); ImGui::TableNextColumn(); ImGui::Text("%.3f", it.m); ImGui::TableNextColumn(); ImGui::Text("%.1f", 100 * it.m / std::max(o.mass, 1e-6)); ImGui::TableNextColumn(); ImGui::Text("%+.3f", it.x); ImGui::TableNextColumn(); ImGui::Text("%+.3f", it.z); }
                    ImGui::TableNextRow(); ImGui::TableNextColumn(); ImGui::TextColored(ImVec4(0.16f, 0.78f, 0.88f, 1), "TOTAL"); ImGui::TableNextColumn(); ImGui::Text("%.2f", o.mass); ImGui::TableNextColumn(); ImGui::Text("100"); ImGui::TableNextColumn(); ImGui::Text("%+.3f", o.rg.x); ImGui::TableNextColumn(); ImGui::Text("%+.3f", o.rg.z);
                    ImGui::EndTable();
                }
                ImGui::EndTabItem();
            }
            if (ImGui::BeginTabItem("Propulsion and shafts", nullptr, tf(2))) {
                if (ImGui::BeginTable("pt", 11, ImGuiTableFlags_RowBg | ImGuiTableFlags_Borders | ImGuiTableFlags_ScrollX | ImGuiTableFlags_ScrollY)) {
                    const char* h[] = {"Role", "Blades", "D mm", "P/D", "EAR (min)", "rpm", "Tip m/s", "Eff. cruise", "Shaft mm (IACS)", "Shaft stress MPa", "Crit. margin"};
                    for (auto s : h) ImGui::TableSetupColumn(s); ImGui::TableHeadersRow();
                    const char* rn[] = {"Main", "Vertical", "Lateral"};
                    for (auto& r : o.prop_rows) { ImGui::TableNextRow(); int c = 0; auto C = [&]() { ImGui::TableSetColumnIndex(c++); };
                        C(); ImGui::Text("%s%s", rn[r.role], r.contra ? " (CRP)" : ""); C(); if (r.Z2) ImGui::Text("%d + %d", r.Z, r.Z2); else ImGui::Text("%d", r.Z); C(); ImGui::Text("%.0f", r.D * 1000); C(); ImGui::Text("%.2f", r.PD);
                        C(); ImGui::Text("%.2f (%.2f)", r.EAR, r.ear_min); C(); if (r.rpm2 > 0) ImGui::Text("%.0f / %.0f", r.rpm, r.rpm2); else ImGui::Text("%.0f", r.rpm); C(); ImGui::Text("%.1f", r.tip); C(); ImGui::Text("%.2f", r.eta_cruise);
                        C(); ImGui::Text("%.1f (%.1f)", r.shaft_d * 1000, r.shaft_iacs * 1000); C(); ImGui::Text("%.0f", r.shaft_sigma / 1e6); C(); ImGui::TextColored(r.crit_margin > 1.4 ? ImVec4(0.4f, 0.9f, 0.5f, 1) : ImVec4(1, 0.5f, 0.4f, 1), "%.2f", r.crit_margin); }
                    ImGui::EndTable();
                }
                ImGui::EndTabItem();
            }
            if (ImGui::BeginTabItem("Fins", nullptr, tf(3))) {
                if (ImGui::BeginTable("ft", 9, ImGuiTableFlags_RowBg | ImGuiTableFlags_Borders | ImGuiTableFlags_ScrollY)) {
                    const char* h[] = {"Angle", "Area cm2", "AR", "CL/deg", "Stall deg", "Max force N", "Stock mm", "Servo Nm", "Rudder / elev."};
                    for (auto s : h) ImGui::TableSetupColumn(s); ImGui::TableHeadersRow();
                    for (auto& r : o.fin_rows) { ImGui::TableNextRow(); ImGui::TableNextColumn(); ImGui::Text("%.0f", r.ang); ImGui::TableNextColumn(); ImGui::Text("%.0f", r.area * 1e4); ImGui::TableNextColumn(); ImGui::Text("%.2f", r.ar); ImGui::TableNextColumn(); ImGui::Text("%.3f", r.cla_deg);
                        ImGui::TableNextColumn(); ImGui::Text("%.0f", r.stall_deg); ImGui::TableNextColumn(); ImGui::Text("%.0f", r.F_max); ImGui::TableNextColumn(); ImGui::Text("%.1f", r.stock_d * 1000); ImGui::TableNextColumn(); ImGui::Text("%.1f", r.servo_Nm); ImGui::TableNextColumn(); ImGui::Text("%+.2f / %+.2f", r.rud, r.elev); }
                    ImGui::EndTable();
                }
                ImGui::EndTabItem();
            }
            if (ImGui::BeginTabItem("Battery and motor", nullptr, tf(4))) {
                const auto& p = o.pack;
                if (ImGui::BeginTable("bt", 2)) {
                    kv("Cells in series", "%.0f", p.Ns); kv("Strings in parallel", "%.0f", p.Np); kv("Pack voltage", "%.1f", p.V, "V"); kv("Capacity", "%.1f", p.Ah, "Ah");
                    kv("Current at cruise", "%.1f", p.I_cruise, "A"); kv("Current at top speed", "%.1f", p.I_max, "A"); kv("Discharge rate", "%.2f", p.C_max, "C"); kv("Cell limit", "%.1f", p.C_limit, "C");
                    kv("Motor Kv", "%.0f", p.kv, "rpm/V"); kv("Gearbox ratio", "%.1f", p.gear, ": 1"); kv("ESC rating", "%.0f", p.I_esc, "A"); kv("Motor torque", "%.2f", p.T_motor, "N m");
                    kv("Sensors", "%.2f", o.sensors_kg, "kg"); kv("Sensors + hotel power", "%.0f", o.hotel_total, "W");
                    kv("Net buoyancy, sea water", "%.2f", o.net_up_sea, "N"); kv("Net buoyancy, fresh water", "%.2f", o.net_up_fresh, "N"); kv("Net buoyancy, cold fresh", "%.2f", o.net_up_cold, "N");
                    ImGui::EndTable();
                }
                ImGui::EndTabItem();
            }
            char wl[40]; std::snprintf(wl, sizeof wl, "Warnings (%d)###warn", (int)o.warnings.size());
            if (ImGui::BeginTabItem(wl, nullptr, tf(5))) {
                if (o.warnings.empty()) ImGui::TextColored(ImVec4(0.4f, 0.9f, 0.5f, 1), "No warnings. The design is consistent.");
                for (auto& w : o.warnings) { ImGui::PushStyleColor(ImGuiCol_Text, is_alarm(w) ? ImVec4(1, 0.4f, 0.35f, 1) : ImVec4(1, 0.8f, 0.3f, 1)); ImGui::TextWrapped("- %s", w.c_str()); ImGui::PopStyleColor(); }
                ImGui::EndTabItem();
            }
            ImGui::EndTabBar();
        }
    }
};
