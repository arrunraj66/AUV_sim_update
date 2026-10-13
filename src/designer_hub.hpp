// designer_hub.hpp - the AUV Designer as a top-level overlay of the whole app (Step 10).
//   * a small "Designer F2" button in every mode (the Cockpit has it in its own toolbar) and the F2 hotkey; Escape or "Close" closes it
//   * while it is open it is a full-screen window on top of everything and the keyboard / gamepad no longer drive the vehicle or the camera
//   * Apply to deployed AUV / Deploy as new / Test this design, the vehicle library (saved designs + real vehicles), published overrides, saved state
// The logic lives in designer_core.hpp (testable without a GUI); this file is the ImGui part.
#pragma once
#include "imgui.h"
#include "imgui_internal.h"
#include "design_ui.hpp"
#include "fleet.hpp"
#include "designer_core.hpp"
#include <memory>

struct DesignerHub {
    Fleet* fleet = nullptr;
    std::unique_ptr<DesignerUI> ui;
    dcore::Source src;
    dcore::DesignerState st;
    dcore::DesignTest test;
    bool open = false, result_open = false, pause_sim = true;      // pause_sim: the vehicles do not step while the page is open
    std::string status, state_path, last_json;
    std::vector<std::string> overridden;
    char save_name[64] = "";
    bool want_replace_popup = false, need_front = false;
    double save_timer = 0;
    // the world, set every frame by the caller (where "Deploy as new" puts the vehicle)
    Vec3f cam_pos = {0, 0, 0}; float cam_yaw = 0; const Terrain* terrain = nullptr;

    void init(Fleet* f, bool restore_open) {
        fleet = f; fleet->refresh_designs();
        state_path = paths::config_dir() + "/designer_state.json";                         // next to the designs folder, under the project root
        fleet->open_designer = [this](int idx) { open_with(idx); };
        std::string err; if (st.load(state_path, err)) {
            ensure_ui();
            design::DesignIn d; if (!st.design_json.empty() && design::from_json(st.design_json, d)) { ui->d = d; ui->recompute(); }
            pause_sim = st.pause_while_open; src.real_base = st.real_base; src.overrides_on = st.overrides_on; src.loaded = st.lib_name; result_open = st.result_open; ui->tab_req_ = st.tab;
            int i = fleet->lib.find(st.lib_name); if (i >= 0) fleet->lib_sel = i;
            std::snprintf(save_name, sizeof save_name, "%s", ui->d.name.c_str()); refresh_overrides();
            if (restore_open && st.open) { open = true; need_front = true; }
        }
    }
    void ensure_ui() {
        if (ui) return;
        ui.reset(new DesignerUI(fleet->design_dir.empty() ? fleet->find_design_dir() : fleet->design_dir));
        ui->lib_hook = [this]() { return library_row(); };
        { std::string nm = safe_name(ui->d.name); for (char& c : nm) if (c == ' ') c = '_'; int i = fleet->lib.find(nm); if (i < 0) i = fleet->lib.find(ui->d.name); if (i >= 0) { fleet->lib_sel = i; src.loaded = fleet->lib.items[i].name; } }
        std::snprintf(save_name, sizeof save_name, "%s", ui->d.name.c_str()); refresh_overrides();
    }
    bool is_open() const { return open; }
    void toggle() { if (open) close(); else open_with(-1); apply_pause(); }
    void apply_pause() { if (fleet) fleet->paused = open && pause_sim; }
    // F2 from the key event: not while a text field has the keyboard (typing a name must not toggle the page). Returns true if it toggled.
    bool hotkey_f2() { if (ImGui::GetIO().WantTextInput) return false; toggle(); return true; }
    // Escape closes the page only when no text field is active (Escape then just ends the text edit)
    bool escape_closes() const { ImGuiIO& io = ImGui::GetIO(); return open && ImGui::IsKeyPressed(ImGuiKey_Escape, false) && !io.WantTextInput && !ImGui::IsAnyItemActive() && !ImGui::IsPopupOpen("", ImGuiPopupFlags_AnyPopupId); }
    void open_with(int idx) {
        ensure_ui(); if (!open) need_front = true; open = true;
        if (idx >= 0) { fleet->lib_sel = idx; load_selected(); }
    }
    void close() { open = false; save_state(); }
    void refresh_overrides() {
        overridden.clear(); if (!ui) return;
        if (src.real_base >= 0) { design::DesignOut o; dcore::build_spec(ui->d, src, &o, &overridden); }
        ui->overridden = overridden; last_json = design::to_json(ui->d);
    }
    void save_state() {
        if (!ui) return;
        st.open = open; st.result_open = result_open; st.pause_while_open = pause_sim; st.designs_dir = fleet->design_dir; st.design_json = design::to_json(ui->d); st.real_base = src.real_base; st.overrides_on = src.overrides_on;
        st.lib_name = src.loaded; std::string err; st.save(state_path, err);
    }

    // load the selected library entry into the editor
    bool load_selected() {
        design::DesignIn d; dcore::Source s; std::string err;
        if (fleet->lib.items.empty()) fleet->refresh_designs();
        if (!dcore::load_entry(fleet->lib, fleet->lib_sel, d, s, err)) { status = err; return false; }
        ui->d = d; src = s; ui->recompute(); std::snprintf(save_name, sizeof save_name, "%s", d.name.c_str()); refresh_overrides();
        status = s.real_base >= 0 ? "Loaded " + s.loaded + " as an editable copy; its published numbers override the designer's results (listed above the result tabs). Save writes a new design file; the real data is never changed."
                                  : "Loaded " + s.loaded + ".";
        return true;
    }

    // the library row inside the designer's control panel (replaces the plain preset list)
    bool library_row() {
        bool ch = false; Fleet& F = *fleet; if (F.lib.items.empty()) F.refresh_designs();
        auto getter = [](void* d, int i, const char** out) { *out = ((std::vector<dcore::LibEntry>*)d)->at(i).label.c_str(); return true; };
        ImGui::TextDisabled("%s", F.lib.summary().c_str());
        if (ImGui::IsItemHovered()) ImGui::SetTooltip("Designs folder:\n%s\n%d file(s) skipped (not designs).\nAfter the saved designs: Tethys, LAUV, ECA A9 and REMUS as editable copies, and BlueROV2 (deploy only).", F.lib.dir.c_str(), F.lib.skipped);
        ImGui::SetNextItemWidth(-60); if (ImGui::Combo("##lib", &F.lib_sel, getter, &F.lib.items, (int)F.lib.items.size(), 12)) ch |= load_selected();
        ImGui::SameLine(); if (ImGui::Button("Reload")) { F.refresh_designs(); ch |= load_selected(); ui->relist(); }
        if (ImGui::IsItemHovered()) ImGui::SetTooltip("Saved designs from %s and the real vehicles that go through the Designer. BlueROV2 is deploy-only.", F.design_dir.c_str());
        if (src.real_base >= 0) {
            ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.72f, 0.3f, 1));
            if (ImGui::Checkbox("Use the published numbers of this real vehicle", &src.overrides_on)) { refresh_overrides(); ch = true; }
            ImGui::PopStyleColor(); if (ImGui::IsItemHovered()) ImGui::SetTooltip("On: mass, inertia, added mass, damping, fins and thrusters of the real vehicle replace the designer's results.\nOff: the designer's own numbers.");
        }
        ImGui::SetNextItemWidth(-60); ImGui::InputText("##sname", save_name, sizeof save_name);
        ImGui::SameLine();
        if (ImGui::Button("Save")) {
            std::string nm = safe_name(save_name), err;
            if (nm.empty()) status = "Give the design a name first.";
            else { ui->d.name = save_name; if (design::save_design(F.design_dir + "/" + nm + ".json", ui->d)) {
                    bool was_real = src.real_base >= 0; src = dcore::Source{}; F.refresh_designs(); ui->relist(); int i = F.lib.find(nm); if (i >= 0) { F.lib_sel = i; src.loaded = nm; } refresh_overrides();
                    status = "Saved " + nm + ".json" + (was_real ? ". A saved design does not carry the published numbers: it uses the designer's own." : ".") + " The real vehicle data was not changed."; }
                else status = "Could not write " + F.design_dir + "/" + nm + ".json"; }
        }
        if (ImGui::IsItemHovered()) ImGui::SetTooltip("Saves a NEW design file %s/<name>.json (never the real vehicle's data).", F.design_dir.c_str());
        return ch;
    }

    // ------------------------------------------------------------------------------------------------ drawing
    // call once per frame near the end of the GUI. in_cockpit: the Cockpit toolbar has its own button.
    void draw(bool in_cockpit) {
        ImGuiIO& io = ImGui::GetIO();
        fleet->input_blocked = open; apply_pause();
        if (!open) {
            if (!in_cockpit) {                                           // the small button
                ImGui::SetNextWindowPos(ImVec2(6, 6)); ImGui::SetNextWindowSize(ImVec2(112, 0)); ImGui::SetNextWindowBgAlpha(0.8f);
                ImGui::Begin("##designer_btn", nullptr, ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoNav);
                if (ImGui::Button("Designer  F2", ImVec2(96, 0))) open_with(-1);
                ImGui::SetWindowFontScale(1.0f); ImGui::End();
                if (ImGuiWindow* w = ImGui::FindWindowByName("##designer_btn")) ImGui::BringWindowToDisplayFront(w);
            }
            if (test.running) test.advance(2.0);                         // a test keeps running in the background if the page was closed
            return;
        }
        ensure_ui();
        if (test.running) test.advance(5.0);                              // 5 s of simulated time per frame: the 130 s test takes under a second
        if (escape_closes()) { close(); apply_pause(); return; }

        float W = io.DisplaySize.x, H = io.DisplaySize.y, bar = 74;
        Unit* u = fleet->act();
        // ---- top bar
        ImGui::SetNextWindowPos(ImVec2(0, 0)); ImGui::SetNextWindowSize(ImVec2(W, bar)); ImGui::SetNextWindowBgAlpha(1.0f);
        ImGui::Begin("##designerbar", nullptr, ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoNav);
        // which vehicle is selected
        if (u) {
            const char* what = u->kind == 1 ? "designed" : (u->kind == 2 ? (u->real_id == RV_BLUEROV2 ? "real, deploy-only" : "real, as published") : "standard model");
            ImGui::Text("Selected vehicle: #%d  %s  (%s)", u->id, u->veh.s.name.c_str(), what);
        } else ImGui::TextDisabled("No vehicle in the water.");
        ImGui::SameLine(0, 24);
        bool can_apply = u && (u->kind == 1 || (u->kind == 2 && u->real_id != RV_BLUEROV2));
        ImGui::BeginDisabled(!can_apply);
        if (ImGui::Button("Apply to deployed AUV") && u) { fleet->apply_design(*u, ui->d, src); status = fleet->msg; }
        ImGui::EndDisabled();
        if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) ImGui::SetTooltip(can_apply ? "Rebuilds the selected vehicle from this design. Position, orientation, speed and depth stay; the battery keeps its percentage of charge."
                                                                                     : "The selected vehicle is not a designed one. Use 'Replace selected vehicle...' (asks first) or 'Deploy as new'.");
        if (u && !can_apply) { ImGui::SameLine(); if (ImGui::Button("Replace selected vehicle...")) { want_replace_popup = true; } }
        bool closing = false;
        ImGui::SameLine(); if (ImGui::Button("Deploy as new")) {
            if (terrain && fleet->deploy_design(ui->d, cam_pos, cam_yaw, *terrain, src)) { status = fleet->msg; closing = true; } else status = fleet->msg; }
        ImGui::SameLine(); if (ImGui::Button("Test this design")) {
            design::DesignOut o; Spec sp = dcore::build_spec(ui->d, src, &o); test.start(sp, ui->d, o.rho_water > 0 ? o.rho_water : 1025.0); result_open = true; }
        ImGui::SameLine(0, 40); if (ImGui::Button("Close  (Esc)")) closing = true;
        ImGui::SameLine(0, 24); if (ImGui::Checkbox("Pause simulation while open", &pause_sim)) { apply_pause(); save_state(); }
        ImGui::TextDisabled("F2 opens and closes this page. Closed, the keys drive the vehicle again.");
        if (!status.empty()) { ImGui::SameLine(0, 16); ImGui::TextColored(ImVec4(0.6f, 0.9f, 1, 1), "%s", status.c_str()); }
        if (want_replace_popup) { ImGui::OpenPopup("Replace vehicle?"); want_replace_popup = false; }
        ImGui::SetNextWindowPos(ImVec2(W * 0.5f, H * 0.4f), ImGuiCond_Always, ImVec2(0.5f, 0.5f));
        if (ImGui::BeginPopupModal("Replace vehicle?", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
            Unit* v = fleet->act();
            if (v) ImGui::Text("Replace #%d  %s  (not a designed vehicle) with this design?\nPosition, orientation and depth are kept; the vehicle becomes the design.", v->id, v->veh.s.name.c_str());
            if (ImGui::Button("Replace", ImVec2(120, 0)) && v) { fleet->apply_design(*v, ui->d, src); status = fleet->msg; ImGui::CloseCurrentPopup(); }
            ImGui::SameLine(); if (ImGui::Button("Cancel", ImVec2(120, 0))) ImGui::CloseCurrentPopup();
            ImGui::EndPopup();
        }

        ImGui::End();
        if (closing) { close(); apply_pause(); return; }

        // ---- the designer page below the bar
        ImGui::SetNextWindowBgAlpha(1.0f); ui->draw(0, bar, W, H - bar);
        ui->tab_req_ = -1;
        if (need_front) {                 // once, when the page opens: over the Cockpit, the setup pages and the Vehicles window. NOT every frame: that would also push this window
                                          // above its own open drop-down lists and dialogs (the library list was invisible because of exactly that)
            auto front = [](const char* n) { if (ImGuiWindow* w = ImGui::FindWindowByName(n)) ImGui::BringWindowToDisplayFront(w); };
            front("AUV Designer"); front("##designerbar"); need_front = false;
        }
        { std::string j = design::to_json(ui->d); if (j != last_json) { refresh_overrides(); save_timer = 1.5; } }
        if (save_timer > 0) { save_timer -= io.DeltaTime; if (save_timer <= 0) save_state(); }
        if (result_open) draw_result();
    }

    void draw_result() {
        ImGuiIO& io = ImGui::GetIO(); const design::DesignOut& o = ui->o; const dcore::TestResult& r = test.r;
        ImGui::SetNextWindowPos(ImVec2(io.DisplaySize.x - 470, io.DisplaySize.y - 300), ImGuiCond_FirstUseEver); ImGui::SetNextWindowSize(ImVec2(460, 270), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowBgAlpha(0.95f);
        if (!ImGui::Begin("Design test (calm water)", &result_open)) { ImGui::End(); return; }
        if (test.running) { ImGui::ProgressBar((float)test.progress(), ImVec2(-1, 0), test.phase == 1 ? "30 s at full surge" : "cruise at the design speed"); ImGui::TextDisabled("Runs a few simulated seconds per frame; the window stays usable."); }
        else if (!r.done) ImGui::TextDisabled("Press 'Test this design'.");
        if (r.done) {
            auto pct = [](double a, double b) { return b != 0 ? 100.0 * (a - b) / std::fabs(b) : 0.0; };
            if (ImGui::BeginTable("res", 4, ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchProp)) {
                ImGui::TableSetupColumn("Quantity"); ImGui::TableSetupColumn("Measured"); ImGui::TableSetupColumn("Designer"); ImGui::TableSetupColumn("Diff"); ImGui::TableHeadersRow();
                auto row = [&](const char* name, double m, double dsg, const char* unit, bool known = true) {
                    ImGui::TableNextRow(); ImGui::TableNextColumn(); ImGui::TextUnformatted(name); ImGui::TableNextColumn(); ImGui::Text("%.2f %s", m, unit);
                    ImGui::TableNextColumn(); if (known) ImGui::Text("%.2f %s", dsg, unit); else ImGui::TextDisabled("-");
                    ImGui::TableNextColumn(); if (known) { double d = pct(m, dsg); ImGui::TextColored(std::fabs(d) > 10 ? ImVec4(1, 0.6f, 0.3f, 1) : ImVec4(0.5f, 0.9f, 0.5f, 1), "%+.1f %%", d); } };
                row("Top speed (full surge)", r.v_top, o.v_top, "m/s");
                row("Main propeller power", r.P_main, o.P_elec_max, "W");
                row("Energy used in 30 s", r.E_full_Wh, (o.P_elec_max + o.hotel_total) * 30.0 / 3600.0, "Wh");
                row("Speed held at cruise", r.v_cruise, ui->d.v_cruise, "m/s");
                row("Propulsion power at cruise", r.P_cruise, o.P_elec_cruise, "W");
                row("Endurance from the drain", r.endurance_h, o.endurance_h, "h");
                ImGui::EndTable();
            }
            ImGui::TextWrapped("Isolated run in calm water (the vehicles in the sea are not touched). Cruise is level flight with neutral buoyancy. %s",
                               src.real_base >= 0 && src.overrides_on ? "The published numbers of the real vehicle are applied; the 'Designer' column is the designer's own prediction." : "");
        }
        ImGui::End();
    }
};
