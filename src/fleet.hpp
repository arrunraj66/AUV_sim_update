// fleet.hpp - deploys vehicles into the sea, drives them (keyboard / gamepad / GUI sliders), draws them, and shows telemetry.
#pragma once
#include "shaders.hpp"
#include "models.hpp"
#include "vmesh.hpp"
#include "vshader.hpp"
#include "water.hpp"
#include "waves.hpp"
#include "terrain.hpp"
#include "imgui.h"
#include "design_model.hpp"
#include "real_models_mesh.hpp"
#include "designer_core.hpp"
#include "remus100.hpp"
#include <functional>
#include <filesystem>
#include <cstring>
#include <cfloat>

struct Unit {
    Vehicle veh; int model = 0; int stl = -1;        // stl = index of its own 3D model when it is a custom STL vehicle
    double ui[5] = {0, 0, 0, 0, 0};                  // slider commands (they stay where you put them)
    int id = 0;
    bool designed = false;                           // built from the AUV Designer or from real data (its 3D model is in stl_models too)
    int kind = 0;                                    // 0 standard model or STL, 1 designed (editable in the Designer), 2 real vehicle as published
    int real_id = -1;                                // kind 1 or 2: the real vehicle it started from
};

struct Fleet {
    std::vector<Unit> units; int active = -1, next_id = 1;
    VModel models[MODEL_COUNT]; std::vector<VModel> stl_models;
    GLuint prog = 0;
    int deploy_model = 0; float deploy_depth = 6.0f;
    int cam_mode = 0;                       // 0 free camera, 1 follow (orbit), 2 onboard
    float chase_scale = 1.0f;
    double key[5] = {0, 0, 0, 0, 0};        // keyboard commands (smoothed)
    double pad[5] = {0, 0, 0, 0, 0};        // gamepad commands
    SDL_GameController* pad_dev = nullptr;
    char stl_path[512] = "";
    // designs from the AUV Designer
    std::string design_dir; std::vector<std::string> design_files; int design_sel = 0;
    dcore::Library lib; int lib_sel = 0;     // saved designs and real vehicles (designer_core.hpp)
    std::function<void(int)> open_designer;  // set by the Designer: opens it, with this library entry loaded (-1: as it was)
    bool input_blocked = false;              // the Designer is open: keyboard and gamepad do not drive the vehicle
    bool paused = false;                     // the Designer is open and 'Pause simulation while open' is on: the vehicles do not step
    bool stl_show_act = true;               // draw thrusters and fins on an imported STL model
    bool stl_swap = false, stl_flip = false; float stl_len = 1.0f; float stl_col[3] = {0.72f, 0.75f, 0.80f};
    std::string msg, models_dir;
    bool snap_cam = false; float snap_yaw = 0, snap_pitch = -0.2f;
    double acc = 0; int steps_last = 0;

    void init() {
        prog = make_program(GLSL_COMMON + VEH_VS, GLSL_COMMON + NOISE + LIGHTING + VEH_FS, "vehicles");
        for (int i = 0; i < M_STL; ++i) { Spec s = make_spec_ex(i); models[i] = build_model(i, s); models[i].upload(); }
        namespace fs = std::filesystem;
        const char* bp = SDL_GetBasePath(); fs::path base = bp ? fs::path(bp) : fs::current_path(); if (bp) SDL_free((void*)bp);
        fs::path dir = (base / ".." / "models").lexically_normal();
        std::error_code ec; fs::create_directories(dir, ec);
        if (ec) { dir = fs::current_path() / "models"; fs::create_directories(dir, ec); }
        models_dir = dir.string();
        fs::path stl = dir / "test_auv.stl";
        if (!fs::exists(stl)) write_stl(stl.string(), make_test_sub());
        std::snprintf(stl_path, sizeof stl_path, "%s", stl.string().c_str());
        SDL_InitSubSystem(SDL_INIT_GAMECONTROLLER);
        open_pad();
    }
    void open_pad() {
        if (pad_dev) return;
        for (int i = 0; i < SDL_NumJoysticks(); ++i) if (SDL_IsGameController(i)) { pad_dev = SDL_GameControllerOpen(i); if (pad_dev) break; }
    }
    void on_event(const SDL_Event& e) {
        if (e.type == SDL_CONTROLLERDEVICEADDED) open_pad();
        if (e.type == SDL_CONTROLLERDEVICEREMOVED && pad_dev) { SDL_GameControllerClose(pad_dev); pad_dev = nullptr; for (double& p : pad) p = 0; }
    }
    Unit* act() { return (active >= 0 && active < (int)units.size()) ? &units[active] : nullptr; }
    const VModel& model_of(const Unit& u) const { return u.stl >= 0 ? stl_models[u.stl] : models[u.model]; }

    // ------------------------------------------------------------------ deploy / remove
    bool deploy(int model, Vec3f cam_pos, float cam_yaw, const Terrain& T) {
        if (!MODEL_CHOICE) model = M_TORPEDO;
        Unit u; u.model = model; u.id = next_id++;
        if (model == M_STL) {
            StlInfo info; VModel vm = build_stl(stl_path, stl_swap, stl_flip, stl_len, info);
            if (!info.ok) { msg = info.error; return false; }
            vm.upload();
            Spec s = make_spec(M_SCOUT); double f = std::max(0.2, (double)stl_len / 1.2), m0 = s.mass;
            s.name = "STL: " + std::filesystem::path(stl_path).filename().string(); s.mesh = M_STL;
            s.length = info.size.x; s.width = info.size.y; s.height = info.size.z;
            double vol = info.volume;
            s.mass = vol > 5e-4 ? 1025.0 * vol : m0 * f * f * f; s.trim = 0.005;
            double mr = s.mass / m0;
            for (auto& t : s.thr) { t.pos = t.pos * f; t.max_thrust *= f * f; }
            for (auto& fin : s.fins) { fin.pos = fin.pos * f; fin.area *= f * f; }
            for (int i = 0; i < 3; ++i) { s.I[i] *= mr * f * f; s.A[i] *= mr; s.A[3 + i] *= mr * f * f; s.Dl[i] *= f * f; s.Dq[i] *= f * f; s.Dl[3 + i] *= f * f * f * f; s.Dq[3 + i] *= f * f * f * f; }
            s.rg = s.rg * f; s.lamp_pos = {0.5 * s.length, 0, 0}; s.cam_pos = {0.48 * s.length, 0, 0.05 * s.height};
            s.contact.clear(); double hx = 0.5 * s.length, hy = 0.4 * s.width, hz = 0.5 * s.height;
            for (int a : {-1, 1}) for (int b : {-1, 1}) s.contact.push_back({0.8 * hx * a, hy * b, -hz});
            s.contact.push_back({hx, 0, 0}); s.contact.push_back({-hx, 0, 0});
            s.chase = 3.0 * s.length; s.note = "Your own 3D model with the Scout's six thrusters and the mass of the water it displaces (neutrally buoyant). Same keys as the Scout.";
            u.veh.s = s; stl_models.push_back(std::move(vm)); u.stl = (int)stl_models.size() - 1;
        } else u.veh.s = make_spec_ex(model);
        return finish_deploy(u, cam_pos, cam_yaw, T);
    }
    // place the new vehicle in front of the camera and select it
    bool finish_deploy(Unit& u, Vec3f cam_pos, float cam_yaw, const Terrain& T) {
        u.veh.rebuild();
        double dist = 8.0 + 3.0 * u.veh.s.length;
        double x = cam_pos.x + std::cos(cam_yaw) * dist, y = cam_pos.y + std::sin(cam_yaw) * dist;
        x = std::max(-380.0, std::min(380.0, x)); y = std::max(-380.0, std::min(380.0, y));
        double z = -std::max(0.0, (double)deploy_depth), gz = T.ground_z((float)x, (float)y);
        z = std::max(z, gz + 0.6 + u.veh.s.height);
        z = std::min(z, 0.0);
        u.veh.place({x, y, z}, cam_yaw);
        units.push_back(u); active = (int)units.size() - 1;
        cam_mode = 1; snap_cam = true; snap_yaw = (float)cam_yaw; snap_pitch = -0.22f;
        msg = "Deployed " + u.veh.s.name;
        return true;
    }
    void remove_active() {
        if (!act()) return;
        units.erase(units.begin() + active); active = units.empty() ? -1 : std::min(active, (int)units.size() - 1);
        if (active < 0) cam_mode = 0;
    }

    // ------------------------------------------------------------------ vehicle library: saved designs, real vehicles, deploy and apply
    std::string find_design_dir() const { return paths::designs_dir(); }          // <project root>/config/designs, whatever the working directory is
    void refresh_designs() {
        if (design_dir.empty()) design_dir = find_design_dir();
        paths::migrate_all(design_dir);                                            // designs an older version left in config/auv move here; 6A presets stay
        lib.dir = design_dir; lib.refresh(); design_files.clear();
        for (const dcore::LibEntry& e : lib.items) if (e.kind == dcore::LibEntry::Saved) design_files.push_back(e.name);
        design_sel = std::max(0, std::min(design_sel, (int)design_files.size() - 1)); lib_sel = std::max(0, std::min(lib_sel, (int)lib.items.size() - 1));
    }
    VModel model_for(const design::DesignIn& din, const design::DesignOut& o, const Spec& s, int real_id) const {
        return real_id == RV_BLUEROV2 ? build_hover(s) : design::build_design_model(din, o, s);
    }
    // put a finished vehicle in front of the camera
    bool deploy_built(const Spec& s, VModel vm, int kind, int real_id, Vec3f cam_pos, float cam_yaw, const Terrain& T) {
        Unit u; u.model = real_id == RV_BLUEROV2 ? M_HOVER : M_TORPEDO; u.id = next_id++; u.designed = true; u.kind = kind; u.real_id = real_id;
        u.veh.s = s; vm.upload(); stl_models.push_back(std::move(vm)); u.stl = (int)stl_models.size() - 1;
        return finish_deploy(u, cam_pos, cam_yaw, T);
    }
    // run the designer on a design (with the published overrides of a real base if there is one) and deploy the vehicle with its own 3D model
    bool deploy_design(const design::DesignIn& din, Vec3f cam_pos, float cam_yaw, const Terrain& T, const dcore::Source& src = dcore::Source{}) {
        design::DesignOut o; std::vector<std::string> over; Spec s = dcore::build_spec(din, src, &o, &over);
        if (o.mass <= 0 || o.V <= 0 || o.outline.empty()) { msg = "This design has no hull volume. Open the designer and check the sizes."; return false; }
        bool ok = deploy_built(s, model_for(din, o, s, -1), 1, src.real_base, cam_pos, cam_yaw, T);
        if (ok && !o.warnings.empty()) msg += "   (" + std::to_string(o.warnings.size()) + " design warnings, see the designer)";
        return ok;
    }
    // a real vehicle (REMUS 100 Fossen data, Tethys, LAUV, ECA A9, BlueROV2) built from its published numbers
    bool deploy_real(int id, Vec3f cam_pos, float cam_yaw, const Terrain& T) {
        if (id < 0 || id >= RV_COUNT) return false;
        Spec s = make_real_spec(id); return deploy_built(s, build_real_model(id, s), 2, id, cam_pos, cam_yaw, T);
    }
    bool deploy_library(int idx, Vec3f cam_pos, float cam_yaw, const Terrain& T) {
        if (lib.items.empty()) refresh_designs();
        if (idx < 0 || idx >= (int)lib.items.size()) return false;
        const dcore::LibEntry& e = lib.items[idx];
        if (e.kind == dcore::LibEntry::DeployOnly) return deploy_real(e.real_id, cam_pos, cam_yaw, T);
        design::DesignIn din; dcore::Source src; std::string err;
        if (!dcore::load_entry(lib, idx, din, src, err)) { msg = err; return false; }
        if (e.kind != dcore::LibEntry::Saved) din.name = e.name;     // a deployed real vehicle keeps its real name; a saved design keeps the name inside its file
        return deploy_design(din, cam_pos, cam_yaw, T, src);
    }
    bool deploy_design_file(const std::string& name, Vec3f cam_pos, float cam_yaw, const Terrain& T) {
        if (lib.items.empty()) refresh_designs();
        int i = lib.find(name); if (i < 0) { msg = "Could not find the design " + name; return false; }
        return deploy_library(i, cam_pos, cam_yaw, T);
    }
    // "Apply to deployed AUV": the selected vehicle gets the new Spec and mesh, keeping position, orientation, speed and depth; the battery keeps its percentage
    bool apply_design(Unit& u, const design::DesignIn& din, const dcore::Source& src) {
        design::DesignOut o; std::vector<std::string> over; Spec s = dcore::build_spec(din, src, &o, &over);
        if (o.mass <= 0 || o.V <= 0 || o.outline.empty()) { msg = "This design has no hull volume."; return false; }
        VModel vm = model_for(din, o, s, -1); vm.upload();
        if (u.stl >= 0) { stl_models[u.stl].release(); stl_models[u.stl] = std::move(vm); } else { stl_models.push_back(std::move(vm)); u.stl = (int)stl_models.size() - 1; }
        dcore::apply_to_vehicle(u.veh, s); u.model = M_TORPEDO; u.designed = true; u.kind = 1; u.real_id = src.real_base;
        msg = "Applied the design to #" + std::to_string(u.id) + " " + s.name + " (position, orientation, speed and battery percentage kept)."; return true;
    }

    // ------------------------------------------------------------------ input
    void read_input(double dt, bool keyboard_ok) {
        const Uint8* k = SDL_GetKeyboardState(nullptr);
        double tg[5] = {0, 0, 0, 0, 0};
        if (keyboard_ok && !input_blocked) {
            auto ax = [&](SDL_Scancode a, SDL_Scancode b) { return (k[a] ? 1.0 : 0.0) - (k[b] ? 1.0 : 0.0); };
            auto pick = [](double a, double b) { return std::fabs(a) > 0 ? a : b; };   // arrows and letters do the same; no cancelling out
            bool letters = cam_mode != 0;                                              // in the free camera W/A/S/D/Q/E fly the camera instead
            tg[0] = pick(ax(SDL_SCANCODE_UP, SDL_SCANCODE_DOWN), letters ? ax(SDL_SCANCODE_W, SDL_SCANCODE_S) : 0);
            tg[1] = pick(ax(SDL_SCANCODE_Z, SDL_SCANCODE_X), letters ? ax(SDL_SCANCODE_Q, SDL_SCANCODE_E) : 0);
            tg[2] = ax(SDL_SCANCODE_R, SDL_SCANCODE_F);
            tg[3] = ax(SDL_SCANCODE_T, SDL_SCANCODE_G);
            tg[4] = pick(ax(SDL_SCANCODE_LEFT, SDL_SCANCODE_RIGHT), letters ? ax(SDL_SCANCODE_A, SDL_SCANCODE_D) : 0);
        }
        for (int i = 0; i < 5; ++i) {
            double step = (tg[i] != 0 ? 3.0 : 5.0) * dt;
            key[i] += std::max(-step, std::min(step, tg[i] - key[i]));
        }
        for (double& p : pad) p = 0;
        if (pad_dev && !input_blocked) {
            auto ax = [&](SDL_GameControllerAxis a) { double v = SDL_GameControllerGetAxis(pad_dev, a) / 32767.0; return std::fabs(v) < 0.12 ? 0.0 : v; };
            pad[0] = -ax(SDL_CONTROLLER_AXIS_LEFTY); pad[4] = -ax(SDL_CONTROLLER_AXIS_LEFTX);
            pad[2] = -ax(SDL_CONTROLLER_AXIS_RIGHTY); pad[1] = -ax(SDL_CONTROLLER_AXIS_RIGHTX);
            pad[3] = (SDL_GameControllerGetButton(pad_dev, SDL_CONTROLLER_BUTTON_DPAD_UP) ? 1 : 0) - (SDL_GameControllerGetButton(pad_dev, SDL_CONTROLLER_BUTTON_DPAD_DOWN) ? 1 : 0);
        }
    }

    // ------------------------------------------------------------------ physics (fixed 200 Hz steps, independent of the frame rate)
    void update(double dt, double sim_time, const Water& water, const Current& cur, const WaveField& waves, const Terrain& T) {
        if (paused) return;                                           // frozen while the Designer is open (see DesignerHub::pause_sim)
        read_input(dt, !ImGui::GetIO().WantCaptureKeyboard);
        for (size_t i = 0; i < units.size(); ++i) {
            Unit& u = units[i]; Vehicle& v = u.veh;
            for (int k = 0; k < 5; ++k) {
                double c = u.ui[k];
                if ((int)i == active) {
                    if (v.s.glider && k >= 2) u.ui[k] = c = std::max(-1.0, std::min(1.0, c + (key[k] + pad[k]) * dt * 0.4));   // glider: keys move the pump / battery slowly and it stays
                    else c += key[k] + pad[k];
                }
                v.cmd[k] = std::max(-1.0, std::min(1.0, c));
            }
        }
        acc = std::min(acc + dt, 0.5);
        const double h = 1.0 / 200.0; int n = 0;
        std::vector<Env> envs(units.size());
        for (size_t i = 0; i < units.size(); ++i) {                     // water density and current at each vehicle (once per frame)
            double depth = std::max(0.0, -units[i].veh.p.z), vx, vy; cur.at(depth, vx, vy);
            envs[i].rho = water.density_at(depth); envs[i].cur = {vx, vy, 0};
            envs[i].ground = [&T](double x, double y) { return (double)T.ground_z((float)x, (float)y); };
        }
        double t0 = sim_time - acc;
        while (acc >= h && n < 120) {
            double tnow = t0 + n * h;
            for (size_t i = 0; i < units.size(); ++i) {
                envs[i].eta = [&waves, tnow](double x, double y) { return waves.elevation(x, y, tnow); };
                units[i].veh.step(h, envs[i]);
            }
            acc -= h; ++n;
        }
        steps_last = n;
        for (Unit& u : units) {                                       // keep the depth history for the GUI plot
            Vehicle& v = u.veh; v.hist_t += dt;
            if (v.hist_t >= 0.25) { v.hist_t = 0; v.hist.push_back((float)-v.p.z); if (v.hist.size() > 240) v.hist.erase(v.hist.begin()); }
            v.p.x = std::max(-390.0, std::min(390.0, v.p.x)); v.p.y = std::max(-390.0, std::min(390.0, v.p.y));
        }
    }

    // ------------------------------------------------------------------ camera, lamp, drawing
    bool follow(float yaw, float pitch, Vec3f& pos) {              // orbit camera around the active vehicle
        Unit* u = act(); if (!u || cam_mode != 1) return false;
        float d = (float)u->veh.s.chase * chase_scale;
        V3 c = u->veh.p;
        Vec3f f = {std::cos(pitch) * std::cos(yaw), std::cos(pitch) * std::sin(yaw), std::sin(pitch)};
        pos = {(float)c.x - f.x * d, (float)c.y - f.y * d, (float)c.z - f.z * d};
        return true;
    }
    bool onboard(Vec3f& eye, Vec3f& fwd, Vec3f& up) {
        Unit* u = act(); if (!u || cam_mode != 2) return false;
        Vehicle& v = u->veh; V3 e = v.p + qrot(v.q, v.s.cam_pos), f = qrot(v.q, {1, 0, 0}), z = qrot(v.q, {0, 0, 1});
        eye = F3(e); fwd = F3(f); up = F3(z); return true;
    }
    bool lamp(Vec3f& pos, Vec3f& dir) {
        Unit* u = act(); if (!u) return false;
        Vehicle& v = u->veh; pos = F3(v.p + qrot(v.q, v.s.lamp_pos)); dir = F3(qrot(v.q, {1, 0, 0})); return true;
    }
    static Mat4 mat_from(Q q, V3 p) {
        V3 ex = qrot(q, {1, 0, 0}), ey = qrot(q, {0, 1, 0}), ez = qrot(q, {0, 0, 1});
        Mat4 m = {}; m.m[0] = (float)ex.x; m.m[1] = (float)ex.y; m.m[2] = (float)ex.z; m.m[4] = (float)ey.x; m.m[5] = (float)ey.y; m.m[6] = (float)ey.z;
        m.m[8] = (float)ez.x; m.m[9] = (float)ez.y; m.m[10] = (float)ez.z; m.m[12] = (float)p.x; m.m[13] = (float)p.y; m.m[14] = (float)p.z; m.m[15] = 1; return m;
    }
    void draw() {
        glDisable(GL_CULL_FACE);
        u3f(prog, "uStlCol", stl_col[0], stl_col[1], stl_col[2]);
        for (const Unit& u : units) {
            const Vehicle& v = u.veh; const VModel& vm = model_of(u);
            u1i(prog, "uMesh", u.model);
            Mat4 body = mat_from(v.q, v.p);
            for (const VPart& p : vm.parts) {
                if (u.stl >= 0 && !u.designed && !stl_show_act && p.kind != 0) continue;
                Mat4 mdl = body;
                if (p.kind != 0) {
                    double ang = 0;
                    if (p.kind == 1 && p.index >= 0 && p.index < (int)v.s.fins.size()) { const Fin& f = v.s.fins[p.index]; ang = v.fin_delta(f); }
                    if (p.kind == 2) ang = v.prop_angle;
                    Q r = qaxis(p.axis, ang); V3 h = p.hinge, hh = h - qrot(r, h);
                    mdl = mul(body, mat_from(r, hh));
                }
                glUniformMatrix4fv(glGetUniformLocation(prog, "uModel"), 1, GL_FALSE, mdl.m);
                glBindVertexArray(p.vao); glDrawElements(GL_TRIANGLES, p.count, GL_UNSIGNED_INT, nullptr);
            }
        }
    }

    // ------------------------------------------------------------------ GUI
    void gui(Vec3f cam_pos, float cam_yaw, const Terrain& T, bool& lamp_on, const Water& water) {
        ImGuiIO& io = ImGui::GetIO();
        ImGui::SetNextWindowPos(ImVec2(io.DisplaySize.x - 440, 52), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(430, 720), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowBgAlpha(0.88f);
        ImGui::Begin("Vehicles");
        ImGui::PushItemWidth(190);
        if (MODEL_CHOICE) ImGui::Combo("Model", &deploy_model, MODEL_NAMES, MODEL_COUNT); else deploy_model = M_TORPEDO;
        ImGui::SliderFloat("Deploy depth (m)", &deploy_depth, 0.0f, 60.0f);
        if (deploy_model == M_STL) {
            ImGui::InputText("STL file", stl_path, sizeof stl_path);
            ImGui::SliderFloat("Length (m)", &stl_len, 0.3f, 6.0f);
            ImGui::Checkbox("File is Y-up", &stl_swap); ImGui::SameLine(); ImGui::Checkbox("Flip front/back", &stl_flip);
            ImGui::ColorEdit3("Paint", stl_col, ImGuiColorEditFlags_NoInputs);
            ImGui::TextDisabled("A ready-made test file is in the models folder (test_auv.stl).");
        }
        if (ImGui::Button("Deploy in front of the camera")) deploy(deploy_model, cam_pos, cam_yaw, T);
        if (ImGui::CollapsingHeader("Vehicle library (saved designs, real vehicles)", ImGuiTreeNodeFlags_DefaultOpen)) {
            if (lib.items.empty()) refresh_designs();
            auto getter = [](void* d, int i, const char** out) { *out = ((std::vector<dcore::LibEntry>*)d)->at(i).label.c_str(); return true; };
            ImGui::Combo("Vehicle", &lib_sel, getter, &lib.items, (int)lib.items.size());
            if (lib_sel >= 0 && lib_sel < (int)lib.items.size()) {
                const dcore::LibEntry& e = lib.items[lib_sel];
                if (e.kind != dcore::LibEntry::Saved) { const real::Info& ri = real::info(e.real_id); ImGui::TextDisabled("%.1f kg, speed %.1f to %.1f m/s, data: %s", ri.mass_pub, ri.v_lo, ri.v_hi, ri.licence);
                    if (ImGui::IsItemHovered()) ImGui::SetTooltip("%s\nEstimated: %s", ri.source, ri.estimated); }
                if (e.kind == dcore::LibEntry::DeployOnly) ImGui::TextDisabled("Deploy only: the Designer cannot edit a vectored-thruster ROV.");
            }
            if (ImGui::Button("Deploy")) deploy_library(lib_sel, cam_pos, cam_yaw, T);
            ImGui::SameLine(); if (ImGui::Button("Designer (F2)")) { if (open_designer) open_designer(lib_sel); }
            ImGui::SameLine(); if (ImGui::SmallButton("Refresh list")) refresh_designs();
        }
        if (!msg.empty()) ImGui::TextColored(ImVec4(0.6f, 0.9f, 1.0f, 1), "%s", msg.c_str());
        if (!units.empty()) {
            ImGui::Separator(); ImGui::Text("In the water (click to control):");
            for (int i = 0; i < (int)units.size(); ++i) {
                char lab[96]; std::snprintf(lab, sizeof lab, "#%d  %s   depth %.1f m", units[i].id, units[i].veh.s.name.c_str(), -units[i].veh.p.z);
                if (ImGui::RadioButton(lab, active == i)) active = i;
            }
        }
        Unit* u = act();
        if (!u) { ImGui::Spacing(); ImGui::TextWrapped("Pick a model and press Deploy. The camera then follows the vehicle."); ImGui::PopItemWidth(); ImGui::End(); return; }
        Vehicle& v = u->veh;
        ImGui::Separator();
        ImGui::Text("Camera:"); ImGui::SameLine();
        ImGui::RadioButton("Free", &cam_mode, 0); ImGui::SameLine(); ImGui::RadioButton("Follow", &cam_mode, 1); ImGui::SameLine(); ImGui::RadioButton("Onboard", &cam_mode, 2);
        if (cam_mode == 1) ImGui::SliderFloat("Follow distance x", &chase_scale, 0.3f, 4.0f);
        ImGui::Checkbox("Vehicle lamp", &lamp_on);
        ImGui::TextWrapped("%s", v.s.note.c_str());
        ImGui::TextDisabled("Keys: W/S or Up/Down surge | A/D or Left/Right turn | Q/E or Z/X sideways | R/F up/down | T/G nose up/down%s", pad_dev ? " | gamepad connected" : "");

        if (ImGui::CollapsingHeader("Commands", ImGuiTreeNodeFlags_DefaultOpen)) {
            const char* names[5] = {"Surge (forward/back)", "Sway (sideways)", "Heave (up/down)", "Pitch (nose up/down)", "Yaw (turn left/right)"};
            if (v.s.glider) { names[2] = "Buoyancy pump (up = lighter)"; names[3] = "Moving battery (up = nose up)"; names[4] = "Battery roll (turn left/right)"; }
            for (int k = 0; k < 5; ++k) { float f = (float)u->ui[k]; if (ImGui::SliderFloat(names[k], &f, -1.0f, 1.0f, "%.2f")) u->ui[k] = f; }
            if (ImGui::Button("Zero all commands")) for (double& c : u->ui) c = 0;
            ImGui::SameLine();
            if (ImGui::Button("Stop and level")) { V3 p = v.p; double yw, pt, rl; v.heading_pitch_roll(yw, pt, rl); v.place(p, yw); for (double& c : u->ui) c = 0; }
            if (v.has_elev || v.has_rud) ImGui::Checkbox("Attitude assist (stick = pitch angle / turn rate)", &v.assist);
        }

        double yw, pt, rl; v.heading_pitch_roll(yw, pt, rl);
        double gz = T.ground_z((float)v.p.x, (float)v.p.y), depth = -v.p.z, alt = v.p.z - gz;
        double hdg = std::fmod(90.0 - yw * 57.29578 + 720.0, 360.0);
        if (ImGui::CollapsingHeader("Telemetry", ImGuiTreeNodeFlags_DefaultOpen)) {
            ImGui::Text("Depth %.2f m   Altitude %.2f m   %s", depth, alt, v.on_bottom ? "ON THE SEA BED" : (v.sub < 0.99 ? (v.sub < 0.05 ? "AIRBORNE" : "AT THE SURFACE") : "submerged"));
            ImGui::Text("Heading %.0f deg   Pitch %+.1f deg   Roll %+.1f deg", hdg, pt * 57.29578, rl * 57.29578);
            ImGui::Text("Speed %.2f m/s (%.1f kn)   surge %+.2f  sway %+.2f  heave %+.2f", len(v.v), len(v.v) * 1.944, v.v.x, v.v.y, v.v.z);
            ImGui::Text("Turn rates (deg/s)  roll %+.1f  pitch %+.1f  yaw %+.1f", v.w.x * 57.29578, v.w.y * 57.29578, v.w.z * 57.29578);
            ImGui::Text("Weight %.1f N   Buoyancy %.1f N   Net %+.2f N   (water %.1f kg/m3)", v.weight, v.buoy, v.buoy - v.weight, v.rho_now);
            if (v.s.glider) ImGui::Text("Pump %+.0f cc   moving mass %+.0f%% / %+.0f%%", v.pump * v.s.pump_range * 1e6, v.mx * 100, v.my * 100);
            if (!v.thrust.empty()) {
                std::string t = "Thrusters (N):"; char b[32];
                for (double f : v.thrust) { std::snprintf(b, sizeof b, " %+.1f", f); t += b; }
                ImGui::TextUnformatted(t.c_str());
            }
            if (v.has_elev || v.has_rud) ImGui::Text("Fins: rudder %+.1f deg, elevator %+.1f deg", v.defl[1] * v.s.fins[0].max_defl * 57.3, v.defl[2] * v.s.fins[0].max_defl * 57.3);
            if (depth > v.s.rated_depth) ImGui::TextColored(ImVec4(1, 0.3f, 0.2f, 1), "Deeper than the %.0f m depth rating!", v.s.rated_depth);
            if (!v.hist.empty()) ImGui::PlotLines("Depth", v.hist.data(), (int)v.hist.size(), 0, "depth history (m)", 0.f, FLT_MAX, ImVec2(0, 60));
        }
        if (ImGui::CollapsingHeader("Tuning (changes the physics live)")) {
            float m = (float)v.s.mass, tr = (float)(v.s.trim * 100), ds = (float)v.drag_scale, am = (float)v.am_scale, ts = (float)v.thrust_scale, cg = (float)v.cg_lift;
            if (ImGui::SliderFloat("Mass (kg)", &m, 1.0f, 400.0f, "%.1f", ImGuiSliderFlags_Logarithmic)) v.s.mass = m;
            if (ImGui::SliderFloat("Net buoyancy (% of weight)", &tr, -5.0f, 5.0f, "%+.2f")) v.s.trim = tr / 100.0;
            if (ImGui::SliderFloat("Drag x", &ds, 0.2f, 3.0f)) v.drag_scale = ds;
            if (ImGui::SliderFloat("Added mass x", &am, 0.2f, 3.0f)) v.am_scale = am;
            if (ImGui::SliderFloat("Thruster power x", &ts, 0.2f, 3.0f)) v.thrust_scale = ts;
            if (ImGui::SliderFloat("Centre of gravity up (m)", &cg, -0.1f, 0.1f)) v.cg_lift = cg;
            ImGui::TextDisabled("Gravity %.5f m/s2, mass and volume give the buoyancy. Heavier water (deep, cold, salty) lifts it more.", 9.80665);
            if (ImGui::Button("Back to model defaults")) { Spec d = make_spec_ex(u->model); if (u->stl < 0) { v.s = d; v.drag_scale = v.am_scale = v.thrust_scale = 1; v.cg_lift = 0; v.rebuild(); } }
        }
        (void)water;
        ImGui::PopItemWidth(); ImGui::End();

        // heads-up display, bottom centre (hidden while the Designer covers the screen)
        if (input_blocked) return;
        char buf[200]; std::snprintf(buf, sizeof buf, "#%d %s   HDG %03.0f   DEPTH %.1f m   ALT %.1f m   SPEED %.2f m/s   PITCH %+.0f", u->id, v.s.name.c_str(), hdg, depth, alt, len(v.v), pt * 57.29578);
        ImVec2 ts = ImGui::CalcTextSize(buf); ImDrawList* dl = ImGui::GetForegroundDrawList();
        ImVec2 a((io.DisplaySize.x - ts.x) * 0.5f, io.DisplaySize.y - ts.y - 24);
        dl->AddRectFilled(ImVec2(a.x - 8, a.y - 5), ImVec2(a.x + ts.x + 8, a.y + ts.y + 5), IM_COL32(0, 0, 0, 150), 5.f);
        dl->AddText(a, IM_COL32(120, 255, 170, 255), buf);
    }
};