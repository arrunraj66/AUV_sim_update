// Step 3: reef, rocks, free diving camera, adjustable sea depth. Everything from the GUI.
#include "gfx.hpp"
#include "shaders.hpp"
#include "water.hpp"
#include "waves.hpp"
#include "terrain.hpp"
#include "reef.hpp"
#include "fleet.hpp"
#include "config_ui.hpp"
#include "cockpit.hpp"
#include "designer_hub.hpp"
#include "propwash.hpp"
#include "imgui.h"
#include "imgui_impl_sdl2.h"
#include "imgui_impl_opengl3.h"
#include <cstring>
#include <cfloat>
#include <algorithm>
#include <array>

static const float PI = 3.14159265f;

struct Camera {                 // a free-flying camera: it can dive and look anywhere
    Vec3f pos = {-30, 0, 5};
    float yaw = 0.0f, pitch = -0.1f;
    Vec3f forward() const {
        return {std::cos(pitch) * std::cos(yaw), std::cos(pitch) * std::sin(yaw), std::sin(pitch)};
    }
};

int main(int argc, char** argv) {
    // optional test arguments: --view N   --sea N   --shot file.ppm
    int start_view = 0, start_sea = 3; const char* shot = nullptr; int cfg_style = -1, cfg_page = -1; bool cfg_start = false; int ck_preset = -1; bool ck_edit = false;
    int design_n = -1, real_n = -1; bool surge_set = false; const char* gui_shot = nullptr; int gui_shot_frame = 12, esc_at = -1, gframe = 0; int dlib = -1; float chase_arg = -1, camyaw_arg = -999, campitch_arg = -999; std::vector<int> f2_at; std::vector<std::array<int, 3>> click_at; bool open_designer_at_start = false; double surge_cmd = 0, shot_sim = -1, tscale = -1; bool design_done = false; int loop_n = 0;   // test: deploy design N, hold surge, speed up time, shoot at sim time
    for (int i = 1; i < argc; ++i) {
        if (!std::strcmp(argv[i], "--view") && i + 1 < argc) start_view = std::atoi(argv[++i]);
        else if (!std::strcmp(argv[i], "--sea") && i + 1 < argc) start_sea = std::atoi(argv[++i]);
        else if (!std::strcmp(argv[i], "--shot") && i + 1 < argc) shot = argv[++i];
        else if (!std::strcmp(argv[i], "--cfgstyle") && i + 1 < argc) cfg_style = std::atoi(argv[++i]);
        else if (!std::strcmp(argv[i], "--cfgpage") && i + 1 < argc) cfg_page = std::atoi(argv[++i]);
        else if (!std::strcmp(argv[i], "--cfgstart")) cfg_start = true;
        else if (!std::strcmp(argv[i], "--ckpreset") && i + 1 < argc) ck_preset = std::atoi(argv[++i]);
        else if (!std::strcmp(argv[i], "--ckedit")) ck_edit = true;
        else if (!std::strcmp(argv[i], "--design") && i + 1 < argc) design_n = std::atoi(argv[++i]);
        else if (!std::strcmp(argv[i], "--real") && i + 1 < argc) real_n = std::atoi(argv[++i]);
        else if (!std::strcmp(argv[i], "--guishot") && i + 1 < argc) gui_shot = argv[++i];           // test: save the whole window (with the GUI) at frame N and quit
        else if (!std::strcmp(argv[i], "--guishotframe") && i + 1 < argc) gui_shot_frame = std::atoi(argv[++i]);
        else if (!std::strcmp(argv[i], "--f2at") && i + 1 < argc) { for (char* t = std::strtok(argv[++i], ","); t; t = std::strtok(nullptr, ",")) f2_at.push_back(std::atoi(t)); }   // test: press F2 at these frames
        else if (!std::strcmp(argv[i], "--clickat") && i + 1 < argc) { int f = 0, x = 0, y = 0; if (std::sscanf(argv[++i], "%d,%d,%d", &f, &x, &y) == 3) click_at.push_back({f, x, y}); }   // test: click at window pixel (x, y) at frame f
        else if (!std::strcmp(argv[i], "--dlib") && i + 1 < argc) dlib = std::atoi(argv[++i]);            // test: open the Designer with this library entry at frame 4
        else if (!std::strcmp(argv[i], "--chase") && i + 1 < argc) chase_arg = (float)std::atof(argv[++i]);          // test: follow-camera distance multiplier (0.3 = close)
        else if (!std::strcmp(argv[i], "--camyaw") && i + 1 < argc) camyaw_arg = (float)std::atof(argv[++i]);        // test: orbit the follow camera (degrees, relative to the vehicle's heading)
        else if (!std::strcmp(argv[i], "--campitch") && i + 1 < argc) campitch_arg = (float)std::atof(argv[++i]);
        else if (!std::strcmp(argv[i], "--escat") && i + 1 < argc) esc_at = std::atoi(argv[++i]);    // test: press Escape at this frame
        else if (!std::strcmp(argv[i], "--surge") && i + 1 < argc) surge_cmd = std::atof(argv[++i]);
        else if (!std::strcmp(argv[i], "--shotsim") && i + 1 < argc) shot_sim = std::atof(argv[++i]);
        else if (!std::strcmp(argv[i], "--timescale") && i + 1 < argc) tscale = std::atof(argv[++i]);
    }
    if (SDL_Init(SDL_INIT_VIDEO) != 0) { SDL_Log("SDL error: %s", SDL_GetError()); return 1; }
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
    SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
    SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);
    SDL_GL_SetAttribute(SDL_GL_MULTISAMPLEBUFFERS, 1);
    SDL_GL_SetAttribute(SDL_GL_MULTISAMPLESAMPLES, 4);
    const char* title = "auvsim - Step 4: vehicles";
    SDL_Window* win = SDL_CreateWindow(title, SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, 1400, 800,
                                       SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE);
    if (!win) {   // no multisampling: try again without
        SDL_GL_SetAttribute(SDL_GL_MULTISAMPLEBUFFERS, 0); SDL_GL_SetAttribute(SDL_GL_MULTISAMPLESAMPLES, 0);
        win = SDL_CreateWindow(title, SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, 1400, 800,
                               SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE);
    }
    if (!win) { SDL_Log("Window error: %s", SDL_GetError()); return 1; }
    SDL_GLContext gl = SDL_GL_CreateContext(win);
    SDL_GL_SetSwapInterval(1);

    IMGUI_CHECKVERSION(); ImGui::CreateContext(); ImGui::StyleColorsDark();
    ImGui::GetIO().IniFilename = nullptr;
    Cockpit cockpit; cockpit.load_fonts(); Cockpit::apply_theme();
    ImGui_ImplSDL2_InitForOpenGL(win, gl); ImGui_ImplOpenGL3_Init("#version 330");

    // ---- GPU programs and meshes ----
    GLuint prog_sky = make_program(SKY_VS, GLSL_COMMON + SKY_FS, "sky");
    GLuint prog_floor = make_program(GLSL_COMMON + NOISE + TERRAIN + FLOOR_VS,
                                     GLSL_COMMON + NOISE + TERRAIN + LIGHTING + FLOOR_FS, "floor");
    GLuint prog_obj = make_program(GLSL_COMMON + OBJ_VS, GLSL_COMMON + NOISE + LIGHTING + OBJ_FS, "objects");
    GLuint prog_part = make_program(GLSL_COMMON + PART_VS, GLSL_COMMON + PART_FS, "particles");
    GLuint prog_sea = make_program(GLSL_COMMON + SEA_VS, GLSL_COMMON + SEA_FS, "sea");
    GridMesh sea_mesh, floor_mesh; sea_mesh.build(320); floor_mesh.build(256);
    const float SEA_SIZE = 500.f, FLOOR_SIZE = 700.f;
    GLuint empty_vao; glGenVertexArrays(1, &empty_vao);

    // particles
    const int NPART = 6000;
    GLuint part_vao, part_vbo;
    {
        std::vector<float> seeds(NPART * 3); Rng pr(99);
        for (auto& s : seeds) s = pr.next();
        glGenVertexArrays(1, &part_vao); glBindVertexArray(part_vao);
        glGenBuffers(1, &part_vbo); glBindBuffer(GL_ARRAY_BUFFER, part_vbo);
        glBufferData(GL_ARRAY_BUFFER, seeds.size() * sizeof(float), seeds.data(), GL_STATIC_DRAW);
        glEnableVertexAttribArray(0); glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, nullptr);
    }

    // ---- the world ----
    Water water; Current current; WaveField waves;
    int sea_state = start_sea;
    auto apply_sea_state = [&](int s) { waves.hs = SEA_STATES[s].hs_typ; waves.tp = WaveField::tp_for(waves.hs); waves.build(); };
    apply_sea_state(sea_state);
    int spectrum_type = 0;
    float chop = 0.55f, foam_amount = 1.0f;
    float sun_elev = 45.f, sun_azim = 20.f;
    float probe_depth = 20.f, time_scale = 1.0f;
    float cam_speed = 4.0f;
    bool lamp_on = true; float lamp_power = 5.0f; bool particles_on = true;
    double sim_time = 0;

    Terrain terrain; terrain.floor_depth = 14.f; terrain.relief = 2.5f;
    int terrain_seed = 1;
    terrain.build(terrain_seed);
    Reef reef; reef.init();
    ReefParams rp; rp.seed = 1; rp.coral = 1.5f;
    reef.generate(terrain, rp);
    Fleet fleet; fleet.init();
    PropWash wash; wash.init();                         // bubble wake behind working propellers and thrusters
    ConfigUI cfg; cfg.fleet = &fleet;
    cfg.w = {&terrain, &rp, &waves, &water, &current, &sun_elev, &sun_azim, &chop, &foam_amount, &sea_state, &spectrum_type, &terrain_seed};
    cfg.live = {&lamp_on, &particles_on, &lamp_power, &cam_speed, &time_scale};
    cfg.init("config"); cockpit.init("config");
    DesignerHub hub; hub.init(&fleet, !shot && !gui_shot);          // the Designer: top-level overlay, F2, saved state in config/designer_state.json
    if (ck_preset >= 0) cockpit.apply_preset(std::max(0, std::min(3, ck_preset)));
    cockpit.edit = ck_edit;
    if (cfg_style >= 0) cfg.style = std::max(0, std::min(2, cfg_style));
    if (cfg_page >= 0) cfg.page = std::max(0, std::min(ConfigUI::PAGES - 1, cfg_page));

    Camera cam;
    auto find_reef = [&]() {          // fly to the busiest patch of coral
        std::vector<const Inst*> all;
        for (int k = 3; k < 8; ++k) for (auto& i : reef.inst[k]) all.push_back(&i);
        if (all.empty()) { cam.pos = {0, -6, terrain.ground_z(0, 0) + 3}; cam.yaw = 1.57f; cam.pitch = -0.3f; return; }
        int best_n = -1; const Inst* bi = all[0];
        for (size_t a = 0; a < all.size(); a += 3) {
            int n = 0;
            for (auto* o : all) { float dx = o->x - all[a]->x, dy = o->y - all[a]->y; if (dx * dx + dy * dy < 36.f) ++n; }
            if (n > best_n) { best_n = n; bi = all[a]; }
        }
        cam.pos = {bi->x - 6.0f, bi->y - 2.0f, 0}; cam.pos.z = terrain.ground_z(cam.pos.x, cam.pos.y) + 2.5f;
        cam.yaw = std::atan2(2.0f, 6.0f); cam.pitch = -0.25f;
    };
    auto set_view = [&](int v) {
        switch (v) {
            case 0: cam.pos = {-40, 0, 6}; cam.yaw = 0; cam.pitch = -0.08f; break;               // above the sea
            case 1: cam.pos = {-15, 0, 0.5f}; cam.yaw = 0; cam.pitch = 0.0f; break;             // at the waterline
            case 2: cam.pos = {-20, 0, -8}; cam.yaw = 0; cam.pitch = 0.0f; break;                // 8 m down
            case 3: cam.pos = {-20, 0, -0.5f * terrain.floor_depth}; cam.yaw = 0; cam.pitch = -0.1f; break;
            case 4: find_reef(); break;                                                          // at the reef
            case 5: cam.pos = {0, 0, -8}; cam.yaw = 0; cam.pitch = 1.1f; break;                  // look up
        }
    };
    set_view(start_view);

    Uint64 last = SDL_GetPerformanceCounter();
    int frames = 0; bool running = true, dragging = false;
    while (running) {
        Uint64 now = SDL_GetPerformanceCounter();
        float dt = (float)((now - last) / (double)SDL_GetPerformanceFrequency()); last = now;
        dt = std::min(dt, 0.1f);

        ++gframe;
        for (int f : f2_at) if (f == gframe) { SDL_Event ev; SDL_zero(ev); ev.type = SDL_KEYDOWN; ev.key.state = SDL_PRESSED; ev.key.keysym.scancode = SDL_SCANCODE_F2; ev.key.keysym.sym = SDLK_F2; SDL_PushEvent(&ev); }   // test: real key events
        if (gframe == 4 && dlib >= 0) hub.open_with(dlib);
        if (gframe == esc_at) { SDL_Event ev; SDL_zero(ev); ev.type = SDL_KEYDOWN; ev.key.state = SDL_PRESSED; ev.key.keysym.scancode = SDL_SCANCODE_ESCAPE; ev.key.keysym.sym = SDLK_ESCAPE; SDL_PushEvent(&ev); }
        SDL_Event e;
        while (SDL_PollEvent(&e)) {
            ImGui_ImplSDL2_ProcessEvent(&e);
            fleet.on_event(e);
            if (e.type == SDL_KEYDOWN && e.key.keysym.scancode == SDL_SCANCODE_F2 && !e.key.repeat) hub.hotkey_f2();      // ignored while a text field wants the keys
            ImGuiIO& io = ImGui::GetIO();
            if (e.type == SDL_QUIT) running = false;
            if (e.type == SDL_MOUSEBUTTONDOWN && !io.WantCaptureMouse) dragging = true;
            if (e.type == SDL_MOUSEBUTTONUP) dragging = false;
            if (e.type == SDL_MOUSEMOTION && dragging) {
                cam.yaw -= e.motion.xrel * 0.004f;
                cam.pitch = std::max(-1.55f, std::min(1.55f, cam.pitch - e.motion.yrel * 0.004f));
            }
            if (e.type == SDL_MOUSEWHEEL && !io.WantCaptureMouse) {
                if (fleet.cam_mode == 1) fleet.chase_scale = std::max(0.3f, std::min(4.f, fleet.chase_scale * std::pow(0.9f, (float)e.wheel.y)));
                else cam_speed = std::max(0.2f, std::min(200.f, cam_speed * std::pow(1.15f, (float)e.wheel.y)));
            }
        }
        // keyboard: W/S forward/back along where you look, A/D sideways, Space/C (or E/Q) up/down
        if (!ImGui::GetIO().WantCaptureKeyboard && !hub.is_open() && fleet.cam_mode == 0) {
            const Uint8* k = SDL_GetKeyboardState(nullptr);
            float sp = cam_speed * (k[SDL_SCANCODE_LSHIFT] ? 5.f : 1.f) * dt;
            Vec3f f = cam.forward(); Vec3f rt = {std::sin(cam.yaw), -std::cos(cam.yaw), 0};
            if (k[SDL_SCANCODE_W]) cam.pos = vadd(cam.pos, vmul(f, sp));
            if (k[SDL_SCANCODE_S]) cam.pos = vadd(cam.pos, vmul(f, -sp));
            if (k[SDL_SCANCODE_D]) cam.pos = vadd(cam.pos, vmul(rt, sp));
            if (k[SDL_SCANCODE_A]) cam.pos = vadd(cam.pos, vmul(rt, -sp));
            if (k[SDL_SCANCODE_SPACE] || k[SDL_SCANCODE_E]) cam.pos.z += sp;
            if (k[SDL_SCANCODE_C] || k[SDL_SCANCODE_Q]) cam.pos.z -= sp;
        }
        if (!fleet.paused) sim_time += dt * time_scale;
        fleet.update(dt * time_scale, sim_time, water, current, waves, terrain);
        if (!fleet.paused) wash.sim.update(dt * time_scale, fleet, [&](double depth, double& vx, double& vy) { current.at(depth, vx, vy); });
        if (fleet.snap_cam) { cam.yaw = fleet.snap_yaw; cam.pitch = fleet.snap_pitch; fleet.snap_cam = false; if (camyaw_arg > -900) cam.yaw += camyaw_arg * 3.14159265f / 180.f; if (campitch_arg > -900) cam.pitch = campitch_arg * 3.14159265f / 180.f; }
        { Vec3f fp; if (fleet.follow(cam.yaw, cam.pitch, fp)) cam.pos = fp; }

        // keep the camera inside the world and above the sea bed
        cam.pos.x = std::max(-380.f, std::min(380.f, cam.pos.x));
        cam.pos.y = std::max(-380.f, std::min(380.f, cam.pos.y));
        cam.pos.z = std::min(cam.pos.z, 80.0f);
        cam.pos.z = std::max(cam.pos.z, terrain.ground_z(cam.pos.x, cam.pos.y) + 0.6f);

        // ---------------------------------------------------------------- GUI
        bool regen = false, rebuild_terrain = false;
        ImGui_ImplOpenGL3_NewFrame(); ImGui_ImplSDL2_NewFrame();
        for (auto& c : click_at) {   // test: a click through ImGui's input queue (the SDL backend re-reads the real pointer every frame, so the position is queued after it)
            ImGuiIO& tio = ImGui::GetIO();
            if (gframe >= c[0] && gframe <= c[0] + 3) tio.AddMousePosEvent((float)c[1], (float)c[2]);
            if (gframe == c[0] + 1) tio.AddMouseButtonEvent(0, true);
            if (gframe == c[0] + 2) tio.AddMouseButtonEvent(0, false);
        }
        ImGui::NewFrame();
        bool in_ck = cfg.run_cockpit && fleet.act();
        if (cfg.run_cockpit && !fleet.act()) cfg.run_cockpit = false;
        if (in_ck) {
            bool designer_req = false;
            CkCtx ctx{&fleet, &terrain, &current, &water, &cfg.unit_sensors, &lamp_on, &designer_req};
            cockpit.update(dt * time_scale, ctx); cockpit.draw(ctx, sim_time);
            if (designer_req) hub.open_with(-1);
            if (cockpit.want_setup) { cockpit.want_setup = false; cfg.run_cockpit = false; cfg.setup_open = true; cfg.preview = false; }
        } else cfg.draw(cam.pos, cam.yaw, terrain);
        if (cfg.regen) { regen = true; cfg.regen = false; }
        if (cfg.rebuild_terrain) { rebuild_terrain = true; cfg.rebuild_terrain = false; }
        if (cfg_start) { cfg_start = false; cfg.start(cam.pos, cam.yaw); }
        if (tscale >= 0) time_scale = (float)tscale;
        if (chase_arg > 0) fleet.chase_scale = chase_arg;
        if (surge_cmd != 0 && fleet.act() && !surge_set) { fleet.act()->ui[0] = surge_cmd; surge_set = true; }          // test: hold the surge slider on whatever vehicle is deployed
        if ((design_n >= 0 || real_n >= 0) && !design_done && ++loop_n >= 2) {       // test options: deploy the Nth designed AUV, or the Nth real vehicle
            design_done = true; fleet.refresh_designs();
            bool ok = real_n >= 0 ? fleet.deploy_real(real_n, cam.pos, cam.yaw, terrain)
                                  : (design_n < (int)fleet.design_files.size() && fleet.deploy_design_file(fleet.design_files[design_n], cam.pos, cam.yaw, terrain));
            if (ok) { if (fleet.act()) fleet.act()->ui[0] = surge_cmd; }
            else std::fprintf(stderr, "vehicle could not be deployed: %s\n", fleet.msg.c_str());
        }
        bool live_open = in_ck ? cockpit.show_live : !cfg.hides_3d();
        float cdepth = -cam.pos.z;
        if (live_open) {
        ImGui::SetNextWindowPos(ImVec2(ImGui::GetIO().DisplaySize.x - 400, 56), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(390, 620), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowCollapsed(true, ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowBgAlpha(0.88f);
        ImGui::Begin("Live tuning (camera and sea)");
        ImGui::PushItemWidth(210);
        ImGui::Text("%.0f FPS   |   %d reef objects (%d corals)", ImGui::GetIO().Framerate, reef.total(), reef.coral_count());
        ImGui::TextDisabled("Drag mouse = look | W/S/A/D = fly | Space/C = up/down | Shift = fast | wheel = speed");
        ImGui::Text("Go to:"); ImGui::SameLine();
        if (ImGui::SmallButton("Above")) set_view(0); ImGui::SameLine();
        if (ImGui::SmallButton("Waterline")) set_view(1); ImGui::SameLine();
        if (ImGui::SmallButton("-8 m")) set_view(2); ImGui::SameLine();
        if (ImGui::SmallButton("Mid-water")) set_view(3); ImGui::SameLine();
        if (ImGui::SmallButton("Reef")) set_view(4); ImGui::SameLine();
        if (ImGui::SmallButton("Look up")) set_view(5);
        if (ImGui::SliderFloat("Camera depth (m)", &cdepth, -20.0f, terrain.floor_depth + 5.0f)) cam.pos.z = -cdepth;
        ImGui::SliderFloat("Camera speed (m/s)", &cam_speed, 0.2f, 100.0f, "%.1f", ImGuiSliderFlags_Logarithmic);
        ImGui::Checkbox("Vehicle lamp", &lamp_on); ImGui::SameLine();
        ImGui::Checkbox("Floating particles", &particles_on); ImGui::SameLine(); ImGui::Checkbox("Propeller bubbles", &wash.on);
        if (lamp_on) ImGui::SliderFloat("Lamp power", &lamp_power, 1.0f, 40.0f);
        ImGui::SliderFloat("Time speed", &time_scale, 0.0f, 5.0f);

        if (ImGui::CollapsingHeader("Sea depth and sea floor", ImGuiTreeNodeFlags_DefaultOpen)) {
            float fd = terrain.floor_depth;
            if (ImGui::InputFloat("Sea level to sea bed (m)", &fd, 5.0f, 50.0f, "%.1f")) {
                terrain.floor_depth = std::max(2.0f, std::min(6000.0f, fd)); regen = true;
            }
            ImGui::TextDisabled("Type any value from 2 to 6000 m. Coral needs light, so it only grows in shallow water.");
            if (ImGui::SliderFloat("Relief: hills & rocks (m)", &terrain.relief, 0.0f, 15.0f)) regen = true;
            if (ImGui::SliderFloat("Rocks", &rp.rock, 0.0f, 3.0f)) regen = true;
            if (ImGui::SliderFloat("Coral", &rp.coral, 0.0f, 3.0f)) regen = true;
            if (ImGui::SliderFloat("Sea grass", &rp.grass, 0.0f, 3.0f)) regen = true;
            if (ImGui::SliderFloat("Reef area, half size (m)", &rp.half, 40.0f, 300.0f)) regen = true;
            if (ImGui::Button("New random sea floor and reef")) { terrain_seed++; rp.seed++; rebuild_terrain = true; regen = true; }
        }

        if (ImGui::CollapsingHeader("Waves (sea state, spectrum)")) {
            bool changed = false;
            const char* names[SEA_STATE_COUNT]; for (int i = 0; i < SEA_STATE_COUNT; ++i) names[i] = SEA_STATES[i].name;
            if (ImGui::Combo("WMO sea state", &sea_state, names, SEA_STATE_COUNT)) apply_sea_state(sea_state);
            float hs = (float)waves.hs, tp = (float)waves.tp, dir = (float)waves.dir_deg, gm = (float)waves.gamma, sp = (float)waves.spread;
            if (ImGui::SliderFloat("Hs - significant height (m)", &hs, 0.01f, 16.0f)) { waves.hs = hs; changed = true; }
            if (ImGui::SliderFloat("Tp - peak period (s)", &tp, 2.8f, 20.0f)) { waves.tp = tp; changed = true; }
            if (ImGui::SliderFloat("Wave direction (deg)", &dir, 0.0f, 360.0f)) { waves.dir_deg = dir; changed = true; }
            if (ImGui::Combo("Spectrum", &spectrum_type, "JONSWAP (growing sea)\0Pierson-Moskowitz (fully developed)\0")) {
                waves.gamma = spectrum_type == 0 ? 3.3 : 1.0; changed = true; }
            if (spectrum_type == 0 && ImGui::SliderFloat("JONSWAP peak gamma", &gm, 1.0f, 7.0f)) { waves.gamma = gm; changed = true; }
            if (ImGui::SliderFloat("Direction spreading", &sp, 1.0f, 20.0f)) { waves.spread = sp; changed = true; }
            if (ImGui::Button("New random sea")) { waves.seed++; changed = true; }
            ImGui::SameLine(); if (ImGui::Button("Fully developed for this Hs")) { waves.tp = WaveField::tp_for(waves.hs); changed = true; }
            ImGui::SliderFloat("Choppiness", &chop, 0.0f, 1.0f);
            ImGui::SliderFloat("Whitecaps", &foam_amount, 0.0f, 2.0f);
            if (changed) waves.build();
            double u = WaveField::wind_for(waves.hs);
            ImGui::Text("Built-wave Hs : %.2f m  (asked %.2f)", waves.hs_real, waves.hs);
            ImGui::Text("Peak wavelength: %.0f m,  wave speed %.1f m/s", 9.81 * waves.tp * waves.tp / (2 * M_PI), 9.81 * waves.tp / (2 * M_PI));
            ImGui::Text("Wind for fully developed sea: %.1f m/s = %.0f kn = Beaufort %.0f", u, u * 1.944, WaveField::beaufort_for(u));
            float sp_plot[100]; for (int i = 0; i < 100; ++i) sp_plot[i] = (float)waves.spectrum(0.1 + i * 0.03);
            ImGui::PlotLines("S(w)", sp_plot, 100, 0, "wave energy vs frequency", 0, FLT_MAX, ImVec2(0, 70));
        }

        if (ImGui::CollapsingHeader("Water (temperature, salinity, density)")) {
            auto slider = [&](const char* label, double& v, float lo, float hi) { float f = (float)v; if (ImGui::SliderFloat(label, &f, lo, hi)) v = f; };
            if (ImGui::Button("Tropical")) { water.surface_temp = 29; water.deep_temp = 4; water.thermocline = 150; water.surface_sal = 35; water.deep_sal = 34.8; water.halocline = 250; }
            ImGui::SameLine(); if (ImGui::Button("Temperate")) { water.surface_temp = 15; water.deep_temp = 4; water.thermocline = 100; water.surface_sal = 35; water.deep_sal = 34.9; water.halocline = 200; }
            ImGui::SameLine(); if (ImGui::Button("Polar")) { water.surface_temp = 0; water.deep_temp = -1; water.thermocline = 50; water.surface_sal = 33; water.deep_sal = 34.5; water.halocline = 80; }
            ImGui::SameLine(); if (ImGui::Button("Baltic")) { water.surface_temp = 12; water.deep_temp = 5; water.thermocline = 25; water.surface_sal = 7; water.deep_sal = 12; water.halocline = 40; }
            slider("Surface temp (C)", water.surface_temp, -2, 35);
            slider("Deep temp (C)", water.deep_temp, -2, 15);
            slider("Thermocline depth (m)", water.thermocline, 10, 500);
            slider("Surface salinity (PSU)", water.surface_sal, 0, 42);
            slider("Deep salinity (PSU)", water.deep_sal, 0, 42);
            slider("Halocline depth (m)", water.halocline, 10, 500);
            ImGui::SliderFloat("Probe depth (m)", &probe_depth, 0, 1000);
            double rho, p; water.state_at(probe_depth, rho, p);
            ImGui::Text("Temperature : %.2f C", water.temperature_at(probe_depth));
            ImGui::Text("Salinity    : %.2f PSU", water.salinity_at(probe_depth));
            ImGui::Text("Density     : %.2f kg/m3", rho);
            ImGui::Text("Pressure    : %.2f bar (%.0f kPa)", p / 1e5, p / 1e3);
            ImGui::Text("Sound speed : %.1f m/s", water.sound_speed_at(probe_depth));
            float prof_t[100], prof_r[100], prof_c[100];
            for (int i = 0; i < 100; ++i) { double d = i * 10.0; prof_t[i] = (float)water.temperature_at(d); prof_r[i] = (float)water.density_at(d); prof_c[i] = (float)water.sound_speed_at(d); }
            ImGui::PlotLines("Temp", prof_t, 100, 0, "0 to 1000 m", FLT_MAX, FLT_MAX, ImVec2(0, 45));
            ImGui::PlotLines("Density", prof_r, 100, 0, "0 to 1000 m", FLT_MAX, FLT_MAX, ImVec2(0, 45));
            ImGui::PlotLines("Sound", prof_c, 100, 0, "0 to 1000 m", FLT_MAX, FLT_MAX, ImVec2(0, 45));
        }

        if (ImGui::CollapsingHeader("Current")) {
            auto slider = [&](const char* label, double& v, float lo, float hi) { float f = (float)v; if (ImGui::SliderFloat(label, &f, lo, hi)) v = f; };
            slider("Surface speed (m/s)", current.speed, 0, 3);
            slider("Flows toward (deg)", current.dir, 0, 360);
            slider("Fade depth (m)", current.decay, 5, 300);
            double vx, vy; current.at(-cam.pos.z, vx, vy);
            ImGui::Text("At camera depth: %.2f m/s (east %.2f, north %.2f)", std::sqrt(vx * vx + vy * vy), vx, vy);
            ImGui::TextDisabled("Floating particles drift with it.");
        }

        if (ImGui::CollapsingHeader("Light and water clarity")) {
            ImGui::SliderFloat("Sun elevation (deg)", &sun_elev, -15, 90);
            ImGui::SliderFloat("Sun direction (deg)", &sun_azim, 0, 360);
            const char* cn[CLARITY_COUNT]; for (int i = 0; i < CLARITY_COUNT; ++i) cn[i] = CLARITY[i].name;
            ImGui::Combo("Water type", &water.clarity, cn, CLARITY_COUNT);
            const double* K = CLARITY[water.clarity].K;
            ImGui::Text("Light left at camera depth: R %.1f%%  G %.1f%%  B %.1f%%", 100 * std::exp(-K[0] * cdepth), 100 * std::exp(-K[1] * cdepth), 100 * std::exp(-K[2] * cdepth));
            ImGui::Text("Red is gone at ~%.0f m, 1%% of green light at ~%.0f m", 4.6 / K[0], 4.6 / K[1]);
        }
        ImGui::PopItemWidth();
        ImGui::End();
        }
        if (!in_ck && cfg.show_vehicles_window()) fleet.gui(cam.pos, cam.yaw, terrain, lamp_on, water);
        hub.cam_pos = cam.pos; hub.cam_yaw = cam.yaw; hub.terrain = &terrain; hub.draw(in_ck);        // the Designer overlay, over everything else

        if (rebuild_terrain) terrain.build(terrain_seed);
        if (regen) reef.generate(terrain, rp);

        // ---------------------------------------------------------------- 3D scene
        int w, h; SDL_GL_GetDrawableSize(win, &w, &h);

        // dive gauge in the top-right corner
        if (!in_ck && !hub.is_open()) {
            double rho, p; water.state_at(std::max(0.f, cdepth), rho, p);
            char buf[160];
            std::snprintf(buf, sizeof buf, "Depth %.1f m    Altitude %.1f m    %.1f C    %.2f bar",
                          cdepth, cam.pos.z - terrain.ground_z(cam.pos.x, cam.pos.y), water.temperature_at(std::max(0.f, cdepth)), p / 1e5);
            ImVec2 ts = ImGui::CalcTextSize(buf);
            ImDrawList* dl = ImGui::GetForegroundDrawList();
            ImVec2 a(ImGui::GetIO().DisplaySize.x - ts.x - 24, 10);
            dl->AddRectFilled(a, ImVec2(a.x + ts.x + 14, a.y + ts.y + 10), IM_COL32(0, 0, 0, 150), 5.f);
            dl->AddText(ImVec2(a.x + 7, a.y + 5), IM_COL32(255, 255, 255, 255), buf);
        }

        ImGui::Render();
        glViewport(0, 0, w, h);
        glClearColor(0, 0, 0, 1); glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        Vec3f eye = cam.pos;
        Vec3f fwd = cam.forward(); Vec3f cam_up = {0, 0, 1};
        fleet.onboard(eye, fwd, cam_up);
        if (eye.z > -0.05f && eye.z < 0.05f) eye.z = 0.05f;
        float aspect = (float)w / (float)h, fovy = 60.f * PI / 180.f;
        Mat4 vp = mul(perspective(fovy, aspect, 0.1f, 4000.f), look_at(eye, vadd(eye, fwd), cam_up));
        Vec3f right = norm(cross(fwd, cam_up)), up = cross(right, fwd);

        float se = sun_elev * PI / 180.f, sa = sun_azim * PI / 180.f;
        Vec3f sun = {std::cos(se) * std::cos(sa), std::cos(se) * std::sin(sa), std::sin(se)};
        float day = std::min(1.f, std::max(0.f, (sun.z + 0.08f) / 0.33f));
        float warm = std::min(1.f, std::max(0.f, sun.z / 0.5f));
        Vec3f sun_col = {day * 1.5f, day * (0.75f + 0.75f * warm), day * (0.45f + 1.05f * warm) * 0.95f};
        const double* K = CLARITY[water.clarity].K;
        float turb = std::min(16.f, (float)K[1] / 0.05f);
        float bb = 0.004f + 0.004f * turb;
        bool underwater = eye.z < 0;

        auto common = [&](GLuint p) {
            glUseProgram(p);
            u3f(p, "uCam", eye.x, eye.y, eye.z);
            u3f(p, "uSunDir", sun.x, sun.y, sun.z);
            u3f(p, "uSunCol", sun_col.x, sun_col.y, sun_col.z);
            u1f(p, "uDay", day);
            u3f(p, "uK", (float)K[0], (float)K[1], (float)K[2]);
            u3f(p, "uBb", bb, bb, bb);
            u1f(p, "uTime", (float)sim_time);
        };
        auto vp_uniform = [&](GLuint p) { glUniformMatrix4fv(glGetUniformLocation(p, "uVP"), 1, GL_FALSE, vp.m); };
        auto snap = [&](float size, int cells, float v) { float c = size / cells; return std::floor(v / c) * c; };
        Vec3f lamp_p = eye, lamp_d = fwd; bool has_lamp = fleet.lamp(lamp_p, lamp_d);
        auto lamp_uniforms = [&](GLuint p) {
            u1f(p, "uLamp", (lamp_on && (underwater || has_lamp)) ? lamp_power : 0.0f);
            u3f(p, "uLampPos", lamp_p.x, lamp_p.y, lamp_p.z); u3f(p, "uLampDir", lamp_d.x, lamp_d.y, lamp_d.z);
        };
        auto terrain_uniforms = [&](GLuint p) {
            glActiveTexture(GL_TEXTURE0); glBindTexture(GL_TEXTURE_2D, terrain.tex);
            u1i(p, "uTerrain", 0); u1f(p, "uHalf", terrain.half);
            u1f(p, "uFloor", terrain.floor_depth); u1f(p, "uRelief", terrain.relief);
        };

        // 1. sky (or open water when under the surface)
        glDisable(GL_DEPTH_TEST); glDisable(GL_BLEND); glDisable(GL_CULL_FACE);
        common(prog_sky);
        u3f(prog_sky, "uRight", right.x, right.y, right.z); u3f(prog_sky, "uUp", up.x, up.y, up.z); u3f(prog_sky, "uFwd", fwd.x, fwd.y, fwd.z);
        u2f(prog_sky, "uTan", std::tan(fovy / 2) * aspect, std::tan(fovy / 2));
        glBindVertexArray(empty_vao); glDrawArrays(GL_TRIANGLES, 0, 3);

        // 2. sea floor
        glEnable(GL_DEPTH_TEST);
        common(prog_floor); vp_uniform(prog_floor); lamp_uniforms(prog_floor); terrain_uniforms(prog_floor);
        u2f(prog_floor, "uCenter", snap(FLOOR_SIZE, 256, eye.x), snap(FLOOR_SIZE, 256, eye.y));
        u1f(prog_floor, "uSize", FLOOR_SIZE);
        floor_mesh.draw();

        // 3. rocks, corals, grass
        common(prog_obj); vp_uniform(prog_obj); lamp_uniforms(prog_obj);
        reef.draw([&](int kind) {
            u1i(prog_obj, "uKind", kind); u1i(prog_obj, "uSway", kind == 5 ? 1 : 0);
            if (kind == 5) glDisable(GL_CULL_FACE); else glEnable(GL_CULL_FACE);
        });
        glDisable(GL_CULL_FACE);

        // 3b. vehicles
        common(fleet.prog); vp_uniform(fleet.prog); lamp_uniforms(fleet.prog); fleet.draw();
        common(wash.prog); vp_uniform(wash.prog); wash.draw((float)h);                 // 3c. propeller bubbles

        // 4. floating particles
        if (underwater && particles_on) {
            glEnable(GL_PROGRAM_POINT_SIZE); glEnable(GL_BLEND); glBlendFunc(GL_ONE, GL_ONE); glDepthMask(GL_FALSE);
            common(prog_part); vp_uniform(prog_part);
            double vx, vy; current.at(std::max(0.f, -eye.z), vx, vy);
            u1f(prog_part, "uBox", 40.f); u3f(prog_part, "uDrift", (float)vx, (float)vy, 0.03f); u1f(prog_part, "uPx", (float)h);
            glBindVertexArray(part_vao); glDrawArrays(GL_POINTS, 0, NPART);
            glDepthMask(GL_TRUE); glDisable(GL_BLEND);
        }

        // 5. sea surface (transparent from above)
        glEnable(GL_BLEND); glBlendFunc(GL_ONE, GL_ONE_MINUS_SRC_ALPHA);
        common(prog_sea); vp_uniform(prog_sea);
        u2f(prog_sea, "uCenter", snap(SEA_SIZE, 320, eye.x), snap(SEA_SIZE, 320, eye.y));
        u1f(prog_sea, "uSize", SEA_SIZE);
        float wv[32 * 4], ph[32];
        for (int i = 0; i < 32; ++i) { wv[i * 4] = waves.c[i].kx; wv[i * 4 + 1] = waves.c[i].ky; wv[i * 4 + 2] = waves.c[i].omega; wv[i * 4 + 3] = waves.c[i].amp; ph[i] = waves.c[i].phase; }
        u1i(prog_sea, "uNW", 32);
        glUniform4fv(glGetUniformLocation(prog_sea, "uWave"), 32, wv);
        glUniform1fv(glGetUniformLocation(prog_sea, "uPhase"), 32, ph);
        u1f(prog_sea, "uChop", chop);
        double wind = WaveField::wind_for(waves.hs);
        u1f(prog_sea, "uRough", (float)std::min(1.6, 0.25 + wind / 10.0));
        u1f(prog_sea, "uWaveDir", (float)(waves.dir_deg * M_PI / 180.0));
        u1f(prog_sea, "uFoam", foam_amount * (float)std::min(1.0, std::max(0.0, (waves.hs - 0.6) / 3.0)));
        sea_mesh.draw();
        glDisable(GL_BLEND); glDisable(GL_DEPTH_TEST);

        if (shot && (shot_sim >= 0 ? ((design_done || surge_set) && sim_time >= shot_sim) : ++frames == 3)) {      // test mode: save the 3D picture (without the GUI) and quit
            std::vector<unsigned char> px((size_t)w * h * 3);
            glReadPixels(0, 0, w, h, GL_RGB, GL_UNSIGNED_BYTE, px.data());
            FILE* f = std::fopen(shot, "wb"); if (!f) { std::fprintf(stderr, "cannot write %s\n", shot); f = std::fopen("/dev/null", "wb"); } std::fprintf(f, "P6\n%d %d\n255\n", w, h);
            for (int y = h - 1; y >= 0; --y) std::fwrite(&px[(size_t)y * w * 3], 1, (size_t)w * 3, f);
            std::fclose(f); running = false;
            if (Unit* u = fleet.act()) { double yw, pt, rl; u->veh.heading_pitch_roll(yw, pt, rl); std::printf("%s: sim time %.1f s, position (%.1f, %.1f, %.1f), speed %.2f m/s, depth %.1f m, pitch %.1f deg, roll %.1f deg, energy left %.1f Wh\n", u->veh.s.name.c_str(), sim_time, u->veh.p.x, u->veh.p.y, u->veh.p.z, len(u->veh.v), -u->veh.p.z, pt * 57.3, rl * 57.3, u->veh.energy_Wh); }
        }

        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        if (gui_shot && gframe >= gui_shot_frame) {          // test mode: the whole window including the GUI
            std::vector<unsigned char> px((size_t)w * h * 3); glReadPixels(0, 0, w, h, GL_RGB, GL_UNSIGNED_BYTE, px.data());
            FILE* f = std::fopen(gui_shot, "wb"); if (!f) { std::fprintf(stderr, "cannot write %s\n", gui_shot); f = std::fopen("/dev/null", "wb"); } std::fprintf(f, "P6\n%d %d\n255\n", w, h);
            for (int y = h - 1; y >= 0; --y) std::fwrite(&px[(size_t)y * w * 3], 1, (size_t)w * 3, f);
            std::fclose(f); running = false; std::printf("designer %s, in cockpit %d, vehicles %d\n", hub.is_open() ? "OPEN" : "closed", (int)in_ck, (int)fleet.units.size());
        }
        SDL_GL_SwapWindow(win);
    }
    hub.save_state();
    ImGui_ImplOpenGL3_Shutdown(); ImGui_ImplSDL2_Shutdown(); ImGui::DestroyContext();
    SDL_GL_DeleteContext(gl); SDL_DestroyWindow(win); SDL_Quit();
}