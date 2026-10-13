// config_ui.hpp - the configuration pages: World, Other, Sensors, AUV, Deploy.  Three ways to show them (switch live at the top of the screen):
//   Wizard   : one window, a step bar, Back / Next buttons
//   Tabs     : one window, free tabs
//   Setup/Run: full-screen pages, then Start puts the AUV in the water and shows the 3D world
// Every list of settings can be saved and loaded as a named preset (JSON files in the "config" folder next to the program).
#pragma once
#include "config.hpp"
#include "fleet.hpp"
#include "reef.hpp"
#include <functional>
#include <map>

struct WorldRefs {
    Terrain* terrain = nullptr; ReefParams* rp = nullptr; WaveField* waves = nullptr; Water* water = nullptr; Current* current = nullptr;
    float *sun_elev = nullptr, *sun_azim = nullptr, *chop = nullptr, *foam = nullptr; int *sea_state = nullptr, *spectrum_type = nullptr, *terrain_seed = nullptr;
};
struct LiveRefs { bool *lamp_on = nullptr, *particles_on = nullptr; float *lamp_power = nullptr, *cam_speed = nullptr, *time_scale = nullptr; };

// ------------------------------------------------------------------------------------------------ world <-> JSON
inline Json world_to_json(const WorldRefs& w) {
    Json j = Json::object(); auto n = [](double v) { return Json::number(v); };
    j["floor_depth"] = n(w.terrain->floor_depth); j["relief"] = n(w.terrain->relief); j["terrain_seed"] = n(*w.terrain_seed);
    j["rock"] = n(w.rp->rock); j["coral"] = n(w.rp->coral); j["grass"] = n(w.rp->grass); j["reef_half"] = n(w.rp->half); j["reef_seed"] = n(w.rp->seed);
    j["sea_state"] = n(*w.sea_state); j["hs"] = n(w.waves->hs); j["tp"] = n(w.waves->tp); j["wave_dir_deg"] = n(w.waves->dir_deg); j["spectrum"] = n(*w.spectrum_type);
    j["gamma"] = n(w.waves->gamma); j["spread"] = n(w.waves->spread); j["wave_seed"] = n(w.waves->seed); j["chop"] = n(*w.chop); j["whitecaps"] = n(*w.foam);
    Json wa = Json::object(); const Water& W = *w.water;
    wa["surface_temp"] = n(W.surface_temp); wa["deep_temp"] = n(W.deep_temp); wa["thermocline"] = n(W.thermocline); wa["surface_sal"] = n(W.surface_sal);
    wa["deep_sal"] = n(W.deep_sal); wa["halocline"] = n(W.halocline); wa["clarity"] = n(W.clarity); j["water"] = wa;
    Json cu = Json::object(); cu["speed"] = n(w.current->speed); cu["dir_deg"] = n(w.current->dir); cu["decay"] = n(w.current->decay); j["current"] = cu;
    j["sun_elev"] = n(*w.sun_elev); j["sun_azim"] = n(*w.sun_azim); return j;
}
// returns which things changed in a way that needs the sea floor / reef rebuilt
inline void world_from_json(const WorldRefs& w, const Json& j, bool& regen, bool& rebuild_terrain) {
    auto cl = [&](const char* k, double cur, double lo, double hi) { return std::max(lo, std::min(hi, j.num_or(k, cur))); };
    float fd = (float)cl("floor_depth", w.terrain->floor_depth, 2, 6000), rl = (float)cl("relief", w.terrain->relief, 0, 15);
    int ts = (int)j.num_or("terrain_seed", *w.terrain_seed);
    float rk = (float)cl("rock", w.rp->rock, 0, 3), co = (float)cl("coral", w.rp->coral, 0, 3), gr = (float)cl("grass", w.rp->grass, 0, 3), hf = (float)cl("reef_half", w.rp->half, 40, 300);
    int rs = (int)j.num_or("reef_seed", w.rp->seed);
    if (fd != w.terrain->floor_depth || rl != w.terrain->relief || rk != w.rp->rock || co != w.rp->coral || gr != w.rp->grass || hf != w.rp->half || rs != w.rp->seed || ts != *w.terrain_seed) regen = true;
    if (ts != *w.terrain_seed || fd != w.terrain->floor_depth || rl != w.terrain->relief) rebuild_terrain = true;
    w.terrain->floor_depth = fd; w.terrain->relief = rl; *w.terrain_seed = ts; w.rp->rock = rk; w.rp->coral = co; w.rp->grass = gr; w.rp->half = hf; w.rp->seed = rs;
    *w.sea_state = (int)cl("sea_state", *w.sea_state, 0, SEA_STATE_COUNT - 1);
    w.waves->hs = cl("hs", w.waves->hs, 0.01, 16); w.waves->tp = cl("tp", w.waves->tp, 2.8, 20); w.waves->dir_deg = cl("wave_dir_deg", w.waves->dir_deg, 0, 360);
    *w.spectrum_type = (int)cl("spectrum", *w.spectrum_type, 0, 1); w.waves->gamma = cl("gamma", w.waves->gamma, 1, 7); w.waves->spread = cl("spread", w.waves->spread, 1, 20);
    w.waves->seed = (int)j.num_or("wave_seed", w.waves->seed); *w.chop = (float)cl("chop", *w.chop, 0, 1); *w.foam = (float)cl("whitecaps", *w.foam, 0, 2);
    w.waves->build();
    if (const Json* wa = j.find("water")) {
        Water& W = *w.water; auto c2 = [&](const char* k, double cur, double lo, double hi) { return std::max(lo, std::min(hi, wa->num_or(k, cur))); };
        W.surface_temp = c2("surface_temp", W.surface_temp, -2, 35); W.deep_temp = c2("deep_temp", W.deep_temp, -2, 15); W.thermocline = c2("thermocline", W.thermocline, 10, 500);
        W.surface_sal = c2("surface_sal", W.surface_sal, 0, 42); W.deep_sal = c2("deep_sal", W.deep_sal, 0, 42); W.halocline = c2("halocline", W.halocline, 10, 500);
        W.clarity = (int)c2("clarity", W.clarity, 0, CLARITY_COUNT - 1);
    }
    if (const Json* cu = j.find("current")) {
        w.current->speed = std::max(0.0, std::min(3.0, cu->num_or("speed", w.current->speed))); w.current->dir = cu->num_or("dir_deg", w.current->dir);
        w.current->decay = std::max(5.0, std::min(300.0, cu->num_or("decay", w.current->decay)));
    }
    *w.sun_elev = (float)cl("sun_elev", *w.sun_elev, -15, 90); *w.sun_azim = (float)cl("sun_azim", *w.sun_azim, 0, 360);
}

// ------------------------------------------------------------------------------------------------ the pages
struct PresetBar { const char* cat; std::vector<std::string> names; int sel = -1; char name[64] = ""; std::string msg; bool refresh = true; };

struct ConfigUI {
    // wiring (set once from main)
    WorldRefs w; LiveRefs live; Fleet* fleet = nullptr;
    // the settings
    int auv_source = 0; std::string lib_name, lib_cached; double lib_battery = 0, lib_mass = 0, lib_len = 0;   // vehicle source: 0 standard model, 1 library (saved design or real vehicle)
    AuvCfg auv; OtherCfg other; DeployCfg dep; std::vector<Sensor> sensors = sensor_suite(1); int sensor_sel = 0, suite_sel = 1, add_type = 0;
    std::map<int, std::vector<Sensor>> unit_sensors;           // sensors of each deployed vehicle (by vehicle id), for the later steps
    // state of the screen
    int style = 0;                      // 0 wizard, 1 tabs, 2 setup/run
    int page = 0; bool setup_open = true, preview = false;
    bool run_cockpit = false, open_cockpit = true;     // after START show the Cockpit (full-screen 3D with HUD widgets)
    bool regen = false, rebuild_terrain = false;
    std::string config_dir = "config", status; double probe_depth = 20;
    PresetBar pb_world{"world"}, pb_other{"other"}, pb_sensors{"sensors"}, pb_auv{"auv"}, pb_mission{"mission"};
    // snapshot of the camera, passed in each frame
    Vec3f cam_pos{0, 0, 0}; float cam_yaw = 0; Terrain* T = nullptr;

    static constexpr int PAGES = 5;
    const char* page_name(int i) const { static const char* n[PAGES] = {"1  World", "2  Other", "3  Sensors", "4  AUV", "5  Deploy"}; return n[i]; }

    void init(const std::string& dir) {
        config_dir = dir; std::error_code ec; std::filesystem::create_directories(dir, ec);
        if (list_presets(dir + "/world").empty()) seed_world_presets();
    }
    void seed_world_presets() {
        struct P { const char* name; double depth, relief, coral, hs, tp; int state; double st, dt, th, ss, ds; int clarity; double cur; };
        const P ps[] = {{"Shallow tropical reef", 14, 2.5, 1.5, 0.3, 4.0, 2, 29, 4, 150, 35, 34.8, 1, 0.3},
                        {"Open ocean", 2000, 8, 0.0, 3.0, 9.0, 5, 15, 4, 100, 35, 34.9, 0, 0.5},
                        {"Murky harbour", 12, 0.6, 0.0, 0.1, 3.0, 1, 12, 5, 25, 7, 12, 6, 0.1}};
        for (const P& p : ps) {
            Json j = world_to_json(w); j["floor_depth"] = Json::number(p.depth); j["relief"] = Json::number(p.relief); j["coral"] = Json::number(p.coral);
            j["hs"] = Json::number(p.hs); j["tp"] = Json::number(p.tp); j["sea_state"] = Json::number(p.state);
            Json& wa = j["water"]; wa["surface_temp"] = Json::number(p.st); wa["deep_temp"] = Json::number(p.dt); wa["thermocline"] = Json::number(p.th);
            wa["surface_sal"] = Json::number(p.ss); wa["deep_sal"] = Json::number(p.ds); wa["clarity"] = Json::number(p.clarity); j["current"]["speed"] = Json::number(p.cur);
            std::string err; save_json_file(config_dir + "/world/" + p.name + ".json", j, err);
        }
    }

    // ---- preset bar: pick a saved set, load it, or save the current one under a name
    void preset_bar(PresetBar& pb, const std::function<Json()>& get, const std::function<void(const Json&)>& set) {
        std::string dir = config_dir + "/" + pb.cat;
        if (pb.refresh) { pb.names = list_presets(dir); pb.refresh = false; if (pb.sel >= (int)pb.names.size()) pb.sel = -1; }
        ImGui::PushID(pb.cat);
        ImGui::TextDisabled("Saved sets"); ImGui::SameLine();
        ImGui::SetNextItemWidth(190);
        if (ImGui::BeginCombo("##sel", pb.sel >= 0 ? pb.names[pb.sel].c_str() : "(choose)")) {
            for (int i = 0; i < (int)pb.names.size(); ++i) if (ImGui::Selectable(pb.names[i].c_str(), pb.sel == i)) { pb.sel = i; std::snprintf(pb.name, sizeof pb.name, "%s", pb.names[i].c_str()); }
            ImGui::EndCombo();
        }
        ImGui::SameLine();
        if (ImGui::Button("Load") && pb.sel >= 0) {
            Json j; std::string err;
            if (load_json_file(dir + "/" + pb.names[pb.sel] + ".json", j, err)) { set(j); pb.msg = "Loaded " + pb.names[pb.sel]; } else pb.msg = "Could not load: " + err;
        }
        ImGui::SameLine(); if (ImGui::Button("Delete") && pb.sel >= 0) {
            std::error_code ec; std::filesystem::remove(dir + "/" + pb.names[pb.sel] + ".json", ec); pb.msg = "Deleted " + pb.names[pb.sel]; pb.sel = -1; pb.refresh = true;
        }
        ImGui::SetNextItemWidth(190); ImGui::InputTextWithHint("##name", "name for the new set", pb.name, sizeof pb.name);
        ImGui::SameLine();
        if (ImGui::Button("Save as") && pb.name[0]) {
            std::string err, nm = safe_name(pb.name);
            if (!nm.empty() && save_json_file(dir + "/" + nm + ".json", get(), err)) { pb.msg = "Saved " + nm; pb.refresh = true; } else pb.msg = "Could not save: " + err;
        }
        if (!pb.msg.empty()) ImGui::TextColored(ImVec4(0.6f, 0.9f, 1.0f, 1), "%s", pb.msg.c_str());
        ImGui::PopID();
    }

    // ---- 1. World -------------------------------------------------------------------------------------------------
    void page_world() {
        ImGui::PushItemWidth(260);
        preset_bar(pb_world, [&] { return world_to_json(w); }, [&](const Json& j) { world_from_json(w, j, regen, rebuild_terrain); });
        ImGui::Separator();
        Terrain& terrain = *w.terrain; ReefParams& rp = *w.rp; WaveField& waves = *w.waves; Water& water = *w.water; Current& current = *w.current;
        if (ImGui::CollapsingHeader("Sea depth and sea floor", ImGuiTreeNodeFlags_DefaultOpen)) {
            float fd = terrain.floor_depth;
            if (ImGui::InputFloat("Sea level to sea bed (m)", &fd, 5.0f, 50.0f, "%.1f")) { terrain.floor_depth = std::max(2.0f, std::min(6000.0f, fd)); regen = true; rebuild_terrain = true; }
            ImGui::TextDisabled("Type any value from 2 to 6000 m. Coral needs light, so it only grows in shallow water.");
            if (ImGui::SliderFloat("Relief: hills & rocks (m)", &terrain.relief, 0.0f, 15.0f)) { regen = true; }
            if (ImGui::SliderFloat("Rocks", &rp.rock, 0.0f, 3.0f)) regen = true;
            if (ImGui::SliderFloat("Coral", &rp.coral, 0.0f, 3.0f)) regen = true;
            if (ImGui::SliderFloat("Sea grass", &rp.grass, 0.0f, 3.0f)) regen = true;
            if (ImGui::SliderFloat("Reef area, half size (m)", &rp.half, 40.0f, 300.0f)) regen = true;
            if (ImGui::Button("New random sea floor and reef")) { ++*w.terrain_seed; ++rp.seed; rebuild_terrain = true; regen = true; }
        }
        if (ImGui::CollapsingHeader("Waves (sea state, spectrum)")) {
            bool changed = false;
            const char* names[SEA_STATE_COUNT]; for (int i = 0; i < SEA_STATE_COUNT; ++i) names[i] = SEA_STATES[i].name;
            if (ImGui::Combo("WMO sea state", w.sea_state, names, SEA_STATE_COUNT)) { waves.hs = SEA_STATES[*w.sea_state].hs_typ; waves.tp = WaveField::tp_for(waves.hs); changed = true; }
            float hs = (float)waves.hs, tp = (float)waves.tp, dir = (float)waves.dir_deg, gm = (float)waves.gamma, sp = (float)waves.spread;
            if (ImGui::SliderFloat("Hs - significant height (m)", &hs, 0.01f, 16.0f)) { waves.hs = hs; changed = true; }
            if (ImGui::SliderFloat("Tp - peak period (s)", &tp, 2.8f, 20.0f)) { waves.tp = tp; changed = true; }
            if (ImGui::SliderFloat("Wave direction (deg)", &dir, 0.0f, 360.0f)) { waves.dir_deg = dir; changed = true; }
            if (ImGui::Combo("Spectrum", w.spectrum_type, "JONSWAP (growing sea)\0Pierson-Moskowitz (fully developed)\0")) { waves.gamma = *w.spectrum_type == 0 ? 3.3 : 1.0; changed = true; }
            if (*w.spectrum_type == 0 && ImGui::SliderFloat("JONSWAP peak gamma", &gm, 1.0f, 7.0f)) { waves.gamma = gm; changed = true; }
            if (ImGui::SliderFloat("Direction spreading", &sp, 1.0f, 20.0f)) { waves.spread = sp; changed = true; }
            if (ImGui::Button("New random sea")) { waves.seed++; changed = true; }
            ImGui::SameLine(); if (ImGui::Button("Fully developed for this Hs")) { waves.tp = WaveField::tp_for(waves.hs); changed = true; }
            ImGui::SliderFloat("Choppiness", w.chop, 0.0f, 1.0f); ImGui::SliderFloat("Whitecaps", w.foam, 0.0f, 2.0f);
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
            slider("Surface temp (C)", water.surface_temp, -2, 35); slider("Deep temp (C)", water.deep_temp, -2, 15); slider("Thermocline depth (m)", water.thermocline, 10, 500);
            slider("Surface salinity (PSU)", water.surface_sal, 0, 42); slider("Deep salinity (PSU)", water.deep_sal, 0, 42); slider("Halocline depth (m)", water.halocline, 10, 500);
            float pd = (float)probe_depth; if (ImGui::SliderFloat("Probe depth (m)", &pd, 0, 1000)) probe_depth = pd;
            double rho, p; water.state_at(probe_depth, rho, p);
            ImGui::Text("Temperature : %.2f C", water.temperature_at(probe_depth)); ImGui::Text("Salinity    : %.2f PSU", water.salinity_at(probe_depth));
            ImGui::Text("Density     : %.2f kg/m3", rho); ImGui::Text("Pressure    : %.2f bar (%.0f kPa)", p / 1e5, p / 1e3);
            ImGui::Text("Sound speed : %.1f m/s", water.sound_speed_at(probe_depth));
            float prof_t[100], prof_r[100], prof_c[100];
            for (int i = 0; i < 100; ++i) { double d = i * 10.0; prof_t[i] = (float)water.temperature_at(d); prof_r[i] = (float)water.density_at(d); prof_c[i] = (float)water.sound_speed_at(d); }
            ImGui::PlotLines("Temp", prof_t, 100, 0, "0 to 1000 m", FLT_MAX, FLT_MAX, ImVec2(0, 45));
            ImGui::PlotLines("Density", prof_r, 100, 0, "0 to 1000 m", FLT_MAX, FLT_MAX, ImVec2(0, 45));
            ImGui::PlotLines("Sound", prof_c, 100, 0, "0 to 1000 m", FLT_MAX, FLT_MAX, ImVec2(0, 45));
        }
        if (ImGui::CollapsingHeader("Current")) {
            auto slider = [&](const char* label, double& v, float lo, float hi) { float f = (float)v; if (ImGui::SliderFloat(label, &f, lo, hi)) v = f; };
            slider("Surface speed (m/s)", current.speed, 0, 3); slider("Flows toward (deg)", current.dir, 0, 360); slider("Fade depth (m)", current.decay, 5, 300);
            ImGui::TextDisabled("Floating particles and sea grass drift with it.");
        }
        if (ImGui::CollapsingHeader("Light and water clarity")) {
            ImGui::SliderFloat("Sun elevation (deg)", w.sun_elev, -15, 90); ImGui::SliderFloat("Sun direction (deg)", w.sun_azim, 0, 360);
            const char* cn[CLARITY_COUNT]; for (int i = 0; i < CLARITY_COUNT; ++i) cn[i] = CLARITY[i].name;
            ImGui::Combo("Water type", &water.clarity, cn, CLARITY_COUNT);
            const double* K = CLARITY[water.clarity].K;
            ImGui::Text("Red is gone at ~%.0f m, 1%% of green light at ~%.0f m", 4.6 / K[0], 4.6 / K[1]);
        }
        ImGui::PopItemWidth();
    }

    // ---- 2. Other ---------------------------------------------------------------------------------------------------
    void page_other() {
        ImGui::PushItemWidth(260);
        preset_bar(pb_other, [&] { Json j = other_to_json(other); Json l = Json::object();
                                   l["lamp"] = Json::boolean(*live.lamp_on); l["lamp_power"] = Json::number(*live.lamp_power); l["particles"] = Json::boolean(*live.particles_on);
                                   l["camera_speed"] = Json::number(*live.cam_speed); l["time_scale"] = Json::number(*live.time_scale); j["live"] = l; return j; },
                   [&](const Json& j) { other_from_json(j, other); if (const Json* l = j.find("live")) {
                        *live.lamp_on = l->bool_or("lamp", *live.lamp_on); *live.lamp_power = (float)std::max(1.0, std::min(40.0, l->num_or("lamp_power", *live.lamp_power)));
                        *live.particles_on = l->bool_or("particles", *live.particles_on); *live.cam_speed = (float)std::max(0.2, std::min(100.0, l->num_or("camera_speed", *live.cam_speed)));
                        *live.time_scale = (float)std::max(0.0, std::min(5.0, l->num_or("time_scale", *live.time_scale))); } });
        ImGui::Separator();
        if (ImGui::CollapsingHeader("Display and time (work now)", ImGuiTreeNodeFlags_DefaultOpen)) {
            ImGui::Checkbox("Vehicle lamp", live.lamp_on); if (*live.lamp_on) ImGui::SliderFloat("Lamp power", live.lamp_power, 1.0f, 40.0f);
            ImGui::Checkbox("Floating particles", live.particles_on);
            ImGui::SliderFloat("Camera speed (m/s)", live.cam_speed, 0.2f, 100.0f, "%.1f", ImGuiSliderFlags_Logarithmic);
            ImGui::SliderFloat("Time speed", live.time_scale, 0.0f, 5.0f);
        }
        if (ImGui::CollapsingHeader("Water physics around the vehicle (used from Step 5B)", ImGuiTreeNodeFlags_DefaultOpen)) {
            ImGui::TextDisabled("Switch whole effects on and off. They take effect when the new physics library drives the vehicles.");
            ImGui::Checkbox("Propeller wash (jet, swirl)", &other.prop_wake); ImGui::SameLine(160); ImGui::Checkbox("Hull wake", &other.hull_wake);
            ImGui::Checkbox("Ship waves (Kelvin wake)", &other.kelvin); ImGui::SameLine(160); ImGui::Checkbox("Propeller ripples", &other.ripples);
            ImGui::Checkbox("Bubbles", &other.bubbles); ImGui::SameLine(160); ImGui::Checkbox("Stirred-up sediment", &other.sediment);
            ImGui::Checkbox("Forces from sea waves", &other.wave_loads); ImGui::SameLine(160); ImGui::Checkbox("Sea bed / surface suction", &other.wall_effects);
            ImGui::Checkbox("Contact with sea floor and rocks", &other.contact); ImGui::SameLine(160); ImGui::Checkbox("Thermocline layers", &other.stratification);
            ImGui::Checkbox("Tether cable", &other.tether);
            if (other.tether) { ImGui::SliderFloat("Cable length (m)", &other.tether_len, 2, 500, "%.0f", ImGuiSliderFlags_Logarithmic); ImGui::SliderFloat("Cable diameter (m)", &other.tether_diam, 0.002f, 0.05f, "%.3f"); }
        }
        if (ImGui::CollapsingHeader("Simulation and safety")) {
            ImGui::SliderInt("Physics rate (Hz)", &other.physics_hz, 50, 1000); ImGui::SliderFloat("Stop thrusters when no command for (s, 0 = off)", &other.failsafe_s, 0.0f, 30.0f, "%.1f");
            ImGui::TextDisabled("Used from Step 5B.");
        }
        if (ImGui::CollapsingHeader("Data logging")) {
            ImGui::Checkbox("Record vehicle and sensor data to files", &other.log_on); ImGui::InputText("Folder", other.log_dir, sizeof other.log_dir); ImGui::SliderInt("Log rate (Hz)", &other.log_hz, 1, 200);
            ImGui::TextDisabled("Used when the sensors are simulated (a later step).");
        }
        ImGui::PopItemWidth();
    }

    // ---- 3. Sensors -------------------------------------------------------------------------------------------------
    void page_sensors() {
        preset_bar(pb_sensors, [&] { return sensors_to_json(sensors); }, [&](const Json& j) { sensors_from_json(j, sensors); sensor_sel = 0; });
        ImGui::SetNextItemWidth(200); ImGui::Combo("##suite", &suite_sel, "Minimal navigation\0Inspection AUV\0Mapping AUV\0Everything\0"); ImGui::SameLine();
        if (ImGui::Button("Replace list with this suite")) { sensors = sensor_suite(suite_sel); sensor_sel = 0; }
        ImGui::Separator();
        float listw = 250;
        ImGui::BeginChild("sensor_list", ImVec2(listw, 0), true);
        for (int i = 0; i < (int)sensors.size(); ++i) {
            char lab[128]; std::snprintf(lab, sizeof lab, "%s##%d", sensors[i].name.c_str(), i);
            if (!sensors[i].enabled) ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.5f, 0.5f, 0.5f, 1));
            if (ImGui::Selectable(lab, sensor_sel == i)) sensor_sel = i;
            if (!sensors[i].enabled) ImGui::PopStyleColor();
            ImGui::SameLine(listw - 100); ImGui::TextDisabled("%.0f Hz", sensors[i].rate);
        }
        ImGui::Separator();
        const auto& types = sensor_types(); std::vector<const char*> tn; for (auto& t : types) tn.push_back(t.name);
        ImGui::SetNextItemWidth(listw - 20); ImGui::Combo("##addtype", &add_type, tn.data(), (int)tn.size());
        if (ImGui::Button("Add")) {
            int n = 1; for (auto& s : sensors) if (s.type == add_type) ++n;
            sensors.push_back(make_sensor(add_type, std::string(types[add_type].id) + "_" + std::to_string(n))); sensor_sel = (int)sensors.size() - 1;
        }
        ImGui::SameLine(); if (ImGui::Button("Copy") && sensor_sel >= 0 && sensor_sel < (int)sensors.size()) { Sensor c = sensors[sensor_sel]; c.name += "_copy"; sensors.push_back(c); sensor_sel = (int)sensors.size() - 1; }
        ImGui::SameLine(); if (ImGui::Button("Remove") && sensor_sel >= 0 && sensor_sel < (int)sensors.size()) { sensors.erase(sensors.begin() + sensor_sel); sensor_sel = std::min(sensor_sel, (int)sensors.size() - 1); }
        ImGui::EndChild(); ImGui::SameLine();
        ImGui::BeginChild("sensor_edit", ImVec2(0, 0), true);
        if (sensor_sel < 0 || sensor_sel >= (int)sensors.size()) { ImGui::TextDisabled("No sensor selected. Add one on the left."); ImGui::EndChild(); return; }
        Sensor& s = sensors[sensor_sel]; const SensorType& t = types[s.type];
        ImGui::PushItemWidth(240);
        ImGui::TextColored(ImVec4(0.6f, 0.9f, 1.0f, 1), "%s", t.name); ImGui::TextWrapped("%s", t.note);
        char nb[64]; std::snprintf(nb, sizeof nb, "%s", s.name.c_str()); if (ImGui::InputText("Name", nb, sizeof nb)) s.name = nb;
        ImGui::Checkbox("Enabled", &s.enabled);
        float rate = (float)s.rate; if (ImGui::SliderFloat("Update rate (Hz)", &rate, (float)t.rate_lo, (float)t.rate_hi, "%.1f", ImGuiSliderFlags_Logarithmic)) s.rate = rate;
        ImGui::SeparatorText("Mounting on the vehicle (x forward, y left, z up)");
        for (int i = 0; i < 3; ++i) { float f = (float)s.mount[i]; const char* nm[3] = {"x (m)", "y (m)", "z (m)"}; if (ImGui::SliderFloat(nm[i], &f, -3.0f, 3.0f, "%.3f")) s.mount[i] = f; }
        for (int i = 0; i < 3; ++i) { float f = (float)s.mount[3 + i]; const char* nm[3] = {"yaw (deg)", "pitch (deg), + looks up", "roll (deg)"}; if (ImGui::SliderFloat(nm[i], &f, -180.0f, 180.0f, "%.0f")) s.mount[3 + i] = f; }
        ImGui::SeparatorText("Sensor settings");
        for (size_t i = 0; i < t.p.size(); ++i) {
            const PDef& d = t.p[i]; if (s.p.size() <= i) s.p.resize(t.p.size(), d.def);
            char lab[128]; std::snprintf(lab, sizeof lab, d.unit[0] ? "%s (%s)###%s" : "%s###%s", d.label, d.unit, d.key);
            if (d.unit[0]) std::snprintf(lab, sizeof lab, "%s (%s)###%s", d.label, d.unit, d.key); else std::snprintf(lab, sizeof lab, "%s###%s", d.label, d.key);
            if (d.kind == PK_FLOAT) { float f = (float)s.p[i]; int fl = (d.lo > 0 && d.hi / d.lo >= 100) ? ImGuiSliderFlags_Logarithmic : 0; if (ImGui::SliderFloat(lab, &f, (float)d.lo, (float)d.hi, "%.4g", fl)) s.p[i] = f; }
            else if (d.kind == PK_INT) { int n = (int)s.p[i]; if (ImGui::SliderInt(lab, &n, (int)d.lo, (int)d.hi)) s.p[i] = n; }
            else if (d.kind == PK_BOOL) { bool b = s.p[i] != 0; if (ImGui::Checkbox(lab, &b)) s.p[i] = b ? 1 : 0; }
            else { int n = (int)s.p[i]; if (ImGui::Combo(lab, &n, d.items)) s.p[i] = n; }
            if (d.tip[0] && ImGui::IsItemHovered()) ImGui::SetTooltip("%s", d.tip);
        }
        ImGui::PopItemWidth(); ImGui::EndChild();
    }

    // ---- 4. AUV -----------------------------------------------------------------------------------------------------
    void page_auv() {
        ImGui::PushItemWidth(260);
        preset_bar(pb_auv, [&] { return auv_json(); }, [&](const Json& j) { auv_load(j); });
        ImGui::Separator();
        // vehicle source: the standard model, or a design / real vehicle from the library (edited in the Designer, F2)
        ImGui::SeparatorText("Vehicle source");
        ImGui::RadioButton("Standard model", &auv_source, 0); ImGui::SameLine(); ImGui::RadioButton("Designed or real vehicle", &auv_source, 1);
        ImGui::SameLine(); if (ImGui::Button("Design AUV...")) { if (fleet->open_designer) fleet->open_designer(auv_source == 1 ? fleet->lib_sel : -1); }
        if (ImGui::IsItemHovered()) ImGui::SetTooltip("Opens the AUV Designer (also F2 in every mode). Deploy from there, or pick the design here.");
        if (auv_source == 1) {
            if (fleet->lib.items.empty()) fleet->refresh_designs();
            auto getter = [](void* d, int i, const char** out) { *out = ((std::vector<dcore::LibEntry>*)d)->at(i).label.c_str(); return true; };
            if (!lib_name.empty()) { int i = fleet->lib.find(lib_name); if (i >= 0) fleet->lib_sel = i; }
            if (ImGui::Combo("Design", &fleet->lib_sel, getter, &fleet->lib.items, (int)fleet->lib.items.size())) lib_cached.clear();
            if (fleet->lib_sel >= 0 && fleet->lib_sel < (int)fleet->lib.items.size()) {
                const dcore::LibEntry& e = fleet->lib.items[fleet->lib_sel]; lib_name = e.name;
                if (lib_cached != lib_name) {                               // numbers of the chosen vehicle, for the read-only mirrors below
                    lib_cached = lib_name; design::DesignIn din; dcore::Source sc; std::string err;
                    if (e.kind == dcore::LibEntry::DeployOnly) { Spec s = make_real_spec(e.real_id); lib_battery = s.battery_Wh; lib_mass = s.mass; lib_len = s.length; }
                    else if (dcore::load_entry(fleet->lib, fleet->lib_sel, din, sc, err)) { Spec s = dcore::build_spec(din, sc); lib_battery = s.battery_Wh; lib_mass = s.mass; lib_len = s.length; }
                }
                ImGui::Text("Length %.2f m   mass %.1f kg   battery %.0f Wh", lib_len, lib_mass, lib_battery);
                ImGui::TextDisabled("Hull, fins, propeller, sensors' mass and battery come from the design. Edit them in the Designer; they are not repeated on this page.");
            }
        }
        if (auv_source == 1) { ImGui::Spacing(); } else
        if (MODEL_CHOICE) ImGui::Combo("Model", &auv.model, MODEL_NAMES, MODEL_COUNT); else { auv.model = M_TORPEDO; ImGui::Text("Model: %s", MODEL_NAMES[M_TORPEDO]); ImGui::TextDisabled("(other AUV models are paused)"); }
        Spec sp = make_spec_ex(auv.model);
        if (auv_source == 0) ImGui::TextWrapped("%s", sp.note.c_str());
        if (auv_source == 1) {
        } else if (auv.model == M_STL) {
            ImGui::SeparatorText("Your own 3D model");
            ImGui::InputText("STL file", fleet->stl_path, sizeof fleet->stl_path);
            ImGui::SliderFloat("Length (m)", &fleet->stl_len, 0.3f, 6.0f);
            ImGui::Checkbox("File is Y-up", &fleet->stl_swap); ImGui::SameLine(); ImGui::Checkbox("Flip front/back", &fleet->stl_flip);
            ImGui::ColorEdit3("Paint", fleet->stl_col, ImGuiColorEditFlags_NoInputs); ImGui::SameLine(); ImGui::Checkbox("Show thrusters and fins", &fleet->stl_show_act);
            ImGui::TextDisabled("A ready-made test file is in the models folder (test_auv.stl).");
        } else {
            ImGui::SeparatorText("Standard vehicle");
            ImGui::Text("Length %.2f m   width %.2f m   height %.2f m   mass %.1f kg", sp.length, sp.width, sp.height, sp.mass);
        }
        ImGui::SeparatorText(auv_source == 1 ? "Changes to the chosen vehicle" : "Changes to the standard vehicle");
        char nb[64]; std::snprintf(nb, sizeof nb, "%s", auv.name); if (ImGui::InputText("Name", nb, sizeof nb)) std::snprintf(auv.name, sizeof auv.name, "%s", nb);
        ImGui::SliderFloat("Mass and inertia (x)", &auv.mass_scale, 0.5f, 2.0f, "%.2f");
        ImGui::SliderFloat("Drag (x)", &auv.drag_scale, 0.5f, 2.0f, "%.2f");
        ImGui::SliderFloat("Net buoyancy trim (%)", &auv.trim_pct, -10.0f, 10.0f, "%+.1f");
        ImGui::TextDisabled("Mass keeps the vehicle neutrally buoyant (buoyancy follows the mass). Trim > 0 makes it float up, < 0 sink.");
        ImGui::SliderFloat("Thrust (x)", &auv.thrust_scale, 0.3f, 3.0f, "%.2f");
        if (auv_source == 1) { float b = (float)lib_battery; ImGui::BeginDisabled(); ImGui::SliderFloat("Battery (Wh), from the design", &b, 50.0f, 20000.0f, "%.0f", ImGuiSliderFlags_Logarithmic); ImGui::EndDisabled(); }
        else ImGui::SliderFloat("Battery (Wh)", &auv.battery_Wh, 50.0f, 20000.0f, "%.0f", ImGuiSliderFlags_Logarithmic);
        ImGui::TextDisabled("Thrust and battery are used from Step 5B (new thrusters and battery model).");
        ImGui::PopItemWidth();
    }
    Json auv_json() const {
        Json j = auv_to_json(auv); j["source"] = Json::number(auv_source); j["library_entry"] = Json::string(lib_name); j["stl_path"] = Json::string(fleet->stl_path); j["stl_length"] = Json::number(fleet->stl_len);
        j["stl_y_up"] = Json::boolean(fleet->stl_swap); j["stl_flip"] = Json::boolean(fleet->stl_flip); j["stl_show_actuators"] = Json::boolean(fleet->stl_show_act); return j;
    }
    void auv_load(const Json& j) {
        auv_from_json(j, auv, MODEL_COUNT); auv_source = (int)j.num_or("source", 0) != 0; lib_name = j.str_or("library_entry", lib_name); lib_cached.clear();
        std::snprintf(fleet->stl_path, sizeof fleet->stl_path, "%s", j.str_or("stl_path", fleet->stl_path).c_str());
        fleet->stl_len = (float)std::max(0.3, std::min(6.0, j.num_or("stl_length", fleet->stl_len)));
        fleet->stl_swap = j.bool_or("stl_y_up", fleet->stl_swap); fleet->stl_flip = j.bool_or("stl_flip", fleet->stl_flip); fleet->stl_show_act = j.bool_or("stl_show_actuators", fleet->stl_show_act);
    }

    // ---- 5. Deploy --------------------------------------------------------------------------------------------------
    int enabled_sensors() const { int n = 0; for (auto& s : sensors) n += s.enabled; return n; }
    bool has_sensor(int type) const { for (auto& s : sensors) if (s.enabled && s.type == type) return true; return false; }
    double spawn_x() const { return dep.x; }
    void page_deploy() {
        ImGui::PushItemWidth(240);
        preset_bar(pb_mission, [&] { return mission_json(); }, [&](const Json& j) { mission_load(j); });
        ImGui::TextDisabled("A mission file holds all five pages: world, other, sensors, AUV and where to deploy.");
        ImGui::Separator();
        ImGui::SeparatorText("Summary");
        const Water& wt = *w.water; Spec sp = make_spec_ex(auv.model);
        ImGui::BulletText("World: sea bed %.0f m deep, %s, %.1f C at the surface, %s", w.terrain->floor_depth, SEA_STATES[*w.sea_state].name, wt.surface_temp, CLARITY[wt.clarity].name);
        ImGui::BulletText("Current: %.2f m/s toward %.0f deg", w.current->speed, w.current->dir);
        ImGui::BulletText("AUV: %s  (%s)  mass x%.2f, drag x%.2f, trim %+.1f %%", auv.name, auv_source == 1 ? lib_name.c_str() : MODEL_NAMES[auv.model], auv.mass_scale, auv.drag_scale, auv.trim_pct);
        std::string names; for (auto& s : sensors) if (s.enabled) names += (names.empty() ? "" : ", ") + s.name;
        ImGui::BulletText("Sensors: %d enabled", enabled_sensors()); if (!names.empty()) { ImGui::SameLine(); ImGui::TextDisabled("(%s)", names.c_str()); }
        ImGui::SeparatorText("Where to put the vehicle");
        ImGui::Checkbox("In front of the camera", &dep.at_camera);
        double x, y, depth_m, heading;
        if (dep.at_camera) {
            double dist = 8.0 + 3.0 * sp.length; x = cam_pos.x + std::cos(cam_yaw) * dist; y = cam_pos.y + std::sin(cam_yaw) * dist; heading = cam_yaw * 180.0 / M_PI;
            ImGui::SliderFloat("Depth (m)", &dep.depth, 0.0f, 100.0f, "%.1f");
            ImGui::TextDisabled("Position and heading follow the camera: fly the camera first (or uncheck this to type the numbers).");
        } else {
            ImGui::SliderFloat("x east (m)", &dep.x, -380, 380, "%.1f"); ImGui::SliderFloat("y north (m)", &dep.y, -380, 380, "%.1f");
            ImGui::SliderFloat("Depth (m)", &dep.depth, 0.0f, 200.0f, "%.1f"); ImGui::SliderFloat("Heading (deg, 0 = east, 90 = north)", &dep.heading, -180, 360, "%.0f");
            if (ImGui::Button("Use the camera position and heading")) { dep.x = cam_pos.x; dep.y = cam_pos.y; dep.heading = cam_yaw * 180.0f / (float)M_PI; }
            x = dep.x; y = dep.y; heading = dep.heading;
        }
        depth_m = dep.depth;
        double gz = T ? T->ground_z((float)x, (float)y) : -1e9, bed = -gz;
        ImGui::Text("Spawn at x %.1f, y %.1f, heading %.0f deg;  sea bed there is %.1f m deep (%.1f m below the vehicle)", x, y, heading, bed, bed - depth_m);
        ImGui::SeparatorText("Check");
        bool any = false;
        auto warn = [&](bool cond, const char* msg) { if (cond) { ImGui::TextColored(ImVec4(1, 0.8f, 0.3f, 1), "! %s", msg); any = true; } };
        warn(depth_m + 0.5 * sp.height > bed - 0.3, "The vehicle would start on or below the sea bed. It will be lifted to just above it.");
        warn(enabled_sensors() == 0, "No sensor is enabled.");
        warn(!has_sensor(S_IMU) && enabled_sensors() > 0, "No IMU: nothing measures attitude or acceleration.");
        warn(has_sensor(S_GPS) && depth_m > 0.5, "GPS works only at the surface; it will give no fix at this depth.");
        warn(has_sensor(S_LIDAR) && depth_m > 0.5, "LiDAR does not work under water; it will see nothing at this depth.");
        warn(has_sensor(S_DVL) && bed - depth_m > sensor_max_alt(), "The sea bed is beyond the DVL range here: no bottom lock until it comes closer.");
        warn(w.terrain->floor_depth < 2 * sp.length, "The water is shallower than twice the vehicle length.");
        if (!any) ImGui::TextColored(ImVec4(0.5f, 1, 0.6f, 1), "Everything looks fine.");
        ImGui::Spacing();
        bool had = !fleet->units.empty();
        ImGui::Checkbox("Open the Cockpit after START", &open_cockpit);
        if (ImGui::Button(had ? "Replace the vehicle  (start again)" : "START: put the AUV in the water", ImVec2(300, 40))) start(cam_pos, cam_yaw);
        if (had) { ImGui::SameLine(); if (ImGui::Button("Open the Cockpit", ImVec2(150, 40))) run_cockpit = true; }
        if (had) { ImGui::SameLine(); if (ImGui::Button("Remove vehicle", ImVec2(150, 40))) { fleet->units.clear(); fleet->active = -1; fleet->cam_mode = 0; } }
        if (!status.empty()) ImGui::TextColored(ImVec4(0.6f, 0.9f, 1.0f, 1), "%s", status.c_str());
        ImGui::TextDisabled("Sensors are saved with the vehicle and will produce data in a later step. Mass, drag and trim apply now.");
        ImGui::PopItemWidth();
    }
    double sensor_max_alt() const { for (auto& s : sensors) if (s.enabled && s.type == S_DVL) return s.get("max_alt"); return 1e9; }

    // put the configured vehicle into the water
    bool start(Vec3f cp, float cy) {
        fleet->units.clear(); fleet->active = -1;
        fleet->deploy_model = auv.model; fleet->deploy_depth = dep.depth;
        float yaw = dep.at_camera ? cy : dep.heading * (float)M_PI / 180.f;
        if (auv_source == 1) {
            if (fleet->lib.items.empty()) fleet->refresh_designs();
            int li = fleet->lib.find(lib_name); if (li < 0) li = fleet->lib_sel;
            if (!fleet->deploy_library(li, cp, cy, *T)) { status = fleet->msg; return false; }
        } else if (!fleet->deploy(auv.model, cp, cy, *T)) { status = fleet->msg; return false; }
        Unit* u = fleet->act(); if (!u) return false;
        Spec& s = u->veh.s;
        s.name = auv.name; s.mass *= auv.mass_scale; for (int i = 0; i < 3; ++i) s.I[i] *= auv.mass_scale;
        for (int i = 0; i < 6; ++i) { s.Dl[i] *= auv.drag_scale; s.Dq[i] *= auv.drag_scale; }
        s.trim += auv.trim_pct / 100.0;
        u->veh.rebuild();
        if (!dep.at_camera) {
            double x = dep.x, y = dep.y, z = -std::max(0.0, (double)dep.depth), gz = T->ground_z((float)x, (float)y);
            z = std::max(z, gz + 0.6 + s.height); z = std::min(z, 0.0);
            u->veh.place({x, y, z}, yaw); fleet->snap_yaw = yaw;
        }
        unit_sensors[u->id] = sensors;
        status = std::string("Deployed ") + auv.name + " with " + std::to_string(enabled_sensors()) + " sensors.";
        if (style == 2) { setup_open = false; preview = false; }
        if (open_cockpit) run_cockpit = true;
        return true;
    }

    // ---- a whole mission in one file
    Json mission_json() const {
        Json j = Json::object(); j["world"] = world_to_json(w); j["auv"] = auv_json(); j["other"] = other_to_json(other);
        Json l = Json::object(); l["lamp"] = Json::boolean(*live.lamp_on); l["lamp_power"] = Json::number(*live.lamp_power); l["particles"] = Json::boolean(*live.particles_on);
        l["camera_speed"] = Json::number(*live.cam_speed); l["time_scale"] = Json::number(*live.time_scale); j["live"] = l;
        j["sensors"] = sensors_to_json(sensors)["sensors"]; j["deploy"] = deploy_to_json(dep); return j;
    }
    void mission_load(const Json& j) {
        if (const Json* x = j.find("world")) world_from_json(w, *x, regen, rebuild_terrain);
        if (const Json* x = j.find("auv")) auv_load(*x);
        if (const Json* x = j.find("other")) other_from_json(*x, other);
        if (const Json* l = j.find("live")) {
            *live.lamp_on = l->bool_or("lamp", *live.lamp_on); *live.lamp_power = (float)std::max(1.0, std::min(40.0, l->num_or("lamp_power", *live.lamp_power)));
            *live.particles_on = l->bool_or("particles", *live.particles_on); *live.cam_speed = (float)std::max(0.2, std::min(100.0, l->num_or("camera_speed", *live.cam_speed)));
            *live.time_scale = (float)std::max(0.0, std::min(5.0, l->num_or("time_scale", *live.time_scale)));
        }
        sensors_from_json(j, sensors); sensor_sel = 0;
        if (const Json* x = j.find("deploy")) deploy_from_json(*x, dep);
    }

    // ---- the frame ----------------------------------------------------------------------------------------------------
    void draw_page(int p) {
        switch (p) { case 0: page_world(); break; case 1: page_other(); break; case 2: page_sensors(); break; case 3: page_auv(); break; default: page_deploy(); break; }
    }
    bool hides_3d() const { return style == 2 && setup_open && !preview && !run_cockpit; }
    bool show_vehicles_window() const { return !fleet->units.empty() && !hides_3d(); }

    void step_bar() {
        for (int i = 0; i < PAGES; ++i) {
            bool cur = i == page; ImGui::PushStyleColor(ImGuiCol_Button, cur ? ImVec4(0.15f, 0.45f, 0.75f, 1) : ImVec4(0.22f, 0.22f, 0.25f, 1));
            if (ImGui::Button(page_name(i))) page = i;
            ImGui::PopStyleColor(); if (i + 1 < PAGES) ImGui::SameLine();
        }
    }
    void nav_bar(bool big) {
        ImVec2 sz = big ? ImVec2(140, 36) : ImVec2(100, 0);
        if (page > 0 && ImGui::Button("< Back", sz)) --page;
        if (page > 0 && page + 1 < PAGES) ImGui::SameLine();
        if (page + 1 < PAGES && ImGui::Button("Next >", sz)) ++page;
        if (style == 2) { ImGui::SameLine(); if (ImGui::Button("Preview the world in 3D", ImVec2(big ? 220 : 0, big ? 36 : 0))) preview = true; }
    }

    void draw(Vec3f cp, float cy, Terrain& terrain) {
        cam_pos = cp; cam_yaw = cy; T = &terrain;
        ImGuiIO& io = ImGui::GetIO();
        // the style switch, always visible at the top
        ImGui::SetNextWindowPos(ImVec2(io.DisplaySize.x * 0.5f, 6), ImGuiCond_Always, ImVec2(0.5f, 0));
        ImGui::SetNextWindowBgAlpha(0.8f);
        ImGui::Begin("##style", nullptr, ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoSavedSettings);
        ImGui::Text("Configuration style:"); ImGui::SameLine();
        if (ImGui::RadioButton("Wizard", style == 0)) { style = 0; preview = false; } ImGui::SameLine();
        if (ImGui::RadioButton("Tabs", style == 1)) { style = 1; preview = false; } ImGui::SameLine();
        if (ImGui::RadioButton("Setup, then run", style == 2)) { style = 2; setup_open = true; preview = false; }
        if (style == 2 && (!setup_open || preview)) { ImGui::SameLine(); if (ImGui::Button("Back to configuration")) { setup_open = true; preview = false; } }
        ImGui::End();

        if (style == 2) {
            if (!setup_open || preview) return;
            ImGui::SetNextWindowPos(ImVec2(0, 0)); ImGui::SetNextWindowSize(io.DisplaySize); ImGui::SetNextWindowBgAlpha(1.0f);
            ImGui::Begin("##setup", nullptr, ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoBringToFrontOnFocus);
            float avail = io.DisplaySize.x, colw = std::min(1000.0f, avail - 40);
            ImGui::SetCursorPos(ImVec2((avail - colw) * 0.5f, 46)); ImGui::BeginGroup();
            ImGui::SetWindowFontScale(1.5f); ImGui::Text("auvsim - set up the experiment"); ImGui::SetWindowFontScale(1.0f);
            step_bar(); ImGui::Separator();
            ImGui::BeginChild("page", ImVec2(colw, io.DisplaySize.y - 170), false); draw_page(page); ImGui::EndChild();
            ImGui::Separator(); nav_bar(true); ImGui::EndGroup(); ImGui::End();
            return;
        }
        ImGui::SetNextWindowPos(ImVec2(10, 40), ImGuiCond_FirstUseEver); ImGui::SetNextWindowSize(ImVec2(600, 740), ImGuiCond_FirstUseEver); ImGui::SetNextWindowBgAlpha(0.92f);
        ImGui::Begin(style == 0 ? "Configuration (wizard)" : "Configuration (tabs)");
        if (style == 0) {
            step_bar(); ImGui::Separator();
            ImGui::BeginChild("page", ImVec2(0, -ImGui::GetFrameHeightWithSpacing() - 6), false); draw_page(page); ImGui::EndChild();
            ImGui::Separator(); nav_bar(false);
        } else if (ImGui::BeginTabBar("pages")) {
            for (int i = 0; i < PAGES; ++i) if (ImGui::BeginTabItem(page_name(i))) { page = i; ImGui::BeginChild("page", ImVec2(0, 0), false); draw_page(i); ImGui::EndChild(); ImGui::EndTabItem(); }
            ImGui::EndTabBar();
        }
        ImGui::End();
    }
};
