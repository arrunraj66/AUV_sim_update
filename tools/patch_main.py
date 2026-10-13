#!/usr/bin/env python3
# Step 6A: wires the configuration pages into src/main.cpp (safe: stops if an anchor is missing, keeps a backup)
import sys, shutil
p = sys.argv[1] if len(sys.argv) > 1 else "src/main.cpp"
s = open(p).read()
if "config_ui.hpp" in s:
    print("already patched"); sys.exit(0)
shutil.copy(p, p + ".bak6a")
def rep(old, new, count=1):
    global s
    assert s.count(old) == count, "anchor not found (%d x): %r" % (s.count(old), old[:60])
    s = s.replace(old, new)
rep('#include "fleet.hpp"\n', '#include "fleet.hpp"\n#include "config_ui.hpp"\n')
# test options
rep('const char* shot = nullptr;', 'const char* shot = nullptr; int cfg_style = -1, cfg_page = -1; bool cfg_start = false;')
rep('        else if (!std::strcmp(argv[i], "--shot") && i + 1 < argc) shot = argv[++i];\n', '        else if (!std::strcmp(argv[i], "--shot") && i + 1 < argc) shot = argv[++i];\n        else if (!std::strcmp(argv[i], "--cfgstyle") && i + 1 < argc) cfg_style = std::atoi(argv[++i]);\n'
    '        else if (!std::strcmp(argv[i], "--cfgpage") && i + 1 < argc) cfg_page = std::atoi(argv[++i]);\n'
    '        else if (!std::strcmp(argv[i], "--cfgstart")) cfg_start = true;\n')
# create the ConfigUI after the fleet
rep('    Fleet fleet; fleet.init();\n', '''    Fleet fleet; fleet.init();
    ConfigUI cfg; cfg.fleet = &fleet;
    cfg.w = {&terrain, &rp, &waves, &water, &current, &sun_elev, &sun_azim, &chop, &foam_amount, &sea_state, &spectrum_type, &terrain_seed};
    cfg.live = {&lamp_on, &particles_on, &lamp_power, &cam_speed, &time_scale};
    cfg.init("config");
    if (cfg_style >= 0) cfg.style = std::max(0, std::min(2, cfg_style));
    if (cfg_page >= 0) cfg.page = std::max(0, std::min(ConfigUI::PAGES - 1, cfg_page));
''')
# the old window becomes a small live-tuning window, collapsed at first, hidden on full-screen setup pages
rep('''        ImGui::SetNextWindowPos(ImVec2(10, 10), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(480, 780), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowBgAlpha(0.88f);
        ImGui::Begin("Sea controls");
''', '''        cfg.draw(cam.pos, cam.yaw, terrain);
        if (cfg.regen) { regen = true; cfg.regen = false; }
        if (cfg.rebuild_terrain) { rebuild_terrain = true; cfg.rebuild_terrain = false; }
        if (cfg_start) { cfg_start = false; cfg.start(cam.pos, cam.yaw); }
        bool live_open = !cfg.hides_3d();
        float cdepth = -cam.pos.z;
        if (live_open) {
        ImGui::SetNextWindowPos(ImVec2(ImGui::GetIO().DisplaySize.x - 400, 56), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(390, 620), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowCollapsed(true, ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowBgAlpha(0.88f);
        ImGui::Begin("Live tuning (camera and sea)");
''')
rep('        float cdepth = -cam.pos.z;\n        if (ImGui::SliderFloat("Camera depth', '        if (ImGui::SliderFloat("Camera depth')
rep('''        ImGui::PopItemWidth();
        ImGui::End();
        fleet.gui(cam.pos, cam.yaw, terrain, lamp_on, water);
''', '''        ImGui::PopItemWidth();
        ImGui::End();
        }
        if (cfg.show_vehicles_window()) fleet.gui(cam.pos, cam.yaw, terrain, lamp_on, water);
''')
open(p, "w").write(s)
print("patched", p)
