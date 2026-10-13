#!/usr/bin/env python3
# Step 6B: adds the Cockpit to config_ui.hpp and main.cpp (run from ~/Documents/auvsim_new). Stops if an anchor is missing, keeps backups (.bak6b).
import sys, shutil
def patch(path, edits):
    s = open(path).read()
    if "cockpit" in s:
        print(path, "already patched"); return
    shutil.copy(path, path + ".bak6b")
    for old, new in edits:
        assert s.count(old) == 1, "anchor not found in %s: %r" % (path, old[:70])
        s = s.replace(old, new)
    open(path, "w").write(s); print("patched", path)

patch("src/config_ui.hpp", [
 ("    int page = 0; bool setup_open = true, preview = false;\n",
  "    int page = 0; bool setup_open = true, preview = false;\n    bool run_cockpit = false, open_cockpit = true;     // after START show the Cockpit (full-screen 3D with HUD widgets)\n"),
 ("    bool hides_3d() const { return style == 2 && setup_open && !preview; }",
  "    bool hides_3d() const { return style == 2 && setup_open && !preview && !run_cockpit; }"),
 ("        if (ImGui::Button(had ? \"Replace the vehicle  (start again)\" : \"START: put the AUV in the water\", ImVec2(300, 40))) start(cam_pos, cam_yaw);\n",
  "        ImGui::Checkbox(\"Open the Cockpit after START\", &open_cockpit);\n        if (ImGui::Button(had ? \"Replace the vehicle  (start again)\" : \"START: put the AUV in the water\", ImVec2(300, 40))) start(cam_pos, cam_yaw);\n        if (had) { ImGui::SameLine(); if (ImGui::Button(\"Open the Cockpit\", ImVec2(150, 40))) run_cockpit = true; }\n"),
 ("        if (style == 2) { setup_open = false; preview = false; }\n        return true;",
  "        if (style == 2) { setup_open = false; preview = false; }\n        if (open_cockpit) run_cockpit = true;\n        return true;"),
])

patch("src/main.cpp", [
 ('#include "config_ui.hpp"\n', '#include "config_ui.hpp"\n#include "cockpit.hpp"\n'),
 ("    ImGui::GetIO().IniFilename = nullptr;\n",
  "    ImGui::GetIO().IniFilename = nullptr;\n    Cockpit cockpit; cockpit.load_fonts(); Cockpit::apply_theme();\n"),
 ('    cfg.init("config");\n', '    cfg.init("config"); cockpit.init("config");\n    if (ck_preset >= 0) cockpit.apply_preset(std::max(0, std::min(3, ck_preset)));\n    cockpit.edit = ck_edit;\n'),
 ('bool cfg_start = false;\n', 'bool cfg_start = false; int ck_preset = -1; bool ck_edit = false;\n'),
 ('        else if (!std::strcmp(argv[i], "--cfgstart")) cfg_start = true;\n',
  '        else if (!std::strcmp(argv[i], "--cfgstart")) cfg_start = true;\n        else if (!std::strcmp(argv[i], "--ckpreset") && i + 1 < argc) ck_preset = std::atoi(argv[++i]);\n        else if (!std::strcmp(argv[i], "--ckedit")) ck_edit = true;\n'),
 ("        cfg.draw(cam.pos, cam.yaw, terrain);\n",
  """        bool in_ck = cfg.run_cockpit && fleet.act();
        if (cfg.run_cockpit && !fleet.act()) cfg.run_cockpit = false;
        if (in_ck) {
            CkCtx ctx{&fleet, &terrain, &current, &water, &cfg.unit_sensors, &lamp_on};
            cockpit.update(dt * time_scale, ctx); cockpit.draw(ctx, sim_time);
            if (cockpit.want_setup) { cockpit.want_setup = false; cfg.run_cockpit = false; cfg.setup_open = true; cfg.preview = false; }
        } else cfg.draw(cam.pos, cam.yaw, terrain);
"""),
 ("        bool live_open = !cfg.hides_3d();\n", "        bool live_open = in_ck ? cockpit.show_live : !cfg.hides_3d();\n"),
 ("        if (cfg.show_vehicles_window()) fleet.gui(", "        if (!in_ck && cfg.show_vehicles_window()) fleet.gui("),
 ("        // dive gauge in the top-right corner\n        {\n", "        // dive gauge in the top-right corner\n        if (!in_ck) {\n"),
])
