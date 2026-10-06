# patch_step4.py - adds the vehicle system to src/main.cpp (13 small edits). Safe to run twice.
import sys
p = 'src/main.cpp'
t = open(p).read()
if 'fleet.hpp' in t:
    print('main.cpp is already patched'); sys.exit(0)
n = 0
def rep(a, b):
    global t, n
    if t.count(a) != 1:
        print('EDIT FAILED (found %d times):' % t.count(a), a[:70]); sys.exit(1)
    t = t.replace(a, b); n += 1
rep('#include "reef.hpp"\n', '#include "reef.hpp"\n#include "fleet.hpp"\n')
rep('"auvsim - Step 3: reef and diving"', '"auvsim - Step 4: vehicles"')
rep('    reef.generate(terrain, rp);\n\n    Camera cam;', '    reef.generate(terrain, rp);\n    Fleet fleet; fleet.init();\n\n    Camera cam;')
rep('            ImGui_ImplSDL2_ProcessEvent(&e);\n', '            ImGui_ImplSDL2_ProcessEvent(&e);\n            fleet.on_event(e);\n')
rep('''            if (e.type == SDL_MOUSEWHEEL && !io.WantCaptureMouse)
                cam_speed = std::max(0.2f, std::min(200.f, cam_speed * std::pow(1.15f, (float)e.wheel.y)));''',
'''            if (e.type == SDL_MOUSEWHEEL && !io.WantCaptureMouse) {
                if (fleet.cam_mode == 1) fleet.chase_scale = std::max(0.3f, std::min(4.f, fleet.chase_scale * std::pow(0.9f, (float)e.wheel.y)));
                else cam_speed = std::max(0.2f, std::min(200.f, cam_speed * std::pow(1.15f, (float)e.wheel.y)));
            }''')
rep('        if (!ImGui::GetIO().WantCaptureKeyboard) {\n            const Uint8* k', '        if (!ImGui::GetIO().WantCaptureKeyboard && fleet.cam_mode == 0) {\n            const Uint8* k')
rep('        sim_time += dt * time_scale;\n', '''        sim_time += dt * time_scale;
        fleet.update(dt * time_scale, sim_time, water, current, waves, terrain);
        if (fleet.snap_cam) { cam.yaw = fleet.snap_yaw; cam.pitch = fleet.snap_pitch; fleet.snap_cam = false; }
        { Vec3f fp; if (fleet.follow(cam.yaw, cam.pitch, fp)) cam.pos = fp; }
''')
rep('        ImGui::End();\n\n        if (rebuild_terrain)', '        ImGui::End();\n        fleet.gui(cam.pos, cam.yaw, terrain, lamp_on, water);\n\n        if (rebuild_terrain)')
rep('''        if (eye.z > -0.05f && eye.z < 0.05f) eye.z = 0.05f;
        Vec3f fwd = cam.forward();''', '''        Vec3f fwd = cam.forward(); Vec3f cam_up = {0, 0, 1};
        fleet.onboard(eye, fwd, cam_up);
        if (eye.z > -0.05f && eye.z < 0.05f) eye.z = 0.05f;''')
rep('look_at(eye, vadd(eye, fwd), {0, 0, 1}))', 'look_at(eye, vadd(eye, fwd), cam_up))')
rep('Vec3f right = norm(cross(fwd, {0, 0, 1})), up = cross(right, fwd);', 'Vec3f right = norm(cross(fwd, cam_up)), up = cross(right, fwd);')
rep('''        auto lamp_uniforms = [&](GLuint p) {
            u1f(p, "uLamp", (lamp_on && underwater) ? lamp_power : 0.0f);
            u3f(p, "uLampPos", eye.x, eye.y, eye.z); u3f(p, "uLampDir", fwd.x, fwd.y, fwd.z);
        };''', '''        Vec3f lamp_p = eye, lamp_d = fwd; bool has_lamp = fleet.lamp(lamp_p, lamp_d);
        auto lamp_uniforms = [&](GLuint p) {
            u1f(p, "uLamp", (lamp_on && (underwater || has_lamp)) ? lamp_power : 0.0f);
            u3f(p, "uLampPos", lamp_p.x, lamp_p.y, lamp_p.z); u3f(p, "uLampDir", lamp_d.x, lamp_d.y, lamp_d.z);
        };''')
rep('        // 4. floating particles', '        // 3b. vehicles\n        common(fleet.prog); vp_uniform(fleet.prog); lamp_uniforms(fleet.prog); fleet.draw();\n\n        // 4. floating particles')
open(p, 'w').write(t)
print('main.cpp patched: %d edits applied' % n)
