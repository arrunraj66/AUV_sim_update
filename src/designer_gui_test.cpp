// designer_gui_test.cpp - the Designer hub's input rules, run through real ImGui frames (no window or GPU needed):
//   1. F2 does not toggle the page while a text field wants the keyboard (io.WantTextInput); Escape closes it only when no text field is active
//   2. "Pause simulation while open": the fleet does not step while the page is open and the checkbox is on; it steps again when closed or switched off;
//      the setting is kept in designer_state.json
// Build: cmake target designer_gui_test (links ImGui).   Run: ./designer_gui_test
#include "design_test_common.hpp"
#include "designer_hub.hpp"
#include <functional>

static void frame(const std::function<void()>& body, bool esc_down = false) {
    ImGuiIO& io = ImGui::GetIO(); io.DisplaySize = ImVec2(800, 600); io.DeltaTime = 1.0f / 60;
    io.AddKeyEvent(ImGuiKey_Escape, esc_down);
    ImGui::NewFrame(); body(); ImGui::EndFrame();
}
static char g_text[32] = "name";
static void with_text_field() { ImGui::SetNextWindowPos(ImVec2(10, 10)); ImGui::Begin("t"); if (ImGui::IsWindowAppearing() || !ImGui::IsAnyItemActive()) ImGui::SetKeyboardFocusHere(); ImGui::InputText("name", g_text, sizeof g_text); ImGui::End(); }
static void without_text_field() { ImGui::SetNextWindowPos(ImVec2(10, 10)); ImGui::Begin("t"); ImGui::Button("ok"); ImGui::End(); }

int main() {
    IMGUI_CHECKVERSION(); ImGui::CreateContext(); ImGuiIO& io = ImGui::GetIO(); io.IniFilename = nullptr;
    unsigned char* px; int w, h; io.Fonts->GetTexDataAsRGBA32(&px, &w, &h);
    Fleet fleet; DesignerHub hub; hub.fleet = &fleet; hub.pause_sim = true;

    std::printf("== F2 and Escape with a text field\n");
    // no text field: F2 toggles
    frame(without_text_field); frame(without_text_field);
    CHECK(!io.WantTextInput, "no text field: WantTextInput must be false");
    CHECK(hub.hotkey_f2() && hub.open, "F2 opens the page when no text field is active");
    // a text field is active: F2 is ignored, the page stays as it is
    hub.close(); hub.open = false;
    frame(with_text_field); frame(with_text_field); frame(with_text_field);
    std::printf("   text field active: WantTextInput = %d\n", (int)io.WantTextInput);
    CHECK(io.WantTextInput, "an active InputText must set WantTextInput");
    bool toggled = hub.hotkey_f2(); CHECK(!toggled && !hub.open, "F2 must not open the page while a text field is active");
    hub.open = true;                                           // page open, typing in a text field: F2 must not close it either
    toggled = hub.hotkey_f2(); CHECK(!toggled && hub.open, "F2 must not close the page while a text field is active");
    // Escape with a text field active: the page stays open
    bool esc_closes = true; frame([&] { with_text_field(); esc_closes = hub.escape_closes(); }, true);
    CHECK(!esc_closes && hub.open, "Escape must not close the page while a text field is active");
    // the text field is gone: F2 and Escape work again
    frame(without_text_field); frame(without_text_field); frame(without_text_field);
    CHECK(!io.WantTextInput, "the text field is gone");
    esc_closes = false; frame([&] { without_text_field(); esc_closes = hub.escape_closes(); }, true);
    CHECK(esc_closes, "Escape closes the page when no text field is active");
    frame(without_text_field, false);
    CHECK(hub.hotkey_f2() && !hub.open, "F2 closes the page again when no text field is active");

    std::printf("== pause simulation while open\n");
    Unit u; u.id = 1; u.veh.s = make_spec(M_TORPEDO); u.veh.rebuild(); u.veh.place({0, 0, -10}, 0); u.veh.v = {1.5, 0, 0}; fleet.units.push_back(u); fleet.active = 0;
    Water water; Current cur; WaveField waves; Terrain terrain;
    auto step = [&](double secs) { V3 p0 = fleet.units[0].veh.p; fleet.update(secs, 1.0, water, cur, waves, terrain); return len(fleet.units[0].veh.p - p0); };
    hub.pause_sim = true; hub.open = false; hub.apply_pause();
    double moved_closed = step(0.5);   CHECK(moved_closed > 0.2, "closed: the vehicle moves (%.3f m)", moved_closed);
    hub.open = true; hub.apply_pause(); CHECK(fleet.paused, "open + pause on: the fleet is paused");
    double moved_open = step(0.5);     CHECK(moved_open == 0.0, "open + pause on: the vehicle does not move (%.6f m)", moved_open);
    hub.pause_sim = false; hub.apply_pause(); CHECK(!fleet.paused, "open + pause off: the fleet runs");
    double moved_off = step(0.5);      CHECK(moved_off > 0.2, "open + pause off: the vehicle moves (%.3f m)", moved_off);
    hub.pause_sim = true; hub.apply_pause(); hub.open = false; hub.apply_pause(); CHECK(!fleet.paused, "closing the page releases the pause");
    std::printf("   moved: closed %.3f m, open+pause %.3f m, open without pause %.3f m\n", moved_closed, moved_open, moved_off);
    // saved with the other window state
    { std::string dir = (std::filesystem::temp_directory_path() / ("auv_gui_" + std::to_string((long)getpid()))).string(), err; std::filesystem::create_directories(dir);
      dcore::DesignerState st; st.pause_while_open = false; st.designs_dir = "/x/config/designs"; CHECK(st.save(dir + "/designer_state.json", err), "save %s", err.c_str());
      dcore::DesignerState r; CHECK(r.load(dir + "/designer_state.json", err) && !r.pause_while_open && r.designs_dir == "/x/config/designs", "pause setting and designs folder are restored");
      dcore::DesignerState def; CHECK(def.pause_while_open, "the default is: pause while open");
      std::ofstream(dir + "/old.json") << "{ \"open\": true }\n"; dcore::DesignerState o; CHECK(o.load(dir + "/old.json", err) && o.pause_while_open, "an older state file without the setting gives the default (on)");
      std::filesystem::remove_all(dir); }

    ImGui::DestroyContext();
    std::printf("\n%d checks, %d failed\n", g_checks, g_fails);
    return g_fails ? 1 : 0;
}
