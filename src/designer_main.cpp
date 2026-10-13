// designer_main.cpp - the AUV Designer as a stand-alone program (Step 8C).
// Run: ./auvdesigner            test options: --preset N  --tab N  --shot file.ppm  --frames N
#include "design_ui.hpp"
#include "project_paths.hpp"
#include "imgui_impl_sdl2.h"
#include "imgui_impl_opengl3.h"
#include <SDL.h>
#include <SDL_opengl.h>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>
#include <filesystem>

static void theme() {
    ImGuiStyle& s = ImGui::GetStyle(); ImVec4* c = s.Colors; auto C = [](float r, float g, float b, float a = 1) { return ImVec4(r, g, b, a); };
    s.WindowRounding = 6; s.ChildRounding = 6; s.FrameRounding = 4; s.GrabRounding = 4; s.TabRounding = 4; s.ItemSpacing = ImVec2(8, 5); s.FramePadding = ImVec2(7, 3); s.GrabMinSize = 12;
    ImVec4 acc = C(0.16f, 0.78f, 0.88f), accD = C(0.10f, 0.42f, 0.52f);
    c[ImGuiCol_WindowBg] = C(0.045f, 0.075f, 0.105f); c[ImGuiCol_ChildBg] = C(0.06f, 0.09f, 0.12f); c[ImGuiCol_PopupBg] = C(0.06f, 0.09f, 0.12f, 0.98f); c[ImGuiCol_Border] = C(0.30f, 0.50f, 0.60f, 0.35f);
    c[ImGuiCol_Text] = C(0.89f, 0.93f, 0.96f); c[ImGuiCol_TextDisabled] = C(0.50f, 0.58f, 0.65f);
    c[ImGuiCol_FrameBg] = C(0.10f, 0.15f, 0.20f); c[ImGuiCol_FrameBgHovered] = C(0.14f, 0.22f, 0.28f); c[ImGuiCol_FrameBgActive] = C(0.16f, 0.27f, 0.34f);
    c[ImGuiCol_Button] = C(0.12f, 0.24f, 0.30f); c[ImGuiCol_ButtonHovered] = accD; c[ImGuiCol_ButtonActive] = acc;
    c[ImGuiCol_Header] = C(0.10f, 0.28f, 0.35f, 0.8f); c[ImGuiCol_HeaderHovered] = accD; c[ImGuiCol_HeaderActive] = acc;
    c[ImGuiCol_SliderGrab] = acc; c[ImGuiCol_SliderGrabActive] = C(0.5f, 0.95f, 1); c[ImGuiCol_CheckMark] = acc;
    c[ImGuiCol_Tab] = C(0.10f, 0.20f, 0.26f); c[ImGuiCol_TabHovered] = accD; c[ImGuiCol_TabActive] = C(0.12f, 0.45f, 0.55f);
    c[ImGuiCol_TableHeaderBg] = C(0.08f, 0.16f, 0.22f); c[ImGuiCol_TableRowBgAlt] = C(1, 1, 1, 0.03f); c[ImGuiCol_TableBorderLight] = C(0.2f, 0.3f, 0.4f, 0.5f); c[ImGuiCol_TableBorderStrong] = C(0.2f, 0.3f, 0.4f, 0.7f);
}

int main(int argc, char** argv) {
    const char* shot = nullptr; int frames = 3, preset = -1, tab = -1;
    for (int i = 1; i < argc; ++i) {
        if (!std::strcmp(argv[i], "--shot") && i + 1 < argc) shot = argv[++i];
        else if (!std::strcmp(argv[i], "--frames") && i + 1 < argc) frames = std::atoi(argv[++i]);
        else if (!std::strcmp(argv[i], "--tab") && i + 1 < argc) tab = std::atoi(argv[++i]);
        else if (!std::strcmp(argv[i], "--preset") && i + 1 < argc) preset = std::atoi(argv[++i]);
    }
    if (SDL_Init(SDL_INIT_VIDEO) != 0) { std::fprintf(stderr, "SDL error: %s\n", SDL_GetError()); return 1; }
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3); SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE); SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
    SDL_Window* win = SDL_CreateWindow("AUV Designer", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, 1500, 860, SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE);
    if (!win) { std::fprintf(stderr, "Window error: %s\n", SDL_GetError()); return 1; }
    SDL_GLContext gl = SDL_GL_CreateContext(win); SDL_GL_SetSwapInterval(1);
    IMGUI_CHECKVERSION(); ImGui::CreateContext(); ImGui::GetIO().IniFilename = nullptr; ImGui::StyleColorsDark(); theme();
    {   const char* fonts[] = {"/usr/share/fonts/truetype/ubuntu/Ubuntu-R.ttf", "/usr/share/fonts/truetype/noto/NotoSans-Regular.ttf", "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", nullptr};
        ImFont* f = nullptr; for (int i = 0; fonts[i] && !f; ++i) if (std::filesystem::exists(fonts[i])) f = ImGui::GetIO().Fonts->AddFontFromFileTTF(fonts[i], 16.f);
        if (!f) ImGui::GetIO().Fonts->AddFontDefault(); }
    ImGui_ImplSDL2_InitForOpenGL(win, gl); ImGui_ImplOpenGL3_Init("#version 330");
    std::string designs = paths::designs_dir(); paths::migrate_all(designs);          // <project root>/config/designs, whatever the working directory is
    DesignerUI ui(designs); if (preset >= 0) ui.load_index_public(preset);
    ui.tab_req_ = tab;
    bool run = true; int n = 0;
    while (run) {
        SDL_Event e; while (SDL_PollEvent(&e)) { ImGui_ImplSDL2_ProcessEvent(&e); if (e.type == SDL_QUIT) run = false; if (e.type == SDL_KEYDOWN && e.key.keysym.sym == SDLK_ESCAPE) run = false; }
        ImGui_ImplOpenGL3_NewFrame(); ImGui_ImplSDL2_NewFrame(); ImGui::NewFrame();
        int w, h; SDL_GetWindowSize(win, &w, &h); ui.draw(0, 0, (float)w, (float)h);
        ImGui::Render(); int fw, fh; SDL_GL_GetDrawableSize(win, &fw, &fh); glViewport(0, 0, fw, fh); glClearColor(0.03f, 0.05f, 0.08f, 1); glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        if (shot && ++n >= frames) {
            std::vector<unsigned char> px((size_t)fw * fh * 3); glReadPixels(0, 0, fw, fh, GL_RGB, GL_UNSIGNED_BYTE, px.data());
            FILE* f = std::fopen(shot, "wb"); std::fprintf(f, "P6\n%d %d\n255\n", fw, fh); for (int y = fh - 1; y >= 0; --y) std::fwrite(&px[(size_t)y * fw * 3], 1, (size_t)fw * 3, f); std::fclose(f); run = false;
        }
        SDL_GL_SwapWindow(win);
    }
    ImGui_ImplOpenGL3_Shutdown(); ImGui_ImplSDL2_Shutdown(); ImGui::DestroyContext(); SDL_GL_DeleteContext(gl); SDL_DestroyWindow(win); SDL_Quit();
    return 0;
}
