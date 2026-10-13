// models.hpp - the vehicles you can deploy: their mass, drag, thrusters and fins.
// Numbers follow published values where I know them (REMUS-100: Prestero 2001; BlueROV2: published
// system identification; Slocum-class glider). They are good engineering approximations, not measurements,
// and every one can be tuned in the GUI.
#pragma once
#include "vehicle.hpp"

enum { M_TORPEDO = 0, M_HOVER, M_GLIDER, M_SCOUT, M_STL, MODEL_COUNT };
// Paused: set to true to bring back the choice between the different AUV models. While false, every vehicle is the Torpedo AUV.
static constexpr bool MODEL_CHOICE = false;
static const char* MODEL_NAMES[MODEL_COUNT] = {
    "Torpedo AUV (REMUS-100 class)", "Hover ROV/AUV (BlueROV2 class)", "Buoyancy glider (Slocum class)",
    "Scout AUV (6 thrusters)", "Custom STL model"};

static Thruster mkthr(V3 pos, V3 dir, double mx, double tau = 0.15, double ufree = 1.5, double rev = 1.0) { return {pos, unit(dir), mx, tau, ufree, rev}; }
static Fin mkfin(V3 pos, V3 n, double area, double cla, double maxd, int group, double sign) { return {pos, n, area, cla, maxd, group, sign}; }

static Spec make_spec(int id) {
    Spec s;
    switch (id) {
    case M_TORPEDO: {
        s.name = "Torpedo AUV"; s.mesh = M_TORPEDO;
        s.note = "Cruises on one propeller and steers with four fins, so it only turns while it is moving. "
                 "Up/Down = propeller, Left/Right = rudders, T/G = elevators.";
        s.mass = 30.48; s.trim = 0.004;                  // real REMUS: W = 299 N, B = 306 N; here only 0.4 % positive so it is easier to drive
        s.length = 1.6; s.width = 0.19; s.height = 0.19;
        s.rg = {0, 0, -0.0196}; s.rb = {0, 0, 0};
        s.I[0] = 0.177; s.I[1] = 3.45; s.I[2] = 3.45;
        double A[6] = {0.93, 35.5, 35.5, 0.0704, 4.88, 4.88}; for (int i = 0; i < 6; ++i) s.A[i] = A[i];
        double Dq[6] = {0.9, 131, 131, 0.3, 120, 80}; for (int i = 0; i < 6; ++i) s.Dq[i] = Dq[i];
        s.thr.push_back(mkthr({-0.80, 0, 0}, {1, 0, 0}, 12.0, 0.4, 5.0, 0.4));
        double fx = -0.56, fr = 0.125, fa = 0.045, cla = 2.5, md = 0.15;
        s.fins.push_back(mkfin({fx, fr, 0}, {0, 0, 1}, fa, cla, md, 2, -1)); s.fins.push_back(mkfin({fx, -fr, 0}, {0, 0, 1}, fa, cla, md, 2, -1));
        s.fins.push_back(mkfin({fx, 0, fr}, {0, 1, 0}, fa, cla, md, 1, -1)); s.fins.push_back(mkfin({fx, 0, -fr}, {0, 1, 0}, fa, cla, md, 1, -1));
        s.contact = {{0.8, 0, 0}, {-0.8, 0, 0}, {0, 0, -0.095}, {0, 0, 0.095}, {0, 0.095, 0}, {0, -0.095, 0}};
        s.lamp_pos = {0.78, 0, 0.03}; s.cam_pos = {0.74, 0, 0.05}; s.chase = 4.0; s.rated_depth = 100;
        break; }
    case M_HOVER: {
        s.name = "Hover ROV/AUV"; s.mesh = M_HOVER;
        s.note = "Four angled thrusters give surge, sway and yaw; two vertical thrusters give heave. It can stop and hold position. "
                 "Up/Down = surge, Z/X = sway, Left/Right = yaw, R/F = up/down.";
        s.mass = 11.5; s.trim = 0.012;
        s.length = 0.457; s.width = 0.338; s.height = 0.254;
        s.rg = {0, 0, -0.04}; s.rb = {0, 0, 0.04};
        s.I[0] = s.I[1] = s.I[2] = 0.16;
        double A[6] = {5.5, 9.0, 10.0, 0.12, 0.12, 0.12}; double Dl[6] = {4.03, 6.22, 5.18, 1.5, 1.5, 1.0}; double Dq[6] = {30.0, 25.0, 40.0, 5.0, 5.0, 3.0};
        for (int i = 0; i < 6; ++i) { s.A[i] = A[i]; s.Dl[i] = Dl[i]; s.Dq[i] = Dq[i]; }
        double T = 24.0;
        s.thr.push_back(mkthr({0.14, -0.10, 0}, {1, 1, 0}, T)); s.thr.push_back(mkthr({0.14, 0.10, 0}, {1, -1, 0}, T));
        s.thr.push_back(mkthr({-0.14, -0.10, 0}, {1, -1, 0}, T)); s.thr.push_back(mkthr({-0.14, 0.10, 0}, {1, 1, 0}, T));
        s.thr.push_back(mkthr({0, 0.12, 0}, {0, 0, 1}, T)); s.thr.push_back(mkthr({0, -0.12, 0}, {0, 0, 1}, T));
        s.contact = {{0.2, 0.15, -0.127}, {0.2, -0.15, -0.127}, {-0.2, 0.15, -0.127}, {-0.2, -0.15, -0.127}};
        s.lamp_pos = {0.24, 0, 0.0}; s.cam_pos = {0.23, 0, 0.0}; s.chase = 1.8; s.rated_depth = 100;
        break; }
    case M_GLIDER: {
        s.name = "Buoyancy glider"; s.mesh = M_GLIDER;
        s.note = "No propeller. A pump changes its volume (R/F = heavier/lighter) and a sliding battery changes its pitch (T/G). "
                 "Wings turn the vertical motion into forward motion, so it flies a slow saw-tooth path. Slow and patient.";
        s.mass = 52.0; s.trim = 0.0; s.glider = true;
        s.length = 1.5; s.width = 0.213; s.height = 0.213;
        s.rg = {0.0, 0, -0.012}; s.rb = {0, 0, 0};
        s.pump_range = 0.00025; s.pump_time = 8.0; s.mass_x = 0.006; s.mass_y = 0.006;
        s.I[0] = 0.35; s.I[1] = 7.0; s.I[2] = 7.0;
        double A[6] = {2.0, 46.0, 46.0, 0.3, 8.0, 8.0}; double Dq[6] = {3.2, 164, 164, 4.0, 40, 40};
        for (int i = 0; i < 6; ++i) { s.A[i] = A[i]; s.Dq[i] = Dq[i]; }
        s.fins.push_back(mkfin({-0.28, 0.33, 0}, {0, 0, 1}, 0.054, 4.0, 0, 0, 1)); s.fins.push_back(mkfin({-0.28, -0.33, 0}, {0, 0, 1}, 0.054, 4.0, 0, 0, 1));
        s.fins.push_back(mkfin({-0.72, 0, 0.12}, {0, 1, 0}, 0.028, 3.0, 0, 0, 1));
        s.contact = {{0.75, 0, 0}, {-0.75, 0, 0}, {0, 0, -0.1065}, {0, 0.6, 0}, {0, -0.6, 0}};
        s.lamp_pos = {0.74, 0, 0.0}; s.cam_pos = {0.70, 0, 0.06}; s.chase = 4.0; s.rated_depth = 200;
        break; }
    case M_SCOUT: default: {
        s.name = "Scout AUV"; s.mesh = M_SCOUT;
        s.note = "A bulkier survey vehicle with six thrusters: two at the stern, two vertical, two sideways. It moves in every direction "
                 "and can hover. Up/Down = surge, Z/X = sway, R/F = heave, T/G = pitch, Left/Right = yaw.";
        s.mass = 65.0; s.trim = 0.008;
        s.length = 1.2; s.width = 0.40; s.height = 0.26;
        s.rg = {0, 0, -0.045}; s.rb = {0, 0, 0};
        s.I[0] = 1.8; s.I[1] = 6.0; s.I[2] = 6.5;
        double A[6] = {8.0, 40.0, 52.0, 1.0, 8.0, 9.0}; double Dl[6] = {6, 20, 24, 2, 4, 4}; double Dq[6] = {20, 85, 110, 8, 22, 22};
        for (int i = 0; i < 6; ++i) { s.A[i] = A[i]; s.Dl[i] = Dl[i]; s.Dq[i] = Dq[i]; }
        double T = 30.0;
        s.thr.push_back(mkthr({-0.55, 0.15, 0}, {1, 0, 0}, T)); s.thr.push_back(mkthr({-0.55, -0.15, 0}, {1, 0, 0}, T));
        s.thr.push_back(mkthr({0.30, 0, 0}, {0, 0, 1}, T)); s.thr.push_back(mkthr({-0.30, 0, 0}, {0, 0, 1}, T));
        s.thr.push_back(mkthr({0.40, 0, 0}, {0, 1, 0}, T)); s.thr.push_back(mkthr({-0.40, 0, 0}, {0, 1, 0}, T));
        s.fins.push_back(mkfin({-0.50, 0.20, 0}, {0, 0, 1}, 0.04, 2.5, 0, 0, 1)); s.fins.push_back(mkfin({-0.50, -0.20, 0}, {0, 0, 1}, 0.04, 2.5, 0, 0, 1));
        s.fins.push_back(mkfin({-0.50, 0, 0.17}, {0, 1, 0}, 0.05, 2.5, 0, 0, 1)); s.fins.push_back(mkfin({-0.50, 0, -0.17}, {0, 1, 0}, 0.05, 2.5, 0, 0, 1));
        s.contact = {{0.45, 0.12, -0.13}, {0.45, -0.12, -0.13}, {-0.45, 0.12, -0.13}, {-0.45, -0.12, -0.13}, {0.6, 0, 0}, {-0.6, 0, 0}};
        s.lamp_pos = {0.58, 0, 0.0}; s.cam_pos = {0.56, 0, 0.04}; s.chase = 3.0; s.rated_depth = 300;
        break; }
    }
    return s;
}