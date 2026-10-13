// real_models.hpp - five real, open-source vehicles as simulator Specs: numbers only, no source code is copied.
//
//   id  vehicle                         data from                                                          licence
//   0   REMUS 100 (Fossen reference)    github.com/cybergalactic/PythonVehicleSimulator  (remus100.py)      MIT (MSS repo: github.com/cybergalactic/MSS)
//   1   MBARI Tethys (LRAUV)            github.com/osrf/lrauv  (lrauv_description, tethys_equipped)         Apache-2.0
//                                       (github.com/mbari-org/gz_lrauv is the MBARI fork, also Apache-2.0)
//   2   LAUV (OceanScan)                github.com/uuvsimulator/lauv_gazebo                                 Apache-2.0
//   3   ECA A9                          github.com/uuvsimulator/eca_a9                                      Apache-2.0
//   4   BlueROV2 (6 thrusters)          github.com/FletcherFT/bluerov2 and github.com/UUVControl/bluerov2   Apache-2.0
//                                       T200 thrust / power: bluerov2_control/config/thrusters/T200-Public-Performance-Data-10-20V-September-2019.csv
//   Published top speeds, depth ratings and sizes that are not in those files come from the makers' public data sheets and papers
//   (listed in docs/real_models.md). Vehicle and company names are used only to say which vehicle the numbers describe.
//
// Where a source does not publish a value, it is estimated with the methods of the design engine (auv_design.hpp: the designer sizes the
// propeller, power, cross-flow drag, fins and contact points of an equivalent hull) and listed in Info::estimated and in Spec::note.
// For the three AUVs the designer's result is the starting point and every published number then replaces the designer's value.
// BlueROV2 is a vectored-thruster ROV, which the designer does not describe, so it is built directly.
//
// Frames: x forward, y left, z up, body origin at the centre of buoyancy (Spec::rb = 0). Source positions are relative to the centre of gravity
// and are moved by rg.
#pragma once
#include "design_spec.hpp"
#include "design_io.hpp"
#include <string>
#include <cmath>

enum { RV_REMUS = 0, RV_TETHYS, RV_LAUV, RV_ECA_A9, RV_BLUEROV2, RV_COUNT };
static const char* REAL_NAMES[RV_COUNT] = {"REMUS 100 (Fossen reference data)", "MBARI Tethys LRAUV", "LAUV (OceanScan)", "ECA A9", "BlueROV2 (6 thrusters)"};

namespace real {

struct Info {
    const char* name; const char* source; const char* licence;
    double mass_pub;                 // kg, as published in the source
    double v_lo, v_hi; const char* v_src;      // published top / operating speed range, m/s, and where it comes from
    bool rov;
    const char* estimated;           // what the source does not give and was estimated
};
inline const Info& info(int id) {
    static const Info I[RV_COUNT] = {
        {"REMUS 100 (Fossen reference data)", "github.com/cybergalactic/PythonVehicleSimulator remus100.py (MIT)", "MIT", 31.0294, 2.5, 2.5, "remus100.py: 2.5 m/s at 1525 rpm", false,
         "battery 1100 Wh and hotel 25 W (REMUS 100 data sheet class), 100 m depth rating, propeller efficiency chain (zeta 0.45), reverse thrust 0.5, sway/heave/yaw/pitch quadratic cross-flow drag (design engine), fin stall angle 0.30 rad, fin servo 150 deg/s"},
        {"MBARI Tethys LRAUV", "github.com/osrf/lrauv lrauv_description/models/tethys_equipped (Apache-2.0)", "Apache-2.0", 147.5671, 1.0, 2.0, "MBARI: 0.5 to 1.0 m/s in service (1.0 m/s science speed); top speed not published", false,
         "length 2.3 m and diameter 0.31 m (MBARI papers), 300 m depth rating, maximum propeller speed (50 rad/s assumed, the source gives no limit), propeller efficiency chain, reverse thrust, fin servo speed, roll added inertia (source: 0)"},
        {"LAUV (OceanScan)", "github.com/uuvsimulator/lauv_gazebo lauv_description (Apache-2.0)", "Apache-2.0", 18.0, 1.5, 2.6, "OceanScan data sheet: up to about 2.6 m/s (5 kn), 1.5 m/s typical", false,
         "battery 300 Wh and hotel 10 W (not in the source), 100 m depth rating (data sheet), fin travel 30 deg (the source's +-90 deg is a joint limit, not a servo range), fin stall angle, power chain, cross terms of the damping matrices are dropped"},
        {"ECA A9", "github.com/uuvsimulator/eca_a9 eca_a9_description + eca_a9_control (Apache-2.0)", "Apache-2.0", 69.7, 2.57, 2.57, "ECA Group data sheet: up to 5 kn", false,
         "propeller thrust and speed limit (design engine, the source gives only a rotor constant), battery 1500 Wh and hotel 20 W, 200 m depth rating (data sheet), fin travel 30 deg (source +-80 deg is a joint limit), forward-speed damping terms evaluated at 1.5 m/s, cross-flow quadratic drag"},
        {"BlueROV2 (6 thrusters)", "github.com/FletcherFT/bluerov2 + github.com/UUVControl/bluerov2 (Apache-2.0)", "Apache-2.0", 11.2, 1.5, 1.5, "Blue Robotics data sheet: about 1.5 m/s", true,
         "quadratic damping (source: all zeros), sway and roll added mass (source: zeros), centre of gravity 0.03 m below the centre of buoyancy (source value 0.3 to 0.39 m is not usable), battery 266 Wh, hotel 20 W, 100 m depth rating, thruster speed loss, rotational quadratic drag"},
    };
    return I[id];
}

// geometry handed to the design engine for the AUVs (the designer needs a hull, fins and one propeller)
inline design::DesignIn design_in(int id) {
    using namespace design;
    DesignIn d = default_design(); d.mast = false; d.sensors = {SE_IMU, SE_COMPASS, SE_DEPTH};
    double xfin = -0.55;
    switch (id) {
        case RV_REMUS: d = builtin_designs()[0]; d.name = REAL_NAMES[id]; d.mast = false; d.props[0].D = 0.14; d.props[0].ducted = false; xfin = -0.72; break;
        case RV_TETHYS: d.name = REAL_NAMES[id]; d.L = 2.3; d.D = 0.31; d.nose_frac = 0.12; d.tail_frac = 0.45; d.tail_end = 0.25; d.rated_depth = 300; d.battery_Wh = 5760; d.hotel_W = 28.8; d.v_cruise = 1.0; d.v_max = 1.5;
            d.props[0].D = 0.20; d.props[0].ducted = false; d.fins[0].count = 4; d.fins[0].cr = 0.14; d.fins[0].ct = 0.08; d.fins[0].span = 0.111; d.fins[0].sweep = 0.02; xfin = -1.05; break;
        case RV_LAUV: d.name = REAL_NAMES[id]; d.L = 1.08; d.D = 0.15; d.rated_depth = 100; d.battery_Wh = 300; d.hotel_W = 10; d.v_cruise = 1.5; d.v_max = 2.5; d.electronics_kg = 1.0; d.payload_kg = 1.0;
            d.props[0].D = 0.09; d.props[0].ducted = false; d.fins[0].cr = 0.10; d.fins[0].ct = 0.06; d.fins[0].span = 0.08; d.fins[0].sweep = 0.01; xfin = -0.40; break;
        case RV_ECA_A9: d.name = REAL_NAMES[id]; d.L = 1.98; d.D = 0.23; d.rated_depth = 200; d.battery_Wh = 1500; d.hotel_W = 20; d.v_cruise = 1.5; d.v_max = 2.57; d.electronics_kg = 3.0; d.payload_kg = 3.0;
            d.props[0].D = 0.18; d.props[0].ducted = false; d.fins[0].angle0 = 45; d.fins[0].cr = 0.26; d.fins[0].ct = 0.20; d.fins[0].span = 0.18; d.fins[0].sweep = 0.03; xfin = -0.708; break;
        default: break;
    }
    // fins are given from the origin (the centre of buoyancy) in the sources; the designer measures from the hull centre
    Designer des; DesignOut o0 = des.run(d); d.fins[0].x = xfin + o0.cx;
    return d;
}

inline void set_note(Spec& s, int id, const char* what) {
    s.note = std::string(what) + "  Source: " + info(id).source + ".  Estimated: " + info(id).estimated + ".";
}

// the AUVs: designer first, then the published numbers replace the designer's
// the published numbers of AUV `id`, written over a Spec that was built by design::to_spec from the vehicle's DesignIn (or an edited copy of it)
inline void apply_published(int id, Spec& s) {
    s.name = REAL_NAMES[id]; s.mesh = 0;
    const double PI = 3.14159265358979323846, DEG = PI / 180.0;
    auto fins = [&](double area, double cla_rud, double cla_elev, double stall, double max_defl_rad, double servo_dps, double x) {
        for (Fin& f : s.fins) { f.area = area; f.cl_alpha = f.group == 2 ? cla_elev : cla_rud; f.stall = stall; f.max_defl = max_defl_rad; f.f_lim = 0; if (x < 1e8) f.pos.x = x; }
        s.servo_rate = servo_dps;
    };
    auto trim_from_volume = [&](double V) { return V * 1025.0 / s.mass - 1.0; };   // the simulator takes the displaced volume as mass / 1025 * (1 + trim)
    switch (id) {
    case RV_REMUS: {   // Fossen, Handbook of Marine Craft Hydrodynamics, sect. 8.4.2, as coded in remus100.py (spheroid L 1.6 m, D 0.19 m, rho 1026)
        s.mass = 31.0294; s.trim = trim_from_volume(31.0294 / 1026.0);
        s.rg = {0, 0, -0.02}; s.rb = {0, 0, 0};
        s.I[0] = 0.11202; s.I[1] = s.I[2] = 4.0278;
        double A[6] = {0.83891, 29.43764, 29.43764, 0.03360, 3.42621, 3.42621}; for (int i = 0; i < 6; ++i) s.A[i] = A[i];
        double Dl[6] = {1.5934, 3.0234, 3.0234, 0.5649, 10.7783, 7.4540}; for (int i = 0; i < 6; ++i) s.Dl[i] = Dl[i];   // from the time constants 20 / 20 / 20 s, 1 s and the damping ratios 0.3 / 0.8
        s.axial_curve.clear(); s.Dq[0] = 6.1089;                                                                                                // 0.5 rho Cd pi b^2, Cd = 0.42
        Thruster& t = s.thr[0]; t.max_thrust = 104.6; t.tau = 1.0; t.u_free = 2.5; t.kdrop = 0.605; t.wake = 0.056; t.kq = 0.0229;   // KT0 0.4566, KTmax 0.1798 at Ja 0.6632, n 1525 rpm, D 0.14 m, t 0.10
        t.disc_ik = 1.0 / (2.0 * 1026.0 * PI * 0.25 * 0.14 * 0.14);
        fins(0.00665, 0.5, 0.7, 0.30, 15 * DEG, 150, -0.8);      // CL 0.5 (rudders) and 0.7 (stern planes), 15 deg, x -0.8 m
        s.battery_Wh = 1100; s.hotel_W = 25; s.rated_depth = 100;
        set_note(s, id, "Fossen's REMUS 100 as published: reference for the Step 7A preset.");
        break; }
    case RV_TETHYS: {  // tethys_equipped/model.sdf and description_generator.py
        s.mass = 147.5671; s.trim = 0.0;                         // the generator makes the hull neutral at 1025 kg/m3 with the buoyancy engine at 500 cc
        s.rg = {0, 0, -0.007}; s.rb = {0, 0, 0};                 // buoyancy_z_offset 0.007 m
        s.I[0] = 3.234; s.I[1] = s.I[2] = 56.36;                 // base link + battery + small parts about the common CG (my sum of the published inertias with parallel axis terms)
        double A[6] = {4.876161, 126.324739, 126.324739, 0.0, 33.46, 33.46}; for (int i = 0; i < 6; ++i) s.A[i] = A[i];
        double Dq[6] = {6.2282, 601.27, 601.27, 0.1916, 632.698957, 632.698957}; for (int i = 0; i < 6; ++i) { s.Dq[i] = Dq[i]; s.Dl[i] = 0; }   // linear terms are 0 in the source
        s.strips.clear(); s.axial_curve.clear();                 // the source's full quadratic matrix is used as published
        s.glider = true; s.pump_range = 0.00044; s.pump_time = 2 * 0.00044 / 3.0e-6; s.mass_x = 31.7307 * 0.03 / 147.5671; s.mass_y = 0;    // buoyancy engine 80..955 cc (500 cc neutral) at 3 cc/s, 31.73 kg battery moving +-0.03 m
        Thruster& t = s.thr[0]; double w = 50.0; double Ct = 0.004312328425753156, D = 0.2;        // Ct and D published, w (rad/s) assumed
        t.max_thrust = 1025.0 * Ct * w * w * std::pow(D, 4); t.rev = 0.5; t.tau = 0.5; t.pos = {-1.43162, 0, 0}; t.disc_ik = 1.0 / (2.0 * 1025.0 * PI * 0.25 * D * D);
        fins(0.0122, 4.13, 4.13, 0.17, 0.261799, 60, -1.05);     // 2 fins per set of the published 0.0244 m2, cla 4.13, stall 0.17 rad, +-15 deg
        s.battery_Wh = 400 * 14.4; s.hotel_W = 28.8; s.rated_depth = 300;
        set_note(s, id, "Buoyancy engine (R/F) and moving battery (T/G) as published; propeller and fins for speed.");
        break; }
    case RV_LAUV: {    // lauv_description: base.xacro, gazebo.xacro, snippets.xacro
        s.mass = 18.0; s.trim = trim_from_volume(0.017551374956865266);
        s.rg = {0, 0, -0.01}; s.rb = {0, 0, 0};                  // cob 0.01 m above the cog
        s.I[0] = 0.0506; s.I[1] = s.I[2] = 1.7749;               // cylinder formulas of the source
        double A[6] = {1, 16, 16, 0.005, 1.3, 1.3}, Dl[6] = {2.4, 23, 23, 0.3, 9.7, 9.7}, Dq[6] = {2.4, 80, 80, 0.0006, 9.1, 9.1};
        for (int i = 0; i < 6; ++i) { s.A[i] = A[i]; s.Dl[i] = Dl[i]; s.Dq[i] = Dq[i]; }
        s.strips.clear(); s.axial_curve.clear();                 // published full damping matrices
        Thruster& t = s.thr[0]; t.max_thrust = 50.0; t.tau = 0.05; t.rev = 1.0; t.pos = {-0.53994, 0, 0};   // thrustMax 50 N, time constant 0.05 s, x -0.54 m
        t.disc_ik = 1.0 / (2.0 * 1028.0 * PI * 0.25 * 0.09 * 0.09);
        fins(0.0064, 3.0, 3.0, 0.30, 30 * DEG, 60, -0.40);       // fin area 0.0064 m2, lift coefficient 3.0, x -0.4 m
        s.battery_Wh = 300; s.hotel_W = 10; s.rated_depth = 100;
        set_note(s, id, "LAUV as in the UUV Simulator description (thruster limit 50 N).");
        break; }
    case RV_ECA_A9: {  // eca_a9_base.xacro, eca_a9_snippets.xacro
        s.mass = 69.7; s.trim = trim_from_volume(0.06799987704121499);       // eca_a9_control model_params.yaml says 0.0823 m3: the two files disagree, the urdf value is used
        s.rg = {0, 0, -0.06}; s.rb = {0, 0, 0};                  // cob 0.06 m above the cog
        s.I[0] = 0.6; s.I[1] = 30.0; s.I[2] = 35.0;
        double A[6] = {4, 95, 75, 0.4, 27, 32}; for (int i = 0; i < 6; ++i) s.A[i] = A[i];
        const double Uref = 1.5;                                  // forward-speed damping: D = coefficient * |u|, taken at 1.5 m/s
        s.axial_curve.clear(); s.Dq[0] = 8.0; s.Dl[0] = 0; s.Dl[1] = 162 * Uref; s.Dl[2] = 108 * Uref; s.Dl[3] = 13 * Uref; s.Dl[4] = 20 * Uref; s.Dl[5] = 32 * Uref;
        s.thr[0].pos = {-0.9676, 0, 0};
        fins(0.04155, 13.98, 13.98, 0.174533, 30 * DEG, 100, -0.70777);    // fin area 0.04155 m2, cla 13.98 (above the 2 pi of a thin foil, as published), stall 10 deg
        s.battery_Wh = 1500; s.hotel_W = 20; s.rated_depth = 200;
        set_note(s, id, "ECA A9 from the UUV Simulator description.");
        break; }
    default: break;
    }
}
inline Spec make_auv(int id) {
    design::DesignIn in = design_in(id); design::Designer des; design::DesignOut o = des.run(in); Spec s = design::to_spec(in, o);
    apply_published(id, s); return s;
}

// what apply_published changed, for the UI: "mass 125.3 -> 147.57 kg" style lines (designer value -> published value)
inline std::vector<std::string> overridden_fields(const Spec& before, const Spec& after) {
    std::vector<std::string> v; char b[160];
    auto add = [&](const char* name, double x, double y, const char* unit) { if (std::fabs(x - y) > 1e-9 * std::max(1.0, std::fabs(x))) { std::snprintf(b, sizeof b, "%s %.4g -> %.4g %s", name, x, y, unit); v.push_back(b); } };
    add("mass", before.mass, after.mass, "kg"); add("net buoyancy (trim)", 100 * before.trim, 100 * after.trim, "%");
    add("CG height above CB", before.rg.z, after.rg.z, "m");
    const char* in[3] = {"inertia Ixx", "inertia Iyy", "inertia Izz"}; for (int i = 0; i < 3; ++i) add(in[i], before.I[i], after.I[i], "kg m2");
    const char* am[6] = {"added mass surge", "added mass sway", "added mass heave", "added inertia roll", "added inertia pitch", "added inertia yaw"}; for (int i = 0; i < 6; ++i) add(am[i], before.A[i], after.A[i], "");
    const char* dl[6] = {"linear drag surge", "linear drag sway", "linear drag heave", "linear drag roll", "linear drag pitch", "linear drag yaw"}; for (int i = 0; i < 6; ++i) add(dl[i], before.Dl[i], after.Dl[i], "");
    const char* dq[6] = {"quadratic drag surge", "quadratic drag sway", "quadratic drag heave", "quadratic drag roll", "quadratic drag pitch", "quadratic drag yaw"}; for (int i = 0; i < 6; ++i) add(dq[i], before.Dq[i], after.Dq[i], "");
    if (before.strips.size() != after.strips.size()) v.push_back("drag law: published matrices instead of the designer's per-strip hull drag");
    if (before.axial_curve.size() != after.axial_curve.size()) v.push_back("axial drag law: the published coefficient instead of the designer's speed table");
    for (size_t i = 0; i < before.thr.size() && i < after.thr.size(); ++i) { char n[40]; std::snprintf(n, sizeof n, "thruster %d max thrust", (int)i + 1); add(n, before.thr[i].max_thrust, after.thr[i].max_thrust, "N"); }
    if (!before.fins.empty() && !after.fins.empty()) { add("fin area (each)", before.fins[0].area, after.fins[0].area, "m2"); add("fin lift slope", before.fins[0].cl_alpha, after.fins[0].cl_alpha, "/rad"); add("fin stroke", before.fins[0].max_defl * 57.2958, after.fins[0].max_defl * 57.2958, "deg"); }
    add("fin servo speed", before.servo_rate, after.servo_rate, "deg/s");
    if (before.glider != after.glider) v.push_back("buoyancy engine and moving battery (R/F, T/G)");
    add("battery", before.battery_Wh, after.battery_Wh, "Wh"); add("hotel load", before.hotel_W, after.hotel_W, "W"); add("depth rating", before.rated_depth, after.rated_depth, "m");
    return v;
}

// BlueROV2: direct Spec, 4 horizontal + 2 vertical T200 thrusters (TAM.yaml and actuators.xacro), the rest from base.xacro / gazebo.xacro
inline Spec make_bluerov2() {
    Spec s; s.name = REAL_NAMES[RV_BLUEROV2]; s.mesh = 3;
    const double PI = 3.14159265358979323846, rho = 1025.0;
    s.mass = 11.2; s.trim = 0.011054 * 1025.0 / 11.2 - 1.0;               // volume 0.011054 m3
    s.length = 0.457; s.width = 0.338; s.height = 0.254;
    s.rg = {0, 0, -0.03}; s.rb = {0, 0, 0};
    s.I[0] = 0.3038; s.I[1] = 0.6260; s.I[2] = 0.5769;                     // 0.2 m (a^2 + b^2) of the source
    double A[6] = {1.7182, 5.468, 5.468, 0.1, 1.2481, 0.4006};             // sway (source 0) and roll (source 0) estimated
    double Dl[6] = {11.7391, 20, 31.8678, 25, 44.9085, 5};                 // published linear damping
    for (int i = 0; i < 6; ++i) { s.A[i] = A[i]; s.Dl[i] = Dl[i]; }
    // quadratic damping: published as zero. Estimated like the design engine's cross-flow drag: 0.5 rho Cd A, Cd = 1.0 for an open frame
    s.Dq[0] = 0.5 * rho * 1.0 * (0.338 * 0.254); s.Dq[1] = 0.5 * rho * 1.0 * (0.457 * 0.254); s.Dq[2] = 0.5 * rho * 1.0 * (0.457 * 0.338);
    s.Dq[3] = 0.1; s.Dq[4] = 0.2; s.Dq[5] = 0.18;
    // per-strip drag of an open frame (Cd 1.0)
    { const int N = 8; const double L = 0.457, zd = 0.0;      // drag acts at the thrust line (z = 0), see the note at the thrusters
      for (int i = 0; i < N; ++i) { DragStrip d; double dx = L / N; d.x = -0.5 * L + (i + 0.5) * dx; d.z = zd; d.cy = 0.5 * rho * 1.0 * 0.254 * dx; d.cz = 0.5 * rho * 1.0 * 0.338 * dx; d.ax = 1.0 / N; s.strips.push_back(d); } }
    auto thr = [&](V3 pos, V3 dir) {
        Thruster t; t.pos = pos; t.dir = unit(dir);                                    // x, y as published (the origin of the source is the cog = cob)
        t.max_thrust = 51.5; t.rev = 39.9 / 51.5; t.tau = 0.05; t.u_free = 2.0; t.kdrop = 0.4;       // T200 at 16 V: +5.25 kgf / -4.07 kgf; thruster time constant 0.05 s
        double Ap = PI * 0.25 * 0.076 * 0.076; t.disc_ik = 1.0 / (2.0 * rho * Ap);
        t.zeta = std::pow(51.5, 1.5) * std::sqrt(t.disc_ik) / 381.0;      // 381 W at full forward thrust (CSV)
        s.thr.push_back(t); };
    const double c = 0.7071067811865476;
    // The source puts the four horizontal thrusters 0.0725 m below the cog. The simulator applies hull drag at the body origin, so a thrust line that far
    // below it would pitch the vehicle with no drag couple to balance it (a real frame has its centre of drag at about thruster height) and it would loop
    // in pitch at full surge. The horizontal thrust line is therefore put through the centre of drag (z = 0 from the origin); x and y are as published.
    // The source puts the four horizontal thrusters 0.0725 m below the cog. Tested with per-strip drag at the frame's reference height, that geometry does
    // NOT fly: full surge (145.7 N) makes a pitching couple of 10.6 N m against a maximum righting moment W * BG = 3.3 N m (BG 0.03 m, estimated), so the
    // vehicle loops in pitch; even BG = 0.10 m only holds 60 % thrust. A real frame has its centre of drag at about thruster height. So the horizontal
    // thrust line and the drag strips are both at z = 0 here (x and y as published).
    const double zt = 0.0;
    thr({0.1355, -0.1, zt}, {c, c, 0}); thr({0.1355, 0.1, zt}, {c, -c, 0});
    thr({-0.1475, -0.1, zt}, {c, -c, 0}); thr({-0.1475, 0.1, zt}, {c, c, 0});   // rear axes flipped from TAM.yaml's sign so that surge is the strong (forward) direction of all four T200
    thr({0.0025, -0.1105, -0.005}, {0, 0, 1}); thr({0.0025, 0.1105, -0.005}, {0, 0, 1});
    s.contact = {{0.2, 0.15, -0.127}, {0.2, -0.15, -0.127}, {-0.2, 0.15, -0.127}, {-0.2, -0.15, -0.127}};
    s.lamp_pos = {0.24, 0, 0.0}; s.cam_pos = {0.23, 0, 0.0}; s.chase = 1.8;
    s.battery_Wh = 266; s.hotel_W = 20; s.rated_depth = 100;
    set_note(s, RV_BLUEROV2, "Four angled horizontal thrusters (surge, sway, yaw), two vertical (heave, roll). Up/Down = surge, Z/X or Q/E = sway, Left/Right = yaw, R/F = up/down.");
    return s;
}

}  // namespace real

// the entry point asked for: the Spec of real vehicle `id` (RV_REMUS .. RV_BLUEROV2)
static inline Spec make_real_spec(int id) { return id == RV_BLUEROV2 ? real::make_bluerov2() : real::make_auv(id); }
