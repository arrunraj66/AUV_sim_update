// physics_test.cpp - checks every physics file on its own against known numbers.
// Build: make physics_test     Run: ./physics_test
#include "physics/seawater.hpp"
#include "physics/rigid_body.hpp"
#include "physics/hull.hpp"
#include "physics/hydrostatics.hpp"
#include "physics/added_mass.hpp"
#include "physics/drag.hpp"
#include "physics/foil.hpp"
#include "physics/fin.hpp"
#include "physics/rudder.hpp"
#include "physics/blade.hpp"
#include "physics/propeller.hpp"
#include "physics/shaft.hpp"
#include "physics/battery.hpp"
#include "physics/thruster.hpp"
#include <cstdio>
using namespace phys;

static int fails = 0;
static void check(const char* what, double got, double lo, double hi, const char* unit) {
    bool ok = got >= lo && got <= hi; if (!ok) ++fails;
    std::printf("  [%s] %-46s %10.4f %-8s (expected %.4g .. %.4g)\n", ok ? " OK " : "FAIL", what, got, unit, lo, hi);
}
// REMUS-100 hull (Prestero 2001): length 1.33 m of body, diameter 0.191 m
static double remus_r(double x) {
    const double R = 0.0955;
    if (x > 0.5) { double t = (x - 0.5) / 0.3; return R * std::sqrt(std::max(0.0, 1 - t * t)); }
    if (x < -0.30) { double t = std::min(1.0, (-0.30 - x) / 0.42); return R * (1.0 - 0.62 * std::pow(t, 1.5)); }
    return R;
}

int main() {
    std::printf("\n1. SEA WATER\n");
    check("density 15 C, 35 PSU, surface", sw_density(15, 35, 0), 1025.5, 1026.5, "kg/m3");
    check("density 2 C, 35 PSU, 1000 m", sw_density(2, 35, 1000), 1032.0, 1033.5, "kg/m3");
    check("kinematic viscosity 15 C", sw_viscosity(15) * 1e6, 1.18, 1.20, "mm2/s");
    check("vapour pressure 20 C", sw_vapour(20), 2200, 2400, "Pa");

    std::printf("\n2. RIGID BODY (parts -> mass, CG, inertia)\n");
    MassProps mp;
    mp.add_cylinder_x("hull", 18.0, {0, 0, 0}, 0.095, 1.4, true);
    mp.add_cylinder_x("battery", 9.0, {0.05, 0, -0.03}, 0.07, 0.4);
    mp.add_point("motor", 1.5, {-0.62, 0, 0});
    mp.add_point("ballast", 2.0, {0.1, 0, -0.07});
    mp.update();
    check("total mass", mp.m, 30.49, 30.51, "kg");
    check("CG below centre line (z)", mp.rg.z, -0.0185, -0.0125, "m");
    check("pitch inertia Iyy about CG", mp.Ig.a[1][1], 3.0, 4.2, "kg m2");

    std::printf("\n3. HULL + HYDROSTATICS (REMUS-like)\n");
    Hull hull; hull.from_profile(remus_r, -0.72, 0.80, 1, 1);
    check("hull volume", hull.volume, 0.033, 0.040, "m3");
    Hydrostatics hs; hs.V0 = 30.48 * 1.004 / 1025.0; hs.cb = {0, 0, 0};
    auto flat = [](double, double) { return 0.0; };
    Hydrostatics::Out a = hs.eval(hull, {0, 0, -5}, Q{}, flat, 1025, G, mp.rg);
    check("buoyancy fully under water (W = 298.9 N)", a.B, 299.5, 300.6, "N");
    check("fraction under water", a.frac, 0.999, 1.001, "");
    Hydrostatics::Out b = hs.eval(hull, {0, 0, 0.0}, Q{}, flat, 1025, G, mp.rg);
    check("half out of the water: fraction", b.frac, 0.45, 0.55, "");
    check("transverse GM at the surface", b.gm_t, 0.02, 0.10, "m");
    Hydrostatics::Out c = hs.eval(hull, {0, 0, -300}, Q{}, flat, sw_density(5, 35, 300), G, mp.rg);
    check("buoyancy change at 300 m (compressibility+rho)", c.B - a.B, 0.5, 3.0, "N");

    std::printf("\n4. ADDED MASS (Prestero REMUS: Xu'=0.93, Yv'=35.5, Nr'=4.88)\n");
    AddedMass am; std::vector<PlateAM> plates;
    am.compute(hull, plates, 1025);
    check("surge added mass", am.diag(0), 0.7, 1.5, "kg");
    check("sway added mass", am.diag(1), 30, 42, "kg");
    check("yaw added inertia", am.diag(5), 3.0, 6.0, "kg m2");

    std::printf("\n5. DRAG (Prestero REMUS: Xu|u| = 1.62 kg/m bare hull ~ 2-3 with fins)\n");
    Drag dr;
    Drag::Out d = dr.eval(hull, {1.5, 0, 0}, {}, 1025, 1.19e-6, 20, G);
    check("axial drag coefficient Xu|u| at 1.5 m/s", d.Xu, 1.3, 2.6, "kg/m");
    Drag::Out d2 = dr.eval(hull, {0, 0.5, 0}, {}, 1025, 1.19e-6, 20, G);
    check("sideways drag Yv|v| (Prestero 131)", -d2.F.y / 0.25, 90, 180, "kg/m");
    Drag::Out d3 = dr.eval(hull, {1.5, 0, 0}, {}, 1025, 1.19e-6, 0.15, G);
    check("wave drag factor just under the surface", d3.wave, 1.3, 4.0, "x");

    std::printf("\n6. FOIL\n");
    Foil w1 = Foil::wing(1.5);
    check("lift slope AR 1.5 (Helmbold)", w1.cla, 2.0, 2.5, "1/rad");
    FoilCoef f10 = w1.coef(10 * M_PI / 180, 3e5), f50 = w1.coef(50 * M_PI / 180, 3e5);
    check("CL at 10 deg", f10.CL, 0.33, 0.45, "");
    check("CL after stall (50 deg) lower than CN max", f50.CL, 0.4, 0.8, "");
    check("CD at 50 deg (separated)", f50.CD, 0.6, 1.2, "");
    Foil s2 = Foil::section(0.12);
    check("2-D section CL at 6 deg", s2.coef(6 * M_PI / 180, 1e6).CL, 0.55, 0.65, "");

    std::printf("\n7. FIN (REMUS stern fin) and RUDDER SERVO\n");
    FinGeom fg; fg.root = {-0.56, 0.066, 0}; fg.span_dir = {0, 1, 0}; fg.normal = {0, 0, 1}; fg.cr = 0.2; fg.ct = 0.12; fg.span = 0.15; fg.sweep = 0.04; fg.tc = 0.10; fg.ar_gain = 1.5; fg.setup();
    check("fin area", fg.area, 0.0239, 0.0241, "m2");
    FinOut fo = fin_force(fg, {1.5, 0, 0}, {}, {}, 0.2, 1025, 1.19e-6);
    check("lift at 1.5 m/s, 11.5 deg deflection (+z)", fo.F.z, 8, 20, "N");
    check("drag of the deflected fin (-x)", fo.F.x, -3.0, -0.2, "N");
    check("hinge moment (wants to go back: negative)", fo.hinge, -0.5, 0.0, "N m");
    ControlSurface cs; cs.fin = fg; cs.k_elevator = 1; cs.servo.max_angle = 0.4; cs.servo.max_rate = 1.0;
    double t_reach = -1;
    for (int i = 0; i < 400; ++i) { cs.step(0, 1.0, 0, {1.5, 0, 0}, {}, {}, 1025, 1.19e-6, 1, 0.005); if (t_reach < 0 && cs.servo.angle > 0.38) t_reach = (i + 1) * 0.005; }
    check("servo time to 95 % of 0.4 rad at 1 rad/s", t_reach, 0.33, 0.6, "s");
    Servo weak = cs.servo; weak.stall_torque = 0.05; weak.angle = 0; weak.rate = 0;
    ControlSurface cw = cs; cw.servo = weak; cw.last = FinOut{};
    for (int i = 0; i < 400; ++i) cw.step(0, 1.0, 0, {3.0, 0, 0}, {}, {}, 1025, 1.19e-6, 1, 0.005);
    check("weak servo blown back at 3 m/s (angle reached)", cw.servo.angle, 0.0, 0.39, "rad");

    std::printf("\n8. BLADES (BEMT) - compare Wageningen B4-55, P/D 1.0: KT0~0.45, 10KQ0~0.60, eta max ~0.65 @ J~0.8\n");
    BladeGeom bg; bg.B = 4; bg.D = 0.2; bg.pd = 1.0; bg.ear = 0.55;
    double n = 15, D4 = std::pow(bg.D, 4);
    BemOut b0 = bem_solve(bg, n, 0, 1025, 1.19e-6), b8 = bem_solve(bg, n, 0.75 * n * bg.D, 1025, 1.19e-6);
    check("KT at J=0 (bollard)", b0.T / (1025 * n * n * D4), 0.38, 0.52, "");
    check("10 KQ at J=0", 10 * b0.Q / (1025 * n * n * D4 * bg.D), 0.45, 0.70, "");
    check("open-water efficiency at J=0.75", b8.eta, 0.55, 0.72, "");

    std::printf("\n9. PROPELLER (tables, reverse, cavitation, ventilation)\n");
    Propeller pr; pr.blade = bg; pr.build(15);
    Propeller::Out p0 = pr.eval(15, 0, 1025, 2.0, sw_pressure(10), 1700), pr_ = pr.eval(-15, 0, 1025, 2.0, sw_pressure(10), 1700);
    check("reverse / forward bollard thrust", -pr_.T / p0.T, 0.5, 0.95, "");
    Propeller::Out pa = pr.eval(15, 0, 1025, -0.2, P_ATM, 1700);
    check("thrust left when the shaft is out of the water", pa.T / p0.T, 0.0, 0.5, "");
    Propeller::Out pc = pr.eval(60, 0, 1025, 2.0, P_ATM + 1025 * G * 0.2, 1700);
    check("cavitation number at 3600 rpm near the surface", pc.sigma, 0.0, 1.0, "");

    std::printf("\n10. SHAFT + MOTOR + BATTERY + THRUSTER (BlueROV2 T200-like, 4S battery)\n");
    ThrusterUnit th; th.pos = {0, 0, 0}; th.dir = {1, 0, 0}; th.rpm_max = 3600;
    th.prop.blade.B = 3; th.prop.blade.D = 0.076; th.prop.blade.pd = 1.0; th.prop.blade.ear = 0.7; th.prop.blade.ducted = true;
    th.drive.motor.kv = 300; th.drive.motor.R = 0.12; th.drive.motor.I_max = 24;
    th.prop.duct_k = 0.25;                                   // the T200 has a short nozzle, not a Kort duct
    th.setup();
    std::printf("  [info] bollard thrust from blade geometry alone       %10.1f N\n", th.rated_thrust());
    th.prop.calibrate(50.0, 3600);                            // datasheet: ~5.1 kgf at 16 V
    check("bollard thrust after calibration (datasheet 50 N)", th.rated_thrust(), 49.5, 50.5, "N");
    Battery bat; bat.Ns = 4; bat.cap_Ah = 18;
    WaterState ws = WaterState::at(15, 35, 5);
    double t95 = -1, Fw = 0.8 * th.rated_thrust();
    for (int i = 0; i < 600; ++i) {
        double Vb = bat.voltage_at(th.drive.I_bus);
        th.step(0.005, Fw, Vb, {0, 0, 0}, 5, ws);
        bat.step(th.drive.I_bus, 0.005);
        if (t95 < 0 && th.T > 0.95 * Fw) t95 = (i + 1) * 0.005;
    }
    check("time to 95 % of a thrust step", t95, 0.02, 0.6, "s");
    check("steady thrust follows the command", th.T / Fw, 0.95, 1.05, "");
    check("motor current", th.drive.I, 3, 24, "A");
    check("electrical power", th.drive.P_elec, 40, 400, "W");
    check("battery voltage under load (4S)", bat.V, 13.5, 16.8, "V");
    check("reaction torque on the hull (roll)", th.M.x, -0.6, -0.02, "N m");
    for (int i = 0; i < 400; ++i) th.step(0.005, Fw, bat.voltage_at(th.drive.I_bus), {1.5, 0, 0}, 5, ws);
    check("thrust at 1.5 m/s with the same rpm command, falls", th.T / Fw, 0.4, 0.95, "");

    std::printf("\n%s  (%d failed)\n\n", fails ? "SOME CHECKS FAILED" : "ALL CHECKS PASSED", fails);
    return fails ? 1 : 0;
}