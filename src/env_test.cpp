// env_test.cpp - checks contact, tether, acoustics and stratification.   Build: g++ -std=c++17 -O2 -o env_test env_test.cpp
#include "physics/seawater.hpp"
#include "physics/hull.hpp"
#include "physics/contact.hpp"
#include "physics/tether.hpp"
#include "physics/acoustics.hpp"
#include "physics/stratification.hpp"
#include <cstdio>
using namespace phys;

static int fails = 0;
static void check(const char* what, double got, double lo, double hi, const char* unit) {
    bool ok = got >= lo && got <= hi; if (!ok) ++fails;
    std::printf("  [%s] %-52s %11.5g %-6s (expected %.4g .. %.4g)\n", ok ? " OK " : "FAIL", what, got, unit, lo, hi);
}

int main() {
    const double rho = 1025;
    std::printf("\n== 1. contact with sea floor and rocks ==\n");
    Hull one; one.st = {{0, 0.1, 0.1, 0.01}};
    { Contact c; c.floor = [](double, double) { return 0.0; }; Q q;
      c.eval(one, {0, 0, 0.1 - 0.002}, q, {}, {});
      check("force on 2 mm penetration = k d^1.5 (lowest point)", c.F.z / (c.k * std::pow(0.002, 1.5)), 0.95, 1.05, "");
      check("no friction when not sliding", std::fabs(c.F.x), 0, 1e-6, "N");
      c.eval(one, {0, 0, 0.1 - 0.002}, q, {0, 0, -0.5}, {});
      check("landing at 0.5 m/s pushes harder (Hunt-Crossley)", c.F.z / (c.k * std::pow(0.002, 1.5)), 1.2, 1.4, "");
      c.eval(one, {0, 0, 0.1 - 0.002}, q, {0.5, 0, 0}, {});
      check("sliding: friction / normal force = mu", std::fabs(c.F.x) / c.F.z, 0.45, 0.5, "");
      check("friction opposes the motion (-x)", c.F.x, -1e9, -1e-9, "N");
      c.eval(one, {0, 0, 0.5}, q, {}, {}); check("above the floor: no force", std::fabs(c.F.z) + c.points_in_contact, 0, 1e-12, "");
      c.eval(one, {0, 0, 0.1 - 0.002}, q, {0, 0, +0.5}, {}); check("leaving the floor: force is never negative", c.F.z, 0.0, 1e9, "N");
      Contact sl; sl.floor = [](double x, double) { return 0.5 * x; };
      sl.eval(one, {0, 0, 0.1 + 0.0 - 0.01}, q, {}, {});
      check("slope 0.5: force leans away from the slope (Fx/Fz = -0.5)", sl.F.x / sl.F.z, -0.55, -0.45, ""); }
    { Contact c; c.spheres = {{{0, 0, -0.3}, 0.25}}; Q q; c.eval(one, {0, 0, 0}, q, {}, {});
      check("rock below: pushed up", c.F.z, 1.0, 1e9, "N"); check("rock below: no sideways force", std::fabs(c.F.y), 0, 1e-6, "N");
      check("penetration depth (m)", c.max_pen, 0.049, 0.051, "m"); }

    std::printf("\n== 2. tether ==\n");
    { Tether t; t.n = 20; t.L = 10.5; t.wet_w = 2.0; t.EA = 5e4; t.extra_damp = 6.0; t.init_line({0, 0, 0}, {10, 0, -0.5});
      for (int i = 0; i <= t.n; ++i) t.p[i].z = 0;                          // straight slack line, will sag
      for (int i = 0; i < 40000; ++i) t.step(0.01, {10, 0, 0}, {});
      double u = 0.54, a = 0; { double lo = 0.01, hi = 5; for (int k = 0; k < 80; ++k) { double m = 0.5 * (lo + hi); if (std::sinh(m) / m > 10.5 / 10.0) hi = m; else lo = m; } u = lo; a = 10.0 / (2 * u); }
      double sag = a * (std::cosh(u) - 1), H = 2.0 * a;
      double zmin = 0; for (auto& p : t.p) zmin = std::min(zmin, p.z);
      check("hanging cable: sag vs catenary (m)", -zmin / sag, 0.95, 1.05, "");
      check("horizontal tension vs w a", std::fabs(t.Fend.x) / H, 0.95, 1.05, "");
      check("vertical pull on the vehicle vs w L / 2", std::fabs(t.Fend.z) / (2.0 * 10.5 / 2), 0.93, 1.07, "");
      check("cable length stays near its rest length", t.length_now() / 10.5, 0.999, 1.01, ""); }
    { Tether t; t.n = 25; t.L = 50; t.wet_w = 0; t.fixed = false; t.init_line({-50, 0, 0}, {0, 0, 0});
      V3 ev{0, 0, 0};
      for (int i = 0; i < 4000; ++i) { t.step(0.01, ev, {1.0, 0, 0}); ev.x += 1.0 * 0.01; }
      double Ft = 0.5 * rho * M_PI * 0.01 * 0.02 * 1.0 * 50;
      check("towed straight: tension vs tangential drag 1/2 rho pi d Cdt U^2 L", t.tension_at_end() / Ft, 0.8, 1.2, "");
      check("towed cable trails behind (end node behind the vehicle)", t.p[0].x - ev.x, -52, -40, "m"); }
    { Tether t; t.n = 10; t.L = 10; t.wet_w = 0; t.fixed = true; t.init_line({0, 0, 0}, {10, 0, 0});
      for (int i = 0; i < 6000; ++i) t.step(0.01, {10, 0, 0}, {}, [](V3) { return V3{0, 0.5, 0}; });
      double ymax = 0; for (auto& p : t.p) ymax = std::max(ymax, std::fabs(p.y));
      check("current across a cable bows it downstream (m)", ymax, 0.01, 6.0, "m");
      check("cross-current drag pulls the vehicle downstream (+y)", t.Fend.y, 0.01, 1e9, "N"); }

    std::printf("\n== 3. acoustics ==\n");
    check("Thorp absorption at 1 kHz", thorp_alpha(1.0), 0.05, 0.09, "dB/km");
    check("Thorp absorption at 10 kHz", thorp_alpha(10.0), 1.0, 1.4, "dB/km");
    check("transmission loss 100 m, 1 kHz (spherical + absorption)", transmission_loss(100, 1.0), 40.0, 40.1, "dB");
    check("transmission loss: 10x range = +20 dB (low freq)", transmission_loss(1000, 0.1) - transmission_loss(100, 0.1), 19.9, 20.3, "dB");
    { PropNoise pn; pn.Z = 5;
      check("blade rate tone, 5 blades at 1800 rpm", pn.blade_rate(1800), 149.9, 150.1, "Hz");
      PropNoise p3; auto a = p3.eval(30, 1.0), b = p3.eval(60, 1.0);
      check("doubling rpm, no cavitation: +60 log10 2 = 18 dB", b.SL - a.SL, 17.9, 18.2, "dB");
      auto c = p3.eval(60, 0.7); check("cavitating propeller is louder than a quiet one (dB)", c.SL - b.SL, 15, 25, "dB");
      check("cavitation flag", c.cavitating ? 1.0 : 0.0, 0.5, 1.5, ""); check("quiet flag", b.cavitating ? 1.0 : 0.0, -0.5, 0.5, "");
      check("blade-rate tone stands above broadband (dB)", a.tone_SL - a.SL, 5, 12, "dB");
      double p10 = P_ATM + 1025 * G * 10, p0 = P_ATM;
      check("inception tip speed at 10 m depth (m/s)", cavitation_inception_tip_speed(p10, 1700, rho), 18, 21, "m/s");
      check("inception tip speed at the surface is lower", cavitation_inception_tip_speed(p0, 1700, rho) / cavitation_inception_tip_speed(p10, 1700, rho), 0.6, 0.8, "");
      check("passive sonar: SL 140, TL 60, NL 60, DI 10 -> SNR", passive_snr(140, 60, 60, 10), 29.9, 30.1, "dB"); }

    std::printf("\n== 4. layers in the sea ==\n");
    check("sound speed 25 C, 35 PSU, 1000 m (Mackenzie check value)", sound_speed(25, 35, 1000), 1550.0, 1551.5, "m/s");
    check("sound speed 10 C, 35 PSU, surface", sound_speed(10, 35, 0), 1488.5, 1491.0, "m/s");
    { Stratification s;
      check("warm above the thermocline (C)", s.T(2), 19.9, 20.0, "C"); check("cold below (C)", s.T(80), 8.0, 8.1, "C");
      check("density increases across the thermocline (kg/m3)", s.rho_at(60) - s.rho_at(5), 1.5, 4.0, "kg/m3");
      double N = s.N(30); check("buoyancy frequency at the thermocline centre", N, 0.02, 0.08, "rad/s");
      check("no stratification in the mixed layer (N)", s.N(0.5) < 0.005 ? 1.0 : 0.0, 0.5, 1.5, "");
      check("vehicle sinking through the layer gains weight (N, 30 L)", s.buoyancy_change(0.03, 5, 60), 0.4, 1.5, "N");
      check("wake collapse time at N = 0.05 (s)", Stratification::wake_collapse_time(0.05), 45, 55, "s");
      check("sound is slower below the thermocline", s.sound_at(60) - s.sound_at(5), -45, -15, "m/s"); }
    { WakeCloud c; c.tau_vortex = 1e9; double G1 = 0.05, s = 0.3;
      Vortex a, b; a.pos = {0, s / 2, 0}; b.pos = {0, -s / 2, 0}; a.axis = b.axis = {1, 0, 0}; a.gamma = G1; b.gamma = -G1; a.len = b.len = 1e4; a.rc0 = b.rc0 = 0.02; a.nut = b.nut = 0;
      c.vort = {a, b}; for (int i = 0; i < 100; ++i) c.step(0.05);
      double zc = 0.5 * (c.vort[0].pos.z + c.vort[1].pos.z);
      check("vortex pair sinks at G/(2 pi s) over 5 s (m)", -zc / (pair_descent_speed(G1, s) * 5), 0.8, 1.2, "");
      check("pair keeps its spacing (m)", c.vort[0].pos.y - c.vort[1].pos.y, 0.28, 0.32, "m");
      check("Crow wavelength 8.6 b", crow_wavelength(s), 2.57, 2.59, "m"); }

    std::printf("\n%s  (%d failed)\n\n", fails ? "SOME CHECKS FAILED" : "ALL CHECKS PASSED", fails);
    return fails ? 1 : 0;
}
