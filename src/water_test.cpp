// water_test.cpp - checks the water-disturbance physics (wake, flow field, surface waves, wave loads, proximity, scour).
// Build: make water_test     Run: ./water_test
#include "physics/seawater.hpp"
#include "physics/hull.hpp"
#include "physics/wake.hpp"
#include "physics/flowfield.hpp"
#include "physics/surface_waves.hpp"
#include "physics/wave_loads.hpp"
#include "physics/proximity.hpp"
#include <cstdio>
using namespace phys;

static int fails = 0;
static void check(const char* what, double got, double lo, double hi, const char* unit) {
    bool ok = got >= lo && got <= hi; if (!ok) ++fails;
    std::printf("  [%s] %-52s %11.5g %-6s (expected %.4g .. %.4g)\n", ok ? " OK " : "FAIL", what, got, unit, lo, hi);
}
static double remus_r(double x) {
    const double R = 0.0955;
    if (x > 0.5) { double t = (x - 0.5) / 0.3; return R * std::sqrt(std::max(0.0, 1 - t * t)); }
    if (x < -0.30) { double t = std::min(1.0, (-0.30 - x) / 0.42); return R * (1.0 - 0.62 * std::pow(t, 1.5)); }
    return R;
}

int main() {
    const double rho = 1025, nu = 1.19e-6;
    std::printf("\n== 1. propeller jet (actuator disc, T200-like: D 0.076 m, 40 N) ==\n");
    PropJet j; j.set({0, 0, 0}, {1, 0, 0}, 0.076, 40, 0.5, 0.0, rho, 1);
    double A = M_PI * 0.038 * 0.038;
    check("jet direction (vehicle pushed +x, water goes -x)", j.dir.x, -1.0001, -0.9999, "");
    check("final jet speed sqrt(2T/rho A)", j.u_far, std::sqrt(80 / (rho * A)) * 0.999, std::sqrt(80 / (rho * A)) * 1.001, "m/s");
    check("jet contracts to R*sqrt(1/2) at bollard", j.R_far / j.R, 0.70, 0.72, "");
    for (double xd : {2.0, 5.0, 10.0, 20.0}) {
        char b[80]; std::snprintf(b, sizeof b, "momentum flux at %.0f D / thrust", xd); check(b, j.momentum_flux(xd * 0.076) / 40.0, 0.92, 1.08, "");
    }
    double uc10 = j.centre_speed(10 * 0.076), uc20 = j.centre_speed(20 * 0.076), uc40 = j.centre_speed(40 * 0.076);
    check("centre speed falls ~1/x far away: u(40D)/u(20D)", uc40 / uc20, 0.45, 0.65, "");
    check("centre speed at 10 D (3-8 m/s range, bollard)", uc10, 1.0, 4.2, "m/s");
    PropJet jc; jc.set({0, 0, 0}, {1, 0, 0}, 0.076, 40, 0.5, 1.0, rho, 1);
    check("with 1 m/s co-flow the jet spreads slower", jc.kj, 0.02, j.kj * 0.99, "");
    check("momentum flux with co-flow at 8 D / thrust", jc.momentum_flux(8 * 0.076) / 40.0, 0.92, 1.08, "");
    // swirl: angular momentum flux = Q
    { double x = 5 * 0.076, b = j.radius_at(x), uc = j.centre_speed(x), sum = 0, dr = b * 0.01;
      double w0 = std::min(1.0, (j.u_disc - j.v0) / j.centre_speed(0)); uc *= w0 + (1 - w0) * (1 - std::exp(-x / (0.5 * j.R)));
      for (double r = 0.5 * dr; r < 5 * b; r += dr) { V3 v = j.velocity({-x, r, 0}); double vt = std::fabs(v.z); double ua = -v.x; sum += rho * ua * vt * r * 2 * M_PI * r * dr; }
      check("angular momentum flux of the swirl / torque", sum / 0.5, 0.9, 1.1, ""); (void)uc; }
    check("suction in front of the disc (inflow toward disc, -x)", j.velocity({0.038, 0, 0}).x, -4.0, -0.01, "m/s");
    check("suction decays upstream (3R vs 1R)", j.velocity({3 * 0.038, 0, 0}).x / j.velocity({1 * 0.038, 0, 0}).x, 0.0, 0.5, "");
    check("jet in reverse flips direction", PropJet().dir.x * 0 + [&] { PropJet r; r.set({0, 0, 0}, {1, 0, 0}, 0.076, -20, 0.2, 0, rho, 1); return r.dir.x; }(), 0.9999, 1.0001, "");

    std::printf("\n== 2. flow around a moving hull (slender-body potential flow) ==\n");
    Hull sp; sp.from_profile([](double x) { return 0.1 * std::sqrt(std::max(0.0, 1 - x * x)); }, -1, 1, 1, 1, 80);
    HullFlow hf; hf.set_hull(sp); hf.set_motion({1, 0, 0}, {});
    check("closed body: net source strength", hf.net_source(), -1e-9, 1e-9, "m3/s");
    double k1; { double e = std::sqrt(1 - 0.01), a0 = 2 * (1 - e * e) / (e * e * e) * (0.5 * std::log((1 + e) / (1 - e)) - e); k1 = a0 / (2 - a0); }
    double Vsp = sp.volume;
    check("far field on axis (4 half-lengths) u/U vs dipole V(1+k1)/(2 pi d^3)", hf.velocity({4, 0, 0}).x / (Vsp * (1 + k1) / (2 * M_PI * 64)), 0.9, 1.1, "");
    HullFlow hs; hs.set_hull(sp); hs.set_motion({0, 1, 0}, {});
    double mu = 2 * M_PI * 0.01 * (4.0 / 3.0);
    check("sideways motion, far field on the y axis vs doublet", hs.velocity({0, 4, 0}).y / (2 * mu / (4 * M_PI * 64)), 0.85, 1.15, "");
    check("water ahead of the nose is pushed forward", hf.velocity({1.1, 0, 0}).x, 1e-3, 1.0, "m/s");
    check("water beside midbody flows backwards (relative to water: x<0)", hf.velocity({0, 0.2, 0}).x, -1.0, -1e-3, "m/s");
    check("water behind the tail is drawn forward into the gap", hf.velocity({-1.1, 0, 0}).x, 1e-3, 1.0, "m/s");
    check("inside the hull the field is not used (0)", hf.velocity({0, 0.01, 0}).x, -1e-12, 1e-12, "");
    check("symmetry: sideways flow at +y and -y mirror (u_y at (0,.3)/(0,-.3))", hs.velocity({0.3, 0.25, 0}).y / hs.velocity({0.3, -0.25, 0}).y, 0.99, 1.01, "");
    { HullFlow ht; ht.set_hull(sp); ht.set_motion({0, 0, 0}, {0, 0, 0.5}); V3 a = ht.velocity({0.6, 0.2, 0}), b = ht.velocity({-0.6, 0.2, 0});
      check("turning: bow and stern push water opposite ways (y)", a.y * b.y, -1.0, -1e-9, ""); }

    std::printf("\n== 3. wake left behind (cloud) ==\n");
    { WakeCloud c; c.rho = rho; double U = 1.5, dt = 0.05, D = 10; double b0 = 0.1;
      for (int i = 0; i < 100; ++i) { double x = U * dt * i; c.hull_wake({x, 0, -2}, {1, 0, 0}, D, dt, b0); c.step(dt); }
      V3 p{U * dt * 90 - 0.5 * U * 1.0, 0, -2};          // ~1 s behind the vehicle
      V3 u = c.velocity_at(p);
      check("hull wake speed 1 s behind ~ D/(rho U pi b^2)", u.x, 0.12, 0.30, "m/s");
      check("wake is dragged in the direction of travel (+x)", u.x / (len(u) + 1e-12), 0.99, 1.01, "");
      V3 far = c.velocity_at({U * dt * 90 - 0.5 * U * 1.0, 3, -2});
      check("3 m to the side the wake has died away", len(far), 0, 0.01, "m/s");
      double u0 = c.max_speed(); for (int i = 0; i < 2000; ++i) c.step(0.05);          // 100 s later
      check("wake decays (100 s later / now)", c.max_speed() / u0, 0.0, 0.1, "");
      for (int i = 0; i < 5000; ++i) c.step(0.05);
      check("wake gone after a few minutes (blobs left)", (double)c.blobs.size(), 0, 5, ""); }
    { WakeCloud c; c.rho = rho; double dt = 0.05; for (int i = 0; i < 200; ++i) { c.hull_wake({0.075 * i, 0, 0}, {1, 0, 0}, 10, dt, 0.1); c.jet_end({0.075 * i - 1, 0, 0}, {-1, 0, 0}, 10, dt, 0.1); c.step(dt); }
      check("self-propelled (T = D): net impulse of the wake", len(c.net_impulse()), 0, 1e-6, "N s"); }
    { WakeCloud c; c.rho = rho; VehicleFlow vf; double U = 1.5, dt = 0.05; vf.set_pose({0, 0, -1}, Q{}, {U, 0, 0});
      for (int i = 0; i < 100; ++i) {
          vf.pos.x = U * dt * i; V3 tA = vf.pos + V3{0, 0.2, 0}, tB = vf.pos + V3{0, -0.2, 0};
          vf.emit_foil(c, tA, tB, {0, 0, 1}, 20.0, rho, dt); c.step(dt); }
      double span = 0.4, s = 0.785 * span, G1 = 20 / (rho * U * s);
      V3 mid{vf.pos.x - 1.0 * U, 0, -1}; V3 w = c.velocity_at(mid);
      check("tip vortices: downwash between them, 2G/(pi s)", -w.z / (2 * G1 / (M_PI * s)), 0.7, 1.3, "");
      V3 out = c.velocity_at({mid.x, 0.2 + 0.25, -1}); V3 outl = c.velocity_at({mid.x, -0.2 - 0.25, -1});
      check("upwash outside the tips (both sides)", std::min(out.z, outl.z), 0.001, 1.0, "m/s");
      check("circulation 20 N lift @1.5 m/s span .4 (G)", G1, 0.03, 0.06, "m2/s"); }
    { WakeCloud c; c.surface_z = 0; c.floor_z = -5; for (int i = 0; i < 4; ++i) c.bubbles({0, 0, -3}, 0.02, 0.1);
      for (int i = 0; i < 100; ++i) c.step(0.05);   // 5 s
      double zc = 0, n = 0; for (auto& p : c.part) { zc += p.pos.z; ++n; }
      check("bubbles have risen after 5 s (m)", zc / std::max(n, 1.0) + 3, 0.9, 1.3, "m");
      WakeCloud d; d.floor_z = -5; double ws3 = WakeCloud::settling_speed(3e-4), ws1 = WakeCloud::settling_speed(1e-4);
      check("sand 0.3 mm settling speed (Soulsby)", ws3, 0.03, 0.06, "m/s"); check("silt 0.1 mm settling speed", ws1, 0.005, 0.012, "m/s"); }

    std::printf("\n== 4. waves on the surface ==\n");
    { KelvinWake k; k.src = {{0, 1.0, 0.5}}; k.U = 2.0; k.x_ref = 0; k.pos = {0, 0, 0}; k.yaw = 0;
      double lam = 2 * M_PI * 4 / G; check("transverse wavelength 2 pi U^2/g", k.wavelength(), lam * 0.999, lam * 1.001, "m");
      // crest spacing along the track
      std::vector<double> cr; double prev = k.elevation(-3, 0), pp = prev;
      for (double x = -3.02; x > -30; x -= 0.02) { double e = k.elevation(x, 0); if (e < prev && prev > pp && prev > 0) cr.push_back(x + 0.02); pp = prev; prev = e; }
      double sp = 0; if (cr.size() > 3) sp = (cr.front() - cr.back()) / (cr.size() - 1);
      check("measured crest spacing along the track", sp, lam * 0.93, lam * 1.07, "m");
      auto rms = [&](double x0, double x1, double y, KelvinWake& kk) { double s = 0; int n = 0; for (double x = x0; x > x1; x -= 0.05) { double e = kk.elevation(x, y); s += e * e; ++n; } return std::sqrt(s / n); };
      double r10 = rms(-8, -12, 0, k), r40 = rms(-38, -42, 0, k);
      check("amplitude falls like 1/sqrt(r): rms(10 m)/rms(40 m)", r10 / r40, 1.6, 2.5, "");
      double amax = 0, ymax = 0; for (double y = 0; y < 30; y += 0.1) { double e = 0; for (double x = -30.5; x < -29.5; x += 0.05) e = std::max(e, std::fabs(k.elevation(x, y))); if (e > amax) { amax = e; ymax = y; } }
      check("wave energy peaks near the Kelvin wedge angle (y/x at 30 m)", ymax / 30.0, 0.27, 0.42, "");
      check("nothing ahead of the vehicle (rms ahead / rms behind)", rms(14, 10, 0, k) / r10, 0.0, 0.08, "");
      KelvinWake k2 = k; k2.src[0].h = 1.0; check("deeper source: amplitude ratio (h 1.0 m / 0.5 m)", rms(-8, -12, 0, k2) / r10, 0.2, 0.5, "");
      KelvinWake k3 = k; k3.src[0].m = 2.0; check("amplitude is linear in source strength", rms(-8, -12, 0, k3) / r10, 1.99, 2.01, "");
      KelvinWake k4 = k; k4.yaw = M_PI / 2; double e1 = k.elevation(-10, 0), e2 = k4.elevation(0, -10); check("rotating the vehicle rotates the pattern", e2 / e1, 0.999, 1.001, ""); }
    { Hull h; h.from_profile(remus_r, -0.76, 0.8, 1, 1, 40); KelvinWake k; k.set_hull(h, 2.0, {0, 0, 0}, 0, 0.3);
      double mx = 0; for (double x = -2; x > -15; x -= 0.05) for (double y = -2; y <= 2; y += 0.1) mx = std::max(mx, std::fabs(k.elevation(x, y)));
      std::printf("  [info] REMUS-size hull at 2 m/s, 0.3 m deep: biggest wake wave %.2f cm\n", mx * 100);
      check("wake of a REMUS-size hull at 0.3 m depth (cm scale)", mx * 100, 0.1, 15, "cm");
      KelvinWake kd; kd.set_hull(h, 2.0, {0, 0, 0}, 0, 3.0); double md = 0; for (double x = -2; x > -15; x -= 0.05) md = std::max(md, std::fabs(kd.elevation(x, 0)));
      check("same hull 3 m deep makes almost no surface wave (ratio)", md / mx, 0.0, 0.05, ""); }
    { Hull h; h.from_profile(remus_r, -0.76, 0.8, 1, 1, 40); HullFlow hf2; hf2.set_hull(h); hf2.set_motion({2, 0, 0}, {});
      Q q; V3 pos{0, 0, -0.3};
      double bow = near_field_eta(hf2, pos, q, {0.95, 0, 0}, 2.0), mid = near_field_eta(hf2, pos, q, {0.0, 0, 0}, 2.0);
      check("rise in front of the nose (cm)", bow * 100, 0.01, 21, "cm"); check("dip over the body (cm)", mid * 100, -21, -0.01, "cm"); }
    { PropJet jb; jb.set({0, 0, -0.08}, {1, 0, 0}, 0.076, 40, 0.5, 0, rho, 1);
      double e = 0; for (double x = -1; x < 1; x += 0.02) e = std::max(e, PropBoil::eta(jb, x, 0));
      check("jet just under the surface heaves it (cm)", e * 100, 0.5, 8.0, "cm");
      PropJet jd; jd.set({0, 0, -2.0}, {1, 0, 0}, 0.076, 40, 0.5, 0, rho, 1); double e2 = 0; for (double x = -1; x < 1; x += 0.02) e2 = std::max(e2, PropBoil::eta(jd, x, 0));
      check("the same jet 2 m deep does nothing", e2 * 100, 0, 1e-6, "cm");
      check("boil intensity shallow vs deep", PropBoil::intensity(jb) - PropBoil::intensity(jd), 0.05, 1.0, ""); }
    { RippleSystem rs; rs.emit(0, 0, 0, 0.01, 0.3); double cg = 0.5 * RippleSystem::phase_speed(0.3);
      double best = 0, rbest = 0; for (double r = 0.05; r < 8; r += 0.01) { double mx = 0; for (double t = 5.0; t < 5.0 + 0.1; t += 0.01) mx = std::max(mx, std::fabs(rs.elevation(r, 0, t))); if (mx > best) { best = mx; rbest = r; } }
      check("ring packet is at c_group * t after 5 s", rbest / (cg * 5.0), 0.9, 1.1, "");
      check("nothing inside the ring (r=0.3)", std::fabs(rs.elevation(0.3, 0, 5.0)), 0, 1e-3, "m");
      check("nothing far outside (r=6)", std::fabs(rs.elevation(6.0, 0, 5.0)), 0, 1e-4, "m");
      check("wavelength of 30 cm water waves: phase speed", RippleSystem::phase_speed(0.3), 0.6, 0.75, "m/s"); }
    { // all surface effects together, one vehicle 0.3 m deep at 2 m/s with a propeller 12 cm under the surface
      Hull h; h.from_profile(remus_r, -0.76, 0.8, 1, 1, 40); VehicleFlow vf; vf.hull.set_hull(h); vf.hull.set_motion({2, 0, 0}, {}); vf.set_pose({0, 0, -0.3}, Q{}, {2, 0, 0});
      std::vector<PropJet> jets(1); jets[0].set({-0.8, 0, -0.12}, {1, 0, 0}, 0.1, 30, 0.4, 2.0, rho, 1);
      SurfaceDisturbance sd; sd.update_vessel(0, vf, h, 0.0, 0.3);
      for (int i = 0; i < 100; ++i) sd.step(i * 0.05, 0.05, jets);
      double mx = 0, mn = 0; for (double x = -10; x < 2; x += 0.1) for (double y = -4; y <= 4; y += 0.1) { double e = sd.elevation(x, y, 5.0, jets); mx = std::max(mx, e); mn = std::min(mn, e); }
      check("combined surface elevation, highest crest (cm)", mx * 100, 0.1, 20, "cm"); check("combined surface elevation, deepest trough (cm)", mn * 100, -20, -0.1, "cm");
      check("ripple rings were emitted by the shallow propeller (count)", (double)sd.ripples.rings.size(), 1, 120, ""); }

    std::printf("\n== 5. sea waves on the vehicle ==\n");
    { double k = AirySea::wavenumber(2 * M_PI / 8.0, 1e5); check("deep water wavelength for T = 8 s (m)", 2 * M_PI / k, 99.5, 100.3, "m");
      double ks = AirySea::wavenumber(2 * M_PI / 8.0, 10.0); double w = 2 * M_PI / 8.0;
      check("finite depth dispersion w^2 = g k tanh(k d), d = 10 m", w * w / (G * ks * std::tanh(ks * 10.0)), 0.9999, 1.0001, "");
      AirySea s; s.add(1.0, 8.0, 0.0);
      double v0 = 0, vd = 0; for (double t = 0; t < 8; t += 0.02) { v0 = std::max(v0, std::fabs(s.velocity(0, 0, 0, t).x)); vd = std::max(vd, std::fabs(s.velocity(0, 0, -0.5 * 100, t).x)); }
      check("orbital speed at the surface = a w", v0 / (1.0 * w), 0.99, 1.01, "");
      check("at depth L/2 only exp(-pi) = 4.3 % is left", vd / v0, 0.040, 0.046, "");
      V3 a = s.acceleration(3, 0, -4, 1.3), an = (s.velocity(3, 0, -4, 1.3 + 1e-4) - s.velocity(3, 0, -4, 1.3 - 1e-4)) * (1 / 2e-4);
      check("acceleration = d(velocity)/dt (x)", a.x / an.x, 0.999, 1.001, ""); check("acceleration = d(velocity)/dt (z)", a.z / an.z, 0.999, 1.001, "");
      AirySea sf; sf.depth = 10; sf.add(1.0, 8.0, 0.0); double vb = 0, vt = 0; for (double t = 0; t < 8; t += 0.02) { vb = std::max(vb, std::fabs(sf.velocity(0, 0, -9.9, t).z)); vt = std::max(vt, std::fabs(sf.velocity(0, 0, -9.9, t).x)); }
      check("vertical motion vanishes at the sea floor (finite depth)", vb / v0, 0.0, 0.02, "");
      check("horizontal motion stays at the sea floor", vt / v0, 0.3, 1.0, "");
      Hull h; h.from_profile([](double) { return 0.1; }, -1, 1, 1, 1, 20); AirySea sy; sy.add(0.02, 8.0, M_PI / 2); WaveLoads wl; double fy = 0, ah = 0;
      for (double t = 0; t < 8; t += 0.02) { wl.eval(h, sy, t, {0, 0, -5}, Q{}, {}, {}, rho); fy = std::max(fy, std::fabs(wl.F.y)); ah = std::max(ah, std::fabs(sy.acceleration(0, 0, -5, t).y)); }
      check("horizontal wave force = 2 rho V a (circle, Ca = 1)", fy / (2 * rho * h.volume * ah), 0.9, 1.1, "");
      wl.eval(h, sy, 0.0, {0, 0, +5}, Q{}, {}, {}, rho); check("vehicle out of the water feels no wave load", std::fabs(wl.F.y), 0, 1e-12, "N"); }

    std::printf("\n== 6. wall and sea-bed proximity ==\n");
    { Hull h; h.from_profile([](double) { return 0.1; }, -1, 1, 1, 1, 20); Proximity p; Q q;
      p.eval(h, {0, 0, -10}, q, 2.0, rho, -20, 0, 1);
      check("far from everything: no suction (N)", std::fabs(p.F.z), 0, 0.1, "N"); check("far: added mass factor", p.ca_bed, 0.999, 1.001, "");
      p.eval(h, {0, 0, -9.75}, q, 2.0, rho, -10, 0, 1);
      double expect = M_PI * rho * 1e-4 * 4.0 / (4 * std::pow(0.25, 3)) * 2.0;
      check("0.25 m above the bed, a = 0.1 m: suction vs pi rho a^4 U^2/(4h^3) L", -p.F.z / expect, 0.97, 1.03, "");
      check("added mass up by 1 + (a/2h)^2 = 1.04", p.ca_bed, 1.035, 1.045, "");
      p.eval(h, {0, 0, -1.0}, q, 6.0, rho, -2.0, 0, 1); check("fast near the surface: lid effect fades out (Fr > 1)", p.f_lid, 0.0, 0.1, "");
      p.eval(h, {0, 0, -0.3}, q, 1.0, rho, -20.0, 0, 1); check("shallow slow vehicle is sucked toward the surface (+z)", p.F.z, 1.0, 1e4, "N");
      check("surface reduces fast heave added mass", p.ca_surf, 0.5, 0.99, ""); }
    { Scour sc; PropJet j2;
      j2.set({0, 0, 0.25}, {0.7, 0, 0.7}, 0.076, 40, 0.5, 0, rho, 1);           // water goes toward -axis = down and back
      auto o = sc.eval(j2, 0.0, rho, nu);
      check("critical Shields number for 0.3 mm sand (Soulsby)", o.theta_cr, 0.03, 0.055, "");
      check("jet 0.25 m above the bed: near-bed speed", o.u_bed, 0.3, 4.0, "m/s"); check("bed material is mobile", o.mobile ? 1.0 : 0.0, 0.5, 1.5, "");
      check("reference concentration (kg/m3)", o.conc, 0.001, 20.0, "kg/m3");
      PropJet j3; j3.set({0, 0, 5.0}, {0.7, 0, 0.7}, 0.076, 40, 0.5, 0, rho, 1); auto o3 = sc.eval(j3, 0.0, rho, nu);
      check("same jet 5 m above the bed: nothing moves", o3.mobile ? 1.0 : 0.0, -0.5, 0.5, "");
      PropJet j4; j4.set({0, 0, 0.25}, {0.7, 0, 0.7}, 0.076, 4, 0.05, 0, rho, 1); auto o4 = sc.eval(j4, 0.0, rho, nu);
      check("weak jet (4 N): bed speed is lower", o4.u_bed / o.u_bed, 0.0, 0.7, ""); }

    std::printf("\n== 7. the whole flow world ==\n");
    { FlowWorld fw; fw.current = {0.1, 0, 0}; Hull h; h.from_profile(remus_r, -0.76, 0.8, 1, 1, 40);
      VehicleFlow a, b; a.hull.set_hull(h); b.hull.set_hull(h); a.set_pose({0, 0, -3}, Q{}, {1.5, 0, 0}); b.set_pose({3, 0.5, -3}, Q{}, {0, 0, 0});
      a.hull.set_motion({1.5, 0, 0}, {}); a.jets.resize(1); a.jets[0].set({-0.9, 0, -3}, {1, 0, 0}, 0.1, 30, 0.4, 0.5, rho, 1);
      fw.vehicles = {&a, &b};
      V3 full = fw.velocity_at({3, 1.0, -3});
      V3 parts = fw.current + a.velocity({3, 1.0, -3}) + b.velocity({3, 1.0, -3});
      check("world velocity = current + every vehicle (x)", full.x - parts.x, -1e-12, 1e-12, "");
      V3 own = fw.velocity_at({-3, 0, -3}, &a), other = fw.velocity_at({-3, 0, -3});
      check("own displacement flow is not applied to itself", (len(other - own) > 0) ? 1.0 : 0.0, 0.5, 1.5, "");
      check("2nd vehicle sits in the jet of the 1st: |u| (m/s) at 2.1 m behind", len(a.jets[0].velocity({-3, 0, -3})), 0.05, 5.0, "");
      double ts = 0; for (int i = 0; i < 400; ++i) { a.set_pose({1.5 * 0.05 * i, 0, -3}, Q{}, {1.5, 0, 0}); a.emit(fw.cloud, 0.05, 12.0, nu); fw.step(0.05); ts += 0.05; }
      check("cloud filled by hull and jet emission (blobs)", (double)fw.cloud.blobs.size(), 10, 1500, "");
      std::printf("  [info] blobs in the cloud: %zu, vortices: %zu\n", fw.cloud.blobs.size(), fw.cloud.vort.size()); }

    std::printf("\n%s  (%d failed)\n\n", fails ? "SOME CHECKS FAILED" : "ALL CHECKS PASSED", fails);
    return fails ? 1 : 0;
}