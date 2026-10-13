// propwash_test.cpp - the bubble wake simulation (WashSim in propwash.hpp, no OpenGL needed):
//   no bubbles from a thruster at rest or out of the water; the rate grows with thrust; far more near the surface; bubbles start behind the propeller,
//   are carried by the current, rise, and pop at the surface; at speed the trail stays where the vehicle was.
// Build: cmake target propwash_test.   Run: ./propwash_test
#include "design_test_common.hpp"
#include "propwash.hpp"
#include "remus100.hpp"

static Fleet make_fleet(double depth, double speed = 0) {
    Fleet f; Unit u; u.id = 1; u.veh.s = make_spec_ex(M_TORPEDO); u.veh.rebuild(); u.veh.place({0, 0, -depth}, 0); u.veh.v = {speed, 0, 0};
    f.units.push_back(u); f.active = 0; return f;
}
static void run(WashSim& w, Fleet& f, double secs, double thrust_frac, double cur_x = 0, bool move = false, double dt = 0.02) {
    auto cur = [&](double, double& vx, double& vy) { vx = cur_x; vy = 0; };
    for (int i = 0; i < (int)(secs / dt); ++i) {
        Vehicle& v = f.units[0].veh; v.thrust[0] = thrust_frac * v.s.thr[0].max_thrust; if (!v.rot.empty()) v.rot[0].wet = 1;
        if (move) v.p.x += v.v.x * dt;
        w.update(dt, f, cur);
    }
}
int main() {
    std::printf("== bubble wake\n");
    // at rest, or out of the water: nothing
    { Fleet f = make_fleet(5); WashSim w; run(w, f, 3.0, 0.0); CHECK(w.emitted == 0 && w.alive() == 0, "a thruster at rest sheds no bubbles (%d)", w.emitted);
      Fleet g = make_fleet(5); WashSim w2; auto cur = [](double, double& x, double& y) { x = y = 0; }; g.units[0].veh.thrust[0] = 8; g.units[0].veh.rot[0].wet = 0.0; for (int i = 0; i < 100; ++i) w2.update(0.02, g, cur);
      CHECK(w2.emitted == 0, "a propeller out of the water sheds none (%d)", w2.emitted);
      Fleet h = make_fleet(5); WashSim w3; run(w3, h, 3.0, 0.02); CHECK(w3.emitted == 0, "below 3 %% thrust: none (%d)", w3.emitted); }
    // the rate grows with thrust (about thrust^1.3)
    { Fleet f1 = make_fleet(6), f2 = make_fleet(6); WashSim a, b; run(a, f1, 2.0, 1.0); run(b, f2, 2.0, 0.5);
      double ratio = (double)a.emitted / std::max(1, b.emitted); std::printf("   full thrust %d bubbles in 2 s, half thrust %d: ratio %.2f (thrust^1.3 gives %.2f)\n", a.emitted, b.emitted, ratio, std::pow(2.0, 1.3));
      CHECK(std::fabs(ratio - std::pow(2.0, 1.3)) < 0.35 * std::pow(2.0, 1.3), "emission ratio %.2f", ratio); CHECK(a.emitted > 1500, "a working propeller sheds a dense cloud (%d)", a.emitted); }
    // near the surface the propeller draws in air
    { Fleet fs = make_fleet(0.4), fd = make_fleet(8.0); WashSim a, b; run(a, fs, 2.0, 0.6); run(b, fd, 2.0, 0.6);
      std::printf("   at 0.4 m depth %d bubbles, at 8 m %d (x%.1f)\n", a.emitted, b.emitted, (double)a.emitted / std::max(1, b.emitted)); CHECK(a.emitted > 3 * b.emitted, "far more bubbles near the surface"); }
    // the bubbles start behind the propeller, rise, and are carried by the current
    { Fleet f = make_fleet(10); WashSim w; run(w, f, 0.5, 1.0, 0.0);
      V3 prop = f.units[0].veh.p + qrot(f.units[0].veh.q, f.units[0].veh.s.thr[0].pos); int behind = 0, n = 0;
      for (const Bubble& b : w.pool) if (b.alive) { ++n; if (b.x < prop.x + 0.02) ++behind; }
      CHECK(n > 100 && behind >= 0.97 * n, "bubbles start behind the propeller (%d of %d)", behind, n);
      Fleet g = make_fleet(10); WashSim w2; run(w2, g, 2.0, 0.0); int e0 = w2.emitted; (void)e0;
      auto cloud = [&](double cur_x, double& mz, double& mx) { Fleet h = make_fleet(10); WashSim w3; run(w3, h, 0.3, 1.0, cur_x); run(w3, h, 3.0, 0.0, cur_x); double z = 0, x = 0; int n = 0;
          for (const Bubble& b : w3.pool) if (b.alive) { z += b.z; x += b.x; ++n; } mz = n ? z / n : 0; mx = n ? x / n : 0; return n; };
      double z_still, x_still, z_cur, x_cur; int n_still = cloud(0.0, z_still, x_still), n_cur = cloud(0.5, z_cur, x_cur);
      CHECK(n_still > 0 && n_cur > 0, "bubbles are still there 3.3 s after the start (%d, %d)", n_still, n_cur);
      std::printf("   3.3 s after the start (thrust cut after 0.3 s): the cloud is %.2f m above where it started, and a 0.5 m/s current moves it %.2f m further than still water\n", z_still + 10, x_cur - x_still);
      CHECK(z_still + 10 > 0.15 && z_still + 10 < 1.3, "the bubbles rise (%.2f m)", z_still + 10); CHECK(x_cur - x_still > 1.0 && x_cur - x_still < 2.2, "the current carries them (%.2f m further in 3.3 s)", x_cur - x_still); }
    // they pop at the surface
    { Fleet f = make_fleet(0.6); WashSim w; run(w, f, 8.0, 0.5); int above = 0; for (const Bubble& b : w.pool) if (b.alive && b.z >= 0) ++above;
      CHECK(above == 0, "no bubble lives above the surface (%d)", above); CHECK(w.alive() < w.emitted, "bubbles pop (%d alive of %d emitted)", w.alive(), w.emitted); }
    // at speed the trail stays where the vehicle was
    { Fleet f = make_fleet(10, 2.0); WashSim w; run(w, f, 4.0, 0.7, 0.0, true); double mn = 1e9, mx = -1e9, sp = 0; int n = 0;
      for (const Bubble& b : w.pool) if (b.alive) { mn = std::min(mn, (double)b.x); mx = std::max(mx, (double)b.x); sp += std::sqrt(b.vx * b.vx + b.vy * b.vy); ++n; }
      std::printf("   moving at 2 m/s for 4 s: the trail spans %.1f m in x, mean bubble speed %.2f m/s\n", mx - mn, sp / n);
      CHECK(mx - mn > 4.5, "a trail %.1f m long is left behind", mx - mn); CHECK(sp / n < 0.7, "the wash moves slowly in the water (%.2f m/s)", sp / n); }
    // bounded
    { Fleet f = make_fleet(0.3); WashSim w(2000); run(w, f, 10.0, 1.0); CHECK(w.alive() <= (int)w.pool.size(), "the pool never overflows"); }
    std::printf("\n%d checks, %d failed\n", g_checks, g_fails);
    return g_fails ? 1 : 0;
}
