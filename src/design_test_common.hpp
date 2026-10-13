// design_test_common.hpp - helpers shared by design_spec_test.cpp and design_manoeuvre_test.cpp (calm water, speed hold, check macro).
#pragma once
#include "design_spec.hpp"
#include "design_io.hpp"
#include <cstdio>
#include <cmath>
#include <vector>

static int g_fails = 0, g_checks = 0, g_flags = 0;
#define CHECK(cond, ...) do { ++g_checks; if (!(cond)) { ++g_fails; std::printf("   FAIL: "); std::printf(__VA_ARGS__); std::printf("\n"); } } while (0)

inline Env calm_water(double rho) {
    Env e; e.rho = rho; e.eta = [](double, double) { return 0.0; }; e.ground = [](double, double) { return -1000.0; }; return e;
}
inline double rel_pct(double sim, double pred) { return pred != 0 ? 100.0 * (sim - pred) / std::fabs(pred) : 0.0; }

// holds the forward speed with a PI controller on the surge command (a stand-in for the autopilot's speed loop)
struct SpeedHold {
    double target = 0, kp = 0.6, ki = 0.25, integ = 0, cmd = 0;
    double update(double u, double dt) {
        double e = target - u; integ = std::max(-1.0, std::min(1.0, integ + ki * e * dt));
        cmd = std::max(-1.0, std::min(1.0, kp * e + integ)); return cmd;
    }
};
