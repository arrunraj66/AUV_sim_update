// physics/stratification.hpp - layers in the sea (thermocline), what they do to buoyancy, wakes and sound, plus vortex-pair helpers.
//   Stratification : T(z) with a tanh thermocline, density, buoyancy frequency N (internal wave frequency), sound speed (Mackenzie 1981)
//                    - a vehicle crossing the thermocline changes buoyancy by (rho2 - rho1) g V: use rho_at(depth) in hydrostatics
//                    - wakes collapse in a stratified sea after N t ~ 2.5 (Lin & Pao 1979) -> wake_collapse_time
//                    - sound bends toward lower sound speed: a thermocline makes a shadow zone (sonar)
//   Vortex pair    : the pair of tip vortices sinks at G/(2 pi s) and breaks into rings through Crow instability (wavelength ~ 8.6 b)
#pragma once
#include "seawater.hpp"
#include <cmath>
#include <algorithm>

namespace phys {

inline double sound_speed(double T, double S, double D) {                 // Mackenzie (1981), m/s, D in m
    return 1448.96 + 4.591 * T - 5.304e-2 * T * T + 2.374e-4 * T * T * T + 1.340 * (S - 35) + 1.630e-2 * D + 1.675e-7 * D * D
           - 1.025e-2 * T * (S - 35) - 7.139e-13 * T * D * D * D;
}

struct Stratification {
    double T_top = 20, T_bot = 8, S = 35, z_th = 30, width = 8;            // temperatures (C), thermocline centre depth and half width (m)
    double T(double depth) const { return T_bot + (T_top - T_bot) * 0.5 * (1 - std::tanh((depth - z_th) / width)); }
    double rho_at(double depth) const { return sw_density(T(depth), S, depth); }
    double sound_at(double depth) const { return sound_speed(T(depth), S, depth); }
    double N2(double depth) const {                                          // buoyancy frequency squared (1/s2), > 0 = stable
        double e = 0.25, r0 = rho_at(depth - e) - 4.4e-6 * rho_at(depth - e) * (depth - e), r1 = rho_at(depth + e) - 4.4e-6 * rho_at(depth + e) * (depth + e);
        return G / rho_at(depth) * (r1 - r0) / (2 * e);                      // compression removed: only the temperature layering counts
    }
    double N(double depth) const { return std::sqrt(std::max(0.0, N2(depth))); }
    double buoyancy_change(double volume, double from_depth, double to_depth) const { return (rho_at(to_depth) - rho_at(from_depth)) * G * volume; }
    static double wake_collapse_time(double N) { return N > 1e-4 ? 2.5 / N : 1e9; }          // s
    static double wake_life(double N) { return std::min(240.0, std::max(20.0, 6.0 / std::max(N, 1e-4))); }
};

inline double pair_descent_speed(double gamma, double spacing) { return std::fabs(gamma) / (2 * M_PI * spacing); }
inline double crow_wavelength(double spacing) { return 8.6 * spacing; }
inline double crow_growth_rate(double gamma, double spacing) { return 0.8 * std::fabs(gamma) / (2 * M_PI * spacing * spacing); }   // 1/s, fastest mode (approximate)

}  // namespace phys
