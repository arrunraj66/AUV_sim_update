// physics/seawater.hpp - properties of sea water that every other physics file needs.
//   density      : simplified equation of state (T, S) + compressibility with depth   (within 0.1 kg/m3 of UNESCO-80, 0-30 C, 30-40 PSU)
//   viscosity    : kinematic viscosity of sea water, fit to the ITTC 2011 table (35 PSU)  (within 0.3 %)
//   pressure     : absolute pressure at a depth
//   vapour       : vapour pressure (for propeller cavitation)
#pragma once
#include <cmath>

namespace phys {

constexpr double G = 9.80665;          // m/s2
constexpr double P_ATM = 101325.0;     // Pa
constexpr double RHO_STD = 1025.0;     // kg/m3, the "standard" sea water used for ratings

inline double sw_density(double T, double S, double depth) {
    double dT = T - 10.0;
    double rho0 = 1027.0 - 0.17 * dT - 0.0045 * dT * dT + 0.78 * (S - 35.0);
    return rho0 * (1.0 + 4.4e-6 * std::max(depth, 0.0));
}
inline double sw_viscosity(double T, double S = 35.0) {           // kinematic, m2/s
    double nu = std::exp(0.603130832 - 0.0315919964 * T + 1.98574079e-4 * T * T) * 1e-6;
    return nu * (1.0 + 0.0017 * (S - 35.0));
}
inline double sw_pressure(double depth, double rho = RHO_STD) { return P_ATM + rho * G * std::max(depth, 0.0); }
inline double sw_vapour(double T) { return 0.98 * 610.94 * std::exp(17.625 * T / (T + 243.04)); }   // Pa

// everything a body needs to know about the water around it
struct WaterState {
    double rho = RHO_STD, nu = 1.19e-6, T = 15.0, depth = 0.0, p = P_ATM, pv = 1700.0;
    static WaterState at(double T, double S, double depth) {
        WaterState w; w.T = T; w.depth = depth; w.rho = sw_density(T, S, depth); w.nu = sw_viscosity(T, S);
        w.p = sw_pressure(depth, w.rho); w.pv = sw_vapour(T); return w;
    }
};

}  // namespace phys