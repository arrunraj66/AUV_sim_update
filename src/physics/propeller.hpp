// physics/propeller.hpp - a whole propeller: blades (BEMT, blade.hpp) turned into open-water curves, plus the duct,
// cavitation, ventilation and the slipstream.
//
//   * At start-up the blades are solved by BEMT for advance ratios J = Va/(|n| D) from -1 to 1.6, for forward AND
//     reverse rotation, and stored as KT(J), KQ(J) tables. At run time thrust and torque are
//         T = rho n|n| D^4 KT(J)      Q = rho n|n| D^5 KQ(J)          (very fast, very stable)
//   * Duct (Kort nozzle): the nozzle itself makes thrust at low speed (up to ~45 % of the total at bollard pull)
//     and drag at high speed.
//   * Cavitation: cavitation number sigma = (p - pv) / (0.5 rho (Va^2 + (0.7 pi n D)^2)); below the breakdown value the
//     thrust and torque collapse.
//   * Ventilation: a propeller close to the surface sucks air down and loses thrust; in air it only spins.
//   * Slipstream: speed of the jet behind the disc (momentum theory), used by fins in the wash.
//   * calibrate(): scale the curves so the bollard pull matches a maker's datasheet at one rpm.
#pragma once
#include "blade.hpp"
#include <vector>

namespace phys {

struct Propeller {
    BladeGeom blade;
    double duct_k = 0.75, duct_J = 0.65;     // nozzle thrust at J=0 relative to the blades, and the J where it becomes zero
    double sigma_break = 0.25;               // cavitation breakdown number (small fast propellers 0.2-0.4)
    double k_cal = 1.0;                      // calibration factor on KT and KQ
    static constexpr double J0 = -1.0, J1 = 1.6, dJ = 0.05;
    std::vector<double> kt_f, kq_f, kt_r, kq_r;    // forward and reverse rotation tables

    static void smooth(std::vector<double>& v) {     // 3-point median: removes the odd unconverged point of the vortex-ring state
        std::vector<double> o = v;
        for (size_t i = 1; i + 1 < v.size(); ++i) { double a = v[i - 1], b = v[i], c = v[i + 1]; o[i] = std::max(std::min(a, b), std::min(std::max(a, b), c)); }
        v = o;
    }
    void build(double n_ref = 20.0, double rho = 1025.0, double nu = 1.19e-6) {
        int N = (int)std::round((J1 - J0) / dJ) + 1;
        kt_f.assign(N, 0); kq_f.assign(N, 0); kt_r.assign(N, 0); kq_r.assign(N, 0);
        double D = blade.D, a = rho * n_ref * n_ref;
        for (int i = 0; i < N; ++i) {
            double J = J0 + i * dJ;
            BemOut f = bem_solve(blade, n_ref, J * n_ref * D, rho, nu);
            BemOut r = bem_solve(blade, -n_ref, -J * n_ref * D, rho, nu);      // reverse: J measured along the reversed thrust
            double duct = blade.ducted ? 1.0 + duct_k * (1.0 - J / duct_J) : 1.0;
            if (blade.ducted && J > duct_J) duct = 1.0 - 0.25 * duct_k * (J - duct_J);      // nozzle drag at speed
            kt_f[i] = f.T / (a * std::pow(D, 4)) * std::max(duct, 0.0); kq_f[i] = f.Q / (a * std::pow(D, 5));
            kt_r[i] = -r.T / (a * std::pow(D, 4)) * std::max(1.0 + 0.5 * (duct - 1.0), 0.0); kq_r[i] = -r.Q / (a * std::pow(D, 5));   // stored as positive magnitudes
        }
        smooth(kt_r); smooth(kq_r); smooth(kt_f); smooth(kq_f);
    }
    static double lerp_tab(const std::vector<double>& t, double J) {
        double u = (std::max(J0, std::min(J1, J)) - J0) / dJ; int i = std::min((int)u, (int)t.size() - 2); double f = u - i;
        return t[i] * (1 - f) + t[i + 1] * f;
    }
    void curves(double n, double J, double& KT, double& KQ) const {
        if (n >= 0) { KT = lerp_tab(kt_f, J); KQ = lerp_tab(kq_f, J); }
        else { KT = lerp_tab(kt_r, J); KQ = lerp_tab(kq_r, J); }      // with n|n| < 0 these give negative thrust and torque
        KT *= k_cal; KQ *= k_cal;
    }
    // make the bollard pull equal T_ref (N) at rpm_ref
    void calibrate(double T_ref, double rpm_ref, double rho = 1025.0) {
        double n = rpm_ref / 60.0, KT, KQ; k_cal = 1.0; curves(n, 0.0, KT, KQ);
        k_cal = T_ref / (rho * n * n * std::pow(blade.D, 4) * KT);
    }

    struct Out { double T = 0, Q = 0, J = 0, KT = 0, KQ = 0, sigma = 99, cav = 1, vent = 1, jet = 0; };
    // n: rev/s (signed), Va: water arriving along +thrust axis, immersion: depth of the shaft under the surface (m),
    // p: static pressure at the propeller (Pa), pv: vapour pressure
    Out eval(double n, double Va, double rho, double immersion, double p, double pv) const {
        Out o; double D = blade.D, an = std::fabs(n);
        if (an < 1e-4) return o;
        double J = (n >= 0 ? Va : -Va) / (an * D);
        curves(n, J, o.KT, o.KQ); o.J = J;
        double vt = 0.7 * M_PI * an * D;
        o.sigma = (p - pv) / (0.5 * rho * (Va * Va + vt * vt));
        o.cav = o.sigma >= sigma_break ? 1.0 : std::max(0.15, std::pow(o.sigma / sigma_break, 0.7));
        double R = 0.5 * D, hr = immersion / R;
        o.vent = std::max(0.0, std::min(1.0, (hr + 1.0) / 2.2));             // starts losing thrust at 1.2 R, nothing when fully out
        double k = o.cav * o.vent;
        o.T = rho * n * an * std::pow(D, 4) * o.KT * k;
        o.Q = rho * n * an * std::pow(D, 5) * o.KQ * (0.04 + 0.96 * k);        // in air the blades still feel a little drag
        double A = M_PI * R * R, v0 = std::max(Va, 0.0);
        o.jet = std::sqrt(v0 * v0 + 2.0 * std::fabs(o.T) / (rho * A)) - v0;
        return o;
    }
};

}  // namespace phys