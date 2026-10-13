// physics/acoustics.hpp - the sound a vehicle makes and how far it carries.  (for the sonar / passive-listening side of the simulator)
//   thorp_alpha      : sea water absorption (Thorp), dB/km, f in kHz
//   transmission_loss: spherical spreading + absorption
//   PropNoise        : blade-rate tone and harmonics + broadband noise.  Source level follows scaling laws
//                        non-cavitating : rotor dipole noise ~ V_tip^6     -> 60 log10(V_tip)
//                        cavitating     : a large extra rise as the cavitation develops   (Ross, 1976, qualitatively)
//                      SL_ref and cav_boost are CALIBRATION constants: set them from a measured thruster, the shape is physics, the
//                      absolute level is an engineering estimate.
//   cavitation_inception_tip_speed : V_tip at which sigma = (p - pv)/(1/2 rho V^2) drops to the inception number
//   passive_snr      : passive sonar equation  SNR = SL - TL - NL + DI
#pragma once
#include <cmath>
#include <algorithm>

namespace phys {

inline double thorp_alpha(double f_kHz) {                       // dB per km
    double f2 = f_kHz * f_kHz;
    return 0.11 * f2 / (1 + f2) + 44 * f2 / (4100 + f2) + 2.75e-4 * f2 + 0.003;
}
inline double transmission_loss(double range_m, double f_kHz) { return 20 * std::log10(std::max(range_m, 1.0)) + thorp_alpha(f_kHz) * range_m / 1000.0; }
inline double passive_snr(double SL, double TL, double NL, double DI = 0) { return SL - TL - NL + DI; }
inline double cavitation_inception_tip_speed(double p_static, double p_vapour, double rho, double sigma_i = 1.0) {
    return std::sqrt(2 * (p_static - p_vapour) / (rho * sigma_i));
}

struct PropNoise {
    int Z = 3; double D = 0.076, SL_ref = 125.0, cav_boost = 20.0;          // dB re 1 uPa at 1 m
    struct Out { double SL = 0, f_bpf = 0, tone_SL = 0, f_peak = 0; bool cavitating = false; };
    double blade_rate(double rpm) const { return Z * std::fabs(rpm) / 60.0; }
    // n: rev/s, cav: cavitation thrust factor from Propeller::Out (1 = none, < 1 = cavitating), Vtip: tip speed (m/s)
    Out eval(double n, double cav) const {
        Out o; double an = std::fabs(n), Vtip = M_PI * an * D; if (Vtip < 0.5) return o;
        double area = 10 * std::log10((Z * D * D) / (3 * 0.076 * 0.076));
        double nc = SL_ref + 60 * std::log10(Vtip / 10.0) + area;
        double frac = std::min(1.0, std::max(0.0, (1.0 - cav) / 0.15));      // cavitation develops as the thrust factor drops
        o.cavitating = frac > 0; o.SL = nc + cav_boost * frac;
        o.f_bpf = Z * an; o.tone_SL = nc + 8;                                // the blade-rate tone stands ~8 dB above the broadband level
        o.f_peak = std::max(100.0, 4000.0 / std::max(1.0, Vtip / 3.0));      // cavitation noise peak moves down for bigger, slower props
        return o;
    }
};

}  // namespace phys
