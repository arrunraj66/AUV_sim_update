// physics/shaft.hpp - what turns the propeller: battery voltage -> ESC -> brushless motor -> gearbox -> shaft -> seal -> propeller.
//
//   motor   : DC-equivalent brushless model.  Ke = Kt = 60 / (2 pi Kv).   I = (V - Ke w_m) / R   (limited by the ESC),
//             torque = Kt (I - I0)   (I0 = no-load current: iron and bearing losses inside the motor)
//   gearbox : ratio (motor turns per propeller turn) and efficiency (loses power in both directions)
//   shaft   : inertia of rotor + shaft + propeller + the water the blades drag round with them;
//             bearing friction (Coulomb + viscous) and a SHAFT SEAL whose friction grows with the sea pressure
//   ESC     : "duty" mode (open loop, like a hobby ESC) or "rpm" mode (closed-loop speed governor, like a REMUS)
//   output  : shaft speed, motor current, electrical power taken from the battery.
// The motor is stiff (time constant of a few ms), so it is integrated in small sub-steps inside one physics step.
#pragma once
#include <cmath>
#include <algorithm>
#include <functional>

namespace phys {

struct Motor {
    double kv = 300;            // rpm per volt
    double R = 0.12;            // winding resistance, ohm
    double I0 = 0.5;            // no-load current, A
    double I_max = 25;          // ESC current limit, A
    double J = 2.0e-5;          // rotor inertia, kg m2
    double Ke() const { return 60.0 / (2 * M_PI * kv); }
};

struct Drive {
    Motor motor;
    double gear = 1.0, gear_eff = 1.0;               // motor turns per propeller turn
    double J_shaft = 5e-6, J_prop = 0;               // kg m2 (J_prop includes the entrained water; 0 = estimate it from the propeller)
    double fric_c = 0.003, fric_b = 1e-6;            // bearings: N m and N m s/rad
    double seal_c = 0.002, seal_kp = 0.004;          // shaft seal: N m, plus N m per MPa of sea pressure
    int mode = 1;                                    // 0 duty (open loop), 1 rpm governor
    double kp = 0.004, ki = 0.04;                    // governor gains (duty per rad/s, per rad)
    double esc_eff = 0.97, esc_idle = 0.4;           // ESC efficiency and standby power (W)
    // state
    double w = 0;                                     // propeller shaft speed, rad/s (signed)
    double I = 0, V = 0, duty = 0, P_elec = 0, I_bus = 0, tau_m = 0, integ = 0;

    double J_total() const { return motor.J * gear * gear + J_shaft + J_prop; }

    // cmd: duty (-1..1) in mode 0, wanted propeller rpm in mode 1.  Vbus: battery voltage.
    // Qload(w): torque the propeller asks for at shaft speed w.  p_gauge: sea pressure above atmosphere (Pa)
    void step(double dt, double cmd, double Vbus, const std::function<double(double)>& Qload, double p_gauge) {
        const double Ke = motor.Ke(), Kt = Ke, Jt = J_total();
        double c = gear * gear * Kt * Ke / motor.R;                       // back-EMF damping on the shaft
        double tau_e = Jt / std::max(c, 1e-12);
        int n = std::min(400, std::max(1, (int)std::ceil(dt / (0.4 * tau_e)))); double h = dt / n;
        double Psum = 0, Isum = 0;
        for (int k = 0; k < n; ++k) {
            if (mode == 1) {
                double wt = cmd * 2 * M_PI / 60.0, e = wt - w;
                double ff = wt * gear * Ke / std::max(Vbus, 1e-3);
                integ = std::max(-1.0, std::min(1.0, integ + ki * e * h));
                duty = std::max(-1.0, std::min(1.0, ff + kp * e + integ));
            } else duty = std::max(-1.0, std::min(1.0, cmd));
            V = duty * Vbus;
            double wm = w * gear;
            I = std::max(-motor.I_max, std::min(motor.I_max, (V - Ke * wm) / motor.R));
            double sgn = std::tanh(wm / 2.0);
            tau_m = Kt * (I - motor.I0 * sgn);
            double tp = tau_m * gear;
            tp *= (tp * w >= 0) ? gear_eff : 1.0 / gear_eff;
            double fr = (fric_c + seal_c + seal_kp * std::max(p_gauge, 0.0) * 1e-6) * std::tanh(w / 1.0) + fric_b * w;
            double dw = (tp - Qload(w) - fr) / Jt;
            w += h * dw;
            double Pm = V * I;                                              // power into the motor (negative = regenerating)
            Psum += Pm >= 0 ? Pm / esc_eff : Pm * esc_eff; Isum += I;
        }
        P_elec = Psum / n + esc_idle;
        I_bus = P_elec / std::max(Vbus, 1e-3);
        I = Isum / n;
    }
    double rpm() const { return w * 60.0 / (2 * M_PI); }
};

}  // namespace phys