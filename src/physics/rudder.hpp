// physics/rudder.hpp - moving control surfaces: rudders, elevators, ailerons, X-tails, and the servo that drives them.
//
// Servo (electromechanical actuator), a second-order position loop with real limits:
//   * deflection limit (mechanical stops), no-load speed (rate limit), stall torque that falls with speed,
//   * the HYDRODYNAMIC HINGE MOMENT pushes back - at high speed a weak servo cannot reach the command,
//   * rotor + fin inertia, gear backlash (dead zone), power drawn from the battery.
// Control surface = fin geometry + servo + mixing:
//   delta_cmd = sign * (k_rudder * rudder + k_elevator * elevator + k_aileron * roll)
//   A "+" tail uses k = (1,0,0) or (0,1,0); an "X" tail uses 0.707 of both, so every fin helps steer and dive.
#pragma once
#include "fin.hpp"

namespace phys {

struct Servo {
    double max_angle = 0.45;          // rad, mechanical stops
    double max_rate = 1.0;            // rad/s, no-load speed
    double stall_torque = 3.0;        // N m at the fin shaft
    double err_sat = 0.035;           // rad of position error at which the motor gives its full torque (position-loop stiffness)
    double zeta = 0.9;                // damping of the position loop
    double J = 2e-4;                  // kg m2, rotor + gear + fin about the hinge
    double backlash = 0.004;          // rad, half width of the gear play
    double eff = 0.5, idle_W = 0.3;   // efficiency and standby power
    // state
    double angle = 0, rate = 0, torque = 0, power = 0; bool saturated = false;

    // cmd: wanted angle (rad), hinge: hydrodynamic hinge moment (N m, + tries to increase the angle)
    void step(double cmd, double hinge, double dt) {
        cmd = std::max(-max_angle, std::min(max_angle, cmd));
        double Kp = stall_torque / err_sat, Kd = 2 * zeta * std::sqrt(Kp * J);
        int n = std::max(1, (int)std::ceil(dt * std::sqrt(Kp / J) / 0.3)); double h = dt / n;
        double pw = 0;
        for (int i = 0; i < n; ++i) {
            double err = cmd - angle;
            if (std::fabs(err) < backlash) err = 0; else err -= (err > 0 ? backlash : -backlash);
            double want = Kp * err - Kd * rate - hinge;                                // torque the motor is asked for
            double avail = stall_torque * std::max(0.0, 1.0 - std::fabs(rate) / max_rate);
            if (rate * want < 0) avail = stall_torque;                                 // braking is always possible
            double tq = std::max(-avail, std::min(avail, want));
            saturated = std::fabs(want) > avail * 0.999;
            rate += h * (tq + hinge) / J;
            rate = std::max(-1.2 * max_rate, std::min(1.2 * max_rate, rate));
            angle += h * rate;
            if (angle > max_angle) { angle = max_angle; rate = std::min(rate, 0.0); }
            if (angle < -max_angle) { angle = -max_angle; rate = std::max(rate, 0.0); }
            torque = tq; pw += std::fabs(tq * rate) / eff;
        }
        power = pw / n + idle_W;
    }
};

struct ControlSurface {
    FinGeom fin; Servo servo;
    double sign = 1.0, k_rudder = 0, k_elevator = 0, k_aileron = 0;
    FinOut last;
    double command(double rudder, double elevator, double aileron) const {
        return sign * (k_rudder * rudder + k_elevator * elevator + k_aileron * aileron) * servo.max_angle;
    }
    // one time step: servo moves (fighting the hinge moment of the last step), then the new force is computed
    FinOut step(double rudder, double elevator, double aileron, V3 vrel, V3 w, V3 wash, double rho, double nu, double hyd, double dt) {
        servo.step(command(rudder, elevator, aileron), last.hinge, dt);
        last = fin_force(fin, vrel, w, wash, servo.angle, rho, nu, hyd);
        return last;
    }
};

}  // namespace phys