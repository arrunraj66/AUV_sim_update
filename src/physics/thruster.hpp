// physics/thruster.hpp - a complete thruster on the vehicle: Drive (shaft.hpp) + Propeller (propeller.hpp) + where it sits.
//
//   * wake fraction w : a propeller at the tail of a hull sees slower water, Va = V (1 - w)
//   * thrust deduction t : the propeller sucks on the hull in front of it, net thrust = T (1 - t)
//   * reaction torque : the shaft torque twists the hull the other way (handed = +1 right-hand, -1 left-hand)
//   * thrust command  : a wanted force (N) is turned into a wanted rpm with the bollard curve, then the drive follows it
//   * slipstream      : jet speed and where it goes, for fins behind the propeller
#pragma once
#include "propeller.hpp"
#include "shaft.hpp"
#include "seawater.hpp"

namespace phys {

struct ThrusterUnit {
    V3 pos, dir = {1, 0, 0};           // propeller disc centre and direction of the thrust for positive rpm
    int handed = 1;
    double wake = 0.0, ded = 0.0;
    double rpm_max = 3000;
    Propeller prop; Drive drive;
    // results
    V3 F, M; double T = 0, Q = 0, Va = 0; Propeller::Out po;

    void setup(double rho = RHO_STD) { prop.build(rpm_max / 60.0 * 0.7, rho); if (drive.J_prop <= 0) drive.J_prop = 0.006 * rho * std::pow(prop.blade.D, 5); }
    double rated_thrust(double rho = RHO_STD) const { double KT, KQ, n = rpm_max / 60.0; prop.curves(n, 0, KT, KQ); return rho * n * n * std::pow(prop.blade.D, 4) * KT; }
    double rpm_for(double Fwant, double rho = RHO_STD) const {            // bollard curve, inverse
        double D = prop.blade.D, KTf, KTr, KQ; prop.curves(1, 0, KTf, KQ); prop.curves(-1, 0, KTr, KQ);
        double n = Fwant >= 0 ? std::sqrt(Fwant / (rho * std::pow(D, 4) * std::max(KTf, 1e-4))) : -std::sqrt(-Fwant / (rho * std::pow(D, 4) * std::max(KTr, 1e-4)));
        return std::max(-rpm_max, std::min(rpm_max, n * 60.0));
    }
    // v_prop: velocity of the propeller through the water (body frame); immersion: depth of the shaft under the surface
    void step(double dt, double F_cmd, double Vbus, V3 v_prop, double immersion, const WaterState& ws) {
        Va = dot(v_prop, dir) * (1.0 - wake);
        double cmd = drive.mode == 1 ? rpm_for(F_cmd, ws.rho) : F_cmd;    // in duty mode F_cmd is the duty itself
        double pg = ws.p - P_ATM;
        auto Qload = [&](double w) { return prop.eval(w / (2 * M_PI), Va, ws.rho, immersion, ws.p, ws.pv).Q; };
        drive.step(dt, cmd, Vbus, Qload, pg);
        po = prop.eval(drive.w / (2 * M_PI), Va, ws.rho, immersion, ws.p, ws.pv);
        T = po.T * (1.0 - ded); Q = po.Q;
        F = dir * T;
        M = cross(pos, F) - dir * (handed * Q);
    }
};

}  // namespace phys