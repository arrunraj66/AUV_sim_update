// physics/battery.hpp - lithium-ion battery pack (equivalent circuit model).
//
//   cell voltage = OCV(SOC) - I R0 - V1          V1: polarisation of one RC branch (R1, C1) -> slow voltage sag
//   OCV(SOC)     : typical NMC cell curve (3.0 V empty ... 4.18 V full)
//   R0 rises when the cell is cold or nearly empty;  the BMS cuts off at the minimum cell voltage.
//   Pack = Ns cells in series x Np in parallel.  Tracks state of charge, energy used, voltage and current.
#pragma once
#include <cmath>
#include <algorithm>

namespace phys {

struct Battery {
    int Ns = 4, Np = 1;                 // cells in series / parallel
    double cap_Ah = 18.0;               // capacity of ONE parallel group (Ah)
    double R0 = 0.010, R1 = 0.006, C1 = 2500.0;    // per cell: ohm, ohm, farad
    double V_cut = 3.0;                 // cell cut-off voltage
    double T = 15.0;                    // cell temperature (sea water cools the pack)
    // state
    double soc = 1.0, v1 = 0, V = 0, I = 0, used_Wh = 0; bool cut = false;

    static double ocv(double s) {
        static const double t[11] = {3.00, 3.45, 3.55, 3.62, 3.67, 3.72, 3.79, 3.86, 3.95, 4.06, 4.18};
        s = std::max(0.0, std::min(1.0, s)) * 10.0; int i = std::min(9, (int)s); double f = s - i;
        return t[i] * (1 - f) + t[i + 1] * f;
    }
    double r0_now() const { return R0 * (1.0 + 0.02 * std::max(0.0, 25.0 - T)) * (1.0 + 1.5 * std::max(0.0, 0.15 - soc) / 0.15); }
    double capacity_Wh() const { return cap_Ah * Np * Ns * 3.7; }
    double voltage_at(double I_pack) const {       // terminal voltage if I_pack flowed now (used before stepping)
        double i = I_pack / Np; return Ns * (ocv(soc) - i * r0_now() - v1);
    }
    // I_pack = current drawn by everything on the bus (A, + = discharge)
    void step(double I_pack, double dt) {
        if (cut) I_pack = 0;
        I = I_pack; double i = I / Np;
        soc -= I * dt / (3600.0 * cap_Ah * Np);
        soc = std::max(0.0, std::min(1.0, soc));
        v1 += dt * (i / C1 - v1 / (R1 * C1));
        V = Ns * (ocv(soc) - i * r0_now() - v1);
        used_Wh += V * I * dt / 3600.0;
        if (V / Ns < V_cut || soc <= 0.0) cut = true;
    }
};

}  // namespace phys