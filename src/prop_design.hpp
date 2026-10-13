// prop_design.hpp - propeller, shaft and fin DESIGN rules from the open literature.
//
//  1. Wageningen B-series open-water polynomials (Oosterveld & van Oossanen 1975): KT and KQ from J, P/D, AE/A0 and the number of blades Z.
//     Valid for Z 2..7, P/D 0.5..1.4, AE/A0 0.3..1.05, at Reynolds number 2e6 on the blade (small AUV propellers run lower: expect a few % less).
//  2. Cavitation: Keller's minimum blade-area rule  AE/A0 >= (1.3 + 0.3 Z) T / ((p0 - pv) D^2) + K   (K = 0.2 single screw), and the tip speed.
//  3. Contra-rotating (dual) propeller: a second, counter-turning propeller in the slipstream of the first. It recovers the swirl (published gains
//     about 10 %), cancels the roll torque, but needs a coaxial shaft (mechanical loss). Modelled with momentum theory + B-series for each rotor.
//  4. Propeller shaft: torsion + keyway + thrust with von Mises stress, the classification-society formula (IACS UR M68) for comparison, and
//     the first lateral critical speed of the overhung propeller.
//  5. Fin: planform from area / aspect ratio / taper, stock (hinge shaft) diameter from bending + torsion, servo torque class.
#pragma once
#include <cmath>
#include <algorithm>
#include <string>
#include <vector>

namespace pdz {

// ---- 1. B-series ----------------------------------------------------------------------------------------
struct Term { double C; int s, t, u, v; };
static const Term KT_TERMS[39] = {
    {0.00880496, 0, 0, 0, 0}, {-0.204554, 1, 0, 0, 0}, {0.166351, 0, 1, 0, 0}, {0.158114, 0, 2, 0, 0}, {-0.147581, 2, 0, 1, 0}, {-0.481497, 1, 1, 1, 0},
    {0.415437, 0, 2, 1, 0}, {0.0144043, 0, 0, 0, 1}, {-0.0530054, 2, 0, 0, 1}, {0.0143481, 0, 1, 0, 1}, {0.0606826, 1, 1, 0, 1}, {-0.0125894, 0, 0, 1, 1},
    {0.0109689, 1, 0, 1, 1}, {-0.133698, 0, 3, 0, 0}, {0.00638407, 0, 6, 0, 0}, {-0.00132718, 2, 6, 0, 0}, {0.168496, 3, 0, 1, 0}, {-0.0507214, 0, 0, 2, 0},
    {0.0854559, 2, 0, 2, 0}, {-0.0504475, 3, 0, 2, 0}, {0.010465, 1, 6, 2, 0}, {-0.00648272, 2, 6, 2, 0}, {-0.00841728, 0, 3, 0, 1}, {0.0168424, 1, 3, 0, 1},
    {-0.00102296, 3, 3, 0, 1}, {-0.0317791, 0, 3, 1, 1}, {0.018604, 1, 0, 2, 1}, {-0.00410798, 0, 2, 2, 1}, {-0.000606848, 0, 0, 0, 2}, {-0.0049819, 1, 0, 0, 2},
    {0.0025983, 2, 0, 0, 2}, {-0.000560528, 3, 0, 0, 2}, {-0.00163652, 1, 2, 0, 2}, {-0.000328787, 1, 6, 0, 2}, {0.000116502, 2, 6, 0, 2}, {0.000690904, 0, 0, 1, 2},
    {0.00421749, 0, 3, 1, 2}, {0.0000565229, 3, 6, 1, 2}, {-0.00146564, 0, 3, 2, 2}};
static const Term KQ_TERMS[47] = {
    {0.00379368, 0, 0, 0, 0}, {0.00886523, 2, 0, 0, 0}, {-0.032241, 1, 1, 0, 0}, {0.00344778, 0, 2, 0, 0}, {-0.0408811, 0, 1, 1, 0}, {-0.108009, 1, 1, 1, 0},
    {-0.0885381, 2, 1, 1, 0}, {0.188561, 0, 2, 1, 0}, {-0.00370871, 1, 0, 0, 1}, {0.00513696, 0, 1, 0, 1}, {0.0209449, 1, 1, 0, 1}, {0.00474319, 2, 1, 0, 1},
    {-0.00723408, 2, 0, 1, 1}, {0.00438388, 1, 1, 1, 1}, {-0.0269403, 0, 2, 1, 1}, {0.0558082, 3, 0, 1, 0}, {0.0161886, 0, 3, 1, 0}, {0.00318086, 1, 3, 1, 0},
    {0.015896, 0, 0, 2, 0}, {0.0471729, 1, 0, 2, 0}, {0.0196283, 3, 0, 2, 0}, {-0.0502782, 0, 1, 2, 0}, {-0.030055, 3, 1, 2, 0}, {0.0417122, 2, 2, 2, 0},
    {-0.0397722, 0, 3, 2, 0}, {-0.00350024, 0, 6, 2, 0}, {-0.0106854, 3, 0, 0, 1}, {0.00110903, 3, 3, 0, 1}, {-0.000313912, 0, 6, 0, 1}, {0.0035985, 3, 0, 1, 1},
    {-0.00142121, 0, 6, 1, 1}, {-0.00383637, 1, 0, 2, 1}, {0.0126803, 0, 2, 2, 1}, {-0.00318278, 2, 3, 2, 1}, {0.00334268, 0, 6, 2, 1}, {-0.00183491, 1, 1, 0, 2},
    {0.000112451, 3, 2, 0, 2}, {-0.0000297228, 3, 6, 0, 2}, {0.000269551, 1, 0, 1, 2}, {0.00083265, 2, 0, 1, 2}, {0.00155334, 0, 2, 1, 2}, {0.000302683, 0, 6, 1, 2},
    {-0.0001843, 0, 0, 2, 2}, {-0.000425399, 0, 3, 2, 2}, {0.0000869243, 3, 3, 2, 2}, {-0.0004659, 0, 6, 2, 2}, {0.0000554194, 1, 6, 2, 2}};

inline double cl(double v, double a, double b) { return std::max(a, std::min(b, v)); }

// KT and KQ at advance ratio J (the polynomial is cubic in J; J is allowed to go slightly negative or past KT = 0)
inline void bseries(double J, double PD, double EAR, int Z, double& KT, double& KQ) {
    PD = cl(PD, 0.5, 1.4); EAR = cl(EAR, 0.3, 1.05); double z = cl(Z, 2, 7);
    double ct[4] = {0, 0, 0, 0}, cq[4] = {0, 0, 0, 0};
    for (const Term& t : KT_TERMS) ct[t.s] += t.C * std::pow(PD, t.t) * std::pow(EAR, t.u) * std::pow(z, t.v);
    for (const Term& t : KQ_TERMS) cq[t.s] += t.C * std::pow(PD, t.t) * std::pow(EAR, t.u) * std::pow(z, t.v);
    J = cl(J, -0.2, 1.6);
    KT = ct[0] + J * (ct[1] + J * (ct[2] + J * ct[3]));
    KQ = cq[0] + J * (cq[1] + J * (cq[2] + J * cq[3]));
}

// ---- a propeller (one rotor), with an optional Kort nozzle ------------------------------------------------
struct Rotor1 {
    int Z = 3; double D = 0.19, PD = 1.0, EAR = 0.55; bool duct = false; double duct_k = 0.30;
    // thrust T and shaft torque Q at rpm and advance speed Va (water arriving along the thrust direction)
    void eval(double rpm, double Va, double rho, double& T, double& Q) const {
        double n = rpm / 60.0, an = std::fabs(n);
        if (an < 1e-4) { T = Q = 0; return; }
        double J = Va / (an * D), KT, KQ; bseries(J, PD, EAR, Z, KT, KQ);
        if (J > 1.6) { KT = std::min(KT, 0.0); }
        double dk = duct ? std::max(0.0, 1.0 + duct_k * (1.0 - J / 0.65)) : 1.0;            // the nozzle adds thrust at low speed, drag at high speed
        double sg = n >= 0 ? 1.0 : -1.0;
        T = sg * rho * n * n * std::pow(D, 4) * KT * dk; Q = sg * rho * n * n * std::pow(D, 5) * KQ;
    }
    double efficiency(double J) const { double KT, KQ; bseries(J, PD, EAR, Z, KT, KQ); return KQ > 1e-9 && KT > 0 ? J * KT / (2 * M_PI * KQ) : 0.0; }
};

// ---- 2. cavitation and tip speed -------------------------------------------------------------------------
inline double keller_ear(double T, int Z, double D, double p0, double pv, double K = 0.2) { return (1.3 + 0.3 * Z) * std::max(T, 0.0) / ((p0 - pv) * D * D) + K; }
inline double tip_speed(double D, double rpm) { return M_PI * D * rpm / 60.0; }

// ---- 3. contra-rotating (dual) propeller -----------------------------------------------------------------
struct CRP {
    Rotor1 a, b;                          // a: front rotor, b: rear rotor (turns the other way)
    double swirl_gain = 0.08;             // extra thrust of the rear rotor from the swirl it recovers (published gains about 10 %)
    double mech_eff = 0.97;               // coaxial shaft and bearings
    double gap = 0.25;                    // spacing between the discs / D
    struct Out { double T = 0, Q1 = 0, Q2 = 0, Qnet = 0, P = 0, rpm2 = 0, eta = 0; };
    // front rotor at rpm1; the rear rotor's rpm is solved so that both torques cancel (no roll)
    Out eval(double rpm1, double Va, double rho) const {
        Out o; double T1, Q1; a.eval(rpm1, Va, rho, T1, Q1);
        double A = M_PI * 0.25 * a.D * a.D, u1 = 0;                                          // induced velocity of the front rotor (momentum theory)
        double Tpos = std::max(T1, 0.0); u1 = 0.5 * (-Va + std::sqrt(Va * Va + 2 * Tpos / (rho * A)));
        double Va2 = Va + (1.0 + std::min(1.0, 0.4 + gap)) * u1;                              // the rear rotor sits in the accelerated, contracting jet
        double lo = 0, hi = 4.0 * std::fabs(rpm1) + 100; double T2 = 0, Q2 = 0;
        for (int i = 0; i < 50; ++i) { double mid = 0.5 * (lo + hi), t, q; b.eval(mid, Va2, rho, t, q); (q < Q1 ? lo : hi) = mid; }
        o.rpm2 = 0.5 * (lo + hi); b.eval(o.rpm2, Va2, rho, T2, Q2);
        o.T = T1 + T2 * (1.0 + swirl_gain); o.Q1 = Q1; o.Q2 = Q2; o.Qnet = Q1 - Q2;
        double w1 = 2 * M_PI * rpm1 / 60, w2 = 2 * M_PI * o.rpm2 / 60; o.P = (Q1 * w1 + Q2 * w2) / mech_eff;
        o.eta = o.P > 1e-9 ? o.T * Va / o.P : 0.0; return o;
    }
};

// ---- 4. shaft ---------------------------------------------------------------------------------------------
struct ShaftMat { const char* name; double rho, yield, uts, E; };            // kg/m3, Pa, Pa, Pa
static const ShaftMat SHAFT_MATS[4] = {
    {"316 stainless (annealed)", 8000, 205e6, 515e6, 193e9}, {"17-4PH stainless (H1025)", 7800, 1000e6, 1070e6, 197e9},
    {"Titanium Ti-6Al-4V", 4430, 880e6, 950e6, 114e9}, {"Aluminium 7075-T6", 2810, 500e6, 570e6, 71.7e9}};

struct ShaftOut { double d_torsion = 0, d_vonmises = 0, d_iacs = 0, d = 0, Q_design = 0, sigma_eq = 0, n_crit = 0, margin = 0, mass = 0; bool crit_ok = true; };

// P: rated shaft power (W), rpm, thrust at that point (N), m_prop: propeller + hub mass (kg), a: overhang of the propeller beyond the last bearing (m),
// L: shaft length (m), keyway: has a key way (k 1.10 as IACS), bore: di/do for a hollow shaft (0 = solid)
inline ShaftOut shaft_design(double P, double rpm, double thrust, double m_prop, double a, double L, int mat, bool keyway, double bore, double max_rpm) {
    const ShaftMat& m = SHAFT_MATS[std::max(0, std::min(3, mat))];
    ShaftOut o; double w = 2 * M_PI * rpm / 60.0;
    double Q = P / std::max(w, 1e-6);
    o.Q_design = 1.5 * Q;                                            // starting / stall overload factor
    double k = keyway ? 1.10 : 1.0, SF = 2.0, tau = 0.577 * m.yield / SF;
    double hol = 1.0 / (1.0 - std::pow(bore, 4));
    o.d_torsion = std::cbrt(16.0 * o.Q_design * k * hol / (M_PI * tau));
    double Mb = m_prop * 9.80665 * 0.3 * a + 0.12 * thrust * a;      // bending of the overhung propeller: weight (wet, partly buoyant) + a side load of 12 % of the thrust
    double d = std::max(o.d_torsion, 0.003);
    for (int i = 0; i < 40; ++i) {                                   // von Mises with torsion, bending and thrust
        double sig_b = 32 * Mb / (M_PI * d * d * d), sig_a = 4 * thrust / (M_PI * d * d), tau_t = 16 * o.Q_design * k / (M_PI * d * d * d);
        double se = std::sqrt((sig_b + sig_a) * (sig_b + sig_a) + 3 * tau_t * tau_t) * std::sqrt(hol);
        if (se > m.yield / SF) d *= 1.04; else break;
    }
    o.d_vonmises = d;
    double Pk = P / 1000.0, n = std::max(rpm, 1.0);                  // IACS UR M68:  d = F k (P/n * 1/(1-(di/do)^4) * 560/(Rm+160))^(1/3), mm, kW, rpm, MPa
    o.d_iacs = 1e-3 * 100.0 * k * std::cbrt(Pk / n * hol * 560.0 / (m.uts / 1e6 + 160.0));
    o.d = std::max({o.d_torsion, o.d_vonmises, o.d_iacs, 0.004});   // never thinner than 4 mm (bearings and seals)
    double sig_b = 32 * Mb / (M_PI * o.d * o.d * o.d), sig_a = 4 * thrust / (M_PI * o.d * o.d), tau_t = 16 * o.Q_design * k / (M_PI * o.d * o.d * o.d);
    o.sigma_eq = std::sqrt((sig_b + sig_a) * (sig_b + sig_a) + 3 * tau_t * tau_t);
    double I = M_PI * std::pow(o.d, 4) / 64.0 * (1 - std::pow(bore, 4)), msh = m.rho * M_PI * 0.25 * o.d * o.d * (1 - bore * bore) * L;
    double kst = 3 * m.E * I / std::pow(std::max(a, 0.01), 3), meff = m_prop + 0.23 * msh * a / std::max(L, a);
    o.n_crit = 60.0 / (2 * M_PI) * std::sqrt(kst / meff);             // first lateral critical speed of an overhung disc, rpm
    o.margin = o.n_crit / std::max(max_rpm, 1.0); o.crit_ok = o.margin > 1.4;
    o.mass = msh; return o;
}

// ---- 5. fins ----------------------------------------------------------------------------------------------
struct FinPlan { double cr, ct, span, mac, ar; };
// planform from area, aspect ratio and taper ratio ct/cr
inline FinPlan fin_planform(double area, double ar, double taper) {
    FinPlan p; p.span = std::sqrt(std::max(area * ar, 1e-9)); double cm = area / p.span; p.cr = 2 * cm / (1 + taper); p.ct = p.cr * taper;
    p.mac = (2.0 / 3.0) * p.cr * (1 + taper + taper * taper) / (1 + taper); p.ar = ar; return p;
}
struct StockOut { double d = 0, sigma = 0, servo_Nm = 0, servo_kg = 0; };
// F: normal force of the fin (N) at full deflection and top speed, arm: distance from the stock to the centre of pressure along the span (m),
// hinge: hinge moment (N m) the servo must hold
inline StockOut fin_stock(double F, double arm, double hinge, int mat) {
    const ShaftMat& m = SHAFT_MATS[std::max(0, std::min(3, mat))]; StockOut o;
    double M = F * arm, SF = 2.0, T = std::fabs(hinge);
    o.d = std::max(0.004, std::cbrt(32.0 * SF / (M_PI * m.yield) * std::sqrt(M * M + 0.75 * T * T)));
    o.sigma = 32.0 / (M_PI * o.d * o.d * o.d) * std::sqrt(M * M + 0.75 * T * T);
    o.servo_Nm = 1.5 * T + 0.05;                                      // margin for friction and seals
    static const double cls[6] = {0.5, 1, 2, 4, 8, 16}, kg[6] = {0.06, 0.10, 0.20, 0.40, 0.80, 1.60};   // waterproof servo classes (assumed catalogue)
    int i = 0; while (i < 5 && cls[i] < o.servo_Nm) ++i;
    o.servo_Nm = cls[i]; o.servo_kg = kg[i]; return o;
}

}  // namespace pdz
