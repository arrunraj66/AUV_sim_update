// remus100.hpp - the Step 7A REMUS 100 ("Torpedo AUV") with the published Prestero coefficients.
//
// Source: T. Prestero, "Verification of a Six-Degree of Freedom Simulation Model for the REMUS Autonomous Underwater Vehicle", MIT / WHOI Joint Program,
// M.S. thesis, September 2001, https://dspace.mit.edu/handle/1721.1/65068 (numbers only; page numbers are the printed ones; tables were read from the page images).
//
// KEPT from the earlier Step 7A preset in models.hpp (it already holds Prestero's numbers): mass 30.48 kg (Table 2.3, W = 299 N), centre of gravity 0.0196 m below the CB
// (Table 2.5), inertia 0.177 / 3.45 / 3.45 kg m2 (Table 2.6), added mass 0.93 / 35.5 / 35.5 kg and 0.0704 / 4.88 / 4.88 kg m2 (Table C.5), the propeller (12 N, a driving
// aid of the simulator), and the sea-bed contact points.
// REPLACED here (models.hpp is not edited; make_spec_ex() below routes M_TORPEDO to this):
//   axial drag     Xu|u|  = 1.62 kg/m                      Table C.1, p. 105
//   cross-flow     Yv|v|  = 1310 kg/m (sway)               Table C.2, p. 105 (his adjusted value, adjustment factor 10, Table 8.2)
//                  Zw|w|  = 131  kg/m (heave)              Table C.2, p. 105 (printed as 1.31e+002 although Table 8.2 lists a factor 10 for it: used as printed)
//                  Mq|q|  = 188  kg m2/rad2 (pitch rate)   Table C.2, p. 105
//                  Nr|r|  = 94   kg m2/rad2 (yaw rate)     Table C.2, p. 105
//   roll           Kp|p|  = 0.13 kg m2/rad2                Table C.3, p. 105
//   fins           S_fin = 6.65e-3 m2 each, span 8.57e-2 m, x_fin = -0.638 m from the CB, max angle 13.6 deg      Table 2.2, p. 17
//                  lift per fin pair 9.64 kg/(m rad)       Table C.10, p. 110 -> cl_alpha = 9.64 / (rho S_fin) = 1.414 per fin in this simulator
//   there are no linear damping terms in the thesis (all zero here).
// NOT published, estimated: the radial position of the fins (hull radius 0.095 m + half the span), the fin stall angle (0.30 rad), the fin servo speed (the simulator default, 60 deg/s).
// NOT in this simulator: the body lift forces (Yuv, Zuw, Muw, Nuv = +-28.6 kg/m and +-24 kg) and the added-mass cross terms other than the ones the equations of motion make themselves.
// The thesis is inconsistent in two places, noted rather than resolved: the text of Section 4.2.1 adopts the tow-tank drag coefficient Cd = 0.267 (Xu|u| = 3.9 kg/m) but Table C.1 prints 1.62 kg/m
// (Cd 0.11, the empirical estimate), which is also what his propeller thrust of 3.86 N at 1.51 m/s (Table C.9) matches; the table value is used. And Table C.10's 9.64 is 0.45 of rho CLa S_fin with
// CLa = 3.12 (Table 2.2); the table value is used.
#pragma once
#include "models.hpp"

static inline Spec make_remus100_step7a() {
    Spec s = make_spec(M_TORPEDO);
    const double rho = 1025.0, S_fin = 6.65e-3, span = 8.57e-2, x_fin = -0.638, hull_r = 0.095, dmax = 13.6 * 3.14159265358979 / 180.0;
    s.Dq[0] = 1.62;                         // Table C.1
    s.Dq[1] = 1310.0; s.Dq[2] = 131.0;      // Table C.2: Yvv, Zww
    s.Dq[4] = 188.0;  s.Dq[5] = 94.0;       // Table C.2: Mqq, Nrr
    s.Dq[3] = 0.13;                         // Table C.3: Kpp
    for (int i = 0; i < 6; ++i) s.Dl[i] = 0;
    double cla = 9.64 / (rho * S_fin);      // Table C.10: Yuudr = rho S_fin cla for a pair
    double r = hull_r + 0.5 * span;         // estimated: the fin hangs from the hull surface
    s.fins.clear();
    s.fins.push_back(mkfin({x_fin, r, 0}, {0, 0, 1}, S_fin, cla, dmax, 2, -1)); s.fins.push_back(mkfin({x_fin, -r, 0}, {0, 0, 1}, S_fin, cla, dmax, 2, -1));
    s.fins.push_back(mkfin({x_fin, 0, r}, {0, 1, 0}, S_fin, cla, dmax, 1, -1)); s.fins.push_back(mkfin({x_fin, 0, -r}, {0, 1, 0}, S_fin, cla, dmax, 1, -1));
    // The published fins give about a twelfth of the old preset's lift, so the simulator's attitude assist (tuned for the old fins) no longer had the authority to hold the hull's
    // nose-down (Munk) moment: the nose pitched down and hunted +-10 deg with W held. A gain of 5 on the assist restores level flight (pitch -0.4 deg, 0.3 m depth drift in 60 s at full
    // surge) and an accurate stick response. Estimated, not published.
    s.assist_gain = 5.0;
    s.note = "Cruises on one propeller and steers with four fins, so it only turns while it is moving. Up/Down = propeller, Left/Right = rudders, T/G = elevators. "
             "Drag, damping and fins are Prestero's published REMUS values (MIT/WHOI 2001).";
    return s;
}
// the simulator's vehicle specs: the Step 7A REMUS now carries the Prestero values, everything else is models.hpp as it was
static inline Spec make_spec_ex(int id) { return id == M_TORPEDO ? make_remus100_step7a() : make_spec(id); }
