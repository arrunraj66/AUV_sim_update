// auv_design.hpp - the AUV DESIGN ENGINE.
// You choose: length, diameter, nose and tail shape, material, depth rating, battery, payload, how many fins (and their shape),
// and which propellers / thrusters to add. The engine works out everything else with the Step 5A physics library:
//   hull volume and wetted area, pressure-hull wall thickness, a full MASS BUDGET (every part), ballast to make it float right,
//   centre of gravity and buoyancy (and a stability check), inertia tensor, added mass, drag at any speed, propeller thrust
//   (blade element theory), power, top speed, endurance, fin lift and control authority, thrust per axis, warnings.
// Body frame: x forward, y starboard, z UP; the origin is the centre of buoyancy.
#pragma once
#include "physics/rigid_body.hpp"
#include "physics/hull.hpp"
#include "physics/hydrostatics.hpp"
#include "physics/added_mass.hpp"
#include "physics/drag.hpp"
#include "physics/fin.hpp"
#include "physics/propeller.hpp"
#include "physics/seawater.hpp"
#include "prop_design.hpp"
#include "design_parts.hpp"
#include <utility>
#include <string>
#include <vector>
#include <cstdio>

namespace design {

struct Material { const char* name; double rho, E, sigma; };               // density kg/m3, Young's modulus Pa, allowable stress Pa
static const Material MATERIALS[4] = {
    {"Aluminium 6061", 2700, 69e9, 240e6}, {"Carbon fibre", 1600, 70e9, 450e6}, {"PVC / ABS", 1400, 3e9, 40e6}, {"Titanium Ti-6Al-4V", 4430, 114e9, 800e6}};

struct FinSet {                    // a ring of identical fins around the hull
    bool on = true;
    int count = 4;                 // 1..8, spaced evenly
    double angle0 = 0;             // angle of the first fin from the top, degrees (0 = "+" layout for 4 fins, 45 = "X")
    double x = -0.55;              // quarter-chord position along the hull, measured from the centre of the hull (+ = forward)
    double cr = 0.12, ct = 0.07, span = 0.09, sweep = 0.03, tc = 0.12;     // root chord, tip chord, span (m), sweep (m), thickness / chord
    bool control = true;           // true: moves (rudder / elevator / both); false: fixed stabiliser
    double max_defl = 25;          // degrees
    int stock_mat = 0;             // material of the fin stock (hinge shaft)
};

struct PropSpec {                  // a propeller or thruster
    int role = 0;                  // 0 main (axial, at the tail), 1 vertical thruster, 2 lateral thruster
    double D = 0.19;               // propeller diameter, m
    double pd = 1.2;               // pitch / diameter
    int blades = 3; bool ducted = true;
    int model = 1;                 // 0 = blade-element model of Step 5A, 1 = Wageningen B-series (published open-water data)
    double ear = 0.55;             // blade area ratio AE/A0 (0.3 .. 1.05), B-series
    bool contra = false;           // main propeller only: dual contra-rotating propellers on a coaxial shaft
    int blades2 = 4; double d2_ratio = 0.94;     // rear propeller of a contra-rotating pair: blades and diameter / front diameter
    int shaft_mat = 0; bool keyway = true; double bore = 0;   // shaft material (see pdz::SHAFT_MATS), key way, hollow shaft ratio di/do
    double rpm = 1500;             // maximum rpm (ignored for the main propeller when auto_rpm is on)
    bool auto_rpm = true;          // main propeller: choose the rpm that reaches the target top speed
    double x = 0, y = 0, z = 0;    // position for roles 1 and 2 (the main propeller sits behind the tail)
};

struct DesignIn {
    std::string name = "My AUV";
    // hull
    double L = 1.60, D = 0.19;                 // overall length and diameter, m
    double nose_frac = 0.14, tail_frac = 0.32; // length of the nose and of the tail cone as fractions of L
    int nose_kind = 0;                         // 0 ellipsoid, 1 bullet (blunter, more room for sensors)
    double tail_end = 0.35;                    // diameter at the very tail / D
    int material = 0; double rated_depth = 100;
    double trim = 0.02;                        // wanted positive buoyancy as a fraction of the weight
    // equipment
    double battery_Wh = 1000, pack_Wh_per_kg = 150;
    double payload_kg = 3.0, electronics_kg = 2.5, hotel_W = 20;
    bool mast = true;                          // antenna mast (drag)
    // fins and propellers
    std::vector<FinSet> fins;
    std::vector<PropSpec> props;
    // mission
    double v_cruise = 1.5, v_max = 2.5;        // m/s
    double temp = 15, salinity = 35;
    int water = 0;                             // 0 sea water, 1 brackish, 2 fresh water, 3 cold fresh water, 4 custom (temp + salinity above)
    std::vector<int> sensors;                  // chosen sensors (design::SE_CAMERA ...): each adds mass, power, drag and sits in its place
    double bus_V = 24, cell_Ah = 3.0, cell_maxC = 2.0;     // battery bus voltage, one cell's capacity (Ah) and its maximum discharge rate (C)
};

// a ready-made starting point: close to a REMUS 100
inline DesignIn default_design() {
    DesignIn d; d.fins.push_back(FinSet{}); d.props.push_back(PropSpec{}); return d;
}

struct MassItem { std::string name; double m; double x, z; };
struct FinOutRow { double area, ar, cla_deg, stall_deg, ang, x, rud, elev; double stock_d = 0, stock_sigma = 0, servo_Nm = 0, servo_kg = 0, F_max = 0, hinge = 0; };
struct PropRow { int role = 0; int Z = 3, Z2 = 0; double D = 0, PD = 0, EAR = 0, rpm = 0, rpm2 = 0, ear_min = 0, tip = 0, eta_cruise = 0, Re = 0, Qnet = 0;
                 double shaft_d = 0, shaft_iacs = 0, shaft_sigma = 0, n_crit = 0, crit_margin = 0, shaft_kg = 0; bool contra = false;
                 double Tb = 0, Pb_elec = 0; };   // thrusters (roles 1, 2): bollard thrust (N) and the electrical power it takes (W)
struct DesignOut {
    // hull
    double V = 0, wetted = 0, front = 0, wall_t = 0, Lcyl = 0, fineness = 0, cx = 0;
    phys::Hull hull;
    // mass
    std::vector<MassItem> items; double mass = 0, ballast = 0, W = 0, B = 0, net_up = 0;
    V3 rg; double I[3] = {0, 0, 0}; double BG = 0; double T_roll = 0, T_pitch = 0;
    double AM[6] = {0, 0, 0, 0, 0, 0};
    // resistance and propulsion
    double drag_cruise = 0, drag_max = 0, Xuu = 0;
    double rpm_main = 0, thrust_bollard = 0, thrust_cruise = 0, P_shaft_cruise = 0, P_elec_cruise = 0, P_elec_max = 0, P_rated_motor = 0;
    double rpm_cruise = 0, thrust_vmax = 0;                     // main propeller: rpm at cruise speed, thrust at the top speed and full rpm
    double v_top = 0, endurance_h = 0, range_km = 0, endurance_max_h = 0;
    double cap_surge = 0, cap_heave = 0, cap_sway = 0;       // largest thruster force per axis, N
    // fins
    std::vector<FinOutRow> fin_rows; double fin_area_total = 0;
    std::vector<PropRow> prop_rows;
    double yaw_rate_max = 0, turn_radius = 0, pitch_max_deg = 0, stability_ratio = 0;
    std::vector<std::string> warnings;
    bool ok = true;
    // sensors, battery and motor
    double hotel_total = 0, sensors_kg = 0, sensors_W = 0, rho_water = 1025;
    PackOut pack; double net_up_sea = 0, net_up_fresh = 0, net_up_cold = 0;
    // what to draw
    struct FinDraw { double x_le, cr, ct, span, sweep, ang; bool control; };
    struct PropDraw { double x, D; int role; bool contra, ducted; };
    std::vector<std::pair<double, double>> outline; std::vector<FinDraw> fin_draw; std::vector<PropDraw> prop_draw; double x_nose = 0, x_tail = 0;
};

inline std::string num(double v, const char* f = "%.1f") { char b[48]; std::snprintf(b, sizeof b, f, v); return b; }
inline double clampd(double v, double a, double b) { return std::max(a, std::min(b, v)); }

// radius of the outline: x from the tail (-L/2) to the nose (+L/2)
struct Outline {
    double L, R, ln, lt, rt; int kind;
    double r(double x) const {
        double xn0 = 0.5 * L - ln, xt0 = -0.5 * L + lt;
        if (x > xn0) { double t = std::min(1.0, (x - xn0) / ln); return kind == 0 ? R * std::sqrt(std::max(0.0, 1 - t * t)) : R * std::pow(std::max(0.0, 1 - std::pow(t, 2.6)), 0.385); }
        if (x < xt0) { double t = std::min(1.0, (xt0 - x) / lt); return R * (1.0 - (1.0 - rt) * std::pow(t, 1.7)); }
        return R;
    }
};

struct PropPerf { double T = 0, Q = 0, rpm = 0; };

class Designer {
public:
    phys::WaterState ws;
    // thrust and shaft torque of a propeller at rpm and advance speed
    static void prop_at(const phys::Propeller& p, double rpm, double Va, const phys::WaterState& w, double& T, double& Q) {
        phys::Propeller::Out o = p.eval(rpm / 60.0, Va, w.rho, 5.0, w.p + 0 * w.rho * phys::G * 10.0, w.pv); T = o.T; Q = o.Q;
    }

    DesignOut run(const DesignIn& in) {
        using namespace phys;
        DesignOut o;
        if (in.water >= 0 && in.water < 4) ws = WaterState::at(WATERS[in.water].T, WATERS[in.water].S, 10.0); else ws = WaterState::at(in.temp, in.salinity, 10.0);
        const double rho = ws.rho, g = G;
        Outline ol{in.L, 0.5 * in.D, clampd(in.nose_frac, 0.05, 0.4) * in.L, clampd(in.tail_frac, 0.1, 0.6) * in.L, clampd(in.tail_end, 0.1, 1.0), in.nose_kind};
        // hull, then move the origin to the centre of buoyancy
        Hull h0; h0.from_profile([&](double x) { return ol.r(x); }, -0.5 * in.L, 0.5 * in.L, 1, 1, 100);
        o.cx = h0.centroid.x;
        const double cx = o.cx;
        o.hull.from_profile([&](double x) { return ol.r(x + cx); }, -0.5 * in.L - cx, 0.5 * in.L - cx, 1, 1, 100);
        o.V = o.hull.volume; o.wetted = o.hull.wetted; o.front = o.hull.front; o.fineness = in.L / in.D;
        o.B = rho * g * o.V; o.rho_water = rho;
        for (int i = 0; i <= 120; ++i) { double xx = -0.5 * in.L + in.L * i / 120.0; o.outline.push_back({xx - cx, ol.r(xx)}); }
        o.x_tail = -0.5 * in.L - cx; o.x_nose = 0.5 * in.L - cx;
        const double xtail = -0.5 * in.L - cx, xnose = 0.5 * in.L - cx;       // ends of the hull in body coordinates
        const double xcyl0 = xtail + ol.lt, xcyl1 = xnose - ol.ln;
        o.Lcyl = std::max(0.0, xcyl1 - xcyl0);

        // pressure hull wall: buckling of a long cylinder under the rated pressure, yield, and a manufacturing minimum
        const Material& mat = MATERIALS[std::max(0, std::min(3, in.material))];
        double Pd = 1.5 * rho * g * in.rated_depth, nu_p = 0.33;
        double t_buck = in.D * std::cbrt(Pd * (1 - nu_p * nu_p) / (2 * mat.E)), t_yield = Pd * in.D / (2 * mat.sigma);
        o.wall_t = std::max({t_buck, t_yield, 0.0025});
        double A_shell = M_PI * in.D * o.wall_t;
        double m_shell = mat.rho * A_shell * o.Lcyl;
        double m_caps = 2 * mat.rho * M_PI * 0.25 * in.D * in.D * (1.2 * o.wall_t);           // two end bulkheads
        double A_fair = o.wetted - M_PI * in.D * o.Lcyl;                                      // nose and tail fairings (fibreglass 2 mm + foam)
        double m_fair = std::max(0.0, A_fair) * 3.6;

        std::vector<MassItem>& it = o.items;
        auto add = [&](const char* n, double m, double x, double z) { MassItem a{n, m, x, z}; it.push_back(a); };
        double xmid = 0.5 * (xcyl0 + xcyl1);
        add("pressure hull wall", m_shell, xmid, 0);
        add("end bulkheads", m_caps, xmid, 0);
        add("nose + tail fairings", m_fair, 0.5 * (xtail + xnose) - 0.05 * in.L, 0);
        double m_bat = in.battery_Wh / std::max(in.pack_Wh_per_kg, 50.0);
        add("battery pack", m_bat, xmid, -0.30 * in.D);                                   // x is solved below
        add("computer + electronics", in.electronics_kg, xcyl0 + 0.25 * o.Lcyl, 0.05 * in.D);
        add("payload (extra)", in.payload_kg, xcyl1 - 0.2 * o.Lcyl, -0.05 * in.D);
        // sensors: each one has its own mass, power, drag and place
        double sens_A = 0;
        { const SensorPart* sp = sensor_parts();
          for (int id : in.sensors) {
              if (id < 0 || id >= SE_COUNT) continue;
              const SensorPart& sn = sp[id]; double sx = xmid, sz = 0;
              switch (sn.place) {
                  case PL_NOSE: sx = xnose - 0.45 * ol.ln; sz = 0; break;
                  case PL_BODY: sx = xmid; sz = 0.02; break;
                  case PL_BELLY: sx = xmid + 0.15 * o.Lcyl; sz = -0.5 * in.D + 0.03; break;
                  case PL_MAST: sx = xmid - 0.10; sz = 0.5 * in.D + 0.05; break;
                  case PL_SIDES: sx = xmid; sz = 0; break;
              }
              add((std::string("sensor: ") + sn.name).c_str(), sn.mass, sx, sz);
              o.sensors_kg += sn.mass; o.sensors_W += sn.power; sens_A += sn.drag_area;
          } }
        o.hotel_total = in.hotel_W + o.sensors_W;

        // ---- propellers (BEMT) ---------------------------------------------------------------------
        double Cd0_fins = 0, S_fins = 0;
        std::vector<FinGeom> fg; std::vector<double> fth;
        std::vector<PlateAM> plates; int fin_id = 0;
        for (const FinSet& fs : in.fins) {
            if (!fs.on || fs.count < 1) continue;
            for (int k = 0; k < fs.count; ++k) {
                double th = (fs.angle0 + 360.0 * k / fs.count) * M_PI / 180.0;
                FinGeom f; double xf = fs.x;                                              // measured from the hull centre -> body frame
                f.root = V3{xf - cx, 0.5 * in.D * std::sin(th), 0.5 * in.D * std::cos(th)};
                f.span_dir = V3{0, std::sin(th), std::cos(th)}; f.normal = V3{0, std::cos(th), -std::sin(th)};
                f.cr = fs.cr; f.ct = fs.ct; f.span = fs.span; f.sweep = fs.sweep; f.tc = fs.tc; f.ar_gain = 1.6;
                f.setup();
                fg.push_back(f); fth.push_back(th);
                // loads at top speed and full deflection (fixed fins: 8 degrees of sideslip) -> stock diameter and servo size
                double dl = (fs.control ? fs.max_defl : 8.0) * M_PI / 180.0;
                FinOut fo = fin_force(f, V3{in.v_max, 0, 0}, V3{}, V3{}, dl, rho, ws.nu, 1.0);
                double Fn = len(fo.F), ymac_arm = f.y_mac;
                pdz::StockOut so = pdz::fin_stock(Fn, ymac_arm, fo.hinge, fs.stock_mat);
                double m_stock = pdz::SHAFT_MATS[fs.stock_mat].rho * M_PI * 0.25 * so.d * so.d * (fs.span + 0.06);
                double mf = 1800 * f.area * fs.tc * 0.5 * (fs.cr + fs.ct) * 0.8 + m_stock + (fs.control ? so.servo_kg : 0.0);
                add("fin / rudder", mf, xf - cx, 0.5 * in.D * std::cos(th));
                S_fins += f.area; ++fin_id;
                plates.push_back({f.ac, f.normal, 0.5 * (fs.cr + fs.ct), fs.span});
                FinOutRow r; r.area = f.area; r.ar = f.ar; r.cla_deg = f.foil.cla * M_PI / 180.0; r.stall_deg = f.foil.alpha_stall * 180 / M_PI; r.ang = th * 180 / M_PI; r.x = xf;
                r.rud = fs.control ? std::cos(th) : 0; r.elev = fs.control ? -std::sin(th) : 0;
                r.stock_d = so.d; r.stock_sigma = so.sigma; r.servo_Nm = fs.control ? so.servo_Nm : 0; r.servo_kg = fs.control ? so.servo_kg : 0; r.F_max = Fn; r.hinge = fo.hinge;
                o.fin_rows.push_back(r);
                { DesignOut::FinDraw fd; fd.x_le = xf - cx + 0.25 * fs.cr; fd.cr = fs.cr; fd.ct = fs.ct; fd.span = fs.span; fd.sweep = fs.sweep; fd.ang = th * 180 / M_PI; fd.control = fs.control; o.fin_draw.push_back(fd); }
            }
        }
        o.fin_area_total = S_fins;

        struct PU {
            PropSpec s; Propeller p; pdz::Rotor1 r1; pdz::CRP crp; bool dual = false; double rpm = 0; double wake = 0, ded = 0; V3 pos, dir; double rho = 1025, pv = 1700, pp = 101325; double nu = 1.19e-6;
            // thrust (N) and shaft power (W) at rpm and advance speed
            void at(double rp, double Va, double& T, double& P, double* Qnet = nullptr, double* rpm2 = nullptr) const {
                if (s.model == 0) { Propeller::Out o = p.eval(rp / 60.0, Va, rho, 5.0, pp, pv); T = o.T; P = o.Q * 2 * M_PI * rp / 60.0; return; }
                if (dual) { pdz::CRP::Out o = crp.eval(rp, Va, rho); T = o.T; P = o.P; if (Qnet) *Qnet = o.Qnet; if (rpm2) *rpm2 = o.rpm2; return; }
                double Q; r1.eval(rp, Va, rho, T, Q); P = Q * 2 * M_PI * rp / 60.0;
            }
        };
        std::vector<PU> pus;
        for (const PropSpec& ps : in.props) {
            PU u; u.s = ps; BladeGeom bg; bg.B = ps.blades; bg.D = ps.D; bg.pd = ps.pd; bg.ducted = ps.ducted; bg.ear = 0.55;
            u.p.blade = bg;
            u.p.duct_k = 0.40; u.p.k_cal = 0.90;      // calibration: static thrust of a Kort nozzle propeller, figure of merit about 0.7
            u.p.build(25.0, rho, ws.nu);
            u.rho = rho; u.pv = ws.pv; u.pp = ws.p; u.nu = ws.nu;
            u.r1.Z = ps.blades; u.r1.D = ps.D; u.r1.PD = ps.pd; u.r1.EAR = ps.ear; u.r1.duct = ps.ducted;
            if (ps.role == 0 && ps.contra && ps.model == 1) {
                u.dual = true; u.crp.a = u.r1; u.crp.b = u.r1; u.crp.b.Z = ps.blades2; u.crp.b.D = ps.D * ps.d2_ratio; u.crp.b.duct = false; u.crp.a.duct = false;
            }
            if (ps.role == 0) { u.pos = V3{xtail - 0.02 - (ps.ducted ? 0.03 : 0.0), 0, 0}; u.dir = V3{1, 0, 0}; u.wake = 0.10; u.ded = 0.08; }
            else if (ps.role == 1) { u.pos = V3{ps.x - cx, ps.y, ps.z}; u.dir = V3{0, 0, 1}; }
            else { u.pos = V3{ps.x - cx, ps.y, ps.z}; u.dir = V3{0, 1, 0}; }
            pus.push_back(u);
        }

        // drag at a speed: hull + fins + mast + tunnel openings
        auto drag_at = [&](double U) {
            Drag dr; dr.cd_base = 0.0; dr.form = 0;
            double Dh = dr.eval(o.hull, V3{U, 0, 0}, V3{}, rho, ws.nu, 8.0, g).F.x; double Fh = -Dh;
            double q = 0.5 * rho * U * U, Ff = 0;
            for (const FinGeom& f : fg) { FoilCoef c = f.foil.coef(0.0, std::max(U, 0.05) * f.mac / ws.nu); Ff += q * f.area * c.CD; }
            double Fm = in.mast ? 0.5 * rho * 1.0 * 0.0006 * U * U : 0.0;
            double Ft = 0; for (const PU& u : pus) if (u.s.role != 0) Ft += q * 0.12 * M_PI * 0.25 * u.s.D * u.s.D;
            double Fs = 0.5 * rho * 0.8 * sens_A * U * U;
            return Fh + Ff + Fm + Ft + Fs;
        };
        o.drag_cruise = drag_at(in.v_cruise); o.drag_max = drag_at(in.v_max);
        o.Xuu = o.drag_cruise / (in.v_cruise * in.v_cruise);

        // main propeller: size rpm for the top speed (with a 15 % margin), then solve cruise
        PU* main = nullptr; for (PU& u : pus) if (u.s.role == 0) { main = &u; break; }
        double main_T_vmax = 0, main_rc = 0, thr_P = 0;
        double eta_chain = 0.80 * 0.97 * 0.95;                                      // motor x ESC x gearbox
        if (main) {
            auto thrust_net = [&](double rpm, double U, double* Pw = nullptr) { double T, Pq; main->at(rpm, U * (1 - main->wake), T, Pq); if (Pw) *Pw = Pq; return T * (1 - main->ded); };
            double rpm = main->s.rpm;
            if (main->s.auto_rpm) {
                double want = 1.15 * o.drag_max; double lo = 100, hi = 9000;
                for (int i = 0; i < 40; ++i) { double mid = 0.5 * (lo + hi); (thrust_net(mid, in.v_max) < want ? lo : hi) = mid; }
                rpm = 0.5 * (lo + hi);
            }
            main->rpm = rpm; o.rpm_main = rpm;
            double Pb; o.thrust_bollard = thrust_net(rpm, 0, &Pb);
            double Pm; double T_vmax = thrust_net(rpm, in.v_max, &Pm);
            double lo = 50, hi = rpm;                                                  // cruise rpm
            for (int i = 0; i < 40; ++i) { double mid = 0.5 * (lo + hi); (thrust_net(mid, in.v_cruise) < o.drag_cruise ? lo : hi) = mid; }
            double rc = 0.5 * (lo + hi), Pc_sh; o.thrust_cruise = thrust_net(rc, in.v_cruise, &Pc_sh);
            o.P_shaft_cruise = Pc_sh; o.P_elec_cruise = o.P_shaft_cruise / eta_chain;
            o.P_elec_max = Pm / eta_chain;
            // top speed: thrust = drag at maximum rpm
            double ul = 0, uh = 8;
            for (int i = 0; i < 40; ++i) { double um = 0.5 * (ul + uh); (thrust_net(rpm, um) > drag_at(um) ? ul : uh) = um; }
            o.v_top = 0.5 * (ul + uh);
            o.cap_surge += o.thrust_bollard;
            main_T_vmax = T_vmax; main_rc = rc; o.rpm_cruise = rc; o.thrust_vmax = T_vmax;
        }
        for (PU& u : pus) {
            double Pb, Tb; u.at(u.s.rpm, 0, Tb, Pb);
            if (u.s.role != 0) thr_P += 1.1 * Pb / eta_chain;
            if (u.s.role != 0) { u.rpm = u.s.rpm; (u.s.role == 1 ? o.cap_heave : o.cap_sway) += Tb; }
            double P_rated = (u.s.role == 0 ? o.P_elec_max * eta_chain : Pb) * 1.1;
            double rpm_use = u.s.role == 0 ? u.rpm : u.s.rpm; (void)rpm_use;
            if (u.s.role == 0) o.P_rated_motor = P_rated;
            double m_motor = 0.25 + P_rated / 220.0, m_prop = 0.12 * std::pow(u.s.D / 0.1, 3) * u.s.blades / 3.0 + (u.s.ducted ? 0.10 * std::pow(u.s.D / 0.1, 2) : 0.0);
            if (u.dual) m_prop += 0.12 * std::pow(u.crp.b.D / 0.1, 3) * u.crp.b.Z / 3.0;
            add(u.s.role == 0 ? "main motor + gearbox" : "thruster motor", m_motor, u.pos.x + 0.06, 0);
            if (u.dual) add("coaxial contra-rotating gearbox", 0.35 + 0.004 * P_rated, u.pos.x + 0.10, 0);
            // propeller report row and shaft design
            {
                PropRow r; r.role = u.s.role; r.Z = u.s.blades; r.Z2 = u.dual ? u.s.blades2 : 0; r.D = u.s.D; r.PD = u.s.pd; r.EAR = u.s.ear; r.contra = u.dual; if (u.s.role != 0) { r.Tb = Tb; r.Pb_elec = Pb / eta_chain; }
                double rp = u.s.role == 0 ? u.rpm : u.s.rpm; r.rpm = rp;
                double Tref = u.s.role == 0 ? main_T_vmax : Tb;
                double rho_p = ws.rho; double p0 = P_ATM + rho_p * g * 3.0;
                r.ear_min = pdz::keller_ear(Tref, u.s.blades, u.s.D, p0, ws.pv);
                r.tip = pdz::tip_speed(u.s.D, rp);
                double Va_c = in.v_cruise * (1 - u.wake);
                r.Re = 1.96 * u.s.ear * u.s.D / u.s.blades * std::sqrt(Va_c * Va_c + std::pow(0.7 * M_PI * rp / 60.0 * u.s.D, 2)) / ws.nu;
                if (u.s.role == 0) {
                    double T, Pq, Qn = 0, r2 = 0; u.at(main_rc, Va_c, T, Pq, &Qn, &r2); r.eta_cruise = Pq > 1e-9 ? T * (1 - u.ded) * in.v_cruise / Pq : 0; r.Qnet = Qn; r.rpm2 = u.dual ? r2 : 0;
                } else { r.eta_cruise = 0; }
                double P_sh = u.s.role == 0 ? o.P_elec_max * eta_chain : Pb;
                double a_ov = 0.35 * u.s.D + 0.05;
                pdz::ShaftOut sh = pdz::shaft_design(P_sh, rp, Tref, m_prop, a_ov, 0.45, u.s.shaft_mat, u.s.keyway, u.s.bore, rp);
                r.shaft_d = sh.d; r.shaft_iacs = sh.d_iacs; r.shaft_sigma = sh.sigma_eq; r.n_crit = sh.n_crit; r.crit_margin = sh.margin; r.shaft_kg = sh.mass;
                add("propeller shaft + bearings + seal", sh.mass + 0.12 + 0.6 * sh.d * 10.0 * 0.1, u.pos.x + 0.20, 0);
                o.prop_rows.push_back(r);
                { DesignOut::PropDraw pd; pd.x = u.pos.x; pd.D = u.s.D; pd.role = u.s.role; pd.contra = u.dual; pd.ducted = u.s.ducted; o.prop_draw.push_back(pd); }
            }
            add(u.s.role == 0 ? "main propeller + duct" : "thruster propeller + duct", m_prop, u.pos.x, 0);
            if (u.s.role != 0) add("thruster tunnel", 0.08 + 0.9 * u.s.D, u.pos.x, u.pos.z);
        }
        if (in.mast) add("antenna mast + GPS", 0.25, xmid - 0.1, 0.5 * in.D + 0.05);
        double sub = 0; for (const MassItem& a : it) sub += a.m;
        add("cables, fittings, fasteners (6 %)", 0.06 * sub, xmid, 0);
        sub *= 1.06;

        // ---- ballast and balance --------------------------------------------------------------------
        double m_target = o.B / g / (1.0 + in.trim);
        o.ballast = m_target - sub;
        if (o.ballast < 0) { o.warnings.push_back("TOO HEAVY: " + num(-o.ballast) + " kg more than it can float. Make the hull longer or wider, use a lighter material, or reduce the payload / battery."); o.ballast = 0; o.ok = false; }
        add("lead ballast", o.ballast, xnose - 0.35 * in.L, -0.40 * in.D);
        // solve the battery's fore/aft position so the centre of gravity sits under the centre of buoyancy
        int ib = 3;
        double mt = 0, mx = 0; for (size_t i = 0; i < it.size(); ++i) if ((int)i != ib) { mt += it[i].m; mx += it[i].m * it[i].x; }
        double xb = -mx / std::max(it[ib].m, 1e-6);
        double xlo = xcyl0 + 0.5 * m_bat / 10, xhi = xcyl1;
        double need = it[ib].m > 0 ? 1.0 : 0.0; (void)need;
        if (xb < xcyl0 || xb > xcyl1) {                    // the battery alone can not balance it: put it at its limit and slide the lead ballast to finish the job
            xb = clampd(xb, xcyl0, xcyl1); it[ib].x = xb;
            size_t ibl = it.size() - 1; double mx2 = 0; for (size_t i = 0; i < ibl; ++i) mx2 += it[i].m * it[i].x;
            double xball = it[ibl].m > 1e-6 ? -mx2 / it[ibl].m : 0.0, xlim0 = xtail + 0.06 * in.L, xlim1 = xnose - 0.06 * in.L;
            it[ibl].x = clampd(xball, xlim0, xlim1);
            if (xball < xlim0 || xball > xlim1) o.warnings.push_back("It can not be balanced fore and aft: the nose or the tail is too heavy even with the battery and the ballast at their limits. Move the payload, sensors or fins, or add weight at the other end.");
        }
        (void)xlo; (void)xhi;
        it[ib].x = xb;
        MassProps mp;
        for (const MassItem& a : it) {
            if (a.name == "pressure hull wall") mp.add_cylinder_x(a.name, a.m, V3{a.x, 0, a.z}, 0.5 * in.D, o.Lcyl, true);
            else if (a.name == "battery pack") mp.add_cylinder_x(a.name, a.m, V3{a.x, 0, a.z}, 0.30 * in.D, std::min(o.Lcyl, m_bat / (2600.0 * M_PI * 0.09 * in.D * in.D) + 0.1), false);
            else mp.add_point(a.name, a.m, V3{a.x, 0, a.z});
            if (a.name == "pressure hull wall") { /* shell */ }
        }
        mp.update();
        o.mass = mp.m; o.W = o.mass * g; o.net_up = o.B - o.W; o.rg = mp.rg;
        o.I[0] = mp.Ig.a[0][0]; o.I[1] = mp.Ig.a[1][1]; o.I[2] = mp.Ig.a[2][2];
        o.BG = -mp.rg.z;
        if (o.BG < 0.01) o.warnings.push_back("Centre of gravity is almost at the centre of buoyancy: it will roll over easily. Put heavy things lower (the battery and ballast go low by default).");
        AddedMass am; am.compute(o.hull, plates, rho);
        for (int i = 0; i < 6; ++i) o.AM[i] = am.MA[i][i];
        double WBG = std::max(o.W * o.BG, 1e-6);
        o.T_roll = 2 * M_PI * std::sqrt((o.I[0] + o.AM[3]) / WBG); o.T_pitch = 2 * M_PI * std::sqrt((o.I[1] + o.AM[4]) / WBG);

        // ---- endurance ------------------------------------------------------------------------------
        { double Qs = o.P_elec_max * eta_chain / std::max(2 * M_PI * o.rpm_main / 60.0, 1e-6);
          o.pack = pack_design(in.battery_Wh, in.bus_V, in.cell_Ah, in.cell_maxC, o.hotel_total + o.P_elec_cruise, o.hotel_total + o.P_elec_max + thr_P, Qs, o.rpm_main); }
        double Pc = o.hotel_total + o.P_elec_cruise; o.endurance_h = in.battery_Wh * 0.9 / std::max(Pc, 1e-3); o.range_km = o.endurance_h * in.v_cruise * 3.6;
        o.endurance_max_h = in.battery_Wh * 0.9 / std::max(o.hotel_total + o.P_elec_max, 1e-3);

        // ---- fins: control authority and stability ----------------------------------------------------
        double q = 0.5 * rho * in.v_cruise * in.v_cruise, N_rud = 0, M_elv = 0, Nfin_beta = 0;
        size_t k = 0;
        for (const FinSet& fs : in.fins) {
            if (!fs.on) continue;
            for (int j = 0; j < fs.count; ++j, ++k) {
                if (k >= fg.size()) break;
                const FinGeom& f = fg[k]; double th = fth[k];
                double arm = f.ac.x - o.rg.x;
                double dmax = fs.max_defl * M_PI / 180.0;
                double Fn = q * f.area * f.foil.coef(dmax, in.v_cruise * f.mac / ws.nu).CL;          // normal force at full deflection
                double slope = q * f.area * f.foil.cla;                                               // N per rad of angle of attack
                double ry = std::cos(th), rz = -std::sin(th);                                         // component of the normal along y and z
                Nfin_beta += slope * ry * ry * (-arm);                                                // sideslip restoring (fins aft: arm < 0)
                if (fs.control) { N_rud += std::fabs(Fn * ry) * std::fabs(arm); M_elv += std::fabs(Fn * rz) * std::fabs(arm); }
            }
        }
        // steady turn rate: rudder moment balances the yaw damping of the hull (cross-flow slices)
        {
            Drag dr; double lo = 0, hi = 3.0;
            for (int i = 0; i < 40; ++i) { double r = 0.5 * (lo + hi); V3 vr{in.v_cruise, 0, 0}, wr{0, 0, r}; Drag::Out d = dr.eval(o.hull, vr, wr, rho, ws.nu, 8.0, g); (std::fabs(d.M.z) < N_rud ? lo : hi) = r; }
            o.yaw_rate_max = 0.5 * (lo + hi);
            o.turn_radius = o.yaw_rate_max > 1e-3 ? in.v_cruise / o.yaw_rate_max : 1e9;
        }
        o.pitch_max_deg = std::asin(std::min(1.0, M_elv / std::max(WBG, 1e-6))) * 180 / M_PI;
        double munk = (o.AM[1] - o.AM[0]) * in.v_cruise * in.v_cruise;
        o.stability_ratio = Nfin_beta / std::max(munk, 1e-6);

        // ---- warnings -------------------------------------------------------------------------------
        if (o.fineness < 4.5) o.warnings.push_back("Short and fat (L/D < 4.5): high drag. A fineness ratio of 6 to 9 is best.");
        if (o.fineness > 12) o.warnings.push_back("Very slender (L/D > 12): it will be hard to turn and to fit the hardware.");
        if (in.fins.empty() || S_fins < 1e-6) o.warnings.push_back("No fins: the vehicle has no way to steer except by thrusters.");
        else if (N_rud < 1e-3) o.warnings.push_back("No moving fins: you can not steer in yaw with a fin. Set at least one fin set to 'moves'.");
        else if (o.stability_ratio < 1.0) o.warnings.push_back("The fins are too small for natural directional stability (ratio " + num(o.stability_ratio, "%.2f") + " < 1): it will need an autopilot, like the real REMUS. Larger or more aft fins fix it.");
        if (!main) o.warnings.push_back("No main propeller: it can only move with thrusters.");
        else {
            if (main->s.D > 1.08 * in.D) o.warnings.push_back("The propeller is wider than the hull: drag and risk of damage.");
            double tip = M_PI * main->s.D * main->rpm / 60.0; if (tip > 28) o.warnings.push_back("Propeller tip speed " + num(tip, "%.0f") + " m/s: noisy and prone to cavitation. Use more diameter and fewer rpm.");
            if (o.v_top < in.v_max * 0.98) o.warnings.push_back("It can not reach the wanted top speed (" + num(o.v_top, "%.2f") + " m/s): bigger propeller or higher rpm.");
        }
        for (const PropRow& r : o.prop_rows) {
            const char* who = r.role == 0 ? "Main propeller" : (r.role == 1 ? "Vertical thruster" : "Lateral thruster");
            if (r.EAR < r.ear_min) o.warnings.push_back(std::string(who) + ": blade area ratio " + num(r.EAR, "%.2f") + " is below the Keller cavitation minimum " + num(r.ear_min, "%.2f") + ". Use more blades, a bigger diameter or a larger AE/A0.");
            if (!r.crit_margin || r.crit_margin < 1.4) o.warnings.push_back(std::string(who) + " shaft: first critical speed " + num(r.n_crit, "%.0f") + " rpm is too close to the running speed. Use a thicker shaft or a shorter overhang.");
        }
        double Vint = M_PI * 0.25 * in.D * in.D * o.Lcyl * 0.8, Vbat = in.battery_Wh / 300.0 / 1000.0 + (in.payload_kg + o.sensors_kg) / 1500.0 + in.electronics_kg / 1200.0;
        if (Vbat > 0.55 * Vint) o.warnings.push_back("Not enough room inside: the battery, payload and electronics need " + num(100 * Vbat / Vint, "%.0f") + " % of the pressure hull volume. Make it longer or reduce the battery.");
        if (o.endurance_h < 2) o.warnings.push_back("Endurance under 2 hours at cruise speed.");
        if (std::fabs(o.net_up) > 0.1 * o.W) o.warnings.push_back("Buoyancy differs from the weight by more than 10 %.");
        // pack, motor and water
        if (main) {
            if (!o.pack.ok) o.warnings.push_back("Battery cannot give the peak current: " + num(o.pack.I_max, "%.0f") + " A at " + num(o.pack.C_max, "%.1f") + " C, the cells allow " + num(o.pack.C_limit, "%.1f") + " C. Use at least " + std::to_string(o.pack.Np_needed) + " cells in parallel (more capacity) or a higher voltage.");
            if (o.pack.gear > 25) o.warnings.push_back("The motor needs a " + num(o.pack.gear, "%.0f") + ":1 gearbox at this bus voltage: a higher battery voltage or a slower propeller makes it simpler.");
        }
        { double Vv = o.V, Wt = o.W; o.net_up_sea = water_density(WATERS[0]) * g * Vv - Wt; o.net_up_fresh = water_density(WATERS[2]) * g * Vv - Wt; o.net_up_cold = water_density(WATERS[3]) * g * Vv - Wt;
          if (in.water == 0 && o.net_up_fresh < 0) o.warnings.push_back("Balanced for sea water. In fresh water it is " + num(-o.net_up_fresh, "%.1f") + " N too heavy and will sink: take out " + num(-o.net_up_fresh / g, "%.2f") + " kg of ballast for lake work."); }
        return o;
    }
};

inline void print_report(const DesignIn& in, const DesignOut& o) {
    std::printf("\n=== %s : %.2f m x %.0f mm, %s ===\n", in.name.c_str(), in.L, in.D * 1000, MATERIALS[in.material].name);
    std::printf("hull: volume %.4f m3  wetted %.3f m2  L/D %.1f  wall %.1f mm  pressure hull %.2f m long\n", o.V, o.wetted, o.fineness, o.wall_t * 1000, o.Lcyl);
    std::printf("mass budget:\n"); for (const MassItem& a : o.items) std::printf("   %-34s %7.2f kg  x %+.2f\n", a.name.c_str(), a.m, a.x);
    std::printf("total %.2f kg  weight %.1f N  buoyancy %.1f N  net up %+.1f N (%.1f %%)\n", o.mass, o.W, o.B, o.net_up, 100 * o.net_up / o.W);
    std::printf("CG (%.3f, %.3f, %.3f) m from CB, BG %.3f m,  I = %.3f %.3f %.3f kg m2,  roll period %.2f s, pitch period %.2f s\n", o.rg.x, o.rg.y, o.rg.z, o.BG, o.I[0], o.I[1], o.I[2], o.T_roll, o.T_pitch);
    std::printf("added mass: %.2f %.2f %.2f kg  %.3f %.3f %.3f kg m2\n", o.AM[0], o.AM[1], o.AM[2], o.AM[3], o.AM[4], o.AM[5]);
    std::printf("sensors: %.2f kg, %.0f W; hotel load %.0f W.  battery: %dS%dP (%.1f V, %.1f Ah), peak %.0f A = %.1f C%s; motor Kv %.0f rpm/V, gearbox %.1f:1, ESC %.0f A\n", o.sensors_kg, o.sensors_W, o.hotel_total, o.pack.Ns, o.pack.Np, o.pack.V, o.pack.Ah, o.pack.I_max, o.pack.C_max, o.pack.ok ? "" : " (TOO HIGH)", o.pack.kv, o.pack.gear, o.pack.I_esc);
    std::printf("net up: sea %+.1f N, fresh %+.1f N, cold fresh %+.1f N\n", o.net_up_sea, o.net_up_fresh, o.net_up_cold);
    std::printf("drag at %.1f m/s: %.1f N, at %.1f m/s: %.1f N\n", in.v_cruise, o.drag_cruise, in.v_max, o.drag_max);
    std::printf("propulsion: main rpm %.0f, bollard %.1f N, cruise thrust %.1f N, shaft %.0f W, electric %.0f W, top speed %.2f m/s\n", o.rpm_main, o.thrust_bollard, o.thrust_cruise, o.P_shaft_cruise, o.P_elec_cruise, o.v_top);
    for (const PropRow& r : o.prop_rows) {
        std::printf("%s: %s%d%s blades D %.0f mm P/D %.2f AE/A0 %.2f  rpm %.0f%s  tip %.1f m/s  Keller min AE/A0 %.2f  Re %.0f  eff(cruise) %.2f\n", r.role == 0 ? "main prop" : (r.role == 1 ? "vertical thruster" : "lateral thruster"), r.contra ? "CRP " : "", r.Z, r.contra ? ("+" + std::to_string(r.Z2)).c_str() : "", r.D * 1000, r.PD, r.EAR, r.rpm, r.contra ? (" / " + num(r.rpm2, "%.0f")).c_str() : "", r.tip, r.ear_min, r.Re, r.eta_cruise);
        std::printf("   shaft: %.1f mm (IACS %.1f mm), stress %.0f MPa, critical speed %.0f rpm (x%.1f), shaft mass %.2f kg\n", r.shaft_d * 1000, r.shaft_iacs * 1000, r.shaft_sigma / 1e6, r.n_crit, r.crit_margin, r.shaft_kg);
    }
    for (const FinOutRow& f : o.fin_rows) std::printf("fin at %.0f deg: area %.4f m2 AR %.2f CLa %.3f/deg stall %.0f deg  max force %.1f N hinge %.2f Nm -> stock %.1f mm servo %.1f Nm (%.2f kg)\n", f.ang, f.area, f.ar, f.cla_deg, f.stall_deg, f.F_max, f.hinge, f.stock_d * 1000, f.servo_Nm, f.servo_kg);
    std::printf("endurance %.1f h at cruise (%.0f km), %.1f h flat out; thruster capacity: surge %.0f N, heave %.0f N, sway %.0f N\n", o.endurance_h, o.range_km, o.endurance_max_h, o.cap_surge, o.cap_heave, o.cap_sway);
    std::printf("fins: %d, total area %.4f m2; turn rate %.1f deg/s (radius %s m); max pitch %.0f deg; stability ratio %.2f\n", (int)o.fin_rows.size(), o.fin_area_total, o.yaw_rate_max * 180 / M_PI, o.turn_radius > 1e6 ? "n/a" : num(o.turn_radius).c_str(), o.pitch_max_deg, o.stability_ratio);
    for (const std::string& w : o.warnings) std::printf("  ! %s\n", w.c_str());
}

}  // namespace design
