#!/usr/bin/env python3
# patch_8b.py - Step 8B: adds propeller / shaft / fin design (src/prop_design.hpp) to src/auv_design.hpp. Run: python3 tools/patch_8b.py
import sys, os
os.chdir(os.path.expanduser(sys.argv[1] if len(sys.argv) > 1 else '~/Documents/auvsim_new'))
p = 'src/auv_design.hpp'; s = open(p).read()
if 'prop_design.hpp' in s: print('already patched'); sys.exit(0)
def rep(a, b):
    global s
    if a not in s: sys.exit('ANCHOR NOT FOUND (is src/auv_design.hpp the Step 8A version?):\n' + a[:90])
    s = s.replace(a, b, 1)
rep('#include "physics/seawater.hpp"\n', '#include "physics/seawater.hpp"\n#include "prop_design.hpp"\n')
rep('    int blades = 3; bool ducted = true;\n', '''    int blades = 3; bool ducted = true;
    int model = 1;                 // 0 = blade-element model of Step 5A, 1 = Wageningen B-series (published open-water data)
    double ear = 0.55;             // blade area ratio AE/A0 (0.3 .. 1.05), B-series
    bool contra = false;           // main propeller only: dual contra-rotating propellers on a coaxial shaft
    int blades2 = 4; double d2_ratio = 0.94;     // rear propeller of a contra-rotating pair: blades and diameter / front diameter
    int shaft_mat = 0; bool keyway = true; double bore = 0;   // shaft material (see pdz::SHAFT_MATS), key way, hollow shaft ratio di/do
''')
rep('    double max_defl = 25;          // degrees\n', '    double max_defl = 25;          // degrees\n    int stock_mat = 0;             // material of the fin stock (hinge shaft)\n')
rep('struct FinOutRow { double area, ar, cla_deg, stall_deg, ang, x, rud, elev; };', '''struct FinOutRow { double area, ar, cla_deg, stall_deg, ang, x, rud, elev; double stock_d = 0, stock_sigma = 0, servo_Nm = 0, servo_kg = 0, F_max = 0, hinge = 0; };
struct PropRow { int role = 0; int Z = 3, Z2 = 0; double D = 0, PD = 0, EAR = 0, rpm = 0, rpm2 = 0, ear_min = 0, tip = 0, eta_cruise = 0, Re = 0, Qnet = 0;
                 double shaft_d = 0, shaft_iacs = 0, shaft_sigma = 0, n_crit = 0, crit_margin = 0, shaft_kg = 0; bool contra = false; };''')
rep('    std::vector<FinOutRow> fin_rows; double fin_area_total = 0;', '    std::vector<FinOutRow> fin_rows; double fin_area_total = 0;\n    std::vector<PropRow> prop_rows;')
rep('''                double mf = 1800 * f.area * fs.tc * 0.5 * (fs.cr + fs.ct) * 0.8 + (fs.control ? (f.area > 0.02 ? 0.35 : 0.22) : 0.0);
                add("fin / rudder", mf, xf - cx, 0.5 * in.D * std::cos(th));''', '''                // loads at top speed and full deflection (fixed fins: 8 degrees of sideslip) -> stock diameter and servo size
                double dl = (fs.control ? fs.max_defl : 8.0) * M_PI / 180.0;
                FinOut fo = fin_force(f, V3{in.v_max, 0, 0}, V3{}, V3{}, dl, rho, ws.nu, 1.0);
                double Fn = len(fo.F), ymac_arm = f.y_mac;
                pdz::StockOut so = pdz::fin_stock(Fn, ymac_arm, fo.hinge, fs.stock_mat);
                double m_stock = pdz::SHAFT_MATS[fs.stock_mat].rho * M_PI * 0.25 * so.d * so.d * (fs.span + 0.06);
                double mf = 1800 * f.area * fs.tc * 0.5 * (fs.cr + fs.ct) * 0.8 + m_stock + (fs.control ? so.servo_kg : 0.0);
                add("fin / rudder", mf, xf - cx, 0.5 * in.D * std::cos(th));''')
rep('r.rud = fs.control ? std::cos(th) : 0; r.elev = fs.control ? -std::sin(th) : 0; o.fin_rows.push_back(r);', '''r.rud = fs.control ? std::cos(th) : 0; r.elev = fs.control ? -std::sin(th) : 0;
                r.stock_d = so.d; r.stock_sigma = so.sigma; r.servo_Nm = fs.control ? so.servo_Nm : 0; r.servo_kg = fs.control ? so.servo_kg : 0; r.F_max = Fn; r.hinge = fo.hinge;
                o.fin_rows.push_back(r);''')
rep('struct PU { PropSpec s; Propeller p; double T0 = 0, Q0 = 0; double rpm = 0; double wake = 0, ded = 0; V3 pos, dir; };', '''struct PU {
            PropSpec s; Propeller p; pdz::Rotor1 r1; pdz::CRP crp; bool dual = false; double rpm = 0; double wake = 0, ded = 0; V3 pos, dir; double rho = 1025, pv = 1700, pp = 101325; double nu = 1.19e-6;
            // thrust (N) and shaft power (W) at rpm and advance speed
            void at(double rp, double Va, double& T, double& P, double* Qnet = nullptr, double* rpm2 = nullptr) const {
                if (s.model == 0) { Propeller::Out o = p.eval(rp / 60.0, Va, rho, 5.0, pp, pv); T = o.T; P = o.Q * 2 * M_PI * rp / 60.0; return; }
                if (dual) { pdz::CRP::Out o = crp.eval(rp, Va, rho); T = o.T; P = o.P; if (Qnet) *Qnet = o.Qnet; if (rpm2) *rpm2 = o.rpm2; return; }
                double Q; r1.eval(rp, Va, rho, T, Q); P = Q * 2 * M_PI * rp / 60.0;
            }
        };''')
rep('            u.p.build(25.0, rho, ws.nu);\n', '''            u.p.build(25.0, rho, ws.nu);
            u.rho = rho; u.pv = ws.pv; u.pp = ws.p; u.nu = ws.nu;
            u.r1.Z = ps.blades; u.r1.D = ps.D; u.r1.PD = ps.pd; u.r1.EAR = ps.ear; u.r1.duct = ps.ducted;
            if (ps.role == 0 && ps.contra && ps.model == 1) {
                u.dual = true; u.crp.a = u.r1; u.crp.b = u.r1; u.crp.b.Z = ps.blades2; u.crp.b.D = ps.D * ps.d2_ratio; u.crp.b.duct = false; u.crp.a.duct = false;
            }
''')
rep('auto thrust_net = [&](double rpm, double U, double* Q = nullptr) { double T, Qq; prop_at(main->p, rpm, U * (1 - main->wake), ws, T, Qq); if (Q) *Q = Qq; return T * (1 - main->ded); };',
    'auto thrust_net = [&](double rpm, double U, double* Pw = nullptr) { double T, Pq; main->at(rpm, U * (1 - main->wake), T, Pq); if (Pw) *Pw = Pq; return T * (1 - main->ded); };')
rep('double Qb; o.thrust_bollard = thrust_net(rpm, 0, &Qb);', 'double Pb; o.thrust_bollard = thrust_net(rpm, 0, &Pb);')
rep('double Qm; thrust_net(rpm, in.v_max, &Qm);', 'double Pm; double T_vmax = thrust_net(rpm, in.v_max, &Pm);')
rep('double rc = 0.5 * (lo + hi), Qc; o.thrust_cruise = thrust_net(rc, in.v_cruise, &Qc);', 'double rc = 0.5 * (lo + hi), Pc_sh; o.thrust_cruise = thrust_net(rc, in.v_cruise, &Pc_sh);')
rep('o.P_shaft_cruise = Qc * 2 * M_PI * rc / 60.0; o.P_elec_cruise', 'o.P_shaft_cruise = Pc_sh; o.P_elec_cruise')
rep('o.P_elec_max = Qm * 2 * M_PI * rpm / 60.0 / eta_chain;', 'o.P_elec_max = Pm / eta_chain;')
rep('o.cap_surge += o.thrust_bollard;', '''o.cap_surge += o.thrust_bollard;
            main_T_vmax = T_vmax; main_rc = rc;''')
rep('        double eta_chain = 0.80 * 0.97 * 0.95;', '        double main_T_vmax = 0, main_rc = 0;\n        double eta_chain = 0.80 * 0.97 * 0.95;')
rep('''            double Qb, Tb; prop_at(u.p, u.s.rpm, 0, ws, Tb, Qb);
            if (u.s.role != 0) { u.rpm = u.s.rpm; (u.s.role == 1 ? o.cap_heave : o.cap_sway) += Tb; }
            double P_rated = (u.s.role == 0 ? o.P_elec_max * eta_chain : Qb * 2 * M_PI * u.s.rpm / 60.0) * 1.1;''', '''            double Pb, Tb; u.at(u.s.rpm, 0, Tb, Pb);
            if (u.s.role != 0) { u.rpm = u.s.rpm; (u.s.role == 1 ? o.cap_heave : o.cap_sway) += Tb; }
            double P_rated = (u.s.role == 0 ? o.P_elec_max * eta_chain : Pb) * 1.1;''')
rep('''            add(u.s.role == 0 ? "main motor + gearbox" : "thruster motor", m_motor, u.pos.x + 0.06, 0);''', '''            if (u.dual) m_prop += 0.12 * std::pow(u.crp.b.D / 0.1, 3) * u.crp.b.Z / 3.0;
            add(u.s.role == 0 ? "main motor + gearbox" : "thruster motor", m_motor, u.pos.x + 0.06, 0);
            if (u.dual) add("coaxial contra-rotating gearbox", 0.35 + 0.004 * P_rated, u.pos.x + 0.10, 0);
            // propeller report row and shaft design
            {
                PropRow r; r.role = u.s.role; r.Z = u.s.blades; r.Z2 = u.dual ? u.s.blades2 : 0; r.D = u.s.D; r.PD = u.s.pd; r.EAR = u.s.ear; r.contra = u.dual;
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
            }''')
a = '        double Vint = M_PI * 0.25'
rep(a, '''        for (const PropRow& r : o.prop_rows) {
            const char* who = r.role == 0 ? "Main propeller" : (r.role == 1 ? "Vertical thruster" : "Lateral thruster");
            if (r.EAR < r.ear_min) o.warnings.push_back(std::string(who) + ": blade area ratio " + num(r.EAR, "%.2f") + " is below the Keller cavitation minimum " + num(r.ear_min, "%.2f") + ". Use more blades, a bigger diameter or a larger AE/A0.");
            if (!r.crit_margin || r.crit_margin < 1.4) o.warnings.push_back(std::string(who) + " shaft: first critical speed " + num(r.n_crit, "%.0f") + " rpm is too close to the running speed. Use a thicker shaft or a shorter overhang.");
        }
''' + a)
b = '    std::printf("endurance %.1f h at cruise'
rep(b, '''    for (const PropRow& r : o.prop_rows) {
        std::printf("%s: %s%d%s blades D %.0f mm P/D %.2f AE/A0 %.2f  rpm %.0f%s  tip %.1f m/s  Keller min AE/A0 %.2f  Re %.0f  eff(cruise) %.2f\\n", r.role == 0 ? "main prop" : (r.role == 1 ? "vertical thruster" : "lateral thruster"), r.contra ? "CRP " : "", r.Z, r.contra ? ("+" + std::to_string(r.Z2)).c_str() : "", r.D * 1000, r.PD, r.EAR, r.rpm, r.contra ? (" / " + num(r.rpm2, "%.0f")).c_str() : "", r.tip, r.ear_min, r.Re, r.eta_cruise);
        std::printf("   shaft: %.1f mm (IACS %.1f mm), stress %.0f MPa, critical speed %.0f rpm (x%.1f), shaft mass %.2f kg\\n", r.shaft_d * 1000, r.shaft_iacs * 1000, r.shaft_sigma / 1e6, r.n_crit, r.crit_margin, r.shaft_kg);
    }
    for (const FinOutRow& f : o.fin_rows) std::printf("fin at %.0f deg: area %.4f m2 AR %.2f CLa %.3f/deg stall %.0f deg  max force %.1f N hinge %.2f Nm -> stock %.1f mm servo %.1f Nm (%.2f kg)\\n", f.ang, f.area, f.ar, f.cla_deg, f.stall_deg, f.F_max, f.hinge, f.stock_d * 1000, f.servo_Nm, f.servo_kg);
''' + b)
open(p, 'w').write(s)
print('src/auv_design.hpp patched: propeller, shaft and fin design added')
