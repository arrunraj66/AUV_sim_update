// design_spec.hpp - turns a design from the AUV Designer (DesignIn + DesignOut) into a simulator Spec (vehicle.hpp).
// No OpenGL in here, so it can be used from tests.
//
// Frames: the designer uses x forward, z up, origin at the centre of buoyancy, y to the starboard side.
// The simulator uses x forward, z up, y to the LEFT (right-handed), and also keeps the body origin at the centre of buoyancy
// (Spec::rb = 0). So only y changes sign.
//
// How the numbers are matched to the report:
//  * mass, CG, inertia, added mass, volume (buoyancy) come straight from DesignOut.
//  * the main propeller gets the bollard thrust of the report; its "thrust falls with speed" line is fitted so that thrust = drag at the
//    reported top speed, and the surge drag coefficient is the one of the report (drag at v_max / v_max^2, hull + fins + mast + sensors).
//  * sway / heave / yaw / pitch / roll drag come from the cross-flow slices of the hull outline (the same law the designer uses).
//  * the electric power of the main propeller is fitted to the reported power at cruise (P = pk * T^1.5).
#pragma once
#include "vehicle.hpp"
#include "auv_design.hpp"
#include <cmath>
#include <algorithm>
#include <array>

namespace design {

inline Spec to_spec(const DesignIn& in, const DesignOut& o) {
    const double PI = 3.14159265358979323846, rho = o.rho_water > 0 ? o.rho_water : 1025.0;
    Spec s;
    s.name = in.name; s.mesh = 0;
    char note[256]; std::snprintf(note, sizeof note, "Your own design: %.2f m x %.0f mm, %.1f kg, top speed %.2f m/s, %.1f h at %.1f m/s. Keys as the Torpedo AUV.", in.L, in.D * 1000, o.mass, o.v_top, o.endurance_h, in.v_cruise);
    s.note = note;
    // ---- rigid body -------------------------------------------------------------------------------------------------
    s.mass = o.mass;
    s.trim = o.V * 1025.0 / std::max(o.mass, 1e-6) - 1.0;      // the simulator takes the displaced volume as mass / 1025 * (1 + trim)
    s.length = in.L; s.width = s.height = in.D;
    s.rg = {o.rg.x, -o.rg.y, o.rg.z}; s.rb = {0, 0, 0};
    for (int i = 0; i < 3; ++i) s.I[i] = std::max(o.I[i], 1e-4);
    for (int i = 0; i < 6; ++i) s.A[i] = o.AM[i];
    s.rated_depth = in.rated_depth; s.battery_Wh = in.battery_Wh; s.hotel_W = o.hotel_total;
    s.chase = std::max(1.5, 3.0 * in.L);
    s.lamp_pos = {o.x_nose, 0, 0}; s.cam_pos = {o.x_nose - 0.02, 0, 0.05 * in.D};

    // ---- hull drag: surge law D = a v^2 + b v through the report's two drag points (cruise and top speed), the rest from the cross-flow slices
    double fin_cd0 = 0.5 * rho * 0.008 * o.fin_area_total;       // the simulator adds the fin drag by itself, so it is taken out of the surge term
    double qa = o.Xuu, lb = 0;
    if (o.drag_max > 0 && o.drag_cruise > 0 && in.v_max > in.v_cruise + 1e-6) {
        double a2 = (o.drag_max / in.v_max - o.drag_cruise / in.v_cruise) / (in.v_max - in.v_cruise), b2 = o.drag_cruise / in.v_cruise - a2 * in.v_cruise;
        if (a2 > 0 && b2 >= 0) { qa = a2; lb = b2; } else qa = o.drag_max / (in.v_max * in.v_max);
    }
    const double qa_design = qa;
    qa = std::max(qa - fin_cd0, 0.2 * qa);
    s.Dq[0] = qa; s.Dl[0] = lb;                      // the two-point fit: kept as a summary (and for vehicles whose axial law is replaced), the table below is what the vehicle uses
    // axial drag table sampled from the designer's own drag law (friction with the ITTC line that falls with Reynolds number, form drag, fins, mast, sensors). A two-point fit through cruise and top
    // speed was 23 to 60 % high below 0.3 m/s; the table follows the designer's law at every speed from 0.05 m/s to 1.2 x the top speed.
    std::vector<std::array<double, 2>> design_curve;
    { std::vector<double> sp_ = {0.05, 0.1, 0.2, 0.35, 0.6, 1.0, in.v_cruise, in.v_max, 1.2 * in.v_max}; std::sort(sp_.begin(), sp_.end()); design::Designer dd;
      for (size_t i = 0; i < sp_.size(); ++i) { if (i && sp_[i] < sp_[i - 1] * 1.05) continue; if (sp_[i] <= 0) continue;
          design::DesignIn i2 = in; i2.v_cruise = sp_[i]; i2.v_max = std::max(sp_[i] + 0.01, in.v_max); double D = dd.run(i2).drag_cruise; if (D > 0) design_curve.push_back({sp_[i], D}); } }
    if (design_curve.size() >= 3) {
        for (const auto& pt : design_curve) s.axial_curve.push_back({pt[0], std::max(pt[1] - fin_cd0 * pt[0] * pt[0], 0.2 * pt[1])});    // the simulator adds the fins' own drag itself
    }
    double dy = 0, dz = 0, d_yaw = 0, d_pitch = 0, d_roll = 0;
    for (const phys::Strip& st : o.hull.st) {
        double cdy = phys::crossflow_cd(st.hw / std::max(st.hh, 1e-6), 1.0e5, false), cdz = phys::crossflow_cd(st.hh / std::max(st.hw, 1e-6), 1.0e5, false);
        double ay = 0.5 * rho * cdy * 2 * st.hh * st.dx, az = 0.5 * rho * cdz * 2 * st.hw * st.dx, x3 = std::fabs(st.x * st.x * st.x);
        dy += ay; dz += az; d_yaw += ay * x3; d_pitch += az * x3;
        DragStrip ds; ds.x = st.x; ds.cy = ay; ds.cz = az; ds.ax = st.dx / std::max(in.L, 1e-6); s.strips.push_back(ds);       // per-strip drag: acts at the slice
        double per = 2 * PI * std::sqrt(0.5 * (st.hw * st.hw + st.hh * st.hh));
        d_roll += 0.5 * rho * 0.01 * per * st.dx * std::pow(0.5 * (st.hw + st.hh), 3);
    }
    s.Dq[1] = dy; s.Dq[2] = dz; s.Dq[3] = d_roll; s.Dq[4] = d_pitch; s.Dq[5] = d_yaw;

    // ---- thrusters ----------------------------------------------------------------------------------------------------
    // Electrical power comes from the actuator-disc law  P = T (Va/2 + sqrt(Va^2/4 + T / (2 rho A))) / zeta  with zeta (the overall efficiency
    // of propeller, motor, ESC and gearbox) taken from the designer's own power figures: for the main propeller from its cruise and top-speed
    // operating points, for tunnel thrusters from the bollard power in PropRow.
    bool have_main = false;
    for (size_t i = 0; i < in.props.size() && i < o.prop_rows.size() && i < o.prop_draw.size(); ++i) {
        const PropSpec& p = in.props[i]; const DesignOut::PropDraw& pd = o.prop_draw[i]; const PropRow& r = o.prop_rows[i];
        Thruster t; double A = PI * 0.25 * p.D * p.D; t.disc_ik = 1.0 / (2.0 * rho * A);
        if (p.role == 0) {
            if (have_main) continue;                       // the designer sizes one main propeller; extra ones are not simulated
            have_main = true;
            t.pos = {pd.x, 0, 0}; t.dir = {1, 0, 0};
            t.max_thrust = std::max(o.thrust_bollard, 1.0); t.tau = 0.4; t.rev = 0.5; t.wake = 0.10;      // 0.10 = the designer's wake fraction
            double Dtop = design_curve.size() >= 3 ? curve_drag(design_curve, o.v_top) : qa_design * o.v_top * o.v_top + lb * o.v_top;       // drag at the top speed (designer's law)
            t.u_free = std::max(o.v_top, 0.3);                                                          // thrust falls linearly from the bollard value to the drag at the top speed
            t.kdrop = std::max(0.05, std::min(0.95, 1.0 - Dtop / t.max_thrust));
            auto zeta_at = [&](double T, double U, double P) {
                double Va = U * (1.0 - t.wake), id = T * (0.5 * Va + std::sqrt(0.25 * Va * Va + T * t.disc_ik)); return P > 1e-9 ? id / P : 0.0; };
            double zc = zeta_at(o.thrust_cruise, in.v_cruise, o.P_elec_cruise), zm = zeta_at(std::max(o.thrust_vmax, 1e-6), in.v_max, o.P_elec_max);
            t.zeta = (zc > 0 && zm > 0) ? 0.5 * (zc + zm) : std::max(zc, zm);
            if (t.zeta <= 0) t.zeta = 0.45;
            // reaction torque of the shaft on the hull: from the shaft power at cruise (single propeller) or the residual of the pair (contra-rotating)
            double Tc = std::max(o.thrust_cruise, 1e-6);
            if (r.contra) t.kq = r.Qnet / Tc;
            else if (o.rpm_cruise > 1.0) t.kq = o.P_shaft_cruise / (2.0 * PI * o.rpm_cruise / 60.0) / Tc;
        } else {
            t.pos = {pd.x, -p.y, p.z}; t.dir = p.role == 1 ? V3{0, 0, 1} : V3{0, 1, 0};
            t.max_thrust = std::max(r.Tb, 1.0); t.tau = 0.25; t.u_free = 2.0; t.rev = 1.0;
            t.zeta = r.Pb_elec > 1e-9 ? std::pow(t.max_thrust, 1.5) * std::sqrt(t.disc_ik) / r.Pb_elec : 0.5;
        }
        s.thr.push_back(t);
    }

    // ---- fins ---------------------------------------------------------------------------------------------------------
    size_t k = 0;
    for (const FinSet& fs : in.fins) {
        if (!fs.on || fs.count < 1) continue;
        for (int j = 0; j < fs.count; ++j, ++k) {
            if (k >= o.fin_rows.size()) break;
            const FinOutRow& r = o.fin_rows[k];
            double th = (fs.angle0 + 360.0 * j / fs.count) * PI / 180.0, xq = fs.x - o.cx;
            double sy = std::sin(th), cz = std::cos(th);
            Fin f;
            f.pos = {xq, -(0.5 * in.D + 0.5 * fs.span) * sy, (0.5 * in.D + 0.5 * fs.span) * cz};      // mid-span, on the quarter-chord line
            f.normal = {0, -cz, -sy};                                                                  // designer normal (0, cos, -sin), y mirrored
            f.area = r.area; f.cl_alpha = r.cla_deg * 180.0 / PI; f.stall = std::max(0.2, r.stall_deg * PI / 180.0);
            f.max_defl = fs.control ? fs.max_defl * PI / 180.0 : 0.0; f.sign = 1.0; f.group = 0;
            if (fs.control) {
                double sg = (f.pos.x - o.rg.x) >= 0 ? 1.0 : -1.0;       // a fin ahead of the CG turns the other way
                double kr = sg * f.normal.y, ke = sg * f.normal.z;
                if (std::fabs(kr) > 0.97) { f.group = 1; f.sign = kr > 0 ? 1.0 : -1.0; }
                else if (std::fabs(ke) > 0.97) { f.group = 2; f.sign = ke > 0 ? 1.0 : -1.0; }
                else { f.group = 3; f.kr = kr; f.ke = ke; }
                if (r.hinge > 1e-6 && r.servo_Nm > 0) f.f_lim = r.servo_Nm * r.F_max / r.hinge;       // force the servo can hold = servo torque / hinge arm
            }
            s.fins.push_back(f);
        }
    }

    double dmax = 0; for (const Fin& f : s.fins) if (f.group) dmax = std::max(dmax, f.max_defl * 180.0 / PI);
    s.servo_rate = std::max(60.0, 3.0 * dmax);                         // deg/s: 60, or three times the maximum deflection per second if that is faster

    // ---- touching the sea bed -------------------------------------------------------------------------------------------
    double R = 0.5 * in.D;
    s.contact = {{o.x_nose, 0, 0}, {o.x_tail, 0, 0}, {0.5 * (o.x_nose + o.x_tail) + 0.25 * in.L, 0, -R}, {0.5 * (o.x_nose + o.x_tail) - 0.25 * in.L, 0, -R},
                 {0.5 * (o.x_nose + o.x_tail), 0, R}, {0.5 * (o.x_nose + o.x_tail), R, 0}, {0.5 * (o.x_nose + o.x_tail), -R, 0}};
    return s;
}

}  // namespace design
