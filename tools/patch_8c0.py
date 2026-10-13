#!/usr/bin/env python3
# patch_8c0.py - Step 8C-0: sensors with mass/power/drag, battery pack + motor selection, water types, drawing data -> src/auv_design.hpp
import sys, os
os.chdir(os.path.expanduser(sys.argv[1] if len(sys.argv) > 1 else '~/Documents/auvsim_new'))
p = 'src/auv_design.hpp'; s = open(p).read()
if 'design_parts.hpp' in s: print('already patched'); sys.exit(0)
def rep(a, b):
    global s
    if a not in s: sys.exit('ANCHOR NOT FOUND (is src/auv_design.hpp the Step 8A + 8B version?):\n' + a[:90])
    s = s.replace(a, b, 1)
rep('#include "prop_design.hpp"\n', '#include "prop_design.hpp"\n#include "design_parts.hpp"\n#include <utility>\n')
rep('    double temp = 15, salinity = 35;\n', '''    double temp = 15, salinity = 35;
    int water = 0;                             // 0 sea water, 1 brackish, 2 fresh water, 3 cold fresh water, 4 custom (temp + salinity above)
    std::vector<int> sensors;                  // chosen sensors (design::SE_CAMERA ...): each adds mass, power, drag and sits in its place
    double bus_V = 24, cell_Ah = 3.0, cell_maxC = 2.0;     // battery bus voltage, one cell's capacity (Ah) and its maximum discharge rate (C)
''')
rep('    bool ok = true;\n};', '''    bool ok = true;
    // sensors, battery and motor
    double hotel_total = 0, sensors_kg = 0, sensors_W = 0, rho_water = 1025;
    PackOut pack; double net_up_sea = 0, net_up_fresh = 0, net_up_cold = 0;
    // what to draw
    struct FinDraw { double x_le, cr, ct, span, sweep, ang; bool control; };
    struct PropDraw { double x, D; int role; bool contra, ducted; };
    std::vector<std::pair<double, double>> outline; std::vector<FinDraw> fin_draw; std::vector<PropDraw> prop_draw; double x_nose = 0, x_tail = 0;
};''')
rep('        ws = WaterState::at(in.temp, in.salinity, 10.0);', '''        if (in.water >= 0 && in.water < 4) ws = WaterState::at(WATERS[in.water].T, WATERS[in.water].S, 10.0); else ws = WaterState::at(in.temp, in.salinity, 10.0);''')
rep('''        add("payload / sensors", in.payload_kg, xcyl1 - 0.2 * o.Lcyl, -0.05 * in.D);''', '''        add("payload (extra)", in.payload_kg, xcyl1 - 0.2 * o.Lcyl, -0.05 * in.D);
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
        o.hotel_total = in.hotel_W + o.sensors_W;''')
rep('            return Fh + Ff + Fm + Ft;', '            double Fs = 0.5 * rho * 0.8 * sens_A * U * U;\n            return Fh + Ff + Fm + Ft + Fs;')
rep('        double main_T_vmax = 0, main_rc = 0;', '        double main_T_vmax = 0, main_rc = 0, thr_P = 0;')
rep('            if (u.s.role != 0) { u.rpm = u.s.rpm;', '            if (u.s.role != 0) thr_P += 1.1 * Pb / eta_chain;\n            if (u.s.role != 0) { u.rpm = u.s.rpm;')
rep('                o.fin_rows.push_back(r);', '''                o.fin_rows.push_back(r);
                { DesignOut::FinDraw fd; fd.x_le = xf - cx + 0.25 * fs.cr; fd.cr = fs.cr; fd.ct = fs.ct; fd.span = fs.span; fd.sweep = fs.sweep; fd.ang = th * 180 / M_PI; fd.control = fs.control; o.fin_draw.push_back(fd); }''')
rep('                o.prop_rows.push_back(r);', '''                o.prop_rows.push_back(r);
                { DesignOut::PropDraw pd; pd.x = u.pos.x; pd.D = u.s.D; pd.role = u.s.role; pd.contra = u.dual; pd.ducted = u.s.ducted; o.prop_draw.push_back(pd); }''')
rep('        o.B = rho * g * o.V;', '''        o.B = rho * g * o.V; o.rho_water = rho;
        for (int i = 0; i <= 120; ++i) { double xx = -0.5 * in.L + in.L * i / 120.0; o.outline.push_back({xx - cx, ol.r(xx)}); }
        o.x_tail = -0.5 * in.L - cx; o.x_nose = 0.5 * in.L - cx;''')
rep('        double Pc = in.hotel_W + o.P_elec_cruise;', '''        { double Qs = o.P_elec_max * eta_chain / std::max(2 * M_PI * o.rpm_main / 60.0, 1e-6);
          o.pack = pack_design(in.battery_Wh, in.bus_V, in.cell_Ah, in.cell_maxC, o.hotel_total + o.P_elec_cruise, o.hotel_total + o.P_elec_max + thr_P, Qs, o.rpm_main); }
        double Pc = o.hotel_total + o.P_elec_cruise;''')
rep('in.battery_Wh * 0.9 / std::max(in.hotel_W + o.P_elec_max, 1e-3);', 'in.battery_Wh * 0.9 / std::max(o.hotel_total + o.P_elec_max, 1e-3);')
rep('in.payload_kg / 1500.0 + in.electronics_kg / 1200.0;', '(in.payload_kg + o.sensors_kg) / 1500.0 + in.electronics_kg / 1200.0;')
rep("""        if (xb < xcyl0 || xb > xcyl1) { o.warnings.push_back("Battery cannot reach the balance point: the nose or tail is too heavy. Move payload or fins, or add ballast at the other end."); xb = clampd(xb, xcyl0, xcyl1); }
        (void)xlo; (void)xhi;
        it[ib].x = xb;""", """        if (xb < xcyl0 || xb > xcyl1) {                    // the battery alone can not balance it: put it at its limit and slide the lead ballast to finish the job
            xb = clampd(xb, xcyl0, xcyl1); it[ib].x = xb;
            size_t ibl = it.size() - 1; double mx2 = 0; for (size_t i = 0; i < ibl; ++i) mx2 += it[i].m * it[i].x;
            double xball = it[ibl].m > 1e-6 ? -mx2 / it[ibl].m : 0.0, xlim0 = xtail + 0.06 * in.L, xlim1 = xnose - 0.06 * in.L;
            it[ibl].x = clampd(xball, xlim0, xlim1);
            if (xball < xlim0 || xball > xlim1) o.warnings.push_back("It can not be balanced fore and aft: the nose or the tail is too heavy even with the battery and the ballast at their limits. Move the payload, sensors or fins, or add weight at the other end.");
        }
        (void)xlo; (void)xhi;
        it[ib].x = xb;""")
rep('        return o;\n    }\n};', '''        // pack, motor and water
        if (main) {
            if (!o.pack.ok) o.warnings.push_back("Battery cannot give the peak current: " + num(o.pack.I_max, "%.0f") + " A at " + num(o.pack.C_max, "%.1f") + " C, the cells allow " + num(o.pack.C_limit, "%.1f") + " C. Use at least " + std::to_string(o.pack.Np_needed) + " cells in parallel (more capacity) or a higher voltage.");
            if (o.pack.gear > 25) o.warnings.push_back("The motor needs a " + num(o.pack.gear, "%.0f") + ":1 gearbox at this bus voltage: a higher battery voltage or a slower propeller makes it simpler.");
        }
        { double Vv = o.V, Wt = o.W; o.net_up_sea = water_density(WATERS[0]) * g * Vv - Wt; o.net_up_fresh = water_density(WATERS[2]) * g * Vv - Wt; o.net_up_cold = water_density(WATERS[3]) * g * Vv - Wt;
          if (in.water == 0 && o.net_up_fresh < 0) o.warnings.push_back("Balanced for sea water. In fresh water it is " + num(-o.net_up_fresh, "%.1f") + " N too heavy and will sink: take out " + num(-o.net_up_fresh / g, "%.2f") + " kg of ballast for lake work."); }
        return o;
    }
};''')
rep('    std::printf("drag at %.1f m/s', '''    std::printf("sensors: %.2f kg, %.0f W; hotel load %.0f W.  battery: %dS%dP (%.1f V, %.1f Ah), peak %.0f A = %.1f C%s; motor Kv %.0f rpm/V, gearbox %.1f:1, ESC %.0f A\\n", o.sensors_kg, o.sensors_W, o.hotel_total, o.pack.Ns, o.pack.Np, o.pack.V, o.pack.Ah, o.pack.I_max, o.pack.C_max, o.pack.ok ? "" : " (TOO HIGH)", o.pack.kv, o.pack.gear, o.pack.I_esc);
    std::printf("net up: sea %+.1f N, fresh %+.1f N, cold fresh %+.1f N\\n", o.net_up_sea, o.net_up_fresh, o.net_up_cold);
    std::printf("drag at %.1f m/s''')
open(p, 'w').write(s)
print('src/auv_design.hpp patched: sensors, battery pack, motor, water types, drawing data')
