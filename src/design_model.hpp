// design_model.hpp - the 3D model of a designed AUV: a body of revolution from the designer's outline, its fins, a propeller (or two) and the
// thrusters, mast and sensor windows. It uses the same building blocks and colours as the ready-made Torpedo AUV (vmesh.hpp), so it is drawn
// by the same shader. Needs OpenGL only when the model is uploaded (VModel::upload), not here.
#pragma once
#include "models.hpp"
#include "vmesh.hpp"
#include "design_spec.hpp"

namespace design {

inline VModel build_design_model(const DesignIn& in, const DesignOut& o, const Spec& s) {
    VModel vm;
    const float xt = (float)o.x_tail, xn = (float)o.x_nose, L = xn - xt;
    // radius of the hull at x, by linear interpolation of the designer's outline (x, r) points
    auto rad = [&o](float x) {
        const auto& v = o.outline; if (v.empty()) return 0.f;
        if (x <= v.front().first) return (float)v.front().second; if (x >= v.back().first) return (float)v.back().second;
        size_t lo = 0, hi = v.size() - 1; while (hi - lo > 1) { size_t m = (lo + hi) / 2; (v[m].first <= x ? lo : hi) = m; }
        double t = (x - v[lo].first) / std::max(v[hi].first - v[lo].first, 1e-9); return (float)(v[lo].second + t * (v[hi].second - v[lo].second));
    };
    const float xtail_cone = xt + std::min(0.45f, (float)in.tail_frac) * L * 0.85f, xnose_cap = xn - 0.045f * L;
    { VPart& p = add_part(vm); full_hull(p.mesh, xnose_cap, xn, 14, rad, 1, 1, 1); }                  // nose cap (black)
    { VPart& p = add_part(vm); full_hull(p.mesh, xtail_cone, xnose_cap, 90, rad, 1, 1, 0); }           // body (yellow)
    { VPart& p = add_part(vm); full_hull(p.mesh, xt, xtail_cone, 36, rad, 1, 1, 1); }                  // tail cone (black)

    // sensor windows and the antenna mast
    { VPart& p = add_part(vm); bool any = false;
      for (const MassItem& it : o.items) {
          if (it.name.rfind("sensor: ", 0) != 0) continue;
          std::string nm = it.name.substr(8); const SensorPart* sp = sensor_parts();
          for (int id = 0; id < SE_COUNT; ++id) {
              if (nm != sp[id].name) continue; any = true;
              switch (sp[id].place) {
                  case PL_NOSE: add_tube(p.mesh, P3(xn - 0.004, 0, 0), P3(xn + 0.004 - 0.0, 0, 0), (float)(0.22 * in.D), (float)(0.2 * in.D), 20, 6, 6, true, 6); break;
                  case PL_BELLY: add_box(p.mesh, P3(it.x, 0, -0.5 * in.D - 0.004), P3(0.5 * sp[id].length, 0.28 * in.D, 0.012), 2); break;
                  case PL_SIDES: for (int sg = -1; sg <= 1; sg += 2) add_box(p.mesh, P3(it.x, sg * (0.5 * in.D + 0.002), 0), P3(0.5 * sp[id].length + 0.06, 0.004, 0.012), 1); break;
                  default: break;
              }
          }
      }
      if (in.mast) for (const MassItem& it : o.items) if (it.name == "antenna mast + GPS") {
          add_tube(p.mesh, P3(it.x, 0, 0.45 * in.D), P3(it.x, 0, it.z + 0.12), 0.006f, 0.005f, 8, 1, 1);
          add_tube(p.mesh, P3(it.x, 0, it.z + 0.12), P3(it.x, 0, it.z + 0.145), 0.012f, 0.012f, 12, 7, 7, true, 7); any = true;
      }
      if (any) finish(p); }

    // fins: hinged parts, drawn from the report's fin rows (quarter-chord at the design position)
    Spec sd = s;
    for (size_t i = 0; i < o.fin_draw.size() && i < sd.fins.size(); ++i) {
        const DesignOut::FinDraw& fd = o.fin_draw[i];
        sd.fins[i].pos.x = fd.x_le - 0.5 * fd.cr;                                  // fin_part centres the plate on mid-chord
        fin_part(vm, sd, (int)i, (float)fd.cr, (float)fd.ct, (float)fd.span, std::max(0.006f, (float)(0.12 * fd.cr)), 1, (float)fd.sweep);
    }

    // propellers and thrusters
    const PropSpec* mainp = nullptr; const DesignOut::PropDraw* maind = nullptr;
    for (size_t i = 0; i < in.props.size() && i < o.prop_draw.size(); ++i) if (in.props[i].role == 0 && !mainp) { mainp = &in.props[i]; maind = &o.prop_draw[i]; }
    if (mainp) {
        const float px = (float)maind->x, R = (float)(0.5 * mainp->D), hubr = 0.17f * R;
        { VPart& p = add_part(vm);                                                         // hub shaft, duct and struts
          add_tube(p.mesh, P3(px - 0.05, 0, 0), P3(xt + 0.02, 0, 0), hubr, hubr, 16, 2, 2, true, 2);
          if (mainp->ducted) {
              add_tube(p.mesh, P3(px - 0.045, 0, 0), P3(px + 0.05, 0, 0), R * 1.12f, R * 1.12f, 40, 2, 2, false);
              add_tube(p.mesh, P3(px + 0.05, 0, 0), P3(px + 0.058, 0, 0), R * 1.16f, R * 1.16f, 40, 7, 7, false);
              for (int k = 0; k < 4; ++k) { float a = 0.7854f + 1.5708f * k; add_tube(p.mesh, P3(px + 0.02, 0.2f * R * std::cos(a), 0.2f * R * std::sin(a)), P3(px + 0.02, 1.12f * R * std::cos(a), 1.12f * R * std::sin(a)), 0.005f, 0.005f, 8, 2, 2); }
          }
          finish(p); }
        { VPart& p = add_part(vm, 2); p.hinge = {(double)px, 0, 0}; p.axis = {1, 0, 0};            // the spinning propeller(s): twisted, skewed blades (see add_propeller)
          const float skew = 22.f + 6.f * std::max(0, mainp->blades - 3);                         // more blades, more skew, like the big skewed propellers of submarines
          add_propeller(p.mesh, px, R * 0.97f, hubr, std::max(2, mainp->blades), (float)std::max(0.6, mainp->pd), (float)std::max(0.3, mainp->ear), skew, 9, 1);
          if (maind->contra) add_propeller(p.mesh, px - 0.05f, R * 0.97f * (float)mainp->d2_ratio, hubr * 0.9f, std::max(2, mainp->blades2), (float)std::max(0.6, mainp->pd), (float)std::max(0.3, mainp->ear), skew, 9, -1);
          finish(p); }
    }
    { VPart& p = add_part(vm); bool any = false;                                           // vertical and lateral thrusters: a tube through the hull
      for (size_t i = 0; i < in.props.size() && i < o.prop_draw.size(); ++i) {
          const PropSpec& ps = in.props[i]; if (ps.role == 0) continue; any = true;
          V3 c = {o.prop_draw[i].x, -ps.y, ps.z}, d = ps.role == 1 ? V3{0, 0, 1} : V3{0, 1, 0};
          float r = (float)(0.5 * ps.D + 0.008);
          add_tube(p.mesh, F3(c - d * 0.05), F3(c + d * 0.05), r, r, 24, 1, 1);
          add_tube(p.mesh, F3(c + d * 0.05), F3(c + d * 0.058), r + 0.004f, r + 0.004f, 24, 5, 5);
          add_tube(p.mesh, F3(c - d * 0.058), F3(c - d * 0.05), r + 0.004f, r + 0.004f, 24, 5, 5);
      }
      if (any) finish(p); }
    return vm;
}

}  // namespace design
