// real_models_mesh.hpp - 3D models of the real vehicles of real_models.hpp.
// The three AUVs (and the REMUS 100) are drawn by the designed-AUV builder from the hull and fins they were built from; the BlueROV2 uses
// the vectored-thruster frame of build_hover (vmesh.hpp) with its six real thrusters.
#pragma once
#include "real_models.hpp"
#include "design_model.hpp"

static inline VModel build_real_model(int id, const Spec& s) {
    if (id == RV_BLUEROV2) return build_hover(s);
    design::DesignIn in = real::design_in(id); design::Designer des; design::DesignOut o = des.run(in);
    return design::build_design_model(in, o, s);
}
