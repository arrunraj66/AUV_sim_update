// water.hpp - physical properties of sea water (numbers only, no graphics).
//
//  Density      : UNESCO 1983 (EOS-80) at surface pressure + Tait compression with depth
//  Sound speed  : Mackenzie (1981)
//  Pressure     : hydrostatic, integrated layer by layer with the real density
//  Light        : attenuation per colour for the Jerlov water types
//  Current      : surface current that fades with depth
#pragma once
#include <cmath>
#include <algorithm>

struct Clarity {            // Jerlov water type (approximate diffuse attenuation, 1/m)
    const char* name;
    double K[3];            // red, green, blue
};
static const Clarity CLARITY[] = {
    {"Jerlov I   - clearest open ocean", {0.33, 0.045, 0.020}},
    {"Jerlov IB  - clear ocean",         {0.36, 0.060, 0.030}},
    {"Jerlov II  - temperate ocean",     {0.42, 0.090, 0.055}},
    {"Jerlov III - productive ocean",    {0.50, 0.150, 0.110}},
    {"Coastal 1",                        {0.60, 0.240, 0.210}},
    {"Coastal 5",                        {0.90, 0.420, 0.400}},
    {"Coastal 9  - murky harbour",       {1.40, 0.800, 0.750}},
};
static const int CLARITY_COUNT = sizeof(CLARITY) / sizeof(CLARITY[0]);

struct Water {
    // ---- settings (all changed from the GUI) ----
    double surface_temp = 15.0;    // deg C
    double deep_temp    = 4.0;     // deg C
    double thermocline  = 100.0;   // m   e-folding depth of the temperature change
    double surface_sal  = 35.0;    // PSU
    double deep_sal     = 34.9;    // PSU
    double halocline    = 200.0;   // m   e-folding depth of the salinity change
    int    clarity      = 1;       // index into CLARITY[]

    double temperature_at(double z) const {
        return deep_temp + (surface_temp - deep_temp) * std::exp(-z / thermocline);
    }
    double salinity_at(double z) const {
        return deep_sal + (surface_sal - deep_sal) * std::exp(-z / halocline);
    }

    // UNESCO 1983: density of sea water at surface pressure, kg/m3
    static double density_atm(double T, double S) {
        double rw = 999.842594 + 6.793952e-2 * T - 9.095290e-3 * T * T + 1.001685e-4 * T * T * T
                  - 1.120083e-6 * T * T * T * T + 6.536332e-9 * T * T * T * T * T;
        double A = 8.24493e-1 - 4.0899e-3 * T + 7.6438e-5 * T * T - 8.2467e-7 * T * T * T + 5.3875e-9 * T * T * T * T;
        double B = -5.72466e-3 + 1.0227e-4 * T - 1.6546e-6 * T * T;
        double C = 4.8314e-4;
        return rw + A * S + B * std::pow(std::max(S, 0.0), 1.5) + C * S * S;
    }

    // Density and pressure at a depth. Integrates down in 1 m layers: each layer is
    // squeezed by the pressure of all the water above it (Tait equation, n = 6).
    void state_at(double depth, double& rho, double& p) const {
        const double g = 9.81, n = 6.0;
        p = 101325.0;   // atmosphere on the surface
        double z = 0.0;
        rho = layer_density(0.0, p, n);
        while (z < depth) {
            double dz = std::min(1.0, depth - z);
            rho = layer_density(z + 0.5 * dz, p, n);
            p += rho * g * dz;
            z += dz;
        }
        rho = layer_density(depth, p, n);
    }
    double density_at(double depth) const { double r, p; state_at(depth, r, p); return r; }
    double pressure_at(double depth) const { double r, p; state_at(depth, r, p); return p; }

    // Mackenzie (1981): speed of sound in m/s
    double sound_speed_at(double D) const {
        double T = temperature_at(D), S = salinity_at(D);
        return 1448.96 + 4.591 * T - 5.304e-2 * T * T + 2.374e-4 * T * T * T + 1.340 * (S - 35.0)
             + 1.630e-2 * D + 1.675e-7 * D * D - 1.025e-2 * T * (S - 35.0) - 7.139e-13 * T * D * D * D;
    }

private:
    double layer_density(double z, double p, double n) const {
        double T = temperature_at(z), S = salinity_at(z);
        double K0 = 2.217e9 + 0.0077e9 * (T - 5.0) + 0.0054e9 * (S - 35.0);   // bulk modulus
        return density_atm(T, S) * std::pow(1.0 + n * (p - 101325.0) / K0, 1.0 / n);
    }
};

// Ocean current (Fossen: Vc). Direction is where the water flows TOWARD,
// degrees counter-clockwise from east.
struct Current {
    double speed  = 0.5;    // m/s at the surface
    double dir    = 90.0;   // deg
    double decay  = 60.0;   // m, depth at which the current has dropped to 37%
    void at(double depth, double& vx, double& vy) const {
        double s = speed * std::exp(-depth / decay), a = dir * M_PI / 180.0;
        vx = s * std::cos(a); vy = s * std::sin(a);
    }
};