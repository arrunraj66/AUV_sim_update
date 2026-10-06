// Step 1: the water. Temperature and salinity at a depth -> density.
#include <cstdio>
#include <cmath>
#include <initializer_list>

// Temperature in deg C at a depth in metres: warm on top, cold deep.
double temperature(double depth) {
    const double surface = 20.0;       // deg C at the surface
    const double deep = 4.0;           // deg C in the deep ocean
    const double thermocline = 100.0;  // metres, how fast it cools
    return deep + (surface - deep) * std::exp(-depth / thermocline);
}

// Salinity in practical salinity units (about g of salt per kg of water).
double salinity(double /*depth*/) { return 35.0; }

// Density of sea water in kg/m3 (simple linear approximation,
// good to about 1% for normal ocean water).
double density(double T, double S, double depth) {
    const double rho0 = 1027.0, T0 = 10.0, S0 = 35.0;
    const double alpha = 0.00017;    // thermal expansion per deg C
    const double beta = 0.00076;     // haline contraction per salinity unit
    const double compress = 4.4e-6;  // per metre of depth (pressure squeezes water)
    return rho0 * (1.0 - alpha * (T - T0) + beta * (S - S0)) * (1.0 + compress * depth);
}

int main() {
    std::printf("%8s %8s %8s %10s\n", "depth m", "temp C", "salinity", "density");
    for (double d : {0.0, 10.0, 50.0, 100.0, 500.0, 1000.0}) {
        double T = temperature(d), S = salinity(d);
        std::printf("%8.0f %8.2f %8.1f %10.2f\n", d, T, S, density(T, S, d));
    }
}
