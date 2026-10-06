// waves.hpp - irregular sea surface built from a wave spectrum (Fossen / ITTC style).
//
// A real sea is a sum of many small regular waves. The spectrum S(w) says how much
// wave energy there is at each frequency w. We pick 32 frequencies, give each wave
//     amplitude = sqrt(2 * S(w) * dw),  random phase,  direction spread around the main one
// and add them up. The result has the Significant Wave Height Hs you asked for.
#pragma once
#include <cmath>
#include <vector>
#include <cstdlib>
#include <algorithm>

struct SeaStateRow { const char* name; double hs_lo, hs_hi, hs_typ; };
// WMO sea state code (Douglas scale), as in Fossen's tables
static const SeaStateRow SEA_STATES[] = {
    {"0  Calm (glassy)",      0.00,  0.00,  0.01},
    {"1  Calm (rippled)",     0.00,  0.10,  0.05},
    {"2  Smooth (wavelets)",  0.10,  0.50,  0.30},
    {"3  Slight",             0.50,  1.25,  0.90},
    {"4  Moderate",           1.25,  2.50,  1.90},
    {"5  Rough",              2.50,  4.00,  3.20},
    {"6  Very rough",         4.00,  6.00,  5.00},
    {"7  High",               6.00,  9.00,  7.50},
    {"8  Very high",          9.00, 14.00, 11.50},
    {"9  Phenomenal",        14.00, 20.00, 16.00},
};
static const int SEA_STATE_COUNT = 10;

struct WaveComp { float kx, ky, omega, amp, phase; };

struct WaveField {
    static const int N = 32;
    // ---- settings ----
    double hs      = 1.9;     // significant wave height, m
    double tp      = 6.9;     // peak period, s
    double dir_deg = 0.0;     // direction the waves travel TOWARD, deg CCW from east
    double gamma   = 3.3;     // JONSWAP peak enhancement (1.0 = Pierson-Moskowitz, fully developed sea)
    double spread  = 4.0;     // directional spreading: energy ~ cos^spread (bigger = more one-directional)
    int    seed    = 1;

    // ---- results ----
    WaveComp c[N];
    double hs_real = 0;       // Hs of the 32 waves we really built
    double wp() const { return 2.0 * M_PI / tp; }

    // Typical peak period and wind speed of a fully developed sea with this Hs
    static double tp_for(double hs) { return std::max(2.8, 5.0 * std::sqrt(hs)); }
    static double wind_for(double hs) { return 6.85 * std::sqrt(hs); }        // m/s at 19.5 m height
    static double beaufort_for(double u) { return std::pow(u / 0.836, 2.0 / 3.0); }

    // ITTC / Fossen two-parameter spectrum (Bretschneider) with optional JONSWAP peak
    double raw_spectrum(double w) const {
        double T1 = 0.834 * tp, T4 = T1 * T1 * T1 * T1;
        double s = 155.0 * hs * hs / (T4 * std::pow(w, 5)) * std::exp(-944.0 / (T4 * std::pow(w, 4)));
        if (gamma > 1.0001) {
            double sigma = (w <= wp()) ? 0.07 : 0.09;
            double Y = std::exp(-std::pow((0.191 * w * T1 - 1.0) / (std::sqrt(2.0) * sigma), 2));
            s *= std::pow(gamma, Y);
        }
        return s;
    }
    double norm = 1.0;        // scales the spectrum so that 4*sqrt(m0) = Hs exactly
    double spectrum(double w) const { return norm * raw_spectrum(w); }

    void build() {
        // 1. normalise: area under S(w) must equal Hs^2/16
        norm = 1.0;
        double m0 = 0, dw = 0.002;
        for (double w = 0.05; w < 8.0; w += dw) m0 += raw_spectrum(w) * dw;
        norm = (m0 > 0) ? hs * hs / 16.0 / m0 : 1.0;

        // 2. choose 32 frequencies around the peak and build the waves
        const double g = 9.81;
        double lo = std::max(0.18, 0.55 * wp());
        double hi = std::min(2.6, 3.2 * wp());
        if (hi < lo + 0.3) hi = lo + 0.3;
        double d = (hi - lo) / N;
        std::srand(seed);
        auto rnd = []() { return std::rand() / (double)RAND_MAX; };
        double sum_a2 = 0;
        for (int i = 0; i < N; ++i) {
            double w = lo + (i + 0.1 + 0.8 * rnd()) * d;
            double a = std::sqrt(2.0 * spectrum(w) * d);
            // direction: draw an angle offset with probability ~ cos^spread (rejection sampling)
            double off;
            do { off = (rnd() - 0.5) * M_PI; } while (rnd() > std::pow(std::cos(off), spread));
            double th = dir_deg * M_PI / 180.0 + off;
            double k = w * w / g;                       // deep-water dispersion relation
            c[i].kx = (float)(k * std::cos(th)); c[i].ky = (float)(k * std::sin(th));
            c[i].omega = (float)w; c[i].amp = (float)a; c[i].phase = (float)(rnd() * 2.0 * M_PI);
            sum_a2 += a * a;
        }
        hs_real = 4.0 * std::sqrt(sum_a2 / 2.0);
    }

    // water surface height (m, + up) at a place and time - needed later for buoyancy
    double elevation(double x, double y, double t) const {
        double h = 0;
        for (int i = 0; i < N; ++i)
            h += c[i].amp * std::cos(c[i].kx * x + c[i].ky * y - c[i].omega * t + c[i].phase);
        return h;
    }
};

