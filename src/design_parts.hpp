// design_parts.hpp - catalogues the design engine picks parts from: sensors (mass, power, size, where they sit), water types,
// battery pack and motor / gearbox selection.
// The sensor numbers are TYPICAL values for each class of instrument (not one maker's data sheet): they are meant for sizing a vehicle.
#pragma once
#include <cmath>
#include <algorithm>
#include <string>
#include <vector>

namespace design {

enum { SE_CAMERA, SE_IMU, SE_COMPASS, SE_DEPTH, SE_DVL, SE_GPS, SE_FLS, SE_MBES, SE_SSS, SE_LIDAR, SE_COUNT };
enum Place { PL_NOSE, PL_BODY, PL_BELLY, PL_MAST, PL_SIDES };

struct SensorPart {
    const char* id; const char* name;
    double mass;            // kg, in air, with its housing and cable
    double power;           // W while working
    double drag_area;       // m2 sticking out of the hull (drag area, Cd 0.8)
    Place place;            // where it is mounted
    double length;          // m of hull length it takes (pressure hull) - for the room check
    const char* note;
};

inline const SensorPart* sensor_parts() {
    static const SensorPart T[SE_COUNT] = {
        {"camera",  "Camera + lights",              0.65, 14.0, 0.0,    PL_NOSE,  0.10, "colour camera in a dome port, two LED lights"},
        {"imu",     "IMU (accelerometer + gyro)",   0.12, 0.6,  0.0,    PL_BODY,  0.04, "MEMS unit, mounted near the centre of gravity"},
        {"compass", "Magnetometer (compass)",       0.06, 0.2,  0.0,    PL_BODY,  0.03, "keep away from the motor and battery"},
        {"depth",   "Depth / pressure sensor",      0.08, 0.1,  0.0003, PL_BODY,  0.02, "pressure port in the hull"},
        {"dvl",     "DVL (Doppler velocity log)",   1.60, 4.5,  0.0020, PL_BELLY, 0.10, "four beams looking down; bottom lock to about 50 m"},
        {"gps",     "GPS antenna",                  0.15, 0.5,  0.0004, PL_MAST,  0.02, "works at the surface only, on the mast"},
        {"fls",     "Forward-looking sonar",        2.40, 12.0, 0.0,    PL_NOSE,  0.22, "multibeam imaging sonar behind an acoustic window"},
        {"mbes",    "Multibeam echosounder",        2.00, 22.0, 0.0030, PL_BELLY, 0.18, "downward fan; bathymetry"},
        {"sss",     "Side-scan sonar (2 heads)",    1.80, 9.0,  0.0030, PL_SIDES, 0.12, "one transducer on each side"},
        {"lidar",   "LiDAR (in a pressure housing)",1.60, 10.0, 0.0,    PL_NOSE,  0.12, "needs a window; short range under water"}};
    return T;
}

// water types: sea water, brackish, fresh water, cold fresh water
struct WaterType { const char* name; double T, S; };
static const WaterType WATERS[4] = {{"Sea water (35 PSU, 15 C)", 15, 35}, {"Brackish (15 PSU, 15 C)", 15, 15}, {"Fresh water (lake, 15 C)", 15, 0}, {"Cold fresh water (4 C)", 4, 0}};
inline double water_density(const WaterType& w) { double dT = w.T - 10.0; return 1027.0 - 0.17 * dT - 0.0045 * dT * dT + 0.78 * (w.S - 35.0); }   // same law as physics/seawater.hpp (surface)

// battery pack and motor selection ----------------------------------------------------------------------
struct PackOut {
    int Ns = 1, Np = 1; double V = 3.7, Ah = 0, I_cruise = 0, I_max = 0, C_max = 0, C_limit = 2.0;
    double kv = 0, gear = 1, I_esc = 0, T_motor = 0; int Np_needed = 1; bool ok = true;
};
// P_cruise, P_max: electrical power drawn by the whole vehicle (W); Q_shaft: propeller torque at top speed (N m); rpm_max: propeller rpm at top speed
inline PackOut pack_design(double Wh, double bus_V, double cell_Ah, double cell_maxC, double P_cruise, double P_max, double Q_shaft, double rpm_max) {
    PackOut p; p.C_limit = cell_maxC;
    p.Ns = std::max(1, (int)std::lround(bus_V / 3.7)); p.V = p.Ns * 3.7;
    p.Ah = Wh / p.V; p.Np = std::max(1, (int)std::ceil(p.Ah / std::max(cell_Ah, 0.5)));
    p.I_cruise = P_cruise / p.V; p.I_max = P_max / p.V;
    double cap_Ah = p.Np * cell_Ah; p.C_max = p.I_max / std::max(cap_Ah, 1e-6);
    p.Np_needed = std::max(p.Np, (int)std::ceil(p.I_max / (std::max(cell_maxC, 0.1) * cell_Ah)));
    p.ok = p.C_max <= cell_maxC + 1e-9;
    // motor: choose a gearbox so that the Kv of the motor is in the common range (about 150 - 400 rpm/V) at 85 % of the bus voltage
    double kv_target = 250.0, g = kv_target * 0.85 * p.V / std::max(rpm_max, 1.0);
    p.gear = std::max(1.0, std::min(40.0, g));
    p.kv = rpm_max * p.gear / (0.85 * p.V);
    p.T_motor = Q_shaft / (p.gear * 0.95);
    p.I_esc = 1.25 * p.I_max;
    return p;
}

}  // namespace design
