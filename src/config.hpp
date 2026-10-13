// config.hpp - everything you can configure, as plain data, with JSON save/load. (No GUI code here: the pages are in config_ui.hpp.)
//   Sensors  : a list of sensors; each has a type, a name, a rate, a mount position (x y z, yaw pitch roll) and type-specific parameters
//   AuvCfg   : which vehicle and how it differs from the standard one
//   OtherCfg : physics options, logging, safety
//   DeployCfg: where and how the vehicle is put into the water
#pragma once
#include "mini_json.hpp"
#include <algorithm>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <cstring>

// ------------------------------------------------------------------------------------------------ sensor catalogue
enum ParamKind { PK_FLOAT, PK_INT, PK_BOOL, PK_CHOICE };
struct PDef { const char* key; const char* label; double def, lo, hi; const char* unit; ParamKind kind; const char* items; const char* tip; };
struct SensorType { const char* id; const char* name; double rate_def, rate_lo, rate_hi; const char* note; std::vector<PDef> p; };
enum { S_CAMERA, S_IMU, S_COMPASS, S_DEPTH, S_DVL, S_GPS, S_FLS, S_MBES, S_SSS, S_LIDAR, S_COUNT };

inline const std::vector<SensorType>& sensor_types() {
    static const std::vector<SensorType> T = {
        {"camera", "Camera", 15, 1, 60, "Colour camera. Water absorbs red first, so the picture turns blue-green with depth.", {
            {"width", "Width", 1280, 160, 3840, "px", PK_INT, "", ""},
            {"height", "Height", 720, 120, 2160, "px", PK_INT, "", ""},
            {"hfov", "Horizontal field of view", 90, 20, 170, "deg", PK_FLOAT, "", "In water the field of view shrinks by ~25 % behind a flat port (refraction)."},
            {"near", "Near plane", 0.1, 0.02, 2, "m", PK_FLOAT, "", ""},
            {"far", "Far plane", 100, 5, 500, "m", PK_FLOAT, "", ""},
            {"gain", "Gain", 1, 0.1, 8, "x", PK_FLOAT, "", ""},
            {"noise", "Image noise", 0.01, 0, 0.2, "", PK_FLOAT, "", "Fraction of full scale."},
            {"absorb", "Underwater colour loss and haze", 1, 0, 1, "", PK_BOOL, "", "Uses the water type from the World page."}}},
        {"imu", "IMU (accelerometer + gyro)", 100, 10, 1000, "Specific force and angular rate with white noise and a constant turn-on bias.", {
            {"accel_nd", "Accelerometer noise density", 0.003, 0, 0.05, "m/s2/rtHz", PK_FLOAT, "", "Typical MEMS 0.002 - 0.004."},
            {"gyro_nd", "Gyro noise density", 0.0002, 0, 0.01, "rad/s/rtHz", PK_FLOAT, "", "Typical MEMS 0.0001 - 0.0003; a fibre-optic gyro is ~1e-5."},
            {"accel_bias", "Accelerometer bias", 0.005, 0, 0.1, "m/s2", PK_FLOAT, "", ""},
            {"gyro_bias", "Gyro bias", 0.0003, 0, 0.01, "rad/s", PK_FLOAT, "", "0.0003 rad/s = 62 deg/h."}}},
        {"compass", "Magnetometer (compass)", 20, 1, 200, "Magnetic field in the body frame. Steel and thrusters disturb it (hard iron).", {
            {"noise", "Noise", 0.3, 0, 5, "uT", PK_FLOAT, "", ""},
            {"field", "Local field strength", 50, 20, 70, "uT", PK_FLOAT, "", "About 25 uT near the equator, 60 uT near the poles."},
            {"incl", "Inclination", 60, -90, 90, "deg", PK_FLOAT, "", ""},
            {"decl", "Declination", 0, -30, 30, "deg", PK_FLOAT, "", ""},
            {"hard_iron", "Hard-iron offset", 1, 0, 20, "uT", PK_FLOAT, "", ""}}},
        {"depth", "Depth / pressure sensor", 10, 1, 100, "Pressure from the water density at the vehicle (not a fixed 1 bar per 10 m).", {
            {"noise", "Noise", 0.01, 0, 0.5, "m", PK_FLOAT, "", ""},
            {"resolution", "Resolution", 0.002, 0.0001, 0.1, "m", PK_FLOAT, "", ""},
            {"bias", "Bias", 0, -0.5, 0.5, "m", PK_FLOAT, "", ""},
            {"max_depth", "Rated depth", 300, 5, 6000, "m", PK_FLOAT, "", "Readings stop above this depth."}}},
        {"dvl", "DVL (Doppler velocity log)", 5, 1, 20, "Velocity over the sea floor (bottom lock) or through the water (water track).", {
            {"freq", "Frequency", 1, 0, 2, "kHz", PK_CHOICE, "300\0" "600\0" "1200\0", "Lower frequency = longer range, coarser velocity."},
            {"max_alt", "Maximum altitude (bottom lock)", 90, 5, 300, "m", PK_FLOAT, "", ""},
            {"min_alt", "Minimum altitude", 0.3, 0.05, 2, "m", PK_FLOAT, "", ""},
            {"noise", "Velocity noise", 3, 0, 50, "mm/s", PK_FLOAT, "", ""},
            {"beam_angle", "Beam angle from vertical", 30, 20, 40, "deg", PK_FLOAT, "", ""},
            {"water_track", "Water track when no bottom", 1, 0, 1, "", PK_BOOL, "", ""},
            {"dropouts", "Random dropouts", 1, 0, 50, "%", PK_FLOAT, "", ""}}},
        {"gps", "GPS", 1, 0.2, 10, "Works only when the antenna is out of the water.", {
            {"noise_h", "Horizontal noise", 1.5, 0.1, 10, "m", PK_FLOAT, "", ""},
            {"noise_v", "Vertical noise", 3, 0.1, 20, "m", PK_FLOAT, "", ""},
            {"ant_h", "Antenna height above the hull", 0.2, 0, 2, "m", PK_FLOAT, "", ""}}},
        {"fls", "Forward-looking imaging sonar", 10, 1, 30, "Imaging sonar like the Gemini 720s (720 kHz, 120 deg, 512 beams, ~120 m).", {
            {"freq", "Frequency", 0, 0, 1, "kHz", PK_CHOICE, "720\0" "1200\0", "Higher = finer image, shorter range."},
            {"range", "Range", 50, 1, 120, "m", PK_FLOAT, "", ""},
            {"hfov", "Horizontal field of view", 120, 20, 120, "deg", PK_FLOAT, "", ""},
            {"vfov", "Vertical beam width", 20, 5, 40, "deg", PK_FLOAT, "", ""},
            {"beams", "Beams", 512, 64, 1024, "", PK_INT, "", ""},
            {"noise", "Image noise", 0.05, 0, 0.5, "", PK_FLOAT, "", ""}}},
        {"mbes", "Multibeam echosounder", 10, 1, 30, "Range to the sea floor in a fan across the track (bathymetry).", {
            {"freq", "Frequency", 1, 0, 2, "kHz", PK_CHOICE, "200\0" "400\0" "700\0", ""},
            {"swath", "Swath", 140, 30, 170, "deg", PK_FLOAT, "", ""},
            {"beams", "Beams", 256, 32, 1024, "", PK_INT, "", ""},
            {"range", "Range", 100, 5, 500, "m", PK_FLOAT, "", ""},
            {"noise", "Range noise", 0.2, 0, 2, "%", PK_FLOAT, "", "Percent of the range."}}},
        {"sss", "Side-scan sonar", 10, 1, 30, "Acoustic image of the sea floor on both sides of the track.", {
            {"freq", "Frequency", 2, 0, 3, "kHz", PK_CHOICE, "100\0" "300\0" "450\0" "900\0", ""},
            {"range", "Range per side", 75, 10, 300, "m", PK_FLOAT, "", ""},
            {"beam_h", "Along-track beam width", 0.5, 0.1, 2, "deg", PK_FLOAT, "", ""},
            {"beam_v", "Vertical beam width", 50, 20, 70, "deg", PK_FLOAT, "", ""},
            {"tilt", "Tilt down", 20, 0, 60, "deg", PK_FLOAT, "", ""},
            {"side", "Sides", 0, 0, 2, "", PK_CHOICE, "both\0" "port\0" "starboard\0", ""}}},
        {"lidar", "LiDAR", 10, 1, 30, "Light does not travel far in water: this is for the surfaced or out-of-water case.", {
            {"channels", "Channels", 16, 1, 128, "", PK_INT, "", ""},
            {"hfov", "Horizontal field of view", 360, 10, 360, "deg", PK_FLOAT, "", ""},
            {"vfov", "Vertical field of view", 30, 1, 60, "deg", PK_FLOAT, "", ""},
            {"points", "Points per turn", 1800, 100, 4000, "", PK_INT, "", ""},
            {"range", "Range", 100, 5, 300, "m", PK_FLOAT, "", ""},
            {"noise", "Range noise", 0.02, 0, 0.2, "m", PK_FLOAT, "", ""}}},
    };
    return T;
}

struct Sensor {
    int type = 0; std::string name; bool enabled = true; double rate = 10; double mount[6] = {0, 0, 0, 0, 0, 0};   // x y z (m, body frame: x forward y left z up), yaw pitch roll (deg)
    std::vector<double> p;
    double get(const char* key) const {
        const auto& d = sensor_types()[type].p;
        for (size_t i = 0; i < d.size(); ++i) if (!std::strcmp(d[i].key, key)) return i < p.size() ? p[i] : d[i].def;
        return 0;
    }
};
inline Sensor make_sensor(int type, const std::string& name, double x = 0, double y = 0, double z = 0, double yaw = 0, double pitch = 0, double roll = 0) {
    Sensor s; s.type = type; s.name = name; const SensorType& t = sensor_types()[type]; s.rate = t.rate_def;
    for (const PDef& d : t.p) s.p.push_back(d.def);
    s.mount[0] = x; s.mount[1] = y; s.mount[2] = z; s.mount[3] = yaw; s.mount[4] = pitch; s.mount[5] = roll; return s;
}
inline const char* const* sensor_suite_names() { static const char* n[] = {"Minimal navigation", "Inspection AUV", "Mapping AUV", "Everything", nullptr}; return n; }
inline std::vector<Sensor> sensor_suite(int k) {
    std::vector<Sensor> v;
    v.push_back(make_sensor(S_IMU, "imu", 0, 0, 0)); v.push_back(make_sensor(S_COMPASS, "compass", -0.2, 0, 0.05)); v.push_back(make_sensor(S_DEPTH, "depth", 0.1, 0, 0.1));
    if (k == 0) return v;
    v.push_back(make_sensor(S_DVL, "dvl", -0.3, 0, -0.12, 0, -90));
    if (k == 1 || k == 3) {
        v.push_back(make_sensor(S_CAMERA, "camera_front", 0.45, 0, 0.0));
        v.push_back(make_sensor(S_CAMERA, "camera_down", 0.2, 0, -0.1, 0, -90));
        v.push_back(make_sensor(S_FLS, "sonar_front", 0.4, 0, -0.05, 0, -10));
    }
    if (k == 2 || k == 3) {
        v.push_back(make_sensor(S_MBES, "multibeam", 0.0, 0, -0.12, 0, -90));
        v.push_back(make_sensor(S_SSS, "sidescan", -0.1, 0, -0.1));
    }
    if (k == 3) { v.push_back(make_sensor(S_GPS, "gps", -0.5, 0, 0.15)); v.push_back(make_sensor(S_LIDAR, "lidar", 0.1, 0, 0.2)); }
    return v;
}

inline Json sensor_to_json(const Sensor& s) {
    const SensorType& t = sensor_types()[s.type]; Json j = Json::object();
    j["type"] = Json::string(t.id); j["name"] = Json::string(s.name); j["enabled"] = Json::boolean(s.enabled); j["rate_hz"] = Json::number(s.rate);
    Json m = Json::array(); for (int i = 0; i < 6; ++i) m.push(Json::number(s.mount[i])); j["mount_xyz_ypr"] = m;
    Json p = Json::object(); for (size_t i = 0; i < t.p.size(); ++i) p[t.p[i].key] = Json::number(i < s.p.size() ? s.p[i] : t.p[i].def); j["params"] = p;
    return j;
}
inline bool sensor_from_json(const Json& j, Sensor& out) {
    std::string id = j.str_or("type", ""); int type = -1;
    for (int i = 0; i < S_COUNT; ++i) if (id == sensor_types()[i].id) type = i;
    if (type < 0) return false;
    const SensorType& t = sensor_types()[type];
    out = make_sensor(type, j.str_or("name", t.id)); out.enabled = j.bool_or("enabled", true);
    out.rate = std::max(t.rate_lo, std::min(t.rate_hi, j.num_or("rate_hz", t.rate_def)));
    if (const Json* m = j.find("mount_xyz_ypr")) if (m->t == Json::Arr) for (size_t i = 0; i < 6 && i < m->a.size(); ++i) out.mount[i] = m->a[i].n;
    if (const Json* p = j.find("params")) for (size_t i = 0; i < t.p.size(); ++i) out.p[i] = std::max(t.p[i].lo, std::min(t.p[i].hi, p->num_or(t.p[i].key, t.p[i].def)));
    return true;
}
inline Json sensors_to_json(const std::vector<Sensor>& v) { Json j = Json::object(); Json a = Json::array(); for (auto& s : v) a.push(sensor_to_json(s)); j["sensors"] = a; return j; }
inline void sensors_from_json(const Json& j, std::vector<Sensor>& v) {
    const Json* a = j.find("sensors"); if (!a || a->t != Json::Arr) return;
    v.clear(); for (const Json& e : a->a) { Sensor s; if (sensor_from_json(e, s)) v.push_back(s); }
}

// ------------------------------------------------------------------------------------------------ other settings
struct AuvCfg {
    int model = 0; char name[64] = "AUV 1";
    float mass_scale = 1.0f, drag_scale = 1.0f;          // multiply the vehicle's mass (and inertia) and drag (applied when the vehicle is deployed)
    float trim_pct = 0.0f;                               // extra net buoyancy in % of the weight: + floats up, - sinks
    float thrust_scale = 1.0f, battery_Wh = 1500.0f;     // used from Step 5B (new thrusters and battery)
};
inline Json auv_to_json(const AuvCfg& a) {
    Json j = Json::object(); j["model"] = Json::number(a.model); j["name"] = Json::string(a.name);
    j["mass_scale"] = Json::number(a.mass_scale); j["drag_scale"] = Json::number(a.drag_scale);
    j["trim_pct"] = Json::number(a.trim_pct); j["thrust_scale"] = Json::number(a.thrust_scale); j["battery_Wh"] = Json::number(a.battery_Wh); return j;
}
inline void auv_from_json(const Json& j, AuvCfg& a, int model_count) {
    a.model = std::max(0, std::min(model_count - 1, (int)j.num_or("model", a.model)));
    std::snprintf(a.name, sizeof a.name, "%s", j.str_or("name", a.name).c_str());
    auto cl = [&](const char* k, float& v, float lo, float hi) { v = (float)std::max((double)lo, std::min((double)hi, j.num_or(k, v))); };
    cl("mass_scale", a.mass_scale, 0.5f, 2.0f); cl("drag_scale", a.drag_scale, 0.5f, 2.0f); cl("trim_pct", a.trim_pct, -10.0f, 10.0f); cl("thrust_scale", a.thrust_scale, 0.3f, 3.0f); cl("battery_Wh", a.battery_Wh, 50, 20000);
}

struct OtherCfg {
    // physics options: switch whole effects on and off (used from Step 5B, when the physics library drives the vehicles)
    bool prop_wake = true, hull_wake = true, kelvin = true, ripples = true, bubbles = true, sediment = true;
    bool wave_loads = true, wall_effects = true, contact = true, tether = false, stratification = true;
    float tether_len = 30, tether_diam = 0.012f;
    int physics_hz = 200; float failsafe_s = 2.0f;
    bool log_on = false; char log_dir[200] = "logs"; int log_hz = 10;
};
inline Json other_to_json(const OtherCfg& c) {
    Json j = Json::object();
    auto b = [&](const char* k, bool v) { j[k] = Json::boolean(v); };
    b("prop_wake", c.prop_wake); b("hull_wake", c.hull_wake); b("kelvin", c.kelvin); b("ripples", c.ripples); b("bubbles", c.bubbles); b("sediment", c.sediment);
    b("wave_loads", c.wave_loads); b("wall_effects", c.wall_effects); b("contact", c.contact); b("tether", c.tether); b("stratification", c.stratification);
    j["tether_len"] = Json::number(c.tether_len); j["tether_diam"] = Json::number(c.tether_diam);
    j["physics_hz"] = Json::number(c.physics_hz); j["failsafe_s"] = Json::number(c.failsafe_s);
    b("log_on", c.log_on); j["log_dir"] = Json::string(c.log_dir); j["log_hz"] = Json::number(c.log_hz); return j;
}
inline void other_from_json(const Json& j, OtherCfg& c) {
    c.prop_wake = j.bool_or("prop_wake", c.prop_wake); c.hull_wake = j.bool_or("hull_wake", c.hull_wake); c.kelvin = j.bool_or("kelvin", c.kelvin);
    c.ripples = j.bool_or("ripples", c.ripples); c.bubbles = j.bool_or("bubbles", c.bubbles); c.sediment = j.bool_or("sediment", c.sediment);
    c.wave_loads = j.bool_or("wave_loads", c.wave_loads); c.wall_effects = j.bool_or("wall_effects", c.wall_effects); c.contact = j.bool_or("contact", c.contact);
    c.tether = j.bool_or("tether", c.tether); c.stratification = j.bool_or("stratification", c.stratification);
    c.tether_len = (float)std::max(2.0, std::min(500.0, j.num_or("tether_len", c.tether_len))); c.tether_diam = (float)std::max(0.002, std::min(0.05, j.num_or("tether_diam", c.tether_diam)));
    c.physics_hz = (int)std::max(50.0, std::min(1000.0, j.num_or("physics_hz", c.physics_hz))); c.failsafe_s = (float)std::max(0.0, std::min(30.0, j.num_or("failsafe_s", c.failsafe_s)));
    c.log_on = j.bool_or("log_on", c.log_on); std::snprintf(c.log_dir, sizeof c.log_dir, "%s", j.str_or("log_dir", c.log_dir).c_str());
    c.log_hz = (int)std::max(1.0, std::min(200.0, j.num_or("log_hz", c.log_hz)));
}

struct DeployCfg {
    bool at_camera = true;            // put the vehicle in front of the camera
    float x = 0, y = 0, depth = 6.0f, heading = 0;   // used when at_camera is false; heading: degrees, 0 = toward +x (east), 90 = +y (north)
};
inline Json deploy_to_json(const DeployCfg& d) {
    Json j = Json::object(); j["at_camera"] = Json::boolean(d.at_camera); j["x"] = Json::number(d.x); j["y"] = Json::number(d.y); j["depth"] = Json::number(d.depth); j["heading_deg"] = Json::number(d.heading); return j;
}
inline void deploy_from_json(const Json& j, DeployCfg& d) {
    d.at_camera = j.bool_or("at_camera", d.at_camera);
    d.x = (float)std::max(-380.0, std::min(380.0, j.num_or("x", d.x))); d.y = (float)std::max(-380.0, std::min(380.0, j.num_or("y", d.y)));
    d.depth = (float)std::max(0.0, std::min(6000.0, j.num_or("depth", d.depth))); d.heading = (float)j.num_or("heading_deg", d.heading);
}

// ------------------------------------------------------------------------------------------------ files and presets
inline bool save_json_file(const std::string& path, const Json& j, std::string& err) {
    std::error_code ec; std::filesystem::create_directories(std::filesystem::path(path).parent_path(), ec);
    std::ofstream f(path); if (!f) { err = "cannot write " + path; return false; }
    f << j.dump(); return true;
}
inline bool load_json_file(const std::string& path, Json& j, std::string& err) {
    std::ifstream f(path); if (!f) { err = "cannot open " + path; return false; }
    std::stringstream ss; ss << f.rdbuf(); return Json::parse(ss.str(), j, err);
}
inline std::vector<std::string> list_presets(const std::string& dir) {
    std::vector<std::string> v; std::error_code ec;
    for (auto& e : std::filesystem::directory_iterator(dir, ec)) if (e.path().extension() == ".json") v.push_back(e.path().stem().string());
    std::sort(v.begin(), v.end()); return v;
}
inline std::string safe_name(const std::string& s) {
    std::string o; for (char c : s) o += (std::isalnum((unsigned char)c) || c == '-' || c == '_' || c == ' ') ? c : '_';
    while (!o.empty() && o.back() == ' ') o.pop_back();
    return o;
}
