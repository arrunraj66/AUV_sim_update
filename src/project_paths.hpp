// project_paths.hpp - where things live, independent of the directory the program was started from.
// The project root is the folder that holds CMakeLists.txt: the parent of the executable's folder (build/), or the executable's folder itself.
// Designs go in <root>/config/designs. The 6A page presets stay in config/auv (they are resolved by the config code and are not touched here).
#pragma once
#include <string>
#include <vector>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <cstdlib>
#include <unistd.h>

namespace paths {
namespace fs = std::filesystem;

inline std::string exe_dir() {
    char b[4096]; ssize_t n = readlink("/proc/self/exe", b, sizeof b - 1);
    if (n <= 0) return fs::current_path().string();
    b[n] = 0; return fs::path(b).parent_path().string();
}
inline std::string project_root(const std::string& hint = "") {
    std::error_code ec;
    if (hint.empty()) if (const char* env = std::getenv("AUV_PROJECT_ROOT")) if (*env) return fs::absolute(env, ec).lexically_normal().string();      // tests and unusual installs
    if (!hint.empty()) return fs::absolute(hint, ec).lexically_normal().string();
    fs::path e = exe_dir();
    for (fs::path c : {e.parent_path(), e, e.parent_path().parent_path()}) if (fs::exists(c / "CMakeLists.txt", ec) && fs::exists(c / "src", ec)) return c.lexically_normal().string();
    return fs::current_path().string();
}
inline std::string designs_dir(const std::string& root_hint = "") { return (fs::path(project_root(root_hint)) / "config" / "designs").lexically_normal().string(); }
inline std::string config_dir(const std::string& root_hint = "") { return (fs::path(project_root(root_hint)) / "config").lexically_normal().string(); }

// does this file hold an AUV design (and not a 6A page preset)?
inline bool looks_like_design(const std::string& path) {
    std::ifstream f(path); if (!f) return false; std::string t((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
    return t.find("\"hull\"") != std::string::npos && t.find("\"props\"") != std::string::npos;
}
// move design files out of an old folder (config/auv, shared with the 6A presets) into `to`; presets stay. A file that already exists in `to` is left where it is.
inline int migrate_designs(const std::string& from, const std::string& to) {
    std::error_code ec; if (!fs::is_directory(from, ec)) return 0; fs::create_directories(to, ec); int moved = 0;
    for (const auto& e : fs::directory_iterator(from, ec)) {
        if (e.path().extension() != ".json" || !looks_like_design(e.path().string())) continue;
        fs::path dst = fs::path(to) / e.path().filename();
        if (fs::exists(dst, ec)) {                                        // already there: an identical stale copy is removed, a different file is left alone
            std::ifstream a(e.path()), b(dst); std::string ta((std::istreambuf_iterator<char>(a)), std::istreambuf_iterator<char>()), tb((std::istreambuf_iterator<char>(b)), std::istreambuf_iterator<char>());
            if (ta == tb) fs::remove(e.path(), ec);
            continue;
        }
        fs::rename(e.path(), dst, ec); if (ec) { ec.clear(); fs::copy_file(e.path(), dst, ec); if (!ec) fs::remove(e.path(), ec); }
        if (!ec) ++moved;
    }
    return moved;
}
// every place an older version may have left designs
inline int migrate_all(const std::string& designs, const std::string& root_hint = "") {
    int n = 0; std::error_code ec;
    std::vector<std::string> from = {(fs::path(config_dir(root_hint)) / "auv").string(), (fs::path(exe_dir()) / "config" / "auv").string(), (fs::current_path(ec) / "config" / "auv").string()};
    std::vector<std::string> done;
    for (const std::string& f : from) { std::string a = fs::absolute(f, ec).lexically_normal().string(); bool dup = false; for (auto& d : done) dup |= d == a; if (dup) continue; done.push_back(a); n += migrate_designs(a, designs); }
    return n;
}
}  // namespace paths
