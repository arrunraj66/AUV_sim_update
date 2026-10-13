// design_report.cpp - print the full report of a design. Build: g++ -std=c++17 -O1 -Isrc design_report.cpp -o design_report
#include "auv_design.hpp"
using namespace design;
int main() {
    Designer ds;
    DesignIn d = default_design(); d.name = "REMUS-like (default)";
    print_report(d, ds.run(d));
    DesignIn e = default_design(); e.name = "6 fins + 2 thrusters"; e.fins[0].count = 6;
    PropSpec v; v.role = 1; v.D = 0.07; v.rpm = 3000; v.x = 0.3; e.props.push_back(v);
    PropSpec s = v; s.role = 2; s.x = -0.3; e.props.push_back(s);
    print_report(e, ds.run(e));
}
