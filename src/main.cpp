// Scenario runner: steps encounters forward in time and prints advisories as they change.
#include <cstdio>
#include <map>
#include <string>

#include "tcas.hpp"

using namespace tcas;

struct Scenario {
    std::string name;
    Aircraft own;
    std::vector<Aircraft> traffic;
};

static std::vector<Scenario> scenarios() {
    return {
        {"Head-on, co-altitude (FL120)",
         {"OWN", {0, 0, 12000}, {0, 280, 0}},
         {{"TRK1", {0, 12, 12000}, {0, -260, 0}}}},
        {"Converging climb into level traffic (5000 ft)",
         {"OWN", {0, 0, 4200}, {0, 220, 1500}},
         {{"TRK2", {4, 6, 5000}, {-200, -60, 0}}}},
        {"Overtake, 400 ft below, descending (FL250)",
         {"OWN", {0, 0, 25000}, {0, 300, 0}},
         {{"TRK3", {0, -3, 25400}, {0, 420, -800}}}},
        {"Multi-threat terminal area (3000 ft)",
         {"OWN", {0, 0, 3000}, {0, 180, -700}},
         {{"TRK4", {3, 4, 2700}, {-150, -80, 0}},
          {"TRK5", {-2, 7, 3500}, {60, -150, -500}},
          {"TRK6", {8, 1, 6000}, {-100, 0, 0}}}},
    };
}

int main() {
    for (auto sc : scenarios()) {
        std::printf("\n=== %s ===\n", sc.name.c_str());
        std::printf("  t(s)  intruder  adv   range  relalt   tau   cpa(s) miss   sense       callout\n");
        std::map<std::string, Advisory> last;
        for (int t = 0; t <= 120; ++t) {
            for (const auto& th : evaluate_all(sc.own, sc.traffic)) {
                auto it = last.find(th.intruder);
                const bool was_ra = it != last.end() && it->second == Advisory::ResolutionAdvisory;
                // RA hysteresis: keep the RA until the encounter is past CPA.
                if (was_ra && th.level != Advisory::ResolutionAdvisory && th.t_cpa_s > 0) continue;
                if (was_ra && th.level != Advisory::ResolutionAdvisory) {
                    std::printf("  %4d  %-8s  COC   clear of conflict, return to clearance\n", t, th.intruder.c_str());
                    sc.own.vel.z = 0;
                }
                if (it != last.end() && it->second == th.level) continue;
                last[th.intruder] = th.level;
                std::printf("  %4d  %-8s  %-5s %5.2f  %+6.0f  %5.1f  %5.1f  %4.2f  %-10s  %s\n", t, th.intruder.c_str(),
                            to_string(th.level), th.range_nm, th.rel_alt_ft, th.tau_range_s > 999 ? 999.0 : th.tau_range_s,
                            th.t_cpa_s, th.miss_dist_nm, to_string(th.sense), th.callout.c_str());
                // Pilot follows RA after 5 s delay (modeled as immediate VS change here).
                if (th.level == Advisory::ResolutionAdvisory && th.sense != Sense::MonitorVS)
                    sc.own.vel.z = th.target_vs_fpm;
            }
            propagate(sc.own, 1.0);
            for (auto& a : sc.traffic) propagate(a, 1.0);
        }
    }
    return 0;
}
