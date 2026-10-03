// TCAS II-style conflict detection and resolution logic.
#pragma once

#include <optional>
#include <string>
#include <vector>

namespace tcas {

struct Vec3 {
    double x{}, y{}, z{};  // x/y in NM, z in ft
};

struct Aircraft {
    std::string id;
    Vec3 pos;   // NM, NM, ft
    Vec3 vel;   // kt, kt, fpm
};

enum class Advisory { Clear, Proximate, TrafficAdvisory, ResolutionAdvisory };
enum class Sense { None, Climb, Descend, MonitorVS };

struct SensitivityLevel {
    int sl;
    double tau_ta_s, tau_ra_s;      // time thresholds
    double dmod_ta_nm, dmod_ra_nm;  // distance modification
    double zthr_ta_ft, zthr_ra_ft;  // vertical thresholds
    double alim_ft;                 // required altitude separation at CPA
};

struct Threat {
    std::string intruder;
    Advisory level{Advisory::Clear};
    Sense sense{Sense::None};
    double range_nm{};
    double rel_alt_ft{};
    double tau_range_s{};
    double tau_vert_s{};
    double t_cpa_s{};
    double miss_dist_nm{};
    double vsep_at_cpa_ft{};
    int target_vs_fpm{};
    std::string callout;
};

SensitivityLevel sensitivity_for(double own_alt_ft);

/// Closest point of approach in the horizontal plane. Returns {t_cpa_s, miss_nm}.
std::pair<double, double> horizontal_cpa(const Aircraft& own, const Aircraft& intr);

/// Evaluate one intruder against ownship.
Threat evaluate(const Aircraft& own, const Aircraft& intr);

/// Evaluate all intruders and return threats sorted by severity then tau.
std::vector<Threat> evaluate_all(const Aircraft& own, const std::vector<Aircraft>& traffic);

/// Advance an aircraft by dt seconds.
void propagate(Aircraft& a, double dt_s);

const char* to_string(Advisory a);
const char* to_string(Sense s);

}  // namespace tcas
