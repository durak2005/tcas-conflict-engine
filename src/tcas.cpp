#include "tcas.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace tcas {
namespace {
constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr double kHrToS = 3600.0;

// RTCA DO-185B sensitivity level table (simplified).
const SensitivityLevel kTable[] = {
    {3, 25, 15, 0.33, 0.20, 850, 600, 300},
    {4, 30, 20, 0.48, 0.35, 850, 600, 300},
    {5, 40, 25, 0.75, 0.55, 850, 600, 350},
    {6, 45, 30, 1.00, 0.80, 850, 600, 400},
    {7, 48, 35, 1.30, 1.10, 850, 700, 600},
};

double hypot2(double a, double b) { return std::sqrt(a * a + b * b); }
}  // namespace

SensitivityLevel sensitivity_for(double alt) {
    if (alt < 2350) return kTable[0];
    if (alt < 5000) return kTable[1];
    if (alt < 10000) return kTable[2];
    if (alt < 20000) return kTable[3];
    return kTable[4];
}

std::pair<double, double> horizontal_cpa(const Aircraft& own, const Aircraft& intr) {
    const double rx = intr.pos.x - own.pos.x, ry = intr.pos.y - own.pos.y;
    const double vx = (intr.vel.x - own.vel.x) / kHrToS, vy = (intr.vel.y - own.vel.y) / kHrToS;  // NM/s
    const double v2 = vx * vx + vy * vy;
    double t = v2 < 1e-12 ? 0.0 : -(rx * vx + ry * vy) / v2;
    t = std::max(0.0, t);
    return {t, hypot2(rx + vx * t, ry + vy * t)};
}

Threat evaluate(const Aircraft& own, const Aircraft& intr) {
    Threat th;
    th.intruder = intr.id;
    const auto sl = sensitivity_for(own.pos.z);

    const double rx = intr.pos.x - own.pos.x, ry = intr.pos.y - own.pos.y;
    const double range = hypot2(rx, ry);
    const double vx = (intr.vel.x - own.vel.x) / kHrToS, vy = (intr.vel.y - own.vel.y) / kHrToS;
    const double range_rate = (rx * vx + ry * vy) / std::max(range, 1e-9);  // NM/s, negative = closing

    const double h = intr.pos.z - own.pos.z;
    const double hdot = (intr.vel.z - own.vel.z) / 60.0;  // ft/s

    th.range_nm = range;
    th.rel_alt_ft = h;
    // Modified tau (DMOD) keeps protection at slow closure rates.
    th.tau_range_s = range_rate < 0 ? -(range * range - sl.dmod_ra_nm * sl.dmod_ra_nm) / (range * range_rate) : kInf;
    th.tau_range_s = std::max(0.0, th.tau_range_s);
    th.tau_vert_s = (h * hdot < 0) ? -h / hdot : kInf;

    auto [t_cpa, miss] = horizontal_cpa(own, intr);
    th.t_cpa_s = t_cpa;
    th.miss_dist_nm = miss;
    th.vsep_at_cpa_ft = h + hdot * t_cpa;

    auto test = [&](double tau, double dmod, double zthr) {
        const bool horiz = range < dmod || (range_rate < 0 && th.tau_range_s < tau);
        const bool vert = std::abs(h) < zthr || th.tau_vert_s < tau;
        return horiz && vert;
    };

    if (test(sl.tau_ra_s, sl.dmod_ra_nm, sl.zthr_ra_ft) && std::abs(th.vsep_at_cpa_ft) < sl.alim_ft) {
        th.level = Advisory::ResolutionAdvisory;
        // Sense selection: choose the direction giving the larger separation at CPA,
        // assuming a 0.25 g response after 5 s pilot delay to 1500 fpm.
        auto sep_if = [&](double target_fpm) {
            const double t = std::max(0.0, t_cpa - 5.0);
            const double own_vs = own.vel.z / 60.0, tgt = target_fpm / 60.0;
            const double accel = 8.0;  // ft/s^2 (~0.25 g)
            const double t_acc = std::min(t, std::abs(tgt - own_vs) / accel);
            const double dz = own_vs * 5.0 + own_vs * t_acc + 0.5 * std::copysign(accel, tgt - own_vs) * t_acc * t_acc +
                              tgt * (t - t_acc);
            return (intr.pos.z + intr.vel.z / 60.0 * t_cpa) - (own.pos.z + dz);
        };
        const double up = sep_if(1500), down = sep_if(-1500);
        // Prefer non-crossing: if already above, climb unless descend is clearly better.
        if (std::abs(up) >= std::abs(down)) {
            th.sense = Sense::Climb;
            th.target_vs_fpm = 1500;
        } else {
            th.sense = Sense::Descend;
            th.target_vs_fpm = -1500;
        }
        if (std::abs(th.vsep_at_cpa_ft) > sl.alim_ft * 0.8) {
            th.sense = Sense::MonitorVS;
            th.target_vs_fpm = static_cast<int>(own.vel.z);
        }
        const bool crossing = (h > 0) == (th.sense == Sense::Climb);
        th.callout = th.sense == Sense::MonitorVS ? "MONITOR VERTICAL SPEED"
                     : th.sense == Sense::Climb   ? (crossing ? "CLIMB, CROSSING CLIMB" : "CLIMB, CLIMB")
                                                  : (crossing ? "DESCEND, CROSSING DESCEND" : "DESCEND, DESCEND");
    } else if (test(sl.tau_ta_s, sl.dmod_ta_nm, sl.zthr_ta_ft)) {
        th.level = Advisory::TrafficAdvisory;
        th.callout = "TRAFFIC, TRAFFIC";
    } else if (range < 6.0 && std::abs(h) < 1200) {
        th.level = Advisory::Proximate;
    }
    return th;
}

std::vector<Threat> evaluate_all(const Aircraft& own, const std::vector<Aircraft>& traffic) {
    std::vector<Threat> out;
    out.reserve(traffic.size());
    for (const auto& a : traffic) out.push_back(evaluate(own, a));
    std::sort(out.begin(), out.end(), [](const Threat& a, const Threat& b) {
        if (a.level != b.level) return a.level > b.level;
        return a.tau_range_s < b.tau_range_s;
    });
    return out;
}

void propagate(Aircraft& a, double dt) {
    a.pos.x += a.vel.x / kHrToS * dt;
    a.pos.y += a.vel.y / kHrToS * dt;
    a.pos.z += a.vel.z / 60.0 * dt;
}

const char* to_string(Advisory a) {
    switch (a) {
        case Advisory::Clear: return "CLEAR";
        case Advisory::Proximate: return "PROXIMATE";
        case Advisory::TrafficAdvisory: return "TA";
        case Advisory::ResolutionAdvisory: return "RA";
    }
    return "?";
}

const char* to_string(Sense s) {
    switch (s) {
        case Sense::None: return "-";
        case Sense::Climb: return "CLIMB";
        case Sense::Descend: return "DESCEND";
        case Sense::MonitorVS: return "MONITOR VS";
    }
    return "?";
}

}  // namespace tcas
