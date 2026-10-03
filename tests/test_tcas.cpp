#include <cassert>
#include <cmath>
#include <cstdio>

#include "tcas.hpp"

using namespace tcas;

int main() {
    // CPA of a head-on encounter is midway in time
    Aircraft own{"O", {0, 0, 10000}, {0, 300, 0}};
    Aircraft intr{"I", {0, 10, 10000}, {0, -300, 0}};
    auto [t, miss] = horizontal_cpa(own, intr);
    assert(std::abs(t - 60.0) < 1e-6);
    assert(miss < 1e-6);

    // Far traffic is clear
    Aircraft far{"F", {30, 30, 30000}, {0, 0, 0}};
    assert(evaluate(own, far).level == Advisory::Clear);

    // Close co-altitude closing traffic triggers RA with a vertical sense
    Aircraft close{"C", {0, 2.5, 10100}, {0, -300, 0}};
    auto th = evaluate(own, close);
    assert(th.level == Advisory::ResolutionAdvisory);
    assert(th.sense == Sense::Descend || th.sense == Sense::Climb);

    // Intruder above -> descend preferred (non-crossing)
    Aircraft above{"A", {0, 2.5, 10250}, {0, -300, 0}};
    assert(evaluate(own, above).sense == Sense::Descend);

    // Sensitivity level increases with altitude
    assert(sensitivity_for(1000).sl < sensitivity_for(30000).sl);

    std::puts("all tests passed");
    return 0;
}
