# TCAS Conflict Engine

C++17 implementation of TCAS II-style airborne collision avoidance logic: threat detection, Traffic Advisories (TA), Resolution Advisories (RA) with sense selection, and an encounter simulator.

## Why

TCAS is the last safety layer against mid-air collision and the clearest example of a cockpit system that must *prioritize* — many targets, one aural channel, seconds to act. This engine reproduces the core logic to study timing, sensitivity and advisory behaviour across encounter geometries.

## Features

- **Sensitivity levels** (SL3–SL7) by ownship altitude, after RTCA DO-185B tables
- **Modified tau** (range / closure with DMOD) and vertical tau
- **Horizontal CPA** — time to closest approach and miss distance
- **TA / RA thresholds** with ALIM check on projected vertical separation at CPA
- **RA sense selection** — models a 5 s pilot delay and 0.25 g response to ±1500 fpm, picks the sense with larger separation; detects *crossing* RAs; *Monitor VS* when separation is nearly adequate
- **Multi-threat ranking** — RA > TA > Proximate, then by tau
- **Encounter simulator** — head-on, converging climb, overtake and multi-threat terminal scenarios with RA hysteresis and Clear-of-Conflict

## Build

```bash
cmake -S . -B build
cmake --build build
./build/tcas_sim
ctest --test-dir build
```

Or without CMake:

```bash
g++ -std=c++17 -Iinclude src/tcas.cpp src/main.cpp -o tcas_sim
```

## Sample output

```
=== Head-on, co-altitude (FL120) ===
  t(s)  intruder  adv   range  relalt   tau   cpa(s) miss   sense       callout
    35  TRK1      TA     6.75      +0   44.4   45.0  0.00  -           TRAFFIC, TRAFFIC
    50  TRK1      RA     4.50      +0   29.1   30.0  0.00  CLIMB       CLIMB, CLIMB
    80  TRK1      COC   clear of conflict, return to clearance
```

## Layout

```
include/tcas.hpp   public API
src/tcas.cpp       detection + resolution logic
src/main.cpp       scenario simulator
tests/             unit tests
```

## Disclaimer

Educational model. Not the certified TCAS II algorithm (ACAS X / DO-185B logic is far more extensive).
