// Copyright by BeeX [2026]

#ifndef N_KINE_ANGLE_H
#define N_KINE_ANGLE_H

#include <n_kine/Geom.h>

#include <cmath>

namespace kine {

inline double deg2rad(double d) { return d * M_PI / 180.0; }
inline double rad2deg(double r) { return r * 180.0 / M_PI; }

inline double wrapPi(double a) {
    while (a > M_PI) {
        a -= 2.0 * M_PI;
    }
    while (a <= -M_PI) {
        a += 2.0 * M_PI;
    }
    return a;
}

inline double clamp(double v, double lo, double hi) { return v < lo ? lo : (v > hi ? hi : v); }

// The one place the wire convention is applied. The driver publishes wire
// degrees; everything in this package works in kinematic radians.
inline double toKinematic(const Params &p, int j, double wire_deg) {
    return p.direction_sign[j] * (deg2rad(wire_deg) - deg2rad(p.zero_offset_deg[j]));
}

inline double toPubDeg(const Params &p, int j, double q_kin) {
    return rad2deg(deg2rad(p.zero_offset_deg[j]) + p.direction_sign[j] * q_kin);
}

inline Joints toKinematic(const Params &p, const double wire_deg[DOF]) {
    Joints q;
    for (int j = 0; j < DOF; ++j) {
        q[j] = toKinematic(p, j, wire_deg[j]);
    }
    return q;
}

}  // namespace kine

#endif  // N_KINE_ANGLE_H
