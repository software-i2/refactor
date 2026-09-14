// Copyright by BeeX [2026]

#include <n_kine/Angle.h>
#include <n_kine/Fk.h>
#include <n_kine/Ik.h>

#include <cmath>
#include <limits>

namespace kine {
namespace {

constexpr double kGeomEps = 1e-9;
constexpr double kAxisEps = 1e-3;

int rank(Fail f) {
    switch (f) {
    case Fail::LIMIT:
        return 3;
    case Fail::TOO_FAR:
        return 2;
    case Fail::TOO_CLOSE:
        return 1;
    default:
        return 0;
    }
}

// Shoulder and elbow for a point in the arm plane, by the cosine rule.
Fail solvePlanar(const Geom &g,
                 double plane_x,
                 double plane_z,
                 const Link &arm,
                 const Joints &seed,
                 bool elbow_up,
                 double &q_shoulder,
                 double &q_elbow) {

    const Params &p  = g.params();
    const Link    l1 = g.upperArm();

    const double X = plane_x - p.e_to_d_x;
    const double Z = plane_z - p.e_to_d_z;
    const double r = std::hypot(X, Z);

    if (r > l1.len + arm.len + kGeomEps) {
        return Fail::TOO_FAR;
    }
    if (r < std::fabs(l1.len - arm.len) - kGeomEps) {
        return Fail::TOO_CLOSE;
    }

    const double cos_c = clamp((r * r - l1.len * l1.len - arm.len * arm.len)
                                       / (2.0 * l1.len * arm.len),
                               -1.0, 1.0);
    const double c = elbow_up ? std::acos(cos_c) : -std::acos(cos_c);
    const double a = std::atan2(X, Z) - std::atan2(arm.len * std::sin(c),
                                                   l1.len + arm.len * std::cos(c));

    const double raw_shoulder = a - l1.psi;
    // a + c = arm.psi + elbow_sign * q_elbow + q_shoulder, and 1/sign == sign.
    const double raw_elbow = g.elbowSign() * (a + c - arm.psi - raw_shoulder);

    if (!fitToWindow(g, SHOULDER, raw_shoulder, seed[SHOULDER], q_shoulder)) {
        return Fail::LIMIT;
    }
    if (!fitToWindow(g, ELBOW, raw_elbow, seed[ELBOW], q_elbow)) {
        return Fail::LIMIT;
    }
    return Fail::NONE;
}

}  // namespace

const char *reason(Fail f) {
    switch (f) {
    case Fail::TOO_FAR:
        return "beyond the arm's reach";
    case Fail::TOO_CLOSE:
        return "inside the dead zone the arm cannot fold into";
    case Fail::LIMIT:
        return "reachable, but only past a joint limit";
    case Fail::BAD_PARAMS:
        return "the geometry itself is not solvable";
    default:
        return "";
    }
}

Fail worse(Fail a, Fail b) { return rank(b) > rank(a) ? b : a; }

bool fitToWindow(const Geom &g, int joint, double q, double seed, double &out) {
    if (joint < 0 || joint >= DOF) {
        return false;
    }
    const double lo = g.windowLo(joint);
    const double hi = g.windowHi(joint);

    bool   found = false;
    double best  = 0.0;
    double near  = std::numeric_limits<double>::max();

    for (int k = -1; k <= 1; ++k) {
        const double candidate = q + 2.0 * M_PI * k;
        if (candidate < lo || candidate > hi) {
            continue;
        }
        const double d = std::fabs(candidate - seed);
        if (d < near) {
            near  = d;
            best  = candidate;
            found = true;
        }
    }

    if (found) {
        out = best;
    }
    return found;
}

Fail solve(const Geom &g,
           const Vec3 &target,
           double along,
           bool elbow_up,
           const Joints &seed,
           Joints &out) {

    if (!g.ok()) {
        return Fail::BAD_PARAMS;
    }

    // The tool point is on the wrist axis, so the base points at the target.
    const double radial = std::hypot(target.x, target.y);
    double       raw_base = seed[BASE];
    if (radial >= kGeomEps) {
        raw_base = std::atan2(target.y, target.x);
    }

    double q_base = 0.0;
    if (!fitToWindow(g, BASE, raw_base, seed[BASE], q_base)) {
        return Fail::LIMIT;
    }

    double     q_shoulder = 0.0;
    double     q_elbow    = 0.0;
    const Fail f          = solvePlanar(g,
                               radial,
                               target.z - g.params().base_to_e_z,
                               g.forearm(along),
                               seed,
                               elbow_up,
                               q_shoulder,
                               q_elbow);
    if (f != Fail::NONE) {
        return f;
    }

    out[BASE]     = q_base;
    out[SHOULDER] = q_shoulder;
    out[ELBOW]    = q_elbow;
    out[WRIST]    = seed[WRIST];  // the roll cannot move a point on its own axis
    return Fail::NONE;
}

void branches(const Geom &g,
              const Vec3 &target,
              double along,
              const Joints &seed,
              std::vector<Branch> &out,
              Fail &why) {

    why = Fail::NONE;
    out.clear();
    if (!g.ok()) {
        why = Fail::BAD_PARAMS;
        return;
    }

    for (int e = 0; e < 2; ++e) {
        Branch     b;
        const Fail bad = solve(g, target, along, e == 1, seed, b.q);
        if (bad != Fail::NONE) {
            why = worse(why, bad);
            continue;
        }
        b.approach = frame(g, b.q).approach;
        b.elbow_up = e == 1;
        out.push_back(b);
    }

    if (out.empty() && rank(why) == 0) {
        why = Fail::LIMIT;
    }
}

double travel(const Geom &g, const Joints &from, const Joints &to) {
    double cost = 0.0;
    for (int j = 0; j < DOF; ++j) {
        cost += g.params().weight[j] * std::fabs(to[j] - from[j]);
    }
    return cost;
}

int clocking(const Geom &g, const Joints &q, const Vec3 &axis, double out[2]) {
    const Vec3 a = unit(axis);
    if (norm(a) < 0.5) {
        return 0;
    }

    // hinge(q) = hinge(0) cos q + close(0) sin q, so one atan2 in that basis is
    // the roll; the other root is the same grasp with the jaws swapped.
    Joints at_zero = q;
    at_zero[WRIST] = 0.0;
    const Frame f  = frame(g, at_zero);

    const double u = dot(f.hinge, a);
    const double v = dot(f.close, a);
    if (std::hypot(u, v) < kAxisEps) {
        return 0;
    }

    out[0] = std::atan2(v, u);
    out[1] = wrapPi(out[0] + M_PI);
    return 2;
}

}  // namespace kine
