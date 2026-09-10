// Copyright by BeeX [2026]

#ifndef N_KINE_IK_H
#define N_KINE_IK_H

#include <n_kine/Geom.h>
#include <n_kine/Vec3.h>

#include <vector>

namespace kine {

// Kinematic refusals only. The floor, obstacles and path shape belong to
// whoever plans with these, not here.
enum class Fail { NONE, TOO_FAR, TOO_CLOSE, LIMIT, BAD_PARAMS };

const char *reason(Fail f);

// Keeps the refusal that got furthest through the solve, so a merged result
// names the real obstruction rather than the first one tried.
Fail worse(Fail a, Fail b);

// One posture that puts the tool point on the target. The arm can face the
// target or reach back over itself, with the elbow either way up.
struct Branch {
    Joints q{};
    Vec3   approach;
    bool   facing_out = true;
    bool   elbow_up   = false;
};

// `along` is the tool point being aimed, measured from the axis_b pivot:
// g.throatAlong() to put the jaw throat on the target, g.tipAlong() for the tips.
Fail solve(const Geom &g,
           const Vec3 &target,
           double along,
           bool facing_out,
           bool elbow_up,
           const Joints &seed,
           Joints &out);

// Every posture that reaches the target, up to four.
void branches(const Geom &g,
              const Vec3 &target,
              double along,
              const Joints &seed,
              std::vector<Branch> &out,
              Fail &why);

// Weighted joint travel, for choosing between branches.
double travel(const Geom &g, const Joints &from, const Joints &to);

// Wrist rolls that square the jaw hinge to `axis`: 0 or 2, half a turn apart.
int clocking(const Geom &g, const Joints &q, const Vec3 &axis, double out[2]);

// Nearest legal representative of an angle, since a joint angle is only defined
// up to a full turn. False when no representative fits.
bool fitToWindow(const Geom &g, int joint, double q, double seed, double &out);

}  // namespace kine

#endif  // N_KINE_IK_H
