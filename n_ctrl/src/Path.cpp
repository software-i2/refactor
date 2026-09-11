// Copyright by BeeX [2026]

#include <n_ctrl/Path.h>
#include <n_kine/Angle.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>

namespace ctrl {
namespace {

constexpr int    kMaxSteps  = 100000;
constexpr double kSamePoint = 1e-9;

Status statusFor(kine::Fail f) {
    switch (f) {
    case kine::Fail::LIMIT:
        return Status::LIMIT;
    case kine::Fail::TOO_FAR:
    case kine::Fail::TOO_CLOSE:
    case kine::Fail::BAD_PARAMS:
        return Status::UNREACHABLE;
    default:
        return Status::OK;
    }
}

// A non-finite ratio would cast to INT_MIN and collapse the whole ramp to one
// waypoint, so the count is clamped rather than trusted.
int stepCount(double span, double per_step) {
    if (!(per_step > 0.0) || !std::isfinite(span)) {
        return kMaxSteps;
    }
    const double steps = std::ceil(span / per_step);
    if (!(steps > 1.0)) {
        return 1;
    }
    return steps >= static_cast<double>(kMaxSteps) ? kMaxSteps : static_cast<int>(steps);
}

int stepsBetween(const Params &p, const kine::Joints &from, const kine::Joints &to) {
    double worst_deg = 0.0;
    for (int j = 0; j < kine::DOF; ++j) {
        worst_deg = std::max(worst_deg, kine::rad2deg(std::fabs(to[j] - from[j])));
    }
    return stepCount(worst_deg, p.max_joint_step_deg);
}

double offLine(const kine::Vec3 &at, const kine::Vec3 &a, const kine::Vec3 &b) {
    const kine::Vec3 span = b - a;
    const double     dist = kine::norm(span);
    const double     t    = dist > 1e-12 ? kine::dot(at - a, span) / (dist * dist) : 0.0;
    return kine::norm(at - (a + span * t));
}

void interpolate(const kine::Joints &from, const kine::Joints &to, int steps, Path &out) {
    for (int s = 1; s <= steps; ++s) {
        const double u = static_cast<double>(s) / static_cast<double>(steps);
        kine::Joints q;
        for (int j = 0; j < kine::DOF; ++j) {
            q[j] = from[j] + (to[j] - from[j]) * u;
        }
        out.push_back(q);
    }
}

}  // namespace

const char *Params::missing() const {
    struct Field {
        const char   *name;
        const double *at;
        bool          must_be_positive;
    };
    const Field fields[] = {
            {"ctrl.rate_hz", &rate_hz, true},
            {"ctrl.max_joint_step_deg", &max_joint_step_deg, true},
            {"ctrl.line_step_m", &line_step_m, true},
            {"ctrl.goal_tolerance_deg", &goal_tolerance_deg, true},
            {"ctrl.feedback_timeout_s", &feedback_timeout_s, true},
            {"ctrl.arrival_timeout_s", &arrival_timeout_s, true},
            {"world.floor_z_m", &floor_z_m, false},
            {"ctrl.pillow_min_step_deg", &pillow_min_step_deg, true},
            {"ctrl.pillow_follow_frac", &pillow_follow_frac, true},
    };

    for (const Field &f : fields) {
        if (!std::isfinite(*f.at) || (f.must_be_positive && *f.at <= 0.0)) {
            return f.name;
        }
    }
    return pillow_strikes >= 1 ? NULL : "ctrl.pillow_strikes";
}

const char *reason(Status s) {
    switch (s) {
    case Status::UNREACHABLE:
        return "no posture puts the jaw throat on that point";
    case Status::LIMIT:
        return "reachable, but only by driving a joint past its limit";
    case Status::FLOOR:
        return "reachable, but the arm would drop through the safety floor";
    case Status::OBSTACLE:
        return "reachable, but the arm would sweep through something the camera mapped";
    case Status::NOT_STRAIGHT:
        return "both ends reachable, but the straight line between them cannot be "
               "followed; ask for a joint move instead";
    case Status::EMPTY:
        return "the arm is already there";
    case Status::BUSY:
        return "a trajectory is still running; call ctrl/stop first";
    case Status::BAD_STATE:
        return "the arm is not where this move assumed it would be";
    default:
        return "";
    }
}

Status solveTarget(const kine::Geom &g,
                   const kine::Joints &seed,
                   const kine::Vec3 &target,
                   kine::Joints &out,
                   kine::Branch *chosen) {

    std::vector<kine::Branch> found;
    kine::Fail                why = kine::Fail::NONE;
    kine::branches(g, target, g.throatAlong(), seed, found, why);
    if (found.empty()) {
        return statusFor(why);
    }

    size_t best = 0;
    for (size_t i = 1; i < found.size(); ++i) {
        if (kine::travel(g, seed, found[i].q) < kine::travel(g, seed, found[best].q)) {
            best = i;
        }
    }
    out = found[best].q;
    if (chosen != NULL) {
        *chosen = found[best];
    }
    return Status::OK;
}

void planJoint(const Params &p,
               const kine::Joints &from,
               const kine::Joints &to,
               Path &out) {
    interpolate(from, to, stepsBetween(p, from, to), out);
}

Status planLine(const kine::Geom &g,
                const Params &p,
                const kine::Joints &from,
                const kine::Vec3 &target,
                Path &out,
                double &deviation_m) {

    kine::Joints goal;
    kine::Branch chosen;
    const Status s = solveTarget(g, from, target, goal, &chosen);
    if (s != Status::OK) {
        return s;
    }

    Leg leg;
    leg.start      = kine::forward(g, from).throat;
    leg.target     = target;
    leg.q_wrist    = from[kine::WRIST];
    leg.facing_out = chosen.facing_out;
    leg.elbow_up   = chosen.elbow_up;
    return planLine(g, p, from, leg, out, deviation_m);
}

Status planLine(const kine::Geom &g,
                  const Params &p,
                  const kine::Joints &from,
                  const Leg &leg,
                  Path &out,
                  double &deviation_m) {

    if (!std::isfinite(leg.q_wrist)) {
        return Status::BAD_STATE;
    }

    const kine::Vec3 here  = kine::forward(g, from).throat;
    const kine::Vec3 span  = leg.target - leg.start;
    const int        knots = stepCount(kine::norm(span), p.line_step_m);
    const int        first = kine::norm(leg.start - here) > kSamePoint ? 0 : 1;

    Path solved;
    kine::Joints origin = from;
    origin[kine::WRIST] = leg.q_wrist;
    solved.push_back(origin);
    for (int k = first; k <= knots; ++k) {
        const double     u  = static_cast<double>(k) / static_cast<double>(knots);
        const kine::Vec3 at = leg.start + span * u;

        kine::Joints     q;
        const kine::Fail bad = kine::solve(g, at, g.throatAlong(), leg.facing_out, leg.elbow_up,
                                           solved.back(), q);
        if (bad != kine::Fail::NONE) {
            return statusFor(bad);
        }
        q[kine::WRIST] = leg.q_wrist;
        solved.push_back(q);
    }

    int sub = 1;
    for (size_t k = 0; k + 1 < solved.size(); ++k) {
        sub = std::max(sub, stepsBetween(p, solved[k], solved[k + 1]));
    }

    deviation_m = 0.0;
    for (size_t k = 0; k + 1 < solved.size(); ++k) {
        const size_t      begin = out.size();
        const bool        onto  = first == 0 && k == 0;
        const kine::Vec3 &a     = onto ? here : leg.start;
        const kine::Vec3 &b     = onto ? leg.start : leg.target;
        interpolate(solved[k], solved[k + 1], sub, out);
        for (size_t i = begin; i < out.size(); ++i) {
            deviation_m = std::max(deviation_m, offLine(kine::forward(g, out[i]).throat, a, b));
        }
    }
    return deviation_m > p.line_step_m ? Status::NOT_STRAIGHT : Status::OK;
}

Status admit(const kine::Geom &g,
             const Params &p,
             const Path &path,
             const check::Field &field,
             const check::Body &body,
             std::vector<kine::Vec3> &scratch,
             std::string &why) {

    if (path.empty()) {
        why = reason(Status::EMPTY);
        return Status::EMPTY;
    }

    const kine::Params &kp = g.params();
    char                buf[224];
    check::Body::Volume vol;

    for (size_t i = 0; i < path.size(); ++i) {
        // The window is the intersection of arm.limit_*_deg and the hardware's own
        // range, so this catches a move_q goal that never went through IK.
        for (int j = 0; j < kine::DOF; ++j) {
            if (path[i][j] >= g.windowLo(j) && path[i][j] <= g.windowHi(j)) {
                continue;
            }
            const double edge_a = kine::toPubDeg(kp, j, g.windowLo(j));
            const double edge_b = kine::toPubDeg(kp, j, g.windowHi(j));
            std::snprintf(buf, sizeof(buf),
                          "waypoint %u would need joint %d at %.2f deg, outside the [%.2f, %.2f] "
                          "the arm accepts. This usually means zero_offset_deg is uncalibrated.",
                          static_cast<uint32_t>(i), j, kine::toPubDeg(kp, j, path[i][j]),
                          std::min(edge_a, edge_b), std::max(edge_a, edge_b));
            why = buf;
            return Status::LIMIT;
        }

        body.volume(g, path[i], scratch, vol);

        const double low = check::lowestZ(vol);
        if (low < p.floor_z_m) {
            std::snprintf(buf, sizeof(buf), "%s: waypoint %u drops to %.3f m", reason(Status::FLOOR),
                          static_cast<uint32_t>(i), low);
            why = buf;
            return Status::FLOOR;
        }

        if (field.ok()) {
            const int hit = check::firstBlocked(field, vol);
            if (hit >= 0) {
                std::snprintf(buf, sizeof(buf), "%s: the %s does, at waypoint %u of %u",
                              reason(Status::OBSTACLE), check::name(hit),
                              static_cast<uint32_t>(i), static_cast<uint32_t>(path.size()));
                why = buf;
                return Status::OBSTACLE;
            }
        }
    }

    return Status::OK;
}

}