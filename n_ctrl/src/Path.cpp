// Copyright by BeeX [2026]

#include <n_ctrl/Path.h>
#include <n_kine/Angle.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>

namespace ctrl {
namespace {

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

int stepsBetween(const Params &p, const kine::Joints &from, const kine::Joints &to) {
    double worst_deg = 0.0;
    for (int j = 0; j < kine::DOF; ++j) {
        worst_deg = std::max(worst_deg, kine::rad2deg(std::fabs(to[j] - from[j])));
    }
    const int steps = static_cast<int>(std::ceil(worst_deg / p.max_joint_step_deg));
    return steps < 1 ? 1 : steps;
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
    };
    const Field fields[] = {
            {"ctrl.rate_hz", &rate_hz},
            {"ctrl.max_joint_step_deg", &max_joint_step_deg},
            {"ctrl.line_step_m", &line_step_m},
            {"ctrl.goal_tolerance_deg", &goal_tolerance_deg},
            {"ctrl.feedback_timeout_s", &feedback_timeout_s},
            {"ctrl.arrival_timeout_s", &arrival_timeout_s},
            {"world.floor_z_m", &floor_z_m},
            {"ctrl.pillow_min_step_deg", &pillow_min_step_deg},
            {"ctrl.pillow_follow_frac", &pillow_follow_frac},
    };

    for (const Field &f : fields) {
        if (!std::isfinite(*f.at)) {
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
    leg.target     = target;
    leg.q_wrist    = from[kine::WRIST];
    leg.facing_out = chosen.facing_out;
    leg.elbow_up   = chosen.elbow_up;
    return planLineOn(g, p, from, leg, out, deviation_m);
}

Status planLineOn(const kine::Geom &g,
                  const Params &p,
                  const kine::Joints &from,
                  const Leg &leg,
                  Path &out,
                  double &deviation_m) {

    if (!std::isfinite(leg.q_wrist)) {
        return Status::BAD_STATE;
    }

    const kine::Vec3 start = kine::forward(g, from).throat;
    const kine::Vec3 target = leg.target;
    const kine::Vec3 span  = target - start;
    const double     dist  = kine::norm(span);

    int knots = static_cast<int>(std::ceil(dist / p.line_step_m));
    if (knots < 1) {
        knots = 1;
    }

    Path solved;
    kine::Joints first = from;
    first[kine::WRIST] = leg.q_wrist;
    solved.push_back(first);
    for (int k = 1; k <= knots; ++k) {
        const double     u = static_cast<double>(k) / static_cast<double>(knots);
        const kine::Vec3 at{start.x + span.x * u, start.y + span.y * u, start.z + span.z * u};

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

    deviation_m           = 0.0;
    const size_t measured = out.size();
    for (size_t k = 0; k + 1 < solved.size(); ++k) {
        interpolate(solved[k], solved[k + 1], sub, out);
    }
    for (size_t i = measured; i < out.size(); ++i) {
        const kine::Vec3 at = kine::forward(g, out[i]).throat;
        const kine::Vec3 d  = at - start;
        const double     t  = dist > 1e-12 ? kine::dot(d, span) / (dist * dist) : 0.0;
        const kine::Vec3 on{start.x + span.x * t, start.y + span.y * t, start.z + span.z * t};
        deviation_m = std::max(deviation_m, kine::norm(at - on));
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
        const double low = check::lowestZ(g, path[i]);
        if (low < p.floor_z_m) {
            std::snprintf(buf, sizeof(buf), "%s: waypoint %u drops to %.3f m", reason(Status::FLOOR),
                          static_cast<uint32_t>(i), low);
            why = buf;
            return Status::FLOOR;
        }

        if (field.ok()) {
            body.volume(g, path[i], scratch, vol);
            const int hit = check::firstBlocked(field, vol);
            if (hit >= 0) {
                std::snprintf(buf, sizeof(buf), "%s: the %s does, at waypoint %u of %u",
                              reason(Status::OBSTACLE), check::name(hit),
                              static_cast<uint32_t>(i), static_cast<uint32_t>(path.size()));
                why = buf;
                return Status::OBSTACLE;
            }
        }

        for (int j = 0; j < kine::DOF; ++j) {
            const double wire = kine::toWire(kp, j, path[i][j]);
            if (wire >= kp.wire_lo_deg[j] && wire <= kp.wire_hi_deg[j]) {
                continue;
            }
            std::snprintf(buf, sizeof(buf),
                          "waypoint %u would need joint %d at %.2f deg, outside the [%.2f, %.2f] "
                          "the arm accepts. This usually means zero_offset_deg is uncalibrated.",
                          static_cast<uint32_t>(i), j, wire, kp.wire_lo_deg[j], kp.wire_hi_deg[j]);
            why = buf;
            return Status::LIMIT;
        }
    }

    return Status::OK;
}
