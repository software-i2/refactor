// Copyright by BeeX [2026]

#include <n_ctrl/Path.h>
#include <n_task/PickRrt.h>

#include <limits>

namespace task {
namespace {

constexpr double kNoStandoff = 1e-6;

check::Block routeBlock(rrt::Result r) {
    switch (r) {
    case rrt::Result::OK:
        return check::Block::NONE;
    case rrt::Result::GOAL_FLOOR:
        return check::Block::FLOOR;
    case rrt::Result::GOAL_OBSTACLE:
        return check::Block::OBSTACLE;
    default:
        return check::Block::NO_ROUTE;
    }
}

}  // namespace

check::Block drivableRrt(const kine::Geom &g,
                         const check::Body &b,
                         const check::Field &f,
                         const ctrl::Params &motion,
                         const rrt::Settings &settings,
                         const kine::Joints &from,
                         const check::Hold &h,
                         std::vector<kine::Vec3> &scratch) {
    rrt::Path  route;
    rrt::Stats stats;

    if (!settings.standoff) {
        return routeBlock(rrt::plan(g, b, f, settings, motion.floor_z_m, from, h.joints,
                                    scratch, route, stats));
    }

    const check::Block straight = drivable(g, b, f, motion, from, h, scratch);
    if (straight != check::Block::NO_ROUTE) {
        return straight;
    }
    return rrt::plan(g, b, f, settings, motion.floor_z_m, from, h.standoff, scratch,
                     route, stats) == rrt::Result::OK
                   ? check::Block::NONE
                   : check::Block::NO_ROUTE;
}

Choice chooseRrt(const kine::Geom &g,
                 const check::Body &b,
                 const check::Field &f,
                 const check::Ask &policy,
                 const ctrl::Params &motion,
                 const rrt::Settings &settings,
                 const std::vector<Candidate> &candidates,
                 const kine::Joints &seed,
                 std::vector<kine::Vec3> &scratch) {

    Choice out;
    out.per.assign(candidates.size(), check::Block::UNREACHABLE);
    out.travel.assign(candidates.size(), std::numeric_limits<double>::quiet_NaN());

    const kine::Joints      from = ctrl::snapToWindow(g, seed, motion.goal_tolerance_deg);
    std::vector<kine::Vec3> leg_scratch;
    const check::Drive      drive = [&](const check::Hold &h) {
        return drivableRrt(g, b, f, motion, settings, from, h, leg_scratch);
    };

    for (size_t i = 0; i < candidates.size(); ++i) {
        check::Ask ask = policy;
        ask.point      = candidates[i].point;
        ask.axis       = candidates[i].axis;
        ask.approach   = candidates[i].approach;
        if (!settings.standoff) {
            ask.standoff_m = kNoStandoff;
        }

        const check::Hold h = check::holdable(g, b, f, ask, seed, scratch, drive);
        out.per[i]          = h.block;

        if (!h.ok()) {
            out.block = check::worse(out.block, h.block);
            continue;
        }
        out.travel[i] = h.travel;

        if (!out.found || h.travel < out.hold.travel) {
            out.hold  = h;
            out.index = i;
            out.found = true;
        }
    }

    if (out.found) {
        out.block = check::Block::NONE;
    }
    return out;
}

}  // namespace task
