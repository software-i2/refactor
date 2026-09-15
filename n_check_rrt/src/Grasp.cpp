// Copyright by BeeX [2026]

#include <n_check_rrt/Grasp.h>
#include <n_kine/Angle.h>

#include <chrono>
#include <limits>

namespace rrt {
namespace {

constexpr double kNoStandoff = 1e-6;

struct Found {
    kine::Joints goal;
    Path         path;
    Stats        stats;
};

}  // namespace

Choice choose(const kine::Geom &g,
              const check::Body &b,
              const check::Field &f,
              const check::Ask &policy,
              const Settings &s,
              const std::vector<Handle> &handles,
              const kine::Joints &seed,
              std::vector<kine::Vec3> &scratch) {

    Choice out;
    out.per.assign(handles.size(), check::Block::UNREACHABLE);
    out.travel.assign(handles.size(), std::numeric_limits<double>::quiet_NaN());
    out.seconds.assign(handles.size(), 0.0);

    kine::Joints from;
    for (int j = 0; j < kine::DOF; ++j) {
        from[j] = kine::clamp(seed[j], g.windowLo(j), g.windowHi(j));
    }

    std::vector<kine::Vec3> plan_scratch;
    std::vector<Found>      found;

    for (size_t i = 0; i < handles.size(); ++i) {
        check::Ask ask = policy;
        ask.point      = handles[i].point;
        ask.axis       = handles[i].axis;
        ask.approach   = handles[i].approach;
        ask.standoff_m = kNoStandoff;

        found.clear();
        const check::Drive drive = [&](const check::Hold &h) {
            Found        fd;
            const Result r = plan(g, b, f, s, ask.floor_z_m, from, h.joints,
                                  plan_scratch, fd.path, fd.stats);
            ++out.plans;
            out.direct += fd.stats.direct ? 1 : 0;
            out.poses += fd.stats.poses;
            switch (r) {
            case Result::OK:
                fd.goal = h.joints;
                found.push_back(fd);
                return check::Block::NONE;
            case Result::GOAL_FLOOR:
                return check::Block::FLOOR;
            case Result::GOAL_OBSTACLE:
                return check::Block::OBSTACLE;
            default:
                return check::Block::NO_ROUTE;
            }
        };

        const auto        t0 = std::chrono::steady_clock::now();
        const check::Hold h  = check::holdable(g, b, f, ask, seed, scratch, drive);
        out.seconds[i] =
                std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        out.total_s += out.seconds[i];
        out.per[i] = h.block;

        if (!h.ok()) {
            out.block = check::worse(out.block, h.block);
            continue;
        }
        out.travel[i] = h.travel;

        if (out.found && h.travel >= out.hold.travel) {
            continue;
        }
        for (size_t k = found.size(); k-- > 0;) {
            if (found[k].goal == h.joints) {
                out.hold  = h;
                out.index = i;
                out.found = true;
                out.path  = found[k].path;
                out.stats = found[k].stats;
                break;
            }
        }
    }

    if (out.found) {
        out.block = check::Block::NONE;
    }
    return out;
}

}  // namespace rrt
