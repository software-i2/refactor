// Copyright by BeeX [2026]

#include <n_check/Grasp.h>
#include <n_kine/Angle.h>
#include <n_kine/Ik.h>

#include <algorithm>
#include <cmath>

namespace check {
namespace {

constexpr double kUnitEps = 1e-9;

// Sample the straight run from standoff to handle so the scorer sees the corridor,
// not just the final pose.
bool legClear(const kine::Geom &g,
              const Body &b,
              const Field &f,
              const kine::Joints &grasp,
              const kine::Joints &standoff,
              double floor_z,
              int steps,
              std::vector<kine::Vec3> &scratch,
              bool &hit_floor) {

    hit_floor = false;
    Body::Volume vol;

    for (int s = 0; s <= steps; ++s) {
        const double u = static_cast<double>(s) / static_cast<double>(steps);

        kine::Joints q;
        for (int j = 0; j < kine::DOF; ++j) {
            q[j] = standoff[j] + (grasp[j] - standoff[j]) * u;
        }

        if (lowestZ(g, q) < floor_z) {
            hit_floor = true;
            return false;
        }
        if (f.ok()) {
            b.volume(g, q, scratch, vol);
            if (firstBlocked(f, vol) >= 0) {
                return false;
            }
        }
    }
    return true;
}

}  // namespace

const char *name(Block b) {
    switch (b) {
    case Block::UNREACHABLE:
        return "unreachable";
    case Block::OFF_APPROACH:
        return "off_approach";
    case Block::NO_CLOCKING:
        return "no_clocking";
    case Block::FLOOR:
        return "floor";
    case Block::NO_STANDOFF:
        return "no_standoff";
    case Block::OBSTACLE:
        return "obstacle";
    default:
        return "none";
    }
}

const char *reason(Block b) {
    switch (b) {
    case Block::UNREACHABLE:
        return "nothing the arm can reach puts the jaw throat on the handle";
    case Block::OFF_APPROACH:
        return "the arm can reach the handle, but only from a direction further from the "
               "requested approach than allowed";
    case Block::NO_CLOCKING:
        return "the approach is fine, but no wrist angle squares the jaws to the handle "
               "within the wrist's travel";
    case Block::FLOOR:
        return "holdable, but the arm would drop through the safety floor to do it";
    case Block::NO_STANDOFF:
        return "the handle itself is holdable, but the arm cannot reach a standoff to "
               "approach it from";
    case Block::OBSTACLE:
        return "the arm can hold the handle, but the grasp, the standoff, or the run "
               "between them sweeps through a mapped obstacle";
    default:
        return "";
    }
}

Block worse(Block a, Block b) {
    return static_cast<int>(b) > static_cast<int>(a) ? b : a;
}

const char *Ask::missing() const {
    struct Field {
        const char   *name;
        const double *at;
    };
    const Field fields[] = {
            {"task.standoff_m", &standoff_m},
            {"task.max_approach_dev_deg", &max_approach_dev_deg},
            {"world.floor_z_m", &floor_z_m},
            {"task.depth_min_m", &depth_min_m},
            {"task.depth_max_m", &depth_max_m},
            {"task.depth_step_m", &depth_step_m},
    };

    for (const Field &f : fields) {
        if (!std::isfinite(*f.at)) {
            return f.name;
        }
    }
    return leg_samples >= 1 ? NULL : "task.leg_samples";
}

Hold holdable(const kine::Geom &g,
              const Body &b,
              const Field &f,
              const Ask &ask,
              const kine::Joints &seed,
              std::vector<kine::Vec3> &scratch) {

    Hold out;
    if (!g.ok() || ask.missing() != NULL || kine::norm(ask.axis) < kUnitEps) {
        return out;
    }

    const kine::Vec3 axis = kine::unit(ask.axis);
    const double     pn   = kine::norm(ask.approach);

    // The gate only matters when a preferred approach direction is supplied.
    const bool       gated = pn >= kUnitEps && ask.max_approach_dev_deg > 0.0;
    const kine::Vec3 want  = pn >= kUnitEps ? kine::unit(ask.approach) : kine::Vec3();
    const double     worst_dev =
            kine::deg2rad(kine::clamp(ask.max_approach_dev_deg, 0.0, 180.0));

    // Without a specified approach direction, the throat depth itself is the only
    // valid target offset.
    const double nominal = g.params().mount_to_throat;
    double       deepest = std::min(ask.depth_max_m, b.maxDepth());
    double       shallow = std::min(ask.depth_min_m, deepest);
    const double step    = ask.depth_step_m;
    if (pn < kUnitEps) {
        deepest = shallow = nominal;
    }

    Block worst = Block::NONE;
    bool  found = false;

    std::vector<kine::Branch> branches;  // reused across depths

    // Try the deepest valid hold first; the throat opens toward the tip, so it is
    // the most forgiving clearance point.
    for (double depth = deepest; depth >= shallow - 1e-9 && !found; depth -= step) {
        const double     back = depth - nominal;
        const kine::Vec3 target =
                pn < kUnitEps ? ask.point : ask.point - want * back;

        kine::Fail why = kine::Fail::NONE;
        kine::branches(g, target, g.throatAlong(), seed, branches, why);
        if (branches.empty()) {
            worst = worse(worst, Block::UNREACHABLE);
            continue;
        }

        for (size_t i = 0; i < branches.size(); ++i) {
            const kine::Branch &br = branches[i];

            const double dev = gated ? kine::angleTo(br.approach, want) : 0.0;
            if (gated && dev > worst_dev) {
                worst = worse(worst, Block::OFF_APPROACH);
                continue;
            }

            double roll[2] = {0.0, 0.0};
            if (kine::clocking(g, br.q, axis, roll) != 2) {
                worst = worse(worst, Block::NO_CLOCKING);
                continue;
            }

            // One wrist option is usually outside the window; a single miss is not a
            // clocking failure until both options fail.
            bool any_fit = false;
            for (int t = 0; t < 2; ++t) {
                double q_wrist = 0.0;
                if (!kine::fitToWindow(g, kine::WRIST, roll[t], seed[kine::WRIST], q_wrist)) {
                    continue;
                }
                any_fit = true;

                kine::Joints hold = br.q;
                hold[kine::WRIST] = q_wrist;
                if (lowestZ(g, hold) < ask.floor_z_m) {
                    worst = worse(worst, Block::FLOOR);
                    continue;
                }

                // The standoff must solve on the SAME branch, or the arm changes
                // posture mid-approach and the clocking arrives wrong.
                const kine::Vec3 back_off = target - br.approach * ask.standoff_m;

                kine::Joints stand;
                if (kine::solve(g, back_off, g.throatAlong(), br.facing_out, br.elbow_up, hold,
                                stand) != kine::Fail::NONE) {
                    worst = worse(worst, Block::NO_STANDOFF);
                    continue;
                }
                stand[kine::WRIST] = q_wrist;

                bool hit_floor = false;
                if (!legClear(g, b, f, hold, stand, ask.floor_z_m, ask.leg_samples, scratch,
                              hit_floor)) {
                    worst = worse(worst, hit_floor ? Block::FLOOR : Block::OBSTACLE);
                    continue;
                }

                // Least travel wins. The old code scaled this by a weight_travel
                // that multiplied every candidate alike, so it never decided
                // anything; it is not carried over.
                const double travel = kine::travel(g, seed, stand);
                if (found && travel >= out.travel) {
                    continue;
                }

                found                = true;
                out.block            = Block::NONE;
                out.joints           = hold;
                out.standoff         = stand;
                out.point            = target;
                out.standoff_point   = back_off;
                out.approach         = br.approach;
                out.depth_m          = depth;
                out.approach_dev_rad = dev;
                out.wrist_margin_rad = std::min(q_wrist - g.windowLo(kine::WRIST),
                                                g.windowHi(kine::WRIST) - q_wrist);
                out.travel           = travel;
                out.facing_out       = br.facing_out;
                out.elbow_up         = br.elbow_up;
            }

            if (!any_fit) {
                worst = worse(worst, Block::NO_CLOCKING);
            }
        }
    }

    if (!found) {
        out.block = worst == Block::NONE ? Block::UNREACHABLE : worst;
    }
    return out;
}

}  // namespace check
