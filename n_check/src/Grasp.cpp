// Copyright by BeeX [2026]

#include <n_check/Grasp.h>
#include <n_kine/Angle.h>
#include <n_kine/Fk.h>
#include <n_kine/Ik.h>

#include <algorithm>
#include <cmath>

namespace check {
namespace {

constexpr double kUnitEps  = 1e-9;
constexpr int    kPostures = 4;
constexpr int    kAimTries = 8;
constexpr double kAimTol   = 1e-6;

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

        b.volume(g, q, scratch, vol);
        if (lowestZ(vol) < floor_z) {
            hit_floor = true;
            return false;
        }
        if (f.ok() && firstBlocked(f, vol) >= 0) {
            return false;
        }
    }
    return true;
}

bool aim(const kine::Geom &g,
         const kine::Vec3 &point,
         double back,
         const kine::Vec3 &guess,
         bool facing_out,
         bool elbow_up,
         const kine::Joints &seed,
         kine::Branch &out,
         kine::Vec3 &target) {

    target = point - guess * back;
    for (int i = 0;; ++i) {
        if (kine::solve(g, target, g.throatAlong(), facing_out, elbow_up, seed, out.q)
            != kine::Fail::NONE) {
            return false;
        }
        out.approach          = kine::frame(g, out.q).approach;
        const kine::Vec3 next = point - out.approach * back;
        if (kine::norm(next - target) < kAimTol || i + 1 >= kAimTries) {
            break;
        }
        target = next;
    }
    out.facing_out = facing_out;
    out.elbow_up   = elbow_up;
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
    case Block::NO_LINE:
        return "no_line";
    case Block::OBSTACLE:
        return "obstacle";
    case Block::NO_ROUTE:
        return "no_route";
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
    case Block::NO_LINE:
        return "the standoff and the handle both solve, but the straight run between them "
               "cannot be driven on one posture";
    case Block::OBSTACLE:
        return "the arm can hold the handle, but the grasp, the standoff, or the run "
               "between them sweeps through a mapped obstacle";
    case Block::NO_ROUTE:
        return "the hold is clear, but the move from where the arm is now to its standoff "
               "crosses the floor, an obstacle or a joint limit";
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
        bool          must_be_positive;
    };
    const Field fields[] = {
            {"task.standoff_m", &standoff_m, true},
            {"task.max_approach_dev_deg", &max_approach_dev_deg, false},
            {"world.floor_z_m", &floor_z_m, false},
            {"task.depth_min_m", &depth_min_m, true},
            {"task.depth_max_m", &depth_max_m, true},
            {"task.depth_step_m", &depth_step_m, true},
    };

    for (const Field &f : fields) {
        if (!std::isfinite(*f.at) || (f.must_be_positive && *f.at <= 0.0)) {
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
              std::vector<kine::Vec3> &scratch,
              const Drive &drive) {

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

    const int facings = std::hypot(ask.point.x, ask.point.y) < kUnitEps ? 1 : 2;

    Block worst = Block::NONE;
    Hold  best[kPostures];
    bool  settled[kPostures] = {false, false, false, false};

    // Try the deepest valid hold first; the throat opens toward the tip, so it is
    // the most forgiving clearance point.
    for (double depth = deepest; depth >= shallow - 1e-9; depth -= step) {
        const double back = depth - nominal;

        for (int p = 0; p < 2 * facings; ++p) {
            if (settled[p]) {
                continue;
            }

            kine::Branch br;
            kine::Vec3   target;
            if (!aim(g, ask.point, back, want, p < 2, p % 2 == 1, seed, br, target)) {
                worst = worse(worst, Block::UNREACHABLE);
                continue;
            }

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

                Body::Volume held;
                b.volume(g, hold, scratch, held);
                if (lowestZ(held) < ask.floor_z_m) {
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

                Hold h;
                h.block            = Block::NONE;
                h.joints           = hold;
                h.standoff         = stand;
                h.point            = target;
                h.standoff_point   = back_off;
                h.approach         = br.approach;
                h.depth_m          = depth;
                h.approach_dev_rad = dev;
                h.wrist_margin_rad = std::min(q_wrist - g.windowLo(kine::WRIST),
                                              g.windowHi(kine::WRIST) - q_wrist);
                h.travel           = kine::travel(g, seed, stand);
                h.facing_out       = br.facing_out;
                h.elbow_up         = br.elbow_up;

                if (settled[p] && h.travel >= best[p].travel) {
                    continue;
                }

                Block leg = Block::NONE;
                if (drive) {
                    leg = drive(h);
                } else {
                    bool hit_floor = false;
                    if (!legClear(g, b, f, hold, stand, ask.floor_z_m, ask.leg_samples, scratch,
                                  hit_floor)) {
                        leg = hit_floor ? Block::FLOOR : Block::OBSTACLE;
                    }
                }
                if (leg != Block::NONE) {
                    worst = worse(worst, leg);
                    continue;
                }

                best[p]    = h;
                settled[p] = true;
            }

            if (!any_fit) {
                worst = worse(worst, Block::NO_CLOCKING);
            }
        }
    }

    // Least travel wins. The old code scaled this by a weight_travel
    // that multiplied every candidate alike, so it never decided
    // anything; it is not carried over.
    int pick = -1;
    for (int p = 0; p < kPostures; ++p) {
        if (settled[p] && (pick < 0 || best[p].travel < best[pick].travel)) {
            pick = p;
        }
    }
    if (pick < 0) {
        out.block = worst == Block::NONE ? Block::UNREACHABLE : worst;
        return out;
    }
    return best[pick];
}

}  // namespace check
