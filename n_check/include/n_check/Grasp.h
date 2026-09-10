// Copyright by BeeX [2026]

#ifndef N_CHECK_GRASP_H
#define N_CHECK_GRASP_H

#include <n_check/Body.h>
#include <n_check/Field.h>
#include <n_kine/Geom.h>

#include <vector>

namespace check {

// Why a hold was refused, ordered by how far through the search it got. The
// deepest failure is the one worth reporting: calling an obstacle hit
// "no clocking" because some other branch failed earlier sends you tuning the
// wrong knob.
enum class Block {
    NONE = 0,
    UNREACHABLE,   // no posture puts the throat on the handle at all
    OFF_APPROACH,  // it can be reached, but only from further off than asked
    NO_CLOCKING,   // approach is fine, no wrist angle squares the jaws to it
    FLOOR,         // holdable, but the arm would drop through the floor
    NO_STANDOFF,   // holdable, but not approachable from a standoff
    OBSTACLE       // the grasp, the standoff, or the run between them is blocked
};

const char *reason(Block b);
const char *name(Block b);
Block       worse(Block a, Block b);

// What the caller wants held. The numbers here are the caller's policy, so they
// arrive as arguments rather than living in this package -- and like everything
// else in this tree they carry no default. n_task fills them from the `task`
// and `world` sections.
struct Ask {
    kine::Vec3 point;     // a point on the handle
    kine::Vec3 axis;      // the handle's axis; its sign does not matter
    kine::Vec3 approach;  // preferred approach; zero length means no preference

    double standoff_m           = kine::NONE;
    double max_approach_dev_deg = kine::NONE;
    double floor_z_m            = kine::NONE;

    // The gap between the blades widens down the claw, so where to hold is
    // searched deepest-first rather than assumed.
    double depth_min_m  = kine::NONE;
    double depth_max_m  = kine::NONE;
    double depth_step_m = kine::NONE;

    // Samples along the straight run from standoff to hold when judging whether
    // that run is clear.
    int leg_samples = -1;

    // Fills the policy half from `task` and `world`. The handle itself --
    // point, axis, approach -- comes from perception, not the config.
    void load(conf::Doc &doc) {
        standoff_m           = doc.num("task.standoff_m");
        max_approach_dev_deg = doc.num("task.max_approach_dev_deg");
        floor_z_m            = doc.num("world.floor_z_m");
        depth_min_m          = doc.num("task.depth_min_m");
        depth_max_m          = doc.num("task.depth_max_m");
        depth_step_m         = doc.num("task.depth_step_m");
        leg_samples          = doc.integer("task.leg_samples");
    }

    // Name of the first field still unset, or NULL when all of them are filled.
    const char *missing() const;
};

struct Hold {
    Block block = Block::UNREACHABLE;

    kine::Joints joints{};    // at the handle
    kine::Joints standoff{};  // at the standoff, on the same branch

    kine::Vec3 point;
    kine::Vec3 standoff_point;
    kine::Vec3 approach;

    double depth_m          = 0.0;
    double approach_dev_rad = 0.0;  // only meaningful when the ask had a preference
    double wrist_margin_rad = 0.0;  // clocking angle's distance to the nearer stop
    double travel           = 0.0;  // weighted joint travel to the standoff

    bool facing_out = true;
    bool elbow_up   = false;

    bool ok() const { return block == Block::NONE; }
};

// Can the jaws take hold here, and in what posture. `scratch` is the blade
// buffer, reused across the many postures this tries.
Hold holdable(const kine::Geom &g,
              const Body &b,
              const Field &f,
              const Ask &ask,
              const kine::Joints &seed,
              std::vector<kine::Vec3> &scratch);

}  // namespace check

#endif  // N_CHECK_GRASP_H
