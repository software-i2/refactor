// Copyright by BeeX [2026]

#ifndef N_CHECK_GRASP_H
#define N_CHECK_GRASP_H

#include <n_check/Body.h>
#include <n_check/Field.h>
#include <n_kine/Geom.h>

#include <vector>

namespace check {

enum class Block {
    NONE = 0,
    UNREACHABLE,
    OFF_APPROACH,
    NO_CLOCKING,
    FLOOR,
    NO_STANDOFF,
    OBSTACLE
};

const char *reason(Block b);
const char *name(Block b);
Block       worse(Block a, Block b);

struct Ask {
    kine::Vec3 point;
    kine::Vec3 axis;
    kine::Vec3 approach;

    double standoff_m           = kine::NONE;
    double max_approach_dev_deg = kine::NONE;
    double floor_z_m            = kine::NONE;

    double depth_min_m  = kine::NONE;
    double depth_max_m  = kine::NONE;
    double depth_step_m = kine::NONE;

    int leg_samples = -1;

    void load(conf::Doc &doc) {
        standoff_m           = doc.num("task.standoff_m");
        max_approach_dev_deg = doc.num("task.max_approach_dev_deg");
        floor_z_m            = doc.num("world.floor_z_m");
        depth_min_m          = doc.num("task.depth_min_m");
        depth_max_m          = doc.num("task.depth_max_m");
        depth_step_m         = doc.num("task.depth_step_m");
        leg_samples          = doc.integer("task.leg_samples");
    }

    const char *missing() const;
};

struct Hold {
    Block block = Block::UNREACHABLE;

    kine::Joints joints{};
    kine::Joints standoff{};

    kine::Vec3 point;
    kine::Vec3 standoff_point;
    kine::Vec3 approach;

    double depth_m          = 0.0;
    double approach_dev_rad = 0.0;
    double wrist_margin_rad = 0.0;
    double travel           = 0.0;

    bool facing_out = true;
    bool elbow_up   = false;

    bool ok() const { return block == Block::NONE; }
};

Hold holdable(const kine::Geom &g,
              const Body &b,
              const Field &f,
              const Ask &ask,
              const kine::Joints &seed,
              std::vector<kine::Vec3> &scratch);

}  // namespace check

#endif  // N_CHECK_GRASP_H
