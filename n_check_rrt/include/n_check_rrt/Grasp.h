// Copyright by BeeX [2026]

#ifndef N_CHECK_RRT_GRASP_H
#define N_CHECK_RRT_GRASP_H

#include <n_check/Grasp.h>
#include <n_check_rrt/Rrt.h>

#include <vector>

namespace rrt {

struct Handle {
    kine::Vec3 point;
    kine::Vec3 axis;
    kine::Vec3 approach;
};

struct Choice {
    check::Hold  hold;
    size_t       index = 0;
    bool         found = false;
    check::Block block = check::Block::UNREACHABLE;

    std::vector<check::Block> per;
    std::vector<double>       travel;
    std::vector<double>       seconds;

    Path  path;
    Stats stats;

    int    plans   = 0;
    int    direct  = 0;
    long   poses   = 0;
    double total_s = 0.0;
};

Choice choose(const kine::Geom &g,
              const check::Body &b,
              const check::Field &f,
              const check::Ask &policy,
              const Settings &s,
              const std::vector<Handle> &handles,
              const kine::Joints &seed,
              std::vector<kine::Vec3> &scratch);

}  // namespace rrt

#endif  // N_CHECK_RRT_GRASP_H
