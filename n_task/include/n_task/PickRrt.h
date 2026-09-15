// Copyright by BeeX [2026]

#ifndef N_TASK_PICK_RRT_H
#define N_TASK_PICK_RRT_H

#include <n_check_rrt/Rrt.h>
#include <n_task/Pick.h>

#include <vector>

namespace task {

check::Block drivableRrt(const kine::Geom &g,
                         const check::Body &b,
                         const check::Field &f,
                         const ctrl::Params &motion,
                         const rrt::Settings &settings,
                         const kine::Joints &from,
                         const check::Hold &h,
                         std::vector<kine::Vec3> &scratch);

Choice chooseRrt(const kine::Geom &g,
                 const check::Body &b,
                 const check::Field &f,
                 const check::Ask &policy,
                 const ctrl::Params &motion,
                 const rrt::Settings &settings,
                 const std::vector<Candidate> &candidates,
                 const kine::Joints &seed,
                 std::vector<kine::Vec3> &scratch);

}  // namespace task

#endif  // N_TASK_PICK_RRT_H
