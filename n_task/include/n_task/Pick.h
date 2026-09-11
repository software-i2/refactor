// Copyright by BeeX [2026]

#ifndef N_TASK_PICK_H
#define N_TASK_PICK_H

#include <n_check/Grasp.h>
#include <n_ctrl/Params.h>

#include <string>
#include <vector>

namespace task {

// One place the jaws could take hold, as perception offers it.
struct Candidate {
    kine::Vec3 point;
    kine::Vec3 axis;
    kine::Vec3 approach;  // zero length when the caller has no preference
};

// 6 floats per candidate (point, axis) or 9 (point, axis, approach), at least
// two. `why` is set on failure, and on success when the count is ambiguous --
// 18 floats is both 3 of 6 and 2 of 9, and guessing silently grasps thin air.
bool readCandidates(const std::vector<float> &data,
                    std::vector<Candidate> &out,
                    std::string &why);

// The best hold across every candidate, and why each of the rest was refused.
struct Choice {
    check::Hold              hold;
    size_t                   index = 0;      // which candidate won
    bool                     found = false;
    check::Block             block = check::Block::UNREACHABLE;  // deepest refusal seen
    std::vector<check::Block> per;           // one per candidate, in order

    // What each candidate cost, NaN where it was refused. Kept so the choice can
    // show its working: with no obstacle field and a wide approach window most
    // candidates hold, and then the spread is the only thing separating them.
    std::vector<double> travel;
};

// `policy` carries the standoff, approach window, depth range, floor and leg
// sampling; each candidate supplies only the handle itself.
Choice choose(const kine::Geom &g,
              const check::Body &b,
              const check::Field &f,
              const check::Ask &policy,
              const ctrl::Params &motion,
              const std::vector<Candidate> &candidates,
              const kine::Joints &seed,
              std::vector<kine::Vec3> &scratch);

check::Block drivable(const kine::Geom &g,
                      const check::Body &b,
                      const check::Field &f,
                      const ctrl::Params &motion,
                      const kine::Joints &from,
                      const check::Hold &h,
                      std::vector<kine::Vec3> &scratch);

// "3 unreachable, 1 no_clocking" — what stopped the ones that failed.
std::string tally(const std::vector<check::Block> &per);

// Indices of the candidates that could be held, in order.
std::vector<size_t> graspable(const std::vector<check::Block> &per);

// "0-5, 8, 11" — runs of consecutive indices, so a long arc reads at a glance.
std::string ranges(const std::vector<size_t> &ids);

// The whole evaluation as prose: how many held, which ones, what the winner cost
// against the field of the others, and what stopped the rest. Nothing moves --
// this is what you read before deciding to.
std::string summarise(const Choice &c, const std::vector<Candidate> &candidates);

}  // namespace task

#endif  // N_TASK_PICK_H
