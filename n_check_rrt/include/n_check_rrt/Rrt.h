// Copyright by BeeX [2026]

#ifndef N_CHECK_RRT_RRT_H
#define N_CHECK_RRT_RRT_H

#include <n_check/Body.h>
#include <n_check/Field.h>
#include <n_conf/Doc.h>
#include <n_kine/Geom.h>

#include <vector>

namespace rrt {

using Path = std::vector<kine::Joints>;

struct Settings {
    int    iterations        = -1;
    int    refine_iterations = -1;
    double budget_s          = kine::NONE;
    double extend_rad        = kine::NONE;
    double rewire_gamma      = kine::NONE;
    double check_step_deg    = kine::NONE;
    double grip_zone_m       = kine::NONE;
    int    shortcut_tries    = -1;
    int    seed              = -1;
    bool   standoff          = true;

    void load(conf::Doc &doc) {
        standoff          = doc.flag("rrt.standoff");
        iterations        = doc.integer("rrt.iterations");
        refine_iterations = doc.integer("rrt.refine_iterations");
        budget_s          = doc.num("rrt.budget_s");
        extend_rad        = doc.num("rrt.extend_rad");
        rewire_gamma      = doc.num("rrt.rewire_gamma");
        check_step_deg    = doc.num("rrt.check_step_deg");
        grip_zone_m       = doc.num("rrt.grip_zone_m");
        shortcut_tries    = doc.integer("rrt.shortcut_tries");
        seed              = doc.integer("rrt.seed");
    }

    const char *missing() const;
};

enum class Result { OK, START_OUTSIDE, GOAL_OUTSIDE, GOAL_FLOOR, GOAL_OBSTACLE, NO_ROUTE };

const char *name(Result r);

struct Stats {
    int    iterations = 0;
    size_t nodes      = 0;
    long   poses      = 0;
    size_t corners    = 0;
    bool   direct     = false;
    double first_s    = -1.0;
    double total_s    = 0.0;
    double cost       = 0.0;
    double travel     = 0.0;
};

double distance(const kine::Geom &g, const kine::Joints &a, const kine::Joints &b);

Result plan(const kine::Geom &g,
            const check::Body &b,
            const check::Field &f,
            const Settings &s,
            double floor_z_m,
            const kine::Joints &from,
            const kine::Joints &to,
            std::vector<kine::Vec3> &scratch,
            Path &out,
            Stats &stats);

}  // namespace rrt

#endif  // N_CHECK_RRT_RRT_H
