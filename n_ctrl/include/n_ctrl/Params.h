// Copyright by BeeX [2026]

#ifndef N_CTRL_PARAMS_H
#define N_CTRL_PARAMS_H

#include <n_conf/Doc.h>
#include <n_kine/Geom.h>

namespace ctrl {

// ─────────────────────────────────────────────────────────────────────────────
// EVERY TUNABLE MOTION HAS. Filled by load() from the `ctrl` and `world`
// sections of n_conf/config/arm.yaml. Nothing here carries
// a default: a field still holding kine::NONE means the load was skipped.
// ─────────────────────────────────────────────────────────────────────────────
struct Params {
    // Waypoints issued per second. With max_joint_step_deg this sets the
    // fastest any joint can travel: 1.0 deg at 5 Hz is 5 deg/s.
    double rate_hz = kine::NONE;

    // Path shape.
    double max_joint_step_deg = kine::NONE;  // biggest joint move in one waypoint
    double line_step_m        = kine::NONE;  // knot spacing on a straight-line move

    // Arrival.
    double goal_tolerance_deg = kine::NONE;

    // joint_states older than this cannot say whether the arm is following, so
    // pillow detection is suspended rather than tripping on stale numbers.
    double feedback_timeout_s = kine::NONE;
    double arrival_timeout_s  = kine::NONE;  // then release to standby, report STALLED

    // No part of the arm may go below this height. Shared with n_task, so it
    // lives in `world` and neither can set it a different way.
    double floor_z_m = kine::NONE;

    // Pillow stop: a joint told to move at least min_step but following less
    // than follow_frac of it, strikes times running, has hit something the
    // obstacle field does not know about.
    bool   pillow_stop         = false;
    double pillow_min_step_deg = kine::NONE;
    double pillow_follow_frac  = kine::NONE;
    int    pillow_strikes      = -1;

    // Fills every field from the `ctrl` and `world` sections.
    void load(conf::Doc &doc) {
        rate_hz            = doc.num("ctrl.rate_hz");
        max_joint_step_deg = doc.num("ctrl.max_joint_step_deg");
        line_step_m        = doc.num("ctrl.line_step_m");
        goal_tolerance_deg = doc.num("ctrl.goal_tolerance_deg");
        feedback_timeout_s = doc.num("ctrl.feedback_timeout_s");
        arrival_timeout_s  = doc.num("ctrl.arrival_timeout_s");

        pillow_stop         = doc.flag("ctrl.pillow_stop");
        pillow_min_step_deg = doc.num("ctrl.pillow_min_step_deg");
        pillow_follow_frac  = doc.num("ctrl.pillow_follow_frac");
        pillow_strikes      = doc.integer("ctrl.pillow_strikes");

        floor_z_m = doc.num("world.floor_z_m");
    }

    // Name of the first field still unset, or NULL when all of them are filled.
    const char *missing() const;
};

}  // namespace ctrl

#endif  // N_CTRL_PARAMS_H
