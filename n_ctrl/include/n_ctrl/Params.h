// Copyright by BeeX [2026]

#ifndef N_CTRL_PARAMS_H
#define N_CTRL_PARAMS_H

#include <n_conf/Doc.h>
#include <n_kine/Geom.h>

namespace ctrl {

struct Params {
    double rate_hz = kine::NONE;

    double max_joint_step_deg = kine::NONE;
    double line_step_m        = kine::NONE;

    double goal_tolerance_deg = kine::NONE;

    double feedback_timeout_s = kine::NONE;
    double arrival_timeout_s  = kine::NONE;

    double floor_z_m = kine::NONE;

    bool   pillow_stop         = false;
    double pillow_min_step_deg = kine::NONE;
    double pillow_follow_frac  = kine::NONE;
    int    pillow_strikes      = -1;

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

    const char *missing() const;
};

}

#endif
