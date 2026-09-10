// Copyright by BeeX [2026]

#ifndef N_TASK_PARAMS_H
#define N_TASK_PARAMS_H

#include <n_check/Grasp.h>
#include <n_conf/Doc.h>

#include <cmath>

namespace task {

// ─────────────────────────────────────────────────────────────────────────────
// EVERY TUNABLE STRATEGY HAS. Filled by load() from the `task` and
// `world` sections of n_conf/config/arm.yaml.
//
// Everything about *which hold to take* lives in `ask`, because that is what
// n_check needs and passing it whole means there is no second copy to keep in
// step. Only what n_task itself does with the answer is separate.
// ─────────────────────────────────────────────────────────────────────────────
struct Params {
    // Standoff, approach window, depth range, floor and leg sampling.
    check::Ask ask;

    // Ticks per second.
    double rate_hz = kine::NONE;

    // Run the whole pick unattended, or stop after each leg for the operator.
    bool auto_sequence = false;

    // A leg that has not finished by now has gone wrong; give up on it.
    double leg_timeout_s = kine::NONE;

    // Fills every field from the `task` and `world` sections.
    void load(conf::Doc &doc) {
        ask.load(doc);
        rate_hz       = doc.num("task.rate_hz");
        auto_sequence = doc.flag("task.auto_sequence");
        leg_timeout_s = doc.num("task.leg_timeout_s");
    }

    // Name of the first field still unset, or NULL when all of them are filled.
    const char *missing() const {
        const char *from_ask = ask.missing();
        if (from_ask != NULL) {
            return from_ask;
        }
        if (!std::isfinite(rate_hz)) {
            return "task.rate_hz";
        }
        return std::isfinite(leg_timeout_s) ? NULL : "task.leg_timeout_s";
    }
};

}  // namespace task

#endif  // N_TASK_PARAMS_H
