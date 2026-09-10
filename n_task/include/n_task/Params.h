// Copyright by BeeX [2026]

#ifndef N_TASK_PARAMS_H
#define N_TASK_PARAMS_H

#include <n_check/Grasp.h>
#include <n_conf/Doc.h>

#include <cmath>

namespace task {

struct Params {
    check::Ask ask;

    double rate_hz = kine::NONE;
    bool auto_sequence = false;
    double leg_timeout_s = kine::NONE;

    void load(conf::Doc &doc) {
        ask.load(doc);
        rate_hz       = doc.num("task.rate_hz");
        auto_sequence = doc.flag("task.auto_sequence");
        leg_timeout_s = doc.num("task.leg_timeout_s");
    }

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
