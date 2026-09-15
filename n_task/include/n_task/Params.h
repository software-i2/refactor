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
    double ctrl_silence_s = kine::NONE;

    double jaw_held_mm  = kine::NONE;
    double jaw_still_mm = kine::NONE;
    double jaw_settle_s = kine::NONE;

    double jaw_open_mm     = kine::NONE;
    double jaw_open_tol_mm = kine::NONE;
    double jaw_timeout_s   = kine::NONE;

    void load(conf::Doc &doc) {
        ask.load(doc);
        rate_hz         = doc.num("task.rate_hz");
        auto_sequence   = doc.flag("task.auto_sequence");
        ctrl_silence_s  = doc.num("task.ctrl_silence_s");
        jaw_held_mm     = doc.num("task.jaw_held_mm");
        jaw_still_mm    = doc.num("task.jaw_still_mm");
        jaw_settle_s    = doc.num("task.jaw_settle_s");
        jaw_open_mm     = doc.num("jaws.open_mm");
        jaw_open_tol_mm = doc.num("task.jaw_open_tol_mm");
        jaw_timeout_s   = doc.num("task.jaw_timeout_s");
    }

    const char *missing() const {
        const char *from_ask = ask.missing();
        if (from_ask != NULL) {
            return from_ask;
        }
        if (!std::isfinite(rate_hz)) {
            return "task.rate_hz";
        }
        if (!std::isfinite(ctrl_silence_s)) {
            return "task.ctrl_silence_s";
        }
        if (!std::isfinite(jaw_held_mm) || jaw_held_mm <= 0.0) {
            return "task.jaw_held_mm";
        }
        if (!std::isfinite(jaw_still_mm) || jaw_still_mm <= 0.0) {
            return "task.jaw_still_mm";
        }
        if (!std::isfinite(jaw_settle_s) || jaw_settle_s <= 0.0) {
            return "task.jaw_settle_s";
        }
        if (!std::isfinite(jaw_open_mm) || jaw_open_mm <= 0.0) {
            return "jaws.open_mm";
        }
        if (!std::isfinite(jaw_open_tol_mm) || jaw_open_tol_mm <= 0.0) {
            return "task.jaw_open_tol_mm";
        }
        return std::isfinite(jaw_timeout_s) && jaw_timeout_s > jaw_settle_s ? NULL
                                                                           : "task.jaw_timeout_s";
    }
};

}  // namespace task

#endif  // N_TASK_PARAMS_H
