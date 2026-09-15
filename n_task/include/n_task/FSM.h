// Copyright by BeeX [2026]

#ifndef N_TASK_FSM_H
#define N_TASK_FSM_H

#include <n_ctrl/Exec.h>
#include <n_task/Params.h>

#include <cstdint>
#include <string>

namespace task {

enum class Step : uint8_t {
    IDLE = 0,
    PLANNING,
    CHOSEN,
    OPEN_JAW,
    MOVE_TO_STANDOFF,
    AT_STANDOFF,
    MOVE_TO_HANDLE,
    AT_HANDLE,
    CLOSE_JAW,
    CHECK_GRASP,
    HOLDING,
    MOVE_TO_BASE,
    SUCCESS,
    FAILED,
    E_STOP
};

enum class Grip : uint8_t {
    NONE = 0,
    CLOSING,
    HELD,
    EMPTY
};

enum class Command : uint8_t { PLAN, START, ADVANCE, CLOSE, OPEN, HOME, STOP };

enum class Action : uint8_t { NONE, MOVE_TO_STANDOFF, MOVE_TO_HANDLE, CLOSE_JAW, HOME };

const char *name(Step s);
bool        busy(Step s);

struct Sense {
    double now_s     = 0.0;
    double entered_s = 0.0;

    ctrl::State ctrl      = ctrl::State::IDLE;
    bool        ctrl_seen = false;
    bool        ctrl_busy = false;
    double      ctrl_at_s = 0.0;

    double jaw_mm    = 0.0;
    bool   jaw_fresh = false;
    bool   jaw_still = false;
    Grip   grip      = Grip::NONE;

    bool carrying      = false;
    bool auto_sequence = false;
};

struct Next {
    Step        step   = Step::IDLE;
    Action      action = Action::NONE;
    std::string why;
};

Next next(Step s, const Sense &in, const Params &p);

bool accepts(Step s, Command c, std::string &why);
Step afterCommand(Step s, Command c);
Step afterPlan(Step from, bool planned);

}  // namespace task

#endif  // N_TASK_FSM_H
