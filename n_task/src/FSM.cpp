// Copyright by BeeX [2026]

#include <n_task/FSM.h>

#include <cstdio>

namespace task {
namespace {

enum class Leg { RUNNING, REACHED, BROKEN };

Leg watchLeg(const Sense &in, const Params &p, std::string &why) {
    char buf[200];
    if (in.ctrl_seen && in.now_s - in.ctrl_at_s > p.ctrl_silence_s) {
        std::snprintf(buf, sizeof(buf), "n_ctrl has not reported for %.1f s; giving up on the leg",
                      p.ctrl_silence_s);
        why = buf;
        return Leg::BROKEN;
    }

    // ctrl/state is only published from n_ctrl's tick, so until it says it is
    // moving, what it reports still describes the leg before this one.
    if (!in.ctrl_seen || !in.ctrl_busy) {
        if (in.now_s - in.entered_s > p.ctrl_silence_s) {
            why = std::string("n_ctrl never took the leg; it still reports ")
                  + (in.ctrl_seen ? ctrl::name(in.ctrl) : "nothing on ctrl/state");
            return Leg::BROKEN;
        }
        return Leg::RUNNING;
    }

    // n_ctrl owns whether a leg finished; this only reacts to what it reports.
    if (in.ctrl == ctrl::State::STALLED || in.ctrl == ctrl::State::PILLOW
        || in.ctrl == ctrl::State::ABORTED || in.ctrl == ctrl::State::IDLE) {
        why = std::string("the arm stopped mid-leg (") + ctrl::name(in.ctrl)
              + "), so the pick is off";
        return Leg::BROKEN;
    }
    return in.ctrl == ctrl::State::REACHED ? Leg::REACHED : Leg::RUNNING;
}

}  // namespace

const char *name(Step s) {
    switch (s) {
    case Step::IDLE:
        return "IDLE";
    case Step::PLANNING:
        return "PLANNING";
    case Step::CHOSEN:
        return "CHOSEN";
    case Step::OPEN_JAW:
        return "OPEN_JAW";
    case Step::MOVE_TO_STANDOFF:
        return "MOVE_TO_STANDOFF";
    case Step::AT_STANDOFF:
        return "AT_STANDOFF";
    case Step::MOVE_TO_HANDLE:
        return "MOVE_TO_HANDLE";
    case Step::AT_HANDLE:
        return "AT_HANDLE";
    case Step::CLOSE_JAW:
        return "CLOSE_JAW";
    case Step::CHECK_GRASP:
        return "CHECK_GRASP";
    case Step::HOLDING:
        return "HOLDING";
    case Step::MOVE_TO_BASE:
        return "MOVE_TO_BASE";
    case Step::SUCCESS:
        return "SUCCESS";
    case Step::FAILED:
        return "FAILED";
    case Step::E_STOP:
        return "E_STOP";
    }
    return "UNKNOWN";
}

bool busy(Step s) {
    return s == Step::PLANNING || s == Step::OPEN_JAW || s == Step::MOVE_TO_STANDOFF
           || s == Step::MOVE_TO_HANDLE || s == Step::CLOSE_JAW || s == Step::CHECK_GRASP
           || s == Step::MOVE_TO_BASE;
}

Next next(Step s, const Sense &in, const Params &p) {
    Next n;
    n.step = s;

    const double in_step = in.now_s - in.entered_s;
    char         buf[200];

    switch (s) {
    case Step::OPEN_JAW:
        if (in.jaw_fresh && in.jaw_mm >= p.jaw_open_mm - p.jaw_open_tol_mm) {
            n.step   = Step::MOVE_TO_STANDOFF;
            n.action = Action::MOVE_TO_STANDOFF;
        } else if (in_step > p.jaw_timeout_s) {
            std::snprintf(buf, sizeof(buf),
                          "the jaw did not open to %.2f mm within %.1f s; it reads %.2f mm%s",
                          p.jaw_open_mm, p.jaw_timeout_s, in.jaw_mm,
                          in.jaw_fresh ? "" : ", from before the command");
            n.step = Step::FAILED;
            n.why  = buf;
        }
        return n;

    case Step::MOVE_TO_STANDOFF:
    case Step::MOVE_TO_HANDLE:
    case Step::MOVE_TO_BASE: {
        const Leg leg = watchLeg(in, p, n.why);
        if (leg == Leg::BROKEN) {
            n.step = Step::FAILED;
            if (s != Step::MOVE_TO_BASE) {
                n.why += ". The outbound path is kept: task/home will back out.";
            }
        } else if (leg == Leg::REACHED) {
            n.step = s == Step::MOVE_TO_STANDOFF ? Step::AT_STANDOFF
                     : s == Step::MOVE_TO_HANDLE ? Step::AT_HANDLE
                     : in.carrying               ? Step::SUCCESS
                                                 : Step::IDLE;
        }
        return n;
    }

    case Step::AT_STANDOFF:
        if (in.auto_sequence) {
            n.step   = Step::MOVE_TO_HANDLE;
            n.action = Action::MOVE_TO_HANDLE;
        }
        return n;

    case Step::AT_HANDLE:
        if (in.auto_sequence) {
            n.step   = Step::CLOSE_JAW;
            n.action = Action::CLOSE_JAW;
        }
        return n;

    case Step::CLOSE_JAW:
        if (in.grip == Grip::HELD || in.grip == Grip::EMPTY || in.jaw_still) {
            n.step = Step::CHECK_GRASP;
        } else if (in_step > p.jaw_timeout_s) {
            std::snprintf(buf, sizeof(buf),
                          "the jaw has not settled %.1f s after the close; holding without a "
                          "grip verdict", p.jaw_timeout_s);
            n.step = Step::HOLDING;
            n.why  = buf;
        }
        return n;

    case Step::CHECK_GRASP:
        if (in.grip == Grip::HELD || in.grip == Grip::EMPTY) {
            n.step = Step::HOLDING;
        } else if (in_step > p.jaw_timeout_s) {
            std::snprintf(buf, sizeof(buf),
                          "no grip verdict %.1f s into the check; holding without one",
                          p.jaw_timeout_s);
            n.step = Step::HOLDING;
            n.why  = buf;
        }
        return n;

    case Step::HOLDING:
        if (in.auto_sequence) {
            n.step   = Step::MOVE_TO_BASE;
            n.action = Action::HOME;
        }
        return n;

    default:
        return n;
    }
}

bool accepts(Step s, Command c, std::string &why) {
    const std::string at = std::string("the task is in ") + name(s);
    switch (c) {
    case Command::STOP:
        return true;

    case Command::PLAN:
        if (s == Step::IDLE || s == Step::CHOSEN || s == Step::SUCCESS || s == Step::FAILED
            || s == Step::E_STOP) {
            return true;
        }
        why = at + "; task/home or task/stop before planning again";
        return false;

    case Command::START:
        if (s == Step::CHOSEN) {
            return true;
        }
        why = busy(s) || s == Step::AT_STANDOFF || s == Step::AT_HANDLE || s == Step::HOLDING
                      ? at + "; start needs a fresh plan"
                      : "nothing planned; call task/plan first";
        return false;

    case Command::ADVANCE:
        if (s == Step::AT_STANDOFF) {
            return true;
        }
        why = at + "; advance needs AT_STANDOFF";
        return false;

    case Command::CLOSE:
        if (!busy(s) && s != Step::AT_STANDOFF) {
            return true;
        }
        why = at + (s == Step::AT_STANDOFF ? "; closing here shuts the jaw before the handle"
                                           : "; wait for it to finish, or task/stop");
        return false;

    case Command::OPEN:
    case Command::HOME:
        if (!busy(s)) {
            return true;
        }
        why = at + "; wait for it to finish, or task/stop";
        return false;
    }
    return false;
}

Step afterCommand(Step s, Command c) {
    switch (c) {
    case Command::PLAN:
        return Step::PLANNING;
    case Command::START:
        return Step::OPEN_JAW;
    case Command::ADVANCE:
        return Step::MOVE_TO_HANDLE;
    case Command::CLOSE:
        return s == Step::AT_HANDLE || s == Step::HOLDING ? Step::CLOSE_JAW : s;
    case Command::OPEN:
        return s == Step::HOLDING ? Step::AT_HANDLE : s;
    case Command::HOME:
        return Step::MOVE_TO_BASE;
    default:
        return Step::E_STOP;
    }
}

Step afterPlan(Step from, bool planned) {
    if (planned) {
        return Step::CHOSEN;
    }
    return from == Step::CHOSEN ? Step::IDLE : from;
}

}  // namespace task
