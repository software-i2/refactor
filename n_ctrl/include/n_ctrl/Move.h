// Copyright by BeeX [2026]

#ifndef N_CTRL_MOVE_H
#define N_CTRL_MOVE_H

#include <n_kine/Vec3.h>

#include <string>

namespace ctrl {

enum class Status {
    OK,
    UNREACHABLE,   // no posture puts the throat there
    LIMIT,         // reachable, but a joint would have to leave its window
    FLOOR,         // reachable, but the arm would drop through the floor
    OBSTACLE,      // reachable, but the arm would sweep through mapped occupancy
    NOT_STRAIGHT,  // both ends reachable, the line between them is not followable
    EMPTY,         // nothing else to run
    BAD_STATE      // the arm is not where this move assumed it would be
};

const char *reason(Status s);

// Every field means something whatever the outcome.
struct Move {
    Status      status = Status::BAD_STATE;
    std::string note;  // the specific sentence, when there is one
    kine::Vec3  from;
    kine::Vec3  to;
    size_t      waypoints  = 0;
    double      duration_s = 0.0;  // estimate at rate_hz, not a measurement

    bool ok() const { return status == Status::OK; }
};

}  // namespace ctrl

#endif  // N_CTRL_MOVE_H
