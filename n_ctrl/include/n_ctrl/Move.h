// Copyright by BeeX [2026]

#ifndef N_CTRL_MOVE_H
#define N_CTRL_MOVE_H

#include <n_kine/Vec3.h>

#include <string>

namespace ctrl {

enum class Status {
    OK,
    UNREACHABLE,
    LIMIT,
    FLOOR,
    OBSTACLE,
    NOT_STRAIGHT,
    EMPTY,
    BAD_STATE
};

const char *reason(Status s);

struct Move {
    Status      status = Status::BAD_STATE;
    std::string note;
    kine::Vec3  from;
    kine::Vec3  to;
    size_t      waypoints  = 0;
    double      duration_s = 0.0;

    bool ok() const { return status == Status::OK; }
};

}

#endif
