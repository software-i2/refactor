// Copyright by BeeX [2026]

#ifndef N_CTRL_EXEC_H
#define N_CTRL_EXEC_H

#include <n_ctrl/Params.h>
#include <n_ctrl/Path.h>

#include <cstdint>

namespace ctrl {

enum class State : uint8_t {
    IDLE        = 0,
    APPROACHING = 1,  // still issuing waypoints
    SETTLING    = 2,  // all issued, waiting for the arm to catch up
    REACHED     = 3,
    STALLED     = 4,  // did not arrive before arrival_timeout_s
    PILLOW      = 5,  // a joint stopped following while still being driven
    ABORTED     = 6
};

inline const char *name(State s) {
    switch (s) {
    case State::IDLE:
        return "idle";
    case State::APPROACHING:
        return "approaching";
    case State::SETTLING:
        return "settling";
    case State::REACHED:
        return "reached";
    case State::STALLED:
        return "stalled";
    case State::PILLOW:
        return "pillow";
    default:
        return "aborted";
    }
}

// Where waypoints go. The node's implementation publishes them; the tests use
// one that just records.
class Sink {
public:
    virtual ~Sink() = default;
    virtual void send(const kine::Joints &q) = 0;
    virtual void release() = 0;  // hand the arm back to standby
};

// Plays a path back one waypoint per tick and watches whether the arm follows.
class Exec {
public:
    Exec(const Params &p, Sink &sink) : p_(p), sink_(sink) {}

    bool load(const Path &path);
    void abort();

    // Latest measurement; call before tick.
    void measure(const kine::Joints &q);
    // Nothing answered this cycle, so following cannot be judged.
    void blind();

    State tick(double now_s);

    State state() const { return state_; }
    bool  busy() const { return state_ == State::APPROACHING || state_ == State::SETTLING; }
    size_t issued() const { return next_; }
    int   pillowJoint() const { return pillow_joint_; }

private:
    bool arrived() const;
    int  notFollowing() const;

    Params p_;
    Sink  &sink_;

    Path   path_;
    size_t next_  = 0;
    State  state_ = State::IDLE;
    double settle_since_s_ = 0.0;

    kine::Joints now_{};
    kine::Joints prev_{};
    bool         have_now_  = false;
    bool         have_prev_ = false;

    int strikes_      = 0;
    int pillow_joint_ = -1;
};

}  // namespace ctrl

#endif  // N_CTRL_EXEC_H
