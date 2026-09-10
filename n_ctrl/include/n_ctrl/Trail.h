// Copyright by BeeX [2026]

#ifndef N_CTRL_TRAIL_H
#define N_CTRL_TRAIL_H

#include <n_ctrl/Path.h>

namespace ctrl {

// Breadcrumbs of where the arm has been driven, so it can back out the way it
// came. The old driver spread this over four members and seven call sites; the
// rules live here instead.
class Trail {
public:
    // Begin a fresh outbound run from where the arm is now.
    void start(const kine::Joints &at);

    // Append a leg that is about to run. Ignored unless started.
    void add(const Path &leg);

    // The arm stopped part way through the leg that is running: drop what it
    // never travelled, then pin the pose it actually stopped at.
    void stoppedAfter(size_t issued, const kine::Joints &at);

    // The way home: everywhere it has been, newest first.
    Path back() const;

    void   clear();
    bool   recording() const { return recording_; }
    size_t size() const { return crumbs_.size(); }
    bool   empty() const { return crumbs_.empty(); }

private:
    Path   crumbs_;
    size_t leg_start_ = 0;
    bool   recording_ = false;
};

}  // namespace ctrl

#endif  // N_CTRL_TRAIL_H
