// Copyright by BeeX [2026]

#ifndef N_CTRL_TRAIL_H
#define N_CTRL_TRAIL_H

#include <n_ctrl/Path.h>

namespace ctrl {

class Trail {
public:
    void start(const kine::Joints &at);
    void add(const Path &leg);
    void stoppedAfter(size_t issued, const kine::Joints &at);
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

}

#endif
