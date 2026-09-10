// Copyright by BeeX [2026]

#include <n_ctrl/Exec.h>
#include <n_kine/Angle.h>

#include <cmath>

namespace ctrl {

bool Exec::load(const Path &path) {
    if (path.empty()) {
        return false;
    }
    path_         = path;
    next_         = 0;
    strikes_      = 0;
    pillow_joint_ = -1;
    have_prev_    = false;
    state_        = State::APPROACHING;
    return true;
}

void Exec::abort() {
    path_.clear();
    next_  = 0;
    state_ = State::ABORTED;
    sink_.release();
}

void Exec::measure(const kine::Joints &q) {
    now_      = q;
    have_now_ = true;
}

void Exec::blind() {
    have_prev_ = false;
    strikes_   = 0;
}

bool Exec::arrived() const {
    if (path_.empty() || !have_now_) {
        return false;
    }
    const kine::Joints &goal = path_.back();
    for (int j = 0; j < kine::DOF; ++j) {
        if (kine::rad2deg(std::fabs(now_[j] - goal[j])) > p_.goal_tolerance_deg) {
            return false;
        }
    }
    return true;
}

int Exec::notFollowing() const {
    if (!p_.pillow_stop || !have_prev_ || !have_now_ || next_ < 2) {
        return -1;
    }
    for (int j = 0; j < kine::DOF; ++j) {
        const double asked = kine::rad2deg(std::fabs(path_[next_ - 1][j] - path_[next_ - 2][j]));
        if (asked < p_.pillow_min_step_deg) {
            continue;
        }
        const double moved = kine::rad2deg(std::fabs(now_[j] - prev_[j]));
        if (moved < p_.pillow_follow_frac * asked) {
            return j;
        }
    }
    return -1;
}

State Exec::tick(double now_s) {
    if (state_ == State::APPROACHING) {
        const int held = notFollowing();
        if (held >= 0 && ++strikes_ >= p_.pillow_strikes) {
            pillow_joint_ = held;
            state_        = State::PILLOW;
            sink_.release();
            return state_;
        }
        if (held < 0) {
            strikes_ = 0;
        }

        prev_      = now_;
        have_prev_ = have_now_;

        if (next_ < path_.size()) {
            sink_.send(path_[next_]);
            ++next_;
        }
        if (next_ >= path_.size()) {
            settle_since_s_ = now_s;
            state_          = State::SETTLING;
        }
        return state_;
    }

    if (state_ != State::SETTLING) {
        return state_;
    }

    if (arrived()) {
        state_ = State::REACHED;
        return state_;
    }
    if (now_s - settle_since_s_ > p_.arrival_timeout_s) {
        state_ = State::STALLED;
        sink_.release();
    }
    return state_;
}

}