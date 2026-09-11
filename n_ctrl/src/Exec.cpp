// Copyright by BeeX [2026]

#include <n_ctrl/Exec.h>
#include <n_kine/Angle.h>

#include <algorithm>
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

// Stale feedback must not count as an arrival, so the current reading is dropped
// too, not just the one the pillow stop compares against.
void Exec::blind() {
    have_now_  = false;
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

int Exec::notClosing() const {
    if (!p_.pillow_stop || !have_prev_ || !have_now_ || path_.empty()) {
        return -1;
    }
    const kine::Joints &goal = path_.back();
    for (int j = 0; j < kine::DOF; ++j) {
        const double was  = kine::rad2deg(std::fabs(goal[j] - prev_[j]));
        const double owed = std::min(was - p_.goal_tolerance_deg, p_.max_joint_step_deg);
        if (owed < p_.pillow_min_step_deg) {
            continue;
        }
        const double closed = was - kine::rad2deg(std::fabs(goal[j] - now_[j]));
        if (closed < p_.pillow_follow_frac * owed) {
            return j;
        }
    }
    return -1;
}

bool Exec::strike(int held) {
    if (held >= 0 && ++strikes_ >= p_.pillow_strikes) {
        pillow_joint_ = held;
        state_        = State::PILLOW;
        sink_.release();
        return true;
    }
    if (held < 0) {
        strikes_ = 0;
    }
    prev_      = now_;
    have_prev_ = have_now_;
    return false;
}

State Exec::tick(double now_s) {
    if (state_ == State::APPROACHING) {
        if (strike(notFollowing())) {
            return state_;
        }

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
    if (strike(notClosing())) {
        return state_;
    }
    if (now_s - settle_since_s_ > p_.arrival_timeout_s) {
        state_ = State::STALLED;
        sink_.release();
    }
    return state_;
}

}