// Copyright by BeeX [2026]

#include <n_ctrl/Trail.h>

namespace ctrl {

void Trail::start(const kine::Joints &at) {
    crumbs_.clear();
    crumbs_.push_back(at);
    leg_start_ = crumbs_.size();
    recording_ = true;
}

void Trail::add(const Path &leg) {
    if (!recording_) {
        return;
    }
    leg_start_ = crumbs_.size();
    crumbs_.insert(crumbs_.end(), leg.begin(), leg.end());
}

void Trail::stoppedAfter(size_t issued, const kine::Joints &at) {
    if (!recording_) {
        return;
    }
    const size_t ran = leg_start_ + issued;
    if (ran < crumbs_.size()) {
        crumbs_.resize(ran);
    }
    crumbs_.push_back(at);
}

Path Trail::back() const { return Path(crumbs_.rbegin(), crumbs_.rend()); }

void Trail::clear() {
    crumbs_.clear();
    leg_start_ = 0;
    recording_ = false;
}

}  // namespace ctrl
