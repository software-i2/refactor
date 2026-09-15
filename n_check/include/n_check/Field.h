// Copyright by BeeX [2026]

#ifndef N_CHECK_FIELD_H
#define N_CHECK_FIELD_H

#include <n_kine/Vec3.h>

#include <cstdint>
#include <string>
#include <vector>

namespace check {

enum Link : int { UPPER_ARM = 0, FOREARM, WRIST_MOUNT, PALM, JAW, JAW_HANDLE, N_LINKS };

class Field {
public:
    bool load(const std::string &path, std::string &err);
    void clear();

    bool ok() const { return ok_; }
    bool empty() const { return !ok_ || occupied_ == 0; }

    bool blocked(const kine::Vec3 &p, int link) const;
    bool blockedSegment(const kine::Vec3 &a, const kine::Vec3 &b, int link) const;
    bool blockedPoints(const kine::Vec3 *pts, size_t count, int link) const;

    void samples(const kine::Vec3 &a, const kine::Vec3 &b, int link,
                 std::vector<kine::Vec3> &out) const;

    kine::Vec3 lo() const;
    kine::Vec3 hi() const;
    kine::Vec3 placement() const;
    kine::Vec3 placementRpy() const;

    double   res() const { return res_; }
    double   step() const { return step_; }
    double   radius(int link) const;
    uint64_t occupied() const { return occupied_; }
    uint64_t digest() const { return digest_; }
    size_t   bytes() const { return data_.size(); }
    const std::string &source() const { return source_; }

private:
    size_t at(int i, int j, int k) const {
        return (static_cast<size_t>(i) * dims_[1] + j) * dims_[2] + k;
    }

    bool walk(const kine::Vec3 &a, const kine::Vec3 &b, int link,
              std::vector<kine::Vec3> *out) const;

    bool     ok_   = false;
    double   res_  = 0.0;
    double   step_ = 0.01;
    double   lo_[3] = {0.0, 0.0, 0.0};
    int32_t  dims_[3] = {0, 0, 0};
    double   radii_[N_LINKS] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    double   placement_[3] = {0.0, 0.0, 0.0};
    double   rpy_[3]       = {0.0, 0.0, 0.0};
    uint64_t digest_   = 0;
    uint64_t occupied_ = 0;

    std::vector<uint8_t> data_;
    std::string          source_;
};

}  // namespace check

#endif  // N_CHECK_FIELD_H
