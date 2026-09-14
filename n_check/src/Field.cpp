// Copyright by BeeX [2026]

#include <n_check/Field.h>

#include <cstring>
#include <cmath>
#include <cstdio>

namespace check {
namespace {

const char kMagic[8] = {'B', 'X', 'F', 'I', 'E', 'L', 'D', '4'};

template <typename T>
bool readAll(std::FILE *f, T *dst, size_t count) {
    return std::fread(dst, sizeof(T), count, f) == count;
}

}  // namespace

void Field::clear() {
    ok_       = false;
    occupied_ = 0;
    digest_   = 0;
    dims_[0] = dims_[1] = dims_[2] = 0;
    placement_[0] = placement_[1] = placement_[2] = 0.0;
    rpy_[0] = rpy_[1] = rpy_[2] = 0.0;
    data_.clear();
    source_.clear();
}

bool Field::load(const std::string &path, std::string &err) {
    clear();

    std::FILE *f = std::fopen(path.c_str(), "rb");
    if (f == NULL) {
        err = "cannot open " + path;
        return false;
    }

    char    magic[8];
    int32_t nlinks   = 0;
    double  unused   = 0.0;  // was the dilation margin, always 0 since it was dropped

    bool read = readAll(f, magic, 8) && std::memcmp(magic, kMagic, 8) == 0;
    read = read && readAll(f, &res_, 1) && readAll(f, &step_, 1) && readAll(f, &unused, 1);
    read = read && readAll(f, lo_, 3) && readAll(f, dims_, 3) && readAll(f, &nlinks, 1);
    read = read && readAll(f, &digest_, 1) && readAll(f, placement_, 3) && readAll(f, rpy_, 3);
    read = read && readAll(f, radii_, N_LINKS);

    if (!read) {
        std::fclose(f);
        clear();
        err = path + " is not a BXFIELD4 file";
        return false;
    }
    if (nlinks != N_LINKS) {
        char msg[128];
        std::snprintf(msg, sizeof(msg), "%s carries %d links, this build expects %d",
                      path.c_str(), nlinks, static_cast<int>(N_LINKS));
        std::fclose(f);
        clear();
        err = msg;
        return false;
    }
    if (res_ <= 0.0 || step_ <= 0.0 || dims_[0] <= 0 || dims_[1] <= 0 || dims_[2] <= 0) {
        std::fclose(f);
        clear();
        err = path + " has a degenerate grid";
        return false;
    }

    const size_t cells = static_cast<size_t>(dims_[0]) * dims_[1] * dims_[2];
    data_.resize(cells);
    const bool whole = std::fread(&data_[0], 1, cells, f) == cells;
    std::fclose(f);

    if (!whole) {
        clear();
        err = path + " ended before the grid did";
        return false;
    }

    for (size_t n = 0; n < cells; ++n) {
        occupied_ += data_[n] != 0;
    }

    source_ = path;
    ok_     = true;
    return true;
}

double Field::radius(int link) const {
    return link >= 0 && link < N_LINKS ? radii_[link] : 0.0;
}

kine::Vec3 Field::lo() const { return {lo_[0], lo_[1], lo_[2]}; }

kine::Vec3 Field::hi() const {
    return {lo_[0] + dims_[0] * res_, lo_[1] + dims_[1] * res_, lo_[2] + dims_[2] * res_};
}

kine::Vec3 Field::placement() const { return {placement_[0], placement_[1], placement_[2]}; }

kine::Vec3 Field::placementRpy() const { return {rpy_[0], rpy_[1], rpy_[2]}; }

bool Field::blocked(const kine::Vec3 &p, int link) const {
    if (!ok_ || link < 0 || link >= N_LINKS) {
        return false;
    }

    const double q[3] = {p.x, p.y, p.z};
    int          idx[3];
    for (int a = 0; a < 3; ++a) {
        const double cell = std::floor((q[a] - lo_[a]) / res_);
        if (cell < 0.0 || cell >= static_cast<double>(dims_[a])) {
            return false;
        }
        idx[a] = static_cast<int>(cell);
    }
    return (data_[at(idx[0], idx[1], idx[2])] & (1u << link)) != 0;
}

bool Field::walk(const kine::Vec3 &a, const kine::Vec3 &b, int link,
                 std::vector<kine::Vec3> *out) const {
    if (!ok_) {
        return false;
    }

    const kine::Vec3 span = b - a;
    int steps = static_cast<int>(std::ceil(kine::norm(span) / step_));
    if (steps < 1) {
        steps = 1;
    }

    bool hit = false;
    for (int n = 0; n <= steps; ++n) {
        const double     t = static_cast<double>(n) / static_cast<double>(steps);
        const kine::Vec3 p{a.x + span.x * t, a.y + span.y * t, a.z + span.z * t};
        if (!blocked(p, link)) {
            continue;
        }
        hit = true;
        if (out == NULL) {
            return true;
        }
        out->push_back(p);
    }
    return hit;
}

bool Field::blockedSegment(const kine::Vec3 &a, const kine::Vec3 &b, int link) const {
    return walk(a, b, link, NULL);
}

bool Field::blockedPoints(const kine::Vec3 *pts, size_t count, int link) const {
    if (!ok_) {
        return false;
    }
    for (size_t n = 0; n < count; ++n) {
        if (blocked(pts[n], link)) {
            return true;
        }
    }
    return false;
}

void Field::samples(const kine::Vec3 &a, const kine::Vec3 &b, int link,
                    std::vector<kine::Vec3> &out) const {
    out.clear();
    walk(a, b, link, &out);
}

}  // namespace check
