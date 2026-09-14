// Copyright by BeeX [2026]

#include <n_ctrl/Path.h>
#include <n_kine/Angle.h>
#include <n_task/Pick.h>

#include <cmath>
#include <cstdio>
#include <limits>

namespace task {
namespace {

constexpr double kUnitEps = 1e-9;

}  // namespace

bool readCandidates(const std::vector<float> &data,
                    std::vector<Candidate> &out,
                    std::string &why) {
    out.clear();

    char buf[192];
    if (data.empty()) {
        why = "no candidates given";
        return false;
    }

    why.clear();

    // 9 wins when both fit, since a 9-stride list is the more specific reading,
    // but say so: read the wrong way the points are silently nonsense.
    const size_t stride = data.size() % 9 == 0 ? 9 : 6;
    if (stride == 9 && data.size() % 6 == 0 && data.size() >= 12) {
        std::snprintf(buf, sizeof(buf),
                      "%u values is both %u candidates of 9 (point, axis, approach) and %u of 6 "
                      "(point, axis). Reading it as 9. Send a count that is not a multiple of 18 "
                      "to mean 6.",
                      static_cast<uint32_t>(data.size()), static_cast<uint32_t>(data.size() / 9),
                      static_cast<uint32_t>(data.size() / 6));
        why = buf;
    }
    if (data.size() % stride != 0) {
        std::snprintf(buf, sizeof(buf),
                      "%u values is neither a whole number of 6-value candidates (point, axis) "
                      "nor of 9 (point, axis, approach)",
                      static_cast<uint32_t>(data.size()));
        why = buf;
        return false;
    }
    if (data.size() / stride < 2) {
        why = "a handle needs at least two candidates; one point is not a handle";
        return false;
    }

    const size_t n = data.size() / stride;
    out.reserve(n);
    for (size_t i = 0; i < n; ++i) {
        const size_t b = i * stride;

        Candidate c;
        c.point = {data[b + 0], data[b + 1], data[b + 2]};
        c.axis  = {data[b + 3], data[b + 4], data[b + 5]};
        if (stride == 9) {
            c.approach = {data[b + 6], data[b + 7], data[b + 8]};
        }

        if (kine::norm(c.axis) < kUnitEps) {
            std::snprintf(buf, sizeof(buf), "candidate %u has a zero-length handle axis",
                          static_cast<uint32_t>(i));
            why = buf;
            out.clear();
            return false;
        }
        out.push_back(c);
    }
    return true;
}

check::Block drivable(const kine::Geom &g,
                      const check::Body &b,
                      const check::Field &f,
                      const ctrl::Params &motion,
                      const kine::Joints &from,
                      const check::Hold &h,
                      std::vector<kine::Vec3> &scratch) {
    std::string why;

    ctrl::Leg leg;
    leg.start    = h.standoff_point;
    leg.target   = h.point;
    leg.q_wrist  = h.joints[kine::WRIST];
    leg.elbow_up = h.elbow_up;

    ctrl::Path line;
    double     dev = 0.0;
    if (ctrl::planLine(g, motion, h.standoff, leg, line, dev) != ctrl::Status::OK) {
        return check::Block::NO_LINE;
    }
    switch (ctrl::admit(g, motion, line, f, b, scratch, why)) {
    case ctrl::Status::OK:
        break;
    case ctrl::Status::FLOOR:
        return check::Block::FLOOR;
    case ctrl::Status::OBSTACLE:
        return check::Block::OBSTACLE;
    default:
        return check::Block::NO_LINE;
    }

    ctrl::Path route;
    ctrl::planJoint(motion, from, h.standoff, route);
    if (ctrl::admit(g, motion, route, f, b, scratch, why) != ctrl::Status::OK) {
        return check::Block::NO_ROUTE;
    }
    return check::Block::NONE;
}

Choice choose(const kine::Geom &g,
              const check::Body &b,
              const check::Field &f,
              const check::Ask &policy,
              const ctrl::Params &motion,
              const std::vector<Candidate> &candidates,
              const kine::Joints &seed,
              std::vector<kine::Vec3> &scratch) {

    Choice out;
    out.per.assign(candidates.size(), check::Block::UNREACHABLE);
    out.travel.assign(candidates.size(), std::numeric_limits<double>::quiet_NaN());

    const kine::Joints      from = ctrl::snapToWindow(g, seed, motion.goal_tolerance_deg);
    std::vector<kine::Vec3> leg_scratch;
    const check::Drive      drive = [&](const check::Hold &h) {
        return drivable(g, b, f, motion, from, h, leg_scratch);
    };

    for (size_t i = 0; i < candidates.size(); ++i) {
        check::Ask ask = policy;
        ask.point      = candidates[i].point;
        ask.axis       = candidates[i].axis;
        ask.approach   = candidates[i].approach;

        const check::Hold h = check::holdable(g, b, f, ask, seed, scratch, drive);
        out.per[i]          = h.block;

        if (h.ok()) {
            out.travel[i] = h.travel;
        }
        if (!h.ok()) {
            out.block = check::worse(out.block, h.block);
            continue;
        }

        // Least travel to the standoff wins, same rule as within one candidate.
        if (!out.found || h.travel < out.hold.travel) {
            out.hold  = h;
            out.index = i;
            out.found = true;
        }
    }

    if (out.found) {
        out.block = check::Block::NONE;
    }
    return out;
}

std::vector<size_t> graspable(const std::vector<check::Block> &per) {
    std::vector<size_t> ids;
    for (size_t i = 0; i < per.size(); ++i) {
        if (per[i] == check::Block::NONE) {
            ids.push_back(i);
        }
    }
    return ids;
}

std::string ranges(const std::vector<size_t> &ids) {
    std::string out;
    char        buf[48];

    for (size_t i = 0; i < ids.size();) {
        size_t j = i;
        while (j + 1 < ids.size() && ids[j + 1] == ids[j] + 1) {
            ++j;
        }
        if (i == j) {
            std::snprintf(buf, sizeof(buf), "%u", static_cast<uint32_t>(ids[i]));
        } else {
            std::snprintf(buf, sizeof(buf), "%u-%u", static_cast<uint32_t>(ids[i]),
                          static_cast<uint32_t>(ids[j]));
        }
        out += out.empty() ? buf : std::string(", ") + buf;
        i = j + 1;
    }
    return out.empty() ? "none" : out;
}

std::string summarise(const Choice &c, const std::vector<Candidate> &candidates) {
    char        buf[320];
    std::string out;

    const std::vector<size_t> held = graspable(c.per);
    std::snprintf(buf, sizeof(buf), "%u of %u candidates can be held (%s).",
                  static_cast<uint32_t>(held.size()), static_cast<uint32_t>(candidates.size()),
                  ranges(held).c_str());
    out = buf;

    if (!c.found) {
        std::snprintf(buf, sizeof(buf), " Nothing was chosen: %s. Refused: %s.",
                      check::reason(c.block), tally(c.per).c_str());
        return out + buf;
    }

    // The spread matters as much as the winner. With no obstacle field and a
    // wide approach window most candidates hold, and a narrow spread means the
    // choice was near arbitrary rather than considered.
    double lo = c.hold.travel;
    double hi = c.hold.travel;
    for (size_t i = 0; i < c.travel.size(); ++i) {
        if (!std::isfinite(c.travel[i])) {
            continue;
        }
        lo = std::min(lo, c.travel[i]);
        hi = std::max(hi, c.travel[i]);
    }

    std::snprintf(buf, sizeof(buf),
                  " Chose %u at (%.3f, %.3f, %.3f), least travel %.3f of %.3f..%.3f.",
                  static_cast<uint32_t>(c.index), c.hold.point.x, c.hold.point.y, c.hold.point.z,
                  c.hold.travel, lo, hi);
    out += buf;

    std::snprintf(buf, sizeof(buf),
                  " Approach (%.3f, %.3f, %.3f), %.1f deg off the one asked for; depth %.3f m; "
                  "wrist %.1f deg from its stop.",
                  c.hold.approach.x, c.hold.approach.y, c.hold.approach.z,
                  kine::rad2deg(c.hold.approach_dev_rad), c.hold.depth_m,
                  kine::rad2deg(c.hold.wrist_margin_rad));
    out += buf;

    if (held.size() < candidates.size()) {
        std::snprintf(buf, sizeof(buf), " Refused: %s.", tally(c.per).c_str());
        out += buf;
    }
    return out;
}

std::string tally(const std::vector<check::Block> &per) {
    const int kBlocks = static_cast<int>(check::Block::NO_ROUTE) + 1;
    std::vector<int> count(kBlocks, 0);
    for (size_t i = 0; i < per.size(); ++i) {
        const int b = static_cast<int>(per[i]);
        if (b > 0 && b < kBlocks) {
            ++count[b];
        }
    }

    std::string out;
    char        buf[64];
    for (int b = 1; b < kBlocks; ++b) {
        if (count[b] == 0) {
            continue;
        }
        std::snprintf(buf, sizeof(buf), "%d %s", count[b],
                      check::name(static_cast<check::Block>(b)));
        out += out.empty() ? buf : std::string(", ") + buf;
    }
    return out.empty() ? "nothing" : out;
}

}  // namespace task
