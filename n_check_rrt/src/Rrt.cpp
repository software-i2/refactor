// Copyright by BeeX [2026]

#include <n_check_rrt/Rrt.h>
#include <n_kine/Angle.h>
#include <n_kine/Fk.h>
#include <n_kine/Ik.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <random>
#include <utility>

namespace rrt {
namespace {

using Clock = std::chrono::steady_clock;

constexpr double kReached   = 1e-9;
constexpr int    kMaxDense  = 100000;

enum Verdict { CLEAR = 0, OUTSIDE, FLOOR, OBSTACLE };

struct Node {
    kine::Joints     q;
    int              parent = -1;
    double           cost   = 0.0;
    std::vector<int> kids;
};

using Tree = std::vector<Node>;

class Planner {
public:
    Planner(const kine::Geom &g, const check::Body &b, const check::Field &f, const Settings &s,
            double floor_z_m, std::vector<kine::Vec3> &scratch, Stats &stats)
        : g_(g), b_(b), f_(f), s_(s), floor_(floor_z_m), scratch_(scratch),
          stats_(stats), rng_(static_cast<uint64_t>(s.seed)) {}

    Verdict verdict(const kine::Joints &q) {
        ++stats_.poses;
        for (int j = 0; j < kine::DOF; ++j) {
            if (q[j] < g_.windowLo(j) || q[j] > g_.windowHi(j)) {
                return OUTSIDE;
            }
        }
        b_.volume(g_, q, scratch_, vol_);
        if (check::lowestZ(vol_) < floor_) {
            return FLOOR;
        }
        if (f_.ok() && check::firstBlocked(f_, vol_, false) >= 0) {
            return OBSTACLE;
        }
        return CLEAR;
    }

    bool clear(const kine::Joints &a, const kine::Joints &b) {
        const int n = samples(a, b);
        int       top = 1;
        while (top * 2 < n) {
            top *= 2;
        }
        for (int stride = top; stride >= 1; stride /= 2) {
            for (int i = stride; i < n; i += 2 * stride) {
                if (verdict(lerp(a, b, static_cast<double>(i) / n)) != CLEAR) {
                    return false;
                }
            }
        }
        return true;
    }

    int samples(const kine::Joints &a, const kine::Joints &b) const {
        double worst = 0.0;
        for (int j = 0; j < kine::DOF; ++j) {
            worst = std::max(worst, kine::rad2deg(std::fabs(b[j] - a[j])));
        }
        const double n = std::ceil(worst / s_.check_step_deg);
        return n < 1.0 ? 1 : (n > kMaxDense ? kMaxDense : static_cast<int>(n));
    }

    static kine::Joints lerp(const kine::Joints &a, const kine::Joints &b, double u) {
        kine::Joints q;
        for (int j = 0; j < kine::DOF; ++j) {
            q[j] = a[j] + (b[j] - a[j]) * u;
        }
        return q;
    }

    double dist(const kine::Joints &a, const kine::Joints &b) const { return distance(g_, a, b); }

    kine::Joints sample() {
        std::uniform_real_distribution<double> u(0.0, 1.0);
        kine::Joints                           q;
        for (int j = 0; j < kine::DOF; ++j) {
            q[j] = g_.windowLo(j) + (g_.windowHi(j) - g_.windowLo(j)) * u(rng_);
        }
        return q;
    }

    std::mt19937_64 &rng() { return rng_; }

    int extend(Tree &t, const kine::Joints &target) {
        int    nearest = 0;
        double best    = dist(t[0].q, target);
        for (size_t i = 1; i < t.size(); ++i) {
            const double d = dist(t[i].q, target);
            if (d < best) {
                best    = d;
                nearest = static_cast<int>(i);
            }
        }

        const kine::Joints q = best <= s_.extend_rad
                                       ? target
                                       : lerp(t[nearest].q, target, s_.extend_rad / best);
        if (best < kReached || verdict(q) != CLEAR) {
            return -1;
        }

        const double n      = static_cast<double>(t.size() + 1);
        const double radius = std::min(s_.extend_rad,
                                       s_.rewire_gamma * std::pow(std::log(n) / n, 0.25));

        std::vector<std::pair<double, int> > near;
        for (size_t i = 0; i < t.size(); ++i) {
            const double d = dist(t[i].q, q);
            if (d <= radius || static_cast<int>(i) == nearest) {
                near.push_back(std::make_pair(t[i].cost + d, static_cast<int>(i)));
            }
        }
        std::sort(near.begin(), near.end());

        int parent = -1;
        for (size_t k = 0; k < near.size(); ++k) {
            if (clear(t[near[k].second].q, q)) {
                parent = near[k].second;
                break;
            }
        }
        if (parent < 0) {
            return -1;
        }

        const int idx = static_cast<int>(t.size());
        t.push_back(Node());
        t[idx].q      = q;
        t[idx].parent = parent;
        t[idx].cost   = t[parent].cost + dist(t[parent].q, q);
        t[parent].kids.push_back(idx);

        for (size_t k = 0; k < near.size(); ++k) {
            const int m = near[k].second;
            if (m == parent) {
                continue;
            }
            const double c = t[idx].cost + dist(q, t[m].q);
            if (c + kReached < t[m].cost && clear(q, t[m].q)) {
                reparent(t, m, idx, c);
            }
        }
        return idx;
    }

    int connect(Tree &t, const kine::Joints &target) {
        for (;;) {
            const int idx = extend(t, target);
            if (idx < 0) {
                return -1;
            }
            if (dist(t[idx].q, target) < kReached) {
                return idx;
            }
        }
    }

private:
    static void reparent(Tree &t, int m, int parent, double cost) {
        std::vector<int> &old = t[t[m].parent].kids;
        old.erase(std::remove(old.begin(), old.end(), m), old.end());
        t[m].parent = parent;
        t[parent].kids.push_back(m);

        const double     delta = cost - t[m].cost;
        std::vector<int> stack(1, m);
        while (!stack.empty()) {
            const int n = stack.back();
            stack.pop_back();
            t[n].cost += delta;
            stack.insert(stack.end(), t[n].kids.begin(), t[n].kids.end());
        }
    }

    const kine::Geom        &g_;
    const check::Body       &b_;
    const check::Field      &f_;
    const Settings          &s_;
    double                   floor_;
    std::vector<kine::Vec3> &scratch_;
    Stats                   &stats_;
    std::mt19937_64          rng_;
    check::Body::Volume      vol_;
};

double seconds(const Clock::time_point &since) {
    return std::chrono::duration<double>(Clock::now() - since).count();
}

void densify(Planner &p, const Path &corners, Path &out) {
    out.clear();
    for (size_t k = 0; k + 1 < corners.size(); ++k) {
        const int n = p.samples(corners[k], corners[k + 1]);
        for (int i = 1; i <= n; ++i) {
            out.push_back(Planner::lerp(corners[k], corners[k + 1], static_cast<double>(i) / n));
        }
    }
}

void finish(const kine::Geom &g, Planner &p, const Path &corners, Path &out, Stats &stats) {
    stats.corners = corners.size();
    stats.cost    = 0.0;
    stats.travel  = 0.0;
    for (size_t k = 0; k + 1 < corners.size(); ++k) {
        stats.cost += distance(g, corners[k], corners[k + 1]);
        stats.travel += kine::travel(g, corners[k], corners[k + 1]);
    }
    densify(p, corners, out);
}

}  // namespace

const char *Settings::missing() const {
    if (iterations < 1) return "rrt.iterations";
    if (refine_iterations < 0) return "rrt.refine_iterations";
    if (!(budget_s > 0.0)) return "rrt.budget_s";
    if (!(extend_rad > 0.0)) return "rrt.extend_rad";
    if (!(rewire_gamma > 0.0)) return "rrt.rewire_gamma";
    if (!(check_step_deg > 0.0)) return "rrt.check_step_deg";
    if (!(grip_zone_m >= 0.0)) return "rrt.grip_zone_m";
    if (shortcut_tries < 0) return "rrt.shortcut_tries";
    return seed >= 0 ? NULL : "rrt.seed";
}

const char *name(Result r) {
    switch (r) {
    case Result::OK:
        return "ok";
    case Result::START_OUTSIDE:
        return "start_outside";
    case Result::GOAL_OUTSIDE:
        return "goal_outside";
    case Result::GOAL_FLOOR:
        return "goal_floor";
    case Result::GOAL_OBSTACLE:
        return "goal_obstacle";
    default:
        return "no_route";
    }
}

double distance(const kine::Geom &g, const kine::Joints &a, const kine::Joints &b) {
    double sum = 0.0;
    for (int j = 0; j < kine::DOF; ++j) {
        const double d = g.params().weight[j] * (b[j] - a[j]);
        sum += d * d;
    }
    return std::sqrt(sum);
}

Result plan(const kine::Geom &g,
            const check::Body &b,
            const check::Field &f,
            const Settings &s,
            double floor_z_m,
            const kine::Joints &from,
            const kine::Joints &to,
            std::vector<kine::Vec3> &scratch,
            Path &out,
            Stats &stats) {

    const Clock::time_point t0 = Clock::now();
    stats = Stats();
    out.clear();

    Planner p(g, b, f, s, floor_z_m, scratch, stats);

    for (int j = 0; j < kine::DOF; ++j) {
        if (from[j] < g.windowLo(j) || from[j] > g.windowHi(j)) {
            return Result::START_OUTSIDE;
        }
    }
    switch (p.verdict(to)) {
    case OUTSIDE:
        return Result::GOAL_OUTSIDE;
    case FLOOR:
        return Result::GOAL_FLOOR;
    case OBSTACLE:
        return Result::GOAL_OBSTACLE;
    default:
        break;
    }

    if (p.clear(from, to)) {
        stats.direct  = true;
        stats.first_s = seconds(t0);
        finish(g, p, Path{from, to}, out, stats);
        stats.total_s = seconds(t0);
        return Result::OK;
    }

    Tree ta(1), tb(1);
    ta[0].q = from;
    tb[0].q = to;
    Tree *grow  = &ta;
    Tree *other = &tb;

    std::vector<std::pair<int, int> > joins;
    int                               found_at = -1;

    int it = 0;
    for (; it < s.iterations; ++it) {
        if (seconds(t0) > s.budget_s) {
            break;
        }
        if (found_at >= 0 && it - found_at >= s.refine_iterations) {
            break;
        }

        const int na = p.extend(*grow, p.sample());
        if (na >= 0) {
            const int nb = p.connect(*other, (*grow)[na].q);
            if (nb >= 0) {
                joins.push_back(grow == &ta ? std::make_pair(na, nb) : std::make_pair(nb, na));
                if (found_at < 0) {
                    found_at      = it;
                    stats.first_s = seconds(t0);
                }
            }
        }
        std::swap(grow, other);
    }
    stats.iterations = it;
    stats.nodes      = ta.size() + tb.size();

    if (joins.empty()) {
        stats.total_s = seconds(t0);
        return Result::NO_ROUTE;
    }

    size_t best = 0;
    for (size_t k = 1; k < joins.size(); ++k) {
        if (ta[joins[k].first].cost + tb[joins[k].second].cost
            < ta[joins[best].first].cost + tb[joins[best].second].cost) {
            best = k;
        }
    }

    Path corners;
    for (int n = joins[best].first; n >= 0; n = ta[n].parent) {
        corners.push_back(ta[n].q);
    }
    std::reverse(corners.begin(), corners.end());
    for (int n = tb[joins[best].second].parent; n >= 0; n = tb[n].parent) {
        corners.push_back(tb[n].q);
    }

    for (int k = 0; k < s.shortcut_tries && corners.size() > 2; ++k) {
        std::uniform_int_distribution<size_t> pick(0, corners.size() - 1);
        size_t i = pick(p.rng());
        size_t j = pick(p.rng());
        if (i > j) {
            std::swap(i, j);
        }
        if (j - i > 1 && p.clear(corners[i], corners[j])) {
            corners.erase(corners.begin() + i + 1, corners.begin() + j);
        }
    }

    finish(g, p, corners, out, stats);
    stats.total_s = seconds(t0);
    return Result::OK;
}

}  // namespace rrt
