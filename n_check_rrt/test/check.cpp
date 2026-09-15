// Copyright by BeeX [2026]

#include <n_check/Body.h>
#include <n_check/Field.h>
#include <n_check_rrt/Grasp.h>
#include <n_check_rrt/Rrt.h>
#include <n_kine/Angle.h>
#include <n_kine/Fk.h>

#include <cmath>
#include <cstdio>
#include <random>
#include <string>
#include <vector>

static int failures = 0;

static void ok(bool pass, const char *what) {
    std::printf("%-58s %s\n", what, pass ? "ok" : "FAILED");
    failures += pass ? 0 : 1;
}

static bool pathSound(const kine::Geom &g, const check::Body &body, const check::Field &f,
                      const rrt::Settings &s, double floor_z,
                      const kine::Joints &from, const rrt::Path &path) {
    std::vector<kine::Vec3> buf;
    check::Body::Volume     v;
    kine::Joints            prev = from;
    for (size_t i = 0; i < path.size(); ++i) {
        for (int j = 0; j < kine::DOF; ++j) {
            if (path[i][j] < g.windowLo(j) || path[i][j] > g.windowHi(j)) return false;
            if (kine::rad2deg(std::fabs(path[i][j] - prev[j])) > s.check_step_deg + 1e-9) return false;
        }
        body.volume(g, path[i], buf, v);
        if (check::lowestZ(v) < floor_z) return false;
        if (f.ok() && check::firstBlocked(f, v, false) >= 0) return false;
        prev = path[i];
    }
    return true;
}

int main(int argc, char **argv) {
    const char *arm_path = argc > 1 ? argv[1] : "../n_conf/config/arm.yaml";
    const char *rrt_path = argc > 2 ? argv[2] : "config/rrt.yaml";

    conf::Doc arm_doc, rrt_doc;
    arm_doc.load(arm_path, {"arm", "jaws"}, {"driver", "ctrl", "task", "world"});
    rrt_doc.load(rrt_path, {"rrt"});

    kine::Params  arm;
    check::Jaws   jaws;
    check::Ask    ask;
    rrt::Settings s;
    arm.load(arm_doc);
    jaws.load(arm_doc);
    ask.load(arm_doc);
    s.load(rrt_doc);
    ok(arm_doc.problems().empty() && rrt_doc.ok(), "both configs load");
    ok(s.missing() == NULL, "every rrt setting is filled");
    if (!arm_doc.problems().empty() || !rrt_doc.ok() || s.missing() != NULL) {
        std::printf("%s%s", arm_doc.report().c_str(), rrt_doc.report().c_str());
        return 1;
    }

    const kine::Geom g(arm);
    ok(g.ok(), "the geometry is solvable");

    check::Field f;
    std::string  err;
    if (argc > 3) {
        ok(f.load(argv[3], err), "the field loads");
    }
    const check::Body body(jaws, check::Jaws::bladePitch(f.ok() ? f.res() : 0.005));

    const double       rest_wire[kine::DOF] = {0.0, 90.0, 1.0, 1.0};
    const kine::Joints rest = kine::toKinematic(g.params(), rest_wire);

    std::vector<kine::Vec3> scratch;
    rrt::Path               path;
    rrt::Stats              st;
    const check::Field      none;

    kine::Joints goal = rest;
    goal[kine::BASE] += 0.8;
    goal[kine::ELBOW] += 0.6;
    rrt::Result r = rrt::plan(g, body, none, s, ask.floor_z_m, rest, goal, scratch, path, st);
    ok(r == rrt::Result::OK && st.direct, "an open joint move comes back as a straight line");
    ok(!path.empty() && path.back() == goal, "the path ends on the goal");
    ok(pathSound(g, body, none, s, ask.floor_z_m, rest, path),
       "every waypoint is in the window, clear and one step apart");

    kine::Joints outside = rest;
    outside[kine::SHOULDER] = g.windowHi(kine::SHOULDER) + 0.1;
    ok(rrt::plan(g, body, none, s, ask.floor_z_m, rest, outside, scratch, path, st)
               == rrt::Result::GOAL_OUTSIDE,
       "a goal past a joint limit is refused");

    std::vector<rrt::Handle> handles(1);
    handles[0].point = {0.15, 0.0, 0.10};
    handles[0].axis  = {0.0, 1.0, 0.0};
    const rrt::Choice c = rrt::choose(g, body, none, ask, s, handles, rest, scratch);
    std::printf("  hold at (0.15, 0, 0.10) with no field: %s\n",
                c.found ? "yes" : check::name(c.block));
    ok(c.found && !c.path.empty() && c.path.back() == c.hold.joints,
       "the chosen path ends on the hold");
    if (c.found) {
        ok(kine::norm(kine::forward(g, c.path.back()).throat - c.hold.point) < 1e-9,
           "the throat lands on the handle");
    }

    if (f.ok()) {
        std::mt19937_64                        rng(7);
        std::uniform_real_distribution<double> u(0.0, 1.0);
        int tried = 0, planned = 0, direct = 0, sound = 0;
        double worst_s = 0.0;
        while (tried < 20) {
            kine::Joints q;
            for (int j = 0; j < kine::DOF; ++j) {
                q[j] = g.windowLo(j) + (g.windowHi(j) - g.windowLo(j)) * u(rng);
            }
            r = rrt::plan(g, body, f, s, ask.floor_z_m, rest, q, scratch, path, st);
            if (r == rrt::Result::GOAL_FLOOR || r == rrt::Result::GOAL_OBSTACLE) {
                continue;
            }
            ++tried;
            worst_s = std::max(worst_s, st.total_s);
            if (r != rrt::Result::OK) {
                continue;
            }
            ++planned;
            direct += st.direct ? 1 : 0;
            sound += pathSound(g, body, f, s, ask.floor_z_m, rest, path) ? 1 : 0;
        }
        std::printf("  %d random clear goals: %d planned (%d direct), slowest %.3f s\n",
                    tried, planned, direct, worst_s);
        ok(sound == planned, "every planned path re-checks clear against the field");
    }

    std::printf("\n%s\n", failures == 0 ? "ALL OK" : "SOME CHECKS FAILED");
    return failures == 0 ? 0 : 1;
}
