#include <n_check/Body.h>
#include <n_check/Grasp.h>
#include <n_ctrl/Exec.h>
#include <n_ctrl/Path.h>
#include <n_ctrl/Trail.h>
#include <n_kine/Angle.h>
#include <n_kine/Fk.h>

#include <cstdio>
#include <string>
#include <vector>

using namespace ctrl;

static int failures = 0;

static bool loadConfig(const char *path, conf::Doc &doc,
                       const std::vector<std::string> &own,
                       const std::vector<std::string> &borrow) {
    doc.load(path, own, borrow);
    if (!doc.problems().empty()) {
        std::printf("cannot read %s\n%s", path, doc.report().c_str());
        return false;
    }
    std::printf("  config %s\n", path);
    return true;
}


static void expect(bool pass, const char *what) {
    std::printf("%-52s %s\n", what, pass ? "ok" : "FAILED");
    failures += pass ? 0 : 1;
}

class FakeArm : public Sink {
public:
    void send(const kine::Joints &q) override {
        ++sent;
        for (int j = 0; j < kine::DOF; ++j) {
            if (j != stuck) {
                at[j] = q[j];
            }
        }
    }
    void release() override { ++released; }

    kine::Joints at{};
    int          stuck    = -1;
    int          sent     = 0;
    int          released = 0;
};

int main(int argc, char **argv) {
    conf::Doc doc;
    if (!loadConfig(argc > 1 ? argv[1] : "../n_conf/config/arm.yaml", doc,
                    {"ctrl"}, {"arm", "jaws", "world", "driver", "task"})) {
        return 1;
    }

    Params       p;
    kine::Params geom;
    check::Jaws  jaws;
    p.load(doc);
    geom.load(doc);
    jaws.load(doc);
    expect(doc.ok(), "the config fills the arm, the jaws and motion");
    if (!doc.ok()) {
        std::printf("%s", doc.report().c_str());
        return 1;
    }
    expect(p.missing() == NULL, "every motion tunable is a usable value");
    if (p.missing() != NULL) {
        std::printf("  -> %s\n", p.missing());
        return 1;
    }

    const kine::Geom g(geom);
    expect(g.ok(), "the loaded geometry is solvable");
    if (!g.ok()) {
        std::printf("  -> %s\n", g.fault());
        return 1;
    }

    const double rest_wire[kine::DOF] = {0.0, 90.0, 1.0, 1.0};
    const kine::Joints rest = kine::toKinematic(g.params(), rest_wire);

    const kine::Vec3 target{0.15, 0.05, 0.10};
    kine::Joints     goal;

    solveTarget(g, rest, target, goal);

    Path path;
    planJoint(p, rest, goal, path);

    double worst_step = 0.0;
    for (size_t i = 1; i < path.size(); ++i) {
        for (int j = 0; j < kine::DOF; ++j) {
            worst_step = std::max(worst_step, kine::rad2deg(std::fabs(path[i][j] - path[i - 1][j])));
        }
    }
    std::printf("  %zu waypoints, biggest joint step %.3f deg (limit %.2f)\n",
                path.size(), worst_step, p.max_joint_step_deg);
    expect(worst_step <= p.max_joint_step_deg + 1e-9, "no waypoint exceeds max_joint_step_deg");

    check::Field no_field;
    const check::Body       body(jaws, 0.0);
    std::vector<kine::Vec3> scratch;
    std::string             why;
    expect(admit(g, p, path, no_field, body, false, scratch, why) == Status::OK, "the path is admitted");

    Path   line;
    double dev = 0.0;
    const Status ls = planLine(g, p, rest, target, line, dev);
    std::printf("  line: %zu waypoints, strays %.2e m off it\n", line.size(), dev);
    expect(ls == Status::OK, "a straight line to the same point plans");
    expect(dev <= p.line_step_m, "the throat stays within line_step_m of the line");
    expect(line.size() > path.size(), "the line costs more waypoints than the joint move");

    kine::Joints ignored;
    expect(solveTarget(g, rest, {5.0, 0.0, 0.1}, ignored) == Status::UNREACHABLE,
          "a far target is UNREACHABLE");

    Params floored = p;
    floored.floor_z_m = 1.0;
    expect(admit(g, floored, path, no_field, body, false, scratch, why) == Status::FLOOR,
          "a path under the floor is refused");

    FakeArm arm;
    arm.at = rest;
    Exec    exec(p, arm);

    exec.load(path);
    double t = 0.0;
    for (size_t i = 0; i < path.size() + 2 && exec.busy(); ++i) {
        exec.measure(arm.at);
        exec.tick(t);
        t += 1.0 / p.rate_hz;
    }
    exec.measure(arm.at);
    const State done = exec.tick(t);
    std::printf("  executed %d waypoints, ended %s\n", arm.sent, name(done));
    expect(arm.sent == static_cast<int>(path.size()), "every waypoint is issued exactly once");
    expect(done == State::REACHED, "a followed path reaches");

    FakeArm stuck_arm;
    stuck_arm.at    = rest;
    stuck_arm.stuck = kine::SHOULDER;
    Exec pillow(p, stuck_arm);
    pillow.load(path);
    t = 0.0;
    for (int i = 0; i < 40 && pillow.busy(); ++i) {
        pillow.measure(stuck_arm.at);
        pillow.tick(t);
        t += 1.0 / p.rate_hz;
    }
    std::printf("  seized shoulder: %s after %d waypoints, joint %d\n",
                name(pillow.state()), stuck_arm.sent, pillow.pillowJoint());
    expect(pillow.state() == State::PILLOW, "a joint that stops following trips the pillow stop");
    expect(pillow.pillowJoint() == kine::SHOULDER, "the pillow stop names the seized joint");
    expect(stuck_arm.released == 1, "a pillow stop releases the arm to standby");

    FakeArm frozen;
    frozen.at = rest;
    Exec stall(p, frozen);
    Path one;
    one.push_back(goal);
    stall.load(one);
    stall.measure(rest);
    stall.tick(0.0);
    stall.measure(rest);
    const State late = stall.tick(p.arrival_timeout_s + 1.0);
    expect(late == State::STALLED, "not arriving before the timeout STALLS");
    expect(frozen.released == 1, "a stall releases the arm to standby");

    int mover = 0;
    for (int j = 1; j < kine::DOF; ++j) {
        if (std::fabs(goal[j] - rest[j]) > std::fabs(goal[mover] - rest[mover])) {
            mover = j;
        }
    }
    FakeArm late_arm;
    late_arm.at = rest;
    Exec jam(p, late_arm);
    jam.load(path);
    t = 0.0;
    double all_sent = -1.0;
    for (int i = 0; i < 400 && jam.busy(); ++i) {
        if (late_arm.sent + 3 >= static_cast<int>(path.size())) {
            late_arm.stuck = mover;
        }
        if (all_sent < 0.0 && late_arm.sent == static_cast<int>(path.size())) {
            all_sent = t;
        }
        jam.measure(late_arm.at);
        jam.tick(t);
        t += 1.0 / p.rate_hz;
    }
    std::printf("  joint %d seized 3 waypoints from the end: %s %.1f s after the last was sent\n",
                mover, name(jam.state()), t - all_sent);
    expect(jam.state() == State::PILLOW, "a joint that seizes at the end trips while settling");
    expect(all_sent >= 0.0 && t - all_sent < p.arrival_timeout_s,
           "and trips long before the arrival timeout");

    Path outside;
    outside.push_back(goal);
    outside[0][kine::BASE] = g.windowHi(kine::BASE) + 0.1;
    expect(admit(g, floored, outside, no_field, body, false, scratch, why) == Status::LIMIT,
           "a waypoint outside the window is refused as LIMIT before the floor is asked");

    kine::Joints edge = rest;
    edge[kine::BASE]  = g.windowLo(kine::BASE) - kine::deg2rad(0.05);
    expect(snapToWindow(g, edge, p.goal_tolerance_deg)[kine::BASE] == g.windowLo(kine::BASE),
           "a reading a hair past the window snaps onto its edge");
    edge[kine::BASE] = g.windowLo(kine::BASE) - kine::deg2rad(5.0);
    expect(snapToWindow(g, edge, p.goal_tolerance_deg)[kine::BASE] == edge[kine::BASE],
           "a reading well past the window is left for admit to refuse");

    Trail trail;
    trail.start(rest);
    trail.add(path);
    expect(trail.size() == path.size() + 1, "the trail records the start plus the leg");
    expect(kine::norm(kine::forward(g, trail.back().back()).throat
                     - kine::forward(g, rest).throat) < 1e-9,
          "the way back ends where the arm set out");

    trail.stoppedAfter(3, rest);
    expect(trail.size() == 5, "a stop part way drops what was never travelled");

    Trail fresh;
    fresh.add(path);
    expect(fresh.empty(), "a trail that was never started records nothing");

    if (argc > 2) {
        check::Field real;
        std::string  err;
        if (real.load(argv[2], err)) {
            const check::Body real_body(jaws, check::Jaws::bladePitch(real.res()));
            const Status s = admit(g, p, path, real, real_body, false, scratch, why);
            std::printf("  with %s: %s\n", argv[2], why.empty() ? "admitted" : why.c_str());
            expect(s != Status::OBSTACLE || why.find("waypoint") != std::string::npos,
                   "an obstacle refusal names the part and the waypoint");
        } else {
            std::printf("  could not load %s: %s\n", argv[2], err.c_str());
        }
    }

    {
        std::printf("\n  -- the pick, run leg by leg --\n");

        check::Ask ask;
        ask.load(doc);
        ask.point = {0.15, 0.0, 0.10};
        ask.axis  = {0.0, 1.0, 0.0};

        check::Field            none;
        std::vector<kine::Vec3> grasp_scratch;
        const check::Hold h = check::holdable(g, body, none, ask, rest, grasp_scratch);
        expect(h.ok(), "the gate finds a hold to run");

        if (h.ok()) {
            FakeArm arm;
            arm.at = rest;
            Exec    exec(p, arm);
            double  t = 0.0;

            Path leg1;
            planJoint(p, rest, h.standoff, leg1);
            expect(admit(g, p, leg1, none, body, false, scratch, why) == Status::OK,
                   "the standoff leg is admitted");
            exec.load(leg1);
            while (exec.busy()) { exec.measure(arm.at); exec.tick(t); t += 1.0 / p.rate_hz; }
            exec.measure(arm.at);
            exec.tick(t);

            int    worst_j = 0;
            double worst   = 0.0;
            for (int j = 0; j < kine::DOF; ++j) {
                const double d = kine::rad2deg(std::fabs(arm.at[j] - h.standoff[j]));
                if (d > worst) { worst = d; worst_j = j; }
            }
            std::printf("  after leg 1 the arm is %.2e deg off the solved standoff (joint %d)\n",
                        worst, worst_j);
            expect(worst < 1e-9, "leg 1 lands on the standoff posture exactly");

            Leg leg;
            leg.start    = h.standoff_point;
            leg.target   = h.point;
            leg.q_wrist  = h.joints[kine::WRIST];
            leg.elbow_up = h.elbow_up;

            Leg         wired;
            std::string decode_why;
            expect(decodeLeg(encodeLeg(leg), wired, decode_why), "the leg survives the wire");
            expect(!decodeLeg(std::vector<float>{0.15f, 0.0f, 0.10f}, wired, decode_why),
                   "a bare point is refused as a grasp leg");
            std::printf("  -> %s\n", decode_why.c_str());

            Path   leg2;
            double dev = 0.0;
            const Status s2 = planLine(g, p, arm.at, wired, leg2, dev);
            expect(s2 == Status::OK, "the advance leg plans");
            expect(admit(g, p, leg2, none, body, true, scratch, why) == Status::OK,
                   "the advance leg is admitted");
            std::printf("  advance: %zu waypoints, strays %.2e m off the approach line\n",
                        leg2.size(), dev);

            exec.load(leg2);
            while (exec.busy()) { exec.measure(arm.at); exec.tick(t); t += 1.0 / p.rate_hz; }
            exec.measure(arm.at);
            exec.tick(t);

            worst = 0.0;
            for (int j = 0; j < kine::DOF; ++j) {
                worst = std::max(worst, kine::rad2deg(std::fabs(arm.at[j] - h.joints[j])));
            }
            std::printf("  after leg 2 the arm is %.2e deg off the solved hold\n", worst);
            expect(worst < 1e-3, "leg 2 lands on the hold posture");

            const double square = kine::rad2deg(
                    kine::angleTo(kine::frame(g, arm.at).hinge, ask.axis));
            std::printf("  jaw hinge sits %.2f deg from the handle axis\n", square);
            expect(square < 1e-6 || std::fabs(square - 180.0) < 1e-6,
                   "the jaws arrive square to the handle");

            kine::Joints naive_standoff, naive_hold;
            expect(solveTarget(g, rest, h.standoff_point, naive_standoff) == Status::OK
                           && solveTarget(g, naive_standoff, h.point, naive_hold) == Status::OK,
                   "the old point-only chain still solves");
            const double lost =
                    kine::rad2deg(std::fabs(naive_hold[kine::WRIST] - h.joints[kine::WRIST]));
            const double crooked =
                    kine::rad2deg(kine::angleTo(kine::frame(g, naive_hold).hinge, ask.axis));
            std::printf("  point-only would leave the wrist %.2f deg out, hinge %.2f deg "
                        "from the axis\n", lost, crooked);
            expect(lost > 1.0, "a point cannot carry the roll -- which is why Leg exists");
        }
    }

    std::printf("\n%s\n", failures == 0 ? "ALL OK" : "SOME CHECKS FAILED");
    return failures == 0 ? 0 : 1;
}
