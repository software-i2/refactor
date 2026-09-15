// Copyright by BeeX [2026]
//
// Plain g++, no ROS:
//   g++ -std=c++17 -Iinclude -I../n_kine/include -I../n_check/include -I../n_conf/include
//       test/check.cpp src/Pick.cpp src/FSM.cpp ../n_kine/src/*.cpp ../n_check/src/*.cpp
//       ../n_conf/src/Doc.cpp -lyaml-cpp -o /tmp/check
//
// argv[1] is the config; it defaults to ../n_conf/config/arm.yaml.

#include <n_ctrl/Path.h>
#include <n_kine/Angle.h>
#include <n_kine/Fk.h>
#include <n_task/Params.h>
#include <n_task/Pick.h>
#include <n_task/FSM.h>

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

using namespace task;

static int failures = 0;

// The config is the only source of these numbers, so the test reads the same
// file the nodes do rather than a second copy that could drift from it.
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
    if (what[0] != '\0' || !pass) {
        std::printf("%-52s %s\n", what, pass ? "ok" : "FAILED");
    }
    failures += pass ? 0 : 1;
}

// An arc of candidates along a handle lying across the approach, the way
// perception offers one.
static std::vector<float> arc(int n, float radius, bool with_approach) {
    std::vector<float> d;
    for (int i = 0; i < n; ++i) {
        const float t = static_cast<float>(i) / static_cast<float>(n - 1);
        d.push_back(0.13f + radius * t);
        d.push_back(0.0f);
        d.push_back(0.10f);
        d.push_back(0.0f);
        d.push_back(1.0f);
        d.push_back(0.0f);
        if (with_approach) {
            d.push_back(1.0f);
            d.push_back(0.0f);
            d.push_back(0.0f);
        }
    }
    return d;
}

int main(int argc, char **argv) {
    // ── reading what perception sends ─────────────────────────────────────
    std::vector<Candidate> cs;
    std::string            why;

    expect(!readCandidates({1, 2, 3, 4, 5}, cs, why), "a ragged array is refused");
    std::printf("  -> %s\n", why.c_str());

    expect(readCandidates({0, 0, 0, 0, 1, 0}, cs, why) && cs.size() == 1,
           "a single candidate is accepted");

    // 18 floats is 3 candidates of 6 and 2 of 9. It reads as 9, and says so.
    expect(readCandidates(arc(3, 0.04f, false), cs, why) && cs.size() == 2,
           "an 18-float array reads as two 9-value candidates");
    expect(!why.empty(), "and warns that it could have been read as six");
    std::printf("  -> %s\n", why.c_str());

    std::vector<float> zero_axis = {0, 0, 0, 0, 0, 0, 0.1f, 0, 0.1f, 0, 1, 0};
    expect(!readCandidates(zero_axis, cs, why), "a zero-length axis is refused");
    std::printf("  -> %s\n", why.c_str());

    // ── choosing among them ───────────────────────────────────────────────
    conf::Doc doc;
    if (!loadConfig(argc > 1 ? argv[1] : "../n_conf/config/arm.yaml", doc,
                    {"task"}, {"arm", "jaws", "world", "driver", "ctrl"})) {
        return 1;
    }

    Params       p;
    ctrl::Params motion;
    kine::Params geom;
    check::Jaws  jaws;
    p.load(doc);
    motion.load(doc);
    geom.load(doc);
    jaws.load(doc);
    expect(doc.ok(), "the config fills the arm, the jaws and strategy");
    if (!doc.ok()) {
        std::printf("%s", doc.report().c_str());
        return 1;
    }
    expect(p.missing() == NULL && motion.missing() == NULL,
           "every strategy and motion tunable is a usable value");
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

    // No field, so no blade lattice: there is nothing to sample against.
    const check::Body       body(jaws, 0.0);
    check::Field            none;
    std::vector<kine::Vec3> scratch;

    const double       rest_wire[kine::DOF] = {0.0, 90.0, 1.0, 1.0};
    const kine::Joints rest = kine::toKinematic(g.params(), rest_wire);

    readCandidates(arc(5, 0.05f, false), cs, why);
    Choice c = choose(g, body, none, p.ask, motion, cs, rest, scratch);
    std::printf("  %zu candidates: %s, refused: %s\n", cs.size(),
                c.found ? "held" : check::name(c.block), tally(c.per).c_str());

    // The winner must be the cheapest of those that worked.
    if (c.found) {
        double best = c.hold.travel;
        bool   cheapest = true;
        for (size_t i = 0; i < cs.size(); ++i) {
            if (c.per[i] != check::Block::NONE) {
                continue;
            }
            check::Ask a = p.ask;
            a.point      = cs[i].point;
            a.axis       = cs[i].axis;
            std::vector<kine::Vec3> leg_scratch;
            const check::Hold h = check::holdable(g, body, none, a, rest, scratch,
                    [&](const check::Hold &x) {
                        return drivable(g, body, none, motion, rest, x, leg_scratch);
                    });
            cheapest = cheapest && (!h.ok() || h.travel >= best - 1e-9);
        }
        expect(cheapest, "the cheapest workable candidate is the one chosen");
    }

    // ── refusals report the deepest reason, not the first ─────────────────
    std::vector<float> far;
    for (int i = 0; i < 3; ++i) {
        far.push_back(5.0f + i);
        far.push_back(0.0f);
        far.push_back(0.1f);
        far.push_back(0.0f);
        far.push_back(1.0f);
        far.push_back(0.0f);
    }
    readCandidates(far, cs, why);
    c = choose(g, body, none, p.ask, motion, cs, rest, scratch);
    expect(!c.found && c.block == check::Block::UNREACHABLE,
           "an arc out of reach reports UNREACHABLE");
    std::printf("  out of reach (%zu candidates), refused: %s\n", cs.size(), tally(c.per).c_str());
    expect(tally(c.per) == "2 unreachable", "the tally counts each refusal");

    check::Ask fussy = p.ask;
    fussy.max_approach_dev_deg = 1.0;
    readCandidates(arc(4, 0.04f, true), cs, why);
    for (size_t i = 0; i < cs.size(); ++i) {
        cs[i].approach = {0.0, 0.0, 1.0};  // insist on straight down
    }
    c = choose(g, body, none, fussy, motion, cs, rest, scratch);
    std::printf("  1 deg approach window: %s, refused: %s\n",
                c.found ? "held" : check::name(c.block), tally(c.per).c_str());

    // The reported block must be the deepest any candidate reached, not the
    // first or the most common.
    check::Block deepest = check::Block::UNREACHABLE;
    for (size_t i = 0; i < c.per.size(); ++i) {
        deepest = check::worse(deepest, c.per[i]);
    }
    expect(c.block == deepest, "the reported refusal is the deepest any candidate reached");

    // ── the preview: showing its working ──────────────────────────────────
    {
        std::printf("\n  -- preview --\n");
        readCandidates(arc(8, 0.06f, false), cs, why);
        c = choose(g, body, none, p.ask, motion, cs, rest, scratch);

        for (size_t i = 0; i < cs.size(); ++i) {
            std::printf("  %3u  (%7.3f, %7.3f, %7.3f)  %-12s %s\n",
                        static_cast<uint32_t>(i), cs[i].point.x, cs[i].point.y, cs[i].point.z,
                        check::name(c.per[i]),
                        c.found && i == c.index ? "<- chosen" : "");
        }
        std::printf("  %s\n", summarise(c, cs).c_str());

        const std::vector<size_t> held = graspable(c.per);
        expect(held.size() == cs.size() || !held.empty(), "the preview names which candidates hold");
        expect(!c.found || c.per[c.index] == check::Block::NONE,
               "the chosen candidate is one of the ones that held");

        // With no field and a wide window every candidate holds, so the choice
        // rests on travel alone. The preview has to make that visible.
        expect(!c.found || c.travel[c.index] == c.hold.travel,
               "the winner's travel is reported alongside the rest");
        for (size_t i = 0; i < cs.size(); ++i) {
            expect(!c.found || c.per[i] != check::Block::NONE
                           || c.travel[i] >= c.hold.travel - 1e-9,
                   i == 0 ? "no candidate that held is cheaper than the one chosen" : "");
        }

        expect(ranges({0, 1, 2, 5, 7, 8, 9}) == "0-2, 5, 7-9", "runs of indices collapse");
        expect(ranges({}) == "none", "an empty run reads as none");
    }

    // ── what the gate passes, ctrl drives ─────────────────────────────────
    {
        std::printf("\n  -- gate against the planner --\n");
        readCandidates(arc(8, 0.06f, true), cs, why);
        for (size_t i = 0; i < cs.size(); ++i) {
            cs[i].approach = {0.866, 0.0, -0.5};
        }
        c = choose(g, body, none, p.ask, motion, cs, rest, scratch);
        expect(c.found, "an arc with an approach finds a hold");

        if (c.found) {
            const kine::Vec3 v   = cs[c.index].point - c.hold.point;
            const kine::Vec3 a   = kine::unit(c.hold.approach);
            const double     off = kine::norm(v - a * kine::dot(v, a));
            std::printf("  depth %.3f m, %.1f deg off the asked approach, handle %.2e m off "
                        "the jaw axis\n",
                        c.hold.depth_m, kine::rad2deg(c.hold.approach_dev_rad), off);
            expect(off < 1e-5, "the handle sits on the jaw axis the jaws arrive along");

            ctrl::Leg leg;
            leg.start    = c.hold.standoff_point;
            leg.target   = c.hold.point;
            leg.q_wrist  = c.hold.joints[kine::WRIST];
            leg.elbow_up = c.hold.elbow_up;

            bool        drives = true;
            std::string admit_why;
            for (int corner = 0; corner < (1 << kine::DOF); ++corner) {
                kine::Joints from = c.hold.standoff;
                for (int j = 0; j < kine::DOF; ++j) {
                    const double sign = (corner >> j) & 1 ? 1.0 : -1.0;
                    from[j] += kine::deg2rad(sign * motion.goal_tolerance_deg);
                }
                from = ctrl::snapToWindow(g, from, motion.goal_tolerance_deg);

                ctrl::Path line;
                double     dev = 0.0;
                drives = drives
                         && ctrl::planLine(g, motion, from, leg, line, dev) == ctrl::Status::OK
                         && ctrl::admit(g, motion, line, none, body, false, scratch, admit_why)
                                    == ctrl::Status::OK
                         && kine::norm(kine::forward(g, line.back()).throat - c.hold.point) < 1e-9;
            }
            expect(drives, "the advance drives from every corner of the arrival tolerance");
        }
    }

    {
        std::printf("\n  -- the step machine --\n");
        bool named = std::string(name(Step::E_STOP)) == "E_STOP";
        for (int i = 0; i <= static_cast<int>(Step::E_STOP); ++i) {
            for (int j = 0; j < i; ++j) {
                named = named
                        && std::string(name(static_cast<Step>(i)))
                                   != name(static_cast<Step>(j));
            }
        }
        expect(named, "every step publishes its own name on task/step");
        expect(accepts(Step::IDLE, Command::PLAN, why) && !accepts(Step::IDLE, Command::START, why),
               "a pick starts only from a plan");
        expect(!accepts(Step::MOVE_TO_STANDOFF, Command::PLAN, why),
               "no planning while a leg runs");
        std::printf("  -> %s\n", why.c_str());
        expect(!accepts(Step::HOLDING, Command::START, why),
               "start while holding is refused, so the load is not dropped");
        expect(!accepts(Step::CHECK_GRASP, Command::HOME, why)
                       && accepts(Step::HOLDING, Command::HOME, why),
               "home waits for the grip verdict");
        expect(!accepts(Step::AT_STANDOFF, Command::CLOSE, why)
                       && accepts(Step::AT_HANDLE, Command::CLOSE, why),
               "the jaw closes at the handle, not the standoff");
        expect(accepts(Step::MOVE_TO_HANDLE, Command::STOP, why)
                       && afterCommand(Step::MOVE_TO_HANDLE, Command::STOP) == Step::E_STOP,
               "stop is always taken and lands in E_STOP");
        expect(afterCommand(Step::HOLDING, Command::OPEN) == Step::AT_HANDLE
                       && afterCommand(Step::IDLE, Command::CLOSE) == Step::IDLE,
               "a jaw command moves the step only mid-pick");
        expect(afterPlan(Step::IDLE, true) == Step::CHOSEN
                       && afterPlan(Step::CHOSEN, false) == Step::IDLE
                       && afterPlan(Step::FAILED, false) == Step::FAILED,
               "a failed plan drops an old one and leaves a failure standing");

        Sense in;
        in.ctrl_seen = true;
        in.now_s     = 100.0;
        in.entered_s = 100.0;
        in.ctrl_at_s = 100.0;

        in.jaw_fresh = true;
        in.jaw_mm    = p.jaw_open_mm;
        Next n       = next(Step::OPEN_JAW, in, p);
        expect(n.step == Step::MOVE_TO_STANDOFF && n.action == Action::MOVE_TO_STANDOFF,
               "an open jaw sends the arm to the standoff");
        in.jaw_mm = 1.3;
        in.now_s  = 100.0 + p.jaw_timeout_s + 0.1;
        n         = next(Step::OPEN_JAW, in, p);
        expect(n.step == Step::FAILED, "a jaw that never opens fails the pick");
        std::printf("  -> %s\n", n.why.c_str());

        in.now_s     = 100.0;
        in.ctrl_busy = true;
        in.ctrl      = ctrl::State::APPROACHING;
        expect(next(Step::MOVE_TO_STANDOFF, in, p).step == Step::MOVE_TO_STANDOFF,
               "a running leg is left alone");
        in.ctrl = ctrl::State::REACHED;
        expect(next(Step::MOVE_TO_STANDOFF, in, p).step == Step::AT_STANDOFF,
               "arriving parks at the standoff");
        in.auto_sequence = true;
        n                = next(Step::AT_STANDOFF, in, p);
        expect(n.step == Step::MOVE_TO_HANDLE && n.action == Action::MOVE_TO_HANDLE,
               "auto_sequence moves on from the standoff");
        in.auto_sequence = false;
        expect(next(Step::AT_STANDOFF, in, p).action == Action::NONE,
               "without it the arm waits there");

        in.ctrl = ctrl::State::PILLOW;
        n       = next(Step::MOVE_TO_HANDLE, in, p);
        expect(n.step == Step::FAILED, "a pillow mid-leg fails the pick");
        std::printf("  -> %s\n", n.why.c_str());

        in.ctrl_busy = false;
        in.ctrl      = ctrl::State::REACHED;
        in.now_s     = 100.0 + p.ctrl_silence_s + 0.1;
        in.ctrl_at_s = in.now_s;
        expect(next(Step::MOVE_TO_BASE, in, p).step == Step::FAILED,
               "a leg n_ctrl never takes fails, though it still reports the last arrival");

        in.now_s     = 100.0;
        in.ctrl_at_s = 100.0;
        in.ctrl_busy = true;
        in.carrying  = true;
        expect(next(Step::MOVE_TO_BASE, in, p).step == Step::SUCCESS,
               "home after the grasp is a success");
        in.carrying = false;
        expect(next(Step::MOVE_TO_BASE, in, p).step == Step::IDLE,
               "home from anywhere else is idle");

        in.grip      = Grip::CLOSING;
        in.jaw_still = false;
        expect(next(Step::CLOSE_JAW, in, p).step == Step::CLOSE_JAW, "a moving jaw is still closing");
        in.jaw_still = true;
        expect(next(Step::CLOSE_JAW, in, p).step == Step::CHECK_GRASP, "a jaw that stops is checked");
        in.grip = Grip::EMPTY;
        expect(next(Step::CHECK_GRASP, in, p).step == Step::HOLDING,
               "an empty verdict is recorded, not gated");
        in.grip  = Grip::CLOSING;
        in.now_s = 100.0 + p.jaw_timeout_s + 0.1;
        n        = next(Step::CHECK_GRASP, in, p);
        expect(n.step == Step::HOLDING && !n.why.empty(), "no verdict in time holds anyway, and says so");
        std::printf("  -> %s\n", n.why.c_str());
    }

    std::printf("\n%s\n", failures == 0 ? "ALL OK" : "SOME CHECKS FAILED");
    return failures == 0 ? 0 : 1;
}
