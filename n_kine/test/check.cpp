// Copyright by BeeX [2026]
//
// Plain g++, no ROS:
//   g++ -std=c++17 -Iinclude -I../n_conf/include test/check.cpp src/*.cpp
//       ../n_conf/src/Doc.cpp -lyaml-cpp -o /tmp/check
//
// Pass the config path as argv[1]; it defaults to ../n_conf/config/arm.yaml.

#include <n_kine/Angle.h>
#include <n_kine/Fk.h>
#include <n_kine/Ik.h>

#include <cassert>
#include <cstdio>
#include <string>
#include <vector>

using namespace kine;

static int failures = 0;

// The config is the only source of these numbers, so the test reads the same
// file the nodes do rather than a second copy that could drift from it.
static bool loadConfig(int argc, char **argv, conf::Doc &doc,
                       const std::vector<std::string> &own,
                       const std::vector<std::string> &borrow) {
    const std::string path = argc > 1 ? argv[1] : "../n_conf/config/arm.yaml";
    doc.load(path, own, borrow);
    if (!doc.problems().empty()) {
        std::printf("cannot read %s\n%s", path.c_str(), doc.report().c_str());
        return false;
    }
    std::printf("  config %s\n", path.c_str());
    return true;
}


static void check(bool ok, const char *what) {
    std::printf("%-46s %s\n", what, ok ? "ok" : "FAILED");
    failures += ok ? 0 : 1;
}

int main(int argc, char **argv) {
    conf::Doc doc;
    if (!loadConfig(argc, argv, doc, {"arm"}, {"driver"})) {
        return 1;
    }

    Params arm;
    arm.load(doc);
    check(doc.ok(), "the config fills every solver parameter");
    if (!doc.ok()) {
        std::printf("%s", doc.report().c_str());
        return 1;
    }

    const Geom g(arm);
    check(g.ok(), "the loaded geometry is solvable");
    if (!g.ok()) {
        std::printf("  -> %s\n", g.fault());
        return 1;
    }
    std::printf("  upper arm %.4f m, forearm to throat %.4f m\n",
                g.upperArm().len, g.forearm(g.throatAlong()).len);
    std::printf("  throat reach %.4f .. %.4f m\n",
                g.reachMin(g.throatAlong()), g.reachMax(g.throatAlong()));

    // FK at the rest pose the driver sends.
    const double rest_wire[DOF] = {0.0, 90.0, 1.0, 1.0};
    const Joints rest           = toKinematic(g.params(), rest_wire);
    const Pose   p              = forward(g, rest);
    std::printf("  rest throat (%.4f, %.4f, %.4f)\n", p.throat.x, p.throat.y, p.throat.z);

    // The frame is orthonormal and its approach points down the tool.
    const Frame f = frame(g, rest);
    check(std::fabs(dot(f.approach, f.hinge)) < 1e-9 && std::fabs(dot(f.hinge, f.close)) < 1e-9
                  && std::fabs(dot(f.approach, f.close)) < 1e-9,
          "frame axes are orthogonal");
    check(norm(unit(p.tip - p.mount) - f.approach) < 1e-9, "approach points down the tool");

    // IK -> FK round trip on the throat, over a spread of reachable targets.
    double worst = 0.0;
    int    solved = 0, tried = 0;
    for (double x = -0.25; x <= 0.25; x += 0.05) {
        for (double y = -0.25; y <= 0.25; y += 0.05) {
            for (double z = 0.0; z <= 0.30; z += 0.05) {
                const Vec3 target{x, y, z};
                ++tried;

                std::vector<Branch> bs;
                Fail                why = Fail::NONE;
                branches(g, target, g.throatAlong(), rest, bs, why);
                if (bs.empty()) {
                    continue;
                }
                ++solved;
                for (const Branch &b : bs) {
                    const Vec3 got = forward(g, b.q).throat;
                    const double e = norm(got - target);
                    worst = std::max(worst, e);
                }
            }
        }
    }
    std::printf("  %d of %d grid targets solved, worst throat error %.3e m\n", solved, tried, worst);
    check(worst < 1e-9, "every branch lands the throat on its target");

    // Refusals are specific.
    Joints out;
    check(solve(g, {5.0, 0.0, 0.1}, g.throatAlong(), true, false, rest, out) == Fail::TOO_FAR,
          "far target reports TOO_FAR");

    // Every solved joint is inside its window.
    std::vector<Branch> bs;
    Fail                why = Fail::NONE;
    branches(g, {0.15, 0.0, 0.10}, g.throatAlong(), rest, bs, why);
    bool inside = !bs.empty();
    for (const Branch &b : bs) {
        for (int j = 0; j < DOF; ++j) {
            inside = inside && b.q[j] >= g.windowLo(j) - 1e-12 && b.q[j] <= g.windowHi(j) + 1e-12;
        }
    }
    check(inside, "solved joints stay inside the window");

    // Clocking squares the hinge to the handle axis.
    if (!bs.empty()) {
        double roll[2] = {0.0, 0.0};
        const int n    = clocking(g, bs[0].q, {0.0, 1.0, 0.0}, roll);
        bool      square = n == 2;
        for (int i = 0; i < n; ++i) {
            Joints q = bs[0].q;
            q[WRIST] = roll[i];
            const double off = angleTo(frame(g, q).hinge, {0.0, 1.0, 0.0});
            square = square && (off < 1e-9 || std::fabs(off - M_PI) < 1e-9);
        }
        check(square, "clocking lines the hinge up with the handle");
    }

    // Wire <-> kinematic is its own inverse.
    bool round = true;
    for (int j = 0; j < DOF; ++j) {
        const double wire = 42.0;
        round = round && std::fabs(toPubDeg(g.params(), j, toKinematic(g.params(), j, wire)) - wire) < 1e-9;
    }
    check(round, "wire <-> kinematic round trips");

    std::printf("\n%s\n", failures == 0 ? "ALL OK" : "SOME CHECKS FAILED");
    return failures == 0 ? 0 : 1;
}
