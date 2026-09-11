// Copyright by BeeX [2026]
//
// Plain g++, no ROS:
//   g++ -std=c++17 -Iinclude -I../n_kine/include -I../n_conf/include test/check.cpp
//       src/*.cpp ../n_kine/src/*.cpp ../n_conf/src/Doc.cpp -lyaml-cpp -o /tmp/check
//
// argv[1] is the config (default ../n_conf/config/arm.yaml); argv[2] an optional
// field file, to exercise the loader against a real one:
//   /tmp/check ../n_conf/config/arm.yaml ../../../data/field.bin

#include <n_check/Body.h>
#include <n_check/Field.h>
#include <n_check/Grasp.h>
#include <n_kine/Angle.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

using namespace check;

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

static void ok(bool pass, const char *what) {
    std::printf("%-52s %s\n", what, pass ? "ok" : "FAILED");
    failures += pass ? 0 : 1;
}

int main(int argc, char **argv) {
    // ── the config, which is the only source of the geometry ──────────────
    conf::Doc doc;
    if (!loadConfig(argc > 1 ? argv[1] : "../n_conf/config/arm.yaml", doc,
                    {"arm", "jaws"}, {"driver", "task", "world"})) {
        return 1;
    }

    kine::Params arm;
    Jaws         jaws;
    arm.load(doc);
    jaws.load(doc);
    ok(doc.ok(), "the config fills the arm and the jaws");
    if (!doc.ok()) {
        std::printf("%s", doc.report().c_str());
        return 1;
    }

    const kine::Geom g(arm);
    ok(g.ok(), "the loaded geometry is solvable");
    if (!g.ok()) {
        std::printf("  -> %s\n", g.fault());
        return 1;
    }

    Field       f;
    std::string err;

    // ── refusals, without any file ────────────────────────────────────────
    ok(!f.load("/definitely/not/here.bin", err), "a missing file fails to load");
    std::printf("  -> %s\n", err.c_str());

    ok(!f.load("/etc/hostname", err), "a non-BXFIELD4 file fails to load");
    std::printf("  -> %s\n", err.c_str());

    // ── the real artifact, when one was given ─────────────────────────────
    if (argc > 2) {
        ok(f.load(argv[2], err), "the exported field loads");
        if (!f.ok()) {
            std::printf("  -> %s\n", err.c_str());
            return 1;
        }
    }

    // ── the arm's body ────────────────────────────────────────────────────
    // Blade sampling is derived from the field, never typed: half a voxel is
    // what the exporter sized the jaw dilation radius for.
    const double pitch = f.ok() ? Jaws::bladePitch(f.res()) : Jaws::bladePitch(0.005);
    const Body   body(jaws, pitch);
    const double       rest_wire[kine::DOF] = {0.0, 90.0, 1.0, 1.0};
    const kine::Joints rest = kine::toKinematic(g.params(), rest_wire);

    std::vector<kine::Vec3> buf;
    Body::Volume            v;
    body.volume(g, rest, buf, v);
    std::printf("  blade lattice: %zu samples at %.1f mm pitch\n",
                body.latticeSize(), body.bladePitchM() * 1000.0);

    // The two blades must straddle the wrist axis, not sit on one side of it.
    const kine::Frame fr = kine::frame(g, rest);
    double            side_l = 0.0, side_r = 0.0;
    for (size_t i = 0; i < v.left; ++i) side_l += kine::dot(v.blades[i] - v.mount, fr.close);
    for (size_t i = v.left; i < v.count; ++i) side_r += kine::dot(v.blades[i] - v.mount, fr.close);
    ok(side_l * side_r < 0.0, "the two blades sit on opposite sides of the axis");

    // The gap between the inner faces at the throat. reach_pcloud's fold_jaws
    // docstring names 15 mm here, and 50 mm as the symptom of folding it wrong,
    // so this is a cross-check of the fold against the python side. Taken at the
    // hinge's own 0.5 rad limit, which is what the python side folded to, rather
    // than at jaws.open_mm -- otherwise retuning the opening moves the baseline.
    Jaws full     = jaws;
    full.open_mm  = 0.5 / Jaws::HINGE_RAD_PER_MM;
    const double gap_mm = 2000.0 * Body(full, 0.0).throatHalfGap(g.params().mount_to_throat);
    std::printf("  jaws open %.1f mm (%.3f rad), gap at the throat %.1f mm;"
                " at full open %.1f mm\n",
                jaws.open_mm, jaws.openRad(),
                2000.0 * body.throatHalfGap(g.params().mount_to_throat), gap_mm);
    ok(gap_mm > 12.0 && gap_mm < 18.0, "the throat gap is the 15 mm the fold should give");
    ok(body.throatHalfGap(0.070) > body.throatHalfGap(0.040),
       "the gap widens further down the claw");

    // The derived pitch must satisfy the field it is tested against, and a
    // coarser one must be refused -- that inequality is the whole soundness
    // argument for sampling the blades at all.
    std::string why;
    if (f.ok()) {
        ok(fitsField(body, f, why), "the derived blade pitch fits the field");
        if (!why.empty()) {
            std::printf("  -> %s\n", why.c_str());
        }
        why.clear();
        ok(!fitsField(Body(jaws, 4.0 * pitch), f, why),
           "a pitch four times coarser is refused");
        std::printf("  -> %s\n", why.c_str());
    }

    // ── holding a handle ──────────────────────────────────────────────────
    // No field for these: whether the jaws CAN hold it is geometry, and the
    // field only ever takes options away.
    Field                   none;
    std::vector<kine::Vec3> scratch;

    Ask ask;
    ask.load(doc);
    ask.point = {0.15, 0.0, 0.10};
    ask.axis  = {0.0, 1.0, 0.0};   // a handle lying across the approach

    Hold h = holdable(g, body, none, ask, rest, scratch);
    std::printf("  hold at (0.15, 0, 0.10): %s\n", h.ok() ? "yes" : name(h.block));
    if (h.ok()) {
        std::printf("    depth %.3f m, wrist margin %.1f deg, travel %.3f, elbow %s\n",
                    h.depth_m, kine::rad2deg(h.wrist_margin_rad), h.travel,
                    h.elbow_up ? "up" : "down");
    }

    if (h.ok()) {
        // The throat must actually land on the handle point it solved for.
        const kine::Vec3 at = kine::forward(g, h.joints).throat;
        ok(kine::norm(at - h.point) < 1e-9, "the throat lands on the handle");

        // The jaw hinge must be square to the handle axis, or the jaws close
        // across it instead of around it.
        const double off = kine::angleTo(kine::frame(g, h.joints).hinge, ask.axis);
        std::printf("    hinge is %.2e deg off the handle axis\n", kine::rad2deg(off));
        ok(off < 1e-6 || std::fabs(off - M_PI) < 1e-6, "the jaws are square to the handle");

        // The standoff sits back along the approach, on the same branch.
        const kine::Vec3 sp = kine::forward(g, h.standoff).throat;
        ok(kine::norm(sp - h.standoff_point) < 1e-6, "the standoff pose reaches the standoff");
        ok(std::fabs(kine::norm(h.point - h.standoff_point) - ask.standoff_m) < 1e-9,
           "the standoff is standoff_m back from the handle");

        // Depth is searched deepest-first, so it should not come back shallow.
        ok(h.depth_m >= ask.depth_min_m - 1e-9 && h.depth_m <= ask.depth_max_m + 1e-9,
           "the depth stays inside the range asked for");
    }

    // Refusals, each naming the deepest thing that stopped it.
    Ask far = ask;
    far.point = {5.0, 0.0, 0.1};
    ok(holdable(g, body, none, far, rest, scratch).block == Block::UNREACHABLE,
       "a handle out of reach is UNREACHABLE");


    Ask floored = ask;
    floored.floor_z_m = 1.0;
    const Hold fh = holdable(g, body, none, floored, rest, scratch);
    ok(fh.block == Block::FLOOR, "a floor above the arm blocks on FLOOR");

    Ask fussy = ask;
    fussy.approach = {0.0, 0.0, 1.0};      // insist on coming straight down
    fussy.max_approach_dev_deg = 1.0;
    const Hold ah = holdable(g, body, none, fussy, rest, scratch);
    std::printf("  insisting on a 1 deg approach window: %s\n", name(ah.block));
    ok(!ah.ok() && ah.block == Block::OFF_APPROACH, "an impossible approach gate says so");

    // With an approach to shift along, depth is searched deepest-first: the gap
    // between the blades widens down the claw, so a deeper hold has more room.
    Ask deep = ask;
    deep.approach = {1.0, 0.0, 0.0};
    deep.max_approach_dev_deg = 90.0;
    const Hold dh = holdable(g, body, none, deep, rest, scratch);
    std::printf("  with an approach given, depth came back %.3f m (throat sits at %.3f)\n",
                dh.depth_m, g.params().mount_to_throat);
    ok(!dh.ok() || dh.depth_m > g.params().mount_to_throat,
       "the search takes a deeper hold than the throat when it can");
    ok(!dh.ok() || body.throatHalfGap(dh.depth_m) > body.throatHalfGap(deep.depth_min_m),
       "the depth it picked has more room than the shallowest");
    ok(!dh.ok() || dh.depth_m <= body.maxDepth() + 1e-9,
       "it never grasps past the grip margin");

    if (f.ok()) {
        const kine::Vec3 lo = f.lo();
        const kine::Vec3 hi = f.hi();
        std::printf("  %.1f MB, %.0f mm voxels, %.0f mm sample step, %llu occupied\n",
                    f.bytes() / 1e6, f.res() * 1000.0, f.step() * 1000.0,
                    static_cast<unsigned long long>(f.occupied()));
        std::printf("  spans x %.3f..%.3f  y %.3f..%.3f  z %.3f..%.3f\n",
                    lo.x, hi.x, lo.y, hi.y, lo.z, hi.z);
        std::printf("  built from %016llx, radii mm %.1f %.1f %.1f %.1f %.1f\n",
                    static_cast<unsigned long long>(f.digest()),
                    f.radius(UPPER_ARM) * 1000.0, f.radius(FOREARM) * 1000.0,
                    f.radius(WRIST_MOUNT) * 1000.0, f.radius(PALM) * 1000.0,
                    f.radius(JAW) * 1000.0);

        // Far outside the mapped volume is free, not blocked.
        ok(!f.blocked({100.0, 100.0, 100.0}, UPPER_ARM), "outside the map counts as free");

        // Sweep the mapped volume for a cell that is actually blocked.
        kine::Vec3 hit;
        bool       found = false;
        for (double x = lo.x; x < hi.x && !found; x += f.res()) {
            for (double y = lo.y; y < hi.y && !found; y += f.res()) {
                for (double z = lo.z; z < hi.z && !found; z += f.res()) {
                    if (f.blocked({x, y, z}, UPPER_ARM)) {
                        hit   = {x, y, z};
                        found = true;
                    }
                }
            }
        }
        if (!found) {
            std::printf("\nSOME CHECKS FAILED\n");
            return 1;
        }
        std::printf("  first blocked cell at (%.3f, %.3f, %.3f)\n", hit.x, hit.y, hit.z);

        // A segment ending in that cell must report blocked, and name where.
        const kine::Vec3 outside{hit.x, hit.y, hi.z + 1.0};
        ok(f.blockedSegment(outside, hit, UPPER_ARM), "a segment into a blocked cell is blocked");

        std::vector<kine::Vec3> where;
        f.samples(outside, hit, UPPER_ARM, where);
        ok(!where.empty(), "the blocked samples along it are reported");
        std::printf("  %zu of the samples on that segment are blocked\n", where.size());

        bool all_blocked = true;
        for (size_t i = 0; i < where.size(); ++i) {
            all_blocked = all_blocked && f.blocked(where[i], UPPER_ARM);
        }
        ok(all_blocked, "every reported sample is itself blocked");

        // A posture somewhere in the mapped volume should eventually hit something.
        int    hits = 0, tested = 0;
        int    first = -1;
        for (double base = 0.0; base < 6.0; base += 0.05) {
            for (double sh = 0.5; sh < 3.0; sh += 0.1) {
                kine::Joints q = rest;
                q[kine::BASE] = base;
                q[kine::SHOULDER] = sh;
                body.volume(g, q, buf, v);
                const int hit = firstBlocked(f, v);
                ++tested;
                if (hit >= 0) { ++hits; if (first < 0) first = hit; }
            }
        }
        std::printf("  swept %d postures, %d blocked, first refusal was the %s\n",
                    tested, hits, name(first));

        // This field blocks the arm at rest, which is a fact about the field.
        std::string self;
        const bool  bad = blocksItself(f, body, g, rest, self);
        std::printf("  self-blocking: %s\n", bad ? self.c_str() : "no");
        ok(bad == (hits == tested),
           "a field that blocks every posture is caught as self-blocking");

        // With the real field in play the same handle can only get harder.
        const Hold with_field = holdable(g, body, f, ask, rest, scratch);
        std::printf("  same handle against the real field: %s\n",
                    with_field.ok() ? "still holdable" : name(with_field.block));

    } else {
        std::printf("\n  no field file given, skipping the field checks\n");
    }

    std::printf("\n%s\n", failures == 0 ? "ALL OK" : "SOME CHECKS FAILED");
    return failures == 0 ? 0 : 1;
}
