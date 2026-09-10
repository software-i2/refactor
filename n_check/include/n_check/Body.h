// Copyright by BeeX [2026]

#ifndef N_CHECK_BODY_H
#define N_CHECK_BODY_H

#include <n_check/Field.h>
#include <n_conf/Doc.h>
#include <n_kine/Fk.h>
#include <n_kine/Geom.h>

#include <string>
#include <vector>

namespace check {

// ─────────────────────────────────────────────────────────────────────────────
// EVERY TUNABLE THE JAWS HAVE. Geometry from alpha_description
// standard_jaws.urdf.xacro. Filled by load() from the `jaws` section of
// n_conf/config/arm.yaml.
// ─────────────────────────────────────────────────────────────────────────────
struct Jaws {
    double palm_radius_m = kine::NONE;
    double palm_length_m = kine::NONE;

    double hinge_xyz[3]    = {kine::NONE, kine::NONE, kine::NONE};
    double blade_size[3]   = {kine::NONE, kine::NONE, kine::NONE};  // hinge, closing, approach
    double blade_origin[3] = {kine::NONE, kine::NONE, kine::NONE};
    double open_mm         = kine::NONE;

    // Blade that must still lie beyond the grasp point. Without it the search
    // "solves" every hold by grasping at the fingertips, holding nothing.
    double grip_margin_m = kine::NONE;

    // NOT a tunable either: standard_jaws.urdf.xacro mimics the blade hinge off
    // axis_a with multiplier="51", and axis_a is metres.
    static constexpr double HINGE_RAD_PER_MM = 51.0 / 1000.0;
    double openRad() const { return open_mm * HINGE_RAD_PER_MM; }

    // NOT a tunable, and deliberately not in the config. How finely the blade
    // interiors are sampled is fixed by the field they are tested against: the
    // exporter sizes the jaw dilation radius for a pitch of half a voxel, so
    // sampling any coarser leaves a blade able to pass through an occupied cell
    // no sample landed in. bladePitch() derives it; fitsField() proves it.
    static double bladePitch(double field_res_m) { return 0.5 * field_res_m; }

    // Fills every field from the `jaws` section. The sampling pitch is not
    // among them: see bladePitch() above.
    void load(conf::Doc &doc) {
        palm_radius_m = doc.num("jaws.palm_radius_m");
        palm_length_m = doc.num("jaws.palm_length_m");
        doc.nums("jaws.hinge_xyz", hinge_xyz, 3);
        doc.nums("jaws.blade_size", blade_size, 3);
        doc.nums("jaws.blade_origin", blade_origin, 3);
        open_mm       = doc.num("jaws.open_mm");
        grip_margin_m = doc.num("jaws.grip_margin_m");
    }

    // Name of the first field still unset, or NULL when all of them are filled.
    const char *missing() const;
};

// The volumes the field tests, in the order it tests them.
enum Vol : int {
    V_UPPER_ARM = 0,
    V_FOREARM,
    V_WRIST_MOUNT,
    V_PALM,
    V_BLADE_LEFT,
    V_BLADE_RIGHT,
    N_VOLS
};

const char *name(int vol);

// The arm as the field has to see it: four axes to sweep, plus the two blade
// boxes sampled as clouds. The blades are not a capsule on the wrist axis --
// that would swallow whatever the jaws are meant to close on.
class Body {
public:
    // `blade_pitch_m` comes from Jaws::bladePitch(field.res()). Zero or less
    // builds no lattice, which is what a run with no field wants: there is
    // nothing to sample against, and firstBlocked() already skips the blades.
    Body(const Jaws &j, double blade_pitch_m);

    struct Volume {
        kine::Vec3 shoulder, elbow, wrist, mount, palm_end;
        // Left blade is [0, left), right is [left, count).
        const kine::Vec3 *blades = NULL;
        size_t            left  = 0;
        size_t            count = 0;
    };

    // `buf` holds the blade samples and the volume points into it, so a hot
    // loop reuses one allocation.
    void volume(const kine::Geom &g, const kine::Joints &q,
                std::vector<kine::Vec3> &buf, Volume &out) const;

    const Jaws &jaws() const { return j_; }
    double bladePitchM() const { return pitch_; }
    size_t latticeSize() const { return lattice_.size(); }

    // Half the clear span between the blades at `depth` along the claw. The
    // blades open outward, so the gap widens the further down you grasp.
    double throatHalfGap(double depth) const;

    // Deepest hold that still leaves grip_margin_m of blade beyond it.
    double maxDepth() const;

private:
    // Blade interiors in ee components (approach, hinge, closing), built once:
    // the same lattice serves every posture.
    void buildLattice();

    Jaws   j_;
    double pitch_  = 0.0;
    bool   box_ok_ = false;

    // The blade box at full open, in ee components (lat = closing, ax = approach).
    double centre_lat_ = 0.0, centre_ax_ = 0.0;
    double close_lat_ = 0.0, close_ax_ = 0.0;
    double appr_lat_ = 0.0, appr_ax_ = 0.0;
    double half_[3] = {0.0, 0.0, 0.0};

    std::vector<kine::Vec3> lattice_;
};

// Lowest point of the arm at this posture, for a floor check.
double lowestZ(const kine::Geom &g, const kine::Joints &q);

// Which volume the field stops first, or -1 when the posture is clear.
int firstBlocked(const Field &f, const Body::Volume &v);

// True when the field marks the arm's own body as blocked at a posture it is
// known to be safe in. That means the field includes the arm itself, or was
// placed against a different mount -- not that the pose is dangerous.
bool blocksItself(const Field &f, const Body &b, const kine::Geom &g,
                  const kine::Joints &safe, std::string &why);

// A blade pitch coarser than the field's voxels can step over an obstacle.
// False, with a sentence, when the two do not fit together.
bool fitsField(const Body &b, const Field &f, std::string &why);

}  // namespace check

#endif  // N_CHECK_BODY_H
