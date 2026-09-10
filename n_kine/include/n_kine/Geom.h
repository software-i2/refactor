// Copyright by BeeX [2026]

#ifndef N_KINE_GEOM_H
#define N_KINE_GEOM_H

#include <n_conf/Doc.h>

#include <array>
#include <limits>

namespace kine {

enum Joint : int { BASE = 0, SHOULDER, ELBOW, WRIST, DOF };

using Joints = std::array<double, DOF>;  // kinematic radians

// No value yet. Nothing in this tree carries a default: every field below is
// filled from the config file, and one still holding this means the load was
// skipped or was not checked. Params::missing() names the first such field.
constexpr double NONE = std::numeric_limits<double>::quiet_NaN();

// ─────────────────────────────────────────────────────────────────────────────
// EVERY TUNABLE THE SOLVER HAS. Lengths metres, angles degrees, joint arrays in
// BASE, SHOULDER, ELBOW, WRIST order. Filled by load() from the `arm`
// section of n_conf/config/arm.yaml.
// ─────────────────────────────────────────────────────────────────────────────
struct Params {
    // Link transforms in the arm plane, from alpha_description/xacro/alpha.urdf.xacro.
    double base_to_e_z = NONE;     // base -> axis_e
    double e_to_d_x    = NONE;     // axis_e -> axis_d
    double e_to_d_z    = NONE;
    double d_to_c_x    = NONE;     // axis_d -> axis_c, the upper arm
    double d_to_c_z    = NONE;
    double d_to_c_yaw_deg = NONE;  // must be 0 or 180; 180 reverses the elbow
    double c_to_b_x    = NONE;     // axis_c -> axis_b, the forearm
    double c_to_b_z    = NONE;

    // Along the wrist axis from the axis_b pivot. The tool is coaxial on this
    // arm, so these are the only tool numbers the solver needs.
    double b_to_mount      = NONE;  // pivot -> tool mount face
    double mount_to_throat = NONE;  // mount -> where the jaws hold
    double mount_to_tip    = NONE;  // mount -> jaw tips

    // Jaw hinge roll when the wrist reads zero.
    double hinge_offset_deg = NONE;

    // Kinematic limits.
    double limit_lo_deg[DOF] = {NONE, NONE, NONE, NONE};
    double limit_hi_deg[DOF] = {NONE, NONE, NONE, NONE};

    // Wire convention: q_kin = sign * (wire - zero_offset).
    double zero_offset_deg[DOF] = {NONE, NONE, NONE, NONE};
    double direction_sign[DOF]  = {NONE, NONE, NONE, NONE};

    // What the hardware itself accepts, in wire degrees. Not a tunable of its
    // own: load() copies it from driver.limits, which is the authority.
    double wire_lo_deg[DOF] = {NONE, NONE, NONE, NONE};
    double wire_hi_deg[DOF] = {NONE, NONE, NONE, NONE};

    // Cost of moving each joint, for choosing between IK branches.
    double weight[DOF] = {NONE, NONE, NONE, NONE};

    // Fills every field from the `arm` section, plus the wire window from
    // `driver.limits` -- the driver's table is the authority on what the
    // hardware accepts, so it is read rather than written down a second time.
    void load(conf::Doc &doc) {
        base_to_e_z     = doc.num("arm.base_to_e_z");
        e_to_d_x        = doc.num("arm.e_to_d_x");
        e_to_d_z        = doc.num("arm.e_to_d_z");
        d_to_c_x        = doc.num("arm.d_to_c_x");
        d_to_c_z        = doc.num("arm.d_to_c_z");
        d_to_c_yaw_deg  = doc.num("arm.d_to_c_yaw_deg");
        c_to_b_x        = doc.num("arm.c_to_b_x");
        c_to_b_z        = doc.num("arm.c_to_b_z");
        b_to_mount      = doc.num("arm.b_to_mount");
        mount_to_throat = doc.num("arm.mount_to_throat");
        mount_to_tip    = doc.num("arm.mount_to_tip");
        hinge_offset_deg = doc.num("arm.hinge_offset_deg");

        doc.perJoint("arm.limit_lo_deg", limit_lo_deg);
        doc.perJoint("arm.limit_hi_deg", limit_hi_deg);
        doc.perJoint("arm.zero_offset_deg", zero_offset_deg);
        doc.perJoint("arm.direction_sign", direction_sign);
        doc.perJoint("arm.weight", weight);

        for (int j = 0; j < DOF; ++j) {
            const std::string key = std::string("driver.limits.") + conf::Doc::JOINT[j];
            wire_lo_deg[j]        = doc.num(key + ".min");
            wire_hi_deg[j]        = doc.num(key + ".max");
        }
    }

    // Name of the first field still unset, or NULL when all of them are filled.
    const char *missing() const;
};

// A kinked link folded into a straight one: reach `len` at `psi` off the frame.
struct Link {
    double len = 0.0;
    double psi = 0.0;
};

// Params with the folding done once. Everything below is derived, never tuned.
class Geom {
public:
    explicit Geom(const Params &p);

    bool ok() const { return ok_; }

    // Why ok() is false: an unfilled field, or geometry that cannot be solved.
    const char *fault() const { return fault_; }

    const Params &params() const { return p_; }

    // Forearm as seen from the elbow when aiming a point `along` the wrist axis,
    // measured from the axis_b pivot. Throat, tip and the bare wrist are all
    // this function with a different offset.
    Link forearm(double along) const;
    Link forearmRaw() const { return forearm_raw_; }  // to the axis_b pivot
    Link upperArm() const { return upper_; }

    // +1, or -1 when the axis_d -> axis_c yaw reverses the elbow.
    double elbowSign() const { return elbow_sign_; }

    // Reach of a tool point about the shoulder pivot.
    double reachMin(double along) const;
    double reachMax(double along) const;

    // Kinematic limits intersected with what the hardware accepts.
    double windowLo(int j) const { return win_lo_[j]; }
    double windowHi(int j) const { return win_hi_[j]; }

    double throatAlong() const { return p_.b_to_mount + p_.mount_to_throat; }
    double tipAlong() const { return p_.b_to_mount + p_.mount_to_tip; }
    double hingeOffsetRad() const { return hinge_offset_rad_; }

private:
    Link fold(double along) const;

    Params      p_;
    bool        ok_    = false;
    const char *fault_ = "not loaded";

    Link   upper_;             // axis_d -> axis_c
    Link   forearm_raw_;       // axis_d -> axis_b, no tool
    Link   forearm_throat_;    // folded once: every solve aims the throat
    Link   forearm_tip_;
    double elbow_sign_ = 1.0;
    double x2_         = 0.0;  // forearm offset across the axis
    double hinge_offset_rad_ = 0.0;

    Joints win_lo_{};
    Joints win_hi_{};
};

}  // namespace kine

#endif  // N_KINE_GEOM_H
