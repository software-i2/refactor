// Copyright by BeeX [2026]

#include <n_kine/Fk.h>

#include <cmath>

namespace kine {
namespace {

// Arm-plane point (x forward, z up) lifted into the base frame.
Vec3 place(double x, double z, double base_z, double cb, double sb) {
    return spin({x, 0.0, z + base_z}, cb, sb);
}

}  // namespace

Pose forward(const Geom &g, const Joints &q) {
    const Params &p  = g.params();
    const Link    l1 = g.upperArm();
    const Link    l2 = g.forearmRaw();

    const double a1 = l1.psi + q[SHOULDER];                          // upper arm
    const double a2 = g.elbowSign() * q[ELBOW] + q[SHOULDER];        // forearm, and the wrist axis
    const double cb = std::cos(q[BASE]);
    const double sb = std::sin(q[BASE]);

    const double sh_x = p.e_to_d_x;
    const double sh_z = p.e_to_d_z;
    const double el_x = sh_x + l1.len * std::sin(a1);
    const double el_z = sh_z + l1.len * std::cos(a1);
    const double wr_x = el_x + l2.len * std::sin(l2.psi + a2);
    const double wr_z = el_z + l2.len * std::cos(l2.psi + a2);

    // The tool is coaxial, so every point past the wrist is this axis scaled.
    const double ax_x = std::sin(a2);
    const double ax_z = std::cos(a2);

    Pose pose;
    pose.shoulder = place(sh_x, sh_z, p.base_to_e_z, cb, sb);
    pose.elbow    = place(el_x, el_z, p.base_to_e_z, cb, sb);
    pose.wrist    = place(wr_x, wr_z, p.base_to_e_z, cb, sb);
    pose.mount    = place(wr_x + p.b_to_mount * ax_x,
                          wr_z + p.b_to_mount * ax_z, p.base_to_e_z, cb, sb);
    pose.throat   = place(wr_x + g.throatAlong() * ax_x,
                          wr_z + g.throatAlong() * ax_z, p.base_to_e_z, cb, sb);
    pose.tip      = place(wr_x + g.tipAlong() * ax_x,
                          wr_z + g.tipAlong() * ax_z, p.base_to_e_z, cb, sb);
    return pose;
}

Frame frame(const Geom &g, const Joints &q) {
    const double a2 = g.elbowSign() * q[ELBOW] + q[SHOULDER];
    const double sa = std::sin(a2);
    const double ca = std::cos(a2);
    const double cb = std::cos(q[BASE]);
    const double sb = std::sin(q[BASE]);

    const double th = g.hingeOffsetRad() + q[WRIST];
    const double ct = std::cos(th);
    const double st = std::sin(th);

    Frame f;
    f.approach = spin({sa, 0.0, ca}, cb, sb);
    f.hinge    = spin({ct * ca, st, -ct * sa}, cb, sb);
    f.close    = spin({-st * ca, ct, st * sa}, cb, sb);
    return f;
}

}  // namespace kine
