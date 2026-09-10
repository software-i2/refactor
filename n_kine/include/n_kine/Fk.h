// Copyright by BeeX [2026]

#ifndef N_KINE_FK_H
#define N_KINE_FK_H

#include <n_kine/Geom.h>
#include <n_kine/Vec3.h>

namespace kine {

// Every pivot and tool point, base frame.
struct Pose {
    Vec3 shoulder;
    Vec3 elbow;
    Vec3 wrist;
    Vec3 mount;
    Vec3 throat;
    Vec3 tip;
};

// Jaw orientation: down the wrist axis, along the finger pivot, and the line
// the fingers close on.
struct Frame {
    Vec3 approach;
    Vec3 hinge;
    Vec3 close;
};

// One pass, no repeated trig.
Pose forward(const Geom &g, const Joints &q);

Frame frame(const Geom &g, const Joints &q);

}  // namespace kine

#endif  // N_KINE_FK_H
