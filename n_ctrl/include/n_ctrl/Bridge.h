// Copyright by BeeX [2026]

#ifndef N_CTRL_BRIDGE_H
#define N_CTRL_BRIDGE_H

#include <n_driver/Joints.h>
#include <n_kine/Angle.h>
#include <n_kine/Geom.h>
#include <sensor_msgs/JointState.h>

namespace ctrl {

// The driver counts jaw first; the solver counts from the base out.
constexpr uint32_t WIRE_SLOT[kine::DOF] = {reach::BASE, reach::SHOULDER, reach::ELBOW, reach::WRIST};

inline kine::Joints toKine(const kine::Params &p, const float wire_deg[reach::N_JOINTS]) {
    kine::Joints q;
    for (int j = 0; j < kine::DOF; ++j) {
        q[j] = kine::toKinematic(p, j, wire_deg[WIRE_SLOT[j]]);
    }
    return q;
}

// The rest pose, taken from the driver's own table rather than written down
// again. Both come from the same config keys, so they cannot disagree.
inline kine::Joints restPose(const kine::Params &p, const reach::Limits &limits) {
    return toKine(p, limits.rest_pos);
}

// The driver's joint_states, in kinematic radians. False when the message is
// not the driver's -- a joint missing means someone else published it.
inline bool readJointState(const kine::Params &p,
                           const sensor_msgs::JointState &msg,
                           kine::Joints &out) {
    float wire_deg[reach::N_JOINTS] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    for (int k = 0; k < kine::DOF; ++k) {
        const uint32_t j     = WIRE_SLOT[k];
        bool           found = false;
        for (size_t i = 0; i < msg.name.size() && i < msg.position.size(); ++i) {
            if (msg.name[i] == reach::URDF_NAME[j]) {
                wire_deg[j] = static_cast<float>(kine::rad2deg(msg.position[i]));
                found       = true;
                break;
            }
        }
        if (!found) {
            return false;
        }
    }

    out = toKine(p, wire_deg);
    return true;
}

}  // namespace ctrl

#endif  // N_CTRL_BRIDGE_H
