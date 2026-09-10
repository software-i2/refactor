// Copyright by BeeX [2026]

#ifndef N_CTRL_PATH_H
#define N_CTRL_PATH_H

#include <n_check/Body.h>
#include <n_check/Field.h>
#include <n_ctrl/Move.h>
#include <n_ctrl/Params.h>
#include <n_kine/Geom.h>
#include <n_kine/Ik.h>

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

namespace ctrl {

using Path = std::vector<kine::Joints>;

struct Leg {
    kine::Vec3 target;
    double     q_wrist    = kine::NONE;
    bool       facing_out = true;
    bool       elbow_up   = false;
};

inline std::vector<float> encodeLeg(const Leg &leg) {
    std::vector<float> data(6);
    data[0] = static_cast<float>(leg.target.x);
    data[1] = static_cast<float>(leg.target.y);
    data[2] = static_cast<float>(leg.target.z);
    data[3] = static_cast<float>(leg.q_wrist);
    data[4] = leg.facing_out ? 1.0f : 0.0f;
    data[5] = leg.elbow_up ? 1.0f : 0.0f;
    return data;
}

inline bool decodeLeg(const std::vector<float> &data, Leg &out, std::string &why) {
    char buf[192];
    if (data.size() != 6) {
        std::snprintf(buf, sizeof(buf),
                      "a grasp leg is 6 values (x y z wrist_rad facing_out elbow_up), got %u. "
                      "Three values is a bare point, which cannot say how to hold the jaws.",
                      static_cast<uint32_t>(data.size()));
        why = buf;
        return false;
    }
    if (!std::isfinite(data[3])) {
        why = "the leg carries no wrist roll, so the jaws would arrive at whatever angle they "
              "happen to be at";
        return false;
    }

    out.target     = {data[0], data[1], data[2]};
    out.q_wrist    = data[3];
    out.facing_out = data[4] != 0.0f;
    out.elbow_up   = data[5] != 0.0f;
    return true;
}

inline std::vector<float> encodeJoints(const kine::Joints &q) {
    std::vector<float> data(kine::DOF);
    for (int j = 0; j < kine::DOF; ++j) {
        data[j] = static_cast<float>(q[j]);
    }
    return data;
}

inline bool decodeJoints(const std::vector<float> &data, kine::Joints &out, std::string &why) {
    char buf[160];
    if (data.size() != kine::DOF) {
        std::snprintf(buf, sizeof(buf),
                      "a joint target is %d values (base shoulder elbow wrist, kinematic "
                      "radians), got %u",
                      kine::DOF, static_cast<uint32_t>(data.size()));
        why = buf;
        return false;
    }
    for (int j = 0; j < kine::DOF; ++j) {
        if (!std::isfinite(data[j])) {
            std::snprintf(buf, sizeof(buf), "joint %d is not a finite angle", j);
            why = buf;
            return false;
        }
        out[j] = data[j];
    }
    return true;
}

Status solveTarget(const kine::Geom &g,
                   const kine::Joints &seed,
                   const kine::Vec3 &target,
                   kine::Joints &out,
                   kine::Branch *chosen = NULL);

void planJoint(const Params &p,
               const kine::Joints &from,
               const kine::Joints &to,
               Path &out);

Status planLine(const kine::Geom &g,
                const Params &p,
                const kine::Joints &from,
                const kine::Vec3 &target,
                Path &out,
                double &deviation_m);

Status planLineOn(const kine::Geom &g,
                  const Params &p,
                  const kine::Joints &from,
                  const Leg &leg,
                  Path &out,
                  double &deviation_m);

Status admit(const kine::Geom &g,
             const Params &p,
             const Path &path,
             const check::Field &field,
             const check::Body &body,
             std::vector<kine::Vec3> &scratch,
             std::string &why);

}

#endif
