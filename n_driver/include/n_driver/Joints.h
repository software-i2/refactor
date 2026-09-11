// Copyright by BeeX [2026]

#ifndef N_DRIVER_JOINTS_H
#define N_DRIVER_JOINTS_H

#include <n_conf/Doc.h>

#include <cmath>
#include <cstdint>
#include <limits>

namespace reach {

enum Joint : uint32_t { JAW = 0, WRIST, ELBOW, SHOULDER, BASE, N_JOINTS };

constexpr uint8_t DEVICE[N_JOINTS] = {0x01, 0x02, 0x03, 0x04, 0x05};
constexpr const char *NAME[N_JOINTS] = {"jaw", "wrist", "elbow", "shoulder", "base_rot"};
constexpr const char *CONF_NAME[N_JOINTS] = {"jaw", "wrist", "elbow", "shoulder", "base"};
constexpr const char *LIMIT_KEY[N_JOINTS] = {
        "driver.limits.jaw", "driver.limits.wrist", "driver.limits.elbow",
        "driver.limits.shoulder", "driver.limits.base"};
constexpr const char *URDF_NAME[N_JOINTS] = {"axis_a", "axis_b", "axis_c", "axis_d", "axis_e"};

constexpr float RAD_TO_DEG = static_cast<float>(180.0 / M_PI);
constexpr float SCALE[N_JOINTS] = {1.0f, RAD_TO_DEG, RAD_TO_DEG, RAD_TO_DEG, RAD_TO_DEG};

inline float toPub(uint32_t j, float wire) { return wire * SCALE[j]; }
inline float toWireRad(uint32_t j, float pub) { return pub / SCALE[j]; }

struct Limits {
    static constexpr float NONE = std::numeric_limits<float>::quiet_NaN();

    float min_pos[N_JOINTS] = {NONE, NONE, NONE, NONE, NONE};
    float max_pos[N_JOINTS] = {NONE, NONE, NONE, NONE, NONE};
    float rest_pos[N_JOINTS] = {NONE, NONE, NONE, NONE, NONE};
    float max_vel[N_JOINTS] = {NONE, NONE, NONE, NONE, NONE};

    bool inRange(uint32_t j, float pos) const {
        return std::isfinite(pos) && std::isfinite(min_pos[j]) && pos >= min_pos[j]
               && pos <= max_pos[j];
    }

    bool velOk(uint32_t j, float vel) const {
        return std::isfinite(vel) && std::isfinite(max_vel[j]) && std::fabs(vel) <= max_vel[j];
    }

    void load(conf::Doc &doc) {
        for (uint32_t j = 0; j < N_JOINTS; ++j) {
            const std::string key = std::string("driver.limits.") + CONF_NAME[j];
            min_pos[j]            = static_cast<float>(doc.num(key + ".min"));
            max_pos[j]            = static_cast<float>(doc.num(key + ".max"));
            rest_pos[j]           = static_cast<float>(doc.num(key + ".rest"));
            max_vel[j]            = static_cast<float>(doc.num(key + ".max_vel"));
        }
    }

    const char *missing() const {
        for (uint32_t j = 0; j < N_JOINTS; ++j) {
            if (!std::isfinite(min_pos[j]) || !std::isfinite(max_pos[j])
                || !std::isfinite(rest_pos[j]) || !std::isfinite(max_vel[j])
                || min_pos[j] > max_pos[j] || max_vel[j] <= 0.0f
                || rest_pos[j] < min_pos[j] || rest_pos[j] > max_pos[j]) {
                return LIMIT_KEY[j];
            }
        }
        return NULL;
    }
};

// -1 if this arm has no such joint.
inline int jointOf(uint8_t device) {
    for (uint32_t j = 0; j < N_JOINTS; ++j) {
        if (DEVICE[j] == device) {
            return static_cast<int>(j);
        }
    }
    return -1;
}

}  // namespace reach

#endif  // N_DRIVER_JOINTS_H
