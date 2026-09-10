// Copyright by BeeX [2026]

#ifndef N_DRIVER_JOINTS_H
#define N_DRIVER_JOINTS_H

#include <n_conf/Doc.h>

#include <cmath>
#include <cstdint>
#include <limits>

namespace reach {

// Reach Alpha 5, in the order every ROS array uses. Identities, not settings:
// these name the hardware rather than tune it, so they stay compiled in.
enum Joint : uint32_t { JAW = 0, WRIST, ELBOW, SHOULDER, BASE, N_JOINTS };

constexpr uint8_t DEVICE[N_JOINTS] = {0x01, 0x02, 0x03, 0x04, 0x05};

constexpr const char *NAME[N_JOINTS] = {"jaw", "wrist", "elbow", "shoulder", "base_rot"};

// The same joints as the config keys them, so a lookup cannot land on the
// wrong one. "base_rot" is what ROS calls it; the config says "base".
constexpr const char *CONF_NAME[N_JOINTS] = {"jaw", "wrist", "elbow", "shoulder", "base"};

// Joint names in alpha_description's xacro, same order.
constexpr const char *URDF_NAME[N_JOINTS] = {"axis_a", "axis_b", "axis_c", "axis_d", "axis_e"};

// Units, not tunables: the rotary joints talk radians on the wire and degrees
// everywhere else, and the jaw talks millimetres throughout.
constexpr float RAD_TO_DEG = static_cast<float>(180.0 / M_PI);
constexpr float SCALE[N_JOINTS] = {1.0f, RAD_TO_DEG, RAD_TO_DEG, RAD_TO_DEG, RAD_TO_DEG};

inline float toPub(uint32_t j, float wire) { return wire * SCALE[j]; }
inline float toWire(uint32_t j, float pub) { return pub / SCALE[j]; }

// What the hardware accepts, from driver.limits. No defaults: a field still
// holding NaN means the config was never loaded.
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

    // Keyed by joint name, so there is no ordering to get wrong.
    void load(conf::Doc &doc) {
        for (uint32_t j = 0; j < N_JOINTS; ++j) {
            const std::string key = std::string("driver.limits.") + CONF_NAME[j];
            min_pos[j]            = static_cast<float>(doc.num(key + ".min"));
            max_pos[j]            = static_cast<float>(doc.num(key + ".max"));
            rest_pos[j]           = static_cast<float>(doc.num(key + ".rest"));
            max_vel[j]            = static_cast<float>(doc.num(key + ".max_vel"));
        }
    }

    // Name of the first joint still unset, or NULL when all of them are filled.
    const char *missing() const {
        for (uint32_t j = 0; j < N_JOINTS; ++j) {
            if (!std::isfinite(min_pos[j]) || !std::isfinite(max_pos[j])
                || !std::isfinite(rest_pos[j]) || !std::isfinite(max_vel[j])) {
                return CONF_NAME[j];
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
