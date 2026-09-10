// Copyright by BeeX [2026]

#ifndef N_DRIVER_PARAMS_H
#define N_DRIVER_PARAMS_H

#include <n_conf/Doc.h>
#include <n_driver/Joints.h>

#include <cmath>
#include <string>

namespace reach {

// ─────────────────────────────────────────────────────────────────────────────
// EVERY TUNABLE THE DRIVER HAS. Filled by load() from the `driver`
// section of n_conf/config/arm.yaml, plus jaws.open_mm, which is borrowed so
// that open_jaw and the collision body cannot be told two different openings.
// ─────────────────────────────────────────────────────────────────────────────
struct Params {
    static constexpr double NONE = std::numeric_limits<double>::quiet_NaN();

    std::string port;          // "sim" to run without hardware
    int         baud = -1;
    double      rate_hz = NONE;

    double climate_period_s = NONE;  // one joint's climate per period, round robin
    double log_period_s     = NONE;  // console cadence; a mode change always prints
    double jog_timeout_s    = NONE;  // a jog not re-sent within this is zeroed

    double reply_timeout_s   = NONE;
    double climate_timeout_s = NONE;
    int    climate_max_miss  = -1;

    double sim_joint_speed = NONE;   // sim only, wire units per second
    double sim_jaw_speed   = NONE;

    double jaw_open_mm = NONE;       // where open_jaw drives to; owned by `jaws`

    Limits limits;

    // Fills every field from the `driver` section, limits included.
    void load(conf::Doc &doc) {
        port    = doc.text("driver.port");
        baud    = doc.integer("driver.baud");
        rate_hz = doc.num("driver.rate_hz");

        climate_period_s = doc.num("driver.climate_period_s");
        log_period_s     = doc.num("driver.log_period_s");
        jog_timeout_s    = doc.num("driver.jog_timeout_s");

        reply_timeout_s   = doc.num("driver.reply_timeout_s");
        climate_timeout_s = doc.num("driver.climate_timeout_s");
        climate_max_miss  = doc.integer("driver.climate_max_miss");

        sim_joint_speed = doc.num("driver.sim_joint_speed");
        sim_jaw_speed   = doc.num("driver.sim_jaw_speed");

        jaw_open_mm = doc.num("jaws.open_mm");

        limits.load(doc);
    }

    // Name of the first field still unset, or NULL when all of them are filled.
    const char *missing() const {
        struct Field {
            const char   *name;
            const double *at;
        };
        const Field fields[] = {
                {"driver.rate_hz", &rate_hz},
                {"driver.climate_period_s", &climate_period_s},
                {"driver.log_period_s", &log_period_s},
                {"driver.jog_timeout_s", &jog_timeout_s},
                {"driver.reply_timeout_s", &reply_timeout_s},
                {"driver.climate_timeout_s", &climate_timeout_s},
                {"driver.sim_joint_speed", &sim_joint_speed},
                {"driver.sim_jaw_speed", &sim_jaw_speed},
                {"jaws.open_mm", &jaw_open_mm},
        };

        for (const Field &f : fields) {
            if (!std::isfinite(*f.at)) {
                return f.name;
            }
        }
        if (port.empty()) {
            return "driver.port";
        }
        if (baud < 1) {
            return "driver.baud";
        }
        if (climate_max_miss < 1) {
            return "driver.climate_max_miss";
        }
        return limits.missing();
    }
};

}  // namespace reach

#endif  // N_DRIVER_PARAMS_H
