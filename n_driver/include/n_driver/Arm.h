// Copyright by BeeX [2026]

#ifndef N_DRIVER_ARM_H
#define N_DRIVER_ARM_H

#include <n_driver/Joints.h>
#include <n_driver/Packet.h>
#include <n_driver/Port.h>

#include <memory>
#include <mutex>
#include <vector>

namespace reach {

struct Climate {
    float temp     = 0.0f;
    float pressure = 0.0f;
    float humidity = 0.0f;
};

class Arm {
public:

    Arm(std::shared_ptr<Port> port, double reply_timeout_s, double climate_timeout_s,
        int climate_max_miss);

    void move(uint32_t j, float pos);
    void jog(uint32_t j, float vel);
    void standby(uint32_t j);

    bool position(uint32_t j, float &pos);
    Mode mode(uint32_t j);

    bool climate(uint32_t j, Climate &out);
    bool ping(int tries = 5);

private:
    bool send(uint8_t device, uint8_t id, const std::vector<uint8_t> &data);
    bool ask(uint8_t device, uint8_t field, double timeout, Packet &out);

    std::shared_ptr<Port> port_;
    Reader                reader_;
    std::mutex            mutex_;

    double reply_timeout_s_;
    double climate_timeout_s_;
    int    climate_max_miss_;

    static constexpr uint32_t kFields = 3;  // temperature, pressure, humidity
    uint8_t                   miss_[N_JOINTS][kFields] = {};
};

}  // namespace reach

#endif  // N_DRIVER_ARM_H
