// Copyright by BeeX [2026]

#ifndef N_DRIVER_SIM_H
#define N_DRIVER_SIM_H

#include <n_driver/Joints.h>
#include <n_driver/Packet.h>
#include <n_driver/Port.h>

#include <deque>
#include <mutex>
#include <vector>

namespace reach {

// Answers the same packets as the arm, walks each joint to its target.
class Sim : public Port {
public:
    // Wire units per second: radians for the rotary joints, mm for the jaw.
    // `limits` gives the rest pose it starts at and the stops it walks between.
    Sim(double joint_speed, double jaw_speed, const Limits &limits);

    bool ok() const override;
    int write(const uint8_t *data, size_t len) override;
    int read(uint8_t *buf, size_t len) override;

private:
    struct SimJoint {
        float   position = 0.0f;
        float   target   = 0.0f;
        float   velocity = 0.0f;
        uint8_t mode     = static_cast<uint8_t>(Mode::STANDBY);
    };

    void step();
    void handle(const Packet &pkt);
    void answer(uint8_t device, uint32_t j, uint8_t field);
    void reply(uint8_t device, uint8_t id, const std::vector<uint8_t> &data);

    SimJoint joints_[N_JOINTS];
    Limits   limits_;
    double   joint_speed_;
    double   jaw_speed_;
    double   last_step_s_;

    Reader              reader_;
    std::deque<uint8_t> tx_;
    mutable std::mutex  mutex_;
};

}  // namespace reach

#endif  // N_DRIVER_SIM_H
