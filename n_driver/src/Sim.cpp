// Copyright by BeeX [2026]

#include <n_driver/Sim.h>
#include <n_driver/Time.h>

#include <iostream>
#include <algorithm>
#include <cmath>

namespace reach {
namespace {

constexpr float kTemp     = 24.0f;
constexpr float kPressure = 1013.0f;
constexpr float kHumidity = 35.0f;

}  // namespace

// The speeds arrive already checked by the config load, so they are taken as
// given rather than second-guessed with a fallback nothing would ever see.
Sim::Sim(double joint_speed, double jaw_speed, const Limits &limits)
        : limits_(limits),
          joint_speed_(joint_speed),
          jaw_speed_(jaw_speed),
          last_step_s_(nowSec()) {

    for (uint32_t j = 0; j < N_JOINTS; ++j) {
        joints_[j].position = toWire(j, limits_.rest_pos[j]);
        joints_[j].target   = joints_[j].position;
    }
    std::cout << "[Sim] arm simulated, starting at rest." << std::endl;
}

bool Sim::ok() const { return true; }

int Sim::write(const uint8_t *data, size_t len) {
    if (data == nullptr || len == 0) {
        return 0;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    step();
    for (const Packet &pkt : reader_.feed(data, len)) {
        handle(pkt);
    }
    return static_cast<int>(len);
}

int Sim::read(uint8_t *buf, size_t len) {
    if (buf == nullptr || len == 0) {
        return 0;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    step();

    size_t n = 0;
    while (n < len && !tx_.empty()) {
        buf[n++] = tx_.front();
        tx_.pop_front();
    }
    return static_cast<int>(n);
}

void Sim::step() {
    const double now = nowSec();
    double       dt  = now - last_step_s_;
    last_step_s_     = now;

    // A stall on the host must not teleport the arm on the next packet.
    if (dt <= 0.0) {
        return;
    }
    dt = std::min(dt, 1.0);

    for (uint32_t j = 0; j < N_JOINTS; ++j) {
        SimJoint  &joint = joints_[j];
        const float step = static_cast<float>((j == JAW ? jaw_speed_ : joint_speed_) * dt);

        if (joint.mode == static_cast<uint8_t>(Mode::VELOCITY)) {
            joint.position += static_cast<float>(joint.velocity * dt);
            joint.target = joint.position;
            continue;
        }
        if (joint.mode == static_cast<uint8_t>(Mode::DISABLE)) {
            continue;
        }

        const float err = joint.target - joint.position;
        joint.position += std::fabs(err) <= step ? err : (err > 0.0f ? step : -step);
    }
}

void Sim::handle(const Packet &pkt) {
    const int index = jointOf(pkt.device);
    if (index < 0) {
        return;
    }
    const uint32_t j     = static_cast<uint32_t>(index);
    SimJoint      &joint = joints_[j];

    switch (pkt.id) {
    case Pkt::POSITION: {
        // The firmware takes any target and stalls on the stop, so the sim clamps instead.
        const float lo = toWire(j, limits_.min_pos[j]);
        const float hi = toWire(j, limits_.max_pos[j]);
        const float t  = decodeFloat(pkt.data);

        joint.target   = std::min(std::max(t, std::min(lo, hi)), std::max(lo, hi));
        joint.velocity = 0.0f;
        joint.mode     = static_cast<uint8_t>(Mode::POSITION);
        break;
    }
    case Pkt::VELOCITY:
        joint.velocity = decodeFloat(pkt.data);
        joint.mode     = static_cast<uint8_t>(Mode::VELOCITY);
        break;

    case Pkt::MODE:
        if (!pkt.data.empty()) {
            joint.mode     = pkt.data[0];
            joint.velocity = 0.0f;
            joint.target   = joint.position;
        }
        break;

    case Pkt::REQUEST:
        for (uint8_t field : pkt.data) {
            answer(pkt.device, j, field);
        }
        break;

    default:
        break;
    }
}

void Sim::answer(uint8_t device, uint32_t j, uint8_t field) {
    switch (field) {
    case Pkt::POSITION:
        reply(device, Pkt::POSITION, encodeFloat(joints_[j].position));
        break;
    case Pkt::MODE:
        reply(device, Pkt::MODE, {joints_[j].mode});
        break;
    case Pkt::TEMP:
        reply(device, Pkt::TEMP, encodeFloat(kTemp));
        break;
    case Pkt::PRESSURE:
        reply(device, Pkt::PRESSURE, encodeFloat(kPressure));
        break;
    case Pkt::HUMIDITY:
        reply(device, Pkt::HUMIDITY, encodeFloat(kHumidity));
        break;
    default:
        break;
    }
}

void Sim::reply(uint8_t device, uint8_t id, const std::vector<uint8_t> &data) {
    const std::vector<uint8_t> frame = encode(device, id, data);
    tx_.insert(tx_.end(), frame.begin(), frame.end());
}

}  // namespace reach
