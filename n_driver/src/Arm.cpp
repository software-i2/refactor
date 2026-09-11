// Copyright by BeeX [2026]

#include <n_driver/Arm.h>
#include <n_driver/Time.h>

#include <iostream>
#include <thread>

namespace reach {
namespace {

constexpr uint8_t kClimateField[3] = {Pkt::TEMP, Pkt::PRESSURE, Pkt::HUMIDITY};

}  // namespace

Arm::Arm(std::shared_ptr<Port> port, double reply_timeout_s, double climate_timeout_s,
         int climate_max_miss)
        : port_(std::move(port)),
          reply_timeout_s_(reply_timeout_s),
          climate_timeout_s_(climate_timeout_s),
          climate_max_miss_(climate_max_miss) {
    if (!port_) {
        std::cerr << "[Arm] no port." << std::endl;
    }
}

bool Arm::send(uint8_t device, uint8_t id, const std::vector<uint8_t> &data) {
    if (!port_) {
        return false;
    }
    const std::vector<uint8_t> frame = encode(device, id, data);
    const int written = port_->write(frame.data(), frame.size());
    if (written != static_cast<int>(frame.size())) {
        std::cerr << "[Arm] wrote " << written << " of " << frame.size() << " bytes." << std::endl;
        return false;
    }
    return true;
}

bool Arm::ask(uint8_t device, uint8_t field, double timeout, Packet &out) {
    if (!port_ || !port_->ok() || !send(device, Pkt::REQUEST, {field})) {
        return false;
    }

    const double deadline = nowSec() + timeout;
    uint8_t      buf[256];

    while (nowSec() < deadline) {
        const int n = port_->read(buf, sizeof(buf));
        if (n <= 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }
        for (const Packet &pkt : reader_.feed(buf, static_cast<size_t>(n))) {
            if (pkt.device == device && pkt.id == field) {
                out = pkt;
                return true;
            }
        }
    }
    return false;
}

void Arm::move(uint32_t j, float pos) {
    std::lock_guard<std::mutex> lock(mutex_);
    send(DEVICE[j], Pkt::POSITION, encodeFloat(toWireRad(j, pos)));
}

void Arm::jog(uint32_t j, float vel) {
    std::lock_guard<std::mutex> lock(mutex_);
    send(DEVICE[j], Pkt::VELOCITY, encodeFloat(toWireRad(j, vel)));
}

void Arm::standby(uint32_t j) {
    std::lock_guard<std::mutex> lock(mutex_);
    send(DEVICE[j], Pkt::MODE, {static_cast<uint8_t>(Mode::STANDBY)});
}

bool Arm::position(uint32_t j, float &pos) {
    std::lock_guard<std::mutex> lock(mutex_);
    Packet pkt;
    if (!ask(DEVICE[j], Pkt::POSITION, reply_timeout_s_, pkt) || pkt.data.size() < 4) {
        return false;
    }
    pos = toPub(j, decodeFloat(pkt.data));
    return true;
}

Mode Arm::mode(uint32_t j) {
    std::lock_guard<std::mutex> lock(mutex_);
    Packet pkt;
    if (!ask(DEVICE[j], Pkt::MODE, reply_timeout_s_, pkt) || pkt.data.empty()) {
        return Mode::UNKNOWN;
    }

    switch (pkt.data[0]) {
    case 0x00:
        return Mode::STANDBY;
    case 0x01:
        return Mode::DISABLE;
    case 0x02:
        return Mode::POSITION;
    case 0x03:
        return Mode::VELOCITY;
    case 0x26:
        return Mode::PASSIVE;
    default:
        return Mode::UNKNOWN;
    }
}

bool Arm::climate(uint32_t j, Climate &out) {
    std::lock_guard<std::mutex> lock(mutex_);

    float values[kFields] = {0.0f, 0.0f, 0.0f};
    bool  answered        = false;

    for (uint32_t f = 0; f < kFields; ++f) {
        if (miss_[j][f] >= climate_max_miss_) {
            continue;
        }

        Packet pkt;
        if (ask(DEVICE[j], kClimateField[f], climate_timeout_s_, pkt) && pkt.data.size() >= 4) {
            values[f]  = decodeFloat(pkt.data);
            miss_[j][f] = 0;
            answered   = true;
        } else {
            ++miss_[j][f];
        }
    }

    out.temp     = values[0];
    out.pressure = values[1];
    out.humidity = values[2];
    return answered;
}

bool Arm::ping(int tries) {
    if (!port_ || !port_->ok()) {
        std::cerr << "[Arm] port is not open." << std::endl;
        return false;
    }

    for (int i = 0; i < tries; ++i) {
        {
            std::lock_guard<std::mutex> lock(mutex_);

            // Wake the joint with a tiny velocity nudge.
            send(DEVICE[WRIST], Pkt::VELOCITY, encodeFloat(0.01f));

            Packet pkt;
            if (ask(DEVICE[WRIST], Pkt::MODE, 0.1, pkt) && !pkt.data.empty() &&
                pkt.data[0] == static_cast<uint8_t>(Mode::VELOCITY)) {
                std::cout << "[Arm] connected, wrist answering." << std::endl;
                return true;
            }
        }

        std::cout << "[Arm] no answer, attempt " << (i + 1) << " of " << tries << "." << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(2));
    }

    std::cerr << "[Arm] arm did not answer." << std::endl;
    return false;
}

}  // namespace reach
