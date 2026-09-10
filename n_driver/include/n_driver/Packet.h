// Copyright by BeeX [2026]

#ifndef N_DRIVER_PACKET_H
#define N_DRIVER_PACKET_H

#include <cstddef>
#include <cstdint>
#include <vector>

namespace reach {

// BPL packet ids this driver uses.
namespace Pkt {
constexpr uint8_t MODE     = 0x01;
constexpr uint8_t VELOCITY = 0x02;
constexpr uint8_t POSITION = 0x03;
constexpr uint8_t HUMIDITY = 0x65;
constexpr uint8_t TEMP     = 0x66;
constexpr uint8_t PRESSURE = 0x6E;
constexpr uint8_t REQUEST  = 0x60;
}  // namespace Pkt

enum class Mode : uint8_t {
    STANDBY  = 0x00,
    DISABLE  = 0x01,
    POSITION = 0x02,
    VELOCITY = 0x03,
    PASSIVE  = 0x26,
    UNKNOWN  = 0xFF
};

struct Packet {
    uint8_t device = 0;
    uint8_t id     = 0;
    std::vector<uint8_t> data;
};

// One COBS frame: payload, ids, length, CRC8, delimiter.
std::vector<uint8_t> encode(uint8_t device, uint8_t id, const std::vector<uint8_t> &data);

std::vector<uint8_t> encodeFloat(float value);

// 0 if the payload is short.
float decodeFloat(const std::vector<uint8_t> &data);

// Buffers incoming bytes, hands back whole packets.
class Reader {
public:
    std::vector<Packet> feed(const uint8_t *data, size_t len);

private:
    std::vector<uint8_t> buf_;
};

}  // namespace reach

#endif  // N_DRIVER_PACKET_H
