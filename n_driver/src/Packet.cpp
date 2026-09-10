// Copyright by BeeX [2026]

#include <n_driver/Packet.h>

#include <algorithm>
#include <array>
#include <cstring>

namespace reach {
namespace {

const std::array<uint8_t, 256> kCrcTable = []() {
    std::array<uint8_t, 256> t{};
    for (int i = 0; i < 256; ++i) {
        uint8_t v = static_cast<uint8_t>(i);
        for (int j = 0; j < 8; ++j) {
            v = (v & 1u) ? static_cast<uint8_t>((v >> 1) ^ 0xB2u) : static_cast<uint8_t>(v >> 1);
        }
        t[i] = v;
    }
    return t;
}();

// crcmod(0x14D, init 0xFF, xorOut 0xFF, reversed) pre-XORs init with xorOut, so this starts at 0.
uint8_t crc8(const uint8_t *data, size_t len) {
    uint8_t crc = 0x00;
    for (size_t i = 0; i < len; ++i) {
        crc = kCrcTable[crc ^ data[i]];
    }
    return crc ^ 0xFF;
}

std::vector<uint8_t> cobsEncode(const std::vector<uint8_t> &in) {
    std::vector<uint8_t> out;
    out.reserve(in.size() + in.size() / 254 + 2);

    size_t  marker = 0;
    uint8_t code   = 1;
    out.push_back(0x00);

    for (uint8_t byte : in) {
        if (byte != 0x00) {
            out.push_back(byte);
            if (++code != 0xFF) {
                continue;
            }
        }
        out[marker] = code;
        marker      = out.size();
        out.push_back(0x00);
        code = 1;
    }
    out[marker] = code;
    return out;
}

std::vector<uint8_t> cobsDecode(const std::vector<uint8_t> &in) {
    std::vector<uint8_t> out;
    out.reserve(in.size());

    size_t i = 0;
    while (i < in.size()) {
        const uint8_t code = in[i++];
        for (uint8_t k = 1; k < code; ++k, ++i) {
            if (i >= in.size()) {
                return {};  // malformed
            }
            out.push_back(in[i]);
        }
        // Every group but the last stands for a zero byte.
        if (code != 0xFF && i < in.size()) {
            out.push_back(0x00);
        }
    }
    return out;
}

// Trailer is [.. payload .., id, device, length, crc].
bool parse(const std::vector<uint8_t> &raw, Packet &out) {
    const std::vector<uint8_t> body = cobsDecode(raw);
    if (body.size() < 4 || body[body.size() - 2] != static_cast<uint8_t>(body.size())) {
        return false;
    }
    if (crc8(body.data(), body.size() - 1) != body.back()) {
        return false;
    }

    out.device = body[body.size() - 3];
    out.id     = body[body.size() - 4];
    out.data.assign(body.begin(), body.end() - 4);
    return true;
}

}  // namespace

std::vector<uint8_t> encode(uint8_t device, uint8_t id, const std::vector<uint8_t> &data) {
    std::vector<uint8_t> body = data;
    body.push_back(id);
    body.push_back(device);
    body.push_back(static_cast<uint8_t>(data.size() + 4));
    body.push_back(crc8(body.data(), body.size()));

    std::vector<uint8_t> frame = cobsEncode(body);
    frame.push_back(0x00);
    return frame;
}

std::vector<uint8_t> encodeFloat(float value) {
    std::vector<uint8_t> bytes(4);
    std::memcpy(bytes.data(), &value, 4);
    return bytes;
}

float decodeFloat(const std::vector<uint8_t> &data) {
    float value = 0.0f;
    if (data.size() >= 4) {
        std::memcpy(&value, data.data(), 4);
    }
    return value;
}

std::vector<Packet> Reader::feed(const uint8_t *data, size_t len) {
    std::vector<Packet> packets;
    buf_.insert(buf_.end(), data, data + len);

    for (;;) {
        const auto end = std::find(buf_.begin(), buf_.end(), uint8_t{0x00});
        if (end == buf_.end()) {
            break;
        }

        const std::vector<uint8_t> raw(buf_.begin(), end);
        buf_.erase(buf_.begin(), end + 1);

        Packet pkt;
        if (!raw.empty() && parse(raw, pkt)) {
            packets.push_back(pkt);
        }
    }
    return packets;
}

}  // namespace reach
