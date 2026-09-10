// Copyright by BeeX [2026]

#ifndef N_DRIVER_PORT_H
#define N_DRIVER_PORT_H

#include <cstddef>
#include <cstdint>

namespace reach {

// Byte transport to the arm.
class Port {
public:
    virtual ~Port() = default;

    virtual bool ok() const = 0;

    // Bytes written, or -1 on error.
    virtual int write(const uint8_t *data, size_t len) = 0;

    // Bytes read, 0 when nothing is waiting, -1 on error. Never blocks.
    virtual int read(uint8_t *buf, size_t len) = 0;
};

}  // namespace reach

#endif  // N_DRIVER_PORT_H
