// Copyright by BeeX [2026]

#ifndef N_DRIVER_SERIAL_H
#define N_DRIVER_SERIAL_H

#include <n_driver/Port.h>

#include <string>

namespace reach {

// Raw binary serial line. Opens on construction, closes on destruction.
class Serial : public Port {
public:
    Serial(const std::string &device, unsigned int baud);
    ~Serial() override;

    bool ok() const override;
    int write(const uint8_t *data, size_t len) override;
    int read(uint8_t *buf, size_t len) override;

private:
    int fd_ = -1;
};

}  // namespace reach

#endif  // N_DRIVER_SERIAL_H
