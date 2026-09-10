// Copyright by BeeX [2026]

#include <n_driver/Serial.h>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <iostream>
#include <cerrno>

namespace reach {
namespace {

speed_t speedOf(unsigned int baud) {
    switch (baud) {
    case 9600:
        return B9600;
    case 19200:
        return B19200;
    case 38400:
        return B38400;
    case 57600:
        return B57600;
    case 230400:
        return B230400;
    default:
        return B115200;
    }
}

}  // namespace

Serial::Serial(const std::string &device, unsigned int baud) {
    fd_ = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
        std::cerr << "[Serial] cannot open " << device << ": " << strerror(errno) << std::endl;
        return;
    }

    struct termios tty {};
    if (tcgetattr(fd_, &tty) != 0) {
        std::cerr << "[Serial] tcgetattr: " << strerror(errno) << std::endl;
        ::close(fd_);
        fd_ = -1;
        return;
    }

    // Raw mode: any line discipline would corrupt binary BPL bytes.
    cfmakeraw(&tty);
    cfsetispeed(&tty, speedOf(baud));
    cfsetospeed(&tty, speedOf(baud));
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 0;

    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
        std::cerr << "[Serial] tcsetattr: " << strerror(errno) << std::endl;
        ::close(fd_);
        fd_ = -1;
        return;
    }

    tcflush(fd_, TCIOFLUSH);
    std::cout << "[Serial] " << device << " open at " << baud << " baud." << std::endl;
}

Serial::~Serial() {
    if (fd_ >= 0) {
        ::close(fd_);
    }
}

bool Serial::ok() const { return fd_ >= 0; }

int Serial::write(const uint8_t *data, size_t len) {
    if (fd_ < 0) {
        return -1;
    }
    const ssize_t n = ::write(fd_, data, len);
    if (n < 0) {
        std::cerr << "[Serial] write: " << strerror(errno) << std::endl;
        return -1;
    }
    return static_cast<int>(n);
}

int Serial::read(uint8_t *buf, size_t len) {
    if (fd_ < 0) {
        return -1;
    }
    const ssize_t n = ::read(fd_, buf, len);
    if (n < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            return 0;
        }
        std::cerr << "[Serial] read: " << strerror(errno) << std::endl;
        return -1;
    }
    return static_cast<int>(n);
}

}  // namespace reach
