// Copyright by BeeX [2026]

#ifndef N_DRIVER_TIME_H
#define N_DRIVER_TIME_H

#include <chrono>

namespace reach {

// Monotonic seconds, for timeouts and periods.
inline double nowSec() {
    using namespace std::chrono;
    return duration_cast<duration<double>>(steady_clock::now().time_since_epoch()).count();
}

}  // namespace reach

#endif  // N_DRIVER_TIME_H
