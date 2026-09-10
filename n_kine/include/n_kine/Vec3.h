// Copyright by BeeX [2026]

#ifndef N_KINE_VEC3_H
#define N_KINE_VEC3_H

#include <cmath>

namespace kine {

struct Vec3 {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

inline Vec3 operator+(const Vec3 &a, const Vec3 &b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
inline Vec3 operator-(const Vec3 &a, const Vec3 &b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
inline Vec3 operator*(const Vec3 &a, double s) { return {a.x * s, a.y * s, a.z * s}; }

inline double dot(const Vec3 &a, const Vec3 &b) { return a.x * b.x + a.y * b.y + a.z * b.z; }

inline double norm(const Vec3 &a) { return std::sqrt(dot(a, a)); }

inline Vec3 unit(const Vec3 &a) {
    const double n = norm(a);
    return n > 1e-12 ? a * (1.0 / n) : Vec3();
}

// Radians between two directions; pi if either is degenerate.
inline double angleTo(const Vec3 &a, const Vec3 &b) {
    const double na = norm(a);
    const double nb = norm(b);
    if (na < 1e-12 || nb < 1e-12) {
        return M_PI;
    }
    const double c = dot(a, b) / (na * nb);
    return std::acos(c < -1.0 ? -1.0 : (c > 1.0 ? 1.0 : c));
}

// Turn about +z, the base joint's axis.
inline Vec3 spin(const Vec3 &v, double cb, double sb) {
    return {v.x * cb - v.y * sb, v.x * sb + v.y * cb, v.z};
}

}  // namespace kine

#endif  // N_KINE_VEC3_H
