// Copyright by BeeX [2026]

#include <n_kine/Angle.h>
#include <n_kine/Geom.h>

#include <algorithm>
#include <cmath>

namespace kine {
namespace {

constexpr double kMinLink = 1e-6;
constexpr double kYawEps  = 1e-3;

}  // namespace

Geom::Geom(const Params &p) : p_(p) {
    // An unfilled field would otherwise sail through as NaN and come back out
    // as a target the arm cannot reach, with nothing saying why.
    fault_ = p_.missing();
    if (fault_ != NULL) {
        return;
    }

    // The elbow frame yaw must be 0 or pi; anything else is not a planar arm.
    const double yaw = std::fabs(wrapPi(deg2rad(p_.d_to_c_yaw_deg)));
    if (yaw < kYawEps) {
        elbow_sign_ = 1.0;
    } else if (std::fabs(yaw - M_PI) < kYawEps) {
        elbow_sign_ = -1.0;
    } else {
        fault_ = "arm.d_to_c_yaw_deg is neither 0 nor 180, so this is not a planar arm";
        return;
    }

    upper_.len = std::hypot(p_.d_to_c_x, p_.d_to_c_z);
    upper_.psi = std::atan2(p_.d_to_c_x, p_.d_to_c_z);

    x2_              = elbow_sign_ * p_.c_to_b_x;
    forearm_raw_.len = std::hypot(p_.c_to_b_x, p_.c_to_b_z);
    forearm_raw_.psi = std::atan2(x2_, p_.c_to_b_z);

    if (upper_.len < kMinLink || forearm_raw_.len < kMinLink) {
        fault_ = "the upper arm or forearm has no length";
        return;
    }

    hinge_offset_rad_ = deg2rad(p_.hinge_offset_deg);

    // Folded here rather than per solve: aiming the throat is what IK does all
    // day, and hypot plus atan2 on every call is most of a solve's cost.
    forearm_throat_ = fold(throatAlong());
    forearm_tip_    = fold(tipAlong());

    for (int j = 0; j < DOF; ++j) {
        double lo = deg2rad(p_.limit_lo_deg[j]);
        double hi = deg2rad(p_.limit_hi_deg[j]);

        // The hardware window in kinematic terms. A negative sign flips it end
        // for end, which is what made reversed joints reject every branch before.
        const double sign = p_.direction_sign[j];
        if (std::fabs(sign) > kMinLink) {
            const double a = toKinematic(p_, j, p_.wire_lo_deg[j]);
            const double b = toKinematic(p_, j, p_.wire_hi_deg[j]);
            lo = std::max(lo, std::min(a, b));
            hi = std::min(hi, std::max(a, b));
        }

        win_lo_[j] = lo;
        win_hi_[j] = hi;

        if (lo > hi) {
            fault_ = "a joint's limits and the hardware window do not overlap";
            return;
        }
    }

    fault_ = NULL;
    ok_    = true;
}

// Reported by name so a half-filled config says which key, not just "invalid".
const char *Params::missing() const {
    struct Field {
        const char   *name;
        const double *at;
        int           count;
    };
    const Field fields[] = {
            {"arm.base_to_e_z", &base_to_e_z, 1},
            {"arm.e_to_d_x", &e_to_d_x, 1},
            {"arm.e_to_d_z", &e_to_d_z, 1},
            {"arm.d_to_c_x", &d_to_c_x, 1},
            {"arm.d_to_c_z", &d_to_c_z, 1},
            {"arm.d_to_c_yaw_deg", &d_to_c_yaw_deg, 1},
            {"arm.c_to_b_x", &c_to_b_x, 1},
            {"arm.c_to_b_z", &c_to_b_z, 1},
            {"arm.b_to_mount", &b_to_mount, 1},
            {"arm.mount_to_throat", &mount_to_throat, 1},
            {"arm.mount_to_tip", &mount_to_tip, 1},
            {"arm.hinge_offset_deg", &hinge_offset_deg, 1},
            {"arm.limit_lo_deg", limit_lo_deg, DOF},
            {"arm.limit_hi_deg", limit_hi_deg, DOF},
            {"arm.zero_offset_deg", zero_offset_deg, DOF},
            {"arm.direction_sign", direction_sign, DOF},
            {"driver.limits.*.min (as arm wire_lo_deg)", wire_lo_deg, DOF},
            {"driver.limits.*.max (as arm wire_hi_deg)", wire_hi_deg, DOF},
            {"arm.weight", weight, DOF},
    };

    for (const Field &f : fields) {
        for (int i = 0; i < f.count; ++i) {
            if (!std::isfinite(f.at[i])) {
                return f.name;
            }
        }
    }
    return NULL;
}

Link Geom::fold(double along) const {
    const double z = p_.c_to_b_z + along;
    Link l;
    l.len = std::hypot(x2_, z);
    l.psi = std::atan2(x2_, z);
    return l;
}

Link Geom::forearm(double along) const {
    if (along == throatAlong()) {
        return forearm_throat_;
    }
    if (along == tipAlong()) {
        return forearm_tip_;
    }
    return fold(along);
}

double Geom::reachMax(double along) const { return upper_.len + forearm(along).len; }

double Geom::reachMin(double along) const { return std::fabs(upper_.len - forearm(along).len); }

}  // namespace kine
