// Copyright by BeeX [2026]

#include <n_check/Body.h>

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace check {

const char *name(int vol) {
    switch (vol) {
    case V_UPPER_ARM:
        return "upper arm";
    case V_FOREARM:
        return "forearm";
    case V_WRIST_MOUNT:
        return "wrist mount";
    case V_PALM:
        return "palm";
    case V_BLADE_LEFT:
        return "left jaw";
    case V_BLADE_RIGHT:
        return "right jaw";
    default:
        return "nothing";
    }
}

Body::Body(const Jaws &j, double blade_pitch_m) : j_(j), pitch_(blade_pitch_m) {
    buildLattice();
}

const char *Jaws::missing() const {
    struct Field {
        const char   *name;
        const double *at;
        int           count;
    };
    const Field fields[] = {
            {"jaws.palm_radius_m", &palm_radius_m, 1},
            {"jaws.palm_length_m", &palm_length_m, 1},
            {"jaws.hinge_xyz", hinge_xyz, 3},
            {"jaws.blade_size", blade_size, 3},
            {"jaws.blade_origin", blade_origin, 3},
            {"jaws.open_mm", &open_mm, 1},
            {"jaws.grip_margin_m", &grip_margin_m, 1},
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

// Geometry note: blade_origin is expressed in the hinge frame, not the EE frame.
// Using the EE-frame convention flips the jaw orientation and misreads the throat.
void Body::buildLattice() {
    lattice_.clear();

    if (j_.missing() != NULL) {
        return;
    }

    half_[0] = 0.5 * j_.blade_size[0];
    half_[1] = 0.5 * j_.blade_size[1];
    half_[2] = 0.5 * j_.blade_size[2];
    if (half_[0] <= 0.0 || half_[1] <= 0.0 || half_[2] <= 0.0) {
        return;
    }

    const double open = j_.openRad();
    const double c    = std::cos(open);
    const double s    = std::sin(open);

    centre_lat_ = j_.hinge_xyz[1] + j_.blade_origin[1] * c + j_.blade_origin[2] * s;
    centre_ax_  = j_.hinge_xyz[2] - j_.blade_origin[1] * s + j_.blade_origin[2] * c;
    close_lat_ = c;  close_ax_ = -s;
    appr_lat_  = s;  appr_ax_  = c;

    box_ok_ = true;

    // Only the lattice needs sampling; without a field there is nothing to sample.
    if (pitch_ <= 0.0) {
        return;
    }

    const double *half = half_;
    const double centre_lat = centre_lat_, centre_ax = centre_ax_;
    const double close_lat = close_lat_, close_ax = close_ax_;
    const double appr_lat = appr_lat_, appr_ax = appr_ax_;

    int n[3];
    for (int a = 0; a < 3; ++a) {
        n[a] = static_cast<int>(std::ceil(2.0 * half[a] / pitch_));
        if (n[a] < 1) {
            n[a] = 1;
        }
    }

    // Left blade first, then right, to keep the side association explicit.
    for (int side = 0; side < 2; ++side) {
        const double mirror = side == 0 ? 1.0 : -1.0;
        for (int i = 0; i <= n[0]; ++i) {
            const double u = -half[0] + 2.0 * half[0] * i / n[0];  // along the hinge
            for (int k = 0; k <= n[1]; ++k) {
                const double v = -half[1] + 2.0 * half[1] * k / n[1];  // closing
                for (int m = 0; m <= n[2]; ++m) {
                    const double w = -half[2] + 2.0 * half[2] * m / n[2];  // approach
                    kine::Vec3   p;
                    p.x = centre_ax + v * close_ax + w * appr_ax;   // approach
                    p.y = u;                                        // hinge
                    p.z = mirror * (centre_lat + v * close_lat + w * appr_lat);
                    lattice_.push_back(p);
                }
            }
        }
    }
}

void Body::volume(const kine::Geom &g, const kine::Joints &q,
                  std::vector<kine::Vec3> &buf, Volume &out) const {

    const kine::Pose  p = kine::forward(g, q);
    const kine::Frame f = kine::frame(g, q);

    out.shoulder = p.shoulder;
    out.elbow    = p.elbow;
    out.wrist    = p.wrist;
    out.mount    = p.mount;
    out.palm_end = p.mount + f.approach * j_.palm_length_m;

    buf.clear();
    out.blades = NULL;
    out.left   = 0;
    out.count  = 0;
    if (lattice_.empty()) {
        return;
    }

    // The lattice is relative to the EE origin, so the blades ride the wrist directly.
    buf.resize(lattice_.size());
    for (size_t i = 0; i < lattice_.size(); ++i) {
        const kine::Vec3 &e = lattice_[i];
        buf[i] = p.mount + f.approach * e.x + f.hinge * e.y + f.close * e.z;
    }

    out.blades = &buf[0];
    out.left   = buf.size() / 2;
    out.count  = buf.size();
}

// Interpolate along the inner face instead of sampling the whole box; the box root
// crosses the axis and would skew the result toward zero.
double Body::throatHalfGap(double depth) const {
    if (!box_ok_) {
        return 0.0;
    }
    const double lat = centre_lat_ - half_[1] * close_lat_;
    const double ax  = centre_ax_ - half_[1] * close_ax_;

    const double root_lat = lat - half_[2] * appr_lat_;
    const double root_ax  = ax - half_[2] * appr_ax_;
    const double tip_lat  = lat + half_[2] * appr_lat_;
    const double tip_ax   = ax + half_[2] * appr_ax_;

    if (std::fabs(tip_ax - root_ax) < 1e-9) {
        return root_lat;
    }
    return root_lat + (tip_lat - root_lat) * (depth - root_ax) / (tip_ax - root_ax);
}

double lowestZ(const kine::Geom &g, const kine::Joints &q) {
    const kine::Pose p = kine::forward(g, q);
    return std::min({p.shoulder.z, p.elbow.z, p.wrist.z, p.mount.z, p.throat.z, p.tip.z});
}

double Body::maxDepth() const {
    if (!box_ok_) {
        return 0.0;
    }
    const double tip_ax = centre_ax_ - half_[1] * close_ax_ + half_[2] * appr_ax_;
    return tip_ax - j_.grip_margin_m;
}

bool blocksItself(const Field &f, const Body &b, const kine::Geom &g,
                  const kine::Joints &safe, std::string &why) {
    if (!f.ok()) {
        return false;
    }

    std::vector<kine::Vec3> buf;
    Body::Volume            v;
    b.volume(g, safe, buf, v);

    const int hit = firstBlocked(f, v);
    if (hit < 0) {
        return false;
    }

    char msg[320];
    std::snprintf(msg, sizeof(msg),
                  "the field blocks the %s at a posture known to be safe, so it has the arm "
                  "itself in it or was placed against a different mount. Check it with "
                  "n_pcloud scene.py --check %s",
                  name(hit), f.source().c_str());
    why = msg;
    return true;
}

int firstBlocked(const Field &f, const Body::Volume &v) {
    if (!f.ok()) {
        return -1;
    }
    if (f.blockedSegment(v.shoulder, v.elbow, UPPER_ARM)) {
        return V_UPPER_ARM;
    }
    if (f.blockedSegment(v.elbow, v.wrist, FOREARM)) {
        return V_FOREARM;
    }
    if (f.blockedSegment(v.wrist, v.mount, WRIST_MOUNT)) {
        return V_WRIST_MOUNT;
    }
    if (f.blockedSegment(v.mount, v.palm_end, PALM)) {
        return V_PALM;
    }
    if (v.count > 0) {
        if (f.blockedPoints(v.blades, v.left, JAW)) {
            return V_BLADE_LEFT;
        }
        if (f.blockedPoints(v.blades + v.left, v.count - v.left, JAW)) {
            return V_BLADE_RIGHT;
        }
    }
    return -1;
}

bool fitsField(const Body &b, const Field &f, std::string &why) {
    if (!f.ok()) {
        why = "no field loaded";
        return false;
    }

    const double pitch = b.bladePitchM();
    char         buf[288];

    if (pitch <= 0.0) {
        why = "the body was built with no blade pitch, so the jaws are not being checked at all";
        return false;
    }

    // The jaw radius has to cover two hops, not one. A blocked voxel can overlap
    // the blade box while its centre lies outside it: up to res*sqrt(3)/2 from
    // that centre to a point shared with the box, then up to pitch*sqrt(3)/2
    // from there to the nearest sample. Covering only the second is the easy
    // mistake, and it leaves a blade able to pass through a voxel no sample
    // landed in. This is the same bound the exporter sizes the radius with.
    const double needed = (f.res() + pitch) * std::sqrt(3.0) / 2.0;
    if (f.radius(JAW) + 1e-12 < needed) {
        const double sound = 2.0 * f.radius(JAW) / std::sqrt(3.0) - f.res();
        std::snprintf(buf, sizeof(buf),
                      "the blades are sampled every %.1f mm against %.1f mm voxels, which needs a "
                      "jaw radius of %.1f mm, but the field carries %.1f mm. A blade can pass "
                      "through an occupied cell no sample landed in. Sample at %.1f mm or finer, "
                      "or re-export the field for this pitch.",
                      pitch * 1000.0, f.res() * 1000.0, needed * 1000.0,
                      f.radius(JAW) * 1000.0, sound * 1000.0);
        why = buf;
        return false;
    }
    return true;
}

}  // namespace check
