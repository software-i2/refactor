// Copyright by BeeX [2026]

#ifndef N_CHECK_BODY_H
#define N_CHECK_BODY_H

#include <n_check/Field.h>
#include <n_conf/Doc.h>
#include <n_kine/Fk.h>
#include <n_kine/Geom.h>

#include <string>
#include <vector>

namespace check {

struct Jaws {
    double palm_radius_m = kine::NONE;
    double palm_length_m = kine::NONE;

    double hinge_xyz[3]    = {kine::NONE, kine::NONE, kine::NONE};
    double blade_size[3]   = {kine::NONE, kine::NONE, kine::NONE};
    double blade_origin[3] = {kine::NONE, kine::NONE, kine::NONE};
    double open_mm         = kine::NONE;

    static constexpr int PROFILE_BANDS = 18;
    std::vector<double>  blade_profile;

    double grip_margin_m = kine::NONE;

    static constexpr double HINGE_RAD_PER_MM = 51.0 / 1000.0;
    double openRad() const { return open_mm * HINGE_RAD_PER_MM; }

    static double bladePitch(double field_res_m) { return 0.5 * field_res_m; }

    void load(conf::Doc &doc) {
        palm_radius_m = doc.num("jaws.palm_radius_m");
        palm_length_m = doc.num("jaws.palm_length_m");
        doc.nums("jaws.hinge_xyz", hinge_xyz, 3);
        doc.nums("jaws.blade_size", blade_size, 3);
        doc.nums("jaws.blade_origin", blade_origin, 3);
        open_mm       = doc.num("jaws.open_mm");
        blade_profile.assign(PROFILE_BANDS * 5, kine::NONE);
        doc.nums("jaws.blade_profile", &blade_profile[0], blade_profile.size());
        grip_margin_m = doc.num("jaws.grip_margin_m");
    }

    const char *missing() const;
};

enum Vol : int {
    V_UPPER_ARM = 0,
    V_FOREARM,
    V_WRIST_MOUNT,
    V_PALM,
    V_BLADE_LEFT,
    V_BLADE_RIGHT,
    N_VOLS
};

const char *name(int vol);

class Body {
public:
    Body(const Jaws &j, double blade_pitch_m);

    struct Volume {
        kine::Vec3 shoulder, elbow, wrist, mount, palm_end, throat, tip;
        const kine::Vec3 *blades = NULL;
        size_t            left  = 0;
        size_t            count = 0;
    };

    void volume(const kine::Geom &g, const kine::Joints &q,
                std::vector<kine::Vec3> &buf, Volume &out) const;

    const Jaws &jaws() const { return j_; }
    double bladePitchM() const { return pitch_; }
    size_t latticeSize() const { return lattice_.size(); }

    double throatHalfGap(double depth) const;
    double maxDepth() const;

private:
    void buildLattice();

    Jaws   j_;
    double pitch_  = 0.0;
    bool   box_ok_ = false;

    double centre_lat_ = 0.0, centre_ax_ = 0.0;
    double close_lat_ = 0.0, close_ax_ = 0.0;
    double appr_lat_ = 0.0, appr_ax_ = 0.0;
    double half_[3] = {0.0, 0.0, 0.0};

    std::vector<kine::Vec3> lattice_;
};

double lowestZ(const Body::Volume &v);
int firstBlocked(const Field &f, const Body::Volume &v);
bool blocksItself(const Field &f, const Body &b, const kine::Geom &g,
                  const kine::Joints &safe, std::string &why);
bool fitsField(const Body &b, const Field &f, std::string &why);

}  // namespace check

#endif  // N_CHECK_BODY_H
