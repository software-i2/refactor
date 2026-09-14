// Copyright by BeeX [2026]

#include <n_check/Scene.h>

#include <cstdio>

namespace check {
namespace {

void say(std::vector<Note> &out, Note::Level level, const std::string &text) {
    Note note;
    note.level = level;
    note.text  = text;
    out.push_back(note);
}

bool openField(const std::string &path, Field &field, std::vector<Note> &out) {
    if (path.empty()) {
        say(out, Note::WARN,
            "NO OBSTACLE FIELD. Paths are checked against the safety floor and the joint "
            "limits only, so the arm WILL drive through anything the camera saw. Point "
            "world.obstacle_field at one written by n_pcloud scene.py --out.");
        return false;
    }

    std::string err;
    if (!field.load(path, err)) {
        say(out, Note::ERROR, "obstacle field: " + err + ". Carrying on with the floor only.");
        return false;
    }

    const kine::Vec3 lo = field.lo();
    const kine::Vec3 hi = field.hi();
    const kine::Vec3 at  = field.placement();
    const kine::Vec3 rpy = field.placementRpy();
    char             buf[416];
    char             tilt[96] = "";

    if (rpy.x != 0.0 || rpy.y != 0.0 || rpy.z != 0.0) {
        std::snprintf(tilt, sizeof(tilt), " --rpy \"%g %g %g\"", rpy.x, rpy.y, rpy.z);
    }

    std::snprintf(buf, sizeof(buf), "obstacle field on: %s, %.1f MB, %.0f mm voxels, %llu occupied",
                  field.source().c_str(), field.bytes() / 1e6, field.res() * 1000.0,
                  static_cast<unsigned long long>(field.occupied()));
    say(out, Note::INFO, buf);

    std::snprintf(buf, sizeof(buf), "  spans x %.3f..%.3f  y %.3f..%.3f  z %.3f..%.3f in arm_base",
                  lo.x, hi.x, lo.y, hi.y, lo.z, hi.z);
    say(out, Note::INFO, buf);

    std::snprintf(buf, sizeof(buf),
                  "  built from %016llx, placed at (%.3f, %.3f, %.3f). Confirm it is current "
                  "with: rosrun n_pcloud scene.py --scene <id> --at \"%.3f %.3f %.3f\"%s --check %s",
                  static_cast<unsigned long long>(field.digest()), at.x, at.y, at.z,
                  at.x, at.y, at.z, tilt, field.source().c_str());
    say(out, Note::INFO, buf);

    if (field.empty()) {
        say(out, Note::WARN,
            "  the field loaded but every voxel is free -- check the placement it was "
            "exported with.");
    }
    return true;
}

}  // namespace

std::vector<Note> openScene(const std::string &path,
                            const Jaws &jaws,
                            const kine::Geom &g,
                            const kine::Joints &safe,
                            Field &field,
                            std::unique_ptr<Body> &body) {

    std::vector<Note> out;
    const bool        loaded = openField(path, field, out);

    // The field fixes the blade sampling pitch; without a field there is nothing
    // to sample against, so no lattice is built.
    const double pitch = loaded ? Jaws::bladePitch(field.res()) : 0.0;
    body.reset(new Body(jaws, pitch));
    if (!loaded) {
        return out;
    }

    char buf[128];
    std::snprintf(buf, sizeof(buf), "  blades sampled every %.1f mm, %u samples per posture",
                  pitch * 1000.0, static_cast<uint32_t>(body->latticeSize()));
    say(out, Note::INFO, buf);

    std::string why;
    if (!fitsField(*body, field, why)) {
        say(out, Note::ERROR, "  " + why);
    }

    // If the rest pose is rejected by the field, the field is likely wrong rather
    // than the pose; this check explains why.
    if (g.ok() && blocksItself(field, *body, g, safe, why)) {
        say(out, Note::ERROR, "  UNUSABLE FIELD: " + why);
    }
    return out;
}

}  // namespace check
