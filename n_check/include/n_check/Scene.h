// Copyright by BeeX [2026]

#ifndef N_CHECK_SCENE_H
#define N_CHECK_SCENE_H

#include <n_check/Body.h>
#include <n_check/Field.h>
#include <n_kine/Geom.h>

#include <memory>
#include <string>
#include <vector>

namespace check {

// One line of the load report, at the level the caller should log it.
struct Note {
    enum Level { INFO, WARN, ERROR };

    Level       level = INFO;
    std::string text;
};

// Opens the field named by `path` (empty means none), builds the Body at the
// pitch that field fixes, and says what it found. `safe` is a posture known to
// be reachable, used to catch a field with the arm itself in it; it is only
// consulted when the geometry solves.
//
// Every node that checks paths opens the same file the same way, so the
// sequence lives here rather than once per node, where the two could drift.
std::vector<Note> openScene(const std::string &path,
                            const Jaws &jaws,
                            const kine::Geom &g,
                            const kine::Joints &safe,
                            Field &field,
                            std::unique_ptr<Body> &body);

}  // namespace check

#endif  // N_CHECK_SCENE_H
