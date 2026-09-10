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

struct Note {
    enum Level { INFO, WARN, ERROR };

    Level       level = INFO;
    std::string text;
};

std::vector<Note> openScene(const std::string &path,
                            const Jaws &jaws,
                            const kine::Geom &g,
                            const kine::Joints &safe,
                            Field &field,
                            std::unique_ptr<Body> &body);

}  // namespace check

#endif  // N_CHECK_SCENE_H
