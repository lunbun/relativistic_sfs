#pragma once

#include "render/scene/dot.h"
#include "render/scene/trajectory.h"

namespace sfs::render {

inline void initRenderSystem() {
    initRenderDotSystem();
    initRenderTrajectorySystem();
}

} // namespace sfs::render
