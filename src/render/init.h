#pragma once

#include "render/scene/body.h"
#include "render/scene/trajectory.h"

namespace sfs::render {

inline void initRenderSystem() {
    initRenderBodySystem();
    initRenderTrajectorySystem();
}

} // namespace sfs::render
