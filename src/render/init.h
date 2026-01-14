#pragma once

#include "render/scene/body.h"
#include "render/scene/dot.h"
#include "render/scene/trajectory.h"

namespace sfs::render {

inline void initRenderSystem() {
    initRenderBodySystem();
    initRenderDotSystem();
    initRenderTrajectorySystem();
}

} // namespace sfs::render
