#pragma once

#include "render/scene/body.h"
#include "render/scene/trajectory.h"

inline void initRenderSystem() {
    initRenderBodySystem();
    initRenderTrajectorySystem();
}
