#pragma once

#include <entt/entt.hpp>

#include "render/camera.h"

struct RenderDot {
    float size;
};
struct RenderTrajectory { };

void initRenderBodySystem();
void renderBodies(entt::registry &registry, entt::entity camera);
void renderTrajectories(entt::registry &registry, entt::entity camera);
