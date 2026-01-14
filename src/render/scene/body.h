#pragma once

#include <entt/entt.hpp>

struct RenderDot {
    float size;
};

void initRenderBodySystem();
void renderBodies(entt::registry &registry, entt::entity camera);
