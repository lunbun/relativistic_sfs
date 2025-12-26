#pragma once

#include <entt/entt.hpp>

struct RenderDotLarge { };
struct RenderDotSmall { };
struct RenderTrajectory { };

void renderBodies(entt::registry &registry);
void renderTrajectories(entt::registry &registry);
