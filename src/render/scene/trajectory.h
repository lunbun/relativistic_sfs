#pragma once

#include <entt/entt.hpp>

struct RenderTrajectory { };

void initRenderTrajectorySystem();
void renderTrajectories(entt::registry &registry, entt::entity camera);
