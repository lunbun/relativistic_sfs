#pragma once

#include <entt/entt.hpp>

namespace sfs::render {

struct RenderTrajectory { };

void initRenderTrajectorySystem();
void renderTrajectories(entt::registry &registry, entt::entity camera);

} // namespace sfs::render
