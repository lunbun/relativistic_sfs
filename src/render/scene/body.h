#pragma once

#include <entt/entt.hpp>

namespace sfs::render {

struct RenderDot {
    float size;
};

void initRenderBodySystem();
void renderBodies(entt::registry &registry, entt::entity camera);

} // namespace sfs::render
