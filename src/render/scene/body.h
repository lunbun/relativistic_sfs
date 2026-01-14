#pragma once

#include <entt/entt.hpp>

namespace sfs::render {

struct RenderBody {
    float radius;
};

void initRenderBodySystem();
void renderBodies(entt::registry &registry, entt::entity camera);

} // namespace sfs::render
