#pragma once

#include <entt/entt.hpp>

namespace sfs::render {

struct RenderDot {
    float size;
};

void initRenderDotSystem();
void renderDots(entt::registry &registry, entt::entity camera);

} // namespace sfs::render
