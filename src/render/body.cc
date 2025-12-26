#include "body.h"

// clang-format off
#include <glad/gl.h>
#include <GLFW/glfw3.h>
#include <entt/entt.hpp>
// clang-format on

#include "physics/kepler.h"
#include "physics/physics.h"

constexpr double kRenderScale = 1.0 / 1.0e12;

void renderBodies(entt::registry &registry) {
    auto largeView = registry.view<BodyState, RenderDotLarge>();
    auto smallView = registry.view<BodyState, RenderDotSmall>();
    glPointSize(15.0f);
    glBegin(GL_POINTS);
    for (auto entity : largeView) {
        auto &body = largeView.get<BodyState>(entity);
        Eigen::Vector3d screenPos = body.st.pos * kRenderScale;
        glVertex2d(screenPos.x(), screenPos.y());
    }
    glEnd();

    glPointSize(5.0f);
    glBegin(GL_POINTS);
    for (auto entity : smallView) {
        auto &body = smallView.get<BodyState>(entity);
        Eigen::Vector3d screenPos = body.st.pos * kRenderScale;
        glVertex2d(screenPos.x(), screenPos.y());
    }
    glEnd();
}

void renderTrajectories(entt::registry &registry) {
    constexpr int n = 250;
    auto view = registry.view<Body, KeplerParameters, RenderTrajectory>();
    glLineWidth(1.0f);
    std::vector<Eigen::Vector3d> points;
    points.reserve(n);
    for (auto entity : view) {
        auto &body = view.get<Body>(entity);
        if (body.primary == entt::null) continue;

        auto &primaryState = registry.get<BodyState>(body.primary);
        auto &p = view.get<KeplerParameters>(entity);
        points.clear();
        sampleTrajectoryPoints(p, points, n);

        glBegin(GL_LINE_STRIP);
        for (const auto &pt : points) {
            Eigen::Vector3d screenPos = (primaryState.st.pos + pt) * kRenderScale;
            glVertex2d(screenPos.x(), screenPos.y());
        }
        glEnd();
    }
}
