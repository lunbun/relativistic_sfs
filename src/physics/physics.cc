#include "physics.h"

#include <cmath>

#include "physics/kepler.h"

namespace {

void gravitySystem(entt::registry &registry) {
    auto view1 = registry.view<ForceAccumulator, BodyState, Body>();
    auto view2 = registry.view<BodyState, Body>();
    for (auto a : view1) {
        auto &stateA = view1.get<BodyState>(a);
        auto &bodyA = view1.get<Body>(a);
        auto &forceAcc = view1.get<ForceAccumulator>(a);

        // We use primary gravity to track the strongest gravitational influence.
        // If there is a stronger influence than the current primary, we change the
        // primary to that body.
        Eigen::Vector3d primaryGravity = Eigen::Vector3d::Zero();
        if (bodyA.primary != entt::null) {
            auto &state = registry.get<BodyState>(bodyA.primary);
            auto &body = registry.get<Body>(bodyA.primary);
            Eigen::Vector3d r = state.st.pos - stateA.st.pos;
            double norm = r.norm();
            primaryGravity = kGravitationalConstant * bodyA.mass * body.mass * r / (norm * norm * norm);
        }

        for (auto b : view2) {
            if (a == b) continue;
            auto &stateB = view2.get<BodyState>(b);
            auto &bodyB = view2.get<Body>(b);
            if (b == bodyA.primary) continue; // Primary's gravity is handled by kepler propagation

            Eigen::Vector3d r = stateB.st.pos - stateA.st.pos;
            double norm = r.norm();
            if (norm < 1e6) continue; // Avoid singularity

            Eigen::Vector3d gravity = kGravitationalConstant * bodyA.mass * bodyB.mass * r / (norm * norm * norm);
            if (gravity.squaredNorm() < primaryGravity.squaredNorm()) {
                forceAcc.force += gravity;
            } else {
                // Apply old primary's gravity, as Kepler no longer handles it
                if (bodyA.primary != entt::null) {
                    forceAcc.force += primaryGravity;
                }

                // Do not apply capturer's gravitational force if we capture since Kepler
                // propagation will handle it.
                primaryGravity = gravity;
                bodyA.primary = b;
            }
        }
    }
}

} // namespace

void physicsUpdate(entt::registry &registry, double dt) {
    auto forcesView = registry.view<ForceAccumulator, BodyState, Body>();
    for (auto entity : forcesView) {
        auto &forceAcc = forcesView.get<ForceAccumulator>(entity);
        forceAcc.force.setZero();
    }

    gravitySystem(registry);

    // Kick momentum update
    for (auto entity : forcesView) {
        auto &forceAcc = forcesView.get<ForceAccumulator>(entity);
        auto &state = forcesView.get<BodyState>(entity);
        auto &body = forcesView.get<Body>(entity);
        state.st.vel += dt * forceAcc.force / body.mass;
    }

    // Kepler drift
    recalculateAllKeplerParameters(registry);
    keplerPropagationSystem(registry, dt);
}
