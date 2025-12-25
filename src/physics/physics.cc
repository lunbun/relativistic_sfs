#include "physics.h"

#include <cmath>

#include "physics/kepler.h"

void gravitySystem(entt::registry &registry) {
    auto view1 = registry.view<NumIntegrState, Body>();
    auto view2 = registry.view<ForceAccumulator, NumIntegrState, Body>();
    for (auto a : view1) {
        auto &physA = view1.get<NumIntegrState>(a);
        auto &bodyA = view1.get<Body>(a);
        for (auto b : view2) {
            if (a == b) continue;
            auto &physB = view2.get<NumIntegrState>(b);
            auto &bodyB = view2.get<Body>(b);
            auto &forceAcc = view2.get<ForceAccumulator>(b);

            if (a == bodyB.primary) continue; // Primary's gravity is handled by kepler propagation
            Eigen::Vector3d r = physB.st.pos - physA.st.pos;
            double norm = r.norm();
            if (norm < 1e6) continue; // Avoid singularity
            forceAcc.force -= 50 * kGravitationalConstant * bodyA.mass * bodyB.mass * r / (norm * norm * norm);
        }
    }
}

void physicsUpdate(entt::registry &registry, double dt) {
    auto forcesView = registry.view<ForceAccumulator, NumIntegrState, Body>();
    for (auto entity : forcesView) {
        auto &forceAcc = forcesView.get<ForceAccumulator>(entity);
        forceAcc.force.setZero();
    }

    gravitySystem(registry);

    // Kick momentum update
    for (auto entity : forcesView) {
        auto &forceAcc = forcesView.get<ForceAccumulator>(entity);
        auto &physics = forcesView.get<NumIntegrState>(entity);
        auto &body = forcesView.get<Body>(entity);
        physics.st.vel += dt * forceAcc.force / body.mass;
    }

    // Kepler drift
    recalculateAllKeplerParameters(registry);
    keplerPropagationSystem(registry, dt);
}


