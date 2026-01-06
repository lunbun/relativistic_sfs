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

        Eigen::Vector3d posA = calculateAbsolutePosition(registry, stateA);

        // We use primary gravity to track the strongest gravitational influence.
        // If there is a stronger influence than the current primary, we change the
        // primary to that body.
        // TODO: reimplement this
        // Eigen::Vector3d primaryGravity = Eigen::Vector3d::Zero();
        // if (bodyA.primary != entt::null) {
        //     auto &state = registry.get<BodyState>(bodyA.primary);
        //     auto &body = registry.get<Body>(bodyA.primary);
        //     Eigen::Vector3d r = state.st.pos - stateA.st.pos;
        //     double norm = r.norm();
        //     primaryGravity = kGravitationalConstant * bodyA.mass * body.mass * r / (norm * norm * norm);
        // }

        for (auto b : view2) {
            if (a == b) continue;
            auto &stateB = view2.get<BodyState>(b);
            auto &bodyB = view2.get<Body>(b);
            if (isParentBody<BodyState>(registry, a, b)) continue; // Parents' gravity is handled by Kepler propagation or Jacobi coordinates

            Eigen::Vector3d r = calculateAbsolutePosition(registry, stateB) - posA;
            double norm = r.norm();
            if (norm < 1e6) continue; // Avoid singularity

            Eigen::Vector3d gravity = kGravitationalConstant * bodyA.mass * bodyB.mass * r / (norm * norm * norm);
            // if (gravity.squaredNorm() < primaryGravity.squaredNorm()) {
            forceAcc.force += gravity;
            // } else {
            //     // Apply old primary's gravity, as Kepler no longer handles it
            //     if (bodyA.primary != entt::null) {
            //         forceAcc.force += primaryGravity;
            //     }
            //
            //     // Do not apply capturer's gravitational force if we capture since Kepler
            //     // propagation will handle it.
            //     primaryGravity = gravity;
            //     bodyA.primary = b;
            // }
        }
    }
}

void momentumKick(entt::registry &registry, double dt) {
    auto forcesView = registry.view<ForceAccumulator, BodyState, Body>();
    for (auto entity : forcesView) {
        auto &forceAcc = forcesView.get<ForceAccumulator>(entity);
        forceAcc.force.setZero();
    }

    gravitySystem(registry);

    for (auto entity : forcesView) {
        auto &forceAcc = forcesView.get<ForceAccumulator>(entity);
        auto &state = forcesView.get<BodyState>(entity);
        auto &body = forcesView.get<Body>(entity);
        state.st.vel += 0.5 * dt * forceAcc.force / body.mass;
    }
}

void keplerDrift(entt::registry &registry, double dt) {
    recalculateAllKeplerParameters(registry);
    keplerPropagationSystem(registry, dt);
}

} // namespace

void physicsUpdate(entt::registry &registry, double dt) {
    // Kick-drift-kick integrator
    momentumKick(registry, dt / 2);
    keplerDrift(registry, dt);
    momentumKick(registry, dt / 2);
}

void calculateConservedQuantities(entt::registry &registry, Eigen::Vector3d &com, double &energy, Eigen::Vector3d &momentum, Eigen::Vector3d &angularMomentum) {
    com = Eigen::Vector3d::Zero();
    double totalMass = 0.0;
    auto view = registry.view<BodyState, Body>();
    for (auto entity : view) {
        auto &state = view.get<BodyState>(entity);
        auto &body = view.get<Body>(entity);
        totalMass += body.mass;
        com += body.mass * calculateAbsolutePosition(registry, state);
    }
    com /= totalMass;

    energy = 0.0;
    momentum = Eigen::Vector3d::Zero();
    angularMomentum = Eigen::Vector3d::Zero();
    for (auto it1 = view.begin(); it1 != view.end(); ++it1) {
        auto entity1 = *it1;
        auto &state1 = view.get<BodyState>(entity1);
        auto &body1 = view.get<Body>(entity1);

        Eigen::Vector3d pos = calculateAbsolutePosition(registry, state1);
        Eigen::Vector3d vel = calculateAbsoluteVelocity(registry, state1);

        // Kinetic energy and momentum
        energy += 0.5 * body1.mass * vel.squaredNorm();
        momentum += body1.mass * vel;
        angularMomentum += body1.mass * pos.cross(vel);

        // Potential energy
        for (auto it2 = std::next(it1); it2 != view.end(); ++it2) {
            auto entity2 = *it2;
            auto &state2 = view.get<BodyState>(entity2);
            auto &body2 = view.get<Body>(entity2);

            Eigen::Vector3d r = calculateAbsolutePosition(registry, state2) - pos;
            double norm = r.norm();
            energy -= kGravitationalConstant * body1.mass * body2.mass / norm;
        }
    }
}
