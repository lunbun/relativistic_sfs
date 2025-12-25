#pragma once

#include <Eigen/Dense>
#include <entt/entt.hpp>

constexpr double kGravitationalConstant = 6.67430e-11;

struct PhysicsState {
    Eigen::Vector3d pos;
    Eigen::Vector3d vel;
};

struct ForceAccumulator {
    Eigen::Vector3d force;
};

struct Body {
    entt::entity primary;   // Primary body for orbiting, can be entt::null
    double mass;
};

struct NumIntegrState { PhysicsState st; };
struct BodyState { PhysicsState st; };
struct TempState { PhysicsState st; };

void physicsUpdate(entt::registry &registry, double dt);

template<typename From, typename To>
void syncState(entt::registry &registry) {
    auto view = registry.view<From, To>();
    for (auto entity : view) {
        auto &from = view.template get<From>(entity);
        auto &to = view.template get<To>(entity);
        to.st = from.st;
    }
}
